use crate::flight_control::{
    beacon_controller::BeaconControllerState, flight_computer::FlightComputer,
    objective::known_img_objective::KnownImgObjective, task::{TaskController, base_task::Task}
};
use super::{global_mode::GlobalMode, in_orbit_mode::InOrbitMode, zo_prep_mode::ZOPrepMode};
use crate::mode_control::{
    base_mode::BaseMode,
    mode_context::ModeContext,
    signal::{ExecExitSignal, OpExitSignal, WaitExitSignal, OptOpExitSignal},
};
use async_trait::async_trait;
use chrono::{DateTime, Utc};
use std::sync::Arc;
use crate::{log, obj};

/// [`OrbitReturnMode`] is a transitional mode used after executing an out-of-orbit maneuver to
/// complete a zoned objective. It ensures the satellite returns to a valid
/// static orbit and is ready for continued mission operations.
/// This mode is also used when restarting the program to initially make sure a closed orbit is reached.
///
/// This mode performs orbit reentry maneuvers, energy recharging if necessary, and selects
/// the next mode based on the current context, such as available objectives or beacon scanning state.
///
/// While in this mode, tasks are not executed from the main scheduler—this is a maneuver-only state.
#[derive(Clone)]
pub(crate) struct OrbitReturnMode {}

impl OrbitReturnMode {
    /// Static name for the mode, used for logging and diagnostics.
    const MODE_NAME: &'static str = "OrbitReturnMode";

    /// Constructs a new [`OrbitReturnMode`] instance.
    ///
    /// # Returns
    /// * [`OrbitReturnMode`] – A new, empty instance.
    pub(crate) fn new() -> Self { Self {} }

    /// Selects and returns the appropriate next mode after orbit reentry.
    ///
    /// This function inspects the beacon controller and objective buffer to decide
    /// whether to transition into a [`ZOPrepMode`] (if valid objectives exist) or fallback
    /// to [`InOrbitMode`] using the appropriate [`BaseMode`].
    ///
    /// # Arguments
    /// * `context` – Shared mode context containing state and signal access.
    ///
    /// # Returns
    /// * `Box<dyn GlobalMode>` – The next mode to enter after completing return procedures.
    pub(crate) async fn get_next_mode(context: &Arc<ModeContext>) -> Box<dyn GlobalMode> {
        let next_base_mode = Self::get_next_base_mode(context).await;
        let mut obj_mon = context.zo_mon().write().await;
        let mut k_buffer = context.k_buffer().lock().await;
        while let Ok(obj) = obj_mon.try_recv() {
            obj!("Found Zoned Objective, ID: {} in mode {}. Stashing!", obj.id(), Self::MODE_NAME);
            k_buffer.push(obj);
        };        
        k_buffer.retain(|obj| {
            if Utc::now() > obj.end() {
                obj!("Zoned Objective, ID: {} is expired", obj.id());
                return false;
            }
            true
        });
        while let Some(obj) = k_buffer.pop() {
            let res = ZOPrepMode::from_obj(context, obj, next_base_mode).await;
            if let Some(prep_mode) = res {
                return Box::new(prep_mode);
            }
        }
        log!("No Zoned Objective left. Starting InOrbitMode!");
        Box::new(InOrbitMode::new(next_base_mode))
    }

    /// Selects the appropriate [`BaseMode`] to use after orbit return.
    ///
    /// This is determined based on the state of the beacon controller.
    ///
    /// # Arguments
    /// * `context` – Shared mode context.
    ///
    /// # Returns
    /// * `BaseMode` – Either [`MappingMode`] or [`BeaconObjectiveScanningMode`].
    async fn get_next_base_mode(context: &Arc<ModeContext>) -> BaseMode {
        let beacon_cont_state = {
            let mut bo_mon = context.bo_mon().write().await;
            *bo_mon.borrow_and_update()
        };
        match beacon_cont_state {
            BeaconControllerState::ActiveBeacons => BaseMode::BeaconObjectiveScanningMode,
            BeaconControllerState::NoActiveBeacons => BaseMode::MappingMode,
        }
    }
}

#[async_trait]
impl GlobalMode for OrbitReturnMode {
    /// Returns the static string name of the mode.
    fn type_name(&self) -> &'static str { Self::MODE_NAME }

    /// Initializes the orbit return procedure, performing reentry maneuvers and
    /// charging if needed. Handles safe mode interruptions and stores orbit state.
    ///
    /// # Arguments
    /// * `context` – Shared mode context.
    ///
    /// # Returns
    /// * `OpExitSignal` – Indicates continuation or reinitialization.
    async fn init_mode(&self, context: Arc<ModeContext>) -> OpExitSignal {
        let safe_mon = context.super_v().safe_mon();
        let f_cont_clone = context.k().f_cont().clone();
        let fut = async {
            FlightComputer::get_to_static_orbit_vel(&f_cont_clone).await;
            let max_maneuver_batt = FlightComputer::max_or_maneuver_charge();
            let batt = f_cont_clone.read().await.current_battery();
            if batt < max_maneuver_batt {
                FlightComputer::charge_to_wait(&f_cont_clone, max_maneuver_batt).await;
            }
            FlightComputer::or_maneuver(context.k().f_cont(), context.k().c_orbit()).await
        };
        tokio::select! {
        new_i = fut => {
                let pos = context.k().f_cont().read().await.current_pos();
                context.o_ch_lock().write().await.finish_entry(pos, new_i);
                OpExitSignal::ReInit(self.exit_mode(context).await)
            },
        () = safe_mon.notified() => self.safe_handler(context).await
        }
    }

    /// Not implemented. This mode does not wait for scheduled tasks.
    async fn exec_task_wait(&self, _: Arc<ModeContext>, _: DateTime<Utc>) -> WaitExitSignal {
        unimplemented!()
    }

    /// Not implemented. This mode does not execute scheduled tasks.
    async fn exec_task(&self, _: Arc<ModeContext>, _: Task) -> ExecExitSignal { unimplemented!() }

    /// Handles Safe Mode transition during return operations.
    ///
    /// Cancels current maneuver and reinitializes the [`OrbitReturnMode`].
    ///
    /// # Arguments
    /// * `context` – Shared mode context.
    ///
    /// # Returns
    /// * `OpExitSignal::ReInit` – Always restarts orbit return procedures.
    async fn safe_handler(&self, context: Arc<ModeContext>) -> OpExitSignal {
        FlightComputer::escape_safe(context.k().f_cont(), false).await;
        OpExitSignal::ReInit(Box::new(OrbitReturnMode::new()))
    }

    /// Handles discovery of a new Zoned Objective during return. Stashes it into the buffer.
    ///
    /// # Arguments
    /// * `c` – Shared mode context.
    /// * `obj` – The newly discovered objective.
    ///
    /// # Returns
    /// * `None` – The mode does not act immediately but stashes the objective.
    async fn zo_handler(&self, c: &Arc<ModeContext>, obj: KnownImgObjective) -> OptOpExitSignal {
        obj!("Found new Zoned Objective with ID: {} in mode {}.Stashing!", obj.id(), Self::MODE_NAME);
        c.k_buffer().lock().await.push(obj);
        None
    }

    /// Not implemented. Beacon state changes do not affect this mode.
    async fn bo_event_handler(&self, _: &Arc<ModeContext>) -> OptOpExitSignal { unimplemented!() }

    /// Finalizes the return maneuver and selects the next mode to transition into.
    ///
    /// Charges if necessary to reach the minimum battery threshold for nominal operation.
    ///
    /// # Arguments
    /// * `c` – Shared mode context.
    ///
    /// # Returns
    /// * `Box<dyn GlobalMode>` – The next mode to run.
    async fn exit_mode(&self, c: Arc<ModeContext>) -> Box<dyn GlobalMode> {
        if c.k().f_cont().read().await.current_battery() < TaskController::MIN_BATTERY_THRESHOLD {
            FlightComputer::charge_to_wait(&c.k().f_cont(), TaskController::MIN_BATTERY_THRESHOLD)
                .await;
        }
        Self::get_next_mode(&c).await
    }
}
