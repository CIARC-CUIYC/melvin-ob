use crate::flight_control::objective::beacon_objective::BeaconMeas;
use crate::flight_control::objective::beacon_objective_done::BeaconObjectiveDone;
use crate::{
    flight_control::{
        camera_state::CameraAngle, flight_computer::FlightComputer, flight_state::FlightState,
        objective::beacon_objective::BeaconObjective, orbit::IndexedOrbitPosition,
        task::switch_state_task::SwitchStateTask,
    },
    mode_control::mode_context::ModeContext,
};
use chrono::{DateTime, TimeDelta, Utc};
use regex::Regex;
use std::collections::HashMap;
use std::sync::LazyLock;
use std::{future::Future, pin::Pin, sync::Arc};
use strum_macros::Display;
use tokio::time::Instant;
use tokio::{sync::oneshot, task::JoinHandle};
use tokio_util::sync::CancellationToken;

pub(crate) enum MappingModeEnd {
    Timestamp(DateTime<Utc>),
    Join(JoinHandle<()>),
}

#[derive(Debug)]
pub enum PeriodicImagingEndSignal {
    KillNow,
    KillLastImage,
}

#[derive(Display, Clone)]
pub enum BaseMode {
    MappingMode,
    BeaconObjectiveScanningMode(HashMap<usize, BeaconObjective>),
}

pub enum BaseWaitExitSignal {
    Continue,
    BODone(Option<HashMap<usize, BeaconObjective>>),
}

impl BaseMode {
    fn start_beacon_scanning(obj: BeaconObjective) -> BaseMode {
        let mut obj_m = HashMap::new();
        obj_m.insert(obj.id(), obj);
        BaseMode::BeaconObjectiveScanningMode(obj_m)
    }
}

static BO_REGEX: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"(?i)ID[_, ]?(\d+).*?DISTANCE[_, ]?(([0-9]*[.])?[0-9]+)").unwrap()
});

fn extract_id_and_d(input: &str) -> Option<(usize, f64)> {
    // Match the input string
    if let Some(captures) = BO_REGEX.captures(input) {
        // Extract beacon_id and d_noisy
        if let (Some(beacon_id), Some(d_noisy)) = (captures.get(1), captures.get(2)) {
            let beacon_id: usize = beacon_id.as_str().parse().unwrap();
            let d_noisy: f64 = d_noisy.as_str().parse().unwrap();
            return Some((beacon_id, d_noisy));
        }
    }
    None // Return None if values cannot be extracted
}

impl BaseMode {
    const DT_0_STD: std::time::Duration = std::time::Duration::from_secs(0);
    const DEF_MAPPING_ANGLE: CameraAngle = CameraAngle::Narrow;
    const BO_MSG_COMM_PROLONG: std::time::Duration = std::time::Duration::from_secs(60);
    const MIN_COMM_DT: std::time::Duration = std::time::Duration::from_secs(60);
    const MAX_COMM_INTERVAL: std::time::Duration = std::time::Duration::from_secs(500);

    #[allow(clippy::cast_possible_wrap)]
    pub async fn exec_map(
        context: Arc<ModeContext>,
        end: MappingModeEnd,
        c_tok: CancellationToken,
    ) {
        let end_t = {
            match end {
                MappingModeEnd::Timestamp(dt) => dt,
                MappingModeEnd::Join(_) => Utc::now() + TimeDelta::seconds(10000),
            }
        };
        let o_ch_clone = context.o_ch_clone().await;
        let acq_phase = {
            let f_cont_lock = Arc::clone(&context.k().f_cont());
            let (tx, rx) = oneshot::channel();
            let i_start = o_ch_clone.i_entry().new_from_pos(f_cont_lock.read().await.current_pos());
            let k_clone = Arc::clone(context.k());
            let img_dt = o_ch_clone.img_dt();
            let handle = tokio::spawn(async move {
                FlightComputer::set_angle_wait(Arc::clone(&f_cont_lock), Self::DEF_MAPPING_ANGLE)
                    .await;
                k_clone
                    .c_cont()
                    .execute_acquisition_cycle(
                        f_cont_lock,
                        k_clone.con(),
                        (end_t, rx),
                        img_dt,
                        Self::DEF_MAPPING_ANGLE,
                        i_start.index(),
                    )
                    .await
            });
            (handle, tx)
        };

        let ranges = {
            if let MappingModeEnd::Join(join_handle) = end {
                let ((), res) = tokio::join!(
                    async move {
                        tokio::select! {
                            () = c_tok.cancelled() => {
                                let sig = PeriodicImagingEndSignal::KillNow;
                                acq_phase.1.send(sig).expect("[FATAL] Receiver hung up!");
                            },
                            _ = join_handle => {
                                let sig = PeriodicImagingEndSignal::KillLastImage;
                                acq_phase.1.send(sig).expect("[FATAL] Receiver hung up!");
                            }
                        }
                    },
                    async move { acq_phase.0.await.ok().unwrap_or(Vec::new()) }
                );
                res
            } else {
                let img_fut = acq_phase.0;
                tokio::pin!(img_fut);
                tokio::select! {
                    () = c_tok.cancelled() => {
                        let sig = PeriodicImagingEndSignal::KillNow;
                        acq_phase.1.send(sig).expect("[FATAL] Receiver hung up!");
                        img_fut.await.ok().unwrap_or(Vec::new())
                    }
                    res = &mut img_fut => {
                        res.ok().unwrap_or(Vec::new())
                    }
                }
            }
        };
        let fixed_ranges =
            IndexedOrbitPosition::map_ranges(&ranges, o_ch_clone.i_entry().period() as isize);
        print!("[INFO] Marking done: {} - {}", ranges[0].0, ranges[0].1);
        if let Some(r) = ranges.get(1) {
            print!(" and {} - {}", r.0, r.1);
        }
        println!();
        let k_loc = Arc::clone(context.k());
        tokio::spawn(async move {
            let c_orbit_lock = k_loc.c_orbit();
            let mut c_orbit = c_orbit_lock.write().await;
            for (start, end) in &fixed_ranges {
                c_orbit.mark_done(*start, *end);
            }
        });
    }

    async fn exec_comms(context: Arc<ModeContext>, due: DateTime<Utc>, c_tok: CancellationToken) {
        let mut event_rx = context.super_v().subscribe_event_hub();
        let due_secs = (due - Utc::now()).to_std().unwrap_or(Self::DT_0_STD);
        let mut deadline = Instant::now() + due_secs;
        loop {
            tokio::select! {
                // Wait for a message
                Ok(()) = event_rx.changed() => {
                    let val = event_rx.borrow().clone();
                    if let Some((id, d_noisy)) = extract_id_and_d(val.as_str()) {
                        let pos = context.k().f_cont().read().await.current_pos();
                        let meas = BeaconMeas::new(id, pos, d_noisy);
                        // TODO: search beacon with id and add
                        deadline = Instant::now() + Self::BO_MSG_COMM_PROLONG; // Reset timeout
                        println!("[LOG] Received BO message: {id} - {d_noisy}. Prologing by 60s!");
                    } else {
                        println!("[INFO] Received random message: {val:#?}");
                    }
                },
                // If the timeout expires, exit
                _ = tokio::time::sleep_until(deadline) => {
                    println!("Timeout reached. Stopping listener.");
                    // TODO: return beac dictionary;
                    return;
                },
                // If the task gets cancelled exit with the updated beacon vector
                _ = c_tok.cancelled() => {
                    // TODO: return beac dictionary return beac;
                    return;
                }
            }
        }
    }

    pub async fn get_wait(
        &self,
        context: Arc<ModeContext>,
        due: DateTime<Utc>,
        c_tok: CancellationToken,
    ) -> JoinHandle<BaseWaitExitSignal> {
        let current_state = { context.k().f_cont().read().await.state() };
        let task_fut: Pin<Box<dyn Future<Output = BaseWaitExitSignal> + Send>> = match current_state
        {
            FlightState::Acquisition => Box::pin(async move {
                Self::exec_map(context, MappingModeEnd::Timestamp(due), c_tok).await;
                BaseWaitExitSignal::Continue
            }),
            FlightState::Charge => {
                let def = Box::pin(async move {
                    let sleep = (due - Utc::now()).to_std().unwrap_or(Self::DT_0_STD);
                    FlightComputer::wait_for_duration(sleep).await;
                    BaseWaitExitSignal::Continue
                });
                match self {
                    BaseMode::MappingMode => def,
                    BaseMode::BeaconObjectiveScanningMode(obj_m) => {
                        if let Some(obj) = Self::check_b_o_done(obj_m).await {
                            let new_b_o = Self::handle_b_o_done(obj_m.clone(), context, obj).await;
                            Box::pin(async { BaseWaitExitSignal::BODone(new_b_o) })
                        } else {
                            def
                        }
                    }
                }
            }

            FlightState::Comms => Box::pin(async move {
                Self::exec_comms(context, due, c_tok).await;
                BaseWaitExitSignal::Continue
            }),
            _ => {
                panic!("[FATAL] Illegal state ({current_state})!")
            }
        };
        tokio::spawn(task_fut)
    }

    pub async fn get_task(&self, context: Arc<ModeContext>, task: SwitchStateTask) {
        match task.target_state() {
            FlightState::Acquisition => {
                // TODO: change back to acquisition
                FlightComputer::set_state_wait(context.k().f_cont(), FlightState::Comms).await;
            }
            FlightState::Charge => {
                let join_handle = async {
                    FlightComputer::set_state_wait(context.k().f_cont(), FlightState::Charge).await;
                };
                let k_clone = Arc::clone(context.k());
                tokio::spawn(async move {
                    k_clone.c_cont().create_full_snapshot().await.expect("[WARN] Export failed!");
                });
                join_handle.await;
            }
            FlightState::Comms => {}
            _ => match self {
                BaseMode::MappingMode => {
                    panic!("[FATAL] Illegal target state!")
                }
                BaseMode::BeaconObjectiveScanningMode(_) => {
                    FlightComputer::set_state_wait(context.k().f_cont(), FlightState::Comms).await;
                }
            },
        }
    }

    pub(crate) fn handle_b_o(&self, c: &Arc<ModeContext>, obj: BeaconObjective) -> Self {
        match self {
            BaseMode::MappingMode => BaseMode::start_beacon_scanning(obj),
            BaseMode::BeaconObjectiveScanningMode(curr_obj) => {
                let mut obj_m = curr_obj.clone();
                obj_m.insert(obj.id(), obj);
                BaseMode::BeaconObjectiveScanningMode(obj_m)
            }
        }
    }

    async fn handle_b_o_done(
        mut curr: HashMap<usize, BeaconObjective>,
        c: Arc<ModeContext>,
        b_o_done: Vec<BeaconObjectiveDone>,
    ) -> Option<HashMap<usize, BeaconObjective>> {
        for b_o in b_o_done {
            let id = b_o.id();
            curr.remove(&id);
            b_o.guess_max(c.k().f_cont().read().await.client());
        }
        if curr.is_empty() {
            None
        } else {
            Some(curr)
        }
        
    }

    pub(crate) fn exit_base(&self) -> BaseMode {
        match self {
            BaseMode::MappingMode => BaseMode::MappingMode,
            BaseMode::BeaconObjectiveScanningMode(obj) => {
                // TODO: check if obj is finished or has good enough guess
                todo!()
            }
        }
    }

    async fn check_b_o_done(
        b_o_map: &HashMap<usize, BeaconObjective>,
    ) -> Option<Vec<BeaconObjectiveDone>> {
        for (i, obj) in b_o_map {
            // TODO: check if obj is finished
        }
        None
    }
}
