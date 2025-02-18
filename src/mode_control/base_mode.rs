use crate::{
    flight_control::{
        camera_state::CameraAngle, flight_computer::FlightComputer, flight_state::FlightState,
        objective::beacon_objective::BeaconObjective, orbit::IndexedOrbitPosition,
        task::switch_state_task::SwitchStateTask,
    },
    mode_control::mode_context::ModeContext,
};
use chrono::{DateTime, TimeDelta};
use std::{future::Future, pin::Pin, sync::Arc};
use strum_macros::Display;
use tokio::{sync::oneshot, task::JoinHandle};
use tokio_util::sync::CancellationToken;

pub(crate) enum MappingModeEnd {
    Timestamp(DateTime<chrono::Utc>),
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
    BeaconObjectiveScanningMode(BeaconObjective),
}

impl BaseMode {
    const DEF_MAPPING_ANGLE: CameraAngle = CameraAngle::Narrow;
    const DT_0_STD: std::time::Duration = std::time::Duration::from_secs(0);

    #[allow(clippy::cast_possible_wrap)]
    pub async fn exec_map(
        context: Arc<ModeContext>,
        end: MappingModeEnd,
        c_tok: CancellationToken,
    ) {
        let end_t = {
            match end {
                MappingModeEnd::Timestamp(dt) => dt,
                MappingModeEnd::Join(_) => chrono::Utc::now() + TimeDelta::seconds(10000),
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

    pub async fn get_wait(
        &self,
        context: Arc<ModeContext>,
        due_time: TimeDelta,
        c_tok: CancellationToken,
    ) -> JoinHandle<()> {
        let current_state = { context.k().f_cont().read().await.state() };

        let task_fut: Pin<Box<dyn Future<Output = ()> + Send>> = match current_state {
            FlightState::Acquisition => Box::pin(Self::exec_map(
                context,
                MappingModeEnd::Timestamp(chrono::Utc::now() + due_time),
                c_tok,
            )),
            FlightState::Charge => Box::pin(FlightComputer::wait_for_duration(
                due_time.to_std().unwrap_or(Self::DT_0_STD),
            )),
            FlightState::Comms => {
                todo!()
            }
            _ => {
                panic!("[FATAL] Illegal state ({current_state})!")
            }
        };
        tokio::spawn(task_fut)
    }

    pub async fn get_task(&self, context: Arc<ModeContext>, task: SwitchStateTask) {
        match task.target_state() {
            FlightState::Acquisition => {
                FlightComputer::set_state_wait(context.k().f_cont(), FlightState::Acquisition)
                    .await;
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
                    todo!()
                }
            },
        }
    }

    pub(crate) async fn handle_b_o(&self, c: Arc<ModeContext>, obj: BeaconObjective) -> Self {
        match self {
            BaseMode::MappingMode => BaseMode::BeaconObjectiveScanningMode(obj),
            BaseMode::BeaconObjectiveScanningMode(curr_obj) => {
                c.b_buffer().lock().await.push(curr_obj.clone());
                BaseMode::BeaconObjectiveScanningMode(obj)
            }
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
}
