use crate::flight_control::objective::beacon_objective::BeaconMeas;
use crate::flight_control::objective::beacon_objective_done::BeaconObjectiveDone;
use crate::http_handler::http_client::HTTPClient;
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
use tokio::sync::Mutex;
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
    BeaconObjectiveScanningMode(Arc<Mutex<HashMap<usize, BeaconObjective>>>),
}

pub enum BaseWaitExitSignal {
    Continue,
    ReturnAllDone,
    ReturnSomeLeft,
}

impl BaseMode {
    fn start_beacon_scanning(obj: BeaconObjective) -> BaseMode {
        let mut obj_m = HashMap::new();
        obj_m.insert(obj.id(), obj);
        BaseMode::BeaconObjectiveScanningMode(Arc::new(Mutex::new(obj_m)))
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

    async fn exec_comms(context: Arc<ModeContext>, due: DateTime<Utc>, c_tok: CancellationToken, b_o: Arc<Mutex<HashMap<usize, BeaconObjective>>>) {
        let start = Utc::now();
        let mut event_rx = context.super_v().subscribe_event_hub();
        let due_secs = (due - Utc::now()).to_std().unwrap_or(Self::DT_0_STD);
        let sleep_fut = tokio::time::sleep_until(Instant::now() + due_secs);
        tokio::pin!(sleep_fut);

        loop {
            tokio::select! {
                // Wait for a message
                Ok(()) = event_rx.changed() => {
                    let val = event_rx.borrow().clone();
                    if let Some((id, d_noisy)) = extract_id_and_d(val.as_str()) {
                        let pos = context.k().f_cont().read().await.current_pos();
                        let meas = BeaconMeas::new(id, pos, d_noisy);
                        println!("[INFO] Received BO measurement: {val:#?}");
                        println!("[INFO] Position was: {pos}.");
                        if let Some(obj) = b_o.lock().await.get_mut(&id) {
                            println!("[LOG] Updating BO {id} and prolonging!");
                            obj.append_measurement(meas);
                            sleep_fut.set(tokio::time::sleep_until(Instant::now() + Self::BO_MSG_COMM_PROLONG)); // Reset timeout
                            // TODO: these checks shouldnt be here
                            let o_meas = obj.measurements().unwrap();
                            if o_meas.guess_estimate() < 10 {
                                println!("{:?}", o_meas.pack_perfect_circles());
                            }
                        }
                    } else {
                        println!("[WARN] Message has unknown format. Ignoring.");
                    }
                },
                // If the timeout expires, exit
                _ = &mut sleep_fut => {
                    println!("[LOG] Comms Timeout reached after {}. Stopping listener.",
                    (Utc::now() - start).num_seconds());
                    return;
                },
                // If the task gets cancelled exit with the updated beacon vector
                _ = c_tok.cancelled() => {
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
        let (current_state, handler) = {
            let f_cont = context.k().f_cont();
            let lock = f_cont.read().await;
            (lock.state(), lock.client())
        };
        let def = Box::pin(async move {
            let sleep = (due - Utc::now()).to_std().unwrap_or(Self::DT_0_STD);
            FlightComputer::wait_for_duration(sleep).await;
            BaseWaitExitSignal::Continue
        });
        let task_fut: Pin<Box<dyn Future<Output = BaseWaitExitSignal> + Send>> = match current_state
        {
            FlightState::Acquisition => Box::pin(async move {
                Self::exec_map(context, MappingModeEnd::Timestamp(due), c_tok).await;
                BaseWaitExitSignal::Continue
            }),
            FlightState::Charge => {

                match self {
                    BaseMode::MappingMode => def,
                    BaseMode::BeaconObjectiveScanningMode(obj_m) => {
                        if let Some(obj) = Self::check_b_o_done(obj_m.clone()).await {
                            let obj_m_clone = Arc::clone(obj_m);
                            Box::pin(async move {
                                Self::handle_b_o_done(obj_m_clone, obj, handler).await
                            })
                        } else {
                            def
                        }
                    }
                }
            }

            FlightState::Comms => {
                if let Self::BeaconObjectiveScanningMode(b_o_arc) = self {
                    let clone = Arc::clone(b_o_arc);
                    Box::pin(async move {
                        Self::exec_comms(context, due, c_tok, clone).await;
                        BaseWaitExitSignal::Continue
                    })
                } else {
                    println!("[INFO] No known Beacon Objectives. Waiting!");
                    def
                }

            },
            _ => {
                panic!("[FATAL] Illegal state ({current_state})!")
            }
        };
        tokio::spawn(task_fut)
    }

    pub async fn get_task(&self, context: Arc<ModeContext>, task: SwitchStateTask) {
        match task.target_state() {
            FlightState::Acquisition => {
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

    pub(crate) async fn handle_b_o(&self, _: &Arc<ModeContext>, obj: BeaconObjective) -> Self {
        match self {
            BaseMode::MappingMode => BaseMode::start_beacon_scanning(obj),
            BaseMode::BeaconObjectiveScanningMode(curr_obj) => {
                let mut obj_m = curr_obj.lock().await.clone();
                obj_m.insert(obj.id(), obj);
                BaseMode::BeaconObjectiveScanningMode(Arc::new(Mutex::new(obj_m)))
            }
        }
    }

    async fn handle_b_o_done(
        curr: Arc<Mutex<HashMap<usize, BeaconObjective>>>,
        b_o_done: Vec<BeaconObjectiveDone>,
        cl: Arc<HTTPClient>,
    ) -> BaseWaitExitSignal {
        for b_o in b_o_done {
            b_o.guess_max(cl.clone()).await;
        }
        let empty = {
            let curr_lock = curr.lock().await;
            curr_lock.is_empty()
        };
        if empty {
            BaseWaitExitSignal::ReturnAllDone
        } else {
            BaseWaitExitSignal::ReturnSomeLeft
        }
    }

    pub(crate) async fn exit_base(&self, handler: Arc<HTTPClient>) -> BaseMode {
        match self {
            BaseMode::MappingMode => BaseMode::MappingMode,
            BaseMode::BeaconObjectiveScanningMode(b_map) => {
                if let Some(obj) = Self::check_b_o_done(b_map.clone()).await {
                    let new_b_o = Self::handle_b_o_done(b_map.clone(), obj, handler).await;
                    if let BaseWaitExitSignal::ReturnAllDone = new_b_o {
                        return BaseMode::MappingMode;
                    }
                }
                self.clone()
            }
        }
    }

    async fn check_b_o_done(
        b_o_map: Arc<Mutex<HashMap<usize, BeaconObjective>>>,
    ) -> Option<Vec<BeaconObjectiveDone>> {
        let mut done_obj = Vec::new();
        let mut not_done_obj_m = HashMap::new();
        let mut b_o_map_lock = b_o_map.lock().await;
        for obj in b_o_map_lock.drain() {
            if let Some(meas) = obj.1.measurements() {
                if meas.guess_estimate() < 10 {
                    done_obj.push(BeaconObjectiveDone::from(obj.1));
                    println!("[INFO] Found finished objective: ID {}", obj.0);
                    continue;
                }
            }
            not_done_obj_m.insert(obj.0, obj.1);
        }
        *b_o_map_lock = not_done_obj_m;
        drop(b_o_map_lock);
        if done_obj.len() > 0 {
            Some(done_obj)
        } else {
            None
        }
    }
}
