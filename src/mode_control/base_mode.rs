use crate::flight_control::{
    camera_state::CameraAngle,
    flight_computer::FlightComputer,
    flight_state::FlightState,
    objective::{
        beacon_objective::{BeaconMeas, BeaconObjective},
        beacon_objective_done::BeaconObjectiveDone,
    },
    orbit::IndexedOrbitPosition,
    task::{switch_state_task::SwitchStateTask, TaskController},
};
use crate::http_handler::http_client::HTTPClient;
use crate::{event, fatal, info, log, obj, warn};

use crate::mode_control::base_mode::TaskEndSignal::{Join, Timestamp};
use crate::mode_control::mode_context::ModeContext;
use chrono::{DateTime, TimeDelta, Utc};
use regex::Regex;
use std::{
    collections::HashMap,
    future::Future,
    pin::Pin,
    sync::{Arc, LazyLock},
    time::Duration,
};
use strum_macros::Display;
use tokio::{
    sync::{oneshot, Mutex},
    task::JoinHandle,
    time::Instant,
};
use tokio_util::sync::CancellationToken;

pub(crate) enum TaskEndSignal {
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

static BO_REGEX: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"(?i)ID[_, ]?(\d+).*?DISTANCE[_, ]?(([0-9]*[.])?[0-9]+)").unwrap()
});
fn extract_id_and_d(input: &str) -> Option<(usize, f64)> {
    // Match the input string
    if let Some(captures) = BO_REGEX.captures(input) {
        // Extract beacon_id and d_noisy
        if let (Some(beacon_id), Some(d_noisy)) = (captures.get(1), captures.get(2)) {
            let id: usize = beacon_id.as_str().parse().unwrap();
            let d_n: f64 = d_noisy.as_str().parse().unwrap();
            return Some((id, d_n));
        }
    }
    None // Return None if values cannot be extracted
}

impl BaseMode {
    const DT_0_STD: Duration = Duration::from_secs(0);
    const MAX_MSG_DT: TimeDelta = TimeDelta::milliseconds(300);
    const DEF_MAPPING_ANGLE: CameraAngle = CameraAngle::Narrow;
    const BO_MSG_COMM_PROLONG_STD: Duration = Duration::from_secs(60);
    const BO_MSG_COMM_PROLONG: TimeDelta = TimeDelta::seconds(60);
    const MIN_COMM_DT: Duration = Duration::from_secs(60);
    const MAX_COMM_INTERVAL: Duration = Duration::from_secs(500);
    const MAX_COMM_PROLONG_RESCHEDULE: TimeDelta = TimeDelta::seconds(2);
    const BEACON_OBJ_RETURN_MIN_DELAY: TimeDelta = TimeDelta::minutes(10);

    fn start_beacon_scanning(obj: BeaconObjective) -> BaseMode {
        let mut obj_m = HashMap::new();
        obj_m.insert(obj.id(), obj);
        BaseMode::BeaconObjectiveScanningMode(Arc::new(Mutex::new(obj_m)))
    }

    #[allow(clippy::cast_possible_wrap)]
    pub async fn exec_map(context: Arc<ModeContext>, end: TaskEndSignal, c_tok: CancellationToken) {
        let end_t = {
            match end {
                Timestamp(dt) => dt,
                Join(_) => Utc::now() + TimeDelta::seconds(10000),
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
            if let Join(join_handle) = end {
                tokio::pin!(join_handle);
                tokio::select! {
                    () = c_tok.cancelled() => {
                        let sig = PeriodicImagingEndSignal::KillNow;
                        acq_phase.1.send(sig).expect("[FATAL] Receiver hung up!");
                        join_handle.abort();
                        acq_phase.0.await.ok().unwrap_or(vec![(0, 0)])
                    },
                    _ = &mut join_handle => {
                        let sig = PeriodicImagingEndSignal::KillLastImage;
                        acq_phase.1.send(sig).expect("[FATAL] Receiver hung up!");
                        acq_phase.0.await.ok().unwrap_or(vec![(0, 0)])
                    }
                }
            } else {
                let img_fut = acq_phase.0;
                tokio::pin!(img_fut);
                tokio::select! {
                    () = c_tok.cancelled() => {
                        let sig = PeriodicImagingEndSignal::KillNow;
                        acq_phase.1.send(sig).expect("[FATAL] Receiver hung up!");
                        img_fut.await.ok().unwrap_or(vec![(0, 0)])
                    }
                    res = &mut img_fut => {
                        res.ok().unwrap_or(vec![(0, 0)])
                    }
                }
            }
        };
        let fixed_ranges =
            IndexedOrbitPosition::map_ranges(&ranges, o_ch_clone.i_entry().period() as isize);
        let and = if let Some(r) = ranges.get(1) {
            format!(" and {} - {}", r.0, r.1)
        } else {
            String::new()
        };
        log!("Marking done: {} - {}{and}", ranges[0].0, ranges[0].1);
        let k_loc = Arc::clone(context.k());
        tokio::spawn(async move {
            let c_orbit_lock = k_loc.c_orbit();
            let mut c_orbit = c_orbit_lock.write().await;
            for (start, end) in &fixed_ranges {
                if start != end {
                    c_orbit.mark_done(*start, *end);
                }
            }
        });
    }

    async fn exec_comms(
        context: Arc<ModeContext>,
        due: TaskEndSignal,
        c_tok: CancellationToken,
        b_o: Arc<Mutex<HashMap<usize, BeaconObjective>>>,
    ) {
        let start = Utc::now();
        let obj_length = b_o.lock().await.len();
        let mut event_rx = context.super_v().subscribe_event_hub();
        let due_t = {
            if let Timestamp(x) = due {
                Some(x)
            } else {
                None
            }
        };

        let mut fut: Pin<Box<dyn Future<Output = ()> + Send>> = match due {
            Timestamp(t) => {
                let due_secs = (t - Utc::now()).to_std().unwrap_or(Self::DT_0_STD);
                Box::pin(tokio::time::sleep_until(Instant::now() + due_secs))
            }
            Join(join_handle) => Box::pin(async { join_handle.await.ok().unwrap() }),
        };
        info!("Starting Comms Listener.");
        loop {
            tokio::select! {
                // Wait for a message
                Ok(msg) = event_rx.recv() => {
                    let (t, val) = msg;
                    if let Some((id, d_noisy)) = extract_id_and_d(val.as_str()) {
                        let (pos, res_batt) = {
                            let f_cont = context.k().f_cont();
                            let lock = f_cont.read().await;
                            (lock.current_pos(), lock.batt_in_dt(Self::BO_MSG_COMM_PROLONG))
                        };
                        let msg_delay = Utc::now() - t;
                        let meas = BeaconMeas::new(id, pos, d_noisy, msg_delay);
                        obj!("Received BO measurement at {pos} for ID {id} with distance {d_noisy}.");
                        if let Some(obj) = b_o.lock().await.get_mut(&id) {
                            obj!("Updating BO {id} and prolonging!");
                            obj.append_measurement(meas);
                            let new_end = Utc::now() + Self::BO_MSG_COMM_PROLONG;
                            if let Some(t) = due_t {
                                let prolong_cond = new_end > t || obj_length == 1;
                                if prolong_cond && res_batt > TaskController::MIN_BATTERY_THRESHOLD {
                                    fut = Box::pin(tokio::time::sleep_until(Instant::now() + Self::BO_MSG_COMM_PROLONG_STD));
                                }
                            }
                        } else {
                            warn!("Unknown BO ID {id}. Ignoring!");
                        }
                    } else {
                        event!("Message has unknown format {val:#?}. Ignoring.");
                    }
                },
                // If the timeout expires, exit
                () = &mut fut => {
                    log!("Comms Deadline reached after {}s. Stopping listener.",
                    (Utc::now() - start).num_seconds());
                    break;
                },
                // If the task gets cancelled exit with the updated beacon vector
                () = c_tok.cancelled() => {
                    log!("Comms Listener cancelled. Stopping listener.");
                    break;
                }
            }
        }
    }

    pub async fn handle_sched_preconditions(&self, context: Arc<ModeContext>) {
        match self {
            BaseMode::MappingMode => (),
            BaseMode::BeaconObjectiveScanningMode(_) => {
                FlightComputer::get_to_comms(context.k().f_cont()).await;
            }
        }
    }

    pub async fn get_schedule_handle(
        &self,
        context: Arc<ModeContext>,
        c_tok: CancellationToken,
    ) -> JoinHandle<()> {
        let k = Arc::clone(context.k());
        let o_ch = context.o_ch_clone().await;
        let j_handle = match self {
            BaseMode::MappingMode => tokio::spawn(TaskController::sched_opt_orbit(
                k.t_cont(),
                k.c_orbit(),
                k.f_cont(),
                o_ch.i_entry(),
            )),
            BaseMode::BeaconObjectiveScanningMode(obj_m) => {
                let last_obj_end = obj_m
                    .lock()
                    .await
                    .values()
                    .map(BeaconObjective::end)
                    .max()
                    .unwrap_or(Utc::now());
                tokio::spawn(TaskController::sched_opt_orbit_w_comms(
                    k.t_cont(),
                    k.c_orbit(),
                    k.f_cont(),
                    o_ch.i_entry(),
                    last_obj_end,
                ))
            }
        };
        let state = context.k().f_cont().read().await.state();
        if state == FlightState::Acquisition {
            tokio::spawn(BaseMode::exec_map(context, Join(j_handle), c_tok))
        } else if state == FlightState::Comms {
            match self {
                BaseMode::MappingMode => fatal!("Illegal state ({state})!"),
                BaseMode::BeaconObjectiveScanningMode(b_o_arc) => {
                    let b_o_clone = Arc::clone(b_o_arc);
                    tokio::spawn(BaseMode::exec_comms(
                        context,
                        Join(j_handle),
                        c_tok,
                        b_o_clone,
                    ))
                }
            }
        } else {
            tokio::spawn(async { j_handle.await.ok().unwrap() })
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
        let c_tok_clone = c_tok.clone();
        let def = Box::pin(async move {
            let sleep = (due - Utc::now()).to_std().unwrap_or(Self::DT_0_STD);
            tokio::time::timeout(sleep, c_tok_clone.cancelled()).await.ok().unwrap_or(());
            BaseWaitExitSignal::Continue
        });
        let task_fut: Pin<Box<dyn Future<Output = BaseWaitExitSignal> + Send>> = match current_state
        {
            FlightState::Acquisition => Box::pin(async move {
                Self::exec_map(context, Timestamp(due), c_tok).await;
                BaseWaitExitSignal::Continue
            }),
            FlightState::Charge => match self {
                BaseMode::MappingMode => def,
                BaseMode::BeaconObjectiveScanningMode(obj_m) => {
                    if let Some(obj) = Self::check_b_o_done(obj_m.clone()).await {
                        let obj_m_clone = Arc::clone(obj_m);
                        Box::pin(Self::handle_b_o_done(obj_m_clone, obj, handler))
                    } else {
                        def
                    }
                }
            },

            FlightState::Comms => {
                if let Self::BeaconObjectiveScanningMode(b_o_arc) = self {
                    let clone = Arc::clone(b_o_arc);
                    if let Some(obj) = Self::check_b_o_done(b_o_arc.clone()).await {
                        Box::pin(Self::handle_b_o_done(clone, obj, handler))
                    } else {
                        Box::pin(async move {
                            Self::exec_comms(context, Timestamp(due), c_tok, clone).await;
                            if Utc::now() > due + Self::MAX_COMM_PROLONG_RESCHEDULE {
                                BaseWaitExitSignal::ReturnSomeLeft
                            } else {
                                BaseWaitExitSignal::Continue
                            }
                        })
                    }
                } else {
                    warn!("No known Beacon Objectives. Waiting!");
                    def
                }
            }
            _ => {
                fatal!("Illegal state ({current_state})!")
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
                    k_clone.c_cont().create_full_snapshot().await.expect("Export failed!");
                });
                join_handle.await;
            }
            FlightState::Comms => match self {
                BaseMode::MappingMode => {
                    fatal!("Illegal target state!")
                }
                BaseMode::BeaconObjectiveScanningMode(_) => {
                    FlightComputer::set_state_wait(context.k().f_cont(), FlightState::Comms).await;
                }
            },
            _ => fatal!("Illegal target state!"),
        }
    }

    pub(crate) async fn handle_b_o(
        &self,
        _: &Arc<ModeContext>,
        obj: BeaconObjective,
    ) -> Option<Self> {
        match self {
            BaseMode::MappingMode => Some(BaseMode::start_beacon_scanning(obj)),
            BaseMode::BeaconObjectiveScanningMode(curr_obj) => {
                let mut obj_m = curr_obj.lock().await;
                obj_m.insert(obj.id(), obj);
                None
            }
        }
    }

    async fn handle_b_o_done(
        curr: Arc<Mutex<HashMap<usize, BeaconObjective>>>,
        b_o_done: Vec<BeaconObjectiveDone>,
        cl: Arc<HTTPClient>,
    ) -> BaseWaitExitSignal {
        for b_o in b_o_done {
            if b_o.guesses().is_empty() {
                obj!("Beacon Objective {} has no guesses. Not submitting!", b_o.id());
            } else {
                obj!("Submitting Beacon Objective: {}", b_o.id());
                b_o.guess_max(cl.clone()).await;
            }
        }
        let empty = {
            let curr_lock = curr.lock().await;
            curr_lock.is_empty()
        };
        if empty {
            info!("All Beacon Objectives done! Switching to Mapping Mode.");
            BaseWaitExitSignal::ReturnAllDone
        } else {
            info!("There are still Beacon Objectives left. Waiting for them to finish!");
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
            if obj.1.end() < Utc::now() + Self::BEACON_OBJ_RETURN_MIN_DELAY {
                let beac_done = BeaconObjectiveDone::from(obj.1);
                done_obj.push(beac_done);
                obj!("Almost ending Beacon objective: ID {}.", obj.0);
                continue;
            } else if let Some(meas) = obj.1.measurements() {
                let min_guesses = meas.guess_estimate();
                if min_guesses < 15 {
                    let beac_done = BeaconObjectiveDone::from(obj.1);
                    obj!(
                        "Finished Beacon objective: ID {} has {} guesses.",
                        obj.0,
                        beac_done.guesses().len()
                    );
                    done_obj.push(beac_done);
                    continue;
                }
            }
            not_done_obj_m.insert(obj.0, obj.1);
        }
        *b_o_map_lock = not_done_obj_m;
        drop(b_o_map_lock);
        if done_obj.is_empty() {
            None
        } else {
            Some(done_obj)
        }
    }
}
