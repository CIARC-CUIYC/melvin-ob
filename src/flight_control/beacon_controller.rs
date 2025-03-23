use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::objective::beacon_objective::{BeaconMeas, BeaconObjective};
use crate::flight_control::objective::beacon_objective_done::BeaconObjectiveDone;
use crate::flight_control::task::TaskController;
use crate::http_handler::http_client::HTTPClient;
use crate::mode_control::base_mode::BaseWaitExitSignal;
use crate::mode_control::mode_context::ModeContext;
use crate::{event, info, obj, warn};
use chrono::{DateTime, TimeDelta, Utc};
use regex::Regex;
use std::collections::HashMap;
use std::future::Future;
use std::pin::Pin;
use std::sync::{Arc, LazyLock};
use std::time::Duration;
use tokio::sync::mpsc::Receiver;
use tokio::sync::{Mutex, Notify, RwLock};
use tokio::time::interval;
use tokio_util::sync::CancellationToken;

pub struct ScanBeaconParams {
    pub due_t: Option<DateTime<Utc>>,
    pub context: Arc<ModeContext>,
    pub c_tok: CancellationToken,
    pub fut: Pin<Box<dyn Future<Output = ()> + Send>>,
}

pub struct BeaconController {
    active_beacons: Mutex<HashMap<usize, BeaconObjective>>,
    done_beacons: Mutex<HashMap<usize, BeaconObjectiveDone>>,
    beacon_rx: Mutex<Receiver<BeaconObjective>>,
    scan_active_args: Mutex<Option<ScanBeaconParams>>,
    scan_notify: Notify,
}

const TIME_TO_NEXT_PASSIVE_CHECK: Duration = Duration::from_secs(15);
const BEACON_OBJ_RETURN_MIN_DELAY: TimeDelta = TimeDelta::minutes(3);
const BEACON_OBJ_RETURN_WARNING: TimeDelta = TimeDelta::minutes(10);
const THRESHOLD_GUESSES_TO_DONE: usize = 15;
const MAX_MSG_PER_CHECK: usize = 10;

const BO_MSG_COMM_PROLONG: TimeDelta = TimeDelta::seconds(60);

static BO_REGEX: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"(?i)ID[_, ]?(\d+).*?DISTANCE[_, ]?(([0-9]*[.])?[0-9]+)").unwrap()
});

impl BeaconController {
    pub fn new(rx_beac: Receiver<BeaconObjective>) -> Self {
        Self {
            active_beacons: Mutex::new(HashMap::new()),
            done_beacons: Mutex::new(HashMap::new()),
            beacon_rx: Mutex::new(rx_beac),
            scan_active_args: Mutex::new(None),
            scan_notify: Notify::new(),
        }
    }

    pub async fn run(self: Arc<Self>, handler: Arc<HTTPClient>) {
        let mut approaching_end_interval = interval(TIME_TO_NEXT_PASSIVE_CHECK);
        let mut beac_rx_locked = self.beacon_rx.lock().await;
        loop {
            tokio::select! {
                _ = approaching_end_interval.tick() =>
                {self.check_approaching_end(&handler).await}

                Some(beac_obj) = beac_rx_locked.recv() => {
                    self.add_beacon(beac_obj).await;
                }
            }
        }
    }

    pub async fn request_scan_active_beacons(&self, args: ScanBeaconParams) {
        let mut scan_args_lock = self.scan_active_args.lock().await;
        *scan_args_lock = Some(args);
        self.scan_notify.notify_one();
    }

    pub async fn last_active_beac_end(&self) -> Option<DateTime<Utc>> {
        self.active_beacons.lock().await.values().map(BeaconObjective::end).max()
    }

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

    pub async fn scan_active_beacons(
        &self,
        msg: (DateTime<Utc>, String),
        due_t: Option<DateTime<Utc>>,
        f_cont: Arc<RwLock<FlightComputer>>,
    ) -> bool {
        let (t, val) = msg;
        if let Some((id, d_noisy)) = Self::extract_id_and_d(val.as_str()) {
            let (pos, res_batt) = {
                let f_cont_lock = f_cont.read().await;
                (
                    f_cont_lock.current_pos(),
                    f_cont_lock.batt_in_dt(BO_MSG_COMM_PROLONG),
                )
            };
            let msg_delay = Utc::now() - t;
            let meas = BeaconMeas::new(id, pos, d_noisy, msg_delay);
            obj!("Received BO measurement at {pos} for ID {id} with distance {d_noisy}.");
            let mut active_lock = self.active_beacons.lock().await;
            if let Some(obj) = active_lock.get_mut(&id) {
                obj!("Updating BO {id} and prolonging!");
                obj.append_measurement(meas);
                let new_end = Utc::now() + BO_MSG_COMM_PROLONG;
                if let Some(t) = due_t {
                    let is_last = active_lock.len() == 1;
                    let prolong_cond = new_end > t || is_last;
                    if prolong_cond && res_batt > TaskController::MIN_BATTERY_THRESHOLD {
                        return true;
                    }
                }
                false
            } else {
                warn!("Unknown BO ID {id}. Ignoring!");
                false
            }
        } else {
            event!("Message has unknown format {val:#?}. Ignoring.");
            false
        }
    }

    async fn add_beacon(&self, obj: BeaconObjective) {
        self.active_beacons.lock().await.insert(obj.id(), obj);
    }

    pub async fn latest_active_beac_end(&self) -> DateTime<Utc> {
        self.active_beacons
            .lock()
            .await
            .values()
            .map(BeaconObjective::end)
            .max()
            .unwrap_or(Utc::now())
    }

    async fn move_to_done(&self, beacon: BeaconObjective) {
        let done_beacon = BeaconObjectiveDone::from(beacon);
        self.active_beacons.lock().await.remove(&done_beacon.id());
        self.done_beacons.lock().await.insert(done_beacon.id(), done_beacon.clone());

        obj!("Marking Beacon objective as done: ID {}", done_beacon.id());

        if done_beacon.guesses().is_empty() {
            obj!(
                "Almost ending Beacon objective: ID {}. No guesses :(",
                done_beacon.id() // TODO: Randomize 3 guesses
            );
        }

        obj!(
            "Finished Beacon objective: ID {} has {} guesses.",
            done_beacon.id(),
            done_beacon.guesses().len()
        );
    }

    async fn check_approaching_end(&self, handler: &Arc<HTTPClient>) {
        let active_beacon_tasks = self.active_beacons.lock().await.clone();
        for active_beacon in active_beacon_tasks {
            if active_beacon.1.end() < Utc::now() + BEACON_OBJ_RETURN_MIN_DELAY {
                obj!(
                    "Active Beacon objective end is less than {} min away: ID {}. Submitting this now!",
                    BEACON_OBJ_RETURN_MIN_DELAY,
                    active_beacon.1.id(),
                );
                self.move_to_done(active_beacon.1.clone()).await;

                continue;
            } else if active_beacon.1.end() < Utc::now() + BEACON_OBJ_RETURN_WARNING {
                obj!(
                    "Active Beacon objective end time is less than {} min away: ID {}. Submitting this soon!",
                    BEACON_OBJ_RETURN_WARNING,
                    active_beacon.1.id()
                );
            }
        }
        // TODO: what to do with this signal
        let signal = self.handle_beacon_submission(handler).await;
    }

    async fn check_enough_measurements(&mut self) {
        let active_beacon_tasks = self.active_beacons.lock().await.clone();
        for active_beacon in active_beacon_tasks {
            if let Some(meas) = active_beacon.1.measurements() {
                let min_guesses = meas.guess_estimate();
                // obj!(
                //     "ID {} has {} min guesses.",
                //     active_beacon.1.id(),
                //     min_guesses
                // );
                if min_guesses < THRESHOLD_GUESSES_TO_DONE {
                    obj!(
                        "Active beacon objective has less than {} min guesses: ID {}. Marking as done.",
                        min_guesses,
                        active_beacon.1.id(),
                    );
                    self.move_to_done(active_beacon.1.clone()).await;
                }
            }
        }
    }

    async fn handle_beacon_submission(&self, handler: &Arc<HTTPClient>) -> BaseWaitExitSignal {
        let done_beacons = self.done_beacons.lock().await.clone();
        for done_beacon in done_beacons {
            obj!("Submitting Beacon Objective: {}", done_beacon.1.id());
            done_beacon.1.guess_max(Arc::clone(handler)).await;
        }
        self.check_exit_signal().await
    }

    pub async fn check_exit_signal(&self) -> BaseWaitExitSignal {
        if self.active_beacons.lock().await.is_empty() {
            info!("All Beacon Objectives done! Switching to Mapping Mode.");
            BaseWaitExitSignal::ReturnAllDone
        } else {
            info!("There are still Beacon Objectives left. Waiting for them to finish!");
            BaseWaitExitSignal::ReturnSomeLeft
        }
    }
}
