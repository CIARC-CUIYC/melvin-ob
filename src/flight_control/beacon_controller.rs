use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::objective::beacon_objective::{BeaconMeas, BeaconObjective};
use crate::flight_control::objective::beacon_objective_done::BeaconObjectiveDone;
use crate::http_handler::http_client::HTTPClient;
use crate::{event, obj, warn};
use chrono::{DateTime, TimeDelta, Utc};
use regex::Regex;
use std::collections::HashMap;
use std::sync::{Arc, LazyLock};
use std::time::Duration;
use tokio::sync::mpsc::Receiver;
use tokio::sync::{Mutex, RwLock, watch};
use tokio::time::interval;

pub struct BeaconController {
    active_bo: RwLock<HashMap<usize, BeaconObjective>>,
    done_bo: RwLock<HashMap<usize, BeaconObjectiveDone>>,
    beacon_rx: Mutex<Receiver<BeaconObjective>>,
    state_rx: watch::Sender<BeaconControllerState>,
}

#[derive(Copy, Clone)]
pub enum BeaconControllerState {
    ActiveBeacons,
    NoActiveBeacons,
}

static BO_REGEX: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"(?i)ID[_, ]?(\d+).*?DISTANCE[_, ]?(([0-9]*[.])?[0-9]+)").unwrap()
});

impl BeaconController {
    const TIME_TO_NEXT_PASSIVE_CHECK: Duration = Duration::from_secs(30);
    const BEACON_OBJ_RETURN_WARNING: TimeDelta = TimeDelta::minutes(10);
    const THRESHOLD_GUESSES_TO_DONE: usize = 15;
    const BO_MSG_COMM_PROLONG: TimeDelta = TimeDelta::seconds(60);
    const MAX_ESTIMATE_GUESSES: usize = 5;

    pub fn new(
        rx_beac: Receiver<BeaconObjective>,
    ) -> (Self, watch::Receiver<BeaconControllerState>) {
        let (tx, rx) = watch::channel(BeaconControllerState::NoActiveBeacons);
        (
            Self {
                active_bo: RwLock::new(HashMap::new()),
                done_bo: RwLock::new(HashMap::new()),
                beacon_rx: Mutex::new(rx_beac),
                state_rx: tx,
            },
            rx,
        )
    }

    pub async fn run(self: Arc<Self>, handler: Arc<HTTPClient>) {
        let mut approaching_end_interval = interval(Self::TIME_TO_NEXT_PASSIVE_CHECK);
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

    pub async fn last_active_beac_end(&self) -> Option<DateTime<Utc>> {
        self.active_bo.read().await.values().map(BeaconObjective::end).max()
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

    pub async fn handle_poss_bo_ping(
        &self,
        msg: (DateTime<Utc>, String),
        f_cont: Arc<RwLock<FlightComputer>>,
    ) {
        let (t, val) = msg;
        if let Some((id, d_noisy)) = Self::extract_id_and_d(val.as_str()) {
            let f_cont_lock = f_cont.read().await;
            let pos = f_cont_lock.current_pos();

            let msg_delay = Utc::now() - t;
            let meas = BeaconMeas::new(id, pos, d_noisy, msg_delay);
            obj!("Received BO measurement at {pos} for ID {id} with distance {d_noisy}.");
            let mut active_lock = self.active_bo.write().await;
            if let Some(obj) = active_lock.get_mut(&id) {
                obj!("Updating BO {id} and prolonging!");
                obj.append_measurement(meas);
            } else {
                warn!("Unknown BO ID {id}. Ignoring!");
            }
        } else {
            event!("Message has unknown format {val:#?}. Ignoring.");
        }
    }

    async fn add_beacon(&self, obj: BeaconObjective) {
        obj!(
            "The Beacon {}-'{}' is lit! Gondor calls for Aid! Available Timeframe {} - {}.",
            obj.id(),
            obj.name(),
            obj.start().format("%d %H:%M:%S").to_string(),
            obj.end().format("%d %H:%M:%S").to_string()
        );
        let empty = self.active_bo.read().await.is_empty();
        self.active_bo.write().await.insert(obj.id(), obj);
        if empty {
            self.state_rx.send(BeaconControllerState::ActiveBeacons).expect("Failed to send state");
        }
    }

    pub async fn latest_active_beac_end(&self) -> DateTime<Utc> {
        self.active_bo.read().await.values().map(BeaconObjective::end).max().unwrap_or(Utc::now())
    }

    async fn move_to_done(&self, finished: HashMap<usize, BeaconObjective>) {
        let mut done_bo = self.done_bo.write().await;
        for (id, beacon) in finished {
            let done_beacon = BeaconObjectiveDone::from(beacon);
            let guesses = done_beacon.guesses().len();
            obj!("Finished Beacon objective: ID {id} with {guesses} guesses.");

            done_bo.insert(done_beacon.id(), done_beacon.clone());
        }
    }

    async fn check_approaching_end(&self, handler: &Arc<HTTPClient>) {
        let mut finished = HashMap::new();
        let deadline = Utc::now() + Self::TIME_TO_NEXT_PASSIVE_CHECK - TimeDelta::seconds(10);
        let no_more_beacons = {
            let mut active_beacon_tasks = self.active_bo.write().await;
            active_beacon_tasks.retain(|id, beacon: &mut BeaconObjective| {
                let finished_cond = beacon
                    .measurements()
                    .is_some_and(|b| b.guess_estimate() < Self::MAX_ESTIMATE_GUESSES);
                let deadline_cond = beacon.end() < deadline;
                if deadline_cond || finished_cond {
                    obj!(
                        "Active BO end is less than {} s away: ID {id}. Submitting this now!",
                        Self::TIME_TO_NEXT_PASSIVE_CHECK.as_secs(),
                    );
                    finished.insert(*id, beacon.clone());
                    false
                } else {
                    true
                }
            });
            active_beacon_tasks.is_empty()
        };
        self.move_to_done(finished).await;
        if no_more_beacons {
            self.state_rx
                .send(BeaconControllerState::NoActiveBeacons)
                .expect("Failed to send state");
        }
        self.handle_beacon_submission(handler).await;
    }

    async fn handle_beacon_submission(&self, handler: &Arc<HTTPClient>) {
        let mut done_beacons = self.done_bo.write().await.clone();
        for beacon in done_beacons.values_mut() {
            if !beacon.submitted() {
                if beacon.guesses().is_empty() {
                    beacon.randomize_no_meas_guesses(Arc::clone(handler)).await;
                } else {
                    beacon.guess_max(Arc::clone(handler)).await;
                }
            }
        }
    }
}
