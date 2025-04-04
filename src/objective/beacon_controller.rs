use super::{BeaconObjective, BeaconMeas, beacon_objective_done::BeaconObjectiveDone};
use crate::flight_control::FlightComputer;
use crate::http_handler::http_client::HTTPClient;
use crate::util::logger::JsonDump;
use crate::{event, obj, warn};
use chrono::{DateTime, TimeDelta, Utc};
use regex::Regex;
use std::{collections::HashMap, sync::{Arc, LazyLock}, time::Duration};
use tokio::{time::interval, sync::{mpsc::Receiver, Mutex, RwLock, watch}};

/// The [`BeaconController`] manages active and completed Beacon Objectives,
/// handles beacon measurements received via communication messages,
/// and submits results to the backend.
///
/// This controller supports:
/// - Tracking currently active beacon objectives
/// - Monitoring for objectives nearing their end
/// - Handling and filtering incoming ping messages
/// - Estimating distances from noisy measurements
/// - Submitting completed objectives through the endpoint
pub struct BeaconController {
    /// Map of active beacon objectives indexed by ID.
    active_bo: RwLock<HashMap<usize, BeaconObjective>>,
    /// Map of completed beacon objectives that were already submitted.
    done_bo: RwLock<HashMap<usize, BeaconObjectiveDone>>,
    /// Receiver channel for newly announced beacon objectives.
    beacon_rx: Mutex<Receiver<BeaconObjective>>,
    /// State broadcast channel for notifying listeners when beacon activity changes.
    state_rx: watch::Sender<BeaconControllerState>,
}

/// Enum representing whether any active beacon objectives are currently available.
#[derive(Copy, Clone)]
pub enum BeaconControllerState {
    /// At least one active beacon objective is being tracked.
    ActiveBeacons,
    /// No active beacon objectives are available.
    NoActiveBeacons,
}

/// Regular expression used to extract beacon ID and noisy distance value
/// from a ping message received via telemetry (e.g. `"ID 17 DISTANCE 242.5"`).
static BO_REGEX: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"(?i)ID[_, ]?(\d+).*?DISTANCE[_, ]?(([0-9]*[.])?[0-9]+)").unwrap()
});

impl BeaconController {
    /// Interval between automatic passive checks for near-expiring objectives.
    const TIME_TO_NEXT_PASSIVE_CHECK: Duration = Duration::from_secs(30);
    /// Maximum number of guesses allowed before beacon is considered resolved.
    const MAX_ESTIMATE_GUESSES: usize = 5;

    /// Creates a new [`BeaconController`] and associated state receiver.
    ///
    /// # Arguments
    /// * `rx_beac` – A receiver channel to receive newly active beacon objectives.
    ///
    /// # Returns
    /// A tuple `(BeaconController, watch::Receiver<BeaconControllerState>)`
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

    /// Starts the main controller loop:
    /// - Periodically checks for objectives nearing completion
    /// - Reacts to newly received beacon objectives
    ///
    /// Should be spawned as a background task.
    ///
    /// # Arguments
    /// * `handler` – A shared HTTP client for submitting finished objectives.
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

    /// Returns the latest end timestamp of all currently active beacon objectives.
    ///
    /// # Returns
    /// * `Some(DateTime)` if at least one active objective exists, `None` otherwise.
    pub async fn last_active_beac_end(&self) -> Option<DateTime<Utc>> {
        self.active_bo.read().await.values().map(BeaconObjective::end).max()
    }

    /// Attempts to extract a beacon ID and noisy distance from a telemetry message.
    ///
    /// # Arguments
    /// * `input` – The string message from the beacon communication system.
    ///
    /// # Returns
    /// * `Some((id, distance))` if parsing succeeds, `None` otherwise.
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

    /// Processes a received ping message during comms window.
    ///
    /// If the ID matches an active beacon, updates it with a new noisy measurement.
    ///
    /// # Arguments
    /// * `msg` – Tuple of timestamp and message string.
    /// * `f_cont` – Lock to the flight computer for obtaining position.
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

    /// Registers a newly received beacon objective into the active tracking list.
    ///
    /// Notifies downstream listeners if this is the first active beacon.
    ///
    /// # Arguments
    /// * `obj` – The received `BeaconObjective`.
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

    /// Moves finished objectives from `active_bo` to `done_bo`.
    ///
    /// Also logs and stores submission results.
    ///
    /// # Arguments
    /// * `finished` – Map of completed objectives to move.
    async fn move_to_done(&self, finished: HashMap<usize, BeaconObjective>) {
        let mut done_bo = self.done_bo.write().await;
        for (id, beacon) in finished {
            beacon.dump_json();
            let done_beacon = BeaconObjectiveDone::from(beacon);
            let guesses = done_beacon.guesses().len();
            obj!("Finished Beacon objective: ID {id} with {guesses} guesses.");
            done_bo.insert(done_beacon.id(), done_beacon.clone());
        }
    }

    /// Checks for objectives that are:
    /// - About to end within `TIME_TO_NEXT_PASSIVE_CHECK`
    /// - Have enough guesses already
    ///
    /// Submits them and updates internal state.
    ///
    /// # Arguments
    /// * `handler` – Shared HTTP client for submission.
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

    /// Handles submission of all completed (done) beacon objectives.
    ///
    /// Applies random guesses or estimates based on measurement data.
    ///
    /// # Arguments
    /// * `handler` – Shared HTTP client used to send results.
    async fn handle_beacon_submission(&self, handler: &Arc<HTTPClient>) {
        let mut done_beacons = self.done_bo.write().await;
        for beacon in done_beacons.values_mut() {
            if !beacon.submitted() {
                beacon.set_submitted();
                if beacon.guesses().is_empty() {
                    beacon.randomize_no_meas_guesses(Arc::clone(handler)).await;
                } else {
                    beacon.guess_max(Arc::clone(handler)).await;
                }
            }
        }
    }
}
