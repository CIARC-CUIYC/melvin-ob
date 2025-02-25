use crate::flight_control::objective::beacon_objective::BeaconObjective;
use crate::flight_control::objective::beacon_objective_done::BeaconObjectiveDone;
use crate::http_handler::http_client::HTTPClient;
use crate::mode_control::base_mode::{BaseMode, BaseWaitExitSignal};
use crate::{info, obj};
use chrono::{DateTime, TimeDelta, Utc};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Mutex;

pub enum BeaconControllerMode {
    Active,
    Passive,
}

pub struct BeaconController {
    active_beacons: HashMap<usize, BeaconObjective>,
    done_beacon: HashMap<usize, BeaconObjectiveDone>,
    controller_mode: BeaconControllerMode,
}

const TIME_TO_NEXT_PASSIVE_CHECK: Duration = Duration::from_secs(15);

const BEACON_OBJ_RETURN_MIN_DELAY: TimeDelta = TimeDelta::minutes(10);

impl BeaconController {
    pub fn new() -> Self {
        Self {
            active_beacons: HashMap::new(),
            done_beacon: HashMap::new(),
            controller_mode: BeaconControllerMode::Passive,
        }
    }

    pub async fn run(&self, handler: Arc<HTTPClient>) {
        loop {
            self.check_beacons(&handler);
            match self.controller_mode {
                BeaconControllerMode::Active => self.scan_active_beacons(),
                BeaconControllerMode::Passive => self.handle_passive_mode(&handler).await,
            }
        }
    }

    pub fn set_mode(&mut self, mode: BeaconControllerMode) { self.controller_mode = mode; }

    pub fn get_mode(&self) -> &BeaconControllerMode { &self.controller_mode }

    pub fn active_beacons(&self) -> &HashMap<usize, BeaconObjective> { &self.active_beacons }

    fn done_beacon(&self) -> &HashMap<usize, BeaconObjectiveDone> { &self.done_beacon }

    fn get_mut(&mut self, beacon_id: usize) -> Option<&mut BeaconObjective> {
        self.active_beacons.get_mut(&beacon_id)
    }

    pub(crate) fn add_beacon(&mut self, obj: BeaconObjective) {
        self.active_beacons.insert(obj.id(), obj);
    }

    fn remove_beacon(&mut self, beacon_id: usize) { self.active_beacons.remove(&beacon_id); }

    pub fn latest_objective_end(&self) -> DateTime<Utc> {
        self.active_beacons().values().map(BeaconObjective::end).max().unwrap_or(Utc::now())
    }

    fn move_to_done(&mut self, beacon: BeaconObjective) -> BeaconObjectiveDone {
        let done_beacon = BeaconObjectiveDone::from(beacon);
        self.active_beacons.remove(&done_beacon.id());
        self.done_beacon.insert(done_beacon.id(), done_beacon.clone());
        done_beacon
    }

    async fn handle_active_mode(&mut self) {}

    async fn handle_passive_mode(&mut self, handler: &Arc<HTTPClient>) {
        self.check_approaching_end(handler).await;
        tokio::time::sleep(TIME_TO_NEXT_PASSIVE_CHECK).await;
    }

    pub async fn check_approaching_end(
        &mut self,
        handler: &Arc<HTTPClient>,
    ) -> Option<BaseWaitExitSignal> {
        let active_beacon_tasks = self.active_beacons.clone();
        for active_beacon in active_beacon_tasks {
            if active_beacon.1.end() < Utc::now() + BEACON_OBJ_RETURN_MIN_DELAY {
                let done_beac = self.move_to_done(active_beacon.1.clone());
                if done_beac.guesses().is_empty() {
                    obj!(
                        "Almost ending Beacon objective: ID {}. No guesses :(",
                        active_beacon.0
                    );
                }
                // TODO: This does not submit anything by itself!
                obj!(
                    "Almost ending Beacon objective: ID {}. Submitting this soon!",
                    active_beacon.0
                );
                continue;
            } else if let Some(meas) = active_beacon.1.measurements() {
                let min_guesses = meas.guess_estimate();
                obj!(
                    "ID {} has {} min guesses.",
                    active_beacon.1.id(),
                    min_guesses
                );
                if min_guesses < 15 {
                    let beac_done = BeaconObjectiveDone::from(active_beacon.1.clone());
                    obj!(
                        "Finished Beacon objective: ID {} has {} guesses.",
                        active_beacon.0,
                        beac_done.guesses().len()
                    );
                }
                continue;
            }
        }

        if !self.done_beacon.is_empty() {
            let signal = self.handle_beacon_submission(handler).await;
            Option::from(signal)
        } else {
            None
        }
    }

    async fn handle_beacon_submission(&mut self, handler: &Arc<HTTPClient>) -> BaseWaitExitSignal {
        let done_beacons = self.done_beacon.clone();
        for done_beacon in done_beacons {
            obj!("Submitting Beacon Objective: {}", done_beacon.1.id());
            done_beacon.1.guess_max(Arc::clone(handler)).await;
        }
        if self.active_beacons.is_empty() {
            info!("All Beacon Objectives done! Switching to Mapping Mode.");
            BaseWaitExitSignal::ReturnAllDone
        } else {
            info!("There are still Beacon Objectives left. Waiting for them to finish!");
            BaseWaitExitSignal::ReturnSomeLeft
        }
    }
}
