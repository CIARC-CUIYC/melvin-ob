use crate::flight_control::common::pinned_dt::PinnedTimeDelay;
use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::flight_state::FlightState;
use crate::flight_control::task::base_task::Task;
use crate::flight_control::{
    common::{linked_box::LinkedBox, vec2d::Vec2D},
    orbit::closed_orbit::ClosedOrbit,
};
use crate::{MAX_BATTERY_THRESHOLD, MIN_BATTERY_THRESHOLD};
use chrono::Duration;
use std::collections::VecDeque;
use std::sync::{Arc, Condvar};
use tokio::sync::{Mutex, RwLock};

/// `TaskController` manages and schedules image capture tasks for a satellite.
/// It leverages a thread-safe task queue and notifies waiting threads when
/// new tasks are added or processed.
///
/// # Fields
/// - `image_schedule`: An `Arc` Reference to a `LockedTaskQueue`.
/// - `next_image_notify`: An `Arc` Reference to a `Condvar` indicating changes to the first element
///   in `image_schedule`
#[derive(Debug)]
pub struct TaskController {
    /// Schedule for the next images, represented by image tasks.
    task_schedule: Arc<Mutex<VecDeque<Task>>>,
    /// Notification condition variable to signal changes to the first element in `image_schedule`.
    next_task_notify: Arc<Condvar>,
}

#[derive(Debug, Clone, Copy)]
enum AtomicDecision {
    StayInCharge,
    StayInAcquisition,
    SwitchToCharge,
    SwitchToAcquisition,
}

type AtomicDecisionBox = Box<[AtomicDecision]>; // A layer of AtomicDecision
type AtomicDecisionGrid = Box<[AtomicDecisionBox]>; // A "2D" grid of AtomicDecision
type AtomicDecisionCube = Box<[AtomicDecisionGrid]>;

type CoverageGrid = Box<[Box<[u16]>]>;

struct OptimalOrbitResult {
    pub decisions: AtomicDecisionCube,
    pub coverage_slice: LinkedBox<CoverageGrid>,
}

impl TaskController {
    const MAX_ORBIT_PREDICTION_SECS: u32 = 80000;
    const BATTERY_RESOLUTION: f32 = 0.1;
    const TIME_RESOLUTION: f32 = 1.0;

    /// Creates a new instance of the `TaskController` struct.
    ///
    /// # Returns
    /// - A new `TaskController` with an empty task schedule.
    pub fn new() -> Self {
        Self {
            task_schedule: Arc::new(Mutex::new(VecDeque::new())),
            next_task_notify: Arc::new(Condvar::new()),
        }
    }

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    fn calculate_optimal_orbit_schedule(
        orbit: &ClosedOrbit,
        p_t_shift: usize,
    ) -> OptimalOrbitResult {
        let states = [FlightState::Charge, FlightState::Acquisition];
        let usable_batt_range = MAX_BATTERY_THRESHOLD - MIN_BATTERY_THRESHOLD;
        let max_battery = (usable_batt_range / Self::BATTERY_RESOLUTION).round() as usize;

        let prediction_secs = Self::MAX_ORBIT_PREDICTION_SECS.min(orbit.period().0 as u32) as usize;

        let mut p_t_iter = orbit
            .get_p_t_reordered(p_t_shift)
            .rev()
            .skip(orbit.period().0 as usize - prediction_secs);

        // initiate buffers
        let cov_dt_temp =
            vec![vec![0u16; 2].into_boxed_slice(); max_battery + 1].into_boxed_slice();
        let mut decision_buffer =
            vec![
                vec![vec![AtomicDecision::StayInCharge; 2].into_boxed_slice(); max_battery + 1]
                    .into_boxed_slice();
                prediction_secs
            ]
            .into_boxed_slice();
        // initiate fixed-length double linked list with first value
        let mut max_cov_buffer: LinkedBox<CoverageGrid> = LinkedBox::new(180);
        max_cov_buffer.push(cov_dt_temp.clone());
        for t in (0..prediction_secs).rev() {
            let mut cov_dt = cov_dt_temp.clone();
            let p_dt = u16::from(!*p_t_iter.next().unwrap());
            for e in 0..=max_battery {
                for s in &states {
                    match *s {
                        FlightState::Charge => {
                            let stay = max_cov_buffer.front().unwrap()[(e + 1).min(max_battery)][0];
                            let switch = max_cov_buffer.back().unwrap()[e][1];
                            if stay >= switch {
                                decision_buffer[t][e][0] = AtomicDecision::StayInCharge;
                                cov_dt[e][0] = stay;
                            } else {
                                decision_buffer[t][e][0] = AtomicDecision::SwitchToAcquisition;
                                cov_dt[e][0] = switch;
                            }
                        }
                        FlightState::Acquisition => {
                            let switch = max_cov_buffer.back().unwrap()[e][0];
                            let stay = if e > 0 {
                                max_cov_buffer.front().unwrap()[e - 1][1] + p_dt
                            } else {
                                0
                            };

                            if e > 0 && stay >= switch {
                                decision_buffer[t][e][1] = AtomicDecision::StayInAcquisition;
                                cov_dt[e][1] = stay;
                            } else {
                                decision_buffer[t][e][1] = AtomicDecision::SwitchToCharge;
                                cov_dt[e][1] = switch;
                            }
                        }
                        _ => break,
                    }
                }
            }
            max_cov_buffer.push(cov_dt);
        }
        OptimalOrbitResult {
            decisions: decision_buffer,
            coverage_slice: max_cov_buffer,
        }
    }

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss, clippy::cast_precision_loss)]
    pub async fn schedule_optimal_orbit(
        &mut self,
        orbit_lock: Arc<Mutex<ClosedOrbit>>,
        f_cont_lock: Arc<RwLock<FlightComputer>>,
        p_t_shift: usize,
    ) {
        let computation_start = chrono::Utc::now();
        println!("[INFO] Calculating optimal orbit schedule...");
        let (decisions, orbit_period) = {
            let orbit = orbit_lock.lock().await;
            (Self::calculate_optimal_orbit_schedule(&orbit, p_t_shift), orbit.period())
            
        };
        let dt_calc = (chrono::Utc::now() - computation_start).num_milliseconds() as f32 / 1000.0;
        println!("[INFO] Optimal Orbit Calculation complete after {dt_calc:.2}");
        let mut dt = dt_calc.ceil() as usize;
        let (batt_f32, mut state) = {
            let f_cont = f_cont_lock.read().await;
            let batt: f32 = f_cont.current_battery();
            let st: usize = match f_cont.state() {
                FlightState::Acquisition => 1,
                FlightState::Charge => 0,
                state => panic!("[FATAL] Unexpected flight state: {state}"),
            };
            (batt, st)
        };
        let (min_batt, max_batt) = (MIN_BATTERY_THRESHOLD, MAX_BATTERY_THRESHOLD);
        let max_mapped = (max_batt / Self::BATTERY_RESOLUTION - min_batt / Self::BATTERY_RESOLUTION)
            .round() as i32;
        let mut batt = ((batt_f32 - min_batt) / Self::BATTERY_RESOLUTION) as usize;

        let pred_secs = Self::MAX_ORBIT_PREDICTION_SECS.min(orbit_period.0 as u32) as usize;
        let mut decision_list: Vec<AtomicDecision> = Vec::new();
        while dt < pred_secs {
            let decision = decisions.decisions[dt][batt][state];
            decision_list.push(decision);
            match decision {
                AtomicDecision::StayInCharge => {
                    state = 0;
                    batt = (batt + 1).min(max_mapped as usize);
                    dt += 1;
                }
                AtomicDecision::StayInAcquisition => {
                    state = 1;
                    batt -= 1;
                    dt += 1;
                }
                AtomicDecision::SwitchToCharge => {
                    let sched_t = computation_start + Duration::seconds(dt as i64);
                    self.schedule_switch(FlightState::Charge, sched_t).await;
                    state = 0;
                    dt = (dt + 180).min(pred_secs);
                }
                AtomicDecision::SwitchToAcquisition => {
                    let sched_t = computation_start + Duration::seconds(dt as i64);
                    self.schedule_switch(FlightState::Acquisition, sched_t).await;
                    state = 1;
                    dt = (dt + 180).min(pred_secs);
                }
            }
        }
        println!(
            "[INFO] Number of tasks after scheduling: {}",
            self.task_schedule.lock().await.len()
        );
    }

    /// Provides a reference to the image task schedule.
    ///
    /// # Returns
    /// - An `Arc` pointing to the `LockedTaskQueue`.
    pub fn sched_arc(&self) -> Arc<Mutex<VecDeque<Task>>> { Arc::clone(&self.task_schedule) }

    /// Provides a reference to the `Convar` signaling changes to the first item in `image_schedule`.
    ///
    /// # Returns
    /// - An `Arc` pointing to the `Condvar`.
    pub fn notify_arc(&self) -> Arc<Condvar> { Arc::clone(&self.next_task_notify) }

    async fn schedule_switch(
        &mut self,
        target: FlightState,
        sched_t: chrono::DateTime<chrono::Utc>,
    ) {
        if self.task_schedule.lock().await.is_empty() {
            self.next_task_notify.notify_all();
        }
        let dt = PinnedTimeDelay::from_end(sched_t);
        self.task_schedule.lock().await.push_back(Task::switch_target(target, dt));
    }

    /// Clears all pending tasks in the schedule.
    ///
    /// # Side Effects
    /// - Notifies all waiting threads if the schedule is cleared.
    pub async fn clear_schedule(&mut self) {
        let schedule = &*self.task_schedule;
        if !schedule.lock().await.is_empty() {
            self.next_task_notify.notify_all();
        }
        schedule.lock().await.clear();
    }
}
