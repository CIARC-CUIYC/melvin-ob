use crate::flight_control::common::math;
use crate::flight_control::common::vec2d::Vec2D;
use crate::flight_control::orbit::index::IndexedOrbitPosition;
use crate::flight_control::task::vel_change_task::VelocityChangeType;
use crate::flight_control::{
    common::{linked_box::LinkedBox, pinned_dt::PinnedTimeDelay},
    flight_computer::FlightComputer,
    flight_state::FlightState,
    orbit::closed_orbit::ClosedOrbit,
    task::base_task::Task,
};
use crate::http_handler::ZonedObjective;
use crate::{MAX_BATTERY_THRESHOLD, MIN_BATTERY_THRESHOLD};
use num::traits::float::FloatCore;
use std::{
    collections::VecDeque,
    sync::{Arc, Condvar},
};
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
    const OBJECTIVE_SCHEDULE_MIN_DT: usize = 200;
    const OBJECTIVE_MIN_RETRIEVAL_TOL: usize = 100;
    const OFF_ORBIT_DT_WEIGHT: f32 = 2.0;
    const FUEL_CONSUMPTION_WEIGHT: f32 = 1.0;
    /// Default magin number for the initialization of min_maneuver_time
    const DEF_MAX_MANEUVER_TIME: i64 = 1_000_000_000;
    /// Maximum absolute deviation after correction burn
    const MAX_AFTER_CB_DEV: f32 = 1.0;

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
        // TODO: optimize vecs with custom indexed struct in one dimension (Cache optimization)
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

    #[allow(clippy::cast_possible_truncation)]
    pub fn calculate_orbit_correction_burn(
        initial_vel: Vec2D<f32>,
        deviation: Vec2D<f32>,
        due: PinnedTimeDelay,
    ) -> (Vec<Vec2D<f32>>, i64, Vec2D<f32>) {
        let is_clockwise = initial_vel.is_clockwise_to(&deviation).unwrap_or(false);

        let min_maneuver_time = chrono::TimeDelta::seconds(Self::DEF_MAX_MANEUVER_TIME);
        let mut max_acc_secs = 1;
        let mut best_maneuver = Vec::new();
        let mut best_min_dev = Vec2D::new(f32::infinity(), f32::infinity());
        let mut best_vel_hold_dt = 0;
        let mut last_vel = initial_vel;
        while min_maneuver_time > due.time_left() {
            let mut res_vel_diff = Vec2D::<f32>::zero();
            let mut remaining_deviation = deviation;
            let mut current_maneuver = Vec::new();
            for acc_secs in 0..max_acc_secs {
                let perp_acc = last_vel.perp_unit(is_clockwise) * FlightComputer::ACC_CONST;
                let new_vel = FlightComputer::trunc_vel(last_vel + perp_acc);
                current_maneuver.push(new_vel);
                let vel_diff = new_vel - last_vel;
                res_vel_diff = res_vel_diff + vel_diff;
                remaining_deviation = remaining_deviation - res_vel_diff * 2;
                last_vel = new_vel;
            }

            let x_vel_hold_dt = (remaining_deviation.x() / res_vel_diff.x()).floor();
            let y_vel_hold_dt = (remaining_deviation.y() / res_vel_diff.y()).floor();
            let min_vel_hold_dt = (x_vel_hold_dt.min(y_vel_hold_dt)) as i64;

            if min_vel_hold_dt + max_acc_secs > due.time_left().num_seconds() {
                continue;
            }

            let max_x_dev = (remaining_deviation - res_vel_diff * x_vel_hold_dt);
            let max_y_dev = (remaining_deviation - res_vel_diff * y_vel_hold_dt);

            let (min_t, res_dev) = math::find_min_y_for_x_range(
                x_vel_hold_dt,
                max_x_dev.into(),
                y_vel_hold_dt,
                max_y_dev.into(),
            );
            let res_dev_vec = Vec2D::from(res_dev);
            let min_t_i64 = min_t.floor() as i64;
            if best_min_dev.abs() > res_dev_vec.abs() {
                best_min_dev = res_dev_vec;
                best_maneuver = current_maneuver;
                best_vel_hold_dt = min_t_i64;
            }
            if best_min_dev.abs() < Self::MAX_AFTER_CB_DEV {
                break;
            }
            max_acc_secs += 1;
        }
        (best_maneuver, best_vel_hold_dt, best_min_dev)
    }

    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss,
        clippy::cast_possible_wrap
    )]
    pub async fn calculate_single_point_maneuver(
        orbit_lock: Arc<Mutex<ClosedOrbit>>,
        curr_i: IndexedOrbitPosition,
        f_cont_lock: Arc<RwLock<FlightComputer>>,
        target_pos: Vec2D<f32>,
        target_end_time: chrono::DateTime<chrono::Utc>,
    ) {
        let computation_start = chrono::Utc::now();
        let time_left = target_end_time - chrono::Utc::now();
        let max_dt = {
            let max = usize::try_from(time_left.num_seconds()).unwrap_or(0);
            max - Self::OBJECTIVE_MIN_RETRIEVAL_TOL
        };
        let (due_i, orbit_vel) = {
            let f_cont = f_cont_lock.read().await;
            (f_cont.pos_in_dt(curr_i, time_left), f_cont.current_vel())
        };
        let max_off_orbit_t = max_dt - Self::OBJECTIVE_SCHEDULE_MIN_DT;

        // TODO: precalculate possible velocity changes
        let turns_handle =
            tokio::spawn(async move { FlightComputer::compute_possible_turns(orbit_vel) });

        let orbit_vel_abs = orbit_vel.abs();
        let mut last_possible_dt = 0;
        for dt in (Self::OBJECTIVE_SCHEDULE_MIN_DT..max_dt).rev() {
            let pos = (curr_i.pos() + orbit_vel * dt).wrap_around_map();
            let to_target = pos.unwrapped_to(&target_pos);
            let min_dt = (to_target.abs() / orbit_vel_abs).abs().round() as usize;

            if min_dt + dt < max_dt {
                last_possible_dt = dt;
                break;
            }
        }

        let remaining_range = (Self::OBJECTIVE_SCHEDULE_MIN_DT..=last_possible_dt);
        let offset = *remaining_range.start();
        let mut possible_orbit_changes = vec![None; remaining_range.end() - offset];

        let turns = turns_handle.await.unwrap();
        for dt in remaining_range.rev() {
            let mut next_pos = (curr_i.pos() + orbit_vel * dt).wrap_around_map();
            let maneuver_start =
                curr_i.new_from_future_pos(next_pos, chrono::TimeDelta::seconds(dt as i64));
            let direction_vec = next_pos.unwrapped_to(&target_pos);

            let (turns_in_dir, break_cond) = {
                if direction_vec.is_clockwise_to(&orbit_vel).unwrap_or(false) {
                    (&turns.0, false)
                } else {
                    (&turns.1, true)
                }
            };

            let mut add_dt = 0;
            let mut fin_sequence: Vec<(Vec2D<f32>, Vec2D<f32>)> = Vec::new();
            let mut fin_angle_dev = 0.0;
            let mut fin_dt = 0;
            let max_add_dt = turns_in_dir.len();

            'inner: for atomic_turn in turns_in_dir {
                next_pos = next_pos + atomic_turn.0;
                let next_vel = atomic_turn.1;

                let next_to_target = next_pos.unwrapped_to(&target_pos);
                let min_dt = (next_to_target.abs() / next_vel.abs()).abs().round() as usize;

                if min_dt + dt + add_dt > max_dt {
                    break 'inner;
                }

                if direction_vec.is_clockwise_to(&next_vel).unwrap_or(break_cond) == break_cond {
                    let (last_pos, last_vel) = fin_sequence.last().unwrap();
                    (fin_dt, fin_angle_dev) = {
                        let last_angle_deviation = last_vel.angle_to(&next_to_target);
                        let this_angle_deviation = next_vel.angle_to(&next_to_target);
                        if last_angle_deviation > this_angle_deviation {
                            fin_sequence.push((next_pos, next_vel));
                            (min_dt + dt + add_dt, this_angle_deviation)
                        } else {
                            let last_to_target = last_pos.unwrapped_to(&target_pos);
                            let last_min_dt =
                                (last_to_target.abs() / last_vel.abs()).abs().round() as usize;
                            (last_min_dt + dt + add_dt, last_angle_deviation)
                        }
                    };
                } else {
                    fin_sequence.push((next_pos, next_vel));
                    add_dt += 1;
                }
            }
            possible_orbit_changes[dt - offset] =
                (Some((dt, fin_dt, add_dt, fin_angle_dev, fin_sequence)));
            let normalized_fuel_consumption = math::normalize_f32(
                add_dt as f32 * FlightComputer::FUEL_CONST,
                0.0,
                max_add_dt as f32 * FlightComputer::FUEL_CONST,
            )
            .unwrap();
            let normalized_off_orbit_t =
                math::normalize_f32((fin_dt - dt) as f32, 0.0, max_off_orbit_t as f32).unwrap();
            let orbit_score = Self::OFF_ORBIT_DT_WEIGHT * normalized_off_orbit_t
                + Self::FUEL_CONSUMPTION_WEIGHT * normalized_fuel_consumption;
        }
    }

    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss,
        clippy::cast_possible_wrap
    )]
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
            (
                Self::calculate_optimal_orbit_schedule(&orbit, p_t_shift),
                orbit.period(),
            )
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
                    let sched_t = computation_start + chrono::TimeDelta::seconds(dt as i64);
                    self.schedule_switch(FlightState::Charge, sched_t).await;
                    state = 0;
                    dt = (dt + 180).min(pred_secs);
                }
                AtomicDecision::SwitchToAcquisition => {
                    let sched_t = computation_start + chrono::TimeDelta::seconds(dt as i64);
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

    async fn schedule_vel_change(
        &mut self,
        vel: Box<[Vec2D<f32>]>,
        sched_t: chrono::DateTime<chrono::Utc>,
    ) {
        if self.task_schedule.lock().await.is_empty() {
            self.next_task_notify.notify_all();
        }
        let dt = PinnedTimeDelay::from_end(sched_t);
        if vel.len() == 1 {
            self.task_schedule.lock().await.push_back(Task::vel_change_task(
                VelocityChangeType::AtomicVelChange(vel[0]),
                dt,
            ));
        } else {
            self.task_schedule.lock().await.push_back(Task::vel_change_task(
                VelocityChangeType::SequentialVelChange(vel),
                dt,
            ));
        }
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
