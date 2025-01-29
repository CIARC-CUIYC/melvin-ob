use crate::flight_control::{
    common::{linked_box::LinkedBox, math, pinned_dt::PinnedTimeDelay, vec2d::Vec2D},
    flight_computer::FlightComputer,
    flight_state::FlightState,
    orbit::{burn_sequence::BurnSequence, closed_orbit::ClosedOrbit, index::IndexedOrbitPosition},
    task::{
        atomic_decision::AtomicDecision, atomic_decision_cube::AtomicDecisionCube, base_task::Task,
        score_grid::ScoreGrid,
    },
};
use crate::http_handler::ZonedObjective;
use crate::{MAX_BATTERY_THRESHOLD, MIN_BATTERY_THRESHOLD};
use bitvec::prelude::BitRef;
use num::traits::float::FloatCore;
use std::{
    collections::VecDeque,
    fmt::Debug,
    sync::{Arc, Condvar},
};
use tokio::sync::RwLock;

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
    task_schedule: Arc<RwLock<VecDeque<Task>>>,
    /// Notification condition variable to signal changes to the first element in `image_schedule`.
    next_task_notify: Arc<Condvar>,
}

struct OptimalOrbitResult {
    pub decisions: AtomicDecisionCube,
    pub coverage_slice: LinkedBox<ScoreGrid>,
}

impl TaskController {
    const MAX_ORBIT_PREDICTION_SECS: u32 = 80000;
    const BATTERY_RESOLUTION: f32 = 0.1;
    const TIME_RESOLUTION: f32 = 1.0;
    const OBJECTIVE_SCHEDULE_MIN_DT: usize = 1000;
    const OBJECTIVE_MIN_RETRIEVAL_TOL: usize = 100;
    const MANEUVER_INIT_BATT_TOL: f32 = 10.0;
    pub(crate) const MANEUVER_MIN_DETUMBLE_DT: usize = 50;
    const OFF_ORBIT_DT_WEIGHT: f32 = 2.0;
    const FUEL_CONSUMPTION_WEIGHT: f32 = 1.0;
    const ANGLE_DEV_WEIGHT: f32 = 1.5;
    /// Default magic number for the initialization of `min_burn_sequence_time`
    const DEF_MAX_BURN_SEQUENCE_TIME: i64 = 1_000_000_000;
    /// Maximum absolute deviation after correction burn
    const MAX_AFTER_CB_DEV: f32 = 1.0;

    /// Creates a new instance of the `TaskController` struct.
    ///
    /// # Returns
    /// - A new `TaskController` with an empty task schedule.
    pub fn new() -> Self {
        Self {
            task_schedule: Arc::new(RwLock::new(VecDeque::new())),
            next_task_notify: Arc::new(Condvar::new()),
        }
    }

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    fn init_orbit_sched_calc(
        orbit: &ClosedOrbit,
        p_t_shift: usize,
        dt: Option<usize>,
        end_status: Option<(FlightState, f32)>,
    ) -> OptimalOrbitResult {
        let states = [FlightState::Charge, FlightState::Acquisition];
        let usable_batt_range = MAX_BATTERY_THRESHOLD - MIN_BATTERY_THRESHOLD;
        let max_battery = (usable_batt_range / Self::BATTERY_RESOLUTION).round() as usize;

        let prediction_secs = {
            let max_pred_secs =
                Self::MAX_ORBIT_PREDICTION_SECS.min(orbit.period().0 as u32) as usize;
            if let Some(pred_secs) = dt {
                max_pred_secs.min(pred_secs)
            } else {
                max_pred_secs
            }
        };

        let p_t_iter =
            orbit.get_p_t_reordered(p_t_shift, orbit.period().0 as usize - prediction_secs);

        let decision_buffer =
            AtomicDecisionCube::new(prediction_secs, max_battery + 1, states.len());
        let cov_dt_temp = ScoreGrid::new(max_battery + 1, states.len());
        let cov_dt_first = {
            if let Some(end) = end_status {
                let end_batt_clamp = end.1.clamp(MIN_BATTERY_THRESHOLD, MAX_BATTERY_THRESHOLD);
                let end_batt = (end_batt_clamp / Self::BATTERY_RESOLUTION).round() as usize;
                let end_state = end.0 as usize;
                let end_cast = (end_state, end_batt);
                ScoreGrid::new_from_condition(max_battery + 1, states.len(), end_cast)
            } else {
                cov_dt_temp.clone()
            }
        };
        let mut score_cube = LinkedBox::new(180);
        score_cube.push(cov_dt_first);
        Self::calculate_optimal_orbit_schedule(
            prediction_secs,
            Box::new(p_t_iter),
            score_cube,
            &cov_dt_temp,
            decision_buffer,
        )
    }

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss, clippy::cast_possible_wrap)]
    fn calculate_optimal_orbit_schedule<'a>(
        pred_dt: usize,
        mut p_t_it: impl Iterator<Item = BitRef<'a>>,
        mut score_cube: LinkedBox<ScoreGrid>,
        score_grid_default: &ScoreGrid,
        mut dec_cube: AtomicDecisionCube,
    ) -> OptimalOrbitResult {
        let max_battery = score_grid_default.e_len() - 1;
        for t in (0..pred_dt).rev() {
            let mut cov_dt = score_grid_default.clone();
            let p_dt = u16::from(!*p_t_it.next().unwrap());
            for e in 0..=max_battery {
                for s in 0..=1 {
                    let de = if s == 0 { 1 } else { -1 };
                    let new_e = (e as isize + de) as usize;
                    let stay = {
                        if s == 0 {
                            score_cube.front().unwrap().get(new_e.min(max_battery), s)
                        } else if e > 0 {
                            score_cube.front().unwrap().get(new_e, s) + i32::from(p_dt)
                        } else {
                            i32::MIN
                        }
                    };
                    let switch = score_cube.back().unwrap().get(e, s ^ 1);

                    if stay >= switch {
                        dec_cube.set(t, e, s, AtomicDecision::stay(s));
                        cov_dt.set(e, s, stay);
                    } else {
                        dec_cube.set(t, e, s, AtomicDecision::switch(s ^ 1));
                        cov_dt.set(e, s, switch);
                    }
                }
            }
            score_cube.push(cov_dt);
        }
        OptimalOrbitResult {
            decisions: dec_cube,
            coverage_slice: score_cube,
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn calculate_orbit_correction_burn(
        initial_vel: Vec2D<f32>,
        deviation: Vec2D<f32>,
        due: PinnedTimeDelay,
    ) -> (Vec<Vec2D<f32>>, i64, Vec2D<f32>) {
        let is_clockwise = initial_vel.is_clockwise_to(&deviation).unwrap_or(false);

        let min_burn_sequence_time = chrono::TimeDelta::seconds(Self::DEF_MAX_BURN_SEQUENCE_TIME);
        let mut max_acc_secs = 1;
        let mut best_burn_sequence = Vec::new();
        let mut best_min_dev = Vec2D::new(f32::infinity(), f32::infinity());
        let mut best_vel_hold_dt = 0;
        let mut last_vel = initial_vel;
        while min_burn_sequence_time > due.time_left() {
            let mut res_vel_diff = Vec2D::<f32>::zero();
            let mut remaining_deviation = deviation;
            let mut current_burn_sequence = Vec::new();
            for _ in 0..max_acc_secs {
                let perp_acc = last_vel.perp_unit(is_clockwise) * FlightComputer::ACC_CONST;
                let (new_vel, _) = FlightComputer::trunc_vel(last_vel + perp_acc);
                current_burn_sequence.push(new_vel);
                let vel_diff = new_vel - last_vel;
                res_vel_diff = res_vel_diff + vel_diff;
                remaining_deviation = remaining_deviation - res_vel_diff * 2;
                last_vel = new_vel;
            }

            let x_vel_hold_dt = (remaining_deviation.x() / res_vel_diff.x()).floor();
            let y_vel_hold_dt = (remaining_deviation.y() / res_vel_diff.y()).floor();
            let min_vel_hold_dt = x_vel_hold_dt.min(y_vel_hold_dt) as i64;

            if min_vel_hold_dt + max_acc_secs > due.time_left().num_seconds() {
                continue;
            }

            let max_x_dev = remaining_deviation - res_vel_diff * x_vel_hold_dt;
            let max_y_dev = remaining_deviation - res_vel_diff * y_vel_hold_dt;

            let (min_t, res_dev) = math::find_min_y_abs_for_x_range(
                x_vel_hold_dt,
                max_x_dev.into(),
                y_vel_hold_dt,
                max_y_dev.into(),
            );
            let res_dev_vec = Vec2D::from(res_dev);
            let min_t_i64 = min_t.floor() as i64;
            if best_min_dev.abs() > res_dev_vec.abs() {
                best_min_dev = res_dev_vec;
                best_burn_sequence = current_burn_sequence;
                best_vel_hold_dt = min_t_i64;
            }
            if best_min_dev.abs() < Self::MAX_AFTER_CB_DEV {
                break;
            }
            max_acc_secs += 1;
        }
        (best_burn_sequence, best_vel_hold_dt, best_min_dev)
    }

    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss,
        clippy::cast_possible_wrap,
        clippy::too_many_lines
    )]
    pub async fn calculate_single_target_burn_sequence(
        curr_i: IndexedOrbitPosition,
        f_cont_lock: Arc<RwLock<FlightComputer>>,
        target_pos: Vec2D<f32>,
        target_end_time: chrono::DateTime<chrono::Utc>,
    ) -> (BurnSequence, f32) {
        println!("[INFO] Starting to calculate single target burn towards {target_pos}");
        let time_left = target_end_time - chrono::Utc::now();
        let max_dt = {
            let max = usize::try_from(time_left.num_seconds()).unwrap_or(0);
            max - Self::OBJECTIVE_MIN_RETRIEVAL_TOL
        };

        let orbit_vel = {
            let f_cont = f_cont_lock.read().await;
            f_cont.current_vel()
        };

        let max_off_orbit_t = max_dt - Self::OBJECTIVE_SCHEDULE_MIN_DT;

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

        println!(
            "[INFO] Done skipping impossible start times. Last possible dt: {last_possible_dt}"
        );

        let remaining_range = Self::OBJECTIVE_SCHEDULE_MIN_DT..=last_possible_dt;
        let mut best_burn_sequence: Option<(BurnSequence, f32)> = None;
        let max_angle_dev = {
            let vel_perp = orbit_vel.perp_unit(true) * FlightComputer::ACC_CONST;
            orbit_vel.angle_to(&vel_perp).abs()
        };

        let turns = turns_handle.await.unwrap();
        'outer: for dt in remaining_range.rev() {
            let mut next_pos = (curr_i.pos() + orbit_vel * dt).wrap_around_map();
            let burn_sequence_i =
                curr_i.new_from_future_pos(next_pos, chrono::TimeDelta::seconds(dt as i64));
            let direction_vec = next_pos.unwrapped_to(&target_pos);

            if orbit_vel.angle_to(&direction_vec).abs() > 90.0 {
                continue;
            }

            let (turns_in_dir, break_cond) = {
                if direction_vec.is_clockwise_to(&orbit_vel).unwrap_or(false) {
                    (&turns.0, false)
                } else {
                    (&turns.1, true)
                }
            };

            let mut add_dt = 0;
            let mut fin_sequence_pos: Vec<Vec2D<f32>> = vec![next_pos];
            let mut fin_sequence_vel: Vec<Vec2D<f32>> = vec![orbit_vel];
            let mut fin_angle_dev = 0.0;
            let mut fin_dt = 0;
            let max_add_dt = turns_in_dir.len();

            'inner: for atomic_turn in turns_in_dir {
                next_pos = (next_pos + atomic_turn.0).wrap_around_map();
                let next_vel = atomic_turn.1;

                let next_to_target = next_pos.unwrapped_to(&target_pos);
                let min_dt = (next_to_target.abs() / next_vel.abs()).abs().round() as usize;

                if min_dt + dt + add_dt > max_dt {
                    continue 'outer;
                }

                if next_to_target.is_clockwise_to(&next_vel).unwrap_or(break_cond) == break_cond {
                    let last_pos = fin_sequence_pos.last().unwrap();
                    let last_vel = fin_sequence_vel.last().unwrap();
                    (fin_dt, fin_angle_dev) = {
                        let last_to_target = last_pos.unwrapped_to(&target_pos);
                        let last_angle_deviation = last_vel.angle_to(&last_to_target);
                        let this_angle_deviation = next_vel.angle_to(&next_to_target);

                        let corr_burn_perc = math::interpolate(
                            last_angle_deviation,
                            this_angle_deviation,
                            0.0,
                            1.0,
                            0.0
                        );
                        let acc = (next_vel - *last_vel) * corr_burn_perc;
                        let (corr_vel, _) = 
                            FlightComputer::trunc_vel(next_vel + acc);
                        let corr_pos = *last_pos + corr_vel;
                        let corr_to_target = corr_pos.unwrapped_to(&target_pos);
                        let corr_angle_dev = corr_vel.angle_to(&corr_to_target);
                            fin_sequence_pos.push(corr_pos);
                            fin_sequence_vel.push(corr_vel);
                            add_dt += 1;
                        (min_dt + dt + add_dt, corr_angle_dev)
                    };
                    break 'inner;
                }
                fin_sequence_pos.push(next_pos);
                fin_sequence_vel.push(next_vel);
                add_dt += 1;
            }
            let normalized_fuel_consumption = math::normalize_f32(
                add_dt as f32 * FlightComputer::FUEL_CONST,
                0.0,
                max_add_dt as f32 * FlightComputer::FUEL_CONST,
            )
            .unwrap_or(0.0);

            let normalized_off_orbit_t =
                math::normalize_f32((fin_dt - dt) as f32, 0.0, max_off_orbit_t as f32)
                    .unwrap_or(0.0);
            let normalized_angle_dev =
                math::normalize_f32(fin_angle_dev.abs(), 0.0, max_angle_dev).unwrap_or(0.0);
            let burn_sequence_cost = Self::OFF_ORBIT_DT_WEIGHT * normalized_off_orbit_t
                + Self::FUEL_CONSUMPTION_WEIGHT * normalized_fuel_consumption
                + Self::ANGLE_DEV_WEIGHT * normalized_angle_dev;

            let burn_sequence = BurnSequence::new(
                burn_sequence_i,
                fin_sequence_pos.into_boxed_slice(),
                fin_sequence_vel.into_boxed_slice(),
                add_dt,
                fin_dt - dt - add_dt,
                burn_sequence_cost,
                fin_angle_dev,
            );

            let min_charge = FlightComputer::estimate_min_burn_sequence_charge(&burn_sequence);

            if best_burn_sequence.is_none()
                || burn_sequence_cost < best_burn_sequence.as_ref().unwrap().0.cost()
            {
                if min_charge < MAX_BATTERY_THRESHOLD {
                    best_burn_sequence = Some((burn_sequence, min_charge));
                } else {
                    println!("Cheaper maneuver found but min charge {min_charge} is too high!");
                }
            }
        }
        best_burn_sequence.unwrap()
    }

    #[allow(clippy::cast_precision_loss)]
    pub async fn schedule_zoned_objective(
        self: Arc<TaskController>,
        orbit_lock: Arc<RwLock<ClosedOrbit>>,
        f_cont_lock: Arc<RwLock<FlightComputer>>,
        scheduling_start_i: IndexedOrbitPosition,
        objective: ZonedObjective,
    ) {
        let computation_start = chrono::Utc::now();
        println!(
            "[INFO] Calculating schedule for Retrieval of Objective {}",
            objective.id()
        );
        let (burn_sequence, min_charge) = if objective.min_images() == 1 {
            let target_pos = objective.get_imaging_points()[0];
            let due = objective.end();
            Self::calculate_single_target_burn_sequence(
                scheduling_start_i,
                Arc::clone(&f_cont_lock),
                target_pos,
                due,
            )
            .await
        } else {
            // TODO
            panic!("[FATAL] Zoned Objective with multiple images not yet supported");
        };
        let comp_time = (chrono::Utc::now() - computation_start).num_milliseconds() as f32 / 1000.0;
        println!("[INFO] Maneuver calculation completed after {comp_time}s!");

        let start =
            (burn_sequence.start_i().t() - chrono::Utc::now()).num_milliseconds() as f32 / 1000.0;
        println!(
            "[INFO] Maneuver will start in {start}s, will take {}s. \
            Detumble time will be roughly {}s!",
            burn_sequence.acc_dt(),
            burn_sequence.detumble_dt()
        );
        let after_burn_calc_i = {
            let pos = f_cont_lock.read().await.current_pos();
            scheduling_start_i.new_from_pos(pos)
        };
        let dt = usize::try_from((burn_sequence.start_i().t() - chrono::Utc::now()).num_seconds())
            .unwrap_or(0);
        let result = {
            let orbit = orbit_lock.read().await;

            Self::init_orbit_sched_calc(
                &orbit,
                after_burn_calc_i.index(),
                Some(dt),
                Some((FlightState::Acquisition, min_charge)),
            )
        };
        println!(
            "[INFO] Calculating optimal orbit schedule to be at {min_charge} and in {} in {}s.",
            <&'static str>::from(FlightState::Acquisition),
            dt
        );
        let after_sched_calc_i = {
            let pos = f_cont_lock.read().await.current_pos();
            scheduling_start_i.new_from_pos(pos)
        };
        let dt_shift = after_sched_calc_i.index() - after_burn_calc_i.index();
        self.schedule_optimal_orbit_result(
            f_cont_lock,
            after_burn_calc_i.t(),
            result,
            dt_shift,
            true,
        )
        .await;
        let n_tasks = self.schedule_vel_change(burn_sequence).await;
        let dt_tot = (chrono::Utc::now() - computation_start).num_milliseconds() as f32 / 1000.0;
        println!(
            "[INFO] Number of tasks after scheduling: {n_tasks}. \
            Calculation and processing took {dt_tot:.2}",
        );

        {
            let sched = self.task_schedule.read().await;
            for task in &*sched {
                println!("[INFO] {task} in {}s.", task.dt().time_left());
            }
        }
    }

    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss,
        clippy::cast_possible_wrap
    )]
    pub async fn schedule_optimal_orbit(
        self: Arc<TaskController>,
        orbit_lock: Arc<RwLock<ClosedOrbit>>,
        f_cont_lock: Arc<RwLock<FlightComputer>>,
        scheduling_start_i: IndexedOrbitPosition,
    ) {
        let p_t_shift = scheduling_start_i.index();
        let computation_start = scheduling_start_i.t();
        println!("[INFO] Calculating optimal orbit schedule...");
        let result = {
            let orbit = orbit_lock.read().await;

            Self::init_orbit_sched_calc(&orbit, p_t_shift, None, None)
        };
        let dt_calc = (chrono::Utc::now() - computation_start).num_milliseconds() as f32 / 1000.0;
        println!("[INFO] Optimal Orbit Calculation complete after {dt_calc:.2}");
        let dt_shift = dt_calc.ceil() as usize;

        let n_tasks = self
            .schedule_optimal_orbit_result(f_cont_lock, computation_start, result, dt_shift, true)
            .await;
        let dt_tot = (chrono::Utc::now() - computation_start).num_milliseconds() as f32 / 1000.0;
        println!(
            "[INFO] Number of tasks after scheduling: {n_tasks}. \
            Calculation and processing took {dt_tot:.2}",
        );
    }

    #[allow(clippy::cast_possible_wrap, clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    async fn schedule_optimal_orbit_result(
        &self,
        f_cont_lock: Arc<RwLock<FlightComputer>>,
        base_t: chrono::DateTime<chrono::Utc>,
        res: OptimalOrbitResult,
        dt_sh: usize,
        trunc: bool,
    ) -> usize {
        println!("[INFO] Scheduling optimal orbit result...");
        if trunc {
            self.clear_schedule().await;
        }

        let (batt_f32, mut state) = {
            let f_cont = f_cont_lock.read().await;
            let batt: f32 = f_cont.current_battery();
            let st: usize = match f_cont.state() {
                FlightState::Acquisition => 1,
                FlightState::Charge => 0,
                // TODO: transition is possible here too!
                state => panic!("[FATAL] Unexpected flight state: {state}"),
            };
            (batt, st)
        };

        let mut dt = dt_sh;
        let (min_batt, max_batt) = (MIN_BATTERY_THRESHOLD, MAX_BATTERY_THRESHOLD);
        let max_mapped = (max_batt / Self::BATTERY_RESOLUTION - min_batt / Self::BATTERY_RESOLUTION)
            .round() as i32;
        let mut batt = ((batt_f32 - min_batt) / Self::BATTERY_RESOLUTION) as usize;
        let pred_secs = res.decisions.dt_len();
        let decisions = &res.decisions;
        while dt < pred_secs {
            let decision = decisions.get(dt, batt, state);
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
                    let sched_t = base_t + chrono::TimeDelta::seconds(dt as i64);
                    self.schedule_switch(FlightState::Charge, sched_t).await;
                    state = 0;
                    dt = (dt + 180).min(pred_secs);
                }
                AtomicDecision::SwitchToAcquisition => {
                    let sched_t = base_t + chrono::TimeDelta::seconds(dt as i64);
                    self.schedule_switch(FlightState::Acquisition, sched_t).await;
                    state = 1;
                    dt = (dt + 180).min(pred_secs);
                }
            }
        }
        self.task_schedule.read().await.len()
    }

    /// Provides a reference to the image task schedule.
    ///
    /// # Returns
    /// - An `Arc` pointing to the `LockedTaskQueue`.
    pub fn sched_arc(&self) -> Arc<RwLock<VecDeque<Task>>> { Arc::clone(&self.task_schedule) }

    /// Provides a reference to the `Convar` signaling changes to the first item in `image_schedule`.
    ///
    /// # Returns
    /// - An `Arc` pointing to the `Condvar`.
    pub fn notify_arc(&self) -> Arc<Condvar> { Arc::clone(&self.next_task_notify) }

    async fn schedule_switch(&self, target: FlightState, sched_t: chrono::DateTime<chrono::Utc>) {
        let dt = PinnedTimeDelay::from_end(sched_t);
        if self.task_schedule.read().await.is_empty() {
            self.task_schedule.write().await.push_back(Task::switch_target(target, dt));
            self.next_task_notify.notify_all();
        } else {
            self.task_schedule.write().await.push_back(Task::switch_target(target, dt));
        }
    }

    async fn schedule_vel_change(&self, burn: BurnSequence) -> usize {
        let mut has_to_notify = false;
        if self.task_schedule.read().await.is_empty() {
            has_to_notify = true;
        }
        let dt = PinnedTimeDelay::from_end(burn.start_i().t());
        self.task_schedule.write().await.push_back(Task::vel_change_task(burn, dt));

        if has_to_notify {
            self.next_task_notify.notify_all();
        }
        self.task_schedule.read().await.len()
    }

    pub async fn clear_after_dt(&self, dt: PinnedTimeDelay) {
        let schedule_lock = &*self.task_schedule;
        if !schedule_lock.read().await.is_empty() {
            return;
        }
        let mut schedule = schedule_lock.write().await;
        let schedule_len = schedule.len();
        let mut first_remove = 0;
        for i in 0..schedule_len {
            if schedule[i].dt().get_end() > dt.get_end() {
                first_remove = i;
                break;
            }
        }
        schedule.drain(first_remove..schedule_len);
    }

    /// Clears all pending tasks in the schedule.
    ///
    /// # Side Effects
    /// - Notifies all waiting threads if the schedule is cleared.
    pub async fn clear_schedule(&self) {
        let schedule = &*self.task_schedule;
        schedule.write().await.clear();
    }
}
