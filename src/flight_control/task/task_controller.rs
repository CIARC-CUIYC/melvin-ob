use super::{
    atomic_decision::AtomicDecision, atomic_decision_cube::AtomicDecisionCube, base_task::Task,
    score_grid::ScoreGrid, vel_change_task::VelocityChangeTaskRationale,
};
use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::flight_state::TRANS_DEL;
use crate::flight_control::orbit::BurnSequenceEvaluator;
use crate::flight_control::task::end_condition::EndCondition;
use crate::flight_control::{
    common::{linked_box::LinkedBox, vec2d::Vec2D},
    flight_computer::FlightComputer,
    flight_state::FlightState,
    orbit::{BurnSequence, ClosedOrbit, IndexedOrbitPosition},
};
use crate::{error, info, log};
use bitvec::prelude::BitRef;
use chrono::{DateTime, TimeDelta, Utc};
use fixed::types::I32F32;
use num::Zero;
use std::{collections::VecDeque, fmt::Debug, sync::Arc};
use tokio::sync::RwLock;

/// `TaskController` manages and schedules tasks for MELVIN.
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
}

/// Struct holding the result of the optimal orbit dynamic program
struct OptimalOrbitResult {
    /// Flattened 3D-Array holding decisions in time, energy, state dimension
    pub decisions: AtomicDecisionCube,
    /// `LinkedBox` holding some of the last scores over the energy and the state dimension for the calculation
    pub coverage_slice: LinkedBox<ScoreGrid>,
}

impl TaskController {
    /// The maximum number of seconds for orbit prediction calculations.
    const MAX_ORBIT_PREDICTION_SECS: u32 = 80000;

    /// The resolution for battery levels used in calculations, expressed in fixed-point format.
    const BATTERY_RESOLUTION: I32F32 = I32F32::lit("0.1");

    pub const MIN_BATTERY_THRESHOLD: I32F32 = I32F32::lit("10.00");
    pub const MAX_BATTERY_THRESHOLD: I32F32 = I32F32::lit("100.00");

    /// The resolution for time duration calculations, expressed in fixed-point format.
    const TIME_RESOLUTION: I32F32 = I32F32::lit("1.0");

    /// The minimum delta time for scheduling objectives, in seconds.
    const OBJECTIVE_SCHEDULE_MIN_DT: usize = 1000;

    /// The minimum tolerance for retrieving scheduled objectives.
    const OBJECTIVE_MIN_RETRIEVAL_TOL: usize = 100;

    /// The initial battery threshold for performing a maneuver.
    const MANEUVER_INIT_BATT_TOL: I32F32 = I32F32::lit("10.0");

    /// The minimum delta time required for detumble maneuvers, in seconds.
    pub(crate) const MANEUVER_MIN_DETUMBLE_DT: usize = 100;

    /// Default maximum time duration for burn sequences, in seconds.
    const DEF_MAX_BURN_SEQUENCE_TIME: i64 = 1_000_000_000;

    const ZO_IMAGE_FIRST_DEL: TimeDelta = TimeDelta::seconds(5);

    /// Maximum allowable absolute deviation after a correction burn.
    const MAX_AFTER_CB_DEV: I32F32 = I32F32::lit("5.0");

    pub const IN_COMMS_SCHED_SECS: usize = 585;
    const COMMS_SCHED_PERIOD: usize = 1025;
    #[allow(clippy::cast_possible_wrap)]
    const COMMS_SCHED_USABLE_TIME: TimeDelta =
        TimeDelta::seconds((Self::COMMS_SCHED_PERIOD - 2 * 180) as i64);
    pub const COMMS_CHARGE_USAGE: I32F32 = I32F32::lit("4.68");
    pub const MIN_COMMS_START_CHARGE: I32F32 = I32F32::lit("20.0");

    /// Creates a new instance of the `TaskController` struct.
    ///
    /// # Returns
    /// - A new `TaskController` with an empty task schedule.
    pub fn new() -> Self {
        Self {
            task_schedule: Arc::new(RwLock::new(VecDeque::new())),
        }
    }

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    /// Initializes the optimal orbit schedule calculation.
    ///
    /// This method sets up the required data structures and parameters necessary for determining
    /// the most efficient orbit path based on the given parameters.
    ///
    /// # Arguments
    /// * `orbit` - Reference to the `ClosedOrbit` structure representing the current orbit configuration.
    /// * `p_t_shift` - The starting index used to shift and reorder the bitvector of the orbit.
    /// * `dt` - Optional maximum prediction duration in seconds. If `None`, defaults to the orbit period or the maximum prediction length.
    /// * `end_status` - Optional tuple containing the end flight state (`FlightState`) and battery level (`I32F32`) constraints.
    ///
    /// # Returns
    /// * `OptimalOrbitResult` - The final result containing calculated decisions and coverage slice used in the optimization.
    fn init_sched_dp(
        orbit: &ClosedOrbit,
        p_t_shift: usize,
        dt: Option<usize>,
        end_state: Option<FlightState>,
        end_batt: Option<I32F32>,
    ) -> OptimalOrbitResult {
        // List of potential states during the orbit scheduling process.
        let states = [FlightState::Charge, FlightState::Acquisition];
        // Calculate the usable battery range based on the fixed thresholds.
        let usable_batt_range = Self::MAX_BATTERY_THRESHOLD - Self::MIN_BATTERY_THRESHOLD;
        // Determine the maximum number of battery levels that can be represented.
        let max_battery = (usable_batt_range / Self::BATTERY_RESOLUTION).round().to_num::<usize>();
        // Determine the prediction duration in seconds, constrained by the orbit period or `dt` if provided.
        let prediction_secs = {
            if let Some(pred_secs) = dt {
                // Ensure the prediction duration does not exceed the maximum prediction length or the provided duration.
                pred_secs
            } else {
                Self::MAX_ORBIT_PREDICTION_SECS.min(orbit.period().0.to_num::<u32>()) as usize
            }
        };

        // Retrieve a reordered iterator over the orbit's completion bitvector to optimize scheduling.
        let p_t_iter = orbit.get_p_t_reordered(
            p_t_shift,
            orbit.period().0.to_num::<usize>() - prediction_secs,
        );
        // Create a blank decision buffer and score grid for the orbit schedule calculation.
        let decision_buffer =
            AtomicDecisionCube::new(prediction_secs, max_battery + 1, states.len());
        let cov_dt_temp = ScoreGrid::new(max_battery + 1, states.len());
        // Initialize the first coverage grid based on the end status or use a default grid.
        let cov_dt_first = {
            let batt = end_batt.map_or(max_battery + 1, Self::map_e_to_dp);
            let state = end_state.map(|o| o as usize);
            let end_cast = (state, batt);
            ScoreGrid::new_from_condition(max_battery + 1, states.len(), end_cast)
        };
        // Initialize a linked list of score cubes with a fixed size and push the initial coverage grid.
        let mut score_cube = LinkedBox::new(180);
        score_cube.push(cov_dt_first);
        // Perform the calculation for the optimal orbit schedule using the prepared variables.
        Self::calculate_optimal_orbit_schedule(
            prediction_secs,
            p_t_iter,
            score_cube,
            &cov_dt_temp,
            decision_buffer,
        )
    }

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss, clippy::cast_possible_wrap)]
    /// Calculates the optimal orbit schedule based on predicted states and actions.
    ///
    /// This function iterates backward over a prediction window (`pred_dt`) to compute the best decisions
    /// and score grid values for optimizing orbit transitions. It uses battery levels, state transitions,
    /// and the orbits `done`-`BitBox`.
    ///
    /// # Arguments
    /// - `pred_dt`: The number of prediction time steps.
    /// - `p_t_it`: Iterator over the orbit's completion bitvector, providing timed scores.
    /// - `score_cube`: A linked list holding previous and current score grids for dynamic programming.
    /// - `score_grid_default`: A grid initialized with default scores used during calculations.
    /// - `dec_cube`: A decision cube to store the selected actions at each time step.
    ///
    /// # Returns
    /// - `OptimalOrbitResult`: Contains the final decision cube and the score grid linked box.
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
            let p_dt = i32::from(!*p_t_it.next().unwrap());
            for e in 0..=max_battery {
                for s in 0..=1 {
                    let de = if s == 0 { 1 } else { -1 };
                    let new_e = (e as isize + de) as usize;
                    // Compute score for the decision to stay in the current state.
                    let stay = if s == 0 {
                        // If in charge state, calculate score for staying.
                        score_cube.front().unwrap().get(new_e.min(max_battery), s)
                    } else if e > 0 {
                        // If in acquisition state, consider score and state.
                        score_cube.front().unwrap().get(new_e, s) + p_dt
                    } else {
                        // If battery is depleted, staying is not possible.
                        i32::MIN
                    };

                    let switch = if score_cube.len() < score_cube.size() {
                        // We do not swap here as the time after the maximum prediction time is not predictable
                        i32::MIN
                    } else {
                        // Compute score for the decision to switch to the other state.
                        score_cube.back().unwrap().get(e, s ^ 1)
                    };
                    // Choose the better decision and record it.
                    if stay >= switch {
                        dec_cube.set(t, e, s, AtomicDecision::stay(s));
                        cov_dt.set(e, s, stay);
                    } else {
                        dec_cube.set(t, e, s, AtomicDecision::switch(s ^ 1));
                        cov_dt.set(e, s, switch);
                    }
                }
            }
            // Push the updated score grid for the current time step into the linked box.
            score_cube.push(cov_dt);
        }
        // Return the resulting decision cube and the score grid linked box.
        OptimalOrbitResult {
            decisions: dec_cube,
            coverage_slice: score_cube,
        }
    }
    /*
    #[allow(clippy::cast_possible_truncation)]
    /// Calculates the optimal sequence of thrust burns needed to correct the orbit
    /// based on the current velocity and the desired deviation.
    ///
    /// This method iteratively determines the best sequence of acceleration vectors
    /// to minimize the deviation from the target trajectory while respecting the
    /// available time constraints.
    ///
    /// # Arguments
    /// * `initial_vel` - The initial velocity vector of the spacecraft.
    /// * `deviation` - The desired deviation vector to correct towards.
    /// * `due` - A `PinnedTimeDelay` representing the time until the correction is due.
    ///
    /// # Returns
    /// * A tuple containing:
    ///   - `Vec<Vec2D<I32F32>>`: The optimal sequence of velocity vectors during the burn.
    ///   - `i64`: The time duration to hold the final velocity.
    ///   - `Vec2D<I32F32>`: The final deviation vector after applying all burns.
    pub fn calculate_orbit_correction_burn(
        initial_vel: Vec2D<I32F32>,
        deviation: Vec2D<I32F32>,
        due: DateTime<Utc>,
    ) -> (Vec<Vec2D<I32F32>>, i64, Vec2D<FixedI64<U32>>) {
        let mut acc_secs = 1;
        // TODO: fix or scrap this
        let mut best_burn_sequence = Vec::new();
        let mut best_hold_dt = Self::DEF_MAX_BURN_SEQUENCE_TIME;
        let mut best_res_dev = deviation;

        while acc_secs <= (due - Utc::now()).num_seconds() / 2 {
            let mut res_vel_diff = Vec2D::<I32F32>::zero();
            let mut remaining_deviation = deviation;
            let mut current_burn_sequence = Vec::new();
            let mut last_vel = initial_vel;

            let mut vel_underflow = Vec2D::<I64F64>::zero();
            for _ in 0..acc_secs {
                let acc_vector = if vel_underflow.abs() >= FlightComputer::ACC_CONST {
                    let underflow_dir = vel_underflow.normalize();
                    let underflow_comp = underflow_dir * FlightComputer::ACC_CONST;
                    vel_underflow = vel_underflow - underflow_comp;
                    Vec2D::new(
                        I32F32::from_num(underflow_comp.x()),
                        I32F32::from_num(underflow_comp.y()),
                    )
                } else {
                    remaining_deviation.normalize() * FlightComputer::ACC_CONST
                };
                let (new_vel, underflow) = FlightComputer::trunc_vel(last_vel + acc_vector);
                vel_underflow = vel_underflow + underflow;
                current_burn_sequence.push(new_vel);
                let vel_diff = new_vel - last_vel;
                res_vel_diff = res_vel_diff + vel_diff;
                remaining_deviation = remaining_deviation - res_vel_diff * I32F32::lit("2.0");
                last_vel = new_vel;
            }
            println!("remaining_deviation: {remaining_deviation}, res_vel_diff: {res_vel_diff}");
            let x_vel_hold_dt =
                remaining_deviation.x().checked_div(res_vel_diff.x()).unwrap_or(I32F32::MAX);
            let y_vel_hold_dt =
                remaining_deviation.y().checked_div(res_vel_diff.y()).unwrap_or(I32F32::MAX);
            if x_vel_hold_dt
                .abs()
                .checked_sub(y_vel_hold_dt.abs())
                .is_none_or(|diff| diff > I32F32::lit("1.0"))
            {
                acc_secs += 1;
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
            let vel_hold_dt = min_t.floor().to_num::<i64>();

            if 2 * acc_secs + vel_hold_dt < (due - Utc::now()).num_seconds() {
                if res_dev_vec.abs() < Self::MAX_AFTER_CB_DEV {
                    return (current_burn_sequence, vel_hold_dt, res_dev_vec);
                } else if res_dev_vec.abs() < best_res_dev.abs() {
                    best_burn_sequence = current_burn_sequence;
                    best_hold_dt = vel_hold_dt;
                    best_res_dev = res_dev_vec;
                }
            }
            acc_secs += 1;
        }
        (best_burn_sequence, best_hold_dt, best_res_dev)
    }*/

    fn find_last_possible_dt(
        i: &IndexedOrbitPosition,
        vel: &Vec2D<I32F32>,
        target_pos: &Vec2D<I32F32>,
        max_dt: usize,
    ) -> usize {
        let orbit_vel_abs = vel.abs();

        for dt in (Self::OBJECTIVE_SCHEDULE_MIN_DT..max_dt).rev() {
            let pos = (i.pos() + *vel * I32F32::from_num(dt)).wrap_around_map();
            let to_target = pos.unwrapped_to(target_pos);
            let min_dt = (to_target.abs() / orbit_vel_abs).abs().round().to_num::<usize>();

            if min_dt + dt < max_dt {
                return dt;
            }
        }
        Self::OBJECTIVE_SCHEDULE_MIN_DT
    }

    /// Calculates the optimal burn sequence to reach a single target position
    /// within a specified end time.
    ///
    /// This function determines the most efficient sequence of orbital maneuvers
    /// (acceleration vectors) to steer the spacecraft from its current position
    /// towards a given target position, considering time and energy constraints.
    /// The resulting burn sequence minimizes fuel consumption, angle deviation,
    /// and off-orbit time while ensuring sufficient battery charge.
    ///
    /// # Arguments
    /// * `curr_i` - The current indexed orbit position of the spacecraft.
    /// * `f_cont_lock` - A shared lock on the `FlightComputer` for velocity and control access.
    /// * `target_pos` - The target position as a `Vec2D<I32F32>`.
    /// * `target_end_time` - The deadline by which the target must be reached.
    ///
    /// # Returns
    /// * `(BurnSequence, I32F32)` - A tuple containing:
    ///     - The optimized `BurnSequence` object representing the maneuver sequence.
    ///     - The minimum battery charge needed for the burn sequence.
    ///
    /// # Panics
    /// Panics if no valid burn sequence is found or the target is unreachable.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss,
        clippy::cast_possible_wrap,
        clippy::too_many_lines
    )]
    pub async fn calculate_single_target_burn_sequence(
        curr_i: IndexedOrbitPosition,
        curr_vel: Vec2D<I32F32>,
        target_pos: Vec2D<I32F32>,
        target_end_time: DateTime<Utc>,
        fuel_left: I32F32,
    ) -> Option<BurnSequence> {
        info!("Starting to calculate single target burn towards {target_pos}");

        // Calculate maximum allowed time delta for the maneuver
        let time_left = target_end_time - curr_i.t();
        let max_dt = {
            let max = usize::try_from(time_left.num_seconds()).unwrap_or(0);
            max - Self::OBJECTIVE_MIN_RETRIEVAL_TOL
        };

        let max_off_orbit_dt = max_dt - Self::OBJECTIVE_SCHEDULE_MIN_DT;

        // Spawn a task to compute possible turns asynchronously
        let turns_handle =
            tokio::spawn(async move { FlightComputer::compute_possible_turns(curr_vel) });

        let last_possible_dt = Self::find_last_possible_dt(&curr_i, &curr_vel, &target_pos, max_dt);

        // Define range for evaluation and initialize best burn sequence tracker
        let remaining_range = Self::OBJECTIVE_SCHEDULE_MIN_DT..=last_possible_dt;

        // Await the result of possible turn computations
        let turns = turns_handle.await.unwrap();
        let mut evaluator = BurnSequenceEvaluator::new(
            curr_i,
            curr_vel,
            target_pos,
            max_dt,
            max_off_orbit_dt,
            turns,
            fuel_left
        );

        for dt in remaining_range.rev() {
            evaluator.process_dt(dt, Self::MAX_BATTERY_THRESHOLD);
        }
        // Return the best burn sequence, panicking if none was found
        evaluator.get_best_burn().map(|(burn, _)| burn)
    }

    #[allow(clippy::cast_possible_wrap)]
    async fn sched_single_comms_cycle(
        &self,
        c_end: (DateTime<Utc>, I32F32),
        sched_start: (DateTime<Utc>, usize),
        orbit: &ClosedOrbit,
        strict_end: (DateTime<Utc>, usize),
    ) -> Option<(DateTime<Utc>, I32F32)> {
        let t_time = FlightState::Charge.dt_to(FlightState::Comms);
        let sched_end = sched_start.0 + Self::COMMS_SCHED_USABLE_TIME;
        let t_ch = Self::MIN_COMMS_START_CHARGE;

        if sched_end + t_time > strict_end.0 {
            let dt = usize::try_from((strict_end.0 - sched_start.0).num_seconds()).unwrap_or(0);
            let result = Self::init_sched_dp(orbit, sched_start.1, Some(dt), None, None);
            let target = {
                let st =
                    result.coverage_slice.front().unwrap().get_max_s(Self::map_e_to_dp(c_end.1));
                (c_end.1, st)
            };
            self.schedule_switch(FlightState::from_dp_usize(target.1), c_end.0).await;
            self.sched_opt_orbit_res(sched_start.0, result, 0, false, target).await;
            None
        } else {
            let dt = usize::try_from((sched_end - sched_start.0).num_seconds()).unwrap_or(0);
            let result = Self::init_sched_dp(orbit, sched_start.1, Some(dt), None, Some(t_ch));
            let target = {
                let st =
                    result.coverage_slice.front().unwrap().get_max_s(Self::map_e_to_dp(c_end.1));
                (c_end.1, st)
            };
            self.schedule_switch(FlightState::from_dp_usize(target.1), c_end.0).await;
            let (_, batt) = self.sched_opt_orbit_res(sched_start.0, result, 0, false, target).await;
            self.schedule_switch(FlightState::Comms, sched_end).await;
            let next_c_end =
                sched_end + t_time + TimeDelta::seconds(Self::IN_COMMS_SCHED_SECS as i64);
            Some((next_c_end, batt - Self::COMMS_CHARGE_USAGE))
        }
    }

    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss,
        clippy::cast_possible_wrap
    )]
    pub async fn sched_opt_orbit_w_comms(
        self: Arc<TaskController>,
        orbit_lock: Arc<RwLock<ClosedOrbit>>,
        f_cont_lock: Arc<RwLock<FlightComputer>>,
        scheduling_start_i: IndexedOrbitPosition,
        last_bo_end_t: DateTime<Utc>,
        first_comms_end: DateTime<Utc>,
        end_cond: Option<EndCondition>,
    ) {
        log!("Calculating/Scheduling optimal orbit with passive beacon scanning.");
        let computation_start = Utc::now();
        // TODO: later maybe this shouldnt be cleared here anymore
        self.clear_schedule().await;
        let t_time = FlightState::Charge.dt_to(FlightState::Comms);
        let t_time_ch = TimeDelta::from_std(t_time).unwrap();
        let strict_end = (last_bo_end_t, scheduling_start_i.index_then(last_bo_end_t));

        let is_next_possible: Box<dyn Fn(DateTime<Utc>) -> bool + Send> =
            if let Some(end) = &end_cond {
                let dt = end.abs_charge_dt() + t_time_ch * 2;
                Box::new(move |comms_end: DateTime<Utc>| -> bool {
                    let n_end = comms_end
                        + TaskController::COMMS_SCHED_USABLE_TIME
                        + t_time_ch * 2
                        + TimeDelta::seconds(TaskController::IN_COMMS_SCHED_SECS as i64);
                    n_end + dt <= end.time()
                })
            } else {
                Box::new(|_| -> bool { true })
            };

        let mut curr_comms_end = {
            let dt = first_comms_end - Utc::now();
            let batt = f_cont_lock.read().await.batt_in_dt(dt);
            Some((first_comms_end, batt))
        };

        let mut next_start = (Utc::now(), scheduling_start_i.index());
        let mut next_start_e = I32F32::zero();

        let orbit = orbit_lock.read().await;
        while let Some(end) = curr_comms_end {
            (next_start, next_start_e) = {
                let t = end.0 + TimeDelta::from_std(t_time).unwrap();
                let i = scheduling_start_i.index_then(t);
                ((t, i), end.1)
            };
            if is_next_possible(next_start.0) {
                curr_comms_end =
                    self.sched_single_comms_cycle(end, next_start, &orbit, strict_end).await;
            } else {
                break;
            }
        }

        if let Some(e) = &end_cond {
            let (left_dt, ch, s) = {
                let dt = usize::try_from((e.time() - next_start.0).num_seconds()).unwrap_or(0);
                (Some(dt), Some(e.charge()), Some(e.state()))
            };
            let result = Self::init_sched_dp(&orbit, next_start.1, left_dt, s, ch);
            let target = {
                let st = result
                    .coverage_slice
                    .front()
                    .unwrap()
                    .get_max_s(Self::map_e_to_dp(next_start_e));
                (next_start_e, st)
            };
            self.schedule_switch(
                FlightState::from_dp_usize(target.1),
                next_start.0 - t_time_ch,
            )
            .await;
            self.sched_opt_orbit_res(next_start.0, result, 0, false, target).await;
        }

        let n_tasks = self.task_schedule.read().await.len();
        let dt_tot = (Utc::now() - computation_start).num_milliseconds() as f32 / 1000.0;
        info!(
            "Number of tasks after scheduling: {n_tasks}. \
            Calculation and processing took {dt_tot:.2}s.",
        );
    }

    /// Calculates and schedules the optimal orbit trajectory based on the current position and state.
    ///
    /// # Arguments
    /// - `self`: A reference-counted `TaskController` used for task scheduling.
    /// - `orbit_lock`: An `Arc<RwLock<ClosedOrbit>>` containing the shared closed orbit data.
    /// - `f_cont_lock`: An `Arc<RwLock<FlightComputer>>` containing the flight control state.
    /// - `scheduling_start_i`: The starting orbital position as an `IndexedOrbitPosition`.
    ///
    /// # Behavior
    /// - Reads the closed orbit data and calculates the optimal orbit path.
    /// - Determines the time shifts required for optimal scheduling.
    /// - Schedules the resulting tasks with the `FlightComputer` state and logs their details.
    ///
    /// # Logs
    /// - Outputs detailed timing information about the calculation.
    /// - Logs the number of tasks generated during scheduling.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss,
        clippy::cast_possible_wrap
    )]
    pub async fn sched_opt_orbit(
        self: Arc<TaskController>,
        orbit_lock: Arc<RwLock<ClosedOrbit>>,
        f_cont_lock: Arc<RwLock<FlightComputer>>,
        scheduling_start_i: IndexedOrbitPosition,
        end: Option<EndCondition>,
    ) {
        log!("Calculating/Scheduling optimal orbit.");
        self.clear_schedule().await;
        let p_t_shift = scheduling_start_i.index();
        let comp_start = scheduling_start_i.t();
        let (dt, batt, state) = if let Some(end_c) = end {
            let end_t = (end_c.time() - Utc::now()).num_seconds().max(0) as usize;
            (Some(end_t), Some(end_c.charge()), Some(end_c.state()))
        } else {
            (None, None, None)
        };
        let result = {
            let orbit = orbit_lock.read().await;
            Self::init_sched_dp(&orbit, p_t_shift, dt, state, batt)
        };
        let dt_calc = (Utc::now() - comp_start).num_milliseconds() as f32 / 1000.0;
        let dt_shift = dt_calc.ceil() as usize;

        let (st_batt, dt_sh) = {
            let (batt, st) = Self::get_batt_and_state(&f_cont_lock).await;
            if st == 2 {
                let best_st =
                    result.coverage_slice.back().unwrap().get_max_s(Self::map_e_to_dp(batt));
                self.schedule_switch(FlightState::from_dp_usize(best_st), comp_start).await;
                ((batt, best_st), dt_shift + 180)
            } else {
                ((batt, st), dt_shift)
            }
        };
        let (n_tasks, _) =
            self.sched_opt_orbit_res(comp_start, result, dt_sh, false, st_batt).await;
        let dt_tot = (Utc::now() - comp_start).num_milliseconds() as f32 / 1000.0;
        info!("Tasks after scheduling: {n_tasks}. Calculation and processing took {dt_tot:.2}s.");
    }

    async fn get_batt_and_state(f_cont_lock: &Arc<RwLock<FlightComputer>>) -> (I32F32, usize) {
        // Retrieve the current battery level and satellite state
        let f_cont = f_cont_lock.read().await;
        let batt: I32F32 = f_cont.current_battery();
        (batt, f_cont.state().to_dp_usize())
    }

    fn map_e_to_dp(e: I32F32) -> usize {
        let e_clamp = e.clamp(Self::MIN_BATTERY_THRESHOLD, Self::MAX_BATTERY_THRESHOLD);

        ((e_clamp - Self::MIN_BATTERY_THRESHOLD) / Self::BATTERY_RESOLUTION)
            .round()
            .to_num::<usize>()
    }

    fn map_dp_to_e(dp: usize) -> I32F32 {
        (Self::MIN_BATTERY_THRESHOLD + (I32F32::from_num(dp) * Self::BATTERY_RESOLUTION))
            .min(Self::MAX_BATTERY_THRESHOLD)
    }

    #[allow(clippy::cast_possible_wrap, clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    /// Schedules the result of an optimal orbit calculation as tasks.
    ///
    /// # Arguments
    /// - `f_cont_lock`: A reference-counted and thread-safe lock for accessing the flight computer state.
    /// - `base_t`: The base timestamp used for scheduling adjustments.
    /// - `res`: The result of the optimal orbit calculation, including decisions about state transitions.
    /// - `dt_sh`: The initial shift in time steps to apply during scheduling.
    /// - `trunc`: A flag indicating whether to clear the current schedule before scheduling new tasks.
    ///
    /// # Returns
    /// - The total number of tasks added to the task schedule.
    ///
    /// # Behavior
    /// - Reads the current battery level and flight state from the flight computer.
    /// - Maps the battery level to a normalized range using defined thresholds.
    /// - Iterates over the predicted decisions in the optimal orbit result to schedule tasks.
    /// - Tasks are dynamically scheduled based on the required transitions and predicted durations.
    /// - Adjusts the state and battery level during simulation.
    /// - Handles transitions between charging and acquisition states with proper time delays.
    async fn sched_opt_orbit_res(
        &self,
        base_t: DateTime<Utc>,
        res: OptimalOrbitResult,
        dt_sh: usize,
        trunc: bool,
        (batt_f32, mut state): (I32F32, usize),
    ) -> (usize, I32F32) {
        if trunc {
            // Clear the existing schedule if truncation is requested.
            self.clear_schedule().await;
        }

        let mut dt = dt_sh;
        let max_mapped = Self::map_e_to_dp(Self::MAX_BATTERY_THRESHOLD);

        // Map the current battery level into a discrete range.
        let mut batt = Self::map_e_to_dp(batt_f32);
        let pred_secs = res.decisions.dt_len();
        let decisions = &res.decisions;

        // Iterate through each time step and apply the corresponding decision logic.
        while dt < pred_secs {
            let decision = decisions.get(dt, batt, state);

            match decision {
                AtomicDecision::StayInCharge => {
                    // Stay in the charge state, increment battery level.
                    state = 0;
                    batt = (batt + 1).min(max_mapped);
                    dt += 1;
                }
                AtomicDecision::StayInAcquisition => {
                    // Stay in the acquisition state, decrement battery level.
                    state = 1;
                    if batt == 0 {
                        error!("Battery level is already at 0!");
                        error!("current: {dt} max: {pred_secs} init_batt: {batt_f32}");
                    } else {
                        batt -= 1;
                    }
                    dt += 1;
                }
                AtomicDecision::SwitchToCharge => {
                    // Schedule a state change to "Charge" with an appropriate time delay.
                    let sched_t = base_t + TimeDelta::seconds(dt as i64);
                    self.schedule_switch(FlightState::Charge, sched_t).await;
                    state = 0;
                    dt = (dt + 180).min(pred_secs); // Add a delay for the transition.
                }
                AtomicDecision::SwitchToAcquisition => {
                    // Schedule a state change to "Acquisition" with an appropriate time delay.
                    let sched_t = base_t + TimeDelta::seconds(dt as i64);
                    self.schedule_switch(FlightState::Acquisition, sched_t).await;
                    state = 1;
                    dt = (dt + 180).min(pred_secs); // Add a delay for the transition.
                }
            }
        }
        // Return the final number of tasks in the schedule.
        (
            self.task_schedule.read().await.len(),
            Self::map_dp_to_e(batt),
        )
    }

    /// Provides a reference to the image task schedule.
    ///
    /// # Returns
    /// - An `Arc` pointing to the `LockedTaskQueue`.
    pub fn sched_arc(&self) -> Arc<RwLock<VecDeque<Task>>> { Arc::clone(&self.task_schedule) }

    /// Schedules a task to switch the flight state at a specific time.
    ///
    /// # Arguments
    /// - `target`: The target flight state to switch to.
    /// - `sched_t`: The scheduled time for the state change as a `DateTime`.
    ///
    /// # Behavior
    /// - If the task schedule is empty, the task is enqueued, and all waiting threads are notified.
    /// - Otherwise, the task is simply enqueued without sending notifications.
    async fn schedule_switch(&self, target: FlightState, sched_t: DateTime<Utc>) {
        self.enqueue_task(Task::switch_target(target, sched_t)).await;
    }

    async fn schedule_zo_image(&self, t: DateTime<Utc>, pos: Vec2D<I32F32>, lens: CameraAngle) {
        let pos_u32 = Vec2D::new(pos.x().to_num::<u32>(), pos.y().to_num::<u32>());
        self.enqueue_task(Task::image_task(pos_u32, lens, t)).await;
    }

    pub async fn schedule_retrieval_phase(
        &self,
        t: DateTime<Utc>,
        pos: Vec2D<I32F32>,
        lens: CameraAngle,
    ) {
        let t_first = t - Self::ZO_IMAGE_FIRST_DEL;
        self.schedule_zo_image(t_first, pos, lens).await;
        let trans_time = TRANS_DEL.get(&(FlightState::Acquisition, FlightState::Charge)).unwrap();
        if Utc::now() + TimeDelta::from_std(*trans_time).unwrap() * 2 < t_first {
            self.schedule_switch(FlightState::Charge, Utc::now()).await;
            let last_charge_leave = t_first - TimeDelta::from_std(*trans_time).unwrap();
            self.schedule_switch(FlightState::Acquisition, last_charge_leave).await;
        }
    }

    /// Schedules a velocity change task for a given burn sequence.
    ///
    /// # Arguments
    /// - `burn`: The `BurnSequence` containing the velocity change details.
    ///
    /// # Returns
    /// - The total number of tasks in the schedule after adding the velocity change task.
    ///
    /// # Behavior
    /// - If the task schedule is empty before scheduling, all waiting threads are notified.
    pub async fn schedule_vel_change(
        self: Arc<TaskController>,
        burn: BurnSequence,
        rationale: VelocityChangeTaskRationale,
    ) -> usize {
        let due = burn.start_i().t();
        self.enqueue_task(Task::vel_change_task(burn, rationale, due)).await;
        self.task_schedule.read().await.len()
    }

    /// Clears tasks scheduled after a specified delay.
    ///
    /// # Arguments
    /// - `dt`: The `PinnedTimeDelay` representing the cutoff time for retaining tasks.
    ///
    /// # Behavior
    /// - Removes all tasks occurring after the specified delay.
    /// - Does nothing if the task schedule is empty.
    pub async fn clear_after_dt(&self, dt: DateTime<Utc>) {
        let schedule_lock = &*self.task_schedule;
        if !schedule_lock.read().await.is_empty() {
            return;
        }
        let mut schedule = schedule_lock.write().await;
        let schedule_len = schedule.len();
        let mut first_remove = 0;
        for i in 0..schedule_len {
            if schedule[i].t() > dt {
                first_remove = i;
                break;
            }
        }
        schedule.drain(first_remove..schedule_len);
    }

    /// Adds a task to the task schedule.
    ///
    /// # Arguments
    /// - `task`: The `Task` to be added to the task schedule.
    ///
    /// # Behavior
    /// - Appends the given task to the task schedule.
    async fn enqueue_task(&self, task: Task) { self.task_schedule.write().await.push_back(task); }

    /// Clears all pending tasks in the schedule.
    ///
    /// # Side Effects
    /// - Notifies all waiting threads if the schedule is cleared.
    pub async fn clear_schedule(&self) {
        let schedule = &*self.task_schedule;
        log!("Clearing task schedule...");
        schedule.write().await.clear();
    }
}
