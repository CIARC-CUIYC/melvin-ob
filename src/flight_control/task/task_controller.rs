use crate::flight_control::{
    common::{linked_box::LinkedBox, math, pinned_dt::PinnedTimeDelay, vec2d::Vec2D},
    flight_computer::FlightComputer,
    flight_state::FlightState,
    orbit::{BurnSequence, ClosedOrbit, IndexedOrbitPosition},
    task::{
        atomic_decision::AtomicDecision, atomic_decision_cube::AtomicDecisionCube, base_task::Task,
        score_grid::ScoreGrid,
    },
};
use crate::http_handler::ZonedObjective;
use crate::{MAX_BATTERY_THRESHOLD, MIN_BATTERY_THRESHOLD};
use bitvec::prelude::BitRef;
use fixed::types::I32F32;
use num::{ToPrimitive, Zero};
use std::{
    collections::VecDeque,
    fmt::Debug,
    sync::{Arc, Condvar},
};
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
    /// Notification condition variable to signal changes to the first element in `image_schedule`.
    next_task_notify: Arc<Condvar>,
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

    /// The resolution for time duration calculations, expressed in fixed-point format.
    const TIME_RESOLUTION: I32F32 = I32F32::lit("1.0");

    /// The minimum delta time for scheduling objectives, in seconds.
    const OBJECTIVE_SCHEDULE_MIN_DT: usize = 1000;

    /// The minimum tolerance for retrieving scheduled objectives.
    const OBJECTIVE_MIN_RETRIEVAL_TOL: usize = 100;

    /// The initial battery threshold for performing a maneuver.
    const MANEUVER_INIT_BATT_TOL: I32F32 = I32F32::lit("10.0");

    /// The minimum delta time required for detumble maneuvers, in seconds.
    pub(crate) const MANEUVER_MIN_DETUMBLE_DT: usize = 50;

    /// Weight assigned to off-orbit delta time in optimization calculations.
    const OFF_ORBIT_DT_WEIGHT: I32F32 = I32F32::lit("3.0");

    /// Weight assigned to fuel consumption in optimization calculations.
    const FUEL_CONSUMPTION_WEIGHT: I32F32 = I32F32::lit("1.0");

    /// Weight assigned to angle deviation in optimization calculations.
    const ANGLE_DEV_WEIGHT: I32F32 = I32F32::lit("2.0");

    /// Default maximum time duration for burn sequences, in seconds.
    const DEF_MAX_BURN_SEQUENCE_TIME: i64 = 1_000_000_000;

    /// Maximum allowable absolute deviation after a correction burn.
    const MAX_AFTER_CB_DEV: I32F32 = I32F32::lit("1.0");

    /// A constant representing a 90-degree angle, in fixed-point format.
    const NINETY_DEG: I32F32 = I32F32::lit("90.0");

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
    fn init_orbit_sched_calc(
        orbit: &ClosedOrbit,
        p_t_shift: usize,
        dt: Option<usize>,
        end_status: Option<(FlightState, I32F32)>,
    ) -> OptimalOrbitResult {
        // List of potential states during the orbit scheduling process.
        let states = [FlightState::Charge, FlightState::Acquisition];
        // Calculate the usable battery range based on the fixed thresholds.
        let usable_batt_range = MAX_BATTERY_THRESHOLD - MIN_BATTERY_THRESHOLD;
        // Determine the maximum number of battery levels that can be represented.
        let max_battery =
            (usable_batt_range / Self::BATTERY_RESOLUTION).round().to_usize().unwrap();
        // Determine the prediction duration in seconds, constrained by the orbit period or `dt` if provided.
        let prediction_secs = {
            let max_pred_secs =
                Self::MAX_ORBIT_PREDICTION_SECS.min(orbit.period().0.to_u32().unwrap()) as usize;
            if let Some(pred_secs) = dt {
                // Ensure the prediction duration does not exceed the maximum prediction length or the provided duration.
                max_pred_secs.min(pred_secs)
            } else {
                max_pred_secs
            }
        };
        // Retrieve a reordered iterator over the orbit's completion bitvector to optimize scheduling.
        let p_t_iter = orbit.get_p_t_reordered(
            p_t_shift,
            orbit.period().0.to_usize().unwrap() - prediction_secs,
        );
        // Create a blank decision buffer and score grid for the orbit schedule calculation.
        let decision_buffer =
            AtomicDecisionCube::new(prediction_secs, max_battery + 1, states.len());
        let cov_dt_temp = ScoreGrid::new(max_battery + 1, states.len());
        // Initialize the first coverage grid based on the end status or use a default grid.
        let cov_dt_first = {
            if let Some(end) = end_status {
                let end_batt_clamp = end.1.clamp(MIN_BATTERY_THRESHOLD, MAX_BATTERY_THRESHOLD);
                let end_batt =
                    (end_batt_clamp / Self::BATTERY_RESOLUTION).round().to_usize().unwrap();
                let end_state = end.0 as usize;
                let end_cast = (end_state, end_batt);
                ScoreGrid::new_from_condition(max_battery + 1, states.len(), end_cast)
            } else {
                cov_dt_temp.clone()
            }
        };
        // Initialize a linked list of score cubes with a fixed size and push the initial coverage grid.
        let mut score_cube = LinkedBox::new(180);
        score_cube.push(cov_dt_first);
        // Perform the calculation for the optimal orbit schedule using the prepared variables.
        Self::calculate_optimal_orbit_schedule(
            prediction_secs,
            Box::new(p_t_iter),
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
            let p_dt = u16::from(!*p_t_it.next().unwrap());
            for e in 0..=max_battery {
                for s in 0..=1 {
                    let de = if s == 0 { 1 } else { -1 };
                    let new_e = (e as isize + de) as usize;
                    // Compute score for the decision to stay in the current state.
                    let stay = {
                        if s == 0 {
                            // If in charge state, calculate score for staying.
                            score_cube.front().unwrap().get(new_e.min(max_battery), s)
                        } else if e > 0 {
                            // If in acquisition state, consider score and state.
                            score_cube.front().unwrap().get(new_e, s) + i32::from(p_dt)
                        } else {
                            // If battery is depleted, staying is not possible.
                            i32::MIN
                        }
                    };
                    // Compute score for the decision to switch to the other state.
                    let switch = score_cube.back().unwrap().get(e, s ^ 1);
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
        due: PinnedTimeDelay,
    ) -> (Vec<Vec2D<I32F32>>, i64, Vec2D<I32F32>) {
        let is_clockwise = initial_vel.is_clockwise_to(&deviation).unwrap_or(false);

        let min_burn_sequence_time = chrono::TimeDelta::seconds(Self::DEF_MAX_BURN_SEQUENCE_TIME);
        let mut max_acc_secs = 1;
        let mut best_burn_sequence = Vec::new();
        let mut best_min_dev = Vec2D::new(I32F32::MAX, I32F32::MAX);
        let mut best_vel_hold_dt = 0;
        let mut last_vel = initial_vel;

        while min_burn_sequence_time > due.time_left() {
            let mut res_vel_diff = Vec2D::<I32F32>::zero();
            let mut remaining_deviation = deviation;
            let mut current_burn_sequence = Vec::new();

            for _ in 0..max_acc_secs {
                let perp_acc = last_vel.perp_unit(is_clockwise) * FlightComputer::ACC_CONST;
                let (new_vel, _) = FlightComputer::trunc_vel(last_vel + perp_acc);
                current_burn_sequence.push(new_vel);
                let vel_diff = new_vel - last_vel;
                res_vel_diff = res_vel_diff + vel_diff;
                remaining_deviation = remaining_deviation - res_vel_diff * I32F32::lit("2.0");
                last_vel = new_vel;
            }

            let x_vel_hold_dt = (remaining_deviation.x() / res_vel_diff.x()).floor();
            let y_vel_hold_dt = (remaining_deviation.y() / res_vel_diff.y()).floor();
            let min_vel_hold_dt = x_vel_hold_dt.min(y_vel_hold_dt).to_i64().unwrap();

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
            let min_t_i64 = min_t.floor().to_i64().unwrap();

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
        f_cont_lock: Arc<RwLock<FlightComputer>>,
        target_pos: Vec2D<I32F32>,
        target_end_time: chrono::DateTime<chrono::Utc>,
    ) -> (BurnSequence, I32F32) {
        println!("[INFO] Starting to calculate single target burn towards {target_pos}");

        // Calculate maximum allowed time delta for the maneuver
        let time_left = target_end_time - chrono::Utc::now();
        let max_dt = {
            let max = usize::try_from(time_left.num_seconds()).unwrap_or(0);
            max - Self::OBJECTIVE_MIN_RETRIEVAL_TOL
        };

        // Retrieve the current orbital velocity
        let orbit_vel = {
            let f_cont = f_cont_lock.read().await;
            f_cont.current_vel()
        };

        let max_off_orbit_t = max_dt - Self::OBJECTIVE_SCHEDULE_MIN_DT;

        // Spawn a task to compute possible turns asynchronously
        let turns_handle =
            tokio::spawn(async move { FlightComputer::compute_possible_turns(orbit_vel) });

        let orbit_vel_abs = orbit_vel.abs();
        let mut last_possible_dt = 0;

        // Determine the last possible time delta where a maneuver is feasible
        for dt in (Self::OBJECTIVE_SCHEDULE_MIN_DT..max_dt).rev() {
            let pos = (curr_i.pos() + orbit_vel * I32F32::from_num(dt)).wrap_around_map();
            let to_target = pos.unwrapped_to(&target_pos);
            let min_dt = (to_target.abs() / orbit_vel_abs).abs().round().to_usize().unwrap();

            if min_dt + dt < max_dt {
                last_possible_dt = dt;
                break;
            }
        }
        println!(
            "[INFO] Done skipping impossible start times. Last possible dt: {last_possible_dt}"
        );

        // Define range for evaluation and initialize best burn sequence tracker
        let remaining_range = Self::OBJECTIVE_SCHEDULE_MIN_DT..=last_possible_dt;
        let mut best_burn_sequence: Option<(BurnSequence, I32F32)> = None;

        // Calculate the maximum allowable angle deviation
        let max_angle_dev = {
            let vel_perp = orbit_vel.perp_unit(true) * FlightComputer::ACC_CONST;
            orbit_vel.angle_to(&vel_perp).abs()
        };

        // Await the result of possible turn computations
        let turns = turns_handle.await.unwrap();

        'outer: for dt in remaining_range.rev() {
            // Calculate the next position and initialize the burn sequence
            let mut next_pos = (curr_i.pos() + orbit_vel * I32F32::from_num(dt)).wrap_around_map();
            let burn_sequence_i =
                curr_i.new_from_future_pos(next_pos, chrono::TimeDelta::seconds(dt as i64));
            let direction_vec = next_pos.unwrapped_to(&target_pos);

            // Skip iterations where the current velocity is too far off-target
            if orbit_vel.angle_to(&direction_vec).abs() > Self::NINETY_DEG {
                continue;
            }

            // Determine the turn sequence based on the relative direction to the target
            let (turns_in_dir, break_cond) = {
                if direction_vec.is_clockwise_to(&orbit_vel).unwrap_or(false) {
                    (&turns.0, false)
                } else {
                    (&turns.1, true)
                }
            };

            // Initialize variables to calculate the final burn sequence
            let mut add_dt = 0;
            let mut fin_sequence_pos: Vec<Vec2D<I32F32>> = vec![next_pos];
            let mut fin_sequence_vel: Vec<Vec2D<I32F32>> = vec![orbit_vel];
            let mut fin_angle_dev = I32F32::zero();
            let mut fin_dt = 0;
            let max_add_dt = turns_in_dir.len();

            'inner: for atomic_turn in turns_in_dir {
                // Update position and velocity based on turns
                next_pos = (next_pos + atomic_turn.0).wrap_around_map();
                let next_vel = atomic_turn.1;

                let next_to_target = next_pos.unwrapped_to(&target_pos);
                let min_dt =
                    (next_to_target.abs() / next_vel.abs()).abs().round().to_usize().unwrap();

                // Check if the maneuver exceeds the maximum allowed time
                if min_dt + dt + add_dt > max_dt {
                    continue 'outer;
                }

                // Break and finalize the burn sequence if close enough to the target
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
                            I32F32::zero(),
                            I32F32::lit("1.0"),
                            I32F32::zero(),
                        );
                        let acc = (next_vel - *last_vel) * corr_burn_perc;
                        let (corr_vel, _) = FlightComputer::trunc_vel(next_vel + acc);
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

            // Normalize the factors contributing to burn sequence cost
            let normalized_fuel_consumption = math::normalize_fixed32(
                I32F32::from_num(add_dt) * FlightComputer::FUEL_CONST,
                I32F32::zero(),
                I32F32::from_num(max_add_dt) * FlightComputer::FUEL_CONST,
            )
            .unwrap_or(I32F32::zero());

            let normalized_off_orbit_t = math::normalize_fixed32(
                I32F32::from_num(fin_dt - dt),
                I32F32::zero(),
                I32F32::from_num(max_off_orbit_t),
            )
            .unwrap_or(I32F32::zero());
            let normalized_angle_dev =
                math::normalize_fixed32(fin_angle_dev.abs(), I32F32::zero(), max_angle_dev)
                    .unwrap_or(I32F32::zero());

            // Compute the total cost of the burn sequence
            let burn_sequence_cost = Self::OFF_ORBIT_DT_WEIGHT * normalized_off_orbit_t
                + Self::FUEL_CONSUMPTION_WEIGHT * normalized_fuel_consumption
                + Self::ANGLE_DEV_WEIGHT * normalized_angle_dev;

            // Create the burn sequence object
            let burn_sequence = BurnSequence::new(
                burn_sequence_i,
                fin_sequence_pos.into_boxed_slice(),
                fin_sequence_vel.into_boxed_slice(),
                add_dt,
                fin_dt - dt - add_dt,
                burn_sequence_cost,
                fin_angle_dev,
            );

            // Estimate the minimum required battery charge for the sequence
            let min_charge = FlightComputer::estimate_min_burn_sequence_charge(&burn_sequence);

            // Update the best burn sequence if a cheaper and feasible one is found
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

        // Return the best burn sequence, panicking if none was found
        best_burn_sequence.unwrap()
    }

    /// Schedules a zoned objective, calculating the necessary burn maneuvers and updating the task schedule.
    ///
    /// # Arguments
    /// - `self`: A reference-counted `TaskController` used for scheduling and managing tasks.
    /// - `orbit_lock`: An `Arc<RwLock<ClosedOrbit>>` containing the shared closed orbit data.
    /// - `f_cont_lock`: An `Arc<RwLock<FlightComputer>>` containing the flight control state and operations.
    /// - `scheduling_start_i`: The starting orbital position as an `IndexedOrbitPosition`.
    /// - `objective`: A `ZonedObjective` representing the target objective to be scheduled.
    ///
    /// # Behavior
    /// - Calculates the best maneuver to reach a single imaging target if the objective requires only one image.
    /// - Logs the computation process and results.
    /// - Updates the task schedule with velocity and state changes required to achieve the objective.
    ///
    /// # Panics
    /// - Panics if the objective requires more than one image as multi-image scheduling is not yet supported.
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
            // Clear the existing schedule if truncation is requested.
            self.clear_schedule().await;
        }

        // Retrieve the current battery level and satellite state
        let (batt_f32, mut state) = {
            let f_cont = f_cont_lock.read().await;
            let batt: I32F32 = f_cont.current_battery();
            let st: usize = match f_cont.state() {
                FlightState::Acquisition => 1,
                FlightState::Charge => 0,
                state => panic!("[FATAL] Unexpected flight state: {state}"),
            };
            (batt, st)
        };

        let mut dt = dt_sh;
        let (min_batt, max_batt) = (MIN_BATTERY_THRESHOLD, MAX_BATTERY_THRESHOLD); // Battery thresholds
        let max_mapped = (max_batt / Self::BATTERY_RESOLUTION
            - min_batt / Self::BATTERY_RESOLUTION)
            .round()
            .to_i32()
            .unwrap();

        // Map the current battery level into a discrete range.
        let mut batt = ((batt_f32 - min_batt) / Self::BATTERY_RESOLUTION).to_usize().unwrap();
        let pred_secs = res.decisions.dt_len();
        let decisions = &res.decisions;

        // Iterate through each time step and apply the corresponding decision logic.
        while dt < pred_secs {
            let decision = decisions.get(dt, batt, state);

            match decision {
                AtomicDecision::StayInCharge => {
                    // Stay in the charge state, increment battery level.
                    state = 0;
                    batt = (batt + 1).min(max_mapped as usize);
                    dt += 1;
                }
                AtomicDecision::StayInAcquisition => {
                    // Stay in the acquisition state, decrement battery level.
                    state = 1;
                    batt -= 1;
                    dt += 1;
                }
                AtomicDecision::SwitchToCharge => {
                    // Schedule a state change to "Charge" with an appropriate time delay.
                    let sched_t = base_t + chrono::TimeDelta::seconds(dt as i64);
                    self.schedule_switch(FlightState::Charge, sched_t).await;
                    state = 0;
                    dt = (dt + 180).min(pred_secs); // Add a delay for the transition.
                }
                AtomicDecision::SwitchToAcquisition => {
                    // Schedule a state change to "Acquisition" with an appropriate time delay.
                    let sched_t = base_t + chrono::TimeDelta::seconds(dt as i64);
                    self.schedule_switch(FlightState::Acquisition, sched_t).await;
                    state = 1;
                    dt = (dt + 180).min(pred_secs); // Add a delay for the transition.
                }
            }
        }

        // Return the final number of tasks in the schedule.
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

    /// Schedules a task to switch the flight state at a specific time.
    ///
    /// # Arguments
    /// - `target`: The target flight state to switch to.
    /// - `sched_t`: The scheduled time for the state change as a `chrono::DateTime`.
    ///
    /// # Behavior
    /// - If the task schedule is empty, the task is enqueued, and all waiting threads are notified.
    /// - Otherwise, the task is simply enqueued without sending notifications.
    async fn schedule_switch(&self, target: FlightState, sched_t: chrono::DateTime<chrono::Utc>) {
        let dt = PinnedTimeDelay::from_end(sched_t);
        if self.task_schedule.read().await.is_empty() {
            self.enqueue_task(Task::switch_target(target, dt)).await;
            self.next_task_notify.notify_all();
        } else {
            self.enqueue_task(Task::switch_target(target, dt)).await;
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
    async fn schedule_vel_change(&self, burn: BurnSequence) -> usize {
        let mut has_to_notify = false;
        if self.task_schedule.read().await.is_empty() {
            has_to_notify = true;
        }
        let dt = PinnedTimeDelay::from_end(burn.start_i().t());
        self.enqueue_task(Task::vel_change_task(burn, dt)).await;

        if has_to_notify {
            self.next_task_notify.notify_all();
        }
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

    /// Adds a task to the task schedule.
    ///
    /// # Arguments
    /// - `task`: The `Task` to be added to the task schedule.
    ///
    /// # Behavior
    /// - Appends the given task to the task schedule.
    async fn enqueue_task(&self, task: Task) {
        self.task_schedule.write().await.push_back(task);
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
