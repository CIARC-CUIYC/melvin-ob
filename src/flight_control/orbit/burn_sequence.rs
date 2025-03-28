use super::index::IndexedOrbitPosition;
use crate::flight_control::{
    common::{math, vec2d::Vec2D},
    flight_computer::{FlightComputer, TurnsClockCClockTup},
    flight_state::FlightState,
    task::TaskController,
};
use chrono::{TimeDelta, Utc};
use fixed::types::I32F32;
use num::Zero;
use crate::logger::JsonDump;

/// Represents a sequence of corrective burns for orbital adjustments.
///
/// The `BurnSequence` contains position and velocity sequences, along with
/// timing and cost information, for controlling orbit behavior.
#[derive(Debug, Clone, serde::Serialize)]
pub struct BurnSequence {
    /// The orbital position where the sequence starts.
    start_i: IndexedOrbitPosition,
    /// The sequence of positional corrections.
    sequence_pos: Box<[Vec2D<I32F32>]>,
    /// The sequence of velocity corrections.
    sequence_vel: Box<[Vec2D<I32F32>]>,
    /// Acceleration time in seconds.
    acc_dt: usize,
    /// Time duration for detumbling after acceleration, in seconds.
    detumble_dt: usize,
    /// Remaining angular deviation after the sequence.
    rem_angle_dev: I32F32,
    /// Minimum battery needed to initiate the sequence.
    min_charge: I32F32,
    /// Minimum needed fuel to initiate the sequence.
    min_fuel: I32F32,
}

impl BurnSequence {
    /// Additional approximate detumble + return fuel need per exit maneuver
    const ADD_FUEL_CONST: I32F32 = I32F32::lit("10.0");
    const ADD_SECOND_MANEUVER_FUEL_CONST: I32F32 = I32F32::lit("5.0");

    /// Creates a new `BurnSequence` with the provided parameters.
    ///
    /// # Parameters
    /// * `start_i` - The initial orbital position for the sequence.
    /// * `sequence_pos` - A boxed slice of positional corrections.
    /// * `sequence_vel` - A boxed slice of velocity corrections.
    /// * `acc_dt` - Acceleration time duration, in seconds.
    /// * `detumble_dt` - Detumbling time duration, in seconds.
    /// * `cost_factor` - The predetermined cost factor for the sequence.
    /// * `rem_angle_dev` - The remaining angular deviation
    pub fn new(
        start_i: IndexedOrbitPosition,
        sequence_pos: Box<[Vec2D<I32F32>]>,
        sequence_vel: Box<[Vec2D<I32F32>]>,
        acc_dt: usize,
        detumble_dt: usize,
        rem_angle_dev: I32F32,
        second_target_add_dt: usize,
    ) -> Self {
        let acq_db = FlightState::Acquisition.get_charge_rate();
        let acq_acc_db = acq_db + FlightState::ACQ_ACC_ADDITION;
        let trunc_detumble_time = detumble_dt.saturating_sub(TaskController::MANEUVER_MIN_DETUMBLE_DT);
        let acq_charge_dt =
            i32::try_from(FlightState::Acquisition.dt_to(FlightState::Charge).as_secs())
                .unwrap_or(i32::MAX);
        let charge_acq_dt =
            i32::try_from(FlightState::Acquisition.dt_to(FlightState::Charge).as_secs())
                .unwrap_or(i32::MAX);
        let poss_charge_dt =
            i32::try_from(trunc_detumble_time).unwrap_or(i32::MAX) - acq_charge_dt - charge_acq_dt;
        let acq_time = {
            if poss_charge_dt > 0 {
                0
            } else {
                trunc_detumble_time
            }
        };

        let poss_charge = (I32F32::from_num(poss_charge_dt)
            * FlightState::Charge.get_charge_rate())
        .clamp(I32F32::zero(), TaskController::MAX_BATTERY_THRESHOLD);
        let acq_acc_time = I32F32::from_num(acc_dt + TaskController::MANEUVER_MIN_DETUMBLE_DT);

        let mut min_fuel = acq_acc_time * FlightComputer::ACC_CONST + Self::ADD_FUEL_CONST;

        let min_acc_acq_batt = (I32F32::from_num(acq_acc_time) * acq_acc_db).abs();
        let min_acq_batt = (I32F32::from_num(acq_time) * acq_db).abs();
        let mut add_acq_secs =
            2 * usize::try_from(TaskController::ZO_IMAGE_FIRST_DEL.num_seconds()).unwrap_or(0);
        
        if second_target_add_dt > 0 {
            add_acq_secs += second_target_add_dt;
            min_fuel += Self::ADD_SECOND_MANEUVER_FUEL_CONST;
        }
        
        let second_need = (I32F32::from_num(add_acq_secs) * acq_acc_db).abs();
        let add_charge = (second_need - poss_charge).max(I32F32::zero());
        let min_charge = TaskController::MIN_BATTERY_THRESHOLD + min_acc_acq_batt + min_acq_batt + add_charge;

        Self {
            start_i,
            sequence_pos,
            sequence_vel,
            acc_dt,
            detumble_dt,
            rem_angle_dev,
            min_charge,
            min_fuel,
        }
    }

    /// Returns the starting orbital position as `IndexedOrbitPosition` for the sequence.
    pub fn start_i(&self) -> IndexedOrbitPosition { self.start_i }

    /// Returns the sequence of positional corrections.
    pub fn sequence_pos(&self) -> &[Vec2D<I32F32>] { &self.sequence_pos }

    /// Returns the sequence of velocity corrections.
    pub fn sequence_vel(&self) -> &[Vec2D<I32F32>] { &self.sequence_vel }

    /// Returns the detumbling time duration, in seconds.
    pub fn detumble_dt(&self) -> usize { self.detumble_dt }

    /// Returns the acceleration time duration, in seconds.
    pub fn acc_dt(&self) -> usize { self.acc_dt }

    /// Returns the remaining angular deviation after the sequence.
    pub fn rem_angle_dev(&self) -> I32F32 { self.rem_angle_dev }

    /// Returns the minimum charge to initiate the burn.
    pub fn min_charge(&self) -> I32F32 { self.min_charge }

    /// Returns the minimum fuel to initiate the burn
    pub fn min_fuel(&self) -> I32F32 { self.min_fuel }
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct ExitBurnResult {
    sequence: BurnSequence,
    cost: I32F32,
    target_pos: Vec2D<I32F32>,
    add_target: Option<Vec2D<I32F32>>,
    unwrapped_target: Vec2D<I32F32>,
    target_id: usize,
}

impl JsonDump for ExitBurnResult {
    fn file_name(&self) -> String {format!("zo_burn_{}", self.target_id)  }

    fn dir_name(&self) -> &'static str { "zoned_objectives"  }
}

impl ExitBurnResult {
    pub fn new(
        sequence: BurnSequence,
        target: (Vec2D<I32F32>, Vec2D<I32F32>),
        unwrapped_target: Vec2D<I32F32>,
        cost: I32F32,
        target_id: usize
    ) -> Self {
        let target_pos = target.0;
        let add_target = if target.1 == Vec2D::zero() {
            None
        } else {
            Some((target.0 + target.1).wrap_around_map())
        };
        Self { sequence, cost, target_pos, add_target, unwrapped_target, target_id }
    }

    pub fn cost(&self) -> I32F32 { self.cost }
    pub fn sequence(&self) -> &BurnSequence { &self.sequence }
    pub fn target_pos(&self) -> &Vec2D<I32F32> { &self.target_pos }
    pub fn add_target(&self) -> Option<Vec2D<I32F32>> { self.add_target }
    pub fn unwrapped_target(&self) -> &Vec2D<I32F32> { &self.unwrapped_target }
}

pub struct BurnSequenceEvaluator<'a> {
    i: IndexedOrbitPosition,
    vel: Vec2D<I32F32>,
    targets: &'a [(Vec2D<I32F32>, Vec2D<I32F32>)],
    max_dt: usize,
    min_dt: usize,
    max_off_orbit_dt: usize,
    max_angle_dev: I32F32,
    turns: TurnsClockCClockTup,
    best_burn: Option<ExitBurnResult>,
    fuel_left: I32F32,
    dynamic_fuel_w: I32F32,
    target_id: usize,
}

impl<'a> BurnSequenceEvaluator<'a> {
    /// A constant representing a 90-degree angle, in fixed-point format.
    const NINETY_DEG: I32F32 = I32F32::lit("90.0");
    /// Weight assigned to off-orbit delta time in optimization calculations.
    const OFF_ORBIT_W: I32F32 = I32F32::lit("2.0");
    /// Maximum Weight assigned to fuel consumption in optimization calculations.
    const MAX_FUEL_W: I32F32 = I32F32::lit("3.0");
    /// Minimum Weight assigned to fuel consumption in optimization calculations.
    const MIN_FUEL_W: I32F32 = I32F32::lit("1.0");
    /// Weight assigned to angle deviation in optimization calculations.
    const ANGLE_DEV_W: I32F32 = I32F32::lit("1.5");
    /// Weight assigned to additional target angle deviation.
    const ADD_ANGLE_DEV_W: I32F32 = I32F32::lit("3.0");

    #[allow(clippy::too_many_arguments)]
    pub fn new(
        i: IndexedOrbitPosition,
        vel: Vec2D<I32F32>,
        targets: &'a [(Vec2D<I32F32>, Vec2D<I32F32>)],
        min_dt: usize,
        max_dt: usize,
        max_off_orbit_dt: usize,
        turns: TurnsClockCClockTup,
        fuel_left: I32F32,
        target_id: usize,
    ) -> Self {
        let max_angle_dev = {
            let vel_perp = vel.perp_unit(true) * FlightComputer::ACC_CONST;
            vel.angle_to(&vel_perp).abs()
        };
        let dynamic_fuel_w = math::interpolate(
            FlightComputer::MIN_0,
            FlightComputer::MAX_100,
            Self::MIN_FUEL_W,
            Self::MAX_FUEL_W,
            fuel_left,
        );
        Self {
            i,
            vel,
            targets,
            max_dt,
            min_dt,
            max_off_orbit_dt,
            max_angle_dev,
            turns,
            fuel_left,
            dynamic_fuel_w,
            target_id,
            best_burn: None,
        }
    }

    #[allow(clippy::cast_possible_wrap)]
    pub fn process_dt(&mut self, dt: usize, max_needed_batt: I32F32) {
        let pos = (self.i.pos() + self.vel * I32F32::from_num(dt)).wrap_around_map().round();
        let bs_i = self.i.new_from_future_pos(pos, self.i.t() + TimeDelta::seconds(dt as i64));

        let n_target = *self.targets.iter().min_by_key(|t| pos.unwrapped_to(&t.0).abs()).unwrap();
        let shortest_dir = pos.unwrapped_to(&n_target.0);

        if self.vel.angle_to(&shortest_dir).abs() > Self::NINETY_DEG {
            return;
        }
        let (turns_in_dir, break_cond) = {
            if shortest_dir.is_clockwise_to(&self.vel).unwrap_or(false) {
                (&self.turns.0, false)
            } else {
                (&self.turns.1, true)
            }
        };
        if let Some(b) = self.build_burn_sequence(bs_i, turns_in_dir, break_cond, &n_target) {
            let cost = self.get_bs_cost(&b);
            let add_cost = Self::get_add_target_cost(&b, &n_target);
            let curr_cost = self.best_burn.as_ref().map_or(I32F32::MAX, ExitBurnResult::cost);
            if curr_cost > cost.saturating_add(add_cost)
                && b.min_charge() <= max_needed_batt
                && b.min_fuel() <= self.fuel_left
            {
                let unwrapped_target = Self::get_unwrapped_target(&b, &n_target.0);
                self.best_burn = Some(ExitBurnResult::new(b, n_target, unwrapped_target, cost, self.target_id));
            }
        }
    }

    pub fn get_unwrapped_target(b: &BurnSequence, tar: &Vec2D<I32F32>) -> Vec2D<I32F32> {
        let impact_pos = *b.sequence_pos().last().unwrap()
            + *b.sequence_vel().last().unwrap() * I32F32::from_num(b.detumble_dt());
        let wrapped_impact_pos = impact_pos.wrap_around_map();
        let offset = wrapped_impact_pos.to(tar);
        impact_pos + offset
    }

    pub fn get_best_burn(self) -> Option<ExitBurnResult> { self.best_burn }

    #[allow(clippy::cast_possible_wrap, clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    fn build_burn_sequence(
        &self,
        burn_i: IndexedOrbitPosition,
        turns_in_dir: &[(Vec2D<I32F32>, Vec2D<I32F32>)],
        break_cond: bool,
        best_target: &(Vec2D<I32F32>, Vec2D<I32F32>),
    ) -> Option<BurnSequence> {
        let mut add_dt = 0;
        let mut fin_sequence_pos: Vec<Vec2D<I32F32>> = vec![burn_i.pos()];
        let mut fin_sequence_vel: Vec<Vec2D<I32F32>> = vec![self.vel];

        for atomic_turn in turns_in_dir {
            // Update position and velocity based on turns
            let next_seq_pos = (burn_i.pos() + atomic_turn.0).wrap_around_map();
            let next_vel = atomic_turn.1;

            let next_to_target = next_seq_pos.unwrapped_to(&best_target.0);
            let min_dt = (next_to_target.abs() / next_vel.abs()).round().to_num::<usize>();
            let min_add_target_dt =
                (best_target.1.abs() / next_vel.abs()).abs().round().to_num::<usize>();
            let dt = (burn_i.t() - Utc::now()).num_seconds() as usize;
            // Check if the maneuver exceeds the maximum allowed time
            add_dt += 1;
            let obj_finish_dt = min_dt + dt + add_dt + min_add_target_dt;
            let obj_arrival_dt = min_dt + dt + add_dt;
            if obj_finish_dt > self.max_dt || obj_arrival_dt < self.min_dt || min_dt < TaskController::MANEUVER_MIN_DETUMBLE_DT 
            {
                return None;
            }

            // Break and finalize the burn sequence if close enough to the target
            if next_to_target.is_clockwise_to(&next_vel).unwrap_or(break_cond) == break_cond {
                let last_pos = fin_sequence_pos.last().unwrap();
                let last_vel = fin_sequence_vel.last().unwrap();
                let (fin_dt, fin_angle_dev) = {
                    let last_to_target = last_pos.unwrapped_to(&best_target.0);
                    let last_angle_deviation = -last_vel.angle_to(&last_to_target);
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
                    let corr_pos = (*last_pos + corr_vel).wrap_around_map();
                    let corr_to_target = corr_pos.unwrapped_to(&best_target.0);
                    let corr_angle_dev = corr_vel.angle_to(&corr_to_target);
                    fin_sequence_pos.push(corr_pos.round());
                    fin_sequence_vel.push(corr_vel);
                    add_dt += 1;
                    (min_dt + dt + add_dt, corr_angle_dev)
                };
                let last_vel = fin_sequence_vel.last().unwrap();
                let add_target_traversal_time =
                    (best_target.1.abs() / last_vel.abs()).to_num::<usize>();
                return Some(BurnSequence::new(
                    burn_i,
                    Box::from(fin_sequence_pos),
                    Box::from(fin_sequence_vel),
                    add_dt,
                    fin_dt - dt - add_dt,
                    fin_angle_dev,
                    add_target_traversal_time,
                ));
            }
            fin_sequence_pos.push(next_seq_pos);
            fin_sequence_vel.push(next_vel);
        }
        None
    }

    fn get_add_target_cost(bs: &BurnSequence, target: &(Vec2D<I32F32>, Vec2D<I32F32>)) -> I32F32 {
        let last_pos = bs.sequence_pos().last().unwrap();
        let last_to_target = last_pos.unwrapped_to(&target.0);

        let angle_deviation = target.1.angle_to(&last_to_target);

        let add_angle_dev =
            math::normalize_fixed32(angle_deviation, I32F32::zero(), Self::NINETY_DEG)
                .unwrap_or(I32F32::zero());
        add_angle_dev * Self::ADD_ANGLE_DEV_W
    }

    fn get_bs_cost(&self, bs: &BurnSequence) -> I32F32 {
        let max_add_dt = self.turns.0.len().max(self.turns.1.len());
        // Normalize the factors contributing to burn sequence cost
        let norm_fuel = math::normalize_fixed32(
            I32F32::from_num(bs.acc_dt()) * FlightComputer::FUEL_CONST,
            I32F32::zero(),
            I32F32::from_num(max_add_dt) * FlightComputer::FUEL_CONST,
        )
        .unwrap_or(I32F32::zero());

        let norm_off_orbit_dt = math::normalize_fixed32(
            I32F32::from_num(bs.acc_dt() + bs.detumble_dt()),
            I32F32::zero(),
            I32F32::from_num(self.max_off_orbit_dt),
        )
        .unwrap_or(I32F32::zero());

        let norm_angle_dev =
            math::normalize_fixed32(bs.rem_angle_dev().abs(), I32F32::zero(), self.max_angle_dev)
                .unwrap_or(I32F32::zero());

        // Compute the total cost of the burn sequence
        Self::OFF_ORBIT_W * norm_off_orbit_dt
            + self.dynamic_fuel_w * norm_fuel
            + Self::ANGLE_DEV_W * norm_angle_dev
    }
}
