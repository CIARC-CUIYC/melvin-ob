use crate::flight_control::common::linked_box::LinkedBox;
use crate::flight_control::{common::{
    image_task::ImageTask, locked_task_queue::LockedTaskQueue, vec2d::Vec2D},
                            orbit::closed_orbit::ClosedOrbit};
use crate::flight_control::flight_state::FlightState;
use crate::{CHARGE_CHARGE_PER_S, MAX_BATTERY_THRESHOLD, MIN_BATTERY_THRESHOLD};
use std::sync::{Arc, Condvar};

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
    image_schedule: Arc<LockedTaskQueue>,
    /// Notification condition variable to signal changes to the first element in `image_schedule`.
    next_image_notify: Arc<Condvar>,
}

#[derive(Debug, Clone, Copy)]
enum AtomicDecision {
    StayInCharge,
    StayInAcquisition,
    SwitchToCharge,
    SwitchToAcquisition,
}

impl TaskController {
    const MAX_ORBIT_PREDICTION_SECS: u32 = 20000;
    /// Creates a new instance of the `TaskController` struct.
    ///
    /// # Returns
    /// - A new `TaskController` with an empty task schedule.
    pub fn new() -> Self {
        Self {
            image_schedule: Arc::new(LockedTaskQueue::new()),
            next_image_notify: Arc::new(Condvar::new()),
        }
    }

    /// Adds a new image capture task to the back of the scheduling system.
    ///
    /// # Arguments
    /// - `task`: A `ImageTask` value.
    ///
    /// # Side Effects
    /// - Notifies all waiting threads if the schedule was previously empty.
    pub fn schedule_image(&mut self, task: ImageTask) {
        let schedule = &*self.image_schedule;
        if schedule.is_empty() {
            self.next_image_notify.notify_all();
        }
        schedule.push(task);
    }

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    pub fn calculate_optimal_orbit_schedule(&mut self, orbit: &ClosedOrbit, current_pos: Vec2D<f32>) {
        type CoverageDoubleBox = Box<[Box<[u16]>]>;

        let now = chrono::Local::now();
        println!("0: Calculating optimal orbit schedule...");
        let states = [FlightState::Charge, FlightState::Acquisition];
        let usable_batt_range = MAX_BATTERY_THRESHOLD - MIN_BATTERY_THRESHOLD;
        let max_battery = (usable_batt_range / CHARGE_CHARGE_PER_S).round() as usize + 1;
        let max_battery_index = max_battery - 1;
        let mut p_t = orbit.get_p_t_reordered(current_pos).rev();

        // initiate buffers
        let cov_dt_temp = vec![vec![0u16; 2].into_boxed_slice(); max_battery].into_boxed_slice();
        let mut decision_buffer =
            vec![
                vec![vec![AtomicDecision::StayInCharge; 2].into_boxed_slice(); max_battery]
                    .into_boxed_slice();
                Self::MAX_ORBIT_PREDICTION_SECS as usize
            ]
                .into_boxed_slice();
        // initiate fixed-length double linked list with first value
        let mut max_cov_buffer: LinkedBox<CoverageDoubleBox> =
            LinkedBox::new(Self::MAX_ORBIT_PREDICTION_SECS as usize);
        max_cov_buffer.push(cov_dt_temp.clone());
        println!("{}: Initialization complete.", chrono::Local::now() - now);
        for t in (0..Self::MAX_ORBIT_PREDICTION_SECS as usize).rev() {
            let mut cov_dt = cov_dt_temp.clone();
            let p_dt = u16::from(*p_t.next().unwrap());
            for e in 0..max_battery {
                for s in &states {
                    match *s {
                        FlightState::Charge => {
                            let stay = max_cov_buffer.front().unwrap()[(e + 1).min(max_battery_index)][0];
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
                                cov_dt[e][0] = stay;
                            } else {
                                decision_buffer[t][e][1] = AtomicDecision::SwitchToCharge;
                                cov_dt[e][0] = switch;
                            }
                        }
                        _ => break,
                    }
                }
            }
            max_cov_buffer.push(cov_dt.clone());
        }
        println!("{}: Calculation complete.", chrono::Local::now() - now);
    }

    /// Provides a reference to the image task schedule.
    ///
    /// # Returns
    /// - An `Arc` pointing to the `LockedTaskQueue`.
    pub fn sched_arc(&self) -> Arc<LockedTaskQueue> { Arc::clone(&self.image_schedule) }

    /// Provides a reference to the `Convar` signaling changes to the first item in `image_schedule`.
    ///
    /// # Returns
    /// - An `Arc` pointing to the `Condvar`.
    pub fn notify_arc(&self) -> Arc<Condvar> { Arc::clone(&self.next_image_notify) }

    /// Removes the next scheduled image capture task from the schedule.
    ///
    /// # Side Effects
    /// - Notifies all waiting threads if a task is removed.
    pub fn remove_next_image(&mut self) {
        let schedule = &*self.image_schedule;
        if schedule.pop().is_some() {
            self.next_image_notify.notify_all()
        }
    }

    /// Retrieves the time remaining until the next scheduled image.
    ///
    /// # Returns
    /// - `Some<chrono::Duration>` representing the time until the next image,
    ///   or `None` if the schedule is empty.
    pub fn get_time_to_next_image(&self) -> Option<chrono::Duration> {
        let schedule = &*self.image_schedule;
        schedule.copy_front().map(|task| task.dt().time_left())
    }

    /// Clears all pending tasks in the schedule.
    ///
    /// # Side Effects
    /// - Notifies all waiting threads if the schedule is cleared.
    pub fn clear_schedule(&mut self) {
        let schedule = &*self.image_schedule;
        if !schedule.is_empty() {
            self.next_image_notify.notify_all();
        }
        schedule.clear();
    }
}
