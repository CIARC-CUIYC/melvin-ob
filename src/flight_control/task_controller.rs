use crate::flight_control::common::{image_task::ImageTask, locked_task_queue::LockedTaskQueue};
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

impl TaskController {
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
        if schedule.pop().is_some() { self.next_image_notify.notify_all() }
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
