use crate::flight_control::common::image_task::{ImageTask, LockedTaskQueue};
use std::sync::{Arc, Condvar};

#[derive(Debug)]
pub struct TaskController {
    /// Schedule for the next images, represented by image tasks.
    image_schedule: Arc<LockedTaskQueue>,
    next_image_notify: Arc<Condvar>,
}

impl TaskController {
    /// Creates a new instance of the `TaskController` struct.
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
    
    /// Removes the next scheduled image capture from the schedule.
    pub fn remove_next_image(&mut self) {
        let schedule = &*self.image_schedule;
        schedule.pop().map(|_| self.next_image_notify.notify_all());
    }
    
    pub fn get_time_to_next_image(&self) -> Option<chrono::Duration> {
        let schedule = &*self.image_schedule;
        schedule.copy_front().map_or(None, |task| Some(task.dt().time_left()))
    }

    /// Clears the queue with all scheduled images.
    pub fn clear_schedule(&mut self) {
        let schedule = &*self.image_schedule;
        if schedule.is_empty() {
            self.next_image_notify.notify_all();
        }
        schedule.clear();
    }
}
