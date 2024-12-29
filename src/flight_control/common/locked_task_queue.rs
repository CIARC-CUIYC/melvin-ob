use std::collections::VecDeque;
use std::sync::Mutex;
use crate::flight_control::common::image_task::ImageTask;

#[derive(Debug)]
pub(crate) struct LockedTaskQueue {
    queue: Mutex<VecDeque<ImageTask>>,
}

impl LockedTaskQueue {
    pub fn new() -> Self {
        Self {
            queue: Mutex::new(VecDeque::new()),
        }
    }

    pub fn lock_queue(&self) -> std::sync::MutexGuard<VecDeque<ImageTask>> {
        self.queue
            .lock()
            .expect("[FATAL] Mutex poisoned: Failed to acquire lock")
    }

    /// Pushes an element onto the back of the queue.
    pub fn push(&self, task: ImageTask) { self.lock_queue().push_back(task) }

    /// Removes and returns the front element of the queue.
    pub fn pop(&self) -> Option<ImageTask> { self.lock_queue().pop_front() }

    /// Returns a copy to the front element of the queue.
    pub fn copy_front(&self) -> Option<ImageTask> {
        let locked_queue = self.lock_queue();
        let first_ref = locked_queue.front();
        match first_ref {
            Some(x) => Some(*x),
            None => None,
        }
    }

    /// Returns the length of the queue.
    pub fn len(&self) -> usize { self.lock_queue().len() }

    /// Checks if the queue is empty.
    pub fn is_empty(&self) -> bool { self.lock_queue().is_empty() }

    /// Clears the queue.
    pub fn clear(&self) { self.lock_queue().clear() }

    /// Provides a mutable iterator for the queue.
    pub fn iter_mut<F>(&self, mut func: F)
    where
        F: FnMut(&mut ImageTask),
    {
        self.lock_queue().iter_mut().for_each(|x| func(x));
    }
}