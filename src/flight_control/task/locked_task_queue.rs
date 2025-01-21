use super::base_task::Task;
use std::collections::VecDeque;
use std::sync::Mutex;

/// A thread-safe task queue for managing image capture tasks.
/// This structure wraps a `VecDeque` with a `Mutex` to ensure thread safety.
#[derive(Debug)]
pub(crate) struct LockedTaskQueue {
    /// The queue storing image capture tasks.
    queue: Mutex<VecDeque<Task>>,
}

impl LockedTaskQueue {
    /// Creates a new `LockedTaskQueue`.
    ///
    /// # Returns
    /// - A new instance of `LockedTaskQueue` with an empty task queue.
    pub fn new() -> Self {
        Self {
            queue: Mutex::new(VecDeque::new()),
        }
    }

    /// Acquires a lock on the task queue.
    ///
    /// # Returns
    /// - A `MutexGuard` that allows access to the underlying `VecDeque`.
    ///
    /// # Panics
    /// - If the Mutex is poisoned.
    pub fn lock_queue(&self) -> std::sync::MutexGuard<VecDeque<Task>> {
        self.queue
            .lock()
            .expect("[FATAL] Mutex poisoned: Failed to acquire lock")
    }

    /// Adds a new task to the back of the queue.
    ///
    /// # Arguments
    /// - `task`: The `ImageTask` to add.
    pub fn push(&self, task: Task) { self.lock_queue().push_back(task) }

    /// Removes and returns the task at the front of the queue.
    ///
    /// # Returns
    /// - An `Option<ImageTask>` containing the removed task, or `None` if the queue is empty.
    pub fn pop(&self) -> Option<Task> { self.lock_queue().pop_front() }

    /// Returns a copy of the task at the front of the queue without removing it.
    ///
    /// # Returns
    /// - `Some<ImageTask>` containing the first task, or `None` if the queue is empty.
    pub fn copy_front(&self) -> Option<Task> {
        let locked_queue = self.lock_queue();
        let first_ref = locked_queue.front();
        first_ref.copied()
    }

    /// Returns the length of the task queue.
    ///
    /// # Returns
    /// - The number of tasks currently in the queue.
    pub fn len(&self) -> usize { self.lock_queue().len() }

    /// Checks if the task queue is empty.
    ///
    /// # Returns
    /// - `true` if the queue is empty, `false` otherwise.
    pub fn is_empty(&self) -> bool { self.lock_queue().is_empty() }

    /// Clears all tasks from the queue.
    pub fn clear(&self) { self.lock_queue().clear() }

    /// Iterates over the tasks in the queue and applies a provided function to each.
    ///
    /// # Arguments
    /// - `func`: A closure to apply to each task in the queue.
    pub fn for_each<F>(&self, func: F)
    where F: FnMut(&mut Task) {
        self.lock_queue().iter_mut().for_each(func);
    }
}
