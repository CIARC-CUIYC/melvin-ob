use std::collections::VecDeque;

/// A fixed-size linked list data structure. 
/// This structure uses a `VecDeque` internally and maintains a maximum size. 
/// When the maximum size is exceeded, the oldest element (at the back) is removed.
pub struct LinkedBox<T> {
    /// `VecDeque` holding the actual data
    list: VecDeque<T>,
    /// `max_size` represents the maximum length of the queue
    max_size: usize,
}

impl<T> LinkedBox<T> {
    /// Creates a new `LinkedBox` with the specified maximum size.
    ///
    /// # Arguments
    ///
    /// * `max_size` - The maximum number of elements that the linked list can hold.
    ///
    /// # Returns
    /// A new, empty `LinkedBox` instance.
    pub fn new(max_size: usize) -> Self {
        Self {
            list: VecDeque::new(),
            max_size,
        }
    }

    /// Pushes an element to the front of the list.
    ///
    /// If the size limit is exceeded, the element at the back of the list is removed.
    ///
    /// # Arguments
    /// * `item` - The element to be added to the front of the list.
    pub fn push(&mut self, item: T) {
        self.list.push_front(item);
        if self.len() > self.max_size {
            self.list.pop_back();
        }
    }

    /// Returns a reference to the first element in the list, if present.
    ///
    /// # Returns
    /// An `Option` containing a reference to the first element, or `None` if the list is empty.
    pub fn front(&self) -> Option<&T> { self.list.front() }

    /// Returns a reference to the last element in the list, if present.
    ///
    /// # Returns
    /// An `Option` containing a reference to the last element, or `None` if the list is empty.
    pub fn back(&self) -> Option<&T> { self.list.back() }

    /// Returns the current number of elements in the list.
    ///
    /// # Returns
    /// The number of elements in the list as a `usize`.
    pub fn len(&self) -> usize { self.list.len() }

    /// Checks if the list is empty.
    ///
    /// # Returns
    /// A boolean value, `true` if the list is empty, `false` otherwise.
    pub fn is_empty(&self) -> bool { self.list.is_empty() }
}
