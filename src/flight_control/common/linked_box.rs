use std::collections::LinkedList;

pub struct LinkedBox<T> {
    list: LinkedList<T>,
    max_size: usize,
}

impl<T> LinkedBox<T> {
    /// Creates a new fixed-size linked list with a maximum size.
    pub fn new(max_size: usize) -> Self {
        Self {
            list: LinkedList::new(),
            max_size,
        }
    }

    /// Pushes an element to the front of the list.
    /// If the size limit is exceeded, the last element is removed.
    pub fn push(&mut self, item: T) {
        self.list.push_front(item);
        if self.len() > self.max_size {
            self.list.pop_back();
        }
    }

    /// Returns a reference to the first element, if present.
    pub fn front(&self) -> Option<&T> {
        self.list.front()
    }

    /// Returns a reference to the last element, if present.
    pub fn back(&self) -> Option<&T> {
        self.list.back()
    }

    /// Returns the current number of elements in the list.
    pub fn len(&self) -> usize {
        self.list.len()
    }

    /// Checks if the list is empty.
    pub fn is_empty(&self) -> bool {
        self.list.is_empty()
    }    
}