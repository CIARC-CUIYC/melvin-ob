use crate::flight_control::common::pinned_dt::PinnedTimeDelay;
use std::collections::VecDeque;
use std::sync::Mutex;

#[derive(Debug, Copy, Clone)]
pub struct ImageTask {
    dt: PinnedTimeDelay,
    // TODO: here should be additional information: planned pos, actual pos, px deviation?
}

impl ImageTask {
    pub fn new(dt: PinnedTimeDelay) -> Self { Self { dt } }
    pub fn dt_mut(&mut self) -> &mut PinnedTimeDelay { &mut self.dt }
    pub fn dt(&self) -> &PinnedTimeDelay { &self.dt }
}
