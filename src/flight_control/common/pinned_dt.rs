use chrono::{DateTime, TimeDelta, Utc};

#[derive(Debug, Clone, Copy)]
pub struct PinnedTimeDelay {
    start_time: DateTime<Utc>,
    delay: TimeDelta,
}

impl PinnedTimeDelay {
    pub fn new(delta: TimeDelta) -> Self {
        Self {
            start_time: Utc::now(),
            delay: delta,
        }
    }
    pub fn remove_delay(&mut self, delta: TimeDelta) { self.delay -= delta; }
    pub fn add_delay(&mut self, delta: TimeDelta) { self.delay += delta; }
    pub fn set_delay(&mut self, delta: TimeDelta) { self.delay = delta; }
    pub fn get_end(&self) -> DateTime<Utc> { self.start_time + self.delay }
    pub fn get_start(&self) -> DateTime<Utc> { self.start_time }
    pub fn time_left(&self) -> TimeDelta { self.get_end() - Utc::now() }
}
