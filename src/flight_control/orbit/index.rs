use crate::flight_control::common::vec2d::Vec2D;

#[derive(Debug, Copy, Clone)]
pub struct IndexedOrbitPosition {
    t: chrono::DateTime<chrono::Utc>,
    index: usize,
    pos: Vec2D<f32>,
    period: usize,
}

impl IndexedOrbitPosition {
    pub fn new(index: usize, period: usize, pos: Vec2D<f32>) -> Self {
        Self {
            t: chrono::Utc::now(),
            index,
            pos,
            period,
        }
    }

    pub fn t(&self) -> chrono::DateTime<chrono::Utc> { self.t }

    pub fn pos(&self) -> Vec2D<f32> { self.pos }

    pub fn index(&self) -> usize { self.index }

    pub fn get_ranges_to_now(&self) -> Vec<(usize, usize)> {
        let end = self.index_now() % self.period;
        if end < self.index {
            vec![(self.index, self.period), (0, end)]
        } else {
            vec![(self.index, self.index_now() % self.period)]
        }
    }

    pub fn new_from_pos(&self, pos: Vec2D<f32>) -> Self {
        Self {
            t: chrono::Utc::now(),
            index: self.index_now(),
            pos,
            period: self.period,
        }
    }

    pub fn new_from_future_pos(&self, pos: Vec2D<f32>, dt: chrono::Duration) -> Self {
        Self {
            t: self.t + dt,
            index: self.index_then(dt),
            pos,
            period: self.period,
        }
    }

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    fn index_now(&self) -> usize {
        (self.index + (chrono::Utc::now() - self.t).num_seconds() as usize) % self.period
    }

    fn index_then(&self, dt: chrono::Duration) -> usize {
        (self.index + dt.num_seconds() as usize) % self.period 
    }
}
