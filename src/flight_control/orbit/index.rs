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

    pub fn get_range_to_now(&self) -> Vec<(usize, usize)> { 
        let end = self.i_now() % self.period;
        if end < self.index {
            vec![(self.index, self.period), (0, end)]
        } else {
            vec![(self.index, self.i_now() % self.period)]
        }
        
    }

    pub fn new_from_pos(&self, pos: Vec2D<f32>) -> Self {
        Self {
            t: chrono::Utc::now(),
            index: self.i_now(),
            pos,
            period: self.period
        }
    }

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    fn i_now(&self) -> usize { (chrono::Utc::now() - self.t).num_seconds() as usize }
}
