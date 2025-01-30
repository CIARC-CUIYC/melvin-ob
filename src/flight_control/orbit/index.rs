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

    pub fn period(&self) -> usize { self.period }

    pub fn get_ranges_to_now(&self, shift: Option<usize>) -> Vec<(usize, usize)> {
        let end = {
            if let Some(sh) = shift {
                (((self.index_now() - sh) % self.period) + self.period) % self.period
            } else {
                self.index_now() % self.period
            }
        };
        if end < self.index {
            vec![(self.index, self.period), (0, end)]
        } else {
            vec![(self.index, self.index_now() % self.period)]
        }
    }

    #[allow(clippy::cast_sign_loss)]
    pub fn map_ranges(ranges: &Vec<(isize, isize)>, max: isize) -> Vec<(usize, usize)> {
        let mut mapped_ranges = Vec::new();
        for range in ranges {
            let start = (((range.0 % max) + max) % max) as usize;
            let end = (((range.0 % max) + max) % max) as usize;
            if start < end {
                mapped_ranges.push((start, end));
            } else {
                mapped_ranges.push((start, (max - 1) as usize));
                mapped_ranges.push((0, end));
            }
        }
        mapped_ranges
    }

    pub fn new_from_pos(&self, pos: Vec2D<f32>) -> Self {
        Self {
            t: chrono::Utc::now(),
            index: self.index_now(),
            pos,
            period: self.period,
        }
    }

    pub fn new_from_future_pos(&self, pos: Vec2D<f32>, dt: chrono::TimeDelta) -> Self {
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

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    fn index_then(&self, dt: chrono::TimeDelta) -> usize {
        (self.index + dt.num_seconds() as usize) % self.period
    }
}
