use num_traits::{real::Real, Num, NumAssignOps, NumCast, Signed, ToPrimitive};
use std::ops::{Add, Mul};
use chrono::{DateTime, TimeDelta, Utc};

#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub struct Vec2D<T> {
    x: T,
    y: T,
}

impl<T> Vec2D<T>
where T: Real + NumCast + NumAssignOps
{
    pub fn abs(&self) -> T { (self.x.powi(2) + self.y.powi(2)).sqrt() }

    pub fn to(&self, other: &Vec2D<T>) -> Vec2D<T> {
        Vec2D::new(other.x - self.x, other.y - self.y)
    }

    pub fn normalize(self) -> Self {
        let magnitude = self.abs();
        if magnitude.is_zero() {
            self
        } else {
            Self::new(self.x / magnitude, self.y / magnitude)
        }
    }

    pub fn rotate_by(&mut self, angle_degrees: f32) {
        let angle_radians = T::from(angle_degrees.to_radians()).unwrap();
        let new_x = self.x * angle_radians.cos() - self.y * angle_radians.sin();
        self.y = self.x * angle_radians.sin() + self.y * angle_radians.cos();
        self.x = new_x;
    }

    pub fn euclid_distance(&self, other: &Self) -> T {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
}

impl<T> Vec2D<T> {
    pub fn new(x: T, y: T) -> Self { Self { x, y } }
}

// TODO: wrap_around_map should be implemented in this block to work with integer types
// TODO: fix vec2D implementation to fix vector types everywhere
impl<T: Num + NumCast + Copy> Vec2D<T> {
    pub fn dot(self, other: Vec2D<T>) -> T { self.x * other.x + self.y * other.y }

    pub fn euclid_distance_f64(&self, other: &Self) -> f64 {
        let self_x = self.x.to_f64().unwrap();
        let self_y = self.y.to_f64().unwrap();
        let other_x = other.x.to_f64().unwrap();
        let other_y = other.y.to_f64().unwrap();
        ((self_x - self_y).powi(2) + (other_x - other_y).powi(2)).sqrt()
    }

    pub fn in_radius_of(&self, other: &Self, rad: T) -> bool {
        self.euclid_distance_f64(other) <= rad.to_f64().unwrap()
    }

    pub fn abs_f64(self) -> f64 { (self.x * self.x + self.y * self.y).to_f64().unwrap().sqrt() }

    pub fn zero() -> Self { Self::new(T::zero(), T::zero()) }

    pub fn x(&self) -> T { self.x }
    pub fn y(&self) -> T { self.y }

    pub fn map_size() -> Vec2D<T> {
        Vec2D {
            x: T::from(21600.0).unwrap(),
            y: T::from(10800.0).unwrap(),
        }
    }

    pub fn wrap_around_map(&mut self) {
        let map_size_x = Self::map_size().x();
        let map_size_y = Self::map_size().y();

        self.x = Self::wrap_coordinate(self.x, map_size_x);
        self.y = Self::wrap_coordinate(self.y, map_size_y);
    }

    /// Helper function to wrap a single coordinate
    pub fn wrap_coordinate(value: T, max_value: T) -> T {
        (value + max_value) % max_value
    }
}

impl<T, TAdd> Add<Vec2D<TAdd>> for Vec2D<T>
where
    T: Num + NumCast,
    TAdd: Num + NumCast,
{
    type Output = Vec2D<T>;

    fn add(self, rhs: Vec2D<TAdd>) -> Self::Output {
        Self::Output {
            x: self.x + T::from(rhs.x).unwrap(),
            y: self.y + T::from(rhs.y).unwrap(),
        }
    }
}

impl<T, TMul> Mul<TMul> for Vec2D<T>
where
    T: Num + NumCast,
    TMul: Num + NumCast + Copy,
{
    type Output = Vec2D<T>;
    fn mul(self, rhs: TMul) -> Self::Output {
        Self::Output {
            x: self.x * T::from(rhs).unwrap(),
            y: self.y * T::from(rhs).unwrap(),
        }
    }
}

impl<T: Num + NumCast> From<(T, T)> for Vec2D<T> {
    fn from(tuple: (T, T)) -> Self {
        Vec2D {
            x: tuple.0,
            y: tuple.1,
        }
    }
}

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
