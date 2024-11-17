use std::ops::{Add, Mul};
use std::f64::consts::PI;
use num_traits::{NumCast, real::Real, Num};

#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub struct Vec2D<T: Num + NumCast> {
    x: T,
    y: T,
}

impl<T: Real + NumCast + std::ops::AddAssign<T>> Vec2D<T> {
    pub fn map_size() -> Vec2D<T> {
        Vec2D {
            x: T::from(21600.0).unwrap(),
            y: T::from(10600.0).unwrap(),
        }
    }

    pub fn new(x: T, y: T) -> Self { Self { x, y } }

    pub fn abs(&self) -> T { (self.x.powi(2) + self.y.powi(2)).sqrt() }

    pub fn wrap_around_map(&mut self) {
        let mut new_x = self.x % Self::map_size().x();
        let mut new_y = self.y % Self::map_size().y();
        if new_x < T::from(0).unwrap() { new_x += Self::map_size().x(); }
        if new_y < T::from(0).unwrap() { new_y += Self::map_size().y(); }
        self.x = new_x;
        self.y = new_y;
    }

    pub fn euclid_distance(&self, other: &Self) -> T {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }

    pub fn x(&self) -> T { self.x }
    pub fn y(&self) -> T { self.y }
}

impl<T, TAdd> Add<Vec2D<TAdd>> for Vec2D<T>
where
    T: Num + NumCast,
    TAdd: Num + NumCast,
{
    type Output = Vec2D<T>;

    fn add(self, rhs: Vec2D<TAdd>) -> Self::Output {
        Self::Output { x: self.x + T::from(rhs.x).unwrap(), y: self.y + T::from(rhs.y).unwrap() }
    }
}

impl<T, TMul> Mul<TMul> for Vec2D<T>
where
    T: Num + NumCast,
    TMul: Num + NumCast + Copy,
{
    type Output = Vec2D<T>;
    fn mul(self, rhs: TMul) -> Self::Output {
        Self::Output { x: self.x * T::from(rhs).unwrap(), y: self.y * T::from(rhs).unwrap() }
    }
}

impl<T: Num + NumCast> From<(T, T)> for Vec2D<T> {
    fn from(tuple: (T, T)) -> Self {
        Vec2D { x: tuple.0, y: tuple.1 }
    }
}
