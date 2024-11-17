use std::ops::Add;
use num_traits::Num;

#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub struct Vec<T: Num> {
    x: T,
    y: T,
}

impl<T: Num> Vec<T> {
    pub fn new(x: T, y: T) -> Self {
        Self { x, y }
    }
}

impl<T: Num> Add for Vec<T> {
    type Output = Vec<T>;

    fn add(self, rhs: Self) -> Self::Output {
        Self::Output { x: self.x + rhs.x, y: self.y + rhs.y }
    }
}

impl<T: Num> From<(T, T)> for Vec<T> {
    fn from(tuple: (T, T)) -> Self {
        Vec { x: tuple.0, y: tuple.1 }
    }
}
