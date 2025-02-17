use crate::flight_control::common::matrix::Matrix;
use crate::flight_control::common::vec2d::Vec2D;
use num_traits::Zero;
use std::ops::{Add, Index, Sub};

/// A Fixed-Size State Vector for Kalman Filter
#[derive(Copy, Clone)]
pub struct StateVector<T, const N: usize> {
    pub data: [T; N],
}

impl<T: Copy + Zero, const N: usize> StateVector<T, N> {
    pub fn zero() -> Self {
        Self {
            data: [T::zero(); N],
        }
    }

    pub fn from_matrix(matrix: Matrix<T, N, 1>) -> Self {
        let mut vec = Self::zero();
        for i in 0..N {
            vec.data[i] = *matrix.get(i, 0);
        }
        vec
    }

    pub fn to_matrix(self) -> Matrix<T, N, 1> {
        let mut mat = Matrix::<T, N, 1>::zero();
        for i in 0..N {
            mat.set(i, 0, self.data[i]);
        }
        mat
    }

    pub fn from_vec2d(vec2d: Vec2D<T>) -> Self {
        let mut vec = Self::zero();
        vec.data[0] = vec2d.x();
        vec.data[1] = vec2d.y();

        vec
    }

    pub fn get_slice(&self, start_idx: usize, range: usize) -> &[T] {
        &self.data[start_idx..(start_idx + range)]
    }
}

impl<T: Copy + Add<Output = T>, const N: usize> Add for StateVector<T, N> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        let mut result = self;
        for i in 0..N {
            result.data[i] = self.data[i] + rhs.data[i];
        }
        result
    }
}

impl<T: Copy + Sub<Output = T>, const N: usize> Sub for StateVector<T, N> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        let mut result = self;
        for i in 0..N {
            result.data[i] = self.data[i] - rhs.data[i];
        }
        result
    }
}

impl<T: Copy, const N: usize> Index<usize> for StateVector<T, N> {
    type Output = T;

    fn index(&self, idx: usize) -> &Self::Output { &self.data[idx] }
}
