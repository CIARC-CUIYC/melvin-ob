use num_traits::{One, Zero};
use std::ops::{Add, Div, Mul, Sub};

#[derive(Copy, Clone)]
pub struct Matrix<T, const M: usize, const N: usize> {
    pub(crate) data: [[T; N]; M],
}

impl<T, const M: usize, const N: usize> Matrix<T, M, N>
where T: Copy
{
    pub fn new(data: [[T; N]; M]) -> Self { Matrix { data } }

    pub fn get(&self, row: usize, col: usize) -> &T { &self.data[row][col] }

    pub fn set(&mut self, row: usize, col: usize, value: T) { self.data[row][col] = value; }
}

impl<T, const M: usize, const N: usize> Matrix<T, M, N>
where T: Copy + Zero
{
    pub fn zero() -> Self {
        Self {
            data: [[T::zero(); N]; M],
        }
    }
}

impl<T, const N: usize> Matrix<T, N, N>
where T: Copy + Default + One
{
    pub fn identity() -> Self {
        let mut result = [[T::default(); N]; N];

        for i in 0..N {
            result[i][i] = T::one();
        }

        Matrix::new(result)
    }
}

impl<T, const M: usize, const N: usize> Matrix<T, M, N>
where T: Copy + Default + One
{
    pub fn eye() -> Self {
        let mut result = [[T::default(); N]; M];

        for i in 0..M.min(N) {
            // Ensures diagonal elements exist
            result[i][i] = T::one();
        }

        Matrix::new(result)
    }
}

impl<T, const M: usize, const N: usize> Add for Matrix<T, M, N>
where T: Copy + Default + Add<Output = T>
{
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        let mut result = [[T::default(); N]; M].map(|row| row.map(|_| T::default()));

        for i in 0..M {
            for j in 0..N {
                result[i][j] = self.data[i][j] + rhs.data[i][j];
            }
        }

        Matrix::new(result)
    }
}

impl<T, const M: usize, const N: usize> Sub for Matrix<T, M, N>
where T: Copy + Default + Sub<Output = T>
{
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        let mut result = [[T::default(); N]; M];

        for i in 0..M {
            for j in 0..N {
                result[i][j] = self.data[i][j] - rhs.data[i][j];
            }
        }

        Matrix::new(result)
    }
}

impl<T, const M: usize, const N: usize, const P: usize> Mul<Matrix<T, N, P>> for Matrix<T, M, N>
where T: Copy + Default + Add<Output = T> + Mul<Output = T>
{
    type Output = Matrix<T, M, P>;

    fn mul(self, rhs: Matrix<T, N, P>) -> Self::Output {
        let mut result = [[T::default(); P]; M];

        for i in 0..M {
            for j in 0..P {
                let mut sum = T::default();
                for k in 0..N {
                    sum = sum + self.data[i][k] * rhs.data[k][j];
                }
                result[i][j] = sum;
            }
        }

        Matrix::new(result)
    }
}

impl<T, const M: usize, const N: usize> Matrix<T, M, N>
where T: Copy + Default
{
    pub fn transpose(&self) -> Matrix<T, N, M> {
        let mut result = Matrix {
            data: [[T::default(); M]; N],
        };

        for i in 0..M {
            for j in 0..N {
                result.data[j][i] = self.data[i][j];
            }
        }

        result
    }
}

impl<T, const N: usize> Matrix<T, N, N>
where T: Copy
        + Default
        + Zero
        + One
        + Add<Output = T>
        + Sub<Output = T>
        + Mul<Output = T>
        + Div<Output = T>
        + PartialEq
{
    pub fn try_inverse(&self) -> Option<Self> {
        let mut left = self.clone();

        // Create an identity matrix (right side of augmentation)
        let mut right = Matrix::<T, N, N>::identity();

        // Perform Gauss-Jordan elimination
        for i in 0..N {
            // Check if pivot element is zero (matrix is singular)
            if left.data[i][i] == T::zero() {
                return None; // No inverse exists
            }

            let pivot = left.data[i][i];

            // Normalize pivot row
            for j in 0..N {
                left.data[i][j] = left.data[i][j] / pivot;
                right.data[i][j] = right.data[i][j] / pivot;
            }

            // Eliminate other rows
            for k in 0..N {
                if k != i {
                    let factor = left.data[k][i];
                    for j in 0..N {
                        left.data[k][j] = left.data[k][j] - factor * left.data[i][j];
                        right.data[k][j] = right.data[k][j] - factor * right.data[i][j];
                    }
                }
            }
        }

        Some(right)
    }
}
