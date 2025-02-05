use nalgebra::{RealField, SMatrix, SVector};

struct Kalman<T: RealField, const N: usize, const M: usize> {
    x: SVector<T, N>,    // state estimate
    p: SMatrix<T, N, N>, // estimate covariance matrix

    h: SMatrix<T, M, N>, // observation measurement matrix
    r: SMatrix<T, M, M>, // measurement noise covariance

    f: SMatrix<T, N, N>, // state transition matrix
    q: SMatrix<T, N, N>, // process noise variance
}

impl<T: RealField + Copy, const N: usize, const M: usize> Kalman<T, N, M> {
    fn new() -> Self {
        Self {
            x: SVector::zeros(),
            p: SMatrix::identity(),
            h: SMatrix::identity(),
            r: SMatrix::identity(),
            f: SMatrix::identity(),
            q: SMatrix::identity(),
        }
    }

    fn predict(&mut self) {
        self.x = self.f * self.x;
        self.p = self.f * self.p * self.f.transpose() + self.q;
    }

    fn update(&mut self, z: &SVector<T, M>) {
        let y = z - (self.h * self.x);

        let s = self.h * self.p * self.h.transpose() + self.r;

        let k = self.p * self.h.transpose() * s.try_inverse().unwrap();

        self.x += k * y;

        self.p = self.p - k * self.h * self.p;
    }
}
