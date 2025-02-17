use crate::flight_control::common::matrix::Matrix;
use crate::flight_control::common::state_vector::StateVector;
use crate::flight_control::common::vec2d::Vec2D;
use fixed::types::I32F32;

pub struct Kalman<const N: usize, const M: usize> {
    pub x: StateVector<I32F32, N>, // state estimate
    p: Matrix<I32F32, N, N>,       // estimate covariance matrix

    h: Matrix<I32F32, M, N>, // observation measurement matrix
    r: Matrix<I32F32, M, M>, // measurement noise covariance

    f: Matrix<I32F32, N, N>, // state transition matrix
    q: Matrix<I32F32, N, N>, // process noise variance
}

impl<const N: usize, const M: usize> Kalman<N, M> {
    pub fn new() -> Self {
        Self {
            x: StateVector::zero(),
            p: Matrix::identity(),
            h: Matrix::eye(),
            r: Matrix::identity(),
            f: Matrix::identity(),
            q: Matrix::identity(),
        }
    }

    pub fn predict(&mut self) {
        self.x = StateVector::from_matrix(self.f * self.x.to_matrix());
        self.p = self.f * self.p * self.f.transpose() + self.q;
    }

    pub fn update(&mut self, z: StateVector<I32F32, M>) {
        let y = z - StateVector::from_matrix(self.h * self.x.to_matrix());
        // calculate innovation covariance matrix
        let s = self.h * self.p * self.h.transpose() + self.r;
        // calculate kalman gain
        let k = self.p * self.h.transpose() * s.try_inverse().unwrap();
        // update state estimate
        self.x = self.x + StateVector::from_matrix(k * y.to_matrix());
        // update estimate covariance matrix
        self.p = self.p - k * self.h * self.p;
    }

    // pub fn predict_pos_deviation(&self, steps: usize) -> Vec2D<I32F32> {
    //     let dx = self.x[0];
    //     let dy = self.x[1];
    //     let vx = self.x[2];
    //     let vy = self.x[3];

    //     let predicted_x = dx + vx * steps;
    //     let predicted_y = dy + vy * steps;

    //     Vec2D::new(predicted_x, predicted_y)
    // }

    pub fn log_deviation(&mut self, current_pos: Vec2D<I32F32>) {
        self.predict();
        self.update(StateVector::from_vec2d(current_pos));

        let est_deviation = self.x.get_slice(0, 2);
        let est_vel = self.x.get_slice(2, 2);

        println!(
            "Deviation: [{:.2}, {:.2}], Velocity: [{:.2}, {:.2}]",
            est_deviation[0], est_deviation[1], est_vel[0], est_vel[1]
        );
    }
}
