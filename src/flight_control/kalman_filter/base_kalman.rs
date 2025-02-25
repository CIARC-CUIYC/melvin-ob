use crate::flight_control::common::matrix::Matrix;
use crate::flight_control::common::state_vector::StateVector;
use fixed::types::I32F32;

/// base Kalman Filter implementation for state estimation
pub struct BaseKalman<const N: usize, const M: usize> {
    /// state vector (x): represents state estimate
    pub state_vec: StateVector<I32F32, N>,
    /// covariance matrix (P): Represents the uncertainty of the current state estimate
    pub cov_mat: Matrix<I32F32, N, N>,
    /// observation matrix (H): Maps the true state space into the observed space
    pub obs_matrix: Matrix<I32F32, M, N>,
    /// measurement noise covariance matrix (R): Represents uncertainty in measurements
    pub meas_noise_cov_mat: Matrix<I32F32, M, M>,
    /// state transition matrix (F): Defines how the state evolves from one step to the next
    pub state_trans_mat: Matrix<I32F32, N, N>,
    /// noise covariance matrix (Q): Models the uncertainty in the system dynamics
    pub process_noise_cov_mat: Matrix<I32F32, N, N>,
}

impl<const N: usize, const M: usize> BaseKalman<N, M> {
    /// Predicts the next state and updates the covariance matrix
    pub fn predict(&mut self) {
        // Compute the predicted state estimate: x = F * x
        self.state_vec =
            StateVector::from_matrix(self.state_trans_mat * self.state_vec.to_matrix());

        // Compute the predicted covariance: P = F * P * F^T + Q
        self.cov_mat = self.state_trans_mat * self.cov_mat * self.state_trans_mat.transpose()
            + self.process_noise_cov_mat;
    }

    /// Updates the state estimate using a new measurement.
    ///
    /// # Parameters
    /// - `z`: The measurement vector representing observed state.
    ///
    /// # Returns
    /// - `Ok(())` if the update succeeds.
    /// - `Err(&static str)` if matrix inversion fails.
    pub fn update(&mut self, z: StateVector<I32F32, M>) -> Result<(), &str> {
        // Compute innovation (measurement residual): y = z - H * x
        let y = z - StateVector::from_matrix(self.obs_matrix * self.state_vec.to_matrix());

        // Compute innovation covariance: S = H * P * H^T + R
        let s =
            self.obs_matrix * self.cov_mat * self.obs_matrix.transpose() + self.meas_noise_cov_mat;

        // Attempt to compute the inverse of S (innovation covariance)
        let Some(s_inv) = s.try_inverse() else {
            return Err("Matrix inversion failed: Innovation covariance matrix is singular.");
        };

        // Compute Kalman gain: K = P * H^T * S^-1
        let kalman_gain_mat = self.cov_mat * self.obs_matrix.transpose() * s_inv;

        // Update state estimate: x = x + K * y
        self.state_vec = self.state_vec + StateVector::from_matrix(kalman_gain_mat * y.to_matrix());

        // Update covariance matrix: P = P - K * H * P
        self.cov_mat = self.cov_mat - kalman_gain_mat * self.obs_matrix * self.cov_mat;

        Ok(())
    }
}
