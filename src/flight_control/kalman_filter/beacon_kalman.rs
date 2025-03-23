use crate::flight_control::common::beacon::BeaconMeasurement;
use crate::flight_control::common::matrix::Matrix;
use crate::flight_control::common::state_vector::StateVector;
use crate::flight_control::common::vec2d::Vec2D;
use crate::flight_control::kalman_filter::base_kalman::BaseKalman;
use fixed::types::I32F32;

/// `BeaconKalman` is a Kalman filter specifically designed to track a **stationary beacon** in 2D space.
///
/// # State Vector (`state_vec`)
/// - `[pos_x, pos_y]`: Represents the beacon's position in the 2D plane.
///
/// # Measurement Vector (`z`)
/// - `[d_noisy]`: Represents the observed (noisy) distance measurement to the beacon.
pub type BeaconKalman = BaseKalman<2, 1>;

impl BeaconKalman {
    /// Initial uncertainty in the beacon's position estimate.
    /// Set to a high value since the initial position is highly uncertain.
    const INITIAL_UNCERTAINTY: f32 = 1000.0;

    /// Initial measurement noise variance.
    /// Represents the uncertainty in the sensor's distance measurements.
    const INITIAL_MEASUREMENT_NOISE: f32 = 400.0;

    /// Creates a new `BeaconKalman` instance with an initial position.
    ///
    /// # Parameters
    /// - `current_pos`: The initial estimated position of the beacon.
    ///
    /// # Returns
    /// - A `BeaconKalman` instance initialized with the given position.
    pub fn new(current_pos: Vec2D<I32F32>) -> Self {
        let kalman = BaseKalman::<2, 1> {
            // State Vector (x): Represents the beacon’s position.
            // Initialized from the first known position.
            state_vec: StateVector::from_array([current_pos.x, current_pos.y]),

            // Covariance Matrix (P): Represents uncertainty in the state estimate.
            // Initialized with high uncertainty since the beacon's exact position is initially unknown.
            cov_mat: Matrix::identity() * I32F32::from_num(Self::INITIAL_UNCERTAINTY), // TODO: Tune for better performance

            // Observation Matrix (H): Maps the beacon's position (x, y) into a distance measurement d.
            // Since distance is a function of both x and y, we use `[1, 1]` as a placeholder (needs proper adjustment).
            obs_matrix: Matrix::new([[I32F32::from_num(1), I32F32::from_num(1)]]),

            // Measurement Noise Covariance Matrix (R): Represents sensor measurement noise.
            // Initialized with an estimated noise level based on expected sensor performance.
            meas_noise_cov_mat: Matrix::identity()
                * I32F32::from_num(Self::INITIAL_MEASUREMENT_NOISE), // TODO: Tune this parameter

            // State Transition Matrix (F): Defines how the state evolves over time.
            // Since the beacon is stationary, we use an identity matrix (state remains constant).
            state_trans_mat: Matrix::identity(),

            // Process Noise Covariance Matrix (Q): Represents model uncertainty over time.
            // Since the beacon is stationary, this is set to an identity matrix (no assumed change in state).
            process_noise_cov_mat: Matrix::identity(),
        };

        kalman
    }

    /// Use triangulation (2 pings) for a better initial estimate
    pub fn new_with_triangulation(&mut self, meas_1: BeaconMeasurement, meas_2: BeaconMeasurement) {
        let estimated_beacon_pos =
            self.triangulate_beacon_position(meas_1, meas_2).unwrap_or(meas_1.pos);

        self.state_vec = StateVector::from_array([estimated_beacon_pos.x, estimated_beacon_pos.y]);
    }

    /// Triangulate the beacon position using two beacon pings
    pub fn triangulate_beacon_position(
        &self,
        meas1: BeaconMeasurement,
        meas2: BeaconMeasurement,
    ) -> Option<Vec2D<I32F32>> {
        let d_squared = ((meas2.pos - meas1.pos).x) * ((meas2.pos - meas1.pos).x)
            + ((meas2.pos - meas1.pos).y) * ((meas2.pos - meas1.pos).y);
        if d_squared == I32F32::from_num(0) {
            return None;
        }

        let a = ((meas1.distance * meas1.distance - meas2.distance * meas2.distance) + d_squared)
            / (I32F32::from_num(2) * d_squared.sqrt());
        let h_squared = meas1.distance * meas1.distance - a * a;
        if h_squared < I32F32::from_num(0) {
            return None;
        }

        let h = h_squared.sqrt();
        let xm = meas1.pos.x + a * ((meas2.pos - meas1.pos).x);
        let ym = meas1.pos.y + a * ((meas2.pos - meas1.pos).y);
        let x_offset = h * ((meas2.pos - meas1.pos).y) / d_squared.sqrt();
        let y_offset = h * ((meas2.pos - meas1.pos).x) / d_squared.sqrt();

        Some(Vec2D::from((xm + x_offset, ym - y_offset)))
    }

    /// Refine beacon position with more than 2
    pub fn refine_beacon_position(
        &self,
        measurements: &[(Vec2D<I32F32>, I32F32)], // (MELVIN Position, Distance)
    ) -> Option<Vec2D<I32F32>> {
        if measurements.len() < 2 {
            return None;
        }

        let mut beacon_estimates = Vec::new();

        for i in 0..measurements.len() - 1 {
            let (pos1, d1) = measurements[i];
            let (pos2, d2) = measurements[i + 1];

            if let Some(estimate) = self.triangulate_beacon_position(pos1, d1, pos2, d2) {
                beacon_estimates.push(estimate);
            }
        }

        // Compute an average of the estimates
        let avg_x = beacon_estimates.iter().map(|b| b.x).sum::<I32F32>()
            / I32F32::from_num(beacon_estimates.len());
        let avg_y = beacon_estimates.iter().map(|b| b.y).sum::<I32F32>()
            / I32F32::from_num(beacon_estimates.len());

        Some(Vec2D::from((avg_x, avg_y)))
    }

    /// **Update the beacon's position with a new measurement using weighted averaging**
    pub fn update_position(&mut self, new_measurement: Vec2D<I32F32>, alpha: I32F32) {
        if self.reject_outliers(new_measurement, I32F32::from_num(200.0)) {
            self.state_vec[0] =
                alpha * self.state_vec[0] + (I32F32::from_num(1) - alpha) * new_measurement.x;
            self.state_vec[1] =
                alpha * self.state_vec[1] + (I32F32::from_num(1) - alpha) * new_measurement.y;
        }
    }

    pub fn reject_outliers(&self, new_measurement: Vec2D<I32F32>, threshold: I32F32) -> bool {
        let dx = (new_measurement.x - self.state_vec[0]).abs();
        let dy = (new_measurement.y - self.state_vec[1]).abs();

        if dx > threshold || dy > threshold {
            println!("Rejected outlier: X: {:.2}, Y: {:.2}", dx, dy);
            return false;
        }
        true
    }

    pub fn update_measurement_noise(&mut self, d_noisy: I32F32) {
        // Adaptive R based on distance
        let noise_level =
            I32F32::from_num(225) + (I32F32::from_num(0.1) * (d_noisy + I32F32::from_num(1)));
        self.meas_noise_cov_mat = Matrix::identity() * noise_level * noise_level;
    }

    pub fn display_uncertainty(&self) {
        let uncertainty_x = self.cov_mat.get(0, 0).sqrt();
        let uncertainty_y = self.cov_mat.get(1, 1).sqrt();
        println!(
            "Position Uncertainty: X: {:.2}, Y: {:.2}",
            uncertainty_x, uncertainty_y
        );
    }

    pub fn log_beacon_state(&self) {
        println!(
            "Beacon Position Estimate: X: {:.2}, Y: {:.2}",
            self.state_vec[0], self.state_vec[1]
        );
        self.display_uncertainty();
    }

    pub fn new_with_dynamic_p(
        &self,
        melvin_pos_1: Vec2D<I32F32>,
        d1: I32F32,
        melvin_pos_2: Vec2D<I32F32>,
        d2: I32F32,
    ) -> Self {
        let estimated_beacon_pos = self
            .triangulate_beacon_position(melvin_pos_1, d1, melvin_pos_2, d2)
            .unwrap_or(melvin_pos_1);

        let initial_uncertainty = Self::compute_initial_uncertainty(d1, d2);

        BaseKalman::<2, 1> {
            state_vec: StateVector::from_array([estimated_beacon_pos.x, estimated_beacon_pos.y]),
            cov_mat: Matrix::identity() * initial_uncertainty,
            obs_matrix: Matrix::new([[I32F32::from_num(1), I32F32::from_num(1)]]),
            meas_noise_cov_mat: Matrix::identity() * I32F32::from_num(400),
            state_trans_mat: Matrix::identity(),
            process_noise_cov_mat: Matrix::identity(),
        }
    }

    /// Compute initial uncertainty based on the first two pings
    pub fn compute_initial_uncertainty(d1: I32F32, d2: I32F32) -> I32F32 {
        let avg_distance = (d1 + d2) / I32F32::from_num(2);
        let uncertainty = avg_distance * I32F32::from_num(0.1);
        uncertainty.max(I32F32::from_num(500))
    }

    /// Adapt uncertainty if new measurement deviates significantly
    pub fn adapt_uncertainty(&mut self, new_measurement: Vec2D<I32F32>) {
        let dx = (new_measurement.x - self.state_vec[0]).abs();
        let dy = (new_measurement.y - self.state_vec[1]).abs();
        let deviation = (dx * dx + dy * dy).sqrt();

        if deviation > I32F32::from_num(300) {
            self.cov_mat = self.cov_mat * I32F32::from_num(1.5);
        }
    }

    /// Gradually reduce uncertainty instead of an instant drop
    pub fn smooth_uncertainty_reduction(&mut self) {
        self.cov_mat = self.cov_mat * I32F32::from_num(0.9);
    }

    pub fn update_h(&mut self, melvin_pos: Vec2D<I32F32>) {
        let beacon_x = self.state_vec[0];
        let beacon_y = self.state_vec[1];

        let dx = beacon_x - melvin_pos.x;
        let dy = beacon_y - melvin_pos.y;
        let distance = (dx * dx + dy * dy).sqrt();

        if distance < I32F32::from_num(1) {
            self.obs_matrix = Matrix::new([[I32F32::from_num(1), I32F32::from_num(1)]]);
            return;
        }

        self.obs_matrix = Matrix::new([[dx / distance, dy / distance]]);
    }

    /// **Process a new measurement and update the beacon position**
    pub fn process_measurement(&mut self, melvin_pos: Vec2D<I32F32>, d_noisy: I32F32) {
        self.update_h(melvin_pos);

        let predicted_distance = (self.obs_matrix * self.state_vec.to_matrix()).get(0, 0);
        let measurement_residual = d_noisy - predicted_distance;

        let s =
            self.obs_matrix * self.cov_mat * self.obs_matrix.transpose() + self.meas_noise_cov_mat;
        let k = self.cov_mat * self.obs_matrix.transpose() * s.try_inverse().unwrap();
        self.state_vec = self.state_vec
            + StateVector::from_matrix(
                k * StateVector::from_array([measurement_residual]).to_matrix(),
            );
        self.cov_mat = self.cov_mat - k * self.obs_matrix * self.cov_mat;
    }

    pub fn compute_measurement_noise(d: I32F32) -> I32F32 {
        let noise = I32F32::from_num(225) + I32F32::from_num(0.1) * (d + I32F32::from_num(1));
        noise * noise // Store variance (not standard deviation)
    }

    /// **Update measurement noise covariance \( R \) based on new distance**
    pub fn update_r(&mut self, d_noisy: I32F32) {
        let new_noise = Self::compute_measurement_noise(d_noisy);
        let alpha = I32F32::from_num(0.8);
        self.meas_noise_cov_mat = Matrix::identity()
            * (alpha * self.meas_noise_cov_mat.get(0, 0)
                + (I32F32::from_num(1) - alpha) * new_noise);
    }

    /// **Adapt \( R \) based on sudden changes in measurement noise**
    pub fn adapt_r_based_on_deviation(&mut self, d_noisy: I32F32, previous_d: I32F32) {
        let deviation = (d_noisy - previous_d).abs();

        if deviation > I32F32::from_num(500) {
            self.meas_noise_cov_mat = self.meas_noise_cov_mat * I32F32::from_num(1.5);
        } else if deviation < I32F32::from_num(50) {
            self.meas_noise_cov_mat = self.meas_noise_cov_mat * I32F32::from_num(0.95);
        }
    }
}
