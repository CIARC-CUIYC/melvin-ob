use crate::flight_control::common::matrix::Matrix;
use crate::flight_control::common::state_vector::StateVector;
use crate::flight_control::common::vec2d::Vec2D;
use crate::flight_control::kalman_filter::base_kalman::BaseKalman;
use fixed::types::I32F32;

// State vector state_vec: [x, y, v_x, v_y]
// Measurements z: [x, y]
pub type PosDeviationKalman = BaseKalman<4, 2>;

impl PosDeviationKalman {
    pub fn new(v_x: I32F32, v_y: I32F32) -> Self {
        BaseKalman::<4, 2> {
            state_vec: StateVector::from_array([
                I32F32::from_num(0),
                I32F32::from_num(0),
                v_x,
                v_y,
            ]),
            cov_mat: Matrix::identity(),
            obs_matrix: Matrix::eye(),
            meas_noise_cov_mat: Matrix::identity(),
            state_trans_mat: Matrix::identity(),
            process_noise_cov_mat: Matrix::identity(),
        }
    }

    pub fn predict_pos_deviation(&self, steps: usize) -> Vec2D<I32F32> {
        let dx = self.state_vec[0];
        let dy = self.state_vec[1];

        let predicted_future_deviation_x = dx * I32F32::from_num(steps) - I32F32::from_num(dx);
        let predicted_future_deviation_y = dy * I32F32::from_num(steps) - I32F32::from_num(dy);

        Vec2D::from((predicted_future_deviation_x, predicted_future_deviation_y))
    }

    pub fn log_deviation(&mut self, current_pos: Vec2D<I32F32>) {
        self.predict();
        self.update(StateVector::from_vec2d(current_pos));

        let est_deviation = self.state_vec.get_slice(0, 2);
        let est_vel = self.state_vec.get_slice(2, 2);

        println!(
            "Deviation: [{:.2}, {:.2}], Velocity: [{:.2}, {:.2}]",
            est_deviation[0], est_deviation[1], est_vel[0], est_vel[1]
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use csv::{Reader, Writer};
    use std::collections::VecDeque;
    use std::fs::{create_dir_all, File};
    use std::path::Path;

    const X_VEL: I32F32 = I32F32::lit("6.4");
    const Y_VEL: I32F32 = I32F32::lit("7.4");
    const PREDICTION_INTERVAL: usize = 250;

    #[test]
    #[allow(clippy::too_many_lines)]
    fn csv_based_test_kalman() {
        let asset_path = Path::new("assets");
        let input_path = Path::new("assets/pos.csv");
        let output_file = asset_path.join("kalman_test.csv");

        create_dir_all(asset_path).expect("Failed to create output directory");

        let file = File::open(input_path).expect("Failed to open CSV file");
        let mut reader = Reader::from_reader(file);
        let mut writer = Writer::from_path(output_file).expect("Failed to create CSV writer");

        // Write headers
        writer
            .write_record([
                "step",
                "estimated_average_deviation_x",
                "estimated_average_deviation_y",
                "diff_act_exp_pos",
                "diff_est_exp_pos",
            ])
            .expect("Failed to write header row");

        let mut kalman = PosDeviationKalman::new(X_VEL, Y_VEL);
        let mut prediction_queue: VecDeque<(usize, Vec2D<I32F32>)> = VecDeque::new();

        for (step, result) in reader.records().enumerate() {
            let record = result.expect("Failed to read CSV row!");

            let actual_x: I32F32 = record[0].parse().unwrap();
            let actual_y: I32F32 = record[1].parse().unwrap();
            let actual_pos = Vec2D::from((actual_x, actual_y));

            let expected_x: I32F32 = record[2].parse().unwrap();
            let expected_y: I32F32 = record[3].parse().unwrap();
            let expected_pos = Vec2D::from((expected_x, expected_y));

            let dev_act_exp = actual_pos - expected_pos;

            let measurements = StateVector::<I32F32, 2>::from_vec2d(dev_act_exp);

            kalman.predict();
            kalman.update(measurements);

            let est_average_deviation = Vec2D::from((kalman.state_vec[0], kalman.state_vec[1]));
            let diff_act_exp_pos = actual_pos - expected_pos;

            let diff_est_exp_pos = est_average_deviation - expected_pos;

            if step % PREDICTION_INTERVAL == 0 {
                let kalman_predicted_deviation = kalman.predict_pos_deviation(PREDICTION_INTERVAL);

                let future_step = step + PREDICTION_INTERVAL;

                let actual_dev = actual_pos - expected_pos;
                let dev_diff = kalman_predicted_deviation - actual_dev;

                let abs_err_pos_diff_x = (dev_diff.x / actual_dev.x) * 100;
                let abs_err_pos_diff_y = (dev_diff.y / actual_dev.y) * 100;

                let err_percentage = Vec2D::from((abs_err_pos_diff_x, abs_err_pos_diff_y));

                println!("**************************************************************");
                println!("[INFO] Predictions made by Kalman filter for step {future_step}");
                println!("[INFO] Predictions made using data from step {step}");
                println!("[DEBUG] Predicted Deviation ({kalman_predicted_deviation})");
                println!("[DEBUG] Actual Deviation ({actual_dev})");
                // println!("[DEBUG] Positional difference: Predicted - Expected: {pos_diff}");
                println!("[DEBUG] Deviation difference: Predicted - Actual: {dev_diff}");
                println!("**************************************************************");

                prediction_queue
                    .push_back((step + PREDICTION_INTERVAL, kalman_predicted_deviation));
            }

            if let Some((predicted_step, predicted_pos)) = prediction_queue.front() {
                if *predicted_step == step {
                    let act_pre_x = actual_x - predicted_pos.x;
                    let act_pre_y = actual_y - predicted_pos.y;
                    let exp_pre_x = expected_x - predicted_pos.x;
                    let exp_pre_y = expected_y - predicted_pos.y;
                    // diff_act_est_x = actual_x - estimated_x;
                    // diff_act_est_y = actual_y - estimated_y;
                    // println!("[INFO]: Comparison for Sim Step {step}");
                    // println!("[DEBUG]: Difference Actual-Estimated_Prediction: ({act_pre_x},{act_pre_y})");
                    // println!("[DEBUG]: Difference Predicated-Estimated: ({exp_pre_x},{exp_pre_y})");

                    prediction_queue.pop_front();
                }
            }

            writer
                .write_record([
                    step.to_string(),
                    est_average_deviation.x.to_string(),
                    est_average_deviation.y.to_string(),
                    diff_act_exp_pos.to_string(),
                    diff_est_exp_pos.to_string(),
                ])
                .expect("Failed to write record");

            //step += 1;
        }

        writer.flush().expect("Failed to flush CSV writer");
    }
}
