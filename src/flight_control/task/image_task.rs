use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::common::vec2d::Vec2D;

/// Represents the status of an image capture task.
#[derive(Debug, Copy, Clone)]
enum ImageTaskStatus {
    /// The task is planned but has not yet been completed.
    Planned,
    /// The task has been completed, including metadata for the actual capture.
    Done {
        /// The actual position where the capture occurred.
        actual_pos: Vec2D<u32>,
        /// The relative number of pixels which deviate from the planned picture.
        px_dev_rel: f64,
    },
}

/// Represents a specific image capture task, including timing, planning,
/// and lens configuration.
#[derive(Debug, Copy, Clone)]
pub struct ImageTask {
    /// The current status of the task (e.g., `Planned` or `Done`).
    image_status: ImageTaskStatus,
    /// The target position for the image capture.
    planned_pos: Vec2D<u32>,
    /// The lens configuration for the capture.
    lens: CameraAngle,
}

impl ImageTask {
    /// Creates a new instance of an `ImageTask`.
    ///
    /// # Arguments
    /// - `dt`: The planned time delay before the task execution.
    /// - `planned_pos`: The target position for the image capture.
    /// - `lens`: The lens configuration for the capture.
    ///
    /// # Returns
    /// - A new `ImageTask` instance with the given parameters.
    pub fn new(planned_pos: Vec2D<u32>, lens: CameraAngle) -> Self {
        Self {
            image_status: ImageTaskStatus::Planned,
            planned_pos,
            lens,
        }
    }

    /// Marks the task as completed and records the actual capture position.
    ///
    /// # Arguments
    /// - `actual_pos`: The position where the image was actually captured.
    ///
    /// # Side Effects
    /// - Updates the task status to `Done`, including deviation from
    ///   the planned position.
    pub fn done(&mut self, actual_pos: Vec2D<u32>) {
        let square_side = f64::from(self.lens.get_square_side_length());
        let center_dev_x = (f64::from(self.planned_pos.x()) - f64::from(actual_pos.x())).abs();
        let center_dev_y = (f64::from(self.planned_pos.y()) - f64::from(actual_pos.y())).abs();
        let px_dev = square_side * center_dev_x + (square_side - center_dev_x) * center_dev_y;
        let px_dev_rel = px_dev / (square_side * square_side);
        let new_status = ImageTaskStatus::Done {
            actual_pos,
            px_dev_rel,
        };
        self.image_status = new_status;
        // TODO: maybe we could also perform a check here which redundant pixels where photographed and which pixels were "lost", we would need to pass a camera_controller reference for that maybe?
    }
}
