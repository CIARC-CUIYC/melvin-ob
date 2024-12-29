use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::common::pinned_dt::PinnedTimeDelay;
use crate::flight_control::common::vec2d::Vec2D;

#[derive(Debug, Copy, Clone)]
enum ImageTaskStatus {
    Planned,
    Done {
        actual_pos: Vec2D<u32>,
        px_dev_rel: f64,
    },
}

#[derive(Debug, Copy, Clone)]
pub struct ImageTask {
    dt: PinnedTimeDelay,
    image_status: ImageTaskStatus,
    planned_pos: Vec2D<u32>,
    lens: CameraAngle
}

impl ImageTask {
    pub fn new(dt: PinnedTimeDelay, planned_pos: Vec2D<u32>, lens: CameraAngle) -> Self { 
        Self {
            image_status: ImageTaskStatus::Planned,
            planned_pos,
            dt,
            lens
        } 
    }
    
    pub fn dt_mut(&mut self) -> &mut PinnedTimeDelay { &mut self.dt }
    
    pub fn dt(&self) -> &PinnedTimeDelay { &self.dt }
    
    pub fn done(&mut self, actual_pos: Vec2D<u32>) {
        let square_side = self.lens.get_square_side_length() as f64;
        let center_dev_x = (self.planned_pos.x() as f64 - actual_pos.x() as f64).abs();
        let center_dev_y = (self.planned_pos.y() as f64 - actual_pos.y() as f64).abs();
        let px_dev = square_side * center_dev_x + (square_side - center_dev_x) * center_dev_y;
        let px_dev_rel = px_dev / (square_side*square_side);
        let new_status = ImageTaskStatus::Done {
            actual_pos,
            px_dev_rel,
        };
        self.image_status = new_status;
        // TODO: maybe we could also perform a check here which redundant pixels where photographed and which pixels were "lost", we would need to pass a camera_controller reference for that maybe?
    }
}
