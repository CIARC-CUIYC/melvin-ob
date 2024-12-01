use crate::flight_control::common::Vec2D;

pub struct PixelData {
    rgb: [u8; 3],
    // NOTE: possible Metadata field could be useful here
}

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct Buffer {
    data: Vec<[u8; 3]>,
}

impl Buffer {
    pub fn new() -> Self {
        let map_size = Vec2D::<usize>::map_size();
        Self {
            data: vec![[0, 0, 0]; map_size.x() * map_size.y()],
        }
    }

    pub fn save_pixel(&mut self, wrapped_pos: Vec2D<f32>, rgb: [u8; 3]) {
        let index = wrapped_pos.y() * Vec2D::<f32>::map_size().x() + wrapped_pos.x();
        self.data[index as usize] = rgb;
    }

    pub fn get_at(&self, index: usize) -> Option<&[u8; 3]> {
        self.data.get(index)
    }
}
