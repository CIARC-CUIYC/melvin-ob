use crate::flight_control::common::Vec2D;

pub struct PixelData {
    rgb: [u8; 3],
    // NOTE: possible Metadata field could be useful here
}

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

    pub fn save_pixel(&mut self, pos: Vec2D<usize>, rgb: [u8; 3]) {
        let map_size = Vec2D::<usize>::map_size();
        if pos.x() >= map_size.x() || pos.y() >= map_size.y {
            // TODO: implement border switch
        }
        let index = pos.y * map_size.x + pos.x();
        self.data[index] = rgb;
    }
}
