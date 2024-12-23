use super::vec2d::Vec2D;

pub struct PixelData {
    rgb: [u8; 3],
    // NOTE: possible Metadata field could be useful here
}

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct Buffer {
    data: Vec<[u8; 3]>,
    width: u32,
    height: u32,
}

impl Buffer {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            data: vec![[0, 0, 0]; width as usize * height as usize],
            width,
            height,
        }
    }

    pub fn from_mapsize() -> Self {
        let map_size = Vec2D::<u32>::map_size();
        Self::new(map_size.x(), map_size.y())
    }

    pub fn save_pixel(&mut self, wrapped_pos: Vec2D<isize>, rgb: [u8; 3]) {
        let index = self.get_buffer_index(wrapped_pos.x() as usize, wrapped_pos.y() as usize);
        self.data[index] = rgb;
    }

    pub fn get_buffer_index(&self, x: usize, y: usize) -> usize {
        y * self.width as usize + x
    }
}
