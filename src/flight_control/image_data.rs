use std::collections::HashMap;

pub struct PixelData {
    rgb: [u8; 3],
    // NOTE: possible Metadata field could be useful here
}

pub struct Buffer {
    storage: HashMap<(usize, usize), PixelData>,
}

impl Buffer {
    pub fn new() -> Self {
        Self {
            storage: HashMap::new(),
        }
    }

    pub fn save_pixel(&mut self, x: usize, y: usize, rgb: [u8; 3]) {
        let coordinates = (x, y);

        self.storage.insert(coordinates, PixelData { rgb });
    }
}
