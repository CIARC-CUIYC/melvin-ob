use bit_vec::BitVec;

pub struct Bitmap {
    width: usize,
    height: usize,
    data: BitVec,
}

impl Bitmap {
    pub fn new(width: usize, height: usize) -> Self {
        let size = width * height;
        Bitmap{
            width,
            height,
            data: BitVec::from_elem(size, false),
        }
    }

    pub fn flip_region(&mut self, x: usize, y: usize) {
        // TODO: approaching edge
        for row in y..y + 600 {
            // TODO: check how the API returns position of the satellite
            for col in x..x + 600 {
                self.flip_pixel(col, row);
            }
        }
    }

    fn is_photographed(&self, x: usize, y: usize) -> Result<bool, String> {
        if x >= self.width || y >= self.height {
            // TODO: approaching edge
            return Err("EDGE".to_string());
        }
        Ok(self.get_pixel(x, y,))
    }

    fn check_pixel(&self, x: usize, y: usize) -> bool {
        let index = self.get_bitmap_index(x, y);
        self.data.get(index).unwrap_or(false)
    }

    fn flip_pixel(&mut self, x: usize, y: usize) {
        let index = self.get_bitmap_index(x, y);
        self.data.set(index, true);
    }

    // Converts 2D (x, y) coordinates to a 1D index of memory
    fn get_bitmap_index(&self, x: usize, y: usize) -> usize {
        y * self.width + x
    }
}