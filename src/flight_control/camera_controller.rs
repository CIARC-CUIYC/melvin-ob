use crate::http_handler::http_client::HTTPClient;
use crate::http_handler::http_request::request_common::NoBodyHTTPRequestType;
use crate::http_handler::http_request::shoot_image_get::ShootImageRequest;
use bit_vec::BitVec;
use futures::StreamExt;
use tokio::fs::File;
use tokio::io::AsyncWriteExt;

pub struct Bitmap {
    width: usize,
    height: usize,
    data: BitVec,
}

impl Bitmap {
    pub fn new(width: usize, height: usize) -> Self {
        let size = width * height;
        Bitmap {
            width,
            height,
            data: BitVec::from_elem(size, false),
        }
    }

    //
    pub async fn shoot_image(
        &mut self,
        http_client: &HTTPClient,
        file_path: &str,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let mut file = File::create(file_path).await?;
        let mut response_stream = ShootImageRequest {}.send_request(http_client).await?;
        while let Some(Ok(chunk)) = response_stream.next().await {
            file.write_all(&chunk).await?;
        }

        file.flush().await?;

        //TODO: Coordinate parsing?
        // self._region(x, y);

        Ok(())
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
        Ok(self.check_pixel(x, y))
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
