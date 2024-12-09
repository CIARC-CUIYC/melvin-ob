use crate::flight_control::common::Vec2D;
use crate::flight_control::image_data::Buffer;
use bitvec::prelude::Lsb0;
use bitvec::slice::Iter;
use bitvec::{bitbox, prelude::BitBox};
use futures::StreamExt;
use image::{imageops::Lanczos3, ImageReader};
use tokio::fs::File;
use tokio::io::{AsyncReadExt, AsyncWriteExt};

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct Bitmap {
    width: u32,
    height: u32,
    pub data: BitBox,
}
#[derive(serde::Serialize, serde::Deserialize)]
pub struct CameraController {
    pub bitmap: Bitmap,
    pub buffer: Buffer,
}

impl Bitmap {
    pub fn new(width: u32, height: u32) -> Self {
        let len = width * height;
        Self {
            width,
            height,
            data: bitbox![usize, Lsb0; 0; len as usize],
        }
    }

    pub fn iter(&self) -> bitvec::slice::Iter<usize, Lsb0> {
        self.data.iter()
    }

    pub fn from_mapsize() -> Self {
        let bitmap_size = Vec2D::<u32>::map_size();
        Self::new(bitmap_size.x(), bitmap_size.y())
    }
}

impl CameraController {
    pub fn new() -> Self {
        let png_bitmap = Bitmap::from_mapsize();
        let png_buffer = Buffer::new();
        Self {
            bitmap: png_bitmap,
            buffer: png_buffer,
        }
    }

    pub async fn from_file(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let mut file = File::open(path).await?;
        let mut buffer = Vec::new();
        file.read_to_end(&mut buffer).await?;

        let (bitmap, buffer_data): (Bitmap, Buffer) = bincode::deserialize(&buffer)?;

        Ok(CameraController {
            bitmap,
            buffer: buffer_data,
        })
    }

    pub async fn export_bin(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        let data_to_serialize = (self.bitmap.clone(), self.buffer.clone());
        let encoded = bincode::serialize(&data_to_serialize)?;
        let mut bin_file = File::create(path).await?;
        bin_file.write_all(&encoded).await?;
        bin_file.flush().await?;
        Ok(())
    }
}
