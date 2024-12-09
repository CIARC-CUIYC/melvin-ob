use image::{Rgb, RgbImage};
use std::env;
use std::path::{Path, PathBuf};

mod flight_control;

use crate::flight_control::camera_controller::CameraController;
use crate::flight_control::image_data::Buffer;

#[tokio::main]
async fn main() {
    const BIN_PATH: &str = "src/camera_controller.bin";
    const PNG_PATH: &str = "src/graphics/world.png";

    let cwd = env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    let bin_path = cwd.join(BIN_PATH);
    let png_path = cwd.join(PNG_PATH);

    match CameraController::from_file(bin_path.to_str().unwrap_or("FEHLER")).await {
        Ok(camera_controller) => {
            let recorded_buffer = camera_controller.buffer;

            bin_to_png(&png_path, &recorded_buffer).unwrap();
        }
        Err(e) => {
            eprintln!(
                "Failed to load camera controller from {}: {}",
                bin_path.display(),
                e
            );
        }
    }
}

fn bin_to_png(
    png_path: PathBuf,
    bitmap: Bitmap,
    buffer: Buffer,
) -> Result<(), Box<dyn std::error::Error>> {
    let world_map_width = 21600;
    let world_map_height = 10800;

    let mut world_map_png = RgbImage::new(world_map_width as u32, world_map_height as u32);

    println!("Created new RGB image!");

    for (i, rgb) in buffer.data.iter().enumerate() {
        let x = (i % world_map_width as usize) as u32;
        let y = (i / world_map_width as usize) as u32;

        // Write the RGB data directly to the image
        world_map_png.put_pixel(x, y, Rgb(*rgb));
    }

    println!("Jetz no speichre!");

    world_map_png.save(Path::new(&png_path))?;
    println!("World map saved to {:?}", png_path);

    Ok(())
}
