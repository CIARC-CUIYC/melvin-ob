use std::env;
use image::{Rgb, RgbImage};
use std::path::{Path, PathBuf};

mod flight_control;

use crate::flight_control::camera_controller::{Bitmap, CameraController};
use crate::flight_control::image_data::{Buffer};




#[tokio::main]
async fn main(){
    const BIN_PATH: &str = "src/camera_controller.bin";
    const PNG_PATH: &str = "../graphics/world.png";

    let cwd = env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    let bin_path = cwd.join(BIN_PATH);

    // Use CARGO_MANIFEST_DIR to construct an absolute path
    let project_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    //let bin_path = project_dir.join("src/camera_controller.bin");
    let png_path = project_dir.join("src/graphics/world.png");

    match CameraController::from_file(bin_path.to_str().unwrap_or("FEHLER")).await {
        Ok(camera_controller) => {
            println!("hallo2");
            let camera_controller = CameraController::from_file(BIN_PATH).await.unwrap();

            let recorded_bitmap = camera_controller.bitmap;
            let recorded_buffer = camera_controller.buffer;

            bin_to_png(png_path, recorded_bitmap, recorded_buffer).unwrap();
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
    buffer: Buffer
) -> Result<(), Box<dyn std::error::Error>> {
    let world_map_width = 21600;
    let world_map_height = 10800;

    let mut world_map_png = RgbImage::new(
        world_map_width as u32, world_map_height as u32
    );

    println!("Created new RGB image!");

    let default_color = Rgb([100, 100, 100]);

    for (i, bit) in bitmap.iter().enumerate() {
        let x = (i % world_map_width as usize) as u32;
        let y = (i / world_map_width as usize) as u32;

        let color = if *bit {
            Rgb(*buffer.get_at(i).unwrap())
        } else {
            default_color
        };

        world_map_png.put_pixel(x, y, color);
    }

    world_map_png.save(Path::new(&png_path))?;
    println!("World map saved to {:?}", png_path);

    Ok(())
}
