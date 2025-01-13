mod flight_control;
mod http_handler;

use crate::flight_control::{
    camera_controller::CameraController,
    camera_state::CameraAngle,
    common::orbit::Orbit,
    flight_computer::{
        ChargeCommand::{Duration, TargetCharge},
        FlightComputer,
    },
    task_controller::TaskController,
};
use crate::http_handler::http_client::HTTPClient;
use chrono::TimeDelta;

const FUEL_RESET_THRESHOLD: f32 = 20.0;
const MIN_PX_LEFT_FACTOR: f32 = 0.05;
const ACCEL_FACTOR: f32 = 0.6;
const CHARGE_TIME_THRESHOLD: TimeDelta = TimeDelta::seconds(370);
const DT_0: TimeDelta = TimeDelta::seconds(0);

const STATIC_ORBIT_VEL: (f32, f32) = (6.4f32, 7.4f32);
pub const MIN_BATTERY_THRESHOLD: f32 = 10.0;
pub const MAX_BATTERY_THRESHOLD: f32 = 100.0;
const CONST_ANGLE: CameraAngle = CameraAngle::Narrow;
const BIN_FILEPATH: &str = "camera_controller_narrow.bin";
const ACQUISITION_DISCHARGE_PER_S: f32 = 0.2;
const CHARGE_CHARGE_PER_S: f32 = 0.2;
const JUST_CONVERT: bool = true;

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() {
    if JUST_CONVERT {
        just_convert().await;
        return;
    }
    let client = HTTPClient::new("http://localhost:33000");
    let t_cont = TaskController::new();
    let mut c_cont = CameraController::from_file(BIN_FILEPATH)
        .await
        .unwrap_or_else(|e| {
            println!("[WARN] Failed to read from binary file: {e}");
            CameraController::new()
        });
    let mut f_cont = FlightComputer::new(&client, &t_cont).await;

    f_cont.reset().await;
    f_cont.set_vel_ff(STATIC_ORBIT_VEL.into(), false).await;
    f_cont.set_angle(CONST_ANGLE).await;

    let mut orbit = Orbit::new(&f_cont);
    let period = orbit
        .period()
        .expect("[FATAL] Static orbit is not closed!")
        .round() as i64;
    let img_dt = orbit
        .max_image_dt(CONST_ANGLE)
        .expect("[FATAL] Static orbit is not overlapping enough");
    let mandatory_state_change = orbit.last_possible_state_change().unwrap();
    let mut orbit_s_end =
        orbit.start_timestamp() + chrono::Duration::seconds(i64::from(mandatory_state_change.0));

    // first orbit-behaviour loop
    loop {
        let mut break_flag = false;
        loop {
            c_cont
                .shoot_image_to_buffer(&client, &mut f_cont, CONST_ANGLE)
                .await
                .unwrap();

            if break_flag {
                break;
            }

            let secs_to_min_batt =
                (f_cont.current_battery() - MIN_BATTERY_THRESHOLD) / ACQUISITION_DISCHARGE_PER_S;
            let secs_to_orbit_end = (orbit_s_end - chrono::Utc::now()).num_seconds() as u64;
            let mut sleep_time = img_dt as u64;

            if secs_to_min_batt < img_dt {
                sleep_time = secs_to_min_batt as u64;
                break_flag = true;
            }
            if secs_to_orbit_end < sleep_time {
                sleep_time = secs_to_orbit_end;
                break_flag = true;
            }

            let skipped_time = f_cont
                .make_ff_call(std::time::Duration::from_secs(sleep_time))
                .await;
            orbit_s_end -= TimeDelta::seconds(skipped_time);

            f_cont.update_observation().await;
        }

        if chrono::Utc::now() > orbit_s_end {
            println!(
                "[LOG] First orbit over, position: {:?}",
                f_cont.current_pos()
            );
            break;
        };

        tokio::join!(
            c_cont.export_bin(BIN_FILEPATH),
            f_cont.charge_until(TargetCharge(MAX_BATTERY_THRESHOLD))
        )
        .0
        .unwrap();
    }
}

async fn just_convert() {
    const BIN_PATH: &str = "camera_controller_narrow.bin";
    const PNG_PATH: &str = "world.png";

    let cwd = std::env::current_dir().unwrap_or_else(|_| std::path::PathBuf::from("."));
    let bin_path = cwd.join(BIN_PATH);
    let png_path = cwd.join(PNG_PATH);

    match CameraController::from_file(bin_path.to_str().unwrap_or("FEHLER")).await {
        Ok(camera_controller) => {
            println!("hallo2");

            let recorded_bitmap = camera_controller.bitmap_ref().clone();
            let recorded_buffer = camera_controller.buffer_ref().clone();

            println!("Rein in die Methode");

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
        png_path: std::path::PathBuf,
        bitmap: flight_control::common::bitmap::Bitmap,
        buffer: flight_control::common::img_buffer::Buffer
    ) -> Result<(), Box<dyn std::error::Error>> {
        let world_map_width = 21600;
        let world_map_height = 10800;

        let mut world_map_png = image::RgbImage::new(
            world_map_width as u32, world_map_height as u32
        );

        println!("Created new RGB image!");

        for (i, rgb) in buffer.data.iter().enumerate() {
            let x = (i % world_map_width as usize) as u32;
            let y = (i / world_map_width as usize) as u32;

            // Write the RGB data directly to the image
            world_map_png.put_pixel(x, y, image::Rgb(*rgb));
        }

        println!("Jetz no speichre!");

        world_map_png.save(std::path::Path::new(&png_path))?;
        println!("World map saved to {:?}", png_path);
    Ok(())
}


/* // TODO: legacy code
#[tokio::main]
async fn main() {
    let mut finished = false;
    while !finished {
        let result = std::panic::AssertUnwindSafe(execute_main_loop())
            .catch_unwind()
            .await;
        match result {
            Ok(Ok(..)) => finished = true,
            Ok(Err(e)) => {
                println!("[FATAL] {e}");
            }
            Err(_) => println!("[FATAL] Caught a panic!"),
        };
        println!("[INFO] Restarting!");
        tokio::time::sleep(Duration::from_secs(10)).await;
    }
}

async fn watchdog(timeout: Duration, action: impl Fn() + Send + 'static) {
    // Create a timer for the timeout duration
    time::sleep(timeout).await;
    action();
}

//TODO: this lint exception is only temporary
#[allow(clippy::too_many_lines)]
async fn execute_main_loop() -> Result<(), Box<dyn std::error::Error>> {
    // Spawn the watchdog task
    let timeout = Duration::from_secs(7200);
    //tokio::spawn(watchdog(timeout, || {panic!("[INFO] Resetting normally after 2 hours")}));

    let mut c_cont = CameraController::from_file(BIN_FILEPATH)
        .await
        .unwrap_or_else(|e| {
            println!("[WARN] Failed to read from binary file: {e}");
            CameraController::new()
        });

    let mut t_cont = TaskController::new();

    let http_handler = http_handler::http_client::HTTPClient::new("http://localhost:33000");
    ResetRequest {}.send_request(&http_handler).await?;
    let mut f_cont = FlightComputer::new(&http_handler, &t_cont).await;

    let mut max_orbit_prediction_secs = 60000;

    'outer: while !c_cont.bitmap_ref().data.all() {
        f_cont.update_observation().await;
        f_cont
            .rotate_vel(rand::rng().random_range(-169.0..169.0), ACCEL_FACTOR)
            .await;
        let mut orbit_coverage = Bitmap::from_map_size();
        calculate_orbit_coverage_map(&f_cont, &mut orbit_coverage, max_orbit_prediction_secs);
        orbit_coverage.data &= !c_cont.bitmap_ref().data.clone(); // this checks if there are any possible, unphotographed regions on the current orbit

        println!(
            "[LOG] Total Orbit Possible Coverage Gain: {:.2}",
            orbit_coverage.data.count_ones() as f64 / orbit_coverage.len() as f64
        );

        while orbit_coverage.data.any() {
            let covered_perc = c_cont.bitmap_ref().data.count_ones() as f32
                / c_cont.bitmap_ref().data.len() as f32;
            println!("[INFO] Global Coverage percentage: {:.5}", covered_perc);
            let mut adaptable_tolerance = 5;
            f_cont.update_observation().await;

            // if battery to low go into charge
            if f_cont.current_battery() < MIN_BATTERY_THRESHOLD {
                println!("[INFO] Battery to low, going into charge!");
                f_cont.charge_until(100.0).await;
                println!("[LOG] Charged to: {}", f_cont.current_battery());
            }

            // calculate delay until next image and px threshold
            let mut delay: TimeDelta = TimeDelta::seconds(185 + adaptable_tolerance); // TODO: fix adaptable tolerance
            adaptable_tolerance -= 1;
            let min_pixel = min(
                (CONST_ANGLE.get_square_side_length() as usize - 50).pow(2),
                (orbit_coverage.len() as f32 * MIN_PX_LEFT_FACTOR * covered_perc).round() as usize,
            );
            if f_cont.state() == FlightState::Acquisition {
                delay = TimeDelta::seconds(20);
            }
            match next_image_dt(&f_cont, &orbit_coverage, delay, min_pixel) {
                Some(next_image) => t_cont.schedule_image(next_image),
                None => break,
            }
            // if next image time is more than 6 minutes ahead -> switch to charge
            if t_cont
                .get_time_to_next_image()
                .map_or(true, |dt| dt.ge(&CHARGE_TIME_THRESHOLD))
            {
                // TODO: magic numbers
                f_cont.state_change_ff(FlightState::Charge).await;
            }

            if f_cont.state() == FlightState::Acquisition {
                // TODO: print detailed here to debug this further
                let image_dt = t_cont.get_time_to_next_image().unwrap_or(DT_0);
                let image_dt_std = image_dt.to_std().unwrap_or(Duration::from_secs(0));
                f_cont.make_ff_call(image_dt_std).await;
            } else {
                let sleep_duration = t_cont.get_time_to_next_image().unwrap_or(DT_0)
                    - TimeDelta::seconds(185 - adaptable_tolerance); // tolerance for

                let sleep_duration_std = sleep_duration.to_std().unwrap_or(DT_0.to_std().unwrap());
                adaptable_tolerance -= 1;
                // skip duration until state transition to acquisition needs to be performed
                f_cont.make_ff_call(sleep_duration_std).await;

                // while next image time is more than 3 minutes + tolerance -> wait
                while t_cont
                    .get_time_to_next_image()
                    .unwrap()
                    .ge(&TimeDelta::seconds(185 + adaptable_tolerance))
                {
                    tokio::time::sleep(Duration::from_millis(400)).await;
                }
                // switch to acquisition and wait for exact image taking timestamp
                f_cont.state_change_ff(FlightState::Acquisition).await;
            }
            f_cont.update_observation().await;
            if f_cont.state() != FlightState::Acquisition {
                let dt = t_cont
                    .get_time_to_next_image()
                    .unwrap_or(TimeDelta::seconds(0));
                println!("[WARN] Failed to be ready and in acquisition for image in {dt}");
                continue;
            }
            // skip duration until state transition to acquisition is finished
            while t_cont
                .get_time_to_next_image()
                .unwrap_or(DT_0)
                .ge(&TimeDelta::milliseconds(100))
            {
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
            println!("[INFO] Shooting image!");
            f_cont.set_angle(CONST_ANGLE).await;
            c_cont
                .shoot_image_to_buffer(&http_handler, &mut f_cont, CONST_ANGLE)
                .await?;
            c_cont.export_bin(BIN_FILEPATH).await?;
            orbit_coverage.set_region(
                Vec2D::new(f_cont.current_pos().x(), f_cont.current_pos().y()),
                CONST_ANGLE,
                false,
            );
            t_cont.remove_next_image();
        }

        // if there is not enough fuel left -> reset
        if f_cont.fuel_left() < FUEL_RESET_THRESHOLD {
            ResetRequest {}.send_request(&http_handler).await?;
            max_orbit_prediction_secs += 10000;
            println!("[WARN] No Fuel left: Resetting!");
            continue 'outer;
        }
        println!("[INFO] Performing orbit change and starting over!");
        f_cont.rotate_vel(40.0, ACCEL_FACTOR).await;
    }
    Ok(())
}

fn next_image_dt(
    cont: &FlightComputer,
    map: &Bitmap,
    base_delay: TimeDelta,
    min_px: usize,
) -> Option<ImageTask> {
    let init_pos = cont.pos_in_dt(base_delay);
    let mut time_del = PinnedTimeDelay::new(base_delay);
    let mut current_calculation_time_delta = base_delay;
    let mut current_pos = init_pos;

    while current_calculation_time_delta < TimeDelta::seconds(20000) {
        if map.has_sufficient_set_bits(current_pos, CONST_ANGLE, min_px)
            && time_del.get_start() + current_calculation_time_delta > Utc::now() + base_delay
        {
            time_del.set_delay(current_calculation_time_delta);
            let time_left = time_del.time_left();
            println!(
                "[INFO] Next Image in {:02}:{:02}:{:02}",
                time_left.num_hours(),
                time_left.num_minutes() - time_left.num_hours() * 60,
                time_left.num_seconds() - time_left.num_minutes() * 60
            );
            let planned_pos = Vec2D::new(
                current_pos.x().round() as u32,
                current_pos.y().round() as u32,
            );
            return Some(ImageTask::new(time_del, planned_pos, CONST_ANGLE));
        }
        current_calculation_time_delta += TimeDelta::seconds(1);
        current_pos = cont.pos_in_dt(current_calculation_time_delta);
        current_pos.wrap_around_map();
    }
    println!("[WARN] No possible image time found!");
    None
}

fn calculate_orbit_coverage_map(cont: &FlightComputer, map: &mut Bitmap, max_dt: i64) {
    println!("[INFO] Calculating Orbit Coverage!");
    let mut dt = 0; // TODO: should be higher than 0 to account for spent time during calculation
    loop {
        let mut next_pos = cont.pos_in_dt(TimeDelta::seconds(dt));
        next_pos.wrap_around_map();
        if dt > max_dt {
            println!(
                "[LOG] Coverage Percentage of current Orbit: {}",
                map.data.count_ones() as f64 / map.len() as f64
            );
            break;
        }
        map.set_region(
            Vec2D::new(next_pos.x() as f32, next_pos.y() as f32),
            CONST_ANGLE,
            true,
        );
        dt += 1;
    }
}
*/
