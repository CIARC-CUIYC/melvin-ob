use bitvec::macros::internal::funty::Fundamental;
use std::cmp::min;
use std::panic;
use std::sync::Arc;
use std::time::Duration;

mod flight_control;
mod http_handler;
mod console_communication;

use crate::console_communication::console_endpoint::{ConsoleEndpoint, ConsoleEndpointEvent};
use crate::console_communication::melvin_messages;
use crate::flight_control::{
    camera_controller::CameraController,
    camera_state::CameraAngle,
    common::{bitmap::Bitmap, pinned_dt::PinnedTimeDelay, vec2d::Vec2D},
    flight_computer::FlightComputer,
    flight_state::FlightState,
};
use crate::http_handler::http_request::{
    request_common::NoBodyHTTPRequestType,
    reset_get::ResetRequest,
};
use chrono::{TimeDelta, Utc};
use futures::FutureExt;
use prost::Message;
use rand::Rng;
use tokio::time;
use crate::flight_control::camera_controller::MapImage;

const FUEL_RESET_THRESHOLD: f32 = 20.0;
const BIN_FILEPATH: &str = "camera_controller_narrow.bin";
const MIN_PX_LEFT_FACTOR: f32 = 0.05;
const MIN_BATTERY_THRESHOLD: f32 = 20.0;
const CONST_ANGLE: CameraAngle = CameraAngle::Narrow;
const ACCEL_FACTOR: f32 = 0.6;

#[tokio::main]
async fn main() {
    let mut finished = false;
    let console_endpoint = Arc::new(ConsoleEndpoint::start());

    while !finished {
        let result = panic::AssertUnwindSafe(execute_main_loop(&console_endpoint))
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

async fn execute_main_loop(console_endpoint: &Arc<ConsoleEndpoint>) -> Result<(), Box<dyn std::error::Error>> {
    // Spawn the watchdog task
    let timeout = Duration::from_secs(7200);
    //tokio::spawn(watchdog(timeout, || {panic!("[INFO] Resetting normally after 2 hours")}));

    let mut camera_controller = CameraController::from_file(BIN_FILEPATH)
        .await
        .unwrap_or_else(|e| {
            println!("[WARN] Failed to read from binary file: {e}");
            CameraController::new()
        });


    let camera_controller_local = camera_controller.clone();
    let console_endpoint_local = console_endpoint.clone();
    tokio::spawn(async move {
        let mut receiver = console_endpoint_local.upstream_event_receiver().resubscribe();
        loop {
            if let Ok(event) = receiver.recv().await {
                println!("{:?}", event);
                match event {
                    ConsoleEndpointEvent::Message(melvin_messages::UpstreamContent::CreateSnapshotImage(_)) => {
                        camera_controller_local.create_snapshot().await.unwrap();
                    }
                    ConsoleEndpointEvent::Message(melvin_messages::UpstreamContent::GetSnapshotDiffImage(_)) => {
                        let encoded_image = camera_controller_local.diff_snapshot().await.unwrap();
                        console_endpoint_local.send_downstream(melvin_messages::DownstreamContent::Image(melvin_messages::Image::new_full_size(encoded_image)));
                    }
                    ConsoleEndpointEvent::Message(melvin_messages::UpstreamContent::GetFullImage(_)) => {
                        let encoded_image = camera_controller_local.export_full_thumbnail_png().await.unwrap();
                        console_endpoint_local.send_downstream(melvin_messages::DownstreamContent::Image(melvin_messages::Image::new_full_size(encoded_image)));
                    }
                    _ => {}
                }
            } else {
                break;
            }
        }
    });

    let http_handler = http_handler::http_client::HTTPClient::new("http://10.100.50.1:33000");
    ResetRequest {}.send_request(&http_handler).await?;
    let mut fcont = FlightComputer::new(&http_handler).await;

    let mut max_orbit_prediction_secs = 60000;

    'outer: loop {
        let coverage_bitmap = camera_controller.clone_coverage_bitmap().await;
        if coverage_bitmap.all() {
            break;
        }
        fcont.update_observation().await;
        console_endpoint.send_downstream(melvin_messages::DownstreamContent::Telemetry(melvin_messages::Telemetry {
            timestamp: Utc::now().timestamp_millis(),
            state: 0,
            position_x: fcont.get_current_pos().x().as_i32(),
            position_y: fcont.get_current_pos().y().as_i32(),
            velocity_x: fcont.get_current_vel().x(),
            velocity_y: fcont.get_current_vel().y(),
            battery: 0.0,
            fuel: 0.0,
            data_sent: 0,
            data_received: 0,
            distance_covered: 0.0,
        }));

        fcont
            .rotate_vel(rand::rng().random_range(-60.0..60.0), ACCEL_FACTOR)
            .await;
        let mut orbit_coverage = Bitmap::from_map_size();
        calculate_orbit_coverage_map(&fcont, &mut orbit_coverage, max_orbit_prediction_secs);
        orbit_coverage.data &= !coverage_bitmap.clone(); // this checks if there are any possible, unphotographed regions on the current orbit

        println!(
            "[LOG] Total Orbit Possible Coverage Gain: {:.2}",
            orbit_coverage.data.count_ones() as f64 / orbit_coverage.len() as f64
        );

        while orbit_coverage.data.any() {
            let covered_perc = coverage_bitmap.count_ones() as f32
                / coverage_bitmap.len() as f32;
            println!("[INFO] Global Coverage percentage: {:.5}", covered_perc);
            let mut adaptable_tolerance = 5;
            fcont.update_observation().await;

            // if battery to low go into charge
            if fcont.get_battery() < MIN_BATTERY_THRESHOLD {
                println!("[INFO] Battery to low, going into charge!");
                fcont.charge_until(100.0).await;
                println!("[LOG] Charged to: {}", fcont.get_battery());
            }

            // calculate delay until next image and px threshold
            let mut delay: TimeDelta = TimeDelta::seconds(185 + adaptable_tolerance); // TODO: fix adaptable tolerance
            adaptable_tolerance -= 1;
            let min_pixel = min(
                (CONST_ANGLE.get_square_unit_length() as usize - 50).pow(2),
                (orbit_coverage.len() as f32 * MIN_PX_LEFT_FACTOR * covered_perc).round() as usize,
            );
            if fcont.get_state() == FlightState::Acquisition {
                delay = TimeDelta::seconds(20);
            }
            let next_image = next_image_dt(&fcont, &orbit_coverage, delay, min_pixel);
            if next_image.get_end() < Utc::now() {
                break;
            }
            fcont.set_next_image(next_image);

            // if next image time is more than 6 minutes ahead -> switch to charge
            if fcont
                .get_next_image()
                .time_left()
                .ge(&TimeDelta::seconds(370))
            {
                // TODO: magic numbers
                fcont.set_state(FlightState::Charge).await;
            }
            if fcont.get_state() == FlightState::Acquisition {
                // TODO: print detailed here to debug this further
                let image_dt = fcont.get_next_image().time_left();
                let image_dt_std = image_dt.to_std().unwrap_or(Duration::from_secs(0));
                fcont.make_ff_call(image_dt_std).await;
            } else {
                let sleep_duration = fcont.get_next_image().time_left()
                    - TimeDelta::seconds(185 - adaptable_tolerance); // tolerance for

                let sleep_duration_std = sleep_duration.to_std().unwrap_or(Duration::from_secs(0));
                adaptable_tolerance -= 1;
                // skip duration until state transition to acquisition needs to be performed
                fcont.make_ff_call(sleep_duration_std).await;

                // while next image time is more than 3 minutes + tolerance -> wait
                while fcont
                    .get_next_image()
                    .time_left()
                    .ge(&TimeDelta::seconds(185 + adaptable_tolerance))
                {
                    tokio::time::sleep(Duration::from_millis(400)).await;
                }
                // switch to acquisition and wait for exact image taking timestamp
                fcont.set_state(FlightState::Acquisition).await;
            }
            if fcont.get_state_update().await != FlightState::Acquisition {
                let dt = fcont.get_next_image().time_left();
                println!("[WARN] Failed to be ready and in acquisition for image in {dt}");
                continue;
            }
            // skip duration until state transition to acquisition is finished
            while fcont
                .get_next_image()
                .time_left()
                .ge(&TimeDelta::milliseconds(100))
            {
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
            println!("[INFO] Shooting image!");
            fcont.set_angle(CONST_ANGLE).await;
            camera_controller
                .shoot_image_to_buffer(&http_handler, &mut fcont, CONST_ANGLE)
                .await?;
            camera_controller.export_bin(BIN_FILEPATH).await?;

            let radius = CONST_ANGLE.get_square_radius() as i32;
            let c_pos = fcont.get_current_pos().wrap_around_map();
            let c_pos = Vec2D::new(c_pos.x() as i32, c_pos.y() as i32);
            let encoded_thumbnail = camera_controller.export_thumbnail_png(c_pos, radius as u32).await?;
            let offset = Vec2D::new(c_pos.x() - radius, c_pos.y() - radius).wrap_around_map();
            console_endpoint.send_downstream(melvin_messages::DownstreamContent::Image(melvin_messages::Image {
                height: (radius * 2) / MapImage::THUMBNAIL_SCALE_FACTOR as i32,
                width: (radius * 2) / MapImage::THUMBNAIL_SCALE_FACTOR as i32,
                offset_x: offset.x() / MapImage::THUMBNAIL_SCALE_FACTOR as i32,
                offset_y: offset.y() / MapImage::THUMBNAIL_SCALE_FACTOR as i32,
                data: encoded_thumbnail,
            }));

            orbit_coverage.set_region(
                Vec2D::new(
                    fcont.get_current_pos().x() as f32,
                    fcont.get_current_pos().y() as f32,
                ),
                CONST_ANGLE,
                false,
            );
            fcont.remove_next_image();
        }

        // if there is not enough fuel left -> reset
        if fcont.get_fuel_left() < FUEL_RESET_THRESHOLD {
            ResetRequest {}.send_request(&http_handler).await?;
            max_orbit_prediction_secs += 10000;
            println!("[WARN] No Fuel left: Resetting!");
            continue 'outer;
        }
        println!("[INFO] Performing orbit change and starting over!");
        fcont.rotate_vel(40.0, ACCEL_FACTOR).await;
    }
    Ok(())
}

fn next_image_dt(
    cont: &FlightComputer,
    map: &Bitmap,
    base_delay: TimeDelta,
    min_px: usize,
) -> PinnedTimeDelay {
    let init_pos = cont.pos_in_time_delta(base_delay);
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
            return time_del;
        }
        current_calculation_time_delta += TimeDelta::seconds(1);
        current_pos = cont.pos_in_time_delta(current_calculation_time_delta).wrap_around_map();
    }
    println!("[WARN] No possible image time found!");
    PinnedTimeDelay::new(TimeDelta::seconds(-100))
}

fn calculate_orbit_coverage_map(cont: &FlightComputer, map: &mut Bitmap, max_dt: i64) {
    println!("[INFO] Calculating Orbit Coverage!");
    let mut dt = 0; // TODO: should be higher than 0 to account for spent time during calculation
    loop {
        let mut next_pos = cont.pos_in_time_delta(TimeDelta::seconds(dt)).wrap_around_map();
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
