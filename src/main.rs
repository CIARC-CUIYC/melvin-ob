mod flight_control;
mod http_handler;
use crate::flight_control::camera_controller::{Bitmap, CameraController};
use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::common::Vec2D;
use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::flight_state::FlightState;
use crate::http_handler::http_request::request_common::NoBodyHTTPRequestType;
use crate::http_handler::http_request::reset_get::ResetRequest;
use chrono::{DateTime, TimeDelta, Utc};

#[tokio::main]
async fn main() {
    //const MAP_SIZE_X: usize = 21600;
    //const MAP_SIZE_Y: usize = 10800;
    const FUEL_RESET_THRESHOLD: f32 = 20.0;
    const BIN_FILEPATH: &str = "camera_controller.bin";

    let mut camera_controller = CameraController::from_file(BIN_FILEPATH)
        .await
        .unwrap_or({
            println!("Failed to read camera controller from file, creating new!");
            CameraController::new()
        });

    let http_handler = http_handler::http_client::HTTPClient::new("http://localhost:33000");
    ResetRequest {}.send_request(&http_handler).await.unwrap();
    let mut controller = FlightComputer::new(&http_handler).await;

    let mut max_orbit_prediction_secs = 20000;

    'outer: while !camera_controller.map_ref().all() {
        controller.update_observation().await;
        println!(
            "Global Coverage percentage: {:.2}",
            camera_controller.map_ref().count_ones() as f64
                / camera_controller.map_ref().len() as f64
        );
        let mut possible_orbit_coverage_map = Bitmap::from_mapsize();
        calculate_orbit_coverage_map(
            &controller,
            &mut possible_orbit_coverage_map,
            max_orbit_prediction_secs,
        );
        possible_orbit_coverage_map
            .data
            .difference(camera_controller.map_ref()); // this checks if there are any possible, unphotographed regions on the current orbit
        println!(
            "Total Orbit Possible Coverage Gain: {:.2}",
            possible_orbit_coverage_map.data.count_ones() as f64
                / possible_orbit_coverage_map.size() as f64
        );
        while !possible_orbit_coverage_map.data.none() {
            controller.update_observation().await;
            let next_image_time;
            let delay: TimeDelta;
            if controller.get_state() == FlightState::Acquisition {
                delay = TimeDelta::seconds(5);
            } else {
                delay = TimeDelta::seconds(185);
            }
            next_image_time =
                calculate_next_image_time(&controller, &possible_orbit_coverage_map, delay);
            if next_image_time + delay < Utc::now() {
                break;
            }

            // if next image time is more than 6 minutes ahead -> switch to charge
            if (next_image_time - Utc::now()).ge(&TimeDelta::seconds(370)) {
                controller.set_state(FlightState::Charge).await;
            }
            let sleep_duration = (next_image_time - Utc::now() - TimeDelta::seconds(185))
                .to_std()
                .unwrap();
            controller.fast_forward(sleep_duration).await;

            // while next image time is more than 3 minutes + tolerance -> wait
            while (next_image_time - Utc::now()).ge(&TimeDelta::seconds(185)) {
                tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
            }

            // switch to acquisition and wait for exact image taking timestamp
            controller.set_state(FlightState::Acquisition).await;
            while (next_image_time - Utc::now()).ge(&TimeDelta::milliseconds(100)) {
                tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
            }
            controller.update_observation().await;

            // if during all of this a safe event occured (max_battery < 100 %) -> reset
            if controller.get_max_battery() < 95.0 {
                println!("SAFE event detected, resetting everything!");
                ResetRequest {}.send_request(&http_handler).await.unwrap();
                continue 'outer;
            }

            println!("Shooting image!");
            controller.set_angle(CameraAngle::Wide).await;
            camera_controller
                .shoot_image_to_buffer(
                    &http_handler,
                    Vec2D::new(
                        controller.get_current_pos().x() as isize,
                        controller.get_current_pos().y() as isize,
                    ),
                    CameraAngle::Wide,
                )
                .await
                .unwrap();
            camera_controller.export_bin(BIN_FILEPATH).await.unwrap();

            possible_orbit_coverage_map.region_captured(
                Vec2D::new(
                    controller.get_current_pos().x() as f32,
                    controller.get_current_pos().y() as f32,
                ),
                CameraAngle::Wide,
            );
        }

        // if there is not enough fuel left -> reset
        if controller.get_fuel_left() < FUEL_RESET_THRESHOLD {
            ResetRequest {}.send_request(&http_handler).await.unwrap();
            max_orbit_prediction_secs += 10000;
            println!("No Fuel left: Resetting!");
            continue 'outer;
        }
        println!("Performing orbit change and starting over!");
        controller.rotate_vel(20.0).await;
    }
}

fn calculate_next_image_time(
    cont: &FlightComputer,
    map: &Bitmap,
    base_delay: TimeDelta,
) -> DateTime<Utc> {
    let delay_pos = cont.pos_in_time_delta(base_delay);
    let calculation_start_time = Utc::now();
    let mut current_calculation_time_delta = base_delay;

    loop {
        if !map.is_square_empty(delay_pos, CameraAngle::Wide)
            && calculation_start_time + current_calculation_time_delta <= Utc::now()
        {
            let next_image_time = calculation_start_time + current_calculation_time_delta;
            println!("Next Image Time: {next_image_time}");
            return next_image_time;
        }
        if current_calculation_time_delta > TimeDelta::seconds(20000) {
            break;
        }
        current_calculation_time_delta += TimeDelta::seconds(1);
    }

    Utc::now() - TimeDelta::days(10)
}

fn calculate_orbit_coverage_map(cont: &FlightComputer, map: &mut Bitmap, max_dt: i64) {
    println!("Calculating Orbit Coverage!");
    let mut dt = 0; // TODO: should be higher than 0 to account for spent time during calculation
    loop {
        let next_pos = cont.pos_in_time_delta(TimeDelta::seconds(dt));
        if dt > max_dt {
            println!(
                "Coverage Percentage of current Orbit: {}",
                map.data.count_ones() as f64 / map.size() as f64
            );
            break;
        }
        map.region_captured(
            Vec2D::new(next_pos.x() as f32, next_pos.y() as f32),
            CameraAngle::Wide,
        );
        dt += 1;
    }
    println!("Done Calculating Orbit Coverage!");
}
