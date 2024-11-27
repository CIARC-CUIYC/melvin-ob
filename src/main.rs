mod http_handler;
mod flight_control;

use chrono::{DateTime, Utc, TimeDelta};
use crate::flight_control::camera_controller::Bitmap;
use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::flight_state::FlightState;
use crate::flight_control::camera_state::CameraAngle;
use crate::http_handler::http_request::configure_simulation_put::ConfigureSimulationRequest;
use crate::http_handler::http_request::request_common::NoBodyHTTPRequestType;
use crate::http_handler::http_request::reset_get::ResetRequest;

#[tokio::main]
async fn main() {
    // TODO: read image, read already_covered_map from file
    const MAP_SIZE_X: usize = 21600;
    const MAP_SIZE_Y: usize = 10800;
    const FUEL_RESET_THRESHOLD: f32 = 20.0;
    const FAST_FORWARD_TIME_DELTA_THRESHOLD: i32 = 20;

    let already_covered_map = Bitmap::new(MAP_SIZE_X, MAP_SIZE_Y);
    let http_handler = http_handler::http_client::HTTPClient::new("http://localhost:33000");
    ResetRequest {}.send_request(&http_handler).await.unwrap();
    let mut controller = FlightComputer::new(&http_handler).await;

    let mut max_orbit_prediction_secs = 20000;
    

    'outer: while !already_covered_map.data.all() {
        controller.update_observation().await;
        println!("Global Coverage percentage: {:.2}", already_covered_map.data.count_ones() as f64 / already_covered_map.size() as f64);
        let mut possible_orbit_coverage_map = Bitmap::new(MAP_SIZE_X, MAP_SIZE_Y);
        calculate_orbit_coverage_map(&controller, &mut possible_orbit_coverage_map, max_orbit_prediction_secs);

        possible_orbit_coverage_map.data.difference(&already_covered_map.data); // this checks if there are any possible, unphotographed regions on the current orbit
        println!("Total Orbit Possible Coverage Gain: {:.2}", possible_orbit_coverage_map.data.count_ones() as f64 / possible_orbit_coverage_map.size() as f64);
        while !possible_orbit_coverage_map.data.none() {
            let next_image_time = calculate_next_image_time(&possible_orbit_coverage_map, TimeDelta::seconds(210));

            // if next image time is more than 6 minutes ahead -> switch to charge
            if (next_image_time - Utc::now()).ge(&TimeDelta::seconds(370)) {
                controller.set_state(FlightState::Charge).await;
            }
            let sleep_duration = (next_image_time - Utc::now() - TimeDelta::seconds(185)).to_std().unwrap();
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

            // TODO: shoot image here and set region to photographed
            possible_orbit_coverage_map.region_captured(controller.get_current_pos().x() as isize,
                                                        controller.get_current_pos().y() as isize,
                                                        CameraAngle::Wide);
        }

        // if there is not enough fuel left -> reset
        if controller.get_fuel_left() < FUEL_RESET_THRESHOLD {
            ResetRequest {}.send_request(&http_handler).await.unwrap();
            max_orbit_prediction_secs += 10000;
            continue 'outer;
        }
        println!("Performing orbit change and starting over!");
        controller.rotate_vel(20.0).await;
    }
}

fn calculate_next_image_time(map: &Bitmap, delay: TimeDelta) -> DateTime<Utc> {
    todo!()
}

fn calculate_orbit_coverage_map(cont: &FlightComputer, map: &mut Bitmap, max_dt: i64) {
    println!("Calculating Orbit Coverage!");
    let mut dt = 0; // TODO: should be higher than 0 to account for spent time during calculation
    loop {
        let next_pos = cont.pos_in_time_delta(TimeDelta::seconds(dt));
        if dt > max_dt {
            println!("Coverage Percentage of current Orbit: {}", map.data.count_ones() as f64 / map.size() as f64);
            break;
        }
        map.region_captured(next_pos.x() as isize, next_pos.y() as isize, CameraAngle::Wide);
        dt += 1;
    }
    println!("Done Calculating Orbit Coverage!");
}
