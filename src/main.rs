mod http_handler;
mod flight_control;

use std::process::exit;
use chrono::{DateTime, Utc, TimeDelta};
use crate::flight_control::camera_controller::Bitmap;
use crate::flight_control::common::Vec2D;
use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::flight_state::FlightState;
use crate::flight_control::camera_state::CameraAngle;
use crate::http_handler::http_request::request_common::NoBodyHTTPRequestType;
use crate::http_handler::http_request::reset_get;
use crate::http_handler::http_request::reset_get::ResetRequest;

#[tokio::main]
async fn main() {
    // TODO: read image, read already_covered_map from file
    const MAP_SIZE_X: usize = 21600;
    const MAP_SIZE_Y: usize = 10800;
    let already_covered_map = Bitmap::new(MAP_SIZE_X, MAP_SIZE_Y);
    let http_handler = http_handler::http_client::HTTPClient::new("http://localhost:33000");
    ResetRequest {}.send_request(&http_handler).await.unwrap();
    let mut controller = FlightComputer::new(&http_handler).await;    
    let mut reset_flag = false;
    
    while !already_covered_map.data.all() {
        println!("Global Coverage percentage: {:.2}", already_covered_map.data.count_ones() as f64 / already_covered_map.size() as f64);
        let mut possible_orbit_coverage_map = Bitmap::new(MAP_SIZE_X, MAP_SIZE_Y);
        calculate_orbit_coverage_map(&controller, &mut possible_orbit_coverage_map);

        possible_orbit_coverage_map.data.difference(&already_covered_map.data); // this checks if there are any possible, unphotographed regions on the current orbit
        println!("Total Orbit Possible Coverage Gain: {:.2}", possible_orbit_coverage_map.data.count_ones() as f64 / possible_orbit_coverage_map.size() as f64);
        while !possible_orbit_coverage_map.data.none() {
            let next_image_time = calculate_next_image_time(&possible_orbit_coverage_map, TimeDelta::seconds(210));
            // if next image time is more than 6 minutes ahead -> switch to charge
            if (next_image_time - Utc::now()).ge(&TimeDelta::seconds(370)) {
                controller.set_state(FlightState::Charge).await;
            }
            while (next_image_time - Utc::now()).ge(&TimeDelta::seconds(185)) {
                tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
            }
            controller.set_state(FlightState::Acquisition).await;
            while (next_image_time - Utc::now()).ge(&TimeDelta::milliseconds(100)) {
                tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
            }
            controller.update_observation().await;
            if controller.get_max_battery() < 95.0 {
                println!("SAFE event detected, resetting everything!");
                reset_flag = true;
                break;
            }
            // TODO: shoot image here and set region to photographed
            possible_orbit_coverage_map.region_captured(controller.get_current_pos().x() as isize,
                                                        controller.get_current_pos().y() as isize,
                                                        CameraAngle::Wide);
        }
        if reset_flag {
            ResetRequest {}.send_request(&http_handler).await.unwrap();
            reset_flag = false;
            break;
        }
        println!("Performing orbit change and starting over!");
        // TODO: perform (random?) orbit change and start from new
    }
}

fn calculate_next_image_time(map: &Bitmap, delay: TimeDelta) -> DateTime<Utc> {
    todo!()
}

fn calculate_orbit_coverage_map(cont: &FlightComputer, map: &mut Bitmap) {
    println!("Calculating Orbit Coverage!");
    let mut dt: i64 = 0; // TODO: should be higher than 0 to account for spent time during calculation
    loop{
        let next_pos = cont.pos_in_time_delta(TimeDelta::seconds(dt));
        if dt > 20000 {
            println!("Coverage Percentage of current Orbit: {}", map.data.count_ones() as f64 / map.size() as f64);
            break;
        }
        map.region_captured(next_pos.x() as isize, next_pos.y() as isize, CameraAngle::Wide);
        dt += 1;        
    }
    println!("Done Calculating Orbit Coverage!");
}
