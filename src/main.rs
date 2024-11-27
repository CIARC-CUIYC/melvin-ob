mod http_handler;
mod flight_control;

use chrono::{DateTime, Utc, TimeDelta};
use crate::flight_control::camera_controller::Bitmap;
use crate::flight_control::common::Vec2D;
use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::flight_state::FlightState;

#[tokio::main]
async fn main() {
    const MAP_SIZE_X: usize = 21600;
    const MAP_SIZE_Y: usize = 10800;

    let already_covered_map = Bitmap::new(MAP_SIZE_X, MAP_SIZE_Y);
    let http_handler = http_handler::http_client::HTTPClient::new("http://localhost:33000");
    let mut controller = FlightComputer::new(&http_handler).await;

    let mut possible_orbit_coverage_map = Bitmap::new(MAP_SIZE_X, MAP_SIZE_Y);
    calculate_orbit_coverage_map(&controller, &mut possible_orbit_coverage_map);

    possible_orbit_coverage_map.data.difference(&already_covered_map.data); // this checks if there are any possible, unphotographed regions on the current orbit
    while !possible_orbit_coverage_map.data.none() {
        let (next_image_time, next_image_pos) = calculate_next_image_time(&possible_orbit_coverage_map, TimeDelta::seconds(210));
        while (next_image_time - Utc::now()).ge(&TimeDelta::seconds(185))  {
            tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
        }
        controller.set_state(FlightState::Acquisition).await;
        while(next_image_time - Utc::now()).ge(&TimeDelta::milliseconds(100)) {
            tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
        }
        // TODO: shoot image here and set region to photographed
        possible_orbit_coverage_map.flip_region(next_image_pos.x() as usize, next_image_pos.y() as usize);
    }
}

fn calculate_next_image_time(map: &Bitmap, delay: TimeDelta) -> (DateTime<Utc>, Vec2D<i32>) {
    todo!()
}

fn calculate_orbit_coverage_map(cont: &FlightComputer, map: &mut Bitmap) {
    println!("Calculating Orbit Coverage!");
    let mut dt: i64 = 0;
    let start_pos = cont.pos_in_time_delta(TimeDelta::seconds(0));

    while !map.data.all() {
        let next_pos = cont.pos_in_time_delta(TimeDelta::seconds(dt));
        if next_pos.in_radius_of(&start_pos, 5.0) && dt > 100 { break; }
        map.flip_region(next_pos.x() as usize, next_pos.y() as usize);
        dt += 1;
        /*
        if (dt % 10000 == 0) {
            println!("Position: {}, {}", next_pos.x(), next_pos.y());
        }
        */
    }
    println!("Done Calculating Orbit Coverage!");
}
