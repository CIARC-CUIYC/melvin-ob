mod http_handler;
mod flight_control;

use tokio::runtime::Runtime;

#[tokio::main]
async fn main() {
    const MAP_SIZE_X: usize = 21600;
    const MAP_SIZE_Y: usize = 10800;

    let already_covered_map = flight_control::camera_controller::Bitmap::new(MAP_SIZE_X, MAP_SIZE_Y);
    let http_handler = http_handler::http_client::HTTPClient::new("http://localhost:33000");
    let controller = flight_control::flight_computer::FlightComputer::new(&http_handler).await;

    let mut possible_orbit_coverage_map = flight_control::camera_controller::Bitmap::new(MAP_SIZE_X, MAP_SIZE_Y);
    let mut dt: i64 = 0;
    let start_pos = controller.pos_in_time_delta(chrono::TimeDelta::seconds(0));

    while !possible_orbit_coverage_map.data.all() {
        let next_pos = controller.pos_in_time_delta(chrono::TimeDelta::seconds(dt));
        if next_pos.in_radius_of(&start_pos, 5.0) && dt >100 { break; }
        possible_orbit_coverage_map.flip_region(next_pos.x() as usize, next_pos.y() as usize);
        dt += 1;
        if(dt%10000 == 0){
            println!("Position: {}, {}", next_pos.x(), next_pos.y());
        }
    }
    println!("Done!")
    
}
