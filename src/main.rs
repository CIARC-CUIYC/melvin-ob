use rayon::iter::ParallelIterator;
use std::time::Duration;
mod flight_control;
mod http_handler;

use crate::flight_control::camera_controller::{Bitmap, CameraController};
use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::common::Vec2D;
use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::flight_state::FlightState;
use crate::http_handler::http_request::request_common::NoBodyHTTPRequestType;
use crate::http_handler::http_request::reset_get::ResetRequest;
use chrono::{DateTime, TimeDelta, Timelike, Utc};
use rayon::prelude::*;

struct PinnedTimeDelay {
    start_time: DateTime<Utc>,
    delay: TimeDelta,
}

impl PinnedTimeDelay {
    fn new(delta: TimeDelta) -> Self {
        Self {
            start_time: Utc::now(),
            delay: delta,
        }
    }
    fn remove_delay(&mut self, delta: TimeDelta) { self.delay -= delta; }
    fn add_delay(&mut self, delta: TimeDelta) { self.delay += delta; }
    fn set_delay(&mut self, delta: TimeDelta) { self.delay = delta; }
    fn get_end(&self) -> DateTime<Utc> { self.start_time + self.delay }
    fn get_start(&self) -> DateTime<Utc> { self.start_time }
    fn time_left(&self) -> TimeDelta { self.get_end() - Utc::now() }
}

#[tokio::main]
async fn main() {
    //const MAP_SIZE_X: usize = 21600;
    //const MAP_SIZE_Y: usize = 10800;
    const FUEL_RESET_THRESHOLD: f32 = 20.0;
    const BIN_FILEPATH: &str = "camera_controller.bin";
    const MIN_PX_LEFT_FACTOR: f32 = 0.0005;
    const MIN_BATTERY_THRESHOLD: f32 = 20.0;

    let mut camera_controller = CameraController::from_file(BIN_FILEPATH).await.unwrap_or({
        println!("Failed to read camera controller from file, creating new!");
        CameraController::new()
    });

    let http_handler = http_handler::http_client::HTTPClient::new("http://localhost:33000");
    ResetRequest {}.send_request(&http_handler).await.unwrap();
    let mut controller = FlightComputer::new(&http_handler).await;

    let mut max_orbit_prediction_secs = 20000;

    'outer: while !camera_controller.map_ref().all() {
        controller.update_observation().await;
        let covered_perc = camera_controller.map_ref().count_ones() as f32
            / camera_controller.map_ref().len() as f32;
        println!("Global Coverage percentage: {:.2}", covered_perc);
        let mut orbit_coverage = Bitmap::from_mapsize();
        calculate_orbit_coverage_map(&controller, &mut orbit_coverage, max_orbit_prediction_secs);
        orbit_coverage.data &= !(*camera_controller.map_ref()).clone(); // this checks if there are any possible, unphotographed regions on the current orbit

        println!(
            "Total Orbit Possible Coverage Gain: {:.2}",
            orbit_coverage.data.count_ones() as f64 / orbit_coverage.size() as f64
        );
        while orbit_coverage.data.any() {
            let mut adaptable_tolerance = 5;
            controller.update_observation().await;

            // if battery to low go into charge
            if controller.get_battery() < MIN_BATTERY_THRESHOLD {
                controller.charge_until(100.0).await;
            }

            // calculate delay until next image and px threshold
            let mut delay: TimeDelta = TimeDelta::seconds(185 + adaptable_tolerance); // TODO: fix adaptable tolerance
            adaptable_tolerance -= 1;
            let min_pixel =
                (covered_perc * orbit_coverage.size() as f32 * MIN_PX_LEFT_FACTOR) as usize;
            if controller.get_state() == FlightState::Acquisition {
                delay = TimeDelta::seconds(20);
            }
            let mut image_dt =
                calculate_next_image_time(&controller, &orbit_coverage, delay, min_pixel);
            if image_dt.get_end() < Utc::now() {
                break;
            }

            // if next image time is more than 6 minutes ahead -> switch to charge
            if image_dt.time_left().ge(&TimeDelta::seconds(370)) {
                // TODO: magic numbers
                controller.set_state(FlightState::Charge).await;
            }

            let sleep_duration = image_dt.time_left() - TimeDelta::seconds(185 + adaptable_tolerance); // tolerance for
            adaptable_tolerance -= 1;
            // skip duration until state transition to acquisition needs to be performed
            make_ff_call(&mut controller, &mut image_dt, sleep_duration).await;

            // while next image time is more than 3 minutes + tolerance -> wait
            while image_dt.time_left().ge(&TimeDelta::seconds(185 + adaptable_tolerance)) {
                tokio::time::sleep(Duration::from_millis(400)).await;
            }

            // switch to acquisition and wait for exact image taking timestamp
            controller.set_state(FlightState::Acquisition).await;

            // skip duration until state transition to acquisition is finished
            make_ff_call(&mut controller, &mut image_dt, TimeDelta::seconds(175 - adaptable_tolerance)).await;

            while image_dt.time_left().ge(&TimeDelta::milliseconds(100)) {
                tokio::time::sleep(Duration::from_millis(10)).await;
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

            orbit_coverage.region_captured(
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

async fn make_ff_call(
    cont: &mut FlightComputer<'_>,
    p_time: &mut PinnedTimeDelay,
    sleep: TimeDelta,
) {
    let sleep_duration_std = sleep.to_std().unwrap_or(Duration::from_secs(0));
    if sleep_duration_std.as_secs() == 0  {
        println!("Fast Forward Call rejected! Duration was git0!");
        return;
    }
    println!("Fast forwarding for {} seconds!", sleep.num_seconds());
    cont.fast_forward(sleep_duration_std).await;
    p_time.remove_delay(sleep.into());
    let time_left = p_time.time_left();
    println!(
        "Return from Fast Forwarding! Next Image in {:02}:{:02}:{:02}",
        time_left.num_hours(),
        time_left.num_minutes() - time_left.num_hours() * 60,
        time_left.num_seconds() - time_left.num_milliseconds() * 60
    );
}

fn calculate_next_image_time(
    cont: &FlightComputer,
    map: &Bitmap,
    base_delay: TimeDelta,
    min_px: usize,
) -> PinnedTimeDelay {
    let init_pos = cont.pos_in_time_delta(base_delay);
    let mut time_del = PinnedTimeDelay::new(base_delay);
    let mut current_calculation_time_delta = base_delay;
    let mut current_pos = init_pos;

    loop {
        if map.enough_ones_in_square(current_pos, CameraAngle::Wide, min_px)
            && time_del.get_end() <= time_del.get_start() + current_calculation_time_delta
        {
            time_del.set_delay(current_calculation_time_delta);
            let time_left = time_del.time_left();
            println!(
                "Next Image in {:02}:{:02}:{:02}",
                time_left.num_hours(),
                time_left.num_minutes() - time_left.num_hours() * 60,
                time_left.num_seconds() - time_left.num_milliseconds() * 60
            );
            return time_del;
        }
        if current_calculation_time_delta > TimeDelta::seconds(20000) {
            break;
        }
        current_calculation_time_delta += TimeDelta::seconds(1);
        current_pos = cont.pos_in_time_delta(current_calculation_time_delta);
    }
    println!("No possible image time found!");
    PinnedTimeDelay::new(TimeDelta::seconds(-100))
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
