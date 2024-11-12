// TODO: 422 Response Code: Validation Error -> not implemented

#[cfg(debug_assertions)]
#[derive(serde::Deserialize, Debug)]
pub struct ControlSatelliteResponse {
    vel_x: f64,
    vel_y: f64,
    camera_angle: String,
    state: String,
    status: String,
}

