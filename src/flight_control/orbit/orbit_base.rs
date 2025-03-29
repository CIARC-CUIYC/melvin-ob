use crate::flight_control::{
    camera_state::CameraAngle,
    common::{
        math::{MAX_DEC, gcd_fixed64, lcm_fixed64},
        vec2d::{MapSize, Vec2D},
    },
    flight_computer::FlightComputer,
};
use chrono::{DateTime, Utc};
use fixed::types::I32F32;

/// Struct representing the base properties of an orbit.
#[derive(serde::Serialize, serde::Deserialize, Debug)]
pub struct OrbitBase {
    /// The timestamp of the orbit initialization.
    init_timestamp: DateTime<Utc>,
    /// The initial position (in fixed-point coordinates) of the object in orbit.
    fp: Vec2D<I32F32>,
    /// The velocity vector (in fixed-point coordinates) of the object in orbit.
    vel: Vec2D<I32F32>,
}

impl OrbitBase {
    /// Simulation time step used for calculations.
    const SIM_TIMESTEP: I32F32 = I32F32::lit("0.5");

    /// Creates a new [`OrbitBase`] instance based on the current state of the flight computer.
    ///
    /// # Arguments
    /// - `cont`: Reference to the [`FlightComputer`] to initialize the orbit data.
    ///
    /// # Returns
    /// - A new [`OrbitBase`] instance.
    #[allow(clippy::cast_possible_truncation)]
    pub fn new(cont: &FlightComputer) -> Self {
        let vel = cont.current_vel();
        Self {
            init_timestamp: Utc::now(),
            fp: cont.current_pos(),
            vel,
        }
    }
    
    #[cfg(test)]
    pub fn test(pos: Vec2D<I32F32>, vel: Vec2D<I32F32>) -> Self {
        Self {
            init_timestamp: Utc::now(),
            fp: pos,
            vel
        }
    }

    /// Returns the timestamp when the orbit was initialized.
    ///
    /// # Returns
    /// - A `chrono::DateTime` with the UTC initialization timestamp.
    pub fn start_timestamp(&self) -> DateTime<Utc> { self.init_timestamp }

    /// Calculates the period of the orbit along with the individual periods in the x and y
    /// directions.
    ///
    /// # Returns
    /// - `Some((tts, t_x, t_y))`: The total orbit period (time to full repeat) and the x/y periods.
    /// - `None`: If the orbit period cannot be determined.
    #[allow(clippy::cast_possible_truncation)]
    pub fn period(&self) -> Option<(I32F32, I32F32, I32F32)> {
        let gcd_x = gcd_fixed64(self.vel.x(), Vec2D::map_size().x(), MAX_DEC);
        let gcd_y = gcd_fixed64(self.vel.y(), Vec2D::map_size().y(), MAX_DEC);
        let t_x = Vec2D::<I32F32>::map_size().x() / gcd_x;
        let t_y = Vec2D::<I32F32>::map_size().y() / gcd_y;
        let tts = lcm_fixed64(t_x, t_y);
        let disp_x = self.vel.x() * tts;
        let disp_y = self.vel.y() * tts;
        let disp = Vec2D::new(disp_x, disp_y);

        let mut resulting_point = self.fp + disp;
        resulting_point = resulting_point.wrap_around_map();
        let resulting_dist = (resulting_point - self.fp).abs();
        let delta = I32F32::DELTA * tts;
        if resulting_dist.round() <= delta {
            Some((tts.round(), t_x.round(), t_y.round()))
        } else {
            None
        }
    }

    /// Calculates the maximum time between image captures ensuring sufficient area overlap.
    ///
    /// # Arguments
    /// - `used_lens`: The camera lens configuration (field of view).
    /// - `periods`: A tuple containing the orbit periods `(tts, t_x, t_y)`.
    ///
    /// # Returns
    /// - `Some(max_dt)`: Maximum allowable time interval for image captures.
    /// - `None`: If the overlap conditions are not satisfied.
    #[allow(clippy::cast_possible_truncation)]
    pub fn max_image_dt(
        &self,
        used_lens: CameraAngle,
        periods: (I32F32, I32F32, I32F32),
    ) -> Option<I32F32> {
        let wraps_x = periods.0 / periods.1;
        let wraps_y = periods.0 / periods.2;
        let img_side_length = I32F32::from_num(used_lens.get_square_side_length());
        let ver_wrap_hor_dist = Vec2D::<I32F32>::map_size().y() / wraps_x;
        let hor_wrap_ver_dist = Vec2D::<I32F32>::map_size().x() / wraps_y;

        let dominant_vel = self.vel.x().max(self.vel.y());
        let overlap_hor = ver_wrap_hor_dist - (img_side_length / I32F32::lit("2.0"));
        let overlap_ver = hor_wrap_ver_dist - (img_side_length / I32F32::lit("2.0"));
        if overlap_hor < img_side_length || overlap_ver < img_side_length {
            if self.vel.x() / Vec2D::<I32F32>::map_size().x()
                < self.vel.y() / Vec2D::<I32F32>::map_size().y()
            {
                Some(
                    (overlap_hor / self.vel.x())
                        .min((img_side_length / I32F32::lit("2.0")) / dominant_vel),
                )
            } else {
                Some(
                    (overlap_ver / self.vel.y())
                        .min((img_side_length / I32F32::lit("2.0")) / dominant_vel),
                )
            }
        } else {
            None
        }
    }

    pub fn fp(&self) -> &Vec2D<I32F32> { &self.fp }
    pub fn vel(&self) -> &Vec2D<I32F32> { &self.vel }
}
