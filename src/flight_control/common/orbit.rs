use crate::flight_control::{
    camera_state::CameraAngle,
    common::{
        math::{fmod_f32, gcd_i32, lcm_f64},
        vec2d::Vec2D,
    },
    flight_computer::FlightComputer,
    flight_state::FlightState,
};
use crate::{CHARGE_CHARGE_PER_S, MAX_BATTERY_THRESHOLD, MIN_BATTERY_THRESHOLD};
use num::{rational::Ratio, ToPrimitive};

pub struct Orbit {
    init_timestamp: chrono::DateTime<chrono::Utc>,
    init_state: FlightState,
    init_battery: f32,
    fp: Vec2D<f32>,
    vel: Vec2D<f32>,
    vel_exact: Vec2D<Ratio<i32>>,
    orbit_period: Option<(f64, f64, f64)>,
}

impl Orbit {
    const SIM_TIMESTEP: f32 = 0.5;
    const VEL_DECIMALS: i32 = 2;

    #[allow(clippy::cast_possible_truncation)]
    pub fn new(cont: &FlightComputer) -> Self {
        let vel_imprecise = cont.current_vel();
        let x_nom = (vel_imprecise.x() * 10.0f32.powi(Self::VEL_DECIMALS)).round() as i32;
        let y_nom = (vel_imprecise.y() * 10.0f32.powi(Self::VEL_DECIMALS)).round() as i32;
        let denom = i32::from(10i16.pow(Self::VEL_DECIMALS as u32));
        let vel_exact = Vec2D::new(Ratio::new(x_nom, denom), Ratio::new(y_nom, denom));
        Self {
            init_timestamp: chrono::Utc::now(),
            init_state: cont.state(),
            init_battery: cont.current_battery(),
            fp: cont.current_pos(),
            vel: vel_imprecise,
            orbit_period: None,
            vel_exact,
        }
    }

    pub fn start_timestamp(&self) -> chrono::DateTime<chrono::Utc> {
        self.init_timestamp
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn period(&mut self) -> Option<f32> {
        if let Some(period) = self.orbit_period.as_mut() {
            Some(period.0 as f32)
        } else {
            let gcd_x = gcd_i32(*self.vel_exact.x().numer(), Vec2D::map_size().x());
            let gcd_y = gcd_i32(*self.vel_exact.y().numer(), Vec2D::map_size().y());
            let t_x = Vec2D::<f64>::map_size().x() / f64::from(gcd_x);
            let t_y = Vec2D::<f64>::map_size().y() / f64::from(gcd_y);
            let tts = lcm_f64(
                t_x * f64::from(*self.vel_exact.x().denom()),
                t_y * f64::from(*self.vel_exact.y().denom()),
            );
            let disp_x = (self.vel_exact.x() * tts as i32).to_f32().unwrap();
            let disp_y = (self.vel_exact.y() * tts as i32).to_f32().unwrap();
            let disp = Vec2D::new(disp_x, disp_y);

            let resulting_point = (self.fp + disp).wrap_around_map();
            let resulting_dist = resulting_point - self.fp;
            if resulting_dist.x().abs() < f32::EPSILON && resulting_dist.y().abs() < f32::EPSILON {
                self.orbit_period = Some((
                    tts,
                    t_x * f64::from(*self.vel_exact.x().denom()),
                    t_y * f64::from(*self.vel_exact.y().denom()),
                ));
                Some(tts as f32)
            } else {
                None
            }
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn max_image_dt(&self, used_lens: CameraAngle) -> Option<f32> {
        match self.orbit_period {
            None => None,
            Some(periods) => {
                let wraps_x = (periods.0 / periods.1) as f32;
                let wraps_y = (periods.0 / periods.2) as f32;
                let img_side_length = f32::from(used_lens.get_square_side_length());
                let ver_wrap_hor_dist = Vec2D::<f32>::map_size().y() / wraps_x;
                let hor_wrap_ver_dist = Vec2D::<f32>::map_size().x() / wraps_y;

                let dominant_vel = self.vel.x().max(self.vel.y());
                let overlap_hor = ver_wrap_hor_dist - (img_side_length / 2.0);
                let overlap_ver = hor_wrap_ver_dist - (img_side_length / 2.0);
                if overlap_hor < img_side_length || overlap_ver < img_side_length {
                    if self.vel.x() / Vec2D::<f32>::map_size().x()
                        < self.vel.y() / Vec2D::<f32>::map_size().y()
                    {
                        Some(
                            (overlap_hor / self.vel.x())
                                .min((img_side_length / 2.0) / dominant_vel),
                        )
                    } else {
                        Some(
                            (overlap_ver / self.vel.y())
                                .min((img_side_length / 2.0) / dominant_vel),
                        )
                    }
                } else {
                    None
                }
            }
        }
    }

    //pub fn is_closed(&self) -> bool { self.period().is_some() }

    pub fn will_visit(&self, pos: Vec2D<f32>) -> bool {
        let pos_dist = pos - self.fp;
        let map_x = Vec2D::<f32>::map_size().x();
        let map_y = Vec2D::<f32>::map_size().y();
        // Compute displacements modulo the map dimensions
        let delta_x = fmod_f32(pos_dist.x(), map_x);
        let delta_y = fmod_f32(pos_dist.y(), map_y);

        // Check if the displacements align with the velocity ratios
        if self.vel.x().abs() > f32::EPSILON && self.vel.y().abs() > f32::EPSILON {
            let tx = delta_x / self.vel.x();
            let ty = delta_y / self.vel.y();
            // Check if the times align within tolerance
            ((tx % map_x) - (ty % map_y)).abs() < f32::EPSILON
        } else {
            false
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn last_possible_state_change(&self) -> Option<(i32, FlightState)> {
        match self.orbit_period {
            None => None,
            Some(periods) => {
                if self.init_state == FlightState::Acquisition {
                    let battery_left = self.init_battery - MIN_BATTERY_THRESHOLD;
                    // TODO: handle battery_left < 0
                    let min_rendezvous_batt = MAX_BATTERY_THRESHOLD - battery_left;
                    let orbit_cycle_end_shift =
                        (min_rendezvous_batt - MIN_BATTERY_THRESHOLD) / CHARGE_CHARGE_PER_S;
                    Some((
                        (periods.0 as f32 - orbit_cycle_end_shift) as i32,
                        FlightState::Charge,
                    ))
                } else {
                    // This should never happen ???
                    None
                }
            }
        }
    }
}
