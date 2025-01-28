use crate::flight_control::flight_computer::FlightComputer;
use crate::flight_control::orbit::closed_orbit::ClosedOrbit;
use crate::flight_control::orbit::index::IndexedOrbitPosition;
use tokio::sync::RwLock;

#[derive(Debug, Copy, Clone)]
pub struct OrbitCharacteristics {
    img_dt: f32,
    orbit_s_end: chrono::DateTime<chrono::Utc>,
    orbit_full_period: usize,
    i_entry: IndexedOrbitPosition,
}

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
impl OrbitCharacteristics {
    pub async fn new(c_orbit: &ClosedOrbit, f_cont: &RwLock<FlightComputer>) -> Self {
        let img_dt = c_orbit.max_image_dt();
        let orbit_s_end = c_orbit.base_orbit_ref().start_timestamp()
            + chrono::TimeDelta::seconds(c_orbit.period().0 as i64);
        let orbit_full_period = c_orbit.period().0 as usize;
        let i_entry =
            IndexedOrbitPosition::new(0, orbit_full_period, { f_cont.read().await.current_pos() });
        Self {
            img_dt,
            orbit_s_end,
            orbit_full_period,
            i_entry
        }
    }
    
    pub fn img_dt(&self) -> f32 {self.img_dt}
    pub fn orbit_s_end(&self) -> chrono::DateTime<chrono::Utc> {self.orbit_s_end}
    pub fn orbit_full_period(&self) -> usize {self.orbit_full_period}
    pub fn i_entry(&self) -> IndexedOrbitPosition {self.i_entry}
}
