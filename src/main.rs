#![allow(dead_code, clippy::similar_names)]
#![warn(clippy::shadow_reuse, clippy::shadow_same, clippy::builtin_type_shadow)]
mod console_communication;
mod flight_control;
mod http_handler;
mod keychain;
mod logger;
mod mode_control;

use crate::flight_control::beacon_controller::BeaconController;
use crate::flight_control::{
    camera_state::CameraAngle,
    flight_computer::FlightComputer,
    flight_state::FlightState,
    orbit::{ClosedOrbit, OrbitBase, OrbitCharacteristics, OrbitUsabilityError},
    supervisor::Supervisor,
};
use crate::keychain::{Keychain, KeychainWithOrbit};
use crate::mode_control::{
    base_mode::BaseMode,
    mode::{global_mode::GlobalMode, in_orbit_mode::InOrbitMode},
    mode_context::ModeContext,
    signal::OpExitSignal,
};
use chrono::TimeDelta;
use fixed::types::I32F32;
use std::{env, sync::Arc, time::Duration};

const DT_MIN: TimeDelta = TimeDelta::seconds(5);
const DT_0: TimeDelta = TimeDelta::seconds(0);
const DT_0_STD: Duration = Duration::from_secs(0);
const DETUMBLE_TOL: TimeDelta = DT_MIN;

const STATIC_ORBIT_VEL: (I32F32, I32F32) = (I32F32::lit("6.40"), I32F32::lit("7.40"));
const CONST_ANGLE: CameraAngle = CameraAngle::Narrow;

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() {
    let base_url_var = env::var("DRS_BASE_URL");
    let base_url = base_url_var.as_ref().map_or("http://localhost:33000", |v| v.as_str());
    let context = Arc::new(init(base_url).await);

    let mut global_mode: Box<dyn GlobalMode> = Box::new(InOrbitMode::new(BaseMode::MappingMode));
    loop {
        let phase = context.o_ch_clone().await.mode_switches();
        info!("Starting phase {phase} in {}!", global_mode.type_name());
        match global_mode.init_mode(Arc::clone(&context)).await {
            OpExitSignal::ReInit(mode) => {
                global_mode = mode;
                continue;
            }
            OpExitSignal::Continue => (),
        };
        match global_mode.exec_task_queue(Arc::clone(&context)).await {
            OpExitSignal::ReInit(mode) => {
                global_mode = mode;
                continue;
            }
            OpExitSignal::Continue => {
                global_mode = global_mode.exit_mode(Arc::clone(&context)).await;
                continue;
            }
        }
    }
    // drop(console_messenger);
}

#[allow(clippy::cast_precision_loss)]
async fn init(url: &str) -> ModeContext {
    let init_k = Keychain::new(url).await;
    init_k.f_cont().write().await.reset().await;
    let init_k_f_cont_clone = init_k.f_cont();
    let (supervisor, obj_rx, beac_rx) = {
        let (sv, rx_obj, rx_beac) = Supervisor::new(init_k_f_cont_clone);
        (Arc::new(sv), rx_obj, rx_beac)
    };
    let (beac_cont, beac_state_rx) = {
        let res = BeaconController::new(beac_rx);
        (Arc::new(res.0), res.1)
    };
    let supervisor_clone = Arc::clone(&supervisor);
    tokio::spawn(async move {
        supervisor_clone.run_obs_obj_mon().await;
    });
    let supervisor_clone_clone = Arc::clone(&supervisor);
    tokio::spawn(async move {
        supervisor_clone_clone.run_announcement_hub().await;
    });
    let beac_cont_clone = Arc::clone(&beac_cont);
    let handler = Arc::clone(&init_k.client());
    tokio::spawn(async move {
        beac_cont_clone.run(handler).await;
    });

    tokio::time::sleep(DT_MIN.to_std().unwrap()).await;

    let c_orbit: ClosedOrbit = {
        let f_cont_lock = init_k.f_cont();
        FlightComputer::set_state_wait(init_k.f_cont(), FlightState::Acquisition).await;
        FlightComputer::set_vel_wait(init_k.f_cont(), STATIC_ORBIT_VEL.into(), false).await;
        FlightComputer::set_angle_wait(init_k.f_cont(), CONST_ANGLE).await;
        let f_cont = f_cont_lock.read().await;
        ClosedOrbit::new(OrbitBase::new(&f_cont), CameraAngle::Wide).unwrap_or_else(|e| match e {
            OrbitUsabilityError::OrbitNotClosed => fatal!("Static orbit is not closed"),
            OrbitUsabilityError::OrbitNotEnoughOverlap => {
                fatal!("Static orbit is not overlapping enough")
            }
        })
    };

    supervisor.reset_pos_mon().notify_one();

    let orbit_char = OrbitCharacteristics::new(&c_orbit, &init_k.f_cont()).await;
    ModeContext::new(
        KeychainWithOrbit::new(init_k, c_orbit),
        orbit_char,
        obj_rx,
        beac_state_rx,
        supervisor,
        beac_cont,
    )
}
