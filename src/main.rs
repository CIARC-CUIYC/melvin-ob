#![allow(dead_code, clippy::similar_names)]
#![warn(clippy::shadow_reuse, clippy::shadow_same, clippy::builtin_type_shadow)]
//! Welcome to the onboard software for **Team 03 — "Cache us if you can"** competing in the **2024/2025 ESA Computer in a Room Challenge**. 
//! This repository contains the embedded code running on the simulated MELVIN onboard computer, responsible 
//! for command execution, event detection, task scheduling and DRS communication during the mission.

mod console_communication;
mod flight_control;
mod http_handler;
mod imaging;
mod mode_control;
mod objective;
mod scheduling;
mod util;

#[cfg(not(target_env = "msvc"))]
use tikv_jemallocator::Jemalloc;

#[cfg(not(target_env = "msvc"))]
#[global_allocator]
static GLOBAL: Jemalloc = Jemalloc;

use crate::flight_control::{
    FlightComputer, FlightState,
    orbit::{ClosedOrbit, OrbitBase, OrbitCharacteristics, OrbitUsabilityError},
};
use crate::imaging::CameraAngle;
use crate::mode_control::{
    ModeContext, OpExitSignal,
    mode::{GlobalMode, OrbitReturnMode},
};
use crate::objective::BeaconController;
use crate::util::{Keychain, KeychainWithOrbit};
use chrono::TimeDelta;
use fixed::types::I32F32;
use std::{env, sync::Arc, time::Duration};

const DT_MIN: TimeDelta = TimeDelta::seconds(5);
const DT_0: TimeDelta = TimeDelta::seconds(0);
const DT_0_STD: Duration = Duration::from_secs(0);
const DETUMBLE_TOL: TimeDelta = DT_MIN;

const STATIC_ORBIT_VEL: (I32F32, I32F32) = (I32F32::lit("6.40"), I32F32::lit("7.40"));
const CONST_ANGLE: CameraAngle = CameraAngle::Narrow;
const ENV_BASE_URL: &str = "DRS_BASE_URL";
const ENV_SKIP_RESET: &str = "SKIP_RESET";

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() {
    let base_url_var = env::var(ENV_BASE_URL);
    let base_url = base_url_var.as_ref().map_or("http://localhost:33000", |v| v.as_str());
    let (context, start_mode) = init(base_url).await;

    let mut global_mode = start_mode;
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
async fn init(url: &str) -> (Arc<ModeContext>, Box<dyn GlobalMode>) {
    let (init_k, obj_rx, beac_rx) = Keychain::new(url).await;

    let supervisor_clone = init_k.supervisor();
    tokio::spawn(async move {
        supervisor_clone.run_obs_obj_mon().await;
    });

    if env::var(ENV_SKIP_RESET).is_ok_and(|s| s == "1") {
        warn!("Skipping reset!");
        FlightComputer::avoid_transition(&init_k.f_cont()).await;
    } else {
        init_k.f_cont().write().await.reset().await;
    }

    let (beac_cont, beac_state_rx) = {
        let res = BeaconController::new(beac_rx);
        (Arc::new(res.0), res.1)
    };

    let supervisor_clone = init_k.supervisor();
    tokio::spawn(async move {
        supervisor_clone.run_announcement_hub().await;
    });
    let supervisor_clone = init_k.supervisor();
    let init_k_c_cont = init_k.c_cont();
    tokio::spawn(async move {
        supervisor_clone.run_daily_map_uploader(init_k_c_cont).await;
    });
    let beac_cont_clone = Arc::clone(&beac_cont);
    let handler = Arc::clone(&init_k.client());
    tokio::spawn(async move {
        beac_cont_clone.run(handler).await;
    });

    tokio::time::sleep(DT_MIN.to_std().unwrap()).await;

    if let Some(c_orbit) = ClosedOrbit::try_from_env() {
        info!(
            "Imported existing Orbit with {}% coverage!",
            c_orbit.get_coverage() * 100
        );
        let orbit_char = OrbitCharacteristics::new(&c_orbit, &init_k.f_cont()).await;
        let supervisor = init_k.supervisor();
        let mode_context = ModeContext::new(
            KeychainWithOrbit::new(init_k, c_orbit),
            orbit_char,
            obj_rx,
            beac_state_rx,
            supervisor,
            beac_cont,
        );
        return (mode_context, Box::new(OrbitReturnMode::new()));
    }

    let c_orbit: ClosedOrbit = {
        info!("Creating new Static Orbit!");
        if init_k.f_cont().read().await.current_battery() < I32F32::lit("50") {
            FlightComputer::charge_full_wait(&init_k.f_cont()).await;
        }
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

    let orbit_char = OrbitCharacteristics::new(&c_orbit, &init_k.f_cont()).await;
    let supervisor = init_k.supervisor();
    let mode_context = ModeContext::new(
        KeychainWithOrbit::new(init_k, c_orbit),
        orbit_char,
        obj_rx,
        beac_state_rx,
        supervisor,
        beac_cont,
    );
    let mode = OrbitReturnMode::get_next_mode(&mode_context).await;
    (mode_context, mode)
}
