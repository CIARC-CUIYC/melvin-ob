#![allow(dead_code)]
#![warn(clippy::shadow_reuse, clippy::shadow_same, clippy::builtin_type_shadow)]

mod console_communication;
mod flight_control;
mod mode_control;
mod http_handler;
mod keychain;

use crate::flight_control::{
    camera_state::CameraAngle,
    flight_computer::FlightComputer,
    flight_state::FlightState,
    orbit::{ClosedOrbit, OrbitBase, OrbitCharacteristics, OrbitUsabilityError},
    supervisor::Supervisor,
};
use crate::keychain::{Keychain, KeychainWithOrbit};
use crate::mode_control::{
    global_mode::global_mode::{GlobalMode, OpExitSignal},
    global_mode::in_orbit_mode::InOrbitMode,
    mode_context::ModeContext,
};
use chrono::TimeDelta;
use fixed::types::I32F32;
use std::{env, sync::Arc};

const DT_MIN: TimeDelta = TimeDelta::seconds(5);
const DT_0: TimeDelta = TimeDelta::seconds(0);
const DT_0_STD: std::time::Duration = std::time::Duration::from_secs(0);
const DETUMBLE_TOL: TimeDelta = DT_MIN;

const STATIC_ORBIT_VEL: (I32F32, I32F32) = (I32F32::lit("6.40"), I32F32::lit("7.40"));
pub const MIN_BATTERY_THRESHOLD: I32F32 = I32F32::lit("10.00");
pub const MAX_BATTERY_THRESHOLD: I32F32 = I32F32::lit("100.00");
const CONST_ANGLE: CameraAngle = CameraAngle::Narrow;

#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]
#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() {    
    let base_url_var = env::var("DRS_BASE_URL");
    let base_url = base_url_var.as_ref().map_or("http://localhost:33000", |v| v.as_str());
    
    let context = Arc::new(init(base_url).await);
    
    let mut global_mode: Box<dyn GlobalMode> = Box::new(InOrbitMode::new());
    loop {
        let phase = context.o_ch_clone().await.mode_switches();
        println!("[INFO] Starting phase {phase} in {}!", global_mode.type_name());
        match global_mode.init_mode(Arc::clone(&context)).await{
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
            },
        }
        
    }
    // drop(console_messenger);
}

#[allow(clippy::cast_precision_loss)]
async fn init(
    url: &str,
) -> ModeContext {
    let init_k = Keychain::new(url).await;
    init_k.f_cont().write().await.reset().await;
    let init_k_f_cont_clone = init_k.f_cont();
    let (supervisor, obj_rx) = {
        let (sv, rx) = Supervisor::new(init_k_f_cont_clone);
        (Arc::new(sv), rx)
    };
    let supervisor_clone = Arc::clone(&supervisor);
    tokio::spawn(async move {
        supervisor_clone.run_obs_obj_mon().await;
    });
    let supervisor_clone_clone = Arc::clone(&supervisor);
    tokio::spawn(async move {
        supervisor_clone_clone.run_announcement_hub().await;
    });

    tokio::time::sleep(DT_MIN.to_std().unwrap()).await;

    let c_orbit: ClosedOrbit = {
        let f_cont_lock = init_k.f_cont();
        FlightComputer::set_state_wait(init_k.f_cont(), FlightState::Acquisition).await;
        FlightComputer::set_vel_wait(init_k.f_cont(), STATIC_ORBIT_VEL.into()).await;
        FlightComputer::set_angle_wait(init_k.f_cont(), CONST_ANGLE).await;
        let f_cont = f_cont_lock.read().await;
        ClosedOrbit::new(OrbitBase::new(&f_cont), CameraAngle::Wide).unwrap_or_else(|e| match e {
            OrbitUsabilityError::OrbitNotClosed => panic!("[FATAL] Static orbit is not closed"),
            OrbitUsabilityError::OrbitNotEnoughOverlap => {
                panic!("[FATAL] Static orbit is not overlapping enough")
            }
        })
    };

    supervisor.reset_pos_mon().notify_one();

    let orbit_char = OrbitCharacteristics::new(&c_orbit, &init_k.f_cont()).await;
    ModeContext::new(KeychainWithOrbit::new(init_k, c_orbit), orbit_char, obj_rx, supervisor)
}
// TODO: translate this into state
/*
fn recv_all_obj(
    obj_monitor: &mut Receiver<ObjectiveBase>,
    k_img_buffer: &mut BinaryHeap<KnownImgObjective>,
    beacon_buffer: &mut BinaryHeap<BeaconObjective>,
    s_img_buffer: &mut VecDeque<SecretImgObjective>,
) -> (usize, usize) {
    let mut k_img_count = 0;
    let mut beacon_count = 0;
    while let Ok(obj) = obj_monitor.try_recv() {
        let id = obj.id();
        let name = obj.name().to_string();
        let start = obj.start();
        let end = obj.end();

        match obj.obj_type() {
            ObjectiveType::Beacon { attempts_made } => {
                /*beacon_buffer.push(BeaconObjective::new(id, name, start, end));
                beacon_count += 1;*/
            }
            ObjectiveType::SecretImage {
                optic_required,
                coverage_required,
            } => {
                /*s_img_buffer.push_front(SecretImgObjective::new(
                    id,
                    name,
                    start,
                    end,
                    *optic_required,
                    *coverage_required,
                ));*/
            }
            ObjectiveType::KnownImage {
                zone,
                optic_required,
                coverage_required,
            } => {
                /*k_img_buffer.push(KnownImgObjective::new(
                    id,
                    name,
                    start,
                    end,
                    *zone,
                    *optic_required,
                    *coverage_required,
                ));
                k_img_count += 1;*/
            }
        }
        println!("[INFO] Found new objective: {}!", obj.obj_type());
    }
    (k_img_count, beacon_count)
}



async fn handle_orbit_escape(
    mode: GlobalMode,
    vel_change: &VelocityChangeTask,
    k: &Arc<KeychainWithOrbit>,
) -> GlobalMode {
    if let GlobalMode::ZonedObjectivePrepMode(obj) = mode.clone() {
        let burn = vel_change.burn();
        FlightComputer::execute_burn(k.f_cont(), vel_change.burn()).await;
        let exp_pos = burn.sequence_pos().last().unwrap();
        let current_pos = k.f_cont().read().await.current_pos();
        let diff = *exp_pos - current_pos;
        let detumble_time_delta = TimeDelta::seconds(burn.detumble_dt() as i64);
        let detumble_dt = PinnedTimeDelay::new(detumble_time_delta - DETUMBLE_TOL);
        println!("[INFO] Orbit Escape done! Expected position {exp_pos}, Actual Position {current_pos}, Diff {diff}");
        // TODO: here we shouldn't use objective.get_imaging_points but something already created,
        // TODO: also the mode change to global mode should happen sometime else
        let (vel, dev) =
            FlightComputer::evaluate_burn(k.f_cont(), burn, obj.get_imaging_points()[0]).await;
        TaskController::calculate_orbit_correction_burn(vel, dev, detumble_dt);
        GlobalMode::ZonedObjectiveRetrievalMode(obj)
    } else {
        println!(
            "[ERROR] Orbit escape change requested, global mode illegal. Skipping velocity change!"
        );
        GlobalMode::MappingMode
    }
}

#[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
async fn start_periodic_imaging(
    k_clone: Arc<KeychainWithOrbit>,
    end_time: DateTime<chrono::Utc>,
    img_dt: I32F32,
    angle: CameraAngle,
    i_shift: IndexedOrbitPosition,
) -> (
    JoinHandle<Vec<(isize, isize)>>,
    oneshot::Sender<PeriodicImagingEndSignal>,
) {
    let f_cont_lock = Arc::clone(&k_clone.f_cont());
    let (tx, rx) = oneshot::channel();

    let i_start = i_shift.new_from_pos(f_cont_lock.read().await.current_pos());

    let handle = tokio::spawn(async move {
        k_clone
            .c_cont()
            .execute_acquisition_cycle(
                f_cont_lock,
                k_clone.con(),
                (end_time, rx),
                img_dt,
                angle,
                i_start.index(),
            )
            .await
    });
    (handle, tx)
}
 */