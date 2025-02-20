use std::{collections::HashSet, sync::Arc};
use chrono::{TimeDelta, Utc};
use crate::{
    flight_control::{
        common::vec2d::Vec2D,
        flight_computer::FlightComputer,
        flight_state::FlightState,
        objective::{
            objective_base::ObjectiveBase,
        },
    },
    http_handler::http_request::{objective_list_get::ObjectiveListRequest, request_common::NoBodyHTTPRequestType},
};
use fixed::types::I32F32;
use futures::StreamExt;
use reqwest_eventsource::{Event, EventSource};
use tokio::sync::{mpsc, mpsc::Receiver, watch, Notify, RwLock};
use crate::flight_control::objective::known_img_objective::KnownImgObjective;
use crate::http_handler::ZoneType;

pub struct Supervisor {
    f_cont_lock: Arc<RwLock<FlightComputer>>,
    safe_mon: Arc<Notify>,
    reset_pos_mon: Arc<Notify>,
    obj_mon: mpsc::Sender<ObjectiveBase>,
    event_hub: watch::Sender<String>,
}

impl Supervisor {
    /// Constant update interval for observation updates in the `run()` method
    const OBS_UPDATE_INTERVAL: std::time::Duration = std::time::Duration::from_millis(500);
    /// Constant update interval for objective updates in the `run()` method
    const OBJ_UPDATE_INTERVAL: TimeDelta = TimeDelta::seconds(300);
    /// Constant minimum time delta to the objective start for sending the objective to `main`
    const B_O_MIN_DT: TimeDelta = TimeDelta::minutes(20);
    /// Creates a new instance of `Supervisor`
    pub fn new(f_cont_lock: Arc<RwLock<FlightComputer>>) -> (Supervisor, Receiver<ObjectiveBase>) {
        let (tx, rx) = mpsc::channel(10);
        let (event_send, _) = watch::channel(String::new());
        (
            Self {
                f_cont_lock,
                safe_mon: Arc::new(Notify::new()),
                reset_pos_mon: Arc::new(Notify::new()),
                obj_mon: tx,
                event_hub: event_send,
            },
            rx,
        )
    }

    pub fn safe_mon(&self) -> Arc<Notify> { Arc::clone(&self.safe_mon) }

    pub fn reset_pos_mon(&self) -> Arc<Notify> { Arc::clone(&self.reset_pos_mon) }
    
    pub fn subscribe_event_hub(&self) -> watch::Receiver<String> { self.event_hub.subscribe() }

    pub async fn run_announcement_hub(&self) {
        let url = {
            let client = self.f_cont_lock.read().await.client();
            client.url().to_string()
        };
        println!("[INFO] Starting announcement supervisor loop!");
        let mut es = EventSource::get(url + "/announcements");
        while let Some(event) = es.next().await {
            match event {
                Ok(Event::Open) => println!("[INFO] EventSource connected!"),
                Ok(Event::Message(msg)) => {
                    let msg_str = format!("{msg:#?}");
                    self.event_hub.send(msg_str).unwrap_or_else(
                        |_| println!("[EVENT] No Receiver for: {msg:#?}")
                    );
                },
                Err(err) => {
                    println!("[ERROR] EventSource error: {err}");
                    es.close();
                },
            }
        }
        println!("[FATAL] EventSource disconnected!");
    }

    /// Starts the supervisor loop to periodically call `update_observation`
    /// and monitor position & state deviations.
    #[allow(clippy::cast_precision_loss)]
    pub async fn run_obs_obj_mon(&self) {
        // ******** DEBUG INITS
        let start = Utc::now();
        let mut next_safe = start + TimeDelta::seconds(500);
        let debug_objective = KnownImgObjective::new(
            0,
            "Test Objective".to_string(),
            chrono::Utc::now(),
            chrono::Utc::now() + TimeDelta::hours(7),
            [4750, 5300, 5350, 5900],
            "narrow".into(),
            100.0,
        );
        // **********
        // TODO: pos monitoring in kalman filter + listening for reset_pos events
        let mut last_pos: Option<Vec2D<I32F32>> = None;
        let mut last_timestamp = Utc::now();
        let mut last_vel = self.f_cont_lock.read().await.current_vel();
        let mut last_objective_check = Utc::now() - Self::OBJ_UPDATE_INTERVAL;
        let mut id_list: HashSet<usize> = HashSet::new();
        println!("[INFO] Starting obs/obj supervisor loop!");
        loop {
            let mut f_cont = self.f_cont_lock.write().await;
            // Update observation and fetch new position
            f_cont.update_observation().await;
            
            /*
            if Utc::now() > next_safe {
                let act_state = f_cont.state();
                f_cont.one_time_safe();
                next_safe += TimeDelta::seconds(5000);
                println!("[INFO] One time safe mode activated Actual State was {act_state}!");
            }*/
            
            let current_pos = f_cont.current_pos();
            let current_vel = f_cont.current_vel();
            let is_safe_trans = {
                let current_state = f_cont.state();
                let target_state = f_cont.target_state();
                current_state == FlightState::Transition && target_state.is_none()
            };

            drop(f_cont); // Release the lock early to avoid blocking

            if let Some(previous_pos) = last_pos {
                let dt = Utc::now() - last_timestamp;
                let dt_secs = dt.num_milliseconds() as f32 / 1000.0;

                let expected_pos = previous_pos + last_vel * I32F32::from_num(dt_secs);

                let expected_pos_wrapped = expected_pos.wrap_around_map();
                // TODO: this diff value should also consider wrapping (if actual pos wrapped, but expected didnt)
                let diff = current_pos - expected_pos_wrapped;
                /*
                println!(
                    "[INFO] Position tracking: Current: {current_pos}, \
                    Expected: {expected_pos_wrapped}, Diff: {diff}");
                    */
                // Check flight state and handle safe mode (placeholder for now)
                if is_safe_trans {
                    println!("[WARN] Unplanned Safe Mode Transition Detected! Notifying!");
                    self.safe_mon.notify_one();
                    self.f_cont_lock.write().await.safe_detected();
                }
            }

            last_pos = Some(current_pos);
            last_timestamp = Utc::now();
            last_vel = current_vel;

            if last_objective_check + Self::OBJ_UPDATE_INTERVAL < Utc::now() {
                let handle = self.f_cont_lock.read().await.client();
                let objective_list = ObjectiveListRequest {}.send_request(&handle).await.unwrap();
                let mut send_objs = Vec::new();
                
                for img_obj in objective_list.img_objectives() {
                    let obj_on = img_obj.start() < Utc::now() && img_obj.end() > Utc::now();
                    let is_secret = matches!(img_obj.zone_type(), ZoneType::SecretZone(_));
                    if obj_on && !id_list.contains(&img_obj.id()) && !is_secret {
                        send_objs.push(ObjectiveBase::from(img_obj.clone()));
                    }
                }
                for b_o in objective_list.beacon_objectives() {
                    let obj_about = b_o.start() < Utc::now() + TimeDelta::minutes(20) && b_o.end() > Utc::now();
                    if obj_about && !id_list.contains(&b_o.id()) {
                        send_objs.push(ObjectiveBase::from(b_o.clone()));
                    }
                }
                for obj in send_objs {
                    id_list.insert(obj.id());
                    self.obj_mon.send(obj).await.unwrap();
                }
                last_objective_check = Utc::now();
            }

            tokio::time::sleep(Self::OBS_UPDATE_INTERVAL).await;
        }
    }
}
