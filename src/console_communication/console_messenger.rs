use crate::flight_control::flight_state::FlightState;
use crate::flight_control::task::base_task::BaseTask;
use crate::flight_control::task::image_task::ImageTaskStatus;
use crate::flight_control::{
    camera_controller::CameraController, camera_state::CameraAngle, common::vec2d::Vec2D,
};
use crate::{
    console_communication::{
        console_endpoint::{ConsoleEndpoint, ConsoleEvent},
        melvin_messages,
    },
    flight_control::task::TaskController,
};

use std::sync::Arc;

/// Handles communication with the console.
///
/// `ConsoleMessenger` coordinates various operations involving the camera
/// controller, task controller, and console endpoint. It provides methods
/// to process upstream events and send downstream responses or data while
/// ensuring proper synchronization through asynchronous tasks.
///
/// # Fiedls
/// - `camera_controller`: A shared reference to the camera controller, used for image-related operations.
/// - `task_controller`: A shared reference to the task controller, used for managing tasks.
/// - `endpoint`: A shared reference to the console endpoint, used for sending and receiving messages.
pub struct ConsoleMessenger {
    /// A shared reference to the camera controller, used for image-related operations.
    camera_controller: Arc<CameraController>,
    /// A shared reference to the task controller, used for managing tasks.
    task_controller: Arc<TaskController>,
    /// A shared reference to the console endpoint, used for sending and receiving messages.
    endpoint: Arc<ConsoleEndpoint>,
}

impl ConsoleMessenger {
    /// Starts the `ConsoleMessenger`, initializing the console endpoint.
    /// Listens for incoming console events asynchronously.
    ///
    /// # Arguments
    /// - `camera_controller`: Shared reference to `CameraController`.
    /// - `task_controller`: Shared reference to `TaskController`.
    ///
    /// # Returns
    /// An instance of `ConsoleMessenger`.
    pub(crate) fn start(
        camera_controller: Arc<CameraController>,
        task_controller: Arc<TaskController>,
    ) -> Self {
        let endpoint = Arc::new(ConsoleEndpoint::start());
        let mut receiver = endpoint.subscribe_upstream_events();
        let endpoint_local = endpoint.clone();
        let camera_controller_local = camera_controller.clone();

        tokio::spawn(async move {
            while let Ok(event) = receiver.recv().await {
                match event {
                    ConsoleEvent::Message(
                        melvin_messages::UpstreamContent::CreateSnapshotImage(_),
                    ) => {
                        camera_controller_local.create_thumb_snapshot().await.unwrap();
                    }
                    ConsoleEvent::Message(
                        melvin_messages::UpstreamContent::GetSnapshotDiffImage(_),
                    ) => {
                        if let Ok(encoded_image) =
                            camera_controller_local.diff_thumb_snapshot().await
                        {
                            endpoint_local.send_downstream(
                                melvin_messages::DownstreamContent::Image(
                                    melvin_messages::Image::from_encoded_image_extract(
                                        encoded_image,
                                    ),
                                ),
                            );
                        }
                    }
                    ConsoleEvent::Message(melvin_messages::UpstreamContent::GetFullImage(_)) => {
                        if let Ok(encoded_image) =
                            camera_controller_local.export_full_thumbnail_png().await
                        {
                            endpoint_local.send_downstream(
                                melvin_messages::DownstreamContent::Image(
                                    melvin_messages::Image::from_encoded_image_extract(
                                        encoded_image,
                                    ),
                                ),
                            );
                        }
                    }
                    ConsoleEvent::Message(melvin_messages::UpstreamContent::SubmitObjective(
                        submit_objective,
                    )) => {
                        let c_cont_lock_local_clone = camera_controller_local.clone();
                        let endpoint_local_clone = endpoint_local.clone();
                        tokio::spawn(async move {
                            let objective_id = submit_objective.objective_id;
                            let result = c_cont_lock_local_clone
                                .export_and_upload_objective_png(
                                    objective_id as usize,
                                    Vec2D::new(
                                        submit_objective.offset_x,
                                        submit_objective.offset_y,
                                    ),
                                    Vec2D::new(submit_objective.width, submit_objective.height),
                                )
                                .await;
                            println!("[Info] Submitted objective '{objective_id}' with result: {result:?}");

                            endpoint_local_clone.send_downstream(
                                melvin_messages::DownstreamContent::SubmitResponse(
                                    melvin_messages::SubmitResponse {
                                        success: result.is_ok(),
                                        objective_id: Some(submit_objective.objective_id),
                                    },
                                ),
                            );
                        });
                    }
                    ConsoleEvent::Message(melvin_messages::UpstreamContent::SubmitDailyMap(_)) => {
                        let c_cont_lock_local_clone = camera_controller_local.clone();
                        let endpoint_local_clone = endpoint_local.clone();
                        tokio::spawn(async move {
                            let mut success =
                                c_cont_lock_local_clone.create_full_snapshot().await.is_ok();
                            if success {
                                success =
                                    c_cont_lock_local_clone.upload_daily_map_png().await.is_ok();
                            }
                            endpoint_local_clone.send_downstream(
                                melvin_messages::DownstreamContent::SubmitResponse(
                                    melvin_messages::SubmitResponse {
                                        success,
                                        objective_id: None,
                                    },
                                ),
                            );
                        });
                    }
                    _ => {}
                }
            }
        });

        Self {
            camera_controller,
            task_controller,
            endpoint,
        }
    }

    /// Sends a thumbnail image to the operator console.
    ///
    /// If the console is not connected, this method does nothing.
    ///
    /// # Arguments
    /// - `offset`: The offset coordinates for the thumbnail image.
    /// - `angle`: The camera angle for the thumbnail.
    pub(crate) fn send_thumbnail(&self, offset: Vec2D<u32>, angle: CameraAngle) {
        if !self.endpoint.is_console_connected() {
            return;
        }
        let endpoint_local = self.endpoint.clone();
        let camera_controller_local = self.camera_controller.clone();
        tokio::spawn(async move {
            if !endpoint_local.is_console_connected() {
                return;
            }
            if let Ok(encoded_image) =
                camera_controller_local.export_thumbnail_png(offset, angle).await
            {
                endpoint_local.send_downstream(melvin_messages::DownstreamContent::Image(
                    melvin_messages::Image::from_encoded_image_extract(encoded_image),
                ));
            }
        });
    }

    /// Sends the task list to the operator console.
    ///
    /// If the console is not connected, this method does nothing.
    pub(crate) async fn send_tasklist(&self) {
        if !self.endpoint.is_console_connected() {
            return;
        }
        let tasks = self
            .task_controller
            .sched_arc()
            .read()
            .await
            .iter()
            .map(|task| melvin_messages::Task {
                scheduled_on: task.dt().get_end().timestamp_millis(),
                task: Some(match task.task_type() {
                    BaseTask::TakeImage(take_image) => {
                        let actual_position = if let ImageTaskStatus::Done { actual_pos, .. } =
                            take_image.image_status
                        {
                            Some(actual_pos)
                        } else {
                            None
                        };
                        melvin_messages::TaskType::TakeImage(melvin_messages::TakeImage {
                            planned_position_x: take_image.planned_pos.x(),
                            planned_position_y: take_image.planned_pos.y(),
                            actual_position_x: actual_position.map(|p| p.x()),
                            actual_position_y: actual_position.map(|p| p.y()),
                        })
                    }
                    BaseTask::SwitchState(state) => {
                        melvin_messages::TaskType::SwitchState(match state.target_state() {
                            FlightState::Charge => melvin_messages::SatelliteState::Charge,
                            FlightState::Acquisition => {
                                melvin_messages::SatelliteState::Acquisition
                            }
                            FlightState::Deployment => melvin_messages::SatelliteState::Deployment,
                            FlightState::Transition => melvin_messages::SatelliteState::Transition,
                            FlightState::Comms => melvin_messages::SatelliteState::Communication,
                            FlightState::Safe => melvin_messages::SatelliteState::Safe,
                        } as i32)
                    }
                    BaseTask::ChangeVelocity(_velocity_change_task) => {
                        melvin_messages::TaskType::VelocityChange(melvin_messages::BurnSequence {})
                    }
                }),
            })
            .collect();

        self.endpoint.send_downstream(melvin_messages::DownstreamContent::TaskList(
            melvin_messages::TaskList { tasks },
        ));
    }
}
