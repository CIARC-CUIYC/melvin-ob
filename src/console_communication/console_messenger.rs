use crate::console_communication::console_endpoint::{ConsoleEndpoint, ConsoleEvent};
use crate::console_communication::melvin_messages;
use crate::flight_control::camera_controller::CameraController;
use crate::flight_control::camera_state::CameraAngle;
use crate::flight_control::common::vec2d::Vec2D;
use std::sync::Arc;

pub(crate) struct ConsoleMessenger {
    camera_controller: Arc<CameraController>,
    endpoint: Arc<ConsoleEndpoint>,
}

impl ConsoleMessenger {
    pub(crate) fn start(camera_controller: Arc<CameraController>) -> Self {
        let endpoint = Arc::new(ConsoleEndpoint::start());
        let mut receiver = endpoint.upstream_event_receiver().resubscribe();
        let endpoint_local = endpoint.clone();
        let camera_controller_local = camera_controller.clone();

        tokio::spawn(async move {
            while let Ok(event) = receiver.recv().await {
                match event {
                    ConsoleEvent::Message(
                        melvin_messages::UpstreamContent::CreateSnapshotImage(_),
                    ) => {
                        camera_controller_local.create_snapshot_thumb().await.unwrap();
                    }
                    ConsoleEvent::Message(
                        melvin_messages::UpstreamContent::GetSnapshotDiffImage(_),
                    ) => {
                        if let Ok(encoded_image) = camera_controller_local.diff_snapshot().await {
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
                        let camera_controller_local = camera_controller_local.clone();
                        let endpoint_local = endpoint_local.clone();
                        tokio::spawn(async move {
                            let objective_id = submit_objective.objective_id;
                            let result = camera_controller_local
                                .upload_objective_png(
                                    objective_id as usize,
                                    Vec2D::new(
                                        submit_objective.offset_x,
                                        submit_objective.offset_y,
                                    ),
                                    Vec2D::new(
                                        submit_objective.width,
                                        submit_objective.height,
                                    ),
                                )
                                .await;
                            println!("[Info] Submitted objective '{objective_id}' with result: {result:?}");

                            endpoint_local.send_downstream(
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
                        let camera_controller_local = camera_controller_local.clone();
                        let endpoint_local = endpoint_local.clone();
                        tokio::spawn(async move {
                            let mut success =
                                camera_controller_local.create_snapshot_full().await.is_ok();
                            if success {
                                success = camera_controller_local.upload_daily_map().await.is_ok();
                            }
                            endpoint_local.send_downstream(
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
            endpoint,
        }
    }

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
}
