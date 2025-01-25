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
            loop {
                if let Ok(event) = receiver.recv().await {
                    match event {
                        ConsoleEvent::Message(
                            melvin_messages::UpstreamContent::CreateSnapshotImage(_),
                        ) => {
                            camera_controller_local.create_snapshot_thumb().await.unwrap();
                        }
                        ConsoleEvent::Message(
                            melvin_messages::UpstreamContent::GetSnapshotDiffImage(_),
                        ) => {
                            if let Ok(encoded_image) = camera_controller_local.diff_snapshot().await
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
                        ConsoleEvent::Message(melvin_messages::UpstreamContent::GetFullImage(
                            _,
                        )) => {
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
                        _ => {}
                    }
                } else {
                    break;
                }
            }
        });

        Self {
            endpoint,
            camera_controller,
        }
    }

    pub(crate) fn send_thumbnail(&self, position: Vec2D<u32>, angle: CameraAngle) {
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
                camera_controller_local.export_thumbnail_png(position, angle).await
            {
                endpoint_local.send_downstream(melvin_messages::DownstreamContent::Image(
                    melvin_messages::Image::from_encoded_image_extract(encoded_image),
                ));
            }
        });
    }
}
