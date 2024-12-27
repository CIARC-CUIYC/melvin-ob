use crate::melvin_messages;
use chrono::Utc;
use num_traits::ToPrimitive;
use prost::Message;
use rand::{random_bool, random_range};
use std::io::ErrorKind;
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::Arc;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::tcp::{ReadHalf, WriteHalf};
use tokio::net::TcpListener;
use tokio::sync::broadcast;

pub(crate) struct ConsoleEndpoint {
    connected: AtomicI32,
    downstream: broadcast::Sender<Vec<u8>>,
}

impl ConsoleEndpoint {
    async fn handle_connection_rx(self: &Arc<Self>, mut socket: &mut ReadHalf<'_>) -> Result<(), std::io::Error> {
        loop {
            let length = socket
                .read_u32()
                .await?;

            let mut buffer = vec![0u8; length as usize];
            socket
                .read_exact(&mut buffer)
                .await?;
        }
    }
    async fn handle_connection_tx(self: &Arc<Self>, mut socket: &mut WriteHalf<'_>) -> Result<(), std::io::Error> {
        loop {
            if let Ok(message_buffer) = self.downstream.subscribe().recv().await
            {
                socket.write_u32(message_buffer.len() as u32).await?;
                socket.write_all(&message_buffer).await?;
            } else {
                break;
            }
        }
        Ok(())
    }

    pub(crate) fn start() -> Arc::<Self> {
        let sender = broadcast::Sender::new(10);
        let inst = Arc::new(Self {
            connected: AtomicI32::new(0),
            downstream: sender,
        });


        let inst_local = inst.clone();
        tokio::spawn(async move {
            let listener = TcpListener::bind("0.0.0.0:1337").await.unwrap();

            loop {
                if let Ok((mut socket, _)) = listener.accept().await {
                    let inst_local = inst_local.clone();
                    tokio::spawn(async move {
                        let (mut rx_socket, mut tx_socket) = socket.split();
                        let result = tokio::select! {
                            res = ConsoleEndpoint::handle_connection_tx(&inst_local, &mut tx_socket) => res,
                            res = ConsoleEndpoint::handle_connection_rx(&inst_local, &mut rx_socket) => res
                        };

                        match result {
                            Err(e) if e.kind() == ErrorKind::UnexpectedEof || e.kind() == ErrorKind::ConnectionReset || e.kind() == ErrorKind::ConnectionAborted => {
                                return;
                            }
                            Err(e) => eprintln!("[WARN]: Closing connection to console due to {:?}", e),
                            _ => {}
                        };
                        let _ = socket.shutdown().await;
                    });
                } else {
                    break;
                }
            }
        });
        inst
    }

    pub(crate) fn is_connected(self: &Arc<Self>) -> bool {
        self.connected.load(Ordering::Relaxed) > 0
    }

    pub(crate) fn send_downstream(self: &Arc<Self>, msg: melvin_messages::Content) {
        let _ = self.downstream.send(melvin_messages::Downstream {
            content: Some(msg)
        }.encode_to_vec());
    }
}

pub(crate) fn random_message() -> Vec<u8> {
    if random_bool(0.5f64) {
        melvin_messages::Downstream {
            content: Some(melvin_messages::Content::Telemetry(melvin_messages::Telemetry {
                timestamp: Utc::now().timestamp_millis(),
                state: melvin_messages::SatelliteState::Communication as i32,
                battery: random_range(0..=100u8).to_f32().unwrap(),
                fuel: random_range(0..=100u8).to_f32().unwrap(),
                position_x: random_range(0..=4000i32),
                position_y: random_range(0..=4000i32),
                velocity_x: random_range(-1..=2i8).to_f32().unwrap(),
                velocity_y: random_range(-1..=2i8).to_f32().unwrap(),
                data_sent: 43,
                data_received: 44,
                distance_covered: 1024f32,
            }))
        }.encode_to_vec()
    } else {
        let data = include_bytes!("penguin.png");
        melvin_messages::Downstream {
            content: Some(melvin_messages::Content::Image(melvin_messages::Image {
                height: 32,
                width: 32,
                offset_x: random_range(0..=700),
                offset_y: random_range(0..=360),
                data: data.to_vec(),
            }))
        }.encode_to_vec()
    }
}