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
                    inst_local.connected.fetch_add(1, Ordering::Relaxed);
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
                        inst_local.connected.fetch_sub(1, Ordering::Relaxed);
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