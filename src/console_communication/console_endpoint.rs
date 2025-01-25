use super::melvin_messages;
use prost::Message;
use std::io::{Cursor, ErrorKind};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::tcp::{ReadHalf, WriteHalf};
use tokio::net::TcpListener;
use tokio::sync::broadcast;
use tokio::sync::oneshot;

#[derive(Debug, Clone)]
pub enum ConsoleEvent {
    Connected,
    Disconnected,
    Message(melvin_messages::UpstreamContent),
}

pub(crate) struct ConsoleEndpoint {
    downstream_sender: broadcast::Sender<Option<Vec<u8>>>,
    upstream_event_receiver: broadcast::Receiver<ConsoleEvent>,
    close_oneshot_sender: Option<oneshot::Sender<()>>,
}

impl ConsoleEndpoint {
    async fn handle_connection_rx(
        mut socket: &mut ReadHalf<'_>,
        upstream_event_sender: &broadcast::Sender<ConsoleEvent>,
    ) -> Result<(), std::io::Error> {
        loop {
            let length = socket.read_u32().await?;

            let mut buffer = vec![0u8; length as usize];
            socket.read_exact(&mut buffer).await?;

            if let Ok(message) = melvin_messages::Upstream::decode(&mut Cursor::new(buffer)) {
                upstream_event_sender
                    .send(ConsoleEvent::Message(message.content.unwrap()))
                    .unwrap();
            }
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    async fn handle_connection_tx(
        mut socket: &mut WriteHalf<'_>,
        downstream_receiver: &mut broadcast::Receiver<Option<Vec<u8>>>,
    ) -> Result<(), std::io::Error> {
        while let Ok(Some(message_buffer)) = downstream_receiver.recv().await {
            socket.write_u32(message_buffer.len() as u32).await?;
            socket.write_all(&message_buffer).await?;
        }

        Ok(())
    }

    pub(crate) fn start() -> Self {
        let downstream_sender = broadcast::Sender::new(10);
        let upstream_event_sender = broadcast::Sender::new(10);
        let (close_oneshot_sender, mut close_oneshot_receiver) = oneshot::channel();
        let inst = Self {
            downstream_sender: downstream_sender.clone(),
            upstream_event_receiver: upstream_event_sender.subscribe(),
            close_oneshot_sender: Some(close_oneshot_sender),
        };

        tokio::spawn(async move {
            let listener = TcpListener::bind("0.0.0.0:1337").await.unwrap();
            loop {
                let accept = tokio::select! {
                    accept = listener.accept() => accept,
                    _ = &mut close_oneshot_receiver => break
                };

                if let Ok((mut socket, _)) = accept {
                    upstream_event_sender.send(ConsoleEvent::Connected).unwrap();
                    let mut upstream_event_sender_local = upstream_event_sender.clone();
                    let mut downstream_receiver = downstream_sender.subscribe();

                    tokio::spawn(async move {
                        let (mut rx_socket, mut tx_socket) = socket.split();

                        let result = tokio::select! {
                            res = ConsoleEndpoint::handle_connection_tx(&mut tx_socket, &mut downstream_receiver) => res,
                            res = ConsoleEndpoint::handle_connection_rx(&mut rx_socket, &upstream_event_sender_local) => res
                        };

                        upstream_event_sender_local.send(ConsoleEvent::Disconnected).unwrap();
                        match result {
                            Err(e)
                                if e.kind() == ErrorKind::UnexpectedEof
                                    || e.kind() == ErrorKind::ConnectionReset
                                    || e.kind() == ErrorKind::ConnectionAborted =>
                            {
                                return;
                            }
                            Err(e) => {
                                eprintln!("[WARN]: Closing connection to console due to {e:?}");
                            }
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

    pub(crate) fn send_downstream(&self, msg: melvin_messages::DownstreamContent) {
        let _ = self.downstream_sender.send(Some(
            melvin_messages::Downstream { content: Some(msg) }.encode_to_vec(),
        ));
    }
    pub(crate) fn is_console_connected(&self) -> bool {
        self.downstream_sender.receiver_count() > 0
    }

    pub(crate) fn upstream_event_receiver(&self) -> &broadcast::Receiver<ConsoleEvent> {
        &self.upstream_event_receiver
    }
}

impl Drop for ConsoleEndpoint {
    fn drop(&mut self) {
        self.close_oneshot_sender.take().unwrap().send(()).unwrap();
        self.downstream_sender.send(None).unwrap();
    }
}
