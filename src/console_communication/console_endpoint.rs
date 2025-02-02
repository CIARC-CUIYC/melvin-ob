use super::melvin_messages;
use prost::Message;
use std::{
    io::{Cursor, ErrorKind},
    sync::Arc,
};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{
        tcp::{ReadHalf, WriteHalf},
        TcpListener,
    },
    sync::{broadcast, oneshot},
};

#[derive(Debug, Clone)]
pub enum ConsoleEvent {
    Connected,
    Disconnected,
    Message(melvin_messages::UpstreamContent),
}

pub(crate) struct ConsoleEndpoint {
    downstream_sender: broadcast::Sender<Option<Arc<Vec<u8>>>>,
    upstream_event_sender: broadcast::Sender<ConsoleEvent>,
    close_oneshot_sender: Option<oneshot::Sender<()>>,
}

impl ConsoleEndpoint {
    async fn handle_connection_rx(
        socket: &mut ReadHalf<'_>,
        upstream_event_sender: &broadcast::Sender<ConsoleEvent>,
    ) -> Result<(), std::io::Error> {
        loop {
            let length = socket.read_u32().await?;

            let mut buffer = vec![0u8; length as usize];
            socket.read_exact(&mut buffer).await?;

            if let Ok(melvin_messages::Upstream {
                content: Some(content),
            }) = melvin_messages::Upstream::decode(&mut Cursor::new(buffer))
            {
                println!("[Info] Received upstream message: {content:?}");
                upstream_event_sender.send(ConsoleEvent::Message(content)).unwrap();
            }
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    async fn handle_connection_tx(
        socket: &mut WriteHalf<'_>,
        downstream_receiver: &mut broadcast::Receiver<Option<Arc<Vec<u8>>>>,
    ) -> Result<(), std::io::Error> {
        while let Ok(Some(message_buffer)) = downstream_receiver.recv().await {
            socket.write_u32(message_buffer.len() as u32).await?;
            socket.write_all(&message_buffer).await?;
        }

        Ok(())
    }

    pub(crate) fn start() -> Self {
        let downstream_sender = broadcast::Sender::new(5);
        let upstream_event_sender = broadcast::Sender::new(5);
        let (close_oneshot_sender, mut close_oneshot_receiver) = oneshot::channel();
        let inst = Self {
            downstream_sender: downstream_sender.clone(),
            upstream_event_sender: upstream_event_sender.clone(),
            close_oneshot_sender: Some(close_oneshot_sender),
        };
        tokio::spawn(async move {
            println!("[INFO] Started Console Endpoint",);
            let listener = TcpListener::bind("0.0.0.0:1337").await.unwrap();
            loop {
                let accept = tokio::select! {
                    accept = listener.accept() => accept,
                    _ = &mut close_oneshot_receiver => break
                };

                if let Ok((mut socket, _)) = accept {
                    let upstream_event_sender_local = upstream_event_sender.clone();
                    upstream_event_sender_local.send(ConsoleEvent::Connected).unwrap();
                    let mut downstream_receiver = downstream_sender.subscribe();

                    tokio::spawn(async move {
                        println!("[INFO] New connection from console",);
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
        let _ = self.downstream_sender.send(Some(Arc::new(
            melvin_messages::Downstream { content: Some(msg) }.encode_to_vec(),
        )));
    }
    pub(crate) fn is_console_connected(&self) -> bool {
        self.downstream_sender.receiver_count() > 0
    }

    pub(crate) fn subscribe_upstream_events(&self) -> broadcast::Receiver<ConsoleEvent> {
        self.upstream_event_sender.subscribe()
    }
}

impl Drop for ConsoleEndpoint {
    fn drop(&mut self) {
        self.close_oneshot_sender.take().unwrap().send(()).unwrap();
        self.downstream_sender.send(None).unwrap();
    }
}
