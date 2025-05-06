use defmt::{debug, error, info};
use embassy_net::tcp::TcpSocket;
use embedded_tls::{Aes256GcmSha384, TlsConnection};
use heapless::Vec;

use mqttrs::{
    Connack, Connect, ConnectReturnCode, Packet, Pid, Protocol, Publish, QosPid, Suback, Subscribe,
    SubscribeReturnCodes, SubscribeTopic, decode_slice, encode_slice,
};

use crate::wifi::{BROKER_CLIENT_ID, BROKER_PASS, BROKER_USER};

use thiserror::Error;

#[derive(Error, Debug)]
#[allow(dead_code)]
pub enum MqttError {
    #[error("Unexpected packet over tcp")]
    Packet,
    #[error("Connection Failed")]
    Connection,
    #[error("Socket Read Error")]
    SocketRead,
    #[error("Socket Write Error")]
    SocketWrite,
}

pub(super) async fn mqtt_connect(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
    mqtt_buffer: &mut [u8; 1024],
    keep_alive_sec: &u16,
) -> Result<(), MqttError> {
    let pkt = Packet::Connect(Connect {
        protocol: Protocol::MQTT311,
        keep_alive: *keep_alive_sec,
        client_id: BROKER_CLIENT_ID,
        // We can optionally attempt to reconnect to the last session (saves having to resubscribe, etc.)
        clean_session: true,
        last_will: None,
        username: BROKER_USER,
        password: BROKER_PASS,
    });
    let mut tcp_read_buffer = [0; 4];
    let len = encode_slice(&pkt, mqtt_buffer).unwrap();
    write_with_flush(socket, &mqtt_buffer[..len])
        .await
        .expect("Failed to send MQTT send message");

    // Receive the connack packet
    loop {
        socket.read(&mut tcp_read_buffer).await.unwrap();

        match decode_slice(&tcp_read_buffer) {
            Ok(Some(Packet::Connack(Connack {
                session_present: false,
                code: ConnectReturnCode::Accepted,
            }))) => {
                debug!("Successfully connected");
                return Ok(());
            }
            Ok(Some(Packet::Connack(_resp))) => {
                error!("Unsuccessful connection attempt");
                return Err(MqttError::Connection);
            }
            Ok(None) => info!("Insufficient data to decode MQTT message"),
            Err(_e) => {
                error!("Read error in connection setup");
                // continue;
                return Err(MqttError::SocketRead);
            }
            _ => {
                error!("Received unexpected packet over TCP");
                return Err(MqttError::Packet);
            }
        }
    }
}

pub(super) async fn mqtt_subscribe(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
    mqtt_buffer: &mut [u8; 1024],
    topics: Vec<SubscribeTopic, 5>,
) -> Result<Vec<SubscribeReturnCodes, 5>, MqttError> {
    let pid = Pid::new();
    let pkt = Packet::Subscribe(Subscribe { pid, topics });

    let len = encode_slice(&pkt, mqtt_buffer).unwrap();
    write_with_flush(socket, &mqtt_buffer[..len])
        .await
        .expect("Failed to send MQTT subscribe message");

    let mut tcp_read_buffer = [0; 64];
    loop {
        socket.read(&mut tcp_read_buffer).await.unwrap();

        match decode_slice(&tcp_read_buffer) {
            Ok(Some(Packet::Suback(Suback {
                pid: _,
                return_codes,
            }))) => {
                debug!("Successfully subscribed");
                return Ok(return_codes);
            }
            Ok(None) => info!("Insufficient data to decode MQTT message"),
            Err(_) => {
                error!("Read error in connection setup");
                return Err(MqttError::SocketRead);
            }
            _ => {
                error!("Received unexpected packet over TCP");
                return Err(MqttError::Packet);
            }
        }
    }
}

pub(super) async fn mqtt_pingreq(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
) -> Result<(), MqttError> {
    // We simply cast this out into the void. Reception is handled elsewhere for now.
    let mut pingreq_buf = [0u8; 2];
    let _ = encode_slice(&Packet::Pingreq, &mut pingreq_buf).unwrap();

    write_with_flush(socket, &pingreq_buf).await
}

pub(super) async fn mqtt_puback(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
    pid: Pid,
) -> Result<(), MqttError> {
    // We simply cast this out into the void. Reception is handled elsewhere for now.
    let mut puback_buf = [0u8; 4];
    let _ = encode_slice(&Packet::Puback(pid), &mut puback_buf).unwrap();
    debug!("Puback buffer contents: {:?}", puback_buf);

    write_with_flush(socket, &puback_buf).await
}

pub(super) async fn mqtt_publish(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
    topic: &str,
    retain: bool,
    payload: &str,
) -> Result<(), MqttError> {
    let mut publish_buf = [0u8; 256];
    let len = encode_slice(
        &Packet::Publish(Publish {
            dup: false,
            qospid: QosPid::AtMostOnce,
            retain,
            topic_name: topic,
            payload: payload.as_bytes(),
        }),
        &mut publish_buf,
    )
    .unwrap();

    write_with_flush(socket, &publish_buf[..len]).await
}

async fn write_with_flush(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
    buf: &[u8],
) -> Result<(), MqttError> {
    socket.write(buf).await.expect("Error writing TLS data");
    match socket.flush().await {
        Ok(()) => Ok(()),
        Err(err) => {
            error!("Failed to flush TCP socket: {:?}", err);
            Err(MqttError::SocketWrite)
        }
    }
}
