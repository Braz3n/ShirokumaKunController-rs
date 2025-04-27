use defmt::*;
use embassy_net::tcp::TcpSocket;
use embedded_tls::{Aes256GcmSha384, TlsConnection};
use heapless::Vec;

use mqttrs::*;

use crate::wifi::{BROKER_CLIENT_ID, BROKER_PASS, BROKER_USER};

use thiserror::Error;

#[derive(Error, Debug)]
#[allow(dead_code)]
pub enum MqttError {
    #[error("Unexpected packet over tcp")]
    PacketError,
    #[error("Connection Failed")]
    ConnectionError,
    #[error("Socket Read Error")]
    SocketReadError,
    #[error("Socket Write Error")]
    SocketWriteError,
}

pub(super) async fn mqtt_connect(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
    mqtt_buffer: &mut [u8; 1024],
    tcp_read_buffer: &mut [u8; 16384],
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

    let len = encode_slice(&pkt, mqtt_buffer).unwrap();
    write_with_flush(socket, &mqtt_buffer[..len]).await;

    // Receive the connack packet
    loop {
        socket.read(tcp_read_buffer).await.unwrap();

        match decode_slice(tcp_read_buffer) {
            Ok(Some(Packet::Connack(Connack {
                session_present: false,
                code: ConnectReturnCode::Accepted,
            }))) => {
                debug!("Successfully connected");
                return Ok(());
            }
            Ok(Some(Packet::Connack(_resp))) => {
                error!("Unsuccessful connection attempt");
                return Err(MqttError::ConnectionError);
            }
            Ok(None) => info!("Insufficient data to decode MQTT message"),
            Err(_e) => {
                error!("Read error in connection setup");
                // continue;
                return Err(MqttError::SocketReadError);
            }
            _ => {
                error!("Received unexpected packet over TCP");
                return Err(MqttError::PacketError);
            }
        }
    }
}

pub(super) async fn mqtt_subscribe(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
    mqtt_buffer: &mut [u8; 1024],
    tcp_read_buffer: &mut [u8; 16384],
    topics: Vec<SubscribeTopic, 5>,
) -> Result<Vec<SubscribeReturnCodes, 5>, MqttError> {
    let pid = Pid::new();
    let pkt = Packet::Subscribe(Subscribe { pid, topics });

    let len = encode_slice(&pkt, mqtt_buffer).unwrap();
    write_with_flush(socket, &mqtt_buffer[..len]).await;

    loop {
        socket.read(tcp_read_buffer).await.unwrap();

        match decode_slice(tcp_read_buffer) {
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
                return Err(MqttError::SocketReadError);
            }
            _ => {
                error!("Received unexpected packet over TCP");
                return Err(MqttError::PacketError);
            }
        }
    }
}

pub(super) async fn mqtt_pingreq(socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>) {
    // We simply cast this out into the void. Reception is handled elsewhere for now.
    let mut pingreq_buf = [0u8; 2];
    let _ = encode_slice(&Packet::Pingreq, &mut pingreq_buf).unwrap();

    write_with_flush(socket, &pingreq_buf).await;
}

pub(super) async fn mqtt_puback(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
    pid: Pid,
) {
    // We simply cast this out into the void. Reception is handled elsewhere for now.
    let mut puback_buf = [0u8; 4];
    let _ = encode_slice(&Packet::Puback(pid), &mut puback_buf).unwrap();
    debug!("Puback buffer contents: {:?}", puback_buf);

    write_with_flush(socket, &puback_buf).await;
}

async fn write_with_flush(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
    buf: &[u8],
) {
    socket.write(&(buf)).await.expect("Error writing TLS data");
    socket.flush().await.expect("Error flushing TLS data");
}
