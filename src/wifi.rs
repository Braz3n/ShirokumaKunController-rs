use cyw43::JoinOptions;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_net::dns::DnsQueryType;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config, IpEndpoint, StackResources};
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker, Timer};
use embedded_tls::{
    Aes128GcmSha256, Certificate, TlsConfig, TlsConnection, TlsContext, UnsecureProvider,
};
use heapless::{String, Vec};
use rand::{RngCore, SeedableRng, rngs::StdRng};
use rustls_pemfile::Item;
use static_cell::StaticCell;

use mqttrs::*;

use crate::ir_tx;

use thiserror::Error;

const TOPIC_AIRCON_TEMP: &str = "aircon/temp";
const TOPIC_AIRCON_MODE: &str = "aircon/mode";
const TOPIC_AIRCON_FAN_SPEED: &str = "aircon/fan_speed";

static CA: &[u8] = include_bytes!("../root-ca.pem");

const BROKER_URL: &str = "";
const BROKER_PORT: u16 = 8883;
const BROKER_USER: Option<&str> = Some("");
const BROKER_PASS: Option<&[u8]> = Some("".as_bytes());
const BROKER_CLIENT_ID: &str = "";

#[derive(Error, Debug)]
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

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task(pool_size = 1)]
pub async fn wifi_task(
    spawner: Spawner,
    pwr: Output<'static>,
    wifi_spi: PioSpi<'static, PIO0, 0, DMA_CH0>,
    wifi_ssid: &'static str,
    wifi_pass: &'static str,
    wifi_tcp_port: u16,
    aircon_channel: &'static Channel<CriticalSectionRawMutex, ir_tx::AirconState, 1>,
) {
    info!("Starting TCP server");

    let mut rng = RoscRng;

    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, wifi_spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let config = Config::dhcpv4(Default::default());
    let seed = rng.next_u64();

    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        net_device,
        config,
        RESOURCES.init(StackResources::new()),
        seed,
    );

    unwrap!(spawner.spawn(net_task(runner)));

    loop {
        match control
            .join(wifi_ssid, JoinOptions::new(wifi_pass.as_bytes()))
            .await
        {
            Ok(_) => break,
            Err(err) => {
                info!("Failed to join network with status={}", err.status);
            }
        }
    }

    info!("Waiting for DHCP...");
    while !stack.is_config_up() {
        Timer::after_millis(100).await
    }
    info!("DHCP is now up!");

    info!("waiting for stack to be up...");
    stack.wait_config_up().await;
    info!("Stack is up!");

    let broker_ip = stack.dns_query(BROKER_URL, DnsQueryType::A).await.unwrap();
    let broker_ip = broker_ip.first().unwrap().clone();
    info!("Broker IP is {:?}", broker_ip);

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket
        .connect(IpEndpoint::new(broker_ip, BROKER_PORT))
        .await
        .unwrap();

    const KEEP_ALIVE_SEC: u16 = 60 * 10; // ten minutes
    socket.set_timeout(Some(Duration::from_secs(
        (KEEP_ALIVE_SEC as f32 * 1.2) as u64,
    )));

    let tls_write_buffer = &mut [0; 16640];
    let tls_read_buffer = &mut [0; 16640];

    warn!("Made it this far");

    let Some((Item::X509Certificate(ca), _)) = rustls_pemfile::read_one_from_slice(CA).unwrap()
    else {
        self::panic!();
    };
    warn!("Made it this far");

    let tls_config = TlsConfig::new()
        .with_server_name(BROKER_URL)
        .with_ca(Certificate::X509(ca.as_ref()))
        .enable_rsa_signatures();

    let mut tls: TlsConnection<'_, TcpSocket<'_>, Aes128GcmSha256> =
        TlsConnection::new(socket, tls_read_buffer, tls_write_buffer);

    let rand_seed = [0; 32]; // TODO: Fucking fix this lol
    match tls
        .open(TlsContext::new(
            &tls_config,
            UnsecureProvider::new::<Aes128GcmSha256>(StdRng::from_seed(rand_seed)),
        ))
        .await
    {
        Ok(()) => info!("TLS connection established"),
        Err(embedded_tls::TlsError::HandshakeAborted(_, e)) => {
            error!("Handshake error during TLS connection: {:?}", e);
        }
        Err(e) => {
            error!("Error establishing TLS connection: {:?}", e);
        }
    }

    let mut tcp_read_buffer = [0; 16384];
    let mut mqtt_buf = [0u8; 1024];
    mqtt_connect(
        &mut tls,
        &mut mqtt_buf,
        &mut tcp_read_buffer,
        &KEEP_ALIVE_SEC,
    )
    .await
    .unwrap();

    let mut subscribe_topics = Vec::<SubscribeTopic, 5>::new();
    let topics = [TOPIC_AIRCON_TEMP, TOPIC_AIRCON_MODE, TOPIC_AIRCON_FAN_SPEED];
    for topic in topics {
        let x = SubscribeTopic {
            topic_path: String::try_from(topic).unwrap(),
            qos: QoS::AtLeastOnce,
        };
        subscribe_topics.push(x).unwrap();
    }

    mqtt_subscribe(
        &mut tls,
        &mut mqtt_buf,
        &mut tcp_read_buffer,
        subscribe_topics,
    )
    .await
    .unwrap();

    let mut keep_alive_ticker = Ticker::every(Duration::from_secs(u64::from(
        KEEP_ALIVE_SEC - (f32::from(KEEP_ALIVE_SEC) * 0.2) as u16,
    )));

    let mut aircon_state = ir_tx::AirconState {
        update_type: ir_tx::ir_cmd_gen::AirconUpdateType::Mode,
        mode: ir_tx::ir_cmd_gen::AirconMode::Off,
        fan_speed: ir_tx::ir_cmd_gen::AirconFanSpeed::SpeedAuto,
        target_temp: 22,
        timer_on_duration: 0,
        timer_off_duration: 0,
    };

    loop {
        // https://docs.rs/embassy-futures/latest/embassy_futures/select/fn.select.html
        // https://github.com/embassy-rs/embassy/blob/443ffd8b6df67844208ceec58d5443e01b99a159/examples/rp/src/bin/orchestrate_tasks.rs#L23
        // Here we're handling both reception of messages, and maintenance of the keepalive
        match select(keep_alive_ticker.next(), tls.read(&mut tcp_read_buffer)).await {
            Either::First(_) => {
                // Keep alive
                mqtt_pingreq(&mut tls).await;
            }
            Either::Second(_) => {
                // Handle any other kind of incoming packet
                match decode_slice(&mut tcp_read_buffer) {
                    Ok(Some(Packet::Pingresp)) => {
                        // Ping response
                        info!("Received a pingresp!");
                    }
                    Ok(Some(Packet::Publish(packet))) => {
                        // Response from subscription
                        // Handle packet from user device

                        // Convert u8 slice into String
                        let mut payload_vec = Vec::<u8, 100>::new();
                        payload_vec.extend_from_slice(&packet.payload).unwrap();
                        let payload_string: String<100> = String::from_utf8(payload_vec).unwrap();
                        info!(
                            "Received packet from topic {:?} with payload {:?}",
                            packet.topic_name, payload_string
                        );

                        // Convert String into iterator
                        // let payload_vec: Vec<&str, 2> = payload_string.split(' ').collect();
                        match packet.topic_name {
                            TOPIC_AIRCON_TEMP => {
                                handle_aircon_temp(&mut aircon_state, &payload_string).await
                            }
                            TOPIC_AIRCON_MODE => {
                                handle_aircon_mode(&mut aircon_state, &payload_string).await
                            }
                            TOPIC_AIRCON_FAN_SPEED => {
                                handle_aircon_fan_speed(&mut aircon_state, &payload_string).await
                            }
                            _ => {
                                warn!("Unexpected payload string! {:?}", payload_string);
                                continue;
                            }
                        }

                        match packet.qospid {
                            QosPid::AtMostOnce => {
                                // No ack necessary. Do nothing.
                            }
                            QosPid::AtLeastOnce(pid) => {
                                info!("Expecting at least one response");
                                mqtt_puback(&mut tls, pid).await;
                            }
                            QosPid::ExactlyOnce(_pid) => {
                                // We either get QoS 0, which requires no response, or QoS 2, which
                                // we don't currently support
                                warn!("Unsupported QoS level!")
                            }
                        }

                        aircon_channel.send(aircon_state).await;
                    }
                    Ok(None) => {
                        info!("Insufficient data to decode MQTT message");
                    }
                    Err(_error) => {
                        warn!("Failed to decode MQTT message");
                    }
                    _ => {
                        // Unhandled packet types
                    }
                }
            }
        }
    }
}

async fn mqtt_connect(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes128GcmSha256>,
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
    if let Err(e) = socket.write(&(mqtt_buffer[..len])).await {
        warn!("Write error in connection setup {:?}", e);
    }

    // Receive the connack packet
    loop {
        socket.read(tcp_read_buffer).await.unwrap();

        match decode_slice(tcp_read_buffer) {
            Ok(Some(Packet::Connack(Connack {
                session_present: false,
                code: ConnectReturnCode::Accepted,
            }))) => {
                info!("Successfully connected");
                return Ok(());
            }
            Ok(Some(Packet::Connack(_resp))) => {
                warn!("Unsuccessful connection attempt");
                return Err(MqttError::ConnectionError);
            }
            Ok(None) => info!("Insufficient data to decode MQTT message"),
            Err(_e) => {
                info!("Read error in connection setup");
                // continue;
                return Err(MqttError::SocketReadError);
            }
            _ => {
                warn!("Received unexpected packet over TCP");
                return Err(MqttError::PacketError);
            }
        }
    }
}

async fn mqtt_subscribe(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes128GcmSha256>,
    mqtt_buffer: &mut [u8; 1024],
    tcp_read_buffer: &mut [u8; 16384],
    topics: Vec<SubscribeTopic, 5>,
) -> Result<Vec<SubscribeReturnCodes, 5>, MqttError> {
    let pid = Pid::new();
    let pkt = Packet::Subscribe(Subscribe { pid, topics });

    let len = encode_slice(&pkt, mqtt_buffer).unwrap();
    if let Err(e) = socket.write(&mqtt_buffer[..len]).await {
        warn!("Write error in connection setup {:?}", e);
        return Err(MqttError::SocketWriteError);
    }

    loop {
        socket.read(tcp_read_buffer).await.unwrap();

        match decode_slice(tcp_read_buffer) {
            Ok(Some(Packet::Suback(Suback {
                pid: _,
                return_codes,
            }))) => {
                info!("Successfully subscribed");
                return Ok(return_codes);
            }
            Ok(None) => info!("Insufficient data to decode MQTT message"),
            Err(_) => {
                warn!("Read error in connection setup");
                return Err(MqttError::SocketReadError);
            }
            _ => {
                warn!("Received unexpected packet over TCP");
                return Err(MqttError::PacketError);
            }
        }
    }
}

async fn mqtt_pingreq(socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes128GcmSha256>) {
    // We simply cast this out into the void. Reception is handled elsewhere for now.
    let mut pingreq_buf = [0u8; 2];
    let _ = encode_slice(&Packet::Pingreq, &mut pingreq_buf).unwrap();

    socket.write(&pingreq_buf).await.unwrap();
}

async fn mqtt_puback(socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes128GcmSha256>, pid: Pid) {
    // We simply cast this out into the void. Reception is handled elsewhere for now.
    let mut puback_buf = [0u8; 4];
    let _ = encode_slice(&Packet::Puback(pid), &mut puback_buf).unwrap();
    info!("Puback buffer contents: {:?}", puback_buf);

    socket.write(&puback_buf).await.unwrap();
}

async fn handle_aircon_temp(aircon_state: &mut ir_tx::AirconState, arg: &str) {
    //
    let target_temp = arg.parse::<u8>().unwrap();
    info!("Received aircon temp {:?}", target_temp);

    aircon_state.target_temp = target_temp;
}

async fn handle_aircon_mode(aircon_state: &mut ir_tx::AirconState, arg: &str) {
    //
    let mode = match arg {
        "OFF" => ir_tx::ir_cmd_gen::AirconMode::Off,
        "HEAT" => ir_tx::ir_cmd_gen::AirconMode::Heating,
        "COOL" => ir_tx::ir_cmd_gen::AirconMode::Cooling,
        "FAN" => ir_tx::ir_cmd_gen::AirconMode::Ventilation,
        _ => {
            warn!("Unexpected aircon mode {:?}", arg);
            ir_tx::ir_cmd_gen::AirconMode::Off
        }
    };

    info!("Received aircon mode {:?}", mode as u8);
    aircon_state.mode = mode;
}

async fn handle_aircon_fan_speed(aircon_state: &mut ir_tx::AirconState, arg: &str) {
    //
    let speed = match arg {
        "1" => ir_tx::ir_cmd_gen::AirconFanSpeed::Speed0,
        "2" => ir_tx::ir_cmd_gen::AirconFanSpeed::Speed1,
        "3" => ir_tx::ir_cmd_gen::AirconFanSpeed::Speed2,
        "4" => ir_tx::ir_cmd_gen::AirconFanSpeed::Speed3,
        "5" => ir_tx::ir_cmd_gen::AirconFanSpeed::Speed5,
        "AUTO" => ir_tx::ir_cmd_gen::AirconFanSpeed::SpeedAuto,
        _ => {
            warn!("Unexpected aircon fan speed {:?}", arg);
            ir_tx::ir_cmd_gen::AirconFanSpeed::SpeedAuto
        }
    };

    info!("Received aircon fan speed {:?}", speed as u8);
    aircon_state.fan_speed = speed;
}
