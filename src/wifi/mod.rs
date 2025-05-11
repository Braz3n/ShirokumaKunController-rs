use cyw43::JoinOptions;
use cyw43_pio::PioSpi;
use defmt::{debug, error, info, panic, unwrap, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_net::dns::DnsQueryType;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config, IpAddress, IpEndpoint, StackResources};
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker, Timer};
use embedded_tls::{
    Aes256GcmSha384, Certificate, TlsConfig, TlsConnection, TlsContext, UnsecureProvider,
};
use heapless::{String, Vec};
use mqtt::MqttError;
use rand::{RngCore, SeedableRng, rngs::StdRng};
use rustls_pemfile::Item;
use static_cell::StaticCell;

use mqttrs::{Packet, QoS, QosPid, SubscribeTopic, decode_slice};

use crate::ir_tx;

mod mqtt;

const TOPIC_AIRCON_TEMP: &str = "aircon/temp";
const TOPIC_AIRCON_MODE: &str = "aircon/mode";
const TOPIC_AIRCON_FAN_SPEED: &str = "aircon/fan_speed";
const TOPIC_AIRCON_GET: &str = "aircon/get";
const TOPIC_AIRCON_INFO: &str = "aircon/info";

static CA: &[u8] = include_bytes!("../../secrets/root_ca.pem");

const WIFI_SSID: &str = include_str!("../../secrets/wifi_ssid.txt");
const WIFI_PASS: &[u8] = include_bytes!("../../secrets/wifi_pass.txt");

const BROKER_URL: &str = include_str!("../../secrets/broker_url.txt");
const BROKER_PORT: u16 = 8883;
const BROKER_USER: Option<&str> = Some(include_str!("../../secrets/broker_user.txt"));
const BROKER_PASS: Option<&[u8]> = Some(include_bytes!("../../secrets/broker_pass.txt"));
const BROKER_CLIENT_ID: &str = "Pico";

const TCP_KEEP_ALIVE_SEC: u16 = 60 * 10; // ten minutes
const TCP_KEEP_ALIVE_OFFSET_SEC: u16 = TCP_KEEP_ALIVE_SEC >> 4; // A little bit of wiggle room before the error triggers
const WATCHDOG_TIMEOUT_SEC: u16 = 10; // ten seconds (can't be greater than ~16.78s)
const WATCHDOG_TIMEOUT_OFFSET_SEC: u16 = WATCHDOG_TIMEOUT_SEC >> 2; // A little bit of wiggle room before the error triggers

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
    aircon_channel: &'static Channel<CriticalSectionRawMutex, ir_tx::AirconState, 1>,
    tls_seed: [u8; 32],
    watchdog: &'static mut Watchdog,
) {
    let stack = setup_network_stack(spawner, pwr, wifi_spi).await;
    info!("Network stack is up!");

    let broker_ip = resolve_ip_addr(&stack, BROKER_URL).await;
    info!("Broker IP is {:?}", broker_ip);

    let mut tcp_rx_buffer = [0; 4096];
    let mut tcp_tx_buffer = [0; 4096];
    let mut tls_rx_buffer = [0; 16640];
    let mut tls_tx_buffer = [0; 16640];
    loop {
        let socket =
            open_tcp_socket(&stack, broker_ip, &mut tcp_tx_buffer, &mut tcp_rx_buffer).await;
        info!("Opened TCP socket");

        let mut tls =
            open_tls_connection(socket, tls_seed, &mut tls_tx_buffer, &mut tls_rx_buffer).await;

        info!("Opened TLS connection");

        let mut mqtt_buf = [0u8; 1024];
        mqtt::mqtt_connect(&mut tls, &mut mqtt_buf, &TCP_KEEP_ALIVE_SEC)
            .await
            .unwrap();
        info!("Connected to MQTT broker");

        let mut subscribe_topics = Vec::<SubscribeTopic, 5>::new();
        let topics = [
            TOPIC_AIRCON_TEMP,
            TOPIC_AIRCON_MODE,
            TOPIC_AIRCON_FAN_SPEED,
            TOPIC_AIRCON_GET,
        ];
        for topic in topics {
            let x = SubscribeTopic {
                topic_path: String::try_from(topic).unwrap(),
                qos: QoS::AtLeastOnce,
            };
            subscribe_topics.push(x).unwrap();
        }

        mqtt::mqtt_subscribe(&mut tls, &mut mqtt_buf, subscribe_topics)
            .await
            .unwrap();
        info!("Subscribed to topics");

        wifi_loop(&mut tls, aircon_channel, watchdog).await; // This should only return on a connection error

        // Close the TLS connection and socket in the case of an error
        // If we forget to do this, the system locks up do to an overabundance of tls tasks
        let mut socket = match tls.close().await {
            Ok(socket) => socket,
            Err((socket, err)) => {
                error!("TLS close failed {:?}", err);
                socket
            }
        };
        socket.close();
    }
}

async fn setup_network_stack(
    spawner: Spawner,
    pwr: Output<'static>,
    wifi_spi: PioSpi<'static, PIO0, 0, DMA_CH0>,
) -> embassy_net::Stack<'static> {
    let mut rng = RoscRng;

    let fw = include_bytes!("../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../cyw43-firmware/43439A0_clm.bin");

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, wifi_spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;
    info!("Fetching IP!");
    let config = Config::dhcpv4(Default::default());
    let seed = rng.next_u64();
    debug!("Fetched IP!");

    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        net_device,
        config,
        RESOURCES.init(StackResources::new()),
        seed,
    );
    info!("Joining network!");

    unwrap!(spawner.spawn(net_task(runner)));

    match control.join(WIFI_SSID, JoinOptions::new(WIFI_PASS)).await {
        Ok(()) => {}
        Err(err) => {
            self::panic!("Failed to join network with status={}", err.status);
        }
    }

    info!("Joined network!");
    while !stack.is_config_up() {
        Timer::after_millis(100).await;
    }
    info!("wait_config_up!");
    stack.wait_config_up().await;
    info!("Returning stack!");
    stack
}

async fn resolve_ip_addr(stack: &embassy_net::Stack<'_>, url: &str) -> IpAddress {
    let ip_vec = stack.dns_query(url, DnsQueryType::A).await.unwrap();
    *ip_vec.first().unwrap()
}

async fn open_tcp_socket<'buffer>(
    stack: &embassy_net::Stack<'static>,
    ip_addr: IpAddress,
    tx_buffer: &'buffer mut [u8],
    rx_buffer: &'buffer mut [u8],
) -> TcpSocket<'buffer> {
    let mut socket = TcpSocket::new(*stack, rx_buffer, tx_buffer);
    socket
        .connect(IpEndpoint::new(ip_addr, BROKER_PORT))
        .await
        .unwrap();

    socket.set_timeout(Some(Duration::from_secs(u64::from(
        TCP_KEEP_ALIVE_SEC + TCP_KEEP_ALIVE_OFFSET_SEC,
    ))));

    socket
}

async fn open_tls_connection<'buffer>(
    socket: TcpSocket<'buffer>,
    tls_seed: [u8; 32],
    tls_tx_buffer: &'buffer mut [u8],
    tls_rx_buffer: &'buffer mut [u8],
) -> TlsConnection<'buffer, TcpSocket<'buffer>, Aes256GcmSha384> {
    let Some((Item::X509Certificate(ca), _)) = rustls_pemfile::read_one_from_slice(CA).unwrap()
    else {
        self::panic!();
    };

    let tls_config = TlsConfig::new()
        .with_server_name(BROKER_URL)
        .with_ca(Certificate::X509(ca.as_ref()))
        .enable_rsa_signatures();

    let mut tls: TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384> =
        TlsConnection::new(socket, tls_rx_buffer, tls_tx_buffer);

    match tls
        .open(TlsContext::new(
            &tls_config,
            UnsecureProvider::new::<Aes256GcmSha384>(StdRng::from_seed(tls_seed)),
        ))
        .await
    {
        Ok(()) => debug!("TLS connection established"),
        Err(embedded_tls::TlsError::HandshakeAborted(_, e)) => {
            error!("Handshake error during TLS connection: {:?}", e);
        }
        Err(e) => {
            error!("Error establishing TLS connection: {:?}", e);
        }
    }

    tls
}

async fn handle_aircon_temp(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
    qospid: QosPid,
    aircon_state: &mut ir_tx::AirconState,
    arg: &str,
) -> Result<(), MqttError> {
    let target_temp = arg.parse::<u8>().unwrap();
    info!("Received aircon temp {:?}", target_temp);

    aircon_state.target_temp = target_temp;

    send_ack(socket, qospid).await
}

async fn handle_aircon_mode(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
    qospid: QosPid,
    aircon_state: &mut ir_tx::AirconState,
    arg: &str,
) -> Result<(), MqttError> {
    let mode = match arg {
        "OFF" => ir_tx::ir_cmd_gen::AirconMode::Off,
        "HEAT" => ir_tx::ir_cmd_gen::AirconMode::Heating,
        "COOL" => ir_tx::ir_cmd_gen::AirconMode::Cooling,
        "FAN" => ir_tx::ir_cmd_gen::AirconMode::Ventilation,
        _ => {
            error!("Unexpected aircon mode {:?}", arg);
            ir_tx::ir_cmd_gen::AirconMode::Off
        }
    };

    info!("Received aircon mode {:?}", mode as u8);
    aircon_state.mode = mode;

    send_ack(socket, qospid).await
}

async fn handle_aircon_fan_speed(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
    qospid: QosPid,
    aircon_state: &mut ir_tx::AirconState,
    arg: &str,
) -> Result<(), MqttError> {
    let speed = match arg {
        "1" => ir_tx::ir_cmd_gen::AirconFanSpeed::Speed0,
        "2" => ir_tx::ir_cmd_gen::AirconFanSpeed::Speed1,
        "3" => ir_tx::ir_cmd_gen::AirconFanSpeed::Speed2,
        "4" => ir_tx::ir_cmd_gen::AirconFanSpeed::Speed3,
        "5" => ir_tx::ir_cmd_gen::AirconFanSpeed::Speed5,
        "AUTO" => ir_tx::ir_cmd_gen::AirconFanSpeed::SpeedAuto,
        _ => {
            error!("Unexpected aircon fan speed {:?}", arg);
            ir_tx::ir_cmd_gen::AirconFanSpeed::SpeedAuto
        }
    };

    info!("Received aircon fan speed {:?}", speed as u8);
    aircon_state.fan_speed = speed;

    send_ack(socket, qospid).await
}

async fn handle_get_state(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
    qospid: QosPid,
    aircon_state: &mut ir_tx::AirconState,
) -> Result<(), MqttError> {
    let mut state_buffer = [0u8; 1024]; // 
    let state_string =
        match format_no_std::show(&mut state_buffer, format_args!("{aircon_state:#?}")) {
            Ok(result) => result,
            Err(_err) => "Failed to create state string. Is the buffer too short?",
        };
    send_ack(socket, qospid).await.unwrap();

    info!("Sending state buffer: {:?}", state_string);
    mqtt::mqtt_publish(socket, TOPIC_AIRCON_INFO, true, state_string).await
}

async fn handle_get_help(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
    qospid: QosPid,
) -> Result<(), MqttError> {
    let state_string = "Messages for each of the topics consists of a single ascii word as described below\n \
                                        - `aircon/temp`: Set aircon temperature (Degrees Celsius)\n \
                                        - `aircon/mode`: Set the aircon mode (`OFF`, `HEAT`, `COOL`, `FAN`)\n \
                                        - `aircon/fan_speed`: Set the aircon fan speed (`1`, `2`, `3`, `4`, `5`, `AUTO`)\n \
                                        - `aircon/get: Either \"state\" for status info or \"help\" for this message";
    send_ack(socket, qospid).await.unwrap();

    info!("Sending help string");
    mqtt::mqtt_publish(socket, TOPIC_AIRCON_INFO, true, state_string).await
}

async fn send_ack(
    socket: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
    qospid: QosPid,
) -> Result<(), MqttError> {
    match qospid {
        QosPid::AtMostOnce => {
            // No ack necessary. Do nothing.
            debug!("QoS Level 0: No puback required");
            Ok(())
        }
        QosPid::AtLeastOnce(pid) => {
            debug!("QoS Level 1: Server expects a puback");
            mqtt::mqtt_puback(socket, pid).await
        }
        QosPid::ExactlyOnce(_pid) => {
            // We either get QoS 0, which requires no response, or QoS 2, which
            // we don't currently support
            warn!("Unsupported QoS level! Skipping handling.");
            Ok(())
        }
    }
}

async fn wifi_loop(
    tls: &mut TlsConnection<'_, TcpSocket<'_>, Aes256GcmSha384>,
    aircon_channel: &'static Channel<CriticalSectionRawMutex, ir_tx::AirconState, 1>,
    watchdog: &mut Watchdog,
) -> MqttError {
    let mut keep_alive_ticker = Ticker::every(Duration::from_secs(u64::from(
        TCP_KEEP_ALIVE_SEC - TCP_KEEP_ALIVE_OFFSET_SEC,
    )));

    watchdog.start(Duration::from_secs(u64::from(WATCHDOG_TIMEOUT_SEC)));
    let mut watchdog_ticker = Ticker::every(Duration::from_secs(u64::from(
        WATCHDOG_TIMEOUT_SEC - WATCHDOG_TIMEOUT_OFFSET_SEC,
    )));

    let mut aircon_state = ir_tx::AirconState {
        update_type: ir_tx::ir_cmd_gen::AirconUpdateType::Mode,
        mode: ir_tx::ir_cmd_gen::AirconMode::Off,
        fan_speed: ir_tx::ir_cmd_gen::AirconFanSpeed::SpeedAuto,
        target_temp: 22,
        timer_on_duration: 0,
        timer_off_duration: 0,
    };

    let mut tls_read_buffer = [0; 4096];
    loop {
        // Here we're handling both reception of messages, and maintenance of the keepalive
        match select3(
            keep_alive_ticker.next(),
            tls.read(&mut tls_read_buffer),
            watchdog_ticker.next(),
        )
        .await
        {
            Either3::First(()) => {
                // Keep alive
                match mqtt::mqtt_pingreq(tls).await {
                    Ok(()) => (),
                    Err(err) => {
                        error!("Failed pingreq");
                        return err;
                    }
                }
            }
            Either3::Second(_) => {
                // Handle any other kind of incoming packet
                match decode_slice(&tls_read_buffer) {
                    Ok(Some(Packet::Pingresp)) => {
                        // Ping response
                        debug!("Received a pingresp!");
                    }
                    Ok(Some(Packet::Publish(packet))) => {
                        // Response from subscription
                        // Handle packet from user device

                        // Convert u8 slice into String
                        let mut payload_vec = Vec::<u8, 1024>::new();
                        payload_vec.extend_from_slice(packet.payload).unwrap();
                        let payload_string: String<1024> = String::from_utf8(payload_vec).unwrap();
                        debug!(
                            "Received packet from topic {:?} with payload {:?}",
                            packet.topic_name, payload_string
                        );

                        let (tx_result, update_aircon) = match packet.topic_name {
                            TOPIC_AIRCON_TEMP => (
                                handle_aircon_temp(
                                    tls,
                                    packet.qospid,
                                    &mut aircon_state,
                                    &payload_string,
                                )
                                .await,
                                true,
                            ),
                            TOPIC_AIRCON_MODE => (
                                handle_aircon_mode(
                                    tls,
                                    packet.qospid,
                                    &mut aircon_state,
                                    &payload_string,
                                )
                                .await,
                                true,
                            ),
                            TOPIC_AIRCON_FAN_SPEED => (
                                handle_aircon_fan_speed(
                                    tls,
                                    packet.qospid,
                                    &mut aircon_state,
                                    &payload_string,
                                )
                                .await,
                                true,
                            ),
                            TOPIC_AIRCON_GET => match payload_string.to_lowercase().as_str() {
                                "state" => (
                                    handle_get_state(tls, packet.qospid, &mut aircon_state).await,
                                    false,
                                ),
                                _ => (handle_get_help(tls, packet.qospid).await, false),
                            },
                            _ => {
                                warn!("Unexpected payload string! {:?}", payload_string);
                                (Ok(()), false)
                            }
                        };

                        if tx_result.is_ok() && update_aircon {
                            aircon_channel.send(aircon_state).await;
                        } else if tx_result.is_err() {
                            break tx_result.expect_err("Somehow got okay in error path");
                        }
                    }
                    Ok(None) => {
                        debug!("Insufficient data to decode MQTT message");
                    }
                    Err(_error) => {
                        error!("Failed to decode MQTT message");
                    }
                    _ => {
                        // Unhandled packet types
                    }
                }
            }
            Either3::Third(()) => {
                watchdog.feed();
            }
        }
    }
}
