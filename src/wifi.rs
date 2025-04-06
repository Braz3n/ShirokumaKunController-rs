use core::str::from_utf8;
use cyw43::JoinOptions;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config, StackResources};
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use rand::RngCore;
use static_cell::StaticCell;

use crate::ir_tx;

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

    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
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

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 4096];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));

        control.gpio_set(0, false).await;
        info!("Listening on TCP:{}...", wifi_tcp_port);
        if let Err(e) = socket.accept(wifi_tcp_port).await {
            warn!("Accept error: {:?}", e);
            continue;
        }

        info!("Received connection from {:?}", socket.remote_endpoint());
        control.gpio_set(0, true).await;

        loop {
            let _ = match socket.read(&mut buf).await {
                Ok(0) => {
                    warn!("Read EOF");
                    break;
                }
                Ok(n) => {
                    parse_message(from_utf8(&buf[..n]).unwrap(), &aircon_channel).await;
                    _ = socket.write("OK".as_bytes()).await;
                }
                Err(e) => {
                    warn!("Read error: {:?}", e);
                    break;
                }
            };
        }
    }
}

async fn parse_message(
    message: &str,
    channel: &Channel<CriticalSectionRawMutex, ir_tx::AirconState, 1>,
) {
    info!("Received message: {}", message);

    // message.split_ascii_whitespace()

    let mode = if message.trim() == "on" {
        ir_tx::ir_cmd_gen::AirconMode::Heating
    } else {
        ir_tx::ir_cmd_gen::AirconMode::Off
    };

    let state = ir_tx::AirconState {
        update_type: ir_tx::ir_cmd_gen::AirconUpdateType::Mode,
        mode: mode,
        fan_speed: ir_tx::ir_cmd_gen::AirconFanSpeed::Speed5,
        target_temp: 24,
        timer_on_duration: 0,
        timer_off_duration: 0,
    };

    channel.send(state).await;
}
