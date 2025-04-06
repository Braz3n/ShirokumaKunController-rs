#![no_std]
#![no_main]

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::*;
use embassy_executor::{Executor, Spawner};

use embassy_rp;
use embassy_rp::gpio;
use embassy_rp::gpio::Output;
use embassy_rp::i2c::Async;
use embassy_rp::i2c::InterruptHandler as i2cInterruptHandler;
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIO0};
use embassy_rp::pio::InterruptHandler as pioInterruptHander;
use embassy_rp::{pio, pwm};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
use ir_tx::IrLed;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

mod ir_tx;
mod scd4x;
mod wifi;

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static IR_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, ir_tx::AirconState, 1> = Channel::new();

const WIFI_SSID: &str = "";
const WIFI_PASS: &str = "";
const TCP_PORT: u16 = 1234;

embassy_rp::bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2cInterruptHandler<embassy_rp::peripherals::I2C0>;
    PIO0_IRQ_0 => pioInterruptHander<PIO0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Initialize wifi
    let mut wifi_pio = pio::Pio::new(p.PIO0, Irqs);
    let wifi_pwr = Output::new(p.PIN_23, gpio::Level::Low);

    let wifi_spi: PioSpi<'_, PIO0, 0, DMA_CH0> = PioSpi::new(
        &mut wifi_pio.common,
        wifi_pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        wifi_pio.irq0,
        Output::new(p.PIN_25, gpio::Level::High),
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );
    // spawn_core1(
    //     p.CORE1,
    //     unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
    //     move || {
    //         let executor1 = EXECUTOR1.init(Executor::new());
    //         executor1.run(|spawner| {
    //             unwrap!(spawner.spawn(wifi::wifi_task(
    //                 spawner,
    //                 wifi_pwr,
    //                 wifi_spi,
    //                 WIFI_SSID,
    //                 WIFI_PASS,
    //                 TCP_PORT,
    //                 &IR_COMMAND_CHANNEL,
    //             )))
    //         });
    //     },
    // );

    unwrap!(spawner.spawn(wifi::wifi_task(
        spawner,
        wifi_pwr,
        wifi_spi,
        WIFI_SSID,
        WIFI_PASS,
        TCP_PORT,
        &IR_COMMAND_CHANNEL,
    )));
    // return;

    // Initialize I2C
    let scd4x_sda = p.PIN_4;
    let scd4x_scl = p.PIN_5;
    let scd4x_config = embassy_rp::i2c::Config::default();
    let bus = embassy_rp::i2c::I2c::new_async(p.I2C0, scd4x_scl, scd4x_sda, Irqs, scd4x_config);

    // Initialize IR LED
    let ir_slice = p.PWM_SLICE0;
    let ir_pin = p.PIN_16;
    let mut ir_config = pwm::Config::default();
    ir_config.top = ((125e9 / 38e6) as u16) - 1; // System Freq / 38kHz
    let ir_pwm = pwm::Pwm::new_output_a(ir_slice, ir_pin, ir_config);

    // Initialize IR Input (for logic analyzer verification only)
    let _ir_in = gpio::Input::new(p.PIN_15, gpio::Pull::Up);

    Timer::after(Duration::from_millis(100)).await; // Wait for initialization

    // unwrap!(spawner.spawn(scd4x_task(bus)));
    unwrap!(spawner.spawn(ir_task(ir_pwm, &IR_COMMAND_CHANNEL)));

    // let executor0 = EXECUTOR0.init(Executor::new());
    // executor0.run(|spawner| unwrap!(spawner.spawn(ir_task(ir_pwm, &IR_COMMAND_CHANNEL))));
}

#[embassy_executor::task(pool_size = 1)]
async fn ir_task(
    ir_pwm: embassy_rp::pwm::Pwm<'static>,
    aircon_channel: &'static Channel<CriticalSectionRawMutex, ir_tx::AirconState, 1>,
) {
    info!("Starting IR Task");

    // const LOOP_PERIOD: Duration = Duration::from_secs(5);
    // let mut ticker = Ticker::every(LOOP_PERIOD);
    // let mut start_time = Instant::now();
    let mut ir_led = IrLed::ir_init(ir_pwm);

    loop {
        let aircon_state = aircon_channel.receive().await;

        info!("Received aircon state");

        let mut command_buffer: [u8; ir_tx::ir_cmd_gen::COMMAND_BYTE_COUNT] =
            [0; ir_tx::ir_cmd_gen::COMMAND_BYTE_COUNT];

        ir_tx::ir_cmd_gen::populate_command_buffer(
            aircon_state.update_type,
            aircon_state.mode,
            aircon_state.fan_speed,
            aircon_state.target_temp,
            aircon_state.timer_on_duration,
            aircon_state.timer_off_duration,
            &mut command_buffer,
        );

        ir_led.send_command_buffer(&mut command_buffer).await;

        // // Ensure the loop is run every LOOP_PERIOD seconds
        // if start_time + LOOP_PERIOD < Instant::now() {
        //     let runtime = Instant::now().duration_since(start_time);
        //     error!(
        //         "Didn't complete loop in time! Took {:?}ms, expected {:?}ms!",
        //         runtime.as_millis(),
        //         LOOP_PERIOD.as_millis()
        //     );
        // };
        // start_time = Instant::now();

        // info!("Loop instance");
        // ir_led.test_fn().await;

        // ticker.next().await;
    }
}

#[embassy_executor::task(pool_size = 1)]
async fn scd4x_task(mut scd4x_i2c: embassy_rp::i2c::I2c<'static, I2C0, Async>) {
    const LOOP_PERIOD: Duration = Duration::from_secs(5);
    const SCD4X_ADDR: u8 = 0x62;
    let mut ticker = Ticker::every(LOOP_PERIOD);
    let mut start_time = Instant::now();
    let scd40 = scd4x::Scd4x::init(SCD4X_ADDR);
    loop {
        // Ensure the loop is run every LOOP_PERIOD seconds
        if start_time + LOOP_PERIOD < Instant::now() {
            let runtime = Instant::now().duration_since(start_time);
            error!(
                "Didn't complete loop in time! Took {:?}ms, expected {:?}ms!",
                runtime.as_millis(),
                LOOP_PERIOD.as_millis()
            );
        };
        start_time = Instant::now();

        let serial_number = scd40.get_serial_number(&mut scd4x_i2c).await.unwrap();
        info!("Serial Number: 0x{:X}", serial_number);

        let mut delay = Delay;
        let delay_duration: u32 = 5500;
        delay.delay_ms(delay_duration);

        ticker.next().await;
    }
}
