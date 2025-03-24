#![no_std]
#![no_main]

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::Output;
use embassy_rp::i2c::{Async, InterruptHandler};
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIO0};
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
use {defmt_rtt as _, panic_probe as _};

mod scd4x;

embassy_rp::bind_interrupts!(struct Irqs {
    I2C0_IRQ => InterruptHandler<embassy_rp::peripherals::I2C0>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let scd4x_sda = p.PIN_4;
    let scd4x_scl = p.PIN_5;
    let scd4x_config = embassy_rp::i2c::Config::default();
    let bus = embassy_rp::i2c::I2c::new_async(p.I2C0, scd4x_scl, scd4x_sda, Irqs, scd4x_config);

    Timer::after(Duration::from_millis(100)).await; // Wait for initialization

    unwrap!(spawner.spawn(scd4x_task(bus)))
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
        }
        start_time = Instant::now();

        let serial_number = scd40.get_serial_number(&mut scd4x_i2c).await.unwrap();
        info!("Serial Number: 0x{:X}", serial_number);

        let mut delay = Delay;
        let delay_duration: u32 = 5500;
        delay.delay_ms(delay_duration);

        ticker.next().await;
    }
}
