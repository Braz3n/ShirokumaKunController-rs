#![no_std]
#![no_main]

use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::Output;
use embassy_rp::i2c::{Async, InterruptHandler};
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIO0};
use embassy_time::{Duration, Timer};
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
async fn scd4x_task(scd4x_i2c: embassy_rp::i2c::I2c<'static, I2C0, Async>) {
    const SCD4X_ADDR: u8 = 0x62;
    loop {
        // Do a thing
    }
}
