use embassy_rp::peripherals::{PIN_16, PWM_SLICE0};
use embassy_rp::pwm::{Config, Pwm, SetDutyCycle};
use embassy_time::Timer;

mod ir_cmd_gen;

// use thiserror::Error;

pub struct IrLed {
    pwm: embassy_rp::pwm::Pwm<'static>,
    duty_cycle_percent: u8,
}

impl IrLed {
    pub async fn ir_init(slice0: PWM_SLICE0, pin16: PIN_16) -> Self {
        const PWM_IR_WRAP: u16 = ((125e9 / 38e6) as u16) - 1;
        const PWM_IR_SEND_LEVEL: u16 = ((PWM_IR_WRAP as f32) * 0.3) as u16;
        let mut pwm_config = Config::default();
        pwm_config.top = PWM_IR_WRAP;
        pwm_config.compare_a = PWM_IR_SEND_LEVEL;

        Self {
            pwm: Pwm::new_output_a(slice0, pin16, pwm_config),
            duty_cycle_percent: 30,
        }
    }

    async fn send_pulse(&mut self) {
        const IR_PULSE_DURATION_US: u32 = 410; // Bits are deliniated by 410us long PWM pulses
        self.pwm
            .set_duty_cycle_percent(self.duty_cycle_percent)
            .unwrap();
        Timer::after_micros(IR_PULSE_DURATION_US.into()).await;
    }

    async fn send_pause(&mut self, symbol: bool) {
        const IR_PAUSE_HI_DURATION_US: u32 = 1260; // Logic 1 signalled by 1260us PWM pauses
        const IR_PAUSE_LO_DURATION_US: u32 = 425; // Logic 1 signalled by 425us PWM pauses
        self.pwm.set_duty_cycle_fully_off().unwrap();
        if symbol {
            Timer::after_micros(IR_PAUSE_HI_DURATION_US.into()).await;
        } else {
            Timer::after_micros(IR_PAUSE_LO_DURATION_US.into()).await;
        }
    }

    pub async fn send_symbol(
        &mut self,
        symbol: bool,
        new_transmission: bool,
        end_transmission: bool,
    ) {
        if new_transmission {
            // Start of transmission preamble
            self.pwm
                .set_duty_cycle_percent(self.duty_cycle_percent)
                .unwrap();
            Timer::after_micros(30000).await;

            self.pwm.set_duty_cycle_fully_off().unwrap();
            Timer::after_micros(49500).await;

            self.pwm
                .set_duty_cycle_percent(self.duty_cycle_percent)
                .unwrap();
            Timer::after_micros(3380).await;

            self.pwm.set_duty_cycle_fully_off().unwrap();
            Timer::after_micros(1700).await;
        }

        self.send_pulse().await;
        self.send_pause(symbol).await;

        if end_transmission {
            // Deliniate the end of the last bit in a transmission
            self.send_pulse().await;
            self.pwm.set_duty_cycle_fully_off().unwrap();
        }
    }
}
