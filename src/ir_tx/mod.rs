#![allow(dead_code)]

use embassy_rp::pwm::SetDutyCycle;
use embassy_time::Timer;

pub mod ir_cmd_gen;

pub struct AirconState {
    pub update_type: ir_cmd_gen::AirconUpdateType,
    pub mode: ir_cmd_gen::AirconMode,
    pub fan_speed: ir_cmd_gen::AirconFanSpeed,
    pub target_temp: u8,
    pub timer_on_duration: u16,
    pub timer_off_duration: u16,
}

pub struct IrLed {
    pwm: embassy_rp::pwm::Pwm<'static>,
    duty_cycle_percent: u8,
}

impl IrLed {
    // pub async fn ir_init(slice0: PWM_SLICE0, pin16: PIN_16) -> Self {
    pub fn ir_init(pwm: embassy_rp::pwm::Pwm<'static>) -> Self {
        Self {
            pwm: pwm,
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

    async fn send_symbol(&mut self, symbol: bool, new_transmission: bool, end_transmission: bool) {
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

    pub async fn send_command_buffer(
        &mut self,
        command_buffer: &mut [u8; ir_cmd_gen::COMMAND_BYTE_COUNT],
    ) {
        for byte in 0..command_buffer.len() {
            for bit in 0..8 {
                let symbol = (command_buffer[byte] >> bit) & 0x01 == 1;
                self.send_symbol(
                    symbol,
                    byte == 0 && bit == 0,
                    byte == command_buffer.len() - 1 && bit == 8 - 1,
                )
                .await;
            }
        }
    }

    pub async fn test_fn(&mut self) {
        let mut command_buffer: [u8; ir_cmd_gen::COMMAND_BYTE_COUNT] =
            [0; ir_cmd_gen::COMMAND_BYTE_COUNT];

        ir_cmd_gen::populate_command_buffer(
            ir_cmd_gen::AirconUpdateType::Mode,
            ir_cmd_gen::AirconMode::Heating,
            ir_cmd_gen::AirconFanSpeed::Speed5,
            24,
            0,
            0,
            &mut command_buffer,
        );

        self.send_command_buffer(&mut command_buffer).await;
    }
}
