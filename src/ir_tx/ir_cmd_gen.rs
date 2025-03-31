#![allow(dead_code)]

#[derive(Clone, Copy)]
pub enum AirconUpdateType {
    Mode = 0x13,
    TimerOn = 0x22,
    TimerOff = 0x24,
    FanSpeed = 0x42,
    TempDown = 0x43,
    TempUp = 0x44,
    FinDirection = 0x81,
}

#[derive(Clone, Copy)]
pub enum AirconMode {
    Off = 0x0,
    Ventilation = 0x1,
    Cooling = 0x3,
    Dehumidify = 0x5,
    Heating = 0x6,
}

#[derive(Clone, Copy)]
pub enum AirconFanSpeed {
    Speed0 = 0x1,
    Speed1 = 0x2,
    Speed2 = 0x3,
    Speed3 = 0x4,
    SpeedAuto = 0x5,
    Speed5 = 0x6,
}

pub const COMMAND_BYTE_COUNT: usize = 53; // Data including preamble and parity bytes
const COMMAND_DATA_COUNT: usize = 25; // Data only count
const COMMAND_PREAMBLE: [u8; 3] = [0x01, 0x10, 0x00]; // Fixed preamble at start of each packet

pub fn populate_command_buffer(
    update_type: AirconUpdateType,
    mode: AirconMode,
    fan_speed: AirconFanSpeed,
    temperature: u8,
    timer_on_duration: u16,
    timer_off_duration: u16,
    command_buffer: &mut [u8; COMMAND_BYTE_COUNT],
) {
    let mut raw_data_buffer: [u8; COMMAND_DATA_COUNT] = [0; COMMAND_DATA_COUNT];

    raw_data_buffer[0] = 0x40; // Constant
    raw_data_buffer[1] = 0xFF; // Constant
    raw_data_buffer[2] = 0xCC; // Constant
    raw_data_buffer[3] = 0x92; // Constant
    raw_data_buffer[4] = update_type as u8; // Update Type
    raw_data_buffer[5] = temperature << 2; // Temperature
    raw_data_buffer[6] = 0x00; // Constant
    raw_data_buffer[7] = ((timer_off_duration & 0xF) << 4) as u8; // Byte7[7:4] Timer Off Minutes low nibble
    raw_data_buffer[8] = ((timer_off_duration >> 4) & 0xFF) as u8; // Byte8[7:0] Timer Off Minutes high byte
    raw_data_buffer[9] = (timer_on_duration & 0xFF) as u8; // Byte9[7:0] Timer On Minutes low byte
    raw_data_buffer[10] = ((timer_on_duration >> 8) & 0xF) as u8; // Byte10[3:0] Timer On Minutes high nibble
    raw_data_buffer[10] |= (((timer_off_duration > 0) as u8) << 4) as u8; // Timer Off Flag: Byte10[4]
    raw_data_buffer[10] |= (((timer_on_duration > 0) as u8) << 5) as u8; // Timer On Flag: Byte10[5]

    raw_data_buffer[11] = if matches!(mode, AirconMode::Off) {
        ((fan_speed as u8) << 4) | (AirconMode::Heating as u8)
    } else {
        ((fan_speed as u8) << 4) | (mode as u8)
    };

    raw_data_buffer[12] = if matches!(mode, AirconMode::Off) {
        0xE1
    } else if matches!(&mode, AirconMode::Heating) || matches!(&mode, AirconMode::Cooling) {
        0xF1
    } else {
        0xF0
    };

    raw_data_buffer[13] = 0x00; // Constant
    raw_data_buffer[14] = 0x00; // Constant
    raw_data_buffer[15] = 0x80; // Constant
    raw_data_buffer[16] = 0x03; // Constant
    raw_data_buffer[17] = 0x01; // Constant
    raw_data_buffer[18] = 0x88; // Constant
    raw_data_buffer[19] = 0x00; // Constant
    raw_data_buffer[20] = 0x00; // Constant
    raw_data_buffer[21] = 0xFF; // Constant
    raw_data_buffer[22] = 0xFF; // Constant
    raw_data_buffer[23] = 0xFF; // Constant
    raw_data_buffer[24] = 0xFF; // Constant

    // Convert to command buffer
    // Every second byte after the first 3 are the bitwise inverse of the first byte in a given pair
    // i.e. byte[3] == !byte[4]
    command_buffer[0] = COMMAND_PREAMBLE[0];
    command_buffer[1] = COMMAND_PREAMBLE[1];
    command_buffer[2] = COMMAND_PREAMBLE[2];
    for i in 0..COMMAND_DATA_COUNT {
        command_buffer[2 * i + 3] = raw_data_buffer[i];
        command_buffer[2 * i + 4] = !raw_data_buffer[i];
    }
}
