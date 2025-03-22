use embassy_rp::i2c::Async;
use embassy_rp::peripherals::I2C0;
pub enum Scd4xCommand {
    // Basic Commands
    Scd4xCmdStartPeriodicMeasurement = 0x21B1,
    Scd4xCmdReadMeasurement = 0xEC05,
    Scd4xCmdStopPeriodicMeasurement = 0x3F86,

    // On-chip Output Signal Compensation
    Scd4xCmdSetTemperatureOffset = 0x241D,
    Scd4xCmdGetTemperatureOffset = 0x2318,
    Scd4xCmdSetSensorAltitude = 0x2427,
    Scd4xCmdGetSensorAltitude = 0x2322,
    Scd4xCmdSetAmbientPressure = 0xE000,

    // Field Calibration
    Scd4xCmdPerformForcedRecalibration = 0x362F,
    Scd4xCmdSetAutomaticSelfCalibrationEnabled = 0x2416,
    Scd4xCmdGetAutomaticSelfCalibrationEnabled = 0x2313,

    // Low Power
    Scd4xCmdStartLowPowerPeriodicMeasurement = 0x21AC,
    Scd4xCmdGetDataReadyStatus = 0xE4B8,

    // Advanced Features
    Scd4xCmdPersistSettings = 0x3615,
    Scd4xCmdGetSerialNumber = 0x3682,
    Scd4xCmdPerformSelfTest = 0x3639,
    Scd4xCmdPerformFactoryReset = 0x3632,
    Scd4xCmdReinit = 0x3646,

    // Low Power Single Shot (SCD41 only)
    Scd4xCmdMeasureSingleShot = 0x219D,
    Scd4xCmdMeasureSingleShotRhtOnly = 0x2196,
}

pub struct Scd4x {
    address: u8,
}

impl Scd4x {
    pub fn init(address: u8) -> Self {
        Self { address }
    }

    pub async fn send_command(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
        command: Scd4xCommand,
    ) {
        let command_bytes = (command as u16).to_be_bytes();
        _ = bus.write_async(self.address, command_bytes).await;
    }

    pub async fn read_response(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
        data: &mut [u8; 9],
    ) {
        _ = bus.read_async(self.address, data).await;
    }

    pub async fn read_sequence(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
        command: Scd4xCommand,
        data: &mut [u8; 9],
    ) {
        self.send_command(bus, command).await;
        self.read_response(bus, data).await;
    }
}
