#![allow(dead_code)]
use core::fmt::Debug;

use defmt::{Format, debug};
use embassy_rp::i2c::Async;
use embassy_rp::peripherals::I2C0;
use embassy_time::Timer;

use thiserror::Error;

#[derive(Error, Debug)]
pub enum Scd4xError {
    #[error("SCD4x Checksum Failed")]
    ChecksumError,
    #[error("SCD4x Illegal Command During Measurement")]
    CommandDuringMeasurementError,
    #[error("Unsupported SCD40 Command")]
    UnsupportedScd40Command,
}

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
    Scd4xCmdPerformForcedRecalibration = 0x362F, // Not implemented
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
    Scd4xCmdPowerDown = 0x36e0, // Not implemented
    Scd4xCmdWakeUp = 0x36f6,    // Not implemented
    Scd4xCmdSetAutomaticSelfCalibrationInitialPeriod = 0x2445, // Not implemented
    Scd4xCmdGetAutomaticSelfCalibrationInitialPeriod = 0x2340, // Not implemented
    Scd4xCmdSetAutomaticSelfCalibrationStandardPeriod = 0x244e, // Not implemented
    Scd4xCmdGetAutomaticSelfCalibrationStandardPeriod = 0x234b, // Not implemented
}

pub struct Scd4x {
    address: u8,
    measurement_active: bool,
    scd40: bool,
}

#[derive(Debug, Format)]
pub struct Scd4xMeasurement {
    relative_humidity: u16,
    temperature_celcius: u16,
    co2_ppm: u16,
}

impl Scd4x {
    pub fn init(address: u8) -> Self {
        Self {
            address,
            measurement_active: false,
            scd40: true, // Assume an SCD40, as opposed to SCD41, etc.
        }
    }

    async fn send_command(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
        command: Scd4xCommand,
    ) {
        let command_bytes = (command as u16).to_be_bytes();
        _ = bus.write_async(self.address, command_bytes).await;
    }

    async fn read_response(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
        data: &mut [u8],
    ) -> Result<(), Scd4xError> {
        _ = bus.read_async(self.address, data).await;

        self.verify_checksum(data)
    }

    async fn read_sequence(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
        command: Scd4xCommand,
        data: &mut [u8],
        delay_ms: u32,
    ) -> Result<(), Scd4xError> {
        let mut raw_data: [u8; 9] = [0; 9];
        self.send_command(bus, command).await;

        if delay_ms > 0 {
            Timer::after_millis(delay_ms.into()).await;
        }

        // Read in the response and return an error if the checksum failed
        match self.read_response(bus, &mut raw_data).await {
            Ok(()) => (),
            Err(e) => return Err(e),
        };

        // Throw away the checksum bytes and return the data of interest
        for i in 0..data.len() {
            let raw_data_idx = i % 2 + (i / 2) * 3;
            data[i] = raw_data[raw_data_idx];
        }

        Ok(())
    }

    async fn write_payload(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
        command: Scd4xCommand,
        data: &[u8; 2],
        delay_ms: u32,
    ) -> Result<(), Scd4xError> {
        let mut payload: [u8; 5] = [0; 5];
        payload[..2].clone_from_slice(&(command as u16).to_be_bytes());
        payload[2..4].clone_from_slice(data);

        payload[5] = self.calculate_checksum(payload[..2].try_into().unwrap());
        _ = bus.write_async(self.address, payload).await;

        if delay_ms > 0 {
            Timer::after_millis(delay_ms.into()).await;
        }

        Ok(())
    }

    fn verify_checksum(&self, data: &[u8]) -> Result<(), Scd4xError> {
        let checksum_blocks = data.len() / 3;

        for block in 0..checksum_blocks {
            let start_idx = block * 3;
            let end_idx = block * 3 + 2;
            let expected_checksum = data[block * 3 + 2];
            let calculated_checksum =
                self.calculate_checksum(data[start_idx..end_idx].try_into().unwrap());
            if calculated_checksum != expected_checksum {
                return Err(Scd4xError::ChecksumError);
            }
        }

        Ok(())
    }

    fn calculate_checksum(&self, data: &[u8; 2]) -> u8 {
        // For input 0xBEEF.to_be_bytes(), we expect 0x92
        // let input: u16 = 0xBEEF;
        // let crc = scd40.checksum(&(input.to_be_bytes()));
        // info!("Checksum: {:?}", crc);
        const CRC8_POLYNOMIAL: u8 = 0x31;
        const CRC8_INIT_VALUE: u8 = 0xFF;

        let mut crc: u8 = CRC8_INIT_VALUE;
        for byte in data {
            crc ^= byte;
            for _ in 0..8 {
                if crc & 0x80 > 0 {
                    crc = (crc << 1) ^ CRC8_POLYNOMIAL;
                } else {
                    crc = crc << 1;
                }
            }
        }

        crc
    }

    ///////////////////////////
    // Command Only Commands //
    ///////////////////////////
    pub async fn start_periodic_measurement(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
    ) -> Result<(), Scd4xError> {
        debug!("Calling scd4x.start_periodic_measurement()");
        if self.measurement_active {
            return Err(Scd4xError::CommandDuringMeasurementError);
        }

        self.send_command(bus, Scd4xCommand::Scd4xCmdStartPeriodicMeasurement)
            .await;

        Ok(())
    }

    pub async fn start_low_power_periodic_measurement(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
    ) -> Result<(), Scd4xError> {
        debug!("Calling scd4x.start_low_power_periodic_measurement()");
        if self.measurement_active {
            return Err(Scd4xError::CommandDuringMeasurementError);
        }

        self.send_command(bus, Scd4xCommand::Scd4xCmdStartLowPowerPeriodicMeasurement)
            .await;

        Ok(())
    }

    pub async fn stop_periodic_measurement(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
    ) -> Result<(), Scd4xError> {
        debug!("Calling scd4x.stop_periodic_measurement()");

        self.send_command(bus, Scd4xCommand::Scd4xCmdStopPeriodicMeasurement)
            .await;

        Timer::after_millis(500).await;

        Ok(())
    }

    pub async fn perform_factory_reset(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
    ) -> Result<(), Scd4xError> {
        debug!("Calling scd4x.perform_factory_reset()");

        if self.measurement_active {
            return Err(Scd4xError::CommandDuringMeasurementError);
        }

        self.send_command(bus, Scd4xCommand::Scd4xCmdPerformFactoryReset)
            .await;

        Timer::after_millis(1200).await;

        Ok(())
    }

    pub async fn reinit(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
    ) -> Result<(), Scd4xError> {
        debug!("Calling scd4x.reinit()");

        if self.measurement_active {
            return Err(Scd4xError::CommandDuringMeasurementError);
        }

        self.send_command(bus, Scd4xCommand::Scd4xCmdReinit).await;

        Timer::after_millis(20).await;

        Ok(())
    }

    pub async fn measure_single_shot(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
    ) -> Result<(), Scd4xError> {
        debug!("Calling scd4x.measure_single_shot()");

        if self.measurement_active {
            return Err(Scd4xError::CommandDuringMeasurementError);
        } else if self.scd40 {
            return Err(Scd4xError::UnsupportedScd40Command);
        }

        self.send_command(bus, Scd4xCommand::Scd4xCmdMeasureSingleShot)
            .await;

        Timer::after_millis(5000).await;

        Ok(())
    }

    pub async fn measure_single_shot_rht_only(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
    ) -> Result<(), Scd4xError> {
        debug!("Calling scd4x.measure_single_shot_rht_only()");

        if self.measurement_active {
            return Err(Scd4xError::CommandDuringMeasurementError);
        } else if self.scd40 {
            return Err(Scd4xError::UnsupportedScd40Command);
        }

        self.send_command(bus, Scd4xCommand::Scd4xCmdMeasureSingleShotRhtOnly)
            .await;

        Timer::after_millis(50).await;

        Ok(())
    }

    /////////////////////////////
    // Command + Read Commands //
    /////////////////////////////
    pub async fn read_measurement(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
    ) -> Result<Scd4xMeasurement, Scd4xError> {
        debug!("Calling scd4x.read_measurement()");
        let mut measurement_bytes: [u8; 6] = [0; 6];

        self.read_sequence(
            bus,
            Scd4xCommand::Scd4xCmdReadMeasurement,
            &mut measurement_bytes,
            1,
        )
        .await
        .unwrap();

        let measurement = Scd4xMeasurement {
            relative_humidity: 100
                * u16::from_be_bytes(measurement_bytes[4..5].try_into().unwrap())
                / (1 << 16),
            temperature_celcius: 175
                * u16::from_be_bytes(measurement_bytes[2..3].try_into().unwrap())
                / (1 << 16)
                - 45,
            co2_ppm: u16::from_be_bytes(measurement_bytes[0..1].try_into().unwrap()),
        };

        debug!("Measurement: {:#?}", measurement);

        Ok(measurement)
    }

    pub async fn get_temperature_offset(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
    ) -> Result<u16, Scd4xError> {
        debug!("Calling scd4x.get_temperature_offset()");
        let mut temperature_offset_bytes: [u8; 2] = [0; 2];

        if self.measurement_active {
            return Err(Scd4xError::CommandDuringMeasurementError);
        }

        self.read_sequence(
            bus,
            Scd4xCommand::Scd4xCmdGetTemperatureOffset,
            &mut temperature_offset_bytes,
            1,
        )
        .await
        .unwrap();

        let temperature_offset = 175 * (u16::from_be_bytes(temperature_offset_bytes) / (1 << 16));
        debug!("Temperature Offset: {}°C", temperature_offset);

        Ok(temperature_offset)
    }

    pub async fn get_sensor_altitude(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
    ) -> Result<u16, Scd4xError> {
        debug!("Calling scd4x.get_sensor_altitude()");
        let mut altitude_m_bytes: [u8; 2] = [0; 2];

        if self.measurement_active {
            return Err(Scd4xError::CommandDuringMeasurementError);
        }

        self.read_sequence(
            bus,
            Scd4xCommand::Scd4xCmdGetSensorAltitude,
            &mut altitude_m_bytes,
            1,
        )
        .await
        .unwrap();

        let altitude_m = u16::from_be_bytes(altitude_m_bytes);
        debug!("Altitude: {}m", altitude_m);

        Ok(altitude_m)
    }

    pub async fn get_ambient_pressure(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
    ) -> Result<u16, Scd4xError> {
        debug!("Calling scd4x.get_ambient_pressure()");
        let mut pressure_pa_bytes: [u8; 2] = [0; 2];

        self.read_sequence(
            bus,
            Scd4xCommand::Scd4xCmdSetAmbientPressure, // This uses the same opcode for some reason
            &mut pressure_pa_bytes,
            1,
        )
        .await
        .unwrap();

        let pressure_pa = 100 * u16::from_be_bytes(pressure_pa_bytes);
        debug!("Ambient Pressure: {}pa", pressure_pa);

        Ok(pressure_pa)
    }

    pub async fn get_automatic_self_calibration_enabled(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
    ) -> Result<bool, Scd4xError> {
        debug!("Calling scd4x.get_automatic_self_calibration_enabled()");
        let mut enabled_bytes: [u8; 2] = [0; 2];

        if self.measurement_active {
            return Err(Scd4xError::CommandDuringMeasurementError);
        }

        self.read_sequence(
            bus,
            Scd4xCommand::Scd4xCmdGetAutomaticSelfCalibrationEnabled,
            &mut enabled_bytes,
            1,
        )
        .await
        .unwrap();

        let enabled = u16::from_be_bytes(enabled_bytes) > 0;
        debug!("Enabled: {}", enabled);

        Ok(enabled)
    }

    pub async fn get_data_ready_status(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
    ) -> Result<bool, Scd4xError> {
        debug!("Calling scd4x.get_data_ready_status()");
        let mut ready_bytes: [u8; 2] = [0; 2];

        self.read_sequence(
            bus,
            Scd4xCommand::Scd4xCmdGetDataReadyStatus,
            &mut ready_bytes,
            1,
        )
        .await
        .unwrap();

        let ready = (u16::from_be_bytes(ready_bytes) & 0xFFF) > 0; // If bits 11:0 are non-zero, data is waiting
        debug!("Ready: {}", ready);

        Ok(ready)
    }

    pub async fn get_serial_number(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
    ) -> Result<u64, Scd4xError> {
        debug!("Calling scd4x.get_serial_number()");
        let mut serial_number_bytes: [u8; 8] = [0; 8]; // 8 bytes for the output as u64

        if self.measurement_active {
            return Err(Scd4xError::CommandDuringMeasurementError);
        }

        self.read_sequence(
            bus,
            Scd4xCommand::Scd4xCmdGetSerialNumber,
            &mut serial_number_bytes[..6], // Only pass 6 bytes for a 48-bit serial number
            1,
        )
        .await
        .unwrap();

        let serial_number = u64::from_be_bytes(serial_number_bytes);
        debug!("Serial Number: 0x{:X}", serial_number);

        Ok(serial_number)
    }

    pub async fn perform_self_test(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
    ) -> Result<u64, Scd4xError> {
        debug!("Calling scd4x.perform_self_test()");
        let mut test_result_bytes: [u8; 8] = [0; 8]; // 8 bytes for the output as u64

        if self.measurement_active {
            return Err(Scd4xError::CommandDuringMeasurementError);
        }

        self.read_sequence(
            bus,
            Scd4xCommand::Scd4xCmdPerformSelfTest,
            &mut test_result_bytes[..6], // Only pass 6 bytes for a 48-bit serial number
            10000,
        )
        .await
        .unwrap();

        let test_result = u64::from_be_bytes(test_result_bytes);
        if test_result == 0 {
            debug!("Self-Test OK!");
        } else {
            debug!("Self-Test NG!");
        }

        Ok(test_result)
    }

    ///////////////////////////////////
    // Command + Write Only Commands //
    ///////////////////////////////////
    pub async fn set_temperature_offset(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
        offset: &u16,
    ) -> Result<(), Scd4xError> {
        debug!("Calling set_temperature_offset({})", offset);

        if self.measurement_active {
            return Err(Scd4xError::CommandDuringMeasurementError);
        }

        self.write_payload(
            bus,
            Scd4xCommand::Scd4xCmdSetTemperatureOffset,
            &offset.to_be_bytes(),
            1,
        )
        .await
    }

    pub async fn set_sensor_altitude(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
        altitude: &u16,
    ) -> Result<(), Scd4xError> {
        debug!("Calling set_sensor_altitude({})", altitude);

        if self.measurement_active {
            return Err(Scd4xError::CommandDuringMeasurementError);
        }

        self.write_payload(
            bus,
            Scd4xCommand::Scd4xCmdSetSensorAltitude,
            &altitude.to_be_bytes(),
            1,
        )
        .await
    }

    pub async fn set_ambient_pressure(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
        pressure_pa: &u16,
    ) -> Result<(), Scd4xError> {
        debug!("Calling set_ambient_pressure({})", pressure_pa);

        self.write_payload(
            bus,
            Scd4xCommand::Scd4xCmdSetAmbientPressure,
            &pressure_pa.to_be_bytes(),
            1,
        )
        .await
    }

    pub async fn set_automatic_calibration_enabled(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
        enabled: &bool,
    ) -> Result<(), Scd4xError> {
        debug!("Calling set_automatic_calibration_enabled({})", enabled);

        if self.measurement_active {
            return Err(Scd4xError::CommandDuringMeasurementError);
        }

        let data: u16 = if *enabled { 1 } else { 0 };

        self.write_payload(
            bus,
            Scd4xCommand::Scd4xCmdSetAutomaticSelfCalibrationEnabled,
            &data.to_be_bytes(),
            1,
        )
        .await
    }
}
