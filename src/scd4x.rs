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
    StartPeriodicMeasurement = 0x21B1,
    ReadMeasurement = 0xEC05,
    StopPeriodicMeasurement = 0x3F86,

    // On-chip Output Signal Compensation
    SetTemperatureOffset = 0x241D,
    GetTemperatureOffset = 0x2318,
    SetSensorAltitude = 0x2427,
    GetSensorAltitude = 0x2322,
    SetAmbientPressure = 0xE000,

    // Field Calibration
    PerformForcedRecalibration = 0x362F, // Not implemented
    SetAutomaticSelfCalibrationEnabled = 0x2416,
    GetAutomaticSelfCalibrationEnabled = 0x2313,

    // Low Power
    StartLowPowerPeriodicMeasurement = 0x21AC,
    GetDataReadyStatus = 0xE4B8,

    // Advanced Features
    PersistSettings = 0x3615,
    GetSerialNumber = 0x3682,
    PerformSelfTest = 0x3639,
    PerformFactoryReset = 0x3632,
    Reinit = 0x3646,

    // Low Power Single Shot (SCD41 only)
    MeasureSingleShot = 0x219D,
    MeasureSingleShotRhtOnly = 0x2196,
    PowerDown = 0x36e0,                                 // Not implemented
    WakeUp = 0x36f6,                                    // Not implemented
    SetAutomaticSelfCalibrationInitialPeriod = 0x2445,  // Not implemented
    GetAutomaticSelfCalibrationInitialPeriod = 0x2340,  // Not implemented
    SetAutomaticSelfCalibrationStandardPeriod = 0x244e, // Not implemented
    GetAutomaticSelfCalibrationStandardPeriod = 0x234b, // Not implemented
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
        for (i, item) in data.iter_mut().enumerate() {
            let raw_data_idx = i % 2 + (i / 2) * 3;
            *item = raw_data[raw_data_idx];
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

        payload[4] = Self::calculate_checksum(payload[..2].try_into().unwrap());
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
                Self::calculate_checksum(data[start_idx..end_idx].try_into().unwrap());
            if calculated_checksum != expected_checksum {
                return Err(Scd4xError::ChecksumError);
            }
        }

        Ok(())
    }

    fn calculate_checksum(data: &[u8; 2]) -> u8 {
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
                    crc <<= 1;
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

        self.send_command(bus, Scd4xCommand::StartPeriodicMeasurement)
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

        self.send_command(bus, Scd4xCommand::StartLowPowerPeriodicMeasurement)
            .await;

        Ok(())
    }

    pub async fn stop_periodic_measurement(
        &self,
        bus: &mut embassy_rp::i2c::I2c<'static, I2C0, Async>,
    ) -> Result<(), Scd4xError> {
        debug!("Calling scd4x.stop_periodic_measurement()");

        self.send_command(bus, Scd4xCommand::StopPeriodicMeasurement)
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

        self.send_command(bus, Scd4xCommand::PerformFactoryReset)
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

        self.send_command(bus, Scd4xCommand::Reinit).await;

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

        self.send_command(bus, Scd4xCommand::MeasureSingleShot)
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

        self.send_command(bus, Scd4xCommand::MeasureSingleShotRhtOnly)
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
            Scd4xCommand::ReadMeasurement,
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
            Scd4xCommand::GetTemperatureOffset,
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
            Scd4xCommand::GetSensorAltitude,
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
            Scd4xCommand::SetAmbientPressure, // This uses the same opcode for some reason
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
            Scd4xCommand::GetAutomaticSelfCalibrationEnabled,
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

        self.read_sequence(bus, Scd4xCommand::GetDataReadyStatus, &mut ready_bytes, 1)
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
            Scd4xCommand::GetSerialNumber,
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
            Scd4xCommand::PerformSelfTest,
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
            Scd4xCommand::SetTemperatureOffset,
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
            Scd4xCommand::SetSensorAltitude,
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
            Scd4xCommand::SetAmbientPressure,
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

        let data: u16 = u16::from(*enabled);

        self.write_payload(
            bus,
            Scd4xCommand::SetAutomaticSelfCalibrationEnabled,
            &data.to_be_bytes(),
            1,
        )
        .await
    }
}
