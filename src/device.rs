use crate::bus::{Bus, I2cBus};
use crate::calibration::{
    calibrate_pressure, calibrate_temperature, decode_i24, i24_to_f32,
    process_calibration_coefficients, CalibrationCoeffs,
};
use crate::config::{Config, ConfigError, Mode, Oversampling, Rate};
use crate::device_internal::{
    calc_total_wait_ms, cfg_reg_value, inv_scale_factor, prs_cfg_value, tmp_cfg_value, PRODUCT_ID,
    RESET_FIFO_FLUSH, RESET_SOFT_AND_FIFO_FLUSH,
};
use crate::register::Register;
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum I2cAddress {
    Primary,
    Secondary,
    Custom(u8),
}

impl I2cAddress {
    #[must_use]
    pub const fn addr(self) -> u8 {
        match self {
            Self::Primary => 0x76,
            Self::Secondary => 0x77,
            Self::Custom(v) => v,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Poll<T = ()> {
    Pending { wait_ms: u32 },
    Ready(T),
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InitStage {
    SensorReady,
    CoefficientsReady,
    TemperatureReady,
    ApplyConfig,
    Ready,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum InitPhase {
    VerifyDevice,
    WaitSensorReady,
    WaitCoefficientsReady,
    WaitTemperatureReady,
    ApplyConfig,
    Ready,
}

impl InitPhase {
    const fn stage(self) -> InitStage {
        match self {
            Self::VerifyDevice | Self::WaitSensorReady => InitStage::SensorReady,
            Self::WaitCoefficientsReady => InitStage::CoefficientsReady,
            Self::WaitTemperatureReady => InitStage::TemperatureReady,
            Self::ApplyConfig => InitStage::ApplyConfig,
            Self::Ready => InitStage::Ready,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error<I2cError> {
    Bus(I2cError),
    InvalidDevice { id: u8 },
    InvalidConfig { reason: ConfigError },
    Timeout { stage: InitStage },
    NotInitialized,
    NoSampleReady,
}

impl<I2cError: core::fmt::Display> core::fmt::Display for Error<I2cError> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Bus(err) => write!(f, "I2C bus error: {err}"),
            Self::InvalidDevice { id } => write!(f, "invalid DPS3xx product ID: 0x{id:02x}"),
            Self::InvalidConfig {
                reason: ConfigError::MeasurementBusyTimeExceeded,
            } => f.write_str("invalid config: measurement busy time exceeded"),
            Self::InvalidConfig {
                reason: ConfigError::TemperatureSourceMismatch,
            } => {
                f.write_str("invalid config: temperature source does not match factory calibration")
            }
            Self::Timeout { stage } => write!(f, "initialization timeout at stage {stage:?}"),
            Self::NotInitialized => f.write_str("sensor is not initialized"),
            Self::NoSampleReady => f.write_str("no sensor sample ready"),
        }
    }
}

impl<I2cError: core::fmt::Debug + core::fmt::Display> core::error::Error for Error<I2cError> {}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub pressure_pa: f32,
    pub temperature_c: f32,
}

impl Sample {
    /// Returns the calibrated pressure in hectopascals (hPa / mbar).
    #[must_use]
    pub fn pressure_hpa(self) -> f32 {
        self.pressure_pa * 0.01
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RawSample {
    pub pressure: i32,
    pub temperature: i32,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FifoEntry {
    Pressure(Sample),
    Temperature(f32),
}

#[expect(
    clippy::struct_excessive_bools,
    reason = "status struct mirrors the 4 readiness flags of MEAS_CFG"
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Status {
    pub coefficients_ready: bool,
    pub sensor_ready: bool,
    pub temperature_ready: bool,
    pub pressure_ready: bool,
}

impl Status {
    const fn from_bits(status: u8) -> Self {
        Self {
            coefficients_ready: (status & (1 << 7)) != 0,
            sensor_ready: (status & (1 << 6)) != 0,
            temperature_ready: (status & (1 << 5)) != 0,
            pressure_ready: (status & (1 << 4)) != 0,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InterruptStatus {
    pub fifo_full: bool,
    pub temperature_ready: bool,
    pub pressure_ready: bool,
}

impl InterruptStatus {
    const fn from_bits(bits: u8) -> Self {
        Self {
            fifo_full: (bits & (1 << 2)) != 0,
            temperature_ready: (bits & (1 << 1)) != 0,
            pressure_ready: (bits & 1) != 0,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FifoStatus {
    pub full: bool,
    pub empty: bool,
}

impl FifoStatus {
    const fn from_bits(bits: u8) -> Self {
        Self {
            full: (bits & (1 << 1)) != 0,
            empty: (bits & 1) != 0,
        }
    }
}

pub struct Dps3xx<I2C> {
    bus: I2cBus<I2C>,
    coeffs: CalibrationCoeffs,
    config: Config,
    inv_pressure_scale: f32,
    inv_temperature_scale: f32,
    last_temp_scaled: f32,
    coef_source_ext: bool,
    phase: InitPhase,
}

impl<I2C> Dps3xx<I2C> {
    #[must_use]
    pub fn release(self) -> I2C {
        self.bus.release()
    }

    #[must_use]
    pub const fn config(&self) -> Config {
        self.config
    }

    #[must_use]
    pub const fn is_ready(&self) -> bool {
        matches!(self.phase, InitPhase::Ready)
    }

    #[must_use]
    pub const fn init_stage(&self) -> InitStage {
        self.phase.stage()
    }
}

impl<I2C: I2c> Dps3xx<I2C> {
    /// Creates a new driver instance after validating the configuration.
    ///
    /// Does not perform any I2C transactions until [`Self::init_blocking`] or [`Self::poll_init`]
    /// is called.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidConfig`] if the configured measurement rates and oversampling
    /// exceed the 1-second hardware conversion budget.
    pub fn new(i2c: I2C, address: I2cAddress, config: Config) -> Result<Self, Error<I2C::Error>> {
        if let Err(reason) = config.validate() {
            return Err(Error::InvalidConfig { reason });
        }
        Ok(Self {
            bus: I2cBus::new(i2c, address.addr()),
            coeffs: CalibrationCoeffs::default(),
            config,
            inv_pressure_scale: inv_scale_factor(config.pressure.oversampling),
            inv_temperature_scale: inv_scale_factor(config.temperature.oversampling),
            last_temp_scaled: 0.0,
            coef_source_ext: false,
            phase: InitPhase::VerifyDevice,
        })
    }

    /// Alias for [`Self::new`].
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidConfig`] if the configuration is invalid.
    pub fn new_i2c(
        i2c: I2C,
        address: I2cAddress,
        config: Config,
    ) -> Result<Self, Error<I2C::Error>> {
        Self::new(i2c, address, config)
    }

    /// Creates and initializes a driver instance using a blocking [`DelayNs`] implementation.
    ///
    /// # Errors
    ///
    /// Returns an error if the configuration is invalid, the product ID does not match a `DPS3xx`
    /// sensor, an I2C transaction fails, or initialization times out.
    pub fn new_blocking<D: DelayNs>(
        i2c: I2C,
        address: I2cAddress,
        config: Config,
        delay: &mut D,
    ) -> Result<Self, Error<I2C::Error>> {
        let mut dps = Self::new(i2c, address, config)?;
        dps.init_blocking(delay)?;
        Ok(dps)
    }

    /// Initializes the sensor using a blocking [`DelayNs`] implementation.
    ///
    /// On completion, the sensor automatically enters the configured [`Mode`] (which defaults to
    /// [`Mode::BackgroundAll`]).
    ///
    /// # Errors
    ///
    /// Returns an error if the device ID is invalid, an I2C transaction fails, or initialization
    /// exceeds `config.init_timeout_ms`.
    pub fn init_blocking<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        let mut remaining = self.config.init_timeout_ms;
        loop {
            match self.poll_init()? {
                Poll::Pending { wait_ms } => {
                    if wait_ms > remaining {
                        return Err(Error::Timeout {
                            stage: self.init_stage(),
                        });
                    }
                    delay.delay_ms(wait_ms);
                    remaining -= wait_ms;
                }
                Poll::Ready(()) => return Ok(()),
            }
        }
    }

    /// Advances the non-blocking initialization state machine.
    ///
    /// When [`Poll::Pending`] is returned, wait at least `wait_ms` milliseconds before calling
    /// `poll_init` again. When [`Poll::Ready`] is returned, the sensor is initialized and running
    /// in the configured [`Mode`].
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidDevice`] if the product ID does not match,
    /// [`Error::InvalidConfig`] if the requested temperature source differs from factory
    /// calibration, or [`Error::Bus`] on I2C communication failure.
    pub fn poll_init(&mut self) -> Result<Poll<()>, Error<I2C::Error>> {
        loop {
            match self.phase {
                InitPhase::VerifyDevice => {
                    let id = self.product_id()?;
                    if (id & 0x0F) != PRODUCT_ID {
                        return Err(Error::InvalidDevice { id });
                    }
                    self.write_reg(Register::MEAS_CFG, Mode::Idle.bits())?;
                    self.phase = InitPhase::WaitSensorReady;
                }
                InitPhase::WaitSensorReady => {
                    let status = self.status()?;
                    if !status.sensor_ready {
                        return Ok(Poll::Pending { wait_ms: 12 });
                    }
                    self.phase = InitPhase::WaitCoefficientsReady;
                    if !status.coefficients_ready {
                        return Ok(Poll::Pending { wait_ms: 40 });
                    }
                    let wait_ms = self.start_temp_calibration()?;
                    return Ok(Poll::Pending { wait_ms });
                }
                InitPhase::WaitCoefficientsReady => {
                    let status = self.status()?;
                    if !status.coefficients_ready {
                        return Ok(Poll::Pending { wait_ms: 10 });
                    }
                    let wait_ms = self.start_temp_calibration()?;
                    return Ok(Poll::Pending { wait_ms });
                }
                InitPhase::WaitTemperatureReady => {
                    if !self.status()?.temperature_ready {
                        return Ok(Poll::Pending {
                            wait_ms: calc_total_wait_ms(Rate::Hz1.reg(), Oversampling::X1.reg()),
                        });
                    }
                    let raw_temp = self.read_i24(Register::TMP_B2)?;
                    self.last_temp_scaled =
                        i24_to_f32(raw_temp) * inv_scale_factor(Oversampling::X1);
                    self.phase = InitPhase::ApplyConfig;
                }
                InitPhase::ApplyConfig => {
                    let config = self.config;
                    self.write_reg(Register::MEAS_CFG, Mode::Idle.bits())?;
                    self.apply_config(&config)?;
                    if config.fifo.enable {
                        self.flush_fifo()?;
                    }
                    if config.mode != Mode::Idle {
                        self.write_reg(Register::MEAS_CFG, config.mode.bits())?;
                    }
                    self.phase = InitPhase::Ready;
                    return Ok(Poll::Ready(()));
                }
                InitPhase::Ready => return Ok(Poll::Ready(())),
            }
        }
    }

    /// Reads the sensor readiness status flags from `MEAS_CFG`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Bus`] if the I2C read fails.
    pub fn status(&mut self) -> Result<Status, Error<I2C::Error>> {
        Ok(Status::from_bits(self.read_status()?))
    }

    /// Reads and clears the hardware interrupt status flags from `INT_STS` (`0x0A`).
    ///
    /// # Errors
    ///
    /// Returns [`Error::Bus`] if the I2C read fails.
    pub fn interrupt_status(&mut self) -> Result<InterruptStatus, Error<I2C::Error>> {
        Ok(InterruptStatus::from_bits(
            self.read_reg(Register::INT_STS)?,
        ))
    }

    /// Reads the FIFO buffer status flags from `FIFO_STS` (`0x0B`).
    ///
    /// # Errors
    ///
    /// Returns [`Error::Bus`] if the I2C read fails.
    pub fn fifo_status(&mut self) -> Result<FifoStatus, Error<I2C::Error>> {
        Ok(FifoStatus::from_bits(self.read_reg(Register::FIFO_STS)?))
    }

    /// Flushes all entries from the hardware FIFO buffer (`RESET.FIFO_FLUSH`).
    ///
    /// # Errors
    ///
    /// Returns [`Error::Bus`] if the I2C write fails.
    pub fn flush_fifo(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_reg(Register::RESET, RESET_FIFO_FLUSH)
    }

    /// Reads the product and revision ID from `PROD_ID` (`0x0D`).
    ///
    /// # Errors
    ///
    /// Returns [`Error::Bus`] if the I2C read fails.
    pub fn product_id(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_reg(Register::PROD_ID)
    }

    /// Issues a soft reset and FIFO flush to the sensor, returning the driver to the
    /// uninitialized state so [`Self::init_blocking`] or [`Self::poll_init`] must be run again.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Bus`] if the I2C write fails.
    pub fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_reg(Register::RESET, RESET_SOFT_AND_FIFO_FLUSH)?;
        self.phase = InitPhase::VerifyDevice;
        Ok(())
    }

    /// Sets the sensor operating mode (`MEAS_CFG` bits `2:0`).
    ///
    /// Revalidates the conversion budget for the requested mode before writing to the device.
    ///
    /// # Errors
    ///
    /// Returns [`Error::NotInitialized`] if the sensor has not finished initialization, or
    /// [`Error::InvalidConfig`] if the requested mode exceeds the conversion budget, or
    /// [`Error::Bus`] if the I2C write fails.
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<I2C::Error>> {
        self.ensure_ready()?;
        self.config
            .mode(mode)
            .validate()
            .map_err(|reason| Error::InvalidConfig { reason })?;
        self.write_reg(Register::MEAS_CFG, mode.bits())?;
        self.config.mode = mode;
        Ok(())
    }

    /// Starts continuous background pressure and temperature measurements ([`Mode::BackgroundAll`]).
    ///
    /// # Errors
    ///
    /// Returns [`Error::NotInitialized`] if the sensor has not finished initialization, or
    /// [`Error::Bus`] if the I2C write fails.
    pub fn start_background(&mut self) -> Result<(), Error<I2C::Error>> {
        self.set_mode(Mode::BackgroundAll)
    }

    /// Places the sensor in idle/standby mode ([`Mode::Idle`]).
    ///
    /// # Errors
    ///
    /// Returns [`Error::NotInitialized`] if the sensor has not finished initialization, or
    /// [`Error::Bus`] if the I2C write fails.
    pub fn standby(&mut self) -> Result<(), Error<I2C::Error>> {
        self.set_mode(Mode::Idle)
    }

    /// Updates the sensor configuration at runtime.
    ///
    /// Validates the new configuration, places the sensor in standby, writes the updated
    /// configuration registers, flushes the FIFO if enabled, and enters the new `config.mode`.
    ///
    /// If the initial standby write fails, the previous configuration and readiness state remain
    /// intact. If a subsequent register write fails after entering standby, the driver transitions
    /// to [`InitStage::ApplyConfig`] (not ready) so [`Self::poll_init`] or [`Self::reconfigure`]
    /// can safely retry without reading corrupted samples.
    ///
    /// # Errors
    ///
    /// Returns [`Error::NotInitialized`] if not initialized, [`Error::InvalidConfig`] if the
    /// new configuration exceeds the measurement busy-time budget or selects a temperature
    /// sensor without matching factory calibration, or [`Error::Bus`] on I2C error.
    pub fn reconfigure(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        if !matches!(self.phase, InitPhase::Ready | InitPhase::ApplyConfig) {
            return Err(Error::NotInitialized);
        }
        if let Err(reason) = config.validate() {
            return Err(Error::InvalidConfig { reason });
        }

        config
            .validate_temperature_source(self.coef_source_ext)
            .map_err(|reason| Error::InvalidConfig { reason })?;
        self.write_reg(Register::MEAS_CFG, Mode::Idle.bits())?;
        self.phase = InitPhase::ApplyConfig;

        self.apply_config(&config)?;
        if config.fifo.enable {
            self.flush_fifo()?;
        }
        if config.mode != Mode::Idle {
            self.write_reg(Register::MEAS_CFG, config.mode.bits())?;
        }

        self.config = config;
        self.inv_pressure_scale = inv_scale_factor(config.pressure.oversampling);
        self.inv_temperature_scale = inv_scale_factor(config.temperature.oversampling);
        self.phase = InitPhase::Ready;
        Ok(())
    }

    /// Checks whether a new pressure sample is ready and returns the calibrated [`Sample`].
    ///
    /// Temperature is tracked independently using the latest available temperature measurement:
    /// - If both `pressure_ready` and `temperature_ready` are set, reads all 6 bytes (`0x00..0x05`)
    ///   in a single burst, updates the cached temperature, and returns the compensated [`Sample`].
    /// - If only `pressure_ready` is set (common when running pressure at a higher rate than
    ///   temperature), reads only the 3 pressure bytes (`0x00..0x02`) and compensates using the
    ///   most recent temperature reading.
    /// - If only `temperature_ready` is set, reads the 3 temperature bytes (`0x03..0x05`), updates
    ///   the cached temperature, and returns `Ok(None)`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::NotInitialized`] if the sensor has not been initialized, or [`Error::Bus`]
    /// on I2C communication failure.
    pub fn try_read_sample(&mut self) -> Result<Option<Sample>, Error<I2C::Error>> {
        self.ensure_ready()?;
        let status = self.status()?;

        if status.pressure_ready && status.temperature_ready {
            let raw = self.read_raw_sample()?;
            self.last_temp_scaled = i24_to_f32(raw.temperature) * self.inv_temperature_scale;
            let pres_scaled = i24_to_f32(raw.pressure) * self.inv_pressure_scale;
            return Ok(Some(self.compensate_pressure(pres_scaled)));
        }

        if status.pressure_ready {
            let raw_pressure = self.read_i24(Register::PSR_B2)?;
            let pres_scaled = i24_to_f32(raw_pressure) * self.inv_pressure_scale;
            return Ok(Some(self.compensate_pressure(pres_scaled)));
        }

        if status.temperature_ready {
            let raw_temp = self.read_i24(Register::TMP_B2)?;
            self.last_temp_scaled = i24_to_f32(raw_temp) * self.inv_temperature_scale;
        }

        Ok(None)
    }

    /// Reads a calibrated [`Sample`] if ready, or returns [`Error::NoSampleReady`].
    ///
    /// # Errors
    ///
    /// Returns [`Error::NoSampleReady`] if no pressure sample is ready, [`Error::NotInitialized`]
    /// if the sensor has not been initialized, or [`Error::Bus`] on I2C communication failure.
    pub fn read_sample(&mut self) -> Result<Sample, Error<I2C::Error>> {
        self.try_read_sample()?.ok_or(Error::NoSampleReady)
    }

    /// Returns a new calibrated temperature in degrees Celsius, if available.
    ///
    /// Reads only the temperature registers and updates the temperature cache used for
    /// pressure compensation. Does not consume a pending pressure measurement.
    /// With FIFO enabled, use [`Self::try_pop_fifo`] instead.
    ///
    /// # Errors
    ///
    /// Returns [`Error::NotInitialized`] before initialization, or [`Error::Bus`] on I2C failure.
    pub fn try_read_temperature(&mut self) -> Result<Option<f32>, Error<I2C::Error>> {
        self.ensure_ready()?;
        if !self.status()?.temperature_ready {
            return Ok(None);
        }
        let raw = self.read_i24(Register::TMP_B2)?;
        self.last_temp_scaled = i24_to_f32(raw) * self.inv_temperature_scale;
        Ok(Some(calibrate_temperature(
            &self.coeffs,
            self.last_temp_scaled,
        )))
    }

    /// Reads only the signed 24-bit pressure registers (`0x00..0x02`).
    ///
    /// Does not check readiness or consume a pending temperature measurement. Reading clears
    /// pressure readiness; call [`Self::status`] first if freshness matters.
    /// With FIFO enabled, use [`Self::try_pop_fifo`] to decode tagged entries instead.
    ///
    /// # Errors
    ///
    /// Returns [`Error::NotInitialized`] before initialization, or [`Error::Bus`] on I2C failure.
    pub fn read_pressure_raw(&mut self) -> Result<i32, Error<I2C::Error>> {
        self.ensure_ready()?;
        self.read_i24(Register::PSR_B2)
    }

    /// Burst-reads the raw 24-bit pressure and temperature registers (`0x00..0x05`).
    ///
    /// # Errors
    ///
    /// Returns [`Error::NotInitialized`] if the sensor has not been initialized, or [`Error::Bus`]
    /// on I2C communication failure.
    pub fn read_raw_sample(&mut self) -> Result<RawSample, Error<I2C::Error>> {
        self.ensure_ready()?;
        let mut bytes = [0_u8; 6];
        self.bus
            .read_many(Register::PSR_B2, &mut bytes)
            .map_err(Error::Bus)?;

        Ok(RawSample {
            pressure: decode_i24([bytes[0], bytes[1], bytes[2]]),
            temperature: decode_i24([bytes[3], bytes[4], bytes[5]]),
        })
    }

    /// Pops and decodes a single entry from the hardware FIFO if the FIFO is not empty.
    ///
    /// In FIFO mode, both temperature and pressure measurements are pushed into the FIFO at
    /// `0x00..0x02` and tagged via bit 0 (`0` = temperature, `1` = pressure).
    /// Temperature entries automatically update the internal temperature reference used to
    /// compensate subsequent pressure entries.
    ///
    /// # Errors
    ///
    /// Returns [`Error::NotInitialized`] if the sensor has not been initialized, or [`Error::Bus`]
    /// on I2C communication failure.
    pub fn try_pop_fifo(&mut self) -> Result<Option<FifoEntry>, Error<I2C::Error>> {
        self.ensure_ready()?;
        if self.fifo_status()?.empty {
            return Ok(None);
        }

        let mut bytes = [0_u8; 3];
        self.bus
            .read_many(Register::PSR_B2, &mut bytes)
            .map_err(Error::Bus)?;

        let is_pressure = (bytes[2] & 0x01) != 0;
        let raw = decode_i24(bytes);

        if is_pressure {
            let pres_scaled = i24_to_f32(raw) * self.inv_pressure_scale;
            Ok(Some(FifoEntry::Pressure(
                self.compensate_pressure(pres_scaled),
            )))
        } else {
            self.last_temp_scaled = i24_to_f32(raw) * self.inv_temperature_scale;
            let temp_c = calibrate_temperature(&self.coeffs, self.last_temp_scaled);
            Ok(Some(FifoEntry::Temperature(temp_c)))
        }
    }

    const fn ensure_ready(&self) -> Result<(), Error<I2C::Error>> {
        if self.is_ready() {
            Ok(())
        } else {
            Err(Error::NotInitialized)
        }
    }

    fn compensate_pressure(&self, pres_scaled: f32) -> Sample {
        Sample {
            pressure_pa: calibrate_pressure(&self.coeffs, pres_scaled, self.last_temp_scaled),
            temperature_c: calibrate_temperature(&self.coeffs, self.last_temp_scaled),
        }
    }

    fn start_temp_calibration(&mut self) -> Result<u32, Error<I2C::Error>> {
        self.read_coefficients()?;
        self.coef_source_ext = (self.read_reg(Register::TMP_COEF_SRCE)? & 0x80) != 0;
        self.config
            .validate_temperature_source(self.coef_source_ext)
            .map_err(|reason| Error::InvalidConfig { reason })?;

        // The MCU may have restarted while the sensor stayed powered. Disable FIFO and
        // both result shifts before the X1 priming conversion, and discard stale results.
        self.write_reg(Register::CFG_REG, 0)?;
        self.flush_fifo()?;
        let _ = self.read_i24(Register::TMP_B2)?;

        let fast_temp_cfg = if self.coef_source_ext { 0x80 } else { 0x00 };
        self.write_reg(Register::TEMP_CFG, fast_temp_cfg)?;
        self.apply_temp_workaround_registers()?;
        self.write_reg(Register::MEAS_CFG, Mode::CommandTemperature.bits())?;
        self.phase = InitPhase::WaitTemperatureReady;
        Ok(calc_total_wait_ms(Rate::Hz1.reg(), Oversampling::X1.reg()))
    }

    fn read_status(&mut self) -> Result<u8, Error<I2C::Error>> {
        Ok(self.read_reg(Register::MEAS_CFG)? & 0xF0)
    }

    fn write_reg(&mut self, reg: Register, value: u8) -> Result<(), Error<I2C::Error>> {
        self.bus.write_reg(reg, value).map_err(Error::Bus)
    }

    fn write_addr(&mut self, addr: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.bus.write_addr(addr, value).map_err(Error::Bus)
    }

    fn read_reg(&mut self, reg: Register) -> Result<u8, Error<I2C::Error>> {
        self.bus.read_reg(reg).map_err(Error::Bus)
    }

    fn read_i24(&mut self, reg: Register) -> Result<i32, Error<I2C::Error>> {
        let mut bytes = [0_u8; 3];
        self.bus.read_many(reg, &mut bytes).map_err(Error::Bus)?;
        Ok(decode_i24(bytes))
    }

    fn apply_temp_workaround_registers(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_addr(0x0E, 0xA5)?;
        self.write_addr(0x0F, 0x96)?;
        self.write_addr(0x62, 0x02)?;
        self.write_addr(0x0E, 0x00)?;
        self.write_addr(0x0F, 0x00)?;
        Ok(())
    }

    fn apply_config(&mut self, config: &Config) -> Result<(), Error<I2C::Error>> {
        self.write_reg(Register::PRS_CFG, prs_cfg_value(config))?;
        self.write_reg(
            Register::TEMP_CFG,
            tmp_cfg_value(config, self.coef_source_ext),
        )?;

        let temp_shift = config.temperature.oversampling.reg() > Oversampling::X8.reg();
        let pres_shift = config.pressure.oversampling.reg() > Oversampling::X8.reg();
        self.write_reg(
            Register::CFG_REG,
            cfg_reg_value(config, temp_shift, pres_shift),
        )?;

        Ok(())
    }

    fn read_coefficients(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut bytes = [0_u8; 18];
        self.bus
            .read_many(Register::COEFF_REG_1, &mut bytes)
            .map_err(Error::Bus)?;
        process_calibration_coefficients(&mut self.coeffs, &bytes);
        Ok(())
    }
}
