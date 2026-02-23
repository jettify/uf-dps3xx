use crate::bus::{Bus, I2cBus};
use crate::calibration::{
    calibrate_pressure, get_twos_complement, process_calibration_coefficients, CalibrationCoeffs,
};
use crate::config::{Config, PressureResolution, TemperatureResolution};
use crate::device_internal::{
    cfg_reg_value, prs_cfg_value, tmp_cfg_value, PRODUCT_ID, SCALE_FACTORS,
};
use crate::register::Register;
use core::marker::PhantomData;
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

pub use crate::device_internal::{
    calc_busy_time_ms, calc_busy_time_units, calc_total_wait_ms, BUSYTIME_FAILSAFE_MS,
    BUSYTIME_SCALING, MAX_BUSYTIME_UNITS,
};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Unconfigured;
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Configured;
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Calibrated;
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct InitInProgress;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InitPoll {
    Pending(u32),
    Ready,
}

pub trait IsConfigured {}
impl IsConfigured for Configured {}
impl IsConfigured for Calibrated {}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub enum Error<I2CError> {
    /// I2C Interface Error
    I2CError(I2CError),
    InvalidProductId,
    BusyTimeExceeded,
    CoefficientsNotReady,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MeasurementMode {
    OneShotPressure,
    OneShotTemperature,
    BackgroundPressure,
    BackgroundTemperature,
    BackgroundPressureAndTemperature,
}

impl MeasurementMode {
    const fn meas_ctrl(self) -> u8 {
        match self {
            Self::OneShotPressure => 0b001,
            Self::OneShotTemperature => 0b010,
            Self::BackgroundPressure => 0b101,
            Self::BackgroundTemperature => 0b110,
            Self::BackgroundPressureAndTemperature => 0b111,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Status {
    pub coef_ready: bool,
    pub init_complete: bool,
    pub temp_ready: bool,
    pub pres_ready: bool,
}

impl Status {
    fn from_bits(status: u8) -> Self {
        Self {
            coef_ready: (status & (1 << 7)) != 0,
            init_complete: (status & (1 << 6)) != 0,
            temp_ready: (status & (1 << 5)) != 0,
            pres_ready: (status & (1 << 4)) != 0,
        }
    }
}

impl<I2CError> From<I2CError> for Error<I2CError> {
    fn from(err: I2CError) -> Self {
        Error::I2CError(err)
    }
}

pub struct DPS3xx<I2C, S> {
    bus: I2cBus<I2C>,
    coeffs: CalibrationCoeffs,
    config: Config,
    init_ready: bool,
    init_temp_started: bool,
    _state: PhantomData<S>,
}

impl<I2C, I2CError> DPS3xx<I2C, Unconfigured>
where
    I2C: I2c<Error = I2CError>,
{
    pub fn new(i2c: I2C, address: u8, config: &Config) -> Result<Self, Error<I2CError>> {
        let dps3xx = Self {
            bus: I2cBus::new(i2c, address),
            coeffs: CalibrationCoeffs::default(),
            config: *config,
            init_ready: false,
            init_temp_started: false,
            _state: PhantomData,
        };
        Ok(dps3xx)
    }

    pub fn start_init(mut self) -> Result<DPS3xx<I2C, InitInProgress>, Error<I2CError>> {
        let id = self.get_product_id()?;
        if (id & 0xF0) != (PRODUCT_ID & 0xF0) {
            return Err(Error::InvalidProductId);
        }
        self.apply_config()?;
        self.standby()?;

        self.apply_temp_workaround_registers()?;

        self.init_ready = false;
        self.init_temp_started = false;

        Ok(self.into_state())
    }

    pub fn init_and_calibrate<D>(
        self,
        delay: &mut D,
    ) -> Result<DPS3xx<I2C, Calibrated>, Error<I2CError>>
    where
        D: DelayNs,
    {
        let mut dps = self.start_init()?;

        let mut dps = loop {
            match dps.poll_init()? {
                InitPoll::Pending(wait_ms) => delay.delay_ms(wait_ms),
                InitPoll::Ready => match dps.finish_init() {
                    Ok(dps) => break dps,
                    Err(unfinished) => dps = unfinished,
                },
            }
        };

        while !dps.coef_ready()? {
            delay.delay_ms(10);
        }

        dps.read_calibration_coefficients_unchecked()
    }
}

impl<I2C, I2CError> DPS3xx<I2C, InitInProgress>
where
    I2C: I2c<Error = I2CError>,
{
    fn init_wait_ms(&self) -> u32 {
        calc_total_wait_ms(
            self.config.temp_rate.unwrap_or_default() as u8,
            self.config.temp_res.unwrap_or_default() as u8,
        )
    }

    pub fn poll_init(&mut self) -> Result<InitPoll, Error<I2CError>> {
        if self.init_ready {
            return Ok(InitPoll::Ready);
        }

        let status = self.status()?;
        if !status.init_complete {
            return Ok(InitPoll::Pending(self.init_wait_ms()));
        }

        if !self.init_temp_started {
            self.write_reg(
                Register::MEAS_CFG,
                MeasurementMode::OneShotTemperature.meas_ctrl(),
            )?;
            self.init_temp_started = true;
            return Ok(InitPoll::Pending(self.init_wait_ms()));
        }

        if !status.temp_ready {
            return Ok(InitPoll::Pending(self.init_wait_ms()));
        }

        let _ = self.read_i24(Register::TMP_B2)?;
        self.standby()?;
        self.init_ready = true;
        self.init_temp_started = false;
        Ok(InitPoll::Ready)
    }

    pub fn finish_init(self) -> Result<DPS3xx<I2C, Configured>, Self> {
        if self.init_ready {
            Ok(self.into_state())
        } else {
            Err(self)
        }
    }
}

impl<I2C, I2CError> DPS3xx<I2C, Configured>
where
    I2C: I2c<Error = I2CError>,
{
    /// Read calibration coefficients. User must wait for `Self::coef_ready()` to return true before reading coefficients.
    ///
    /// Taken from official Arduino library, see <https://github.com/Infineon/DPS3xx-Pressure-Sensor/blob/888200c7efd8edb19ce69a2144e28ba31cdad449/src/Dps310.cpp#L89>
    ///
    /// See Sec 8.11
    pub fn read_calibration_coefficients(
        mut self,
    ) -> Result<DPS3xx<I2C, Calibrated>, Error<I2CError>> {
        if !self.coef_ready()? {
            return Err(Error::CoefficientsNotReady);
        }

        self.read_calibration_coefficients_unchecked()
    }

    fn read_calibration_coefficients_unchecked(
        mut self,
    ) -> Result<DPS3xx<I2C, Calibrated>, Error<I2CError>> {
        let mut bytes: [u8; 18] = [0; 18];
        self.bus.read_many(Register::COEFF_REG_1, &mut bytes)?;

        process_calibration_coefficients(&mut self.coeffs, &mut bytes);

        Ok(self.into_state())
    }
}

impl<I2C, I2CError, S> DPS3xx<I2C, S>
where
    I2C: I2c<Error = I2CError>,
{
    fn max_busy_time_exceeded(&self, mode: MeasurementMode) -> bool {
        let temp_rate = self.config.temp_rate.unwrap_or_default() as u8;
        let temp_res = self.config.temp_res.unwrap_or_default() as u8;
        let pres_rate = self.config.pres_rate.unwrap_or_default() as u8;
        let pres_res = self.config.pres_res.unwrap_or_default() as u8;

        match mode {
            MeasurementMode::BackgroundPressure => {
                calc_busy_time_units(pres_rate, pres_res) >= MAX_BUSYTIME_UNITS
            }
            MeasurementMode::BackgroundTemperature => {
                calc_busy_time_units(temp_rate, temp_res) >= MAX_BUSYTIME_UNITS
            }
            MeasurementMode::BackgroundPressureAndTemperature => {
                calc_busy_time_units(temp_rate, temp_res)
                    + calc_busy_time_units(pres_rate, pres_res)
                    >= MAX_BUSYTIME_UNITS
            }
            MeasurementMode::OneShotPressure | MeasurementMode::OneShotTemperature => false,
        }
    }

    pub fn release(self) -> I2C {
        self.bus.release()
    }

    pub fn status(&mut self) -> Result<Status, Error<I2CError>> {
        let status = self.read_status()?;
        Ok(Status::from_bits(status))
    }

    /// Read status bits from MEAS_CFG reg.
    /// MEAS_CFG register is masked with 0xF0
    pub fn read_status(&mut self) -> Result<u8, Error<I2CError>> {
        let meas_cfg = self.read_reg(Register::MEAS_CFG)?;
        Ok(meas_cfg & 0xF0)
    }

    /// Returns true if sensor coeficients are available
    pub fn coef_ready(&mut self) -> Result<bool, Error<I2CError>> {
        Ok(self.status()?.coef_ready)
    }

    /// Returns true if sensor initialized and ready to take measurements
    pub fn init_complete(&mut self) -> Result<bool, Error<I2CError>> {
        Ok(self.status()?.init_complete)
    }

    /// Returns true if temperature measurement is ready
    pub fn temp_ready(&mut self) -> Result<bool, Error<I2CError>> {
        Ok(self.status()?.temp_ready)
    }

    /// Returns true if pressure measurement is ready
    pub fn pres_ready(&mut self) -> Result<bool, Error<I2CError>> {
        Ok(self.status()?.pres_ready)
    }
}

impl<I2C, I2CError, S> DPS3xx<I2C, S>
where
    I2C: I2c<Error = I2CError>,
    S: IsConfigured,
{
    pub fn start_measurement(&mut self, mode: MeasurementMode) -> Result<(), Error<I2CError>> {
        if self.max_busy_time_exceeded(mode) {
            return Err(Error::BusyTimeExceeded);
        }

        let mut meas_cfg: u8 = self.read_reg(Register::MEAS_CFG)?;
        meas_cfg = (meas_cfg & 0xF8) | mode.meas_ctrl();

        self.write_reg(Register::MEAS_CFG, meas_cfg)
    }

    /// Read raw temperature contents
    pub fn read_temp_raw(&mut self) -> Result<i32, Error<I2CError>> {
        self.read_i24(Register::TMP_B2)
    }

    /// See section 4.9.2:
    fn read_temp_scaled(&mut self) -> Result<f32, Error<I2CError>> {
        let temp_cfg = self.read_reg(Register::TEMP_CFG)?;
        let osr = (temp_cfg & 0x07) as usize;
        let raw_sc: f32 = self.read_temp_raw()? as f32 / SCALE_FACTORS[osr];
        Ok(raw_sc)
    }

    /// Read raw pressure contents
    pub fn read_pressure_raw(&mut self) -> Result<i32, Error<I2CError>> {
        self.read_i24(Register::PSR_B2)
    }

    fn read_pressure_scaled(&mut self) -> Result<f32, Error<I2CError>> {
        let prs_cfg = self.read_reg(Register::PRS_CFG)?;
        let osr = (prs_cfg & 0x07) as usize;
        let pres_raw = self.read_pressure_raw()?;
        let pres_scaled = pres_raw as f32 / SCALE_FACTORS[osr];

        Ok(pres_scaled)
    }
}

impl<I2C, I2CError> DPS3xx<I2C, Calibrated>
where
    I2C: I2c<Error = I2CError>,
{
    /// Read calibrated temperature data in degrees Celsius.
    ///
    /// This method uses the pre calculated constants based on the calibration coefficients
    /// which have to be initialized with [Self::read_calibration_coefficients()] beforehand.
    ///
    /// See section 4.9.2 in the datasheet (formula), Sec 8.11 (coefficients)
    pub fn read_temp_calibrated(&mut self) -> Result<f32, Error<I2CError>> {
        let scaled = self.read_temp_scaled()?;
        Ok((self.coeffs.C0 as f32 * 0.5) + (self.coeffs.C1 as f32 * scaled))
    }

    pub fn try_read_temp_calibrated(&mut self) -> nb::Result<f32, Error<I2CError>> {
        if !self.temp_ready()? {
            return Err(nb::Error::WouldBlock);
        }
        self.read_temp_calibrated().map_err(nb::Error::Other)
    }

    /// Read calibrated pressure data in Pa.
    ///
    /// This method uses the pre calculated constants based on the calibration coefficients
    /// which have to be initialized with [Self::read_calibration_coefficients()] beforehand.
    ///
    /// See section 8.11 in the datasheet.
    /// See section 4.9.1 for calculation method.
    pub fn read_pressure_calibrated(&mut self) -> Result<f32, Error<I2CError>> {
        let pres_scaled = self.read_pressure_scaled()?;
        let temp_scaled = self.read_temp_scaled()?;
        let pres_cal = calibrate_pressure(&self.coeffs, pres_scaled, temp_scaled);
        Ok(pres_cal)
    }

    pub fn try_read_pressure_calibrated(&mut self) -> nb::Result<f32, Error<I2CError>> {
        if !self.pres_ready()? {
            return Err(nb::Error::WouldBlock);
        }
        self.read_pressure_calibrated().map_err(nb::Error::Other)
    }
}

impl<I2C, S, I2CError> DPS3xx<I2C, S>
where
    I2C: I2c<Error = I2CError>,
{
    fn apply_config(&mut self) -> Result<(), Error<I2CError>> {
        let config = self.config;
        let prs_cfg = self.read_reg(Register::PRS_CFG)?;

        let new_prs_cfg = prs_cfg_value(prs_cfg, &config);

        self.write_reg(Register::PRS_CFG, new_prs_cfg)?;

        let temp_res = config.temp_res.unwrap_or_default() as u8;
        let pressure_res = config.pres_res.unwrap_or_default() as u8;

        let temp_cfg = self.read_reg(Register::TEMP_CFG)?;
        let coef_source = if config.temp_ext.is_none() {
            Some((self.read_reg(Register::TMP_COEF_SRCE)? & 0x80) != 0)
        } else {
            None
        };
        let new_temp_cfg = tmp_cfg_value(temp_cfg, &config, coef_source);
        self.write_reg(Register::TEMP_CFG, new_temp_cfg)?;

        let temp_shift = config.temp_shift || temp_res > TemperatureResolution::_8_SAMPLES as u8;
        let pres_shift = config.pres_shift || pressure_res > PressureResolution::_8_SAMPLES as u8;

        let cfg = cfg_reg_value(&config, temp_shift, pres_shift);

        self.write_reg(Register::CFG_REG, cfg)?;

        Ok(())
    }

    /// Set measurement mode to `idle`
    fn standby(&mut self) -> Result<(), Error<I2CError>> {
        self.write_reg(Register::MEAS_CFG, 0)
    }

    /// Returns the product ID from PROD_ID register.
    /// This value is expected to be 0x10
    pub fn get_product_id(&mut self) -> Result<u8, Error<I2CError>> {
        self.read_reg(Register::PROD_ID)
    }

    /// Issue a full reset and fifo flush
    pub fn reset(mut self) -> Result<DPS3xx<I2C, Unconfigured>, Error<I2CError>> {
        self.write_reg(Register::RESET, 0b10001001)?;
        self.init_ready = false;
        self.init_temp_started = false;

        Ok(self.into_state())
    }

    fn write_reg(&mut self, reg: Register, value: u8) -> Result<(), Error<I2CError>> {
        self.bus.write_reg(reg, value)?;
        Ok(())
    }

    fn read_reg(&mut self, reg: Register) -> Result<u8, Error<I2CError>> {
        Ok(self.bus.read_reg(reg)?)
    }

    fn write_addr(&mut self, addr: u8, value: u8) -> Result<(), Error<I2CError>> {
        self.bus.write_addr(addr, value)?;
        Ok(())
    }
    /// Taken from official Arduino library.
    // Fix IC with a fuse bit problem, which lead to a wrong temperature
    // Should not affect ICs without this problem
    fn apply_temp_workaround_registers(&mut self) -> Result<(), Error<I2CError>> {
        self.write_addr(0x0E, 0xA5)?;
        self.write_addr(0x0F, 0x96)?;
        self.write_addr(0x62, 0x02)?;
        self.write_addr(0x0E, 0x00)?;
        self.write_addr(0x0F, 0x00)?;
        Ok(())
    }

    fn into_state<T>(self) -> DPS3xx<I2C, T> {
        DPS3xx {
            bus: self.bus,
            coeffs: self.coeffs,
            config: self.config,
            init_ready: self.init_ready,
            init_temp_started: self.init_temp_started,
            _state: PhantomData,
        }
    }

    fn read_i24(&mut self, reg: Register) -> Result<i32, Error<I2CError>> {
        let mut bytes: [u8; 3] = [0, 0, 0];
        self.bus.read_many(reg, &mut bytes)?;
        let value = ((bytes[0] as u32) << 16) | ((bytes[1] as u32) << 8) | (bytes[2] as u32);
        Ok(get_twos_complement(value, 24))
    }
}
