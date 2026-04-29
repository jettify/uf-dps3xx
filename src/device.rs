use crate::bus::{Bus, I2cBus};
use crate::calibration::{
    calibrate_pressure, get_twos_complement, process_calibration_coefficients, CalibrationCoeffs,
};
use crate::config::{Config, Oversampling};
use crate::device_internal::{
    calc_busy_time_units, calc_total_wait_ms, cfg_reg_value, prs_cfg_value, scale_factor,
    tmp_cfg_value, MAX_BUSYTIME_UNITS, PRODUCT_ID,
};
use crate::register::Register;
use core::marker::PhantomData;
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Uninit;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Ready;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum I2cAddress {
    Primary,
    Secondary,
    Custom(u8),
}

impl I2cAddress {
    const fn addr(self) -> u8 {
        match self {
            Self::Primary => 0x76,
            Self::Secondary => 0x77,
            Self::Custom(v) => v,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Poll<T> {
    Pending { wait_ms: u32 },
    Ready(T),
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InitStage {
    SensorReady,
    TemperatureReady,
    CoefficientsReady,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigError {
    MeasurementBusyTimeExceeded,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub enum Error<I2cError> {
    Bus(I2cError),
    InvalidDevice { id: u8 },
    InvalidConfig { reason: ConfigError },
    Timeout { stage: InitStage },
    NoSampleReady,
    InitConsumed,
}

impl<I2cError> From<I2cError> for Error<I2cError> {
    fn from(value: I2cError) -> Self {
        Self::Bus(value)
    }
}

pub struct TransitionError<I2C, I2cError, State> {
    error: Error<I2cError>,
    state: Dps3xx<I2C, State>,
}

impl<I2C, I2cError, State> TransitionError<I2C, I2cError, State> {
    pub fn error(&self) -> &Error<I2cError> {
        &self.error
    }

    pub fn into_error(self) -> Error<I2cError> {
        self.error
    }

    pub fn into_state(self) -> Dps3xx<I2C, State> {
        self.state
    }

    pub fn into_parts(self) -> (Error<I2cError>, Dps3xx<I2C, State>) {
        (self.error, self.state)
    }

    pub fn release(self) -> I2C {
        self.state.release()
    }

    fn with_state(error: Error<I2cError>, state: Dps3xx<I2C, State>) -> Self {
        Self { error, state }
    }
}

impl<I2C, I2cError, State> core::fmt::Debug for TransitionError<I2C, I2cError, State>
where
    Error<I2cError>: core::fmt::Debug,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("TransitionError")
            .field("error", &self.error)
            .field("state", &"<Dps3xx>")
            .finish()
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub pressure_pa: f32,
    pub temperature_c: f32,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RawSample {
    pub pressure: i32,
    pub temperature: i32,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Status {
    pub coefficients_ready: bool,
    pub sensor_ready: bool,
    pub temperature_ready: bool,
    pub pressure_ready: bool,
}

impl Status {
    fn from_bits(status: u8) -> Self {
        Self {
            coefficients_ready: (status & (1 << 7)) != 0,
            sensor_ready: (status & (1 << 6)) != 0,
            temperature_ready: (status & (1 << 5)) != 0,
            pressure_ready: (status & (1 << 4)) != 0,
        }
    }
}

pub struct Dps3xx<I2C, State = Uninit> {
    bus: I2cBus<I2C>,
    coeffs: CalibrationCoeffs,
    config: Config,
    pressure_scale: f32,
    temperature_scale: f32,
    _state: PhantomData<State>,
}

pub struct Init<I2C> {
    state: InitState,
    dps: Option<Dps3xx<I2C, Uninit>>,
}

enum InitState {
    Sensor,
    Temperature,
    Coefficients,
}

impl<I2C, S> Dps3xx<I2C, S> {
    fn map_state<T>(self) -> Dps3xx<I2C, T> {
        Dps3xx {
            bus: self.bus,
            coeffs: self.coeffs,
            config: self.config,
            pressure_scale: self.pressure_scale,
            temperature_scale: self.temperature_scale,
            _state: PhantomData,
        }
    }

    pub fn release(self) -> I2C {
        self.bus.release()
    }

    pub fn config(&self) -> Config {
        self.config
    }
}

impl<I2C, I2cError, S> Dps3xx<I2C, S>
where
    I2C: I2c<Error = I2cError>,
{
    pub fn status(&mut self) -> Result<Status, Error<I2cError>> {
        Ok(Status::from_bits(self.read_status()?))
    }

    pub fn product_id(&mut self) -> Result<u8, Error<I2cError>> {
        self.read_reg(Register::PROD_ID)
    }

    pub fn reset(mut self) -> Result<Dps3xx<I2C, Uninit>, TransitionError<I2C, I2cError, S>> {
        if let Err(err) = self.write_reg(Register::RESET, 0b1000_1001) {
            return Err(TransitionError::with_state(err, self));
        }
        Ok(self.map_state())
    }

    fn read_status(&mut self) -> Result<u8, Error<I2cError>> {
        Ok(self.read_reg(Register::MEAS_CFG)? & 0xF0)
    }

    fn write_reg(&mut self, reg: Register, value: u8) -> Result<(), Error<I2cError>> {
        self.bus.write_reg(reg, value)?;
        Ok(())
    }

    fn write_addr(&mut self, addr: u8, value: u8) -> Result<(), Error<I2cError>> {
        self.bus.write_addr(addr, value)?;
        Ok(())
    }

    fn read_reg(&mut self, reg: Register) -> Result<u8, Error<I2cError>> {
        Ok(self.bus.read_reg(reg)?)
    }

    fn read_i24(&mut self, reg: Register) -> Result<i32, Error<I2cError>> {
        let mut bytes = [0_u8; 3];
        self.bus.read_many(reg, &mut bytes)?;
        let value = (u32::from(bytes[0]) << 16) | (u32::from(bytes[1]) << 8) | u32::from(bytes[2]);
        Ok(get_twos_complement(value, 24))
    }

    fn apply_temp_workaround_registers(&mut self) -> Result<(), Error<I2cError>> {
        self.write_addr(0x0E, 0xA5)?;
        self.write_addr(0x0F, 0x96)?;
        self.write_addr(0x62, 0x02)?;
        self.write_addr(0x0E, 0x00)?;
        self.write_addr(0x0F, 0x00)?;
        Ok(())
    }

    fn apply_config(&mut self) -> Result<(), Error<I2cError>> {
        let prs_cfg = self.read_reg(Register::PRS_CFG)?;
        self.write_reg(Register::PRS_CFG, prs_cfg_value(prs_cfg, &self.config))?;

        let temp_cfg = self.read_reg(Register::TEMP_CFG)?;
        let coef_source_ext = (self.read_reg(Register::TMP_COEF_SRCE)? & 0x80) != 0;
        let new_temp_cfg = tmp_cfg_value(temp_cfg, &self.config, coef_source_ext);
        self.write_reg(Register::TEMP_CFG, new_temp_cfg)?;

        let temp_shift = self.config.temperature.oversampling.reg() > Oversampling::X8.reg();
        let pres_shift = self.config.pressure.oversampling.reg() > Oversampling::X8.reg();
        self.write_reg(
            Register::CFG_REG,
            cfg_reg_value(&self.config, temp_shift, pres_shift),
        )?;

        Ok(())
    }

    fn standby_internal(&mut self) -> Result<(), Error<I2cError>> {
        self.write_reg(Register::MEAS_CFG, 0)
    }

    fn read_coefficients(&mut self) -> Result<(), Error<I2cError>> {
        let mut bytes = [0_u8; 18];
        self.bus.read_many(Register::COEFF_REG_1, &mut bytes)?;
        process_calibration_coefficients(&mut self.coeffs, &mut bytes);
        Ok(())
    }

    fn validate_config(config: &Config) -> Result<(), Error<I2cError>> {
        let prs = calc_busy_time_units(
            config.pressure.rate.reg(),
            config.pressure.oversampling.reg(),
        );
        let tmp = calc_busy_time_units(
            config.temperature.rate.reg(),
            config.temperature.oversampling.reg(),
        );
        if prs >= MAX_BUSYTIME_UNITS || tmp >= MAX_BUSYTIME_UNITS || prs + tmp >= MAX_BUSYTIME_UNITS
        {
            return Err(Error::InvalidConfig {
                reason: ConfigError::MeasurementBusyTimeExceeded,
            });
        }
        Ok(())
    }

    fn init_wait_ms(&self) -> u32 {
        calc_total_wait_ms(
            self.config.temperature.rate.reg(),
            self.config.temperature.oversampling.reg(),
        )
    }
}

impl<I2C, I2cError> Dps3xx<I2C, Uninit>
where
    I2C: I2c<Error = I2cError>,
{
    pub fn new_i2c(i2c: I2C, address: I2cAddress, config: Config) -> Result<Self, Error<I2cError>> {
        Self::validate_config(&config)?;
        Ok(Self {
            bus: I2cBus::new(i2c, address.addr()),
            coeffs: CalibrationCoeffs::default(),
            config,
            pressure_scale: scale_factor(config.pressure.oversampling),
            temperature_scale: scale_factor(config.temperature.oversampling),
            _state: PhantomData,
        })
    }

    pub fn init(self) -> Result<Init<I2C>, TransitionError<I2C, I2cError, Uninit>> {
        Ok(Init {
            state: InitState::Sensor,
            dps: Some(self),
        })
    }

    pub fn init_blocking<D>(
        self,
        delay: &mut D,
    ) -> Result<Dps3xx<I2C, Ready>, TransitionError<I2C, I2cError, Uninit>>
    where
        D: DelayNs,
    {
        let mut init = self.init()?;
        let mut remaining = init.config().init_timeout_ms;
        loop {
            match init.poll() {
                Ok(Poll::Pending { wait_ms }) => {
                    if wait_ms > remaining {
                        let stage = init.stage();
                        let state = init.into_uninit();
                        return Err(TransitionError::with_state(Error::Timeout { stage }, state));
                    }
                    delay.delay_ms(wait_ms);
                    remaining -= wait_ms;
                }
                Ok(Poll::Ready(sensor)) => return Ok(sensor),
                Err(err) => return Err(TransitionError::with_state(err, init.into_uninit())),
            }
        }
    }
}

impl<I2C> Init<I2C> {
    pub fn cancel(mut self) -> Option<Dps3xx<I2C, Uninit>> {
        self.dps.take()
    }

    fn into_uninit(mut self) -> Dps3xx<I2C, Uninit> {
        self.dps.take().unwrap_or_else(|| unreachable!())
    }

    pub fn config(&self) -> Config {
        self.dps.as_ref().map(|dps| dps.config).unwrap_or_default()
    }

    pub fn stage(&self) -> InitStage {
        match self.state {
            InitState::Sensor => InitStage::SensorReady,
            InitState::Temperature => InitStage::TemperatureReady,
            InitState::Coefficients => InitStage::CoefficientsReady,
        }
    }
}

impl<I2C, I2cError> Init<I2C>
where
    I2C: I2c<Error = I2cError>,
{
    pub fn poll(&mut self) -> Result<Poll<Dps3xx<I2C, Ready>>, Error<I2cError>> {
        let dps = match self.dps.as_mut() {
            Some(dps) => dps,
            None => return Err(Error::InitConsumed),
        };

        match self.state {
            InitState::Sensor => {
                let id = dps.product_id()?;
                if (id & 0xF0) != (PRODUCT_ID & 0xF0) {
                    return Err(Error::InvalidDevice { id });
                }
                dps.apply_config()?;
                dps.standby_internal()?;
                dps.apply_temp_workaround_registers()?;
                let status = dps.status()?;
                if !status.sensor_ready {
                    return Ok(Poll::Pending {
                        wait_ms: dps.init_wait_ms(),
                    });
                }
                dps.write_reg(Register::MEAS_CFG, 0b010)?;
                self.state = InitState::Temperature;
                Ok(Poll::Pending {
                    wait_ms: dps.init_wait_ms(),
                })
            }
            InitState::Temperature => {
                if !dps.status()?.temperature_ready {
                    return Ok(Poll::Pending {
                        wait_ms: dps.init_wait_ms(),
                    });
                }
                let _ = dps.read_i24(Register::TMP_B2)?;
                dps.standby_internal()?;
                self.state = InitState::Coefficients;
                Ok(Poll::Pending { wait_ms: 10 })
            }
            InitState::Coefficients => {
                if !dps.status()?.coefficients_ready {
                    return Ok(Poll::Pending { wait_ms: 10 });
                }
                dps.read_coefficients()?;
                let ready = self
                    .dps
                    .take()
                    .unwrap_or_else(|| unreachable!())
                    .map_state();
                Ok(Poll::Ready(ready))
            }
        }
    }
}

impl<I2C, I2cError> Dps3xx<I2C, Ready>
where
    I2C: I2c<Error = I2cError>,
{
    pub fn start_background(&mut self) -> Result<(), Error<I2cError>> {
        self.start_measurement(0b111)
    }

    pub fn standby(&mut self) -> Result<(), Error<I2cError>> {
        self.write_reg(Register::MEAS_CFG, 0)
    }

    pub fn try_read_sample(&mut self) -> Result<Option<Sample>, Error<I2cError>> {
        let status = self.status()?;
        if !(status.pressure_ready && status.temperature_ready) {
            return Ok(None);
        }

        let raw = self.read_raw_sample()?;
        let pressure_scaled = raw.pressure as f32 / self.pressure_scale;
        let temperature_scaled = raw.temperature as f32 / self.temperature_scale;

        Ok(Some(Sample {
            pressure_pa: calibrate_pressure(&self.coeffs, pressure_scaled, temperature_scaled),
            temperature_c: (self.coeffs.C0 as f32 * 0.5)
                + (self.coeffs.C1 as f32 * temperature_scaled),
        }))
    }

    pub fn read_sample(&mut self) -> Result<Sample, Error<I2cError>> {
        self.try_read_sample()?.ok_or(Error::NoSampleReady)
    }

    pub fn read_raw_sample(&mut self) -> Result<RawSample, Error<I2cError>> {
        let mut bytes = [0_u8; 6];
        self.bus.read_many(Register::PSR_B2, &mut bytes)?;
        let pressure_bits =
            (u32::from(bytes[0]) << 16) | (u32::from(bytes[1]) << 8) | u32::from(bytes[2]);
        let temperature_bits =
            (u32::from(bytes[3]) << 16) | (u32::from(bytes[4]) << 8) | u32::from(bytes[5]);

        Ok(RawSample {
            pressure: get_twos_complement(pressure_bits, 24),
            temperature: get_twos_complement(temperature_bits, 24),
        })
    }

    fn start_measurement(&mut self, mode_bits: u8) -> Result<(), Error<I2cError>> {
        let mut meas_cfg = self.read_reg(Register::MEAS_CFG)?;
        meas_cfg = (meas_cfg & 0xF8) | mode_bits;
        self.write_reg(Register::MEAS_CFG, meas_cfg)
    }
}
