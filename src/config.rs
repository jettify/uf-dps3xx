use crate::device_internal::{calc_busy_time_units, MAX_BUSYTIME_UNITS};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Default, Clone, Copy, Debug, Eq, PartialEq)]
pub enum Rate {
    #[default]
    Hz1,
    Hz2,
    Hz4,
    Hz8,
    Hz16,
    Hz32,
    Hz64,
    Hz128,
}

impl Rate {
    #[must_use]
    pub const fn reg(self) -> u8 {
        match self {
            Self::Hz1 => 0,
            Self::Hz2 => 1,
            Self::Hz4 => 2,
            Self::Hz8 => 3,
            Self::Hz16 => 4,
            Self::Hz32 => 5,
            Self::Hz64 => 6,
            Self::Hz128 => 7,
        }
    }

    #[must_use]
    pub const fn hz(self) -> u32 {
        match self {
            Self::Hz1 => 1,
            Self::Hz2 => 2,
            Self::Hz4 => 4,
            Self::Hz8 => 8,
            Self::Hz16 => 16,
            Self::Hz32 => 32,
            Self::Hz64 => 64,
            Self::Hz128 => 128,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Default, Clone, Copy, Debug, Eq, PartialEq)]
pub enum Oversampling {
    #[default]
    X1,
    X2,
    X4,
    X8,
    X16,
    X32,
    X64,
    X128,
}

impl Oversampling {
    #[must_use]
    pub const fn reg(self) -> u8 {
        match self {
            Self::X1 => 0,
            Self::X2 => 1,
            Self::X4 => 2,
            Self::X8 => 3,
            Self::X16 => 4,
            Self::X32 => 5,
            Self::X64 => 6,
            Self::X128 => 7,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default)]
pub enum TemperatureSource {
    /// Select the temperature sensor associated with the factory coefficients.
    #[default]
    Auto,
    /// Require factory calibration for the internal sensor; otherwise initialization fails.
    Internal,
    /// Require factory calibration for the external sensor; otherwise initialization fails.
    External,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default)]
pub enum Mode {
    Idle,
    CommandPressure,
    CommandTemperature,
    BackgroundPressure,
    BackgroundTemperature,
    #[default]
    BackgroundAll,
}

impl Mode {
    #[must_use]
    pub const fn bits(self) -> u8 {
        match self {
            Self::Idle => 0b000,
            Self::CommandPressure => 0b001,
            Self::CommandTemperature => 0b010,
            Self::BackgroundPressure => 0b101,
            Self::BackgroundTemperature => 0b110,
            Self::BackgroundAll => 0b111,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Default, Clone, Copy, Debug, Eq, PartialEq)]
pub struct MeasurementConfig {
    pub rate: Rate,
    pub oversampling: Oversampling,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default)]
pub struct FifoConfig {
    pub enable: bool,
    pub interrupt_on_full: bool,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Default)]
pub struct InterruptConfig {
    pub active_high: bool,
    pub temperature_ready: bool,
    pub pressure_ready: bool,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigError {
    MeasurementBusyTimeExceeded,
    TemperatureSourceMismatch,
}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::MeasurementBusyTimeExceeded => f.write_str("measurement busy time exceeded"),
            Self::TemperatureSourceMismatch => {
                f.write_str("temperature source does not match factory calibration")
            }
        }
    }
}

impl core::error::Error for ConfigError {}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Config {
    pub(crate) pressure: MeasurementConfig,
    pub(crate) temperature: MeasurementConfig,
    pub(crate) temperature_source: TemperatureSource,
    pub(crate) fifo: FifoConfig,
    pub(crate) interrupts: InterruptConfig,
    pub(crate) mode: Mode,
    pub(crate) init_timeout_ms: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

impl Config {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            pressure: MeasurementConfig {
                rate: Rate::Hz1,
                oversampling: Oversampling::X1,
            },
            temperature: MeasurementConfig {
                rate: Rate::Hz1,
                oversampling: Oversampling::X1,
            },
            temperature_source: TemperatureSource::Auto,
            fifo: FifoConfig {
                enable: false,
                interrupt_on_full: false,
            },
            interrupts: InterruptConfig {
                active_high: false,
                temperature_ready: false,
                pressure_ready: false,
            },
            mode: Mode::BackgroundAll,
            init_timeout_ms: 5000,
        }
    }

    /// Low-power weather station preset (1 Hz pressure X1, 1 Hz temperature X1).
    #[must_use]
    pub const fn weather() -> Self {
        Self::new()
            .pressure(Rate::Hz1, Oversampling::X1)
            .temperature(Rate::Hz1, Oversampling::X1)
    }

    /// Indoor navigation / floor detection preset (16 Hz pressure X8, 2 Hz temperature X1).
    #[must_use]
    pub const fn indoor() -> Self {
        Self::new()
            .pressure(Rate::Hz16, Oversampling::X8)
            .temperature(Rate::Hz2, Oversampling::X1)
    }

    /// High-precision flight controller / navigation preset (32 Hz pressure X16, 4 Hz temperature X1).
    #[must_use]
    pub const fn navigation() -> Self {
        Self::new()
            .pressure(Rate::Hz32, Oversampling::X16)
            .temperature(Rate::Hz4, Oversampling::X1)
    }

    /// High-rate dynamic motion preset (64 Hz pressure X8, 4 Hz temperature X1).
    #[must_use]
    pub const fn high_rate() -> Self {
        Self::new()
            .pressure(Rate::Hz64, Oversampling::X8)
            .temperature(Rate::Hz4, Oversampling::X1)
    }

    #[must_use]
    pub const fn pressure(mut self, rate: Rate, oversampling: Oversampling) -> Self {
        self.pressure = MeasurementConfig { rate, oversampling };
        self
    }

    #[must_use]
    pub const fn temperature(mut self, rate: Rate, oversampling: Oversampling) -> Self {
        self.temperature = MeasurementConfig { rate, oversampling };
        self
    }

    #[must_use]
    pub const fn temperature_source(mut self, source: TemperatureSource) -> Self {
        self.temperature_source = source;
        self
    }

    #[must_use]
    pub const fn fifo(mut self, cfg: FifoConfig) -> Self {
        self.fifo = cfg;
        self
    }

    #[must_use]
    pub const fn interrupts(mut self, cfg: InterruptConfig) -> Self {
        self.interrupts = cfg;
        self
    }

    #[must_use]
    pub const fn mode(mut self, mode: Mode) -> Self {
        self.mode = mode;
        self
    }

    #[must_use]
    pub const fn init_timeout_ms(mut self, timeout_ms: u32) -> Self {
        self.init_timeout_ms = timeout_ms;
        self
    }

    /// Validates the conversion budget for the selected operating mode.
    ///
    /// Background modes count only enabled channels. Command modes ignore periodic rates,
    /// and idle mode has no conversion budget. Temperature-source compatibility is checked
    /// against the device's factory calibration during initialization and reconfiguration.
    ///
    /// # Errors
    ///
    /// Returns [`ConfigError::MeasurementBusyTimeExceeded`] if the configured rates and
    /// oversampling factors exceed the maximum allowed conversion time per second.
    pub const fn validate(&self) -> Result<(), ConfigError> {
        let prs = calc_busy_time_units(self.pressure.rate.reg(), self.pressure.oversampling.reg());
        let tmp = calc_busy_time_units(
            self.temperature.rate.reg(),
            self.temperature.oversampling.reg(),
        );
        let busy = match self.mode {
            Mode::Idle => 0,
            Mode::CommandPressure => calc_busy_time_units(0, self.pressure.oversampling.reg()),
            Mode::CommandTemperature => {
                calc_busy_time_units(0, self.temperature.oversampling.reg())
            }
            Mode::BackgroundPressure => prs,
            Mode::BackgroundTemperature => tmp,
            Mode::BackgroundAll => prs + tmp,
        };
        if busy >= MAX_BUSYTIME_UNITS {
            return Err(ConfigError::MeasurementBusyTimeExceeded);
        }
        Ok(())
    }

    pub(crate) const fn validate_temperature_source(
        &self,
        external: bool,
    ) -> Result<(), ConfigError> {
        let matches = match self.temperature_source {
            TemperatureSource::Auto => true,
            TemperatureSource::Internal => !external,
            TemperatureSource::External => external,
        };
        if matches {
            Ok(())
        } else {
            Err(ConfigError::TemperatureSourceMismatch)
        }
    }

    #[must_use]
    pub const fn measurement_period_ms(&self) -> u32 {
        let pressure_ms = 1000_u32.div_ceil(self.pressure.rate.hz());
        let temp_ms = 1000_u32.div_ceil(self.temperature.rate.hz());
        if pressure_ms < temp_ms {
            pressure_ms
        } else {
            temp_ms
        }
    }

    #[must_use]
    pub const fn suggested_poll_period_ms(&self) -> u32 {
        let half = self.measurement_period_ms() / 2;
        if half < 5 {
            5
        } else {
            half
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Config, ConfigError, Mode, Oversampling, Rate, TemperatureSource};

    #[test]
    fn budgets_follow_active_channels_and_ignore_command_rates() {
        let config = Config::new()
            .pressure(Rate::Hz4, Oversampling::X128)
            .temperature(Rate::Hz4, Oversampling::X128);
        assert_eq!(
            config.validate(),
            Err(ConfigError::MeasurementBusyTimeExceeded)
        );
        for mode in [
            Mode::BackgroundPressure,
            Mode::BackgroundTemperature,
            Mode::Idle,
            Mode::CommandPressure,
            Mode::CommandTemperature,
        ] {
            assert_eq!(config.mode(mode).validate(), Ok(()), "mode {mode:?}");
        }
        let excessive = Config::new()
            .pressure(Rate::Hz128, Oversampling::X128)
            .temperature(Rate::Hz128, Oversampling::X128);
        for mode in [
            Mode::BackgroundPressure,
            Mode::BackgroundTemperature,
            Mode::BackgroundAll,
        ] {
            assert_eq!(
                excessive.mode(mode).validate(),
                Err(ConfigError::MeasurementBusyTimeExceeded)
            );
        }
        for mode in [Mode::Idle, Mode::CommandPressure, Mode::CommandTemperature] {
            assert_eq!(excessive.mode(mode).validate(), Ok(()));
        }
        // An unused channel must not consume the active channel's budget.
        assert_eq!(
            excessive
                .pressure(Rate::Hz1, Oversampling::X1)
                .mode(Mode::BackgroundPressure)
                .validate(),
            Ok(())
        );
        assert_eq!(
            excessive
                .temperature(Rate::Hz1, Oversampling::X1)
                .mode(Mode::BackgroundTemperature)
                .validate(),
            Ok(())
        );
    }

    #[test]
    fn explicit_temperature_sources_must_match_factory_calibration() {
        for external in [false, true] {
            assert_eq!(Config::new().validate_temperature_source(external), Ok(()));
            let matching = if external {
                TemperatureSource::External
            } else {
                TemperatureSource::Internal
            };
            let mismatching = if external {
                TemperatureSource::Internal
            } else {
                TemperatureSource::External
            };
            assert_eq!(
                Config::new()
                    .temperature_source(matching)
                    .validate_temperature_source(external),
                Ok(())
            );
            assert_eq!(
                Config::new()
                    .temperature_source(mismatching)
                    .validate_temperature_source(external),
                Err(ConfigError::TemperatureSourceMismatch)
            );
        }
    }

    const _: () = {
        assert!(Config::new().validate().is_ok());
        assert!(Config::weather().validate().is_ok());
        assert!(Config::indoor().validate().is_ok());
        assert!(Config::navigation().validate().is_ok());
        assert!(Config::high_rate().validate().is_ok());
    };

    #[test]
    fn presets_are_valid() {
        assert_eq!(Config::weather().validate(), Ok(()));
        assert_eq!(Config::indoor().validate(), Ok(()));
        assert_eq!(Config::navigation().validate(), Ok(()));
        assert_eq!(Config::high_rate().validate(), Ok(()));
    }

    #[test]
    fn excessive_busy_time_is_rejected() {
        let invalid = Config::new()
            .pressure(Rate::Hz128, Oversampling::X128)
            .temperature(Rate::Hz1, Oversampling::X1);
        assert_eq!(
            invalid.validate(),
            Err(ConfigError::MeasurementBusyTimeExceeded)
        );
    }

    #[test]
    fn measurement_period_uses_fastest_channel() {
        let cfg = Config::navigation(); // 32 Hz pressure (~32ms), 4 Hz temp (250ms)
        assert_eq!(cfg.measurement_period_ms(), 32);
        assert_eq!(cfg.suggested_poll_period_ms(), 16);
    }

    #[test]
    fn config_error_display_messages() {
        use core::fmt::Write;
        struct Buf([u8; 64], usize);
        impl Write for Buf {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                let bytes = s.as_bytes();
                self.0[self.1..self.1 + bytes.len()].copy_from_slice(bytes);
                self.1 += bytes.len();
                Ok(())
            }
        }
        let mut buf = Buf([0; 64], 0);
        write!(buf, "{}", ConfigError::MeasurementBusyTimeExceeded).unwrap();
        assert_eq!(&buf.0[..buf.1], b"measurement busy time exceeded");
    }
}
