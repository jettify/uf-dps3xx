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
    pub(crate) const fn reg(self) -> u8 {
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

    pub(crate) const fn hz(self) -> u32 {
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
    pub(crate) const fn reg(self) -> u8 {
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
    #[default]
    Auto,
    Internal,
    External,
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
pub struct Config {
    pub(crate) pressure: MeasurementConfig,
    pub(crate) temperature: MeasurementConfig,
    pub(crate) temperature_source: TemperatureSource,
    pub(crate) fifo: FifoConfig,
    pub(crate) interrupts: InterruptConfig,
    pub(crate) init_timeout_ms: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            pressure: MeasurementConfig::default(),
            temperature: MeasurementConfig::default(),
            temperature_source: TemperatureSource::Auto,
            fifo: FifoConfig::default(),
            interrupts: InterruptConfig::default(),
            init_timeout_ms: 5000,
        }
    }
}

impl Config {
    pub fn pressure(mut self, rate: Rate, oversampling: Oversampling) -> Self {
        self.pressure = MeasurementConfig { rate, oversampling };
        self
    }

    pub fn temperature(mut self, rate: Rate, oversampling: Oversampling) -> Self {
        self.temperature = MeasurementConfig { rate, oversampling };
        self
    }

    pub fn temperature_source(mut self, source: TemperatureSource) -> Self {
        self.temperature_source = source;
        self
    }

    pub fn fifo(mut self, cfg: FifoConfig) -> Self {
        self.fifo = cfg;
        self
    }

    pub fn interrupts(mut self, cfg: InterruptConfig) -> Self {
        self.interrupts = cfg;
        self
    }

    pub fn init_timeout_ms(mut self, timeout_ms: u32) -> Self {
        self.init_timeout_ms = timeout_ms;
        self
    }

    pub const fn measurement_period_ms(&self) -> u32 {
        let pressure_ms = ceil_div(1000, self.pressure.rate.hz());
        let temp_ms = ceil_div(1000, self.temperature.rate.hz());
        if pressure_ms > temp_ms {
            pressure_ms
        } else {
            temp_ms
        }
    }

    pub const fn suggested_poll_period_ms(&self) -> u32 {
        let half = self.measurement_period_ms() / 2;
        if half < 5 {
            5
        } else {
            half
        }
    }
}

const fn ceil_div(a: u32, b: u32) -> u32 {
    a.div_ceil(b)
}
