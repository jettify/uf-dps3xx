/// Pressure rate
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Default, Copy, Clone, Debug, Eq, PartialEq)]
pub enum PressureRate {
    //  See P.9 of manual
    #[default]
    Sps1 = 0b000,
    Sps2 = 0b001,
    Sps4 = 0b010,
    Sps8 = 0b011,
    Sps16 = 0b100,
    Sps32 = 0b101,
    Sps64 = 0b110,
    Sps128 = 0b111,
}

impl PressureRate {
    pub const fn val(self) -> u8 {
        self as u8
    }
}

impl From<PressureRate> for u8 {
    fn from(value: PressureRate) -> Self {
        value as u8
    }
}

/// Pressure resolution
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Default, Copy, Clone, Debug, Eq, PartialEq)]
pub enum PressureResolution {
    // See section 8.3, PM_RC field
    #[default]
    Samples1 = 0b000,
    Samples2 = 0b001,
    Samples4 = 0b010,
    Samples8 = 0b011,
    Samples16 = 0b100,
    Samples32 = 0b101,
    Samples64 = 0b110,
    Samples128 = 0b111, // Available for measurements in background mode only
}

impl PressureResolution {
    pub const fn val(self) -> u8 {
        self as u8
    }
}

impl From<PressureResolution> for u8 {
    fn from(value: PressureResolution) -> Self {
        value as u8
    }
}

/// Temperature rate
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Default, Copy, Clone, Debug, Eq, PartialEq)]
pub enum TemperatureRate {
    // See section 8.4 Temperature Configuration, TMP_RATE
    #[default]
    Sps1 = 0b000,
    Sps2 = 0b001,
    Sps4 = 0b010,
    Sps8 = 0b011,
    Sps16 = 0b100,
    Sps32 = 0b101,
    Sps64 = 0b110,
    Sps128 = 0b111, // Applicable for measurements in Background mode only
}

impl TemperatureRate {
    pub const fn val(self) -> u8 {
        self as u8
    }
}

impl From<TemperatureRate> for u8 {
    fn from(value: TemperatureRate) -> Self {
        value as u8
    }
}

/// Temperature resolution
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Default, Copy, Clone, Debug, Eq, PartialEq)]
pub enum TemperatureResolution {
    // See section 8.4 Temperature Configuration, TMP_PRC
    #[default]
    Samples1 = 0b000,
    Samples2 = 0b001,
    Samples4 = 0b010,
    Samples8 = 0b011,
    Samples16 = 0b100,
    Samples32 = 0b101,
    Samples64 = 0b110,
    Samples128 = 0b111,
}

impl TemperatureResolution {
    pub const fn val(self) -> u8 {
        self as u8
    }
}

impl From<TemperatureResolution> for u8 {
    fn from(value: TemperatureResolution) -> Self {
        value as u8
    }
}

/// Configuration struct
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug)]
pub struct Config {
    pub(crate) pres_rate: Option<PressureRate>,
    pub(crate) pres_res: Option<PressureResolution>,
    pub(crate) temp_rate: Option<TemperatureRate>,
    pub(crate) temp_res: Option<TemperatureResolution>,
    pub(crate) temp_ext: Option<bool>,
    pub(crate) int_hl: bool,
    pub(crate) int_fifo: bool,
    pub(crate) int_temp: bool,
    pub(crate) int_pres: bool,
    pub(crate) temp_shift: bool,
    pub(crate) pres_shift: bool,
    pub(crate) fifo_enable: bool,
    pub(crate) spi_mode: bool,
    pub(crate) init_timeout_ms: u32,
}

impl Config {
    /// Creates a new configuration object with default values
    pub fn new() -> Self {
        Config {
            pres_rate: None,
            pres_res: None,
            temp_rate: None,
            temp_res: None,
            temp_ext: None,
            int_hl: false,
            int_fifo: false,
            int_temp: false,
            int_pres: false,
            temp_shift: false,
            pres_shift: false,
            fifo_enable: false,
            spi_mode: false,
            init_timeout_ms: 5000,
        }
    }

    pub fn pres_rate(&mut self, rate: PressureRate) -> &mut Self {
        self.pres_rate = Some(rate);
        self
    }

    pub fn pres_res(&mut self, res: PressureResolution) -> &mut Self {
        self.pres_res = Some(res);
        self
    }

    pub fn temp_rate(&mut self, rate: TemperatureRate) -> &mut Self {
        self.temp_rate = Some(rate);
        self
    }

    pub fn temp_res(&mut self, res: TemperatureResolution) -> &mut Self {
        self.temp_res = Some(res);
        self
    }

    pub fn temp_external(&mut self, external: bool) -> &mut Self {
        self.temp_ext = Some(external);
        self
    }

    /// Interrupt (on SDO pin) active level
    pub fn int_hl(&mut self, int_on_sdo_pin: bool) -> &mut Self {
        self.int_hl = int_on_sdo_pin;
        self
    }

    pub fn int_fifo(&mut self, int_on_fifo: bool) -> &mut Self {
        self.int_fifo = int_on_fifo;
        self
    }

    pub fn int_temp(&mut self, int_on_temp: bool) -> &mut Self {
        self.int_temp = int_on_temp;
        self
    }

    pub fn int_pres(&mut self, int_on_pres: bool) -> &mut Self {
        self.int_pres = int_on_pres;
        self
    }

    /// Set temperature result bit-shift, Must be set to true when oversampling rate > 8 times
    pub fn temp_shift(&mut self, temp_shift_enable: bool) -> &mut Self {
        self.temp_shift = temp_shift_enable;
        self
    }

    /// Set pressure result bit-shift, Must be set to true when oversampling rate > 8 times
    pub fn pres_shift(&mut self, pres_shift_enable: bool) -> &mut Self {
        self.pres_shift = pres_shift_enable;
        self
    }

    /// Set fifo options
    pub fn fifo(&mut self, interrupt_on_full: bool, enable: bool) -> &mut Self {
        self.int_fifo = interrupt_on_full;
        self.fifo_enable = enable;
        self
    }

    /// Set SPI mode (false -> 4 wire, true -> 3 wire interface)
    pub fn spi_mode(&mut self, three_wire: bool) -> &mut Self {
        self.spi_mode = three_wire;
        self
    }

    pub fn init_timeout_ms(&mut self, timeout_ms: u32) -> &mut Self {
        self.init_timeout_ms = timeout_ms;
        self
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enum_helpers_and_default() {
        assert_eq!(PressureRate::Sps64.val(), 0b110);
        assert_eq!(u8::from(PressureRate::Sps128), 0b111);
        assert_eq!(PressureResolution::Samples32.val(), 0b101);
        assert_eq!(u8::from(PressureResolution::Samples64), 0b110);
        assert_eq!(TemperatureRate::Sps16.val(), 0b100);
        assert_eq!(u8::from(TemperatureRate::Sps8), 0b011);
        assert_eq!(TemperatureResolution::Samples2.val(), 0b001);
        assert_eq!(u8::from(TemperatureResolution::Samples128), 0b111);

        let cfg = Config::default();
        assert_eq!(cfg.pres_rate, None);
        assert_eq!(cfg.init_timeout_ms, 5000);
    }

    #[test]
    fn test_config_new_defaults() {
        let cfg = Config::new();

        assert_eq!(cfg.pres_rate, None);
        assert_eq!(cfg.pres_res, None);
        assert_eq!(cfg.temp_rate, None);
        assert_eq!(cfg.temp_res, None);
        assert_eq!(cfg.temp_ext, None);
        assert!(!cfg.int_hl);
        assert!(!cfg.int_fifo);
        assert!(!cfg.int_temp);
        assert!(!cfg.int_pres);
        assert!(!cfg.temp_shift);
        assert!(!cfg.pres_shift);
        assert!(!cfg.fifo_enable);
        assert!(!cfg.spi_mode);
        assert_eq!(cfg.init_timeout_ms, 5000);
    }

    #[test]
    fn test_config_builder_methods() {
        let mut cfg = Config::new();
        cfg.pres_rate(PressureRate::Sps16)
            .pres_res(PressureResolution::Samples8)
            .temp_rate(TemperatureRate::Sps32)
            .temp_res(TemperatureResolution::Samples64)
            .temp_external(true)
            .int_hl(true)
            .int_fifo(true)
            .int_temp(true)
            .int_pres(true)
            .temp_shift(true)
            .pres_shift(true)
            .fifo(false, true)
            .spi_mode(true)
            .init_timeout_ms(10_000);

        assert_eq!(cfg.pres_rate, Some(PressureRate::Sps16));
        assert_eq!(cfg.pres_res, Some(PressureResolution::Samples8));
        assert_eq!(cfg.temp_rate, Some(TemperatureRate::Sps32));
        assert_eq!(cfg.temp_res, Some(TemperatureResolution::Samples64));
        assert_eq!(cfg.temp_ext, Some(true));
        assert!(cfg.int_hl);
        assert!(!cfg.int_fifo);
        assert!(cfg.int_temp);
        assert!(cfg.int_pres);
        assert!(cfg.temp_shift);
        assert!(cfg.pres_shift);
        assert!(cfg.fifo_enable);
        assert!(cfg.spi_mode);
        assert_eq!(cfg.init_timeout_ms, 10_000);
    }
}
