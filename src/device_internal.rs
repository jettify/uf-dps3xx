use crate::config::{Config, Oversampling, TemperatureSource};

// Low nibble of PROD_ID; the high nibble is the silicon revision.
pub(crate) const PRODUCT_ID: u8 = 0x00;
pub(crate) const RESET_SOFT_AND_FIFO_FLUSH: u8 = 0b1000_1001;
pub(crate) const RESET_FIFO_FLUSH: u8 = 0b1000_0000;
pub const BUSYTIME_SCALING: u32 = 10;
pub const BUSYTIME_FAILSAFE_MS: u32 = 10;
pub const MAX_BUSYTIME_UNITS: u32 = (1000 - BUSYTIME_FAILSAFE_MS) * BUSYTIME_SCALING;

#[must_use]
pub(crate) const fn scale_factor(oversampling: Oversampling) -> f32 {
    match oversampling {
        Oversampling::X1 => 524_288_f32,
        Oversampling::X2 => 1_572_864_f32,
        Oversampling::X4 => 3_670_016_f32,
        Oversampling::X8 => 7_864_320_f32,
        Oversampling::X16 => 253_952_f32,
        Oversampling::X32 => 516_096_f32,
        Oversampling::X64 => 1_040_384_f32,
        Oversampling::X128 => 2_088_960_f32,
    }
}

#[must_use]
pub(crate) const fn inv_scale_factor(oversampling: Oversampling) -> f32 {
    1.0 / scale_factor(oversampling)
}

#[must_use]
pub const fn calc_busy_time_units(rate: u8, oversampling: u8) -> u32 {
    (20u32 << rate) + (16u32 << (oversampling + rate))
}

#[must_use]
pub const fn calc_busy_time_ms(rate: u8, oversampling: u8) -> u32 {
    calc_busy_time_units(rate, oversampling) / BUSYTIME_SCALING
}

#[must_use]
pub const fn calc_total_wait_ms(rate: u8, oversampling: u8) -> u32 {
    calc_busy_time_ms(rate, oversampling) + BUSYTIME_FAILSAFE_MS
}

#[must_use]
pub(crate) const fn prs_cfg_value(config: &Config) -> u8 {
    (config.pressure.rate.reg() << 4) | config.pressure.oversampling.reg()
}

#[must_use]
pub(crate) const fn tmp_cfg_value(config: &Config, coef_source_ext: bool) -> u8 {
    let ext_bit = match config.temperature_source {
        TemperatureSource::Auto => {
            if coef_source_ext {
                0x80
            } else {
                0
            }
        }
        TemperatureSource::Internal => 0,
        TemperatureSource::External => 0x80,
    };

    ext_bit | (config.temperature.rate.reg() << 4) | config.temperature.oversampling.reg()
}

#[must_use]
pub(crate) const fn cfg_reg_value(config: &Config, temp_shift: bool, pres_shift: bool) -> u8 {
    ((config.interrupts.active_high as u8) << 7)
        | ((config.fifo.interrupt_on_full as u8) << 6)
        | ((config.interrupts.temperature_ready as u8) << 5)
        | ((config.interrupts.pressure_ready as u8) << 4)
        | ((temp_shift as u8) << 3)
        | ((pres_shift as u8) << 2)
        | ((config.fifo.enable as u8) << 1)
}
