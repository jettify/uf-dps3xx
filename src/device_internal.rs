use crate::config::{Config, Oversampling, TemperatureSource};

pub(crate) const PRODUCT_ID: u8 = 0x10;
pub const BUSYTIME_SCALING: u32 = 10;
pub const BUSYTIME_FAILSAFE_MS: u32 = 10;
pub const MAX_BUSYTIME_UNITS: u32 = (1000 - BUSYTIME_FAILSAFE_MS) * BUSYTIME_SCALING;
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

pub const fn calc_busy_time_units(rate: u8, oversampling: u8) -> u32 {
    (20u32 << rate) + (16u32 << (oversampling + rate))
}

pub const fn calc_busy_time_ms(rate: u8, oversampling: u8) -> u32 {
    calc_busy_time_units(rate, oversampling) / BUSYTIME_SCALING
}

pub const fn calc_total_wait_ms(rate: u8, oversampling: u8) -> u32 {
    calc_busy_time_ms(rate, oversampling) + BUSYTIME_FAILSAFE_MS
}

pub(crate) fn prs_cfg_value(current: u8, config: &Config) -> u8 {
    (current & 0x80) | (config.pressure.rate.reg() << 4) | config.pressure.oversampling.reg()
}

pub(crate) fn tmp_cfg_value(_current: u8, config: &Config, coef_source_ext: bool) -> u8 {
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

pub(crate) fn cfg_reg_value(config: &Config, temp_shift: bool, pres_shift: bool) -> u8 {
    (u8::from(config.interrupts.active_high) << 7)
        | (u8::from(config.fifo.interrupt_on_full) << 6)
        | (u8::from(config.interrupts.temperature_ready) << 5)
        | (u8::from(config.interrupts.pressure_ready) << 4)
        | (u8::from(temp_shift) << 3)
        | (u8::from(pres_shift) << 2)
        | (u8::from(config.fifo.enable) << 1)
}
