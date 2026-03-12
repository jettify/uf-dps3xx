use crate::config::Config;

pub(crate) const PRODUCT_ID: u8 = 0x10;
pub const BUSYTIME_SCALING: u32 = 10;
pub const BUSYTIME_FAILSAFE_MS: u32 = 10;
pub const MAX_BUSYTIME_UNITS: u32 = (1000 - BUSYTIME_FAILSAFE_MS) * BUSYTIME_SCALING;
pub(crate) const SCALE_FACTORS: [f32; 8] = [
    524_288_f32,
    1_572_864_f32,
    3_670_016_f32,
    7_864_320_f32,
    253_952_f32,
    516_096_f32,
    1_040_384_f32,
    2_088_960_f32,
];

pub fn calc_busy_time_units(measure_rate: u8, oversampling: u8) -> u32 {
    (20u32 << measure_rate) + (16u32 << (oversampling + measure_rate))
}

pub fn calc_busy_time_ms(measure_rate: u8, oversampling: u8) -> u32 {
    calc_busy_time_units(measure_rate, oversampling) / BUSYTIME_SCALING
}

pub fn calc_total_wait_ms(measure_rate: u8, oversampling: u8) -> u32 {
    calc_busy_time_ms(measure_rate, oversampling) + BUSYTIME_FAILSAFE_MS
}

pub(crate) fn prs_cfg_value(current: u8, config: &Config) -> u8 {
    (current & 0x80)
        | ((config.pres_rate.unwrap_or_default() as u8) << 4)
        | (config.pres_res.unwrap_or_default() as u8)
}

pub(crate) fn tmp_cfg_value(current: u8, config: &Config, coef_source: Option<bool>) -> u8 {
    let temp_ext = config.temp_ext.or(coef_source);
    let ext_bit = match temp_ext {
        Some(external) => u8::from(external) << 7,
        None => current & 0x80,
    };

    ext_bit
        | ((config.temp_rate.unwrap_or_default() as u8) << 4)
        | (config.temp_res.unwrap_or_default() as u8)
}

pub(crate) fn cfg_reg_value(config: &Config, temp_shift: bool, pres_shift: bool) -> u8 {
    (u8::from(config.int_hl) << 7)
        | (u8::from(config.int_fifo) << 6)
        | (u8::from(config.int_temp) << 5)
        | (u8::from(config.int_pres) << 4)
        | (u8::from(temp_shift) << 3)
        | (u8::from(pres_shift) << 2)
        | (u8::from(config.fifo_enable) << 1)
        | u8::from(config.spi_mode)
}
