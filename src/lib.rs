#![no_std]
#![doc = include_str!("../README.md")]

mod bus;
mod calibration;
mod config;
mod device;
mod device_internal;
mod register;

pub use config::{
    Config, ConfigError, FifoConfig, InterruptConfig, MeasurementConfig, Mode, Oversampling, Rate,
    TemperatureSource,
};
pub use device::{
    Dps3xx, Error, FifoEntry, FifoStatus, I2cAddress, InitStage, InterruptStatus, Poll, RawSample,
    Sample, Status,
};

pub mod advanced {
    pub use crate::device_internal::{calc_busy_time_ms, calc_total_wait_ms, BUSYTIME_FAILSAFE_MS};
    pub use crate::register::Register;
}
