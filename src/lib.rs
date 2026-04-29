#![no_std]
#![allow(
    clippy::needless_doctest_main,
    reason = "This is readme example, not doctest"
)]
#![doc = include_str!("../README.md")]

mod bus;
mod calibration;
mod config;
mod device;
mod device_internal;
mod register;

pub use config::{
    Config, FifoConfig, InterruptConfig, MeasurementConfig, Oversampling, Rate, TemperatureSource,
};
pub use device::{
    ConfigError, Dps3xx, Error, I2cAddress, Init, InitStage, Poll, RawSample, Ready, Sample,
    Status, TransitionError, Uninit,
};

pub mod advanced {
    pub use crate::device_internal::{calc_busy_time_ms, calc_total_wait_ms, BUSYTIME_FAILSAFE_MS};
    pub use crate::register::Register;
}
