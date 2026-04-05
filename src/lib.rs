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
    //
    Config,
    PressureRate,
    PressureResolution,
    TemperatureRate,
    TemperatureResolution,
};
pub use device::{
    //
    calc_busy_time_ms,
    calc_busy_time_units,
    calc_total_wait_ms,
    Calibrated,
    Configured,
    DPS3xx,
    Error,
    InitInProgress,
    InitPoll,
    InitStage,
    IsConfigured,
    MeasurementMode,
    Status,
    Unconfigured,
    BUSYTIME_FAILSAFE_MS,
    BUSYTIME_SCALING,
    MAX_BUSYTIME_UNITS,
};
pub use register::Register;
