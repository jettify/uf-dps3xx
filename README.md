# uf-dps3xx

[![CI](https://github.com/jettify/uf-dps3xx/actions/workflows/CI.yml/badge.svg)](https://github.com/jettify/uf-dps3xx/actions/workflows/CI.yml)
[![codecov](https://codecov.io/gh/jettify/uf-dps3xx/graph/badge.svg?token=XZK0JJQ9QN)](https://codecov.io/gh/jettify/uf-dps3xx)
[![crates.io](https://img.shields.io/crates/v/uf-dps3xx)](https://crates.io/crates/uf-dps3xx)
[![docs.rs](https://img.shields.io/docsrs/uf-dps3xx)](https://docs.rs/uf-dps3xx/latest/uf_dps3xx/)

`uf-dps3xx` is a platform-agnostic, `no_std` driver for the `DPS3xx` (`DPS310`, `DPS368`) family of pressure and temperature sensors.
It uses `embedded-hal` 1.0 I2C traits and is designed to stay simple and ergonomic in embedded code.

## Supported Hardware

* DPS310
* DPS368
* I2C transport only
* SPI is not implemented.

## Installation

Add the crate to your `Cargo.toml`:

```toml
[dependencies]
uf-dps3xx = "0.1"
```

Or use Cargo:

```bash
cargo add uf-dps3xx
```

If you want `defmt` formatting:

```toml
[dependencies]
uf-dps3xx = { version = "0.1", features = ["defmt"] }
```

## Quick Start

The driver uses a typed state machine:

- `DPS3xx<_, Unconfigured>` after `new`
- `DPS3xx<_, InitInProgress>` after `start_init`
- `DPS3xx<_, Configured>` after initialization
- `DPS3xx<_, Calibrated>` after calibration coefficients are loaded

The shortest path is `init_and_calibrate`, which performs initialization, waits for the sensor, and loads calibration data.

```rust
use embedded_hal::delay::DelayNs;
use uf_dps3xx::{Config, DPS3xx, MeasurementMode, PressureRate, PressureResolution};

fn read_pressure<I2C, E, D>(
    i2c: I2C,
    address: u8,
    delay: &mut D,
) -> Result<f32, uf_dps3xx::Error<E>>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    D: DelayNs,
{
    let mut config = Config::new();
    config
        .pres_rate(PressureRate::Sps8)
        .pres_res(PressureResolution::Samples8)
        .init_timeout_ms(5_000);

    let mut sensor = DPS3xx::new(i2c, address, &config)?.init_and_calibrate(delay)?;
    sensor.start_measurement(MeasurementMode::BackgroundPressureAndTemperature)?;

    loop {
        match sensor.try_read_pressure_calibrated() {
            Ok(value) => return Ok(value),
            Err(nb::Error::WouldBlock) => delay.delay_ms(100),
            Err(nb::Error::Other(err)) => return Err(err),
        }
    }
}
```


## Measurements

Use `start_measurement` to select one of the supported modes:

- `OneShotPressure`
- `OneShotTemperature`
- `BackgroundPressure`
- `BackgroundTemperature`
- `BackgroundPressureAndTemperature`

For blocking reads, use:

- `read_temp_calibrated`
- `read_pressure_calibrated`

For polling loops, use:

- `try_read_temp_calibrated`
- `try_read_pressure_calibrated`

The `try_*` methods return `nb::Error::WouldBlock` until the sensor reports data ready.

## State Machine

| State | What you can do |
| --- | --- |
| `Unconfigured` | create the driver, start init, reset, release the bus |
| `InitInProgress` | poll initialization, finish initialization |
| `Configured` | start measurements, read raw data, check status, read coefficients |
| `Calibrated` | read calibrated temperature and pressure, poll ready state, release the bus |

`reset` is available from any state and returns the driver to `Unconfigured`.

## Errors

Common errors returned by the crate:

- `UnexpectedProductId(u8)` when the sensor ID does not match the expected `DPS3xx` family value
- `InvalidConfigBusyTime` when configuration would violate busy-time constraints
- `BusyTimeExceeded` when a requested runtime measurement mode is unsafe for the active config
- `CoefficientsNotReady` when calibration data is read too early
- `InitTimeout` when initialization takes longer than `Config::init_timeout_ms`
- `InvalidOversampling` when the register state contains an unsupported oversampling value
- `I2CError` for transport failures

## Examples

- [examples/simple.rs](https://github.com/jettify/uf-dps3xx/blob/main/examples/simple.rs) shows the full mocked init, calibration, and pressure read flow.
- [examples/tbs-lucid-h7/src/main.rs](https://github.com/jettify/uf-dps3xx/blob/main/examples/tbs-lucid-h7/src/main.rs) shows a embedded setup for `TBS Lucid H7` flight controller.

## Inspirations

1. https://github.com/Infineon/arduino-xensiv-dps3xx
2. https://github.com/dprotsiv/dps310-rs

## License

This project is licensed under `Apache-2.0`. See `LICENSE` for details.
