# uf-dps3xx

[![CI](https://github.com/jettify/uf-dps3xx/actions/workflows/CI.yml/badge.svg)](https://github.com/jettify/uf-dps3xx/actions/workflows/CI.yml)
[![codecov](https://codecov.io/gh/jettify/uf-dps3xx/graph/badge.svg?token=XZK0JJQ9QN)](https://codecov.io/gh/jettify/uf-dps3xx)
[![crates.io](https://img.shields.io/crates/v/uf-dps3xx)](https://crates.io/crates/uf-dps3xx)
[![docs.rs](https://img.shields.io/docsrs/uf-dps3xx)](https://docs.rs/uf-dps3xx/latest/uf_dps3xx/)

`uf-dps3xx` is a platform-agnostic, `no_std` driver for the `DPS3xx` (`DPS310`, `DPS368`) family of pressure and temperature sensors.
It uses `embedded-hal` 1.0 I2C traits and is designed to stay simple in embedded applications.

## Supported Hardware

- DPS310
- DPS368
- I2C transport only
- SPI is not implemented

## Installation

```toml
[dependencies]
uf-dps3xx = "0.1"
```

Enable `defmt` formatting support with:

```toml
[dependencies]
uf-dps3xx = { version = "0.1", features = ["defmt"] }
```

## Quick Start

The driver uses a typed state machine:

- `Dps3xx<_, Uninit>` after `new_i2c`
- `Dps3xx<_, Ready>` after initialization completes

```rust
use embedded_hal::delay::DelayNs;
use uf_dps3xx::{Config, Dps3xx, I2cAddress, Oversampling, Rate, Uninit};

fn read_sample<I2C, E, D>(
    i2c: I2C,
    delay: &mut D,
) -> Result<uf_dps3xx::Sample, uf_dps3xx::Error<E>>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    D: DelayNs,
{
    let config = Config::default()
        .pressure(Rate::Hz8, Oversampling::X8)
        .temperature(Rate::Hz8, Oversampling::X8)
        .init_timeout_ms(5_000);

    let poll_ms = config.suggested_poll_period_ms();
    let mut sensor = Dps3xx::<_, Uninit>::new_i2c(i2c, I2cAddress::Primary, config)?
        .init_blocking(delay)
        .map_err(|err| err.into_error())?;

    sensor.start_background()?;

    loop {
        if let Some(sample) = sensor.try_read_sample()? {
            return Ok(sample);
        }

        delay.delay_ms(poll_ms);
    }
}
```

## I2C Address

Use `I2cAddress::Primary` for `0x76` and `I2cAddress::Secondary` for `0x77`.
If your board uses a different address, pass `I2cAddress::Custom(address)`.

## Initialization

Use `init_blocking` when you have a delay implementation and want a simple setup flow.
It waits for the sensor, performs the required temperature initialization sequence, reads calibration coefficients, and returns a ready driver.

```rust,ignore
let mut sensor = Dps3xx::<_, Uninit>::new_i2c(i2c, I2cAddress::Primary, Config::default())?
    .init_blocking(delay)
    .map_err(|err| err.into_error())?;
```

Use `init` when your application has its own scheduler or event loop:

```rust,ignore
use uf_dps3xx::{Dps3xx, I2cAddress, Poll, Uninit};

let sensor = Dps3xx::<_, Uninit>::new_i2c(i2c, I2cAddress::Primary, Config::default())?;
let mut init = sensor.init().map_err(|err| err.into_error())?;

loop {
    match init.poll()? {
        Poll::Pending { wait_ms } => {
            // Poll again after at least `wait_ms`.
        }
        Poll::Ready(sensor) => {
            break sensor;
        }
    }
}
```

## Measurements

After initialization, call `start_background` to start continuous pressure and temperature measurement.

```rust,ignore
sensor.start_background()?;
```

Read calibrated values with:

- `try_read_sample()`: returns `Ok(Some(Sample))` when pressure and temperature are ready, otherwise `Ok(None)`.
- `read_sample()`: returns `Error::NoSampleReady` when a full sample is not ready.
- `read_raw_sample()`: reads raw 24-bit pressure and temperature ADC values.

Use `standby()` to stop measurement.

`Sample` contains calibrated pressure in pascals and temperature in degrees Celsius:

```rust
pub struct Sample {
    pub pressure_pa: f32,
    pub temperature_c: f32,
}
```

## Configuration

`Config::default()` uses 1 Hz pressure and temperature measurements with 1x oversampling.

```rust,ignore
use uf_dps3xx::{Config, FifoConfig, InterruptConfig, Oversampling, Rate, TemperatureSource};

let config = Config::default()
    .pressure(Rate::Hz16, Oversampling::X8)
    .temperature(Rate::Hz16, Oversampling::X8)
    .temperature_source(TemperatureSource::Auto)
    .fifo(FifoConfig {
        enable: false,
        interrupt_on_full: false,
    })
    .interrupts(InterruptConfig {
        active_high: false,
        temperature_ready: false,
        pressure_ready: false,
    })
    .init_timeout_ms(5_000);
```

Available rates are `Hz1`, `Hz2`, `Hz4`, `Hz8`, `Hz16`, `Hz32`, `Hz64`, and `Hz128`.
Available oversampling values are `X1`, `X2`, `X4`, `X8`, `X16`, `X32`, `X64`, and `X128`.

The driver validates measurement busy time in `new_i2c`. Invalid combinations return `Error::InvalidConfig`.

`Config::suggested_poll_period_ms()` returns a conservative polling interval for the configured rates.

## Status And Device Info

`status()` reads the sensor status bits:

```rust,ignore
let status = sensor.status()?;

if status.pressure_ready && status.temperature_ready {
    let sample = sensor.read_sample()?;
}
```

`product_id()` reads the device product ID register.

## Errors

The main error type is:

```rust,ignore
pub enum Error<I2cError> {
    Bus(I2cError),
    InvalidDevice { id: u8 },
    InvalidConfig { reason: ConfigError },
    Timeout { stage: InitStage },
    NoSampleReady,
    InitConsumed,
}
```

Methods that consume the driver during a state transition return `TransitionError` on failure.
It keeps the driver so callers can recover the bus or inspect the error:

```rust,ignore
match sensor.reset() {
    Ok(sensor) => {
        // Sensor is back in the Uninit state.
    }
    Err(err) => {
        let i2c = err.release();
    }
}
```

## Bus Recovery

Use `release()` to recover the underlying I2C bus:

```rust,ignore
let i2c = sensor.release();
```

## Examples

- [examples/simple.rs](https://github.com/jettify/uf-dps3xx/blob/main/examples/simple.rs) shows a mocked init and sample read flow.
- [examples/tbs-lucid-h7/src/main.rs](https://github.com/jettify/uf-dps3xx/blob/main/examples/tbs-lucid-h7/src/main.rs) shows an embedded setup for the TBS Lucid H7 flight controller.

## Inspirations

1. https://github.com/Infineon/arduino-xensiv-dps3xx
2. https://github.com/dprotsiv/dps310-rs

## License

This project is licensed under `Apache-2.0`. See `LICENSE` for details.
