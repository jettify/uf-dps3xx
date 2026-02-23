# uf-dps3xx

[![CI](https://github.com/jettify/uf-dps3xx/actions/workflows/CI.yml/badge.svg)](https://github.com/jettify/uf-dps3xx/actions/workflows/CI.yml)
[![codecov](https://codecov.io/gh/jettify/uf-dps3xx/graph/badge.svg?token=XZK0JJQ9QN)](https://codecov.io/gh/jettify/uf-dps3xx)

`uf-dps3xx` is a platform-agnostic, `no_std` driver for DPS3xx pressure and temperature sensors using `embedded-hal` I2C traits.

SPI support may be added later (the sensor supports it, but this crate currently implements I2C only).


## Installation

Add `uf-dps3xx` to your `Cargo.toml`:

```toml
[dependencies]
uf-dps3xx = "*" # replace * by the latest version of the crate.
```

Or use the command line:

```bash
cargo add uf-dps3xx
```

## Usage

The crate is built around a typed state machine:

- `DPS3xx<_, Unconfigured>` after `new`
- `DPS3xx<_, Configured>` after `init`/`init_and_calibrate`
- `DPS3xx<_, Calibrated>` after calibration

Minimal flow:

```rust
use embedded_hal::delay::DelayNs;
use uf_dps3xx::{Config, DPS3xx};

fn init_sensor<I2C, E, D>(i2c: I2C, address: u8, delay: &mut D)
    -> Result<f32, uf_dps3xx::Error<E>>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    D: DelayNs,
{
    let config = Config::new();
    let sensor = DPS3xx::new(i2c, address, &config)?;
    let mut sensor = sensor.init_and_calibrate(delay)?;
    sensor.read_pressure_calibrated()
}
```

## License

This project is licensed under `Apache-2.0`. See `LICENSE` for details.
