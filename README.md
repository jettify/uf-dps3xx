# uf-dps3xx

[![CI](https://github.com/jettify/uf-dps3xx/actions/workflows/CI.yml/badge.svg)](https://github.com/jettify/uf-dps3xx/actions/workflows/CI.yml)
[![codecov](https://codecov.io/gh/jettify/uf-dps3xx/graph/badge.svg?token=XZK0JJQ9QN)](https://codecov.io/gh/jettify/uf-dps3xx)
[![crates.io](https://img.shields.io/crates/v/uf-dps3xx)](https://crates.io/crates/uf-dps3xx)
[![docs.rs](https://img.shields.io/docsrs/uf-dps3xx)](https://docs.rs/uf-dps3xx/latest/uf_dps3xx/)

`uf-dps3xx` is a platform-agnostic, `no_std` driver for the Infineon `DPS3xx` (`DPS310`, `DPS368`) family of high-precision barometric pressure and temperature sensors.
It uses `embedded-hal` 1.0 I2C traits and provides both blocking (`DelayNs`) and non-blocking poll-driven interfaces.

## Supported Hardware

- `DPS310`
- `DPS368`
- I2C transport (`0x76` primary, `0x77` secondary, or custom address)

## Installation

```toml
[dependencies]
uf-dps3xx = "0.2"
```

Enable `defmt` formatting support with:

```toml
[dependencies]
uf-dps3xx = { version = "0.2", features = ["defmt"] }
```

## Quick Start

```rust
use embedded_hal::delay::DelayNs;
use uf_dps3xx::{Config, Dps3xx, Error, I2cAddress, Sample};

fn read_sample<I2C, E, D>(i2c: I2C, delay: &mut D) -> Result<Sample, Error<E>>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    D: DelayNs,
{
    let config = Config::navigation();
    let poll_ms = config.suggested_poll_period_ms();

    let mut sensor = Dps3xx::new_blocking(i2c, I2cAddress::Primary, config, delay)?;

    loop {
        if let Some(sample) = sensor.try_read_sample()? {
            return Ok(sample);
        }

        delay.delay_ms(poll_ms);
    }
}
```

## Initialization

### Blocking Initialization

Use `Dps3xx::new_blocking` (or `Dps3xx::new` followed by `init_blocking`) when you have an `embedded_hal::delay::DelayNs` implementation.
On completion, the sensor automatically enters the configured operating `Mode` (which defaults to `Mode::BackgroundAll`).

If you want to retain ownership of the `Dps3xx` instance (and recover the I2C bus via `release()`) if initialization fails:

```rust,ignore
let mut sensor = Dps3xx::new(i2c, I2cAddress::Primary, Config::navigation())?;
if let Err(err) = sensor.init_blocking(delay) {
    let i2c = sensor.release();
    return Err(err);
}
```

### Non-Blocking / Scheduler-Driven Initialization

Use `poll_init()` when your application has its own async runtime or tick scheduler:

```rust,ignore
use uf_dps3xx::{Config, Dps3xx, I2cAddress, Poll};

let mut sensor = Dps3xx::new(i2c, I2cAddress::Primary, Config::navigation())?;

while let Poll::Pending { wait_ms } = sensor.poll_init()? {
    // Yield or sleep for at least `wait_ms` milliseconds.
}
```

## Measurements & Decoupled Rates

By default, `Config` starts continuous background pressure and temperature measurement (`Mode::BackgroundAll`) as soon as initialization completes.

Pressure and temperature rates can be configured independently (for example, 32 Hz or 64 Hz pressure paired with 4 Hz temperature in flight controllers):

- `try_read_sample()`: returns `Ok(Some(Sample))` whenever a new pressure measurement is ready, automatically compensating it with the latest cached temperature reading.
- `read_sample()`: returns `Error::NoSampleReady` when a pressure sample is not ready.
- `read_raw_sample()`: burst-reads the raw 24-bit signed pressure and temperature ADC values.
- `try_read_temperature()`: returns a new calibrated temperature in degrees Celsius when ready and updates the temperature cache used for pressure compensation. Supports temperature-only operation without consuming pressure data.
- `read_pressure_raw()`: reads only the signed 24-bit pressure ADC value, leaving temperature data untouched. Check `status()` first when freshness matters.
- `try_pop_fifo()`: pops and decodes a single `FifoEntry` (`FifoEntry::Pressure(Sample)` or `FifoEntry::Temperature(f32)`) when hardware FIFO mode is enabled.

`Sample` contains calibrated pressure in pascals (`pressure_pa`), a helper for hectopascals (`pressure_hpa()`), and temperature in degrees Celsius (`temperature_c`):

```rust
use uf_dps3xx::Sample;

let sample = Sample {
    pressure_pa: 101_325.0,
    temperature_c: 25.0,
};
assert!((sample.pressure_hpa() - 1013.25).abs() < 1e-2);
```

## Configuration & Application Presets

Conversion-budget validation follows the selected mode: background modes count only active
channels, command modes ignore periodic rates, and idle mode has no conversion budget.
Changing modes revalidates the budget before starting measurements.

`TemperatureSource::Auto` selects the sensor associated with the factory calibration.
Explicit `Internal` or `External` selections must match that calibration; otherwise initialization
or reconfiguration returns `Error::InvalidConfig` with `ConfigError::TemperatureSourceMismatch`.

`Config` provides built-in presets matching Infineon datasheet recommendations:

- `Config::weather()` — 1 Hz pressure (`X1`), 1 Hz temperature (`X1`)
- `Config::indoor()` — 16 Hz pressure (`X8`), 2 Hz temperature (`X1`)
- `Config::navigation()` — 32 Hz pressure (`X16`), 4 Hz temperature (`X1`)
- `Config::high_rate()` — 64 Hz pressure (`X8`), 4 Hz temperature (`X1`)

Or customize every field with the `const` builder:

```rust
use uf_dps3xx::{
    Config, FifoConfig, InterruptConfig, Mode, Oversampling, Rate, TemperatureSource,
};

let config = Config::new()
    .pressure(Rate::Hz32, Oversampling::X16)
    .temperature(Rate::Hz4, Oversampling::X1)
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
    .mode(Mode::BackgroundAll)
    .init_timeout_ms(5_000);

assert!(config.validate().is_ok());
```

## Examples

- [examples/simple.rs](https://github.com/jettify/uf-dps3xx/blob/main/examples/simple.rs) shows a mocked poll-based init and sample read flow.
- [examples/tbs-lucid-h7/src/main.rs](https://github.com/jettify/uf-dps3xx/blob/main/examples/tbs-lucid-h7/src/main.rs) shows an Embassy setup for the TBS Lucid H7 flight controller.

## Inspirations

1. <https://github.com/Infineon/arduino-xensiv-dps3xx>
2. <https://github.com/dprotsiv/dps310-rs>

## License

This project is licensed under `Apache-2.0`. See `LICENSE` for details.
