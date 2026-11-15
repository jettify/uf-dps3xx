# uf-dps3xx

[![CI](https://github.com/jettify/uf-dps3xx/actions/workflows/CI.yml/badge.svg)](https://github.com/jettify/uf-dps3xx/actions/workflows/CI.yml)
[![codecov](https://codecov.io/gh/jettify/uf-dps3xx/graph/badge.svg?token=XZK0JJQ9QN)](https://codecov.io/gh/jettify/uf-dps3xx)
[![crates.io](https://img.shields.io/crates/v/uf-dps3xx)](https://crates.io/crates/uf-dps3xx)
[![docs.rs](https://img.shields.io/docsrs/uf-dps3xx)](https://docs.rs/uf-dps3xx/latest/uf_dps3xx/)

`uf-dps3xx` is a platform-agnostic, `no_std` driver for the Infineon `DPS3xx` (`DPS310`, `DPS368`) family of high-precision barometric pressure and temperature sensors.
It supports `embedded-hal` 1.0 blocking I2C and optional `embedded-hal-async` 1.0 I2C.
Initialization can use a HAL delay or application-controlled polling. Requires Rust 1.81 or newer.

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

## Native Async Support

Enable the `async` feature to use `AsyncDps3xx`. The blocking `Dps3xx` remains available.
The async driver works with any runtime providing `embedded-hal-async` I2C and delay implementations;
it requires no allocator or runtime dependency.

```toml
[dependencies]
uf-dps3xx = { version = "0.2", features = ["async"] }
```

```rust
# #[cfg(feature = "async")]
# mod async_example {
use embedded_hal_async::{delay::DelayNs, i2c::I2c};
use uf_dps3xx::{AsyncDps3xx, Config, Error, I2cAddress, Sample};

async fn read_sample<I: I2c, D: DelayNs>(
    i2c: I,
    delay: &mut D,
) -> Result<Sample, Error<I::Error>> {
    let config = Config::navigation();
    let poll_ms = config.suggested_poll_period_ms();
    let mut sensor = AsyncDps3xx::new_async(
        i2c, I2cAddress::Primary, config, delay,
    ).await?;

    loop {
        if let Some(sample) = sensor.try_read_sample().await? {
            return Ok(sample);
        }
        delay.delay_ms(poll_ms).await;
    }
}
# }
```

For Embassy, pass an async I2C peripheral and `embassy_time::Delay`.
Every method that accesses I2C is async, including `poll_init()`, `reconfigure()`,
`set_mode()`, and `try_pop_fifo()`. `new()`, `new_i2c()`, `config()`, `is_ready()`,
`init_stage()`, and `release()` are synchronous.

`try_read_sample().await` awaits I2C transactions and returns `None` if no pressure
sample is ready. It does not wait for a new sensor conversion. `poll_init().await`
likewise awaits I2C; callers must still wait the returned `wait_ms` between polls.

### Async Cancellation and Recovery

Prefer `AsyncDps3xx::new()` followed by `init_async(&mut delay).await` when retaining
ownership of the driver on failure or cancellation matters. Cancelling `new_async()`
drops the driver and its I2C peripheral.

Cancelling or failing an async `reconfigure()` or `set_mode()` invalidates readiness
before the first write. The cached configuration is committed only on success.
After an interrupted `set_mode()`, the driver reports `InitStage::ApplyMode`. Retry
`set_mode()`, `standby()`, or `start_background()` directly to select a mode, or call
`init_async()` / `poll_init()` to restore the last committed mode. These mode-only
recovery paths write only `MEAS_CFG`, preserving FIFO entries and configuration registers.

After an interrupted `reconfigure()`, call `init_async()` or `poll_init()` to restore
the last committed configuration, or retry `reconfigure()` with the requested config.
Full reconfiguration recovery flushes FIFO when enabled. Mode-only retries are rejected
while full configuration recovery is pending; finish that recovery before changing modes.
A cancelled or failed `reset()` requires initialization again.

Initialization can be retried with `init_async()` after cancellation. If its priming
delay was interrupted, it starts a fresh temperature conversion. After cancelling
`poll_init()` itself, use `init_async()` to handle the remaining timing automatically.
Cancelled reads may consume a sample, clear an interrupt, or pop a FIFO entry without
returning it. A cancelled temperature read can leave pressure compensation using the
previous cached temperature; reset and reinitialize if that reference must be refreshed
before continuing. Recovery assumes the underlying HAL permits reuse after cancellation;
otherwise recover the bus with `release()` and follow the HAL's recovery procedure.

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

Use `poll_init()` when your application has its own tick scheduler. With `Dps3xx`,
I2C transactions still block; use `AsyncDps3xx` for native async I2C:

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
