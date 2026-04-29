#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c, Master};
use embassy_stm32::mode::Blocking;
use embassy_stm32::time::Hertz;
use embassy_stm32::Config as StmConfig;
use embassy_time::Timer;
use uf_dps3xx::{Config as DpsConfig, Dps3xx, I2cAddress, Poll, Uninit};
use {defmt_rtt as _, panic_probe as _};

type BoardI2c = I2c<'static, Blocking, Master>;

async fn init_async(
    i2c: BoardI2c,
    config: DpsConfig,
) -> Result<
    uf_dps3xx::Dps3xx<BoardI2c, uf_dps3xx::Ready>,
    uf_dps3xx::Error<embassy_stm32::i2c::Error>,
> {
    let sensor = Dps3xx::<_, Uninit>::new_i2c(i2c, I2cAddress::Primary, config)?;
    let mut init = sensor.init().map_err(|e| e.into_error())?;

    loop {
        match init.poll()? {
            Poll::Pending { wait_ms } => Timer::after_millis(u64::from(wait_ms)).await,
            Poll::Ready(sensor) => return Ok(sensor),
        }
    }
}

#[embassy_executor::task]
async fn blink_task(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after_millis(100).await;
        led.set_low();
        Timer::after_millis(900).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = StmConfig::default();
    let p = embassy_stm32::init(config);

    let led = Output::new(p.PE3, Level::High, Speed::Low);
    spawner.spawn(blink_task(led).unwrap());

    let mut cfg = I2cConfig::default();
    cfg.frequency = Hertz::khz(400);

    let i2c = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, cfg);

    let dps_config = DpsConfig::default();
    let mut dps = init_async(i2c, dps_config).await.expect("dps init failed");

    dps.start_background()
        .expect("dps start background measurement failed");

    loop {
        if let Some(sample) = dps.try_read_sample().expect("dps read sample failed") {
            info!(
                "pressure {} temp {}",
                sample.pressure_pa, sample.temperature_c
            );
        }
        Timer::after_millis(u64::from(dps.config().suggested_poll_period_ms())).await;
    }
}
