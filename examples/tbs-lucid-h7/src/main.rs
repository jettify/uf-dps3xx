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
use uf_dps3xx::{
    Calibrated, Config as DspConfig, DPS3xx, Error as DpsError, InitInProgress, InitPoll,
    MeasurementMode,
};
use {defmt_rtt as _, panic_probe as _};

type BoardI2c = I2c<'static, Blocking, Master>;

async fn wait_for_init(
    dps: &mut DPS3xx<BoardI2c, InitInProgress>,
) -> Result<(), DpsError<embassy_stm32::i2c::Error>> {
    loop {
        match dps.poll_init()? {
            InitPoll::Ready => return Ok(()),
            InitPoll::Pending(wait_ms) => Timer::after_millis(wait_ms as u64).await,
        }
    }
}

async fn wait_until_ready<S>(
    dps: &mut DPS3xx<BoardI2c, S>,
    mut ready: impl FnMut(&mut DPS3xx<BoardI2c, S>) -> Result<bool, DpsError<embassy_stm32::i2c::Error>>,
) -> Result<(), DpsError<embassy_stm32::i2c::Error>> {
    while !ready(dps)? {
        Timer::after_millis(10).await;
    }
    Ok(())
}

async fn init_and_calibrate_async(
    i2c: BoardI2c,
    address: u8,
    config: &DspConfig,
) -> Result<DPS3xx<BoardI2c, Calibrated>, DpsError<embassy_stm32::i2c::Error>> {
    let dps = DPS3xx::new(i2c, address, config)?;
    let mut dps = dps.start_init()?;
    wait_for_init(&mut dps).await?;
    let mut dps = dps
        .finish_init()
        .unwrap_or_else(|_| panic!("dps init not ready"));
    wait_until_ready(&mut dps, DPS3xx::coef_ready).await?;
    Ok(dps.read_calibration_coefficients()?)
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

    const ADDR: u8 = 0x76;
    let dps_config = DspConfig::new();
    let mut dps = init_and_calibrate_async(i2c, ADDR, &dps_config)
        .await
        .expect("dps init failed");

    dps.start_measurement(MeasurementMode::BackgroundPressureAndTemperature)
        .expect("dps start background measurement failed");

    loop {
        wait_until_ready(&mut dps, DPS3xx::pres_ready)
            .await
            .expect("dps pres ready failed");
        let pressure = dps.read_pressure_calibrated().unwrap();
        wait_until_ready(&mut dps, DPS3xx::temp_ready)
            .await
            .expect("dps temp ready failed");
        let temperature = dps.read_temp_calibrated().unwrap();
        info!("pressure {} temp {}", pressure, temperature);
        Timer::after_millis(500).await;
    }
}
