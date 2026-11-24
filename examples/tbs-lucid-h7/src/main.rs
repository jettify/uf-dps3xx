#![no_std]
#![no_main]

use bsp::hal::{i2c, time::Hertz};
use cortex_m as _;
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Ticker};
use panic_probe as _;
use tbs_lucid_h7_bsp as bsp;
use uf_dps3xx::{Config, Dps3xx, I2cAddress};

const CONFIG: Config = Config::navigation();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let board = bsp::Board::new(bsp::hal::init(bsp::config()));

    let mut i2c_cfg = i2c::Config::default();
    i2c_cfg.frequency = Hertz::khz(400);
    let i2c = board.baro.new_i2c_blocking(i2c_cfg);

    let mut dps = Dps3xx::new_blocking(i2c, I2cAddress::Primary, CONFIG, &mut Delay)
        .expect("dps init failed");
    let mut ticker = Ticker::every(Duration::from_millis(u64::from(
        CONFIG.suggested_poll_period_ms(),
    )));

    loop {
        ticker.next().await;
        if let Some(sample) = dps.try_read_sample().expect("dps read sample failed") {
            info!(
                "pressure={} Pa ({} hPa) temp={} C",
                sample.pressure_pa,
                sample.pressure_hpa(),
                sample.temperature_c
            );
        }
    }
}
