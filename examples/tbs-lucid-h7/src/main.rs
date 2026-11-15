#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, dma, i2c, peripherals, Config as StmConfig};
use embassy_time::{Delay, Timer};
use uf_dps3xx::{AsyncDps3xx, Config as DpsConfig, I2cAddress};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
    DMA1_STREAM0 => dma::InterruptHandler<peripherals::DMA1_CH0>;
    DMA1_STREAM1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
});

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

    let i2c = I2c::new(p.I2C2, p.PB10, p.PB11, p.DMA1_CH0, p.DMA1_CH1, Irqs, cfg);

    let mut dps = AsyncDps3xx::new_async(
        i2c,
        I2cAddress::Primary,
        DpsConfig::navigation(),
        &mut Delay,
    )
    .await
    .expect("dps init failed");

    loop {
        if let Some(sample) = dps.try_read_sample().await.expect("dps read sample failed") {
            info!(
                "pressure {} Pa ({} hPa) temp {} C",
                sample.pressure_pa,
                sample.pressure_hpa(),
                sample.temperature_c
            );
        }
        Timer::after_millis(u64::from(dps.config().suggested_poll_period_ms())).await;
    }
}
