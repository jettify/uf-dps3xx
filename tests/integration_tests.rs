use embedded_hal::delay::DelayNs;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use uf_dps3xx::{advanced::Register, Config, Dps3xx, I2cAddress, Poll, Sample, Uninit};

const ADDR: u8 = 0x76;

struct NoopDelay;

impl DelayNs for NoopDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}

fn init_expectations() -> Vec<I2cTransaction> {
    vec![
        I2cTransaction::write_read(ADDR, vec![Register::PROD_ID.addr()], vec![0x10]),
        I2cTransaction::write_read(ADDR, vec![Register::PRS_CFG.addr()], vec![0x00]),
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::TEMP_CFG.addr()], vec![0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_COEF_SRCE.addr()], vec![0x00]),
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![0x0E, 0xA5]),
        I2cTransaction::write(ADDR, vec![0x0F, 0x96]),
        I2cTransaction::write(ADDR, vec![0x62, 0x02]),
        I2cTransaction::write(ADDR, vec![0x0E, 0x00]),
        I2cTransaction::write(ADDR, vec![0x0F, 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x40]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x02]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x60]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0, 0, 0]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x80]),
        I2cTransaction::write_read(ADDR, vec![Register::COEFF_REG_1.addr()], vec![0; 18]),
    ]
}

fn init_sensor(i2c: I2cMock) -> Dps3xx<I2cMock, uf_dps3xx::Ready> {
    let config = Config::default();
    let sensor = Dps3xx::<_, Uninit>::new_i2c(i2c, I2cAddress::Primary, config).unwrap();
    let mut init = sensor.init().unwrap();
    loop {
        match init.poll().unwrap() {
            Poll::Pending { .. } => {}
            Poll::Ready(sensor) => break sensor,
        }
    }
}

#[test]
fn init_poll_completes() {
    let mut i2c = I2cMock::new(&init_expectations());
    let _sensor = init_sensor(i2c.clone());
    i2c.done();
}

#[test]
fn init_cancel_after_ready_returns_none() {
    let mut i2c = I2cMock::new(&init_expectations());
    let config = Config::default();
    let sensor = Dps3xx::<_, Uninit>::new_i2c(i2c.clone(), I2cAddress::Primary, config).unwrap();
    let mut init = sensor.init().unwrap();

    loop {
        match init.poll().unwrap() {
            Poll::Pending { .. } => {}
            Poll::Ready(_sensor) => break,
        }
    }

    assert!(init.cancel().is_none());
    i2c.done();
}

#[test]
fn try_read_sample_none_when_not_ready() {
    let mut expectations = init_expectations();
    expectations.push(I2cTransaction::write_read(
        ADDR,
        vec![Register::MEAS_CFG.addr()],
        vec![0x00],
    ));

    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = init_sensor(i2c.clone());

    let sample = sensor.try_read_sample().unwrap();
    assert!(sample.is_none());
    i2c.done();
}

#[test]
fn try_read_sample_reads_burst() {
    let mut expectations = init_expectations();
    expectations.extend([
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x30]),
        I2cTransaction::write_read(ADDR, vec![Register::PSR_B2.addr()], vec![0, 0, 0, 0, 0, 0]),
    ]);

    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = init_sensor(i2c.clone());

    let sample = sensor.try_read_sample().unwrap();
    assert_eq!(
        sample,
        Some(Sample {
            pressure_pa: 0.0,
            temperature_c: 0.0
        })
    );
    i2c.done();
}

#[test]
fn init_blocking_completes() {
    let mut i2c = I2cMock::new(&init_expectations());
    let mut delay = NoopDelay;
    let config = Config::default();

    let sensor = Dps3xx::<_, Uninit>::new_i2c(i2c.clone(), I2cAddress::Primary, config).unwrap();
    let _sensor = sensor.init_blocking(&mut delay).unwrap();
    i2c.done();
}
