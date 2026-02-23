use embedded_hal::{delay::DelayNs, i2c::I2c};
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use uf_dps3xx::{
    calc_total_wait_ms, Config, Configured, DPS3xx, Error, InitInProgress, InitPoll,
    MeasurementMode, PressureRate, PressureResolution, Register, TemperatureRate,
    TemperatureResolution,
};

const ADDR: u8 = 0x77;

struct TestDelay;

impl DelayNs for TestDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}

fn finish_init<I2C>(dps: DPS3xx<I2C, InitInProgress>) -> DPS3xx<I2C, Configured>
where
    I2C: I2c,
{
    match dps.finish_init() {
        Ok(dps) => dps,
        Err(_) => panic!("init not ready"),
    }
}

fn poll_init_ready<I2C>(dps: &mut DPS3xx<I2C, InitInProgress>)
where
    I2C: I2c,
{
    assert!(matches!(dps.poll_init().unwrap(), InitPoll::Pending(_)));
    assert!(matches!(dps.poll_init().unwrap(), InitPoll::Ready));
}

#[test]
fn test_new_dps3xx_defaults() {
    let expectations = [
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
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0x00, 0x00, 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS3xx::new(i2c.clone(), ADDR, &config).unwrap();
    let mut dps = dps.start_init().unwrap();
    poll_init_ready(&mut dps);
    let _dps = finish_init(dps);
    i2c.done();
}

#[test]
fn test_finish_init_requires_poll() {
    let expectations = [
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
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0x00, 0x00, 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS3xx::new(i2c.clone(), ADDR, &config).unwrap();
    let dps = dps.start_init().unwrap();

    let mut dps = match dps.finish_init() {
        Ok(_) => panic!("finish_init should require poll_init"),
        Err(dps) => dps,
    };

    poll_init_ready(&mut dps);
    let _dps = finish_init(dps);
    i2c.done();
}

#[test]
fn test_poll_init_pending_reports_configured_wait_time() {
    let expectations = [
        I2cTransaction::write_read(ADDR, vec![Register::PROD_ID.addr()], vec![0x10]),
        I2cTransaction::write_read(ADDR, vec![Register::PRS_CFG.addr()], vec![0x00]),
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::TEMP_CFG.addr()], vec![0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_COEF_SRCE.addr()], vec![0x00]),
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), 0x34]),
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0x08]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![0x0E, 0xA5]),
        I2cTransaction::write(ADDR, vec![0x0F, 0x96]),
        I2cTransaction::write(ADDR, vec![0x62, 0x02]),
        I2cTransaction::write(ADDR, vec![0x0E, 0x00]),
        I2cTransaction::write(ADDR, vec![0x0F, 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x00]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let mut config = Config::new();
    config
        .temp_rate(TemperatureRate::_8_SPS)
        .temp_res(TemperatureResolution::_16_SAMPLES);

    let dps = DPS3xx::new(i2c.clone(), ADDR, &config).unwrap();
    let mut dps = dps.start_init().unwrap();

    let expected_wait = calc_total_wait_ms(
        TemperatureRate::_8_SPS as u8,
        TemperatureResolution::_16_SAMPLES as u8,
    );

    assert!(matches!(
        dps.poll_init().unwrap(),
        InitPoll::Pending(wait) if wait == expected_wait
    ));
    i2c.done();
}

#[test]
fn test_read_calibration_coefficients() {
    let expectations = [
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
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0x00, 0x00, 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x80]),
        I2cTransaction::write_read(ADDR, vec![Register::COEFF_REG_1.addr()], vec![0; 18]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS3xx::new(i2c.clone(), ADDR, &config).unwrap();
    let mut dps = dps.start_init().unwrap();
    poll_init_ready(&mut dps);
    let dps = finish_init(dps);

    let _dps = dps.read_calibration_coefficients().unwrap();
    i2c.done();
}

#[test]
fn test_read_calibration_coefficients_requires_coef_ready() {
    let expectations = [
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
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0x00, 0x00, 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x00]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS3xx::new(i2c.clone(), ADDR, &config).unwrap();
    let mut dps = dps.start_init().unwrap();
    poll_init_ready(&mut dps);
    let dps = finish_init(dps);

    assert!(matches!(
        dps.read_calibration_coefficients(),
        Err(Error::CoefficientsNotReady)
    ));
    i2c.done();
}

#[test]
fn test_init_and_calibrate() {
    let expectations = [
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
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0x00, 0x00, 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x80]),
        I2cTransaction::write_read(ADDR, vec![Register::COEFF_REG_1.addr()], vec![0; 18]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS3xx::new(i2c.clone(), ADDR, &config).unwrap();
    let mut delay = TestDelay;

    let _dps = dps.init_and_calibrate(&mut delay).unwrap();
    i2c.done();
}

#[test]
fn test_start_measurement() {
    let expectations = [
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
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0x00, 0x00, 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x02]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS3xx::new(i2c.clone(), ADDR, &config).unwrap();
    let mut dps = dps.start_init().unwrap();
    poll_init_ready(&mut dps);
    let mut dps = finish_init(dps);

    dps.start_measurement(MeasurementMode::OneShotTemperature)
        .unwrap();
    i2c.done();
}

#[test]
fn test_read_temp_calibrated() {
    let expectations = [
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
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0x00, 0x00, 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x80]),
        I2cTransaction::write_read(ADDR, vec![Register::COEFF_REG_1.addr()], vec![0; 18]),
        I2cTransaction::write_read(ADDR, vec![Register::TEMP_CFG.addr()], vec![0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0x00, 0x00, 0x00]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS3xx::new(i2c.clone(), ADDR, &config).unwrap();
    let mut dps = dps.start_init().unwrap();
    poll_init_ready(&mut dps);
    let dps = finish_init(dps);
    let mut dps = dps.read_calibration_coefficients().unwrap();

    let _temp = dps.read_temp_calibrated().unwrap();
    i2c.done();
}

#[test]
fn test_status_and_ready_flags() {
    let expectations = [
        // Init
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
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0x00, 0x00, 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0xF0]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x80]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x40]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x20]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x10]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS3xx::new(i2c.clone(), ADDR, &config).unwrap();
    let mut dps = dps.start_init().unwrap();
    poll_init_ready(&mut dps);
    let mut dps = finish_init(dps);

    assert_eq!(dps.read_status().unwrap(), 0xF0);
    assert!(dps.coef_ready().unwrap());
    assert!(dps.init_complete().unwrap());
    assert!(dps.temp_ready().unwrap());
    assert!(dps.pres_ready().unwrap());
    i2c.done();
}

#[test]
fn test_read_pressure_calibrated() {
    let expectations = [
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
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0x00, 0x00, 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x80]),
        I2cTransaction::write_read(ADDR, vec![Register::COEFF_REG_1.addr()], vec![0; 18]),
        I2cTransaction::write_read(ADDR, vec![Register::PRS_CFG.addr()], vec![0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::PSR_B2.addr()], vec![0x00, 0x04, 0x00]), // 1024
        I2cTransaction::write_read(ADDR, vec![Register::TEMP_CFG.addr()], vec![0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0x00, 0x04, 0x00]), // 1024
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS3xx::new(i2c.clone(), ADDR, &config).unwrap();
    let mut dps = dps.start_init().unwrap();
    poll_init_ready(&mut dps);
    let dps = finish_init(dps);
    let mut dps = dps.read_calibration_coefficients().unwrap();

    let pres = dps.read_pressure_calibrated().unwrap();
    // With all coeffs 0 and raw values 1024, scaled values 1024/524288 = 0.001953125
    // pres_cal = C00 + pres_scaled * (C10 + pres_scaled * (C20 + pres_scaled * C30)) + ...
    // = 0 + 0.00195... * (0 + ...) = 0
    assert_eq!(pres, 0.0);
    i2c.done();
}

#[test]
fn test_reset() {
    let expectations = [
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
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0x00, 0x00, 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::RESET.addr(), 0x89]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS3xx::new(i2c.clone(), ADDR, &config).unwrap();
    let mut dps = dps.start_init().unwrap();
    poll_init_ready(&mut dps);
    let dps = finish_init(dps);

    let _dps = dps.reset().unwrap();
    i2c.done();
}

#[test]
fn test_release() {
    let expectations = [];

    let i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS3xx::new(i2c.clone(), ADDR, &config).unwrap();

    let mut released_i2c = dps.release();
    released_i2c.done();
}

#[test]
fn test_start_init_accepts_revision_variants() {
    let expectations = [
        I2cTransaction::write_read(ADDR, vec![Register::PROD_ID.addr()], vec![0x1F]),
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
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS3xx::new(i2c.clone(), ADDR, &config).unwrap();

    assert!(dps.start_init().is_ok());
    i2c.done();
}

#[test]
fn test_start_init_rejects_wrong_product_family() {
    let expectations = [I2cTransaction::write_read(
        ADDR,
        vec![Register::PROD_ID.addr()],
        vec![0x20],
    )];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS3xx::new(i2c.clone(), ADDR, &config).unwrap();

    assert!(dps.start_init().is_err());
    i2c.done();
}

#[test]
fn test_start_measurement_rejects_background_busytime_overflow() {
    let expectations = [
        I2cTransaction::write_read(ADDR, vec![Register::PROD_ID.addr()], vec![0x10]),
        I2cTransaction::write_read(ADDR, vec![Register::PRS_CFG.addr()], vec![0x00]),
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 0x77]),
        I2cTransaction::write_read(ADDR, vec![Register::TEMP_CFG.addr()], vec![0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_COEF_SRCE.addr()], vec![0x00]),
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), 0x77]),
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0x0C]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![0x0E, 0xA5]),
        I2cTransaction::write(ADDR, vec![0x0F, 0x96]),
        I2cTransaction::write(ADDR, vec![0x62, 0x02]),
        I2cTransaction::write(ADDR, vec![0x0E, 0x00]),
        I2cTransaction::write(ADDR, vec![0x0F, 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x40]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x02]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x60]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0x00, 0x00, 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let mut config = Config::new();
    config
        .temp_rate(TemperatureRate::_128_SPS)
        .temp_res(TemperatureResolution::_128_SAMPLES)
        .pres_rate(PressureRate::_128_SPS)
        .pres_res(PressureResolution::_128_SAMPLES);

    let dps = DPS3xx::new(i2c.clone(), ADDR, &config).unwrap();
    let mut dps = dps.start_init().unwrap();
    poll_init_ready(&mut dps);
    let mut dps = finish_init(dps);

    assert!(matches!(
        dps.start_measurement(MeasurementMode::BackgroundPressureAndTemperature),
        Err(Error::BusyTimeExceeded)
    ));
    i2c.done();
}
