use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use uf_dps310::{Config, Register, DPS310};

const ADDR: u8 = 0x77;

#[test]
fn test_new_dps310_defaults() {
    let expectations = [
        // get_product_id -> read PROD_ID (0x0D) -> returns 0x10
        I2cTransaction::write_read(ADDR, vec![Register::PROD_ID.addr()], vec![0x10]),
        // apply_config
        // read PRS_CFG (0x06)
        I2cTransaction::write_read(ADDR, vec![Register::PRS_CFG.addr()], vec![0x00]),
        // write PRS_CFG (0x06) -> default config writes 0x00
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 0x00]),
        // read TEMP_CFG (0x07)
        I2cTransaction::write_read(ADDR, vec![Register::TEMP_CFG.addr()], vec![0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_COEF_SRCE.addr()], vec![0x00]),
        // write TEMP_CFG (0x07) -> default 0x00
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), 0x00]),
        // write CFG_REG (0x09) -> default 0x00
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0x00]),
        // standby -> write MEAS_CFG (0x08) -> 0x00
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS310::new(i2c.clone(), ADDR, &config).unwrap();
    let _dps = dps.init().unwrap();
    i2c.done();
}

#[test]
fn test_read_calibration_coefficients() {
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
        // read_calibration_coefficients
        // read 18 bytes from COEFF_REG_1 (0x10)
        I2cTransaction::write_read(ADDR, vec![Register::COEFF_REG_1.addr()], vec![0; 18]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS310::new(i2c.clone(), ADDR, &config).unwrap();
    let dps = dps.init().unwrap();

    let _dps = dps.read_calibration_coefficients().unwrap();
    i2c.done();
}

#[test]
fn test_trigger_measurement() {
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
        // trigger_measurement(temp=true, pres=false, continuous=false)
        // read MEAS_CFG (0x08)
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x00]),
        // write MEAS_CFG (0x08)
        // continuous=0, temp=1, pres=0 -> 0b010 -> 0x02
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x02]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS310::new(i2c.clone(), ADDR, &config).unwrap();
    let mut dps = dps.init().unwrap();

    dps.trigger_measurement(true, false, false).unwrap();
    i2c.done();
}

#[test]
fn test_read_temp_calibrated() {
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
        // read_calibration_coefficients
        I2cTransaction::write_read(ADDR, vec![Register::COEFF_REG_1.addr()], vec![0; 18]),
        // read_temp_raw via read_temp_calibrated
        // reads TMP_B2 (0x03) -> 3 bytes
        // 0x00, 0x00, 0x00 -> 0
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0x00, 0x00, 0x00]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS310::new(i2c.clone(), ADDR, &config).unwrap();
    let dps = dps.init().unwrap();
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
        // read_status -> MEAS_CFG (0x08) -> returns 0xF0 (all ready bits set)
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0xF0]),
        // coef_ready -> read_status -> returns 0x80
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x80]),
        // init_complete -> read_status -> returns 0x40
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x40]),
        // temp_ready -> read_status -> returns 0x20
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x20]),
        // pres_ready -> read_status -> returns 0x10
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x10]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS310::new(i2c.clone(), ADDR, &config).unwrap();
    let mut dps = dps.init().unwrap();

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
        // Init
        I2cTransaction::write_read(ADDR, vec![Register::PROD_ID.addr()], vec![0x10]),
        I2cTransaction::write_read(ADDR, vec![Register::PRS_CFG.addr()], vec![0x00]),
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::TEMP_CFG.addr()], vec![0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_COEF_SRCE.addr()], vec![0x00]),
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        // read_calibration_coefficients
        I2cTransaction::write_read(ADDR, vec![Register::COEFF_REG_1.addr()], vec![0; 18]),
        // read_pressure_calibrated
        // 1. read_pressure_scaled -> read_pressure_raw -> PSR_B2 (0x00)
        I2cTransaction::write_read(ADDR, vec![Register::PSR_B2.addr()], vec![0x00, 0x04, 0x00]), // 1024
        // 2. read_temp_scaled -> read_temp_raw -> TMP_B2 (0x03)
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0x00, 0x04, 0x00]), // 1024
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS310::new(i2c.clone(), ADDR, &config).unwrap();
    let dps = dps.init().unwrap();
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
        // Init
        I2cTransaction::write_read(ADDR, vec![Register::PROD_ID.addr()], vec![0x10]),
        I2cTransaction::write_read(ADDR, vec![Register::PRS_CFG.addr()], vec![0x00]),
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::TEMP_CFG.addr()], vec![0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_COEF_SRCE.addr()], vec![0x00]),
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        // reset -> write RESET (0x0C) -> 0x89
        I2cTransaction::write(ADDR, vec![Register::RESET.addr(), 0x89]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let config = Config::new();
    let dps = DPS310::new(i2c.clone(), ADDR, &config).unwrap();
    let dps = dps.init().unwrap();

    let _dps = dps.reset().unwrap();
    i2c.done();
}
