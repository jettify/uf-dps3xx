use embedded_hal::delay::DelayNs;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use uf_dps3xx::{Config, DPS3xx, Register};

const ADDR: u8 = 0x77;

struct NoopDelay;

impl DelayNs for NoopDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}

fn main() {
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
    let mut delay = NoopDelay;
    let mut dps = dps.init_and_calibrate(&mut delay).unwrap();
    let pres = dps.read_pressure_calibrated().unwrap();
    i2c.done();
    println!("Done: {pres}")
}
