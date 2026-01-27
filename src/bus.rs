use crate::register::Register;
use embedded_hal::i2c::I2c;

pub(crate) trait Bus {
    type Error;

    fn write_reg(&mut self, reg: Register, value: u8) -> Result<(), Self::Error>;
    fn read_reg(&mut self, reg: Register) -> Result<u8, Self::Error>;
    fn read_many(&mut self, start: Register, buf: &mut [u8]) -> Result<(), Self::Error>;
}

pub(crate) struct I2cBus<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> I2cBus<I2C> {
    pub(crate) fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    pub(crate) fn release(self) -> I2C {
        self.i2c
    }
}

impl<I2C, I2CError> Bus for I2cBus<I2C>
where
    I2C: I2c<Error = I2CError>,
{
    type Error = I2CError;

    fn write_reg(&mut self, reg: Register, value: u8) -> Result<(), Self::Error> {
        let bytes = [reg.addr(), value];
        self.i2c.write(self.address, &bytes)
    }

    fn read_reg(&mut self, reg: Register) -> Result<u8, Self::Error> {
        let mut buffer: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[reg.addr()], &mut buffer)?;
        Ok(buffer[0])
    }

    fn read_many(&mut self, start: Register, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[start.addr()], buf)
    }
}
