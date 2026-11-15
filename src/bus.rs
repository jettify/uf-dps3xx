#[maybe_async_cfg::maybe(
    sync(self = "blocking"),
    async(self = "asynchronous", feature = "async"),
    idents(
        I2cBus(sync, async = "AsyncI2cBus"),
        embedded_hal_async(sync = "embedded_hal", async)
    )
)]
mod implementation {
    use crate::register::Register;
    use embedded_hal_async::i2c::I2c;
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

    impl<I2C, I2CError> I2cBus<I2C>
    where
        I2C: I2c<Error = I2CError>,
    {
        pub(crate) async fn write_reg(&mut self, reg: Register, value: u8) -> Result<(), I2CError> {
            let bytes = [reg.addr(), value];
            self.i2c.write(self.address, &bytes).await
        }

        pub(crate) async fn write_addr(&mut self, addr: u8, value: u8) -> Result<(), I2CError> {
            let bytes = [addr, value];
            self.i2c.write(self.address, &bytes).await
        }

        pub(crate) async fn read_reg(&mut self, reg: Register) -> Result<u8, I2CError> {
            let mut buffer: [u8; 1] = [0];
            self.i2c
                .write_read(self.address, &[reg.addr()], &mut buffer)
                .await?;
            Ok(buffer[0])
        }

        pub(crate) async fn read_many(
            &mut self,
            start: Register,
            buf: &mut [u8],
        ) -> Result<(), I2CError> {
            self.i2c
                .write_read(self.address, &[start.addr()], buf)
                .await
        }
    }
}
#[cfg(feature = "async")]
pub(crate) use asynchronous::AsyncI2cBus;
pub(crate) use blocking::I2cBus;
