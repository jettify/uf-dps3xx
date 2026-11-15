#[expect(
    non_camel_case_types,
    reason = "register names match DPS3xx datasheet mnemonics"
)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Register {
    PSR_B2 = 0x00,
    PSR_B1 = 0x01,
    PSR_B0 = 0x02,
    TMP_B2 = 0x03,
    TMP_B1 = 0x04,
    TMP_B0 = 0x05,
    PRS_CFG = 0x06,
    TEMP_CFG = 0x07,
    MEAS_CFG = 0x08,
    CFG_REG = 0x09,
    INT_STS = 0x0A,
    FIFO_STS = 0x0B,
    RESET = 0x0C,
    PROD_ID = 0x0D,

    COEFF_REG_1 = 0x10,
    COEFF_REG_2 = 0x11,
    COEFF_REG_3 = 0x12,
    COEFF_REG_4 = 0x13,
    COEFF_REG_5 = 0x14,
    COEFF_REG_6 = 0x15,
    COEFF_REG_7 = 0x16,
    COEFF_REG_8 = 0x17,
    COEFF_REG_9 = 0x18,
    COEFF_REG_10 = 0x19,
    COEFF_REG_11 = 0x1A,
    COEFF_REG_12 = 0x1B,
    COEFF_REG_13 = 0x1C,
    COEFF_REG_14 = 0x1D,
    COEFF_REG_15 = 0x1E,
    COEFF_REG_16 = 0x1F,
    COEFF_REG_17 = 0x20,
    COEFF_REG_18 = 0x21,
    TMP_COEF_SRCE = 0x28,
}

impl Register {
    #[must_use]
    pub const fn addr(self) -> u8 {
        self as u8
    }
}
