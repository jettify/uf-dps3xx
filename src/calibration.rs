#[derive(Default, Debug, Clone, Copy)]
#[allow(non_snake_case)]
pub(crate) struct CalibrationCoeffs {
    pub C0: i32,
    pub C1: i32,
    pub C00: i32,
    pub C01: i32,
    pub C10: i32,
    pub C11: i32,
    pub C20: i32,
    pub C21: i32,
    pub C30: i32,
}

pub(crate) fn process_calibration_coefficients(
    coeffs: &mut CalibrationCoeffs,
    bytes: &mut [u8; 18],
) {
    coeffs.C0 = get_twos_complement(
        (u32::from(bytes[0]) << 4) | ((u32::from(bytes[1]) >> 4) & 0x0F),
        12,
    );

    coeffs.C1 = get_twos_complement(
        ((u32::from(bytes[1]) & 0x0F) << 8) | u32::from(bytes[2]),
        12,
    );

    coeffs.C00 = get_twos_complement(
        (u32::from(bytes[3]) << 12)
            | (u32::from(bytes[4]) << 4)
            | ((u32::from(bytes[5]) >> 4) & 0x0F),
        20,
    );

    coeffs.C10 = get_twos_complement(
        ((u32::from(bytes[5]) & 0x0F) << 16) | (u32::from(bytes[6]) << 8) | u32::from(bytes[7]),
        20,
    );

    coeffs.C01 = get_twos_complement((u32::from(bytes[8]) << 8) | u32::from(bytes[9]), 16);

    coeffs.C11 = get_twos_complement((u32::from(bytes[10]) << 8) | u32::from(bytes[11]), 16);

    coeffs.C20 = get_twos_complement((u32::from(bytes[12]) << 8) | u32::from(bytes[13]), 16);

    coeffs.C21 = get_twos_complement((u32::from(bytes[14]) << 8) | u32::from(bytes[15]), 16);

    coeffs.C30 = get_twos_complement((u32::from(bytes[16]) << 8) | u32::from(bytes[17]), 16);
}

pub(crate) fn calibrate_pressure(
    coeffs: &CalibrationCoeffs,
    pres_scaled: f32,
    temp_scaled: f32,
) -> f32 {
    (coeffs.C00 as f32)
        + (pres_scaled
            * (coeffs.C10 as f32
                + pres_scaled * (coeffs.C20 as f32 + pres_scaled * coeffs.C30 as f32)))
        + (temp_scaled * coeffs.C01 as f32)
        + (temp_scaled * pres_scaled * (coeffs.C11 as f32 + pres_scaled * coeffs.C21 as f32))
}

pub(crate) fn get_twos_complement(val: u32, length: u8) -> i32 {
    let mut ret = val as i32;
    if (val & ((1) << (length - 1))) > 0 {
        ret -= 1 << length;
    }
    ret
}

#[cfg(test)]
mod tests {
    use super::get_twos_complement;

    #[test]
    fn test_get_twos_complement_negative_value() {
        assert_eq!(get_twos_complement(0b1111_1111_1111, 12), -1);
    }
}
