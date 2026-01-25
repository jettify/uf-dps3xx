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
        ((bytes[0] as u32) << 4) | (((bytes[1] as u32) >> 4) & 0x0F),
        12,
    );

    coeffs.C1 = get_twos_complement((((bytes[1] as u32) & 0x0F) << 8) | (bytes[2] as u32), 12);

    coeffs.C00 = get_twos_complement(
        ((bytes[3] as u32) << 12) | ((bytes[4] as u32) << 4) | (((bytes[5] as u32) >> 4) & 0x0F),
        20,
    );

    coeffs.C10 = get_twos_complement(
        (((bytes[5] as u32) & 0x0F) << 16) | ((bytes[6] as u32) << 8) | (bytes[7] as u32),
        20,
    );

    coeffs.C01 = get_twos_complement(((bytes[8] as u32) << 8) | (bytes[9] as u32), 16);

    coeffs.C11 = get_twos_complement(((bytes[10] as u32) << 8) | (bytes[11] as u32), 16);

    coeffs.C20 = get_twos_complement(((bytes[12] as u32) << 8) | (bytes[13] as u32), 16);

    coeffs.C21 = get_twos_complement(((bytes[14] as u32) << 8) | (bytes[15] as u32), 16);

    coeffs.C30 = get_twos_complement(((bytes[16] as u32) << 8) | (bytes[17] as u32), 16);
}

pub(crate) fn calibrate_preasure(
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
