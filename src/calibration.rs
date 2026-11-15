#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub(crate) struct CalibrationCoeffs {
    pub c0_half: f32,
    pub c1: f32,
    pub c00: f32,
    pub c01: f32,
    pub c10: f32,
    pub c11: f32,
    pub c20: f32,
    pub c21: f32,
    pub c30: f32,
}

#[inline]
pub(crate) const fn decode_i12_high(b0: u8, b1: u8) -> i16 {
    i16::from_be_bytes([b0, b1 & 0xF0]) >> 4
}

#[inline]
pub(crate) const fn decode_i12_low(b0: u8, b1: u8) -> i16 {
    i16::from_be_bytes([(b0 << 4) | (b1 >> 4), b1 << 4]) >> 4
}

#[inline]
pub(crate) fn decode_i24(bytes: [u8; 3]) -> i32 {
    let hi = i16::from_be_bytes([bytes[0], bytes[1]]);
    i32::from(hi) * 256 + i32::from(bytes[2])
}

#[inline]
pub(crate) fn i24_to_f32(raw: i32) -> f32 {
    let bytes = raw.to_be_bytes();
    let hi = i16::from_be_bytes([bytes[1], bytes[2]]);
    f32::from(hi) * 256.0 + f32::from(bytes[3])
}

pub(crate) fn process_calibration_coefficients(coeffs: &mut CalibrationCoeffs, bytes: &[u8; 18]) {
    let c0 = decode_i12_high(bytes[0], bytes[1]);
    let c1 = decode_i12_low(bytes[1], bytes[2]);

    let c00_hi = i16::from_be_bytes([bytes[3], bytes[4]]);
    let c00_lo = bytes[5] >> 4;

    let c10_hi = decode_i12_low(bytes[5], bytes[6]);
    let c10_lo = bytes[7];

    let c01 = i16::from_be_bytes([bytes[8], bytes[9]]);
    let c11 = i16::from_be_bytes([bytes[10], bytes[11]]);
    let c20 = i16::from_be_bytes([bytes[12], bytes[13]]);
    let c21 = i16::from_be_bytes([bytes[14], bytes[15]]);
    let c30 = i16::from_be_bytes([bytes[16], bytes[17]]);

    coeffs.c0_half = f32::from(c0) * 0.5;
    coeffs.c1 = f32::from(c1);
    coeffs.c00 = f32::from(c00_hi) * 16.0 + f32::from(c00_lo);
    coeffs.c10 = f32::from(c10_hi) * 256.0 + f32::from(c10_lo);
    coeffs.c01 = f32::from(c01);
    coeffs.c11 = f32::from(c11);
    coeffs.c20 = f32::from(c20);
    coeffs.c21 = f32::from(c21);
    coeffs.c30 = f32::from(c30);
}

#[inline]
pub(crate) fn calibrate_temperature(coeffs: &CalibrationCoeffs, temp_scaled: f32) -> f32 {
    coeffs.c0_half + (coeffs.c1 * temp_scaled)
}

#[inline]
pub(crate) fn calibrate_pressure(
    coeffs: &CalibrationCoeffs,
    pres_scaled: f32,
    temp_scaled: f32,
) -> f32 {
    coeffs.c00
        + (pres_scaled * (coeffs.c10 + pres_scaled * (coeffs.c20 + pres_scaled * coeffs.c30)))
        + (temp_scaled * coeffs.c01)
        + (temp_scaled * pres_scaled * (coeffs.c11 + pres_scaled * coeffs.c21))
}

#[cfg(test)]
mod tests {
    use super::{
        calibrate_pressure, calibrate_temperature, decode_i12_high, decode_i12_low, decode_i24,
        i24_to_f32, process_calibration_coefficients, CalibrationCoeffs,
    };
    use crate::{config::Oversampling, device_internal::inv_scale_factor};

    #[test]
    fn test_decode_i12_negative_and_positive() {
        assert_eq!(decode_i12_high(0xFF, 0xF0), -1);
        assert_eq!(decode_i12_low(0x0F, 0xFF), -1);
        assert_eq!(decode_i12_high(0x80, 0x00), -2048);
        assert_eq!(decode_i12_low(0x08, 0x00), -2048);
        assert_eq!(decode_i12_high(0x7F, 0xF0), 2047);
        assert_eq!(decode_i12_low(0x07, 0xFF), 2047);
    }

    #[test]
    fn test_decode_i24_and_f32_conversion() {
        assert_eq!(decode_i24([0xFF, 0xFF, 0xFF]), -1);
        assert_eq!(i24_to_f32(-1), -1.0);
        assert_eq!(decode_i24([0x80, 0x00, 0x00]), -8_388_608);
        assert_eq!(i24_to_f32(-8_388_608), -8_388_608.0);
        assert_eq!(decode_i24([0x7F, 0xFF, 0xFF]), 8_388_607);
        assert_eq!(i24_to_f32(8_388_607), 8_388_607.0);
    }

    #[test]
    fn fixed_reference_vectors_cover_signed_coefficients_and_all_scales() {
        // Independently packed signed fields, cross-checked against dps310-rs decoding.
        // The second set negates every coefficient, exercising the opposite sign in each field.
        let coefficient_sets = [
            (
                [
                    0x0C, 0x8E, 0xD6, 0x13, 0xE6, 0x3E, 0xFE, 0x25, 0xF5, 0x3B, 0x04, 0xA1, 0xD8,
                    0x35, 0x00, 0x75, 0xFA, 0x6C,
                ],
                1.0,
            ),
            (
                [
                    0xF3, 0x81, 0x2A, 0xEC, 0x19, 0xD1, 0x01, 0xDB, 0x0A, 0xC5, 0xFB, 0x5F, 0x27,
                    0xCB, 0xFF, 0x8B, 0x05, 0x94,
                ],
                -1.0,
            ),
        ];
        // Fixed results calculated offline using exact rational arithmetic and the expanded
        // compensation polynomial (equivalent to dps310-rs), not the production Horner form.
        // Raw pressure = -1_234_567; raw temperature = 234_567 at every scale.
        let vectors = [
            (
                Oversampling::X1,
                196_914.384_083_302_f64,
                -33.325_511_932_f64,
            ),
            (Oversampling::X2, 127_195.408_351_067, 55.558_162_689),
            (Oversampling::X4, 102_413.378_490_249, 80.953_498_295),
            (Oversampling::X8, 91_536.409_028_459, 91.111_632_538),
            (Oversampling::X16, 320_412.702_319_105, -175.252_669_796),
            (Oversampling::X32, 198_430.799_399_040, -35.441_789_900),
            (Oversampling::X64, 146_978.682_792_192, 32.812_340_444),
            (Oversampling::X128, 116_872.297_422_927, 66.537_910_731),
        ];
        for (bytes, sign) in coefficient_sets {
            let mut coeffs = CalibrationCoeffs::default();
            process_calibration_coefficients(&mut coeffs, &bytes);
            assert_eq!(
                coeffs,
                CalibrationCoeffs {
                    c0_half: 100.0 * sign,
                    c1: -298.0 * sign,
                    c00: 81_507.0 * sign,
                    c10: -66_011.0 * sign,
                    c01: -2_757.0 * sign,
                    c11: 1_185.0 * sign,
                    c20: -10_187.0 * sign,
                    c21: 117.0 * sign,
                    c30: -1_428.0 * sign,
                }
            );
            for (oversampling, pressure, temperature) in vectors {
                let inv = inv_scale_factor(oversampling);
                let actual_pressure =
                    calibrate_pressure(&coeffs, -1_234_567.0 * inv, 234_567.0 * inv);
                let actual_temperature = calibrate_temperature(&coeffs, 234_567.0 * inv);
                // Allow f32 rounding while remaining far below scale/decoding errors.
                assert!(
                    (f64::from(actual_pressure) - pressure * f64::from(sign)).abs() < 0.1,
                    "pressure mismatch for {oversampling:?}, sign {sign}: {actual_pressure}"
                );
                assert!(
                    (f64::from(actual_temperature) - temperature * f64::from(sign)).abs() < 0.0001,
                    "temperature mismatch for {oversampling:?}, sign {sign}: {actual_temperature}"
                );
            }
        }
    }

    #[test]
    fn test_infineon_datasheet_example_compensation() {
        // Values matching standard DPS310 calibration coefficient unpacking
        // c0 = 200 (0x0C8), c1 = -298 (0xED6)
        // c00 = 81507 (0x13E63), c10 = -66011 (0xEFE25)
        // c01 = -2757 (0xF53B), c11 = 1185 (0x04A1)
        // c20 = -10187 (0xD835), c21 = 117 (0x0075), c30 = -1428 (0xFA6C)
        let mut bytes = [
            0x0C, 0x8E, 0xD6, // c0 = 200, c1 = -298
            0x13, 0xE6, 0x3E, 0xFE, 0x25, // c00 = 81507, c10 = -66011
            0xF5, 0x3B, // c01 = -2757
            0x04, 0xA1, // c11 = 1185
            0xD8, 0x35, // c20 = -10187
            0x00, 0x75, // c21 = 117
            0xFA, 0x6C, // c30 = -1428
        ];
        let mut coeffs = CalibrationCoeffs::default();
        process_calibration_coefficients(&mut coeffs, &mut bytes);

        assert_eq!(coeffs.c0_half, 100.0);
        assert_eq!(coeffs.c1, -298.0);
        assert_eq!(coeffs.c00, 81_507.0);
        assert_eq!(coeffs.c10, -66_011.0);
        assert_eq!(coeffs.c01, -2_757.0);
        assert_eq!(coeffs.c11, 1_185.0);
        assert_eq!(coeffs.c20, -10_187.0);
        assert_eq!(coeffs.c21, 117.0);
        assert_eq!(coeffs.c30, -1_428.0);

        let temp_c = calibrate_temperature(&coeffs, 0.25);
        assert!((temp_c - 25.5).abs() < 1e-4, "expected 25.5C, got {temp_c}");

        let pres_pa = calibrate_pressure(&coeffs, -0.3, 0.25);
        assert!(
            (pres_pa - 99_656.53).abs() < 0.05,
            "expected ~99656.53 Pa, got {pres_pa}"
        );
    }
}
