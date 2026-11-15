use embedded_hal::delay::DelayNs;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use uf_dps3xx::{
    advanced::Register, Config, ConfigError, Dps3xx, Error, FifoConfig, FifoEntry, I2cAddress,
    InitStage, Mode, Oversampling, Poll, Rate, Sample, TemperatureSource,
};

const ADDR: u8 = 0x76;

// Synthetic nonzero coefficients: c0=20, c1=40, c00=100000, c10=1000,
// c01=200, c11=80, c20=40, c21=16, c30=8.
const NONZERO_COEFFICIENTS: [u8; 18] = [
    0x01, 0x40, 0x28, 0x18, 0x6A, 0x00, 0x03, 0xE8, 0x00, 0xC8, 0x00, 0x50, 0x00, 0x28, 0x00, 0x10,
    0x00, 0x08,
];

struct NoopDelay;

impl DelayNs for NoopDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}

fn init_expectations() -> Vec<I2cTransaction> {
    init_expectations_with_coefficients(&[0; 18])
}

fn init_expectations_with_coefficients(coefficients: &[u8; 18]) -> Vec<I2cTransaction> {
    device_init_expectations(coefficients, 0x10, 0, [0; 3], [0; 3])
}

fn device_init_expectations(
    coefficients: &[u8; 18],
    id: u8,
    source: u8,
    stale: [u8; 3],
    prime: [u8; 3],
) -> Vec<I2cTransaction> {
    vec![
        I2cTransaction::write_read(ADDR, vec![Register::PROD_ID.addr()], vec![id]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0xC0]),
        I2cTransaction::write_read(
            ADDR,
            vec![Register::COEFF_REG_1.addr()],
            coefficients.to_vec(),
        ),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_COEF_SRCE.addr()], vec![source]),
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::RESET.addr(), 0x80]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], stale.to_vec()),
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), source]),
        I2cTransaction::write(ADDR, vec![0x0E, 0xA5]),
        I2cTransaction::write(ADDR, vec![0x0F, 0x96]),
        I2cTransaction::write(ADDR, vec![0x62, 0x02]),
        I2cTransaction::write(ADDR, vec![0x0E, 0x00]),
        I2cTransaction::write(ADDR, vec![0x0F, 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x02]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x20]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], prime.to_vec()),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), source]),
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x07]),
    ]
}

fn init_sensor(i2c: I2cMock) -> Dps3xx<I2cMock> {
    let mut sensor = Dps3xx::new(i2c, I2cAddress::Primary, Config::default()).unwrap();
    while let Poll::Pending { .. } = sensor.poll_init().unwrap() {}
    sensor
}

#[test]
fn product_id_accepts_revisions_and_rejects_other_products() {
    for id in [0x00, 0x10, 0x20, 0xF0] {
        let mut i2c = I2cMock::new(&device_init_expectations(&[0; 18], id, 0, [0; 3], [0; 3]));
        let sensor = init_sensor(i2c.clone());
        assert!(sensor.is_ready(), "revision in {id:#04x} must be accepted");
        i2c.done();
    }
    for id in [0x01, 0x11, 0x1F, 0xFF] {
        let mut i2c = I2cMock::new(&[I2cTransaction::write_read(
            ADDR,
            vec![Register::PROD_ID.addr()],
            vec![id],
        )]);
        let mut sensor = Dps3xx::new(i2c.clone(), I2cAddress::Primary, Config::new()).unwrap();
        assert_eq!(sensor.poll_init(), Err(Error::InvalidDevice { id }));
        i2c.done();
    }
}

#[test]
fn warm_initialization_discards_stale_temperature_and_primes_with_factory_source() {
    let coefficients = [
        0x0C, 0x8E, 0xD6, 0x13, 0xE6, 0x3E, 0xFE, 0x25, 0xF5, 0x3B, 0x04, 0xA1, 0xD8, 0x35, 0x00,
        0x75, 0xFA, 0x6C,
    ];
    // The sequence requires CFG_REG=0 (FIFO/shift off), a FIFO flush and a stale read
    // before triggering the new X1 temperature conversion. Exercise both factory sources.
    for (source, selection) in [
        (0, TemperatureSource::Internal),
        (0x80, TemperatureSource::External),
    ] {
        let mut expectations =
            device_init_expectations(&coefficients, 0x20, source, [0x7F, 0xFF, 0xFF], [2, 0, 0]);
        expectations.extend([
            I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x10]),
            I2cTransaction::write_read(ADDR, vec![Register::PSR_B2.addr()], vec![0; 3]),
        ]);
        let mut i2c = I2cMock::new(&expectations);
        let mut sensor = Dps3xx::new(
            i2c.clone(),
            I2cAddress::Primary,
            Config::new().temperature_source(selection),
        )
        .unwrap();
        while let Poll::Pending { .. } = sensor.poll_init().unwrap() {}
        assert_eq!(
            sensor.read_sample().unwrap(),
            Sample {
                pressure_pa: 80_817.75,
                temperature_c: 25.5
            }
        );
        i2c.done();
    }
}

#[test]
fn mismatched_temperature_sources_are_rejected_before_configuration() {
    for (source, selection) in [
        (0, TemperatureSource::External),
        (0x80, TemperatureSource::Internal),
    ] {
        let expectations = [
            I2cTransaction::write_read(ADDR, vec![Register::PROD_ID.addr()], vec![0x10]),
            I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0]),
            I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0xC0]),
            I2cTransaction::write_read(ADDR, vec![Register::COEFF_REG_1.addr()], vec![0; 18]),
            I2cTransaction::write_read(ADDR, vec![Register::TMP_COEF_SRCE.addr()], vec![source]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut sensor = Dps3xx::new(
            i2c.clone(),
            I2cAddress::Primary,
            Config::new().temperature_source(selection),
        )
        .unwrap();
        assert_eq!(
            sensor.poll_init(),
            Err(Error::InvalidConfig {
                reason: ConfigError::TemperatureSourceMismatch
            })
        );
        assert!(!sensor.is_ready());
        i2c.done();
    }
    let mut i2c = I2cMock::new(&init_expectations());
    let mut sensor = init_sensor(i2c.clone());
    let original = sensor.config();
    assert_eq!(
        sensor.reconfigure(Config::new().temperature_source(TemperatureSource::External)),
        Err(Error::InvalidConfig {
            reason: ConfigError::TemperatureSourceMismatch
        })
    );
    assert!(sensor.is_ready());
    assert_eq!(sensor.config(), original);
    i2c.done();
}

#[test]
fn mode_changes_revalidate_the_conversion_budget() {
    let mut expectations = init_expectations();
    expectations.extend([
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0]),
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 0x27]),
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), 0x27]),
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0x0C]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 5]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 6]),
    ]);
    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = init_sensor(i2c.clone());
    let config = Config::new()
        .pressure(Rate::Hz4, Oversampling::X128)
        .temperature(Rate::Hz4, Oversampling::X128)
        .mode(Mode::BackgroundPressure);
    sensor.reconfigure(config).unwrap();
    assert_eq!(
        sensor.start_background(),
        Err(Error::InvalidConfig {
            reason: ConfigError::MeasurementBusyTimeExceeded
        })
    );
    assert_eq!(sensor.config(), config);
    assert!(sensor.is_ready());
    sensor.set_mode(Mode::BackgroundTemperature).unwrap();
    i2c.done();
}

#[test]
fn init_poll_completes() {
    let mut i2c = I2cMock::new(&init_expectations());
    let sensor = init_sensor(i2c.clone());
    assert!(sensor.is_ready());
    assert_eq!(sensor.init_stage(), InitStage::Ready);
    i2c.done();
}

#[test]
fn init_poll_cold_boot_stages() {
    let expectations = vec![
        I2cTransaction::write_read(ADDR, vec![Register::PROD_ID.addr()], vec![0x10]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        // 1st status check: sensor not ready yet
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x00]),
        // 2nd status check: sensor ready, coefficients not ready yet
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x40]),
        // 3rd status check: coefficients still not ready
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x40]),
        // 4th status check: coefficients ready
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0xC0]),
        I2cTransaction::write_read(ADDR, vec![Register::COEFF_REG_1.addr()], vec![0; 18]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_COEF_SRCE.addr()], vec![0x80]),
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::RESET.addr(), 0x80]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0, 0, 0]),
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), 0x80]),
        I2cTransaction::write(ADDR, vec![0x0E, 0xA5]),
        I2cTransaction::write(ADDR, vec![0x0F, 0x96]),
        I2cTransaction::write(ADDR, vec![0x62, 0x02]),
        I2cTransaction::write(ADDR, vec![0x0E, 0x00]),
        I2cTransaction::write(ADDR, vec![0x0F, 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x02]),
        // 5th status check: temperature not ready yet
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0xC0]),
        // 6th status check: temperature ready
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0xE0]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0, 0, 0]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), 0x80]),
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x07]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = Dps3xx::new(i2c.clone(), I2cAddress::Primary, Config::default()).unwrap();

    assert_eq!(sensor.poll_init().unwrap(), Poll::Pending { wait_ms: 12 });
    assert_eq!(sensor.init_stage(), InitStage::SensorReady);

    assert_eq!(sensor.poll_init().unwrap(), Poll::Pending { wait_ms: 40 });
    assert_eq!(sensor.init_stage(), InitStage::CoefficientsReady);

    assert_eq!(sensor.poll_init().unwrap(), Poll::Pending { wait_ms: 10 });
    assert_eq!(sensor.init_stage(), InitStage::CoefficientsReady);

    assert_eq!(sensor.poll_init().unwrap(), Poll::Pending { wait_ms: 13 });
    assert_eq!(sensor.init_stage(), InitStage::TemperatureReady);

    assert_eq!(sensor.poll_init().unwrap(), Poll::Pending { wait_ms: 13 });
    assert_eq!(sensor.init_stage(), InitStage::TemperatureReady);

    assert_eq!(sensor.poll_init().unwrap(), Poll::Ready(()));
    assert!(sensor.is_ready());
    i2c.done();
}

#[test]
fn init_blocking_completes() {
    let mut i2c = I2cMock::new(&init_expectations());
    let mut delay = NoopDelay;
    let sensor = Dps3xx::new_blocking(
        i2c.clone(),
        I2cAddress::Primary,
        Config::default(),
        &mut delay,
    )
    .unwrap();
    assert!(sensor.is_ready());
    i2c.done();
}

#[test]
fn init_blocking_timeout_preserves_bus() {
    let expectations = vec![
        I2cTransaction::write_read(ADDR, vec![Register::PROD_ID.addr()], vec![0x10]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x00]),
    ];
    let i2c = I2cMock::new(&expectations);
    let mut delay = NoopDelay;
    let config = Config::default().init_timeout_ms(5);

    let mut sensor = Dps3xx::new(i2c, I2cAddress::Primary, config).unwrap();
    let err = sensor.init_blocking(&mut delay).unwrap_err();
    assert_eq!(
        err,
        Error::Timeout {
            stage: InitStage::SensorReady
        }
    );

    let mut recovered_i2c = sensor.release();
    recovered_i2c.done();
}

#[test]
fn invalid_device_id_returns_error() {
    let expectations = vec![I2cTransaction::write_read(
        ADDR,
        vec![Register::PROD_ID.addr()],
        vec![0xFF],
    )];
    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = Dps3xx::new(i2c.clone(), I2cAddress::Primary, Config::default()).unwrap();

    assert_eq!(
        sensor.poll_init().unwrap_err(),
        Error::InvalidDevice { id: 0xFF }
    );
    i2c.done();
}

#[test]
fn uninitialized_methods_return_not_initialized() {
    let mut i2c = I2cMock::new(&[]);
    let mut sensor = Dps3xx::new(i2c.clone(), I2cAddress::Primary, Config::default()).unwrap();

    assert_eq!(sensor.try_read_sample().unwrap_err(), Error::NotInitialized);
    assert_eq!(
        sensor.try_read_temperature().unwrap_err(),
        Error::NotInitialized
    );
    assert_eq!(
        sensor.read_pressure_raw().unwrap_err(),
        Error::NotInitialized
    );
    assert_eq!(sensor.standby().unwrap_err(), Error::NotInitialized);
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
fn try_read_sample_reads_burst_when_both_ready() {
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
fn try_read_sample_reads_3_bytes_when_only_pressure_ready() {
    let mut expectations = init_expectations();
    expectations.extend([
        // Only PRS_RDY (0x10) is set, TMP_RDY is not set
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x10]),
        // Should read only 3 bytes from PSR_B2..B0 and use cached temperature
        I2cTransaction::write_read(ADDR, vec![Register::PSR_B2.addr()], vec![0, 0, 0]),
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
fn try_read_sample_updates_cached_temp_when_only_temp_ready() {
    let mut expectations = init_expectations();
    expectations.extend([
        // Only TMP_RDY (0x20) is set
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x20]),
        // Should read 3 bytes from TMP_B2..B0 and return Ok(None)
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0, 0, 0]),
    ]);

    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = init_sensor(i2c.clone());

    let sample = sensor.try_read_sample().unwrap();
    assert!(sample.is_none());
    i2c.done();
}

#[test]
fn standalone_temperature_uses_configured_scale_and_updates_pressure_cache() {
    let coefficients = [
        0x0C, 0x8E, 0xD6, 0x13, 0xE6, 0x3E, 0xFE, 0x25, 0xF5, 0x3B, 0x04, 0xA1, 0xD8, 0x35, 0x00,
        0x75, 0xFA, 0x6C,
    ];
    let mut expectations = init_expectations_with_coefficients(&coefficients);
    expectations.extend([
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0]),
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 0]),
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), 1]),
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 6]),
        // No temperature ready: do not read data, even if pressure is ready.
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x10]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x20]),
        // X2 scale 1_572_864: raw 393_216 gives scaled temperature 0.25.
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![6, 0, 0]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 7]),
        // Both ready: temperature read must leave the pressure measurement untouched.
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x30]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![6, 0, 0]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x10]),
        I2cTransaction::write_read(ADDR, vec![Register::PSR_B2.addr()], vec![0, 0, 0]),
    ]);
    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = init_sensor(i2c.clone());
    sensor
        .reconfigure(
            Config::new()
                .temperature(Rate::Hz1, Oversampling::X2)
                .mode(Mode::BackgroundTemperature),
        )
        .unwrap();
    assert_eq!(sensor.try_read_temperature().unwrap(), None);
    assert_eq!(sensor.try_read_temperature().unwrap(), Some(25.5));
    sensor.start_background().unwrap();
    assert_eq!(sensor.try_read_temperature().unwrap(), Some(25.5));
    assert_eq!(
        sensor.read_sample().unwrap(),
        Sample {
            pressure_pa: 80_817.75,
            temperature_c: 25.5,
        }
    );
    i2c.done();
}

#[test]
fn raw_pressure_reads_only_pressure_and_sign_extends() {
    let mut expectations = init_expectations();
    expectations.extend([
        I2cTransaction::write_read(ADDR, vec![Register::PSR_B2.addr()], vec![0x80, 0, 0]),
        I2cTransaction::write_read(ADDR, vec![Register::PSR_B2.addr()], vec![0x7F, 0xFF, 0xFF]),
        I2cTransaction::write_read(ADDR, vec![Register::PSR_B2.addr()], vec![0xFF, 0xFF, 0xFF]),
        // A pending temperature is still available after reading raw pressure.
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x20]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0, 0, 0]),
    ]);
    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = init_sensor(i2c.clone());
    assert_eq!(sensor.read_pressure_raw().unwrap(), -8_388_608);
    assert_eq!(sensor.read_pressure_raw().unwrap(), 8_388_607);
    assert_eq!(sensor.read_pressure_raw().unwrap(), -1);
    assert_eq!(sensor.try_read_temperature().unwrap(), Some(0.0));
    i2c.done();
}

#[test]
fn standalone_reads_propagate_bus_errors_without_replacing_cached_temperature() {
    use embedded_hal::i2c::ErrorKind;
    let mut expectations = init_expectations();
    expectations.extend([
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0])
            .with_error(ErrorKind::Other),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x20]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0, 0, 0])
            .with_error(ErrorKind::Other),
        I2cTransaction::write_read(ADDR, vec![Register::PSR_B2.addr()], vec![0, 0, 0])
            .with_error(ErrorKind::Other),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x10]),
        I2cTransaction::write_read(ADDR, vec![Register::PSR_B2.addr()], vec![0, 0, 0]),
    ]);
    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = init_sensor(i2c.clone());
    assert_eq!(
        sensor.try_read_temperature(),
        Err(Error::Bus(ErrorKind::Other))
    );
    assert_eq!(
        sensor.try_read_temperature(),
        Err(Error::Bus(ErrorKind::Other))
    );
    assert_eq!(
        sensor.read_pressure_raw(),
        Err(Error::Bus(ErrorKind::Other))
    );
    assert_eq!(
        sensor.read_sample().unwrap(),
        Sample {
            pressure_pa: 0.0,
            temperature_c: 0.0
        }
    );
    i2c.done();
}

#[test]
fn fifo_pressure_uses_latest_preceding_temperature() {
    let mut expectations = init_expectations_with_coefficients(&NONZERO_COEFFICIENTS);
    expectations.extend([
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0]),
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 4]), // X16
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), 1]), // X2
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 6]), // FIFO + pressure shift
        I2cTransaction::write(ADDR, vec![Register::RESET.addr(), 0x80]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 7]),
    ]);
    // Pressure before any FIFO temperature uses the initialization reference (scaled 0).
    // Pressure retains the tag bit, matching Infineon's getFIFOvalue behavior.
    let entries = [
        [0x03, 0xE0, 0x01], // pressure 253953, X16 scale 253952
        [0x06, 0x00, 0x00], // temperature 393216, X2 scale 1572864 -> 0.25
        [0x03, 0xE0, 0x01], // same pressure, new temperature reference
        [0x03, 0xE0, 0x01], // consecutive pressure retains that reference
        [0xF4, 0x00, 0x00], // temperature -786432 -> -0.5
        [0xFC, 0x20, 0x01], // negative pressure -253951
    ];
    for bytes in entries {
        expectations.extend([
            I2cTransaction::write_read(ADDR, vec![Register::FIFO_STS.addr()], vec![0]),
            I2cTransaction::write_read(ADDR, vec![Register::PSR_B2.addr()], bytes.to_vec()),
        ]);
    }
    expectations.push(I2cTransaction::write_read(
        ADDR,
        vec![Register::FIFO_STS.addr()],
        vec![1],
    ));
    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = init_sensor(i2c.clone());
    sensor
        .reconfigure(
            Config::new()
                .pressure(Rate::Hz1, Oversampling::X16)
                .temperature(Rate::Hz1, Oversampling::X2)
                .fifo(FifoConfig {
                    enable: true,
                    interrupt_on_full: false,
                }),
        )
        .unwrap();

    // Fixed expectations calculated independently from the expanded polynomial.
    let expected = [
        FifoEntry::Pressure(Sample {
            pressure_pa: 101_048.004_347,
            temperature_c: 10.0,
        }),
        FifoEntry::Temperature(20.0),
        FifoEntry::Pressure(Sample {
            pressure_pa: 101_122.004_458,
            temperature_c: 20.0,
        }),
        FifoEntry::Pressure(Sample {
            pressure_pa: 101_122.004_458,
            temperature_c: 20.0,
        }),
        FifoEntry::Temperature(-10.0),
        FifoEntry::Pressure(Sample {
            pressure_pa: 98_964.003_623,
            temperature_c: -10.0,
        }),
    ];
    for expected_entry in expected {
        let entry = sensor.try_pop_fifo().unwrap().unwrap();
        match (entry, expected_entry) {
            (FifoEntry::Pressure(actual), FifoEntry::Pressure(expected)) => {
                assert!(
                    (actual.pressure_pa - expected.pressure_pa).abs() < 0.02,
                    "expected {expected:?}, got {actual:?}"
                );
                assert_eq!(actual.temperature_c, expected.temperature_c);
            }
            (actual, expected) => assert_eq!(actual, expected),
        }
    }
    assert_eq!(sensor.try_pop_fifo().unwrap(), None);
    i2c.done();
}

#[test]
fn repeated_x128_temperature_and_pressure_commands() {
    let mut expectations = init_expectations_with_coefficients(&NONZERO_COEFFICIENTS);
    expectations.extend([
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0]),
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 7]),
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), 7]),
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0x0C]),
    ]);
    // Two full cycles, with positive and negative ADC values. Command conversions
    // return to idle in hardware; the next command needs no intervening standby write.
    let cycles = [
        ([0x07, 0xF8, 0x00], [0x1F, 0xE0, 0x00], 20.0, 101_122.0),
        ([0xF0, 0x10, 0x00], [0xE0, 0x20, 0x00], -10.0, 98_964.0),
    ];
    for (temperature, pressure, _, _) in cycles {
        expectations.extend([
            I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 2]),
            I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0]),
            I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x20]),
            I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], temperature.to_vec()),
            I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0]),
            I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 1]),
            I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0]),
            I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x10]),
            I2cTransaction::write_read(ADDR, vec![Register::PSR_B2.addr()], pressure.to_vec()),
            I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0]),
        ]);
    }
    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = init_sensor(i2c.clone());
    sensor
        .reconfigure(
            Config::new()
                .pressure(Rate::Hz1, Oversampling::X128)
                .temperature(Rate::Hz1, Oversampling::X128)
                .mode(Mode::Idle),
        )
        .unwrap();
    for (_, _, temperature_c, pressure_pa) in cycles {
        sensor.set_mode(Mode::CommandTemperature).unwrap();
        assert_eq!(sensor.try_read_temperature().unwrap(), None);
        assert_eq!(sensor.try_read_temperature().unwrap(), Some(temperature_c));
        assert_eq!(sensor.try_read_temperature().unwrap(), None);
        sensor.set_mode(Mode::CommandPressure).unwrap();
        assert_eq!(sensor.try_read_sample().unwrap(), None);
        let sample = sensor.read_sample().unwrap();
        assert!(
            (sample.pressure_pa - pressure_pa).abs() < 0.02,
            "expected pressure {pressure_pa}, got {sample:?}"
        );
        assert_eq!(sample.temperature_c, temperature_c);
        assert_eq!(sensor.read_sample(), Err(Error::NoSampleReady));
    }
    i2c.done();
}

#[test]
fn fifo_pop_decodes_temperature_and_pressure_entries() {
    let mut expectations = init_expectations();
    expectations.extend([
        // 1st pop: FIFO not empty (0x00), entry has LSB=0 -> Temperature
        I2cTransaction::write_read(ADDR, vec![Register::FIFO_STS.addr()], vec![0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::PSR_B2.addr()], vec![0x00, 0x00, 0x00]),
        // 2nd pop: FIFO not empty (0x00), entry has LSB=1 -> Pressure
        I2cTransaction::write_read(ADDR, vec![Register::FIFO_STS.addr()], vec![0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::PSR_B2.addr()], vec![0x00, 0x00, 0x01]),
        // 3rd pop: FIFO empty (0x01) -> None
        I2cTransaction::write_read(ADDR, vec![Register::FIFO_STS.addr()], vec![0x01]),
    ]);

    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = init_sensor(i2c.clone());

    let first = sensor.try_pop_fifo().unwrap();
    assert_eq!(first, Some(FifoEntry::Temperature(0.0)));

    let second = sensor.try_pop_fifo().unwrap();
    assert_eq!(
        second,
        Some(FifoEntry::Pressure(Sample {
            pressure_pa: 0.0,
            temperature_c: 0.0,
        }))
    );

    let third = sensor.try_pop_fifo().unwrap();
    assert!(third.is_none());
    i2c.done();
}

#[test]
fn reconfigure_and_interrupt_status_and_reset() {
    let mut expectations = init_expectations();
    expectations.extend([
        // interrupt_status read
        I2cTransaction::write_read(ADDR, vec![Register::INT_STS.addr()], vec![0b0000_0101]),
        // reconfigure with FIFO enabled and Idle mode
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 0x43]), // Hz16 (4<<4) | X8 (3)
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), 0x10]), // Hz2 (1<<4) | X1 (0)
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0x02]), // FIFO enable
        I2cTransaction::write(ADDR, vec![Register::RESET.addr(), 0x80]),   // FIFO flush
        // reset
        I2cTransaction::write(ADDR, vec![Register::RESET.addr(), 0x89]),
    ]);

    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = init_sensor(i2c.clone());

    let irq = sensor.interrupt_status().unwrap();
    assert!(irq.fifo_full);
    assert!(!irq.temperature_ready);
    assert!(irq.pressure_ready);

    let new_cfg = Config::indoor()
        .pressure(Rate::Hz16, Oversampling::X8)
        .temperature(Rate::Hz2, Oversampling::X1)
        .fifo(FifoConfig {
            enable: true,
            interrupt_on_full: false,
        })
        .mode(Mode::Idle);
    sensor.reconfigure(new_cfg).unwrap();
    assert_eq!(sensor.config(), new_cfg);

    sensor.reset().unwrap();
    assert!(!sensor.is_ready());
    i2c.done();
}

#[test]
fn reconfigure_standby_error_preserves_ready_and_config() {
    use embedded_hal::i2c::ErrorKind;

    let mut expectations = init_expectations();
    expectations.push(
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00])
            .with_error(ErrorKind::Other),
    );

    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = init_sensor(i2c.clone());
    let original_cfg = sensor.config();

    let err = sensor.reconfigure(Config::navigation()).unwrap_err();
    assert_eq!(err, Error::Bus(ErrorKind::Other));
    assert!(sensor.is_ready());
    assert_eq!(sensor.config(), original_cfg);
    i2c.done();
}

#[test]
fn reconfigure_partial_failure_invalidates_ready_and_recovers() {
    use embedded_hal::i2c::ErrorKind;

    let mut expectations = init_expectations();
    expectations.extend([
        // Standby succeeds, then PRS_CFG write fails
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 0x54])
            .with_error(ErrorKind::Other),
        // Retry reconfigure succeeds
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 0x54]),
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), 0x20]),
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0x04]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x07]),
    ]);

    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = init_sensor(i2c.clone());
    let original_cfg = sensor.config();

    let nav_cfg = Config::navigation();
    let err = sensor.reconfigure(nav_cfg).unwrap_err();
    assert_eq!(err, Error::Bus(ErrorKind::Other));

    // Readiness is invalidated and old config is not overwritten yet
    assert!(!sensor.is_ready());
    assert_eq!(sensor.init_stage(), InitStage::ApplyConfig);
    assert_eq!(sensor.config(), original_cfg);
    assert_eq!(sensor.try_read_sample().unwrap_err(), Error::NotInitialized);

    // Retrying reconfigure succeeds and commits the new config
    sensor.reconfigure(nav_cfg).unwrap();
    assert!(sensor.is_ready());
    assert_eq!(sensor.config(), nav_cfg);
    i2c.done();
}

#[test]
fn init_late_bus_error_resumes_at_apply_config_without_re_waiting_temp() {
    use embedded_hal::i2c::ErrorKind;

    let expectations = vec![
        I2cTransaction::write_read(ADDR, vec![Register::PROD_ID.addr()], vec![0x10]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0xC0]),
        I2cTransaction::write_read(ADDR, vec![Register::COEFF_REG_1.addr()], vec![0; 18]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_COEF_SRCE.addr()], vec![0x00]),
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::RESET.addr(), 0x80]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0, 0, 0]),
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![0x0E, 0xA5]),
        I2cTransaction::write(ADDR, vec![0x0F, 0x96]),
        I2cTransaction::write(ADDR, vec![0x62, 0x02]),
        I2cTransaction::write(ADDR, vec![0x0E, 0x00]),
        I2cTransaction::write(ADDR, vec![0x0F, 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x02]),
        // 2nd poll: TMP_RDY is set and TMP_B2 is read (clearing TMP_RDY), then PRS_CFG write fails
        I2cTransaction::write_read(ADDR, vec![Register::MEAS_CFG.addr()], vec![0x20]),
        I2cTransaction::write_read(ADDR, vec![Register::TMP_B2.addr()], vec![0, 0, 0]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 0x00])
            .with_error(ErrorKind::Other),
        // 3rd poll: resumes directly at ApplyConfig without polling TMP_RDY again
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::PRS_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::TEMP_CFG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::CFG_REG.addr(), 0x00]),
        I2cTransaction::write(ADDR, vec![Register::MEAS_CFG.addr(), 0x07]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = Dps3xx::new(i2c.clone(), I2cAddress::Primary, Config::default()).unwrap();

    assert_eq!(sensor.poll_init().unwrap(), Poll::Pending { wait_ms: 13 });
    assert_eq!(
        sensor.poll_init().unwrap_err(),
        Error::Bus(ErrorKind::Other)
    );
    assert_eq!(sensor.init_stage(), InitStage::ApplyConfig);

    assert_eq!(sensor.poll_init().unwrap(), Poll::Ready(()));
    assert!(sensor.is_ready());
    i2c.done();
}
