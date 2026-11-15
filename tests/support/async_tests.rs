use super::{asynchronous::init_expectations, support::block_on};
use embedded_hal::i2c::{ErrorKind, ErrorType, I2c as BlockingI2c, Operation};
use embedded_hal_async::{delay::DelayNs, i2c::I2c};
use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
use std::cell::{Cell, RefCell};
use std::future::{poll_fn, Future};
use std::rc::Rc;
use std::sync::Arc;
use std::task::{Context, Poll, Wake, Waker};
use uf_dps3xx::{
    AsyncDps3xx, Config, Error, FifoConfig, FifoEntry, I2cAddress, InitStage, Mode,
    Poll as InitPoll,
};

const ADDR: u8 = 0x76;

#[derive(Default)]
struct Control {
    transfers: Cell<usize>,
    pause_at: Cell<Option<usize>>,
    pause_before: Cell<bool>,
    paused: Cell<bool>,
}

impl Control {
    async fn pause(&self, index: usize, before: bool) {
        poll_fn(|_| {
            if self.pause_at.get() == Some(index) && self.pause_before.get() == before {
                self.paused.set(true);
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
    }

    fn resume(&self) {
        self.pause_at.set(None);
        self.paused.set(false);
    }
}

// Every transfer yields. Selected transfers can additionally stall before touching
// hardware or after hardware completion but before the caller sees success.
struct YieldingI2c {
    mock: Mock,
    control: Rc<Control>,
}

async fn yield_once() {
    let mut yielded = false;
    poll_fn(|cx| {
        if yielded {
            Poll::Ready(())
        } else {
            yielded = true;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    })
    .await;
}

impl ErrorType for YieldingI2c {
    type Error = ErrorKind;
}

impl YieldingI2c {
    async fn begin(&self) -> usize {
        let index = self.control.transfers.get();
        self.control.transfers.set(index + 1);
        self.control.pause(index, true).await;
        index
    }

    async fn finish<T>(&self, index: usize, result: T) -> T {
        self.control.pause(index, false).await;
        yield_once().await;
        result
    }
}

impl I2c for YieldingI2c {
    async fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        let index = self.begin().await;
        let result = BlockingI2c::write(&mut self.mock, address, bytes);
        self.finish(index, result).await
    }

    async fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        let index = self.begin().await;
        let result = BlockingI2c::write_read(&mut self.mock, address, bytes, read);
        self.finish(index, result).await
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let index = self.begin().await;
        let result = BlockingI2c::transaction(&mut self.mock, address, operations);
        self.finish(index, result).await
    }
}

#[derive(Default)]
struct YieldingDelay(Rc<RefCell<Vec<u32>>>);
impl DelayNs for YieldingDelay {
    async fn delay_ns(&mut self, ns: u32) {
        self.0.borrow_mut().push(ns);
        yield_once().await;
    }
}

fn bus(expectations: &[Transaction]) -> (YieldingI2c, Rc<Control>, Mock) {
    let mock = Mock::new(expectations);
    let control = Rc::new(Control::default());
    (
        YieldingI2c {
            mock: mock.clone(),
            control: control.clone(),
        },
        control,
        mock,
    )
}

struct NoopWake;
impl Wake for NoopWake {
    fn wake(self: Arc<Self>) {}
}

fn cancel_at_pause(future: impl Future, control: &Control) {
    let waker = Waker::from(Arc::new(NoopWake));
    let mut context = Context::from_waker(&waker);
    let mut future = std::pin::pin!(future);
    for _ in 0..100 {
        assert!(future.as_mut().poll(&mut context).is_pending());
        if control.paused.get() {
            return; // Drops the future while the transaction's outcome is unknown.
        }
    }
    panic!("future did not reach the selected suspension point");
}

fn restore_configuration() -> Vec<Transaction> {
    vec![
        Transaction::write(ADDR, vec![0x08, 0]),
        Transaction::write(ADDR, vec![0x06, 0]),
        Transaction::write(ADDR, vec![0x07, 0]),
        Transaction::write(ADDR, vec![0x09, 0]),
        Transaction::write(ADDR, vec![0x08, 7]),
    ]
}

#[test]
fn initialization_awaits_i2c_and_delay() {
    let (i2c, control, mut mock) = bus(&init_expectations());
    let mut delay = YieldingDelay::default();
    let sensor = block_on(AsyncDps3xx::new_async(
        i2c,
        I2cAddress::Primary,
        Config::new(),
        &mut delay,
    ))
    .unwrap();
    assert!(sensor.is_ready());
    assert_eq!(*delay.0.borrow(), vec![13_000_000]);
    assert_eq!(control.transfers.get(), init_expectations().len());
    let _i2c = sensor.release();
    mock.done();
}

#[test]
fn cancelled_reconfiguration_recovers_old_configuration_at_every_write() {
    let writes = [
        Transaction::write(ADDR, vec![0x08, 0]),
        Transaction::write(ADDR, vec![0x06, 0x54]),
        Transaction::write(ADDR, vec![0x07, 0x20]),
        Transaction::write(ADDR, vec![0x09, 0x06]),
        Transaction::write(ADDR, vec![0x0C, 0x80]),
        Transaction::write(ADDR, vec![0x08, 7]),
    ];
    for before in [true, false] {
        for offset in 0..writes.len() {
            let mut expectations = init_expectations();
            let initial_count = expectations.len();
            let completed = offset + usize::from(!before);
            expectations.extend_from_slice(&writes[..completed]);
            expectations.extend(restore_configuration());
            let (i2c, control, mut mock) = bus(&expectations);
            let mut sensor = block_on(AsyncDps3xx::new_async(
                i2c,
                I2cAddress::Primary,
                Config::new(),
                &mut YieldingDelay::default(),
            ))
            .unwrap();
            let original = sensor.config();
            control.pause_at.set(Some(initial_count + offset));
            control.pause_before.set(before);
            let requested = Config::navigation().fifo(FifoConfig {
                enable: true,
                interrupt_on_full: false,
            });
            cancel_at_pause(sensor.reconfigure(requested), &control);
            assert!(!sensor.is_ready());
            assert_eq!(sensor.init_stage(), InitStage::ApplyConfig);
            assert_eq!(sensor.config(), original);
            assert_eq!(block_on(sensor.read_sample()), Err(Error::NotInitialized));
            // Mode-only retries must never mark a partially rewritten config ready.
            assert_eq!(
                block_on(sensor.set_mode(Mode::Idle)),
                Err(Error::NotInitialized)
            );
            assert_eq!(block_on(sensor.standby()), Err(Error::NotInitialized));
            assert_eq!(
                block_on(sensor.start_background()),
                Err(Error::NotInitialized)
            );
            control.resume();
            assert_eq!(block_on(sensor.poll_init()).unwrap(), InitPoll::Ready(()));
            assert!(sensor.is_ready());
            assert_eq!(sensor.config(), original);
            mock.done();
        }
    }
}

fn fifo_config() -> Config {
    Config::new().fifo(FifoConfig {
        enable: true,
        interrupt_on_full: false,
    })
}

fn fifo_init_expectations() -> Vec<Transaction> {
    let mut expectations = init_expectations();
    // Replace the final CFG_REG/mode writes with FIFO enable, flush and mode.
    expectations.truncate(expectations.len() - 2);
    expectations.extend([
        Transaction::write(ADDR, vec![0x09, 2]),
        Transaction::write(ADDR, vec![0x0C, 0x80]),
        Transaction::write(ADDR, vec![0x08, 7]),
    ]);
    expectations
}

fn queued_temperature() -> [Transaction; 2] {
    [
        Transaction::write_read(ADDR, vec![0x0B], vec![0]),
        Transaction::write_read(ADDR, vec![0x00], vec![0, 0, 0]),
    ]
}

// Exercise explicit mode selection, both convenience methods, and both initialization
// entry points. The strict transaction queue forbids config rewrites or a FIFO flush.
fn recover_mode(sensor: &mut AsyncDps3xx<YieldingI2c>, route: usize) {
    match route {
        0 => block_on(sensor.set_mode(Mode::Idle)).unwrap(),
        1 => block_on(sensor.standby()).unwrap(),
        2 => block_on(sensor.start_background()).unwrap(),
        3 => assert_eq!(block_on(sensor.poll_init()).unwrap(), InitPoll::Ready(())),
        4 => block_on(sensor.init_async(&mut YieldingDelay::default())).unwrap(),
        _ => unreachable!(),
    }
}

#[test]
fn cancelled_mode_change_supports_direct_and_polled_recovery_without_flushing_fifo() {
    for before in [true, false] {
        for route in 0..5 {
            let mut expectations = fifo_init_expectations();
            let initial_count = expectations.len();
            if !before {
                expectations.push(Transaction::write(ADDR, vec![0x08, 0]));
            }
            let recovered_mode = if route < 2 {
                Mode::Idle
            } else {
                Mode::BackgroundAll
            };
            expectations.push(Transaction::write(
                ADDR,
                vec![0x08, if route < 2 { 0 } else { 7 }],
            ));
            expectations.extend(queued_temperature());
            let (i2c, control, mut mock) = bus(&expectations);
            let mut sensor = block_on(AsyncDps3xx::new_async(
                i2c,
                I2cAddress::Primary,
                fifo_config(),
                &mut YieldingDelay::default(),
            ))
            .unwrap();
            control.pause_at.set(Some(initial_count));
            control.pause_before.set(before);
            cancel_at_pause(sensor.standby(), &control);
            assert!(!sensor.is_ready());
            assert_eq!(sensor.init_stage(), InitStage::ApplyMode);
            assert_eq!(sensor.config(), fifo_config());
            assert_eq!(block_on(sensor.try_pop_fifo()), Err(Error::NotInitialized));
            control.resume();
            recover_mode(&mut sensor, route);
            assert!(sensor.is_ready());
            assert_eq!(sensor.config(), fifo_config().mode(recovered_mode));
            assert_eq!(
                block_on(sensor.try_pop_fifo()).unwrap(),
                Some(FifoEntry::Temperature(0.0))
            );
            mock.done();
        }
    }
}

#[test]
fn failed_mode_change_supports_direct_and_polled_recovery_without_flushing_fifo() {
    for route in 0..5 {
        let mut expectations = fifo_init_expectations();
        expectations.extend([
            Transaction::write(ADDR, vec![0x08, 0]).with_error(ErrorKind::Other),
            Transaction::write(ADDR, vec![0x08, if route < 2 { 0 } else { 7 }]),
        ]);
        expectations.extend(queued_temperature());
        let (i2c, _, mut mock) = bus(&expectations);
        let mut sensor = block_on(AsyncDps3xx::new_async(
            i2c,
            I2cAddress::Primary,
            fifo_config(),
            &mut YieldingDelay::default(),
        ))
        .unwrap();
        assert_eq!(
            block_on(sensor.set_mode(Mode::Idle)),
            Err(Error::Bus(ErrorKind::Other))
        );
        assert_eq!(sensor.init_stage(), InitStage::ApplyMode);
        assert_eq!(sensor.config(), fifo_config());
        recover_mode(&mut sensor, route);
        assert!(sensor.is_ready());
        assert_eq!(
            block_on(sensor.try_pop_fifo()).unwrap(),
            Some(FifoEntry::Temperature(0.0))
        );
        mock.done();
    }
}

#[test]
fn interrupted_mode_recovery_remains_retryable() {
    for before in [true, false] {
        let mut expectations = fifo_init_expectations();
        expectations.push(Transaction::write(ADDR, vec![0x08, 0]).with_error(ErrorKind::Other));
        let recovery_index = expectations.len();
        if !before {
            expectations.push(Transaction::write(ADDR, vec![0x08, 7]));
        }
        expectations.extend([
            Transaction::write(ADDR, vec![0x08, 7]).with_error(ErrorKind::Other),
            Transaction::write(ADDR, vec![0x08, 0]),
        ]);
        expectations.extend(queued_temperature());
        let (i2c, control, mut mock) = bus(&expectations);
        let mut sensor = block_on(AsyncDps3xx::new_async(
            i2c,
            I2cAddress::Primary,
            fifo_config(),
            &mut YieldingDelay::default(),
        ))
        .unwrap();
        assert_eq!(
            block_on(sensor.standby()),
            Err(Error::Bus(ErrorKind::Other))
        );
        control.pause_at.set(Some(recovery_index));
        control.pause_before.set(before);
        cancel_at_pause(sensor.poll_init(), &control);
        assert_eq!(sensor.init_stage(), InitStage::ApplyMode);
        control.resume();
        assert_eq!(
            block_on(sensor.poll_init()),
            Err(Error::Bus(ErrorKind::Other))
        );
        assert_eq!(sensor.init_stage(), InitStage::ApplyMode);
        assert_eq!(sensor.config(), fifo_config());
        block_on(sensor.standby()).unwrap();
        assert!(sensor.is_ready());
        assert_eq!(sensor.config(), fifo_config().mode(Mode::Idle));
        assert_eq!(
            block_on(sensor.try_pop_fifo()).unwrap(),
            Some(FifoEntry::Temperature(0.0))
        );
        mock.done();
    }
}

#[test]
fn cancelled_reset_requires_initialization() {
    let mut expectations = init_expectations();
    let initial_count = expectations.len();
    expectations.push(Transaction::write(ADDR, vec![0x0C, 0x89]));
    expectations.extend(init_expectations());
    let (i2c, control, mut mock) = bus(&expectations);
    let mut sensor = block_on(AsyncDps3xx::new_async(
        i2c,
        I2cAddress::Primary,
        Config::new(),
        &mut YieldingDelay::default(),
    ))
    .unwrap();
    control.pause_at.set(Some(initial_count));
    cancel_at_pause(sensor.reset(), &control);
    assert!(!sensor.is_ready());
    assert_eq!(sensor.init_stage(), InitStage::SensorReady);
    control.resume();
    block_on(sensor.init_async(&mut YieldingDelay::default())).unwrap();
    assert!(sensor.is_ready());
    mock.done();
}

#[test]
fn cancelled_priming_read_triggers_a_fresh_conversion() {
    let initial = init_expectations();
    let mut expectations = initial[..17].to_vec();
    expectations.push(Transaction::write_read(ADDR, vec![0x08], vec![0xC0]));
    expectations.extend_from_slice(&initial[3..]);
    let (i2c, control, mut mock) = bus(&expectations);
    let mut sensor = AsyncDps3xx::new(i2c, I2cAddress::Primary, Config::new()).unwrap();
    assert_eq!(
        block_on(sensor.poll_init()).unwrap(),
        InitPoll::Pending { wait_ms: 13 }
    );
    control.pause_at.set(Some(16));
    cancel_at_pause(sensor.poll_init(), &control);
    assert!(!sensor.is_ready());
    assert_eq!(sensor.init_stage(), InitStage::CoefficientsReady);
    control.resume();
    block_on(sensor.init_async(&mut YieldingDelay::default())).unwrap();
    assert!(sensor.is_ready());
    mock.done();
}

struct StalledDelay(Rc<Control>);
impl DelayNs for StalledDelay {
    async fn delay_ns(&mut self, _ns: u32) {
        self.0.pause(0, false).await;
    }
}

#[test]
fn cancelled_initialization_delay_is_repeated_before_priming_read() {
    let initial = init_expectations();
    let mut expectations = initial[..15].to_vec();
    expectations.push(Transaction::write_read(ADDR, vec![0x08], vec![0xC0]));
    expectations.extend_from_slice(&initial[3..]);
    let (i2c, _, mut mock) = bus(&expectations);
    let mut sensor = AsyncDps3xx::new(i2c, I2cAddress::Primary, Config::new()).unwrap();
    let delay_control = Rc::new(Control::default());
    delay_control.pause_at.set(Some(0));
    let mut delay = StalledDelay(delay_control.clone());
    cancel_at_pause(sensor.init_async(&mut delay), &delay_control);
    assert_eq!(sensor.init_stage(), InitStage::TemperatureReady);
    assert!(!sensor.is_ready());
    let mut delay = YieldingDelay::default();
    block_on(sensor.init_async(&mut delay)).unwrap();
    assert_eq!(*delay.0.borrow(), vec![13_000_000]);
    assert!(sensor.is_ready());
    mock.done();
}
