//! Blocking front-end: the same driver, run to completion on `embedded-hal` bus and delay types.
//!
//! A private adapter implements the async traits by calling the blocking ones, so the futures of
//! the async driver never pend; each method here polls them to completion with a no-op waker.

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

use embedded_hal::delay::DelayNs as BlockingDelay;
use embedded_hal::i2c::{ErrorType, I2c as BlockingI2c, Operation};

use crate::{
    Calibration, Config, Error, ErrorStatus, Field, Info, InterruptStatus, PmuStatus, PowerMode,
    RawSample, SelfTestReport, TimedSample,
};

/// Async-trait view of a blocking bus.
struct Bus<I2C>(I2C);

impl<I2C: BlockingI2c> ErrorType for Bus<I2C> {
    type Error = I2C::Error;
}

impl<I2C: BlockingI2c> embedded_hal_async::i2c::I2c for Bus<I2C> {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.0.read(address, read)
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.0.write(address, write)
    }

    async fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.0.write_read(address, write, read)
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.0.transaction(address, operations)
    }
}

/// Async-trait view of a blocking delay.
struct Delay<'a, D>(&'a mut D);

impl<D: BlockingDelay> embedded_hal_async::delay::DelayNs for Delay<'_, D> {
    async fn delay_ns(&mut self, ns: u32) {
        self.0.delay_ns(ns);
    }

    async fn delay_us(&mut self, us: u32) {
        self.0.delay_us(us);
    }

    async fn delay_ms(&mut self, ms: u32) {
        self.0.delay_ms(ms);
    }
}

/// Run a future that never pends.
fn run<F: Future>(f: F) -> F::Output {
    let mut f = pin!(f);
    let mut cx = Context::from_waker(Waker::noop());
    loop {
        if let Poll::Ready(v) = f.as_mut().poll(&mut cx) {
            return v;
        }
    }
}

/// BMM350 on a blocking I2C bus. Same methods as [`crate::Bmm350`] without `async`.
pub struct Bmm350<I2C> {
    inner: crate::Bmm350<Bus<I2C>>,
}

impl<I2C: BlockingI2c> Bmm350<I2C> {
    /// Wrap a bus. Nothing is sent until [`init`](Self::init).
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            inner: crate::Bmm350::new(Bus(i2c), address),
        }
    }

    /// Give the bus back.
    pub fn release(self) -> I2C {
        self.inner.release().0
    }

    /// See [`crate::Bmm350::calibration`].
    pub fn calibration(&self) -> &Calibration {
        self.inner.calibration()
    }

    /// See [`crate::Bmm350::init`].
    pub fn init(
        &mut self,
        delay: &mut impl BlockingDelay,
        config: &Config,
    ) -> Result<Info, Error<I2C::Error>> {
        run(self.inner.init(&mut Delay(delay), config))
    }

    /// See [`crate::Bmm350::configure`].
    pub fn configure(
        &mut self,
        delay: &mut impl BlockingDelay,
        config: &Config,
    ) -> Result<Info, Error<I2C::Error>> {
        run(self.inner.configure(&mut Delay(delay), config))
    }

    /// See [`crate::Bmm350::config`].
    pub fn config(&self) -> Option<&Config> {
        self.inner.config()
    }

    /// See [`crate::Bmm350::magnetic_reset`].
    pub fn magnetic_reset(
        &mut self,
        delay: &mut impl BlockingDelay,
    ) -> Result<(), Error<I2C::Error>> {
        run(self.inner.magnetic_reset(&mut Delay(delay)))
    }

    /// See [`crate::Bmm350::magnetic_reset_fast`].
    pub fn magnetic_reset_fast(
        &mut self,
        delay: &mut impl BlockingDelay,
    ) -> Result<(), Error<I2C::Error>> {
        run(self.inner.magnetic_reset_fast(&mut Delay(delay)))
    }

    /// See [`crate::Bmm350::self_test`].
    pub fn self_test(
        &mut self,
        delay: &mut impl BlockingDelay,
    ) -> Result<SelfTestReport, Error<I2C::Error>> {
        run(self.inner.self_test(&mut Delay(delay)))
    }

    /// See [`crate::Bmm350::set_power_mode`].
    pub fn set_power_mode(
        &mut self,
        delay: &mut impl BlockingDelay,
        mode: PowerMode,
    ) -> Result<(), Error<I2C::Error>> {
        run(self.inner.set_power_mode(&mut Delay(delay), mode))
    }

    /// See [`crate::Bmm350::trigger_forced`].
    pub fn trigger_forced(&mut self) -> Result<(), Error<I2C::Error>> {
        run(self.inner.trigger_forced())
    }

    /// See [`crate::Bmm350::trigger_forced_fast`].
    pub fn trigger_forced_fast(&mut self) -> Result<(), Error<I2C::Error>> {
        run(self.inner.trigger_forced_fast())
    }

    /// See [`crate::Bmm350::pmu_status`].
    pub fn pmu_status(&mut self) -> Result<PmuStatus, Error<I2C::Error>> {
        run(self.inner.pmu_status())
    }

    /// See [`crate::Bmm350::error_status`].
    pub fn error_status(&mut self) -> Result<ErrorStatus, Error<I2C::Error>> {
        run(self.inner.error_status())
    }

    /// See [`crate::Bmm350::verify`].
    pub fn verify(&mut self) -> Result<(), Error<I2C::Error>> {
        run(self.inner.verify())
    }

    /// See [`crate::Bmm350::recover`].
    pub fn recover(&mut self, delay: &mut impl BlockingDelay) -> Result<(), Error<I2C::Error>> {
        run(self.inner.recover(&mut Delay(delay)))
    }

    /// See [`crate::Bmm350::interrupt_status`].
    pub fn interrupt_status(&mut self) -> Result<InterruptStatus, Error<I2C::Error>> {
        run(self.inner.interrupt_status())
    }

    /// See [`crate::Bmm350::sensor_time`].
    pub fn sensor_time(&mut self) -> Result<u32, Error<I2C::Error>> {
        run(self.inner.sensor_time())
    }

    /// See [`crate::Bmm350::read`].
    pub fn read(&mut self) -> Result<RawSample, Error<I2C::Error>> {
        run(self.inner.read())
    }

    /// See [`crate::Bmm350::read_with_time`].
    pub fn read_with_time(&mut self) -> Result<TimedSample, Error<I2C::Error>> {
        run(self.inner.read_with_time())
    }

    /// See [`crate::Bmm350::read_field`].
    pub fn read_field(&mut self) -> Result<Field, Error<I2C::Error>> {
        run(self.inner.read_field())
    }

    /// See [`crate::Bmm350::read_register`].
    pub fn read_register(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        run(self.inner.read_register(reg))
    }

    /// See [`crate::Bmm350::write_register`].
    pub fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        run(self.inner.write_register(reg, value))
    }

    /// See [`crate::Bmm350::read_registers`].
    pub fn read_registers(&mut self, reg: u8, out: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        run(self.inner.read_registers(reg, out))
    }

    /// See [`crate::Bmm350::read_otp_word`].
    pub fn read_otp_word(
        &mut self,
        delay: &mut impl BlockingDelay,
        word: u8,
    ) -> Result<u16, Error<I2C::Error>> {
        run(self.inner.read_otp_word(&mut Delay(delay), word))
    }
}
