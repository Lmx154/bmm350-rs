//! Blocking front-end: the same driver, run to completion on `embedded-hal` bus and delay types.
//!
//! A private adapter implements the async traits by calling the blocking ones, so the futures of
//! the async driver never pend; each method here polls them to completion with a no-op waker.

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

use embedded_hal::delay::DelayNs as BlockingDelay;
use embedded_hal::i2c::{ErrorType, I2c as BlockingI2c, Operation};

use crate::{Calibration, Config, Error, Field, InterruptStatus, PowerMode, RawSample};

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
    ) -> Result<(), Error<I2C::Error>> {
        run(self.inner.init(&mut Delay(delay), config))
    }

    /// See [`crate::Bmm350::magnetic_reset`].
    pub fn magnetic_reset(
        &mut self,
        delay: &mut impl BlockingDelay,
    ) -> Result<(), Error<I2C::Error>> {
        run(self.inner.magnetic_reset(&mut Delay(delay)))
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

    /// See [`crate::Bmm350::interrupt_status`].
    pub fn interrupt_status(&mut self) -> Result<InterruptStatus, Error<I2C::Error>> {
        run(self.inner.interrupt_status())
    }

    /// See [`crate::Bmm350::read`].
    pub fn read(&mut self) -> Result<RawSample, Error<I2C::Error>> {
        run(self.inner.read())
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reg;
    use std::vec::Vec;

    /// Register-file fake of the chip: two dummy bytes before read data, OTP words served
    /// through the OTP registers, PMU command echo in `PMU_CMD_STATUS_0`.
    struct FakeChip {
        regs: [u8; 0x80],
        otp: [u16; 32],
        writes: Vec<(u8, u8)>,
        reads: Vec<(u8, usize)>,
    }

    impl FakeChip {
        fn new() -> Self {
            let mut regs = [0u8; 0x80];
            regs[reg::CHIP_ID as usize] = reg::CHIP_ID_VALUE;
            let mut otp = [0u16; 32];
            otp[0x0E] = 0x0064; // offset X = 100
            otp[0x10] = 0x0500; // sensitivity X = 5/256
            // Data block: x = 1000, y = -1000, z = 2000, temperature = 51450.
            let data = [
                0xE8, 0x03, 0x00, 0x18, 0xFC, 0xFF, 0xD0, 0x07, 0x00, 0xFA, 0xC8, 0x00,
            ];
            regs[0x31..0x3D].copy_from_slice(&data);
            Self {
                regs,
                otp,
                writes: Vec::new(),
                reads: Vec::new(),
            }
        }

        fn write(&mut self, reg: u8, val: u8) {
            self.writes.push((reg, val));
            self.regs[reg as usize] = val;
            match reg {
                reg::OTP_CMD_REG if val & 0xE0 == reg::OTP_CMD_DIR_READ => {
                    let w = self.otp[(val & 0x1F) as usize];
                    self.regs[reg::OTP_DATA_MSB_REG as usize] = (w >> 8) as u8;
                    self.regs[reg::OTP_DATA_LSB_REG as usize] = w as u8;
                    self.regs[reg::OTP_STATUS_REG as usize] = reg::OTP_STATUS_CMD_DONE;
                }
                reg::PMU_CMD if val == reg::PMU_CMD_BR || val == reg::PMU_CMD_FGR => {
                    self.regs[reg::PMU_CMD_STATUS_0 as usize] = val << 5;
                }
                _ => {}
            }
        }
    }

    impl ErrorType for FakeChip {
        type Error = core::convert::Infallible;
    }

    impl BlockingI2c for FakeChip {
        fn transaction(
            &mut self,
            address: u8,
            ops: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            assert_eq!(address, reg::ADDRESS_ADSEL_LOW);
            let mut reg = 0usize;
            for op in ops {
                match op {
                    Operation::Write(w) if w.len() == 2 => self.write(w[0], w[1]),
                    Operation::Write(w) => reg = w[0] as usize,
                    Operation::Read(r) => {
                        self.reads.push((reg as u8, r.len()));
                        for (i, b) in r.iter_mut().enumerate() {
                            *b = if i < reg::DUMMY_BYTES {
                                0xAA
                            } else {
                                self.regs[reg + i - reg::DUMMY_BYTES]
                            };
                        }
                    }
                }
            }
            Ok(())
        }
    }

    struct NoDelay;

    impl BlockingDelay for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    #[test]
    fn init_then_read() {
        let mut mag = Bmm350::new(FakeChip::new(), reg::ADDRESS_ADSEL_LOW);
        mag.init(&mut NoDelay, &Config::default()).unwrap();
        assert!((mag.calibration().offset[0] - 100.0).abs() < 1e-6);

        let raw = mag.read().unwrap();
        assert_eq!(
            raw,
            RawSample {
                x: 1000,
                y: -1000,
                z: 2000,
                temperature: 51450
            }
        );
        let f = mag.read_field().unwrap();
        assert!(f.x > 100.0 && f.temperature > 20.0 && f.temperature < 40.0);

        let chip = mag.release();
        let mut expected: Vec<(u8, u8)> = std::vec![(reg::CMD, 0xB6), (reg::CMD, 0x00)];
        for w in 0..32u8 {
            expected.push((reg::OTP_CMD_REG, 0x20 | w));
        }
        expected.extend([
            (reg::OTP_CMD_REG, 0x80),
            (reg::PMU_CMD, reg::PMU_CMD_BR),
            (reg::PMU_CMD, reg::PMU_CMD_FGR),
            (reg::PMU_CMD_AGGR_SET, 0x14),
            (reg::PMU_CMD, reg::PMU_CMD_UPD_OAE),
            (reg::PMU_CMD_AXIS_EN, 0x07),
            (reg::INT_CTRL, 0x8E),
            (reg::PMU_CMD, reg::PMU_CMD_NM),
        ]);
        assert_eq!(chip.writes, expected);

        // The data read is one transaction of 12 data bytes plus the 2 dummy bytes.
        assert_eq!(
            chip.reads
                .iter()
                .filter(|r| **r == (reg::MAG_X_XLSB, 14))
                .count(),
            2
        );
    }

    #[test]
    fn wrong_chip_id_is_reported() {
        let mut chip = FakeChip::new();
        chip.regs[reg::CHIP_ID as usize] = 0x42;
        let mut mag = Bmm350::new(chip, reg::ADDRESS_ADSEL_LOW);
        assert_eq!(
            mag.init(&mut NoDelay, &Config::default()),
            Err(Error::WrongId { found: 0x42 })
        );
    }
}
