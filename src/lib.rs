//! Driver for the Bosch BMM350 three-axis magnetometer over I2C.
//!
//! The device knowledge (register map, configuration encoders, raw decoding, OTP calibration and
//! compensation) is plain code in [`reg`], [`config`] and [`data`] and needs no bus. [`Bmm350`]
//! is the one implementation, written against `embedded-hal-async`; [`blocking::Bmm350`] wraps it
//! for `embedded-hal` bus and delay types without duplicating the logic. The driver owns neither
//! the INT pin nor a timer: it configures the chip's interrupt output and the application waits on
//! the pin.
//!
//! ```ignore
//! let mut mag = bmm350::Bmm350::new(i2c, bmm350::DEFAULT_ADDRESS);
//! mag.init(&mut delay, &bmm350::Config::default()).await?;
//! loop {
//!     int_pin.wait_for_rising_edge().await;
//!     let field = mag.read_field().await?; // microtesla, degrees Celsius
//! }
//! ```
//!
//! ```ignore
//! let mut mag = bmm350::blocking::Bmm350::new(i2c, bmm350::DEFAULT_ADDRESS);
//! mag.init(&mut delay, &bmm350::Config::default())?;
//! let raw = mag.read()?;
//! let field = bmm350::compensate(mag.calibration(), &raw);
//! ```
//!
//! Units: [`Field`] is in microtesla and degrees Celsius. [`RawSample`] is in sensor counts.
//! Hard-iron and soft-iron calibration of the installed sensor is the application's job.

#![no_std]
#![forbid(unsafe_code)]

#[cfg(test)]
extern crate std;

pub mod blocking;
pub mod config;
pub mod data;
pub mod reg;

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

pub use config::{Averaging, Axes, Config, InterruptStatus, OutputDataRate, PowerMode};
pub use data::{Calibration, Field, RawSample, compensate};
pub use reg::{ADDRESS_ADSEL_HIGH, ADDRESS_ADSEL_LOW};

/// Default 7-bit address (ADSEL tied to GND).
pub const DEFAULT_ADDRESS: u8 = ADDRESS_ADSEL_LOW;

/// Longest single bus read of register bytes; longer reads are split.
const READ_CHUNK: usize = 32;
/// Polls of `OTP_STATUS_REG` per word before giving up.
const OTP_POLLS: u8 = 10;

/// Driver error. `E` is the bus error type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// The bus failed.
    Bus(E),
    /// `CHIP_ID` did not read 0x33.
    WrongId {
        /// Value read.
        found: u8,
    },
    /// The OTP reported an error or never signalled done; `status` is `OTP_STATUS_REG`.
    OtpError {
        /// Last `OTP_STATUS_REG` value.
        status: u8,
    },
    /// A magnetic reset step was not acknowledged by `PMU_CMD_STATUS_0`.
    ResetFailed {
        /// `PMU_CMD_STATUS_0` value read.
        status: u8,
    },
    /// The configuration enables no axis.
    NoAxes,
}

/// BMM350 on an async I2C bus.
pub struct Bmm350<I2C> {
    i2c: I2C,
    address: u8,
    calibration: Calibration,
}

impl<I2C: I2c> Bmm350<I2C> {
    /// Wrap a bus. Nothing is sent until [`init`](Self::init).
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            calibration: Calibration::default(),
        }
    }

    /// Give the bus back.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Calibration read from OTP by [`init`](Self::init); all-zero coefficients before that.
    pub fn calibration(&self) -> &Calibration {
        &self.calibration
    }

    /// Bring the chip from any state to the configured one, in this order: soft reset, wait,
    /// chip ID check, OTP read, magnetic reset, ODR and averaging, axes, data-ready interrupt
    /// (if configured), power mode. Returns right after the power mode is written; the first
    /// sample follows after the start-up time (datasheet p8: 70 ms typical to normal mode), so
    /// wait for the data-ready edge before the first read.
    ///
    /// Soft-reset wait, OTP sequence and magnetic reset follow the vendor `bmm350_init`
    /// (`bmm350.c`); the datasheet only says that the boot ends with 0x80 in `OTP_CMD_REG` (p14)
    /// and that a soft reset is `0xB6` then `0x00` in `CMD` (p22).
    pub async fn init(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<(), Error<I2C::Error>> {
        if !config.axes.any() {
            return Err(Error::NoAxes);
        }
        delay.delay_us(3_000).await;
        self.write_register(reg::CMD, reg::CMD_SOFTRESET).await?;
        delay.delay_us(24_000).await;
        self.write_register(reg::CMD, reg::CMD_NOP).await?;

        let found = self.read_register(reg::CHIP_ID).await?;
        if found != reg::CHIP_ID_VALUE {
            return Err(Error::WrongId { found });
        }

        let mut words = [0u16; reg::OTP_WORDS];
        for (i, w) in words.iter_mut().enumerate() {
            *w = self.read_otp_word(delay, i as u8).await?;
        }
        self.write_register(reg::OTP_CMD_REG, reg::OTP_CMD_PWR_OFF_OTP)
            .await?;
        self.calibration = Calibration::from_otp(&words);

        self.magnetic_reset(delay).await?;

        self.write_register(
            reg::PMU_CMD_AGGR_SET,
            config::aggr_set(config.odr, config.averaging),
        )
        .await?;
        self.write_register(reg::PMU_CMD, reg::PMU_CMD_UPD_OAE)
            .await?;
        delay.delay_us(1_000).await;
        self.write_register(reg::PMU_CMD_AXIS_EN, config.axes.bits())
            .await?;
        if config.data_ready_interrupt {
            self.write_register(reg::INT_CTRL, config::int_ctrl_data_ready())
                .await?;
        }
        if config.power_mode != PowerMode::Suspend {
            self.write_register(reg::PMU_CMD, config.power_mode.code())
                .await?;
        }
        Ok(())
    }

    /// Bit reset then flux-guide reset, each with full recharge, checking the command echo in
    /// `PMU_CMD_STATUS_0`. The chip must be in suspend. The datasheet (p17) recommends it after
    /// a long stay in suspend or a field shock; `init` already runs it. Sequence, delays and
    /// echo check follow the vendor `bmm350_magnetic_reset_and_wait` (`bmm350.c`).
    pub async fn magnetic_reset(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error<I2C::Error>> {
        for (cmd, wait_us) in [(reg::PMU_CMD_BR, 14_000), (reg::PMU_CMD_FGR, 18_000)] {
            self.write_register(reg::PMU_CMD, cmd).await?;
            delay.delay_us(wait_us).await;
            let status = self.read_register(reg::PMU_CMD_STATUS_0).await?;
            if (status & reg::PMU_STATUS_0_CMD_VALUE_MASK) >> 5 != cmd {
                return Err(Error::ResetFailed { status });
            }
        }
        Ok(())
    }

    /// Change the power mode. A chip in normal mode is sent to suspend first (6 ms wait, vendor
    /// `BMM350_GOTO_SUSPEND_DELAY`), because the chip only accepts forced mode from suspend.
    /// Does not wait for the first sample after the new mode starts.
    pub async fn set_power_mode(
        &mut self,
        delay: &mut impl DelayNs,
        mode: PowerMode,
    ) -> Result<(), Error<I2C::Error>> {
        let status = self.read_register(reg::PMU_CMD_STATUS_0).await?;
        if status & reg::PMU_STATUS_0_NORMAL != 0 {
            self.write_register(reg::PMU_CMD, reg::PMU_CMD_SUS).await?;
            delay.delay_us(6_000).await;
        }
        if mode != PowerMode::Suspend {
            self.write_register(reg::PMU_CMD, mode.code()).await?;
        }
        Ok(())
    }

    /// Trigger one forced-mode conversion (full recharge). The chip must be in suspend; the
    /// result is ready when data-ready asserts, after which the chip is back in suspend.
    pub async fn trigger_forced(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(reg::PMU_CMD, reg::PMU_CMD_FM).await
    }

    /// Read `INT_STATUS`. Clears the flag.
    pub async fn interrupt_status(&mut self) -> Result<InterruptStatus, Error<I2C::Error>> {
        Ok(InterruptStatus::from_bits(
            self.read_register(reg::INT_STATUS).await?,
        ))
    }

    /// Read X, Y, Z and temperature in one bus transaction (dummy bytes included). The chip
    /// holds its data registers still during a burst read that starts at `MAG_X_XLSB`.
    pub async fn read(&mut self) -> Result<RawSample, Error<I2C::Error>> {
        let mut buf = [0u8; reg::DUMMY_BYTES + 12];
        self.i2c
            .write_read(self.address, &[reg::MAG_X_XLSB], &mut buf)
            .await
            .map_err(Error::Bus)?;
        let mut data = [0u8; 12];
        data.copy_from_slice(&buf[reg::DUMMY_BYTES..]);
        Ok(RawSample::decode(&data))
    }

    /// [`read`](Self::read) then [`compensate`].
    pub async fn read_field(&mut self) -> Result<Field, Error<I2C::Error>> {
        let raw = self.read().await?;
        Ok(compensate(&self.calibration, &raw))
    }

    /// Read one register.
    pub async fn read_register(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut b = [0u8; 1];
        self.read_registers(reg, &mut b).await?;
        Ok(b[0])
    }

    /// Write one register.
    pub async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[reg, value])
            .await
            .map_err(Error::Bus)
    }

    /// Read `out.len()` bytes from consecutive registers starting at `reg`, without the dummy
    /// bytes. Reads longer than 32 bytes are split into several transactions.
    pub async fn read_registers(
        &mut self,
        reg: u8,
        out: &mut [u8],
    ) -> Result<(), Error<I2C::Error>> {
        let mut reg = reg;
        for chunk in out.chunks_mut(READ_CHUNK) {
            let mut buf = [0u8; reg::DUMMY_BYTES + READ_CHUNK];
            let n = reg::DUMMY_BYTES + chunk.len();
            self.i2c
                .write_read(self.address, &[reg], &mut buf[..n])
                .await
                .map_err(Error::Bus)?;
            chunk.copy_from_slice(&buf[reg::DUMMY_BYTES..n]);
            reg = reg.wrapping_add(chunk.len() as u8);
        }
        Ok(())
    }

    /// Read one 16-bit OTP word (0..32). Only valid between a reset and the `PWR_OFF_OTP`
    /// command that ends `init`'s OTP phase. Procedure from the vendor `read_otp_word`
    /// (`bmm350.c`): write the read command, wait 300 us, poll `OTP_STATUS_REG`, then read the
    /// MSB and LSB registers.
    pub async fn read_otp_word(
        &mut self,
        delay: &mut impl DelayNs,
        word: u8,
    ) -> Result<u16, Error<I2C::Error>> {
        self.write_register(reg::OTP_CMD_REG, reg::OTP_CMD_DIR_READ | (word & 0x1F))
            .await?;
        let mut status = 0;
        for _ in 0..OTP_POLLS {
            delay.delay_us(300).await;
            status = self.read_register(reg::OTP_STATUS_REG).await?;
            if status & reg::OTP_STATUS_ERROR_MASK != 0 {
                break;
            }
            if status & reg::OTP_STATUS_CMD_DONE != 0 {
                let mut d = [0u8; 2];
                self.read_registers(reg::OTP_DATA_MSB_REG, &mut d).await?;
                return Ok(u16::from_be_bytes(d));
            }
        }
        Err(Error::OtpError { status })
    }
}
