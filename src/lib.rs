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
#![warn(missing_docs)]
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::unreachable,
        clippy::todo,
        clippy::unimplemented,
        clippy::indexing_slicing,
        clippy::undocumented_unsafe_blocks
    )
)]

#[cfg(test)]
extern crate std;

pub mod blocking;
pub mod config;
pub mod data;
mod image;
pub mod plan;
pub mod reg;
pub mod selftest;
pub mod timing;

#[cfg(test)]
mod driver_tests;
#[cfg(test)]
mod testbus;
#[cfg(test)]
mod tests;

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

use plan::{Apply, Plan, Step};

pub use config::{
    Averaging, Axes, Config, ConfigError, I2cWatchdog, InterruptDrive, InterruptMode, InterruptPin,
    InterruptPolarity, InterruptStatus, OutputDataRate, PowerMode, StartMode,
};
pub use data::{Calibration, Field, RawSample, compensate};
pub use reg::{ADDRESS_ADSEL_HIGH, ADDRESS_ADSEL_LOW};
pub use selftest::{SelfTestReport, exceeds_high_field};

/// Default 7-bit address (ADSEL tied to GND).
pub const DEFAULT_ADDRESS: u8 = ADDRESS_ADSEL_LOW;

/// Longest single bus read of register bytes; longer reads are split.
const READ_CHUNK: usize = 32;
/// Polls of `OTP_STATUS_REG` per word before giving up.
const OTP_POLLS: u8 = 10;
/// Wait before each re-read of a busy `PMU_CMD_STATUS_0`, in microseconds.
const PMU_POLL_US: u32 = 1_000;
/// Quiet time of `recover` before its first read, in microseconds (contract A4g).
const RECOVER_QUIET_US: u32 = 2_000;
/// The `TMR_SELFTEST_USER` value that switches the stimulus off (8.33, p42).
const ST_OFF: u8 = reg::TMR_SELFTEST_USER_RESET;
/// Self-test: `PMU_CMD_AGGR_SET` while it runs, 100 Hz and averaging 2 (vendor
/// `self_test_entry_config`), which is also the reset value.
const SELF_TEST_AGGR_SET: u8 = reg::PMU_CMD_AGGR_SET_RESET;
/// Self-test: `PMU_CMD_AXIS_EN` while it runs, all axes on (vendor `self_test_entry_config`).
const SELF_TEST_AXIS_EN: u8 = reg::PMU_CMD_AXIS_EN_RESET;
/// Self-test: wait after a write of `TMR_SELFTEST_USER`, before the next command, in
/// microseconds (vendor `self_test_config`: 1 ms after the stimulus on and after it off).
const SELF_TEST_SETTLE_US: u32 = 1_000;
/// Self-test: `FM_FAST` at averaging 2, no status poll before this, in microseconds. Measured on
/// the board, 2026-11-10: 4.3 ms to data ready; the vendor waits 5 ms
/// (`BMM350_SUS_TO_FORCEDMODE_FAST_AVG_2_DELAY`); 6 ms leaves a margin over both.
const SELF_TEST_FORCED_MIN_WAIT_US: u32 = 6_000;
/// Self-test: the busy poll of `FM_FAST` gives up after this, in microseconds from the write: more
/// than four times the 4.3 ms measured on the board, 2026-11-10.
const SELF_TEST_FORCED_BOUND_US: u32 = 20_000;
/// `BR_FAST`: no status poll before this, in microseconds (vendor `self_test_entry_config`: 4 ms
/// after `BR_FAST`).
const RESET_BR_FAST_MIN_WAIT_US: u32 = 4_000;
/// `FGR_FAST`: no status poll before this, in microseconds. No vendor figure; the datasheet says
/// charging the CRST capacitor takes less than 8 ms by design (5.1.5, p17).
const RESET_FGR_FAST_MIN_WAIT_US: u32 = 8_000;
/// The busy poll of a fast reset command gives up after this, in microseconds from the write: the
/// vendor waits 30 ms for the full flux-guide reset in `self_test_entry_config`.
const RESET_FAST_BOUND_US: u32 = 30_000;
/// `PMU_CMD_STATUS_0` bits 7:5 hold the low three bits of the command (8.7, p33).
const PMU_ECHO_MASK: u8 = 0x07;

/// The echo (`PMU_CMD_STATUS_0` bits 7:5) a completed command leaves: the low three bits of the
/// code; for `BR_FAST` (0x08) the vendor's 7, `BMM350_PMU_CMD_STATUS_0_BR_FAST`.
const fn expected_echo(cmd: u8) -> u8 {
    if cmd == reg::PMU_CMD_BR_FAST {
        7
    } else {
        cmd & PMU_ECHO_MASK
    }
}

/// Driver error. `E` is the bus error type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
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
    /// The driver is not configured: `init` (or, after an `init`, `configure`) has not completed
    /// since the driver was created or since the last call that began and did not finish.
    NotConfigured,
    /// The configuration is not allowed; nothing was sent to the device.
    InvalidConfig(ConfigError),
    /// The device finished a PMU command and did not accept it: `PMU_CMD_STATUS_0` flagged it
    /// illegal, did not echo the command, or showed the wrong normal-mode flag after `NM` or
    /// `SUS`.
    PmuCommandRejected {
        /// The `PMU_CMD` code that was sent.
        cmd: u8,
        /// The `PMU_CMD_STATUS_0` value read after the command finished.
        status: u8,
    },
    /// A PMU command was still busy when its time bound was reached.
    PmuTimeout {
        /// The `PMU_CMD` code that was sent.
        cmd: u8,
        /// Last `PMU_CMD_STATUS_0` value read.
        status: u8,
    },
    /// A register the applied configuration owns did not read back as configured, twice in a row.
    /// Returned by [`Bmm350::verify`] and [`Bmm350::recover`]; the driver is unchanged and still
    /// configured.
    ConfigMismatch(Mismatch),
    /// The command would be illegal in the mode the driver last commanded (`NM`, `FM` and
    /// `FM_FAST` only in suspend); nothing was sent.
    WrongMode {
        /// The mode the driver tracks: [`PowerMode::Suspend`] or [`PowerMode::Normal`].
        mode: PowerMode,
    },
}

impl<E> From<ConfigError> for Error<E> {
    fn from(e: ConfigError) -> Self {
        Self::InvalidConfig(e)
    }
}

/// A register that differs from what the applied configuration put there.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Mismatch {
    /// Register address.
    pub register: u8,
    /// The bits that were compared.
    pub mask: u8,
    /// The value expected under `mask`; bits outside `mask` are 0.
    pub expected: u8,
    /// The value read, whole register (the second of the two reads).
    pub found: u8,
}

/// `PMU_CMD_STATUS_0` and `PMU_CMD_STATUS_1` decoded (8.7, p33; 8.8, p34).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct PmuStatus {
    /// `pmu_cmd_busy`: the previous PMU command is still running (bit 0 of `PMU_CMD_STATUS_0`).
    pub busy: bool,
    /// `cmd_is_illegal` (bit 4). Set when an illegal command finishes; it stays set until a later
    /// legal command completes, and reads never clear it (measured on the board, 2026-11-10).
    pub illegal_command: bool,
    /// `pwr_mode_is_normal` (bit 3): the chip runs in normal mode.
    pub normal_mode: bool,
    /// Bits 7:5 of `PMU_CMD_STATUS_0`: the low three bits of the last command (reserved in 8.7,
    /// p33; the vendor driver reads them as the command echo).
    pub command: u8,
    /// `pmu_odr_s` (bits 3:0 of `PMU_CMD_STATUS_1`): the effective rate code. It follows
    /// `PMU_CMD_AGGR_SET` only once `NM` (from suspend), a forced command or `UPD_OAE` (in normal
    /// mode) has completed (measured on the board, 2026-11-10).
    pub odr_code: u8,
    /// `pmu_avg_s` (bits 5:4 of `PMU_CMD_STATUS_1`): the effective averaging code.
    pub avg_code: u8,
    /// `PMU_CMD_STATUS_0` as read.
    pub status_0: u8,
    /// `PMU_CMD_STATUS_1` as read.
    pub status_1: u8,
}

impl PmuStatus {
    /// Decode the two bytes.
    pub const fn from_bytes(status_0: u8, status_1: u8) -> Self {
        Self {
            busy: status_0 & reg::PMU_STATUS_0_BUSY != 0,
            illegal_command: status_0 & reg::PMU_STATUS_0_ILLEGAL != 0,
            normal_mode: status_0 & reg::PMU_STATUS_0_NORMAL != 0,
            command: (status_0 & reg::PMU_STATUS_0_CMD_VALUE_MASK) >> 5,
            odr_code: status_1 & reg::PMU_CMD_STATUS_1_ODR_MASK,
            avg_code: (status_1 & reg::PMU_CMD_STATUS_1_AVG_MASK)
                >> reg::PMU_CMD_STATUS_1_AVG_SHIFT,
            status_0,
            status_1,
        }
    }
}

/// `ERR_REG` decoded (8.2, p29).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ErrorStatus {
    /// `pmu_cmd_error` (bit 0): a PMU command was issued while the previous one had not finished
    /// and was ignored. Bits 7:1 are reserved.
    pub pmu_cmd_error: bool,
    /// `ERR_REG` as read.
    pub raw: u8,
}

impl ErrorStatus {
    /// Decode the register.
    pub const fn from_bits(raw: u8) -> Self {
        Self {
            pmu_cmd_error: raw & reg::ERR_REG_PMU_CMD_ERROR != 0,
            raw,
        }
    }
}

/// One sample and the sensor time read in the same transaction, see [`Bmm350::read_with_time`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct TimedSample {
    /// X, Y, Z and temperature counts.
    pub sample: RawSample,
    /// The 24-bit sensor-time counter in ticks of 1/[`timing::SENSOR_TIME_TICKS_PER_SECOND`] s,
    /// from 0 to `0x00FF_FFFF`: the time of this sample.
    pub sensor_time: u32,
}

/// What `init` and `configure` learned and applied.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Info {
    /// `CHIP_ID` as read by the last `init` (0x33; the low nibble is the OTP variant).
    pub chip_id: u8,
    /// Applied output period in microseconds; 0 when the configuration ends in suspend.
    pub output_period_us: u32,
    /// Applied averaging.
    pub averaging: Averaging,
    /// True when `configure` read `INT_STATUS` to leave a latched interrupt configuration and the
    /// data-ready flag was set: that flag is consumed and the application will not see it. Always
    /// false after `init`.
    pub data_ready_consumed: bool,
}

impl Default for Info {
    fn default() -> Self {
        Self {
            chip_id: 0,
            output_period_us: 0,
            averaging: Averaging::Avg1,
            data_ready_consumed: false,
        }
    }
}

/// BMM350 on an async I2C bus.
pub struct Bmm350<I2C> {
    i2c: I2C,
    address: u8,
    calibration: Calibration,
    calibrated: bool,
    chip_id: u8,
    applied: Option<Config>,
    /// The mode last commanded: `Suspend` or `Normal`.
    mode: PowerMode,
    /// `INT_CTRL` on the chip is, or may be, latched with the data-ready interrupt on: the driver
    /// wrote it so, or a write of it failed or was cancelled. Starts false because the driver
    /// has sent nothing yet and the only way to a configured driver is `init`, whose soft reset
    /// clears a latched configuration with its flag (and sets this field false again); a wrongly
    /// true value would only cost one needless `INT_STATUS` read, but a wrongly false one after
    /// a latched write is the stall of `configure`, which is why every failed or cancelled write
    /// of `INT_CTRL` sets it.
    int_latched: bool,
}

impl<I2C: I2c> Bmm350<I2C> {
    /// Wrap a bus. Nothing is sent until [`init`](Self::init).
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            calibration: Calibration::default(),
            calibrated: false,
            chip_id: 0,
            applied: None,
            mode: PowerMode::Suspend,
            int_latched: false,
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

    /// The applied configuration; `None` while the driver is not configured (before `init`
    /// completed, or after an `init` or `configure` that failed or was cancelled).
    pub fn config(&self) -> Option<&Config> {
        self.applied.as_ref()
    }

    fn require_configured(&self) -> Result<(), Error<I2C::Error>> {
        if self.applied.is_some() {
            Ok(())
        } else {
            Err(Error::NotConfigured)
        }
    }

    fn info(&self, config: &Config, data_ready_consumed: bool) -> Info {
        Info {
            chip_id: self.chip_id,
            output_period_us: if config.start_mode == StartMode::Suspend {
                0
            } else {
                config.odr.period_us()
            },
            averaging: config.averaging,
            data_ready_consumed,
        }
    }

    /// Bring the chip from any state to the configured one: validate `config`, then soft reset,
    /// wait, chip ID check, OTP read, magnetic reset, and the register plan of
    /// [`Config::plan`] for [`Apply::AfterReset`] (ODR and averaging, axes, interrupt pin,
    /// watchdog, then `NM` when the start mode is normal; no `UPD_OAE`, the chip rejects it in
    /// suspend). `NM` is followed by its minimum wait, the bounded busy poll and the status
    /// checks (see [`plan::Step::PmuCommand`]). An invalid `config` is [`Error::InvalidConfig`] and
    /// nothing is sent.
    ///
    /// The driver is not configured (typed methods return [`Error::NotConfigured`]) from the
    /// start of the call until it returns `Ok`; dropping the future or an error leaves it so. This
    /// holds for an invalid `config` too: the call returns [`Error::InvalidConfig`] having sent
    /// nothing, and a previously valid state is cleared all the same.
    ///
    /// The soft reset clears a latched interrupt configuration with its flag, so `init` reads no
    /// `INT_STATUS`; it is the way out of a chip stalled by leaving a latched configuration with
    /// the flag pending (see [`configure`](Self::configure)).
    /// When the plan ends in normal mode `NM` has completed (busy 15.0 ms, measured on the board,
    /// 2026-11-10; datasheet Table 1, printed page 8: 70 ms typical); the first sample follows
    /// 16.9 ms after `NM` at 400 Hz (23.1 ms at 100 Hz, 33.1 ms at 50 Hz, same measurement), so
    /// wait for the next data-ready edge.
    ///
    /// Soft-reset wait, OTP sequence and magnetic reset follow the vendor `bmm350_init`
    /// (`bmm350.c`); the datasheet only says that the boot ends with 0x80 in `OTP_CMD_REG` (p14)
    /// and that a soft reset is `0xB6` then `0x00` in `CMD` (p22).
    pub async fn init(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<Info, Error<I2C::Error>> {
        self.applied = None;
        self.calibrated = false;
        let plan = config.plan(Apply::AfterReset)?;
        delay.delay_us(3_000).await;
        self.write_register(reg::CMD, reg::CMD_SOFTRESET).await?;
        self.int_latched = false;
        delay.delay_us(24_000).await;
        self.write_register(reg::CMD, reg::CMD_NOP).await?;

        let found = self.read_register(reg::CHIP_ID).await?;
        if found != reg::CHIP_ID_VALUE {
            return Err(Error::WrongId { found });
        }
        self.chip_id = found;

        let mut words = [0u16; reg::OTP_WORDS];
        for (i, w) in words.iter_mut().enumerate() {
            *w = self.read_otp_word(delay, i as u8).await?;
        }
        self.write_register(reg::OTP_CMD_REG, reg::OTP_CMD_PWR_OFF_OTP)
            .await?;
        self.calibration = Calibration::from_otp(&words);
        self.calibrated = true;

        self.reset_magnet(delay).await?;
        self.mode = PowerMode::Suspend;
        let consumed = self.execute(delay, &plan, false).await?;
        self.mode = Self::end_mode(config);
        self.applied = Some(*config);
        Ok(self.info(config, consumed))
    }

    /// Apply `config` to a chip that `init` has already brought up: no reset and no OTP
    /// download. Errors with [`Error::NotConfigured`], sending nothing, when no `init` has
    /// completed the OTP download since the driver was created.
    ///
    /// Reads `PMU_CMD_STATUS_0` first; a chip in normal mode is sent to suspend (the plan of
    /// [`Apply::InPlace`] starts with `SUS`), a chip already in suspend is not sent `SUS` (it is
    /// illegal there: measured on the board, 2026-11-10). The rest of the plan is executed
    /// as in [`init`](Self::init). The driver is not configured from the start of the call until
    /// it returns `Ok`; this holds for an invalid `config` too ([`Error::InvalidConfig`], nothing
    /// sent): a previously valid state is cleared all the same.
    ///
    /// Leaving a latched interrupt: a chip whose `INT_CTRL` is latched with the data-ready flag in
    /// `INT_STATUS` never read stalled when it was reconfigured (no data-ready edges, the sensor
    /// time not advancing, every register reading back as written, [`verify`](Self::verify) `Ok`;
    /// only `init` recovered it; measured on the board, 2026-11-10). So when the driver's `INT_CTRL`
    /// is, or may be, latched with data-ready on, the order is: the status read, `SUS` when the
    /// chip is in normal mode (the conversions stop), then one read of `INT_STATUS` (it clears a
    /// pending flag in latched mode, datasheet 5.5, printed page 20), and only then the first
    /// register write. No flag can be set between that read and the writes because nothing
    /// converts in suspend; a forced measurement still running when `configure` is called can set
    /// one, which is why the application waits for it first (see
    /// [`trigger_forced`](Self::trigger_forced)). That read is one more transaction, and it
    /// consumes a pending latched data-ready flag: [`Info::data_ready_consumed`] says whether one
    /// was set. A pulsed or disabled interrupt costs no read.
    pub async fn configure(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<Info, Error<I2C::Error>> {
        self.applied = None;
        if !self.calibrated {
            return Err(Error::NotConfigured);
        }
        let plan = config.plan(Apply::InPlace)?;
        let status = self.read_register(reg::PMU_CMD_STATUS_0).await?;
        let suspended = status & reg::PMU_STATUS_0_NORMAL == 0;
        self.mode = if suspended {
            PowerMode::Suspend
        } else {
            PowerMode::Normal
        };
        let consumed = self.execute(delay, &plan, suspended).await?;
        self.mode = Self::end_mode(config);
        self.applied = Some(*config);
        Ok(self.info(config, consumed))
    }

    fn end_mode(config: &Config) -> PowerMode {
        if config.start_mode == StartMode::Normal {
            PowerMode::Normal
        } else {
            PowerMode::Suspend
        }
    }

    /// Run a plan. The tracked mode follows each `SUS` and `NM` as it completes. `skip_suspend`
    /// leaves out a leading `SUS` command. Before the first register
    /// write (so after a leading `SUS` has stopped the conversions) a latched interrupt is read
    /// out, see [`drain_latched`](Self::drain_latched). Returns whether a data-ready flag was
    /// consumed by that read.
    async fn execute(
        &mut self,
        delay: &mut impl DelayNs,
        plan: &Plan,
        skip_suspend: bool,
    ) -> Result<bool, Error<I2C::Error>> {
        let mut drained = false;
        let mut consumed = false;
        for (i, step) in plan.iter().enumerate() {
            match *step {
                Step::Write { reg, value } => {
                    if !drained {
                        drained = true;
                        consumed = self.drain_latched().await?;
                    }
                    self.write_register(reg, value).await?
                }
                Step::PmuCommand {
                    cmd,
                    min_wait_us,
                    bound_us,
                } => {
                    if i == 0 && skip_suspend && cmd == reg::PMU_CMD_SUS {
                        continue;
                    }
                    self.pmu_command(delay, cmd, min_wait_us, bound_us).await?;
                    if cmd == reg::PMU_CMD_SUS {
                        self.mode = PowerMode::Suspend;
                    } else if cmd == reg::PMU_CMD_NM {
                        self.mode = PowerMode::Normal;
                    }
                }
                Step::DelayUs(us) => delay.delay_us(us).await,
            }
        }
        Ok(consumed)
    }

    /// One read of `INT_STATUS` when `INT_CTRL` is, or may be, latched with data-ready on; nothing
    /// otherwise. Returns whether the flag was set (and is now cleared).
    ///
    /// Why and when: in latched mode reading `INT_STATUS` clears the flag (datasheet 5.5, printed
    /// page 20; 8.11, p35); the datasheet says nothing about leaving latched mode with the flag
    /// set, but on the board that stalled the chip (2026-11-10), and reading the flag while the
    /// latched configuration was still applied avoided it. The flag is set by every conversion, so
    /// one read helps only when nothing converts any more: callers run it after `SUS` has
    /// completed and before the first write that changes the configuration.
    async fn drain_latched(&mut self) -> Result<bool, Error<I2C::Error>> {
        if !self.int_latched {
            return Ok(false);
        }
        let status = self.read_register(reg::INT_STATUS).await?;
        Ok(status & reg::INT_STATUS_DRDY != 0)
    }

    /// Send `cmd`, wait `min_wait_us`, then read `PMU_CMD_STATUS_0` every [`PMU_POLL_US`] while it
    /// shows busy, until `bound_us` after the write ([`Error::PmuTimeout`]). Then the illegal flag
    /// must be clear, the echo equal to the command, and the normal-mode flag set after `NM` and
    /// clear after `SUS` ([`Error::PmuCommandRejected`]).
    async fn pmu_command(
        &mut self,
        delay: &mut impl DelayNs,
        cmd: u8,
        min_wait_us: u32,
        bound_us: u32,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(reg::PMU_CMD, cmd).await?;
        delay.delay_us(min_wait_us).await;
        let mut elapsed_us = min_wait_us;
        let mut status = self.read_register(reg::PMU_CMD_STATUS_0).await?;
        while status & reg::PMU_STATUS_0_BUSY != 0 {
            if elapsed_us >= bound_us {
                return Err(Error::PmuTimeout { cmd, status });
            }
            delay.delay_us(PMU_POLL_US).await;
            elapsed_us = elapsed_us.saturating_add(PMU_POLL_US);
            status = self.read_register(reg::PMU_CMD_STATUS_0).await?;
        }
        let echo = (status & reg::PMU_STATUS_0_CMD_VALUE_MASK) >> 5;
        let normal = status & reg::PMU_STATUS_0_NORMAL != 0;
        let mode_wrong = (cmd == reg::PMU_CMD_NM && !normal) || (cmd == reg::PMU_CMD_SUS && normal);
        if status & reg::PMU_STATUS_0_ILLEGAL != 0 || echo != expected_echo(cmd) || mode_wrong {
            return Err(Error::PmuCommandRejected { cmd, status });
        }
        Ok(())
    }

    /// Bit reset then flux-guide reset, each with fast recharge (`BR_FAST`, `FGR_FAST`), each
    /// sent through the same wait, busy poll and status check as every other PMU command (the
    /// echo of `BR_FAST` is taken as 7, the vendor's `BMM350_PMU_CMD_STATUS_0_BR_FAST`; see
    /// [`selftest`]). Waits before the first status poll: 4 ms after `BR_FAST` (vendor
    /// `self_test_entry_config`), 8 ms after `FGR_FAST` (datasheet 5.1.5, p17: charging takes
    /// less than 8 ms by design); the poll gives up after 30 ms each ([`Error::PmuTimeout`]).
    /// Measured on the MARV V2 board, 2026-11-11: the pair takes 12.6 ms; `PMU_CMD_STATUS_0`
    /// afterwards is 0xC0 (echo field 6, not busy, no illegal-command flag), and the echoes the
    /// driver checks (7 after `BR_FAST`, 6 after `FGR_FAST`) were accepted; the field after the
    /// reset was within 0.3 uT of the field before; in normal mode it returns the wrong-mode
    /// error and changes nothing. Use it instead of
    /// [`magnetic_reset`](Self::magnetic_reset) when the CRST capacitor was charged recently; after
    /// a long stay in suspend (over about 2 s) or a field shock use the full pair.
    ///
    /// The chip must be in suspend: [`Error::NotConfigured`] before the driver is configured and
    /// [`Error::WrongMode`] when the driver last commanded normal mode, both without bus access.
    /// The validity state and the tracked mode never change; a failed or dropped call leaves the
    /// chip in suspend and the call can be repeated. Do not call it while a forced measurement may
    /// still be running (see [`trigger_forced`](Self::trigger_forced)). Transactions: 4
    /// (write, status read, write, status read) when the chip is no longer busy at the first
    /// poll, one more for each extra poll; every transaction is a single write or `write_read`
    /// of at most 3 bytes.
    pub async fn magnetic_reset_fast(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error<I2C::Error>> {
        self.require_configured()?;
        if self.mode != PowerMode::Suspend {
            return Err(Error::WrongMode { mode: self.mode });
        }
        self.pmu_command(
            delay,
            reg::PMU_CMD_BR_FAST,
            RESET_BR_FAST_MIN_WAIT_US,
            RESET_FAST_BOUND_US,
        )
        .await?;
        self.pmu_command(
            delay,
            reg::PMU_CMD_FGR_FAST,
            RESET_FGR_FAST_MIN_WAIT_US,
            RESET_FAST_BOUND_US,
        )
        .await
    }

    /// Bit reset then flux-guide reset, each with full recharge, checking the command echo in
    /// `PMU_CMD_STATUS_0`. The chip must be in suspend. The datasheet (p17) recommends it after
    /// a long stay in suspend or a field shock; `init` already runs it. Sequence, delays and
    /// echo check follow the vendor `bmm350_magnetic_reset_and_wait` (`bmm350.c`). Returns
    /// [`Error::NotConfigured`] before `init` has completed, and [`Error::WrongMode`] without bus
    /// access when the driver last commanded normal mode. Do not call it while a forced
    /// measurement may still be running (see [`trigger_forced`](Self::trigger_forced)).
    ///
    /// Measured on the MARV V2 board, 2026-11-11: 32.6 ms; `PMU_CMD_STATUS_0` afterwards is 0xA0
    /// (echo field 5, the `FGR` echo the driver checks, as it checks 7 after `BR`; both were
    /// accepted), the field after the reset was within 0.3 uT of the field before, and in normal
    /// mode it returns the wrong-mode error and changes nothing.
    pub async fn magnetic_reset(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error<I2C::Error>> {
        self.require_configured()?;
        if self.mode != PowerMode::Suspend {
            return Err(Error::WrongMode { mode: self.mode });
        }
        self.reset_magnet(delay).await
    }

    /// The built-in self-test of X and Y, positive and negative (module [`selftest`] has the
    /// facts and the sources). Needs a configured driver ([`Error::NotConfigured`] otherwise,
    /// nothing sent). The driver is not configured from the first await until the call has
    /// re-applied the configuration; a failure or a drop leaves it so, and the application calls
    /// [`configure`](Self::configure), which also writes the stimulus bits back to zero (a
    /// dropped call cannot).
    ///
    /// Sequence, every PMU command through the wait, busy poll and status check of the executor
    /// (the forced measurements with `FM_FAST` at averaging 2, `SELF_TEST_FORCED_MIN_WAIT_US`
    /// 6 ms before the first poll, 20 ms bound, then one 12-byte data burst each):
    ///
    /// 1. When the tracked mode is normal: `SUS` (2 ms, bound 20 ms) and, with a latched
    ///    interrupt, one read of `INT_STATUS` as in [`set_power_mode`](Self::set_power_mode).
    /// 2. `PMU_CMD_AGGR_SET` = 0x14 (100 Hz, averaging 2) and `PMU_CMD_AXIS_EN` = 0x07, as the
    ///    vendor does; the applied configuration comes back in step 5.
    /// 3. Magnetic reset, full pair: `BR` 14 ms, echo, `FGR` 18 ms, echo (as `init`).
    /// 4. For X positive, X negative, Y positive, Y negative: baseline `FM_FAST` and data; write
    ///    `TMR_SELFTEST_USER` = 0x0D, 0x0B, 0x15, 0x13; 1 ms; `FM_FAST` and data; write
    ///    `TMR_SELFTEST_USER` = 0x00; 1 ms.
    /// 5. [`configure`](Self::configure) with the configuration that was applied (status read,
    ///    `SUS` only when the chip is not in suspend, the plan with the stimulus register at 0x00,
    ///    `NM` when it ends in normal mode).
    ///
    /// On a bus error or a rejected command after step 1 the stimulus register is written to zero
    /// once more (if that write fails too, its error is dropped) and the first error is returned.
    ///
    /// A test that completes and misses the limit returns `Ok` with
    /// [`SelfTestReport::passed`] false. Entries judge the compensated field, `with - without`
    /// against [`selftest::MIN_DELTA_UT`]. Do not call while a forced measurement may still be
    /// running, and keep the sensor still and away from changing fields: the ambient field enters
    /// the baseline and the stimulus reading alike but is not constant.
    ///
    /// Time, measured on the MARV V2 board, 2026-11-11: 113.7 ms from the default configuration,
    /// 96.1 ms from a configuration that starts in suspend, 115.2 ms with a latched interrupt.
    /// The delays alone add up to 106 ms and 88 ms (88 ms from a suspend start with a suspend
    /// configuration: 32 ms reset, 4 x 14 ms; 3 ms more for the `SUS` of a start in normal mode
    /// and 15 ms more for the `NM` of a configuration that ends in normal mode); the rest is bus
    /// time and polling. Stimulus, same measurement: +174.0 / -173.9 uT on X and +174.1 / -174.1 uT
    /// on Y (spread 0.03 to 0.21 uT) against the 130 uT limit; the positive stimulus raises and
    /// the negative lowers the reading on both axes; no offset is left behind (at most 0.2 uT per
    /// axis over 50 samples afterwards). Transactions: 44 from suspend with a suspend
    /// configuration (2 + 4 + 32 + 6); 49 for the default configuration (3 for `SUS` and two
    /// polls, 2, 4, 32, and 8 for the in-place `configure`). The data registers keep the last
    /// measurement of the test (Y with the negative stimulus) until the next conversion.
    pub async fn self_test(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<SelfTestReport, Error<I2C::Error>> {
        let config = self.applied.take().ok_or(Error::NotConfigured)?;
        let readings = match self.self_test_measure(delay).await {
            Ok(r) => r,
            Err(e) => {
                let _ = self.write_register(reg::TMR_SELFTEST_USER, ST_OFF).await;
                return Err(e);
            }
        };
        self.configure(delay, &config).await?;
        Ok(SelfTestReport::from_readings(readings))
    }

    async fn self_test_measure(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<[(f32, f32); 4], Error<I2C::Error>> {
        if self.mode == PowerMode::Normal {
            self.pmu_command(
                delay,
                reg::PMU_CMD_SUS,
                plan::SUSPEND_MIN_WAIT_US,
                plan::SUSPEND_BOUND_US,
            )
            .await?;
            self.mode = PowerMode::Suspend;
            self.drain_latched().await?;
        }
        self.write_register(reg::PMU_CMD_AGGR_SET, SELF_TEST_AGGR_SET)
            .await?;
        self.write_register(reg::PMU_CMD_AXIS_EN, SELF_TEST_AXIS_EN)
            .await?;
        self.reset_magnet(delay).await?;
        let mut readings = [(0.0f32, 0.0f32); 4];
        for (slot, (axis, polarity)) in readings.iter_mut().zip(selftest::ENTRIES) {
            let without = self.forced_axis(delay, axis).await?;
            self.write_register(reg::TMR_SELFTEST_USER, selftest::stimulus(axis, polarity))
                .await?;
            delay.delay_us(SELF_TEST_SETTLE_US).await;
            let with = self.forced_axis(delay, axis).await?;
            self.write_register(reg::TMR_SELFTEST_USER, ST_OFF).await?;
            delay.delay_us(SELF_TEST_SETTLE_US).await;
            *slot = (without, with);
        }
        Ok(readings)
    }

    /// One `FM_FAST` conversion from suspend, waited for through the executor, then the compensated
    /// field of `axis`.
    async fn forced_axis(
        &mut self,
        delay: &mut impl DelayNs,
        axis: selftest::SelfTestAxis,
    ) -> Result<f32, Error<I2C::Error>> {
        self.pmu_command(
            delay,
            reg::PMU_CMD_FM_FAST,
            SELF_TEST_FORCED_MIN_WAIT_US,
            SELF_TEST_FORCED_BOUND_US,
        )
        .await?;
        let raw = self.read_burst().await?;
        let field = compensate(&self.calibration, &raw);
        Ok(match axis {
            selftest::SelfTestAxis::X => field.x,
            selftest::SelfTestAxis::Y => field.y,
        })
    }

    async fn reset_magnet(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
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

    /// Change the power mode, using the mode the driver last commanded. The mode the device is
    /// already in is a no-op without bus access. `Normal` from suspend sends `NM` and waits for it
    /// as `init` does ([`plan::NM_MIN_WAIT_US`], [`plan::NM_BOUND_US`]); `Suspend` from normal
    /// sends `SUS` and waits for it ([`plan::SUSPEND_MIN_WAIT_US`], [`plan::SUSPEND_BOUND_US`]).
    /// `Forced` and `ForcedFast` only from suspend, as [`trigger_forced`](Self::trigger_forced)
    /// (the command is written and not waited for); from normal mode the chip rejects them, so
    /// the call returns [`Error::WrongMode`] without bus access. Returns [`Error::NotConfigured`]
    /// while the driver is not configured. A call that fails or is dropped after the command was
    /// sent leaves the driver not configured (the tracked mode is then unknown); call
    /// [`configure`](Self::configure).
    ///
    /// Rule: do not change the mode while a forced measurement may still be running. After
    /// [`trigger_forced`](Self::trigger_forced) or
    /// [`trigger_forced_fast`](Self::trigger_forced_fast) the application waits for the documented
    /// time first (the data-ready edge alone is not enough: busy is still set then). If the call is
    /// made anyway the chip ignores the command (`ERR_REG` `pmu_cmd_error`), the call returns
    /// [`Error::PmuCommandRejected`] (the echo is that of the forced command) or
    /// [`Error::PmuTimeout`], the tracked mode is left as it was (suspend) and the driver is not
    /// configured; call [`configure`](Self::configure).
    ///
    /// `Suspend` from normal mode with a latched interrupt (see [`configure`](Self::configure)) is
    /// followed, once `SUS` has completed, by one read of `INT_STATUS` (one more transaction),
    /// which consumes a pending data-ready flag; the flag is not reported by this call.
    pub async fn set_power_mode(
        &mut self,
        delay: &mut impl DelayNs,
        mode: PowerMode,
    ) -> Result<(), Error<I2C::Error>> {
        self.require_configured()?;
        if mode == self.mode {
            return Ok(());
        }
        match mode {
            PowerMode::Forced | PowerMode::ForcedFast => {
                if self.mode != PowerMode::Suspend {
                    return Err(Error::WrongMode { mode: self.mode });
                }
                self.write_register(reg::PMU_CMD, mode.code()).await
            }
            PowerMode::Suspend | PowerMode::Normal => {
                let (min_wait_us, bound_us) = if mode == PowerMode::Normal {
                    (plan::NM_MIN_WAIT_US, plan::NM_BOUND_US)
                } else {
                    (plan::SUSPEND_MIN_WAIT_US, plan::SUSPEND_BOUND_US)
                };
                let config = self.applied.take();
                self.pmu_command(delay, mode.code(), min_wait_us, bound_us)
                    .await?;
                if mode == PowerMode::Suspend {
                    self.drain_latched().await?;
                }
                self.mode = mode;
                self.applied = config;
                Ok(())
            }
        }
    }

    /// Trigger one forced-mode conversion (full recharge, `FM`): one write of `PMU_CMD`. The chip
    /// must be in suspend: when the driver last commanded normal mode this returns
    /// [`Error::WrongMode`] without bus access (the chip rejects `FM` in normal mode: measured on
    /// the board, 2026-11-10). The chip is back in suspend when the conversion is done. Returns
    /// [`Error::NotConfigured`] while the driver is not configured.
    ///
    /// Time from the command to DATA READY, for averaging 1, 2, 4, 8, measured on the board,
    /// 2026-11-10: 14.2, 15.4, 18.9, 25.9 ms. The driver's figures, 15, 17, 20, 28 ms (vendor
    /// `BMM350_SUS_TO_FORCEDMODE_*_DELAY`, see [`timing::forced_measurement_us`]), are all above
    /// them. The averaging of `PMU_CMD_AGGR_SET` applies directly. The driver does not wait: the
    /// application waits for the data-ready edge or for the driver's figure.
    ///
    /// At the data-ready edge the busy flag of `PMU_CMD_STATUS_0` is still set (the normal-mode
    /// and illegal flags are clear; measured on the board, 2026-11-10): data ready comes before
    /// busy clears. Busy times measured earlier on the board were 14.5 ms (averaging 1) and 27 ms
    /// (averaging 8). So a measurement is finished, and the chip free for a command, only after the
    /// driver's figure from the command, not at the edge. Do not call
    /// [`set_power_mode`](Self::set_power_mode), [`configure`](Self::configure) or
    /// [`magnetic_reset`](Self::magnetic_reset) before that: the chip ignores a command written
    /// while one is busy.
    pub async fn trigger_forced(&mut self) -> Result<(), Error<I2C::Error>> {
        self.trigger(reg::PMU_CMD_FM).await
    }

    /// Trigger one forced-mode conversion with fast recharge (`FM_FAST`), under the rules of
    /// [`trigger_forced`](Self::trigger_forced): one write of `PMU_CMD`, [`Error::NotConfigured`]
    /// while the driver is not configured, [`Error::WrongMode`] without bus access when the driver
    /// last commanded normal mode (`FM_FAST` is legal in suspend only: measured on the board,
    /// 2026-11-10).
    ///
    /// Time from the command to DATA READY, for averaging 1, 2, 4, 8, measured on the board,
    /// 2026-11-10: 3.1, 4.3, 7.8, 14.8 ms. The driver's figures, 4, 5, 9, 16 ms (vendor
    /// `BMM350_SUS_TO_FORCEDMODE_FAST_*_DELAY`, see [`timing::forced_measurement_us`]), are all
    /// above them. The averaging of `PMU_CMD_AGGR_SET` applies directly. The driver does not wait:
    /// the application waits for the data-ready edge or for the driver's figure.
    ///
    /// As for [`trigger_forced`](Self::trigger_forced), the busy flag is still set at the
    /// data-ready edge (measured on the board, 2026-11-10; busy 3.4 ms for averaging 1 and 16 ms
    /// for averaging 8 in an earlier measurement), so wait for the driver's figure before the next
    /// command.
    pub async fn trigger_forced_fast(&mut self) -> Result<(), Error<I2C::Error>> {
        self.trigger(reg::PMU_CMD_FM_FAST).await
    }

    async fn trigger(&mut self, cmd: u8) -> Result<(), Error<I2C::Error>> {
        self.require_configured()?;
        if self.mode != PowerMode::Suspend {
            return Err(Error::WrongMode { mode: self.mode });
        }
        self.write_register(reg::PMU_CMD, cmd).await
    }

    /// Read `PMU_CMD_STATUS_0` and `PMU_CMD_STATUS_1` in one transaction (they are adjacent).
    /// `PMU_CMD_STATUS_1` reflects the last command as well as the configured rate: measured on
    /// the MARV V2 board, 2026-11-11, it reads 0x17 in suspend before a reset or after
    /// [`self_test`](Self::self_test), and 0x14 after a magnetic reset.
    /// Works whenever the bus works: it does not need a configured driver and changes no state.
    pub async fn pmu_status(&mut self) -> Result<PmuStatus, Error<I2C::Error>> {
        let mut b = [0u8; 2];
        self.read_registers(reg::PMU_CMD_STATUS_0, &mut b).await?;
        let [status_0, status_1] = b;
        Ok(PmuStatus::from_bytes(status_0, status_1))
    }

    /// Reading `ERR_REG` clears it (8.2, p29): a flag you do not keep is gone. One transaction;
    /// works whenever the bus works: it does not need a configured driver and changes no state.
    pub async fn error_status(&mut self) -> Result<ErrorStatus, Error<I2C::Error>> {
        Ok(ErrorStatus::from_bits(
            self.read_register(reg::ERR_REG).await?,
        ))
    }

    /// Read back what the applied configuration owns and compare it with what the configuration
    /// puts there (the expected values are the register images the plan writes, not a second
    /// table).
    ///
    /// [`Error::NotConfigured`] while the driver is not configured, with no bus access. Otherwise
    /// one read transaction per owned entry, in this order: `PMU_CMD_AGGR_SET`, `PMU_CMD_AXIS_EN`,
    /// `INT_CTRL`, `I2C_WDT_SET`, `TMR_SELFTEST_USER` (8.33, p42: all stimulus bits must be zero)
    /// (each under a mask that leaves out the reserved bits), then the
    /// normal-mode flag of `PMU_CMD_STATUS_0` against the mode the driver tracks, then, only when
    /// the tracked mode is normal, the effective rate and averaging of `PMU_CMD_STATUS_1` against
    /// the applied ones. That is 6 transactions when the tracked mode is suspend and 7 when it is
    /// normal, each a single `write_read` of at most 3 bytes, and no await other than those, when
    /// everything matches.
    ///
    /// An entry that differs under its mask is read once more; only if it differs again does the
    /// call return [`Error::ConfigMismatch`] with the register, the mask, the expected value and
    /// the whole byte found (so a mismatch costs one extra transaction and a one-off glitch costs
    /// one and still returns `Ok`).
    ///
    /// A set illegal flag in `PMU_CMD_STATUS_0` and a set bit in `ERR_REG` are not mismatches (they
    /// are not read; [`pmu_status`](Self::pmu_status) and [`error_status`](Self::error_status)
    /// report them). Nothing is written, and the validity state, the tracked mode and the stored
    /// [`Config`] are never changed: the application decides what follows a mismatch
    /// ([`configure`](Self::configure), keep reading, or switch sensors). Cancelling between
    /// transactions is safe; call it again.
    ///
    /// It compares configuration, not liveness: a chip stalled by leaving a latched interrupt with
    /// its flag pending passed it (every register read back as written, the status showed normal
    /// mode and the right rate, 2026-11-10). The liveness check is that data-ready edges keep
    /// coming, or that [`sensor_time`](Self::sensor_time) advances. `INT_CTRL` bit 3 (the pin's
    /// output enable, which the datasheet calls reserved on some samples) is not compared.
    ///
    /// Confirmed on the real chip (measured on the board, 2026-11-10): 200 calls while streaming at
    /// 400 Hz and 20 calls right after a forced trigger all returned `Ok`, so it is safe to call
    /// while streaming and right after a forced trigger.
    pub async fn verify(&mut self) -> Result<(), Error<I2C::Error>> {
        let Some(config) = self.applied else {
            return Err(Error::NotConfigured);
        };
        let owned = config.owned_registers(self.mode);
        for o in owned.entries() {
            let found = self.read_register(o.reg).await?;
            if (found ^ o.expected) & o.mask == 0 {
                continue;
            }
            let found = self.read_register(o.reg).await?;
            if (found ^ o.expected) & o.mask != 0 {
                return Err(Error::ConfigMismatch(Mismatch {
                    register: o.reg,
                    mask: o.mask,
                    expected: o.expected,
                    found,
                }));
            }
        }
        Ok(())
    }

    /// Bring the bus back in step after a cancelled or failed transfer, then check the chip.
    ///
    /// 1. Wait 2 ms through `delay`, with no bus traffic.
    /// 2. One read of `CHIP_ID`, the result discarded (a bus error is still returned).
    /// 3. The identity check done twice, two separate reads that must both return 0x33
    ///    ([`Error::WrongId`] otherwise).
    /// 4. [`verify`](Self::verify) when configured, [`Error::NotConfigured`] otherwise.
    ///
    /// Transactions: 9 when configured, tracking suspend, and everything matches; 10 tracking
    /// normal; 3 when not configured. It writes nothing to the device and never changes the
    /// validity state, the tracked mode or the stored [`Config`]. Cancelling between transactions
    /// is safe; call it again.
    ///
    /// On embassy-rp 0.10 an I2C transfer in flight must not be cancelled: a cancelled read makes
    /// later reads return wrong data with no error until the bus has been quiet for 2 ms and one
    /// read has been discarded, which is what steps 1 and 2 do. Put a timeout around the
    /// data-ready wait, not around the bus call.
    ///
    /// What it cannot do: a data line held low by the device needs a bus clear (pulses on SCL
    /// while SDA is low, START and STOP, rebuild the bus) by the firmware's bus owner; a driver
    /// that only holds an `I2c` cannot.
    pub async fn recover(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        delay.delay_us(RECOVER_QUIET_US).await;
        self.read_register(reg::CHIP_ID).await?;
        for _ in 0..2 {
            let found = self.read_register(reg::CHIP_ID).await?;
            if found != reg::CHIP_ID_VALUE {
                return Err(Error::WrongId { found });
            }
        }
        self.verify().await
    }

    /// Read `INT_STATUS`. Clears the flag. Returns [`Error::NotConfigured`] while the driver is
    /// not configured.
    pub async fn interrupt_status(&mut self) -> Result<InterruptStatus, Error<I2C::Error>> {
        self.require_configured()?;
        Ok(InterruptStatus::from_bits(
            self.read_register(reg::INT_STATUS).await?,
        ))
    }

    /// Read the sensor-time counter (`SENSORTIME_XLSB` to `SENSORTIME_MSB`, 8.26 to 8.28, p40) in
    /// one burst: the raw 24-bit value in ticks of 1/[`timing::SENSOR_TIME_TICKS_PER_SECOND`] s
    /// (39.0625 us), from 0 to `0x00FF_FFFF`; a burst read from a sensor-time address halts the
    /// update, so the three bytes are consistent (datasheet p18). In normal mode it is the time of
    /// the LAST SAMPLE and moves in steps of one sample period (256 ticks at 100 Hz, 64 at 400 Hz;
    /// measured on the MARV V2 board, 2026-11-11); in suspend it stands still, and it did not
    /// advance on the stalled chip (2026-11-10). So two equal readings can mean "no new sample"
    /// as well as "stalled": compare readings at least a sample period apart. It runs from the
    /// sensor's oscillator and drifts against the host clock (0.7 % on the measured unit).
    /// Subtract with [`timing::sensor_ticks_elapsed`] or [`timing::sensor_elapsed_us`], which
    /// handle the wrap at 24 bits (about 655 s). Works whenever the bus works: it does not need a
    /// configured driver and changes no state.
    pub async fn sensor_time(&mut self) -> Result<u32, Error<I2C::Error>> {
        let mut b = [0u8; 3];
        self.read_registers(reg::SENSORTIME_XLSB, &mut b).await?;
        let [xlsb, lsb, msb] = b;
        Ok(u32::from_le_bytes([xlsb, lsb, msb, 0]))
    }

    /// Read X, Y, Z and temperature in one bus transaction (dummy bytes included). The chip
    /// holds its data registers still during a burst read that starts at `MAG_X_XLSB`. Returns
    /// [`Error::NotConfigured`] while the driver is not configured.
    pub async fn read(&mut self) -> Result<RawSample, Error<I2C::Error>> {
        self.require_configured()?;
        self.read_burst().await
    }

    async fn read_burst(&mut self) -> Result<RawSample, Error<I2C::Error>> {
        let mut buf = [0u8; reg::DUMMY_BYTES + 12];
        self.i2c
            .write_read(self.address, &[reg::MAG_X_XLSB], &mut buf)
            .await
            .map_err(Error::Bus)?;
        let mut data = [0u8; 12];
        data.copy_from_slice(&buf[reg::DUMMY_BYTES..]);
        Ok(RawSample::decode(&data))
    }

    /// Read X, Y, Z, temperature and the sensor time in ONE bus transaction: `MAG_X_XLSB` (0x31)
    /// to `SENSORTIME_MSB` (0x3F) are adjacent, 15 register bytes, so 17 bytes with the two dummy
    /// bytes (datasheet p27, p37-40). The data registers stay still during the burst and the
    /// sensor time comes with them, so the time belongs to the sample (contract C3). The time is
    /// that of the last sample, in steps of one sample period, and stands still in suspend; see
    /// [`sensor_time`](Self::sensor_time) for the units, the drift against the host clock and what
    /// two equal readings mean. Convert ticks with [`timing::sensor_elapsed_us`], or with
    /// [`timing::sensor_elapsed_us_with_period`] and a tick period calibrated against the host
    /// clock. Cost, measured on the MARV V2 board, 2026-11-11: 539 us median against 465 us for
    /// [`read`](Self::read) (+75 us), 400 reads, no errors.
    ///
    /// Length: 17 bytes is more than the 15 bytes that are fully queued before the first await on
    /// embassy-rp (contract A4e): this read is NOT fully queued before the first await; a cancel
    /// during it can leave the transfer in flight, see [`recover`](Self::recover). The shorter
    /// [`read`](Self::read) (14 bytes) is fully queued and stays the choice when the time is not
    /// needed. Returns [`Error::NotConfigured`] while the driver is not configured.
    pub async fn read_with_time(&mut self) -> Result<TimedSample, Error<I2C::Error>> {
        self.require_configured()?;
        let mut buf = [0u8; reg::DUMMY_BYTES + 15];
        self.i2c
            .write_read(self.address, &[reg::MAG_X_XLSB], &mut buf)
            .await
            .map_err(Error::Bus)?;
        let mut data = [0u8; 15];
        data.copy_from_slice(&buf[reg::DUMMY_BYTES..]);
        let (sample, sensor_time) = RawSample::decode_with_time(&data);
        Ok(TimedSample {
            sample,
            sensor_time,
        })
    }

    /// [`read`](Self::read) then [`compensate`].
    pub async fn read_field(&mut self) -> Result<Field, Error<I2C::Error>> {
        let raw = self.read().await?;
        Ok(compensate(&self.calibration, &raw))
    }

    /// Read one register. Raw access: outside the validity state, it works whether or not the
    /// driver is configured.
    pub async fn read_register(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut b = [0u8; 1];
        self.read_registers(reg, &mut b).await?;
        Ok(b[0])
    }

    /// Write one register. Raw access, outside the validity state; it can leave the chip in a
    /// state the applied configuration does not describe. A write of `INT_CTRL` is noted for the
    /// latched-interrupt readout of [`configure`](Self::configure); a raw soft reset is not.
    pub async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        let latched = reg == reg::INT_CTRL
            && value & (reg::INT_CTRL_DRDY_EN | reg::INT_CTRL_LATCHED)
                == reg::INT_CTRL_DRDY_EN | reg::INT_CTRL_LATCHED;
        if reg == reg::INT_CTRL {
            self.int_latched |= latched;
        }
        let result = self
            .i2c
            .write(self.address, &[reg, value])
            .await
            .map_err(Error::Bus);
        if reg == reg::INT_CTRL && result.is_ok() {
            self.int_latched = latched;
        }
        result
    }

    /// Read `out.len()` bytes from consecutive registers starting at `reg`, without the dummy
    /// bytes. Reads longer than 32 bytes are split into several transactions. Raw access, outside
    /// the validity state.
    pub async fn read_registers(
        &mut self,
        reg: u8,
        out: &mut [u8],
    ) -> Result<(), Error<I2C::Error>> {
        let mut reg = reg;
        for chunk in out.chunks_mut(READ_CHUNK) {
            let mut buf = [0u8; reg::DUMMY_BYTES + READ_CHUNK];
            let n = (reg::DUMMY_BYTES + chunk.len()).min(buf.len());
            let (window, _) = buf.split_at_mut(n);
            self.i2c
                .write_read(self.address, &[reg], window)
                .await
                .map_err(Error::Bus)?;
            chunk.copy_from_slice(window.get(reg::DUMMY_BYTES..).unwrap_or(&[]));
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
