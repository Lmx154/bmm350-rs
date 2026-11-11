//! The register write plan: the ordered steps that configure the device from a [`Config`].
//!
//! The plan is data. It is built without a bus and without allocation, and a driver only executes
//! it: [`Step::Write`] is one register write, [`Step::PmuCommand`] a `PMU_CMD` write followed by the
//! wait the device needs before the next command, [`Step::DelayUs`] a plain wait.
//!
//! # Order
//!
//! Rules measured on the board, 2026-11-10: in suspend `NM`, `FM` and `FM_FAST` are legal and
//! `SUS` and `UPD_OAE` are illegal; in normal mode `SUS` and `UPD_OAE` are legal and the others are
//! illegal. `PMU_CMD_AGGR_SET` written in suspend is taken over by the following `NM`, so the plan
//! never sends `UPD_OAE`: in suspend the chip rejects it, and in normal mode it is not needed
//! because the plan goes through suspend first.
//!
//! 1. In place only: `SUS`, so that the changes below are made in suspend. The executor leaves it
//!    out when the status read before the plan shows suspend.
//! 2. `PMU_CMD_AGGR_SET`, `PMU_CMD_AXIS_EN`, `INT_CTRL` (always written), `I2C_WDT_SET` (in place
//!    always, after reset when it is not at its reset value), then `TMR_SELFTEST_USER` = 0x00
//!    (always: the plans own the self-test register and leave the stimulus off, 8.33, p42).
//! 3. `NM` when the configuration ends in normal mode.
//!
//! The plan holds no read. The executor of the driver adds one: when `INT_CTRL` on the chip is, or
//! may be, latched with data-ready on, it reads `INT_STATUS` once between step 1 and the first
//! write of step 2, so that no latched flag is pending when the configuration changes (see
//! `Bmm350::configure`).

use crate::config::{Config, ConfigError, StartMode};
use crate::reg;

/// `NM`: no status poll before this, 15 ms (busy for 15.0 ms, measured on the board, 2026-11-10).
/// The datasheet's typical start-up time from suspend to normal mode is 70 ms (Table 1, printed
/// page 8); the vendor driver waits 38 ms (`BMM350_SUSPEND_TO_NORMAL_DELAY`).
pub const NM_MIN_WAIT_US: u32 = 15_000;
/// `NM`: the busy poll gives up after 50 ms in total, more than three times the 15.0 ms measured
/// on the board, 2026-11-10 (the datasheet gives no figure but the 70 ms typical start-up time).
pub const NM_BOUND_US: u32 = 50_000;
/// `SUS`: no status poll before this, 2 ms (busy 1.8 to 4 ms from normal mode, measured on the
/// board, 2026-11-10; the datasheet gives none; the vendor driver waits 6 ms,
/// `BMM350_GOTO_SUSPEND_DELAY`).
pub const SUSPEND_MIN_WAIT_US: u32 = 2_000;
/// `SUS`: the busy poll gives up after 20 ms in total, five times the 4 ms worst case measured on
/// the board, 2026-11-10.
pub const SUSPEND_BOUND_US: u32 = 20_000;

/// Most steps a plan can hold. The longest plan, in place and ending in normal mode, has 7.
pub const MAX_STEPS: usize = 8;

/// One step of a plan.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Step {
    /// Write `value` to register `reg`.
    Write {
        /// Register address.
        reg: u8,
        /// The byte to write.
        value: u8,
    },
    /// Write `cmd` to `PMU_CMD`, wait `min_wait_us`, then poll `PMU_CMD_STATUS_0` until busy
    /// clears or `bound_us` (counted from the write) is reached, then check the illegal flag, the
    /// echo and the normal-mode flag.
    PmuCommand {
        /// The `PMU_CMD` code.
        cmd: u8,
        /// Minimum time before the first status poll, in microseconds.
        min_wait_us: u32,
        /// Longest time to wait for busy to clear, in microseconds from the write.
        bound_us: u32,
    },
    /// Wait this many microseconds.
    DelayUs(u32),
}

/// Where the plan starts from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Apply {
    /// Right after a soft reset: the registers are at their reset values (`CMD` soft reset is a
    /// power-on reset, 8.35, p43), the chip is in suspend, so `I2C_WDT_SET` is not written when its
    /// image equals the reset value; `PMU_CMD_AGGR_SET`, `PMU_CMD_AXIS_EN` and `INT_CTRL` are
    /// always written.
    AfterReset,
    /// On a running device in an unknown state, without a reset: `SUS` first, every register
    /// written.
    InPlace,
}

/// A register write plan. Iterate it with [`Plan::steps`] or `for step in &plan`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Plan {
    steps: [Step; MAX_STEPS],
    len: usize,
}

impl Plan {
    const fn empty() -> Self {
        Self {
            steps: [Step::DelayUs(0); MAX_STEPS],
            len: 0,
        }
    }

    /// The steps, in order.
    pub fn steps(&self) -> &[Step] {
        self.steps.get(..self.len).unwrap_or(&[])
    }

    /// An iterator over the steps.
    pub fn iter(&self) -> core::slice::Iter<'_, Step> {
        self.steps().iter()
    }

    /// Number of steps.
    pub const fn len(&self) -> usize {
        self.len
    }

    /// True when there is no step.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    fn push(&mut self, step: Step) -> Result<(), ConfigError> {
        let slot = self
            .steps
            .get_mut(self.len)
            .ok_or(ConfigError::PlanOverflow)?;
        *slot = step;
        self.len += 1;
        Ok(())
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), ConfigError> {
        self.push(Step::Write { reg, value })
    }

    fn pmu(&mut self, cmd: u8, min_wait_us: u32, bound_us: u32) -> Result<(), ConfigError> {
        self.push(Step::PmuCommand {
            cmd,
            min_wait_us,
            bound_us,
        })
    }
}

impl<'a> IntoIterator for &'a Plan {
    type Item = &'a Step;
    type IntoIter = core::slice::Iter<'a, Step>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl Config {
    /// Validates the configuration and builds the plan for it. A plan that would not fit in
    /// [`MAX_STEPS`] is [`ConfigError::PlanOverflow`], never a plan with a step dropped.
    ///
    /// See the module documentation for the order and the rules it follows.
    pub fn plan(&self, apply: Apply) -> Result<Plan, ConfigError> {
        self.validate()?;
        let mut p = Plan::empty();
        if apply == Apply::InPlace {
            p.pmu(reg::PMU_CMD_SUS, SUSPEND_MIN_WAIT_US, SUSPEND_BOUND_US)?;
        }
        p.write(reg::PMU_CMD_AGGR_SET, self.pmu_cmd_aggr_set())?;
        p.write(reg::PMU_CMD_AXIS_EN, self.pmu_cmd_axis_en())?;
        p.write(reg::INT_CTRL, self.int_ctrl())?;
        let wdt = self.i2c_wdt_set();
        if apply == Apply::InPlace || wdt != reg::I2C_WDT_SET_RESET {
            p.write(reg::I2C_WDT_SET, wdt)?;
        }
        p.write(reg::TMR_SELFTEST_USER, reg::TMR_SELFTEST_USER_RESET)?;
        if self.start_mode == StartMode::Normal {
            p.pmu(reg::PMU_CMD_NM, NM_MIN_WAIT_US, NM_BOUND_US)?;
        }
        Ok(p)
    }
}

#[cfg(test)]
mod overflow_tests {
    use super::*;

    #[test]
    fn plan_push_past_capacity_is_an_error_and_keeps_the_steps() {
        let mut p = Plan::empty();
        for i in 0..MAX_STEPS {
            assert_eq!(p.write(reg::PMU_CMD_AGGR_SET, i as u8), Ok(()));
        }
        assert_eq!(p.len(), MAX_STEPS);
        assert_eq!(
            p.write(reg::PMU_CMD_AXIS_EN, 0xFF),
            Err(ConfigError::PlanOverflow)
        );
        assert_eq!(p.push(Step::DelayUs(1)), Err(ConfigError::PlanOverflow));
        assert_eq!(
            p.pmu(reg::PMU_CMD_SUS, 1, 2),
            Err(ConfigError::PlanOverflow)
        );
        assert_eq!(p.len(), MAX_STEPS);
        assert_eq!(
            p.steps().last(),
            Some(&Step::Write {
                reg: reg::PMU_CMD_AGGR_SET,
                value: (MAX_STEPS - 1) as u8
            })
        );
    }
}
