//! Register images: the value of each configuration register as a pure function of a [`Config`].
//!
//! Every image sets only the bits of fields that `Config` owns; the reserved bits keep their reset
//! value. [`Config::validate`] decides whether the combination is allowed; the images are total
//! functions that do not check it (in particular `pmu_cmd_aggr_set` does not lower an averaging
//! the rate does not allow).

use crate::config::{
    Config, I2cWatchdog, InterruptDrive, InterruptMode, InterruptPolarity, PowerMode,
};
use crate::reg;

const fn bit(cond: bool, mask: u8) -> u8 {
    if cond { mask } else { 0 }
}

impl Config {
    /// `PMU_CMD_AGGR_SET` (8.4, p30): `odr` in bits 3:0, `avg` in bits 5:4.
    pub const fn pmu_cmd_aggr_set(&self) -> u8 {
        (reg::PMU_CMD_AGGR_SET_RESET & reg::PMU_CMD_AGGR_SET_RESERVED)
            | (self.averaging.code() << reg::PMU_CMD_AGGR_SET_AVG_SHIFT)
            | self.odr.code()
    }

    /// `PMU_CMD_AXIS_EN` (8.5, p31): `en_x`, `en_y`, `en_z` in bits 0 to 2.
    pub const fn pmu_cmd_axis_en(&self) -> u8 {
        (reg::PMU_CMD_AXIS_EN_RESET & reg::PMU_CMD_AXIS_EN_RESERVED)
            | bit(self.axes.x, reg::AXIS_EN_X)
            | bit(self.axes.y, reg::AXIS_EN_Y)
            | bit(self.axes.z, reg::AXIS_EN_Z)
    }

    /// `INT_CTRL` (8.11, p35). With the data-ready interrupt on: `drdy_data_reg_en`,
    /// `int_output_en` (set as `init` always did), and the mode, polarity and drive. With it off
    /// the reset value, whatever the other pin settings, so that an earlier program's setting
    /// cannot survive.
    pub const fn int_ctrl(&self) -> u8 {
        let pin = &self.interrupt;
        if !pin.data_ready {
            return reg::INT_CTRL_RESET;
        }
        (reg::INT_CTRL_RESET & reg::INT_CTRL_RESERVED)
            | reg::INT_CTRL_DRDY_EN
            | reg::INT_CTRL_OUTPUT_EN
            | bit(
                matches!(pin.mode, InterruptMode::Latched),
                reg::INT_CTRL_LATCHED,
            )
            | bit(
                matches!(pin.polarity, InterruptPolarity::ActiveHigh),
                reg::INT_CTRL_ACTIVE_HIGH,
            )
            | bit(
                matches!(pin.drive, InterruptDrive::PushPull),
                reg::INT_CTRL_PUSH_PULL,
            )
    }

    /// `TMR_SELFTEST_USER` (8.33, p42): every stimulus bit off, the generator off.
    pub const fn tmr_selftest_user(&self) -> u8 {
        reg::TMR_SELFTEST_USER_RESET
    }

    /// `I2C_WDT_SET` (8.10, p34-35): `i2c_wdt_en` bit 0, `i2c_wdt_sel` bit 1 (1 is 40.96 ms).
    pub const fn i2c_wdt_set(&self) -> u8 {
        (reg::I2C_WDT_SET_RESET & reg::I2C_WDT_SET_RESERVED)
            | match self.watchdog {
                I2cWatchdog::Off => 0,
                I2cWatchdog::Ms1_28 => reg::I2C_WDT_EN,
                I2cWatchdog::Ms40_96 => reg::I2C_WDT_EN | reg::I2C_WDT_SEL_LONG,
            }
    }
}

/// One thing the applied configuration owns: the bits `mask` of register `reg` hold `expected`
/// (already ANDed with `mask`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Owned {
    pub(crate) reg: u8,
    pub(crate) mask: u8,
    pub(crate) expected: u8,
}

/// Most entries [`Config::owned_registers`] lists.
pub(crate) const OWNED_MAX: usize = 7;

/// The owned entries in the order `verify` reads them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct OwnedList {
    entries: [Owned; OWNED_MAX],
    len: usize,
}

impl OwnedList {
    pub(crate) fn entries(&self) -> &[Owned] {
        self.entries.get(..self.len).unwrap_or(&[])
    }
}

impl Config {
    /// What the applied configuration owns when the driver tracks `mode` (`Normal`, anything else
    /// is taken as suspend), in the order `verify` reads it. The expected values are the images
    /// above, the ones the plan writes; there is no second table.
    ///
    /// 1. `PMU_CMD_AGGR_SET` under 0x3F; bits 7:6 are reserved (8.4, p30).
    /// 2. `PMU_CMD_AXIS_EN` under 0x07; bits 7:3 are reserved (8.5, p31).
    /// 3. `INT_CTRL` under 0x87; bits 6:4 are reserved (8.11, p35), and bit 3 `int_output_en` is
    ///    reserved on A samples (8.11, p35), where the datasheet does not say whether it reads
    ///    back as written, so it is left out.
    /// 4. `I2C_WDT_SET` under 0x03; bits 7:2 are reserved (8.10, p34).
    /// 5. `TMR_SELFTEST_USER` under 0x1F, all zero: the stimulus is off (8.33, p42; bits 7:5
    ///    reserved).
    /// 6. `PMU_CMD_STATUS_0` under 0x08, the normal-mode flag (8.7, p33): set when `mode` is
    ///    normal, clear otherwise. Busy, the overwrite flags, the illegal flag and the echo are
    ///    left out: they report commands, not the configuration.
    /// 7. Only when `mode` is normal: `PMU_CMD_STATUS_1` under 0x3F, the effective rate and
    ///    averaging (8.8, p34; bits 7:6 reserved) against the applied ones. In suspend they follow
    ///    the register only once `NM`, a forced command or `UPD_OAE` completed (measured on the
    ///    board, 2026-11-10), so they are not compared.
    pub(crate) const fn owned_registers(&self, mode: PowerMode) -> OwnedList {
        const fn o(reg: u8, mask: u8, image: u8) -> Owned {
            Owned {
                reg,
                mask,
                expected: image & mask,
            }
        }
        let normal = matches!(mode, PowerMode::Normal);
        let aggr = self.pmu_cmd_aggr_set();
        let mut list = OwnedList {
            entries: [
                o(reg::PMU_CMD_AGGR_SET, 0x3F, aggr),
                o(reg::PMU_CMD_AXIS_EN, 0x07, self.pmu_cmd_axis_en()),
                o(reg::INT_CTRL, 0x87, self.int_ctrl()),
                o(reg::I2C_WDT_SET, 0x03, self.i2c_wdt_set()),
                o(reg::TMR_SELFTEST_USER, 0x1F, self.tmr_selftest_user()),
                o(
                    reg::PMU_CMD_STATUS_0,
                    reg::PMU_STATUS_0_NORMAL,
                    bit(normal, reg::PMU_STATUS_0_NORMAL),
                ),
                o(reg::PMU_CMD_STATUS_1, 0x3F, aggr),
            ],
            len: 6,
        };
        if normal {
            list.len = 7;
        }
        list
    }
}
