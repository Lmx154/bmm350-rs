//! Typed configuration and the `const fn` encoders that turn it into register values.

use crate::reg;

/// Output data rate in normal mode, the `odr` field of `PMU_CMD_AGGR_SET` (datasheet p30).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputDataRate {
    /// 400 Hz, code 0x02.
    Hz400,
    /// 200 Hz, code 0x03.
    Hz200,
    /// 100 Hz, code 0x04.
    Hz100,
    /// 50 Hz, code 0x05.
    Hz50,
    /// 25 Hz, code 0x06.
    Hz25,
    /// 12.5 Hz, code 0x07.
    Hz12_5,
    /// 6.25 Hz, code 0x08.
    Hz6_25,
    /// 3.125 Hz, code 0x09.
    Hz3_125,
    /// 1.5625 Hz, code 0x0A.
    Hz1_5625,
}

impl OutputDataRate {
    /// Register code.
    pub const fn code(self) -> u8 {
        match self {
            Self::Hz400 => reg::ODR_400HZ,
            Self::Hz200 => reg::ODR_200HZ,
            Self::Hz100 => reg::ODR_100HZ,
            Self::Hz50 => reg::ODR_50HZ,
            Self::Hz25 => reg::ODR_25HZ,
            Self::Hz12_5 => reg::ODR_12_5HZ,
            Self::Hz6_25 => reg::ODR_6_25HZ,
            Self::Hz3_125 => reg::ODR_3_125HZ,
            Self::Hz1_5625 => reg::ODR_1_5625HZ,
        }
    }

    /// Nominal rate in hertz.
    pub const fn hertz(self) -> f32 {
        match self {
            Self::Hz400 => 400.0,
            Self::Hz200 => 200.0,
            Self::Hz100 => 100.0,
            Self::Hz50 => 50.0,
            Self::Hz25 => 25.0,
            Self::Hz12_5 => 12.5,
            Self::Hz6_25 => 6.25,
            Self::Hz3_125 => 3.125,
            Self::Hz1_5625 => 1.5625,
        }
    }

    /// Nominal output period in normal mode in microseconds: the reciprocal of the rate in
    /// [`hertz`](Self::hertz) (the rates are those of 8.4, p30). The chip times it with an RC
    /// oscillator, so the real period has that oscillator's accuracy (5.1.3, p14).
    pub const fn period_us(self) -> u32 {
        match self {
            Self::Hz400 => 2_500,
            Self::Hz200 => 5_000,
            Self::Hz100 => 10_000,
            Self::Hz50 => 20_000,
            Self::Hz25 => 40_000,
            Self::Hz12_5 => 80_000,
            Self::Hz6_25 => 160_000,
            Self::Hz3_125 => 320_000,
            Self::Hz1_5625 => 640_000,
        }
    }

    /// Highest averaging this rate allows (5.1.3, Table 5, p15). Table 5 prints the rates 400 to
    /// 12.5 Hz and 1.5625 Hz and elides the two between; they are taken as allowing the same
    /// averaging as their neighbours (all four).
    pub const fn max_averaging(self) -> Averaging {
        match self {
            Self::Hz400 => Averaging::Avg1,
            Self::Hz200 => Averaging::Avg2,
            Self::Hz100 => Averaging::Avg4,
            _ => Averaging::Avg8,
        }
    }
}

/// Number of samples the chip averages per output sample, the `avg` field of `PMU_CMD_AGGR_SET`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Averaging {
    /// No averaging, code 0.
    Avg1,
    /// 2 samples, code 1.
    Avg2,
    /// 4 samples, code 2.
    Avg4,
    /// 8 samples, code 3.
    Avg8,
}

impl Averaging {
    /// Register code.
    pub const fn code(self) -> u8 {
        match self {
            Self::Avg1 => reg::AVG_1,
            Self::Avg2 => reg::AVG_2,
            Self::Avg4 => reg::AVG_4,
            Self::Avg8 => reg::AVG_8,
        }
    }
}

/// Power mode, the `pmu_cmd` codes that select an operating state (datasheet p32).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PowerMode {
    /// Suspend: settings kept, no conversions.
    Suspend,
    /// Normal: continuous conversions at the configured rate.
    Normal,
    /// Forced: one conversion, then back to suspend. Only reachable from suspend. Full CRST
    /// recharge; usable at any rate except 200 Hz.
    Forced,
    /// Forced with fast CRST recharge: for trigger rates of 25 Hz and higher.
    ForcedFast,
}

impl PowerMode {
    /// `PMU_CMD` code.
    pub const fn code(self) -> u8 {
        match self {
            Self::Suspend => reg::PMU_CMD_SUS,
            Self::Normal => reg::PMU_CMD_NM,
            Self::Forced => reg::PMU_CMD_FM,
            Self::ForcedFast => reg::PMU_CMD_FM_FAST,
        }
    }
}

/// Which axes are converted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Axes {
    /// Convert X.
    pub x: bool,
    /// Convert Y.
    pub y: bool,
    /// Convert Z.
    pub z: bool,
}

impl Axes {
    /// All three axes.
    pub const ALL: Axes = Axes {
        x: true,
        y: true,
        z: true,
    };

    /// Value of `PMU_CMD_AXIS_EN`.
    pub const fn bits(self) -> u8 {
        (self.x as u8) | ((self.y as u8) << 1) | ((self.z as u8) << 2)
    }

    /// True if at least one axis is enabled; the chip needs one.
    pub const fn any(self) -> bool {
        self.x || self.y || self.z
    }
}

/// Power mode `init` leaves the chip in. Forced measurements are triggers, not a configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StartMode {
    /// Suspend: settings kept, no conversions (5.1.2, p14).
    Suspend,
    /// Normal: continuous conversions at the configured rate (5.1.3, p14).
    Normal,
}

impl StartMode {
    /// `PMU_CMD` code that enters this mode (8.6, p32).
    pub const fn code(self) -> u8 {
        match self {
            Self::Suspend => reg::PMU_CMD_SUS,
            Self::Normal => reg::PMU_CMD_NM,
        }
    }
}

/// Interrupt output mode, `INT_CTRL.int_mode` (8.11, p35; 5.5.4, p20).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptMode {
    /// A pulse per sample; `INT_STATUS` clears itself after 1.25 ms (5.5, p20).
    Pulsed,
    /// The pin stays asserted until `INT_STATUS` is read.
    Latched,
}

/// Interrupt output driver, `INT_CTRL.int_od` (8.11, p35; 5.5.3, p20).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptDrive {
    /// Push-pull (CMOS).
    PushPull,
    /// Open-drain.
    OpenDrain,
}

/// Interrupt output level, `INT_CTRL.int_pol` (8.11, p35; 5.5.2, p20).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptPolarity {
    /// Asserted high.
    ActiveHigh,
    /// Asserted low.
    ActiveLow,
}

/// The INT pin: whether data-ready drives it and how.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct InterruptPin {
    /// Data-ready interrupt on the pin and in `INT_STATUS`. When false `INT_CTRL` is the reset
    /// value (interrupt off) and the other fields have no effect.
    pub data_ready: bool,
    /// Pulsed or latched.
    pub mode: InterruptMode,
    /// Push-pull or open-drain.
    pub drive: InterruptDrive,
    /// Active high or active low.
    pub polarity: InterruptPolarity,
}

impl InterruptPin {
    /// Data-ready on, pulsed, push-pull, active high.
    pub const fn new() -> Self {
        Self {
            data_ready: true,
            mode: InterruptMode::Pulsed,
            drive: InterruptDrive::PushPull,
            polarity: InterruptPolarity::ActiveHigh,
        }
    }

    /// Set [`data_ready`](Self::data_ready).
    pub const fn with_data_ready(mut self, v: bool) -> Self {
        self.data_ready = v;
        self
    }

    /// Set [`mode`](Self::mode).
    pub const fn with_mode(mut self, v: InterruptMode) -> Self {
        self.mode = v;
        self
    }

    /// Set [`drive`](Self::drive).
    pub const fn with_drive(mut self, v: InterruptDrive) -> Self {
        self.drive = v;
        self
    }

    /// Set [`polarity`](Self::polarity).
    pub const fn with_polarity(mut self, v: InterruptPolarity) -> Self {
        self.polarity = v;
        self
    }
}

impl Default for InterruptPin {
    fn default() -> Self {
        Self::new()
    }
}

/// I2C watchdog, `I2C_WDT_SET` (8.10, p34-35; 9.2, p45): the I2C core is reset when no STOP
/// condition arrives within the time-out.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum I2cWatchdog {
    /// Off.
    Off,
    /// On, time-out after 1.28 ms.
    Ms1_28,
    /// On, time-out after 40.96 ms.
    Ms40_96,
}

/// Settings applied by `init`. Build with [`Config::new`] and the `with_` methods; check with
/// [`Config::validate`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Output data rate for normal mode.
    pub odr: OutputDataRate,
    /// Averaging; [`validate`](Self::validate) rejects one the rate does not allow.
    pub averaging: Averaging,
    /// Enabled axes; at least one.
    pub axes: Axes,
    /// Mode entered last. `Suspend` leaves the chip idle.
    pub start_mode: StartMode,
    /// The INT pin.
    pub interrupt: InterruptPin,
    /// I2C watchdog.
    pub watchdog: I2cWatchdog,
}

impl Config {
    /// 100 Hz, 2 samples averaged, all axes, normal mode, data-ready interrupt pulsed, active high,
    /// push-pull, watchdog off.
    pub const fn new() -> Self {
        Self {
            odr: OutputDataRate::Hz100,
            averaging: Averaging::Avg2,
            axes: Axes::ALL,
            start_mode: StartMode::Normal,
            interrupt: InterruptPin::new(),
            watchdog: I2cWatchdog::Off,
        }
    }

    /// Set [`odr`](Self::odr).
    pub const fn with_odr(mut self, v: OutputDataRate) -> Self {
        self.odr = v;
        self
    }

    /// Set [`averaging`](Self::averaging).
    pub const fn with_averaging(mut self, v: Averaging) -> Self {
        self.averaging = v;
        self
    }

    /// Set [`axes`](Self::axes).
    pub const fn with_axes(mut self, v: Axes) -> Self {
        self.axes = v;
        self
    }

    /// Set [`start_mode`](Self::start_mode).
    pub const fn with_start_mode(mut self, v: StartMode) -> Self {
        self.start_mode = v;
        self
    }

    /// Set [`interrupt`](Self::interrupt).
    pub const fn with_interrupt(mut self, v: InterruptPin) -> Self {
        self.interrupt = v;
        self
    }

    /// Set [`watchdog`](Self::watchdog).
    pub const fn with_watchdog(mut self, v: I2cWatchdog) -> Self {
        self.watchdog = v;
        self
    }

    /// Check the configuration against the datasheet. Nothing is sent to the device. Rules are
    /// reported in the order of [`ConfigError`].
    pub const fn validate(&self) -> Result<(), ConfigError> {
        if !self.axes.any() {
            return Err(ConfigError::NoAxes);
        }
        let max = self.odr.max_averaging();
        if self.averaging.code() > max.code() {
            return Err(ConfigError::AveragingTooHigh {
                odr: self.odr,
                averaging: self.averaging,
                max,
            });
        }
        Ok(())
    }
}

impl Default for Config {
    /// Same as [`Config::new`].
    fn default() -> Self {
        Self::new()
    }
}

/// Why a [`Config`] is rejected, or a plan cannot be built.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// No axis is enabled. The datasheet does not state the rule; the vendor driver refuses it
    /// (`bmm350_enable_axes`, vendor).
    NoAxes,
    /// The rate does not allow this averaging (5.1.3, Table 5, p15).
    AveragingTooHigh {
        /// The rate.
        odr: OutputDataRate,
        /// The averaging asked for.
        averaging: Averaging,
        /// The highest averaging the rate allows.
        max: Averaging,
    },
    /// The plan does not fit in [`crate::plan::MAX_STEPS`] steps.
    PlanOverflow,
}

/// Decoded `INT_STATUS`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptStatus {
    /// New magnetic data since the last read of this register (clears on read).
    pub data_ready: bool,
}

impl InterruptStatus {
    /// Decode the register byte.
    pub const fn from_bits(bits: u8) -> Self {
        Self {
            data_ready: bits & reg::INT_STATUS_DRDY != 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn odr_codes() {
        use OutputDataRate::*;
        let all = [
            Hz400, Hz200, Hz100, Hz50, Hz25, Hz12_5, Hz6_25, Hz3_125, Hz1_5625,
        ];
        let codes = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A];
        for (o, c) in all.into_iter().zip(codes) {
            assert_eq!(o.code(), c);
        }
    }

    #[test]
    fn averaging_codes_and_limits() {
        use Averaging::*;
        for (a, c) in [(Avg1, 0), (Avg2, 1), (Avg4, 2), (Avg8, 3)] {
            assert_eq!(a.code(), c);
        }
    }

    #[test]
    fn mode_axes_interrupt_codes() {
        assert_eq!(PowerMode::Suspend.code(), 0x00);
        assert_eq!(PowerMode::Normal.code(), 0x01);
        assert_eq!(PowerMode::Forced.code(), 0x03);
        assert_eq!(PowerMode::ForcedFast.code(), 0x04);
        assert_eq!(Axes::ALL.bits(), 0x07);
        assert_eq!(
            Axes {
                x: true,
                y: false,
                z: true
            }
            .bits(),
            0x05
        );
        assert!(
            !Axes {
                x: false,
                y: false,
                z: false
            }
            .any()
        );
        assert!(InterruptStatus::from_bits(0x04).data_ready);
        assert!(!InterruptStatus::from_bits(0x00).data_ready);
    }
}
