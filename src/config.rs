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
            Self::Hz400 => 0x02,
            Self::Hz200 => 0x03,
            Self::Hz100 => 0x04,
            Self::Hz50 => 0x05,
            Self::Hz25 => 0x06,
            Self::Hz12_5 => 0x07,
            Self::Hz6_25 => 0x08,
            Self::Hz3_125 => 0x09,
            Self::Hz1_5625 => 0x0A,
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

    /// Highest averaging this rate allows (datasheet p15, Table 5).
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
            Self::Avg1 => 0,
            Self::Avg2 => 1,
            Self::Avg4 => 2,
            Self::Avg8 => 3,
        }
    }
}

/// Value of `PMU_CMD_AGGR_SET`. An averaging the rate does not allow is lowered to the highest
/// allowed one, as the vendor driver does.
pub const fn aggr_set(odr: OutputDataRate, avg: Averaging) -> u8 {
    let max = odr.max_averaging();
    let avg = if avg.code() > max.code() { max } else { avg };
    (avg.code() << 4) | odr.code()
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

/// Value of `INT_CTRL` for a data-ready interrupt on the INT pin: pulsed, active high,
/// push-pull, pin output on, data-ready enabled.
pub const fn int_ctrl_data_ready() -> u8 {
    reg::INT_CTRL_ACTIVE_HIGH
        | reg::INT_CTRL_PUSH_PULL
        | reg::INT_CTRL_OUTPUT_EN
        | reg::INT_CTRL_DRDY_EN
}

/// Settings applied by `init`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Output data rate for normal mode.
    pub odr: OutputDataRate,
    /// Averaging; lowered if `odr` does not allow it (see [`aggr_set`]).
    pub averaging: Averaging,
    /// Enabled axes; at least one.
    pub axes: Axes,
    /// Data-ready interrupt on the INT pin: pulsed, active high, push-pull. When false `INT_CTRL`
    /// is left at its reset value (interrupt off).
    pub data_ready_interrupt: bool,
    /// Mode entered last. `Suspend` leaves the chip idle.
    pub power_mode: PowerMode,
}

impl Default for Config {
    /// 100 Hz, 2 samples averaged, all axes, data-ready interrupt on, normal mode.
    fn default() -> Self {
        Self {
            odr: OutputDataRate::Hz100,
            averaging: Averaging::Avg2,
            axes: Axes::ALL,
            data_ready_interrupt: true,
            power_mode: PowerMode::Normal,
        }
    }
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
        assert_eq!(aggr_set(OutputDataRate::Hz100, Averaging::Avg2), 0x14);
        assert_eq!(aggr_set(OutputDataRate::Hz400, Averaging::Avg8), 0x02);
        assert_eq!(aggr_set(OutputDataRate::Hz200, Averaging::Avg8), 0x13);
        assert_eq!(aggr_set(OutputDataRate::Hz100, Averaging::Avg8), 0x24);
        assert_eq!(aggr_set(OutputDataRate::Hz50, Averaging::Avg8), 0x35);
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
        assert_eq!(int_ctrl_data_ready(), 0x8E);
        assert!(InterruptStatus::from_bits(0x04).data_ready);
        assert!(!InterruptStatus::from_bits(0x00).data_ready);
    }
}
