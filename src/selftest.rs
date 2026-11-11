//! Built-in self-test, magnetic reset and the high-field check: the facts, and the pure parts.
//!
//! Sources: "datasheet" is BST-BMM350-DS001-27 (printed page numbers); "vendor" is Bosch's
//! `BMM350_SensorAPI` (`bmm350.c`, `bmm350_defs.h`), named with the function the fact comes from.
//! Where they disagree the datasheet is followed and the disagreement is listed below.
//!
//! # Self-test (datasheet 5.1.6, p17; 8.33, p42)
//!
//! * Axes: X and Y only; there is no self-test for Z. Each axis has a positive and a negative
//!   polarity, so four entries. Only one axis may be activated at a time (5.1.6).
//! * Register `TMR_SELFTEST_USER` (0x60, 8.33), bits: `st_igen_en` bit 0 (current generator on, a
//!   precondition), `st_n` bit 1, `st_p` bit 2, `ist_en_x` bit 3, `ist_en_y` bit 4; bits 7:5
//!   reserved. `st_n` and `st_p` must never be set together (internal short, 8.33). Bit positions
//!   are the vendor's (`BMM350_ST_*_MSK`); 8.33 gives the bit names and the order only. The four
//!   values the vendor writes (`BMM350_SELF_TEST_POS_X` 0x0D, `_NEG_X` 0x0B, `_POS_Y` 0x15,
//!   `_NEG_Y` 0x13) are [`stimulus`].
//! * Mode: the device must be in suspend and forced mode captures the data before and during the
//!   stimulus (5.1.6).
//! * Result: an internal field of about 130 uT is added to the ambient field (5.1.6). With
//!   `x = mag_x(during) - mag_x(before)` the datasheet asks `x >= 130 uT` and the same for y. The
//!   datasheet writes it for the positive polarity; for the negative one this crate requires the
//!   change in the polarity's direction: `during - before <= -130 uT`. The limit is
//!   [`MIN_DELTA_UT`], in microtesla of the compensated field. The scope note attributes the limit
//!   to the vendor; the vendor files do not contain it (they only return the values), the
//!   datasheet does.
//! * Vendor order (`bmm350_perform_self_test`, `self_test_entry_config`, `self_test_config`):
//!   `SUS` and 30 ms; 100 Hz and averaging 2 in `PMU_CMD_AGGR_SET`; all axes on in
//!   `PMU_CMD_AXIS_EN`; `FGR` and 30 ms, echo checked; `BR_FAST` and 4 ms, echo checked;
//!   `FM_FAST`, 16 ms, echo checked; then per entry: write the stimulus value, 1 ms, `FM_FAST`,
//!   6 ms, echo checked, read the data, convert to uT. The vendor has no baseline (it keeps the
//!   values with the stimulus on and returns `positive - negative` per axis, without a limit),
//!   does not write the register back to zero between entries and writes zero at the end with
//!   1 ms after it.
//!
//! The sequence of [`Bmm350::self_test`](crate::Bmm350::self_test) is in its documentation.
//!
//! Measured on the MARV V2 board, 2026-11-11 (three runs, all passing): the test takes 113.7 ms from
//! the default configuration, 96.1 ms from a configuration that starts in suspend and 115.2 ms
//! with a latched interrupt (the delays alone add up to 106 and 88 ms). The stimulus is +174.0 /
//! -173.9 uT on X and +174.1 / -174.1 uT on Y (spread 0.03 to 0.21 uT) against the 130 uT limit;
//! the positive stimulus raises and the negative one lowers the reading on both axes; no offset is
//! left behind (at most 0.2 uT per axis over 50 samples afterwards).
//!
//! # Magnetic reset (datasheet 5.1.5, p17; 8.6, p32)
//!
//! * Commands in `PMU_CMD`: `BR` 0x07 (bit reset) and `FGR` 0x05 (flux-guide reset) with full
//!   recharge of the CRST capacitor; `BR_FAST` 0x08 and `FGR_FAST` 0x06 with fast recharge. A
//!   reset is the pair bit reset then flux-guide reset (vendor `bmm350_magnetic_reset_and_wait`).
//! * Mode: suspend only; the vendor function goes to suspend first when the chip is in normal
//!   mode. In normal and forced mode the sensor resets itself for every ODR tick (5.1.5 hint).
//! * When: after a field shock or when offset or sensitivity drift suggests one happened while
//!   the chip was in suspend (the chip cannot notice that itself), and as a preventive reset
//!   before leaving a suspend longer than about 2 s; the CRST capacitor discharges with a time
//!   constant of about 1 s (5.1.5). A power-up does it by itself.
//! * Waits: full pair, vendor `BMM350_BR_DELAY` 14 ms and `BMM350_FGR_DELAY` 18 ms, each followed
//!   by a check of the command echo in `PMU_CMD_STATUS_0` bits 7:5. Charging takes less than
//!   8 ms by design (5.1.5). The fast pair has no figure in the datasheet; the only vendor use is
//!   `BR_FAST` followed by 4 ms in `self_test_entry_config` (measured: 12.6 ms for the pair). Use the full pair
//!   ([`Bmm350::magnetic_reset`](crate::Bmm350::magnetic_reset), the one `init` uses) after a long
//!   stay in suspend or a shock; the fast pair
//!   ([`Bmm350::magnetic_reset_fast`](crate::Bmm350::magnetic_reset_fast)) when time matters and
//!   the capacitor is known to be charged recently.
//!   Measured on the same board, same day: `magnetic_reset` 32.6 ms with `PMU_CMD_STATUS_0` 0xA0
//!   afterwards (echo field 5), `magnetic_reset_fast` 12.6 ms with 0xC0 (echo field 6); the field
//!   after each was within 0.3 uT of the field before.
//! * Echo: bits 7:5 of `PMU_CMD_STATUS_0` hold three bits of the command. `BR_FAST` is 0x08, which
//!   does not fit; the vendor expects 7 for it (`BMM350_PMU_CMD_STATUS_0_BR_FAST`), the same as
//!   `BR`, and so does this crate.
//!
//! # High field (datasheet 5.3, p19; 5.1.5, p17)
//!
//! The chip handles fields up to +-2000 uT per axis, and the limit on the vector is
//! `sqrt(Hx^2 + Hy^2 + Hz^2) = 2000 uT` (5.3). The datasheet names a vendor function
//! `BMM350_check_for_highfield_and_reset` to be called right after a compensated read, which
//! checks the range and then resets; the vendor sources available here do not contain it, so the
//! predicate [`exceeds_high_field`] is written from 5.3. It is true when the magnitude exceeds
//! [`HIGH_FIELD_LIMIT_UT`] (which includes any single axis beyond it) or when a value is not a
//! finite number. What to do when it is true: run a magnetic reset in suspend (put the chip in
//! suspend with [`Bmm350::set_power_mode`](crate::Bmm350::set_power_mode), call
//! `magnetic_reset`, set the mode again) and read again. The driver never does that by itself.
//!
//! # Datasheet against the vendor
//!
//! * The datasheet compares data during the stimulus with data before it; the vendor compares the
//!   positive with the negative stimulus. This crate follows the datasheet.
//! * The vendor enters the self-test with `FGR` then `BR_FAST`; this crate uses the full pair `BR`
//!   then `FGR` as `init` does.
//! * 5.3 gives a limit on the vector; the per-axis rating (+-2000 uT) is the same number.

use crate::data::Field;
use crate::reg;

/// Least change of the compensated field, in microtesla, in the direction of the stimulus for an
/// entry to pass: 130 uT (datasheet 5.1.6, p17).
pub const MIN_DELTA_UT: f32 = 130.0;

/// Field magnitude in microtesla beyond which the sensor should be magnetically reset: 2000 uT
/// (datasheet 5.3, p19).
pub const HIGH_FIELD_LIMIT_UT: f32 = 2000.0;

/// An axis that has a self-test. Z has none (datasheet 5.1.6).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SelfTestAxis {
    /// X, `ist_en_x`.
    #[default]
    X,
    /// Y, `ist_en_y`.
    Y,
}

/// The polarity of the self-test field.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SelfTestPolarity {
    /// `st_p`: the field adds a positive value.
    #[default]
    Positive,
    /// `st_n`: the field adds a negative value.
    Negative,
}

/// The four entries in the order the self-test runs them.
pub const ENTRIES: [(SelfTestAxis, SelfTestPolarity); 4] = [
    (SelfTestAxis::X, SelfTestPolarity::Positive),
    (SelfTestAxis::X, SelfTestPolarity::Negative),
    (SelfTestAxis::Y, SelfTestPolarity::Positive),
    (SelfTestAxis::Y, SelfTestPolarity::Negative),
];

/// The `TMR_SELFTEST_USER` value that switches one entry's stimulus on: the generator, one
/// polarity and one axis (8.33, p42; bit positions: vendor). Never both polarities.
pub const fn stimulus(axis: SelfTestAxis, polarity: SelfTestPolarity) -> u8 {
    reg::ST_IGEN_EN
        | match polarity {
            SelfTestPolarity::Positive => reg::ST_P,
            SelfTestPolarity::Negative => reg::ST_N,
        }
        | match axis {
            SelfTestAxis::X => reg::IST_EN_X,
            SelfTestAxis::Y => reg::IST_EN_Y,
        }
}

/// One axis and polarity of a self-test.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct SelfTestEntry {
    /// The axis.
    pub axis: SelfTestAxis,
    /// The polarity of the stimulus.
    pub polarity: SelfTestPolarity,
    /// The compensated field of the axis before the stimulus, in microtesla.
    pub without: f32,
    /// The compensated field of the axis during the stimulus, in microtesla.
    pub with: f32,
    /// `with - without`, in microtesla.
    pub delta: f32,
    /// True when the change in the direction of the polarity is at least [`MIN_DELTA_UT`].
    pub passed: bool,
}

/// Judge one entry from the field without and with the stimulus (microtesla). A value that is
/// not a number fails.
pub fn evaluate_entry(
    axis: SelfTestAxis,
    polarity: SelfTestPolarity,
    without: f32,
    with: f32,
) -> SelfTestEntry {
    let delta = with - without;
    let toward = match polarity {
        SelfTestPolarity::Positive => delta,
        SelfTestPolarity::Negative => -delta,
    };
    SelfTestEntry {
        axis,
        polarity,
        without,
        with,
        delta,
        passed: toward >= MIN_DELTA_UT,
    }
}

/// The result of a completed self-test: the four entries of [`ENTRIES`], in that order.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct SelfTestReport {
    /// X positive, X negative, Y positive, Y negative.
    pub entries: [SelfTestEntry; 4],
}

impl Default for SelfTestReport {
    /// All values zero and every entry failed.
    fn default() -> Self {
        Self::from_readings([(0.0, 0.0); 4])
    }
}

impl SelfTestReport {
    /// Judge the four `(without, with)` pairs, in the order of [`ENTRIES`].
    pub fn from_readings(readings: [(f32, f32); 4]) -> Self {
        let [a, b, c, d] = readings;
        let one = |i: usize, (without, with): (f32, f32)| {
            let (axis, polarity) = ENTRIES.get(i).copied().unwrap_or_default();
            evaluate_entry(axis, polarity, without, with)
        };
        Self {
            entries: [one(0, a), one(1, b), one(2, c), one(3, d)],
        }
    }

    /// True when every entry passed.
    pub fn passed(&self) -> bool {
        self.entries.iter().all(|e| e.passed)
    }
}

/// True when the compensated field is beyond what the sensor handles: the magnitude exceeds
/// [`HIGH_FIELD_LIMIT_UT`] (any single axis beyond it included), or a component is not finite.
/// The temperature is ignored. See the module documentation for what to do when it is true.
pub fn exceeds_high_field(field: &Field) -> bool {
    let (x, y, z) = (field.x, field.y, field.z);
    if !(x.is_finite() && y.is_finite() && z.is_finite()) {
        return true;
    }
    x * x + y * y + z * z > HIGH_FIELD_LIMIT_UT * HIGH_FIELD_LIMIT_UT
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stimulus_values_are_the_vendors_and_never_set_both_polarities() {
        use SelfTestAxis::{X, Y};
        use SelfTestPolarity::{Negative, Positive};
        assert_eq!(stimulus(X, Positive), 0x0D);
        assert_eq!(stimulus(X, Negative), 0x0B);
        assert_eq!(stimulus(Y, Positive), 0x15);
        assert_eq!(stimulus(Y, Negative), 0x13);
        for (a, p) in ENTRIES {
            let v = stimulus(a, p);
            assert_ne!(v & reg::ST_IGEN_EN, 0);
            assert_ne!(v & reg::ST_N != 0, v & reg::ST_P != 0);
            assert_ne!(v & reg::IST_EN_X != 0, v & reg::IST_EN_Y != 0);
        }
    }

    #[test]
    fn the_limit_is_inclusive_in_the_direction_of_the_polarity() {
        use SelfTestAxis::X;
        use SelfTestPolarity::{Negative, Positive};
        assert!(evaluate_entry(X, Positive, 10.0, 140.0).passed);
        assert!(!evaluate_entry(X, Positive, 10.0, 139.0).passed);
        assert!(!evaluate_entry(X, Positive, 10.0, 139.999).passed);
        assert!(evaluate_entry(X, Negative, 10.0, -120.0).passed);
        assert!(!evaluate_entry(X, Negative, 10.0, -119.0).passed);
        // The wrong direction fails however large.
        assert!(!evaluate_entry(X, Positive, 10.0, -500.0).passed);
        assert!(!evaluate_entry(X, Negative, 10.0, 500.0).passed);
        assert!(!evaluate_entry(X, Positive, f32::NAN, 500.0).passed);
        let e = evaluate_entry(X, Negative, 10.0, -120.0);
        assert_eq!((e.without, e.with, e.delta), (10.0, -120.0, -130.0));
    }

    #[test]
    fn a_report_passes_only_when_all_four_entries_pass() {
        let good = [(0.0, 130.0), (0.0, -130.0), (5.0, 135.0), (5.0, -125.0)];
        let r = SelfTestReport::from_readings(good);
        assert!(r.passed());
        assert_eq!(r.entries[2].axis, SelfTestAxis::Y);
        assert_eq!(r.entries[3].polarity, SelfTestPolarity::Negative);
        for (i, sign) in [1.0f32, -1.0, 1.0, -1.0].into_iter().enumerate() {
            let mut bad = good;
            if let Some(pair) = bad.get_mut(i) {
                pair.1 = pair.0 + sign * 129.9;
            }
            assert!(!SelfTestReport::from_readings(bad).passed(), "entry {i}");
        }
        assert!(!SelfTestReport::default().passed());
        assert_eq!(
            SelfTestReport::default().entries[1],
            SelfTestEntry {
                polarity: SelfTestPolarity::Negative,
                ..SelfTestEntry::default()
            }
        );
    }

    #[test]
    fn high_field_is_the_magnitude_over_2000_microtesla_or_a_non_number() {
        let f = |x, y, z| Field {
            x,
            y,
            z,
            temperature: 25.0,
        };
        assert!(!exceeds_high_field(&f(0.0, 0.0, 0.0)));
        assert!(!exceeds_high_field(&f(2000.0, 0.0, 0.0)));
        assert!(!exceeds_high_field(&f(0.0, -2000.0, 0.0)));
        assert!(exceeds_high_field(&f(2000.5, 0.0, 0.0)));
        assert!(exceeds_high_field(&f(0.0, 0.0, -2000.5)));
        assert!(!exceeds_high_field(&f(1000.0, 1000.0, 1000.0)));
        assert!(exceeds_high_field(&f(1200.0, 1200.0, 1200.0)));
        assert!(exceeds_high_field(&f(f32::NAN, 0.0, 0.0)));
        assert!(exceeds_high_field(&f(0.0, f32::INFINITY, 0.0)));
    }
}
