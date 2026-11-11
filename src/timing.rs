//! Times and rates as pure functions. "vendor" marks a figure taken from Bosch's
//! `BMM350_SensorAPI` (`bmm350_defs.h`) because the datasheet gives only a bound.

use crate::config::Averaging;

/// Which forced-mode command: `FM` (full CRST recharge) or `FM_FAST` (fast recharge), 5.1.4,
/// p15-16.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Recharge {
    /// `PMU_CMD` FM.
    Full,
    /// `PMU_CMD` FM_FAST.
    Fast,
}

/// Ticks of the sensor time counter per second: 25 600, so one tick is 1/25 600 s = 39.0625 us.
/// The counter is 24-bit unsigned. The datasheet gives "40 us resolution" (section 5, p13), which
/// is this value rounded. Measured on the MARV V2 board, 2026-11-11: the counter advances by
/// exactly 256 ticks per sample at 100 Hz and exactly 64 ticks per sample at 400 Hz.
///
/// The counter runs from the sensor's own oscillator, so it drifts against the host clock: on the
/// measured unit one tick was about 38.8 us of the MCU's time (the sensor's 100 Hz rate measured
/// 100.69 Hz, 0.7 % fast). Use [`sensor_elapsed_us_with_period`] to convert with a tick period
/// calibrated against the host clock.
///
/// In normal mode the counter is not free-running: it is the time of the last sample and moves in
/// steps of one sample period (measured on the board, 2026-11-11), so two equal readings can mean
/// that no new sample arrived as well as a stalled sensor. In suspend it does not advance.
pub const SENSOR_TIME_TICKS_PER_SECOND: u32 = 25_600;

/// The sensor time counter is 24 bits wide: it wraps after `0x00FF_FFFF` ticks (about 655 s).
pub const SENSOR_TIME_MASK: u32 = 0x00FF_FFFF;

/// Sensor time in microseconds for a tick count at the nominal tick of 39.0625 us (`ticks * 625 /
/// 16`, exact up to the final rounding down; `SENSORTIME` registers, 5.2.2, p18). It does not
/// handle the wrap of the 24-bit counter: for a difference of two readings use
/// [`sensor_elapsed_us`]. See [`SENSOR_TIME_TICKS_PER_SECOND`] for the drift against the host
/// clock.
pub const fn sensor_time_us(ticks: u32) -> u64 {
    ((ticks as u64) * 625) >> 4
}

/// Ticks from the reading `earlier` to the reading `later` of the 24-bit counter, with the wrap
/// handled (bits above 24 are ignored). Right for any interval shorter than the wrap period.
pub const fn sensor_ticks_elapsed(earlier: u32, later: u32) -> u32 {
    later.wrapping_sub(earlier) & SENSOR_TIME_MASK
}

/// Microseconds from the reading `earlier` to the reading `later` of the 24-bit counter, with the
/// wrap handled, at the nominal tick of 39.0625 us (`ticks * 625 / 16`, rounded down). The largest
/// value, 655 359 960, fits in a `u32`. See [`SENSOR_TIME_TICKS_PER_SECOND`] for the drift against
/// the host clock.
pub const fn sensor_elapsed_us(earlier: u32, later: u32) -> u32 {
    (((sensor_ticks_elapsed(earlier, later) as u64) * 625) >> 4) as u32
}

/// Microseconds from the reading `earlier` to the reading `later` of the 24-bit counter, with the
/// wrap handled, for a tick period of `tick_period_ps` picoseconds that the caller measured
/// against its own clock (the nominal period is 39 062 500 ps). The product of a 24-bit tick count
/// and a `u32` period fits in a `u64`; the result is rounded down.
pub const fn sensor_elapsed_us_with_period(earlier: u32, later: u32, tick_period_ps: u32) -> u64 {
    (sensor_ticks_elapsed(earlier, later) as u64) * (tick_period_ps as u64) / 1_000_000
}

/// Time to wait after a forced-mode trigger from suspend before the chip is free again, in
/// microseconds, for averaging 1, 2, 4, 8: the vendor figure. Vendor: 15, 17, 20, 28 ms for `FM`
/// and 4, 5, 9, 16 ms for `FM_FAST` (`BMM350_SUS_TO_FORCEDMODE_*_DELAY`). Measured on the board,
/// 2026-11-10, command to data ready: `FM` 14.2, 15.4, 18.9, 25.9 ms and `FM_FAST` 3.1, 4.3, 7.8,
/// 14.8 ms, all below the vendor figures; the busy flag is still set at the data-ready edge, and an
/// earlier measurement of the busy time gave 14.5 ms and 27 ms (`FM`, averaging 1 and 8) and 3.4 ms
/// and 16 ms (`FM_FAST`), also at or below the vendor figures. The datasheet gives bounds only:
/// under 5 ms for averaging 1 with `FM_FAST` and up to 30 ms for averaging 8 with `FM` (5.1.4,
/// p15).
pub const fn forced_measurement_us(avg: Averaging, recharge: Recharge) -> u32 {
    match (recharge, avg) {
        (Recharge::Full, Averaging::Avg1) => 15_000,
        (Recharge::Full, Averaging::Avg2) => 17_000,
        (Recharge::Full, Averaging::Avg4) => 20_000,
        (Recharge::Full, Averaging::Avg8) => 28_000,
        (Recharge::Fast, Averaging::Avg1) => 4_000,
        (Recharge::Fast, Averaging::Avg2) => 5_000,
        (Recharge::Fast, Averaging::Avg4) => 9_000,
        (Recharge::Fast, Averaging::Avg8) => 16_000,
    }
}

/// Highest forced-mode trigger rate in hertz with `FM_FAST`: 200, 150, 80, 40 for averaging 1, 2, 4,
/// 8 (5.1.4, Table 6, p16).
pub const fn forced_fast_max_rate_hz(avg: Averaging) -> u16 {
    match avg {
        Averaging::Avg1 => 200,
        Averaging::Avg2 => 150,
        Averaging::Avg4 => 80,
        Averaging::Avg8 => 40,
    }
}
