//! Host tests of the settings layer: register images, validation, plans, helpers.

use crate::config::*;
use crate::plan::{Apply, MAX_STEPS, Step};
use crate::reg;
use crate::timing::{self, Recharge};
use proptest::prelude::*;

const ODRS: [OutputDataRate; 9] = [
    OutputDataRate::Hz400,
    OutputDataRate::Hz200,
    OutputDataRate::Hz100,
    OutputDataRate::Hz50,
    OutputDataRate::Hz25,
    OutputDataRate::Hz12_5,
    OutputDataRate::Hz6_25,
    OutputDataRate::Hz3_125,
    OutputDataRate::Hz1_5625,
];
const AVGS: [Averaging; 4] = [
    Averaging::Avg1,
    Averaging::Avg2,
    Averaging::Avg4,
    Averaging::Avg8,
];

/// Datasheet 5.1.3, Table 5, p15: rows are the rates 400 Hz down to 1.5625 Hz, columns the
/// averaging 1, 2, 4, 8; true where the table has a current figure, false where it has "-".
const TABLE_5: [[bool; 4]; 9] = [
    [true, false, false, false],
    [true, true, false, false],
    [true, true, true, false],
    [true, true, true, true],
    [true, true, true, true],
    [true, true, true, true],
    [true, true, true, true],
    [true, true, true, true],
    [true, true, true, true],
];

#[test]
fn image_pmu_cmd_aggr_set_matches_8_4() {
    // 8.4, p30: avg in bits 5:4, odr in bits 3:0. Hand computed: (avg << 4) | odr.
    let c = Config::new();
    assert_eq!(c.pmu_cmd_aggr_set(), 0x14); // avg 2 samples (1), 100 Hz (4); also the reset value
    assert_eq!(
        c.with_odr(OutputDataRate::Hz400)
            .with_averaging(Averaging::Avg1)
            .pmu_cmd_aggr_set(),
        0x02
    );
    assert_eq!(
        c.with_odr(OutputDataRate::Hz200)
            .with_averaging(Averaging::Avg2)
            .pmu_cmd_aggr_set(),
        0x13
    );
    assert_eq!(
        c.with_odr(OutputDataRate::Hz100)
            .with_averaging(Averaging::Avg4)
            .pmu_cmd_aggr_set(),
        0x24
    );
    assert_eq!(
        c.with_odr(OutputDataRate::Hz50)
            .with_averaging(Averaging::Avg8)
            .pmu_cmd_aggr_set(),
        0x35
    );
    assert_eq!(
        c.with_odr(OutputDataRate::Hz1_5625)
            .with_averaging(Averaging::Avg8)
            .pmu_cmd_aggr_set(),
        0x3A
    );
    // Not lowered: validate rejects, the image says what was asked.
    assert_eq!(
        c.with_odr(OutputDataRate::Hz400)
            .with_averaging(Averaging::Avg8)
            .pmu_cmd_aggr_set(),
        0x32
    );
}

#[test]
fn image_pmu_cmd_axis_en_matches_8_5() {
    // 8.5, p31: en_x bit 0, en_y bit 1, en_z bit 2.
    let a = |x, y, z| Config::new().with_axes(Axes { x, y, z }).pmu_cmd_axis_en();
    assert_eq!(a(true, true, true), 0x07);
    assert_eq!(a(true, false, false), 0x01);
    assert_eq!(a(false, true, false), 0x02);
    assert_eq!(a(false, false, true), 0x04);
    assert_eq!(a(true, false, true), 0x05);
    assert_eq!(a(false, false, false), 0x00);
}

#[test]
fn image_int_ctrl_matches_8_11() {
    // 8.11, p35: drdy_data_reg_en 0x80, int_output_en 0x08, int_od (push-pull) 0x04,
    // int_pol (high) 0x02, int_mode (latched) 0x01.
    let pin = InterruptPin::new();
    let c = |p| Config::new().with_interrupt(p).int_ctrl();
    assert_eq!(c(pin), 0x8E);
    assert_eq!(c(pin.with_mode(InterruptMode::Latched)), 0x8F);
    assert_eq!(c(pin.with_polarity(InterruptPolarity::ActiveLow)), 0x8C);
    assert_eq!(c(pin.with_drive(InterruptDrive::OpenDrain)), 0x8A);
    assert_eq!(
        c(pin
            .with_mode(InterruptMode::Latched)
            .with_polarity(InterruptPolarity::ActiveLow)
            .with_drive(InterruptDrive::OpenDrain)),
        0x89
    );
    // Data-ready off: the reset value (0x00, 8.11 p35) whatever the other settings are.
    assert_eq!(c(pin.with_data_ready(false)), 0x00);
    assert_eq!(
        c(pin
            .with_data_ready(false)
            .with_mode(InterruptMode::Latched)
            .with_drive(InterruptDrive::OpenDrain)),
        0x00
    );
}

#[test]
fn image_i2c_wdt_set_matches_8_10() {
    // 8.10, p34-35: i2c_wdt_en bit 0, i2c_wdt_sel bit 1 (0 is 1.28 ms, 1 is 40.96 ms).
    let w = |w| Config::new().with_watchdog(w).i2c_wdt_set();
    assert_eq!(w(I2cWatchdog::Off), 0x00);
    assert_eq!(w(I2cWatchdog::Ms1_28), 0x01);
    assert_eq!(w(I2cWatchdog::Ms40_96), 0x03);
}

#[test]
fn start_mode_codes_match_8_6() {
    // 8.6, p32.
    assert_eq!(StartMode::Suspend.code(), 0x00);
    assert_eq!(StartMode::Normal.code(), 0x01);
}

#[test]
fn validate_rejects_each_reason() {
    assert_eq!(Config::new().validate(), Ok(()));
    let none = Axes {
        x: false,
        y: false,
        z: false,
    };
    assert_eq!(
        Config::new().with_axes(none).validate(),
        Err(ConfigError::NoAxes)
    );
    assert_eq!(
        Config::new()
            .with_odr(OutputDataRate::Hz400)
            .with_averaging(Averaging::Avg2)
            .validate(),
        Err(ConfigError::AveragingTooHigh {
            odr: OutputDataRate::Hz400,
            averaging: Averaging::Avg2,
            max: Averaging::Avg1,
        })
    );
    assert_eq!(
        Config::new()
            .with_odr(OutputDataRate::Hz100)
            .with_averaging(Averaging::Avg8)
            .validate(),
        Err(ConfigError::AveragingTooHigh {
            odr: OutputDataRate::Hz100,
            averaging: Averaging::Avg8,
            max: Averaging::Avg4,
        })
    );
    // The plan refuses an invalid configuration with the same reason.
    assert_eq!(
        Config::new().with_axes(none).plan(Apply::AfterReset),
        Err(ConfigError::NoAxes)
    );
}

#[test]
fn plan_overflow_error_is_a_config_error_variant() {
    // The variant is produced by the plan's push when the array is full; see
    // `plan::overflow_tests`. No configuration produces it: the longest plan has 6 of 8 steps.
    let longest = Config::new().with_watchdog(I2cWatchdog::Ms40_96);
    assert_eq!(longest.plan(Apply::InPlace).map(|p| p.len()), Ok(7));
    assert_eq!(MAX_STEPS, 8);
}

#[test]
fn plan_default_after_reset_is_the_literal_list() {
    // Changed against the previous plan, and why: no UPD_OAE (the chip rejects it in suspend, R1,
    // and the NM that follows takes over PMU_CMD_AGGR_SET, R5; measured on the board, 2026-11-10),
    // so the writes are AGGR_SET 0x14, AXIS_EN 0x07, INT_CTRL 0x8E and then NM; NM carries the
    // minimum time before the first status poll, 15 ms (was a 70 ms wait: the datasheet's typical
    // start-up time, Table 1, p8; NM was measured busy for 15.0 ms) and the poll bound, 50 ms.
    // I2C_WDT_SET is skipped because its image equals the reset value.
    let p = Config::new().plan(Apply::AfterReset).unwrap();
    assert_eq!(
        p.steps(),
        &[
            Step::Write {
                reg: 0x04,
                value: 0x14
            },
            Step::Write {
                reg: 0x05,
                value: 0x07
            },
            Step::Write {
                reg: 0x2E,
                value: 0x8E
            },
            Step::Write {
                reg: 0x60,
                value: 0x00
            },
            Step::PmuCommand {
                cmd: 0x01,
                min_wait_us: 15_000,
                bound_us: 50_000
            },
        ]
    );
    // Watchdog not at its reset value: written before NM. Suspend start mode: no NM.
    let p = Config::new()
        .with_watchdog(I2cWatchdog::Ms1_28)
        .with_start_mode(StartMode::Suspend)
        .plan(Apply::AfterReset)
        .unwrap();
    assert_eq!(p.len(), 5);
    assert_eq!(
        p.steps()[3],
        Step::Write {
            reg: 0x0A,
            value: 0x01
        }
    );
}

#[test]
fn plan_in_place_suspends_first_and_writes_everything() {
    // Changed against the previous plan, and why: no UPD_OAE after AGGR_SET (R1, R5: the plan
    // goes through suspend, where the NM at its end takes AGGR_SET over); SUS carries 2 ms
    // minimum and a 20 ms bound (busy 1.8 to 4 ms from normal mode, measured on the board,
    // 2026-11-10), NM 15 ms and 50 ms.
    let c = Config::new().with_watchdog(I2cWatchdog::Ms1_28);
    let p = c.plan(Apply::InPlace).unwrap();
    assert_eq!(
        p.steps(),
        &[
            Step::PmuCommand {
                cmd: 0x00,
                min_wait_us: 2_000,
                bound_us: 20_000
            },
            Step::Write {
                reg: 0x04,
                value: 0x14
            },
            Step::Write {
                reg: 0x05,
                value: 0x07
            },
            Step::Write {
                reg: 0x2E,
                value: 0x8E
            },
            Step::Write {
                reg: 0x0A,
                value: 0x01
            },
            Step::Write {
                reg: 0x60,
                value: 0x00
            },
            Step::PmuCommand {
                cmd: 0x01,
                min_wait_us: 15_000,
                bound_us: 50_000
            },
        ]
    );
    // Suspend start mode, interrupt off: INT_CTRL still written (0x00), no NM.
    let c = Config::new()
        .with_start_mode(StartMode::Suspend)
        .with_interrupt(InterruptPin::new().with_data_ready(false));
    let p = c.plan(Apply::InPlace).unwrap();
    assert_eq!(p.len(), 6);
    assert_eq!(
        p.steps()[3],
        Step::Write {
            reg: 0x2E,
            value: 0x00
        }
    );
    assert_eq!(
        p.steps()[4],
        Step::Write {
            reg: 0x0A,
            value: 0x00
        }
    );
    let after = c.plan(Apply::AfterReset).unwrap();
    assert!(after.iter().any(|s| *s
        == Step::Write {
            reg: 0x2E,
            value: 0x00
        }));
    assert!(
        !after
            .iter()
            .any(|s| matches!(s, Step::Write { reg: 0x0A, .. }))
    );
}

#[test]
fn config_builds_in_a_const() {
    const C: Config = Config::new()
        .with_odr(OutputDataRate::Hz200)
        .with_averaging(Averaging::Avg2)
        .with_watchdog(I2cWatchdog::Ms40_96)
        .with_interrupt(InterruptPin::new().with_mode(InterruptMode::Latched));
    const AGGR: u8 = C.pmu_cmd_aggr_set();
    const INT: u8 = C.int_ctrl();
    const OK: Result<(), ConfigError> = C.validate();
    assert_eq!(AGGR, 0x13);
    assert_eq!(INT, 0x8F);
    assert_eq!(OK, Ok(()));
    assert_eq!(Config::default(), Config::new());
}

#[test]
fn helpers_match_the_tables() {
    // Period = 1 / ODR (8.4, p30).
    let periods = [
        2_500, 5_000, 10_000, 20_000, 40_000, 80_000, 160_000, 320_000, 640_000,
    ];
    for (o, p) in ODRS.into_iter().zip(periods) {
        assert_eq!(o.period_us(), p);
        // hertz() * period = 1e6 within f32 rounding
        assert!((o.hertz() * p as f32 - 1_000_000.0).abs() < 1.0);
    }
    // Vendor times; datasheet bounds 5.1.4, p15: < 5 ms (avg 1, fast) and up to 30 ms (avg 8).
    let full = [15_000, 17_000, 20_000, 28_000];
    let fast = [4_000, 5_000, 9_000, 16_000];
    for (i, a) in AVGS.into_iter().enumerate() {
        assert_eq!(timing::forced_measurement_us(a, Recharge::Full), full[i]);
        assert_eq!(timing::forced_measurement_us(a, Recharge::Fast), fast[i]);
    }
    // Table 6, p16.
    let rates = [200, 150, 80, 40];
    for (a, r) in AVGS.into_iter().zip(rates) {
        assert_eq!(timing::forced_fast_max_rate_hz(a), r);
    }
    // Sensor time: 25 600 ticks per second, 39.0625 us per tick (measured, 2026-11-11).
    assert_eq!(timing::SENSOR_TIME_TICKS_PER_SECOND, 25_600);
    assert_eq!(timing::sensor_time_us(25_600), 1_000_000);
    assert_eq!(timing::sensor_time_us(256), 10_000);
    assert_eq!(timing::sensor_time_us(64), 2_500);
    assert_eq!(timing::sensor_time_us(1), 39);
    assert_eq!(timing::sensor_time_us(0x00FF_FFFF), 655_359_960);
}

#[test]
fn table_5_matches_max_averaging() {
    for (o, row) in ODRS.into_iter().zip(TABLE_5) {
        for (a, ok) in AVGS.into_iter().zip(row) {
            assert_eq!(a <= o.max_averaging(), ok);
        }
    }
}

fn odr() -> impl Strategy<Value = OutputDataRate> {
    (0usize..9).prop_map(|i| ODRS[i])
}
fn avg() -> impl Strategy<Value = Averaging> {
    (0usize..4).prop_map(|i| AVGS[i])
}
fn config() -> impl Strategy<Value = Config> {
    (
        odr(),
        avg(),
        any::<(bool, bool, bool)>(),
        any::<bool>(),
        (any::<bool>(), any::<bool>(), any::<bool>(), any::<bool>()),
        0u8..3,
    )
        .prop_map(|(o, a, (x, y, z), normal, (dr, latch, od, high), w)| {
            Config::new()
                .with_odr(o)
                .with_averaging(a)
                .with_axes(Axes { x, y, z })
                .with_start_mode(if normal {
                    StartMode::Normal
                } else {
                    StartMode::Suspend
                })
                .with_interrupt(
                    InterruptPin::new()
                        .with_data_ready(dr)
                        .with_mode(if latch {
                            InterruptMode::Latched
                        } else {
                            InterruptMode::Pulsed
                        })
                        .with_drive(if od {
                            InterruptDrive::OpenDrain
                        } else {
                            InterruptDrive::PushPull
                        })
                        .with_polarity(if high {
                            InterruptPolarity::ActiveHigh
                        } else {
                            InterruptPolarity::ActiveLow
                        }),
                )
                .with_watchdog(
                    [I2cWatchdog::Off, I2cWatchdog::Ms1_28, I2cWatchdog::Ms40_96][w as usize],
                )
        })
}

proptest! {
    #[test]
    fn images_never_set_a_reserved_bit(c in config()) {
        prop_assert_eq!(c.pmu_cmd_aggr_set() & reg::PMU_CMD_AGGR_SET_RESERVED, 0);
        prop_assert_eq!(c.pmu_cmd_axis_en() & reg::PMU_CMD_AXIS_EN_RESERVED, 0);
        prop_assert_eq!(c.int_ctrl() & reg::INT_CTRL_RESERVED, 0);
        prop_assert_eq!(c.i2c_wdt_set() & reg::I2C_WDT_SET_RESERVED, 0);
        if !c.interrupt.data_ready {
            prop_assert_eq!(c.int_ctrl(), reg::INT_CTRL_RESET);
        }
    }

    #[test]
    fn validate_accepts_exactly_table_5_with_an_axis(c in config()) {
        let oi = ODRS.iter().position(|o| *o == c.odr).unwrap();
        let ai = AVGS.iter().position(|a| *a == c.averaging).unwrap();
        let expect = TABLE_5[oi][ai] && (c.axes.x || c.axes.y || c.axes.z);
        prop_assert_eq!(c.validate().is_ok(), expect);
        prop_assert_eq!(c.plan(Apply::AfterReset).is_ok(), expect);
        prop_assert_eq!(c.plan(Apply::InPlace).is_ok(), expect);
    }
}
