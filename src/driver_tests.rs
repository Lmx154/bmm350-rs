//! Driver tests on the host harness: golden bus traces (D2), transaction counts (E1), validity
//! state (A3), bus errors (D3), cancellation (A2, D4), PMU status checks, the blocking front-end
//! (A11) and `Send` (A5). Datasheet references are BST-BMM350-DS001-27 printed pages.

use core::fmt::Debug;
use embedded_hal_async::delay::DelayNs;
use std::vec::Vec;

use crate::config::{
    Averaging, Axes, ConfigError, InterruptDrive, InterruptMode, InterruptPin, InterruptPolarity,
    OutputDataRate, StartMode,
};
use crate::testbus::{
    BusFault, DUMMY, Event, Glitch, Op, Polled, Shared, TestBus, TestDelay, poll_then_drop, r, run,
    w,
};
use crate::{
    Bmm350, Config, DEFAULT_ADDRESS, Error, ErrorStatus, Field, I2cWatchdog, Info, Mismatch,
    PmuStatus, PowerMode, RawSample, SelfTestReport, TimedSample, reg,
};

const A: u8 = DEFAULT_ADDRESS;

type Drv = Bmm350<TestBus>;
type Res<T> = Result<T, Error<BusFault>>;

fn rig() -> (Shared, Drv, TestDelay) {
    let shared = Shared::new(A);
    let imu = Bmm350::new(shared.bus(), A);
    let delay = shared.delay();
    (shared, imu, delay)
}

/// 100 Hz, averaging 4, no interrupt, watchdog 1.28 ms, normal mode.
fn alt() -> Config {
    Config::new()
        .with_odr(OutputDataRate::Hz100)
        .with_averaging(Averaging::Avg4)
        .with_interrupt(InterruptPin::new().with_data_ready(false))
        .with_watchdog(I2cWatchdog::Ms1_28)
}

fn suspend() -> Config {
    Config::new().with_start_mode(StartMode::Suspend)
}

fn expected_raw() -> RawSample {
    RawSample {
        x: 1000,
        y: -1000,
        z: 2000,
        temperature: 51450,
    }
}

/// An initialised driver (default configuration), logs and counters cleared.
fn up(config: &Config) -> (Shared, Drv, TestDelay) {
    let (shared, mut imu, mut delay) = rig();
    run(imu.init(&mut delay, config)).unwrap();
    shared.arm(None, None);
    (shared, imu, delay)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Method {
    Init,
    Configure,
    Read,
    ReadField,
}

const METHODS: [Method; 4] = [
    Method::Init,
    Method::Configure,
    Method::Read,
    Method::ReadField,
];

#[derive(Debug, PartialEq)]
enum Out {
    Info(Info),
    Sample(RawSample),
    Field(Field),
}

async fn call(imu: &mut Drv, delay: &mut TestDelay, m: Method) -> Res<Out> {
    Ok(match m {
        Method::Init => Out::Info(imu.init(delay, &Config::new()).await?),
        Method::Configure => Out::Info(imu.configure(delay, &alt()).await?),
        Method::Read => Out::Sample(imu.read().await?),
        Method::ReadField => Out::Field(imu.read_field().await?),
    })
}

/// A driver in the state `m` starts from: fresh for `init`, initialised for the others.
fn start(m: Method) -> (Shared, Drv, TestDelay) {
    if m == Method::Init {
        let (shared, imu, delay) = rig();
        shared.arm(None, None);
        (shared, imu, delay)
    } else {
        up(&Config::new())
    }
}

/// The result and the final register file of an undisturbed run of `m`.
fn reference(m: Method) -> (Res<Out>, [u8; 0x80]) {
    let (shared, mut imu, mut delay) = start(m);
    let out = run(call(&mut imu, &mut delay, m));
    let regs = shared.chip().regs;
    (out, regs)
}

/// After a disturbed call: the state is what A3 says, and the same call again gives the
/// reference result and the reference register file.
fn recover_and_compare(m: Method, shared: &Shared, imu: &mut Drv, delay: &mut TestDelay) {
    let configured = matches!(m, Method::Read | Method::ReadField);
    assert_eq!(imu.config().is_some(), configured, "{m:?}");
    shared.arm(None, None);
    let read = run(imu.read());
    if configured {
        assert_eq!(read, Ok(expected_raw()), "{m:?}");
    } else {
        assert_eq!(read, Err(Error::NotConfigured), "{m:?}");
        assert_eq!(
            shared.transactions(),
            0,
            "{m:?}: a refused read sends nothing"
        );
    }
    let (want, regs) = reference(m);
    assert_eq!(run(call(imu, delay, m)), want, "{m:?}");
    assert_eq!(shared.chip().regs, regs, "{m:?}");
    assert_eq!(run(imu.read()), Ok(expected_raw()), "{m:?}");
    assert!(imu.config().is_some());
}

/// Bus error at every transaction index. Returns the number of transactions of `m`.
fn sweep_fail(m: Method) -> usize {
    for k in 1.. {
        let (shared, mut imu, mut delay) = start(m);
        shared.arm(Some(k), None);
        let out = run(call(&mut imu, &mut delay, m));
        if shared.transactions() < k {
            assert!(out.is_ok(), "{m:?}");
            return k - 1;
        }
        assert_eq!(
            out,
            Err(Error::Bus(BusFault::Injected)),
            "{m:?} at transaction {k}"
        );
        recover_and_compare(m, &shared, &mut imu, &mut delay);
    }
    0
}

/// Cancel at every await point. Returns the number of awaits of `m`.
fn sweep_cancel(m: Method) -> u32 {
    for k in 1.. {
        let (shared, mut imu, mut delay) = start(m);
        shared.arm(None, Some(k));
        if let Polled::Done(out) = poll_then_drop(call(&mut imu, &mut delay, m)) {
            assert!(out.is_ok(), "{m:?}");
            return k - 1;
        }
        recover_and_compare(m, &shared, &mut imu, &mut delay);
    }
    0
}

fn delays(shared: &Shared) -> Vec<u64> {
    shared
        .log()
        .iter()
        .filter_map(|e| match e {
            Event::Delay(ns) => Some(*ns),
            Event::I2c { .. } => None,
        })
        .collect()
}

// ---------------------------------------------------------------------------------------------
// Golden traces (D2) and transaction counts (E1)
// ---------------------------------------------------------------------------------------------

#[test]
fn init_default_golden_trace() {
    let (shared, mut imu, mut delay) = rig();
    shared.arm(None, None);
    let info = run(imu.init(&mut delay, &Config::default())).unwrap();
    assert_eq!(
        info,
        Info {
            chip_id: 0x33,
            output_period_us: 10_000,
            averaging: Averaging::Avg2,
            data_ready_consumed: false,
        }
    );
    // Soft reset: CMD 0xB6 then 0x00 (8.35, p43); CHIP_ID (8.1, p27).
    let mut expected = std::vec![
        Event::Delay(3_000_000),
        w(A, 0x7E, 0xB6),
        Event::Delay(24_000_000),
        w(A, 0x7E, 0x00),
        r(A, 0x00, 0x33),
    ];
    // OTP download (flow of p14): per word the read command 0x20|word, 300 us, OTP_STATUS_REG done
    // (0x01), the two data registers (0x52, 0x53) with the two dummy bytes; then PWR_OFF_OTP 0x80.
    let otp = shared.chip().otp;
    for word in 0..32u8 {
        let value = otp[usize::from(word)];
        expected.push(w(A, 0x50, 0x20 | word));
        expected.push(Event::Delay(300_000));
        expected.push(r(A, 0x55, 0x01));
        expected.push(Event::I2c {
            address: A,
            ops: std::vec![
                Op::Write(std::vec![0x52]),
                Op::Read(std::vec![DUMMY, DUMMY, (value >> 8) as u8, value as u8])
            ],
        });
    }
    expected.push(w(A, 0x50, 0x80));
    // Magnetic reset: BR (7) then FGR (5) in PMU_CMD (8.6, p32), echo in bits 7:5 of
    // PMU_CMD_STATUS_0 (8.7, p33).
    expected.extend([
        w(A, 0x06, 0x07),
        Event::Delay(14_000_000),
        r(A, 0x07, 0xE0),
        w(A, 0x06, 0x05),
        Event::Delay(18_000_000),
        r(A, 0x07, 0xA0),
    ]);
    // The plan after reset, `Config::default()`: PMU_CMD_AGGR_SET 0x14 (avg 2, 100 Hz; 8.4, p30);
    // PMU_CMD_AXIS_EN 0x07 (8.5, p31); INT_CTRL 0x8E (8.11, p35); no UPD_OAE (the chip rejects it
    // in suspend: measured on the board, 2026-11-10); NM (1), 15 ms before the first status poll
    // (NM busy for 15.0 ms, measured on the board, 2026-11-10), the first poll already shows
    // echo 1 and the normal-mode flag: 0x28. TMR_SELFTEST_USER 0x60 = 0x00 (stimulus off, 8.33,
    // p42) is written before NM.
    expected.extend([
        w(A, 0x04, 0x14),
        w(A, 0x05, 0x07),
        w(A, 0x2E, 0x8E),
        w(A, 0x60, 0x00),
        w(A, 0x06, 0x01),
        Event::Delay(15_000_000),
        r(A, 0x07, 0x28),
    ]);
    assert_eq!(shared.log(), expected);
    assert_eq!(shared.transactions(), 110);
    assert_eq!(shared.awaits(), 147);
    assert_eq!(shared.violations(), 0);
    assert_eq!(imu.config(), Some(&Config::default()));
}

#[test]
fn data_read_is_one_transaction() {
    let (shared, mut imu, _delay) = up(&Config::new());
    assert_eq!(run(imu.read()), Ok(expected_raw()));
    // 8.1 (p27): MAG_X_XLSB 0x31, two dummy bytes, then the 12 data bytes in one transaction.
    let data = shared.chip().regs[0x31..0x3D].to_vec();
    let mut got = std::vec![DUMMY, DUMMY];
    got.extend(data);
    assert_eq!(
        shared.log(),
        std::vec![Event::I2c {
            address: A,
            ops: std::vec![Op::Write(std::vec![0x31]), Op::Read(got)]
        }]
    );
    assert_eq!(shared.transactions(), 1);
    let field = run(imu.read_field()).unwrap();
    assert!(field.temperature > 20.0 && field.temperature < 40.0);
    assert_eq!(shared.transactions(), 2);
}

#[test]
fn configure_golden_traces() {
    // On a running device: SUS first (0; 2 ms before the first poll, which finds it still busy
    // (SUS is busy 3 ms in the fake, 1.8 to 4 ms measured on the board, 2026-11-10), one more poll
    // 1 ms later), then the plan of `alt()` without UPD_OAE; AGGR_SET 0x24 (avg 4 is code 2 in
    // bits 5:4, 100 Hz is 4; 8.4, p30), INT_CTRL 0x00, I2C_WDT_SET 0x01 (8.10, p34), NM.
    let (shared, mut imu, mut delay) = up(&Config::new());
    let info = run(imu.configure(&mut delay, &alt())).unwrap();
    assert_eq!(
        info,
        Info {
            chip_id: 0x33,
            output_period_us: 10_000,
            averaging: Averaging::Avg4,
            data_ready_consumed: false,
        }
    );
    let tail = [
        w(A, 0x04, 0x24),
        w(A, 0x05, 0x07),
        w(A, 0x2E, 0x00),
        w(A, 0x0A, 0x01),
        w(A, 0x60, 0x00),
        w(A, 0x06, 0x01),
        Event::Delay(15_000_000),
        r(A, 0x07, 0x28),
    ];
    let mut running = std::vec![
        r(A, 0x07, 0x28),
        w(A, 0x06, 0x00),
        Event::Delay(2_000_000),
        r(A, 0x07, 0x29),
        Event::Delay(1_000_000),
        r(A, 0x07, 0x00),
    ];
    running.extend(tail.clone());
    assert_eq!(shared.log(), running);
    assert_eq!(imu.config(), Some(&alt()));

    // On a suspended device no SUS is sent: just the status read, then the rest.
    let (shared, mut imu, mut delay) = up(&suspend());
    run(imu.configure(&mut delay, &alt())).unwrap();
    // The status of a device that was just initialised in suspend: the echo of FGR (5), 0xA0.
    let mut suspended = std::vec![r(A, 0x07, 0xA0)];
    suspended.extend(tail);
    assert_eq!(shared.log(), suspended);
    assert!(!shared.log().contains(&w(A, 0x06, 0x00)));
}

// ---------------------------------------------------------------------------------------------
// Validity state (A3), bus errors (D3), cancellation (A2, D4)
// ---------------------------------------------------------------------------------------------

#[test]
fn bus_error_at_every_transaction() {
    let counts: Vec<usize> = METHODS.iter().map(|m| sweep_fail(*m)).collect();
    // init 110 and configure on a running device 11 transactions: see the golden traces.
    assert_eq!(counts, std::vec![110, 11, 1, 1]);
}

#[test]
fn cancel_at_every_await() {
    let counts: Vec<u32> = METHODS.iter().map(|m| sweep_cancel(*m)).collect();
    assert_eq!(counts, std::vec![147, 14, 1, 1]);
}

#[test]
fn typed_methods_refuse_while_not_configured() {
    let (shared, mut imu, mut delay) = rig();
    shared.arm(None, None);
    assert_eq!(imu.config(), None);
    assert_eq!(run(imu.read()), Err(Error::NotConfigured));
    assert_eq!(run(imu.read_field()), Err(Error::NotConfigured));
    assert_eq!(run(imu.trigger_forced()), Err(Error::NotConfigured));
    assert_eq!(run(imu.interrupt_status()), Err(Error::NotConfigured));
    assert_eq!(
        run(imu.set_power_mode(&mut delay, PowerMode::Suspend)),
        Err(Error::NotConfigured)
    );
    assert_eq!(
        run(imu.magnetic_reset(&mut delay)),
        Err(Error::NotConfigured)
    );
    // `configure` needs the calibration an `init` reads.
    assert_eq!(
        run(imu.configure(&mut delay, &alt())),
        Err(Error::NotConfigured)
    );
    assert_eq!(shared.transactions(), 0);
    assert_eq!(shared.log(), Vec::new());
    // Raw access is outside A2/A3.
    assert_eq!(run(imu.read_register(reg::CHIP_ID)), Ok(0x33));
    run(imu.init(&mut delay, &Config::new())).unwrap();
    assert!(run(imu.interrupt_status()).is_ok());
    assert!(imu.config().is_some());
}

#[test]
fn a_failed_configure_clears_the_state_and_init_failure_keeps_it_cleared() {
    let (shared, mut imu, mut delay) = up(&Config::new());
    shared.arm(Some(2), None);
    assert!(run(imu.configure(&mut delay, &alt())).is_err());
    assert_eq!(imu.config(), None);
    assert_eq!(run(imu.read()), Err(Error::NotConfigured));
    // `init` with a rejected configuration clears it too, before the first await.
    run(imu.configure(&mut delay, &alt())).unwrap();
    shared.arm(None, None);
    let bad = Config::new()
        .with_odr(OutputDataRate::Hz400)
        .with_averaging(Averaging::Avg8);
    assert!(matches!(
        run(imu.init(&mut delay, &bad)),
        Err(Error::InvalidConfig(ConfigError::AveragingTooHigh { .. }))
    ));
    assert_eq!(imu.config(), None);
}

// ---------------------------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------------------------

#[test]
fn invalid_config_sends_nothing() {
    let (shared, mut imu, mut delay) = rig();
    shared.arm(None, None);
    let bad = Config::new()
        .with_odr(OutputDataRate::Hz400)
        .with_averaging(Averaging::Avg8);
    let max = OutputDataRate::Hz400.max_averaging();
    assert_eq!(
        run(imu.init(&mut delay, &bad)),
        Err(Error::InvalidConfig(ConfigError::AveragingTooHigh {
            odr: OutputDataRate::Hz400,
            averaging: Averaging::Avg8,
            max
        }))
    );
    let none = Config::new().with_axes(Axes {
        x: false,
        y: false,
        z: false,
    });
    assert_eq!(
        run(imu.init(&mut delay, &none)),
        Err(Error::InvalidConfig(ConfigError::NoAxes))
    );
    assert_eq!(shared.transactions(), 0);
    assert_eq!(shared.awaits(), 0);
    // `configure` after an init also validates before it sends anything.
    let (shared, mut imu, mut delay) = up(&Config::new());
    assert_eq!(
        run(imu.configure(&mut delay, &none)),
        Err(Error::InvalidConfig(ConfigError::NoAxes))
    );
    assert_eq!(shared.transactions(), 0);
    assert_eq!(imu.config(), None);
}

#[test]
fn init_error_variants() {
    // Wrong chip identity.
    let (shared, mut imu, mut delay) = rig();
    shared.chip().chip_id = 0x42;
    assert_eq!(
        run(imu.init(&mut delay, &Config::new())),
        Err(Error::WrongId { found: 0x42 })
    );
    // OTP never reports done: status 0x10 is the reset value (8.x, p55).
    let (shared, mut imu, mut delay) = rig();
    shared.chip().otp_never_done = true;
    assert_eq!(
        run(imu.init(&mut delay, &Config::new())),
        Err(Error::OtpError { status: 0x10 })
    );
    // The magnetic reset echo is wrong.
    let (shared, mut imu, mut delay) = rig();
    shared.chip().status_xor = 0xE0;
    assert_eq!(
        run(imu.init(&mut delay, &Config::new())),
        Err(Error::ResetFailed { status: 0x00 })
    );
    // A transaction to another address is not acknowledged.
    let shared = Shared::new(A);
    let mut imu = Bmm350::new(shared.bus(), crate::ADDRESS_ADSEL_HIGH);
    let mut delay = shared.delay();
    assert_eq!(
        run(imu.init(&mut delay, &Config::new())),
        Err(Error::Bus(BusFault::NoAck))
    );
    assert_eq!(imu.config(), None);
}

// ---------------------------------------------------------------------------------------------
// PMU status checks
// ---------------------------------------------------------------------------------------------

/// The state a successful `init` or `configure` to `start` must leave on the fake.
fn assert_configured(shared: &Shared, start: &Config, illegal: usize, what: &str) {
    let status = shared.reg(reg::PMU_CMD_STATUS_0);
    let normal = start.start_mode == StartMode::Normal;
    assert_eq!(status & reg::PMU_STATUS_0_NORMAL != 0, normal, "{what}");
    assert_eq!(
        status & (reg::PMU_STATUS_0_ILLEGAL | reg::PMU_STATUS_0_BUSY),
        0,
        "{what}"
    );
    assert_eq!(
        shared.reg(reg::PMU_CMD_AGGR_SET),
        start.pmu_cmd_aggr_set(),
        "{what}"
    );
    if normal {
        // NM took AGGR_SET over: the effective rate and averaging (PMU_CMD_STATUS_1) are the image.
        assert_eq!(
            shared.reg(reg::PMU_CMD_STATUS_1),
            start.pmu_cmd_aggr_set(),
            "{what}"
        );
        assert_eq!(status >> 5, 1, "{what}");
    }
    let c = shared.chip();
    assert_eq!(
        (c.illegal_commands, c.dropped_commands),
        (illegal, 0),
        "{what}"
    );
    assert_eq!(c.violations, 0, "{what}");
}

#[test]
fn init_and_configure_succeed_under_the_measured_rules() {
    let starts = [
        Config::new(),
        alt(),
        suspend(),
        suspend().with_watchdog(I2cWatchdog::Ms1_28),
    ];
    for start in &starts {
        // `init` from a chip never initialised, one in normal mode, one in suspend.
        for from in [None, Some(Config::new()), Some(suspend())] {
            let (shared, mut imu, mut delay) = rig();
            if let Some(from) = from {
                run(imu.init(&mut delay, &from)).unwrap();
            }
            run(imu.init(&mut delay, start)).unwrap();
            assert_configured(
                &shared,
                start,
                0,
                &std::format!("init {start:?} from {from:?}"),
            );
        }
        // `configure` from a device in normal mode and from one in suspend.
        for from in [Config::new(), alt(), suspend()] {
            let (shared, mut imu, mut delay) = up(&from);
            run(imu.configure(&mut delay, start)).unwrap();
            assert_configured(
                &shared,
                start,
                0,
                &std::format!("configure {start:?} from {from:?}"),
            );
        }
    }
}

#[test]
fn busy_is_waited_out_within_the_bound_and_an_error_beyond() {
    // NM: bound 50 ms from the write, first poll at 15 ms, then every 1 ms. Busy until 49.9 ms
    // (15 ms plus 34.9 ms extra): the poll at 50 ms finds it clear.
    let (shared, mut imu, mut delay) = rig();
    shared.chip().extra_busy_us = 34_900;
    run(imu.init(&mut delay, &Config::new())).unwrap();
    assert_eq!(
        delays(&shared).iter().filter(|d| **d == 1_000_000).count(),
        35
    );
    // Busy until 50.001 ms: the poll at 50 ms is the last, and finds it busy. The status then is
    // the echo of FGR (5) and the busy flag.
    let (shared, mut imu, mut delay) = rig();
    shared.chip().extra_busy_us = 35_001;
    assert_eq!(
        run(imu.init(&mut delay, &Config::new())),
        Err(Error::PmuTimeout {
            cmd: reg::PMU_CMD_NM,
            status: 0xA1
        })
    );
    assert_eq!(imu.config(), None);
    assert_eq!(
        delays(&shared).iter().filter(|d| **d == 1_000_000).count(),
        35
    );
    // A command that never finishes gives the same error.
    let (shared, mut imu, mut delay) = rig();
    shared.chip().extra_busy_us = 1_000_000_000;
    assert!(matches!(
        run(imu.init(&mut delay, &Config::new())),
        Err(Error::PmuTimeout {
            cmd: reg::PMU_CMD_NM,
            status: 0xA1
        })
    ));
    // SUS: bound 20 ms. Busy until 20 ms (3 ms plus 17 ms extra) is accepted, 20.001 ms is not.
    let (shared, mut imu, mut delay) = up(&Config::new());
    shared.chip().extra_busy_us = 17_000;
    run(imu.configure(&mut delay, &suspend())).unwrap();
    let (shared, mut imu, mut delay) = up(&Config::new());
    shared.chip().extra_busy_us = 17_001;
    assert_eq!(
        run(imu.configure(&mut delay, &suspend())),
        Err(Error::PmuTimeout {
            cmd: reg::PMU_CMD_SUS,
            status: 0x29
        })
    );
    assert_eq!(imu.config(), None);
    // The next call works once the chip is quick again.
    shared.chip().extra_busy_us = 0;
    assert!(run(imu.configure(&mut delay, &Config::new())).is_ok());
    assert!(imu.config().is_some());
}

#[test]
fn wrong_echo_illegal_flag_and_wrong_mode_flag_are_rejected_with_the_status() {
    // Configure from normal mode: the echo of SUS must be 0; the fake shows 1.
    let (shared, mut imu, mut delay) = up(&Config::new());
    shared.chip().status_xor = 0x20;
    assert_eq!(
        run(imu.configure(&mut delay, &suspend())),
        Err(Error::PmuCommandRejected {
            cmd: reg::PMU_CMD_SUS,
            status: 0x20
        })
    );
    assert_eq!(imu.config(), None);

    let (shared, mut imu, mut delay) = up(&Config::new());
    shared.chip().status_or = reg::PMU_STATUS_0_ILLEGAL;
    assert_eq!(
        run(imu.configure(&mut delay, &suspend())),
        Err(Error::PmuCommandRejected {
            cmd: reg::PMU_CMD_SUS,
            status: 0x10
        })
    );
    // The same in `init`: the failing step is the NM.
    let (shared, mut imu, mut delay) = rig();
    shared.chip().status_or = reg::PMU_STATUS_0_ILLEGAL;
    assert_eq!(
        run(imu.init(&mut delay, &Config::new())),
        Err(Error::PmuCommandRejected {
            cmd: reg::PMU_CMD_NM,
            status: 0x38
        })
    );
    assert_eq!(imu.config(), None);
    // NM completed but the normal-mode flag is not set.
    let (shared, mut imu, mut delay) = rig();
    shared.chip().status_xor = reg::PMU_STATUS_0_NORMAL;
    assert_eq!(
        run(imu.init(&mut delay, &Config::new())),
        Err(Error::PmuCommandRejected {
            cmd: reg::PMU_CMD_NM,
            status: 0x20
        })
    );
}

#[test]
fn configure_recovers_a_device_with_the_illegal_flag_set() {
    // R3: the flag stays until a legal command completes. An FM sent in normal mode sets it; the
    // SUS of `configure` clears it.
    let (shared, mut imu, mut delay) = up(&Config::new());
    run(imu.write_register(reg::PMU_CMD, reg::PMU_CMD_FM)).unwrap();
    run(delay.delay_us(1_000));
    assert_eq!(
        run(imu.read_register(reg::PMU_CMD_STATUS_0)),
        Ok(0x20 | reg::PMU_STATUS_0_NORMAL | reg::PMU_STATUS_0_ILLEGAL)
    );
    run(imu.configure(&mut delay, &alt())).unwrap();
    assert_configured(&shared, &alt(), 1, "after an illegal FM");
}

// ---------------------------------------------------------------------------------------------
// Forced trigger and mode changes
// ---------------------------------------------------------------------------------------------

#[test]
fn commands_illegal_in_the_tracked_mode_are_refused_without_a_transaction() {
    let (shared, mut imu, mut delay) = up(&Config::new());
    let wrong = Err(Error::WrongMode {
        mode: PowerMode::Normal,
    });
    assert_eq!(run(imu.trigger_forced()), wrong);
    assert_eq!(
        run(imu.set_power_mode(&mut delay, PowerMode::Forced)),
        wrong
    );
    assert_eq!(
        run(imu.set_power_mode(&mut delay, PowerMode::ForcedFast)),
        wrong
    );
    // NM in normal mode is a no-op.
    assert_eq!(
        run(imu.set_power_mode(&mut delay, PowerMode::Normal)),
        Ok(())
    );
    assert_eq!((shared.transactions(), shared.awaits()), (0, 0));
    assert_eq!(shared.log(), Vec::new());
    assert!(imu.config().is_some());

    // Suspend from normal mode: SUS, 2 ms, poll busy, 1 ms, poll done.
    assert_eq!(
        run(imu.set_power_mode(&mut delay, PowerMode::Suspend)),
        Ok(())
    );
    assert_eq!(
        shared.log(),
        std::vec![
            w(A, 0x06, 0x00),
            Event::Delay(2_000_000),
            r(A, 0x07, 0x29),
            Event::Delay(1_000_000),
            r(A, 0x07, 0x00),
        ]
    );
    // Now SUS is the no-op and the forced trigger is legal: one write.
    shared.arm(None, None);
    assert_eq!(
        run(imu.set_power_mode(&mut delay, PowerMode::Suspend)),
        Ok(())
    );
    assert_eq!(shared.transactions(), 0);
    assert_eq!(run(imu.trigger_forced()), Ok(()));
    assert_eq!(shared.log(), std::vec![w(A, 0x06, 0x03)]);
    // Ready after the larger of vendor and measured time (averaging 2 here: 17 ms).
    let wait = crate::timing::forced_measurement_us(Averaging::Avg2, crate::timing::Recharge::Full);
    run(delay.delay_us(wait));
    assert_eq!(shared.reg(reg::PMU_CMD_STATUS_0), 0x60);
    assert_eq!(
        shared.reg(reg::PMU_CMD_STATUS_1),
        Config::new().pmu_cmd_aggr_set()
    );
    // Back to normal mode: NM, 15 ms, first poll finds it done.
    shared.arm(None, None);
    assert_eq!(
        run(imu.set_power_mode(&mut delay, PowerMode::Normal)),
        Ok(())
    );
    assert_eq!(
        shared.log(),
        std::vec![w(A, 0x06, 0x01), Event::Delay(15_000_000), r(A, 0x07, 0x28)]
    );
    assert_eq!(run(imu.trigger_forced()), wrong);
    assert_eq!(shared.transactions(), 2);
    assert_eq!(shared.chip().illegal_commands, 0);
}

#[test]
fn a_cancelled_or_failed_mode_change_leaves_the_driver_not_configured() {
    let (shared, mut imu, mut delay) = up(&Config::new());
    shared.arm(None, Some(2));
    assert_eq!(
        poll_then_drop(imu.set_power_mode(&mut delay, PowerMode::Suspend)),
        Polled::Cancelled
    );
    assert_eq!(imu.config(), None);
    shared.arm(None, None);
    run(imu.configure(&mut delay, &Config::new())).unwrap();
    shared.arm(Some(1), None);
    assert_eq!(
        run(imu.set_power_mode(&mut delay, PowerMode::Suspend)),
        Err(Error::Bus(BusFault::Injected))
    );
    assert_eq!(imu.config(), None);
}

// ---------------------------------------------------------------------------------------------
// Blocking front-end (A11) and Send (A5)
// ---------------------------------------------------------------------------------------------

#[test]
fn blocking_matches_async() {
    let a = Shared::new(A);
    let (mut imu, mut delay) = (Bmm350::new(a.bus(), A), a.delay());
    let async_out = (
        run(imu.init(&mut delay, &Config::new())),
        run(imu.read()),
        run(imu.configure(&mut delay, &alt())),
        run(imu.read_field()),
        run(imu.configure(&mut delay, &suspend())),
    );
    let b = Shared::new(A);
    let (mut imu, mut delay) = (crate::blocking::Bmm350::new(b.bus(), A), b.delay());
    let blocking_out = (
        imu.init(&mut delay, &Config::new()),
        imu.read(),
        imu.configure(&mut delay, &alt()),
        imu.read_field(),
        imu.configure(&mut delay, &suspend()),
    );
    assert!(async_out.0.is_ok() && async_out.2.is_ok() && async_out.3.is_ok());
    assert_eq!(imu.config(), Some(&suspend()));
    assert_eq!(async_out, blocking_out);
    assert_eq!(a.log(), b.log());
    assert_eq!(a.chip().regs, b.chip().regs);
}

fn assert_send<T: Send>(_: &T) {}

#[test]
fn driver_and_futures_are_send() {
    let (_shared, mut imu, mut delay) = rig();
    assert_send(&imu);
    assert_send(&imu.init(&mut delay, &Config::new()));
    assert_send(&imu.configure(&mut delay, &Config::new()));
    assert_send(&imu.read());
    assert_send(&imu.read_field());
    assert_send(&imu.verify());
    assert_send(&imu.recover(&mut delay));
    assert_send(&imu.pmu_status());
    assert_send(&imu.error_status());
    assert_send(&imu.trigger_forced_fast());
    assert_send(&imu.trigger_forced());
    assert_send(&imu.set_power_mode(&mut delay, PowerMode::Suspend));
    assert_send(&imu.magnetic_reset(&mut delay));
    assert_send(&imu.sensor_time());
    assert_send(&imu.interrupt_status());
    assert_send(&imu.self_test(&mut delay));
    assert_send(&imu.magnetic_reset_fast(&mut delay));
    assert_send(&imu.read_with_time());
}

// ---------------------------------------------------------------------------------------------
// verify, recover, typed status reads, forced fast trigger
// ---------------------------------------------------------------------------------------------

/// Reads of the one-register kind, in the order `verify` makes them for the default
/// configuration tracking normal mode: `PMU_CMD_AGGR_SET` 0x14 (8.4, p30), `PMU_CMD_AXIS_EN` 0x07
/// (8.5, p31), `INT_CTRL` 0x8E (8.11, p35), `I2C_WDT_SET` 0x00 (8.10, p34), `PMU_CMD_STATUS_0`
/// 0x28 (normal-mode flag and echo of `NM`, 8.7, p33), `PMU_CMD_STATUS_1` 0x14 (8.8, p34).
fn verify_reads_default() -> Vec<Event> {
    std::vec![
        r(A, 0x04, 0x14),
        r(A, 0x05, 0x07),
        r(A, 0x2E, 0x8E),
        r(A, 0x0A, 0x00),
        r(A, 0x60, 0x00),
        r(A, 0x07, 0x28),
        r(A, 0x08, 0x14),
    ]
}

fn unlike_configs() -> [Config; 7] {
    [
        Config::new(),
        alt(),
        suspend(),
        Config::new()
            .with_odr(OutputDataRate::Hz400)
            .with_averaging(Averaging::Avg1)
            .with_axes(Axes {
                x: true,
                y: false,
                z: true,
            })
            .with_interrupt(
                InterruptPin::new()
                    .with_mode(InterruptMode::Latched)
                    .with_drive(InterruptDrive::OpenDrain)
                    .with_polarity(InterruptPolarity::ActiveLow),
            )
            .with_watchdog(I2cWatchdog::Ms40_96),
        Config::new()
            .with_odr(OutputDataRate::Hz25)
            .with_averaging(Averaging::Avg8)
            .with_start_mode(StartMode::Suspend)
            .with_interrupt(InterruptPin::new().with_data_ready(false))
            .with_watchdog(I2cWatchdog::Ms40_96),
        Config::new()
            .with_odr(OutputDataRate::Hz1_5625)
            .with_averaging(Averaging::Avg8)
            .with_axes(Axes {
                x: false,
                y: true,
                z: false,
            }),
        Config::new()
            .with_odr(OutputDataRate::Hz200)
            .with_averaging(Averaging::Avg2)
            .with_start_mode(StartMode::Suspend)
            .with_watchdog(I2cWatchdog::Ms1_28),
    ]
}

fn count_for(config: &Config) -> usize {
    if config.start_mode == StartMode::Normal {
        7
    } else {
        6
    }
}

fn assert_unchanged(imu: &Drv, config: &Config, mode: PowerMode, what: &str) {
    assert_eq!(imu.config(), Some(config), "{what}");
    assert_eq!(imu.mode, mode, "{what}");
}

fn mismatch(register: u8, mask: u8, expected: u8, found: u8) -> Res<()> {
    Err(Error::ConfigMismatch(Mismatch {
        register,
        mask,
        expected,
        found,
    }))
}

#[test]
fn verify_owned_entries_for_the_default_config_follow_the_images() {
    let normal = Config::new().owned_registers(PowerMode::Normal);
    let got: Vec<(u8, u8, u8)> = normal
        .entries()
        .iter()
        .map(|o| (o.reg, o.mask, o.expected))
        .collect();
    assert_eq!(
        got,
        std::vec![
            (0x04, 0x3F, 0x14),
            (0x05, 0x07, 0x07),
            (0x2E, 0x87, 0x86),
            (0x0A, 0x03, 0x00),
            (0x60, 0x1F, 0x00),
            (0x07, 0x08, 0x08),
            (0x08, 0x3F, 0x14),
        ]
    );
    let suspended = Config::new().owned_registers(PowerMode::Suspend);
    assert_eq!(suspended.entries().len(), 6);
    assert_eq!(suspended.entries().get(5).map(|o| o.expected), Some(0));
    assert_eq!(&suspended.entries()[..5], &normal.entries()[..5]);
    // The interrupt off: INT_CTRL is the reset value.
    let off = alt().owned_registers(PowerMode::Normal);
    assert_eq!(off.entries().get(2).map(|o| o.expected), Some(0));
}

#[test]
fn verify_default_golden_trace() {
    let (shared, mut imu, _delay) = up(&Config::new());
    assert_eq!(run(imu.verify()), Ok(()));
    assert_eq!(shared.log(), verify_reads_default());
    assert_eq!((shared.transactions(), shared.awaits()), (7, 7));
    assert_eq!(shared.violations(), 0);
}

#[test]
fn verify_not_configured_sends_nothing() {
    let (shared, mut imu, mut delay) = rig();
    assert_eq!(run(imu.verify()), Err(Error::NotConfigured));
    run(imu.init(&mut delay, &Config::new())).unwrap();
    run(imu.configure(
        &mut delay,
        &Config::new().with_axes(Axes {
            x: false,
            y: false,
            z: false,
        }),
    ))
    .unwrap_err();
    shared.arm(None, None);
    assert_eq!(run(imu.verify()), Err(Error::NotConfigured));
    assert_eq!((shared.transactions(), shared.awaits()), (0, 0));
}

#[test]
fn verify_is_ok_after_init_and_after_configure_in_the_documented_count() {
    for config in unlike_configs() {
        for first in [Config::new(), suspend()] {
            let (shared, mut imu, mut delay) = up(&first);
            run(imu.configure(&mut delay, &config)).unwrap();
            shared.arm(None, None);
            assert_eq!(run(imu.verify()), Ok(()), "{config:?}");
            assert_eq!(shared.transactions(), count_for(&config), "{config:?}");
            assert_eq!(shared.awaits() as usize, count_for(&config));
            assert!(
                shared
                    .log()
                    .iter()
                    .all(|e| matches!(e, Event::I2c { ops, .. }
                if matches!(ops.as_slice(), [Op::Write(p), Op::Read(_)] if p.len() == 1)))
            );
        }
        let (shared, mut imu, _delay) = up(&config);
        assert_eq!(run(imu.verify()), Ok(()), "{config:?}");
        assert_eq!(shared.transactions(), count_for(&config), "{config:?}");
    }
}

#[test]
fn verify_detects_every_masked_bit_and_ignores_every_other_bit() {
    for config in unlike_configs() {
        let (shared, mut imu, _delay) = up(&config);
        let mode = imu.mode;
        let list = config.owned_registers(mode);
        let entries = list.entries();
        assert_eq!(entries.len(), count_for(&config));
        for (i, o) in entries.iter().enumerate() {
            for bit in 0..8u8 {
                let flip = 1u8 << bit;
                shared.arm(None, None);
                shared.chip().regs[usize::from(o.reg)] ^= flip;
                let found = shared.chip().regs[usize::from(o.reg)];
                let got = run(imu.verify());
                if o.mask & flip != 0 {
                    assert_eq!(
                        got,
                        mismatch(o.reg, o.mask, o.expected, found),
                        "{config:?} reg {:#04x} bit {bit}",
                        o.reg
                    );
                    assert_eq!(shared.transactions(), i + 2);
                } else {
                    assert_eq!(got, Ok(()), "{config:?} reg {:#04x} bit {bit}", o.reg);
                    assert_eq!(shared.transactions(), entries.len());
                }
                shared.chip().regs[usize::from(o.reg)] ^= flip;
                assert_unchanged(&imu, &config, mode, "after a flip");
            }
        }
        for addr in 0..0x80u8 {
            if entries.iter().any(|o| o.reg == addr) {
                continue;
            }
            shared.chip().regs[usize::from(addr)] ^= 0xFF;
            assert_eq!(run(imu.verify()), Ok(()), "{config:?} reg {addr:#04x}");
            shared.chip().regs[usize::from(addr)] ^= 0xFF;
        }
        assert_eq!(run(imu.verify()), Ok(()));
    }
}

#[test]
fn verify_reports_a_mode_changed_behind_the_drivers_back() {
    // Normal tracked, the chip put into suspend: SUS is legal in normal mode (8.6, p32).
    let (shared, mut imu, mut delay) = up(&Config::new());
    run(imu.write_register(reg::PMU_CMD, reg::PMU_CMD_SUS)).unwrap();
    run(delay.delay_us(5_000));
    shared.arm(None, None);
    assert_eq!(run(imu.verify()), mismatch(0x07, 0x08, 0x08, 0x00));
    assert_eq!(shared.transactions(), 7);
    assert_unchanged(&imu, &Config::new(), PowerMode::Normal, "normal tracked");
    // The reverse: suspend tracked, the chip put into normal mode.
    let (shared, mut imu, mut delay) = up(&suspend());
    run(imu.write_register(reg::PMU_CMD, reg::PMU_CMD_NM)).unwrap();
    run(delay.delay_us(20_000));
    shared.arm(None, None);
    assert_eq!(run(imu.verify()), mismatch(0x07, 0x08, 0x00, 0x28));
    assert_eq!(shared.transactions(), 7);
    assert_unchanged(&imu, &suspend(), PowerMode::Suspend, "suspend tracked");
}

#[test]
fn verify_reports_a_different_effective_rate_only_in_normal_mode() {
    let (shared, mut imu, _delay) = up(&Config::new());
    shared.chip().regs[0x08] = 0x13;
    assert_eq!(run(imu.verify()), mismatch(0x08, 0x3F, 0x14, 0x13));
    let (shared, mut imu, _delay) = up(&suspend());
    shared.chip().regs[0x08] = 0x13;
    assert_eq!(run(imu.verify()), Ok(()));
}

#[test]
fn verify_leaves_the_illegal_flag_and_the_error_register_alone() {
    let (shared, mut imu, mut delay) = up(&Config::new());
    // FM is illegal in normal mode (measured on the board, 2026-11-10).
    run(imu.write_register(reg::PMU_CMD, reg::PMU_CMD_FM)).unwrap();
    run(delay.delay_us(1_000));
    shared.chip().regs[usize::from(reg::ERR_REG)] = 1;
    shared.arm(None, None);
    assert_eq!(run(imu.verify()), Ok(()));
    assert_eq!(shared.transactions(), 7);
    let status = run(imu.pmu_status()).unwrap();
    assert!(status.illegal_command && status.normal_mode);
    assert_eq!(
        run(imu.error_status()),
        Ok(ErrorStatus {
            pmu_cmd_error: true,
            raw: 1
        })
    );
}

#[test]
fn verify_one_off_corrupted_read_is_ok_with_one_extra_transaction() {
    // `PMU_CMD_STATUS_1` (the effective rate and averaging) is read only when normal mode is tracked.
    for (config, count, regs) in [
        (
            Config::new(),
            7,
            &[0x04, 0x05, 0x2E, 0x0A, 0x60, 0x07, 0x08][..],
        ),
        (suspend(), 6, &[0x04, 0x05, 0x2E, 0x0A, 0x60, 0x07][..]),
    ] {
        for &reg_addr in regs {
            let (shared, mut imu, _delay) = up(&config);
            shared.chip().glitch = Some(Glitch {
                reg: reg_addr,
                // The normal-mode flag for `PMU_CMD_STATUS_0`, the lowest bit elsewhere.
                xor: if reg_addr == 0x07 { 0x08 } else { 0x01 },
                skip: 0,
            });
            assert_eq!(run(imu.verify()), Ok(()), "reg {reg_addr:#04x}");
            assert_eq!(shared.transactions(), count + 1, "reg {reg_addr:#04x}");
            assert!(shared.chip().glitch.is_none());
        }
    }
}

#[test]
fn verify_and_recover_never_change_the_state_in_any_outcome() {
    let config = alt();
    let (shared, mut imu, mut delay) = up(&config);
    let mode = imu.mode;
    assert_eq!(run(imu.verify()), Ok(()));
    assert_unchanged(&imu, &config, mode, "ok");
    shared.chip().regs[0x05] ^= 1;
    assert!(matches!(run(imu.verify()), Err(Error::ConfigMismatch(_))));
    assert_unchanged(&imu, &config, mode, "mismatch");
    assert!(matches!(
        run(imu.recover(&mut delay)),
        Err(Error::ConfigMismatch(_))
    ));
    assert_unchanged(&imu, &config, mode, "recover mismatch");
    shared.chip().regs[0x05] ^= 1;
    shared.arm(Some(2), None);
    assert_eq!(run(imu.verify()), Err(Error::Bus(BusFault::Injected)));
    assert_unchanged(&imu, &config, mode, "bus error");
    shared.chip().regs[0] = 0x34;
    shared.arm(None, None);
    assert_eq!(
        run(imu.recover(&mut delay)),
        Err(Error::WrongId { found: 0x34 })
    );
    assert_unchanged(&imu, &config, mode, "wrong id");
    shared.chip().regs[0] = 0x33;
    assert_eq!(run(imu.read()), Ok(expected_raw()));
}

#[test]
fn recover_default_golden_trace() {
    let (shared, mut imu, mut delay) = up(&Config::new());
    assert_eq!(run(imu.recover(&mut delay)), Ok(()));
    // Quiet 2 ms; CHIP_ID (8.1, p29) read three times (the first discarded); then verify.
    let mut expected = std::vec![
        Event::Delay(2_000_000),
        r(A, 0x00, 0x33),
        r(A, 0x00, 0x33),
        r(A, 0x00, 0x33),
    ];
    expected.extend(verify_reads_default());
    assert_eq!(shared.log(), expected);
    assert_eq!((shared.transactions(), shared.awaits()), (10, 11));
    assert_eq!(shared.violations(), 0);
    let (shared, mut imu, mut delay) = up(&suspend());
    assert_eq!(run(imu.recover(&mut delay)), Ok(()));
    assert_eq!(shared.transactions(), 9);
    assert!(
        shared
            .log()
            .iter()
            .all(|e| !matches!(e, Event::I2c { ops, .. }
        if matches!(ops.first(), Some(Op::Write(b)) if b.len() > 1)))
    );
}

#[test]
fn recover_not_configured_checks_the_identity_then_says_so() {
    let (shared, mut imu, mut delay) = rig();
    shared.arm(None, None);
    assert_eq!(run(imu.recover(&mut delay)), Err(Error::NotConfigured));
    assert_eq!(
        shared.log(),
        std::vec![
            Event::Delay(2_000_000),
            r(A, 0x00, 0x33),
            r(A, 0x00, 0x33),
            r(A, 0x00, 0x33),
        ]
    );
    assert_eq!(imu.config(), None);
}

#[test]
fn recover_wrong_identity_on_the_first_or_the_second_check() {
    // The discarded read may be wrong; the first check wrong; the second check wrong.
    for (skip, outcome, count) in [
        (0, Ok(()), 10),
        (1, Err(Error::WrongId { found: 0x32 }), 2),
        (2, Err(Error::WrongId { found: 0x32 }), 3),
    ] {
        let (shared, mut imu, mut delay) = up(&Config::new());
        shared.chip().glitch = Some(Glitch {
            reg: reg::CHIP_ID,
            xor: 0x01,
            skip,
        });
        assert_eq!(run(imu.recover(&mut delay)), outcome, "skip {skip}");
        assert_eq!(shared.transactions(), count, "skip {skip}");
        assert_unchanged(&imu, &Config::new(), PowerMode::Normal, "wrong id");
    }
}

#[test]
fn pmu_status_reads_both_registers_in_one_transaction_configured_or_not() {
    let (shared, mut imu, _delay) = up(&Config::new());
    assert_eq!(
        run(imu.pmu_status()),
        Ok(PmuStatus {
            busy: false,
            illegal_command: false,
            normal_mode: true,
            command: 1,
            odr_code: 4,
            avg_code: 1,
            status_0: 0x28,
            status_1: 0x14,
        })
    );
    assert_eq!(
        shared.log(),
        std::vec![Event::I2c {
            address: A,
            ops: std::vec![
                Op::Write(std::vec![0x07]),
                Op::Read(std::vec![DUMMY, DUMMY, 0x28, 0x14])
            ],
        }]
    );
    let (shared, mut imu, _delay) = rig();
    shared.arm(None, None);
    assert_eq!(run(imu.pmu_status()), Ok(PmuStatus::default()));
    assert_eq!(shared.transactions(), 1);
}

#[test]
fn pmu_status_from_bytes_decodes_every_field() {
    assert_eq!(PmuStatus::default(), PmuStatus::from_bytes(0, 0));
    assert_eq!(
        PmuStatus::from_bytes(0xF9, 0x35),
        PmuStatus {
            busy: true,
            illegal_command: true,
            normal_mode: true,
            command: 7,
            odr_code: 5,
            avg_code: 3,
            status_0: 0xF9,
            status_1: 0x35,
        }
    );
}

#[test]
fn error_status_reads_err_reg_in_one_transaction_configured_or_not() {
    let (shared, mut imu, _delay) = rig();
    shared.chip().regs[usize::from(reg::ERR_REG)] = 0x01;
    shared.arm(None, None);
    assert_eq!(
        run(imu.error_status()),
        Ok(ErrorStatus {
            pmu_cmd_error: true,
            raw: 0x01
        })
    );
    assert_eq!(shared.log(), std::vec![r(A, 0x02, 0x01)]);
    assert_eq!(ErrorStatus::default(), ErrorStatus::from_bits(0));
    assert!(!ErrorStatus::from_bits(0xFE).pmu_cmd_error);
}

#[test]
fn trigger_forced_fast_writes_fm_fast_and_the_data_is_ready_after_the_documented_time() {
    for avg in [
        Averaging::Avg1,
        Averaging::Avg2,
        Averaging::Avg4,
        Averaging::Avg8,
    ] {
        let config = suspend().with_odr(OutputDataRate::Hz25).with_averaging(avg);
        let (shared, mut imu, mut delay) = up(&config);
        assert_eq!(run(imu.trigger_forced_fast()), Ok(()));
        // FM_FAST is code 4 (8.6, p32).
        assert_eq!(shared.log(), std::vec![w(A, 0x06, 0x04)]);
        let wait = crate::timing::forced_measurement_us(avg, crate::timing::Recharge::Fast);
        if matches!(avg, Averaging::Avg2 | Averaging::Avg4) {
            // Not measured on the board: the fake's busy time is an interpolation (5.2 ms for
            // averaging 2, above the vendor 5 ms), so only the write is checked.
            continue;
        }
        if avg == Averaging::Avg1 {
            // Busy for 3.4 ms (measured on the board, 2026-11-10), the vendor figure is 4 ms.
            run(delay.delay_us(3_300));
            assert!(run(imu.pmu_status()).unwrap().busy);
            run(delay.delay_us(wait - 3_300));
        } else {
            run(delay.delay_us(wait));
        }
        let status = run(imu.pmu_status()).unwrap();
        assert!(
            !status.busy && !status.illegal_command && !status.normal_mode,
            "{avg:?} {status:?}"
        );
        assert_eq!(status.command, 4);
        assert_eq!(status.status_1, config.pmu_cmd_aggr_set());
        assert_eq!(shared.chip().illegal_commands, 0);
    }
}

#[test]
fn trigger_forced_fast_is_refused_in_normal_mode_and_when_not_configured() {
    let (shared, mut imu, _delay) = up(&Config::new());
    assert_eq!(
        run(imu.trigger_forced_fast()),
        Err(Error::WrongMode {
            mode: PowerMode::Normal
        })
    );
    assert_eq!((shared.transactions(), shared.awaits()), (0, 0));
    let (shared, mut imu, _delay) = rig();
    shared.arm(None, None);
    assert_eq!(run(imu.trigger_forced_fast()), Err(Error::NotConfigured));
    assert_eq!((shared.transactions(), shared.awaits()), (0, 0));
}

#[derive(Clone, Copy, Debug)]
enum Q {
    Verify,
    Recover,
    PmuStatus,
    ErrorStatus,
    TriggerFast,
    SensorTime,
}

#[derive(Debug, PartialEq)]
enum Got {
    Time(u32),
    Unit,
    Pmu(PmuStatus),
    Err(ErrorStatus),
}

async fn ask(imu: &mut Drv, delay: &mut TestDelay, q: Q) -> Res<Got> {
    Ok(match q {
        Q::Verify => imu.verify().await.map(|()| Got::Unit)?,
        Q::Recover => imu.recover(delay).await.map(|()| Got::Unit)?,
        Q::PmuStatus => Got::Pmu(imu.pmu_status().await?),
        Q::ErrorStatus => Got::Err(imu.error_status().await?),
        Q::TriggerFast => imu.trigger_forced_fast().await.map(|()| Got::Unit)?,
        Q::SensorTime => Got::Time(imu.sensor_time().await?),
    })
}

fn sweep_new_methods(q: Q, config: &Config, expect_ok: bool) {
    let want = {
        let (_shared, mut imu, mut delay) = up(config);
        run(ask(&mut imu, &mut delay, q))
    };
    assert_eq!(want.is_ok(), expect_ok, "{q:?}");
    let mode = up(config).1.mode;
    for k in 1.. {
        let (shared, mut imu, mut delay) = up(config);
        shared.arm(Some(k), None);
        let out = run(ask(&mut imu, &mut delay, q));
        if shared.transactions() < k {
            assert_eq!(out, want, "{q:?}");
            break;
        }
        assert_eq!(out, Err(Error::Bus(BusFault::Injected)), "{q:?} at {k}");
        assert_unchanged(&imu, config, mode, "bus error");
        shared.arm(None, None);
        assert_eq!(run(ask(&mut imu, &mut delay, q)), want, "{q:?} at {k}");
    }
    for k in 1.. {
        let (shared, mut imu, mut delay) = up(config);
        shared.arm(None, Some(k));
        if let Polled::Done(out) = poll_then_drop(ask(&mut imu, &mut delay, q)) {
            assert_eq!(out, want, "{q:?}");
            break;
        }
        assert_unchanged(&imu, config, mode, "cancel");
        shared.arm(None, None);
        assert_eq!(
            run(ask(&mut imu, &mut delay, q)),
            want,
            "{q:?} at await {k}"
        );
    }
}

#[test]
fn verify_and_recover_and_the_status_reads_survive_a_bus_error_and_a_cancel_anywhere() {
    for config in [Config::new(), suspend(), alt()] {
        for q in [
            Q::Verify,
            Q::Recover,
            Q::PmuStatus,
            Q::ErrorStatus,
            Q::SensorTime,
        ] {
            sweep_new_methods(q, &config, true);
        }
    }
    sweep_new_methods(Q::TriggerFast, &suspend(), true);
    // From the wrong mode the refusal is the same answer with no transaction to disturb.
    sweep_new_methods(Q::TriggerFast, &Config::new(), false);
}

#[test]
fn trigger_forced_fast_after_a_failed_write_sends_one_command() {
    let (shared, mut imu, _delay) = up(&suspend());
    shared.arm(Some(1), None);
    assert_eq!(
        run(imu.trigger_forced_fast()),
        Err(Error::Bus(BusFault::Injected))
    );
    assert_eq!(shared.chip().illegal_commands, 0);
    shared.arm(None, None);
    assert_eq!(run(imu.trigger_forced_fast()), Ok(()));
    assert_eq!(shared.log(), std::vec![w(A, 0x06, 0x04)]);
}

#[test]
fn verify_and_recover_and_the_status_reads_through_blocking_match_the_async_driver() {
    let a = Shared::new(A);
    let (mut imu, mut delay) = (Bmm350::new(a.bus(), A), a.delay());
    let async_out = (
        run(imu.init(&mut delay, &Config::new())),
        run(imu.verify()),
        run(imu.pmu_status()),
        run(imu.error_status()),
        run(imu.recover(&mut delay)),
        run(imu.configure(&mut delay, &suspend())),
        run(imu.trigger_forced_fast()),
        run(imu.verify()),
    );
    let b = Shared::new(A);
    let (mut imu, mut delay) = (crate::blocking::Bmm350::new(b.bus(), A), b.delay());
    let blocking_out = (
        imu.init(&mut delay, &Config::new()),
        imu.verify(),
        imu.pmu_status(),
        imu.error_status(),
        imu.recover(&mut delay),
        imu.configure(&mut delay, &suspend()),
        imu.trigger_forced_fast(),
        imu.verify(),
    );
    assert!(async_out.1.is_ok() && async_out.4.is_ok() && async_out.6.is_ok());
    assert_eq!(async_out, blocking_out);
    assert_eq!(a.log(), b.log());
    assert_eq!(a.chip().regs, b.chip().regs);
}

// ---------------------------------------------------------------------------------------------
// The stall of a latched interrupt (measured on the board, 2026-11-10), sensor time, mode changes
// ---------------------------------------------------------------------------------------------

/// 400 Hz, averaging 1, latched data-ready interrupt, normal mode.
fn latched() -> Config {
    Config::new()
        .with_odr(OutputDataRate::Hz400)
        .with_averaging(Averaging::Avg1)
        .with_interrupt(InterruptPin::new().with_mode(InterruptMode::Latched))
}

const PERIOD_400_HZ_US: u64 = 2_500;

/// Starts a stream at 400 Hz and lets `ms` pass: the latched flag is set and the sensor time runs.
fn stream_for(shared: &Shared, delay: &mut TestDelay, ms: u32) {
    shared.chip().start_stream(PERIOD_400_HZ_US);
    run(delay.delay_us(ms * 1_000));
    // Any register access brings the stream up to date.
    shared.reg(reg::CHIP_ID);
}

#[test]
fn configure_never_stalls_the_chip_when_it_leaves_a_latched_interrupt() {
    let targets = [
        alt(),
        suspend(),
        Config::new(),
        latched().with_interrupt(
            InterruptPin::new()
                .with_mode(InterruptMode::Latched)
                .with_polarity(InterruptPolarity::ActiveLow),
        ),
        latched().with_start_mode(StartMode::Suspend),
    ];
    for start in [latched(), latched().with_start_mode(StartMode::Suspend)] {
        for target in targets {
            let running = start.start_mode == StartMode::Normal;
            let (shared, mut imu, mut delay) = up(&start);
            if running {
                stream_for(&shared, &mut delay, 60);
            } else {
                shared.chip().sample();
            }
            assert!(shared.chip().int_flag);
            let info = run(imu.configure(&mut delay, &target)).unwrap();
            assert!(!shared.chip().stalled, "{running} {target:?}");
            assert!(info.data_ready_consumed, "{running} {target:?}");
            assert_eq!(imu.config(), Some(&target));
            if target.start_mode == StartMode::Normal {
                shared.chip().start_stream(PERIOD_400_HZ_US);
                let t0 = run(imu.sensor_time()).unwrap();
                run(delay.delay_us(30_000));
                let t1 = run(imu.sensor_time()).unwrap();
                assert!(
                    crate::timing::sensor_elapsed_us(t0, t1) > 0,
                    "{running} {target:?}"
                );
            }
        }
    }
}

#[test]
fn a_raw_reconfiguration_without_the_read_stalls_the_chip_and_only_init_recovers_it() {
    let (shared, mut imu, mut delay) = up(&latched());
    stream_for(&shared, &mut delay, 60);
    // SUS, then INT_CTRL pulsed with the flag still set: what a bare register write sequence does.
    run(imu.write_register(reg::PMU_CMD, reg::PMU_CMD_SUS)).unwrap();
    run(delay.delay_us(4_000));
    run(imu.write_register(reg::INT_CTRL, reg::INT_CTRL_DRDY_EN)).unwrap();
    assert!(shared.chip().stalled);
    // Reading INT_STATUS afterwards and a second `configure` do not help (the second one has no
    // latched configuration to leave, as in the measurement).
    run(imu.interrupt_status()).unwrap();
    run(imu.configure(&mut delay, &alt())).unwrap();
    shared.chip().start_stream(PERIOD_400_HZ_US);
    let t0 = run(imu.sensor_time()).unwrap();
    run(delay.delay_us(30_000));
    assert_eq!(run(imu.sensor_time()), Ok(t0));
    assert!(shared.chip().stalled);
    // A full init does.
    run(imu.init(&mut delay, &alt())).unwrap();
    assert!(!shared.chip().stalled);
    shared.chip().start_stream(PERIOD_400_HZ_US);
    run(delay.delay_us(30_000));
    assert_ne!(run(imu.sensor_time()), Ok(t0));
}

#[test]
fn reading_int_status_before_suspend_is_not_enough_the_read_must_follow_the_last_conversion() {
    let (shared, mut imu, mut delay) = up(&latched());
    stream_for(&shared, &mut delay, 60);
    run(imu.read_register(reg::INT_STATUS)).unwrap();
    run(imu.write_register(reg::PMU_CMD, reg::PMU_CMD_SUS)).unwrap();
    // SUS is busy 3 ms and converts meanwhile: the flag is set again before the chip stops.
    run(delay.delay_us(4_000));
    shared.reg(reg::CHIP_ID);
    assert!(shared.chip().int_flag);
    run(imu.write_register(reg::INT_CTRL, reg::INT_CTRL_DRDY_EN)).unwrap();
    assert!(shared.chip().stalled);
}

#[test]
fn configure_from_a_latched_interrupt_golden_trace_and_counts() {
    let (shared, mut imu, mut delay) = up(&latched());
    shared.chip().sample();
    let info = run(imu.configure(&mut delay, &alt())).unwrap();
    assert_eq!(
        info,
        Info {
            chip_id: 0x33,
            output_period_us: 10_000,
            averaging: Averaging::Avg4,
            data_ready_consumed: true,
        }
    );
    // The pulsed trace of `configure_golden_traces` with one more read, INT_STATUS (0x30, flag
    // 0x04), after SUS has completed and before the first write.
    assert_eq!(
        shared.log(),
        std::vec![
            r(A, 0x07, 0x28),
            w(A, 0x06, 0x00),
            Event::Delay(2_000_000),
            r(A, 0x07, 0x29),
            Event::Delay(1_000_000),
            r(A, 0x07, 0x00),
            r(A, 0x30, 0x04),
            w(A, 0x04, 0x24),
            w(A, 0x05, 0x07),
            w(A, 0x2E, 0x00),
            w(A, 0x0A, 0x01),
            w(A, 0x60, 0x00),
            w(A, 0x06, 0x01),
            Event::Delay(15_000_000),
            r(A, 0x07, 0x28),
        ]
    );
    assert_eq!((shared.transactions(), shared.awaits()), (12, 15));
    // The latched configuration is gone: the next configure reads nothing.
    shared.arm(None, None);
    let info = run(imu.configure(&mut delay, &alt())).unwrap();
    assert!(!info.data_ready_consumed);
    assert_eq!((shared.transactions(), shared.awaits()), (11, 14));
    // Latched and suspended: the status read, the INT_STATUS read (no flag), the plan.
    let (shared, mut imu, mut delay) = up(&latched().with_start_mode(StartMode::Suspend));
    let info = run(imu.configure(&mut delay, &suspend())).unwrap();
    assert!(!info.data_ready_consumed);
    assert_eq!(shared.transactions(), 7);
    assert_eq!(shared.log().get(1), Some(&r(A, 0x30, 0x00)));
}

/// Bus error and cancel at every point of a mode change that ends in `to`, from `start` with the
/// latched flag set or not: the counts are discovered, every disturbed call leaves the driver not
/// configured and `configure` brings it back. Returns (transactions, awaits).
fn sweep_mode_change(start: &Config, to: PowerMode) -> (usize, u32) {
    let make = || {
        let (shared, imu, delay) = up(start);
        shared.chip().sample();
        (shared, imu, delay)
    };
    let check_back = |shared: &Shared, imu: &mut Drv, delay: &mut TestDelay| {
        assert_eq!(imu.config(), None);
        shared.arm(None, None);
        run(imu.configure(delay, start)).unwrap();
        assert!(!shared.chip().stalled);
        assert_eq!(run(imu.verify()), Ok(()));
    };
    let mut transactions = 0;
    for k in 1.. {
        let (shared, mut imu, mut delay) = make();
        shared.arm(Some(k), None);
        let out = run(imu.set_power_mode(&mut delay, to));
        if shared.transactions() < k {
            assert_eq!(out, Ok(()));
            assert!(imu.config().is_some());
            transactions = k - 1;
            break;
        }
        assert_eq!(out, Err(Error::Bus(BusFault::Injected)), "bus error at {k}");
        check_back(&shared, &mut imu, &mut delay);
    }
    let mut awaits = 0;
    for k in 1.. {
        let (shared, mut imu, mut delay) = make();
        shared.arm(None, Some(k));
        if let Polled::Done(out) = poll_then_drop(imu.set_power_mode(&mut delay, to)) {
            assert_eq!(out, Ok(()));
            awaits = k - 1;
            break;
        }
        check_back(&shared, &mut imu, &mut delay);
    }
    (transactions, awaits)
}

#[test]
fn set_power_mode_survives_a_bus_error_and_a_cancel_anywhere() {
    let latched_suspend = latched().with_start_mode(StartMode::Suspend);
    // Suspend from normal: SUS, one busy poll, the done poll (3 transactions, 2 delays); latched
    // adds the INT_STATUS read (4 transactions, 6 awaits).
    assert_eq!(
        sweep_mode_change(&Config::new(), PowerMode::Suspend),
        (3, 5)
    );
    assert_eq!(sweep_mode_change(&latched(), PowerMode::Suspend), (4, 6));
    // Normal from suspend: NM, 15 ms, the done poll; no read, latched or not.
    assert_eq!(sweep_mode_change(&suspend(), PowerMode::Normal), (2, 3));
    assert_eq!(
        sweep_mode_change(&latched_suspend, PowerMode::Normal),
        (2, 3)
    );
}

#[test]
fn a_configure_that_fails_after_the_suspend_command_leaves_the_tracked_mode_suspend() {
    let mut after_sus = 0;
    for k in 1.. {
        let (shared, mut imu, mut delay) = up(&Config::new());
        assert_eq!(imu.mode, PowerMode::Normal);
        shared.arm(Some(k), None);
        let out = run(imu.configure(&mut delay, &suspend()));
        if shared.transactions() < k {
            assert!(out.is_ok());
            break;
        }
        assert_eq!(out, Err(Error::Bus(BusFault::Injected)), "bus error at {k}");
        let chip_mode = if shared.chip().normal {
            PowerMode::Normal
        } else {
            PowerMode::Suspend
        };
        assert_eq!(imu.mode, chip_mode, "bus error at {k}");
        if chip_mode == PowerMode::Suspend {
            after_sus += 1;
        }
        shared.arm(None, None);
        run(imu.configure(&mut delay, &suspend())).unwrap();
        assert_eq!(imu.mode, PowerMode::Suspend);
        assert_eq!(run(imu.verify()), Ok(()));
    }
    assert!(after_sus > 0);
}

#[test]
fn set_power_mode_suspend_from_a_latched_interrupt_reads_int_status_after_sus() {
    let (shared, mut imu, mut delay) = up(&latched());
    stream_for(&shared, &mut delay, 60);
    shared.arm(None, None);
    assert_eq!(
        run(imu.set_power_mode(&mut delay, PowerMode::Suspend)),
        Ok(())
    );
    assert_eq!(
        shared.log(),
        std::vec![
            w(A, 0x06, 0x00),
            Event::Delay(2_000_000),
            r(A, 0x07, 0x29),
            Event::Delay(1_000_000),
            r(A, 0x07, 0x00),
            r(A, 0x30, 0x04),
        ]
    );
    assert!(!shared.chip().int_flag);
    // Back in normal mode and out of the latched configuration without a stall.
    run(imu.set_power_mode(&mut delay, PowerMode::Normal)).unwrap();
    run(imu.configure(&mut delay, &alt())).unwrap();
    assert!(!shared.chip().stalled);
}

#[test]
fn a_mode_change_while_a_forced_measurement_runs_is_rejected_and_keeps_the_tracked_mode() {
    // Averaging 8 at full recharge is busy for 27 ms; NM is written while it runs and is ignored.
    let config = suspend()
        .with_odr(OutputDataRate::Hz25)
        .with_averaging(Averaging::Avg8);
    let (shared, mut imu, mut delay) = up(&config);
    run(imu.trigger_forced()).unwrap();
    let out = run(imu.set_power_mode(&mut delay, PowerMode::Normal));
    assert_eq!(
        out,
        Err(Error::PmuCommandRejected {
            cmd: reg::PMU_CMD_NM,
            status: 0x60
        })
    );
    assert_eq!(shared.chip().dropped_commands, 1);
    assert!(!shared.chip().normal);
    assert_eq!(imu.config(), None);
    assert_eq!(imu.mode, PowerMode::Suspend);
    // The tracked mode is still right: configure and a trigger work.
    run(imu.configure(&mut delay, &config)).unwrap();
    assert_eq!(imu.mode, PowerMode::Suspend);
    assert_eq!(run(imu.trigger_forced()), Ok(()));
    // A second trigger while one runs is a write the chip ignores; the driver cannot know.
    let before = shared.chip().dropped_commands;
    assert_eq!(run(imu.trigger_forced_fast()), Ok(()));
    assert_eq!(shared.chip().dropped_commands, before + 1);
    assert_eq!(imu.mode, PowerMode::Suspend);
    assert!(imu.config().is_some());
}

#[test]
fn magnetic_reset_is_refused_in_normal_mode_before_any_bus_access() {
    let (shared, mut imu, mut delay) = up(&Config::new());
    assert_eq!(
        run(imu.magnetic_reset(&mut delay)),
        Err(Error::WrongMode {
            mode: PowerMode::Normal
        })
    );
    assert_eq!((shared.transactions(), shared.awaits()), (0, 0));
    assert!(imu.config().is_some());
    // In suspend it runs: BR and FGR, each a write and a status read.
    let (shared, mut imu, mut delay) = up(&suspend());
    assert_eq!(run(imu.magnetic_reset(&mut delay)), Ok(()));
    assert_eq!(shared.transactions(), 4);
    // After set_power_mode to suspend as well.
    let (shared, mut imu, mut delay) = up(&Config::new());
    run(imu.set_power_mode(&mut delay, PowerMode::Suspend)).unwrap();
    shared.arm(None, None);
    assert_eq!(run(imu.magnetic_reset(&mut delay)), Ok(()));
    assert_eq!(shared.transactions(), 4);
    assert_eq!(shared.chip().illegal_commands, 0);
}

#[test]
fn sensor_time_is_one_transaction_and_the_helpers_handle_the_wrap() {
    let (shared, mut imu, _delay) = up(&Config::new());
    shared.chip().regs[0x3D..=0x3F].copy_from_slice(&[0x56, 0x34, 0x12]);
    assert_eq!(run(imu.sensor_time()), Ok(0x12_3456));
    assert_eq!(
        shared.log(),
        std::vec![Event::I2c {
            address: A,
            ops: std::vec![
                Op::Write(std::vec![0x3D]),
                Op::Read(std::vec![DUMMY, DUMMY, 0x56, 0x34, 0x12])
            ],
        }]
    );
    // It needs no configuration.
    let (shared, mut imu, _delay) = rig();
    shared.chip().regs[0x3D..=0x3F].copy_from_slice(&[0xFF, 0xFF, 0xFF]);
    assert_eq!(run(imu.sensor_time()), Ok(0x00FF_FFFF));

    use crate::timing::{sensor_elapsed_us, sensor_elapsed_us_with_period, sensor_ticks_elapsed};
    assert_eq!(sensor_ticks_elapsed(100, 350), 250);
    assert_eq!(sensor_ticks_elapsed(0x00FF_FFF0, 0x10), 0x20);
    assert_eq!(sensor_ticks_elapsed(7, 7), 0);
    assert_eq!(sensor_ticks_elapsed(0x00FF_FFFF, 0), 1);
    assert_eq!(sensor_elapsed_us(0x00FF_FFF0, 0x10), 1_250);
    assert_eq!(sensor_elapsed_us(1, 0), 655_359_960);
    assert_eq!(sensor_elapsed_us(0, 256), 10_000);
    assert_eq!(sensor_elapsed_us(0, 64), 2_500);
    assert_eq!(sensor_elapsed_us_with_period(0, 256, 39_062_500), 10_000);
    assert_eq!(sensor_elapsed_us_with_period(0, 256, 38_800_000), 9_932);
    assert_eq!(
        sensor_elapsed_us_with_period(1, 0, u32::MAX),
        0x00FF_FFFF * (u32::MAX as u64) / 1_000_000
    );
    // Bits above 24 are ignored.
    assert_eq!(sensor_ticks_elapsed(0x0100_0005, 0x0200_0007), 2);
}

#[test]
fn a_stalled_chip_passes_verify_and_sensor_time_is_how_to_notice() {
    let (shared, mut imu, mut delay) = up(&latched());
    stream_for(&shared, &mut delay, 10);
    let t0 = run(imu.sensor_time()).unwrap();
    run(delay.delay_us(10_000));
    let t1 = run(imu.sensor_time()).unwrap();
    assert!(crate::timing::sensor_elapsed_us(t0, t1) > 9_000);
    assert_eq!(run(imu.verify()), Ok(()));
    shared.chip().stalled = true;
    let edges = shared.chip().edges;
    run(delay.delay_us(10_000));
    let t2 = run(imu.sensor_time()).unwrap();
    run(delay.delay_us(10_000));
    assert_eq!(run(imu.sensor_time()), Ok(t2));
    assert_eq!(shared.chip().edges, edges);
    assert_eq!(run(imu.verify()), Ok(()));
}

// ---------------------------------------------------------------------------------------------
// Self-test, fast magnetic reset, data with sensor time
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Op3 {
    SelfTest,
    ResetFast,
    ReadTime,
}

#[derive(Debug, PartialEq)]
enum Out3 {
    Report(SelfTestReport),
    Unit,
    Timed(TimedSample),
}

async fn call3(imu: &mut Drv, delay: &mut TestDelay, op: Op3) -> Res<Out3> {
    Ok(match op {
        Op3::SelfTest => Out3::Report(imu.self_test(delay).await?),
        Op3::ResetFast => imu.magnetic_reset_fast(delay).await.map(|()| Out3::Unit)?,
        Op3::ReadTime => Out3::Timed(imu.read_with_time().await?),
    })
}

/// After a disturbed call: the state is what A3 says, the stimulus bits are zero once
/// `configure` has run (and already after a bus error), `verify` is Ok and the call works again.
fn after_disturbance(
    op: Op3,
    config: &Config,
    shared: &Shared,
    imu: &mut Drv,
    delay: &mut TestDelay,
    bus_error: bool,
) {
    shared.arm(None, None);
    if op == Op3::SelfTest {
        assert!(imu.config().is_none());
        assert_eq!(run(imu.read()), Err(Error::NotConfigured));
        if bus_error {
            assert_eq!(
                shared.reg(reg::TMR_SELFTEST_USER),
                0,
                "cleanup on a bus error"
            );
        }
        run(delay.delay_us(50_000));
        run(imu.configure(delay, config)).unwrap();
    } else {
        assert!(imu.config().is_some());
    }
    shared.arm(None, None);
    assert_eq!(run(imu.verify()), Ok(()));
    assert_eq!(shared.reg(reg::TMR_SELFTEST_USER), 0);
    run(delay.delay_us(50_000));
    match run(call3(imu, delay, op)) {
        Ok(Out3::Report(r)) => assert!(r.passed()),
        Ok(_) => {}
        Err(e) => panic!("{op:?} again: {e:?}"),
    }
}

/// Bus error at every transaction and cancel at every await: the counts of an undisturbed call.
fn sweep3(op: Op3, config: &Config) -> (usize, u32) {
    let mut txns = 0;
    for k in 1.. {
        let (shared, mut imu, mut delay) = up(config);
        shared.arm(Some(k), None);
        let out = run(call3(&mut imu, &mut delay, op));
        if shared.transactions() < k {
            assert!(out.is_ok(), "{op:?}");
            txns = k - 1;
            break;
        }
        assert_eq!(out, Err(Error::Bus(BusFault::Injected)), "{op:?} at {k}");
        after_disturbance(op, config, &shared, &mut imu, &mut delay, true);
    }
    let mut awaits = 0;
    for k in 1.. {
        let (shared, mut imu, mut delay) = up(config);
        shared.arm(None, Some(k));
        if let Polled::Done(out) = poll_then_drop(call3(&mut imu, &mut delay, op)) {
            assert!(out.is_ok(), "{op:?}");
            awaits = k - 1;
            break;
        }
        after_disturbance(op, config, &shared, &mut imu, &mut delay, false);
    }
    (txns, awaits)
}

#[test]
fn the_new_methods_survive_a_bus_error_and_a_cancel_anywhere() {
    assert_eq!(sweep3(Op3::SelfTest, &suspend()), (44, 62));
    assert_eq!(sweep3(Op3::SelfTest, &Config::new()), (49, 70));
    assert_eq!(sweep3(Op3::ResetFast, &suspend()), (4, 6));
    assert_eq!(sweep3(Op3::ReadTime, &Config::new()), (1, 1));
}

/// The data burst the fake returns for a forced measurement: dummy bytes, 12 data bytes.
fn data_event(x: [u8; 3]) -> Event {
    let mut got = std::vec![DUMMY, DUMMY];
    got.extend(x);
    got.extend([0x18, 0xFC, 0xFF, 0xD0, 0x07, 0x00, 0xFA, 0xC8, 0x00]);
    Event::I2c {
        address: A,
        ops: std::vec![Op::Write(std::vec![0x31]), Op::Read(got)],
    }
}

fn zero_writes(shared: &Shared) -> Vec<u8> {
    shared
        .log()
        .iter()
        .filter_map(|e| match e {
            Event::I2c { ops, .. } => match ops.as_slice() {
                [Op::Write(b)] if b.len() == 2 && b[0] == 0x60 => Some(b[1]),
                _ => None,
            },
            Event::Delay(_) => None,
        })
        .collect()
}

#[test]
fn self_test_golden_trace_from_suspend() {
    let (shared, mut imu, mut delay) = up(&suspend());
    let report = run(imu.self_test(&mut delay)).unwrap();
    assert!(report.passed());
    let log = shared.log();
    // 1 (Table 8, p28; 8.4, 8.5): PMU_CMD_AGGR_SET 0x14 (100 Hz, avg 2), PMU_CMD_AXIS_EN 0x07.
    // Magnetic reset, full pair: BR (7) 14 ms, echo 7; FGR (5) 18 ms, echo 5 (vendor).
    let mut want = std::vec![
        w(A, 0x04, 0x14),
        w(A, 0x05, 0x07),
        w(A, 0x06, 0x07),
        Event::Delay(14_000_000),
        r(A, 0x07, 0xE0),
        w(A, 0x06, 0x05),
        Event::Delay(18_000_000),
        r(A, 0x07, 0xA0),
        // X positive (8.33, p42; value 0x0D: generator, st_p, ist_en_x): baseline FM_FAST (4),
        // 6 ms, echo 4 in bits 7:5 (0x80), data; stimulus on, 1 ms; FM_FAST, 6 ms, echo, data
        // (X + 20000 counts = 21000 = 0x005208); stimulus off, 1 ms.
        w(A, 0x06, 0x04),
        Event::Delay(6_000_000),
        r(A, 0x07, 0x80),
        data_event([0xE8, 0x03, 0x00]),
        w(A, 0x60, 0x0D),
        Event::Delay(1_000_000),
        w(A, 0x06, 0x04),
        Event::Delay(6_000_000),
        r(A, 0x07, 0x80),
        data_event([0x08, 0x52, 0x00]),
        w(A, 0x60, 0x00),
        Event::Delay(1_000_000),
    ];
    assert_eq!(log[..want.len()], want[..]);
    // The other three entries: 0x0B, 0x15, 0x13, each followed by zero.
    assert_eq!(
        zero_writes(&shared),
        std::vec![0x0D, 0, 0x0B, 0, 0x15, 0, 0x13, 0, 0]
    );
    // Then `configure` in place on the suspended chip: status (0x80), the plan with 0x60 = 0.
    want.clear();
    want.extend([
        r(A, 0x07, 0x80),
        w(A, 0x04, 0x14),
        w(A, 0x05, 0x07),
        w(A, 0x2E, 0x8E),
        w(A, 0x0A, 0x00),
        w(A, 0x60, 0x00),
    ]);
    assert_eq!(log[log.len() - want.len()..], want[..]);
    assert_eq!((shared.transactions(), shared.awaits()), (44, 62));
    let slept: u64 = delays(&shared).iter().sum();
    assert_eq!(slept, 88_000_000);
    assert_eq!(shared.violations(), 0);
    assert_eq!(shared.reg(reg::TMR_SELFTEST_USER), 0);
    assert_eq!(imu.config(), Some(&suspend()));
    assert_eq!(shared.chip().st_shorts, 0);
}

#[test]
fn self_test_from_normal_mode_passes_and_leaves_everything_as_it_was() {
    let (shared, mut imu, mut delay) = up(&Config::new());
    let report = run(imu.self_test(&mut delay)).unwrap();
    assert!(report.passed());
    for e in &report.entries {
        assert!(e.delta.abs() > 130.0 && e.delta.abs() < 160.0, "{e:?}");
        assert_eq!(
            e.delta > 0.0,
            e.polarity == crate::selftest::SelfTestPolarity::Positive
        );
    }
    // SUS (write, polls at 2 ms and 3 ms), then as from suspend, then NM with its poll.
    assert_eq!(shared.transactions(), 49);
    let slept: u64 = delays(&shared).iter().sum();
    assert_eq!(slept, 106_000_000);
    assert_eq!(imu.config(), Some(&Config::new()));
    assert_eq!(run(imu.verify()), Ok(()));
    assert_eq!(shared.reg(reg::TMR_SELFTEST_USER), 0);
    // The data registers keep the last measurement of the test (Y with the negative stimulus)
    // until the next conversion: the stimulus itself is off.
    assert_eq!(run(imu.read()).map(|s| (s.x, s.y)), Ok((1000, -21000)));
}

#[test]
fn self_test_that_misses_the_limit_is_ok_with_a_failed_report() {
    let (shared, mut imu, mut delay) = up(&Config::new());
    shared.chip().st_counts = 10_000;
    let report = run(imu.self_test(&mut delay)).unwrap();
    assert!(!report.passed());
    assert!(
        report
            .entries
            .iter()
            .all(|e| !e.passed && e.delta.abs() > 60.0)
    );
    assert_eq!(imu.config(), Some(&Config::new()));
    assert_eq!(run(imu.verify()), Ok(()));
    assert_eq!(shared.reg(reg::TMR_SELFTEST_USER), 0);
    // No stimulus at all: all zero deltas, failed.
    shared.chip().st_counts = 0;
    let report = run(imu.self_test(&mut delay)).unwrap();
    assert!(!report.passed());
    assert!(report.entries.iter().all(|e| e.delta.abs() < 0.001));
}

#[test]
fn self_test_and_the_fast_reset_refuse_without_configuration_or_in_the_wrong_mode() {
    let (shared, mut imu, mut delay) = rig();
    shared.arm(None, None);
    assert_eq!(run(imu.self_test(&mut delay)), Err(Error::NotConfigured));
    assert_eq!(
        run(imu.magnetic_reset_fast(&mut delay)),
        Err(Error::NotConfigured)
    );
    assert_eq!(run(imu.read_with_time()), Err(Error::NotConfigured));
    assert_eq!(shared.transactions(), 0);
    let (shared, mut imu, mut delay) = up(&Config::new());
    assert_eq!(
        run(imu.magnetic_reset_fast(&mut delay)),
        Err(Error::WrongMode {
            mode: PowerMode::Normal
        })
    );
    assert_eq!(shared.transactions(), 0);
    assert!(imu.config().is_some());
    // A failed self-test leaves the driver unconfigured, and the next one refuses.
    shared.arm(Some(3), None);
    assert!(run(imu.self_test(&mut delay)).is_err());
    assert_eq!(imu.config(), None);
    shared.arm(None, None);
    assert_eq!(run(imu.self_test(&mut delay)), Err(Error::NotConfigured));
    assert_eq!(shared.transactions(), 0);
}

#[test]
fn magnetic_reset_fast_golden_trace_and_rejections() {
    let (shared, mut imu, mut delay) = up(&suspend());
    assert_eq!(run(imu.magnetic_reset_fast(&mut delay)), Ok(()));
    // 8.6 (p32): BR_FAST 0x08 then FGR_FAST 0x06, with the waits of the driver (4 ms, 8 ms) and the
    // echo in bits 7:5 of PMU_CMD_STATUS_0 (8.7, p33): 7 for BR_FAST (vendor), 6 for FGR_FAST.
    assert_eq!(
        shared.log(),
        std::vec![
            w(A, 0x06, 0x08),
            Event::Delay(4_000_000),
            r(A, 0x07, 0xE0),
            w(A, 0x06, 0x06),
            Event::Delay(8_000_000),
            r(A, 0x07, 0xC0),
        ]
    );
    assert_eq!((shared.transactions(), shared.awaits()), (4, 6));
    assert_eq!(run(imu.verify()), Ok(()));
    // An illegal flag or a wrong echo is a rejection naming the command and the status.
    shared.chip().status_or = reg::PMU_STATUS_0_ILLEGAL;
    assert!(matches!(
        run(imu.magnetic_reset_fast(&mut delay)),
        Err(Error::PmuCommandRejected { cmd: 0x08, .. })
    ));
    assert!(imu.config().is_some());
}

#[test]
fn read_with_time_is_one_transaction_of_seventeen_bytes() {
    let (shared, mut imu, _delay) = up(&Config::new());
    let at = usize::from(reg::SENSORTIME_XLSB);
    shared.chip().regs[at..at + 3].copy_from_slice(&[0x01, 0x02, 0xFF]);
    let got = run(imu.read_with_time()).unwrap();
    assert_eq!(
        got,
        TimedSample {
            sample: expected_raw(),
            sensor_time: 0x00FF_0201
        }
    );
    // 0x31 to 0x3F are adjacent (p37-40): the dummy bytes and 15 registers, more than the 15 bytes
    // that are fully queued (A4e).
    let mut bytes = std::vec![DUMMY, DUMMY];
    bytes.extend(shared.chip().regs[0x31..0x40].to_vec());
    assert_eq!(bytes.len(), 17);
    assert_eq!(
        shared.log(),
        std::vec![Event::I2c {
            address: A,
            ops: std::vec![Op::Write(std::vec![0x31]), Op::Read(bytes)]
        }]
    );
    assert_eq!(shared.transactions(), 1);
}

#[test]
fn verify_names_the_self_test_register_when_a_stimulus_bit_is_set_behind_its_back() {
    for bit in [
        reg::ST_IGEN_EN,
        reg::ST_N,
        reg::ST_P,
        reg::IST_EN_X,
        reg::IST_EN_Y,
    ] {
        let (shared, mut imu, mut delay) = up(&Config::new());
        shared.chip().regs[usize::from(reg::TMR_SELFTEST_USER)] = bit;
        assert_eq!(
            run(imu.verify()),
            mismatch(0x60, 0x1F, 0x00, bit),
            "{bit:#04x}"
        );
        // `configure` puts it back.
        run(imu.configure(&mut delay, &Config::new())).unwrap();
        assert_eq!(shared.reg(reg::TMR_SELFTEST_USER), 0);
        assert_eq!(run(imu.verify()), Ok(()));
    }
}

#[test]
fn the_new_methods_through_blocking_match_the_async_driver() {
    let a = Shared::new(A);
    let (mut imu, mut delay) = (Bmm350::new(a.bus(), A), a.delay());
    run(imu.init(&mut delay, &suspend())).unwrap();
    let async_out = (
        run(imu.self_test(&mut delay)),
        run(imu.magnetic_reset_fast(&mut delay)),
        run(imu.read_with_time()),
    );
    let b = Shared::new(A);
    let (mut imu, mut delay) = (crate::blocking::Bmm350::new(b.bus(), A), b.delay());
    imu.init(&mut delay, &suspend()).unwrap();
    let blocking_out = (
        imu.self_test(&mut delay),
        imu.magnetic_reset_fast(&mut delay),
        imu.read_with_time(),
    );
    assert!(async_out.0.as_ref().is_ok_and(SelfTestReport::passed));
    assert_eq!(async_out, blocking_out);
    assert_eq!(a.log(), b.log());
    assert_eq!(a.chip().regs, b.chip().regs);
}

#[test]
fn self_test_on_a_running_latched_chip_reads_int_status_after_sus_and_does_not_stall() {
    let (shared, mut imu, mut delay) = up(&latched());
    stream_for(&shared, &mut delay, 60);
    assert!(shared.chip().int_flag);
    shared.arm(None, None);
    let report = run(imu.self_test(&mut delay)).unwrap();
    assert!(report.passed());
    assert!(!shared.chip().stalled);
    let log = shared.log();
    // SUS completes (write, polls), then INT_STATUS (0x30, flag 0x04) is read before the first
    // write of the test (as `set_power_mode` does), and the flag is gone.
    let sus = log.iter().position(|e| *e == w(A, 0x06, 0x00)).unwrap();
    assert_eq!(log[sus + 5], r(A, 0x30, 0x04));
    assert_eq!(log[sus + 6], w(A, 0x04, 0x14));
    assert_eq!(run(imu.verify()), Ok(()));
    assert_eq!(shared.reg(reg::TMR_SELFTEST_USER), 0);
}
