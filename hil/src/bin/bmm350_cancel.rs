//! On-board pass/fail test of contract rules A4g, A4h and A4i for the bmm350 driver: a cancelled I2C read
//! is followed by `recover()` and no read is wrong afterwards. BMM350 alone on I2C0 (SDA GPIO12, SCL
//! GPIO13, 4.7k pull-ups), address 0x14.
//!
//! Only READS of read-only registers are cancelled: `Bmm350::read` (14 bytes on the wire: 2 dummy and the
//! 12 data bytes from MAG_X_XLSB), a 2-register read from CHIP_ID 0x00 (CHIP_ID and register 0x01, 4 bytes
//! on the wire) and, counted separately, `Bmm350::read_with_time` (17 bytes on the wire, more than the HAL
//! queues before its first await). No write, `init`, `configure`, `recover`, `verify`, reset or self-test
//! is ever cancelled.
//!
//! How a read is judged wrong (the true field is not known): the identity read must return
//! [0x33, byte of register 0x01 seen at the start]; a data read must be compensated to a field within
//! B_TOL of the mean of 200 reads taken before any cancel on every axis, a temperature within T_TOL, and
//! no channel may hold the reset pattern 0x7F7F7F. B_TOL and T_TOL come from the spread of those 200
//! reads. Every checking round reads identity, data, identity, data, so a leftover of either kind lands
//! on a read of the other kind. C1 shows that this judge sees the fault when no recovery is made.
#![no_std]
#![no_main]

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

use bmm350::{Averaging, Bmm350, Calibration, Config, OutputDataRate, RawSample, compensate, reg};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Flex, Pull};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::pac;
use embassy_rp::peripherals::{I2C0, PIN_12, PIN_13};
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

macro_rules! check {
    ($fails:expr, $cond:expr, $fmt:literal $(, $arg:expr)* $(,)?) => {
        if $cond {
            defmt::info!($fmt, "PASS" $(, $arg)*);
        } else {
            $fails += 1;
            defmt::error!($fmt, "FAIL" $(, $arg)*);
        }
    };
}

const ADDR: u8 = bmm350::DEFAULT_ADDRESS;

/// Registers dumped before the first cancel and after every block of cancels: CHIP_ID, PAD_CTRL,
/// PMU_CMD_AGGR_SET, PMU_CMD_AXIS_EN, PMU_CMD_STATUS_0, PMU_CMD_STATUS_1, I2C_WDT_SET,
/// TRANSDUCER_REV_ID, INT_CTRL, INT_CTRL_IBI. Left out: ERR_REG and INT_STATUS (cleared by a read),
/// PMU_CMD (a command register), the data and sensor-time registers (they change).
const DUMP: [u8; 10] = [0x00, 0x03, 0x04, 0x05, 0x07, 0x08, 0x0A, 0x0D, 0x2E, 0x2F];
const ND: usize = DUMP.len();

const RESET_RAW: i32 = 0x007F_7F7F;
const T_TOL_MIN: f32 = 2.0;
const B_TOL_MIN: f32 = 2.0;
const TOL_SIGMAS: f32 = 8.0;
const PER_BLOCK: u32 = 50;
const NPHASE: u32 = 8;
const RD_TMO: Duration = Duration::from_millis(20);
const REC_TMO: Duration = Duration::from_millis(100);
const RUN_LIMIT: Duration = Duration::from_secs(300);
const C1_N: u32 = 240;
const C2_BLOCKS: u32 = 20;
const C2B_BLOCKS: u32 = 8;
const C3_BLOCKS: u32 = 10;
const C4_TRIES: u32 = 60;

type Bus<'a> = I2c<'a, I2C0, i2c::Async>;
type Mag<'a> = Bmm350<Bus<'a>>;

struct Hw {
    i2c: Peri<'static, I2C0>,
    scl: Peri<'static, PIN_13>,
    sda: Peri<'static, PIN_12>,
}

impl Hw {
    fn bus(&mut self, freq: u32) -> Bus<'_> {
        let mut c = i2c::Config::default();
        c.frequency = freq;
        c.sda_pullup = false;
        c.scl_pullup = false;
        I2c::new_async(
            self.i2c.reborrow(),
            self.scl.reborrow(),
            self.sda.reborrow(),
            Irqs,
            c,
        )
    }
}

/// (SDA high, SCL high) from the pad inputs; works while the pins belong to the I2C block.
fn lines() -> (bool, bool) {
    let v = pac::SIO.gpio_in(0).read();
    (v & (1 << 12) != 0, v & (1 << 13) != 0)
}

fn spin(us: u64) {
    let t = Instant::now();
    while t.elapsed().as_micros() < us {}
}

fn abs(x: f32) -> f32 {
    if x < 0.0 { -x } else { x }
}

/// Square root by Newton iteration (the driver and the other programs use no libm).
fn sqrt(x: f32) -> f32 {
    if x <= 0.0 {
        return 0.0;
    }
    let mut g = f32::from_bits((x.to_bits() >> 1) + 0x1FBD_1DF5);
    for _ in 0..4 {
        g = 0.5 * (g + x / g);
    }
    g
}

/// Bus clear with the pins as GPIO: SCL pulses only while SDA reads low (at most nine), then a
/// START+STOP made with SDA alone while SCL is high. With SDA high no clock is given, so a device
/// that is receiving can never be clocked into a data byte. Returns (SDA was low, SCL was low, pulses).
fn bus_clear(hw: &mut Hw) -> (bool, bool, u8) {
    let mut scl = Flex::new(hw.scl.reborrow());
    let mut sda = Flex::new(hw.sda.reborrow());
    scl.set_pull(Pull::None);
    sda.set_pull(Pull::None);
    scl.set_low();
    sda.set_low();
    scl.set_as_input();
    sda.set_as_input();
    spin(5);
    let (sda_low, scl_low) = (sda.is_low(), scl.is_low());
    let mut pulses = 0u8;
    while sda.is_low() && pulses < 9 {
        scl.set_as_output();
        spin(5);
        scl.set_as_input();
        spin(5);
        pulses += 1;
    }
    if sda.is_high() && scl.is_high() {
        sda.set_as_output();
        spin(5);
        sda.set_as_input();
        spin(5);
    }
    (sda_low, scl_low, pulses)
}

/// Polls `fut` in a tight loop with a no-op waker and drops it `d_us` after the first poll.
/// None: it was dropped before it finished (a cancel). Some: it finished first.
fn cancel<F: Future>(fut: F, d_us: u64) -> Option<F::Output> {
    let mut cx = Context::from_waker(Waker::noop());
    let t0 = Instant::now();
    let mut fut = pin!(fut);
    loop {
        if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
            return Some(v);
        }
        if t0.elapsed().as_micros() >= d_us {
            return None;
        }
    }
}

fn hang(what: &str, fails: u32) -> ! {
    defmt::error!(
        "[FAIL] {=str}: no answer within its bound; the sensor or the bus is stuck, stopping with no further bus traffic (a power cycle may be needed)",
        what
    );
    hil_common::exit(fails + 1)
}

fn first_diff(a: &[u8; ND], b: &[u8; ND]) -> Option<(u8, u8, u8)> {
    (0..ND)
        .find(|&i| a[i] != b[i])
        .map(|i| (DUMP[i], a[i], b[i]))
}

/// The 12 data bytes a sample was decoded from (each channel 24 bits little-endian).
fn raw_bytes(s: &RawSample) -> [u8; 12] {
    let mut o = [0u8; 12];
    for (i, v) in [s.x, s.y, s.z, s.temperature].iter().enumerate() {
        o[i * 3..i * 3 + 3].copy_from_slice(&v.to_le_bytes()[..3]);
    }
    o
}

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Data = 0,
    Id = 1,
    Timed = 2,
}

const KINDS: [Kind; 3] = [Kind::Data, Kind::Id, Kind::Timed];

impl Kind {
    fn name(self) -> &'static str {
        match self {
            Kind::Data => "data read (14 bytes)",
            Kind::Id => "id read (4 bytes)",
            Kind::Timed => "read_with_time (17 bytes)",
        }
    }
}

#[derive(Clone, Copy)]
struct Refs {
    b: [f32; 3],
    t: f32,
    rev: u8,
    cal: Calibration,
    b_tol: f32,
    t_tol: f32,
}

impl Refs {
    fn id_ok(&self, b: &[u8; 2]) -> bool {
        *b == [reg::CHIP_ID_VALUE, self.rev]
    }
    fn sample_ok(&self, s: &RawSample) -> bool {
        if [s.x, s.y, s.z, s.temperature].contains(&RESET_RAW) {
            return false;
        }
        let f = compensate(&self.cal, s);
        abs(f.x - self.b[0]) <= self.b_tol
            && abs(f.y - self.b[1]) <= self.b_tol
            && abs(f.z - self.b[2]) <= self.b_tol
            && abs(f.temperature - self.t) <= self.t_tol
    }
    fn data_ok(&self, b: &[u8; 12]) -> bool {
        self.sample_ok(&RawSample::decode(b))
    }
}

enum Rd {
    Ok,
    Wrong([u8; 12], u8),
    Bus,
    Timeout,
}

#[derive(Default, Clone, Copy)]
struct Tally {
    dropped: [u32; 3],
    completed: u32,
    rec_fail: u32,
    rec_n: u32,
    rec_sum_us: u64,
    rec_max_us: u32,
    reads: u32,
    wrong: u32,
    bus: u32,
    dump_equal: u32,
    dump_changed: u32,
    dump_fail: u32,
}

struct Setup {
    base: [u8; ND],
    span: [u32; 3],
}

struct Ctx<'a> {
    mag: Mag<'a>,
    refs: Refs,
    fails: u32,
    start: Instant,
    dump_bad: bool,
}

fn show(first: &mut bool, kind: &str, d: u64, what: &str, idx: u32, r: &Rd) {
    if !*first {
        return;
    }
    *first = false;
    match r {
        Rd::Wrong(b, n) => defmt::error!(
            "[INFO] first failing case: cancelled {=str} at d={=u64} us, checking read {=u32} ({=str}) returned {=[u8]:#04x}",
            kind,
            d,
            idx,
            what,
            b[..*n as usize]
        ),
        _ => defmt::error!(
            "[INFO] first failing case: cancelled {=str} at d={=u64} us, checking read {=u32} ({=str}) bus error",
            kind,
            d,
            idx,
            what
        ),
    }
}

impl<'a> Ctx<'a> {
    fn new(bus: Bus<'a>, start: Instant) -> Self {
        Self {
            mag: Bmm350::new(bus, ADDR),
            refs: Refs {
                b: [0.0; 3],
                t: 0.0,
                rev: 0,
                cal: Calibration::default(),
                b_tol: 0.0,
                t_tol: 0.0,
            },
            fails: 0,
            start,
            dump_bad: false,
        }
    }

    async fn init(&mut self, cfg: &Config) -> Result<(), ()> {
        match with_timeout(Duration::from_millis(1000), self.mag.init(&mut Delay, cfg)).await {
            Err(_) => hang("init", self.fails),
            Ok(Err(e)) => {
                defmt::error!("[INFO] init failed: {}", defmt::Debug2Format(&e));
                Err(())
            }
            Ok(Ok(_)) => Ok(()),
        }
    }

    async fn rd_id(&mut self) -> Rd {
        let mut b = [0u8; 2];
        match with_timeout(RD_TMO, self.mag.read_registers(reg::CHIP_ID, &mut b)).await {
            Err(_) => Rd::Timeout,
            Ok(Err(_)) => Rd::Bus,
            Ok(Ok(())) if self.refs.id_ok(&b) => Rd::Ok,
            Ok(Ok(())) => {
                let mut o = [0u8; 12];
                o[..2].copy_from_slice(&b);
                Rd::Wrong(o, 2)
            }
        }
    }

    async fn rd_data(&mut self) -> Rd {
        let mut b = [0u8; 12];
        match with_timeout(RD_TMO, self.mag.read_registers(reg::MAG_X_XLSB, &mut b)).await {
            Err(_) => Rd::Timeout,
            Ok(Err(_)) => Rd::Bus,
            Ok(Ok(())) if self.refs.data_ok(&b) => Rd::Ok,
            Ok(Ok(())) => Rd::Wrong(b, 12),
        }
    }

    async fn rd_timed(&mut self) -> Rd {
        match with_timeout(RD_TMO, self.mag.read_with_time()).await {
            Err(_) => Rd::Timeout,
            Ok(Err(_)) => Rd::Bus,
            Ok(Ok(ts)) if self.refs.sample_ok(&ts.sample) => Rd::Ok,
            Ok(Ok(ts)) => Rd::Wrong(raw_bytes(&ts.sample), 12),
        }
    }

    /// Counts the read in `t`; true when it was right. A timeout ends the run.
    fn tally(&self, t: &mut Tally, r: &Rd) -> bool {
        match r {
            Rd::Ok => true,
            Rd::Wrong(..) => {
                t.wrong += 1;
                false
            }
            Rd::Bus => {
                t.bus += 1;
                false
            }
            Rd::Timeout => hang("checking read", self.fails),
        }
    }

    async fn dump(&mut self) -> Result<[u8; ND], u8> {
        let mut out = [0u8; ND];
        for (i, &r) in DUMP.iter().enumerate() {
            match with_timeout(RD_TMO, self.mag.read_register(r)).await {
                Ok(Ok(v)) => out[i] = v,
                Err(_) => hang("register dump", self.fails),
                _ => return Err(r),
            }
        }
        Ok(out)
    }

    /// The driver's read of `kind`, dropped `d` us after its first poll. None: cancelled. Some(ok):
    /// it finished first, ok is false when it returned an error. One body for C1 and the sweeps.
    fn do_cancel(&mut self, kind: Kind, d: u64) -> Option<bool> {
        match kind {
            Kind::Data => cancel(self.mag.read(), d).map(|r| r.is_ok()),
            Kind::Id => {
                let mut b = [0u8; 2];
                cancel(self.mag.read_registers(reg::CHIP_ID, &mut b), d).map(|r| r.is_ok())
            }
            Kind::Timed => cancel(self.mag.read_with_time(), d).map(|r| r.is_ok()),
        }
    }

    /// Identity, data, identity, data after a cancel (and its recovery, if any).
    async fn check4(&mut self, t: &mut Tally, first: &mut bool, kind: &str, d: u64) {
        for i in 0..4u32 {
            let id = i % 2 == 0;
            let r = if id {
                self.rd_id().await
            } else {
                self.rd_data().await
            };
            t.reads += 1;
            if !self.tally(t, &r) {
                show(
                    first,
                    kind,
                    d,
                    if id {
                        "CHIP_ID+reg 0x01"
                    } else {
                        "data, 12 bytes"
                    },
                    i,
                    &r,
                );
            }
        }
    }

    /// Init with `cfg`, reference, 300 uncancelled checking rounds (timed), oracle controls, baseline
    /// dump. None when a check failed.
    async fn setup(&mut self, cfg: &Config, label: &str) -> Option<Setup> {
        let f0 = self.fails;
        let inited = self.init(cfg).await;
        check!(
            self.fails,
            inited.is_ok(),
            "[{=str}] {=str}: init to the test configuration (normal 100 Hz, averaging 2)",
            label
        );
        inited.ok()?;
        Timer::after_millis(300).await;
        let v = with_timeout(RD_TMO, self.mag.verify()).await;
        check!(
            self.fails,
            matches!(v, Ok(Ok(()))),
            "[{=str}] {=str}: verify() after init",
            label
        );

        let (mut i0, mut i1) = ([0u8; 2], [0u8; 2]);
        let r0 = with_timeout(RD_TMO, self.mag.read_registers(reg::CHIP_ID, &mut i0)).await;
        let r1 = with_timeout(RD_TMO, self.mag.read_registers(reg::CHIP_ID, &mut i1)).await;
        check!(
            self.fails,
            matches!((r0, r1), (Ok(Ok(())), Ok(Ok(())))) && i0 == i1 && i0[0] == reg::CHIP_ID_VALUE,
            "[{=str}] {=str}: identity reads CHIP_ID and register 0x01 twice alike {=[u8]:#04x}",
            label,
            i0
        );
        if self.fails != f0 {
            return None;
        }
        self.refs.rev = i0[1];
        self.refs.cal = *self.mag.calibration();

        let mut first = [0u8; 12];
        let mut mean = [0f32; 4];
        let mut m2 = [0f32; 4];
        let mut got = 0u32;
        for _ in 0..200 {
            let mut b = [0u8; 12];
            if let Ok(Ok(())) =
                with_timeout(RD_TMO, self.mag.read_registers(reg::MAG_X_XLSB, &mut b)).await
            {
                let s = RawSample::decode(&b);
                if ![s.x, s.y, s.z, s.temperature].contains(&RESET_RAW) {
                    if got == 0 {
                        first = b;
                    }
                    let f = compensate(&self.refs.cal, &s);
                    got += 1;
                    let v = [f.x, f.y, f.z, f.temperature];
                    for i in 0..4 {
                        let d = v[i] - mean[i];
                        mean[i] += d / got as f32;
                        m2[i] += d * (v[i] - mean[i]);
                    }
                }
            }
            Timer::after_millis(10).await;
        }
        check!(
            self.fails,
            got == 200,
            "[{=str}] {=str}: 200 reference reads returned samples ({=u32})",
            label,
            got
        );
        if got < 2 {
            return None;
        }
        let sd = |i: usize| sqrt(m2[i] / (got - 1) as f32);
        let sx = sd(0).max(sd(1)).max(sd(2));
        self.refs.b = [mean[0], mean[1], mean[2]];
        self.refs.t = mean[3];
        self.refs.b_tol = (TOL_SIGMAS * sx).max(B_TOL_MIN);
        self.refs.t_tol = (TOL_SIGMAS * sd(3)).max(T_TOL_MIN);
        defmt::info!(
            "[STAT] {=str}: reference field x {=f32} y {=f32} z {=f32} uT, temperature {=f32} degC, spread (std) x {=f32} y {=f32} z {=f32} uT and {=f32} degC, tolerance {=f32} uT per axis and {=f32} degC ({=f32} sigma, floors {=f32} uT and {=f32} degC), register 0x01 = {=u8:#04x}",
            label,
            mean[0],
            mean[1],
            mean[2],
            mean[3],
            sd(0),
            sd(1),
            sd(2),
            sd(3),
            self.refs.b_tol,
            self.refs.t_tol,
            TOL_SIGMAS,
            B_TOL_MIN,
            T_TOL_MIN,
            self.refs.rev
        );

        let mut t = Tally::default();
        let mut tt = Tally::default();
        let (mut w, mut su, mut hi) = ([0u32; 3], [0u64; 3], 0u32);
        for _ in 0..300 {
            for (k, wk) in w.iter_mut().enumerate() {
                let a = Instant::now();
                let r = match k {
                    0 => self.rd_data().await,
                    1 => self.rd_id().await,
                    _ => self.rd_timed().await,
                };
                let us = a.elapsed().as_micros() as u32;
                self.tally(if k == 2 { &mut tt } else { &mut t }, &r);
                *wk = (*wk).max(us);
                su[k] += us as u64;
            }
            hi += 1;
        }
        defmt::info!(
            "[STAT] {=str}: uncancelled read mean/worst: data 14 bytes {=u64}/{=u32} us, id 4 bytes {=u64}/{=u32} us, read_with_time 17 bytes {=u64}/{=u32} us",
            label,
            su[0] / hi as u64,
            w[0],
            su[1] / hi as u64,
            w[1],
            su[2] / hi as u64,
            w[2]
        );
        check!(
            self.fails,
            t.wrong == 0 && t.bus == 0,
            "[{=str}] {=str}: 600 uncancelled reads all right (wrong {=u32}, bus errors {=u32})",
            label,
            t.wrong,
            t.bus
        );
        check!(
            self.fails,
            tt.wrong == 0 && tt.bus == 0,
            "[{=str}] {=str}: 300 uncancelled read_with_time reads all right (wrong {=u32}, bus errors {=u32})",
            label,
            tt.wrong,
            tt.bus
        );

        let mut stale = [0u8; 12];
        stale[0] = reg::CHIP_ID_VALUE;
        stale[1] = self.refs.rev;
        stale[2..].copy_from_slice(&first[..10]);
        let mut late = [0u8; 12];
        late[2..].copy_from_slice(&first[..10]);
        late[0] = first[10];
        late[1] = first[11];
        check!(
            self.fails,
            self.refs.data_ok(&first)
                && !self.refs.data_ok(&stale)
                && !self.refs.data_ok(&late)
                && !self.refs.data_ok(&[reg::DATA_RESET; 12])
                && !self.refs.id_ok(&[reg::CHIP_ID_VALUE, self.refs.rev ^ 1])
                && !self.refs.id_ok(&[first[0], first[1]]),
            "[{=str}] {=str}: control, the judge accepts a real data read and rejects stale bytes ({=[u8]:#04x}), a data read shifted by two, the reset pattern and a wrong identity",
            label,
            stale
        );

        let base = self.dump().await;
        check!(
            self.fails,
            base.is_ok(),
            "[{=str}] {=str}: baseline register dump taken",
            label
        );
        if let Ok(b) = &base {
            defmt::info!(
                "[STAT] {=str}: baseline register dump of {=[u8]:#04x} is {=[u8]:#04x}",
                label,
                DUMP,
                *b
            );
        }
        let base = base.ok()?;
        if self.fails != f0 {
            return None;
        }
        Some(Setup {
            base,
            span: [
                w[0] * 115 / 100 + 5,
                w[1] * 115 / 100 + 5,
                w[2] * 115 / 100 + 5,
            ],
        })
    }

    /// C1: cancels with NO recovery, then six checking reads back to back. Returns false when the judge
    /// saw no wrong read.
    async fn c1(&mut self, s: &Setup, freq: u32) -> bool {
        let (mut n, mut with_wrong, mut worst, mut completed, mut unhealed) =
            ([0u32; 3], [0u32; 3], 0u32, 0u32, 0u32);
        let mut t = Tally::default();
        let mut first_shown = false;
        for i in 0..C1_N {
            let k = (i % 3) as usize;
            let kind = KINDS[k];
            let d = s.span[k] as u64 * (i / 3) as u64 / (C1_N / 3 - 1) as u64;
            let dropped = self.do_cancel(kind, d).is_none();
            let mut w = 0u32;
            for j in 0..6u32 {
                let r = if j % 2 == 0 {
                    self.rd_id().await
                } else {
                    self.rd_data().await
                };
                t.reads += 1;
                if !self.tally(&mut t, &r) {
                    w += 1;
                    if dropped && !first_shown {
                        first_shown = true;
                        show(&mut true, kind.name(), d, "no recovery", j, &r);
                    }
                }
            }
            if dropped {
                n[k] += 1;
                with_wrong[k] += (w > 0) as u32;
                worst = worst.max(w);
            } else {
                completed += 1;
            }
            // Restore without the driver's recover(): 3 ms with no bus traffic, one discarded read,
            // then rounds until identity and data are right.
            Timer::after_millis(3).await;
            let _ = self.rd_id().await;
            let mut ok = false;
            for _ in 0..4 {
                if matches!(self.rd_id().await, Rd::Ok) && matches!(self.rd_data().await, Rd::Ok) {
                    ok = true;
                    break;
                }
            }
            if !ok {
                unhealed += 1;
                Timer::after_millis(10).await;
            }
        }
        let wrong_cancels = with_wrong[0] + with_wrong[1] + with_wrong[2];
        defmt::info!(
            "[STAT] C1 {=u32} kHz, no recovery: cancelled data reads {=u32} (followed by a wrong read: {=u32}), cancelled id reads {=u32} ({=u32}), cancelled read_with_time {=u32} ({=u32}), completed first {=u32}, most wrong reads after one cancel {=u32} of 6, wrong reads {=u32} of {=u32}, bus errors {=u32}, not restored by 3 ms quiet {=u32}",
            freq,
            n[0],
            with_wrong[0],
            n[1],
            with_wrong[1],
            n[2],
            with_wrong[2],
            completed,
            worst,
            t.wrong,
            t.reads,
            t.bus,
            unhealed
        );
        check!(
            self.fails,
            wrong_cancels >= 1,
            "[{=str}] C1 positive control at {=u32} kHz: {=u32} of {=u32} cancels without recovery were followed by a wrong read",
            freq,
            wrong_cancels,
            n[0] + n[1] + n[2]
        );
        wrong_cancels >= 1
    }

    /// After the first failing case: a second `recover()`, then four checking reads.
    async fn diagnose(&mut self, kind: &str, d: u64) {
        let r2 = with_timeout(REC_TMO, self.mag.recover(&mut Delay)).await;
        let rec2 = match r2 {
            Err(_) => hang("second recover()", self.fails),
            Ok(r) => r.is_ok(),
        };
        let mut t = Tally::default();
        self.check4(&mut t, &mut false, kind, d).await;
        defmt::error!(
            "[INFO] diagnosis of that case ({=str}, d={=u64} us): a second recover() returned ok {=bool}, then {=u32} of 4 checking reads wrong and {=u32} bus errors",
            kind,
            d,
            rec2,
            t.wrong,
            t.bus
        );
    }

    /// One sweep: blocks of PER_BLOCK cancels, kinds taken in turn by block, `recover()` after each
    /// cancel, the checking round, and the register dump after each block. Ends at the first block with
    /// a failure. True when it passed.
    async fn sweep(
        &mut self,
        name: &str,
        kinds: &[Kind],
        s: &Setup,
        freq: u32,
        blocks: u32,
        min_dropped: u32,
    ) -> bool {
        let mut t = Tally::default();
        let mut first = true;
        let mut diagnosed = false;
        let nk = kinds.len() as u32;
        'b: for b in 0..blocks {
            let kind = kinds[(b % nk) as usize];
            let span = s.span[kind as usize] as u64;
            let mut bad = false;
            for j in 0..PER_BLOCK {
                if self.start.elapsed() > RUN_LIMIT {
                    hang("run time limit", self.fails);
                }
                let d = span * (j * NPHASE + (b / nk) % NPHASE) as u64
                    / ((PER_BLOCK - 1) * NPHASE + NPHASE - 1) as u64;
                match self.do_cancel(kind, d) {
                    None => t.dropped[kind as usize] += 1,
                    Some(ok) => {
                        t.completed += 1;
                        if !ok {
                            t.bus += 1;
                            bad = true;
                        }
                    }
                }
                let r0 = Instant::now();
                let r = with_timeout(REC_TMO, self.mag.recover(&mut Delay)).await;
                let us = r0.elapsed().as_micros() as u32;
                t.rec_n += 1;
                t.rec_sum_us += us as u64;
                t.rec_max_us = t.rec_max_us.max(us);
                match r {
                    Err(_) => hang("recover()", self.fails),
                    Ok(Err(e)) => {
                        t.rec_fail += 1;
                        bad = true;
                        if first {
                            first = false;
                            defmt::error!(
                                "[INFO] first failing case: cancelled {=str} at d={=u64} us, recover() returned {}",
                                kind.name(),
                                d,
                                defmt::Debug2Format(&e)
                            );
                        }
                    }
                    Ok(Ok(())) => {}
                }
                let w0 = t.wrong + t.bus;
                self.check4(&mut t, &mut first, kind.name(), d).await;
                let this_bad = t.wrong + t.bus != w0;
                bad |= this_bad;
                if bad && !diagnosed {
                    diagnosed = true;
                    self.diagnose(kind.name(), d).await;
                }
            }
            match self.dump().await {
                Err(r) => {
                    t.dump_fail += 1;
                    defmt::error!("[INFO] register dump read of {=u8:#04x} failed", r);
                    break 'b;
                }
                Ok(now) => {
                    if let Some((r, old, new)) = first_diff(&s.base, &now) {
                        t.dump_changed += 1;
                        self.dump_bad = true;
                        defmt::error!(
                            "[INFO] first register change: block {=u32} ({=str}), reg {=u8:#04x} was {=u8:#04x} now {=u8:#04x}",
                            b,
                            kind.name(),
                            r,
                            old,
                            new
                        );
                        break 'b;
                    }
                    t.dump_equal += 1;
                }
            }
            if bad {
                break 'b;
            }
        }
        let dropped = t.dropped[0] + t.dropped[1] + t.dropped[2];
        defmt::info!(
            "[STAT] {=str} sweep {=u32} kHz: cancelled {=u32} (data {=u32}, id {=u32}, read_with_time {=u32}), completed before the drop {=u32}; recover() mean {=u64} us worst {=u32} us over {=u32} calls",
            name,
            freq,
            dropped,
            t.dropped[0],
            t.dropped[1],
            t.dropped[2],
            t.completed,
            t.rec_sum_us / t.rec_n.max(1) as u64,
            t.rec_max_us,
            t.rec_n
        );
        let ok = t.rec_fail == 0
            && t.wrong == 0
            && t.bus == 0
            && t.dump_changed == 0
            && t.dump_fail == 0
            && t.dump_equal == blocks
            && dropped >= min_dropped;
        check!(
            self.fails,
            ok,
            "[{=str}] {=str} sweep {=u32} kHz: {=u32} cancels, recover() failed {=u32}, wrong checking reads {=u32} of {=u32}, bus errors {=u32}, register dumps equal to the baseline {=u32} of {=u32} (changed {=u32}), minimum cancels {=u32}",
            name,
            freq,
            dropped,
            t.rec_fail,
            t.wrong,
            t.reads,
            t.bus,
            t.dump_equal,
            blocks,
            t.dump_changed,
            min_dropped
        );
        ok
    }
}

/// C4. Returns the failures it added. Everything here is bounded.
async fn c4(hw: &mut Hw, cfg: &Config, refs: Refs, start: Instant) -> u32 {
    let mut fails = 0u32;
    const F: u32 = 400_000;
    let mut mag = Bmm350::new(hw.bus(F), ADDR);
    let ok = matches!(
        with_timeout(Duration::from_millis(1000), mag.init(&mut Delay, cfg)).await,
        Ok(Ok(_))
    );
    if !ok {
        check!(fails, false, "[{=str}] C4: init before the attempts");
        return fails;
    }
    Timer::after_millis(300).await;
    let mut stuck = None;
    let mut tries = 0u32;
    for a in 0..C4_TRIES {
        if start.elapsed() > RUN_LIMIT {
            hang("run time limit", fails);
        }
        tries += 1;
        let d = 60 + a as u64 * 8;
        let mut buf = [0u8; 15];
        let _ = cancel(mag.read_registers(reg::MAG_X_XLSB, &mut buf), d);
        // Rebuild the I2C object at once, with the frame still running.
        drop(mag.release());
        mag = Bmm350::new(hw.bus(F), ADDR);
        spin(100);
        let (sda, scl) = lines();
        if !sda || !scl {
            Timer::after_millis(2).await;
            let (sda2, scl2) = lines();
            if !sda2 || !scl2 {
                stuck = Some((a, d, sda2, scl2, "a line held low"));
                break;
            }
        }
        Timer::after_millis(3).await;
        let mut good = 0u32;
        for _ in 0..3 {
            let mut b = [0u8; 2];
            match with_timeout(RD_TMO, mag.read_registers(reg::CHIP_ID, &mut b)).await {
                Ok(Ok(())) if refs.id_ok(&b) => good += 1,
                _ => good = 0,
            }
        }
        if good < 2 {
            let (sda2, scl2) = lines();
            stuck = Some((a, d, sda2, scl2, "reads fail with both lines high"));
            break;
        }
    }
    let Some((a, d, sda, scl, how)) = stuck else {
        defmt::info!(
            "[SKIP] C4: {=u32} rebuilds of the I2C object right after a cancelled read (d=60..{=u32} us, 400 kHz) left no line low and every read answered; the stuck line could not be provoked on purpose",
            tries,
            60 + (C4_TRIES - 1) * 8
        );
        return fails;
    };
    defmt::info!(
        "[STAT] C4: provoked at attempt {=u32} (d={=u64} us): SDA high {=bool}, SCL high {=bool} ({=str})",
        a,
        d,
        sda,
        scl,
        how
    );
    let mut failed = 0u32;
    for _ in 0..3 {
        let mut b = [0u8; 2];
        match with_timeout(RD_TMO, mag.read_registers(reg::CHIP_ID, &mut b)).await {
            Ok(Ok(())) if refs.id_ok(&b) => {}
            _ => failed += 1,
        }
    }
    check!(
        fails,
        failed >= 1,
        "[{=str}] C4: with the line stuck, {=u32} of 3 identity reads failed or hung (20 ms bound each)",
        failed
    );
    drop(mag.release());
    let (sl, cl, pulses) = bus_clear(hw);
    defmt::info!(
        "[STAT] C4: bus clear: SDA low at start {=bool}, SCL low at start {=bool}, {=u8} SCL pulses",
        sl,
        cl,
        pulses
    );
    let mut mag = Bmm350::new(hw.bus(F), ADDR);
    Timer::after_millis(3).await;
    let (sda, scl) = lines();
    let mut ids = 0u32;
    for _ in 0..3 {
        let mut b = [0u8; 2];
        if let Ok(Ok(())) = with_timeout(RD_TMO, mag.read_registers(reg::CHIP_ID, &mut b)).await
            && refs.id_ok(&b)
        {
            ids += 1;
        }
    }
    // The new driver object has not been through init, so recover() would stop at NotConfigured.
    let inited = matches!(
        with_timeout(Duration::from_millis(1000), mag.init(&mut Delay, cfg)).await,
        Ok(Ok(_))
    );
    Timer::after_millis(300).await;
    let rec = with_timeout(REC_TMO, mag.recover(&mut Delay)).await;
    let rec_ok = matches!(rec, Ok(Ok(())));
    let (mut good, mut bad) = (0u32, 0u32);
    for _ in 0..200 {
        let mut b = [0u8; 12];
        match with_timeout(RD_TMO, mag.read_registers(reg::MAG_X_XLSB, &mut b)).await {
            Ok(Ok(())) if refs.data_ok(&b) => good += 1,
            _ => bad += 1,
        }
        Timer::after_micros(500).await;
    }
    check!(
        fails,
        sda && scl && ids == 3 && inited && rec_ok && good == 200 && bad == 0,
        "[{=str}] C4: after the bus clear lines high (SDA {=bool}, SCL {=bool}), identity reads right {=u32} of 3, init ok {=bool}, recover() ok {=bool}, good data reads {=u32} of 200 (bad {=u32})",
        sda,
        scl,
        ids,
        inited,
        rec_ok,
        good,
        bad
    );
    fails
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let start = Instant::now();
    defmt::info!(
        "[INFO] bmm350_cancel: I2C0 SDA=GPIO12 SCL=GPIO13, address {=u8:#04x}",
        ADDR
    );
    let mut hw = Hw {
        i2c: p.I2C0,
        scl: p.PIN_13,
        sda: p.PIN_12,
    };
    // The default configuration (100 Hz, averaging 2, normal mode, pulsed push-pull data-ready) is the
    // one the smoke program starts with; the test configuration is the same, so C5 meets the C0 dump.
    let default = Config::default();
    let test = Config::new()
        .with_odr(OutputDataRate::Hz100)
        .with_averaging(Averaging::Avg2);
    let mut fails = 0u32;
    let mut hard = 0u32;
    let mut refs = Refs {
        b: [0.0; 3],
        t: 0.0,
        rev: 0,
        cal: Calibration::default(),
        b_tol: 0.0,
        t_tol: 0.0,
    };
    let mut cal0 = Calibration::default();
    let mut c1_seen = false;
    let mut c2b_ok = true;
    let setup0: Option<Setup>;

    // C0, C1, C2, C2b at 400 kHz.
    {
        let mut c = Ctx::new(hw.bus(400_000), start);
        setup0 = c.setup(&test, "C0 400 kHz").await;
        if let Some(s) = &setup0 {
            refs = c.refs;
            cal0 = refs.cal;
            c1_seen = c.c1(s, 400).await;
            if c1_seen {
                let c2_ok = c
                    .sweep("C2", &[Kind::Data, Kind::Id], s, 400, C2_BLOCKS, 500)
                    .await;
                if c2_ok {
                    c2b_ok = c
                        .sweep("C2b", &[Kind::Timed], s, 400, C2B_BLOCKS, 200)
                        .await;
                } else {
                    hard += 1;
                    defmt::info!("[SKIP] C2b: C2 did not pass, stopping");
                }
            } else {
                defmt::info!(
                    "[SKIP] C2: the positive control saw no wrong read, so a clean sweep would prove nothing"
                );
            }
        }
        fails += c.fails;
        if c.dump_bad {
            hard += 1;
        }
    }
    if setup0.is_none() || !c1_seen {
        hil_common::exit(fails + 1);
    }
    if hard != 0 {
        hil_common::exit(fails);
    }

    // After a failed C2b: does a bus clear change what the reads return?
    if !c2b_ok {
        let (sl, cl, pulses) = bus_clear(&mut hw);
        defmt::info!(
            "[STAT] C2b diagnosis: bus clear (SDA low {=bool}, SCL low {=bool}, {=u8} pulses)",
            sl,
            cl,
            pulses
        );
        let mut c = Ctx::new(hw.bus(400_000), start);
        c.refs = refs;
        let mut t = Tally::default();
        c.check4(&mut t, &mut false, "after the bus clear", 0).await;
        defmt::error!(
            "[INFO] C2b diagnosis: after the bus clear (no recover()) {=u32} of 4 checking reads wrong, {=u32} bus errors",
            t.wrong,
            t.bus
        );
    }

    // C3.
    for freq in [100_000u32, 1_000_000] {
        let mut c = Ctx::new(hw.bus(freq), start);
        let label = if freq == 100_000 {
            "C3 100 kHz"
        } else {
            "C3 1000 kHz"
        };
        if let Some(s) = c.setup(&test, label).await {
            let same = first_diff(&s.base, &setup0.as_ref().map_or([0; ND], |x| x.base)).is_none();
            check!(
                c.fails,
                same,
                "[{=str}] {=str}: baseline register dump equal to the 400 kHz baseline",
                label
            );
            c.sweep(
                "C3",
                &[Kind::Data, Kind::Id],
                &s,
                freq / 1000,
                C3_BLOCKS,
                200,
            )
            .await;
        }
        fails += c.fails;
        if c.dump_bad || c.fails != 0 {
            hil_common::exit(fails);
        }
    }

    // C4.
    let f4 = c4(&mut hw, &test, refs, start).await;
    fails += f4;
    if f4 != 0 {
        hil_common::exit(fails);
    }

    // C5.
    {
        let mut c = Ctx::new(hw.bus(400_000), start);
        let ok = c.init(&default).await.is_ok();
        Timer::after_millis(2000).await;
        let v = with_timeout(RD_TMO, c.mag.verify()).await;
        let now = c.dump().await;
        let same = matches!((&now, &setup0), (Ok(n), Some(s)) if first_diff(&s.base, n).is_none());
        let cal_same = *c.mag.calibration() == cal0;
        check!(
            c.fails,
            ok && matches!(v, Ok(Ok(()))) && same && cal_same,
            "[{=str}] C5 end: default configuration init ok {=bool}, verify() ok, register dump equal to the default one {=bool}, OTP calibration read again equal to the first {=bool}",
            ok,
            same,
            cal_same
        );
        defmt::info!(
            "[INFO] health: failed checks before C5 {=u32}, config applied {=bool}, run took {=u64} ms, C1 saw the fault {=bool}, C2b passed {=bool}",
            fails,
            c.mag.config().is_some(),
            start.elapsed().as_millis(),
            c1_seen,
            c2b_ok
        );
        fails += c.fails;
    }
    hil_common::exit(fails)
}
