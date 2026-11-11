//! On-board test of the bmm350 `self_test`, `magnetic_reset_fast`, `read_with_time` and
//! `exceeds_high_field`. BMM350 on I2C0 (SDA GPIO12, SCL GPIO13), INT on GPIO4. Typed API only.
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

use bmm350::timing::sensor_elapsed_us;
use bmm350::{
    Averaging, Bmm350, Calibration, Config, Error, Field, InterruptMode, InterruptPin,
    OutputDataRate, StartMode, compensate, exceeds_high_field,
};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::i2c::{self, Async, I2c};
use embassy_rp::peripherals::I2C0;
use embassy_time::{Delay, Duration, Instant, Timer};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

static EDGES: AtomicU32 = AtomicU32::new(0);
static EDGE_US: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
async fn count_edges(mut int: Input<'static>) {
    loop {
        int.wait_for_rising_edge().await;
        EDGE_US.store(Instant::now().as_micros() as u32, Relaxed);
        EDGES.fetch_add(1, Relaxed);
    }
}

fn edge_now() -> (u32, u32) {
    loop {
        let c = EDGES.load(Relaxed);
        let t = EDGE_US.load(Relaxed);
        if EDGES.load(Relaxed) == c {
            return (c, t);
        }
    }
}

/// Wait for an edge after count `e0`, at most `ms` ms. The only timed wait in this file: it
/// polls the edge counter and never wraps a bus call.
async fn wait_edge(e0: u32, ms: u64) -> Option<u32> {
    let end = Instant::now() + Duration::from_millis(ms);
    while Instant::now() < end {
        let (e, t) = edge_now();
        if e != e0 {
            return Some(t);
        }
        Timer::after_micros(20).await;
    }
    None
}

macro_rules! check {
    ($failures:expr, $cond:expr, $fmt:literal $(, $arg:expr)* $(,)?) => {
        if $cond {
            defmt::info!($fmt, "[PASS]" $(, $arg)*);
        } else {
            $failures += 1;
            defmt::error!($fmt, "[FAIL]" $(, $arg)*);
        }
    };
}

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

fn magnitude(f: &Field) -> f32 {
    sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
}

fn plausible(f: &Field) -> bool {
    (10.0..=120.0).contains(&magnitude(f)) && (10.0..=50.0).contains(&f.temperature)
}

fn absf(x: f32) -> f32 {
    if x < 0.0 { -x } else { x }
}

fn within(a: &Field, b: &Field, tol: f32) -> bool {
    absf(a.x - b.x) <= tol && absf(a.y - b.y) <= tol && absf(a.z - b.z) <= tol
}

fn median(v: &mut [u32]) -> u32 {
    for i in 1..v.len() {
        let mut j = i;
        while j > 0 && v[j - 1] > v[j] {
            v.swap(j - 1, j);
            j -= 1;
        }
    }
    if v.is_empty() { 0 } else { v[v.len() / 2] }
}

fn bus_config() -> i2c::Config {
    let mut cfg = i2c::Config::default();
    cfg.frequency = 400_000;
    cfg.sda_pullup = false;
    cfg.scl_pullup = false;
    cfg
}

/// Running mean and variance of x, y, z, temperature, and the magnitude range.
#[derive(Clone, Copy)]
struct Acc {
    n: u32,
    mean: [f32; 4],
    m2: [f32; 4],
    min_mag: f32,
    max_mag: f32,
}

impl Acc {
    fn new() -> Self {
        Self {
            n: 0,
            mean: [0.0; 4],
            m2: [0.0; 4],
            min_mag: f32::MAX,
            max_mag: 0.0,
        }
    }
    fn push(&mut self, f: &Field) {
        self.n += 1;
        let v = [f.x, f.y, f.z, f.temperature];
        for i in 0..4 {
            let d = v[i] - self.mean[i];
            self.mean[i] += d / self.n as f32;
            self.m2[i] += d * (v[i] - self.mean[i]);
        }
        let m = magnitude(f);
        if m < self.min_mag {
            self.min_mag = m;
        }
        if m > self.max_mag {
            self.max_mag = m;
        }
    }
    fn var(&self, i: usize) -> f32 {
        self.m2[i] / (self.n.max(2) as f32 - 1.0)
    }
    fn std(&self, i: usize) -> f32 {
        sqrt(self.var(i))
    }
    fn field(&self) -> Field {
        Field {
            x: self.mean[0],
            y: self.mean[1],
            z: self.mean[2],
            temperature: self.mean[3],
        }
    }
}

const NAMES: [&str; 4] = ["X+", "X-", "Y+", "Y-"];

struct T {
    mag: Bmm350<I2c<'static, I2C0, Async>>,
    delay: Delay,
    failures: u32,
    start: Instant,
    base: Field,
    base_std: [f32; 3],
    deltas: [[f32; 4]; 8],
    nd: usize,
    zt: Option<Field>,
}

impl T {
    fn late(&self) -> bool {
        self.start.elapsed() > Duration::from_secs(48)
    }

    async fn init(&mut self, label: &'static str, cfg: &Config) -> bool {
        let r = self.mag.init(&mut self.delay, cfg).await;
        let ok = r.is_ok();
        check!(self.failures, ok, "{=str} init {=str}", label);
        if let Err(e) = r {
            defmt::error!("[INFO] {=str}: init error {}", label, e);
        }
        ok
    }

    /// `n` samples on data-ready edges (pulsed or latched interrupt).
    async fn collect(&mut self, n: u32, latched: bool) -> (Acc, u32) {
        let mut acc = Acc::new();
        let mut errs = 0u32;
        if latched && self.mag.interrupt_status().await.is_err() {
            errs += 1;
        }
        let mut seen = edge_now().0;
        let end = Instant::now() + Duration::from_secs(4);
        while acc.n < n && Instant::now() < end {
            if wait_edge(seen, 200).await.is_none() {
                errs += 1;
                defmt::error!("[INFO] collect: no data-ready edge within 200 ms");
                break;
            }
            seen = edge_now().0;
            match self.mag.read_field().await {
                Ok(f) => acc.push(&f),
                Err(e) => {
                    errs += 1;
                    defmt::error!("[INFO] collect: read error {}", e);
                }
            }
            if latched && self.mag.interrupt_status().await.is_err() {
                errs += 1;
            }
        }
        (acc, errs)
    }

    /// One forced measurement from suspend, read after the busy time has passed.
    async fn forced_once(&mut self) -> Option<Field> {
        let e0 = edge_now().0;
        if let Err(e) = self.mag.trigger_forced().await {
            defmt::error!("[INFO] forced: trigger error {}", e);
            return None;
        }
        if wait_edge(e0, 300).await.is_none() {
            defmt::error!("[INFO] forced: no data-ready edge within 300 ms");
            return None;
        }
        Timer::after_millis(35).await;
        self.mag.read_field().await.ok()
    }

    async fn forced_mean(&mut self, n: u32) -> Acc {
        let mut acc = Acc::new();
        for _ in 0..n {
            if let Some(f) = self.forced_once().await {
                acc.push(&f);
            }
        }
        acc
    }

    fn conclude(&self, n: u32, before: u32) {
        if self.failures == before {
            defmt::info!("[INFO] CHECK {=u32}: PASS", n);
        } else {
            defmt::error!(
                "[INFO] CHECK {=u32}: FAIL ({=u32} failing lines)",
                n,
                self.failures - before
            );
        }
    }

    fn skip(&mut self, n: u32) {
        self.failures += 1;
        defmt::error!("[INFO] CHECK {=u32}: NOT RUN (time budget)", n);
    }

    /// State checks and the "after" field following one self-test run.
    async fn after(&mut self, label: &'static str, run: u32, cfg: &Config) {
        let suspend = cfg.start_mode == StartMode::Suspend;
        let latched = cfg.interrupt.mode == InterruptMode::Latched;
        if suspend {
            // The data registers keep the last measurement of the test (Y with the negative stimulus).
            match self.mag.read_field().await {
                Ok(f) => {
                    defmt::info!(
                        "[STAT] {=str} run {=u32}: registers right after self_test (suspend): x {} y {} z {} uT, temperature {} C",
                        label,
                        run,
                        f.x,
                        f.y,
                        f.z,
                        f.temperature
                    );
                    self.zt = Some(f);
                }
                Err(e) => defmt::error!("[INFO] {=str} run {=u32}: read error {}", label, run, e),
            }
        }
        let v = self.mag.verify().await;
        check!(
            self.failures,
            v.is_ok(),
            "{=str} {=str} run {=u32}: verify Ok",
            label,
            run
        );
        if let Err(e) = v {
            defmt::error!("[INFO] verify error {}", e);
        }
        check!(
            self.failures,
            self.mag.config() == Some(cfg),
            "{=str} {=str} run {=u32}: config() equals the configuration applied before",
            label,
            run
        );
        match self.mag.pmu_status().await {
            Ok(st) => {
                defmt::info!(
                    "[STAT] {=str} run {=u32}: PMU_CMD_STATUS_0 {=u8:#04x} STATUS_1 {=u8:#04x} (normal {} illegal {})",
                    label,
                    run,
                    st.status_0,
                    st.status_1,
                    st.normal_mode,
                    st.illegal_command
                );
                check!(
                    self.failures,
                    st.normal_mode == !suspend && !st.illegal_command,
                    "{=str} {=str} run {=u32}: pmu_status mode as configured, no illegal flag",
                    label,
                    run
                );
            }
            Err(e) => check!(
                self.failures,
                false,
                "{=str} {=str} run {=u32}: pmu_status error {}",
                label,
                run,
                e
            ),
        }
        match self.mag.error_status().await {
            Ok(er) => {
                defmt::info!(
                    "[STAT] {=str} run {=u32}: ERR_REG {=u8:#04x}",
                    label,
                    run,
                    er.raw
                );
                check!(
                    self.failures,
                    !er.pmu_cmd_error,
                    "{=str} {=str} run {=u32}: error_status clean",
                    label,
                    run
                );
            }
            Err(e) => check!(
                self.failures,
                false,
                "{=str} {=str} run {=u32}: error_status error {}",
                label,
                run,
                e
            ),
        }
        let (after, ok) = if suspend {
            match self.forced_once().await {
                Some(f) => {
                    let mut a = Acc::new();
                    a.push(&f);
                    (a, true)
                }
                None => (Acc::new(), false),
            }
        } else {
            let (a, errs) = self.collect(50, latched).await;
            let good = errs == 0 && a.n == 50;
            (a, good)
        };
        check!(
            self.failures,
            ok,
            "{=str} {=str} run {=u32}: 50 samples (1 forced in suspend) obtained",
            label,
            run
        );
        if after.n > 0 {
            let m = after.field();
            let b = self.base;
            defmt::info!(
                "[STAT] {=str} run {=u32}: mean before x {} y {} z {} uT; after x {} y {} z {} uT (n {=u32}); |B| range {}..{} uT",
                label,
                run,
                b.x,
                b.y,
                b.z,
                m.x,
                m.y,
                m.z,
                after.n,
                after.min_mag,
                after.max_mag
            );
            let plaus = after.min_mag >= 10.0 && after.max_mag <= 120.0;
            check!(
                self.failures,
                plaus,
                "{=str} {=str} run {=u32}: samples plausible (10..120 uT)",
                label,
                run
            );
            check!(
                self.failures,
                within(&m, &b, 3.0),
                "{=str} {=str} run {=u32}: after within 3 uT of before on every axis",
                label,
                run
            );
        }
    }

    async fn one_run(&mut self, label: &'static str, run: u32, cfg: &Config) {
        let t0 = Instant::now();
        let r = self.mag.self_test(&mut self.delay).await;
        let us = t0.elapsed().as_micros() as u32;
        match r {
            Ok(rep) => {
                defmt::info!(
                    "[STAT] {=str} run {=u32}: self_test took {=u32} us",
                    label,
                    run,
                    us
                );
                let mut d = [0.0f32; 4];
                for (i, e) in rep.entries.iter().enumerate() {
                    d[i] = e.delta;
                    defmt::info!(
                        "[STAT] {=str} run {=u32} {=str}: without {} uT, with {} uT, delta {} uT, {=str}",
                        label,
                        run,
                        NAMES[i],
                        e.without,
                        e.with,
                        e.delta,
                        if e.passed { "pass" } else { "fail" }
                    );
                }
                if self.nd < self.deltas.len() {
                    self.deltas[self.nd] = d;
                    self.nd += 1;
                }
                check!(
                    self.failures,
                    rep.passed(),
                    "{=str} {=str} run {=u32}: all four entries pass",
                    label,
                    run
                );
            }
            Err(e) => {
                check!(
                    self.failures,
                    false,
                    "{=str} {=str} run {=u32}: self_test returned an error after {=u32} us",
                    label,
                    run,
                    us
                );
                defmt::error!("[INFO] self_test error: {}", e);
                match self.mag.pmu_status().await {
                    Ok(st) => defmt::info!(
                        "[STAT] after the error: PMU_CMD_STATUS_0 {=u8:#04x} STATUS_1 {=u8:#04x}",
                        st.status_0,
                        st.status_1
                    ),
                    Err(e) => defmt::info!("[INFO] pmu_status error {}", e),
                }
                if let Ok(er) = self.mag.error_status().await {
                    defmt::info!("[STAT] after the error: ERR_REG {=u8:#04x}", er.raw);
                }
                let ok = self.mag.init(&mut self.delay, cfg).await;
                defmt::info!("[INFO] re-init after the error: {}", ok.err());
            }
        }
        self.after(label, run, cfg).await;
    }

    async fn check1(&mut self) {
        let before = self.failures;
        let cfg = Config::new();
        if !self.init("1", &cfg).await {
            self.conclude(1, before);
            return;
        }
        let (acc, errs) = self.collect(50, false).await;
        self.base = acc.field();
        self.base_std = [acc.std(0), acc.std(1), acc.std(2)];
        defmt::info!(
            "[STAT] 1 baseline mean x {} y {} z {} uT temperature {} C; std x {} y {} z {} uT; |B| range {}..{}",
            self.base.x,
            self.base.y,
            self.base.z,
            self.base.temperature,
            self.base_std[0],
            self.base_std[1],
            self.base_std[2],
            acc.min_mag,
            acc.max_mag
        );
        check!(
            self.failures,
            errs == 0 && acc.n == 50 && acc.min_mag >= 10.0 && acc.max_mag <= 120.0,
            "{=str} 1 baseline: 50 plausible samples ({=u32} errors)",
            errs
        );
        for run in 1..=5u32 {
            if self.late() {
                break;
            }
            self.one_run("c1", run, &cfg).await;
        }
        self.conclude(1, before);
    }

    async fn check2(&mut self) {
        let before = self.failures;
        let susp = Config::new().with_start_mode(StartMode::Suspend);
        let latch =
            Config::new().with_interrupt(InterruptPin::new().with_mode(InterruptMode::Latched));
        for (label, cfg) in [("c2 suspend", susp), ("c2 latched", latch)] {
            if self.late() {
                break;
            }
            if self.init(label, &cfg).await {
                self.one_run(label, 1, &cfg).await;
            }
        }
        self.conclude(2, before);
    }

    fn check3(&mut self) {
        let before = self.failures;
        let n = self.nd;
        defmt::info!("[INFO] 3: {=usize} self-test runs recorded", n);
        let mut means = [0.0f32; 4];
        for i in 0..4 {
            let (mut s, mut mn, mut mx) = (0.0f32, f32::MAX, f32::MIN);
            for r in 0..n {
                let d = self.deltas[r][i];
                s += d;
                mn = mn.min(d);
                mx = mx.max(d);
            }
            let mean = if n > 0 { s / n as f32 } else { 0.0 };
            let mut q = 0.0f32;
            for r in 0..n {
                let d = self.deltas[r][i] - mean;
                q += d * d;
            }
            let std = sqrt(q / (n.max(2) as f32 - 1.0));
            means[i] = mean;
            defmt::info!(
                "[STAT] 3 {=str}: delta mean {} uT, std {} uT, min {} uT, max {} uT",
                NAMES[i],
                mean,
                std,
                mn,
                mx
            );
        }
        check!(
            self.failures,
            n > 0 && means[0] > 0.0 && means[1] < 0.0,
            "{=str} 3 X: positive stimulus raised and negative lowered the reading (mean {} / {} uT)",
            means[0],
            means[1]
        );
        check!(
            self.failures,
            n > 0 && means[2] > 0.0 && means[3] < 0.0,
            "{=str} 3 Y: positive stimulus raised and negative lowered the reading (mean {} / {} uT)",
            means[2],
            means[3]
        );
        match self.zt {
            Some(f) => defmt::info!(
                "[STAT] 3 Z and temperature during the Y- stimulus (suspend run, registers after self_test): z {} uT (baseline z {}), temperature {} C (baseline {})",
                f.z,
                self.base.z,
                f.temperature,
                self.base.temperature
            ),
            None => defmt::info!("[STAT] 3 Z and temperature during a stimulus: not obtained"),
        }
        self.conclude(3, before);
    }

    async fn check4(&mut self) {
        let before = self.failures;
        let susp = Config::new().with_start_mode(StartMode::Suspend);
        for fast in [true, false] {
            if self.late() {
                break;
            }
            let name = if fast { "fast" } else { "full" };
            if !self.init(name, &susp).await {
                continue;
            }
            let pre = self.forced_mean(4).await;
            let st0 = self.mag.pmu_status().await.ok();
            let t0 = Instant::now();
            let r = if fast {
                self.mag.magnetic_reset_fast(&mut self.delay).await
            } else {
                self.mag.magnetic_reset(&mut self.delay).await
            };
            let us = t0.elapsed().as_micros() as u32;
            let st = self.mag.pmu_status().await;
            let er = self.mag.error_status().await;
            let post = self.forced_once().await;
            defmt::info!(
                "[STAT] 4 {=str}: took {=u32} us; result {}",
                name,
                us,
                r.err()
            );
            if let Some(s) = st0 {
                defmt::info!(
                    "[STAT] 4 {=str}: status before reset {=u8:#04x} {=u8:#04x}",
                    name,
                    s.status_0,
                    s.status_1
                );
            }
            let mut clean = false;
            if let Ok(s) = st {
                defmt::info!(
                    "[STAT] 4 {=str}: PMU_CMD_STATUS_0 right after {=u8:#04x} (echo field {=u8}, busy {}, normal {}, illegal {}), STATUS_1 {=u8:#04x}",
                    name,
                    s.status_0,
                    s.command,
                    s.busy,
                    s.normal_mode,
                    s.illegal_command,
                    s.status_1
                );
                clean = !s.illegal_command;
            }
            if let Ok(e) = er {
                defmt::info!("[STAT] 4 {=str}: ERR_REG {=u8:#04x}", name, e.raw);
                clean = clean && !e.pmu_cmd_error;
            } else {
                clean = false;
            }
            let pre_f = pre.field();
            let field_ok = match post {
                Some(p) => {
                    defmt::info!(
                        "[STAT] 4 {=str}: field before (mean of {=u32}) x {} y {} z {}; after x {} y {} z {} uT; std before x {} y {} z {}",
                        name,
                        pre.n,
                        pre_f.x,
                        pre_f.y,
                        pre_f.z,
                        p.x,
                        p.y,
                        p.z,
                        pre.std(0),
                        pre.std(1),
                        pre.std(2)
                    );
                    pre.n > 0 && within(&p, &pre_f, 3.0)
                }
                None => false,
            };
            check!(
                self.failures,
                r.is_ok() && clean && field_ok,
                "{=str} 4 {=str} in suspend: Ok, status without illegal flag, ERR_REG clean, field within 3 uT",
                name
            );
        }
        if !self.late() && self.init("4 normal", &Config::new()).await {
            let s0 = self.mag.pmu_status().await.ok();
            let r1 = self.mag.magnetic_reset_fast(&mut self.delay).await;
            let r2 = self.mag.magnetic_reset(&mut self.delay).await;
            let s1 = self.mag.pmu_status().await.ok();
            defmt::info!(
                "[STAT] 4 normal: fast result {}; full result {}",
                r1.err(),
                r2.err()
            );
            let same = match (s0, s1) {
                (Some(a), Some(b)) => {
                    defmt::info!(
                        "[STAT] 4 normal: status before {=u8:#04x} {=u8:#04x}, after {=u8:#04x} {=u8:#04x}",
                        a.status_0,
                        a.status_1,
                        b.status_0,
                        b.status_1
                    );
                    a.status_0 == b.status_0 && a.status_1 == b.status_1
                }
                _ => false,
            };
            check!(
                self.failures,
                matches!(r1, Err(Error::WrongMode { .. }))
                    && matches!(r2, Err(Error::WrongMode { .. }))
                    && same,
                "{=str} 4 normal mode: both resets return WrongMode and pmu_status is unchanged"
            );
        }
        self.conclude(4, before);
    }

    async fn block(&mut self, label: &'static str, cfg: Config, nominal: f32) {
        if !self.init(label, &cfg).await {
            return;
        }
        let cal: Calibration = *self.mag.calibration();
        let (mut acc_t, mut acc_p) = (Acc::new(), Acc::new());
        let mut t_timed = [0u32; 200];
        let mut t_plain = [0u32; 200];
        let mut max_diff = 0.0f32;
        let (mut errs, mut implaus, mut missed, mut n) = (0u32, 0u32, 0u32, 0usize);
        let mut prev: Option<(u32, u32, u32)> = None;
        let (mut dt_min, mut dt_max, mut dt_sum, mut dt_n, mut edge_sum) =
            (u32::MAX, 0u32, 0u64, 0u32, 0u64);
        let mut seen = edge_now().0;
        let end = Instant::now() + Duration::from_secs(6);
        while n < 200 && Instant::now() < end {
            if wait_edge(seen, 100).await.is_none() {
                errs += 1;
                defmt::error!("[INFO] {=str}: no data-ready edge within 100 ms", label);
                break;
            }
            let (e, t) = edge_now();
            missed += e.wrapping_sub(seen).saturating_sub(1);
            seen = e;
            let a = Instant::now();
            let r1 = self.mag.read_with_time().await;
            let us1 = a.elapsed().as_micros() as u32;
            let b = Instant::now();
            let r2 = self.mag.read().await;
            let us2 = b.elapsed().as_micros() as u32;
            let ts = match r1 {
                Ok(v) => v,
                Err(er) => {
                    errs += 1;
                    if errs <= 5 {
                        defmt::error!("[INFO] {=str}: read_with_time error {}", label, er);
                    }
                    continue;
                }
            };
            let raw = match r2 {
                Ok(v) => v,
                Err(er) => {
                    errs += 1;
                    if errs <= 5 {
                        defmt::error!("[INFO] {=str}: read error {}", label, er);
                    }
                    continue;
                }
            };
            let ft = compensate(&cal, &ts.sample);
            let fp = compensate(&cal, &raw);
            if !plausible(&ft) || !plausible(&fp) {
                implaus += 1;
            }
            for d in [absf(ft.x - fp.x), absf(ft.y - fp.y), absf(ft.z - fp.z)] {
                if d > max_diff {
                    max_diff = d;
                }
            }
            acc_t.push(&ft);
            acc_p.push(&fp);
            t_timed[n] = us1;
            t_plain[n] = us2;
            n += 1;
            if let Some((pe, pt, ps)) = prev {
                if e == pe.wrapping_add(1) {
                    let dt = sensor_elapsed_us(ps, ts.sensor_time);
                    dt_min = dt_min.min(dt);
                    dt_max = dt_max.max(dt);
                    dt_sum += dt as u64;
                    dt_n += 1;
                    edge_sum += t.wrapping_sub(pt) as u64;
                }
            }
            prev = Some((e, t, ts.sensor_time));
        }
        let ok_reads = n == 200 && errs == 0;
        check!(
            self.failures,
            ok_reads,
            "{=str} 5 {=str}: 200 reads, no read failed ({=usize} reads, {=u32} errors, {=u32} missed edges)",
            label,
            n,
            errs,
            missed
        );
        check!(
            self.failures,
            implaus == 0,
            "{=str} 5 {=str}: no implausible value ({=u32} seen)",
            label,
            implaus
        );
        let nf = n.max(1) as f32;
        let mut means_ok = n > 0;
        for (i, axis) in ["x", "y", "z"].into_iter().enumerate() {
            let diff = acc_t.mean[i] - acc_p.mean[i];
            let tol = 4.0 * sqrt(acc_t.var(i) / nf + acc_p.var(i) / nf) + 0.05;
            defmt::info!(
                "[STAT] 5 {=str} {=str}: mean timed {} plain {} diff {} uT (tolerance {}), std timed {} plain {}",
                label,
                axis,
                acc_t.mean[i],
                acc_p.mean[i],
                diff,
                tol,
                acc_t.std(i),
                acc_p.std(i)
            );
            if absf(diff) > tol {
                means_ok = false;
            }
        }
        defmt::info!(
            "[STAT] 5 {=str}: largest per-edge difference timed vs plain {} uT",
            label,
            max_diff
        );
        check!(
            self.failures,
            means_ok,
            "{=str} 5 {=str}: field of read_with_time equals the plain read within noise",
            label
        );
        let mean_dt = if dt_n > 0 {
            dt_sum as f32 / dt_n as f32
        } else {
            0.0
        };
        let mean_edge = if dt_n > 0 {
            edge_sum as f32 / dt_n as f32
        } else {
            0.0
        };
        let ratio = if edge_sum > 0 {
            dt_sum as f32 / edge_sum as f32
        } else {
            0.0
        };
        let rate = if edge_sum > 0 {
            dt_n as f32 * 1e6 / edge_sum as f32
        } else {
            0.0
        };
        defmt::info!(
            "[STAT] 5 {=str}: sensor-time step min {=u32} max {=u32} mean {} us over {=u32} pairs; measured edge period {} us, rate {} Hz (nominal {}); sensor/host ratio {}",
            label,
            if dt_n > 0 { dt_min } else { 0 },
            dt_max,
            mean_dt,
            dt_n,
            mean_edge,
            rate,
            nominal,
            ratio
        );
        check!(
            self.failures,
            dt_n >= 150 && absf(ratio - 1.0) < 0.02,
            "{=str} 5 {=str}: sensor-time step equals the measured edge period within 2 %",
            label
        );
        let mt = median(&mut t_timed[..n]);
        let mp = median(&mut t_plain[..n]);
        defmt::info!(
            "[STAT] 5 {=str}: median read_with_time {=u32} us, median plain read {=u32} us",
            label,
            mt,
            mp
        );
    }

    async fn check5(&mut self) {
        let before = self.failures;
        let c100 = Config::new()
            .with_odr(OutputDataRate::Hz100)
            .with_averaging(Averaging::Avg2);
        let c400 = Config::new()
            .with_odr(OutputDataRate::Hz400)
            .with_averaging(Averaging::Avg1);
        self.block("100 Hz", c100, 100.0).await;
        if !self.late() {
            self.block("400 Hz", c400, 400.0).await;
        }
        self.conclude(5, before);
    }

    async fn check6(&mut self) {
        let before = self.failures;
        if !self.init("6", &Config::new()).await {
            self.conclude(6, before);
            return;
        }
        let mut acc = Acc::new();
        let (mut exceed, mut errs) = (0u32, 0u32);
        let mut seen = edge_now().0;
        let end = Instant::now() + Duration::from_secs(5);
        while acc.n < 200 && Instant::now() < end {
            if wait_edge(seen, 100).await.is_none() {
                errs += 1;
                break;
            }
            seen = edge_now().0;
            match self.mag.read_field().await {
                Ok(f) => {
                    if exceeds_high_field(&f) {
                        exceed += 1;
                    }
                    acc.push(&f);
                }
                Err(e) => {
                    errs += 1;
                    defmt::error!("[INFO] 6: read error {}", e);
                }
            }
        }
        defmt::info!(
            "[STAT] 6: samples {=u32}, largest magnitude {} uT, smallest {} uT, exceeds_high_field true {=u32} times",
            acc.n,
            acc.max_mag,
            acc.min_mag,
            exceed
        );
        check!(
            self.failures,
            acc.n == 200 && errs == 0 && exceed == 0,
            "{=str} 6: exceeds_high_field false for 200 samples at rest ({=u32} errors)",
            errs
        );
        self.conclude(6, before);
    }

    async fn check7(&mut self) {
        let before = self.failures;
        if self.init("7", &Config::new()).await {
            let e0 = edge_now().0;
            let _ = wait_edge(e0, 200).await;
            let mut good = 0u32;
            for trial in 1..=3u32 {
                let a = self.mag.sensor_time().await;
                let ta = Instant::now();
                Timer::after_millis(100).await;
                let b = self.mag.sensor_time().await;
                let host = ta.elapsed().as_micros() as u32;
                match (a, b) {
                    (Ok(a), Ok(b)) => {
                        let d = sensor_elapsed_us(a, b);
                        let ok = (95_000..=105_000).contains(&d);
                        if ok {
                            good += 1;
                        }
                        defmt::info!(
                            "[STAT] 7 normal trial {=u32}: sensor time {=u32} -> {=u32} ticks, advanced {=u32} us in {=u32} host us, {=str}",
                            trial,
                            a,
                            b,
                            d,
                            host,
                            if ok { "within 5 %" } else { "outside 5 %" }
                        );
                    }
                    _ => defmt::error!("[INFO] 7: sensor_time read failed"),
                }
            }
            check!(
                self.failures,
                good >= 2,
                "{=str} 7 normal mode: sensor_time advances by 100 ms +-5 % ({=u32} of 3 trials)",
                good
            );
            // Tick length over a 1 s window, host time taken at the middle of each read.
            let a0 = Instant::now();
            let a = self.mag.sensor_time().await;
            let a1 = Instant::now();
            Timer::after_millis(1000).await;
            let b0 = Instant::now();
            let b = self.mag.sensor_time().await;
            let b1 = Instant::now();
            if let (Ok(a), Ok(b)) = (a, b) {
                let ticks = bmm350::timing::sensor_ticks_elapsed(a, b);
                let host = ((b0.as_micros() + b1.as_micros()) / 2
                    - (a0.as_micros() + a1.as_micros()) / 2) as u32;
                defmt::info!(
                    "[STAT] 7 tick over 1 s window: {=u32} ticks in {=u32} host us = {} us per tick (driver nominal 39.0625)",
                    ticks,
                    host,
                    host as f32 / ticks as f32
                );
            }
        }
        if self
            .init("7s", &Config::new().with_start_mode(StartMode::Suspend))
            .await
        {
            let a = self.mag.sensor_time().await;
            Timer::after_millis(100).await;
            let b = self.mag.sensor_time().await;
            match (a, b) {
                (Ok(a), Ok(b)) => defmt::info!(
                    "[STAT] 7 suspend: sensor time {=u32} -> {=u32} ticks over 100 ms, advanced {}",
                    a,
                    b,
                    a != b
                ),
                _ => defmt::error!("[INFO] 7 suspend: sensor_time read failed"),
            }
        }
        self.conclude(7, before);
    }

    async fn finish(&mut self) {
        let cfg = Config::new();
        let ok = self.init("final", &cfg).await;
        if ok {
            let v = self.mag.verify().await;
            check!(self.failures, v.is_ok(), "{=str} final: verify Ok");
            let er = self.mag.error_status().await;
            check!(
                self.failures,
                matches!(er, Ok(e) if e.raw == 0),
                "{=str} final: error_status clean ({})",
                er.ok().map(|e| e.raw)
            );
            let (acc, errs) = self.collect(50, false).await;
            defmt::info!(
                "[STAT] final: {=u32} samples, mean x {} y {} z {} uT, |B| range {}..{} uT",
                acc.n,
                acc.mean[0],
                acc.mean[1],
                acc.mean[2],
                acc.min_mag,
                acc.max_mag
            );
            check!(
                self.failures,
                acc.n == 50 && errs == 0 && acc.min_mag >= 10.0 && acc.max_mag <= 120.0,
                "{=str} final: 50 samples with |B| between 10 and 120 uT ({=u32} errors)",
                errs
            );
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    defmt::info!(
        "[INFO] bmm350_selftest: I2C0 SDA=GPIO12 SCL=GPIO13 400 kHz, INT=GPIO4, address 0x14"
    );
    spawner.spawn(count_edges(Input::new(p.PIN_4, Pull::Down)).unwrap());
    let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, Irqs, bus_config());
    let mut t = T {
        mag: Bmm350::new(i2c, bmm350::ADDRESS_ADSEL_LOW),
        delay: Delay,
        failures: 0,
        start: Instant::now(),
        base: Field::default(),
        base_std: [0.0; 3],
        deltas: [[0.0; 4]; 8],
        nd: 0,
        zt: None,
    };
    t.check1().await;
    if t.late() {
        t.skip(2)
    } else {
        t.check2().await
    }
    t.check3();
    if t.late() {
        t.skip(4)
    } else {
        t.check4().await
    }
    if t.late() {
        t.skip(5)
    } else {
        t.check5().await
    }
    if t.late() {
        t.skip(6)
    } else {
        t.check6().await
    }
    if t.late() {
        t.skip(7)
    } else {
        t.check7().await
    }
    t.finish().await;
    defmt::info!(
        "[INFO] run took {=u32} ms",
        t.start.elapsed().as_millis() as u32
    );
    hil_common::exit(t.failures)
}
