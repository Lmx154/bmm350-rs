//! On-board check of verify, recover, status reads, configure, mode changes and forced triggers.
//! BMM350 on I2C0 (SDA GPIO12, SCL GPIO13), INT on GPIO4.
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

use bmm350::timing::{Recharge, forced_measurement_us};
use bmm350::{
    Averaging, Bmm350, Config, Error, Field, InterruptMode, InterruptPin, InterruptPolarity,
    OutputDataRate, PowerMode, StartMode, reg,
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

/// Wait for an edge after count `e0`, at most `ms` milliseconds (the only timed wait in this file).
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

async fn count_for(ms: u64) -> u32 {
    let (e0, _) = edge_now();
    Timer::after_millis(ms).await;
    edge_now().0.wrapping_sub(e0)
}

fn now_us() -> u32 {
    Instant::now().as_micros() as u32
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

fn bus_config() -> i2c::Config {
    let mut cfg = i2c::Config::default();
    cfg.frequency = 400_000;
    cfg.sda_pullup = false;
    cfg.scl_pullup = false;
    cfg
}

fn cfg_a() -> Config {
    Config::new()
        .with_odr(OutputDataRate::Hz100)
        .with_averaging(Averaging::Avg2)
}
fn cfg_b() -> Config {
    Config::new()
        .with_odr(OutputDataRate::Hz400)
        .with_averaging(Averaging::Avg1)
}
fn cfg_c() -> Config {
    Config::new()
        .with_odr(OutputDataRate::Hz50)
        .with_averaging(Averaging::Avg8)
        .with_interrupt(
            InterruptPin::new()
                .with_mode(InterruptMode::Latched)
                .with_polarity(InterruptPolarity::ActiveLow),
        )
}
fn cfg_d() -> Config {
    cfg_a().with_start_mode(StartMode::Suspend)
}
fn cfg_forced(avg: Averaging) -> Config {
    Config::new()
        .with_odr(OutputDataRate::Hz50)
        .with_averaging(avg)
        .with_start_mode(StartMode::Suspend)
}

type Bus<'a> = I2c<'a, I2C0, Async>;
type DrvError = Error<i2c::Error>;

struct T<'a> {
    mag: Bmm350<Bus<'a>>,
    delay: Delay,
    failures: u32,
    err_bits: u8,
    illegal_seen: u32,
}

impl<'a> T<'a> {
    /// Print pmu_status and error_status; remember any error bit or illegal flag seen.
    async fn status(&mut self, label: &str) -> (Option<bmm350::PmuStatus>, u8) {
        let p = self.mag.pmu_status().await.ok();
        let e = self.mag.error_status().await.ok();
        let raw = e.map(|e| e.raw).unwrap_or(0);
        self.err_bits |= raw;
        if p.map(|p| p.illegal_command).unwrap_or(false) {
            self.illegal_seen += 1;
        }
        defmt::info!("[STAT] {=str}: pmu {} err {}", label, p, e);
        (p, raw)
    }

    async fn init(&mut self, label: &str, cfg: &Config) {
        let t0 = now_us();
        let r = self.mag.init(&mut self.delay, cfg).await;
        let us = now_us().wrapping_sub(t0);
        match r {
            Ok(_) => defmt::info!("[PASS] {=str}: init took {=u32} us", label, us),
            Err(e) => {
                self.failures += 1;
                defmt::error!("[FAIL] {=str}: init error {}", label, e);
            }
        }
        self.status(label).await;
    }

    async fn configure(&mut self, label: &str, cfg: &Config) -> u32 {
        let t0 = now_us();
        let r = self.mag.configure(&mut self.delay, cfg).await;
        let us = now_us().wrapping_sub(t0);
        match r {
            Ok(_) => defmt::info!("[PASS] {=str}: configure took {=u32} us", label, us),
            Err(e) => {
                self.failures += 1;
                defmt::error!("[FAIL] {=str}: configure error {}", label, e);
            }
        }
        self.status(label).await;
        us
    }

    async fn verify(&mut self, label: &str) -> bool {
        match self.mag.verify().await {
            Ok(()) => {
                defmt::info!("[PASS] {=str}: verify Ok", label);
                true
            }
            Err(Error::ConfigMismatch(m)) => {
                self.failures += 1;
                defmt::error!(
                    "[FAIL] {=str}: verify mismatch reg {=u8:#04x} mask {=u8:#04x} expected {=u8:#04x} found {=u8:#04x}",
                    label,
                    m.register,
                    m.mask,
                    m.expected,
                    m.found
                );
                false
            }
            Err(e) => {
                self.failures += 1;
                defmt::error!("[FAIL] {=str}: verify error {}", label, e);
                false
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    defmt::info!("[INFO] bmm350_ops: I2C0 SDA=GPIO12 SCL=GPIO13 400 kHz, INT=GPIO4");
    spawner.spawn(count_edges(Input::new(p.PIN_4, Pull::Down)).unwrap());

    let mut bus = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, Irqs, bus_config());
    let mut b = [0u8; 1];
    if bus
        .read_async(bmm350::DEFAULT_ADDRESS, &mut b)
        .await
        .is_err()
    {
        defmt::error!("[FAIL] scan: 0x14 does not acknowledge");
        hil_common::exit(1)
    }
    let mut t = T {
        mag: Bmm350::new(bus, bmm350::DEFAULT_ADDRESS),
        delay: Delay,
        failures: 0,
        err_bits: 0,
        illegal_seen: 0,
    };
    let start = Instant::now();

    // 1. verify after init and configure with four configurations.
    defmt::info!("[INFO] check 1: verify after init and configure");
    for (label, cfg) in [
        ("1 init a", cfg_a()),
        ("1 init b", cfg_b()),
        ("1 init c", cfg_c()),
        ("1 init d", cfg_d()),
    ] {
        t.init(label, &cfg).await;
        t.verify(label).await;
        let n = count_for(60).await;
        defmt::info!(
            "[STAT] {=str}: INT edges in 60 ms after it: {=u32}",
            label,
            n
        );
        t.verify("1 +60ms").await;
    }
    for (label, cfg) in [
        ("1 configure d->a", cfg_a()),
        ("1 configure a->b", cfg_b()),
        ("1 configure b->c", cfg_c()),
        ("1 configure c->d", cfg_d()),
        ("1 configure d->a", cfg_a()),
    ] {
        t.configure(label, &cfg).await;
        t.verify(label).await;
        let n = count_for(60).await;
        defmt::info!(
            "[STAT] {=str}: INT edges in 60 ms after it: {=u32}",
            label,
            n
        );
        t.verify("1 +60ms").await;
    }

    let n0 = count_for(300).await;
    let st = t.mag.interrupt_status().await.ok();
    let n1 = count_for(300).await;
    let st2 = t.mag.interrupt_status().await.ok();
    defmt::info!(
        "[STAT] 1 INT edges in 300 ms after the latched config c was replaced by a: {=u32}; INT_STATUS read {}; edges in the next 300 ms {=u32}; INT_STATUS read again {}",
        n0,
        st,
        n1,
        st2
    );
    let ic = t.mag.read_register(reg::INT_CTRL).await.ok();
    let mut st_a = [0u8; 3];
    let mut st_b = [0u8; 3];
    let _ = t.mag.read_registers(0x3A, &mut st_a).await;
    Timer::after_millis(50).await;
    let _ = t.mag.read_registers(0x3A, &mut st_b).await;
    let st3 = t.mag.interrupt_status().await.ok();
    defmt::info!(
        "[STAT] 1 wedge probe: INT_CTRL {}, SENSORTIME {=[u8]:#04x} then after 50 ms {=[u8]:#04x}, INT_STATUS after 50 ms {}",
        ic,
        st_a,
        st_b,
        st3
    );
    check!(
        t.failures,
        n0 >= 25,
        "{=str} 1: edges resume without reading INT_STATUS ({=u32} in 300 ms)",
        n0
    );
    check!(
        t.failures,
        n1 >= 25,
        "{=str} 1: edges resume after INT_STATUS was read ({=u32} in 300 ms)",
        n1
    );
    t.configure("1 wedge configure a again", &cfg_a()).await;
    let n4 = count_for(300).await;
    defmt::info!(
        "[STAT] 1 wedge: edges in 300 ms after a second configure a: {=u32}",
        n4
    );
    let _ = t.mag.set_power_mode(&mut t.delay, PowerMode::Suspend).await;
    let _ = t.mag.set_power_mode(&mut t.delay, PowerMode::Normal).await;
    let n5 = count_for(300).await;
    defmt::info!(
        "[STAT] 1 wedge: edges in 300 ms after set_power_mode Suspend then Normal: {=u32}",
        n5
    );
    // Workaround for the rest of the run: a full init, then the same path with INT_STATUS read while latched.
    t.init("1 re-init a", &cfg_a()).await;
    let n2 = count_for(300).await;
    check!(
        t.failures,
        n2 >= 25,
        "{=str} 1: edges after re-init a: {=u32} in 300 ms",
        n2
    );
    t.configure("1 exp configure c", &cfg_c()).await;
    Timer::after_millis(60).await;
    let pin_st = t.mag.interrupt_status().await.ok();
    t.configure("1 exp configure d", &cfg_d()).await;
    t.configure("1 exp configure a", &cfg_a()).await;
    let n3 = count_for(300).await;
    defmt::info!(
        "[STAT] 1 experiment: c latched, INT_STATUS read {}, then d, a: {=u32} edges in 300 ms",
        pin_st,
        n3
    );
    check!(
        t.failures,
        n3 >= 25,
        "{=str} 1: edges after leaving latched c with INT_STATUS read ({=u32} in 300 ms)",
        n3
    );
    if n3 < 25 {
        t.init("1 re-init a 2", &cfg_a()).await;
    }

    // 2. positive control and mode entry.
    defmt::info!("[INFO] check 2: positive control, set_power_mode");
    let before = t
        .mag
        .read_register(reg::PMU_CMD_AXIS_EN)
        .await
        .unwrap_or(0xFF);
    let _ = t
        .mag
        .write_register(reg::PMU_CMD_AXIS_EN, before ^ 0x04)
        .await;
    let after = t
        .mag
        .read_register(reg::PMU_CMD_AXIS_EN)
        .await
        .unwrap_or(0xFF);
    defmt::info!(
        "[STAT] 2 axis_en before {=u8:#04x} after raw write {=u8:#04x}",
        before,
        after
    );
    match t.mag.verify().await {
        Err(Error::ConfigMismatch(m)) => {
            defmt::info!(
                "[STAT] 2 mismatch reg {=u8:#04x} mask {=u8:#04x} expected {=u8:#04x} found {=u8:#04x}",
                m.register,
                m.mask,
                m.expected,
                m.found
            );
            check!(
                t.failures,
                m.register == reg::PMU_CMD_AXIS_EN,
                "{=str} 2: verify reports exactly PMU_CMD_AXIS_EN"
            );
        }
        other => check!(
            t.failures,
            false,
            "{=str} 2: verify after raw write returned {}",
            other.err()
        ),
    }
    t.configure("2 configure a", &cfg_a()).await;
    t.verify("2 after configure a").await;

    let t0 = now_us();
    let r = t.mag.set_power_mode(&mut t.delay, PowerMode::Suspend).await;
    let us = now_us().wrapping_sub(t0);
    check!(
        t.failures,
        r.is_ok(),
        "{=str} 2: set_power_mode Suspend took {=u32} us",
        us
    );
    let (p, _) = t.status("2 suspend").await;
    check!(
        t.failures,
        p.map(|p| !p.normal_mode).unwrap_or(false),
        "{=str} 2: normal-mode flag clear in suspend"
    );
    t.verify("2 suspend").await;
    let t0 = now_us();
    let r = t.mag.set_power_mode(&mut t.delay, PowerMode::Normal).await;
    let us = now_us().wrapping_sub(t0);
    check!(
        t.failures,
        r.is_ok(),
        "{=str} 2: set_power_mode Normal took {=u32} us",
        us
    );
    let (p, _) = t.status("2 normal").await;
    check!(
        t.failures,
        p.map(|p| p.normal_mode).unwrap_or(false),
        "{=str} 2: normal-mode flag set in normal"
    );
    t.verify("2 normal").await;

    // 3. configure on a running device.
    defmt::info!("[INFO] check 3: configure while streaming");
    Timer::after_millis(100).await;
    let n_before = count_for(1000).await;
    let us_ab = t.configure("3 a->b", &cfg_b()).await;
    Timer::after_millis(100).await;
    let n_b = count_for(1000).await;
    t.verify("3 b").await;
    let us_ba = t.configure("3 b->a", &cfg_a()).await;
    Timer::after_millis(100).await;
    let n_a = count_for(1000).await;
    t.verify("3 a again").await;
    defmt::info!(
        "[STAT] 3 edges per second: a {=u32}, b {=u32}, a again {=u32}; configure a->b {=u32} us, b->a {=u32} us",
        n_before,
        n_b,
        n_a,
        us_ab,
        us_ba
    );
    check!(
        t.failures,
        n_before.abs_diff(100) * 100 <= 300,
        "{=str} 3: 100 Hz before: {=u32} edges",
        n_before
    );
    check!(
        t.failures,
        n_b.abs_diff(400) * 100 <= 1200,
        "{=str} 3: 400 Hz after configure: {=u32} edges",
        n_b
    );
    check!(
        t.failures,
        n_a.abs_diff(100) * 100 <= 300,
        "{=str} 3: 100 Hz after configure back: {=u32} edges",
        n_a
    );

    // 4. forced measurements from the suspend start mode.
    defmt::info!("[INFO] check 4: forced measurements");
    for fast in [false, true] {
        for (avg, n) in [
            (Averaging::Avg1, 1u8),
            (Averaging::Avg2, 2),
            (Averaging::Avg4, 4),
            (Averaging::Avg8, 8),
        ] {
            let name = if fast { "4 FM_FAST" } else { "4 FM" };
            t.configure(name, &cfg_forced(avg)).await;
            Timer::after_millis(5).await;
            let (e0, _) = edge_now();
            let r = if fast {
                t.mag.trigger_forced_fast().await
            } else {
                t.mag.trigger_forced().await
            };
            let t0 = now_us();
            check!(
                t.failures,
                r.is_ok(),
                "{=str} {=str} avg {=u8}: trigger written",
                name,
                n
            );
            let figure =
                forced_measurement_us(avg, if fast { Recharge::Fast } else { Recharge::Full })
                    as i32;
            match wait_edge(e0, 100).await {
                Some(te) => {
                    let us = te.wrapping_sub(t0) as i32;
                    let (p, _) = t.status(name).await;
                    defmt::info!(
                        "[STAT] {=str} avg {=u8}: measured {=i32} us, driver figure {=i32} us",
                        name,
                        n,
                        us,
                        figure
                    );
                    check!(
                        t.failures,
                        us <= figure,
                        "{=str} {=str} avg {=u8}: edge time {=i32} us <= figure {=i32} us",
                        name,
                        n,
                        us,
                        figure
                    );
                    check!(
                        t.failures,
                        p.map(|p| !p.normal_mode && !p.illegal_command)
                            .unwrap_or(false),
                        "{=str} {=str} avg {=u8}: pmu normal flag and illegal flag clear after the edge",
                        name,
                        n
                    );
                    match t.mag.read_field().await {
                        Ok(f) => {
                            let m = magnitude(&f);
                            defmt::info!(
                                "[STAT] {=str} avg {=u8}: |B| {} uT, T {} C",
                                name,
                                n,
                                m,
                                f.temperature
                            );
                            check!(
                                t.failures,
                                (10.0..=120.0).contains(&m),
                                "{=str} {=str} avg {=u8}: |B| in 10..120 uT",
                                name,
                                n
                            );
                        }
                        Err(e) => check!(t.failures, false, "{=str} read failed {}", e),
                    }
                }
                None => {
                    defmt::info!(
                        "[STAT] {=str} avg {=u8}: no edge within 100 ms, driver figure {=i32} us",
                        name,
                        n,
                        figure
                    );
                    check!(
                        t.failures,
                        false,
                        "{=str} {=str} avg {=u8}: data-ready edge within 100 ms",
                        name,
                        n
                    );
                    t.status(name).await;
                }
            }
            t.verify(name).await;
        }
    }

    // 5. wrong-mode refusals.
    defmt::info!("[INFO] check 5: wrong-mode refusals");
    t.configure("5 configure a", &cfg_a()).await;
    Timer::after_millis(50).await;
    let (p0, _) = t.status("5 before").await;
    let r1 = t.mag.trigger_forced().await;
    check!(
        t.failures,
        matches!(r1, Err(Error::WrongMode { .. })),
        "{=str} 5: trigger_forced in normal returns WrongMode ({})",
        r1.err()
    );
    let r2 = t.mag.trigger_forced_fast().await;
    check!(
        t.failures,
        matches!(r2, Err(Error::WrongMode { .. })),
        "{=str} 5: trigger_forced_fast in normal returns WrongMode ({})",
        r2.err()
    );
    let (p1, _) = t.status("5 after refusals").await;
    check!(
        t.failures,
        p1.map(|p| !p.illegal_command).unwrap_or(false),
        "{=str} 5: illegal flag still clear after refusals"
    );
    let r3 = t.mag.set_power_mode(&mut t.delay, PowerMode::Normal).await;
    let (p2, _) = t.status("5 after set_power_mode Normal").await;
    check!(
        t.failures,
        r3.is_ok()
            && p1.map(|p| (p.status_0, p.status_1)) == p2.map(|p| (p.status_0, p.status_1))
            && p0.map(|p| (p.status_0, p.status_1)) == p1.map(|p| (p.status_0, p.status_1)),
        "{=str} 5: set_power_mode to the current mode leaves pmu_status unchanged"
    );

    // 6. recover on a healthy bus.
    defmt::info!("[INFO] check 6: recover");
    let t0 = now_us();
    let r = t.mag.recover(&mut t.delay).await;
    let us = now_us().wrapping_sub(t0);
    check!(
        t.failures,
        r.is_ok(),
        "{=str} 6: recover in normal took {=u32} us",
        us
    );
    t.verify("6 after recover normal").await;
    let n = count_for(1000).await;
    check!(
        t.failures,
        n.abs_diff(100) * 100 <= 300,
        "{=str} 6: 100 Hz after recover: {=u32} edges",
        n
    );
    let r = t.mag.set_power_mode(&mut t.delay, PowerMode::Suspend).await;
    check!(t.failures, r.is_ok(), "{=str} 6: set_power_mode Suspend");
    let t0 = now_us();
    let r = t.mag.recover(&mut t.delay).await;
    let us = now_us().wrapping_sub(t0);
    check!(
        t.failures,
        r.is_ok(),
        "{=str} 6: recover in suspend took {=u32} us",
        us
    );
    t.verify("6 after recover suspend").await;
    t.status("6 end").await;

    // 7. verify during activity.
    defmt::info!("[INFO] check 7: verify during activity");
    t.configure("7 configure b", &cfg_b()).await;
    Timer::after_millis(100).await;
    let (e0, _) = edge_now();
    let t0 = now_us();
    let mut bad = 0u32;
    for i in 0..200u32 {
        if let Err(e) = t.mag.verify().await {
            bad += 1;
            if bad <= 3 {
                defmt::info!("[STAT] 7 streaming verify #{=u32} returned {}", i, e);
            }
        }
    }
    let us = now_us().wrapping_sub(t0);
    let edges = edge_now().0.wrapping_sub(e0);
    defmt::info!(
        "[STAT] 7 streaming: 200 verifies took {=u32} us, {=u32} INT edges meanwhile, {=u32} not Ok",
        us,
        edges,
        bad
    );
    check!(
        t.failures,
        bad == 0,
        "{=str} 7: 200 verifies at 400 Hz, not Ok: {=u32}",
        bad
    );

    t.configure("7 configure forced avg8", &cfg_forced(Averaging::Avg8))
        .await;
    Timer::after_millis(5).await;
    let (e0, _) = edge_now();
    let r = t.mag.trigger_forced().await;
    check!(t.failures, r.is_ok(), "{=str} 7: trigger_forced written");
    let (mut bad, mut before_edge) = (0u32, 0u32);
    for i in 0..20u32 {
        let res = t.mag.verify().await;
        if edge_now().0 == e0 {
            before_edge += 1;
        }
        if let Err(e) = res {
            bad += 1;
            if bad <= 3 {
                defmt::info!("[STAT] 7 forced verify #{=u32} returned {}", i, e);
            }
        }
    }
    defmt::info!(
        "[STAT] 7 forced: 20 verifies, {=u32} finished before the data-ready edge, {=u32} not Ok",
        before_edge,
        bad
    );
    check!(
        t.failures,
        bad == 0,
        "{=str} 7: 20 verifies after a forced trigger, not Ok: {=u32}",
        bad
    );
    let _ = wait_edge(e0, 100).await;
    t.status("7 after forced").await;
    t.verify("7 after forced").await;

    // 8. health information.
    defmt::info!("[INFO] check 8: error register and illegal flag");
    let (p, raw) = t.status("8 final").await;
    check!(
        t.failures,
        raw == 0 && p.map(|p| !p.illegal_command).unwrap_or(false),
        "{=str} 8: no error bit and no illegal flag at the end (err {=u8:#04x})",
        raw
    );
    defmt::info!(
        "[STAT] 8 accumulated: ERR_REG bits seen {=u8:#04x}, illegal flag seen in {=u32} status reads",
        t.err_bits,
        t.illegal_seen
    );
    check!(
        t.failures,
        t.err_bits == 0,
        "{=str} 8: no ERR_REG bit seen in any read of the run ({=u8:#04x})",
        t.err_bits
    );

    // Leave the sensor as found: full init to the default configuration.
    t.init("end init a", &cfg_a()).await;
    t.verify("end").await;
    let el = start.elapsed().as_millis() as u32;
    defmt::info!("[INFO] run took {=u32} ms", el);
    check!(t.failures, el < 60_000, "{=str} run ended within 60 s");
    let _: Option<DrvError> = None;
    hil_common::exit(t.failures)
}
