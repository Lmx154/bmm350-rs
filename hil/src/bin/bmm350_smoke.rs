//! On-board check of the bmm350 driver: BMM350 on I2C0 (SDA GPIO12, SCL GPIO13), INT on GPIO4.
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

use bmm350::{Averaging, Bmm350, Config, Field, OutputDataRate, StartMode, reg};
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

/// Rising edges seen on INT, counted by a task that does nothing else.
static EDGES: AtomicU32 = AtomicU32::new(0);
/// Low 32 bits of the microsecond timestamp of the latest edge.
static EDGE_US: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
async fn count_edges(mut int: Input<'static>) {
    loop {
        int.wait_for_rising_edge().await;
        EDGE_US.store(Instant::now().as_micros() as u32, Relaxed);
        EDGES.fetch_add(1, Relaxed);
    }
}

/// Edge count and the timestamp of the latest edge (retries if an edge lands between the loads).
fn edge_now() -> (u32, u32) {
    loop {
        let c = EDGES.load(Relaxed);
        let t = EDGE_US.load(Relaxed);
        if EDGES.load(Relaxed) == c {
            return (c, t);
        }
    }
}

/// Reports one verdict line; the format string starts with `{=str}`, which receives the tag.
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

/// Square root by Newton iteration (the driver and this test use no libm).
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

fn bus_config() -> i2c::Config {
    let mut cfg = i2c::Config::default();
    cfg.frequency = 400_000;
    cfg.sda_pullup = false;
    cfg.scl_pullup = false;
    cfg
}

/// Result of one normal-mode run.
struct Run {
    samples: u32,
    edges: u32,
    missed: u32,
    rate_hz: f32,
    mean_read_us: u32,
    worst_read_us: u32,
    mean: Field,
    std: [f32; 3],
}

/// Read one sample per INT edge for `secs` seconds.
async fn stream<'a>(mag: &mut Bmm350<I2c<'a, I2C0, Async>>, secs: u64) -> Run {
    let end = Instant::now() + Duration::from_secs(secs);
    let (e0, _) = edge_now();
    let mut seen = e0;
    let (mut first, mut last) = ((0u32, 0u32), (0u32, 0u32));
    let (mut n, mut missed, mut sum_us, mut worst_us) = (0u32, 0u32, 0u64, 0u32);
    // Welford running mean of x, y, z, temperature and sum of squared deviations of x, y, z.
    let mut mean = [0.0f32; 4];
    let mut m2 = [0.0f32; 3];
    while Instant::now() < end {
        let (e, t) = edge_now();
        if e == seen {
            Timer::after_micros(20).await;
            continue;
        }
        missed += e - seen - 1;
        seen = e;
        if n == 0 {
            first = (e, t);
        }
        last = (e, t);
        let t0 = Instant::now();
        let Ok(f) = mag.read_field().await else {
            defmt::error!("[FAIL] read_field returned a bus error");
            continue;
        };
        let us = t0.elapsed().as_micros() as u32;
        sum_us += us as u64;
        worst_us = worst_us.max(us);
        n += 1;
        let v = [f.x, f.y, f.z, f.temperature];
        for i in 0..4 {
            let d = v[i] - mean[i];
            mean[i] += d / n as f32;
            if i < 3 {
                m2[i] += d * (v[i] - mean[i]);
            }
        }
    }
    let (e1, _) = edge_now();
    let span_us = last.1.wrapping_sub(first.1);
    let denom = n.max(2) as f32 - 1.0;
    Run {
        samples: n,
        edges: e1 - e0,
        missed,
        rate_hz: if span_us > 0 {
            (last.0 - first.0) as f32 * 1e6 / span_us as f32
        } else {
            0.0
        },
        mean_read_us: if n > 0 { (sum_us / n as u64) as u32 } else { 0 },
        worst_read_us: worst_us,
        mean: Field {
            x: mean[0],
            y: mean[1],
            z: mean[2],
            temperature: mean[3],
        },
        std: [
            sqrt(m2[0] / denom),
            sqrt(m2[1] / denom),
            sqrt(m2[2] / denom),
        ],
    }
}

/// Wait up to 300 ms for an edge after count `e0`; returns its timestamp.
async fn wait_edge(e0: u32) -> Option<u32> {
    let end = Instant::now() + Duration::from_millis(300);
    while Instant::now() < end {
        let (e, t) = edge_now();
        if e != e0 {
            return Some(t);
        }
        Timer::after_micros(20).await;
    }
    None
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());
    let mut failures = 0u32;
    defmt::info!("[INFO] bmm350_smoke: I2C0 SDA=GPIO12 SCL=GPIO13 400 kHz, INT=GPIO4");

    spawner.spawn(count_edges(Input::new(p.PIN_4, Pull::Down)).unwrap());

    let mut i2c = I2c::new_async(
        p.I2C0.reborrow(),
        p.PIN_13.reborrow(),
        p.PIN_12.reborrow(),
        Irqs,
        bus_config(),
    );

    let mut address = None;
    for a in 0x08u8..0x78 {
        let mut b = [0u8; 1];
        if i2c.read_async(a, &mut b).await.is_ok() {
            defmt::info!("[INFO] scan: address {=u8:#04x} acknowledges", a);
            if address.is_none()
                && (a == bmm350::ADDRESS_ADSEL_LOW || a == bmm350::ADDRESS_ADSEL_HIGH)
            {
                address = Some(a);
            }
        }
    }
    let Some(address) = address else {
        defmt::error!(
            "[FAIL] scan: neither 0x14 nor 0x15 acknowledges (hardware finding, stopping)"
        );
        hil_common::exit(1)
    };

    let mut mag = Bmm350::new(i2c, address);
    let mut delay = Delay;
    let base = Config::new()
        .with_odr(OutputDataRate::Hz100)
        .with_averaging(Averaging::Avg2);

    // 1. init, identity, OTP coefficients.
    let t0 = Instant::now();
    let r = mag.init(&mut delay, &base).await;
    let init_ms = t0.elapsed().as_millis() as u32;
    check!(
        failures,
        r.is_ok(),
        "{=str} init at {=u8:#04x} took {=u32} ms",
        address,
        init_ms
    );
    if let Err(e) = r {
        defmt::error!("[INFO] init error: {}", e);
        fail_exit(failures)
    }
    let id = mag.read_register(reg::CHIP_ID).await;
    check!(
        failures,
        id == Ok(reg::CHIP_ID_VALUE),
        "{=str} CHIP_ID = {} (expect 0x33)",
        id.ok()
    );
    defmt::info!("[INFO] OTP calibration: {}", mag.calibration());
    let s0 = mag.read_register(reg::PMU_CMD_STATUS_0).await.ok();
    let s1 = mag.read_register(reg::PMU_CMD_STATUS_1).await.ok();
    let ic = mag.read_register(reg::INT_CTRL).await.ok();
    let er = mag.read_register(reg::ERR_REG).await.ok();
    defmt::info!(
        "[INFO] PMU_CMD_STATUS_0={} PMU_CMD_STATUS_1={} INT_CTRL={} ERR_REG={}",
        s0,
        s1,
        ic,
        er
    );

    // 2. normal mode at 100 Hz and at the highest ODR.
    let mut mean100 = Field::default();
    // 400 Hz allows averaging 1 only (datasheet Table 5); `init` now rejects any other pair instead
    // of lowering the averaging silently, so the pair is stated here.
    for (odr, avg, label, nominal) in [
        (OutputDataRate::Hz100, Averaging::Avg2, "100 Hz", 100.0f32),
        (OutputDataRate::Hz400, Averaging::Avg1, "400 Hz", 400.0),
    ] {
        let cfg = base.with_odr(odr).with_averaging(avg);
        let ok = mag.init(&mut delay, &cfg).await.is_ok();
        check!(failures, ok, "{=str} {=str}: init", label);
        let run = stream(&mut mag, 2).await;
        defmt::info!(
            "[INFO] {=str}: samples {=u32}, INT edges {=u32}, missed edges {=u32}, mean read {=u32} us, worst read {=u32} us",
            label,
            run.samples,
            run.edges,
            run.missed,
            run.mean_read_us,
            run.worst_read_us
        );
        let st0 = mag.read_register(reg::PMU_CMD_STATUS_0).await.unwrap_or(0);
        defmt::info!(
            "[INFO] {=str}: PMU_CMD_STATUS_0 after the run {=u8:#04x}",
            label,
            st0
        );
        check!(
            failures,
            st0 & reg::PMU_STATUS_0_NORMAL != 0,
            "{=str} {=str}: chip reports normal mode",
            label
        );
        let err_pct = (run.rate_hz / nominal - 1.0) * 100.0;
        defmt::info!(
            "[INFO] {=str}: measured rate {} Hz vs nominal {} Hz ({}%)",
            label,
            run.rate_hz,
            nominal,
            err_pct
        );
        check!(
            failures,
            run.samples as f32 >= nominal * 2.0 * 0.9,
            "{=str} {=str}: samples {=u32} of about {=u32} expected",
            label,
            run.samples,
            (nominal * 2.0) as u32
        );
        check!(
            failures,
            err_pct.abs() < 5.0,
            "{=str} {=str}: rate within 5% of nominal",
            label
        );
        check!(
            failures,
            run.missed == 0,
            "{=str} {=str}: missed data-ready edges = {=u32}",
            label,
            run.missed
        );
        check!(
            failures,
            run.edges.abs_diff(run.samples) <= 1,
            "{=str} {=str}: INT edges {=u32} vs samples {=u32}",
            label,
            run.edges,
            run.samples
        );
        defmt::info!(
            "[INFO] {=str}: mean field x {} y {} z {} uT, |B| {} uT, temperature {} C",
            label,
            run.mean.x,
            run.mean.y,
            run.mean.z,
            magnitude(&run.mean),
            run.mean.temperature
        );
        check!(
            failures,
            plausible(&run.mean),
            "{=str} {=str}: |B| in 10..120 uT and temperature in 10..50 C",
            label
        );
        defmt::info!(
            "[STAT] {=str}: std x {} y {} z {} uT",
            label,
            run.std[0],
            run.std[1],
            run.std[2]
        );
        if odr == OutputDataRate::Hz100 {
            mean100 = run.mean;
        }
    }
    let _ = mean100;

    // 3. forced mode once: from suspend, trigger, wait for data-ready, read.
    let cfg = base.with_start_mode(StartMode::Suspend);
    let ok = mag.init(&mut delay, &cfg).await.is_ok();
    check!(failures, ok, "{=str} forced: init to suspend");
    let (e0, _) = edge_now();
    let t0 = Instant::now();
    let trig = mag.trigger_forced().await;
    check!(failures, trig.is_ok(), "{=str} forced: trigger written");
    match wait_edge(e0).await {
        Some(t_edge) => {
            let us = t_edge.wrapping_sub(t0.as_micros() as u32);
            defmt::info!("[INFO] forced: trigger to data-ready edge {=u32} us", us);
            match mag.read_field().await {
                Ok(f) => {
                    defmt::info!(
                        "[INFO] forced: x {} y {} z {} uT, |B| {} uT, temperature {} C",
                        f.x,
                        f.y,
                        f.z,
                        magnitude(&f),
                        f.temperature
                    );
                    check!(
                        failures,
                        plausible(&f),
                        "{=str} forced: |B| in 10..120 uT and temperature in 10..50 C"
                    );
                }
                Err(e) => check!(failures, false, "{=str} forced: read failed {}", e),
            }
        }
        None => check!(
            failures,
            false,
            "{=str} forced: no data-ready edge within 300 ms"
        ),
    }
    let st = mag.interrupt_status().await;
    defmt::info!("[INFO] forced: INT_STATUS after read {}", st.ok());

    // 5. the blocking front-end on the blocking embassy-rp bus.
    drop(mag);
    let bus = I2c::new_blocking(
        p.I2C0.reborrow(),
        p.PIN_13.reborrow(),
        p.PIN_12.reborrow(),
        bus_config(),
    );
    let mut bmag = bmm350::blocking::Bmm350::new(bus, address);
    let cfg = base.with_start_mode(StartMode::Suspend);
    let (e0, _) = edge_now();
    let ok = bmag.init(&mut delay, &cfg).is_ok() && bmag.trigger_forced().is_ok();
    check!(failures, ok, "{=str} blocking: init");
    match wait_edge(e0).await {
        Some(_) => match bmag.read_field() {
            Ok(f) => {
                defmt::info!(
                    "[INFO] blocking: x {} y {} z {} uT, |B| {} uT, temperature {} C",
                    f.x,
                    f.y,
                    f.z,
                    magnitude(&f),
                    f.temperature
                );
                check!(
                    failures,
                    plausible(&f),
                    "{=str} blocking: |B| in 10..120 uT and temperature in 10..50 C"
                );
            }
            Err(e) => check!(failures, false, "{=str} blocking: read failed {}", e),
        },
        None => check!(
            failures,
            false,
            "{=str} blocking: no data-ready edge within 300 ms"
        ),
    }

    hil_common::exit(failures)
}

fn fail_exit(failures: u32) -> ! {
    hil_common::exit(failures.max(1))
}
