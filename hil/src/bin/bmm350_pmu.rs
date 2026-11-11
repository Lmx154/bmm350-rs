//! Fact-finding for the BMM350 PMU commands: raw register access only, no driver `init`.
//! BMM350 on I2C0 (SDA GPIO12, SCL GPIO13), INT on GPIO4.
//! Writes only CMD (soft reset), PMU_CMD (SUS, NM, UPD_OAE, FM, FM_FAST), PMU_CMD_AGGR_SET,
//! PMU_CMD_AXIS_EN and INT_CTRL. No bus transfer is ever cancelled.
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

use bmm350::reg;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::i2c::{self, Async, I2c};
use embassy_rp::peripherals::I2C0;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::i2c::I2c as _;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

const LOG_CAP: usize = 700;
static LOG: [AtomicU32; LOG_CAP] = [const { AtomicU32::new(0) }; LOG_CAP];
static LOG_N: AtomicU32 = AtomicU32::new(0);

/// Pulsed data-ready, active high, push-pull, mapped to the pin, data-ready enabled.
const INT_CTRL_VALUE: u8 = 0x8E;

#[embassy_executor::task]
async fn count_edges(mut int: Input<'static>) {
    loop {
        int.wait_for_rising_edge().await;
        let t = Instant::now().as_micros() as u32;
        let n = LOG_N.load(Relaxed) as usize;
        if n < LOG_CAP {
            LOG[n].store(t, Relaxed);
            LOG_N.store(n as u32 + 1, Relaxed);
        }
    }
}

fn arm() {
    LOG_N.store(0, Relaxed);
}

fn now_us() -> u32 {
    Instant::now().as_micros() as u32
}

struct Dev<'a> {
    i2c: I2c<'a, I2C0, Async>,
    addr: u8,
    bus_err: u32,
}

impl Dev<'_> {
    async fn rd2(&mut self, r: u8) -> (u8, u8) {
        let mut b = [0u8; reg::DUMMY_BYTES + 2];
        match self.i2c.write_read(self.addr, &[r], &mut b).await {
            Ok(()) => (b[2], b[3]),
            Err(_) => {
                self.bus_err += 1;
                (0xEE, 0xEE)
            }
        }
    }

    async fn rd(&mut self, r: u8) -> u8 {
        self.rd2(r).await.0
    }

    /// The only write path: refuses everything outside the safety envelope.
    async fn wr(&mut self, r: u8, v: u8) {
        let allowed = match r {
            reg::CMD => v == reg::CMD_SOFTRESET || v == reg::CMD_NOP,
            reg::PMU_CMD => matches!(
                v,
                reg::PMU_CMD_SUS
                    | reg::PMU_CMD_NM
                    | reg::PMU_CMD_UPD_OAE
                    | reg::PMU_CMD_FM
                    | reg::PMU_CMD_FM_FAST
            ),
            reg::PMU_CMD_AGGR_SET | reg::PMU_CMD_AXIS_EN | reg::INT_CTRL => true,
            _ => false,
        };
        if !allowed {
            defmt::panic!(
                "write outside the safety envelope: {=u8:#04x} <- {=u8:#04x}",
                r,
                v
            );
        }
        if self.i2c.write(self.addr, &[r, v]).await.is_err() {
            self.bus_err += 1;
        }
    }
}

fn show(label: &'static str, t: u32, s0: u8, s1: u8) {
    defmt::info!(
        "[STAT] {=str} t={=u32}us s0={=u8:#04x} s1={=u8:#04x} busy={=bool} ill={=bool} norm={=bool} echo={=u8} odr={=u8} avg={=u8}",
        label,
        t,
        s0,
        s1,
        s0 & 1 != 0,
        s0 & 0x10 != 0,
        s0 & 0x08 != 0,
        s0 >> 5,
        s1 & 0x0F,
        (s1 >> 4) & 3
    );
}

async fn show_now(d: &mut Dev<'_>, label: &'static str) -> (u8, u8) {
    let (s0, s1) = d.rd2(reg::PMU_CMD_STATUS_0).await;
    show(label, 0, s0, s1);
    (s0, s1)
}

struct Tr {
    t0: Instant,
    first_edge: i32,
    n_edges: u32,
    busy_clear: i32,
}

impl Tr {
    fn t0_us(&self) -> u32 {
        self.t0.as_micros() as u32
    }
}

/// Send one PMU command, read STATUS_0/1 at once and then every 1 ms for `dur_ms`, print each
/// change with its time (from just before each read, relative to just before the write).
async fn pmu(
    d: &mut Dev<'_>,
    label: &'static str,
    cmd: u8,
    dur_ms: u64,
    keep_log: bool,
    quiet: bool,
) -> Tr {
    if !keep_log {
        arm();
    }
    let t0 = Instant::now();
    let t0_us = t0.as_micros() as u32;
    d.wr(reg::PMU_CMD, cmd).await;
    let mut ent = [(0u32, 0u8, 0u8); 40];
    let mut ne = 0usize;
    let mut dropped = 0u32;
    let mut last = (0u8, 0u8);
    let mut busy_seen = false;
    let mut busy_clear = -1i32;
    for k in 0..=dur_ms {
        if k > 0 {
            Timer::at(t0 + Duration::from_millis(k)).await;
        }
        let t = now_us().wrapping_sub(t0_us);
        let (s0, s1) = d.rd2(reg::PMU_CMD_STATUS_0).await;
        if s0 & 1 != 0 {
            busy_seen = true;
        } else if busy_seen && busy_clear < 0 {
            busy_clear = t as i32;
        }
        if k == 0 || (s0, s1) != last {
            if let Some(slot) = ent.get_mut(ne) {
                *slot = (t, s0, s1);
                ne += 1;
            } else {
                dropped += 1;
            }
            last = (s0, s1);
        }
    }
    let n = (LOG_N.load(Relaxed) as usize).min(LOG_CAP);
    let first_edge = if n > 0 && !keep_log {
        LOG[0].load(Relaxed).wrapping_sub(t0_us) as i32
    } else {
        -1
    };
    if !quiet {
        defmt::info!("[STAT] {=str} cmd={=u8} sent", label, cmd);
        for &(t, s0, s1) in ent.iter().take(ne) {
            show(label, t, s0, s1);
        }
        defmt::info!(
            "[STAT] {=str} summary: busy_clear_us={=i32} edges_in_trace={=u32} first_edge_us={=i32} dropped={=u32}",
            label,
            busy_clear,
            n as u32,
            first_edge,
            dropped
        );
    }
    Tr {
        t0,
        first_edge,
        n_edges: n as u32,
        busy_clear,
    }
}

async fn soft_reset(d: &mut Dev<'_>, label: &'static str, quiet: bool) {
    Timer::after_millis(3).await;
    d.wr(reg::CMD, reg::CMD_SOFTRESET).await;
    Timer::after_millis(24).await;
    d.wr(reg::CMD, reg::CMD_NOP).await;
    let id = d.rd(reg::CHIP_ID).await;
    let (s0, s1) = d.rd2(reg::PMU_CMD_STATUS_0).await;
    if !quiet {
        defmt::info!(
            "[STAT] {=str} soft reset done: chip_id={=u8:#04x}",
            label,
            id
        );
        show(label, 0, s0, s1);
    }
}

/// Edge statistics of the log, relative to `t_ref`. Returns the rate of the edges after
/// `post_from_us`.
fn edge_report(label: &'static str, t_ref: u32, per_us: i32, post_from_us: i32) -> f32 {
    let n = (LOG_N.load(Relaxed) as usize).min(LOG_CAP);
    let mut d = [0i32; LOG_CAP];
    for (i, slot) in d.iter_mut().enumerate().take(n) {
        *slot = LOG[i].load(Relaxed).wrapping_sub(t_ref) as i32;
    }
    let d = &d[..n];
    if n == 0 {
        defmt::info!("[STAT] {=str} edges: none", label);
        return 0.0;
    }
    fn rate(d: &[i32], lo: i32, hi: i32) -> (u32, f32) {
        let (mut m, mut first, mut last) = (0u32, 0i32, 0i32);
        for &x in d {
            if x >= lo && x < hi {
                if m == 0 {
                    first = x;
                }
                last = x;
                m += 1;
            }
        }
        if m >= 2 && last > first {
            (m, (m - 1) as f32 * 1e6 / (last - first) as f32)
        } else {
            (m, 0.0)
        }
    }
    let (pre_n, pre_hz) = rate(d, i32::MIN, 0);
    let (post_n, post_hz) = rate(d, post_from_us, i32::MAX);
    let (all_n, all_hz) = rate(d, 0, i32::MAX);
    let mut settle = -1i32;
    let mut settle_i = -1i32;
    for i in 0..n.saturating_sub(3) {
        if d[i] >= 0 && (1..=3).all(|k| ((d[i + k] - d[i + k - 1]) - per_us).abs() * 5 <= per_us) {
            settle = d[i];
            settle_i = i as i32;
            break;
        }
    }
    let start = d.iter().position(|&x| x >= 0).unwrap_or(0);
    let mut iv = [0i32; 10];
    let mut ni = 0;
    for i in start..n.saturating_sub(1) {
        if ni < iv.len() {
            iv[ni] = d[i + 1] - d[i];
            ni += 1;
        }
    }
    defmt::info!(
        "[STAT] {=str} edges: before_t0 n={=u32} {=f32}Hz | after_t0 n={=u32} {=f32}Hz | late n={=u32} {=f32}Hz | first_after_t0_us={=i32} settle_us={=i32} settle_idx={=i32}",
        label,
        pre_n,
        pre_hz,
        all_n,
        all_hz,
        post_n,
        post_hz,
        d.iter().copied().find(|&x| x >= 0).unwrap_or(-1),
        settle,
        settle_i
    );
    defmt::info!(
        "[STAT] {=str} first intervals after t0 (us): {}",
        label,
        &iv[..ni]
    );
    post_hz
}

async fn window(label: &'static str, ms: u64, per_us: i32, post_from_ms: i32) -> f32 {
    arm();
    let t_ref = now_us();
    Timer::after_millis(ms).await;
    edge_report(label, t_ref, per_us, post_from_ms * 1000)
}

fn bus_config() -> i2c::Config {
    let mut cfg = i2c::Config::default();
    cfg.frequency = 400_000;
    cfg.sda_pullup = false;
    cfg.scl_pullup = false;
    cfg
}

/// AGGR_SET, optionally UPD_OAE, AXIS_EN, INT_CTRL. Chip in suspend.
async fn setup(d: &mut Dev<'_>, label: &'static str, aggr: u8, upd: bool, quiet: bool) {
    d.wr(reg::PMU_CMD_AGGR_SET, aggr).await;
    if upd {
        pmu(d, label, reg::PMU_CMD_UPD_OAE, 100, false, quiet).await;
    }
    d.wr(reg::PMU_CMD_AXIS_EN, 0x07).await;
    d.wr(reg::INT_CTRL, INT_CTRL_VALUE).await;
}

/// Send one PMU command and poll STATUS_0/1 back to back (about 0.15 ms per read) until busy has
/// been seen and cleared, or `max_ms` passed. Prints every change with the time of the read.
async fn pmu_fine(d: &mut Dev<'_>, label: &'static str, cmd: u8, max_ms: u32) {
    arm();
    let t0 = now_us();
    d.wr(reg::PMU_CMD, cmd).await;
    let mut last = (0u8, 0u8);
    let mut seen = false;
    let mut first = true;
    let mut tprev = 0u32;
    let mut n_reads = 0u32;
    loop {
        let t = now_us().wrapping_sub(t0);
        let (s0, s1) = d.rd2(reg::PMU_CMD_STATUS_0).await;
        n_reads += 1;
        if first || (s0, s1) != last {
            show(label, t, s0, s1);
            last = (s0, s1);
            first = false;
        }
        if s0 & 1 != 0 {
            seen = true;
        } else if seen {
            defmt::info!(
                "[STAT] {=str} busy cleared between {=u32}us and {=u32}us ({=u32} reads)",
                label,
                tprev,
                t,
                n_reads
            );
            break;
        }
        tprev = t;
        if t > max_ms * 1000 {
            defmt::info!(
                "[STAT] {=str} busy not cleared / not seen within {=u32} ms",
                label,
                max_ms
            );
            break;
        }
    }
    let n = (LOG_N.load(Relaxed) as usize).min(LOG_CAP);
    if n > 0 {
        defmt::info!(
            "[STAT] {=str} first edge at {=i32}us",
            label,
            LOG[0].load(Relaxed).wrapping_sub(t0) as i32
        );
    }
}

fn near(x: f32, target: f32) -> bool {
    x > target * 0.9 && x < target * 1.1
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let start = Instant::now();
    let deadline = start + Duration::from_secs(52);
    defmt::info!(
        "[INFO] bmm350_pmu: I2C0 SDA=GPIO12 SCL=GPIO13 400 kHz, INT=GPIO4, raw registers only"
    );
    spawner.spawn(count_edges(Input::new(p.PIN_4, Pull::Down)).unwrap());

    let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, Irqs, bus_config());
    let mut d = Dev {
        i2c,
        addr: bmm350::ADDRESS_ADSEL_LOW,
        bus_err: 0,
    };
    let mut failures = 0u32;
    let id = d.rd(reg::CHIP_ID).await;
    defmt::info!("[INFO] CHIP_ID before anything = {=u8:#04x}", id);
    if id != reg::CHIP_ID_VALUE {
        defmt::error!("[FAIL] sensor does not answer with CHIP_ID 0x33");
        hil_common::exit(1);
    }

    // ---------------------------------------------------------------- S
    defmt::info!("[INFO] ===== S: state found (no soft reset: it ends the boot state, see H)");
    let (s0, _) = show_now(&mut d, "S.initial").await;
    for (n, r) in [
        ("AGGR_SET", reg::PMU_CMD_AGGR_SET),
        ("AXIS_EN", reg::PMU_CMD_AXIS_EN),
        ("INT_CTRL", reg::INT_CTRL),
        ("ERR_REG", reg::ERR_REG),
    ] {
        let v = d.rd(r).await;
        defmt::info!("[STAT] S.{=str}={=u8:#04x}", n, v);
    }
    if s0 & 0x08 != 0 {
        pmu(
            &mut d,
            "S.SUS from normal",
            reg::PMU_CMD_SUS,
            100,
            false,
            false,
        )
        .await;
    }
    d.wr(reg::PMU_CMD_AXIS_EN, 0x07).await;
    d.wr(reg::INT_CTRL, INT_CTRL_VALUE).await;
    pmu(&mut d, "S.NM", reg::PMU_CMD_NM, 150, false, false).await;
    pmu(&mut d, "S.SUS", reg::PMU_CMD_SUS, 100, false, false).await;

    // ---------------------------------------------------------------- A
    defmt::info!("[INFO] ===== A: AGGR_SET 400 Hz avg 1, UPD_OAE, AXIS_EN, INT_CTRL, NM");
    d.wr(reg::PMU_CMD_AGGR_SET, 0x02).await;
    pmu(&mut d, "A.UPD_OAE", reg::PMU_CMD_UPD_OAE, 100, false, false).await;
    d.wr(reg::PMU_CMD_AXIS_EN, 0x07).await;
    d.wr(reg::INT_CTRL, INT_CTRL_VALUE).await;
    let ag = d.rd(reg::PMU_CMD_AGGR_SET).await;
    let er = d.rd(reg::ERR_REG).await;
    defmt::info!(
        "[STAT] A.readback AGGR_SET={=u8:#04x} ERR_REG={=u8:#04x}",
        ag,
        er
    );
    pmu(&mut d, "A.NM", reg::PMU_CMD_NM, 100, false, false).await;
    let a_hz = window("A.rate1s", 1000, 2500, 0).await;
    show_now(&mut d, "A.after1s").await;
    let a_ok = near(a_hz, 400.0);

    // ---------------------------------------------------------------- P and B
    defmt::info!("[INFO] ===== P: from A, AGGR_SET 100 Hz avg 2 and NM without UPD_OAE");
    pmu(&mut d, "P.SUS", reg::PMU_CMD_SUS, 100, false, false).await;
    d.wr(reg::PMU_CMD_AGGR_SET, 0x14).await;
    show_now(&mut d, "P.after AGGR_SET write (suspend)").await;
    pmu(&mut d, "P.NM", reg::PMU_CMD_NM, 100, false, false).await;
    window("P.rate500ms", 500, 10_000, 0).await;
    show_now(&mut d, "P.end").await;
    defmt::info!("[INFO] ===== B: AGGR_SET 400 Hz avg 1, NM without UPD_OAE");
    pmu(&mut d, "B.SUS", reg::PMU_CMD_SUS, 100, false, false).await;
    d.wr(reg::PMU_CMD_AGGR_SET, 0x02).await;
    show_now(&mut d, "B.after AGGR_SET write (suspend)").await;
    pmu(&mut d, "B.NM", reg::PMU_CMD_NM, 100, false, false).await;
    let b_hz = window("B.rate1s", 1000, 2500, 0).await;
    show_now(&mut d, "B.after1s").await;
    let b_ok = near(b_hz, 400.0);
    let use_upd = !b_ok;
    defmt::info!(
        "[INFO] A 400Hz ok={=bool} B 400Hz ok={=bool} -> later experiments use UPD_OAE={=bool}",
        a_ok,
        b_ok,
        use_upd
    );

    // ---------------------------------------------------------------- C
    if Instant::now() < deadline {
        defmt::info!("[INFO] ===== C: normal mode 400 Hz, AGGR_SET 100 Hz avg 2, then UPD_OAE");
        show_now(&mut d, "C.running").await;
        arm();
        Timer::after_millis(300).await;
        d.wr(reg::PMU_CMD_AGGR_SET, 0x14).await;
        show_now(&mut d, "C.after AGGR_SET write").await;
        let tr = pmu(&mut d, "C.UPD_OAE", reg::PMU_CMD_UPD_OAE, 100, true, false).await;
        Timer::at(tr.t0 + Duration::from_millis(400)).await;
        edge_report("C", tr.t0_us(), 10_000, 250_000);
        show_now(&mut d, "C.end").await;
        let er = d.rd(reg::ERR_REG).await;
        defmt::info!("[STAT] C.ERR_REG={=u8:#04x}", er);

        // ------------------------------------------------------------ D
        defmt::info!("[INFO] ===== D: normal mode, AGGR_SET 50 Hz avg 4 without UPD_OAE");
        arm();
        let t_ref = now_us();
        d.wr(reg::PMU_CMD_AGGR_SET, 0x25).await;
        Timer::after_millis(500).await;
        edge_report("D.noupd", t_ref, 20_000, 250_000);
        show_now(&mut d, "D.after500ms").await;
        pmu(&mut d, "D.SUS", reg::PMU_CMD_SUS, 100, false, false).await;
        pmu(&mut d, "D.NM", reg::PMU_CMD_NM, 150, false, false).await;
        window("D.afterNM", 300, 20_000, 100).await;
        show_now(&mut d, "D.end").await;
    }

    // ---------------------------------------------------------------- E
    if Instant::now() < deadline {
        defmt::info!("[INFO] ===== E: suspend, SUS again, forced mode and averaging");
        pmu(
            &mut d,
            "E.SUS from normal",
            reg::PMU_CMD_SUS,
            100,
            false,
            false,
        )
        .await;
        pmu(
            &mut d,
            "E.SUS in suspend",
            reg::PMU_CMD_SUS,
            100,
            false,
            false,
        )
        .await;
        d.wr(reg::PMU_CMD_AGGR_SET, 0x05).await;
        pmu(
            &mut d,
            "E.FM avg1 noUPD",
            reg::PMU_CMD_FM,
            100,
            false,
            false,
        )
        .await;
        d.wr(reg::PMU_CMD_AGGR_SET, 0x35).await;
        pmu(
            &mut d,
            "E.FM avg8 noUPD",
            reg::PMU_CMD_FM,
            100,
            false,
            false,
        )
        .await;
        d.wr(reg::PMU_CMD_AGGR_SET, 0x05).await;
        pmu(
            &mut d,
            "E.UPD avg1",
            reg::PMU_CMD_UPD_OAE,
            100,
            false,
            false,
        )
        .await;
        pmu(&mut d, "E.FM avg1 +UPD", reg::PMU_CMD_FM, 100, false, false).await;
        d.wr(reg::PMU_CMD_AGGR_SET, 0x35).await;
        pmu(
            &mut d,
            "E.UPD avg8",
            reg::PMU_CMD_UPD_OAE,
            100,
            false,
            false,
        )
        .await;
        pmu(&mut d, "E.FM avg8 +UPD", reg::PMU_CMD_FM, 100, false, false).await;
        d.wr(reg::PMU_CMD_AGGR_SET, 0x05).await;
        pmu(
            &mut d,
            "E.FMFAST avg1 noUPD",
            reg::PMU_CMD_FM_FAST,
            100,
            false,
            false,
        )
        .await;
        d.wr(reg::PMU_CMD_AGGR_SET, 0x35).await;
        pmu(
            &mut d,
            "E.FMFAST avg8 noUPD",
            reg::PMU_CMD_FM_FAST,
            100,
            false,
            false,
        )
        .await;
        let er = d.rd(reg::ERR_REG).await;
        defmt::info!("[STAT] E.ERR_REG={=u8:#04x}", er);
    }

    // ---------------------------------------------------------------- F
    if Instant::now() < deadline {
        defmt::info!("[INFO] ===== F: what clears the illegal flag");
        pmu(
            &mut d,
            "F.1 SUS in suspend",
            reg::PMU_CMD_SUS,
            100,
            false,
            false,
        )
        .await;
        Timer::after_millis(200).await;
        show_now(&mut d, "F.1 status read again +300ms").await;
        let st = d.rd(reg::INT_STATUS).await;
        let er = d.rd(reg::ERR_REG).await;
        defmt::info!(
            "[STAT] F.1 INT_STATUS={=u8:#04x} ERR_REG={=u8:#04x} (read both)",
            st,
            er
        );
        show_now(&mut d, "F.1 status after INT_STATUS+ERR_REG read").await;
        pmu(
            &mut d,
            "F.2 NM (legal in suspend)",
            reg::PMU_CMD_NM,
            100,
            false,
            false,
        )
        .await;
        pmu(
            &mut d,
            "F.3 NM in normal",
            reg::PMU_CMD_NM,
            100,
            false,
            false,
        )
        .await;
        pmu(
            &mut d,
            "F.4 UPD_OAE in normal",
            reg::PMU_CMD_UPD_OAE,
            100,
            false,
            false,
        )
        .await;
        pmu(
            &mut d,
            "F.5 SUS from normal",
            reg::PMU_CMD_SUS,
            100,
            false,
            false,
        )
        .await;
        pmu(
            &mut d,
            "F.6 SUS in suspend",
            reg::PMU_CMD_SUS,
            100,
            false,
            false,
        )
        .await;
        pmu(
            &mut d,
            "F.7 FM (legal in suspend)",
            reg::PMU_CMD_FM,
            100,
            false,
            false,
        )
        .await;
    }

    // ---------------------------------------------------------------- G
    if Instant::now() < deadline {
        defmt::info!(
            "[INFO] ===== G: SUS then NM to first data-ready edge, 10 repetitions (no reset)"
        );
        let mut v = [0i32; 10];
        let (mut mn, mut mx, mut sum) = (i32::MAX, i32::MIN, 0i64);
        let mut bc = [0i32; 10];
        for i in 0..10 {
            pmu(&mut d, "G.SUS", reg::PMU_CMD_SUS, 20, false, true).await;
            Timer::after_millis(20).await;
            d.wr(reg::PMU_CMD_AGGR_SET, 0x02).await;
            if use_upd {
                pmu(&mut d, "G.UPD", reg::PMU_CMD_UPD_OAE, 20, false, true).await;
            }
            let tr = pmu(&mut d, "G.NM", reg::PMU_CMD_NM, 150, false, true).await;
            v[i] = tr.first_edge;
            bc[i] = tr.busy_clear;
            if tr.n_edges == 0 {
                defmt::info!("[STAT] G rep {=usize}: no edge within 150 ms", i);
            }
            mn = mn.min(tr.first_edge);
            mx = mx.max(tr.first_edge);
            sum += tr.first_edge as i64;
        }
        defmt::info!("[STAT] G first-edge us per rep: {}", &v[..]);
        defmt::info!("[STAT] G NM busy_clear us per rep: {}", &bc[..]);
        defmt::info!(
            "[STAT] G first edge after NM: min={=i32}us mean={=i32}us max={=i32}us",
            mn,
            (sum / 10) as i32,
            mx
        );
    } else {
        defmt::info!("[INFO] G NOT RUN (deadline)");
    }

    // ---------------------------------------------------------------- K
    defmt::info!("[INFO] ===== K: fine-resolution busy times and extra legality checks");
    pmu_fine(&mut d, "K1 UPD normal 400Hz same", reg::PMU_CMD_UPD_OAE, 80).await;
    Timer::after_millis(50).await;
    d.wr(reg::PMU_CMD_AGGR_SET, 0x14).await;
    pmu_fine(&mut d, "K2 UPD normal 400->100", reg::PMU_CMD_UPD_OAE, 80).await;
    Timer::after_millis(50).await;
    d.wr(reg::PMU_CMD_AGGR_SET, 0x24).await;
    pmu_fine(
        &mut d,
        "K3 UPD normal 100 avg2->avg4",
        reg::PMU_CMD_UPD_OAE,
        80,
    )
    .await;
    Timer::after_millis(50).await;
    pmu_fine(&mut d, "K3b FM in normal", reg::PMU_CMD_FM, 80).await;
    Timer::after_millis(50).await;
    pmu_fine(&mut d, "K3c FM_FAST in normal", reg::PMU_CMD_FM_FAST, 80).await;
    Timer::after_millis(50).await;
    pmu_fine(&mut d, "K4 SUS from normal", reg::PMU_CMD_SUS, 80).await;
    Timer::after_millis(50).await;
    pmu_fine(&mut d, "K5 SUS in suspend", reg::PMU_CMD_SUS, 80).await;
    Timer::after_millis(50).await;
    pmu_fine(&mut d, "K6 UPD in suspend", reg::PMU_CMD_UPD_OAE, 80).await;
    Timer::after_millis(50).await;
    d.wr(reg::PMU_CMD_AGGR_SET, 0x05).await;
    pmu_fine(&mut d, "K7 FM avg1 50Hz", reg::PMU_CMD_FM, 80).await;
    Timer::after_millis(50).await;
    pmu_fine(&mut d, "K7b FM_FAST avg1", reg::PMU_CMD_FM_FAST, 80).await;
    Timer::after_millis(50).await;
    d.wr(reg::PMU_CMD_AGGR_SET, 0x02).await;
    pmu_fine(&mut d, "K8 NM 400Hz", reg::PMU_CMD_NM, 80).await;
    Timer::after_millis(100).await;

    // ---------------------------------------------------------------- H
    defmt::info!(
        "[INFO] ===== H: last: soft reset (ends the boot state; run bmm350_smoke afterwards)"
    );
    pmu(
        &mut d,
        "H.SUS from normal",
        reg::PMU_CMD_SUS,
        100,
        false,
        true,
    )
    .await;
    pmu(
        &mut d,
        "H.SUS in suspend (sets ill)",
        reg::PMU_CMD_SUS,
        100,
        false,
        false,
    )
    .await;
    soft_reset(&mut d, "H.reset", false).await;
    Timer::after_millis(100).await;
    show_now(&mut d, "H.reset+100ms").await;
    pmu(
        &mut d,
        "H.NM after reset",
        reg::PMU_CMD_NM,
        300,
        false,
        false,
    )
    .await;

    if d.bus_err > 0 {
        defmt::error!("[FAIL] bus errors: {=u32}", d.bus_err);
        failures += 1;
    }
    defmt::info!("[INFO] run took {=u64} ms", start.elapsed().as_millis());
    hil_common::exit(failures)
}
