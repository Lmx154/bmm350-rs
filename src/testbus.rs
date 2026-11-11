//! Host test harness: a fake BMM350 on an I2C bus (a register file with the device's semantics as
//! the driver uses them), a fake delay, a counter of await points, and a minimal executor. One
//! copy per driver repository, on purpose.
//!
//! The chip, the bus and the delay share one state so that a test can inspect the traffic after a
//! future was dropped. Awaits are counted in one sequence across bus and delay; a pend at await
//! `k` returns `Poll::Pending` once, before the operation takes effect, and completes afterwards.
//!
//! Device model (datasheet BST-BMM350-DS001-27): a write transaction is the register pointer then
//! data bytes with auto-increment; a read returns the two dummy bytes first (8.1, p27), then the
//! registers with auto-increment; only the chip's own address is acknowledged. The OTP command
//! register serves words from a table the test supplies (8.35 and the OTP flow, p14). The `CMD`
//! soft reset (0xB6) restores the reset values (8.35, p43).
//!
//! `PMU_CMD` (8.6, p32) and `PMU_CMD_STATUS_0` (8.7, p33) follow the rules measured on the board,
//! 2026-11-10, in simulated time (the fake delay advances a clock; bus transactions take none):
//! R1 in suspend `NM`, `FM`, `FM_FAST` (and the reset commands `BR`, `FGR`, ...) are legal and
//! `SUS`, `UPD_OAE` illegal; R2 in normal mode `SUS`, `UPD_OAE` legal and the others illegal; R3
//! the illegal flag is set when an illegal command finishes (busy 0.5 ms first), reads do not clear
//! it, a later legal command's completion or a soft reset does; R4 the echo (bits 7:5) is the last
//! legal command's value, 0 after an illegal `SUS` or `UPD_OAE`, unchanged after another illegal
//! command; R5 `PMU_CMD_AGGR_SET` becomes the effective rate and averaging in `PMU_CMD_STATUS_1`
//! when `NM`, `FM`, `FM_FAST` or `UPD_OAE` completes; R6 busy times: `NM` 15.0 ms, `SUS` 3 ms,
//! `UPD_OAE` 2 ms, `FM` 14.5 ms (averaging 1) to 27 ms (averaging 8), `FM_FAST` 3.4 to 16 ms
//! (averaging 2 and 4 interpolated, not measured); R7 after a soft reset no PMU command completes
//! until `PWR_OFF_OTP` was written. A command written while one is busy is dropped and counted.
//!
//! Data ready and the stall (measured on the board, 2026-11-10): R8 a conversion (`Chip::sample`,
//! or a stream started with `Chip::start_stream`, one sample per period while the chip is in
//! normal mode) advances the sensor time and, with data-ready enabled in `INT_CTRL`, counts an
//! edge and, in latched mode, sets the `INT_STATUS` flag; a read of `INT_STATUS` returns the flag
//! and clears it (a flag left over from latched mode is returned once); R9 writing `INT_CTRL` so that it leaves latched mode while the
//! flag is set STALLS the chip: from then on no sample advances the sensor time or counts an
//! edge, every register still reads as written, and only a soft reset clears the stall.
//!
//! Self-test (datasheet 8.33, p42; vendor values): R10 when a forced measurement (`FM`,
//! `FM_FAST`) completes with `st_igen_en` set and exactly one of `st_p`, `st_n`, the X and Y data
//! are the resting value plus (`st_p`) or minus (`st_n`) `Chip::st_counts` on the axis whose
//! `ist_en_x` or `ist_en_y` bit is set; `st_n` with `st_p` is counted in `Chip::st_shorts` and
//! adds nothing. R11 the echo of `BR_FAST` (8) is 7, as the vendor expects.

use core::future::Future;
use core::pin::{Pin, pin};
use core::task::{Context, Poll, Waker};
use std::sync::{Arc, Mutex, MutexGuard};
use std::vec::Vec;

use embedded_hal::i2c::{ErrorKind, ErrorType, NoAcknowledgeSource, Operation};

use crate::reg;

/// The bus error the fake raises.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum BusFault {
    /// An injected failure of the chosen transaction.
    Injected,
    /// The address was not the chip's.
    NoAck,
}

impl embedded_hal::i2c::Error for BusFault {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::Injected => ErrorKind::Bus,
            Self::NoAck => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address),
        }
    }
}

/// What one operation of a transaction put on the bus and got back.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Op {
    /// `Operation::Write`: the bytes sent.
    Write(Vec<u8>),
    /// `Operation::Read`: the bytes returned, dummy bytes included.
    Read(Vec<u8>),
}

/// One logged event, in order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Event {
    /// A completed transaction.
    I2c {
        /// The 7-bit address.
        address: u8,
        /// The operations.
        ops: Vec<Op>,
    },
    /// A delay, in nanoseconds.
    Delay(u64),
}

/// A write of `value` to `reg`, as the driver sends it.
pub(crate) fn w(address: u8, reg: u8, value: u8) -> Event {
    Event::I2c {
        address,
        ops: std::vec![Op::Write(std::vec![reg, value])],
    }
}

/// A one-register read of `value` from `reg`: pointer write, then dummy bytes and the value.
pub(crate) fn r(address: u8, reg: u8, value: u8) -> Event {
    Event::I2c {
        address,
        ops: std::vec![
            Op::Write(std::vec![reg]),
            Op::Read(std::vec![DUMMY, DUMMY, value])
        ],
    }
}

/// The value the fake returns for a dummy byte.
pub(crate) const DUMMY: u8 = 0xAA;

/// Size of the fake register file; `CMD` is at 0x7E.
const REGS: usize = 0x80;

/// A command the chip is busy with.
#[derive(Clone, Copy)]
struct Pending {
    cmd: u8,
    legal: bool,
    done_at: u64,
    /// `PMU_CMD_AGGR_SET` when the command was written.
    aggr: u8,
}

/// Busy time of an illegal command: 0.2 to 1 ms measured (R1).
const ILLEGAL_BUSY_US: u64 = 500;

/// Busy time of a legal command in microseconds (R6); `aggr` is `PMU_CMD_AGGR_SET`.
fn command_busy_us(cmd: u8, aggr: u8) -> u64 {
    let samples =
        1u64 << ((aggr & reg::PMU_CMD_AGGR_SET_AVG_MASK) >> reg::PMU_CMD_AGGR_SET_AVG_SHIFT);
    let forced = |one: u64, eight: u64| one + (eight - one) * (samples - 1) / 7;
    match cmd {
        reg::PMU_CMD_SUS => 3_000,
        reg::PMU_CMD_NM => 15_000,
        reg::PMU_CMD_UPD_OAE => 2_000,
        reg::PMU_CMD_FM => forced(14_500, 27_000),
        reg::PMU_CMD_FM_FAST => forced(3_400, 16_000),
        // The reset commands: not measured; the driver waits its own fixed times for them.
        _ => 0,
    }
}

/// A one-off corrupted read (see `Chip::glitch`).
#[derive(Clone, Copy)]
pub(crate) struct Glitch {
    pub reg: u8,
    pub xor: u8,
    pub skip: usize,
}

pub(crate) struct Chip {
    pub regs: [u8; REGS],
    /// The OTP words served by the OTP command register.
    pub otp: [u16; reg::OTP_WORDS],
    /// The value `CHIP_ID` reads.
    pub chip_id: u8,
    /// An OTP read command never reports done.
    pub otp_never_done: bool,
    /// Added to the busy time of `NM` and `SUS`, in microseconds.
    pub extra_busy_us: u64,
    /// Simulated time, advanced by the fake delay, in microseconds.
    pub now_us: u64,
    pending: Option<Pending>,
    /// Mode flag, illegal flag and echo as `PMU_CMD_STATUS_0` shows them once no command is busy.
    pub normal: bool,
    pub illegal: bool,
    pub echo: u8,
    /// `PWR_OFF_OTP` was written since the last soft reset (R7).
    pub otp_booted: bool,
    /// Illegal commands that finished since the chip was created.
    pub illegal_commands: usize,
    /// Commands written while another one was busy (dropped).
    pub dropped_commands: usize,
    /// OR-ed into every read of `PMU_CMD_STATUS_0`, then `status_xor` is XOR-ed in.
    pub status_or: u8,
    pub status_xor: u8,
    /// The chip's 7-bit address.
    pub address: u8,
    pub log: Vec<Event>,
    /// Transactions started since the last `arm` (failed ones included).
    pub transactions: usize,
    /// Awaits reached since the last `arm`.
    pub awaits: u32,
    /// Fail this transaction (1-based) with `BusFault::Injected`; it has no effect on the chip.
    pub fail_at: Option<usize>,
    /// Return `Pending` once at this await (1-based).
    pub pend_at: Option<u32>,
    pub pended: bool,
    /// A one-off corrupted read: reads of `reg` are XOR-ed with `xor`, once, after `skip` reads of
    /// it have passed.
    pub glitch: Option<Glitch>,
    /// Bus rules the driver broke: a read without a preceding register pointer.
    pub violations: usize,
    /// R9: the chip stalled by leaving latched mode with the `INT_STATUS` flag set.
    pub stalled: bool,
    /// The `INT_STATUS` data-ready flag (latched mode only).
    pub int_flag: bool,
    /// Data-ready edges since the chip was created.
    pub edges: usize,
    /// One sample per this many microseconds while in normal mode; `None` for no stream.
    pub stream_period_us: Option<u64>,
    pub next_sample_us: u64,
    /// The self-test field in raw counts: a forced measurement that completes with a stimulus
    /// bit set returns the resting value plus or minus this on that axis (R10).
    pub st_counts: i32,
    /// What the stimulus currently adds to the X and Y data registers, in counts.
    st_applied: [i32; 2],
    /// Forced measurements that completed with `st_n` and `st_p` both set.
    pub st_shorts: usize,
}

impl Chip {
    fn new(address: u8) -> Self {
        let mut otp = [0u16; reg::OTP_WORDS];
        otp[0x0E] = 0x0064; // offset X = 100
        otp[0x10] = 0x0500; // sensitivity X = 5/256
        let mut c = Self {
            regs: [0; REGS],
            otp,
            chip_id: reg::CHIP_ID_VALUE,
            otp_never_done: false,
            extra_busy_us: 0,
            now_us: 0,
            pending: None,
            normal: false,
            illegal: false,
            echo: 0,
            otp_booted: true,
            illegal_commands: 0,
            dropped_commands: 0,
            status_or: 0,
            status_xor: 0,
            address,
            log: Vec::new(),
            transactions: 0,
            awaits: 0,
            fail_at: None,
            pend_at: None,
            pended: false,
            glitch: None,
            violations: 0,
            stalled: false,
            int_flag: false,
            edges: 0,
            stream_period_us: None,
            next_sample_us: 0,
            st_counts: 20_000,
            st_applied: [0; 2],
            st_shorts: 0,
        };
        // Data block: x = 1000, y = -1000, z = 2000, temperature = 51450.
        let data = [
            0xE8, 0x03, 0x00, 0x18, 0xFC, 0xFF, 0xD0, 0x07, 0x00, 0xFA, 0xC8, 0x00,
        ];
        let at = reg::MAG_X_XLSB as usize;
        c.regs[at..at + data.len()].copy_from_slice(&data);
        c.restore_reset_values();
        c
    }

    /// The reset values (8.x, p27-43) of the registers the driver touches; the data block, which
    /// the test owns, is kept.
    fn restore_reset_values(&mut self) {
        let (lo, hi) = (reg::MAG_X_XLSB as usize, reg::SENSORTIME_MSB as usize + 1);
        let mut data = [0u8; REGS];
        data[lo..hi].copy_from_slice(&self.regs[lo..hi]);
        self.regs = [0; REGS];
        self.regs[lo..hi].copy_from_slice(&data[lo..hi]);
        self.regs[reg::CHIP_ID as usize] = self.chip_id;
        self.regs[reg::PMU_CMD_AGGR_SET as usize] = reg::PMU_CMD_AGGR_SET_RESET;
        self.regs[reg::PMU_CMD_AXIS_EN as usize] = reg::PMU_CMD_AXIS_EN_RESET;
        self.regs[reg::TRANSDUCER_REV_ID as usize] = reg::TRANSDUCER_REV_ID_RESET;
        self.regs[reg::OTP_STATUS_REG as usize] = reg::OTP_STATUS_REG_RESET;
        self.pending = None;
        self.normal = false;
        self.illegal = false;
        self.echo = 0;
        self.otp_booted = false;
        self.stalled = false;
        self.int_flag = false;
        self.st_applied = [0; 2];
    }

    /// R10: when a forced measurement completes, the X and Y data are the resting value (what the
    /// test put in the data block) plus `st_counts` times the polarity on the axis whose
    /// `ist_en` bit is set, when the generator is on; `st_n` with `st_p` is a short and counted.
    fn apply_stimulus(&mut self) {
        let st = self.regs[usize::from(reg::TMR_SELFTEST_USER)];
        let on = st & reg::ST_IGEN_EN != 0;
        let (n, p) = (st & reg::ST_N != 0, st & reg::ST_P != 0);
        if on && n && p {
            self.st_shorts += 1;
        }
        let sign = match (on, n, p) {
            (true, false, true) => 1,
            (true, true, false) => -1,
            _ => 0,
        };
        let now = [
            if st & reg::IST_EN_X != 0 {
                sign * self.st_counts
            } else {
                0
            },
            if st & reg::IST_EN_Y != 0 {
                sign * self.st_counts
            } else {
                0
            },
        ];
        for (i, first) in [reg::MAG_X_XLSB, reg::MAG_Y_XLSB].into_iter().enumerate() {
            let at = usize::from(first);
            let cur = i32::from_le_bytes([self.regs[at], self.regs[at + 1], self.regs[at + 2], 0]);
            let cur = (cur << 8) >> 8;
            let new = cur - self.st_applied[i] + now[i];
            self.regs[at..at + 3].copy_from_slice(&new.to_le_bytes()[..3]);
        }
        self.st_applied = now;
    }

    /// `INT_CTRL` is latched with data-ready enabled.
    fn int_ctrl_latched(&self) -> bool {
        let both = reg::INT_CTRL_DRDY_EN | reg::INT_CTRL_LATCHED;
        self.regs[usize::from(reg::INT_CTRL)] & both == both
    }

    /// One conversion finished (R8); `ticks` is how far the sensor time advances. A stalled chip
    /// does nothing.
    pub fn sample_ticks(&mut self, ticks: u32) {
        if self.stalled {
            return;
        }
        let at = usize::from(reg::SENSORTIME_XLSB);
        let now = u32::from_le_bytes([self.regs[at], self.regs[at + 1], self.regs[at + 2], 0]);
        let next = now.wrapping_add(ticks) & 0x00FF_FFFF;
        self.regs[at..at + 3].copy_from_slice(&next.to_le_bytes()[..3]);
        if self.regs[usize::from(reg::INT_CTRL)] & reg::INT_CTRL_DRDY_EN != 0 {
            self.edges += 1;
            if self.int_ctrl_latched() {
                self.int_flag = true;
            }
        }
    }

    /// One conversion of 10 ms of sensor time.
    pub fn sample(&mut self) {
        self.sample_ticks(250);
    }

    /// From now on one sample per `period_us` while the chip is in normal mode.
    pub fn start_stream(&mut self, period_us: u64) {
        self.stream_period_us = Some(period_us);
        self.next_sample_us = self.now_us + period_us;
    }

    /// Produces the samples of a stream that fell due up to now. A `SUS` that completes at or
    /// before a sample's time has stopped the conversions; an `NM` that completes by then has
    /// started them.
    fn run_samples(&mut self) {
        let Some(period) = self.stream_period_us else {
            return;
        };
        while self.next_sample_us <= self.now_us {
            let t = self.next_sample_us;
            let done = |cmd: u8| matches!(self.pending, Some(p) if p.legal && p.cmd == cmd && p.done_at <= t);
            let running = (self.normal && !done(reg::PMU_CMD_SUS)) || done(reg::PMU_CMD_NM);
            if running {
                self.sample_ticks((period / 40) as u32);
            }
            self.next_sample_us += period;
        }
    }

    fn reg_mut(&mut self, addr: u8) -> &mut u8 {
        &mut self.regs[usize::from(addr) & (REGS - 1)]
    }

    fn write_reg(&mut self, addr: u8, value: u8) {
        match addr {
            reg::CMD => {
                if value == reg::CMD_SOFTRESET {
                    self.restore_reset_values();
                }
            }
            reg::PMU_CMD => self.pmu_command(value),
            reg::INT_CTRL => {
                self.run_samples();
                let was_latched = self.int_ctrl_latched();
                *self.reg_mut(addr) = value;
                if was_latched && self.int_flag && !self.int_ctrl_latched() {
                    self.stalled = true;
                }
            }
            reg::OTP_CMD_REG => {
                *self.reg_mut(addr) = value;
                if value == reg::OTP_CMD_PWR_OFF_OTP {
                    self.otp_booted = true;
                }
                if value & reg::OTP_CMD_MASK == reg::OTP_CMD_DIR_READ {
                    let word = self.otp[usize::from(value & reg::OTP_CMD_WORD_ADDR_MASK)];
                    self.regs[reg::OTP_DATA_MSB_REG as usize] = (word >> 8) as u8;
                    self.regs[reg::OTP_DATA_LSB_REG as usize] = word as u8;
                    self.regs[reg::OTP_STATUS_REG as usize] = if self.otp_never_done {
                        reg::OTP_STATUS_REG_RESET
                    } else {
                        reg::OTP_STATUS_CMD_DONE
                    };
                }
            }
            // Read-only registers ignore writes.
            reg::CHIP_ID
            | reg::ERR_REG
            | reg::PMU_CMD_STATUS_0
            | reg::PMU_CMD_STATUS_1
            | reg::TRANSDUCER_REV_ID
            | reg::INT_STATUS
            | reg::OTP_DATA_MSB_REG
            | reg::OTP_DATA_LSB_REG
            | reg::OTP_STATUS_REG
            | reg::MAG_X_XLSB..=reg::SENSORTIME_MSB => {}
            _ => *self.reg_mut(addr) = value,
        }
    }

    /// Completes the pending command when its time has come.
    fn settle(&mut self) {
        self.run_samples();
        let Some(p) = self.pending else { return };
        if !self.otp_booted || self.now_us < p.done_at {
            return;
        }
        self.pending = None;
        let aggr = p.aggr;
        if p.legal {
            self.illegal = false;
            // The echo is three bits wide; BR_FAST (8) echoes 7, as the vendor expects.
            self.echo = if p.cmd == reg::PMU_CMD_BR_FAST {
                7
            } else {
                p.cmd & 0x07
            };
            match p.cmd {
                reg::PMU_CMD_SUS => self.normal = false,
                reg::PMU_CMD_NM => {
                    self.normal = true;
                    self.regs[reg::PMU_CMD_STATUS_1 as usize] = aggr;
                }
                reg::PMU_CMD_UPD_OAE => {
                    self.regs[reg::PMU_CMD_STATUS_1 as usize] = aggr;
                }
                reg::PMU_CMD_FM | reg::PMU_CMD_FM_FAST => {
                    self.regs[reg::PMU_CMD_STATUS_1 as usize] = aggr;
                    self.apply_stimulus();
                }
                _ => {}
            }
        } else {
            self.illegal = true;
            self.illegal_commands += 1;
            if p.cmd == reg::PMU_CMD_SUS || p.cmd == reg::PMU_CMD_UPD_OAE {
                self.echo = 0;
            }
        }
        self.regs[reg::PMU_CMD_STATUS_0 as usize] = (self.echo << 5)
            | if self.illegal {
                reg::PMU_STATUS_0_ILLEGAL
            } else {
                0
            }
            | if self.normal {
                reg::PMU_STATUS_0_NORMAL
            } else {
                0
            };
    }

    fn pmu_command(&mut self, value: u8) {
        self.settle();
        *self.reg_mut(reg::PMU_CMD) = value;
        if self.pending.is_some() {
            self.dropped_commands += 1;
            return;
        }
        let cmd = value & reg::PMU_CMD_MASK;
        let legal = if self.normal {
            cmd == reg::PMU_CMD_SUS || cmd == reg::PMU_CMD_UPD_OAE
        } else {
            cmd != reg::PMU_CMD_SUS && cmd != reg::PMU_CMD_UPD_OAE
        };
        let aggr = self.regs[reg::PMU_CMD_AGGR_SET as usize];
        let busy_us = if legal {
            let extra = if cmd == reg::PMU_CMD_SUS || cmd == reg::PMU_CMD_NM {
                self.extra_busy_us
            } else {
                0
            };
            command_busy_us(cmd, aggr) + extra
        } else {
            ILLEGAL_BUSY_US
        };
        let done_at = self.now_us + busy_us;
        self.pending = Some(Pending {
            cmd,
            legal,
            done_at,
            aggr,
        });
    }

    fn read_reg(&mut self, addr: u8) -> u8 {
        self.settle();
        let mut v = *self.reg_mut(addr);
        if addr == reg::INT_STATUS && self.int_flag {
            v |= reg::INT_STATUS_DRDY;
            self.int_flag = false;
        }
        if addr == reg::PMU_CMD_STATUS_0 {
            if self.pending.is_some() {
                v |= reg::PMU_STATUS_0_BUSY;
            }
            v = (v | self.status_or) ^ self.status_xor;
        }
        if let Some(g) = self.glitch.as_mut()
            && g.reg == addr
        {
            if g.skip == 0 {
                v ^= g.xor;
                self.glitch = None;
            } else {
                g.skip -= 1;
            }
        }
        v
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), BusFault> {
        self.transactions += 1;
        if self.fail_at == Some(self.transactions) {
            return Err(BusFault::Injected);
        }
        if address != self.address {
            return Err(BusFault::NoAck);
        }
        let mut pointer: Option<u8> = None;
        let mut ops = Vec::new();
        for op in operations {
            match op {
                Operation::Write(bytes) => {
                    let mut started = false;
                    for &b in bytes.iter() {
                        if !started {
                            pointer = Some(b);
                            started = true;
                        } else if let Some(p) = pointer {
                            self.write_reg(p, b);
                            pointer = Some(p.wrapping_add(1));
                        }
                    }
                    ops.push(Op::Write(bytes.to_vec()));
                }
                Operation::Read(buf) => {
                    if pointer.is_none() {
                        self.violations += 1;
                    }
                    let mut p = pointer.unwrap_or(0);
                    for (i, b) in buf.iter_mut().enumerate() {
                        *b = if i < reg::DUMMY_BYTES {
                            DUMMY
                        } else {
                            let v = self.read_reg(p);
                            p = p.wrapping_add(1);
                            v
                        };
                    }
                    pointer = Some(p);
                    ops.push(Op::Read(buf.to_vec()));
                }
            }
        }
        self.log.push(Event::I2c { address, ops });
        Ok(())
    }
}

/// The shared state; clone it to keep a handle after the bus has moved into the driver.
#[derive(Clone)]
pub(crate) struct Shared(Arc<Mutex<Chip>>);

impl Shared {
    pub fn new(address: u8) -> Self {
        Self(Arc::new(Mutex::new(Chip::new(address))))
    }

    pub fn chip(&self) -> MutexGuard<'_, Chip> {
        self.0.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn bus(&self) -> TestBus {
        TestBus(self.clone())
    }

    pub fn delay(&self) -> TestDelay {
        TestDelay(self.clone())
    }

    /// Clears the log and the counters and sets the injections; the register file stays.
    pub fn arm(&self, fail_at: Option<usize>, pend_at: Option<u32>) {
        let mut c = self.chip();
        c.log.clear();
        c.transactions = 0;
        c.awaits = 0;
        c.fail_at = fail_at;
        c.pend_at = pend_at;
        c.pended = false;
        c.violations = 0;
    }

    pub fn log(&self) -> Vec<Event> {
        self.chip().log.clone()
    }

    pub fn awaits(&self) -> u32 {
        self.chip().awaits
    }

    pub fn transactions(&self) -> usize {
        self.chip().transactions
    }

    pub fn violations(&self) -> usize {
        self.chip().violations
    }

    pub fn reg(&self, addr: u8) -> u8 {
        let mut c = self.chip();
        c.settle();
        c.regs[usize::from(addr)]
    }
}

/// An await point: counts itself, and pends once if it is the chosen one.
struct Tick {
    shared: Shared,
    counted: bool,
}

impl Future for Tick {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
        if self.counted {
            return Poll::Ready(());
        }
        self.counted = true;
        let mut c = self.shared.chip();
        c.awaits += 1;
        if c.pend_at == Some(c.awaits) && !c.pended {
            c.pended = true;
            return Poll::Pending;
        }
        Poll::Ready(())
    }
}

fn tick(shared: &Shared) -> Tick {
    Tick {
        shared: shared.clone(),
        counted: false,
    }
}

/// The fake I2C bus. Both the async and the blocking trait are implemented over one state.
pub(crate) struct TestBus(Shared);

impl ErrorType for TestBus {
    type Error = BusFault;
}

impl embedded_hal_async::i2c::I2c for TestBus {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), BusFault> {
        tick(&self.0).await;
        self.0.chip().transaction(address, operations)
    }
}

impl embedded_hal::i2c::I2c for TestBus {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), BusFault> {
        self.0.chip().transaction(address, operations)
    }
}

/// The fake delay; it records and does not wait.
pub(crate) struct TestDelay(Shared);

impl embedded_hal_async::delay::DelayNs for TestDelay {
    async fn delay_ns(&mut self, ns: u32) {
        tick(&self.0).await;
        let mut c = self.0.chip();
        c.log.push(Event::Delay(u64::from(ns)));
        c.now_us += u64::from(ns).div_ceil(1_000);
    }
}

impl embedded_hal::delay::DelayNs for TestDelay {
    fn delay_ns(&mut self, ns: u32) {
        let mut c = self.0.chip();
        c.log.push(Event::Delay(u64::from(ns)));
        c.now_us += u64::from(ns).div_ceil(1_000);
    }
}

/// Runs a future to completion with a no-op waker, polling again after every `Pending`.
pub(crate) fn run<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    loop {
        if let Poll::Ready(v) = future.as_mut().poll(&mut cx) {
            return v;
        }
    }
}

/// How a future that was polled until its first `Pending` ended.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Polled<T> {
    /// It completed without ever pending: the chosen await is beyond its last.
    Done(T),
    /// It returned `Pending` and was dropped there.
    Cancelled,
}

/// Polls until the first `Pending`, then drops the future.
pub(crate) fn poll_then_drop<F: Future>(future: F) -> Polled<F::Output> {
    let mut future = pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    match future.as_mut().poll(&mut cx) {
        Poll::Ready(v) => Polled::Done(v),
        Poll::Pending => Polled::Cancelled,
    }
}

/// One test per rule of the model, on the chip itself, so that the model cannot drift from the
/// measurements it records.
#[cfg(test)]
mod rules {
    use super::*;

    const NORMAL: u8 = reg::PMU_STATUS_0_NORMAL;
    const ILLEGAL: u8 = reg::PMU_STATUS_0_ILLEGAL;
    const BUSY: u8 = reg::PMU_STATUS_0_BUSY;

    fn chip() -> Chip {
        let mut c = Chip::new(0x14);
        c.write_reg(reg::OTP_CMD_REG, reg::OTP_CMD_PWR_OFF_OTP);
        c
    }

    /// Writes `cmd`, lets `us` pass, returns the status.
    fn run(c: &mut Chip, cmd: u8, us: u64) -> u8 {
        c.write_reg(reg::PMU_CMD, cmd);
        c.now_us += us;
        c.read_reg(reg::PMU_CMD_STATUS_0)
    }

    fn status1(c: &mut Chip) -> u8 {
        c.read_reg(reg::PMU_CMD_STATUS_1)
    }

    fn normal() -> Chip {
        let mut c = chip();
        assert_eq!(run(&mut c, reg::PMU_CMD_NM, 15_000), 0x20 | NORMAL);
        c
    }

    #[test]
    fn r1_in_suspend_nm_fm_and_fm_fast_are_legal_sus_and_upd_oae_are_not() {
        for (cmd, echo) in [
            (reg::PMU_CMD_NM, 1),
            (reg::PMU_CMD_FM, 3),
            (reg::PMU_CMD_FM_FAST, 4),
        ] {
            let mut c = chip();
            let st = run(&mut c, cmd, 30_000);
            assert_eq!(st & (ILLEGAL | BUSY), 0, "cmd {cmd}");
            assert_eq!(st >> 5, echo);
        }
        for cmd in [reg::PMU_CMD_SUS, reg::PMU_CMD_UPD_OAE] {
            let mut c = chip();
            // Busy for 0.5 ms (0.2 to 1 ms measured), then only the illegal flag changes.
            assert_eq!(run(&mut c, cmd, 499), BUSY, "cmd {cmd}");
            c.now_us += 1;
            assert_eq!(c.read_reg(reg::PMU_CMD_STATUS_0), ILLEGAL, "cmd {cmd}");
            assert_eq!(c.illegal_commands, 1);
        }
    }

    #[test]
    fn r2_in_normal_sus_and_upd_oae_are_legal_nm_fm_and_fm_fast_are_not() {
        for cmd in [reg::PMU_CMD_NM, reg::PMU_CMD_FM, reg::PMU_CMD_FM_FAST] {
            let mut c = normal();
            let st = run(&mut c, cmd, 30_000);
            assert_eq!(st, 0x20 | NORMAL | ILLEGAL, "cmd {cmd}");
        }
        let mut c = normal();
        assert_eq!(run(&mut c, reg::PMU_CMD_UPD_OAE, 30_000), 0x40 | NORMAL);
        assert_eq!(run(&mut c, reg::PMU_CMD_SUS, 30_000), 0x00);
        assert_eq!(c.illegal_commands, 0);
    }

    #[test]
    fn r3_the_illegal_flag_is_set_at_completion_survives_reads_and_clears_on_a_legal_command_or_reset()
     {
        let mut c = normal();
        assert_eq!(run(&mut c, reg::PMU_CMD_FM, 100), 0x20 | NORMAL | BUSY);
        c.now_us += 400;
        for _ in 0..3 {
            assert_eq!(c.read_reg(reg::PMU_CMD_STATUS_0), 0x20 | NORMAL | ILLEGAL);
        }
        assert_eq!(run(&mut c, reg::PMU_CMD_UPD_OAE, 2_000), 0x40 | NORMAL);
        run(&mut c, reg::PMU_CMD_FM, 1_000);
        assert_eq!(c.read_reg(reg::PMU_CMD_STATUS_0) & ILLEGAL, ILLEGAL);
        c.write_reg(reg::CMD, reg::CMD_SOFTRESET);
        assert_eq!(c.read_reg(reg::PMU_CMD_STATUS_0), 0x00);
    }

    #[test]
    fn r4_the_echo_is_the_last_legal_command_zero_after_an_illegal_sus_or_upd_oae() {
        let mut c = chip();
        assert_eq!(run(&mut c, reg::PMU_CMD_NM, 15_000) >> 5, 1);
        assert_eq!(run(&mut c, reg::PMU_CMD_FM, 1_000) >> 5, 1);
        assert_eq!(run(&mut c, reg::PMU_CMD_UPD_OAE, 2_000) >> 5, 2);
        assert_eq!(run(&mut c, reg::PMU_CMD_SUS, 3_000) >> 5, 0);
        assert_eq!(run(&mut c, reg::PMU_CMD_FM, 30_000) >> 5, 3);
        assert_eq!(run(&mut c, reg::PMU_CMD_FM_FAST, 30_000) >> 5, 4);
        let st = run(&mut c, reg::PMU_CMD_UPD_OAE, 1_000);
        assert_eq!((st >> 5, st & ILLEGAL), (0, ILLEGAL));
        assert_eq!(run(&mut c, reg::PMU_CMD_FM_FAST, 30_000) >> 5, 4);
        let st = run(&mut c, reg::PMU_CMD_SUS, 1_000);
        assert_eq!((st >> 5, st & ILLEGAL), (0, ILLEGAL));
    }

    #[test]
    fn r5_aggr_set_is_taken_over_by_nm_and_fm_directly_and_in_normal_mode_only_by_upd_oae() {
        // Suspend: written, not effective; the following NM takes it, no UPD_OAE.
        let mut c = chip();
        c.write_reg(reg::PMU_CMD_AGGR_SET, 0x24);
        assert_eq!(status1(&mut c), 0x00);
        run(&mut c, reg::PMU_CMD_NM, 15_000);
        assert_eq!(status1(&mut c), 0x24);
        // Normal: written, no effect until UPD_OAE, or until SUS followed by NM.
        c.write_reg(reg::PMU_CMD_AGGR_SET, 0x13);
        run(&mut c, reg::PMU_CMD_FM, 30_000);
        assert_eq!(status1(&mut c), 0x24);
        run(&mut c, reg::PMU_CMD_UPD_OAE, 2_000);
        assert_eq!(status1(&mut c), 0x13);
        c.write_reg(reg::PMU_CMD_AGGR_SET, 0x35);
        run(&mut c, reg::PMU_CMD_SUS, 3_000);
        assert_eq!(status1(&mut c), 0x13);
        run(&mut c, reg::PMU_CMD_NM, 15_000);
        assert_eq!(status1(&mut c), 0x35);
        // FM and FM_FAST take it directly from suspend.
        for cmd in [reg::PMU_CMD_FM, reg::PMU_CMD_FM_FAST] {
            let mut c = chip();
            c.write_reg(reg::PMU_CMD_AGGR_SET, 0x36);
            run(&mut c, cmd, 30_000);
            assert_eq!(status1(&mut c), 0x36);
        }
    }

    #[test]
    fn r8_a_sample_advances_the_sensor_time_and_sets_the_latched_flag_that_a_read_clears() {
        let mut c = chip();
        c.regs[usize::from(reg::SENSORTIME_XLSB)..=usize::from(reg::SENSORTIME_MSB)]
            .copy_from_slice(&[0xFF, 0xFF, 0xFF]);
        c.write_reg(reg::INT_CTRL, reg::INT_CTRL_DRDY_EN | reg::INT_CTRL_LATCHED);
        c.sample_ticks(2);
        // 24-bit wrap: 0xFFFFFF + 2 = 1.
        assert_eq!(c.regs[usize::from(reg::SENSORTIME_XLSB)..][..3], [1, 0, 0]);
        assert_eq!((c.edges, c.read_reg(reg::INT_STATUS)), (1, 0x04));
        assert_eq!(c.read_reg(reg::INT_STATUS), 0);
        // Pulsed: an edge, no stored flag.
        c.write_reg(reg::INT_CTRL, reg::INT_CTRL_DRDY_EN);
        c.sample();
        assert_eq!((c.edges, c.read_reg(reg::INT_STATUS)), (2, 0));
        // Data-ready disabled: no edge.
        c.write_reg(reg::INT_CTRL, 0);
        c.sample();
        assert_eq!(c.edges, 2);
    }

    #[test]
    fn r9_leaving_latched_mode_with_the_flag_set_stalls_until_a_soft_reset() {
        let latched = reg::INT_CTRL_DRDY_EN | reg::INT_CTRL_LATCHED;
        let time = |c: &Chip| c.regs[usize::from(reg::SENSORTIME_XLSB)..][..3].to_vec();
        let mut c = chip();
        c.write_reg(reg::INT_CTRL, latched);
        c.sample();
        c.write_reg(reg::INT_CTRL, reg::INT_CTRL_DRDY_EN);
        assert!(c.stalled);
        let (edges, t) = (c.edges, time(&c));
        c.sample();
        assert_eq!((c.edges, time(&c)), (edges, t));
        // Registers read as written; reading INT_STATUS (the one flag) and writing again do not help.
        assert_eq!(c.read_reg(reg::INT_CTRL), reg::INT_CTRL_DRDY_EN);
        assert_eq!(c.read_reg(reg::INT_STATUS), 0x04);
        assert_eq!(c.read_reg(reg::INT_STATUS), 0);
        c.write_reg(reg::INT_CTRL, reg::INT_CTRL_DRDY_EN);
        run(&mut c, reg::PMU_CMD_SUS, 3_000);
        assert!(c.stalled);
        c.write_reg(reg::CMD, reg::CMD_SOFTRESET);
        c.write_reg(reg::INT_CTRL, reg::INT_CTRL_DRDY_EN);
        c.sample();
        assert!(!c.stalled);
        assert_eq!(c.edges, edges + 1);
        // No stall: the flag read first, no flag, still latched, or not latched before.
        for (read_first, flag, next) in [
            (true, true, reg::INT_CTRL_DRDY_EN),
            (false, false, reg::INT_CTRL_DRDY_EN),
            (false, true, latched | reg::INT_CTRL_ACTIVE_HIGH),
        ] {
            let mut c = chip();
            c.write_reg(reg::INT_CTRL, latched);
            if flag {
                c.sample();
            }
            if read_first {
                c.read_reg(reg::INT_STATUS);
            }
            c.write_reg(reg::INT_CTRL, next);
            assert!(!c.stalled, "{read_first} {flag} {next:#04x}");
        }
        let mut c = chip();
        c.write_reg(reg::INT_CTRL, reg::INT_CTRL_DRDY_EN);
        c.sample();
        c.write_reg(reg::INT_CTRL, 0);
        assert!(!c.stalled);
    }

    #[test]
    fn r6_busy_times() {
        let busy = |c: &mut Chip, cmd: u8| {
            let mut us = 0;
            c.write_reg(reg::PMU_CMD, cmd);
            while c.read_reg(reg::PMU_CMD_STATUS_0) & BUSY != 0 {
                c.now_us += 1;
                us += 1;
            }
            us
        };
        let mut c = chip();
        c.write_reg(reg::PMU_CMD_AGGR_SET, 0x04);
        assert_eq!(busy(&mut c, reg::PMU_CMD_NM), 15_000);
        assert!((1_800..=4_000).contains(&busy(&mut c, reg::PMU_CMD_SUS)));
        let mut c = chip();
        c.write_reg(reg::PMU_CMD_AGGR_SET, 0x04);
        assert_eq!(busy(&mut c, reg::PMU_CMD_FM), 14_500);
        assert_eq!(busy(&mut c, reg::PMU_CMD_FM_FAST), 3_400);
        c.write_reg(reg::PMU_CMD_AGGR_SET, 0x32);
        assert_eq!(busy(&mut c, reg::PMU_CMD_FM), 27_000);
        assert_eq!(busy(&mut c, reg::PMU_CMD_FM_FAST), 16_000);
    }

    #[test]
    fn r7_after_a_soft_reset_no_command_completes_until_the_otp_boot_is_finished() {
        let mut c = chip();
        c.write_reg(reg::CMD, reg::CMD_SOFTRESET);
        assert_eq!(run(&mut c, reg::PMU_CMD_NM, 2_000_000), BUSY);
        c.write_reg(reg::OTP_CMD_REG, reg::OTP_CMD_PWR_OFF_OTP);
        assert_eq!(c.read_reg(reg::PMU_CMD_STATUS_0), 0x20 | NORMAL);
    }
}
