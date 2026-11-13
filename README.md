# bmm350

Driver for the Bosch BMM350 three-axis magnetometer over I2C, written for the MARV V2 Rev 1 flight controller (RP2354B). `no_std`, no `unsafe`. The crate root is the async driver, written against `embedded-hal-async` 1.0 (I2C and delay). The `blocking` module is the same code run to completion on `embedded-hal` 1.0 bus and delay types. The only other dependency is `defmt` 1.0, optional. The driver owns no interrupt pin, timer or task.

Status: version 0.1.0. The API may still change. The driver has been run on one board with one sensor; see "Measured" and "Not covered".

## Works with

The driver works with any HAL and any MCU whose I2C bus and delay implement `embedded-hal-async` 1.0 (async driver) or `embedded-hal` 1.0 (`blocking` module). No embassy crate is needed.

## Quick start

```rust,no_run
# use core::convert::Infallible;
# use embedded_hal_async::i2c::{ErrorType, Operation};
# struct Bus;
# impl ErrorType for Bus { type Error = Infallible; }
# impl embedded_hal_async::i2c::I2c for Bus {
#     async fn transaction(&mut self, _: u8, _: &mut [Operation<'_>]) -> Result<(), Infallible> { Ok(()) }
# }
# struct Delay;
# impl embedded_hal_async::delay::DelayNs for Delay {
#     async fn delay_ns(&mut self, _: u32) {}
# }
use bmm350::{Averaging, Bmm350, Config, DEFAULT_ADDRESS, OutputDataRate};
use embedded_hal_async::{delay::DelayNs, i2c::I2c};

async fn run<I: I2c>(i2c: I, delay: &mut impl DelayNs) -> Result<(), bmm350::Error<I::Error>> {
    let mut mag = Bmm350::new(i2c, DEFAULT_ADDRESS);
    let config = Config::new()
        .with_odr(OutputDataRate::Hz100)
        .with_averaging(Averaging::Avg2);
    mag.init(delay, &config).await?;

    // The application owns the INT pin: wait for the data-ready edge here, and put any timeout
    // around that wait, never around a driver call.
    let field = mag.read_field().await?; // field.x, .y, .z in microtesla, .temperature in degC
    let _ = field;

    let timed = mag.read_with_time().await?; // timed.sample (counts), timed.sensor_time (ticks)
    let _ = timed;
    let report = mag.self_test(delay).await?; // report.passed(), report.entries
    let _ = report;
    Ok(())
}
```

The blocking front-end has the same methods without `.await`: `bmm350::blocking::Bmm350::new(i2c, address)`, with an `embedded_hal::delay::DelayNs`.

## What the application provides

- An I2C bus: `embedded_hal_async::i2c::I2c` for the async driver, `embedded_hal::i2c::I2c` for `blocking`.
- The 7-bit address: `DEFAULT_ADDRESS` (0x14, ADSEL low) or `ADDRESS_ADSEL_HIGH` (0x15, ADSEL high).
- A delay (`DelayNs`) for `init`, `configure`, `recover`, `self_test`, `set_power_mode` and the magnetic resets.
- The interrupt pin and the waiting on it. `init` configures the chip's INT output (`InterruptPin`); the application waits on the pin, and after `init` waits for the next data-ready edge.

## Rules for correct use

- **Never cancel an I2C transfer in flight.** No timeout and no `select` around a driver call that is on the bus; put the timeout around the wait for data-ready. On embassy-rp 0.10 a cancelled read makes later reads return wrong data with no error (CONTRACT.md A4). That was first measured on the sister barometer driver's bus (about 13 000 cancelled reads) and then on this chip by `bmm350_cancel` (2026-11-11, two runs): without recovery 161 of 182 cancelled reads were followed by a wrong read; with `recover` after each cancel, 0 wrong checking reads of 9600 over 1858 cancels at 100 kHz, 400 kHz and 1 MHz, and no register changed.
- **`recover` is the way back after any doubt** (a cancelled or failed transfer): it waits 2 ms with no bus traffic, makes one discarded `CHIP_ID` read, checks the identity twice, then runs `verify`. It writes nothing to the device. A data line held low needs a bus clear by the firmware's bus owner; a driver that holds only an `I2c` cannot do it. `verify()` is the health check: it reads back the registers the applied `Config` owns, changes nothing, and returns `Error::ConfigMismatch` after two differing reads. It compares configuration, not liveness: a chip stalled by a latched-interrupt reconfiguration passed it once (measured 2026-11-10); check that data-ready edges keep coming or that `sensor_time` advances.
- **`Error::NotConfigured`**: the typed methods (reads, `set_power_mode`, triggers, resets, `self_test`, `verify`, `interrupt_status`) return it until `init` has completed (`configure` needs an earlier `init`: it reads the chip's calibration). It is also returned after a failed or cancelled `init`, `configure`, `set_power_mode` or `self_test`; call `configure` (or `init`) again. A `set_power_mode` refused with `Error::WrongMode` leaves the driver configured. `recover` returns it only after its identity reads. `sensor_time`, `pmu_status`, `error_status` and raw register access work whenever the bus works.
- **A latched interrupt and the data-ready flag.** With a latched interrupt, `configure` and `set_power_mode` to suspend read `INT_STATUS`, which consumes a pending data-ready flag. `Info::data_ready_consumed` (from `configure`) reports it; an application waiting for that edge must look at it. `set_power_mode` does not report it.
- **Magnetic resets** (`magnetic_reset`, `magnetic_reset_fast`) are legal in suspend only; in normal mode they return `Error::WrongMode` and send nothing. Use one after a field beyond the high-field limit (`exceeds_high_field` is true), after a field shock, or (full pair) before leaving a suspend longer than about 2 s. Sequence: `set_power_mode(Suspend)`, reset, `set_power_mode(Normal)`. Apart from `init` and `self_test`, the driver never runs a magnetic reset on its own; in particular not after a high field.
- **Sensor time is not a clock.** It is the time of the last sample, from the sensor's own oscillator, in steps of one sample period; in suspend it stands still; two equal readings can mean "no new sample". It drifts against the host clock.
- **Bus transfers.** On this HAL I2C is interrupt-driven, not DMA. `read` is 14 bytes and is fully queued before the first await (CONTRACT.md A4e: up to 15 bytes). `read_with_time` is 17 bytes, more than the HAL queues in one go, so a cancel during it can leave a transfer in flight; on the board `recover` restored correct reads after each of 318 such cancels (0 wrong of 1600 checking reads, `bmm350_cancel`).
- After a forced trigger, wait the documented time (`timing::forced_measurement_us`) before the next command; the busy flag is still set at the data-ready edge.

## What the driver does

- Settings: `Config` (`#[non_exhaustive]`; `new()` with `with_odr`, `with_averaging`, `with_axes`, `with_start_mode`, `with_interrupt`, `with_watchdog`); applied by `init` and `configure`. `Config::validate` rejects no axis and an averaging the rate does not allow (`ConfigError`). `InterruptPin` sets data-ready on or off, pulsed or latched, push-pull or open-drain, polarity. `I2cWatchdog` sets the chip's I2C watchdog.
- Power modes and forced measurements: `set_power_mode` (`PowerMode`), `trigger_forced`, `trigger_forced_fast`, `timing::forced_measurement_us`, `timing::forced_fast_max_rate_hz`.
- Compensated read: `read_field` gives microtesla and degrees Celsius; `read` gives `RawSample` counts; `compensate` converts them with `Calibration`, which `init` reads from the chip's own OTP.
- Timed read: `read_with_time` (one transaction, sample and sensor time together) and `sensor_time`; conversion helpers `timing::sensor_ticks_elapsed`, `sensor_elapsed_us`, `sensor_elapsed_us_with_period` (24-bit wrap handled) and `sensor_time_us` (no wrap handling). One tick is 1/25600 s.
- `self_test`: X and Y, both polarities (there is none for Z). `SelfTestReport` holds four `SelfTestEntry` (axis, polarity, field without and with the stimulus, `delta`, `passed`); an entry passes if the compensated field moves at least `selftest::MIN_DELTA_UT` (130 uT) in the direction of the stimulus. A completed test that misses the limit is `Ok` with `passed()` false. It runs in suspend, includes a full magnetic reset, and re-applies the configuration at the end.
- `magnetic_reset` (full recharge, as `init` does) and `magnetic_reset_fast`.
- `exceeds_high_field(&Field)`: magnitude above 2000 uT (`selftest::HIGH_FIELD_LIMIT_UT`) or a value that is not finite.
- Status reads: `pmu_status` (`PmuStatus`), `error_status` (`ErrorStatus`; reading clears `ERR_REG`), `interrupt_status` (`InterruptStatus`; clears the flag).
- `verify`, `recover`, `config`, `calibration`, `release`.
- Raw access: `read_register`, `write_register`, `read_registers`, `read_otp_word`, and the register map in `reg`. A raw write can leave the chip in a state the applied `Config` does not describe.
- Blocking front-end: `blocking::Bmm350` has every one of these methods.

## Not covered

- Not implemented: pad drive (`PAD_CTRL`) and I3C with in-band interrupts (`INT_CTRL_IBI`); `reg` holds their addresses as facts only. Out-of-range detection from Bosch's `bmm350_oor.c`. An automatic reset after a high field. Hard-iron and soft-iron calibration of the installed sensor is the application's job.
- Exists but has NOT run on the real chip: no on-board program uses `with_axes`, `with_watchdog`, `InterruptDrive::OpenDrain`, `with_data_ready(false)`, 200 Hz or any rate of 25 Hz and below, `set_power_mode` with `Forced` or `ForcedFast`, or the driver at address 0x15 (`bmm350_smoke` only scans both addresses for an acknowledge). `exceeds_high_field` has been run on a field at rest only. The blocking front-end ran as `init`, `trigger_forced` and a read. No long-duration run has been done.

## Measured

On the MARV V2 Rev 1 board (RP2354B, embassy-rp 0.10, I2C0 at 400 kHz), one sensor, 2026-11-11 unless stated. These are observations, not guarantees.

| Item | Value |
|---|---|
| Self-test stimulus | +174.0 / -173.9 uT on X, +174.1 / -174.1 uT on Y, against the 130 uT limit |
| Offset left after `self_test` | at most 0.2 uT per axis over 50 samples |
| `self_test` time | 113.7 ms from the default configuration, 96.1 ms from a configuration that starts in suspend |
| `magnetic_reset_fast` / `magnetic_reset` | 12.6 ms / 32.6 ms; field afterwards within 0.3 uT of before; both refused in normal mode |
| Plain data read / `read_with_time` | 465 us / 539 us (medians at 100 Hz and 400 Hz, 400 reads, no error) |
| Sensor time | exactly 256 ticks per sample at 100 Hz, 64 at 400 Hz; this unit's oscillator ran 0.7 % fast (100.7 Hz at the 100 Hz setting) |
| Forced, command to data-ready (2026-11-10) | `FM` 14.2, 15.4, 18.9, 25.9 ms; `FM_FAST` 3.1, 4.3, 7.8, 14.8 ms (averaging 1, 2, 4, 8) |
| Data-ready edges (`docs/device.md`) | one per sample, none missed: 200 for 200 samples at 100 Hz, 799 for 799 at about 400 Hz |
| Field at rest on the bench | about 66 to 67 uT in magnitude (the bench, not a specification) |

## Tests

Host tests (no hardware): `cargo test --locked`. 95 unit and property tests pass; the README example is compiled as a doctest (`no_run`).

On-board programs, through a debug probe on the SWD header (the script takes a lock so runs never overlap). Run one with `tools/run.sh <name>`; the exit status is the program's verdict and the log is kept under `hil/logs/`. Pins: I2C0 SDA GPIO12, SCL GPIO13 (4.7k pull-ups), INT GPIO4, address 0x14.

| Program | What it proves |
|---|---|
| `alive` | tool chain, probe and board |
| `bmm350_smoke` | identity, data-ready rate and missed edges at 100 Hz and 400 Hz, read time, plausible field at rest, forced trigger, blocking front-end |
| `bmm350_ops` | latched-interrupt reconfigure, `verify` (including a mismatch after a raw write), `set_power_mode`, `configure` between rates, forced triggers at every averaging, `WrongMode`, `recover`, 200 `verify` calls while streaming |
| `bmm350_selftest` | `self_test`, both resets, `read_with_time` and sensor-time steps, `exceeds_high_field` at rest, final `verify` and `error_status` |
| `bmm350_cancel` | reads of read-only registers cancelled across the whole frame at 100 kHz, 400 kHz and 1 MHz, `recover` after each, the 17-byte `read_with_time` on its own, a positive control without recovery, and the bus clear for a data line held low; no write is ever cancelled |
| `bmm350_pmu` | a probe of the chip's power-mode behaviour with raw register access and no driver `init`; it records timings, it is not a pass/fail test |

## Cargo features

All off by default; without them the crate is hardware-independent `no_std` code.

| Feature | What it adds |
|---|---|
| `defmt` | `defmt::Format` for the public data, configuration and error types |

## Layout

```text
Cargo.toml       the driver crate (its own workspace root)
src/             the driver; selftest.rs and timing.rs carry the long documentation
hil/             on-board programs, own workspace, the only place that uses embassy-rp
tools/run.sh     builds, flashes and runs one on-board program
docs/device.md   register, timing and measured facts, pins, datasheet
```

## License

MIT, see `LICENSE`. The driver is written from the datasheet listed in `docs/device.md`; facts that come only from Bosch's `BMM350_SensorAPI` are marked in the crate and no code is copied from it.
