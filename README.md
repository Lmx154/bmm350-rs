# bmm350

Driver for the Bosch BMM350 three-axis magnetometer over I2C, for the MARV V2 Rev 1 flight controller (RP2354B). It is built on the `embedded-hal` 1.0 traits only: no executor, no HAL, no pin, no timer. The crate root is the async driver. The `blocking` module is the same driver for blocking buses. It wraps the bus in an adapter that never pends and runs the same code, so nothing is written twice. All device knowledge (register map, configuration codes, decoding, OTP calibration and compensation, units) is pure code tested on the host.

## Usage

Async, for example in an embassy task:

```rust
let mut mag = bmm350::Bmm350::new(i2c, bmm350::DEFAULT_ADDRESS);
mag.init(&mut delay, &bmm350::Config::default()).await?;
loop {
    int_pin.wait_for_rising_edge().await;
    let field = mag.read_field().await?; // microtesla, degrees Celsius
}
```

Blocking, on any embedded-hal bus:

```rust
let mut mag = bmm350::blocking::Bmm350::new(i2c, bmm350::DEFAULT_ADDRESS);
mag.init(&mut delay, &bmm350::Config::default())?;
let raw = mag.read()?;
let field = bmm350::compensate(mag.calibration(), &raw);
```

Units: `Field` is in microtesla and degrees Celsius. `RawSample` is in sensor counts. Hard-iron and soft-iron calibration of the installed sensor is the application's job.

## What the application provides

- An I2C bus: `embedded_hal_async::i2c::I2c` for the async driver, the blocking `embedded-hal` I2C trait for `blocking`.
- A `DelayNs` for `init`.
- The address: `DEFAULT_ADDRESS` (0x14, ADSEL low) or `ADDRESS_ADSEL_HIGH` (0x15, ADSEL high).
- The interrupt pin and the waiting on it. The driver owns no interrupt pin, timer or task. `init` configures the chip's INT output; the application waits on the pin. Every sample signalled by data-ready after `init` returns is valid.

Dependencies: `embedded-hal` 1.0, `embedded-hal-async` 1.0 and, optionally, `defmt`.

## Cargo features

All features are off by default. Without them the crate is hardware-independent `no_std` code.

| Feature | What it adds |
|---|---|
| `defmt` | `defmt::Format` for the public data and error types |

## Measured on the MARV V2 Rev 1 board

- 400 Hz with no missed sample.
- 443 us per compensated read at 400 kHz I2C.

## Tests

On the host:

    cargo test

On the board, through a debug probe (the script takes a lock, so runs never overlap):

    tools/run.sh alive
    tools/run.sh bmm350_smoke

Used here: Rust 1.99.1 with the `thumbv8m.main-none-eabihf` target, probe-rs 0.31.0, and a debug probe on the SWD header J10.

| Binary | What it checks |
|---|---|
| `alive` | tool chain, probe and board |
| `bmm350_smoke` | identity, data-ready rate with missed-sample count, time per read, plausible values at rest, the blocking front-end |

The exit status is 0 when every check passed. Full logs are kept under `hil/logs/`.

Pins the test uses: I2C0 SDA on GPIO12 and SCL on GPIO13 (4.7k pull-ups R25 and R26 to V3V3_SYS), INT on GPIO4. The address is 0x14. The test runs async I2C0 at 400 kHz with data-ready on GPIO4. The BMM350 is attached by hand to the magnetometer port on the current board.

## Status

Version 0.1.0, as tested on the Rev 1 board.

- Host: `cargo test` passes. Clippy, rustdoc and rustfmt are clean for the driver crate, and every public item is documented.
- Board: `alive` and `bmm350_smoke` exited 0 when last run.
- On-board test program: `cargo clippy` in `hil/` reports style findings in `bmm350_smoke`. They do not change any result.

Not done for this device:

- Self-test.
- Calibration of the installed sensor: the compass needs hard-iron and soft-iron calibration.

## Layout

```
Cargo.toml       the driver crate (its own workspace root)
src/             the driver
hil/             the on-board test: one package, its own workspace; the only place that uses embassy-rp
  src/lib.rs     reports the verdict to the host
  src/bin/       alive.rs and bmm350_smoke.rs
  build.rs       links memory.x
tools/run.sh     builds, flashes and runs one on-board test
docs/device.md   register, timing and measured facts, pins, datasheet
LICENSE          MIT
```

## License

MIT, see `LICENSE`. The driver is written from the datasheet listed in `docs/device.md`. Facts that come only from Bosch's `BMM350_SensorAPI` are marked in the crate; no code is copied from it.
