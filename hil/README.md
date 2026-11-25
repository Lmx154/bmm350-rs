# hil

Part of [bmm350](../README.md)

On-board test programs for the MARV V2 Rev 1 board. This folder is its own workspace and the only user of embassy-rp. `tools/run.sh <name> [timeout_s]` builds one program, flashes it through a debug probe on the SWD header under a lock that serialises probe use, and runs it. Exit status 0 means every check passed; logs go to `hil/logs/`. Pins: I2C0 SDA GPIO12, SCL GPIO13 (4.7k pull-ups), INT GPIO4, address 0x14.

| Program | What it checks |
|---|---|
| `alive` | tool chain, probe and board |
| `bmm350_smoke` | identity, data-ready rate and missed edges at 100 Hz and 400 Hz, read time, plausible field at rest, forced trigger, blocking front-end |
| `bmm350_ops` | latched-interrupt reconfigure, `verify` (including a mismatch after a raw write), `set_power_mode`, `configure` between rates, forced triggers at every averaging, `WrongMode`, `recover`, 200 `verify` calls while streaming |
| `bmm350_selftest` | `self_test`, both resets, `read_with_time` and sensor-time steps, `exceeds_high_field` at rest, final `verify` and `error_status` |
| `bmm350_cancel` | reads of read-only registers cancelled across the whole frame at 100 kHz, 400 kHz and 1 MHz, `recover` after each, the 17-byte `read_with_time` on its own, a positive control without recovery, and the bus clear for a data line held low; no write is ever cancelled |
| `bmm350_pmu` | a probe of the chip's power-mode behaviour with raw register access and no driver `init`; it records timings, it is not a pass/fail test |

## Related

[driver](../docs/driver.md) · [device](../docs/device.md)
