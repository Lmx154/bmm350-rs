# BMM350 device facts (MARV V2 Rev 1)

Copied from the MARV V2 firmware repository's `docs/devices.md` and `docs/pinout.md`. Page numbers are PDF pages of the datasheet listed at the end. Items marked UNKNOWN or "unverified" have not been checked against the datasheet or on hardware. "Measured" means measured on this board.

## BMM350 (Bosch Sensortec, magnetometer on I2C0, on the bench)

Pins: see "Pins and bus" below (SDA GPIO12, SCL GPIO13, INT on GPIO4). Sources: datasheet BST-BMM350-DS001-27 (rev 1.27, Feb 2025), page numbers below; "vendor" = Bosch `BMM350_SensorAPI` v1.10.0, files `bmm350.c` and `bmm350_defs.h` (BSD-3-Clause), used only where the datasheet defers to the API (OTP layout, compensation, delays).

- Supplies: VDD 1.72-1.98 V, VDDIO 1.72-3.6 V (p8). How the bench breakout derives its supply from the 3.3 V pin: UNKNOWN.
- I2C 7-bit address: 0x14 with ADSEL low, 0x15 with ADSEL high; ADSEL is sampled at boot and must not float (p44-45). Measured: scan of 0x08-0x77 answered at 0x14 only.
- I2C: standard, fast and fast+ modes, no clock stretching (p44). Measured: 400 kHz works. Reset state is I2C; I3C is not used here.
- Read protocol: every read returns 2 dummy bytes first; an n-byte read is n+2 bytes, discard the first 2 (p46, 9.2.3; vendor `BMM350_DUMMY_BYTES` = 2). Register address auto-increments. A write is register, data pairs (p45). A burst read that starts at the sensortime, magnetic or temperature registers freezes their update until it ends; single reads can mix samples (p18).
- CHIP_ID 0x00 = 0x33, read-only (p29, p21). ERR_REG 0x02 bit0 pmu_cmd_error, clears on read (p29).
- Registers (p26-43): PAD_CTRL 0x03 (bits 2:0 drive, reset 0x07); PMU_CMD_AGGR_SET 0x04 (bits 5:4 avg, bits 3:0 odr, reset 0x14); PMU_CMD_AXIS_EN 0x05 (bit0 x, bit1 y, bit2 z, reset 0x07 on p31; the map table on p28 says 0x70, treated as a typo); PMU_CMD 0x06 (bits 3:0); PMU_CMD_STATUS_0 0x07; PMU_CMD_STATUS_1 0x08; I2C_WDT_SET 0x0A; INT_CTRL 0x2E; INT_CTRL_IBI 0x2F (I3C only); INT_STATUS 0x30; data 0x31-0x3C; SENSORTIME 0x3D-0x3F; OTP_CMD_REG 0x50; OTP_DATA_MSB 0x52; OTP_DATA_LSB 0x53; OTP_STATUS 0x55; TMR_SELFTEST_USER 0x60; CTRL_USER 0x61; CMD 0x7E.
- Soft reset: write 0xB6 to CMD 0x7E, then 0x00 (p22, p43); vendor `bmm350_init` writes only 0xB6, waits 24 ms (`BMM350_SOFT_RESET_DELAY`), and waits 3 ms before the reset (`BMM350_START_UP_TIME_FROM_POR`). Datasheet: OFF to suspend 2.5 ms typical, suspend to normal 70 ms typical (p8). The OTP is accessible again after every reset and the boot ends with 0x80 in OTP_CMD_REG (p14).
- OTP read (vendor `read_otp_word`; the datasheet gives only the register layout, p41-42): write OTP_CMD_REG = 0x20 | word (cmd 001 DIR_READ in bits 7:5, word 0-31 in bits 4:0); wait 300 us, poll OTP_STATUS until bit0 otp_cmd_done = 1 (bits 7:5 nonzero = error: 1 boot, 2 page read, 3 page program, 4 sign, 5 invalid command); read OTP_DATA_MSB 0x52 and OTP_DATA_LSB 0x53 (word = MSB << 8 | LSB). The vendor reads all 32 words, then writes 0x80 (PWR_OFF_OTP) to OTP_CMD_REG.
- OTP words used (vendor `update_mag_off_sens`; byte = signed 8 bit): 0x0D low = temperature offset (/5 K), high = temperature sensitivity (/512); 0x0E bits 11:0 = offset X (signed 12 bit), bits 15:12 = offset Y bits 11:8; 0x0F low = offset Y bits 7:0, bits 11:8 = offset Z bits 11:8; 0x10 low = offset Z bits 7:0, high = sensitivity X (/256); 0x11 low = sensitivity Y, high = sensitivity Z (/256); 0x12 / 0x13 / 0x14 low = TCO X / Y / Z (/32), high = TCS X / Y / Z (/16384); 0x15 low = cross X-Y, high = cross Y-X; 0x16 low = cross Z-X, high = cross Z-Y (all four /800); 0x18 = signed 16 bit, T0 = value/512 + 23 degC. Offsets are added to values already in uT.
- Magnetic reset (vendor `bmm350_magnetic_reset_and_wait`; datasheet p17 describes it): chip in suspend; write PMU_CMD = 0x07 (BR, bit reset, full recharge), wait 14 ms, PMU_CMD_STATUS_0 bits 7:5 must read 7; write PMU_CMD = 0x05 (FGR, flux-guide reset), wait 18 ms, bits 7:5 must read 5. The datasheet says the device resets itself after each boot and in normal and forced mode on every conversion; a manual reset is for field shocks and after long suspend (> 2 s). Reset can keep PMU_CMD_STATUS_0 bit0 busy for up to 8 ms (p22).
- Power modes (PMU_CMD, p32): 0x00 SUS, 0x01 NM, 0x02 UPD_OAE (apply odr/avg), 0x03 FM (forced, full CRST recharge), 0x04 FM_FAST (fast recharge, for >= 25 Hz trigger rates; the only way to reach 200 Hz), 0x05 FGR, 0x06 FGR_FAST, 0x07 BR, 0x08 BR_FAST. Suspend is the state after boot; forced mode starts only from suspend and returns to suspend when done; a forced request in normal mode, or during a running forced conversion, is ignored (ERR_REG pmu_cmd_error) (p15-16). In normal mode only SUS and UPD_OAE are accepted. If more than 0.1 s passes between init and the first forced trigger, use FM, not FM_FAST (p15). PMU_CMD_STATUS_0 (p33): bit0 busy, bit1 odr_ovwr, bit2 avg_ovwr, bit3 pwr_mode_is_normal, bit4 cmd_is_illegal, bits 7:5 last command value (3 bits, so BR_FAST reads 7).
- Vendor mode-change delays (`bmm350_defs.h`): normal to suspend 6 ms; suspend to normal 38 ms; suspend to forced 15 / 17 / 20 / 28 ms for avg 1 / 2 / 4 / 8, fast forced 4 / 5 / 9 / 16 ms; UPD_OAE 1 ms. Datasheet: forced conversion under 5 ms (avg 1, FM_FAST), up to 30 ms (avg 8, FM) (p15). Maximum forced trigger rate: 200 / 150 / 80 / 40 Hz for avg 1 / 2 / 4 / 8 with FM_FAST (p16).
- ODR codes (PMU_CMD_AGGR_SET bits 3:0, p30): 0x02 400 Hz, 0x03 200, 0x04 100, 0x05 50, 0x06 25, 0x07 12.5, 0x08 6.25, 0x09 3.125, 0x0A 1.5625 Hz. Typical accuracy +-2 % at room temperature, +-4 % over temperature (p9). avg codes (bits 5:4): 0 no averaging, 1 = 2 samples, 2 = 4, 3 = 8. Allowed combinations (p15 Table 5): 400 Hz avg 0 only; 200 Hz avg 0-1; 100 Hz avg 0-2; 50 Hz and below avg 0-3. The vendor lowers an excessive avg to the highest allowed. After writing odr/avg send UPD_OAE; the next odr/avg update must wait for a data-ready after the busy flag clears (p32). PMU_CMD_STATUS_1 0x08 reports the effective odr (bits 3:0) and avg (bits 5:4).
- Axis enable: PMU_CMD_AXIS_EN bits 0-2; only effective in suspend, ignored on the fly in normal mode; the vendor refuses to disable all three (p20). Registers of disabled axes keep stale data.
- Interrupt (INT_CTRL 0x2E, p35): bit0 int_mode (1 latched, 0 pulsed), bit1 int_pol (1 active high), bit2 int_od (1 push-pull, 0 open-drain), bit3 int_output_en (maps the interrupt to the pin; reserved on A samples), bit7 drdy_data_reg_en. Reset 0x00 = open-drain, active low, pulsed, disabled. Push-pull, active high, pulsed, data-ready on = 0x8E (vendor interrupt example expects the same). INT_STATUS 0x30 bit2 drdy_data_reg, clears on read; in pulsed mode it clears itself after 1.25 ms (p20). INT pulse width: UNKNOWN. Measured: INT_CTRL reads back 0x8E; one rising edge per sample, 200 edges for 200 samples at 100 Hz and 799 for 799 at about 400 Hz, none missed.
- Data (p37-40): X 0x31-0x33, Y 0x34-0x36, Z 0x37-0x39, temperature 0x3A-0x3C, each little-endian (XLSB, LSB, MSB), read as one 12-byte burst from 0x31. The datasheet describes 21 significant bits in a 24-bit register (p13); the vendor sign-extends all 24 bits. Reset value of every data byte 0x7F.
- Temperature: degC = raw * 0.000981282 - 25.49 (vendor), then corrected as below. Measured: about 25.2 degC on the bench.
- Conversion to uT (vendor `read_out_raw_data`, `update_default_coefiecents`): X, Y: raw * 0.007069979 uT/LSB; Z: raw * 0.007174964 uT/LSB; these are (1e6 / 2^20) / (gain products 14.55 * 19.46 for X/Y, 9.0 * 31.0 for Z, * (1/1.5) * 0.714607238769531). Temperature uses 1 / (0.00204 * (1/1.5) * 0.714607238769531 * 2^20) = 0.000981282.
- Compensation (vendor `bmm350_get_compensated_mag_xyz_temp_data`): T = (1 + t_sens) * (degC raw - 25.49) + t_offs. Per axis, B = raw_uT * (1 + sens) + offset + tco * (T - T0), then B = B / (1 + tcs * (T - T0)). Then with d = 1 - cross_y_x * cross_x_y: X = (Bx - cross_x_y * By) / d; Y = (By - cross_y_x * Bx) / d; Z = Bz + (Bx * (cross_y_x * cross_z_y - cross_z_x) - By * (cross_z_y - cross_x_y * cross_z_x)) / d. The macros BMM350_SENS_CORR_Y (0.01) and BMM350_TCS_CORR_Z (0.0001) exist in `bmm350_defs.h` but `bmm350.c` v1.10.0 never uses them.
- Range: +-2000 uT per axis, and the vector sum must stay below 2000 uT (p19). Noise 190 nT rms X/Y and 450 nT rms Z at 100 Hz, avg 2 (p10).
- Not used by the driver: PAD_CTRL drive, I2C watchdog, sensortime, self-test (p17, TMR_SELFTEST_USER), I3C, out-of-range detection (`bmm350_oor.c`).
- Measured on the bench: OTP of this unit: offsets -136 -137 -13 uT, sensitivity -0.0234 -0.0234 0.0039, T0 30.03 degC. Field about 67 uT with std 0.16 / 0.14 / 0.40 uT (x / y / z) at 100 Hz, avg 2, in the bench environment. Rate 100.68 Hz at the 100 Hz setting and 402.7 Hz at 400 Hz (+0.68 %, within the +-2 % typical). Read of 12 data bytes at 400 kHz I2C: about 440 us mean, 480 us worst. init (reset, OTP, magnetic reset, configuration) about 85 ms. Forced mode (FM, avg 2): 15.5 ms from the trigger write to the data-ready edge. Right after init PMU_CMD_STATUS_0 read 0x11 (busy and cmd_is_illegal set, power mode not yet normal) and 0x28 (normal, last command NM) two seconds later; the cause of the transient illegal flag is UNKNOWN.

## Pins and bus (from pinout.md)

All GPIO are 3.3 V logic and are not 5 V tolerant.

| GPIO | Net | Function used | Status |
|---|---|---|---|
| 12 | MAG_SDA | I2C0 SDA (F3) | IO block row 5; 4.7k pull-up R25 |
| 13 | MAG_SCL | I2C0 SCL (F3) | IO block row 6; 4.7k pull-up R26 |
| 4 | PWM6 | PWM2 A (F4) | servo S6, IO block row 8; on the bench the BMM350 INT, a plain GPIO input |

| Peripheral | GPIOs | Device |
|---|---|---|
| I2C0 | SDA 12, SCL 13 | external magnetometer port (IO block rows 5/6) |

| Item | Value |
|---|---|
| Magnetometer I2C0 | 4.7k pull-ups R25/R26 to V3V3_SYS; the module is on the V3V3_SYS power pin of the IO block |

Bench configuration: BMM350 magnetometer on the magnetometer port: SDA = GPIO12, SCL = GPIO13 (I2C0, 4.7k pull-ups to V3V3_SYS, powered from the V3V3_SYS pin of the IO block). INT on silk S6 = GPIO4, a plain GPIO input (S6 is otherwise the servo PWM6 pad). It is attached by hand on the current board; the PCB itself provides the pins above. Silk names for IO block rows 5/6 are not recorded in the sources.

Rails: V3V3_SYS powers the microSD, its pull-ups, the PSRAM socket and the IO block power pins.

## Datasheets

| Device | URL |
|---|---|
| BMM350 datasheet (BST-BMM350-DS001-27) | https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm350-ds001.pdf |
| BMM350 Bosch Sensor API (BSD-3-Clause) | https://github.com/boschsensortec/BMM350_SensorAPI |
