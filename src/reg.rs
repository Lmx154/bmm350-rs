//! Register map, bit masks and command codes. Names follow the datasheet (BST-BMM350-DS001-27,
//! chapter 8, register map Table 8 on p26-28, register descriptions 8.1 to 8.35 on p29-43).
//! Page numbers are the printed ones (the table of contents numbering). Anything taken only from
//! Bosch's `BMM350_SensorAPI` is marked "vendor".
//!
//! A single-bit field is a mask (`1 << n`). A multi-bit field has a `_SHIFT` and a `_MASK` (already
//! shifted). `*_RESET` is the value after power-on or soft reset. `*_RESERVED` is the mask of the
//! bits the datasheet calls reserved; an image built by this crate keeps them at their reset value.

/// 7-bit I2C address with ADSEL tied to GND (datasheet p44-45, Table 10).
pub const ADDRESS_ADSEL_LOW: u8 = 0x14;
/// 7-bit I2C address with ADSEL tied to VDDIO (datasheet p44-45, Table 10).
pub const ADDRESS_ADSEL_HIGH: u8 = 0x15;
/// Number of dummy bytes the device sends before the data of every I2C read (datasheet p46, 9.2.3).
pub const DUMMY_BYTES: usize = 2;

/// `CHIP_ID` (8.1, p29), read-only.
pub const CHIP_ID: u8 = 0x00;
/// Expected content of [`CHIP_ID`] (8.1, p29; datasheet p21, 5.10.1).
pub const CHIP_ID_VALUE: u8 = 0x33;
/// `CHIP_ID` bits 7:4, `chip_id_fixed`, 0x2 (8.1, p29).
pub const CHIP_ID_FIXED_MASK: u8 = 0xF0;
/// `CHIP_ID` bits 3:0, `chip_id_otp` (NVM part), 0x3 (8.1, p29).
pub const CHIP_ID_OTP_MASK: u8 = 0x0F;

/// `ERR_REG` (8.2, p29): cleared on read, a 1 written to a bit clears it.
pub const ERR_REG: u8 = 0x02;
/// `ERR_REG` reset value (8.2, p29).
pub const ERR_REG_RESET: u8 = 0x00;
/// `ERR_REG` bit 0 `pmu_cmd_error`: a PMU command was issued while the previous one was not
/// finished and was ignored (8.2, p29).
pub const ERR_REG_PMU_CMD_ERROR: u8 = 0x01;

/// `PAD_CTRL` (8.3, p30). Fact only: not used by the driver.
pub const PAD_CTRL: u8 = 0x03;
/// `PAD_CTRL` reset value, strongest drive (8.3, p30).
pub const PAD_CTRL_RESET: u8 = 0x07;
/// `PAD_CTRL` bits 2:0 `drv`: 0 weakest, 7 strongest, n is n/7 in between (8.3, p30).
pub const PAD_CTRL_DRV_MASK: u8 = 0x07;

/// `PMU_CMD_AGGR_SET` (8.4, p30): bits 3:0 `odr`, bits 5:4 `avg`.
pub const PMU_CMD_AGGR_SET: u8 = 0x04;
/// `PMU_CMD_AGGR_SET` reset value: 100 Hz, `avg` code 1, which is averaging 2 (8.4, p30).
pub const PMU_CMD_AGGR_SET_RESET: u8 = 0x14;
/// `PMU_CMD_AGGR_SET` bits 3:0 `odr` (8.4, p30).
pub const PMU_CMD_AGGR_SET_ODR_MASK: u8 = 0x0F;
/// `PMU_CMD_AGGR_SET` bit position of `avg` (8.4, p30).
pub const PMU_CMD_AGGR_SET_AVG_SHIFT: u8 = 4;
/// `PMU_CMD_AGGR_SET` bits 5:4 `avg` (8.4, p30).
pub const PMU_CMD_AGGR_SET_AVG_MASK: u8 = 0x30;
/// `PMU_CMD_AGGR_SET` bits 7:6, reserved (8.4, p30).
pub const PMU_CMD_AGGR_SET_RESERVED: u8 = 0xC0;
/// `odr` code: 400 Hz (8.4, p30).
pub const ODR_400HZ: u8 = 0x02;
/// `odr` code: 200 Hz (8.4, p30).
pub const ODR_200HZ: u8 = 0x03;
/// `odr` code: 100 Hz (8.4, p30).
pub const ODR_100HZ: u8 = 0x04;
/// `odr` code: 50 Hz (8.4, p30).
pub const ODR_50HZ: u8 = 0x05;
/// `odr` code: 25 Hz (8.4, p30).
pub const ODR_25HZ: u8 = 0x06;
/// `odr` code: 12.5 Hz (8.4, p30).
pub const ODR_12_5HZ: u8 = 0x07;
/// `odr` code: 6.25 Hz (8.4, p30).
pub const ODR_6_25HZ: u8 = 0x08;
/// `odr` code: 3.125 Hz (8.4, p30).
pub const ODR_3_125HZ: u8 = 0x09;
/// `odr` code: 1.5625 Hz (8.4, p30).
pub const ODR_1_5625HZ: u8 = 0x0A;
/// `avg` code: no averaging (8.4, p30).
pub const AVG_1: u8 = 0x00;
/// `avg` code: 2 samples (8.4, p30).
pub const AVG_2: u8 = 0x01;
/// `avg` code: 4 samples (8.4, p30).
pub const AVG_4: u8 = 0x02;
/// `avg` code: 8 samples (8.4, p30).
pub const AVG_8: u8 = 0x03;

/// `PMU_CMD_AXIS_EN` (8.5, p31): bit 0 `en_x`, bit 1 `en_y`, bit 2 `en_z`.
pub const PMU_CMD_AXIS_EN: u8 = 0x05;
/// `PMU_CMD_AXIS_EN` reset value, all axes on (8.5, p31). Table 8 (p28) prints 0x70 for this
/// register and places the enables in bits 6:4; 8.5 and the vendor driver say 0x07 and bits 2:0.
pub const PMU_CMD_AXIS_EN_RESET: u8 = 0x07;
/// `PMU_CMD_AXIS_EN` bit 0 `en_x` (8.5, p31).
pub const AXIS_EN_X: u8 = 0x01;
/// `PMU_CMD_AXIS_EN` bit 1 `en_y` (8.5, p31).
pub const AXIS_EN_Y: u8 = 0x02;
/// `PMU_CMD_AXIS_EN` bit 2 `en_z` (8.5, p31).
pub const AXIS_EN_Z: u8 = 0x04;
/// `PMU_CMD_AXIS_EN` bits 7:3, reserved (8.5, p31).
pub const PMU_CMD_AXIS_EN_RESERVED: u8 = 0xF8;

/// `PMU_CMD` (8.6, p32): bits 3:0 `pmu_cmd`, see the `PMU_CMD_*` codes.
pub const PMU_CMD: u8 = 0x06;
/// `PMU_CMD` reset value (8.6, p32).
pub const PMU_CMD_RESET: u8 = 0x00;
/// `PMU_CMD` bits 3:0 `pmu_cmd` (8.6, p32).
pub const PMU_CMD_MASK: u8 = 0x0F;
/// `PMU_CMD` code SUS: go to suspend (8.6, p32).
pub const PMU_CMD_SUS: u8 = 0x00;
/// `PMU_CMD` code NM: go to normal mode. In normal mode only SUS and UPD_OAE are accepted; any
/// other command is blocked and sets `cmd_is_illegal`, and `pmu_cmd_busy` must be low before the
/// next command (8.6, p32).
pub const PMU_CMD_NM: u8 = 0x01;
/// `PMU_CMD` code UPD_OAE: apply the `odr` and `avg` of `PMU_CMD_AGGR_SET`. After one update, wait
/// for the data-ready interrupt after `pmu_cmd_busy` cleared before the next update; the busy flag
/// can clear too early (8.6, p32).
pub const PMU_CMD_UPD_OAE: u8 = 0x02;
/// `PMU_CMD` code FM: forced mode, full CRST recharge; only from suspend (8.6, p32; 5.1.4, p15).
pub const PMU_CMD_FM: u8 = 0x03;
/// `PMU_CMD` code FM_FAST: forced mode, fast CRST recharge (8.6, p32).
pub const PMU_CMD_FM_FAST: u8 = 0x04;
/// `PMU_CMD` code FGR: flux-guide reset, full recharge (8.6, p32).
pub const PMU_CMD_FGR: u8 = 0x05;
/// `PMU_CMD` code FGR_FAST: flux-guide reset, fast recharge (8.6, p32).
pub const PMU_CMD_FGR_FAST: u8 = 0x06;
/// `PMU_CMD` code BR: bit reset, full recharge (8.6, p32).
pub const PMU_CMD_BR: u8 = 0x07;
/// `PMU_CMD` code BR_FAST: bit reset, fast recharge (8.6, p32).
pub const PMU_CMD_BR_FAST: u8 = 0x08;

/// `PMU_CMD_STATUS_0` (8.7, p33), read-only.
pub const PMU_CMD_STATUS_0: u8 = 0x07;
/// `PMU_CMD_STATUS_0` reset value (8.7, p33).
pub const PMU_CMD_STATUS_0_RESET: u8 = 0x00;
/// `PMU_CMD_STATUS_0` bit 0 `pmu_cmd_busy`: the previous PMU command is still running (8.7, p33).
pub const PMU_STATUS_0_BUSY: u8 = 0x01;
/// `PMU_CMD_STATUS_0` bit 1 `ODR_ovwr`: the previous `odr` was overwritten (8.7, p33).
pub const PMU_STATUS_0_ODR_OVWR: u8 = 0x02;
/// `PMU_CMD_STATUS_0` bit 2 `AVG_ovwr`: the previous `avg` was overwritten (8.7, p33).
pub const PMU_STATUS_0_AVG_OVWR: u8 = 0x04;
/// `PMU_CMD_STATUS_0` bit 3 `pwr_mode_is_normal` (8.7, p33).
pub const PMU_STATUS_0_NORMAL: u8 = 0x08;
/// `PMU_CMD_STATUS_0` bit 4 `cmd_is_illegal` (8.7, p33).
pub const PMU_STATUS_0_ILLEGAL: u8 = 0x10;
/// `PMU_CMD_STATUS_0` bits 7:5: the value of the last PMU command, 3 bits, so BR and BR_FAST both
/// read 7. Table 8 (p28) names the field `pmu_cmd_val`; 8.7 (p33) calls the bits reserved; the
/// vendor driver reads them as the command echo.
pub const PMU_STATUS_0_CMD_VALUE_MASK: u8 = 0xE0;

/// `PMU_CMD_STATUS_1` (8.8, p34), read-only: the effective `odr` and `avg`.
pub const PMU_CMD_STATUS_1: u8 = 0x08;
/// `PMU_CMD_STATUS_1` reset value (8.8, p34).
pub const PMU_CMD_STATUS_1_RESET: u8 = 0x00;
/// `PMU_CMD_STATUS_1` bits 3:0 `pmu_odr_s` (8.8, p34).
pub const PMU_CMD_STATUS_1_ODR_MASK: u8 = 0x0F;
/// `PMU_CMD_STATUS_1` bit position of `pmu_avg_s` (8.8, p34).
pub const PMU_CMD_STATUS_1_AVG_SHIFT: u8 = 4;
/// `PMU_CMD_STATUS_1` bits 5:4 `pmu_avg_s` (8.8, p34).
pub const PMU_CMD_STATUS_1_AVG_MASK: u8 = 0x30;

/// `I2C_WDT_SET` (8.10, p34-35).
pub const I2C_WDT_SET: u8 = 0x0A;
/// `I2C_WDT_SET` reset value, watchdog off (8.10, p34).
pub const I2C_WDT_SET_RESET: u8 = 0x00;
/// `I2C_WDT_SET` bit 0 `i2c_wdt_en`: watchdog on (8.10, p34).
pub const I2C_WDT_EN: u8 = 0x01;
/// `I2C_WDT_SET` bit 1 `i2c_wdt_sel`: 0 time-out after 1.28 ms, 1 after 40.96 ms (8.10, p35).
pub const I2C_WDT_SEL_LONG: u8 = 0x02;
/// `I2C_WDT_SET` bits 7:2, reserved (8.10, p34).
pub const I2C_WDT_SET_RESERVED: u8 = 0xFC;

/// `TRANSDUCER_REV_ID` (Table 8, p27; no description section), read-only.
pub const TRANSDUCER_REV_ID: u8 = 0x0D;
/// `TRANSDUCER_REV_ID` reset value (Table 8, p27).
pub const TRANSDUCER_REV_ID_RESET: u8 = 0x33;
/// `TRANSDUCER_REV_ID` bits 7:2 `trsdcr_rev_id_fixed` (vendor `BMM350_TRSDCR_REV_ID_FIXED_MSK`; the
/// datasheet gives no bit positions).
pub const TRANSDUCER_REV_ID_FIXED_MASK: u8 = 0xFC;
/// `TRANSDUCER_REV_ID` bits 1:0 `trsdcr_rev_id_otp` (vendor `BMM350_TRSDCR_REV_ID_OTP_MSK`).
pub const TRANSDUCER_REV_ID_OTP_MASK: u8 = 0x03;

/// `INT_CTRL` (8.11, p35): see the `INT_CTRL_*` masks.
pub const INT_CTRL: u8 = 0x2E;
/// `INT_CTRL` reset value, interrupt off (8.11, p35).
pub const INT_CTRL_RESET: u8 = 0x00;
/// `INT_CTRL` bit 0 `int_mode`: 1 latched, 0 pulsed (8.11, p35; 5.5.4, p20).
pub const INT_CTRL_LATCHED: u8 = 0x01;
/// `INT_CTRL` bit 1 `int_pol`: 1 active high, 0 active low (8.11, p35; 5.5.2, p20).
pub const INT_CTRL_ACTIVE_HIGH: u8 = 0x02;
/// `INT_CTRL` bit 2 `int_od`: 1 push-pull, 0 open-drain (8.11, p35; 5.5.3, p20).
pub const INT_CTRL_PUSH_PULL: u8 = 0x04;
/// `INT_CTRL` bit 3 `int_output_en`: map the interrupt to the INT pin (reserved on A samples,
/// where the mapping is automatic; 8.11, p35; 5.5.1, p20).
pub const INT_CTRL_OUTPUT_EN: u8 = 0x08;
/// `INT_CTRL` bit 7 `drdy_data_reg_en`: data-ready interrupt on the pin and in `INT_STATUS`
/// (8.11, p35).
pub const INT_CTRL_DRDY_EN: u8 = 0x80;
/// `INT_CTRL` bits 6:4, reserved (8.11, p35).
pub const INT_CTRL_RESERVED: u8 = 0x70;

/// `INT_CTRL_IBI` (8.12, p36). Fact only (I3C): not used by the driver.
pub const INT_CTRL_IBI: u8 = 0x2F;
/// `INT_CTRL_IBI` reset value (8.12, p36).
pub const INT_CTRL_IBI_RESET: u8 = 0x00;
/// `INT_CTRL_IBI` bit 0 `drdy_int_map_to_ibi` (8.12, p36).
pub const INT_CTRL_IBI_DRDY_MAP: u8 = 0x01;
/// `INT_CTRL_IBI` bit 4 `clear_drdy_int_status_upon_ibi` (8.12, p36).
pub const INT_CTRL_IBI_CLEAR_UPON_IBI: u8 = 0x10;

/// `INT_STATUS` (8.13, p36): cleared on read (latched mode), a 1 written clears the bit; in pulsed
/// mode the bit clears itself after 1.25 ms (5.5, p20).
pub const INT_STATUS: u8 = 0x30;
/// `INT_STATUS` reset value (8.13, p36).
pub const INT_STATUS_RESET: u8 = 0x00;
/// `INT_STATUS` bit 2 `drdy_data_reg`: new magnetic data (8.13, p36).
pub const INT_STATUS_DRDY: u8 = 0x04;

/// Reset value of every data, temperature and sensor-time byte (8.14 to 8.28, p37-40).
pub const DATA_RESET: u8 = 0x7F;
/// `MAG_X_XLSB` (8.14, p37), first byte of the 12-byte block X, Y, Z, temperature (3 bytes each,
/// little-endian: XLSB, LSB, MSB).
pub const MAG_X_XLSB: u8 = 0x31;
/// `MAG_Y_XLSB` (8.17, p38).
pub const MAG_Y_XLSB: u8 = 0x34;
/// `MAG_Z_XLSB` (8.20, p38).
pub const MAG_Z_XLSB: u8 = 0x37;
/// `TEMP_XLSB` (8.23, p39).
pub const TEMP_XLSB: u8 = 0x3A;
/// `SENSORTIME_XLSB` (8.26, p40), 24-bit counter in 0x3D to 0x3F, same burst as the data.
pub const SENSORTIME_XLSB: u8 = 0x3D;
/// `SENSORTIME_LSB` (8.27, p40).
pub const SENSORTIME_LSB: u8 = 0x3E;
/// `SENSORTIME_MSB` (8.28, p40).
pub const SENSORTIME_MSB: u8 = 0x3F;

/// `OTP_CMD_REG` (8.29, p41): bits 7:5 `otp_cmd`, bits 4:0 `word_addr`.
pub const OTP_CMD_REG: u8 = 0x50;
/// `OTP_CMD_REG` reset value (8.29, p41).
pub const OTP_CMD_REG_RESET: u8 = 0x00;
/// `OTP_CMD_REG` bits 7:5 `otp_cmd` (8.29, p41).
pub const OTP_CMD_MASK: u8 = 0xE0;
/// `OTP_CMD_REG` bits 4:0 `word_addr` (8.29, p41).
pub const OTP_CMD_WORD_ADDR_MASK: u8 = 0x1F;
/// `OTP_CMD_REG` command (bits 7:5) DIR_READ: read the word addressed in bits 4:0 (8.29, p41).
pub const OTP_CMD_DIR_READ: u8 = 0x20;
/// `OTP_CMD_REG` command DIR_PRGM_1B (8.29, p41). Programming: never used.
pub const OTP_CMD_DIR_PRGM_1B: u8 = 0x40;
/// `OTP_CMD_REG` command DIR_PRGM (8.29, p41). Programming: never used.
pub const OTP_CMD_DIR_PRGM: u8 = 0x60;
/// `OTP_CMD_REG` command PWR_OFF_OTP: ends the boot phase (8.29, p41; 5.1, p14).
pub const OTP_CMD_PWR_OFF_OTP: u8 = 0x80;
/// `OTP_CMD_REG` command EXT_READ (8.29, p41).
pub const OTP_CMD_EXT_READ: u8 = 0xA0;
/// `OTP_CMD_REG` command EXT_PRGM (8.29, p41). Programming: never used.
pub const OTP_CMD_EXT_PRGM: u8 = 0xE0;
/// `OTP_DATA_MSB_REG` (8.30, p41).
pub const OTP_DATA_MSB_REG: u8 = 0x52;
/// `OTP_DATA_LSB_REG` (8.31, p41).
pub const OTP_DATA_LSB_REG: u8 = 0x53;
/// `OTP_DATA_MSB_REG` and `OTP_DATA_LSB_REG` reset value (8.30, 8.31, p41).
pub const OTP_DATA_RESET: u8 = 0x00;
/// `OTP_STATUS_REG` (8.32, p42).
pub const OTP_STATUS_REG: u8 = 0x55;
/// `OTP_STATUS_REG` reset value (8.32, p42).
pub const OTP_STATUS_REG_RESET: u8 = 0x10;
/// `OTP_STATUS_REG` bit 0 `otp_cmd_done`: a new command can be accepted (8.32, p42).
pub const OTP_STATUS_CMD_DONE: u8 = 0x01;
/// `OTP_STATUS_REG` bits 7:5 `error`, 0 is no error (8.32, p42).
pub const OTP_STATUS_ERROR_MASK: u8 = 0xE0;
/// `OTP_STATUS_REG` error code BOOT_ERR, in place (8.32, p42).
pub const OTP_STATUS_BOOT_ERR: u8 = 0x20;
/// `OTP_STATUS_REG` error code PAGE_RD_ERR, in place (8.32, p42).
pub const OTP_STATUS_PAGE_RD_ERR: u8 = 0x40;
/// `OTP_STATUS_REG` error code PAGE_PRG_ERR, in place (8.32, p42).
pub const OTP_STATUS_PAGE_PRG_ERR: u8 = 0x60;
/// `OTP_STATUS_REG` error code SIGN_ERR, in place (8.32, p42).
pub const OTP_STATUS_SIGN_ERR: u8 = 0x80;
/// `OTP_STATUS_REG` error code INV_CMD_ERR, in place (8.32, p42).
pub const OTP_STATUS_INV_CMD_ERR: u8 = 0xA0;

/// `TMR_SELFTEST_USER` (8.33, p42). Never drive `st_n` and `st_p` together: that shorts the
/// self-test circuit (8.33, p42).
pub const TMR_SELFTEST_USER: u8 = 0x60;
/// `TMR_SELFTEST_USER` reset value (8.33, p42).
pub const TMR_SELFTEST_USER_RESET: u8 = 0x00;
/// `TMR_SELFTEST_USER` bit 0 `st_igen_en`: self-test current generator on; a precondition of the
/// self-test (8.33, p42; bit positions: vendor `BMM350_ST_IGEN_EN_MSK`).
pub const ST_IGEN_EN: u8 = 0x01;
/// `TMR_SELFTEST_USER` bit 1 `st_n`: negative-field self-test (8.33, p42; bit: vendor).
pub const ST_N: u8 = 0x02;
/// `TMR_SELFTEST_USER` bit 2 `st_p`: positive-field self-test (8.33, p42; bit: vendor).
pub const ST_P: u8 = 0x04;
/// `TMR_SELFTEST_USER` bit 3 `ist_en_x`: self-test field on X (8.33, p42; bit: vendor).
pub const IST_EN_X: u8 = 0x08;
/// `TMR_SELFTEST_USER` bit 4 `ist_en_y`: self-test field on Y (8.33, p42; bit: vendor).
pub const IST_EN_Y: u8 = 0x10;

/// `CTRL_USER` (8.34, p43).
pub const CTRL_USER: u8 = 0x61;
/// `CTRL_USER` reset value (8.34, p43).
pub const CTRL_USER_RESET: u8 = 0x00;
/// `CTRL_USER` bit 0 `cfg_sens_tim_aon`: the sensor timer also runs in suspend and forced mode;
/// writable only in suspend (8.34, p43; 5.2.2, p18).
pub const CTRL_USER_SENS_TIM_AON: u8 = 0x01;

/// `CMD`, write-only, reads 0x00 (8.35, p43).
pub const CMD: u8 = 0x7E;
/// `CMD` value: no command (8.35, p43).
pub const CMD_NOP: u8 = 0x00;
/// `CMD` value: soft reset, equivalent to a power-on reset (8.35, p43).
pub const CMD_SOFTRESET: u8 = 0xB6;

/// Number of 16-bit words in the OTP (word address is 5 bits). Vendor `bmm350_defs.h`: `BMM350_OTP_DATA_LENGTH`.
pub const OTP_WORDS: usize = 32;
