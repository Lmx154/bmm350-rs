//! Register map, bit masks and command codes. Names follow the datasheet (BST-BMM350-DS001-27,
//! chapter 8, p26-43); anything taken only from Bosch's `BMM350_SensorAPI` is marked as such.

/// 7-bit I2C address with ADSEL tied to GND (datasheet p44-45, Table 10).
pub const ADDRESS_ADSEL_LOW: u8 = 0x14;
/// 7-bit I2C address with ADSEL tied to VDDIO (datasheet p44-45, Table 10).
pub const ADDRESS_ADSEL_HIGH: u8 = 0x15;
/// Number of dummy bytes the device sends before the data of every I2C read (datasheet p46, 9.2.3).
pub const DUMMY_BYTES: usize = 2;

/// `CHIP_ID`, read-only, value [`CHIP_ID_VALUE`].
pub const CHIP_ID: u8 = 0x00;
/// Expected content of [`CHIP_ID`] (datasheet p21, 5.10.1).
pub const CHIP_ID_VALUE: u8 = 0x33;
/// `ERR_REG`, bit 0 `pmu_cmd_error`; cleared on read.
pub const ERR_REG: u8 = 0x02;
/// `PAD_CTRL`, bits 2:0 pad drive strength. Not used by the driver.
pub const PAD_CTRL: u8 = 0x03;
/// `PMU_CMD_AGGR_SET`: bits 3:0 `odr`, bits 5:4 `avg`.
pub const PMU_CMD_AGGR_SET: u8 = 0x04;
/// `PMU_CMD_AXIS_EN`: bit 0 `en_x`, bit 1 `en_y`, bit 2 `en_z`.
pub const PMU_CMD_AXIS_EN: u8 = 0x05;
/// `PMU_CMD`: bits 3:0 `pmu_cmd`, see the `PMU_CMD_*` codes.
pub const PMU_CMD: u8 = 0x06;
/// `PMU_CMD_STATUS_0`: see the `PMU_STATUS_0_*` masks.
pub const PMU_CMD_STATUS_0: u8 = 0x07;
/// `PMU_CMD_STATUS_1`: bits 3:0 effective `odr`, bits 5:4 effective `avg`.
pub const PMU_CMD_STATUS_1: u8 = 0x08;
/// `I2C_WDT_SET`. Not used by the driver.
pub const I2C_WDT_SET: u8 = 0x0A;
/// `INT_CTRL`: see the `INT_CTRL_*` masks.
pub const INT_CTRL: u8 = 0x2E;
/// `INT_STATUS`: bit 2 `drdy_data_reg`; cleared on read.
pub const INT_STATUS: u8 = 0x30;
/// `MAG_X_XLSB`, first byte of the 12-byte block X, Y, Z, temperature (3 bytes each, little-endian).
pub const MAG_X_XLSB: u8 = 0x31;
/// `MAG_Y_XLSB`.
pub const MAG_Y_XLSB: u8 = 0x34;
/// `MAG_Z_XLSB`.
pub const MAG_Z_XLSB: u8 = 0x37;
/// `TEMP_XLSB`.
pub const TEMP_XLSB: u8 = 0x3A;
/// `SENSORTIME_XLSB`, 24-bit counter. Not used by the driver.
pub const SENSORTIME_XLSB: u8 = 0x3D;
/// `OTP_CMD_REG`: bits 7:5 `otp_cmd`, bits 4:0 `word_addr`.
pub const OTP_CMD_REG: u8 = 0x50;
/// `OTP_DATA_MSB_REG`.
pub const OTP_DATA_MSB_REG: u8 = 0x52;
/// `OTP_DATA_LSB_REG`.
pub const OTP_DATA_LSB_REG: u8 = 0x53;
/// `OTP_STATUS_REG`: bit 0 `otp_cmd_done`, bits 7:5 `error`.
pub const OTP_STATUS_REG: u8 = 0x55;
/// `CMD`, write-only.
pub const CMD: u8 = 0x7E;

/// `CMD` value: no command.
pub const CMD_NOP: u8 = 0x00;
/// `CMD` value: soft reset, equivalent to a power-on reset (datasheet p43).
pub const CMD_SOFTRESET: u8 = 0xB6;

/// `PMU_CMD` code: go to suspend (datasheet p32).
pub const PMU_CMD_SUS: u8 = 0x00;
/// `PMU_CMD` code: go to normal mode.
pub const PMU_CMD_NM: u8 = 0x01;
/// `PMU_CMD` code: apply the new `odr` and `avg` from `PMU_CMD_AGGR_SET`.
pub const PMU_CMD_UPD_OAE: u8 = 0x02;
/// `PMU_CMD` code: forced mode with full CRST recharge.
pub const PMU_CMD_FM: u8 = 0x03;
/// `PMU_CMD` code: forced mode with fast CRST recharge.
pub const PMU_CMD_FM_FAST: u8 = 0x04;
/// `PMU_CMD` code: flux-guide reset, full recharge.
pub const PMU_CMD_FGR: u8 = 0x05;
/// `PMU_CMD` code: flux-guide reset, fast recharge.
pub const PMU_CMD_FGR_FAST: u8 = 0x06;
/// `PMU_CMD` code: bit reset, full recharge.
pub const PMU_CMD_BR: u8 = 0x07;
/// `PMU_CMD` code: bit reset, fast recharge.
pub const PMU_CMD_BR_FAST: u8 = 0x08;

/// `PMU_CMD_STATUS_0` bit 0: the previous PMU command is still running.
pub const PMU_STATUS_0_BUSY: u8 = 0x01;
/// `PMU_CMD_STATUS_0` bit 3: the chip is in normal mode.
pub const PMU_STATUS_0_NORMAL: u8 = 0x08;
/// `PMU_CMD_STATUS_0` bit 4: the last command was illegal.
pub const PMU_STATUS_0_ILLEGAL: u8 = 0x10;
/// `PMU_CMD_STATUS_0` bits 7:5: value of the last PMU command (3 bits: BR and BR_FAST both read 7).
pub const PMU_STATUS_0_CMD_VALUE_MASK: u8 = 0xE0;

/// `INT_CTRL` bit 0: 1 latched, 0 pulsed (datasheet p35).
pub const INT_CTRL_LATCHED: u8 = 0x01;
/// `INT_CTRL` bit 1: 1 active high, 0 active low.
pub const INT_CTRL_ACTIVE_HIGH: u8 = 0x02;
/// `INT_CTRL` bit 2: 1 push-pull, 0 open-drain.
pub const INT_CTRL_PUSH_PULL: u8 = 0x04;
/// `INT_CTRL` bit 3: map the interrupt to the INT pin (reserved on A samples, datasheet p35).
pub const INT_CTRL_OUTPUT_EN: u8 = 0x08;
/// `INT_CTRL` bit 7: data-ready interrupt enable.
pub const INT_CTRL_DRDY_EN: u8 = 0x80;

/// `INT_STATUS` bit 2: new magnetic data.
pub const INT_STATUS_DRDY: u8 = 0x04;

/// `OTP_CMD_REG` command (bits 7:5): read the word addressed in bits 4:0.
pub const OTP_CMD_DIR_READ: u8 = 0x20;
/// `OTP_CMD_REG` command: power the OTP off, ending the boot phase (datasheet p14).
pub const OTP_CMD_PWR_OFF_OTP: u8 = 0x80;
/// `OTP_STATUS_REG` bit 0: command done.
pub const OTP_STATUS_CMD_DONE: u8 = 0x01;
/// `OTP_STATUS_REG` bits 7:5: error code, 0 is no error (datasheet p42).
pub const OTP_STATUS_ERROR_MASK: u8 = 0xE0;

/// Number of 16-bit words in the OTP (word address is 5 bits). Vendor `bmm350_defs.h`: `BMM350_OTP_DATA_LENGTH`.
pub const OTP_WORDS: usize = 32;
