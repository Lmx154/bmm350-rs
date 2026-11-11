//! Raw sample decoding, the calibration coefficients from OTP, and compensation to physical units.
//! Everything here is pure and runs on the host.
//!
//! The datasheet (BST-BMM350-DS001-27 p18) leaves the OTP layout and the compensation arithmetic
//! to Bosch's API. They are reimplemented from the behaviour of `bmm350.c` and `bmm350_defs.h`
//! in `BMM350_SensorAPI` v1.10.0 (BSD-3-Clause); no code is copied.

use crate::reg::OTP_WORDS;

/// Vendor `lut_gain` (0.714607238769531 in `bmm350.c`, rounded to `f32`).
const LUT_GAIN: f32 = 0.714_607_24;

/// Microtesla per LSB, X and Y. From `update_default_coefiecents` in the vendor `bmm350.c`:
/// `(1e6/2^20) / (14.55 * 19.46 * (1/1.5) * 0.714607238769531)`.
pub const XY_UT_PER_LSB: f32 =
    (1_000_000.0 / 1_048_576.0) / (14.55 * 19.46 * (1.0 / 1.5) * LUT_GAIN);
/// Microtesla per LSB, Z. Vendor `bmm350.c`, same function: gain 9.0 * 31.0 instead of 14.55 * 19.46.
pub const Z_UT_PER_LSB: f32 = (1_000_000.0 / 1_048_576.0) / (9.0 * 31.0 * (1.0 / 1.5) * LUT_GAIN);
/// Degrees Celsius per LSB of the temperature channel. Vendor `bmm350.c`:
/// `1 / (0.00204 * (1/1.5) * 0.714607238769531 * 2^20)`.
pub const TEMP_DEG_PER_LSB: f32 = 1.0 / (0.00204 * (1.0 / 1.5) * LUT_GAIN * 1_048_576.0);
/// Offset subtracted from the scaled temperature before the OTP correction. Vendor `bmm350.c`.
pub const TEMP_ZERO_DEG: f32 = 25.49;

/// Sign-extend the low `bits` bits of `v`.
const fn sign_extend(v: u32, bits: u32) -> i32 {
    let shift = 32 - bits;
    ((v << shift) as i32) >> shift
}

/// One burst of uncompensated data, sign-extended.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawSample {
    /// X counts.
    pub x: i32,
    /// Y counts.
    pub y: i32,
    /// Z counts.
    pub z: i32,
    /// Temperature counts.
    pub temperature: i32,
}

impl RawSample {
    /// Decode the 12 data bytes at `MAG_X_XLSB`: X, Y, Z, temperature, each a little-endian
    /// 24-bit two's-complement value (datasheet p13, p37-40).
    pub const fn decode(b: &[u8; 12]) -> Self {
        const fn ch(lo: u8, mid: u8, hi: u8) -> i32 {
            sign_extend((lo as u32) | ((mid as u32) << 8) | ((hi as u32) << 16), 24)
        }
        let [x0, x1, x2, y0, y1, y2, z0, z1, z2, t0, t1, t2] = *b;
        Self {
            x: ch(x0, x1, x2),
            y: ch(y0, y1, y2),
            z: ch(z0, z1, z2),
            temperature: ch(t0, t1, t2),
        }
    }
}

impl RawSample {
    /// Decode the 15 register bytes at `MAG_X_XLSB` to `SENSORTIME_MSB` (X, Y, Z, temperature,
    /// then the 24-bit sensor time, little-endian, datasheet p37-40): the sample and the sensor
    /// time in ticks, from 0 to `0x00FF_FFFF`.
    pub const fn decode_with_time(b: &[u8; 15]) -> (Self, u32) {
        let [x0, x1, x2, y0, y1, y2, z0, z1, z2, t0, t1, t2, s0, s1, s2] = *b;
        (
            Self::decode(&[x0, x1, x2, y0, y1, y2, z0, z1, z2, t0, t1, t2]),
            (s0 as u32) | ((s1 as u32) << 8) | ((s2 as u32) << 16),
        )
    }
}

/// Compensated measurement.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Field {
    /// X in microtesla.
    pub x: f32,
    /// Y in microtesla.
    pub y: f32,
    /// Z in microtesla.
    pub z: f32,
    /// Die temperature in degrees Celsius.
    pub temperature: f32,
}

/// Per-device coefficients decoded from the OTP words. Axis arrays are ordered X, Y, Z.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Calibration {
    /// Offset per axis, microtesla (signed 12 bit).
    pub offset: [f32; 3],
    /// Sensitivity error per axis, fraction (signed 8 bit / 256).
    pub sensitivity: [f32; 3],
    /// Temperature coefficient of the offset per axis, microtesla per kelvin (signed 8 bit / 32).
    pub tco: [f32; 3],
    /// Temperature coefficient of the sensitivity per axis, per kelvin (signed 8 bit / 16384).
    pub tcs: [f32; 3],
    /// Temperature offset, kelvin (signed 8 bit / 5).
    pub temp_offset: f32,
    /// Temperature sensitivity error, fraction (signed 8 bit / 512).
    pub temp_sensitivity: f32,
    /// Reference temperature of the coefficients, degrees Celsius (signed 16 bit / 512 + 23).
    pub t0: f32,
    /// Cross-axis coefficient X from Y (signed 8 bit / 800).
    pub cross_x_y: f32,
    /// Cross-axis coefficient Y from X.
    pub cross_y_x: f32,
    /// Cross-axis coefficient Z from X.
    pub cross_z_x: f32,
    /// Cross-axis coefficient Z from Y.
    pub cross_z_y: f32,
}

impl Default for Calibration {
    /// The coefficients of an all-zero OTP: no correction, reference temperature 23 degrees.
    fn default() -> Self {
        Self::from_otp(&[0; OTP_WORDS])
    }
}

impl Calibration {
    /// Decode the 32 OTP words. Word map and scalings from the vendor `bmm350.c`,
    /// `update_mag_off_sens`; the words used are 0x0D-0x16 and 0x18:
    ///
    /// - 0x0D: low byte temperature offset, high byte temperature sensitivity
    /// - 0x0E: bits 11:0 offset X; bits 15:12 are bits 11:8 of offset Y
    /// - 0x0F: low byte is bits 7:0 of offset Y; bits 11:8 are bits 11:8 of offset Z
    /// - 0x10: low byte is bits 7:0 of offset Z; high byte sensitivity X
    /// - 0x11: low byte sensitivity Y; high byte sensitivity Z
    /// - 0x12, 0x13, 0x14: low byte TCO, high byte TCS, for X, Y, Z
    /// - 0x15: low byte cross X-Y; high byte cross Y-X
    /// - 0x16: low byte cross Z-X; high byte cross Z-Y
    /// - 0x18: signed 16-bit reference temperature
    pub fn from_otp(w: &[u16; OTP_WORDS]) -> Self {
        let word = |i: usize| w.get(i).copied().unwrap_or(0);
        let lo = |i: usize| word(i) as u8 as i8 as f32;
        let hi = |i: usize| (word(i) >> 8) as u8 as i8 as f32;
        let s12 = |v: u16| sign_extend(v as u32, 12) as f32;
        let off_x = w[0x0E] & 0x0FFF;
        let off_y = ((w[0x0E] & 0xF000) >> 4) | (w[0x0F] & 0x00FF);
        let off_z = (w[0x0F] & 0x0F00) | (w[0x10] & 0x00FF);
        Self {
            offset: [s12(off_x), s12(off_y), s12(off_z)],
            sensitivity: [hi(0x10) / 256.0, lo(0x11) / 256.0, hi(0x11) / 256.0],
            tco: [lo(0x12) / 32.0, lo(0x13) / 32.0, lo(0x14) / 32.0],
            tcs: [hi(0x12) / 16384.0, hi(0x13) / 16384.0, hi(0x14) / 16384.0],
            temp_offset: lo(0x0D) / 5.0,
            temp_sensitivity: hi(0x0D) / 512.0,
            t0: (w[0x18] as i16 as f32) / 512.0 + 23.0,
            cross_x_y: lo(0x15) / 800.0,
            cross_y_x: hi(0x15) / 800.0,
            cross_z_x: lo(0x16) / 800.0,
            cross_z_y: hi(0x16) / 800.0,
        }
    }
}

/// Convert a raw sample to microtesla and degrees Celsius. Order of operations from the vendor
/// `bmm350.c`, `bmm350_get_compensated_mag_xyz_temp_data`: scale the counts, correct the
/// temperature, then per axis apply sensitivity, offset, TCO and TCS, then the cross-axis
/// correction. The `BMM350_SENS_CORR_Y` and `BMM350_TCS_CORR_Z` post-solder macros of
/// `bmm350_defs.h` are defined there but not applied by that file, and are not applied here.
/// Values of disabled axes are meaningless.
pub fn compensate(cal: &Calibration, raw: &RawSample) -> Field {
    let temp = (raw.temperature as f32 * TEMP_DEG_PER_LSB - TEMP_ZERO_DEG)
        * (1.0 + cal.temp_sensitivity)
        + cal.temp_offset;
    let dt = temp - cal.t0;
    let axis = |counts: f32, sensitivity: f32, offset: f32, tco: f32, tcs: f32| {
        let v = counts * (1.0 + sensitivity) + offset + tco * dt;
        v / (1.0 + tcs * dt)
    };
    let b = [
        axis(
            raw.x as f32 * XY_UT_PER_LSB,
            cal.sensitivity[0],
            cal.offset[0],
            cal.tco[0],
            cal.tcs[0],
        ),
        axis(
            raw.y as f32 * XY_UT_PER_LSB,
            cal.sensitivity[1],
            cal.offset[1],
            cal.tco[1],
            cal.tcs[1],
        ),
        axis(
            raw.z as f32 * Z_UT_PER_LSB,
            cal.sensitivity[2],
            cal.offset[2],
            cal.tco[2],
            cal.tcs[2],
        ),
    ];
    let d = 1.0 - cal.cross_y_x * cal.cross_x_y;
    Field {
        x: (b[0] - cal.cross_x_y * b[1]) / d,
        y: (b[1] - cal.cross_y_x * b[0]) / d,
        z: b[2]
            + (b[0] * (cal.cross_y_x * cal.cross_z_y - cal.cross_z_x)
                - b[1] * (cal.cross_z_y - cal.cross_x_y * cal.cross_z_x))
                / d,
        temperature: temp,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_handles_negative_and_extremes() {
        let b = [
            0x01, 0x00, 0x00, // x = 1
            0xFF, 0xFF, 0xFF, // y = -1
            0x00, 0x00, 0x80, // z = -8388608
            0xFF, 0xFF, 0x7F, // t = 8388607
        ];
        let r = RawSample::decode(&b);
        assert_eq!(
            r,
            RawSample {
                x: 1,
                y: -1,
                z: -8_388_608,
                temperature: 8_388_607
            }
        );
        let b = [
            0x56, 0x34, 0x12, 0x00, 0xFF, 0xFE, 0x10, 0x00, 0x00, 0, 0, 0,
        ];
        let r = RawSample::decode(&b);
        assert_eq!((r.x, r.y, r.z), (0x123456, -0x010100, 0x10));
    }

    #[test]
    fn scale_constants_match_vendor_fixed_point_constants() {
        // The vendor fixed-point build lists 0.007069979, 0.007174964 and 0.000981282
        // (bmm350_defs.h, A48_16_0_007069979_X4 and neighbours).
        assert!((XY_UT_PER_LSB - 0.007_069_979).abs() < 1e-8);
        assert!((Z_UT_PER_LSB - 0.007_174_964).abs() < 1e-8);
        assert!((TEMP_DEG_PER_LSB - 0.000_981_282).abs() < 1e-8);
    }

    /// Words 0x0D-0x18 packed by `comp.py` (scratch script, not in the repo) from these known
    /// coefficients: temp offset 10, temp sens -20, offsets 100 -50 30, sens 5 -3 10, TCO 16 -32 8,
    /// TCS -10 20 -5, T0 word 256, cross X-Y 8, Y-X -4, Z-X 16, Z-Y -8.
    fn example_otp() -> [u16; OTP_WORDS] {
        let mut w = [0u16; OTP_WORDS];
        for (i, v) in [
            (0x0D, 0xEC0A),
            (0x0E, 0xF064),
            (0x0F, 0x00CE),
            (0x10, 0x051E),
            (0x11, 0x0AFD),
            (0x12, 0xF610),
            (0x13, 0x14E0),
            (0x14, 0xFB08),
            (0x15, 0xFC08),
            (0x16, 0xF810),
            (0x18, 0x0100),
        ] {
            w[i] = v;
        }
        w
    }

    fn close(a: f32, b: f32) {
        assert!((a - b).abs() < 1e-5 * b.abs().max(1.0), "{a} vs {b}");
    }

    #[test]
    fn calibration_parses_example_words() {
        let c = Calibration::from_otp(&example_otp());
        assert_eq!(c.offset, [100.0, -50.0, 30.0]);
        assert_eq!(c.sensitivity, [5.0 / 256.0, -3.0 / 256.0, 10.0 / 256.0]);
        assert_eq!(c.tco, [0.5, -1.0, 0.25]);
        assert_eq!(c.tcs, [-10.0 / 16384.0, 20.0 / 16384.0, -5.0 / 16384.0]);
        assert_eq!(c.temp_offset, 2.0);
        assert_eq!(c.temp_sensitivity, -20.0 / 512.0);
        assert_eq!(c.t0, 23.5);
        assert_eq!((c.cross_x_y, c.cross_y_x), (0.01, -0.005));
        assert_eq!((c.cross_z_x, c.cross_z_y), (0.02, -0.01));
        let z = Calibration::default();
        assert_eq!((z.offset, z.t0, z.cross_z_y), ([0.0; 3], 23.0, 0.0));
    }

    #[test]
    fn compensate_matches_independent_float64_evaluation() {
        // Expected values: `comp.py` evaluated the vendor formula in float64 directly from the
        // coefficient values listed above (no OTP unpacking involved) for raw
        // x=5000 y=-3000 z=6000 t=51450 and printed T=26.020507896, X=138.237776206,
        // Y=-72.565306187, Z=71.929147657. Bosch publishes no example values.
        let c = Calibration::from_otp(&example_otp());
        let f = compensate(
            &c,
            &RawSample {
                x: 5000,
                y: -3000,
                z: 6000,
                temperature: 51450,
            },
        );
        close(f.temperature, 26.020_508);
        close(f.x, 138.237_78);
        close(f.y, -72.565_31);
        close(f.z, 71.929_15);
    }

    #[test]
    fn zero_calibration_is_pure_scaling() {
        let f = compensate(
            &Calibration::default(),
            &RawSample {
                x: 1000,
                y: 0,
                z: -1000,
                temperature: 0,
            },
        );
        close(f.x, 1000.0 * XY_UT_PER_LSB);
        close(f.z, -1000.0 * Z_UT_PER_LSB);
        close(f.temperature, -TEMP_ZERO_DEG);
    }
}
