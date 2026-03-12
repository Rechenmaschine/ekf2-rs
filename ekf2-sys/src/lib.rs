//! Raw FFI bindings for the PX4 EKF2 C++ library.
//!
//! This crate is a low-level `unsafe` interface. Most users should use the
//! safe [`ekf2`] crate instead.
//!
//! # Feature flags
//!
//! Each sensor type is guarded by a Cargo feature that maps 1-to-1 to the
//! corresponding `CONFIG_EKF2_*` C preprocessor define:
//!
//! | Feature            | Default | C define                    |
//! |--------------------|:-------:|-----------------------------|
//! | `gnss`             | yes     | `CONFIG_EKF2_GNSS`          |
//! | `gnss-yaw`         | no      | `CONFIG_EKF2_GNSS_YAW`      |
//! | `magnetometer`     | yes     | `CONFIG_EKF2_MAGNETOMETER`  |
//! | `barometer`        | yes     | `CONFIG_EKF2_BAROMETER`     |
//! | `baro-compensation`| no      | `CONFIG_EKF2_BARO_COMPENSATION` |
//! | `airspeed`         | no      | `CONFIG_EKF2_AIRSPEED`      |
//! | `range-finder`     | no      | `CONFIG_EKF2_RANGE_FINDER`  |
//! | `optical-flow`     | no      | `CONFIG_EKF2_OPTICAL_FLOW`  |
//! | `external-vision`  | no      | `CONFIG_EKF2_EXTERNAL_VISION` |
//! | `aux-vel`          | no      | `CONFIG_EKF2_AUXVEL`        |
//! | `drag-fusion`      | no      | `CONFIG_EKF2_DRAG_FUSION`   |
//! | `wind`             | no      | `CONFIG_EKF2_WIND`          |
//! | `terrain`          | no      | `CONFIG_EKF2_TERRAIN`       |
//! | `gravity-fusion`   | no      | `CONFIG_EKF2_GRAVITY_FUSION`|
//! | `sideslip`         | no      | `CONFIG_EKF2_SIDESLIP`      |

#![no_std]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(clippy::all)]

extern crate alloc;

use alloc::alloc::{
    alloc as rust_alloc, alloc_zeroed as rust_alloc_zeroed, dealloc as rust_dealloc,
};
use core::alloc::Layout;

// Generated bindings from ekf2_wrapper.h
include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

#[cfg(all(target_os = "none", feature = "c-stubs"))]
mod stubs;

/// C ABI allocator shim used by C++ glue code.
#[no_mangle]
pub unsafe extern "C" fn ekf2_rust_alloc(size: usize, align: usize) -> *mut u8 {
    match Layout::from_size_align(size, align) {
        Ok(layout) => unsafe { rust_alloc(layout) },
        Err(_) => core::ptr::null_mut(),
    }
}

/// C ABI zeroed allocator shim used by C++ glue code.
#[no_mangle]
pub unsafe extern "C" fn ekf2_rust_alloc_zeroed(size: usize, align: usize) -> *mut u8 {
    match Layout::from_size_align(size, align) {
        Ok(layout) => unsafe { rust_alloc_zeroed(layout) },
        Err(_) => core::ptr::null_mut(),
    }
}

/// C ABI deallocator shim used by C++ glue code.
#[no_mangle]
pub unsafe extern "C" fn ekf2_rust_dealloc(ptr: *mut u8, size: usize, align: usize) {
    if ptr.is_null() {
        return;
    }
    if let Ok(layout) = Layout::from_size_align(size, align) {
        unsafe { rust_dealloc(ptr, layout) };
    }
}

/// Active primary height sensor reference.
///
/// Matches PX4's `HeightSensor` enum (`common.h`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeightSensor {
    Baro = 0,
    Gnss = 1,
    Range = 2,
    Ev = 3,
    Unknown = 4,
}

impl HeightSensor {
    /// Convert from the raw `u8` returned by `ekf2_get_height_sensor_ref`.
    #[inline]
    pub fn from_raw(v: u8) -> Self {
        match v {
            0 => Self::Baro,
            1 => Self::Gnss,
            2 => Self::Range,
            3 => Self::Ev,
            _ => Self::Unknown,
        }
    }
}

/// WGS-84 origin used by the EKF for local↔global position conversion.
#[derive(Debug, Clone, Copy, Default)]
pub struct GlobalOrigin {
    /// Timestamp when the origin was set [µs].
    pub time_us: u64,
    /// Latitude [deg].
    pub lat: f64,
    /// Longitude [deg].
    pub lon: f64,
    /// Altitude above WGS-84 ellipsoid [m].
    pub alt: f32,
}

/// Position reference frame for external vision samples.
///
/// Matches PX4's `PositionFrame` enum (`common.h`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositionFrame {
    /// NED (North-East-Down) local frame.
    LocalFrameNed = 0,
    /// FRD (Forward-Right-Down) local frame.
    LocalFrameFrd = 1,
}

/// Velocity reference frame for external vision samples.
///
/// Matches PX4's `VelocityFrame` enum (`common.h`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VelocityFrame {
    /// NED (North-East-Down) local frame.
    LocalFrameNed = 0,
    /// FRD (Forward-Right-Down) local frame.
    LocalFrameFrd = 1,
    /// Body FRD frame.
    BodyFrameFrd = 2,
}

impl EkfImuSample {
    /// Construct an IMU sample from delta-angle and delta-velocity values.
    #[inline]
    pub const fn new(
        time_us: u64,
        delta_ang: [f32; 3],
        delta_vel: [f32; 3],
        delta_ang_dt: f32,
        delta_vel_dt: f32,
    ) -> Self {
        Self {
            time_us,
            delta_ang,
            delta_vel,
            delta_ang_dt,
            delta_vel_dt,
            delta_vel_clipping: [0; 3],
            _pad: [0; 5],
        }
    }

    /// Construct an IMU sample with explicit clipping flags.
    #[inline]
    pub const fn with_clipping(
        time_us: u64,
        delta_ang: [f32; 3],
        delta_vel: [f32; 3],
        delta_ang_dt: f32,
        delta_vel_dt: f32,
        delta_vel_clipping: [bool; 3],
    ) -> Self {
        Self {
            time_us,
            delta_ang,
            delta_vel,
            delta_ang_dt,
            delta_vel_dt,
            delta_vel_clipping: [
                delta_vel_clipping[0] as u8,
                delta_vel_clipping[1] as u8,
                delta_vel_clipping[2] as u8,
            ],
            _pad: [0; 5],
        }
    }
}

#[cfg(feature = "gnss")]
impl EkfGnssSample {
    /// Construct a GNSS sample.
    #[inline]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        time_us: u64,
        lat: f64,
        lon: f64,
        alt: f32,
        vel: [f32; 3],
        hacc: f32,
        vacc: f32,
        sacc: f32,
        fix_type: u8,
        nsats: u8,
    ) -> Self {
        Self {
            time_us,
            lat,
            lon,
            alt,
            yaw: f32::NAN,
            yaw_acc: f32::NAN,
            yaw_offset: 0.0,
            vel,
            hacc,
            vacc,
            sacc,
            pdop: 0.0,
            fix_type,
            nsats,
            spoofed: 0,
            jammed: 0,
            _pad: [0; 8],
        }
    }
}

#[cfg(feature = "magnetometer")]
impl EkfMagSample {
    /// Construct a magnetometer sample.
    #[inline]
    pub const fn new(time_us: u64, mag: [f32; 3]) -> Self {
        Self {
            time_us,
            mag,
            reset: 0,
            _pad: [0; 3],
        }
    }

    /// Mark this sample as a sensor-replacement or calibration-change event.
    ///
    /// Set when the magnetometer source changed mid-flight so the EKF can
    /// reinitialise its magnetometer states.
    #[inline]
    pub const fn with_reset(mut self) -> Self {
        self.reset = 1;
        self
    }
}

#[cfg(feature = "barometer")]
impl EkfBaroSample {
    /// Construct a barometer sample.
    #[inline]
    pub const fn new(time_us: u64, hgt: f32) -> Self {
        Self {
            time_us,
            hgt,
            reset: 0,
            _pad: [0; 3],
        }
    }

    /// Mark this sample as a sensor-replacement or calibration-change event.
    ///
    /// Set when the barometer source changed mid-flight so the EKF can
    /// reinitialise its barometer states.
    #[inline]
    pub const fn with_reset(mut self) -> Self {
        self.reset = 1;
        self
    }
}

#[cfg(feature = "airspeed")]
impl EkfAirspeedSample {
    /// Construct an airspeed sample.
    #[inline]
    pub const fn new(time_us: u64, true_airspeed: f32, eas2tas: f32) -> Self {
        Self {
            time_us,
            true_airspeed,
            eas2tas,
        }
    }
}

#[cfg(feature = "range-finder")]
impl EkfRangeSample {
    /// Construct a range-finder sample.
    ///
    /// `quality` is 0–100 (100 = perfect signal, 0 = invalid, -1 = unknown).
    #[inline]
    pub const fn new(time_us: u64, rng: f32, quality: i8) -> Self {
        Self {
            time_us,
            rng,
            quality,
            _pad: [0; 3],
        }
    }
}

#[cfg(feature = "optical-flow")]
impl EkfFlowSample {
    /// Construct an optical-flow sample.
    ///
    /// `flow_rate_xy` and `gyro_rate_xyz` must be rates in rad/s (not integrated values).
    #[inline]
    pub const fn new(
        time_us: u64,
        flow_rate_xy: [f32; 2],
        gyro_rate_xyz: [f32; 3],
        quality: u8,
    ) -> Self {
        Self {
            time_us,
            flow_rate_xy,
            gyro_rate_xyz,
            quality,
            _pad: [0; 3],
        }
    }
}

#[cfg(feature = "external-vision")]
impl EkfExtVisionSample {
    /// Construct an external-vision sample.
    ///
    /// Defaults: `pos_frame = LocalFrameNed`, `vel_frame = LocalFrameNed`,
    /// `reset_counter = 0`, `quality = 100` (perfect).
    ///
    /// `ang_var` is per-axis attitude variance [rad²] for X, Y, Z.
    /// Use `[v; 3]` to broadcast a single scalar variance to all axes.
    #[inline]
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        time_us: u64,
        pos: [f32; 3],
        quat: [f32; 4],
        vel: [f32; 3],
        pos_var: [f32; 3],
        ang_var: [f32; 3],
        vel_var: [f32; 3],
    ) -> Self {
        Self {
            time_us,
            pos,
            quat,
            vel,
            pos_var,
            ang_var,
            vel_var,
            pos_frame: PositionFrame::LocalFrameNed as u8,
            vel_frame: VelocityFrame::LocalFrameNed as u8,
            reset_counter: 0,
            quality: 100,
        }
    }

    /// Override the position and velocity reference frames.
    #[inline]
    pub const fn with_frames(mut self, pos_frame: PositionFrame, vel_frame: VelocityFrame) -> Self {
        self.pos_frame = pos_frame as u8;
        self.vel_frame = vel_frame as u8;
        self
    }

    /// Override the quality indicator (0–100; 100 = perfect, 0 = invalid).
    #[inline]
    pub const fn with_quality(mut self, quality: i8) -> Self {
        self.quality = quality;
        self
    }

    /// Increment the reset counter to signal a tracker discontinuity.
    ///
    /// The EKF uses this to detect when the external-vision pose estimate
    /// has jumped (e.g. loop-closure, tracking re-initialisation).
    #[inline]
    pub const fn with_reset_counter(mut self, reset_counter: u8) -> Self {
        self.reset_counter = reset_counter;
        self
    }
}

#[cfg(feature = "aux-vel")]
impl EkfAuxVelSample {
    /// Construct an auxiliary velocity sample.
    #[inline]
    pub const fn new(time_us: u64, vel: [f32; 2], vel_var: [f32; 2]) -> Self {
        Self {
            time_us,
            vel,
            vel_var,
        }
    }
}

impl EkfSystemFlagUpdate {
    /// Construct a system-flag update sample.
    ///
    /// Defaults:
    /// - `in_air = true`
    /// - all other flags = `false`
    #[inline]
    pub const fn new(time_us: u64) -> Self {
        Self {
            time_us,
            at_rest: false,
            in_air: true,
            is_fixed_wing: false,
            gnd_effect: false,
            constant_pos: false,
            in_transition_to_fw: false,
            _pad: 0,
        }
    }

    /// Set all mode flags explicitly.
    #[inline]
    #[allow(clippy::too_many_arguments)]
    pub const fn with_flags(
        mut self,
        at_rest: bool,
        in_air: bool,
        is_fixed_wing: bool,
        gnd_effect: bool,
        constant_pos: bool,
        in_transition_to_fw: bool,
    ) -> Self {
        self.at_rest = at_rest;
        self.in_air = in_air;
        self.is_fixed_wing = is_fixed_wing;
        self.gnd_effect = gnd_effect;
        self.constant_pos = constant_pos;
        self.in_transition_to_fw = in_transition_to_fw;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::ffi::c_void;

    const PRE_GUARD: u8 = 0xA5;
    const POST_GUARD: u8 = 0x5A;

    #[repr(align(16))]
    struct AlignedObject<const N: usize>([u8; N]);

    #[repr(C)]
    struct Guarded<T> {
        pre: [u8; 32],
        inner: T,
        post: [u8; 32],
    }

    impl<T> Guarded<T> {
        fn new(inner: T) -> Self {
            Self {
                pre: [PRE_GUARD; 32],
                inner,
                post: [POST_GUARD; 32],
            }
        }

        fn assert_guards_intact(&self, label: &str) {
            assert!(
                self.pre.iter().all(|b| *b == PRE_GUARD),
                "pre-guard corrupted for {label}"
            );
            assert!(
                self.post.iter().all(|b| *b == POST_GUARD),
                "post-guard corrupted for {label}"
            );
        }
    }

    #[test]
    fn decode_helpers_and_sample_builders_work() {
        assert_eq!(HeightSensor::from_raw(0), HeightSensor::Baro);
        assert_eq!(HeightSensor::from_raw(1), HeightSensor::Gnss);
        assert_eq!(HeightSensor::from_raw(255), HeightSensor::Unknown);
        assert_eq!(PositionFrame::LocalFrameNed as u8, 0);
        assert_eq!(VelocityFrame::BodyFrameFrd as u8, 2);

        let imu = EkfImuSample::with_clipping(
            456,
            [0.1, 0.2, 0.3],
            [1.0, 2.0, 3.0],
            0.01,
            0.02,
            [true, false, true],
        );
        assert_eq!(imu.time_us, 456);
        assert_eq!(imu.delta_ang, [0.1, 0.2, 0.3]);
        assert_eq!(imu.delta_vel, [1.0, 2.0, 3.0]);
        assert_eq!(imu.delta_ang_dt, 0.01);
        assert_eq!(imu.delta_vel_dt, 0.02);
        assert_eq!(imu.delta_vel_clipping, [1, 0, 1]);
    }

    #[cfg(feature = "gnss")]
    #[test]
    fn gnss_builder_initializes_key_defaults() {
        let sample = EkfGnssSample::new(
            1000,
            47.0,
            8.0,
            500.0,
            [1.0, 2.0, 3.0],
            0.5,
            0.7,
            0.9,
            3,
            12,
        );
        assert_eq!(sample.time_us, 1000);
        assert_eq!(sample.lat, 47.0);
        assert_eq!(sample.lon, 8.0);
        assert_eq!(sample.alt, 500.0);
        assert_eq!(sample.vel, [1.0, 2.0, 3.0]);
        assert_eq!(sample.hacc, 0.5);
        assert_eq!(sample.vacc, 0.7);
        assert_eq!(sample.sacc, 0.9);
        assert_eq!(sample.fix_type, 3);
        assert_eq!(sample.nsats, 12);
        assert!(sample.yaw.is_nan());
        assert_eq!(sample.pdop, 0.0); // default helper value
    }

    #[test]
    fn raw_create_rejects_too_small_object_buffer() {
        let mut obj = AlignedObject::<65536>([0; 65536]);
        let required = unsafe { ekf2_sizeof() };
        assert!(required > 0);

        unsafe {
            let ptr = ekf2_create(obj.0.as_mut_ptr() as *mut c_void, required - 1);
            assert!(ptr.is_null());
        }
    }

    #[test]
    fn raw_lifecycle_respects_guard_bytes() {
        let mut obj = Guarded::new(AlignedObject::<65536>([0; 65536]));

        unsafe {
            let ptr = ekf2_create(obj.inner.0.as_mut_ptr() as *mut c_void, obj.inner.0.len());
            assert!(
                !ptr.is_null(),
                "create should succeed with large object buffer"
            );

            let ok = ekf2_init(ptr, 0);
            assert!(ok, "init should succeed");

            for i in 0..256_u64 {
                let ts = 1_000_000 + i * 10_000;
                let imu =
                    EkfImuSample::new(ts, [0.0, 0.0, 0.0001], [0.0, 0.0, -9.81 * 0.01], 0.01, 0.01);
                ekf2_set_imu_data(ptr, &imu);
                let _ = ekf2_update(ptr);
            }

            ekf2_destroy(ptr);
        }

        obj.assert_guards_intact("object");
    }
}
