//! Sensor sample types.
//!
//! Each type is `#[repr(C)]` with a layout identical to the corresponding
//! C struct in `ekf2_wrapper.h` (validated by `static_assert` in the C++
//! translation unit).
//!
//! All types implement `Copy` and `Default` for ergonomic initialisation.

use ekf2_sys as ffi;

// ── Frame enums ───────────────────────────────────────────────────────────

pub use ffi::PositionFrame;
pub use ffi::VelocityFrame;

// ── IMU (always required) ─────────────────────────────────────────────────

pub use ffi::EkfImuSample as ImuSample;

// ── GNSS ──────────────────────────────────────────────────────────────────

#[cfg(feature = "gnss")]
pub use ffi::EkfGnssSample as GnssSample;

// ── Magnetometer ─────────────────────────────────────────────────────────

#[cfg(feature = "magnetometer")]
pub use ffi::EkfMagSample as MagSample;

// ── Barometer ─────────────────────────────────────────────────────────────

#[cfg(feature = "barometer")]
pub use ffi::EkfBaroSample as BaroSample;

// ── Airspeed ──────────────────────────────────────────────────────────────

#[cfg(feature = "airspeed")]
pub use ffi::EkfAirspeedSample as AirspeedSample;

// ── Range finder ──────────────────────────────────────────────────────────

#[cfg(feature = "range-finder")]
pub use ffi::EkfRangeSample as RangeSample;

// ── Optical flow ──────────────────────────────────────────────────────────

#[cfg(feature = "optical-flow")]
pub use ffi::EkfFlowSample as FlowSample;

// ── External vision ───────────────────────────────────────────────────────

#[cfg(feature = "external-vision")]
pub use ffi::EkfExtVisionSample as ExtVisionSample;

// ── Auxiliary velocity ────────────────────────────────────────────────────

#[cfg(feature = "aux-vel")]
pub use ffi::EkfAuxVelSample as AuxVelSample;

// ── System flags ──────────────────────────────────────────────────────────

pub use ffi::EkfSystemFlagUpdate as SystemFlagUpdate;
