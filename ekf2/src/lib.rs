//! Safe idiomatic Rust API for the PX4 EKF2 navigation filter.
//!
//! # Quick start
//!
//! ```no_run
//! use ekf2::{Ekf, types::ImuSample};
//!
//! let mut ekf = Ekf::new().expect("EKF allocation failed");
//!
//! // Feed IMU data at 100 Hz
//! for i in 0..1000_u64 {
//!     let ts_us = i * 10_000; // 10 ms → 100 Hz
//!     let sample = ImuSample::new(
//!         ts_us,
//!         [0.0, 0.0, 0.01],       // tiny yaw rate
//!         [0.0, 0.0, -9.81*0.01], // gravity
//!         0.01,
//!         0.01,
//!     );
//!     ekf.set_imu_data(&sample);
//!     let _ = ekf.update();
//! }
//!
//! if ekf.attitude_valid() {
//!     let q = ekf.quaternion();
//!     println!("q = [{:.4}, {:.4}, {:.4}, {:.4}]", q[0], q[1], q[2], q[3]);
//! }
//! ```
//!
//! # Feature flags
//!
//! Each sensor type is guarded by a Cargo feature (defaults: `gnss`,
//! `magnetometer`, `barometer`). See
//! [`ekf2-sys`](https://docs.rs/ekf2-sys) for the full table.
//!
//! # Memory model
//!
//! [`Ekf::new`] creates the heap-owned C++ object using Rust's global
//! allocator. [`Ekf::with_config`] selects the initial timing configuration.
//! [`Ekf::new_in`] and [`Ekf::with_config_in`] accept any `allocator-api2`
//! allocator. The selected allocator is passed to C++ as per-instance callbacks
//! and is used for the object and all of its owned dynamic storage.
//!
//! # no_std
//!
//! This crate does not depend on `std`. [`Ekf::new`] uses the target's global
//! allocator, while [`Ekf::new_in`] accepts a custom allocator.

#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]

mod allocator;
pub mod ekf;
pub mod error;
pub mod params;
pub mod types;

pub use ekf::{Ekf, EkfConfig};
pub use error::EkfError;
pub use params::{
    BaroControl, ControlStatus, ExternalVisionControl, FaultStatus, FlowGyroSource,
    GnssCheckControl, GnssCheckFailStatus, GnssControl, GnssFixType, GnssMode, HeightReference,
    ImuControl, InformationEventStatus, MagCheckControl, MagDeclinationControl, MagFusionType,
    OpticalFlowControl, PositionReference, RangeControl, SolnStatus,
};

pub use ekf2_sys::EkfBiasEstimatorStatus as BiasEstimatorStatus;
pub use ekf2_sys::{
    EkfAidSource1d as AidSource1d, EkfAidSource2d as AidSource2d, EkfAidSource3d as AidSource3d,
};
pub use ekf2_sys::{GlobalOrigin, HeightSensor};
pub use types::{PositionFrame, VelocityFrame};
