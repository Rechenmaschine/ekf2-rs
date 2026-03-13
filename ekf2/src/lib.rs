//! Safe idiomatic Rust API for the PX4 EKF2 navigation filter.
//!
//! # Quick start
//!
//! ```no_run
//! use ekf2::{Ekf, types::ImuSample};
//!
//! let mut ekf = Ekf::new(0).expect("EKF init failed");
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
//!     ekf.update().unwrap();
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
//! [`Ekf::new`] allocates the C++ object through `ekf2-sys` and calls
//! `Ekf::init()`. C++ allocations are routed through Rust allocator symbols
//! via the `operator new`/`delete` bridge in `allocator.cpp`. On bare-metal
//! targets the global allocator must be configured (e.g. via `embedded-alloc`).
//!
//! # no_std
//!
//! This crate is `no_std` compatible but requires the `alloc` crate.

#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]

extern crate alloc;

pub mod ekf;
pub mod error;
pub mod params;
pub mod types;

pub use ekf::Ekf;
pub use error::EkfError;
pub use params::{
    BaroControl, ControlStatus, ExternalVisionControl, FaultStatus, FlowGyroSource,
    GnssCheckControl, GnssCheckFailStatus, GnssControl, GnssFixType, GnssMode, HeightReference,
    ImuControl, InformationEventStatus, MagCheckControl, MagDeclinationControl, MagFusionType,
    OpticalFlowControl, PositionReference, RangeControl, SolnStatus,
};

// Output types returned by EKF methods — re-exported so users don't need to depend on ekf2-sys.
pub use ekf2_sys::{
    EkfAidSource1d as AidSource1d, EkfAidSource2d as AidSource2d, EkfAidSource3d as AidSource3d,
};
pub use ekf2_sys::EkfBiasEstimatorStatus as BiasEstimatorStatus;
pub use ekf2_sys::{GlobalOrigin, HeightSensor};
pub use types::{PositionFrame, VelocityFrame};
