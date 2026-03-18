# ekf2

Safe idiomatic Rust API for the PX4 EKF2 navigation filter.

This crate wraps [`ekf2-sys`](../ekf2-sys) — the raw FFI layer — and exposes a
clean, type-safe interface. It is `#![no_std]` compatible and requires `alloc`.

## Quick start

```rust
use ekf2::{Ekf, types::ImuSample};

let mut ekf = Ekf::new(0).expect("EKF init failed");

for i in 0..1000_u64 {
    let ts_us = i * 10_000; // 10 ms → 100 Hz
    let sample = ImuSample::new(
        ts_us,
        [0.0, 0.0, 0.01],       // tiny yaw rate (rad)
        [0.0, 0.0, -9.81*0.01], // gravity (m/s)
        0.01,                   // delta_ang_dt_s
        0.01,                   // delta_vel_dt_s
    );
    ekf.set_imu_data(&sample);
    ekf.update().unwrap();
}

if ekf.attitude_valid() {
    let q = ekf.quaternion(); // [w, x, y, z]
}
```

## API overview

### `Ekf` — the filter instance

`Ekf::new(timestamp_us)` allocates and initialises the C++ object through the
`ekf2-sys` FFI layer. The object is heap-allocated and owned by the `Ekf` struct;
the destructor calls `ekf2_destroy_heap` which runs the C++ destructor and frees
the memory.

Key methods:

| Method | Description |
|---|---|
| `update()` | Run one filter step; returns `Err(UpdateFailed)` on divergence |
| `set_imu_data(&ImuSample)` | Push an IMU sample (always required) |
| `set_gps_data(&GnssSample)` | Push a GNSS fix (feature `gnss`) |
| `set_mag_data(&MagSample)` | Push a magnetometer sample (feature `magnetometer`) |
| `set_baro_data(&BaroSample)` | Push a barometer sample (feature `barometer`) |
| … | Other feature-gated sensor setters |
| `quaternion()` | Attitude quaternion `[w, x, y, z]` |
| `velocity_ned()` | NED velocity `[vn, ve, vd]` m/s |
| `position_ned()` | NED position `[n, e, d]` m (relative to origin) |
| `params()` / `params_mut()` | Typed access to EKF tuning parameters |

### `params()` / `params_mut()`

Returns a `Params` / `ParamsMut` view that borrows the live `Ekf`. All
parameter fields map directly to PX4's `EKF2_*` parameters. Changes take effect
on the next `update()` call.

### `types` module

Re-exports sensor sample types and frame enums:

- `ImuSample`, `GnssSample`, `MagSample`, `BaroSample`, `AirspeedSample`,
  `RangeSample`, `FlowSample`, `ExtVisionSample`, `AuxVelSample`,
  `SystemFlagUpdate` (each behind the corresponding feature flag)
- `PositionFrame`, `VelocityFrame`

### Output and diagnostic types

The following are re-exported at the crate root:

- `AidSource1d`, `AidSource2d`, `AidSource3d` — per-sensor innovation diagnostics
- `BiasEstimatorStatus` — gyro/accel bias estimator state
- `HeightSensor` — active height reference (baro / GNSS / range / EV)
- `GlobalOrigin` — WGS-84 origin for local↔global conversion

## Memory model

`Ekf::new` allocates the C++ `Ekf` object on the heap via `ekf2_create_heap`.
C++ `operator new` / `delete` are bridged to Rust's global allocator by
`allocator.cpp` in `ekf2-sys`. On bare-metal the application must provide a
global allocator (e.g. `embedded-alloc`).

See [`ekf2-sys/README.md`](../ekf2-sys/README.md) for a detailed description of
the allocator bridge and the C++ shim layer.

## Feature flags

Sensor feeds are guarded by Cargo features. Defaults: `gnss`, `magnetometer`,
`barometer`. See the [feature table in ekf2-sys](../ekf2-sys/README.md#feature-flags)
for the full list and the corresponding PX4 `CONFIG_EKF2_*` defines.

## no_std

`#![no_std]` + `alloc`. No `std` features are used; `Ekf` is `Send` — it can
be moved between threads (the underlying C++ object has no thread-local state
after initialisation).
