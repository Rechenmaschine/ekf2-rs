# ekf2-rs

Rust `#![no_std]` bindings for the PX4 EKF2 state-estimation algorithm.

This workspace has two crates:

- `ekf2-sys`: raw `unsafe` FFI bindings to vendored PX4 EKF2 C++
- `ekf2`: safe Rust wrapper API

This crate requires `alloc` due to the design of the underlying C++ code.

## Minimal usage

```rust
use ekf2::{Ekf, EkfError, types::ImuSample};

fn main() -> Result<(), EkfError> {
    let mut ekf = Ekf::new(0)?;
    let dt_s = 0.01_f32;
    let imu = ImuSample::new(
        10_000,                   // time_us
        [0.0, 0.0, 0.0002],       // delta_ang_rad
        [0.0, 0.0, -9.81 * dt_s], // delta_vel_m_s
        dt_s,                     // delta_ang_dt_s
        dt_s,                     // delta_vel_dt_s
    );
    ekf.set_imu_data(&imu);
    let _ = ekf.update();
    let _quat = ekf.quaternion(); // quat_wxyz
    Ok(())
}
```

## Supported Features

`ekf2-rs` in the long term aims to provide full coverage of the PX4 EKF2 algorithm.

Current coverage includes:

- Runtime + inputs: standalone `no_std + alloc` EKF instance with `new()`/`update()`, IMU always-on, and feature-gated
  sensor feeds (GNSS, mag, baro, airspeed, range, flow, EV, aux-vel).
- Outputs + diagnostics: attitude/position/velocity, biases/variances, innovation and aid-source diagnostics,
  validity/solution/fault/control status.
- Config + control: typed `params()` / `params_mut()` access and runtime reset/control helpers (global origin,
  heading/wind, bias, WMM updates).

## Feature Flags

`ekf2-sys` maps C++ compilation features to Rust features. Enabling a feature compiles the corresponding C++ code and
exposes related API in Rust.

| Rust feature        | Default | PX4 compilation term            |
|---------------------|---------|---------------------------------|
| `gnss`              | yes     | `CONFIG_EKF2_GNSS`              |
| `magnetometer`      | yes     | `CONFIG_EKF2_MAGNETOMETER`      |
| `barometer`         | yes     | `CONFIG_EKF2_BAROMETER`         |
| `gnss-yaw`          | no      | `CONFIG_EKF2_GNSS_YAW`          |
| `baro-compensation` | no      | `CONFIG_EKF2_BARO_COMPENSATION` |
| `airspeed`          | no      | `CONFIG_EKF2_AIRSPEED`          |
| `range-finder`      | no      | `CONFIG_EKF2_RANGE_FINDER`      |
| `optical-flow`      | no      | `CONFIG_EKF2_OPTICAL_FLOW`      |
| `external-vision`   | no      | `CONFIG_EKF2_EXTERNAL_VISION`   |
| `aux-vel`           | no      | `CONFIG_EKF2_AUXVEL`            |
| `drag-fusion`       | no      | `CONFIG_EKF2_DRAG_FUSION`       |
| `wind`              | no      | `CONFIG_EKF2_WIND`              |
| `terrain`           | no      | `CONFIG_EKF2_TERRAIN`           |
| `gravity-fusion`    | no      | `CONFIG_EKF2_GRAVITY_FUSION`    |
| `sideslip`          | no      | `CONFIG_EKF2_SIDESLIP`          |
| `logging`           | no      | -                               |
| `c-stubs`           | no      | -                               |

Note: `range-finder` and `optical-flow` implicitly enable `CONFIG_EKF2_TERRAIN` in `ekf2-sys/build.rs`.

## Verified targets

The crate is verified in CI to compile for the following targets:

- `x86_64-unknown-linux-gnu`
- `thumbv7em-none-eabihf`
- `thumbv6m-none-eabi`

Further targets should also work, but are untested.

## License

This repository is licensed under the BSD 3-Clause license.
See [`LICENSE`](./LICENSE).

It also depends on vendored third-party code (PX4 and components inside the
PX4 source tree). See [`THIRD_PARTY_NOTICES.md`](./THIRD_PARTY_NOTICES.md) for
license attributions and paths.
