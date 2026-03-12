# Third-Party Notices

## PX4 dependency

This project depends on PX4 source code via git submodule:

- Path: `ekf2-sys/vendor/PX4-Autopilot`
- Upstream: <https://github.com/PX4/PX4-Autopilot>
- Upstream license: BSD-3-Clause
- Upstream license file: `ekf2-sys/vendor/PX4-Autopilot/LICENSE`

`ekf2-sys` compiles only a subset of PX4 (EKF2 + supporting libraries) as
configured in `ekf2-sys/build.rs`.

## PX4-derived files in this repository

The following files are derived from PX4 message definitions and should be
treated as BSD-3-Clause derived material from PX4:

- `ekf2-sys/wrapper/uORB/topics/estimator_aid_source1d.h`
- `ekf2-sys/wrapper/uORB/topics/estimator_aid_source2d.h`
- `ekf2-sys/wrapper/uORB/topics/estimator_aid_source3d.h`

If you redistribute additional content from the PX4 submodule beyond the files
used by this crate, keep the corresponding upstream license files and notices.
