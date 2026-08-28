# PX4 EKF source snapshot

This directory contains the minimal PX4 source slice compiled by `ekf2-sys`.
It is intentionally vendored as part of `ekf2-rs` so the downstream allocator
and ownership changes are reproducible without modifying or forking PX4.

- Upstream project: [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)
- Upstream commit: `3aa499dfcea0179af04c66291c83ce661593ad0c`
- Upstream license: BSD-3-Clause (see [`LICENSE`](LICENSE))
- Included source: `src/modules/ekf2/EKF` and the PX4 support libraries required
  by the extracted EKF build

The vendored EKF sources contain the downstream changes required by the Rust
FFI boundary, including allocator-aware buffer ownership.
