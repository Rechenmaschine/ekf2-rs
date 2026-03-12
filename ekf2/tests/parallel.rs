use ekf2::{types::ImuSample, Ekf, EkfError};
use std::sync::{Arc, Barrier};
use std::thread;

#[cfg(feature = "barometer")]
use ekf2::types::BaroSample;
#[cfg(feature = "gnss")]
use ekf2::types::GnssSample;
#[cfg(feature = "magnetometer")]
use ekf2::types::MagSample;

mod common;
use common::{abs_max_diff, assert_snapshot_sane, snapshot};

// ── High-contention sensor churn ─────────────────────────────────────────────

#[test]
fn high_contention_sensor_churn_is_stable() {
    const THREADS: usize = 8;
    const STEPS: u64 = 1_200;

    let start = Arc::new(Barrier::new(THREADS));
    let mut handles = Vec::with_capacity(THREADS);

    for thread_idx in 0..THREADS {
        let start = Arc::clone(&start);
        handles.push(thread::spawn(move || {
            let mut ekf = Ekf::new(0).expect("thread-local EKF init should succeed");
            let mut update_failed = 0_u32;
            let mut reset_count_regressions = 0_u32;
            let mut prev_counts = (
                ekf.quat_reset_count(),
                ekf.pos_ne_reset_count(),
                ekf.vel_ne_reset_count(),
                ekf.pos_d_reset_count(),
                ekf.vel_d_reset_count(),
            );

            start.wait();

            for i in 0..STEPS {
                let ts = 10_000_000 + (thread_idx as u64) * 1_000_000 + i * 10_000;
                let imu = ImuSample::new(
                    ts,
                    [0.0, 0.0, 0.00015 + (thread_idx as f32) * 0.00002],
                    [0.0, 0.0, -0.0981],
                    0.01,
                    0.01,
                );
                ekf.set_imu_data(&imu);
                if let Err(EkfError::UpdateFailed) = ekf.update() {
                    update_failed += 1;
                }

                #[cfg(feature = "barometer")]
                if i % 8 == 0 {
                    ekf.set_baro_data(&BaroSample::new(ts, 488.0 + thread_idx as f32 * 0.05));
                }
                #[cfg(feature = "magnetometer")]
                if i % 10 == 0 {
                    ekf.set_mag_data(&MagSample::new(
                        ts,
                        [0.19 + thread_idx as f32 * 0.01, 0.01, 0.44],
                    ));
                }
                #[cfg(feature = "gnss")]
                if i % 12 == 0 {
                    ekf.set_gps_data(&GnssSample::new(
                        ts,
                        47.397742 + thread_idx as f64 * 1e-6,
                        8.545594 + thread_idx as f64 * 1e-6,
                        488.0,
                        [0.1, 0.0, 0.0],
                        0.6,
                        0.9,
                        0.3,
                        3,
                        14,
                    ));
                }

                if i % 16 == 0 {
                    let _ = ekf.quaternion();
                    let _ = ekf.velocity_ned();
                    let _ = ekf.position_ned();
                    let now_counts = (
                        ekf.quat_reset_count(),
                        ekf.pos_ne_reset_count(),
                        ekf.vel_ne_reset_count(),
                        ekf.pos_d_reset_count(),
                        ekf.vel_d_reset_count(),
                    );
                    if now_counts.0 < prev_counts.0
                        || now_counts.1 < prev_counts.1
                        || now_counts.2 < prev_counts.2
                        || now_counts.3 < prev_counts.3
                        || now_counts.4 < prev_counts.4
                    {
                        reset_count_regressions += 1;
                    }
                    prev_counts = now_counts;
                }
                if i % 20 == 0 {
                    thread::yield_now();
                }
            }

            (update_failed, reset_count_regressions, snapshot(&ekf))
        }));
    }

    for handle in handles {
        let (update_failed, reset_count_regressions, snap) = handle
            .join()
            .expect("high-contention worker thread should not panic");
        assert!(
            update_failed < 240,
            "too many update failures in high-contention churn: {update_failed}"
        );
        assert_eq!(
            reset_count_regressions, 0,
            "reset counters regressed during high-contention churn"
        );
        assert_snapshot_sane("high-contention", snap);
    }
}

// ── Determinism ──────────────────────────────────────────────────────────────

#[test]
fn identical_inputs_produce_equivalent_snapshots() {
    const THREADS: usize = 4;
    const STEPS: u64 = 1_100;

    let start = Arc::new(Barrier::new(THREADS));
    let mut handles = Vec::with_capacity(THREADS);

    for _ in 0..THREADS {
        let start = Arc::clone(&start);
        handles.push(thread::spawn(move || {
            let mut ekf = Ekf::new(0).expect("thread-local EKF init should succeed");
            let mut update_failed = 0_u32;

            start.wait();

            for i in 0..STEPS {
                let ts = 1_000_000 + i * 10_000;
                let imu = ImuSample::new(ts, [0.0, 0.0, 0.00023], [0.0, 0.0, -0.0981], 0.01, 0.01);
                ekf.set_imu_data(&imu);
                if let Err(EkfError::UpdateFailed) = ekf.update() {
                    update_failed += 1;
                }
            }

            (update_failed, snapshot(&ekf))
        }));
    }

    let mut snapshots = Vec::with_capacity(THREADS);
    for handle in handles {
        let (update_failed, snap) = handle
            .join()
            .expect("deterministic parallel worker should not panic");
        assert!(update_failed < 120);
        assert_snapshot_sane("deterministic-worker", snap);
        snapshots.push(snap);
    }

    let base = snapshots[0];
    for (idx, s) in snapshots.iter().enumerate().skip(1) {
        assert!(
            abs_max_diff(base.q, s.q) < 1e-4,
            "thread {idx}: quaternion diverged"
        );
        assert!(
            abs_max_diff(base.v, s.v) < 1e-4,
            "thread {idx}: velocity diverged"
        );
        assert!(
            abs_max_diff(base.p, s.p) < 1e-4,
            "thread {idx}: position diverged"
        );
        assert_eq!(base.soln_status_bits, s.soln_status_bits);
        assert_eq!(base.quat_reset_count, s.quat_reset_count);
    }
}
