use ekf2::{types::ImuSample, Ekf, EkfError};

mod common;
use common::{feed_imu, quat_norm};

// ── Basic reset ─────────────────────────────────────────────────────────────

#[test]
fn reset_succeeds_on_fresh_instance() {
    let mut ekf = Ekf::new(0).expect("init should succeed");
    ekf.reset(1_000_000).expect("reset should succeed");
}

#[test]
fn reset_succeeds_after_imu_updates() {
    let mut ekf = Ekf::new(0).expect("init should succeed");
    let _ = feed_imu(&mut ekf, 100_000, 200, 0.0001);
    ekf.reset(5_000_000).expect("reset after updates should succeed");
}

// ── State is wiped ──────────────────────────────────────────────────────────

#[test]
fn reset_wipes_filter_state() {
    let mut ekf = Ekf::new(0).expect("init should succeed");

    // Drive the filter so it accumulates non-trivial state.
    let _ = feed_imu(&mut ekf, 100_000, 400, 0.001);
    let q_before = ekf.quaternion();

    // Reset and take a fresh snapshot.
    ekf.reset(10_000_000).expect("reset should succeed");
    let q_after = ekf.quaternion();

    // After reset the quaternion should be back to identity-ish initial state,
    // distinct from the rotated state we had before.
    let fresh = Ekf::new(10_000_000).expect("fresh init");
    let q_fresh = fresh.quaternion();

    // q_after should be close to q_fresh (both freshly initialized).
    let diff_reset_vs_fresh: f32 = q_after
        .iter()
        .zip(q_fresh.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0_f32, f32::max);
    assert!(
        diff_reset_vs_fresh < 0.05,
        "reset state should match fresh init (max diff = {diff_reset_vs_fresh})"
    );

    // q_before should differ from q_after (state was actually wiped).
    let diff_before_vs_after: f32 = q_before
        .iter()
        .zip(q_after.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0_f32, f32::max);
    assert!(
        diff_before_vs_after > 1e-4,
        "pre-reset state should differ from post-reset state"
    );
}

// ── Usable after reset ──────────────────────────────────────────────────────

#[test]
fn filter_is_usable_after_reset() {
    let mut ekf = Ekf::new(0).expect("init should succeed");
    let _ = feed_imu(&mut ekf, 100_000, 100, 0.0001);

    ekf.reset(5_000_000).expect("reset should succeed");

    // Feed IMU data after reset and verify the filter still works.
    let _ = feed_imu(&mut ekf, 5_100_000, 200, 0.0002);
    let q = ekf.quaternion();
    assert!(
        q.iter().all(|v| v.is_finite()),
        "quaternion must be finite after reset + updates: {q:?}"
    );
    let norm = quat_norm(q);
    assert!(
        (norm - 1.0).abs() < 1e-3,
        "quaternion must be unit-length after reset (norm = {norm:.6})"
    );
}

// ── Repeated resets ─────────────────────────────────────────────────────────

#[test]
fn repeated_resets_remain_stable() {
    let mut ekf = Ekf::new(0).expect("init should succeed");

    for cycle in 0..50_u64 {
        let base_ts = cycle * 2_000_000;
        let _ = feed_imu(&mut ekf, base_ts + 100_000, 80, 0.0002);

        let q = ekf.quaternion();
        assert!(q.iter().all(|v| v.is_finite()), "cycle {cycle}: finite quaternion");

        ekf.reset(base_ts + 1_000_000)
            .unwrap_or_else(|e| panic!("cycle {cycle}: reset failed: {e:?}"));
    }
}

// ── Params after reset ──────────────────────────────────────────────────────

#[test]
fn params_are_default_after_reset() {
    let mut ekf = Ekf::new(0).expect("init should succeed");

    // Change a param from default.
    let default_gyro_noise = ekf.params().gyro_noise();
    let custom_noise = default_gyro_noise + 0.1;
    ekf.params_mut().set_gyro_noise(custom_noise);
    assert!(
        (ekf.params().gyro_noise() - custom_noise).abs() < 1e-6,
        "param should be set before reset"
    );

    // Reset reconstructs the C++ object, so params go back to defaults.
    ekf.reset(1_000_000).expect("reset should succeed");
    let noise_after = ekf.params().gyro_noise();
    assert!(
        (noise_after - default_gyro_noise).abs() < 1e-6,
        "params should be default after reset (got {noise_after}, expected {default_gyro_noise})"
    );
}
