use ekf2::{types::ImuSample, Ekf, EkfError};

mod common;
use common::{feed_imu, quat_norm, yaw_from_quaternion};

// ── Drop safety ──────────────────────────────────────────────────────────────

#[test]
fn drop_of_initialized_instance_is_safe() {
    let ekf = Ekf::new(0).expect("init should succeed");
    drop(ekf);
    // Re-init after drop should succeed.
    let ekf2 = Ekf::new(0).expect("re-init should succeed");
    drop(ekf2);
}

// ── Repeated use ─────────────────────────────────────────────────────────────

#[test]
fn repeated_init_update_drop_cycles_remain_stable() {
    for cycle in 0..200_u64 {
        let mut ekf = Ekf::new(cycle * 1_000_000).expect("init should succeed");
        let _ = feed_imu(&mut ekf, cycle * 1_000_000 + 100_000, 80, 0.0002);
        let q = ekf.quaternion();
        assert!(q.iter().all(|v| v.is_finite()));
    }
}

#[test]
fn quaternion_stays_unit_length_under_imu_loop() {
    let mut ekf = Ekf::new(0).expect("init should succeed");
    let _ = feed_imu(&mut ekf, 100_000, 400, 0.0002);

    let q = ekf.quaternion();
    assert!(
        q.iter().all(|v| v.is_finite()),
        "quaternion components must be finite: {q:?}"
    );
    let norm = quat_norm(q);
    assert!(
        (norm - 1.0).abs() < 1e-3,
        "quaternion must remain unit-length (‖q‖ = {norm:.6})"
    );
}

// ── Instance independence ────────────────────────────────────────────────────

#[test]
fn many_instances_stay_independent() {
    const INSTANCES: usize = 48;
    const DT_S: f32 = 0.01;

    let mut ekfs: Vec<Ekf> = (0..INSTANCES)
        .map(|i| {
            let mut ekf = Ekf::new(0).expect("loop-created EKF init should succeed");
            let mut p = ekf.params_mut();
            p.set_gyro_noise(0.012 + i as f32 * 0.003);
            p.set_accel_noise(0.30 + i as f32 * 0.04);
            ekf
        })
        .collect();

    for (i, ekf) in ekfs.iter().enumerate() {
        let expected_gyro_noise = 0.012 + i as f32 * 0.003;
        let expected_accel_noise = 0.30 + i as f32 * 0.04;
        assert!(
            (ekf.params().gyro_noise() - expected_gyro_noise).abs() < 1e-6,
            "ekf[{i}] gyro_noise should remain instance-local after loop init"
        );
        assert!(
            (ekf.params().accel_noise() - expected_accel_noise).abs() < 1e-6,
            "ekf[{i}] accel_noise should remain instance-local after loop init"
        );
    }

    let mut update_failed = [0_u32; INSTANCES];
    let center = (INSTANCES as f32 - 1.0) * 0.5;

    for step in 1..=1_200_u64 {
        let ts = 1_000_000 + step * 10_000;

        for (i, ekf) in ekfs.iter_mut().enumerate() {
            let yaw_rate = (i as f32 - center) * 0.00008;
            let accel_x = i as f32 * 0.001 * DT_S;
            let imu = ImuSample::new(
                ts,
                [0.0, 0.0, yaw_rate],
                [accel_x, 0.0, -9.81 * DT_S],
                DT_S,
                DT_S,
            );
            ekf.set_imu_data(&imu);
            if let Err(EkfError::UpdateFailed) = ekf.update() {
                update_failed[i] += 1;
            }
        }
    }

    let mut yaws = Vec::with_capacity(INSTANCES);
    for (i, ekf) in ekfs.iter().enumerate() {
        let q = ekf.quaternion();
        assert!(
            q.iter().all(|v| v.is_finite()),
            "ekf[{i}] quaternion should stay finite"
        );
        yaws.push(yaw_from_quaternion(q));
        assert!(
            update_failed[i] < 40,
            "ekf[{i}] had too many update failures: {}",
            update_failed[i]
        );
    }

    for i in 0..yaws.len() {
        for j in (i + 1)..yaws.len() {
            assert!(
                (yaws[i] - yaws[j]).abs() > 0.004,
                "ekf[{i}] and ekf[{j}] ended too similar, possible shared-state regression"
            );
        }
    }
}
