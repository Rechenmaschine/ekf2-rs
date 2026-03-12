use ekf2::{types::ImuSample, Ekf, EkfError};

#[test]
fn new_accepts_varied_timestamps_and_runs_first_update() {
    let timestamps = [0_u64, 1, 10_000, 1_000_000, 50_000_000];
    for &t0 in &timestamps {
        let mut ekf = Ekf::new(t0).expect("EKF init should succeed");
        let imu = ImuSample::new(
            t0.saturating_add(10_000),
            [0.0, 0.0, 0.0001],
            [0.0, 0.0, -9.81 * 0.01],
            0.01,
            0.01,
        );
        ekf.set_imu_data(&imu);
        assert!(matches!(ekf.update(), Ok(()) | Err(EkfError::UpdateFailed)));
        assert!(
            ekf.quaternion().iter().all(|v| v.is_finite()),
            "quaternion must remain finite for start timestamp {t0}"
        );
    }
}
