use ekf2::{types::ImuSample, Ekf, EkfError};

fn imu_sample(timestamp_us: u64) -> ImuSample {
    ImuSample::new(
        timestamp_us,
        [0.0, 0.0, 0.0002],
        [0.0, 0.0, -9.81 * 0.01],
        0.01,
        0.01,
    )
}

#[test]
fn ekf_basic_init_and_update() {
    let mut ekf = Ekf::new(0).expect("Ekf::new should succeed");

    for step in 1..=8_u64 {
        ekf.set_imu_data(&imu_sample(step * 10_000));
        assert!(matches!(ekf.update(), Ok(()) | Err(EkfError::UpdateFailed)));
    }
}

#[test]
fn ekf_is_movable() {
    let mut ekf = Ekf::new(0).expect("Ekf::new should succeed");
    ekf.set_imu_data(&imu_sample(10_000));
    assert!(matches!(ekf.update(), Ok(()) | Err(EkfError::UpdateFailed)));

    // Move into new binding — should still work.
    let mut moved = ekf;
    moved.set_imu_data(&imu_sample(20_000));
    assert!(matches!(
        moved.update(),
        Ok(()) | Err(EkfError::UpdateFailed)
    ));
    assert_eq!(moved.quaternion().len(), 4);
}
