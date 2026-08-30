use ekf2::{types::ImuSample, Ekf, EkfConfig};

#[cfg(feature = "gnss")]
#[test]
fn gnss_constructor_preserves_pdop() {
    let sample = ekf2::types::GnssSample::new(
        0, 47.397742, 8.545594, 488.0, [0.0; 3], 0.5, 0.8, 0.2, 1.5, 3, 16,
    );

    assert_eq!(sample.pdop, 1.5);
}

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
    let mut ekf = Ekf::new().expect("Ekf::new should succeed");

    for step in 1..=8_u64 {
        ekf.set_imu_data(&imu_sample(step * 10_000));
        let _ = ekf.update();
    }
}

#[test]
fn ekf_is_movable() {
    let mut ekf = Ekf::new().expect("Ekf::new should succeed");
    ekf.set_imu_data(&imu_sample(10_000));
    let _ = ekf.update();

    // Move into new binding — should still work.
    let mut moved = ekf;
    moved.set_imu_data(&imu_sample(20_000));
    let _ = moved.update();
    assert_eq!(moved.quaternion().len(), 4);
}

#[test]
fn configured_ekf_uses_named_startup_configuration() {
    let ekf = Ekf::with_config(EkfConfig {
        predict_interval_us: 5_000,
        delay_max_ms: 200.0,
    })
    .expect("configured EKF should succeed");

    assert_eq!(ekf.params().predict_interval_us(), 5_000);
    assert_eq!(ekf.params().delay_max_ms(), 200.0);
}

#[test]
fn global_origin_reports_when_it_is_established() {
    let mut ekf = Ekf::new().expect("Ekf::new should succeed");
    assert!(!ekf.global_origin_valid());

    ekf.set_global_origin(47.397742, 8.545594, 488.0, f32::NAN, f32::NAN)
        .expect("valid global origin should be accepted");

    assert!(ekf.global_origin_valid());
}
