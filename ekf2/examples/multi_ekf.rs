use ekf2::{types::ImuSample, Ekf, EkfError, HeightReference};

#[cfg(feature = "barometer")]
use ekf2::types::BaroSample;
#[cfg(feature = "magnetometer")]
use ekf2::types::MagSample;

fn main() -> Result<(), EkfError> {
    // EKF A: attitude-focused profile.
    let mut attitude_ekf = Ekf::new(0)?;
    {
        let mut p = attitude_ekf.params_mut();
        p.set_gyro_noise(0.018);
        p.set_accel_noise(0.45);
        p.set_delay_max_ms(80.0);
        p.set_hgt_ref(HeightReference::Baro);
    }

    // EKF B: navigation-focused profile.
    let mut nav_ekf = Ekf::new(0)?;
    {
        let mut p = nav_ekf.params_mut();
        p.set_gyro_noise(0.012);
        p.set_accel_noise(0.32);
        p.set_delay_max_ms(150.0);
        p.set_hgt_ref(HeightReference::Baro);
    }

    let dt_s = 0.01_f32;
    let mut attitude_update_failed = 0_u32;
    let mut nav_update_failed = 0_u32;

    #[cfg(feature = "barometer")]
    let mut alt = 488.0_f32;

    for i in 1..=3_000_u64 {
        let ts_us = 1_000_000 + i * 10_000;

        let imu_attitude = ImuSample::with_clipping(
            ts_us,
            [0.0, 0.0, 0.0007],
            [0.0, 0.0, -9.81 * dt_s],
            dt_s,
            dt_s,
            [false, false, i % 700 == 0],
        );
        attitude_ekf.set_imu_data(&imu_attitude);

        let imu_nav = ImuSample::new(
            ts_us,
            [0.0, 0.0, 0.0004],
            [0.03 * dt_s, 0.005 * dt_s, -9.81 * dt_s],
            dt_s,
            dt_s,
        );
        nav_ekf.set_imu_data(&imu_nav);

        #[cfg(feature = "barometer")]
        if i > 30 && i % 4 == 0 {
            alt += 0.0008;
            nav_ekf.set_baro_data(&BaroSample::new(ts_us, alt + 0.15));
            attitude_ekf.set_baro_data(&BaroSample::new(ts_us, 488.0));
        }

        #[cfg(feature = "magnetometer")]
        if i > 30 && i % 5 == 0 {
            let heading = (i as f32) * 0.0003;
            nav_ekf.set_mag_data(&MagSample::new(ts_us, [0.22, 0.02 * heading.sin(), 0.41]));
        }

        if let Err(EkfError::UpdateFailed) = attitude_ekf.update() {
            attitude_update_failed += 1;
        }

        if let Err(EkfError::UpdateFailed) = nav_ekf.update() {
            nav_update_failed += 1;
        }
    }

    println!(
        "attitude_ekf: valid={} q={:?} (failed={attitude_update_failed})",
        attitude_ekf.attitude_valid(),
        attitude_ekf.quaternion(),
    );
    println!(
        "nav_ekf: pos={:?} vel={:?} (failed={nav_update_failed})",
        nav_ekf.position_ned(),
        nav_ekf.velocity_ned(),
    );

    Ok(())
}
