use ekf2::{types::ImuSample, Ekf, EkfError};

fn main() -> Result<(), EkfError> {
    let mut ekf = Ekf::new(0)?;
    let dt_s = 0.01_f32;
    let mut update_failed = 0_u32;

    for i in 1..=2_000_u64 {
        let ts_us = 1_000_000 + i * 10_000;
        let yaw_rate = if i < 1_000 { 0.0006 } else { -0.0005 };
        let clip_z = i % 450 == 0;

        let imu = ImuSample::with_clipping(
            ts_us,
            [0.0, 0.0, yaw_rate],
            [0.0, 0.0, -9.81 * dt_s],
            dt_s,
            dt_s,
            [false, false, clip_z],
        );

        ekf.set_imu_data(&imu);
        if let Err(EkfError::UpdateFailed) = ekf.update() {
            update_failed += 1;
        }
    }

    println!("imu-only run finished");
    println!("attitude_valid: {}", ekf.attitude_valid());
    println!("local_velocity_valid: {}", ekf.local_velocity_valid());
    println!("q: {:?}", ekf.quaternion());
    println!("vel_ned: {:?}", ekf.velocity_ned());
    println!("pos_var: {:?}", ekf.pos_variance());
    println!("update_failed_count: {update_failed}");

    Ok(())
}
