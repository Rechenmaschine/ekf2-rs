#[cfg(not(all(feature = "gnss", feature = "magnetometer", feature = "barometer")))]
fn main() {
    eprintln!(
        "This example requires features: gnss, magnetometer, barometer. \
Run with: cargo run -p ekf2 --example gnss_mag_baro --features 'gnss,magnetometer,barometer'"
    );
}

#[cfg(all(feature = "gnss", feature = "magnetometer", feature = "barometer"))]
fn main() -> Result<(), ekf2::EkfError> {
    use ekf2::{
        types::{BaroSample, GnssSample, ImuSample, MagSample},
        Ekf, EkfError,
    };

    let mut ekf = Ekf::new(0)?;
    let dt_s = 0.01_f32;
    let mut update_failed = 0_u32;

    let mut lat = 47.397742_f64;
    let mut lon = 8.545594_f64;
    let mut alt = 488.0_f32;

    for i in 1..=2_000_u64 {
        let ts_us = 1_000_000 + i * 10_000;

        let imu = ImuSample::new(
            ts_us,
            [0.0, 0.0, 0.0003],
            [0.02 * dt_s, 0.0, -9.81 * dt_s],
            dt_s,
            dt_s,
        );
        ekf.set_imu_data(&imu);

        if i > 30 && i % 10 == 0 {
            lat += 0.00000002;
            lon += 0.00000005;
            alt += 0.001;
            let gps = GnssSample::new(ts_us, lat, lon, alt, [2.0, 0.1, 0.0], 0.5, 0.8, 0.15, 3, 16);
            ekf.set_gps_data(&gps);
        }

        if i > 30 && i % 5 == 0 {
            let mag = MagSample::new(ts_us, [0.215, 0.01, 0.43]);
            ekf.set_mag_data(&mag);
        }

        if i > 30 && i % 4 == 0 {
            let baro = BaroSample::new(ts_us, alt + 0.2);
            ekf.set_baro_data(&baro);
        }

        if let Err(EkfError::UpdateFailed) = ekf.update() {
            update_failed += 1;
        }
    }

    println!("gnss+mag+baro run finished");
    println!("attitude_valid: {}", ekf.attitude_valid());
    println!("local_position_valid: {}", ekf.local_position_valid());
    println!("position_ned: {:?}", ekf.position_ned());
    println!("velocity_ned: {:?}", ekf.velocity_ned());
    println!("pos_var: {:?}", ekf.pos_variance());
    println!("vel_var: {:?}", ekf.vel_variance());
    println!("update_failed_count: {update_failed}");

    Ok(())
}
