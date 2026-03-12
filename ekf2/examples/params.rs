use ekf2::{Ekf, EkfError, HeightReference};

fn main() -> Result<(), EkfError> {
    let baseline = Ekf::new(0)?;
    let mut tuned = Ekf::new(0)?;

    {
        let mut p = tuned.params_mut();
        p.set_gyro_noise(0.015);
        p.set_accel_noise(0.35);
        p.set_delay_max_ms(110.0);
        p.set_hgt_ref(HeightReference::Baro);

        #[cfg(feature = "gnss")]
        {
            p.set_gps_pos_noise(0.45);
            p.set_gps_vel_noise(0.2);
        }

        #[cfg(feature = "magnetometer")]
        {
            p.set_mag_noise(0.04);
            p.set_mag_innov_gate(3.0);
        }
    }

    let base = baseline.params();
    let tuned = tuned.params();

    println!("baseline vs tuned params");
    println!(
        "gyro_noise: {} -> {}",
        base.gyro_noise(),
        tuned.gyro_noise()
    );
    println!(
        "accel_noise: {} -> {}",
        base.accel_noise(),
        tuned.accel_noise()
    );
    println!(
        "delay_max_ms: {} -> {}",
        base.delay_max_ms(),
        tuned.delay_max_ms()
    );
    println!("hgt_ref: {:?} -> {:?}", base.hgt_ref(), tuned.hgt_ref());

    #[cfg(feature = "gnss")]
    println!(
        "gps_pos_noise: {} -> {}",
        base.gps_pos_noise(),
        tuned.gps_pos_noise()
    );

    #[cfg(feature = "magnetometer")]
    println!("mag_noise: {} -> {}", base.mag_noise(), tuned.mag_noise());

    Ok(())
}
