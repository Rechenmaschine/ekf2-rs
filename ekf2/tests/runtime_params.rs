use ekf2::{Ekf, HeightReference, ImuControl, PositionReference};

#[cfg(feature = "barometer")]
use ekf2::BaroControl;
#[cfg(feature = "external-vision")]
use ekf2::ExternalVisionControl;
#[cfg(feature = "range-finder")]
use ekf2::RangeControl;
#[cfg(feature = "optical-flow")]
use ekf2::{FlowGyroSource, OpticalFlowControl};
#[cfg(feature = "gnss")]
use ekf2::{GnssControl, GnssFixType, GnssMode};
#[cfg(feature = "magnetometer")]
use ekf2::{MagDeclinationControl, MagFusionType};

#[test]
fn params_are_instance_local_for_representative_fields() {
    let mut a = Ekf::new(0).expect("init a should succeed");
    let b = Ekf::new(0).expect("init b should succeed");

    {
        let mut p = a.params_mut();
        p.set_gyro_noise(0.019);
        p.set_accel_noise(0.42);
        p.set_predict_interval_us(12_000);
        p.set_imu_control(ImuControl::all());
        p.set_position_sensor_ref(PositionReference::ExternalVision);
        p.set_hgt_ref(HeightReference::Gnss);

        #[cfg(feature = "barometer")]
        {
            p.set_baro_control(BaroControl::Disabled);
            p.set_baro_delay_ms(9.0);
        }

        #[cfg(feature = "gnss")]
        {
            p.set_gps_mode(GnssMode::DeadReckoning);
            p.set_gps_control(GnssControl::empty());
            p.set_required_fix(GnssFixType::RtcmCodeDifferential);
        }

        #[cfg(feature = "magnetometer")]
        {
            p.set_mag_decl_type(MagDeclinationControl::USE_GEO_DECL);
            p.set_mag_fusion_type(MagFusionType::Heading);
            p.set_synthetic_mag_z_enabled(true);
        }

        #[cfg(feature = "range-finder")]
        {
            p.set_range_ctrl(RangeControl::Enabled);
            p.set_range_delay_ms(8.0);
        }

        #[cfg(feature = "external-vision")]
        {
            p.set_ev_ctrl(ExternalVisionControl::HPOS | ExternalVisionControl::VEL);
            p.set_ev_delay_ms(120.0);
        }

        #[cfg(feature = "optical-flow")]
        {
            p.set_of_ctrl(OpticalFlowControl::Enabled);
            p.set_of_gyro_source(FlowGyroSource::Internal);
        }
    }

    let pa = a.params();
    let pb = b.params();

    assert!((pa.gyro_noise() - 0.019).abs() < 1e-6);
    assert!((pa.accel_noise() - 0.42).abs() < 1e-6);
    assert_eq!(pa.predict_interval_us(), 12_000);
    assert!(pa.imu_control().contains(ImuControl::GRAVITY_VECTOR));
    assert_eq!(
        pa.position_sensor_ref(),
        Some(PositionReference::ExternalVision)
    );
    assert_eq!(pa.hgt_ref(), Some(HeightReference::Gnss));

    // Instance locality: untouched instance must not inherit modified state.
    assert!((pb.gyro_noise() - pa.gyro_noise()).abs() > 1e-6);
    assert!((pb.accel_noise() - pa.accel_noise()).abs() > 1e-6);
    assert_ne!(pb.predict_interval_us(), pa.predict_interval_us());
    assert_ne!(pb.imu_control(), pa.imu_control());
    assert_ne!(pb.position_sensor_ref(), pa.position_sensor_ref());
    assert_ne!(pb.hgt_ref(), pa.hgt_ref());

    #[cfg(feature = "barometer")]
    {
        assert_eq!(pa.baro_control(), Some(BaroControl::Disabled));
        assert!((pa.baro_delay_ms() - 9.0).abs() < 1e-6);
        assert_ne!(pb.baro_control(), pa.baro_control());
        assert!((pb.baro_delay_ms() - pa.baro_delay_ms()).abs() > 1e-6);
    }

    #[cfg(feature = "gnss")]
    {
        assert_eq!(pa.gps_mode(), Some(GnssMode::DeadReckoning));
        assert_eq!(pa.gps_control(), GnssControl::empty());
        assert_eq!(pa.required_fix(), Some(GnssFixType::RtcmCodeDifferential));
        assert_ne!(pb.gps_mode(), pa.gps_mode());
        assert_ne!(pb.gps_control(), pa.gps_control());
        assert_ne!(pb.required_fix(), pa.required_fix());
    }

    #[cfg(feature = "magnetometer")]
    {
        assert_eq!(pa.mag_decl_type(), MagDeclinationControl::USE_GEO_DECL);
        assert_eq!(pa.mag_fusion_type(), Some(MagFusionType::Heading));
        assert!(pa.synthetic_mag_z_enabled());
        assert_ne!(pb.mag_decl_type(), pa.mag_decl_type());
        assert_ne!(pb.mag_fusion_type(), pa.mag_fusion_type());
        assert_ne!(pb.synthetic_mag_z_enabled(), pa.synthetic_mag_z_enabled());
    }

    #[cfg(feature = "range-finder")]
    {
        assert_eq!(pa.range_ctrl(), Some(RangeControl::Enabled));
        assert!((pa.range_delay_ms() - 8.0).abs() < 1e-6);
        assert_ne!(pb.range_ctrl(), pa.range_ctrl());
        assert!((pb.range_delay_ms() - pa.range_delay_ms()).abs() > 1e-6);
    }

    #[cfg(feature = "external-vision")]
    {
        assert!(pa.ev_ctrl().contains(ExternalVisionControl::HPOS));
        assert!(pa.ev_ctrl().contains(ExternalVisionControl::VEL));
        assert!((pa.ev_delay_ms() - 120.0).abs() < 1e-6);
        assert_ne!(pb.ev_ctrl(), pa.ev_ctrl());
        assert!((pb.ev_delay_ms() - pa.ev_delay_ms()).abs() > 1e-6);
    }

    #[cfg(feature = "optical-flow")]
    {
        assert_eq!(pa.of_ctrl(), Some(OpticalFlowControl::Enabled));
        assert_eq!(pa.of_gyro_source(), Some(FlowGyroSource::Internal));
        assert_ne!(pb.of_ctrl(), pa.of_ctrl());
        assert_ne!(pb.of_gyro_source(), pa.of_gyro_source());
    }
}
