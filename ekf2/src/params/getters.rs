use ekf2_sys as ffi;

use super::*;

impl<'a> Params<'a> {
    #[inline]
    pub fn gyro_noise(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gyro_noise(self.ekf_ptr) }
    }

    #[inline]
    pub fn accel_noise(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_accel_noise(self.ekf_ptr) }
    }

    #[inline]
    pub fn gyro_bias_noise(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gyro_bias_noise(self.ekf_ptr) }
    }

    #[inline]
    pub fn accel_bias_noise(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_accel_bias_noise(self.ekf_ptr) }
    }

    #[inline]
    pub fn gyro_bias_init(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gyro_bias_init(self.ekf_ptr) }
    }

    #[inline]
    pub fn accel_bias_init(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_accel_bias_init(self.ekf_ptr) }
    }

    #[inline]
    pub fn tilt_error_init_rad(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_angerr_init(self.ekf_ptr) }
    }

    #[inline]
    pub fn delay_max_ms(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_delay_max(self.ekf_ptr) }
    }

    #[inline]
    pub fn predict_interval_us(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_predict_us(self.ekf_ptr) }
    }

    #[inline]
    pub fn imu_ctrl_raw(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_imu_ctrl(self.ekf_ptr) }
    }

    #[inline]
    pub fn imu_control(&self) -> ImuControl {
        ImuControl::from_bits_truncate(self.imu_ctrl_raw())
    }

    #[inline]
    pub fn velocity_limit_m_s(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_vel_lim(self.ekf_ptr) }
    }

    #[inline]
    pub fn position_sensor_ref_raw(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_position_sensor_ref(self.ekf_ptr) }
    }

    #[inline]
    pub fn position_sensor_ref(&self) -> Option<PositionReference> {
        PositionReference::from_i32(self.position_sensor_ref_raw())
    }

    #[inline]
    pub fn no_aid_timeout_us(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_noaid_tout(self.ekf_ptr) }
    }

    #[inline]
    pub fn no_aid_noise_m(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_noaid_noise(self.ekf_ptr) }
    }

    #[inline]
    pub fn heading_innov_gate(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_hdg_gate(self.ekf_ptr) }
    }

    #[inline]
    pub fn heading_noise_rad(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_head_noise(self.ekf_ptr) }
    }

    #[inline]
    pub fn accel_bias_limit_m_s2(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_abl_lim(self.ekf_ptr) }
    }

    #[inline]
    pub fn accel_bias_learning_accel_limit_m_s2(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_abl_acclim(self.ekf_ptr) }
    }

    #[inline]
    pub fn accel_bias_learning_gyro_limit_rad_s(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_abl_gyrlim(self.ekf_ptr) }
    }

    #[inline]
    pub fn accel_bias_learning_tau_s(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_abl_tau(self.ekf_ptr) }
    }

    #[inline]
    pub fn gyro_bias_limit_rad_s(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gyr_b_lim(self.ekf_ptr) }
    }

    #[inline]
    pub fn imu_pos_body_x(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_imu_pos_body_x(self.ekf_ptr) }
    }

    #[inline]
    pub fn imu_pos_body_y(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_imu_pos_body_y(self.ekf_ptr) }
    }

    #[inline]
    pub fn imu_pos_body_z(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_imu_pos_body_z(self.ekf_ptr) }
    }

    #[inline]
    pub fn imu_pos_body(&self) -> [f32; 3] {
        [
            self.imu_pos_body_x(),
            self.imu_pos_body_y(),
            self.imu_pos_body_z(),
        ]
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn baro_noise(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_baro_noise(self.ekf_ptr) }
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn baro_innov_gate(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_baro_gate(self.ekf_ptr) }
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn baro_bias_nsd(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_baro_bias_nsd(self.ekf_ptr) }
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn baro_ctrl(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_baro_ctrl(self.ekf_ptr) }
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn baro_control(&self) -> Option<BaroControl> {
        BaroControl::from_i32(self.baro_ctrl())
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn baro_delay_ms(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_baro_delay(self.ekf_ptr) }
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn ground_effect_deadzone_m(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gnd_eff_dz(self.ekf_ptr) }
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn ground_effect_max_hgt_m(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gnd_max_hgt(self.ekf_ptr) }
    }

    #[cfg(feature = "baro-compensation")]
    #[inline]
    pub fn baro_pcoef_xp(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_pcoef_xp(self.ekf_ptr) }
    }

    #[cfg(feature = "baro-compensation")]
    #[inline]
    pub fn baro_pcoef_xn(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_pcoef_xn(self.ekf_ptr) }
    }

    #[cfg(feature = "baro-compensation")]
    #[inline]
    pub fn baro_pcoef_yp(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_pcoef_yp(self.ekf_ptr) }
    }

    #[cfg(feature = "baro-compensation")]
    #[inline]
    pub fn baro_pcoef_yn(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_pcoef_yn(self.ekf_ptr) }
    }

    #[cfg(feature = "baro-compensation")]
    #[inline]
    pub fn baro_pcoef_z(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_pcoef_z(self.ekf_ptr) }
    }

    #[cfg(feature = "baro-compensation")]
    #[inline]
    pub fn baro_aspd_max_m_s(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_aspd_max(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_pos_noise(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gps_pos_noise(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_vel_noise(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gps_vel_noise(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_pos_gate(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gps_pos_gate(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_vel_gate(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gps_vel_gate(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_mode_raw(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_gps_mode(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_mode(&self) -> Option<GnssMode> {
        GnssMode::from_i32(self.gps_mode_raw())
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_delay_ms(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gps_delay(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_pos_body_x(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gps_pos_body_x(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_pos_body_y(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gps_pos_body_y(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_pos_body_z(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gps_pos_body_z(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_pos_body(&self) -> [f32; 3] {
        [
            self.gps_pos_body_x(),
            self.gps_pos_body_y(),
            self.gps_pos_body_z(),
        ]
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_hgt_bias_nsd(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gps_hgt_bias_nsd(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_ctrl(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_gps_ctrl(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_control(&self) -> GnssControl {
        GnssControl::from_bits_truncate(self.gps_ctrl())
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_check_mask(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_gps_check(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_check_control(&self) -> GnssCheckControl {
        GnssCheckControl::from_bits_truncate(self.gps_check_mask())
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn required_eph_m(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_req_eph(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn required_epv_m(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_req_epv(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn required_sacc_m_s(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_req_sacc(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn required_nsats(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_req_nsats(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn required_pdop(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_req_pdop(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn required_h_drift_m_s(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_req_hdrift(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn required_v_drift_m_s(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_req_vdrift(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn required_fix_type(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_req_fix(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn required_fix(&self) -> Option<GnssFixType> {
        GnssFixType::from_i32(self.required_fix_type())
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gsf_tas_default_m_s(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gsf_tas(self.ekf_ptr) }
    }

    #[cfg(feature = "gnss-yaw")]
    #[inline]
    pub fn gnss_heading_noise(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_gnss_heading_noise(self.ekf_ptr) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_noise(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_mag_noise(self.ekf_ptr) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_declination_deg(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_mag_decl(self.ekf_ptr) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_innov_gate(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_mag_gate(self.ekf_ptr) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_delay_ms(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_mag_delay(self.ekf_ptr) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_decl_type_raw(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_decl_type(self.ekf_ptr) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_decl_type(&self) -> MagDeclinationControl {
        MagDeclinationControl::from_bits_truncate(self.mag_decl_type_raw())
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_fusion_type_raw(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_mag_type(self.ekf_ptr) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_fusion_type(&self) -> Option<MagFusionType> {
        MagFusionType::from_i32(self.mag_fusion_type_raw())
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_check_mask_raw(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_mag_check(self.ekf_ptr) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_check_mask(&self) -> MagCheckControl {
        MagCheckControl::from_bits_truncate(self.mag_check_mask_raw())
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_check_strength_gauss(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_mag_chk_str(self.ekf_ptr) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_check_inclination_deg(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_mag_chk_inc(self.ekf_ptr) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_earth_process_noise_gauss_s(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_mag_e_noise(self.ekf_ptr) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_body_process_noise_gauss_s(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_mag_b_noise(self.ekf_ptr) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_accel_limit_m_s2(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_mag_acclim(self.ekf_ptr) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn synthetic_mag_z_enabled(&self) -> bool {
        unsafe { ffi::ekf2_param_get_synt_mag_z(self.ekf_ptr) != 0 }
    }

    #[inline]
    pub fn hgt_ref_raw(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_hgt_ref(self.ekf_ptr) }
    }

    #[inline]
    pub fn hgt_ref(&self) -> Option<HeightReference> {
        HeightReference::from_i32(self.hgt_ref_raw())
    }

    #[cfg(feature = "airspeed")]
    #[inline]
    pub fn eas_noise(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_eas_noise(self.ekf_ptr) }
    }

    #[cfg(feature = "airspeed")]
    #[inline]
    pub fn tas_gate(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_tas_gate(self.ekf_ptr) }
    }

    #[cfg(feature = "airspeed")]
    #[inline]
    pub fn airspeed_delay_ms(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_asp_delay(self.ekf_ptr) }
    }

    #[cfg(feature = "airspeed")]
    #[inline]
    pub fn airspeed_fusion_threshold_m_s(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_arsp_thr(self.ekf_ptr) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn range_ctrl_raw(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_rng_ctrl(self.ekf_ptr) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn range_ctrl(&self) -> Option<RangeControl> {
        RangeControl::from_i32(self.range_ctrl_raw())
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn range_delay_ms(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_rng_delay(self.ekf_ptr) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn range_noise_m(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_rng_noise(self.ekf_ptr) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn range_gate(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_rng_gate(self.ekf_ptr) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn range_pitch_rad(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_rng_pitch(self.ekf_ptr) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn range_scale_per_meter(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_rng_sfe(self.ekf_ptr) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn range_aid_max_hgt_m(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_rng_a_hmax(self.ekf_ptr) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn range_aid_max_vel_m_s(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_rng_a_vmax(self.ekf_ptr) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn range_quality_min_duration_s(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_rng_qlty_t(self.ekf_ptr) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn range_kinematic_gate(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_rng_k_gate(self.ekf_ptr) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn range_fog_distance_m(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_rng_fog(self.ekf_ptr) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn range_pos_body_x(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_rng_pos_body_x(self.ekf_ptr) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn range_pos_body_y(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_rng_pos_body_y(self.ekf_ptr) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn range_pos_body_z(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_rng_pos_body_z(self.ekf_ptr) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn range_pos_body(&self) -> [f32; 3] {
        [
            self.range_pos_body_x(),
            self.range_pos_body_y(),
            self.range_pos_body_z(),
        ]
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn ev_ctrl_raw(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_ev_ctrl(self.ekf_ptr) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn ev_ctrl(&self) -> ExternalVisionControl {
        ExternalVisionControl::from_bits_truncate(self.ev_ctrl_raw())
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn ev_delay_ms(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_ev_delay(self.ekf_ptr) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn ev_vel_noise(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_evv_noise(self.ekf_ptr) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn ev_pos_noise(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_evp_noise(self.ekf_ptr) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn ev_att_noise(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_eva_noise(self.ekf_ptr) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn ev_quality_min(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_ev_qmin(self.ekf_ptr) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn ev_vel_gate(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_evv_gate(self.ekf_ptr) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn ev_pos_gate(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_evp_gate(self.ekf_ptr) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn ev_hgt_bias_nsd(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_ev_hgt_bias_nsd(self.ekf_ptr) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn ev_pos_body_x(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_ev_pos_body_x(self.ekf_ptr) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn ev_pos_body_y(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_ev_pos_body_y(self.ekf_ptr) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn ev_pos_body_z(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_ev_pos_body_z(self.ekf_ptr) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn ev_pos_body(&self) -> [f32; 3] {
        [
            self.ev_pos_body_x(),
            self.ev_pos_body_y(),
            self.ev_pos_body_z(),
        ]
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn of_ctrl_raw(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_of_ctrl(self.ekf_ptr) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn of_ctrl(&self) -> Option<OpticalFlowControl> {
        OpticalFlowControl::from_i32(self.of_ctrl_raw())
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn of_gyro_source_raw(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_of_gyr_src(self.ekf_ptr) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn of_gyro_source(&self) -> Option<FlowGyroSource> {
        FlowGyroSource::from_i32(self.of_gyro_source_raw())
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn of_delay_ms(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_of_delay(self.ekf_ptr) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn of_noise_min(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_of_n_min(self.ekf_ptr) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn of_noise_max(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_of_n_max(self.ekf_ptr) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn of_quality_min(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_of_qmin(self.ekf_ptr) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn of_quality_min_ground(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_of_qmin_gnd(self.ekf_ptr) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn of_gate(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_of_gate(self.ekf_ptr) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn flow_pos_body_x(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_flow_pos_body_x(self.ekf_ptr) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn flow_pos_body_y(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_flow_pos_body_y(self.ekf_ptr) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn flow_pos_body_z(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_flow_pos_body_z(self.ekf_ptr) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn flow_pos_body(&self) -> [f32; 3] {
        [
            self.flow_pos_body_x(),
            self.flow_pos_body_y(),
            self.flow_pos_body_z(),
        ]
    }

    #[cfg(feature = "wind")]
    #[inline]
    pub fn wind_nsd(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_wind_nsd(self.ekf_ptr) }
    }

    #[cfg(feature = "sideslip")]
    #[inline]
    pub fn sideslip_fusion_raw(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_fuse_beta(self.ekf_ptr) }
    }

    #[cfg(feature = "sideslip")]
    #[inline]
    pub fn sideslip_fusion_enabled(&self) -> bool {
        self.sideslip_fusion_raw() != 0
    }

    #[cfg(feature = "sideslip")]
    #[inline]
    pub fn sideslip_gate(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_beta_gate(self.ekf_ptr) }
    }

    #[cfg(feature = "sideslip")]
    #[inline]
    pub fn sideslip_noise_rad(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_beta_noise(self.ekf_ptr) }
    }

    #[cfg(feature = "terrain")]
    #[inline]
    pub fn terrain_noise(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_terr_noise(self.ekf_ptr) }
    }

    #[cfg(feature = "terrain")]
    #[inline]
    pub fn terrain_gradient(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_terr_grad(self.ekf_ptr) }
    }

    #[cfg(any(
        feature = "terrain",
        feature = "range-finder",
        feature = "optical-flow"
    ))]
    #[inline]
    pub fn min_range_m(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_min_rng(self.ekf_ptr) }
    }

    #[cfg(feature = "gravity-fusion")]
    #[inline]
    pub fn gravity_noise_m_s2(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_grav_noise(self.ekf_ptr) }
    }

    #[cfg(feature = "drag-fusion")]
    #[inline]
    pub fn drag_ctrl_raw(&self) -> i32 {
        unsafe { ffi::ekf2_param_get_drag_ctrl(self.ekf_ptr) }
    }

    #[cfg(feature = "drag-fusion")]
    #[inline]
    pub fn drag_fusion_enabled(&self) -> bool {
        self.drag_ctrl_raw() != 0
    }

    #[cfg(feature = "drag-fusion")]
    #[inline]
    pub fn drag_noise(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_drag_noise(self.ekf_ptr) }
    }

    #[cfg(feature = "drag-fusion")]
    #[inline]
    pub fn drag_bcoef_x(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_bcoef_x(self.ekf_ptr) }
    }

    #[cfg(feature = "drag-fusion")]
    #[inline]
    pub fn drag_bcoef_y(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_bcoef_y(self.ekf_ptr) }
    }

    #[cfg(feature = "drag-fusion")]
    #[inline]
    pub fn drag_mcoef(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_mcoef(self.ekf_ptr) }
    }

    #[cfg(feature = "aux-vel")]
    #[inline]
    pub fn aux_vel_delay_ms(&self) -> f32 {
        unsafe { ffi::ekf2_param_get_avel_delay(self.ekf_ptr) }
    }
}
