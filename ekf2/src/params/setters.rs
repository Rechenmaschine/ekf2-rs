use ekf2_sys as ffi;

use super::*;

impl<'a> ParamsMut<'a> {
    #[inline]
    pub fn set_gyro_noise(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gyro_noise(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_accel_noise(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_accel_noise(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_gyro_bias_noise(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gyro_bias_noise(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_accel_bias_noise(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_accel_bias_noise(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_gyro_bias_init(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gyro_bias_init(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_accel_bias_init(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_accel_bias_init(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_tilt_error_init_rad(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_angerr_init(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_delay_max_ms(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_delay_max(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_predict_interval_us(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_predict_us(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_imu_ctrl_raw(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_imu_ctrl(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_imu_control(&mut self, v: ImuControl) {
        self.set_imu_ctrl_raw(v.bits());
    }

    #[inline]
    pub fn set_velocity_limit_m_s(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_vel_lim(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_position_sensor_ref_raw(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_position_sensor_ref(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_position_sensor_ref(&mut self, v: PositionReference) {
        self.set_position_sensor_ref_raw(v as i32);
    }

    #[inline]
    pub fn set_no_aid_timeout_us(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_noaid_tout(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_no_aid_noise_m(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_noaid_noise(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_heading_innov_gate(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_hdg_gate(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_heading_noise_rad(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_head_noise(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_accel_bias_limit_m_s2(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_abl_lim(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_accel_bias_learning_accel_limit_m_s2(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_abl_acclim(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_accel_bias_learning_gyro_limit_rad_s(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_abl_gyrlim(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_accel_bias_learning_tau_s(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_abl_tau(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_gyro_bias_limit_rad_s(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gyr_b_lim(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_imu_pos_body_x(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_imu_pos_body_x(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_imu_pos_body_y(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_imu_pos_body_y(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_imu_pos_body_z(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_imu_pos_body_z(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_imu_pos_body(&mut self, v: [f32; 3]) {
        self.set_imu_pos_body_x(v[0]);
        self.set_imu_pos_body_y(v[1]);
        self.set_imu_pos_body_z(v[2]);
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn set_baro_noise(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_baro_noise(self.ekf_ptr, v) }
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn set_baro_ctrl(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_baro_ctrl(self.ekf_ptr, v) }
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn set_baro_control(&mut self, v: BaroControl) {
        self.set_baro_ctrl(v as i32);
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn set_baro_delay_ms(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_baro_delay(self.ekf_ptr, v) }
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn set_baro_innov_gate(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_baro_gate(self.ekf_ptr, v) }
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn set_baro_bias_nsd(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_baro_bias_nsd(self.ekf_ptr, v) }
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn set_ground_effect_deadzone_m(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gnd_eff_dz(self.ekf_ptr, v) }
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn set_ground_effect_max_hgt_m(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gnd_max_hgt(self.ekf_ptr, v) }
    }

    #[cfg(feature = "baro-compensation")]
    #[inline]
    pub fn set_baro_pcoef_xp(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_pcoef_xp(self.ekf_ptr, v) }
    }

    #[cfg(feature = "baro-compensation")]
    #[inline]
    pub fn set_baro_pcoef_xn(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_pcoef_xn(self.ekf_ptr, v) }
    }

    #[cfg(feature = "baro-compensation")]
    #[inline]
    pub fn set_baro_pcoef_yp(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_pcoef_yp(self.ekf_ptr, v) }
    }

    #[cfg(feature = "baro-compensation")]
    #[inline]
    pub fn set_baro_pcoef_yn(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_pcoef_yn(self.ekf_ptr, v) }
    }

    #[cfg(feature = "baro-compensation")]
    #[inline]
    pub fn set_baro_pcoef_z(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_pcoef_z(self.ekf_ptr, v) }
    }

    #[cfg(feature = "baro-compensation")]
    #[inline]
    pub fn set_baro_aspd_max_m_s(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_aspd_max(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_pos_noise(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gps_pos_noise(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_vel_noise(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gps_vel_noise(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_pos_gate(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gps_pos_gate(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_vel_gate(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gps_vel_gate(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_mode_raw(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_gps_mode(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_mode(&mut self, v: GnssMode) {
        self.set_gps_mode_raw(v as i32);
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_delay_ms(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gps_delay(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_pos_body_x(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gps_pos_body_x(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_pos_body_y(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gps_pos_body_y(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_pos_body_z(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gps_pos_body_z(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_pos_body(&mut self, v: [f32; 3]) {
        self.set_gps_pos_body_x(v[0]);
        self.set_gps_pos_body_y(v[1]);
        self.set_gps_pos_body_z(v[2]);
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_hgt_bias_nsd(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gps_hgt_bias_nsd(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_ctrl(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_gps_ctrl(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_control(&mut self, v: GnssControl) {
        self.set_gps_ctrl(v.bits());
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_check_mask(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_gps_check(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_check_control(&mut self, v: GnssCheckControl) {
        self.set_gps_check_mask(v.bits());
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_required_eph_m(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_req_eph(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_required_epv_m(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_req_epv(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_required_sacc_m_s(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_req_sacc(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_required_nsats(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_req_nsats(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_required_pdop(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_req_pdop(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_required_h_drift_m_s(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_req_hdrift(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_required_v_drift_m_s(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_req_vdrift(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_required_fix_type(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_req_fix(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_required_fix(&mut self, v: GnssFixType) {
        self.set_required_fix_type(v as i32);
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gsf_tas_default_m_s(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gsf_tas(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gnss-yaw")]
    #[inline]
    pub fn set_gnss_heading_noise(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_gnss_heading_noise(self.ekf_ptr, v) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_mag_noise(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_mag_noise(self.ekf_ptr, v) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_mag_declination_deg(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_mag_decl(self.ekf_ptr, v) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_mag_innov_gate(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_mag_gate(self.ekf_ptr, v) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_mag_delay_ms(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_mag_delay(self.ekf_ptr, v) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_mag_decl_type_raw(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_decl_type(self.ekf_ptr, v) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_mag_decl_type(&mut self, v: MagDeclinationControl) {
        self.set_mag_decl_type_raw(v.bits());
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_mag_fusion_type_raw(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_mag_type(self.ekf_ptr, v) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_mag_fusion_type(&mut self, v: MagFusionType) {
        self.set_mag_fusion_type_raw(v as i32);
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_mag_check_mask_raw(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_mag_check(self.ekf_ptr, v) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_mag_check_mask(&mut self, v: MagCheckControl) {
        self.set_mag_check_mask_raw(v.bits());
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_mag_check_strength_gauss(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_mag_chk_str(self.ekf_ptr, v) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_mag_check_inclination_deg(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_mag_chk_inc(self.ekf_ptr, v) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_mag_earth_process_noise_gauss_s(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_mag_e_noise(self.ekf_ptr, v) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_mag_body_process_noise_gauss_s(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_mag_b_noise(self.ekf_ptr, v) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_mag_accel_limit_m_s2(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_mag_acclim(self.ekf_ptr, v) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_synthetic_mag_z_enabled(&mut self, enabled: bool) {
        unsafe { ffi::ekf2_param_set_synt_mag_z(self.ekf_ptr, enabled as i32) }
    }

    #[inline]
    pub fn set_hgt_ref_raw(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_hgt_ref(self.ekf_ptr, v) }
    }

    #[inline]
    pub fn set_hgt_ref(&mut self, v: HeightReference) {
        self.set_hgt_ref_raw(v as i32);
    }

    #[cfg(feature = "airspeed")]
    #[inline]
    pub fn set_eas_noise(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_eas_noise(self.ekf_ptr, v) }
    }

    #[cfg(feature = "airspeed")]
    #[inline]
    pub fn set_airspeed_delay_ms(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_asp_delay(self.ekf_ptr, v) }
    }

    #[cfg(feature = "airspeed")]
    #[inline]
    pub fn set_tas_gate(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_tas_gate(self.ekf_ptr, v) }
    }

    #[cfg(feature = "airspeed")]
    #[inline]
    pub fn set_airspeed_fusion_threshold_m_s(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_arsp_thr(self.ekf_ptr, v) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_ctrl_raw(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_rng_ctrl(self.ekf_ptr, v) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_ctrl(&mut self, v: RangeControl) {
        self.set_range_ctrl_raw(v as i32);
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_delay_ms(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_rng_delay(self.ekf_ptr, v) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_noise_m(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_rng_noise(self.ekf_ptr, v) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_gate(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_rng_gate(self.ekf_ptr, v) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_pitch_rad(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_rng_pitch(self.ekf_ptr, v) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_scale_per_meter(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_rng_sfe(self.ekf_ptr, v) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_aid_max_hgt_m(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_rng_a_hmax(self.ekf_ptr, v) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_aid_max_vel_m_s(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_rng_a_vmax(self.ekf_ptr, v) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_quality_min_duration_s(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_rng_qlty_t(self.ekf_ptr, v) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_kinematic_gate(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_rng_k_gate(self.ekf_ptr, v) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_fog_distance_m(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_rng_fog(self.ekf_ptr, v) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_pos_body_x(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_rng_pos_body_x(self.ekf_ptr, v) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_pos_body_y(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_rng_pos_body_y(self.ekf_ptr, v) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_pos_body_z(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_rng_pos_body_z(self.ekf_ptr, v) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_pos_body(&mut self, v: [f32; 3]) {
        self.set_range_pos_body_x(v[0]);
        self.set_range_pos_body_y(v[1]);
        self.set_range_pos_body_z(v[2]);
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn set_ev_ctrl_raw(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_ev_ctrl(self.ekf_ptr, v) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn set_ev_ctrl(&mut self, v: ExternalVisionControl) {
        self.set_ev_ctrl_raw(v.bits());
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn set_ev_delay_ms(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_ev_delay(self.ekf_ptr, v) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn set_ev_vel_noise(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_evv_noise(self.ekf_ptr, v) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn set_ev_pos_noise(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_evp_noise(self.ekf_ptr, v) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn set_ev_att_noise(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_eva_noise(self.ekf_ptr, v) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn set_ev_quality_min(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_ev_qmin(self.ekf_ptr, v) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn set_ev_vel_gate(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_evv_gate(self.ekf_ptr, v) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn set_ev_pos_gate(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_evp_gate(self.ekf_ptr, v) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn set_ev_hgt_bias_nsd(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_ev_hgt_bias_nsd(self.ekf_ptr, v) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn set_ev_pos_body_x(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_ev_pos_body_x(self.ekf_ptr, v) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn set_ev_pos_body_y(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_ev_pos_body_y(self.ekf_ptr, v) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn set_ev_pos_body_z(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_ev_pos_body_z(self.ekf_ptr, v) }
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn set_ev_pos_body(&mut self, v: [f32; 3]) {
        self.set_ev_pos_body_x(v[0]);
        self.set_ev_pos_body_y(v[1]);
        self.set_ev_pos_body_z(v[2]);
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn set_of_ctrl_raw(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_of_ctrl(self.ekf_ptr, v) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn set_of_ctrl(&mut self, v: OpticalFlowControl) {
        self.set_of_ctrl_raw(v as i32);
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn set_of_gyro_source_raw(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_of_gyr_src(self.ekf_ptr, v) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn set_of_gyro_source(&mut self, v: FlowGyroSource) {
        self.set_of_gyro_source_raw(v as i32);
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn set_of_delay_ms(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_of_delay(self.ekf_ptr, v) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn set_of_noise_min(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_of_n_min(self.ekf_ptr, v) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn set_of_noise_max(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_of_n_max(self.ekf_ptr, v) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn set_of_quality_min(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_of_qmin(self.ekf_ptr, v) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn set_of_quality_min_ground(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_of_qmin_gnd(self.ekf_ptr, v) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn set_of_gate(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_of_gate(self.ekf_ptr, v) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn set_flow_pos_body_x(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_flow_pos_body_x(self.ekf_ptr, v) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn set_flow_pos_body_y(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_flow_pos_body_y(self.ekf_ptr, v) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn set_flow_pos_body_z(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_flow_pos_body_z(self.ekf_ptr, v) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn set_flow_pos_body(&mut self, v: [f32; 3]) {
        self.set_flow_pos_body_x(v[0]);
        self.set_flow_pos_body_y(v[1]);
        self.set_flow_pos_body_z(v[2]);
    }

    #[cfg(feature = "wind")]
    #[inline]
    pub fn set_wind_nsd(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_wind_nsd(self.ekf_ptr, v) }
    }

    #[cfg(feature = "sideslip")]
    #[inline]
    pub fn set_sideslip_fusion_raw(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_fuse_beta(self.ekf_ptr, v) }
    }

    #[cfg(feature = "sideslip")]
    #[inline]
    pub fn set_sideslip_fusion_enabled(&mut self, enabled: bool) {
        self.set_sideslip_fusion_raw(enabled as i32);
    }

    #[cfg(feature = "sideslip")]
    #[inline]
    pub fn set_sideslip_gate(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_beta_gate(self.ekf_ptr, v) }
    }

    #[cfg(feature = "sideslip")]
    #[inline]
    pub fn set_sideslip_noise_rad(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_beta_noise(self.ekf_ptr, v) }
    }

    #[cfg(feature = "terrain")]
    #[inline]
    pub fn set_terrain_noise(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_terr_noise(self.ekf_ptr, v) }
    }

    #[cfg(feature = "terrain")]
    #[inline]
    pub fn set_terrain_gradient(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_terr_grad(self.ekf_ptr, v) }
    }

    #[cfg(any(
        feature = "terrain",
        feature = "range-finder",
        feature = "optical-flow"
    ))]
    #[inline]
    pub fn set_min_range_m(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_min_rng(self.ekf_ptr, v) }
    }

    #[cfg(feature = "gravity-fusion")]
    #[inline]
    pub fn set_gravity_noise_m_s2(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_grav_noise(self.ekf_ptr, v) }
    }

    #[cfg(feature = "drag-fusion")]
    #[inline]
    pub fn set_drag_ctrl_raw(&mut self, v: i32) {
        unsafe { ffi::ekf2_param_set_drag_ctrl(self.ekf_ptr, v) }
    }

    #[cfg(feature = "drag-fusion")]
    #[inline]
    pub fn set_drag_fusion_enabled(&mut self, enabled: bool) {
        self.set_drag_ctrl_raw(enabled as i32);
    }

    #[cfg(feature = "drag-fusion")]
    #[inline]
    pub fn set_drag_noise(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_drag_noise(self.ekf_ptr, v) }
    }

    #[cfg(feature = "drag-fusion")]
    #[inline]
    pub fn set_drag_bcoef_x(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_bcoef_x(self.ekf_ptr, v) }
    }

    #[cfg(feature = "drag-fusion")]
    #[inline]
    pub fn set_drag_bcoef_y(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_bcoef_y(self.ekf_ptr, v) }
    }

    #[cfg(feature = "drag-fusion")]
    #[inline]
    pub fn set_drag_mcoef(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_mcoef(self.ekf_ptr, v) }
    }

    #[cfg(feature = "aux-vel")]
    #[inline]
    pub fn set_aux_vel_delay_ms(&mut self, v: f32) {
        unsafe { ffi::ekf2_param_set_avel_delay(self.ekf_ptr, v) }
    }
}
