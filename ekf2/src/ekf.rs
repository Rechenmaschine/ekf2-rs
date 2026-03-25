//! Heap-allocated PX4 EKF2 filter handle.

use core::marker::PhantomData;
use core::ptr::NonNull;
use ekf2_sys as ffi;

use crate::error::EkfError;
use crate::params::{Params, ParamsMut};
use crate::types::ImuSample;

/// An initialized PX4 EKF2 filter instance.
///
/// The C++ `Ekf` object is allocated and owned by `ekf2-sys` on the C++
/// heap. `Drop` calls the C++ destructor and frees object memory.
///
/// C++ allocations are routed through Rust's allocator symbols via the
/// `operator new`/`delete` bridge in `allocator.cpp`. On bare-metal targets
/// you must configure a global allocator (e.g. via `embedded-alloc`).
///
/// # Example
///
/// ```no_run
/// use ekf2::{Ekf, types::ImuSample};
///
/// let mut ekf = Ekf::new(0).expect("EKF init failed");
/// let imu = ImuSample::new(10_000, [0.0; 3], [0.0, 0.0, -9.81 * 0.01], 0.01, 0.01);
/// ekf.set_imu_data(&imu);
/// let _ = ekf.update();
/// ```
pub struct Ekf {
    ptr: NonNull<core::ffi::c_void>,
    _not_send_sync: PhantomData<*mut ()>,
}

impl Ekf {
    /// Create and initialize a new EKF2 filter instance.
    ///
    /// Allocates the C++ object and calls `Ekf::init(timestamp_us)`.
    pub fn new(timestamp_us: u64) -> Result<Self, EkfError> {
        let ptr = unsafe { ffi::ekf2_create_heap() };
        if ptr.is_null() {
            return Err(EkfError::AllocFailed);
        }

        // SAFETY: ptr is a valid, constructed Ekf object.
        let ok = unsafe { ffi::ekf2_init(ptr, timestamp_us) };
        if !ok {
            unsafe { ffi::ekf2_destroy_heap(ptr) };
            return Err(EkfError::InitFailed);
        }

        Ok(Self {
            ptr: NonNull::new(ptr).unwrap(),
            _not_send_sync: PhantomData,
        })
    }

    #[inline]
    fn ptr(&self) -> *mut core::ffi::c_void {
        self.ptr.as_ptr()
    }

    #[inline]
    fn ptr_const(&self) -> *const core::ffi::c_void {
        self.ptr.as_ptr() as *const _
    }

    #[inline]
    pub fn params(&self) -> Params<'_> {
        Params {
            ekf_ptr: self.ptr_const(),
            _marker: PhantomData,
        }
    }

    #[inline]
    pub fn params_mut(&mut self) -> ParamsMut<'_> {
        ParamsMut {
            ekf_ptr: self.ptr(),
            _marker: PhantomData,
        }
    }

    /// Run one filter update cycle.
    pub fn update(&mut self) -> Result<(), EkfError> {
        let ok = unsafe { ffi::ekf2_update(self.ptr()) };
        if ok {
            Ok(())
        } else {
            Err(EkfError::UpdateFailed)
        }
    }

    #[inline]
    pub fn set_imu_data(&mut self, sample: &ImuSample) {
        unsafe { ffi::ekf2_set_imu_data(self.ptr(), sample as *const _) };
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_data(&mut self, sample: &crate::types::GnssSample) {
        unsafe { ffi::ekf2_set_gps_data(self.ptr(), sample as *const _) };
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn set_mag_data(&mut self, sample: &crate::types::MagSample) {
        unsafe { ffi::ekf2_set_mag_data(self.ptr(), sample as *const _) };
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn set_baro_data(&mut self, sample: &crate::types::BaroSample) {
        unsafe { ffi::ekf2_set_baro_data(self.ptr(), sample as *const _) };
    }

    #[cfg(feature = "airspeed")]
    #[inline]
    pub fn set_airspeed_data(&mut self, sample: &crate::types::AirspeedSample) {
        unsafe { ffi::ekf2_set_airspeed_data(self.ptr(), sample as *const _) };
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_range_data(&mut self, sample: &crate::types::RangeSample) {
        unsafe { ffi::ekf2_set_range_data(self.ptr(), sample as *const _) };
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn set_optical_flow_data(&mut self, sample: &crate::types::FlowSample) {
        unsafe { ffi::ekf2_set_optical_flow_data(self.ptr(), sample as *const _) };
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn set_ev_data(&mut self, sample: &crate::types::ExtVisionSample) {
        unsafe { ffi::ekf2_set_ev_data(self.ptr(), sample as *const _) };
    }

    #[cfg(feature = "aux-vel")]
    #[inline]
    pub fn set_aux_vel_data(&mut self, sample: &crate::types::AuxVelSample) {
        unsafe { ffi::ekf2_set_aux_vel_data(self.ptr(), sample as *const _) };
    }

    #[inline]
    pub fn set_system_flag_data(&mut self, sample: &crate::types::SystemFlagUpdate) {
        unsafe { ffi::ekf2_set_system_flag_data(self.ptr(), sample as *const _) };
    }

    #[inline]
    pub fn set_in_air_status(&mut self, in_air: bool) {
        unsafe { ffi::ekf2_set_in_air_status(self.ptr(), in_air) };
    }

    #[inline]
    pub fn set_vehicle_at_rest(&mut self, at_rest: bool) {
        unsafe { ffi::ekf2_set_vehicle_at_rest(self.ptr(), at_rest) };
    }

    #[inline]
    pub fn set_constant_pos(&mut self, constant_pos: bool) {
        unsafe { ffi::ekf2_set_constant_pos(self.ptr(), constant_pos) };
    }

    #[inline]
    pub fn set_is_fixed_wing(&mut self, is_fixed_wing: bool) {
        unsafe { ffi::ekf2_set_is_fixed_wing(self.ptr(), is_fixed_wing) };
    }

    #[inline]
    pub fn set_in_transition_to_fw(&mut self, in_transition_to_fw: bool) {
        unsafe { ffi::ekf2_set_in_transition_to_fw(self.ptr(), in_transition_to_fw) };
    }

    #[inline]
    pub fn set_gnd_effect(&mut self) {
        unsafe { ffi::ekf2_set_gnd_effect(self.ptr()) };
    }

    #[inline]
    pub fn set_air_density(&mut self, air_density: f32) {
        unsafe { ffi::ekf2_set_air_density(self.ptr(), air_density) };
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_gps_data_with_pps(
        &mut self,
        sample: &crate::types::GnssSample,
        pps_compensation: bool,
    ) {
        unsafe {
            ffi::ekf2_set_gps_data_with_pps(self.ptr(), sample as *const _, pps_compensation)
        };
    }

    #[cfg(feature = "airspeed")]
    #[inline]
    pub fn set_synthetic_airspeed(&mut self, synthetic_airspeed: bool) {
        unsafe { ffi::ekf2_set_synthetic_airspeed(self.ptr(), synthetic_airspeed) };
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn set_rangefinder_limits(&mut self, min_distance: f32, max_distance: f32) {
        unsafe { ffi::ekf2_set_rangefinder_limits(self.ptr(), min_distance, max_distance) };
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn set_optical_flow_limits(
        &mut self,
        max_flow_rate: f32,
        min_distance: f32,
        max_distance: f32,
    ) {
        unsafe {
            ffi::ekf2_set_optical_flow_limits(self.ptr(), max_flow_rate, min_distance, max_distance)
        };
    }

    #[inline]
    pub fn quaternion(&self) -> [f32; 4] {
        let mut q = [0.0f32; 4];
        unsafe { ffi::ekf2_get_quaternion(self.ptr_const(), q.as_mut_ptr()) };
        q
    }

    #[inline]
    pub fn velocity_ned(&self) -> [f32; 3] {
        let mut v = [0.0f32; 3];
        unsafe { ffi::ekf2_get_velocity(self.ptr_const(), v.as_mut_ptr()) };
        v
    }

    #[inline]
    pub fn position_ned(&self) -> [f32; 3] {
        let mut p = [0.0f32; 3];
        unsafe { ffi::ekf2_get_position(self.ptr_const(), p.as_mut_ptr()) };
        p
    }

    #[inline]
    pub fn gyro_bias(&self) -> [f32; 3] {
        let mut b = [0.0f32; 3];
        unsafe { ffi::ekf2_get_gyro_bias(self.ptr_const(), b.as_mut_ptr()) };
        b
    }

    #[inline]
    pub fn accel_bias(&self) -> [f32; 3] {
        let mut b = [0.0f32; 3];
        unsafe { ffi::ekf2_get_accel_bias(self.ptr_const(), b.as_mut_ptr()) };
        b
    }

    #[inline]
    pub fn angular_velocity_reset_accumulator(&mut self) -> [f32; 3] {
        let mut w = [0.0f32; 3];
        unsafe { ffi::ekf2_get_angular_velocity_reset_accumulator(self.ptr(), w.as_mut_ptr()) };
        w
    }

    #[inline]
    pub fn unaided_yaw(&self) -> f32 {
        unsafe { ffi::ekf2_get_unaided_yaw(self.ptr_const()) }
    }

    #[inline]
    pub fn attitude_valid(&self) -> bool {
        unsafe { ffi::ekf2_attitude_valid(self.ptr_const()) }
    }

    #[inline]
    pub fn local_position_valid(&self) -> bool {
        unsafe { ffi::ekf2_local_position_valid(self.ptr_const()) }
    }

    #[inline]
    pub fn local_velocity_valid(&self) -> bool {
        unsafe { ffi::ekf2_local_velocity_valid(self.ptr_const()) }
    }

    #[inline]
    pub fn global_position_valid(&self) -> bool {
        unsafe { ffi::ekf2_global_position_valid(self.ptr_const()) }
    }

    #[inline]
    pub fn yaw_align_complete(&self) -> bool {
        unsafe { ffi::ekf2_yaw_align_complete(self.ptr_const()) }
    }

    #[inline]
    pub fn pos_variance(&self) -> [f32; 3] {
        let mut v = [0.0f32; 3];
        unsafe { ffi::ekf2_get_pos_variance(self.ptr_const(), v.as_mut_ptr()) };
        v
    }

    #[inline]
    pub fn vel_variance(&self) -> [f32; 3] {
        let mut v = [0.0f32; 3];
        unsafe { ffi::ekf2_get_vel_variance(self.ptr_const(), v.as_mut_ptr()) };
        v
    }

    #[inline]
    pub fn gyro_bias_variance(&self) -> [f32; 3] {
        let mut v = [0.0f32; 3];
        unsafe { ffi::ekf2_get_gyro_bias_variance(self.ptr_const(), v.as_mut_ptr()) };
        v
    }

    #[inline]
    pub fn accel_bias_variance(&self) -> [f32; 3] {
        let mut v = [0.0f32; 3];
        unsafe { ffi::ekf2_get_accel_bias_variance(self.ptr_const(), v.as_mut_ptr()) };
        v
    }

    #[inline]
    pub fn velocity_derivative(&self) -> [f32; 3] {
        let mut v = [0.0f32; 3];
        unsafe { ffi::ekf2_get_velocity_derivative(self.ptr_const(), v.as_mut_ptr()) };
        v
    }

    #[inline]
    pub fn reset_velocity_derivative_accumulation(&mut self) {
        unsafe { ffi::ekf2_reset_velocity_derivative_accumulation(self.ptr()) };
    }

    #[inline]
    pub fn vertical_position_derivative(&self) -> f32 {
        unsafe { ffi::ekf2_get_vertical_position_derivative(self.ptr_const()) }
    }

    #[inline]
    pub fn lat_lon_alt(&self) -> (f64, f64, f32) {
        let (mut lat, mut lon, mut alt) = (0.0f64, 0.0f64, 0.0f32);
        unsafe { ffi::ekf2_get_lat_lon_alt(self.ptr_const(), &mut lat, &mut lon, &mut alt) };
        (lat, lon, alt)
    }

    #[inline]
    pub fn output_tracking_error(&self) -> [f32; 3] {
        let mut e = [0.0f32; 3];
        unsafe { ffi::ekf2_get_output_tracking_error(self.ptr_const(), e.as_mut_ptr()) };
        e
    }

    #[inline]
    pub fn control_status_raw(&self) -> u64 {
        unsafe { ffi::ekf2_get_control_status(self.ptr_const()) }
    }

    #[inline]
    pub fn control_status(&self) -> crate::params::ControlStatus {
        crate::params::ControlStatus::from_bits_truncate(self.control_status_raw())
    }

    #[inline]
    pub fn fault_status_raw(&self) -> u32 {
        unsafe { ffi::ekf2_get_fault_status(self.ptr_const()) }
    }

    #[inline]
    pub fn fault_status(&self) -> crate::params::FaultStatus {
        crate::params::FaultStatus::from_bits_truncate(self.fault_status_raw())
    }

    #[inline]
    pub fn information_event_status_raw(&self) -> u32 {
        unsafe { ffi::ekf2_get_information_event_status(self.ptr_const()) }
    }

    #[inline]
    pub fn information_event_status(&self) -> crate::params::InformationEventStatus {
        crate::params::InformationEventStatus::from_bits_truncate(
            self.information_event_status_raw(),
        )
    }

    #[inline]
    pub fn clear_information_events(&mut self) {
        unsafe { ffi::ekf2_clear_information_events(self.ptr()) };
    }

    #[inline]
    pub fn heading_innov_test_ratio(&self) -> f32 {
        unsafe { ffi::ekf2_get_heading_innov_test_ratio(self.ptr_const()) }
    }

    #[inline]
    pub fn horiz_vel_innov_test_ratio(&self) -> f32 {
        unsafe { ffi::ekf2_get_horiz_vel_innov_test_ratio(self.ptr_const()) }
    }

    #[inline]
    pub fn vert_vel_innov_test_ratio(&self) -> f32 {
        unsafe { ffi::ekf2_get_vert_vel_innov_test_ratio(self.ptr_const()) }
    }

    #[inline]
    pub fn horiz_pos_innov_test_ratio(&self) -> f32 {
        unsafe { ffi::ekf2_get_horiz_pos_innov_test_ratio(self.ptr_const()) }
    }

    #[inline]
    pub fn vert_pos_innov_test_ratio(&self) -> f32 {
        unsafe { ffi::ekf2_get_vert_pos_innov_test_ratio(self.ptr_const()) }
    }

    #[cfg(feature = "airspeed")]
    #[inline]
    pub fn airspeed_innov_test_ratio(&self) -> f32 {
        unsafe { ffi::ekf2_get_airspeed_innov_test_ratio(self.ptr_const()) }
    }

    #[cfg(feature = "sideslip")]
    #[inline]
    pub fn sideslip_innov_test_ratio(&self) -> f32 {
        unsafe { ffi::ekf2_get_sideslip_innov_test_ratio(self.ptr_const()) }
    }

    #[cfg(any(
        feature = "terrain",
        feature = "range-finder",
        feature = "optical-flow"
    ))]
    #[inline]
    pub fn hagl_innov_test_ratio(&self) -> f32 {
        unsafe { ffi::ekf2_get_hagl_innov_test_ratio(self.ptr_const()) }
    }

    #[inline]
    pub fn soln_status(&self) -> crate::params::SolnStatus {
        let raw = unsafe { ffi::ekf2_get_soln_status(self.ptr_const()) };
        crate::params::SolnStatus::from_bits_truncate(raw)
    }

    #[inline]
    pub fn height_sensor_ref(&self) -> ffi::HeightSensor {
        ffi::HeightSensor::from_raw(unsafe { ffi::ekf2_get_height_sensor_ref(self.ptr_const()) })
    }

    #[inline]
    pub fn lpos_accuracy(&self) -> (f32, f32) {
        let (mut eph, mut epv) = (0.0f32, 0.0f32);
        unsafe { ffi::ekf2_get_lpos_accuracy(self.ptr_const(), &mut eph, &mut epv) };
        (eph, epv)
    }

    #[inline]
    pub fn gpos_accuracy(&self) -> (f32, f32) {
        let (mut eph, mut epv) = (0.0f32, 0.0f32);
        unsafe { ffi::ekf2_get_gpos_accuracy(self.ptr_const(), &mut eph, &mut epv) };
        (eph, epv)
    }

    #[inline]
    pub fn vel_accuracy(&self) -> (f32, f32) {
        let (mut evh, mut evv) = (0.0f32, 0.0f32);
        unsafe { ffi::ekf2_get_vel_accuracy(self.ptr_const(), &mut evh, &mut evv) };
        (evh, evv)
    }

    #[inline]
    pub fn ctrl_limits(&self) -> (f32, f32, f32, f32, f32) {
        let (mut vxy_max, mut vz_max, mut hagl_min, mut hagl_max_z, mut hagl_max_xy) =
            (0.0f32, 0.0f32, 0.0f32, 0.0f32, 0.0f32);
        unsafe {
            ffi::ekf2_get_ctrl_limits(
                self.ptr_const(),
                &mut vxy_max,
                &mut vz_max,
                &mut hagl_min,
                &mut hagl_max_z,
                &mut hagl_max_xy,
            )
        };
        (vxy_max, vz_max, hagl_min, hagl_max_z, hagl_max_xy)
    }

    #[inline]
    pub fn global_origin(&self) -> ffi::GlobalOrigin {
        let mut o = ffi::GlobalOrigin::default();
        unsafe {
            ffi::ekf2_get_global_origin(
                self.ptr_const(),
                &mut o.time_us,
                &mut o.lat,
                &mut o.lon,
                &mut o.alt,
            )
        };
        o
    }

    #[inline]
    pub fn set_global_origin(
        &mut self,
        lat: f64,
        lon: f64,
        alt: f32,
        hpos_var: f32,
        vpos_var: f32,
    ) -> Result<(), EkfError> {
        let ok =
            unsafe { ffi::ekf2_set_global_origin(self.ptr(), lat, lon, alt, hpos_var, vpos_var) };
        if ok { Ok(()) } else { Err(EkfError::OperationFailed) }
    }

    #[inline]
    pub fn reset_global_position(
        &mut self,
        lat: f64,
        lon: f64,
        alt: f32,
        hpos_var: f32,
        vpos_var: f32,
    ) -> Result<(), EkfError> {
        let ok = unsafe {
            ffi::ekf2_reset_global_position(self.ptr(), lat, lon, alt, hpos_var, vpos_var)
        };
        if ok { Ok(()) } else { Err(EkfError::OperationFailed) }
    }

    #[inline]
    pub fn reset_global_position_to_external_observation(
        &mut self,
        lat: f64,
        lon: f64,
        alt: f32,
        eph: f32,
        epv: f32,
        timestamp_observation: u64,
    ) -> Result<(), EkfError> {
        let ok = unsafe {
            ffi::ekf2_reset_global_position_to_external_observation(
                self.ptr(),
                lat,
                lon,
                alt,
                eph,
                epv,
                timestamp_observation,
            )
        };
        if ok { Ok(()) } else { Err(EkfError::OperationFailed) }
    }

    #[inline]
    pub fn quat_reset_count(&self) -> u8 {
        unsafe { ffi::ekf2_get_quat_reset_count(self.ptr_const()) }
    }

    #[inline]
    pub fn pos_ne_reset_count(&self) -> u8 {
        unsafe { ffi::ekf2_get_pos_ne_reset_count(self.ptr_const()) }
    }

    #[inline]
    pub fn vel_ne_reset_count(&self) -> u8 {
        unsafe { ffi::ekf2_get_vel_ne_reset_count(self.ptr_const()) }
    }

    #[inline]
    pub fn pos_d_reset_count(&self) -> u8 {
        unsafe { ffi::ekf2_get_pos_d_reset_count(self.ptr_const()) }
    }

    #[inline]
    pub fn vel_d_reset_count(&self) -> u8 {
        unsafe { ffi::ekf2_get_vel_d_reset_count(self.ptr_const()) }
    }

    #[inline]
    pub fn hagl_reset_count(&self) -> u8 {
        unsafe { ffi::ekf2_get_hagl_reset_count(self.ptr_const()) }
    }

    #[inline]
    pub fn quat_reset(&self) -> ([f32; 4], u8) {
        let mut delta = [0.0f32; 4];
        let mut count = 0u8;
        unsafe { ffi::ekf2_get_quat_reset(self.ptr_const(), delta.as_mut_ptr(), &mut count) };
        (delta, count)
    }

    #[inline]
    pub fn pos_ne_reset(&self) -> ([f32; 2], u8) {
        let mut delta = [0.0f32; 2];
        let mut count = 0u8;
        unsafe { ffi::ekf2_get_pos_ne_reset(self.ptr_const(), delta.as_mut_ptr(), &mut count) };
        (delta, count)
    }

    #[inline]
    pub fn vel_ne_reset(&self) -> ([f32; 2], u8) {
        let mut delta = [0.0f32; 2];
        let mut count = 0u8;
        unsafe { ffi::ekf2_get_vel_ne_reset(self.ptr_const(), delta.as_mut_ptr(), &mut count) };
        (delta, count)
    }

    #[inline]
    pub fn pos_d_reset(&self) -> (f32, u8) {
        let mut delta = 0.0f32;
        let mut count = 0u8;
        unsafe { ffi::ekf2_get_pos_d_reset(self.ptr_const(), &mut delta, &mut count) };
        (delta, count)
    }

    #[inline]
    pub fn vel_d_reset(&self) -> (f32, u8) {
        let mut delta = 0.0f32;
        let mut count = 0u8;
        unsafe { ffi::ekf2_get_vel_d_reset(self.ptr_const(), &mut delta, &mut count) };
        (delta, count)
    }

    #[inline]
    pub fn hagl_reset(&self) -> (f32, u8) {
        let mut delta = 0.0f32;
        let mut count = 0u8;
        unsafe { ffi::ekf2_get_hagl_reset(self.ptr_const(), &mut delta, &mut count) };
        (delta, count)
    }

    #[inline]
    pub fn reset_gyro_bias(&mut self) {
        unsafe { ffi::ekf2_reset_gyro_bias(self.ptr()) };
    }

    #[inline]
    pub fn reset_accel_bias(&mut self) {
        unsafe { ffi::ekf2_reset_accel_bias(self.ptr()) };
    }

    #[inline]
    pub fn reset_heading_to_external_observation(&mut self, heading: f32, heading_accuracy: f32) {
        unsafe {
            ffi::ekf2_reset_heading_to_external_observation(self.ptr(), heading, heading_accuracy)
        };
    }

    #[cfg(feature = "wind")]
    #[inline]
    pub fn reset_wind_to_external_observation(
        &mut self,
        wind_speed: f32,
        wind_direction: f32,
        wind_speed_accuracy: f32,
        wind_direction_accuracy: f32,
    ) {
        unsafe {
            ffi::ekf2_reset_wind_to_external_observation(
                self.ptr(),
                wind_speed,
                wind_direction,
                wind_speed_accuracy,
                wind_direction_accuracy,
            )
        };
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn update_world_magnetic_model(&mut self, latitude_deg: f64, longitude_deg: f64) -> bool {
        unsafe { ffi::ekf2_update_world_magnetic_model(self.ptr(), latitude_deg, longitude_deg) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_horizontal_position_drift_rate_m_s(&self) -> f32 {
        unsafe { ffi::ekf2_get_gps_horizontal_position_drift_rate(self.ptr_const()) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_vertical_position_drift_rate_m_s(&self) -> f32 {
        unsafe { ffi::ekf2_get_gps_vertical_position_drift_rate(self.ptr_const()) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_filtered_horizontal_velocity_m_s(&self) -> f32 {
        unsafe { ffi::ekf2_get_gps_filtered_horizontal_velocity(self.ptr_const()) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_checks_passed(&self) -> bool {
        unsafe { ffi::ekf2_gps_checks_passed(self.ptr_const()) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_check_fail_status_raw(&self) -> u16 {
        unsafe { ffi::ekf2_get_gps_check_fail_status(self.ptr_const()) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_check_fail_status(&self) -> crate::params::GnssCheckFailStatus {
        crate::params::GnssCheckFailStatus::from_bits_truncate(self.gps_check_fail_status_raw())
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn yaw_emergency_estimate_available(&self) -> bool {
        unsafe { ffi::ekf2_yaw_emergency_estimate_available(self.ptr_const()) }
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn ekfgsf_data(
        &self,
        yaw_composite: &mut f32,
        yaw_variance: &mut f32,
        yaw: &mut [f32],
        innov_vn: &mut [f32],
        innov_ve: &mut [f32],
        weight: &mut [f32],
    ) -> Option<usize> {
        let capacity = yaw
            .len()
            .min(innov_vn.len())
            .min(innov_ve.len())
            .min(weight.len());
        let mut count = 0usize;
        let ok = unsafe {
            ffi::ekf2_get_ekfgsf_data(
                self.ptr_const(),
                yaw_composite,
                yaw_variance,
                yaw.as_mut_ptr(),
                innov_vn.as_mut_ptr(),
                innov_ve.as_mut_ptr(),
                weight.as_mut_ptr(),
                capacity,
                &mut count,
            )
        };
        if ok {
            Some(count)
        } else {
            None
        }
    }

    #[cfg(any(
        feature = "terrain",
        feature = "range-finder",
        feature = "optical-flow"
    ))]
    #[inline]
    pub fn terrain_valid(&self) -> bool {
        unsafe { ffi::ekf2_terrain_valid(self.ptr_const()) }
    }

    #[cfg(any(
        feature = "terrain",
        feature = "range-finder",
        feature = "optical-flow"
    ))]
    #[inline]
    pub fn terrain_vert_pos(&self) -> f32 {
        unsafe { ffi::ekf2_get_terrain_vert_pos(self.ptr_const()) }
    }

    #[cfg(any(
        feature = "terrain",
        feature = "range-finder",
        feature = "optical-flow"
    ))]
    #[inline]
    pub fn terrain_variance(&self) -> f32 {
        unsafe { ffi::ekf2_get_terrain_variance(self.ptr_const()) }
    }

    #[cfg(any(
        feature = "terrain",
        feature = "range-finder",
        feature = "optical-flow"
    ))]
    #[inline]
    pub fn hagl(&self) -> f32 {
        unsafe { ffi::ekf2_get_hagl(self.ptr_const()) }
    }

    #[inline]
    pub fn aid_src_fake_hgt(&self) -> ffi::EkfAidSource1d {
        let mut out = ffi::EkfAidSource1d::default();
        unsafe { ffi::ekf2_get_aid_src_fake_hgt(self.ptr_const(), &mut out) };
        out
    }

    #[inline]
    pub fn aid_src_fake_pos(&self) -> ffi::EkfAidSource2d {
        let mut out = ffi::EkfAidSource2d::default();
        unsafe { ffi::ekf2_get_aid_src_fake_pos(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn aid_src_baro_hgt(&self) -> ffi::EkfAidSource1d {
        let mut out = ffi::EkfAidSource1d::default();
        unsafe { ffi::ekf2_get_aid_src_baro_hgt(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn aid_src_gnss_hgt(&self) -> ffi::EkfAidSource1d {
        let mut out = ffi::EkfAidSource1d::default();
        unsafe { ffi::ekf2_get_aid_src_gnss_hgt(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn aid_src_gnss_pos(&self) -> ffi::EkfAidSource2d {
        let mut out = ffi::EkfAidSource2d::default();
        unsafe { ffi::ekf2_get_aid_src_gnss_pos(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn aid_src_gnss_vel(&self) -> ffi::EkfAidSource3d {
        let mut out = ffi::EkfAidSource3d::default();
        unsafe { ffi::ekf2_get_aid_src_gnss_vel(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "gnss-yaw")]
    #[inline]
    pub fn aid_src_gnss_yaw(&self) -> ffi::EkfAidSource1d {
        let mut out = ffi::EkfAidSource1d::default();
        unsafe { ffi::ekf2_get_aid_src_gnss_yaw(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn aid_src_mag(&self) -> ffi::EkfAidSource3d {
        let mut out = ffi::EkfAidSource3d::default();
        unsafe { ffi::ekf2_get_aid_src_mag(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "airspeed")]
    #[inline]
    pub fn aid_src_airspeed(&self) -> ffi::EkfAidSource1d {
        let mut out = ffi::EkfAidSource1d::default();
        unsafe { ffi::ekf2_get_aid_src_airspeed(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "sideslip")]
    #[inline]
    pub fn aid_src_sideslip(&self) -> ffi::EkfAidSource1d {
        let mut out = ffi::EkfAidSource1d::default();
        unsafe { ffi::ekf2_get_aid_src_sideslip(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn aid_src_rng_hgt(&self) -> ffi::EkfAidSource1d {
        let mut out = ffi::EkfAidSource1d::default();
        unsafe { ffi::ekf2_get_aid_src_rng_hgt(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn aid_src_optical_flow(&self) -> ffi::EkfAidSource2d {
        let mut out = ffi::EkfAidSource2d::default();
        unsafe { ffi::ekf2_get_aid_src_optical_flow(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn aid_src_ev_hgt(&self) -> ffi::EkfAidSource1d {
        let mut out = ffi::EkfAidSource1d::default();
        unsafe { ffi::ekf2_get_aid_src_ev_hgt(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn aid_src_ev_pos(&self) -> ffi::EkfAidSource2d {
        let mut out = ffi::EkfAidSource2d::default();
        unsafe { ffi::ekf2_get_aid_src_ev_pos(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn aid_src_ev_vel(&self) -> ffi::EkfAidSource3d {
        let mut out = ffi::EkfAidSource3d::default();
        unsafe { ffi::ekf2_get_aid_src_ev_vel(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn aid_src_ev_yaw(&self) -> ffi::EkfAidSource1d {
        let mut out = ffi::EkfAidSource1d::default();
        unsafe { ffi::ekf2_get_aid_src_ev_yaw(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "drag-fusion")]
    #[inline]
    pub fn aid_src_drag(&self) -> ffi::EkfAidSource2d {
        let mut out = ffi::EkfAidSource2d::default();
        unsafe { ffi::ekf2_get_aid_src_drag(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "gravity-fusion")]
    #[inline]
    pub fn aid_src_gravity(&self) -> ffi::EkfAidSource3d {
        let mut out = ffi::EkfAidSource3d::default();
        unsafe { ffi::ekf2_get_aid_src_gravity(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "aux-vel")]
    #[inline]
    pub fn aid_src_aux_vel(&self) -> ffi::EkfAidSource2d {
        let mut out = ffi::EkfAidSource2d::default();
        unsafe { ffi::ekf2_get_aid_src_aux_vel(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "wind")]
    #[inline]
    pub fn wind_velocity(&self) -> [f32; 2] {
        let mut w = [0.0f32; 2];
        unsafe { ffi::ekf2_get_wind_velocity(self.ptr_const(), w.as_mut_ptr()) };
        w
    }

    #[cfg(feature = "wind")]
    #[inline]
    pub fn wind_variance(&self) -> [f32; 2] {
        let mut v = [0.0f32; 2];
        unsafe { ffi::ekf2_get_wind_variance(self.ptr_const(), v.as_mut_ptr()) };
        v
    }

    #[cfg(feature = "wind")]
    #[inline]
    pub fn wind_estimate_active(&self) -> bool {
        unsafe { ffi::ekf2_wind_estimate_active(self.ptr_const()) }
    }

    #[inline]
    pub fn horizontal_aiding_active(&self) -> bool {
        unsafe { ffi::ekf2_horizontal_aiding_active(self.ptr_const()) }
    }

    #[inline]
    pub fn horizontal_position_aiding_active(&self) -> bool {
        unsafe { ffi::ekf2_horizontal_position_aiding_active(self.ptr_const()) }
    }

    #[inline]
    pub fn vertical_aiding_active(&self) -> bool {
        unsafe { ffi::ekf2_vertical_aiding_active(self.ptr_const()) }
    }

    #[inline]
    pub fn north_east_aiding_active(&self) -> bool {
        unsafe { ffi::ekf2_north_east_aiding_active(self.ptr_const()) }
    }

    #[inline]
    pub fn num_active_horizontal_aiding_sources(&self) -> i32 {
        unsafe { ffi::ekf2_num_active_horizontal_aiding_sources(self.ptr_const()) }
    }

    #[inline]
    pub fn num_active_horizontal_position_aiding_sources(&self) -> i32 {
        unsafe { ffi::ekf2_num_active_horizontal_position_aiding_sources(self.ptr_const()) }
    }

    #[inline]
    pub fn num_active_horizontal_velocity_aiding_sources(&self) -> i32 {
        unsafe { ffi::ekf2_num_active_horizontal_velocity_aiding_sources(self.ptr_const()) }
    }

    #[inline]
    pub fn vertical_position_aiding_active(&self) -> bool {
        unsafe { ffi::ekf2_vertical_position_aiding_active(self.ptr_const()) }
    }

    #[inline]
    pub fn num_active_vertical_position_aiding_sources(&self) -> i32 {
        unsafe { ffi::ekf2_num_active_vertical_position_aiding_sources(self.ptr_const()) }
    }

    #[inline]
    pub fn vertical_velocity_aiding_active(&self) -> bool {
        unsafe { ffi::ekf2_vertical_velocity_aiding_active(self.ptr_const()) }
    }

    #[inline]
    pub fn num_active_vertical_velocity_aiding_sources(&self) -> i32 {
        unsafe { ffi::ekf2_num_active_vertical_velocity_aiding_sources(self.ptr_const()) }
    }

    #[inline]
    pub fn dt_ekf_avg_s(&self) -> f32 {
        unsafe { ffi::ekf2_get_dt_ekf_avg(self.ptr_const()) }
    }

    #[inline]
    pub fn rot_var_body(&self) -> [f32; 3] {
        let mut v = [0.0f32; 3];
        unsafe { ffi::ekf2_get_rot_var_body(self.ptr_const(), v.as_mut_ptr()) };
        v
    }

    #[inline]
    pub fn rot_var_ned(&self) -> [f32; 3] {
        let mut v = [0.0f32; 3];
        unsafe { ffi::ekf2_get_rot_var_ned(self.ptr_const(), v.as_mut_ptr()) };
        v
    }

    #[inline]
    pub fn yaw_variance(&self) -> f32 {
        unsafe { ffi::ekf2_get_yaw_variance(self.ptr_const()) }
    }

    #[inline]
    pub fn tilt_variance(&self) -> f32 {
        unsafe { ffi::ekf2_get_tilt_variance(self.ptr_const()) }
    }

    #[inline]
    pub fn state_size() -> usize {
        unsafe { ffi::ekf2_get_state_size() }
    }

    #[inline]
    pub fn covariance_diagonal(&self, out: &mut [f32]) -> usize {
        unsafe { ffi::ekf2_get_covariances_diagonal(self.ptr_const(), out.as_mut_ptr(), out.len()) }
    }

    #[inline]
    pub fn state_covariance(&self, row: u32, col: u32) -> f32 {
        unsafe { ffi::ekf2_get_state_covariance(self.ptr_const(), row, col) }
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_earth_field(&self) -> [f32; 3] {
        let mut v = [0.0f32; 3];
        unsafe { ffi::ekf2_get_mag_earth_field(self.ptr_const(), v.as_mut_ptr()) };
        v
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_bias(&self) -> [f32; 3] {
        let mut v = [0.0f32; 3];
        unsafe { ffi::ekf2_get_mag_bias(self.ptr_const(), v.as_mut_ptr()) };
        v
    }

    #[cfg(feature = "magnetometer")]
    #[inline]
    pub fn mag_bias_variance(&self) -> [f32; 3] {
        let mut v = [0.0f32; 3];
        unsafe { ffi::ekf2_get_mag_bias_variance(self.ptr_const(), v.as_mut_ptr()) };
        v
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn hagl_rate_innov(&self) -> f32 {
        unsafe { ffi::ekf2_get_hagl_rate_innov(self.ptr_const()) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn hagl_rate_innov_var(&self) -> f32 {
        unsafe { ffi::ekf2_get_hagl_rate_innov_var(self.ptr_const()) }
    }

    #[cfg(feature = "range-finder")]
    #[inline]
    pub fn hagl_rate_innov_ratio(&self) -> f32 {
        unsafe { ffi::ekf2_get_hagl_rate_innov_ratio(self.ptr_const()) }
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn flow_vel_body(&self) -> [f32; 2] {
        let mut out = [0.0f32; 2];
        unsafe { ffi::ekf2_get_flow_vel_body(self.ptr_const(), out.as_mut_ptr()) };
        out
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn flow_vel_ne(&self) -> [f32; 2] {
        let mut out = [0.0f32; 2];
        unsafe { ffi::ekf2_get_flow_vel_ne(self.ptr_const(), out.as_mut_ptr()) };
        out
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn filtered_flow_vel_body(&self) -> [f32; 2] {
        let mut out = [0.0f32; 2];
        unsafe { ffi::ekf2_get_filtered_flow_vel_body(self.ptr_const(), out.as_mut_ptr()) };
        out
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn filtered_flow_vel_ne(&self) -> [f32; 2] {
        let mut out = [0.0f32; 2];
        unsafe { ffi::ekf2_get_filtered_flow_vel_ne(self.ptr_const(), out.as_mut_ptr()) };
        out
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn flow_compensated(&self) -> [f32; 2] {
        let mut out = [0.0f32; 2];
        unsafe { ffi::ekf2_get_flow_compensated(self.ptr_const(), out.as_mut_ptr()) };
        out
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn flow_uncompensated(&self) -> [f32; 2] {
        let mut out = [0.0f32; 2];
        unsafe { ffi::ekf2_get_flow_uncompensated(self.ptr_const(), out.as_mut_ptr()) };
        out
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn flow_gyro(&self) -> [f32; 3] {
        let mut out = [0.0f32; 3];
        unsafe { ffi::ekf2_get_flow_gyro(self.ptr_const(), out.as_mut_ptr()) };
        out
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn flow_gyro_bias(&self) -> [f32; 3] {
        let mut out = [0.0f32; 3];
        unsafe { ffi::ekf2_get_flow_gyro_bias(self.ptr_const(), out.as_mut_ptr()) };
        out
    }

    #[cfg(feature = "optical-flow")]
    #[inline]
    pub fn flow_ref_body_rate(&self) -> [f32; 3] {
        let mut out = [0.0f32; 3];
        unsafe { ffi::ekf2_get_flow_ref_body_rate(self.ptr_const(), out.as_mut_ptr()) };
        out
    }

    #[inline]
    pub fn reset_gyro_bias_cov(&mut self) {
        unsafe { ffi::ekf2_reset_gyro_bias_cov(self.ptr()) };
    }

    #[inline]
    pub fn reset_accel_bias_cov(&mut self) {
        unsafe { ffi::ekf2_reset_accel_bias_cov(self.ptr()) };
    }

    #[inline]
    pub fn update_parameters(&mut self) {
        unsafe { ffi::ekf2_update_parameters(self.ptr()) };
    }

    #[inline]
    pub fn check_lat_lon_validity(&mut self, lat: f64, lon: f64) -> bool {
        unsafe { ffi::ekf2_check_lat_lon_validity(self.ptr(), lat, lon) }
    }

    #[inline]
    pub fn check_altitude_validity(&mut self, altitude: f32) -> bool {
        unsafe { ffi::ekf2_check_altitude_validity(self.ptr(), altitude) }
    }

    #[cfg(feature = "barometer")]
    #[inline]
    pub fn baro_bias_estimator_status(&self) -> ffi::EkfBiasEstimatorStatus {
        let mut out = ffi::EkfBiasEstimatorStatus::default();
        unsafe { ffi::ekf2_get_baro_bias_estimator_status(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn ev_hgt_bias_estimator_status(&self) -> ffi::EkfBiasEstimatorStatus {
        let mut out = ffi::EkfBiasEstimatorStatus::default();
        unsafe { ffi::ekf2_get_ev_hgt_bias_estimator_status(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "external-vision")]
    #[inline]
    pub fn ev_pos_bias_estimator_status(&self, axis: u8) -> ffi::EkfBiasEstimatorStatus {
        let mut out = ffi::EkfBiasEstimatorStatus::default();
        unsafe { ffi::ekf2_get_ev_pos_bias_estimator_status(self.ptr_const(), axis, &mut out) };
        out
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn gps_hgt_bias_estimator_status(&self) -> ffi::EkfBiasEstimatorStatus {
        let mut out = ffi::EkfBiasEstimatorStatus::default();
        unsafe { ffi::ekf2_get_gps_hgt_bias_estimator_status(self.ptr_const(), &mut out) };
        out
    }

    #[cfg(feature = "gnss")]
    #[inline]
    pub fn set_min_required_gps_health_time(&mut self, time_us: u32) {
        unsafe { ffi::ekf2_set_min_required_gps_health_time(self.ptr(), time_us) };
    }
}

impl Drop for Ekf {
    fn drop(&mut self) {
        unsafe { ffi::ekf2_destroy_heap(self.ptr()) };
    }
}
