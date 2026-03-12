//! Safe wrapper around EKF2 runtime parameters.

use core::ffi::c_void;
use core::marker::PhantomData;
use ekf2_sys as ffi;

bitflags::bitflags! {
    /// EKF solution status bitmask returned by [`Ekf::soln_status()`](crate::ekf::Ekf::soln_status).
    ///
    /// Matches PX4's `get_ekf_soln_status()` bit layout.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct SolnStatus: u16 {
        const ATTITUDE           = 1 << 0;
        const VELOCITY_HORIZ     = 1 << 1;
        const VELOCITY_VERT      = 1 << 2;
        const POS_HORIZ_REL      = 1 << 3;
        const POS_HORIZ_ABS      = 1 << 4;
        const POS_VERT_ABS       = 1 << 5;
        const POS_VERT_AGL       = 1 << 6;
        const CONST_POS_MODE     = 1 << 7;
        const PRED_POS_HORIZ_REL = 1 << 8;
        const PRED_POS_HORIZ_ABS = 1 << 9;
        const GPS_GLITCH         = 1 << 10;
        const ACCEL_ERROR        = 1 << 11;
    }
}

bitflags::bitflags! {
    /// EKF filter-control status bitmask (`filter_control_status_u::value`).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ControlStatus: u64 {
        const TILT_ALIGN              = 1 << 0;
        const YAW_ALIGN               = 1 << 1;
        const GNSS_POS                = 1 << 2;
        const OPT_FLOW                = 1 << 3;
        const MAG_HDG                 = 1 << 4;
        const MAG_3D                  = 1 << 5;
        const MAG_DEC                 = 1 << 6;
        const IN_AIR                  = 1 << 7;
        const WIND                    = 1 << 8;
        const BARO_HGT                = 1 << 9;
        const RNG_HGT                 = 1 << 10;
        const GPS_HGT                 = 1 << 11;
        const EV_POS                  = 1 << 12;
        const EV_YAW                  = 1 << 13;
        const EV_HGT                  = 1 << 14;
        const FUSE_BETA               = 1 << 15;
        const MAG_FIELD_DISTURBED     = 1 << 16;
        const FIXED_WING              = 1 << 17;
        const MAG_FAULT               = 1 << 18;
        const FUSE_ASPD               = 1 << 19;
        const GND_EFFECT              = 1 << 20;
        const RNG_STUCK               = 1 << 21;
        const GNSS_YAW                = 1 << 22;
        const MAG_ALIGNED_IN_FLIGHT   = 1 << 23;
        const EV_VEL                  = 1 << 24;
        const SYNTHETIC_MAG_Z         = 1 << 25;
        const VEHICLE_AT_REST         = 1 << 26;
        const GNSS_YAW_FAULT          = 1 << 27;
        const RNG_FAULT               = 1 << 28;
        const INERTIAL_DEAD_RECKONING = 1 << 29;
        const WIND_DEAD_RECKONING     = 1 << 30;
        const RNG_KIN_CONSISTENT      = 1 << 31;
        const FAKE_POS                = 1 << 32;
        const FAKE_HGT                = 1 << 33;
        const GRAVITY_VECTOR          = 1 << 34;
        const MAG                     = 1 << 35;
        const EV_YAW_FAULT            = 1 << 36;
        const MAG_HEADING_CONSISTENT  = 1 << 37;
        const AUX_GPOS                = 1 << 38;
        const RNG_TERRAIN             = 1 << 39;
        const OPT_FLOW_TERRAIN        = 1 << 40;
        const VALID_FAKE_POS          = 1 << 41;
        const CONSTANT_POS            = 1 << 42;
        const BARO_FAULT              = 1 << 43;
        const GNSS_VEL                = 1 << 44;
        const GNSS_FAULT              = 1 << 45;
        const YAW_MANUAL              = 1 << 46;
        const GNSS_HGT_FAULT          = 1 << 47;
        const IN_TRANSITION_TO_FW     = 1 << 48;
    }
}

bitflags::bitflags! {
    /// EKF fault status bitmask (`fault_status_u::value`).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct FaultStatus: u32 {
        const BAD_MAG_X        = 1 << 0;
        const BAD_MAG_Y        = 1 << 1;
        const BAD_MAG_Z        = 1 << 2;
        const BAD_HDG          = 1 << 3;
        const BAD_MAG_DECL     = 1 << 4;
        const BAD_AIRSPEED     = 1 << 5;
        const BAD_SIDESLIP     = 1 << 6;
        const BAD_OPTFLOW_X    = 1 << 7;
        const BAD_OPTFLOW_Y    = 1 << 8;
        const BAD_ACC_VERTICAL = 1 << 10;
        const BAD_ACC_CLIPPING = 1 << 11;
    }
}

bitflags::bitflags! {
    /// EKF information-event status bitmask (`information_event_status_u::value`).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct InformationEventStatus: u32 {
        const GPS_CHECKS_PASSED         = 1 << 0;
        const RESET_VEL_TO_GPS          = 1 << 1;
        const RESET_VEL_TO_FLOW         = 1 << 2;
        const RESET_VEL_TO_VISION       = 1 << 3;
        const RESET_VEL_TO_ZERO         = 1 << 4;
        const RESET_POS_TO_LAST_KNOWN   = 1 << 5;
        const RESET_POS_TO_GPS          = 1 << 6;
        const RESET_POS_TO_VISION       = 1 << 7;
        const STARTING_GPS_FUSION       = 1 << 8;
        const STARTING_VISION_POS_FUSION = 1 << 9;
        const STARTING_VISION_VEL_FUSION = 1 << 10;
        const STARTING_VISION_YAW_FUSION = 1 << 11;
        const YAW_ALIGNED_TO_IMU_GPS    = 1 << 12;
        const RESET_HGT_TO_BARO         = 1 << 13;
        const RESET_HGT_TO_GPS          = 1 << 14;
        const RESET_HGT_TO_RNG          = 1 << 15;
        const RESET_HGT_TO_EV           = 1 << 16;
        const RESET_POS_TO_EXT_OBS      = 1 << 17;
        const RESET_WIND_TO_EXT_OBS     = 1 << 18;
    }
}

bitflags::bitflags! {
    /// GNSS quality-check fail status (`GnssChecks::gps_check_fail_status_u::value`).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct GnssCheckFailStatus: u16 {
        const FIX     = 1 << 0;
        const NSATS   = 1 << 1;
        const PDOP    = 1 << 2;
        const HACC    = 1 << 3;
        const VACC    = 1 << 4;
        const SACC    = 1 << 5;
        const HDRIFT  = 1 << 6;
        const VDRIFT  = 1 << 7;
        const HSPEED  = 1 << 8;
        const VSPEED  = 1 << 9;
        const SPOOFED = 1 << 10;
        const JAMMED  = 1 << 11;
    }
}

/// EKF height reference source.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeightReference {
    Baro = 0,
    Gnss = 1,
    Range = 2,
    ExternalVision = 3,
}

impl HeightReference {
    #[inline]
    pub const fn from_i32(value: i32) -> Option<Self> {
        match value {
            0 => Some(Self::Baro),
            1 => Some(Self::Gnss),
            2 => Some(Self::Range),
            3 => Some(Self::ExternalVision),
            _ => None,
        }
    }
}

bitflags::bitflags! {
    /// GNSS fusion control bitmask (`ekf2_gps_ctrl`).
    ///
    /// Mirrors PX4 `GnssCtrl`:
    /// - bit 0: horizontal position fusion
    /// - bit 1: vertical position fusion
    /// - bit 2: velocity fusion
    /// - bit 3: yaw fusion
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct GnssControl: i32 {
        const HPOS    = 1 << 0;
        const VPOS    = 1 << 1;
        const VEL     = 1 << 2;
        const YAW     = 1 << 3;
        /// PX4 default (`HPOS | VEL`).
        const DEFAULT = Self::HPOS.bits() | Self::VEL.bits();
    }
}

/// EKF position reference source (`position_sensor_ref`).
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositionReference {
    Unknown = 0,
    Gnss = 1,
    ExternalVision = 2,
}

impl PositionReference {
    #[inline]
    pub const fn from_i32(value: i32) -> Option<Self> {
        match value {
            0 => Some(Self::Unknown),
            1 => Some(Self::Gnss),
            2 => Some(Self::ExternalVision),
            _ => None,
        }
    }
}

bitflags::bitflags! {
    /// IMU fusion controls (`ekf2_imu_ctrl`).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ImuControl: i32 {
        const GYRO_BIAS     = 1 << 0;
        const ACCEL_BIAS    = 1 << 1;
        const GRAVITY_VECTOR = 1 << 2;
        /// PX4 default (`GYRO_BIAS | ACCEL_BIAS`).
        const DEFAULT = Self::GYRO_BIAS.bits() | Self::ACCEL_BIAS.bits();
    }
}

/// GNSS fusion mode (`ekf2_gps_mode`).
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GnssMode {
    Auto = 0,
    DeadReckoning = 1,
}

impl GnssMode {
    #[inline]
    pub const fn from_i32(value: i32) -> Option<Self> {
        match value {
            0 => Some(Self::Auto),
            1 => Some(Self::DeadReckoning),
            _ => None,
        }
    }
}

/// Minimum GNSS fix quality (`ekf2_req_fix`).
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GnssFixType {
    NoFix0 = 0,
    NoFix1 = 1,
    Fix2d = 2,
    Fix3d = 3,
    RtcmCodeDifferential = 4,
    RealTime = 5,
}

impl GnssFixType {
    #[inline]
    pub const fn from_i32(value: i32) -> Option<Self> {
        match value {
            0 => Some(Self::NoFix0),
            1 => Some(Self::NoFix1),
            2 => Some(Self::Fix2d),
            3 => Some(Self::Fix3d),
            4 => Some(Self::RtcmCodeDifferential),
            5 => Some(Self::RealTime),
            _ => None,
        }
    }
}

/// Range-finder fusion control (`ekf2_rng_ctrl`).
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RangeControl {
    Disabled = 0,
    Conditional = 1,
    Enabled = 2,
}

impl RangeControl {
    #[inline]
    pub const fn from_i32(value: i32) -> Option<Self> {
        match value {
            0 => Some(Self::Disabled),
            1 => Some(Self::Conditional),
            2 => Some(Self::Enabled),
            _ => None,
        }
    }
}

bitflags::bitflags! {
    /// External-vision fusion controls (`ekf2_ev_ctrl`).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ExternalVisionControl: i32 {
        const HPOS = 1 << 0;
        const VPOS = 1 << 1;
        const VEL  = 1 << 2;
        const YAW  = 1 << 3;
    }
}

bitflags::bitflags! {
    /// Magnetic declination source controls (`ekf2_decl_type`).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct MagDeclinationControl: i32 {
        const USE_GEO_DECL  = 1 << 0;
        const SAVE_GEO_DECL = 1 << 1;
    }
}

/// Magnetometer fusion mode (`ekf2_mag_type`).
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MagFusionType {
    Auto = 0,
    Heading = 1,
    None = 5,
    InitOnly = 6,
}

impl MagFusionType {
    #[inline]
    pub const fn from_i32(value: i32) -> Option<Self> {
        match value {
            0 => Some(Self::Auto),
            1 => Some(Self::Heading),
            5 => Some(Self::None),
            6 => Some(Self::InitOnly),
            _ => None,
        }
    }
}

bitflags::bitflags! {
    /// Magnetometer consistency-check controls (`ekf2_mag_check`).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct MagCheckControl: i32 {
        const STRENGTH    = 1 << 0;
        const INCLINATION = 1 << 1;
        const FORCE_WMM   = 1 << 2;
    }
}

bitflags::bitflags! {
    /// GNSS quality-check controls (`ekf2_gps_check`).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct GnssCheckControl: i32 {
        const NSATS   = 1 << 0;
        const PDOP    = 1 << 1;
        const HACC    = 1 << 2;
        const VACC    = 1 << 3;
        const SACC    = 1 << 4;
        const H_DRIFT = 1 << 5;
        const V_DRIFT = 1 << 6;
        const H_SPEED = 1 << 7;
        const V_SPEED = 1 << 8;
        const SPOOFED = 1 << 9;
        const FIX     = 1 << 10;
        const JAMMED  = 1 << 11;
    }
}

/// Barometer aiding control (`ekf2_baro_ctrl`).
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BaroControl {
    Disabled = 0,
    Enabled = 1,
}

impl BaroControl {
    #[inline]
    pub const fn from_i32(value: i32) -> Option<Self> {
        match value {
            0 => Some(Self::Disabled),
            1 => Some(Self::Enabled),
            _ => None,
        }
    }
}

/// Optical-flow aiding control (`ekf2_of_ctrl`).
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OpticalFlowControl {
    Disabled = 0,
    Enabled = 1,
}

impl OpticalFlowControl {
    #[inline]
    pub const fn from_i32(value: i32) -> Option<Self> {
        match value {
            0 => Some(Self::Disabled),
            1 => Some(Self::Enabled),
            _ => None,
        }
    }
}

/// Optical-flow gyro source (`ekf2_of_gyr_src`).
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlowGyroSource {
    Auto = 0,
    Internal = 1,
}

impl FlowGyroSource {
    #[inline]
    pub const fn from_i32(value: i32) -> Option<Self> {
        match value {
            0 => Some(Self::Auto),
            1 => Some(Self::Internal),
            _ => None,
        }
    }
}

/// Immutable view of EKF2 parameters.
///
/// Obtained via [`Ekf::params()`](crate::Ekf::params).
pub struct Params<'a> {
    pub(crate) ekf_ptr: *const c_void,
    pub(crate) _marker: PhantomData<&'a ()>,
}

/// Mutable view of EKF2 parameters.
///
/// Obtained via [`Ekf::params_mut()`](crate::Ekf::params_mut).
pub struct ParamsMut<'a> {
    pub(crate) ekf_ptr: *mut c_void,
    pub(crate) _marker: PhantomData<&'a mut ()>,
}

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
