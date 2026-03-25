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


mod getters;
mod setters;
