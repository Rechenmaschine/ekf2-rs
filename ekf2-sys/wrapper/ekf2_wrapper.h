/**
 * ekf2_wrapper.h — extern "C" API surface for the PX4 EKF2 Rust bindings
 *
 * This header is the sole input to bindgen.  All types are plain C structs
 * with verified layout (static_assert'd against the C++ originals in the
 * .cpp translation unit).
 *
 * Lifetime contract
 * -----------------
 *   A) Heap-owned path (used by `ekf2` safe wrapper):
 *      1. ekf2_create_heap()            — allocates + constructs Ekf
 *      2. ekf2_init(ekf_ptr, timestamp) — calls Ekf::init()
 *      3. ... normal operation ...
 *      4. ekf2_destroy_heap(ekf_ptr)    — calls ~Ekf() + frees object memory
 *
 *   B) Caller-buffer path (for low-level/manual integration):
 *      1. ekf2_create(buf, sizeof_buf)  — placement-new Ekf into caller buffer
 *      2. ekf2_init(ekf_ptr, timestamp) — calls Ekf::init()
 *      3. ... normal operation ...
 *      4. ekf2_destroy(ekf_ptr)         — calls ~Ekf() in-place (no free)
 *
 * All functions taking `void* self` expect a pointer obtained from step 1.
 * All functions taking `const void* self` require the object to be
 * initialised (step 2 complete).
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * Aid source diagnostic structs
 *
 * Self-contained C structs; layout verified against PX4's
 * estimator_aid_source{1,2,3}d_s by static_assert in ekf2_wrapper.cpp.
 * uORB headers are NOT included here — all PX4 type knowledge stays in
 * the .cpp translation unit.
 * ========================================================================= */

/** 1-D aid source (baro height, range height, airspeed, sideslip, …) */
typedef struct {
    uint64_t timestamp;               /**< [µs] system time */
    uint64_t timestamp_sample;        /**< [µs] raw sensor timestamp */
    uint8_t  estimator_instance;
    uint8_t  _pad0[3];
    uint32_t device_id;
    uint64_t time_last_fuse;          /**< [µs] time of last successful fusion */
    float    observation;
    float    observation_variance;
    float    innovation;
    float    innovation_filtered;
    float    innovation_variance;
    float    test_ratio;
    float    test_ratio_filtered;
    uint8_t  innovation_rejected;     /**< 1 if rejected */
    uint8_t  fused;                   /**< 1 if successfully fused */
    uint8_t  _pad1[2];
} EkfAidSource1d;

/** 2-D aid source (GNSS position, optical flow, EV position, …)
 *  Note: observation uses double precision (lat/lon). */
typedef struct {
    uint64_t timestamp;
    uint64_t timestamp_sample;
    uint8_t  estimator_instance;
    uint8_t  _pad0[7];
    uint32_t device_id;
    uint8_t  _pad1[4];
    uint64_t time_last_fuse;
    double   observation[2];
    float    observation_variance[2];
    float    innovation[2];
    float    innovation_filtered[2];
    float    innovation_variance[2];
    float    test_ratio[2];
    float    test_ratio_filtered[2];
    uint8_t  innovation_rejected;
    uint8_t  fused;
    uint8_t  _pad2[6];
} EkfAidSource2d;

/** 3-D aid source (GNSS velocity, magnetometer, EV velocity, gravity, …) */
typedef struct {
    uint64_t timestamp;
    uint64_t timestamp_sample;
    uint8_t  estimator_instance;
    uint8_t  _pad0[3];
    uint32_t device_id;
    uint64_t time_last_fuse;
    float    observation[3];
    float    observation_variance[3];
    float    innovation[3];
    float    innovation_filtered[3];
    float    innovation_variance[3];
    float    test_ratio[3];
    float    test_ratio_filtered[3];
    uint8_t  innovation_rejected;
    uint8_t  fused;
    uint8_t  _pad1[2];
} EkfAidSource3d;

/** Scalar bias-estimator diagnostic status. */
typedef struct {
    float bias;
    float bias_var;
    float innov;
    float innov_var;
    float innov_test_ratio;
} EkfBiasEstimatorStatus;

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Sensor sample structs
 *
 * Layout is verified by static_assert in ekf2_wrapper.cpp against the
 * corresponding PX4 C++ structs.
 * ========================================================================= */

/** IMU delta-angle / delta-velocity sample */
typedef struct {
    uint64_t time_us;            /**< sample timestamp [µs] */
    float    delta_ang[3];       /**< delta angle [rad] (body frame) */
    float    delta_vel[3];       /**< delta velocity [m/s] (body frame) */
    float    delta_ang_dt;       /**< integration time for delta_ang [s] */
    float    delta_vel_dt;       /**< integration time for delta_vel [s] */
    uint8_t  delta_vel_clipping[3]; /**< per-axis clip flag (0/1) */
    uint8_t  _pad[5];            /**< explicit padding to 48 bytes */
} EkfImuSample;

#ifdef CONFIG_EKF2_GNSS
/** GNSS position/velocity sample */
typedef struct {
    uint64_t time_us;        /**< timestamp [µs] */
    double   lat;            /**< latitude [deg] */
    double   lon;            /**< longitude [deg] */
    float    alt;            /**< altitude WGS-84 [m] */
    float    yaw;            /**< true-north heading [rad] (NaN if unavailable) */
    float    yaw_acc;        /**< 1σ yaw accuracy [rad] */
    float    yaw_offset;     /**< heading offset from forward body axis [rad] */
    float    vel[3];         /**< NED velocity [m/s] */
    float    hacc;           /**< horizontal position 1σ [m] */
    float    vacc;           /**< vertical position 1σ [m] */
    float    sacc;           /**< speed 1σ [m/s] */
    float    pdop;           /**< position dilution of precision */
    uint8_t  fix_type;       /**< 0–1: no fix, 2: 2-D, 3: 3-D, 4: RTCM */
    uint8_t  nsats;          /**< number of satellites */
    uint8_t  spoofed;        /**< 1 if signal may be spoofed */
    uint8_t  jammed;         /**< 1 if signal may be jammed (anti-spoofing) */
    uint8_t  _pad[8];        /**< explicit padding to 80 bytes (matches gnssSample) */
} EkfGnssSample;
#endif /* CONFIG_EKF2_GNSS */

#ifdef CONFIG_EKF2_MAGNETOMETER
/** Magnetometer sample */
typedef struct {
    uint64_t time_us;    /**< timestamp [µs] */
    float    mag[3];     /**< field vector in body frame [Gauss] */
    uint8_t  reset;      /**< 1 if sensor replaced or calibration changed mid-flight */
    uint8_t  _pad[3];    /**< explicit padding to 24 bytes */
} EkfMagSample;
#endif /* CONFIG_EKF2_MAGNETOMETER */

#ifdef CONFIG_EKF2_BAROMETER
/** Barometric altitude sample */
typedef struct {
    uint64_t time_us;    /**< timestamp [µs] */
    float    hgt;        /**< altitude above mean sea level [m] */
    uint8_t  reset;      /**< 1 if sensor replaced or calibration changed mid-flight */
    uint8_t  _pad[3];    /**< explicit padding to 16 bytes */
} EkfBaroSample;
#endif /* CONFIG_EKF2_BAROMETER */

#ifdef CONFIG_EKF2_AIRSPEED
/** True airspeed sample */
typedef struct {
    uint64_t time_us;    /**< timestamp [µs] */
    float    true_airspeed;   /**< [m/s] */
    float    eas2tas;         /**< equivalent-to-true airspeed ratio */
} EkfAirspeedSample;
#endif /* CONFIG_EKF2_AIRSPEED */

#ifdef CONFIG_EKF2_RANGE_FINDER
/** Range finder sample */
typedef struct {
    uint64_t time_us;    /**< timestamp [µs] */
    float    rng;        /**< range measurement [m] */
    int8_t   quality;    /**< signal quality 0–100 (100 = perfect, 0 = invalid, -1 = unknown) */
    uint8_t  _pad[3];    /**< explicit padding to 16 bytes */
} EkfRangeSample;
#endif /* CONFIG_EKF2_RANGE_FINDER */

#ifdef CONFIG_EKF2_OPTICAL_FLOW
/** Optical flow sample.
 *
 *  Fields store *rates* (rad/s), matching PX4's flowSample convention.
 *  This removes the division-by-dt ambiguity of the old integrated-value API. */
typedef struct {
    uint64_t time_us;              /**< timestamp [µs] */
    float    flow_rate_xy[2];      /**< optical flow rate [rad/s] about body X and Y axes */
    float    gyro_rate_xyz[3];     /**< gyro rate [rad/s] about body X, Y, Z axes */
    uint8_t  quality;              /**< 0–255 quality metric */
    uint8_t  _pad[3];              /**< explicit padding to 32 bytes */
} EkfFlowSample;
#endif /* CONFIG_EKF2_OPTICAL_FLOW */

#ifdef CONFIG_EKF2_EXTERNAL_VISION
/** External vision (VIO/MoCap) sample */
typedef struct {
    uint64_t time_us;        /**< timestamp [µs] */
    float    pos[3];         /**< position [m] (local frame) */
    float    quat[4];        /**< orientation quaternion [w, x, y, z] */
    float    vel[3];         /**< velocity [m/s] (local frame) */
    float    pos_var[3];     /**< position variance [m²] */
    float    ang_var[3];     /**< per-axis attitude variance [rad²] (X, Y, Z) */
    float    vel_var[3];     /**< velocity variance [(m/s)²] */
    uint8_t  pos_frame;      /**< PositionFrame: 0=LOCAL_FRAME_NED, 1=LOCAL_FRAME_FRD */
    uint8_t  vel_frame;      /**< VelocityFrame: 0=LOCAL_FRAME_NED, 1=LOCAL_FRAME_FRD, 2=BODY_FRAME_FRD */
    uint8_t  reset_counter;  /**< increment when tracker resets to signal discontinuity */
    int8_t   quality;        /**< quality indicator 0–100 (100=perfect, 0=invalid) */
} EkfExtVisionSample;
#endif /* CONFIG_EKF2_EXTERNAL_VISION */

#ifdef CONFIG_EKF2_AUXVEL
/** Auxiliary velocity sample (e.g. from body drag model) */
typedef struct {
    uint64_t time_us;      /**< timestamp [µs] */
    float    vel[2];       /**< horizontal velocity [m/s] */
    float    vel_var[2];   /**< velocity variance [(m/s)²] */
} EkfAuxVelSample;
#endif /* CONFIG_EKF2_AUXVEL */

/** Vehicle/system flags used by EKF mode control. */
typedef struct {
    uint64_t time_us;             /**< timestamp [µs] */
    bool     at_rest;             /**< true if the vehicle is at rest */
    bool     in_air;              /**< true if the vehicle is airborne */
    bool     is_fixed_wing;       /**< true if fixed-wing mode is active */
    bool     gnd_effect;          /**< true to trigger ground-effect compensation */
    bool     constant_pos;        /**< true to force constant-position mode */
    bool     in_transition_to_fw; /**< true if transitioning to fixed-wing */
    uint8_t  _pad;                /**< explicit padding for stable C layout */
} EkfSystemFlagUpdate;

/* =========================================================================
 * Lifecycle
 * ========================================================================= */

/** Size in bytes of the C++ Ekf object (for ekf_buf sizing). */
size_t ekf2_sizeof(void);

/** Required alignment in bytes of the C++ Ekf object. */
size_t ekf2_alignof(void);

/**
 * Allocate and construct an Ekf object on the C++ heap.
 * Allocation follows the active C++ operator new path (bridged to Rust
 * allocator symbols by allocator.cpp in this project).
 *
 * @return Pointer to a constructed object, or NULL on allocation failure.
 */
void* ekf2_create_heap(void);

/**
 * Destruct and free an Ekf object created by ekf2_create_heap().
 */
void ekf2_destroy_heap(void* self);

/**
 * Placement-new an Ekf object into the caller-supplied buffer.
 *
 * @param ekf_buf       Buffer of at least ekf2_sizeof() bytes.
 * @param ekf_buf_size  Size of ekf_buf (checked at runtime in debug builds).
 * @return              Pointer to the constructed object (== ekf_buf), or
 *                      NULL if ekf_buf_size is too small.
 */
void* ekf2_create(void* ekf_buf, size_t ekf_buf_size);

/**
 * Initialise the filter.  Must be called with the pool active.
 *
 * @param self          Pointer from ekf2_create_heap() or ekf2_create().
 * @param timestamp_us  Current system time [µs].
 * @return              true on success.
 */
bool ekf2_init(void* self, uint64_t timestamp_us);

/**
 * Run one filter update step.  Call after every IMU sample.
 *
 * @return  true on success.
 */
bool ekf2_update(void* self);

/**
 * In-place destructor.  Calls ~Ekf() without freeing memory.
 */
void ekf2_destroy(void* self);

/**
 * Reset the filter: destroy in-place and re-construct + re-init.
 * No memory is freed or allocated for the Ekf object itself.
 */
bool ekf2_reset(void* self, uint64_t timestamp_us);

/* =========================================================================
 * Sensor inputs — always available (IMU is mandatory)
 * ========================================================================= */

void ekf2_set_imu_data(void* self, const EkfImuSample* sample);

/* =========================================================================
 * Sensor inputs — feature-gated
 * ========================================================================= */

#ifdef CONFIG_EKF2_GNSS
void ekf2_set_gps_data(void* self, const EkfGnssSample* sample);
#endif

#ifdef CONFIG_EKF2_MAGNETOMETER
void ekf2_set_mag_data(void* self, const EkfMagSample* sample);
#endif

#ifdef CONFIG_EKF2_BAROMETER
void ekf2_set_baro_data(void* self, const EkfBaroSample* sample);
#endif

#ifdef CONFIG_EKF2_AIRSPEED
void ekf2_set_airspeed_data(void* self, const EkfAirspeedSample* sample);
#endif

#ifdef CONFIG_EKF2_RANGE_FINDER
void ekf2_set_range_data(void* self, const EkfRangeSample* sample);
#endif

#ifdef CONFIG_EKF2_OPTICAL_FLOW
void ekf2_set_optical_flow_data(void* self, const EkfFlowSample* sample);
#endif

#ifdef CONFIG_EKF2_EXTERNAL_VISION
void ekf2_set_ev_data(void* self, const EkfExtVisionSample* sample);
#endif

#ifdef CONFIG_EKF2_AUXVEL
void ekf2_set_aux_vel_data(void* self, const EkfAuxVelSample* sample);
#endif

/* Vehicle/system mode flags (always available). */
void ekf2_set_system_flag_data(void* self, const EkfSystemFlagUpdate* sample);
void ekf2_set_in_air_status(void* self, bool in_air);
void ekf2_set_vehicle_at_rest(void* self, bool at_rest);
void ekf2_set_constant_pos(void* self, bool constant_pos);
void ekf2_set_is_fixed_wing(void* self, bool is_fixed_wing);
void ekf2_set_in_transition_to_fw(void* self, bool in_transition_to_fw);
void ekf2_set_gnd_effect(void* self);
void ekf2_set_air_density(void* self, float air_density);

#ifdef CONFIG_EKF2_GNSS
void ekf2_set_gps_data_with_pps(void* self, const EkfGnssSample* sample, bool pps_compensation);
#endif

#ifdef CONFIG_EKF2_AIRSPEED
void ekf2_set_synthetic_airspeed(void* self, bool synthetic_airspeed);
#endif

#ifdef CONFIG_EKF2_RANGE_FINDER
void ekf2_set_rangefinder_limits(void* self, float min_distance, float max_distance);
#endif

#ifdef CONFIG_EKF2_OPTICAL_FLOW
void ekf2_set_optical_flow_limits(void* self, float max_flow_rate, float min_distance, float max_distance);
#endif

/* =========================================================================
 * State outputs
 * ========================================================================= */

/** Attitude quaternion [w, x, y, z] */
void ekf2_get_quaternion(const void* self, float q[4]);

/** NED velocity [m/s] */
void ekf2_get_velocity(const void* self, float v[3]);

/** NED position relative to origin [m] */
void ekf2_get_position(const void* self, float p[3]);

/** Rate gyro bias [rad/s] */
void ekf2_get_gyro_bias(const void* self, float b[3]);

/** Accelerometer bias [m/s²] */
void ekf2_get_accel_bias(const void* self, float b[3]);

#ifdef CONFIG_EKF2_MAGNETOMETER
/** Earth-frame magnetic field state [Gauss]. */
void ekf2_get_mag_earth_field(const void* self, float mag[3]);

/** Body-frame magnetic field bias state [Gauss]. */
void ekf2_get_mag_bias(const void* self, float bias[3]);

/** Magnetometer bias variance [Gauss²]. */
void ekf2_get_mag_bias_variance(const void* self, float var[3]);
#endif

/** Output predictor angular-rate reset accumulator [rad/s]. */
void ekf2_get_angular_velocity_reset_accumulator(void* self, float w[3]);

/** Un-aided yaw estimate [rad]. */
float ekf2_get_unaided_yaw(const void* self);

/** True if attitude estimate is valid (converged) */
bool ekf2_attitude_valid(const void* self);

/** Local (dead-reckoning) position estimate valid */
bool ekf2_local_position_valid(const void* self);

/** Local (dead-reckoning) velocity estimate valid */
bool ekf2_local_velocity_valid(const void* self);

/** Global (GNSS-referenced) horizontal and vertical position valid */
bool ekf2_global_position_valid(const void* self);

/** True once yaw has completed final in-flight alignment */
bool ekf2_yaw_align_complete(const void* self);

/** Diagonal of position covariance [m²] (3 floats: N, E, D) */
void ekf2_get_pos_variance(const void* self, float var[3]);

/** Diagonal of velocity covariance [(m/s)²] (3 floats: N, E, D) */
void ekf2_get_vel_variance(const void* self, float var[3]);

/** Gyro bias variance [rad/s]² (3 floats: X, Y, Z) */
void ekf2_get_gyro_bias_variance(const void* self, float var[3]);

/** Accelerometer bias variance [m/s²]² (3 floats: X, Y, Z) */
void ekf2_get_accel_bias_variance(const void* self, float var[3]);

/** Rotation variance projected into body frame [rad²]. */
void ekf2_get_rot_var_body(const void* self, float var[3]);

/** Rotation variance projected into NED frame [rad²]. */
void ekf2_get_rot_var_ned(const void* self, float var[3]);

/** Yaw state variance [rad²]. */
float ekf2_get_yaw_variance(const void* self);

/** Tilt variance [rad²]. */
float ekf2_get_tilt_variance(const void* self);

/** EKF state vector dimension. */
size_t ekf2_get_state_size(void);

/**
 * Return the full-state covariance diagonal.
 *
 * @param out       Destination buffer (may be NULL to query required length).
 * @param capacity  Number of float elements available in `out`.
 * @return          Required number of elements (state size).
 */
size_t ekf2_get_covariances_diagonal(const void* self, float* out, size_t capacity);

/**
 * Return one covariance element.
 *
 * @return state covariance value, or NaN for out-of-range indices.
 */
float ekf2_get_state_covariance(const void* self, uint32_t row, uint32_t col);

/** Mean earth-frame velocity derivative since last reset [m/s²]. */
void ekf2_get_velocity_derivative(const void* self, float vdot[3]);

/** Reset velocity-derivative accumulation. */
void ekf2_reset_velocity_derivative_accumulation(void* self);

/** Vertical position derivative [m/s]. */
float ekf2_get_vertical_position_derivative(const void* self);

/** Output predictor geodetic position (WGS-84). */
void ekf2_get_lat_lon_alt(const void* self, double* lat, double* lon, float* alt);

/** Output predictor tracking error vector. */
void ekf2_get_output_tracking_error(const void* self, float err[3]);

/** Raw control-status bitmask (`filter_control_status_u::value`). */
uint64_t ekf2_get_control_status(const void* self);

/** Raw fault-status bitmask (`fault_status_u::value`). */
uint32_t ekf2_get_fault_status(const void* self);

/** Raw information-event bitmask (`information_event_status_u::value`). */
uint32_t ekf2_get_information_event_status(const void* self);

/** Clear latched information-event flags. */
void ekf2_clear_information_events(void* self);

/* =========================================================================
 * Innovation test ratios
 *
 * Each ratio is (innovation² / innovation_variance).  Values > 1 indicate
 * the measurement was rejected by the filter.  NaN if not available.
 * ========================================================================= */

float ekf2_get_heading_innov_test_ratio(const void* self);
float ekf2_get_horiz_vel_innov_test_ratio(const void* self);
float ekf2_get_vert_vel_innov_test_ratio(const void* self);
float ekf2_get_horiz_pos_innov_test_ratio(const void* self);
float ekf2_get_vert_pos_innov_test_ratio(const void* self);

#ifdef CONFIG_EKF2_AIRSPEED
float ekf2_get_airspeed_innov_test_ratio(const void* self);
#endif

#ifdef CONFIG_EKF2_SIDESLIP
float ekf2_get_sideslip_innov_test_ratio(const void* self);
#endif

#ifdef CONFIG_EKF2_RANGE_FINDER
float ekf2_get_hagl_rate_innov(const void* self);
float ekf2_get_hagl_rate_innov_var(const void* self);
float ekf2_get_hagl_rate_innov_ratio(const void* self);
#endif

/* Height-above-ground innovation ratio (requires TERRAIN, RANGE_FINDER, or OPTICAL_FLOW). */
#if defined(CONFIG_EKF2_TERRAIN) || defined(CONFIG_EKF2_RANGE_FINDER) || defined(CONFIG_EKF2_OPTICAL_FLOW)
float ekf2_get_hagl_innov_test_ratio(const void* self);
#endif

/* =========================================================================
 * Solution status bitmask
 * ========================================================================= */

/**
 * Return a uint16 bitmask summarising which state estimates are valid.
 * Matches PX4's `get_ekf_soln_status()` bit layout:
 *   bit 0  attitude_solution_good
 *   bit 1  velocity_horiz_solution_good
 *   bit 2  velocity_vert_solution_good
 *   bit 3  pos_horiz_rel_solution_good
 *   bit 4  pos_horiz_abs_solution_good
 *   bit 5  pos_vert_abs_solution_good
 *   bit 6  pos_vert_agl_solution_good
 *   bit 7  const_pos_mode
 *   bit 8  pred_pos_horiz_rel_solution_good
 *   bit 9  pred_pos_horiz_abs_solution_good
 *   bit 10 gps_glitch_detected
 *   bit 11 accel_error_detected
 */
uint16_t ekf2_get_soln_status(const void* self);

/** Active height sensor reference. */
uint8_t ekf2_get_height_sensor_ref(const void* self);

/* =========================================================================
 * Accuracy bounds (1-sigma, metres or m/s)
 * ========================================================================= */

/** Local position EPH (horizontal) and EPV (vertical) accuracy [m]. */
void ekf2_get_lpos_accuracy(const void* self, float* eph, float* epv);

/** Global position EPH (horizontal) and EPV (vertical) accuracy [m]. */
void ekf2_get_gpos_accuracy(const void* self, float* eph, float* epv);

/** Velocity EVH (horizontal) and EVV (vertical) accuracy [m/s]. */
void ekf2_get_vel_accuracy(const void* self, float* evh, float* evv);

/**
 * EKF control limits.
 * Returns NaN for limits that are currently not active.
 */
void ekf2_get_ctrl_limits(const void* self, float* vxy_max, float* vz_max,
                           float* hagl_min, float* hagl_max_z, float* hagl_max_xy);

/* =========================================================================
 * Global origin
 * ========================================================================= */

/** Query the WGS-84 origin set in the EKF. */
void ekf2_get_global_origin(const void* self, uint64_t* time_us,
                             double* lat, double* lon, float* alt);

/**
 * Set the WGS-84 origin used for local↔global position conversion.
 *
 * @param hpos_var  Horizontal position variance [m²]; pass NaN to use default.
 * @param vpos_var  Vertical   position variance [m²]; pass NaN to use default.
 * @return          true on success.
 */
bool ekf2_set_global_origin(void* self, double lat, double lon, float alt,
                             float hpos_var, float vpos_var);

/** Reset global position to a provided WGS-84 target. */
bool ekf2_reset_global_position(void* self, double lat, double lon, float alt,
                                 float hpos_var, float vpos_var);

/** Reset global position from an external observation while dead-reckoning. */
bool ekf2_reset_global_position_to_external_observation(
    void* self,
    double lat,
    double lon,
    float alt,
    float eph,
    float epv,
    uint64_t timestamp_observation);

/* =========================================================================
 * State reset tracking
 *
 * Each `_count` function returns how many times the EKF has reset that
 * state since init.  The corresponding `_reset` function additionally
 * fills in the last reset delta value.
 * ========================================================================= */

uint8_t ekf2_get_quat_reset_count(const void* self);
uint8_t ekf2_get_pos_ne_reset_count(const void* self);
uint8_t ekf2_get_vel_ne_reset_count(const void* self);
uint8_t ekf2_get_pos_d_reset_count(const void* self);
uint8_t ekf2_get_vel_d_reset_count(const void* self);
uint8_t ekf2_get_hagl_reset_count(const void* self);

void ekf2_get_quat_reset(const void* self, float delta_quat[4], uint8_t* count);
void ekf2_get_pos_ne_reset(const void* self, float delta_ne[2], uint8_t* count);
void ekf2_get_vel_ne_reset(const void* self, float delta_ne[2], uint8_t* count);
void ekf2_get_pos_d_reset(const void* self, float* delta, uint8_t* count);
void ekf2_get_vel_d_reset(const void* self, float* delta, uint8_t* count);
void ekf2_get_hagl_reset(const void* self, float* delta, uint8_t* count);

/* =========================================================================
 * Additional estimator diagnostics and control
 * ========================================================================= */

void ekf2_reset_gyro_bias(void* self);
void ekf2_reset_accel_bias(void* self);
void ekf2_reset_gyro_bias_cov(void* self);
void ekf2_reset_accel_bias_cov(void* self);
void ekf2_reset_heading_to_external_observation(void* self, float heading, float heading_accuracy);
void ekf2_update_parameters(void* self);
bool ekf2_check_lat_lon_validity(void* self, double lat, double lon);
bool ekf2_check_altitude_validity(void* self, float altitude);

#ifdef CONFIG_EKF2_WIND
void ekf2_reset_wind_to_external_observation(
    void* self,
    float wind_speed,
    float wind_direction,
    float wind_speed_accuracy,
    float wind_direction_accuracy);
#endif

#ifdef CONFIG_EKF2_MAGNETOMETER
bool ekf2_update_world_magnetic_model(void* self, double latitude_deg, double longitude_deg);
#endif

#ifdef CONFIG_EKF2_GNSS
float ekf2_get_gps_horizontal_position_drift_rate(const void* self);
float ekf2_get_gps_vertical_position_drift_rate(const void* self);
float ekf2_get_gps_filtered_horizontal_velocity(const void* self);
bool  ekf2_gps_checks_passed(const void* self);
uint16_t ekf2_get_gps_check_fail_status(const void* self);
void ekf2_set_min_required_gps_health_time(void* self, uint32_t time_us);
void ekf2_get_gps_hgt_bias_estimator_status(const void* self, EkfBiasEstimatorStatus* out);
bool ekf2_yaw_emergency_estimate_available(const void* self);
bool ekf2_get_ekfgsf_data(
    const void* self,
    float* yaw_composite,
    float* yaw_variance,
    float* yaw,
    float* innov_vn,
    float* innov_ve,
    float* weight,
    size_t capacity,
    size_t* count_out);
#endif

/* =========================================================================
 * Terrain estimation
 * ========================================================================= */

#if defined(CONFIG_EKF2_TERRAIN) || defined(CONFIG_EKF2_RANGE_FINDER) || defined(CONFIG_EKF2_OPTICAL_FLOW)
/** True if the terrain height estimate is valid. */
bool  ekf2_terrain_valid(const void* self);

/** Estimated terrain height in the local NED frame [m above origin]. */
float ekf2_get_terrain_vert_pos(const void* self);

/** Terrain height estimate variance [m²]. */
float ekf2_get_terrain_variance(const void* self);

/** Height above ground level [m] (local down position minus terrain position). */
float ekf2_get_hagl(const void* self);
#endif

#ifdef CONFIG_EKF2_OPTICAL_FLOW
void ekf2_get_flow_vel_body(const void* self, float v[2]);
void ekf2_get_flow_vel_ne(const void* self, float v[2]);
void ekf2_get_filtered_flow_vel_body(const void* self, float v[2]);
void ekf2_get_filtered_flow_vel_ne(const void* self, float v[2]);
void ekf2_get_flow_compensated(const void* self, float flow_rate[2]);
void ekf2_get_flow_uncompensated(const void* self, float flow_rate[2]);
void ekf2_get_flow_gyro(const void* self, float gyro_rate[3]);
void ekf2_get_flow_gyro_bias(const void* self, float gyro_bias[3]);
void ekf2_get_flow_ref_body_rate(const void* self, float body_rate[3]);
#endif

/* =========================================================================
 * Aid source diagnostics
 *
 * Each function copies the internal aid-source struct into the caller-
 * supplied output pointer.  Fields: observation, innovation, test_ratio,
 * innovation_rejected, fused, time_last_fuse — see EkfAidSource{1,2,3}d.
 * ========================================================================= */

void ekf2_get_aid_src_fake_hgt(const void* self, EkfAidSource1d* out);
void ekf2_get_aid_src_fake_pos(const void* self, EkfAidSource2d* out);

#ifdef CONFIG_EKF2_BAROMETER
void ekf2_get_aid_src_baro_hgt(const void* self, EkfAidSource1d* out);
#endif

#ifdef CONFIG_EKF2_GNSS
void ekf2_get_aid_src_gnss_hgt(const void* self, EkfAidSource1d* out);
void ekf2_get_aid_src_gnss_pos(const void* self, EkfAidSource2d* out);
void ekf2_get_aid_src_gnss_vel(const void* self, EkfAidSource3d* out);
#  ifdef CONFIG_EKF2_GNSS_YAW
void ekf2_get_aid_src_gnss_yaw(const void* self, EkfAidSource1d* out);
#  endif
#endif

#ifdef CONFIG_EKF2_MAGNETOMETER
void ekf2_get_aid_src_mag(const void* self, EkfAidSource3d* out);
#endif

#ifdef CONFIG_EKF2_AIRSPEED
void ekf2_get_aid_src_airspeed(const void* self, EkfAidSource1d* out);
#endif

#ifdef CONFIG_EKF2_SIDESLIP
void ekf2_get_aid_src_sideslip(const void* self, EkfAidSource1d* out);
#endif

#ifdef CONFIG_EKF2_RANGE_FINDER
void ekf2_get_aid_src_rng_hgt(const void* self, EkfAidSource1d* out);
#endif

#ifdef CONFIG_EKF2_OPTICAL_FLOW
void ekf2_get_aid_src_optical_flow(const void* self, EkfAidSource2d* out);
#endif

#ifdef CONFIG_EKF2_EXTERNAL_VISION
void ekf2_get_aid_src_ev_hgt(const void* self, EkfAidSource1d* out);
void ekf2_get_aid_src_ev_pos(const void* self, EkfAidSource2d* out);
void ekf2_get_aid_src_ev_vel(const void* self, EkfAidSource3d* out);
void ekf2_get_aid_src_ev_yaw(const void* self, EkfAidSource1d* out);
#endif

#ifdef CONFIG_EKF2_DRAG_FUSION
void ekf2_get_aid_src_drag(const void* self, EkfAidSource2d* out);
#endif

#ifdef CONFIG_EKF2_GRAVITY_FUSION
void ekf2_get_aid_src_gravity(const void* self, EkfAidSource3d* out);
#endif

#ifdef CONFIG_EKF2_AUXVEL
void ekf2_get_aid_src_aux_vel(const void* self, EkfAidSource2d* out);
#endif

#ifdef CONFIG_EKF2_WIND
/** Estimated NE wind velocity [m/s] */
void ekf2_get_wind_velocity(const void* self, float w[2]);

/** Wind velocity variance [(m/s)²] */
void ekf2_get_wind_variance(const void* self, float var[2]);

/** True if wind estimate is available (fusing or externally initialized). */
bool ekf2_wind_estimate_active(const void* self);
#endif

/** True if any horizontal aiding source is active. */
bool ekf2_horizontal_aiding_active(const void* self);
bool ekf2_horizontal_position_aiding_active(const void* self);
bool ekf2_vertical_aiding_active(const void* self);
bool ekf2_north_east_aiding_active(const void* self);
int32_t ekf2_num_active_horizontal_aiding_sources(const void* self);
int32_t ekf2_num_active_horizontal_position_aiding_sources(const void* self);
int32_t ekf2_num_active_horizontal_velocity_aiding_sources(const void* self);
bool ekf2_vertical_position_aiding_active(const void* self);
int32_t ekf2_num_active_vertical_position_aiding_sources(const void* self);
bool ekf2_vertical_velocity_aiding_active(const void* self);
int32_t ekf2_num_active_vertical_velocity_aiding_sources(const void* self);
float ekf2_get_dt_ekf_avg(const void* self);

#ifdef CONFIG_EKF2_BAROMETER
void ekf2_get_baro_bias_estimator_status(const void* self, EkfBiasEstimatorStatus* out);
#endif

#ifdef CONFIG_EKF2_EXTERNAL_VISION
void ekf2_get_ev_hgt_bias_estimator_status(const void* self, EkfBiasEstimatorStatus* out);
void ekf2_get_ev_pos_bias_estimator_status(const void* self, uint8_t axis, EkfBiasEstimatorStatus* out);
#endif

/* =========================================================================
 * Parameters
 * ========================================================================= */

float ekf2_param_get_gyro_noise(const void* self);
void  ekf2_param_set_gyro_noise(void* self, float v);
float ekf2_param_get_accel_noise(const void* self);
void  ekf2_param_set_accel_noise(void* self, float v);
float ekf2_param_get_gyro_bias_noise(const void* self);
void  ekf2_param_set_gyro_bias_noise(void* self, float v);
float ekf2_param_get_accel_bias_noise(const void* self);
void  ekf2_param_set_accel_bias_noise(void* self, float v);
float ekf2_param_get_gyro_bias_init(const void* self);
void  ekf2_param_set_gyro_bias_init(void* self, float v);
float ekf2_param_get_accel_bias_init(const void* self);
void  ekf2_param_set_accel_bias_init(void* self, float v);
float ekf2_param_get_angerr_init(const void* self);
void  ekf2_param_set_angerr_init(void* self, float v);
float ekf2_param_get_delay_max(const void* self);
void  ekf2_param_set_delay_max(void* self, float v);
int32_t ekf2_param_get_predict_us(const void* self);
void    ekf2_param_set_predict_us(void* self, int32_t v);
int32_t ekf2_param_get_imu_ctrl(const void* self);
void    ekf2_param_set_imu_ctrl(void* self, int32_t v);
float   ekf2_param_get_vel_lim(const void* self);
void    ekf2_param_set_vel_lim(void* self, float v);
int32_t ekf2_param_get_position_sensor_ref(const void* self);
void    ekf2_param_set_position_sensor_ref(void* self, int32_t v);
int32_t ekf2_param_get_noaid_tout(const void* self);
void    ekf2_param_set_noaid_tout(void* self, int32_t v);
float ekf2_param_get_noaid_noise(const void* self);
void  ekf2_param_set_noaid_noise(void* self, float v);
float ekf2_param_get_hdg_gate(const void* self);
void  ekf2_param_set_hdg_gate(void* self, float v);
float ekf2_param_get_head_noise(const void* self);
void  ekf2_param_set_head_noise(void* self, float v);
float ekf2_param_get_abl_lim(const void* self);
void  ekf2_param_set_abl_lim(void* self, float v);
float ekf2_param_get_abl_acclim(const void* self);
void  ekf2_param_set_abl_acclim(void* self, float v);
float ekf2_param_get_abl_gyrlim(const void* self);
void  ekf2_param_set_abl_gyrlim(void* self, float v);
float ekf2_param_get_abl_tau(const void* self);
void  ekf2_param_set_abl_tau(void* self, float v);
float ekf2_param_get_gyr_b_lim(const void* self);
void  ekf2_param_set_gyr_b_lim(void* self, float v);

#ifdef CONFIG_EKF2_BAROMETER
int32_t ekf2_param_get_baro_ctrl(const void* self);
void    ekf2_param_set_baro_ctrl(void* self, int32_t v);
float ekf2_param_get_baro_delay(const void* self);
void  ekf2_param_set_baro_delay(void* self, float v);
float ekf2_param_get_baro_noise(const void* self);
void  ekf2_param_set_baro_noise(void* self, float v);
float ekf2_param_get_baro_gate(const void* self);
void  ekf2_param_set_baro_gate(void* self, float v);
float ekf2_param_get_baro_bias_nsd(const void* self);
void  ekf2_param_set_baro_bias_nsd(void* self, float v);
float ekf2_param_get_gnd_eff_dz(const void* self);
void  ekf2_param_set_gnd_eff_dz(void* self, float v);
float ekf2_param_get_gnd_max_hgt(const void* self);
void  ekf2_param_set_gnd_max_hgt(void* self, float v);
# ifdef CONFIG_EKF2_BARO_COMPENSATION
float ekf2_param_get_pcoef_xp(const void* self);
void  ekf2_param_set_pcoef_xp(void* self, float v);
float ekf2_param_get_pcoef_xn(const void* self);
void  ekf2_param_set_pcoef_xn(void* self, float v);
float ekf2_param_get_pcoef_yp(const void* self);
void  ekf2_param_set_pcoef_yp(void* self, float v);
float ekf2_param_get_pcoef_yn(const void* self);
void  ekf2_param_set_pcoef_yn(void* self, float v);
float ekf2_param_get_pcoef_z(const void* self);
void  ekf2_param_set_pcoef_z(void* self, float v);
float ekf2_param_get_aspd_max(const void* self);
void  ekf2_param_set_aspd_max(void* self, float v);
# endif
#endif

#ifdef CONFIG_EKF2_GNSS
int32_t ekf2_param_get_gps_mode(const void* self);
void    ekf2_param_set_gps_mode(void* self, int32_t v);
float ekf2_param_get_gps_delay(const void* self);
void  ekf2_param_set_gps_delay(void* self, float v);
float ekf2_param_get_gps_pos_body_x(const void* self);
void  ekf2_param_set_gps_pos_body_x(void* self, float v);
float ekf2_param_get_gps_pos_body_y(const void* self);
void  ekf2_param_set_gps_pos_body_y(void* self, float v);
float ekf2_param_get_gps_pos_body_z(const void* self);
void  ekf2_param_set_gps_pos_body_z(void* self, float v);
float ekf2_param_get_gps_pos_noise(const void* self);
void  ekf2_param_set_gps_pos_noise(void* self, float v);
float ekf2_param_get_gps_vel_noise(const void* self);
void  ekf2_param_set_gps_vel_noise(void* self, float v);
float ekf2_param_get_gps_hgt_bias_nsd(const void* self);
void  ekf2_param_set_gps_hgt_bias_nsd(void* self, float v);
float ekf2_param_get_gps_pos_gate(const void* self);
void  ekf2_param_set_gps_pos_gate(void* self, float v);
float ekf2_param_get_gps_vel_gate(const void* self);
void  ekf2_param_set_gps_vel_gate(void* self, float v);
int32_t ekf2_param_get_gps_ctrl(const void* self);
void    ekf2_param_set_gps_ctrl(void* self, int32_t v);
int32_t ekf2_param_get_gps_check(const void* self);
void    ekf2_param_set_gps_check(void* self, int32_t v);
float ekf2_param_get_req_eph(const void* self);
void  ekf2_param_set_req_eph(void* self, float v);
float ekf2_param_get_req_epv(const void* self);
void  ekf2_param_set_req_epv(void* self, float v);
float ekf2_param_get_req_sacc(const void* self);
void  ekf2_param_set_req_sacc(void* self, float v);
int32_t ekf2_param_get_req_nsats(const void* self);
void    ekf2_param_set_req_nsats(void* self, int32_t v);
float ekf2_param_get_req_pdop(const void* self);
void  ekf2_param_set_req_pdop(void* self, float v);
float ekf2_param_get_req_hdrift(const void* self);
void  ekf2_param_set_req_hdrift(void* self, float v);
float ekf2_param_get_req_vdrift(const void* self);
void  ekf2_param_set_req_vdrift(void* self, float v);
int32_t ekf2_param_get_req_fix(const void* self);
void    ekf2_param_set_req_fix(void* self, int32_t v);
float ekf2_param_get_gsf_tas(const void* self);
void  ekf2_param_set_gsf_tas(void* self, float v);
# ifdef CONFIG_EKF2_GNSS_YAW
float ekf2_param_get_gnss_heading_noise(const void* self);
void  ekf2_param_set_gnss_heading_noise(void* self, float v);
# endif
#endif

#ifdef CONFIG_EKF2_MAGNETOMETER
float ekf2_param_get_mag_delay(const void* self);
void  ekf2_param_set_mag_delay(void* self, float v);
float ekf2_param_get_mag_noise(const void* self);
void  ekf2_param_set_mag_noise(void* self, float v);
float ekf2_param_get_mag_decl(const void* self);
void  ekf2_param_set_mag_decl(void* self, float v);
float ekf2_param_get_mag_gate(const void* self);
void  ekf2_param_set_mag_gate(void* self, float v);
int32_t ekf2_param_get_decl_type(const void* self);
void    ekf2_param_set_decl_type(void* self, int32_t v);
int32_t ekf2_param_get_mag_type(const void* self);
void    ekf2_param_set_mag_type(void* self, int32_t v);
int32_t ekf2_param_get_mag_check(const void* self);
void    ekf2_param_set_mag_check(void* self, int32_t v);
float ekf2_param_get_mag_chk_str(const void* self);
void  ekf2_param_set_mag_chk_str(void* self, float v);
float ekf2_param_get_mag_chk_inc(const void* self);
void  ekf2_param_set_mag_chk_inc(void* self, float v);
float ekf2_param_get_mag_e_noise(const void* self);
void  ekf2_param_set_mag_e_noise(void* self, float v);
float ekf2_param_get_mag_b_noise(const void* self);
void  ekf2_param_set_mag_b_noise(void* self, float v);
float ekf2_param_get_mag_acclim(const void* self);
void  ekf2_param_set_mag_acclim(void* self, float v);
int32_t ekf2_param_get_synt_mag_z(const void* self);
void    ekf2_param_set_synt_mag_z(void* self, int32_t v);
#endif

int32_t ekf2_param_get_hgt_ref(const void* self);
void    ekf2_param_set_hgt_ref(void* self, int32_t v);

#ifdef CONFIG_EKF2_AIRSPEED
float ekf2_param_get_asp_delay(const void* self);
void  ekf2_param_set_asp_delay(void* self, float v);
float ekf2_param_get_eas_noise(const void* self);
void  ekf2_param_set_eas_noise(void* self, float v);
float ekf2_param_get_tas_gate(const void* self);
void  ekf2_param_set_tas_gate(void* self, float v);
float ekf2_param_get_arsp_thr(const void* self);
void  ekf2_param_set_arsp_thr(void* self, float v);
#endif

#ifdef CONFIG_EKF2_RANGE_FINDER
int32_t ekf2_param_get_rng_ctrl(const void* self);
void    ekf2_param_set_rng_ctrl(void* self, int32_t v);
float ekf2_param_get_rng_delay(const void* self);
void  ekf2_param_set_rng_delay(void* self, float v);
float ekf2_param_get_rng_noise(const void* self);
void  ekf2_param_set_rng_noise(void* self, float v);
float ekf2_param_get_rng_gate(const void* self);
void  ekf2_param_set_rng_gate(void* self, float v);
float ekf2_param_get_rng_pitch(const void* self);
void  ekf2_param_set_rng_pitch(void* self, float v);
float ekf2_param_get_rng_sfe(const void* self);
void  ekf2_param_set_rng_sfe(void* self, float v);
float ekf2_param_get_rng_a_hmax(const void* self);
void  ekf2_param_set_rng_a_hmax(void* self, float v);
float ekf2_param_get_rng_a_vmax(const void* self);
void  ekf2_param_set_rng_a_vmax(void* self, float v);
float ekf2_param_get_rng_qlty_t(const void* self);
void  ekf2_param_set_rng_qlty_t(void* self, float v);
float ekf2_param_get_rng_k_gate(const void* self);
void  ekf2_param_set_rng_k_gate(void* self, float v);
float ekf2_param_get_rng_fog(const void* self);
void  ekf2_param_set_rng_fog(void* self, float v);
float ekf2_param_get_rng_pos_body_x(const void* self);
void  ekf2_param_set_rng_pos_body_x(void* self, float v);
float ekf2_param_get_rng_pos_body_y(const void* self);
void  ekf2_param_set_rng_pos_body_y(void* self, float v);
float ekf2_param_get_rng_pos_body_z(const void* self);
void  ekf2_param_set_rng_pos_body_z(void* self, float v);
#endif

#ifdef CONFIG_EKF2_EXTERNAL_VISION
int32_t ekf2_param_get_ev_ctrl(const void* self);
void    ekf2_param_set_ev_ctrl(void* self, int32_t v);
float ekf2_param_get_ev_delay(const void* self);
void  ekf2_param_set_ev_delay(void* self, float v);
float ekf2_param_get_evv_noise(const void* self);
void  ekf2_param_set_evv_noise(void* self, float v);
float ekf2_param_get_evp_noise(const void* self);
void  ekf2_param_set_evp_noise(void* self, float v);
float ekf2_param_get_eva_noise(const void* self);
void  ekf2_param_set_eva_noise(void* self, float v);
int32_t ekf2_param_get_ev_qmin(const void* self);
void    ekf2_param_set_ev_qmin(void* self, int32_t v);
float ekf2_param_get_evv_gate(const void* self);
void  ekf2_param_set_evv_gate(void* self, float v);
float ekf2_param_get_evp_gate(const void* self);
void  ekf2_param_set_evp_gate(void* self, float v);
float ekf2_param_get_ev_hgt_bias_nsd(const void* self);
void  ekf2_param_set_ev_hgt_bias_nsd(void* self, float v);
float ekf2_param_get_ev_pos_body_x(const void* self);
void  ekf2_param_set_ev_pos_body_x(void* self, float v);
float ekf2_param_get_ev_pos_body_y(const void* self);
void  ekf2_param_set_ev_pos_body_y(void* self, float v);
float ekf2_param_get_ev_pos_body_z(const void* self);
void  ekf2_param_set_ev_pos_body_z(void* self, float v);
#endif

#ifdef CONFIG_EKF2_OPTICAL_FLOW
int32_t ekf2_param_get_of_ctrl(const void* self);
void    ekf2_param_set_of_ctrl(void* self, int32_t v);
int32_t ekf2_param_get_of_gyr_src(const void* self);
void    ekf2_param_set_of_gyr_src(void* self, int32_t v);
float ekf2_param_get_of_delay(const void* self);
void  ekf2_param_set_of_delay(void* self, float v);
float ekf2_param_get_of_n_min(const void* self);
void  ekf2_param_set_of_n_min(void* self, float v);
float ekf2_param_get_of_n_max(const void* self);
void  ekf2_param_set_of_n_max(void* self, float v);
int32_t ekf2_param_get_of_qmin(const void* self);
void    ekf2_param_set_of_qmin(void* self, int32_t v);
int32_t ekf2_param_get_of_qmin_gnd(const void* self);
void    ekf2_param_set_of_qmin_gnd(void* self, int32_t v);
float ekf2_param_get_of_gate(const void* self);
void  ekf2_param_set_of_gate(void* self, float v);
float ekf2_param_get_flow_pos_body_x(const void* self);
void  ekf2_param_set_flow_pos_body_x(void* self, float v);
float ekf2_param_get_flow_pos_body_y(const void* self);
void  ekf2_param_set_flow_pos_body_y(void* self, float v);
float ekf2_param_get_flow_pos_body_z(const void* self);
void  ekf2_param_set_flow_pos_body_z(void* self, float v);
#endif

float ekf2_param_get_imu_pos_body_x(const void* self);
void  ekf2_param_set_imu_pos_body_x(void* self, float v);
float ekf2_param_get_imu_pos_body_y(const void* self);
void  ekf2_param_set_imu_pos_body_y(void* self, float v);
float ekf2_param_get_imu_pos_body_z(const void* self);
void  ekf2_param_set_imu_pos_body_z(void* self, float v);

#ifdef CONFIG_EKF2_WIND
float ekf2_param_get_wind_nsd(const void* self);
void  ekf2_param_set_wind_nsd(void* self, float v);
#endif

#ifdef CONFIG_EKF2_SIDESLIP
int32_t ekf2_param_get_fuse_beta(const void* self);
void    ekf2_param_set_fuse_beta(void* self, int32_t v);
float ekf2_param_get_beta_gate(const void* self);
void  ekf2_param_set_beta_gate(void* self, float v);
float ekf2_param_get_beta_noise(const void* self);
void  ekf2_param_set_beta_noise(void* self, float v);
#endif

#ifdef CONFIG_EKF2_TERRAIN
float ekf2_param_get_terr_noise(const void* self);
void  ekf2_param_set_terr_noise(void* self, float v);
float ekf2_param_get_terr_grad(const void* self);
void  ekf2_param_set_terr_grad(void* self, float v);
#endif

#if defined(CONFIG_EKF2_TERRAIN) || defined(CONFIG_EKF2_OPTICAL_FLOW) || defined(CONFIG_EKF2_RANGE_FINDER)
float ekf2_param_get_min_rng(const void* self);
void  ekf2_param_set_min_rng(void* self, float v);
#endif

#ifdef CONFIG_EKF2_GRAVITY_FUSION
float ekf2_param_get_grav_noise(const void* self);
void  ekf2_param_set_grav_noise(void* self, float v);
#endif

#ifdef CONFIG_EKF2_DRAG_FUSION
int32_t ekf2_param_get_drag_ctrl(const void* self);
void    ekf2_param_set_drag_ctrl(void* self, int32_t v);
float ekf2_param_get_drag_noise(const void* self);
void  ekf2_param_set_drag_noise(void* self, float v);
float ekf2_param_get_bcoef_x(const void* self);
void  ekf2_param_set_bcoef_x(void* self, float v);
float ekf2_param_get_bcoef_y(const void* self);
void  ekf2_param_set_bcoef_y(void* self, float v);
float ekf2_param_get_mcoef(const void* self);
void  ekf2_param_set_mcoef(void* self, float v);
#endif

#ifdef CONFIG_EKF2_AUXVEL
float ekf2_param_get_avel_delay(const void* self);
void  ekf2_param_set_avel_delay(void* self, float v);
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
