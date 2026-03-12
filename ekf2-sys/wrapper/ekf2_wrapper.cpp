/**
 * ekf2_wrapper.cpp — C shims over the PX4 EKF2 C++ API
 *
 * Each function is a thin wrapper around the corresponding Ekf method.
 * Sensor sample structs are converted from the C representation (in the
 * header) to the internal C++ representation via field-by-field copy.
 * static_assert guards verify that sizes match so we catch layout drift
 * early at compile time.
 */

#include "ekf2_wrapper.h"
#include <new>
#include <cstring>   // memcpy

// PX4 EKF headers (from vendor/)
#include <EKF/ekf.h>

// PX4's ekf.h hardcodes these includes, so wrapper/uORB/topics/ provides
// minimal plain-C struct definitions (estimator_aid_sourceXd_s) required to
// compile. These have nothing to do with the uORB pub/sub system — they are
// purely struct definitions that satisfy PX4's internal include paths.
#include <uORB/topics/estimator_aid_source1d.h>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/estimator_aid_source3d.h>

// ── Helpers ──────────────────────────────────────────────────────────────

static inline Ekf* as_ekf(void* p)       { return static_cast<Ekf*>(p); }
static inline const Ekf* as_ekf(const void* p) { return static_cast<const Ekf*>(p); }

#if defined(CONFIG_EKF2_OPTICAL_FLOW) || defined(CONFIG_EKF2_WIND)
static inline void copy_vec2_to_array(const Vector2f &v, float out[2])
{
    out[0] = v(0);
    out[1] = v(1);
}
#endif

static inline void copy_vec3_to_array(const Vector3f &v, float out[3])
{
    out[0] = v(0);
    out[1] = v(1);
    out[2] = v(2);
}

static inline void copy_bias_status_to_c(const BiasEstimator::status &in, EkfBiasEstimatorStatus *out)
{
    if (!out) {
        return;
    }

    out->bias = in.bias;
    out->bias_var = in.bias_var;
    out->innov = in.innov;
    out->innov_var = in.innov_var;
    out->innov_test_ratio = in.innov_test_ratio;
}


// ── Layout verification ───────────────────────────────────────────────────
// These static_asserts catch mismatches between our C structs and the C++
// originals.  A compile error here means the C struct needs updating.

static_assert(sizeof(EkfImuSample) == 48,
    "EkfImuSample must be 48 bytes — update _pad or fields");
static_assert(sizeof(EkfImuSample) == sizeof(imuSample),
    "EkfImuSample / imuSample size mismatch");

#ifdef CONFIG_EKF2_GNSS
static_assert(sizeof(EkfGnssSample) == sizeof(gnssSample),
    "EkfGnssSample size mismatch — check field additions against gnssSample in common.h");
#endif

#ifdef CONFIG_EKF2_MAGNETOMETER
static_assert(sizeof(EkfMagSample) == sizeof(magSample),
    "EkfMagSample size mismatch — check fields against magSample in common.h");
#endif

#ifdef CONFIG_EKF2_BAROMETER
static_assert(sizeof(EkfBaroSample) == sizeof(baroSample),
    "EkfBaroSample size mismatch — check fields against baroSample in common.h");
#endif

#ifdef CONFIG_EKF2_RANGE_FINDER
static_assert(sizeof(EkfRangeSample) == sizeof(estimator::sensor::rangeSample),
    "EkfRangeSample size mismatch — check fields against rangeSample");
#endif

#ifdef CONFIG_EKF2_OPTICAL_FLOW
static_assert(sizeof(EkfFlowSample) == sizeof(flowSample),
    "EkfFlowSample size mismatch — check fields against flowSample in common.h");
#endif

static_assert(sizeof(EkfSystemFlagUpdate) == sizeof(systemFlagUpdate),
    "EkfSystemFlagUpdate size mismatch — check fields against systemFlagUpdate in common.h");

// Aid source struct layout verification.
// Our C structs in ekf2_wrapper.h must be byte-for-byte identical to the
// uORB-generated structs so memcpy in the accessors is correct.
// We verify both overall size AND key field offsets to catch field reordering.
static_assert(sizeof(EkfAidSource1d) == sizeof(estimator_aid_source1d_s),
    "EkfAidSource1d size mismatch — update fields/padding in ekf2_wrapper.h");
static_assert(offsetof(EkfAidSource1d, timestamp) == offsetof(estimator_aid_source1d_s, timestamp),
    "EkfAidSource1d::timestamp offset mismatch");
static_assert(offsetof(EkfAidSource1d, observation) == offsetof(estimator_aid_source1d_s, observation),
    "EkfAidSource1d::observation offset mismatch");
static_assert(offsetof(EkfAidSource1d, innovation) == offsetof(estimator_aid_source1d_s, innovation),
    "EkfAidSource1d::innovation offset mismatch");
static_assert(offsetof(EkfAidSource1d, test_ratio) == offsetof(estimator_aid_source1d_s, test_ratio),
    "EkfAidSource1d::test_ratio offset mismatch");
static_assert(offsetof(EkfAidSource1d, fused) == offsetof(estimator_aid_source1d_s, fused),
    "EkfAidSource1d::fused offset mismatch");

static_assert(sizeof(EkfAidSource2d) == sizeof(estimator_aid_source2d_s),
    "EkfAidSource2d size mismatch — update fields/padding in ekf2_wrapper.h");
static_assert(offsetof(EkfAidSource2d, timestamp) == offsetof(estimator_aid_source2d_s, timestamp),
    "EkfAidSource2d::timestamp offset mismatch");
static_assert(offsetof(EkfAidSource2d, observation) == offsetof(estimator_aid_source2d_s, observation),
    "EkfAidSource2d::observation offset mismatch");
static_assert(offsetof(EkfAidSource2d, innovation) == offsetof(estimator_aid_source2d_s, innovation),
    "EkfAidSource2d::innovation offset mismatch");
static_assert(offsetof(EkfAidSource2d, test_ratio) == offsetof(estimator_aid_source2d_s, test_ratio),
    "EkfAidSource2d::test_ratio offset mismatch");
static_assert(offsetof(EkfAidSource2d, fused) == offsetof(estimator_aid_source2d_s, fused),
    "EkfAidSource2d::fused offset mismatch");

static_assert(sizeof(EkfAidSource3d) == sizeof(estimator_aid_source3d_s),
    "EkfAidSource3d size mismatch — update fields/padding in ekf2_wrapper.h");
static_assert(offsetof(EkfAidSource3d, timestamp) == offsetof(estimator_aid_source3d_s, timestamp),
    "EkfAidSource3d::timestamp offset mismatch");
static_assert(offsetof(EkfAidSource3d, observation) == offsetof(estimator_aid_source3d_s, observation),
    "EkfAidSource3d::observation offset mismatch");
static_assert(offsetof(EkfAidSource3d, innovation) == offsetof(estimator_aid_source3d_s, innovation),
    "EkfAidSource3d::innovation offset mismatch");
static_assert(offsetof(EkfAidSource3d, test_ratio) == offsetof(estimator_aid_source3d_s, test_ratio),
    "EkfAidSource3d::test_ratio offset mismatch");
static_assert(offsetof(EkfAidSource3d, fused) == offsetof(estimator_aid_source3d_s, fused),
    "EkfAidSource3d::fused offset mismatch");

static_assert(sizeof(EkfBiasEstimatorStatus) == sizeof(BiasEstimator::status),
    "EkfBiasEstimatorStatus size mismatch — update fields in ekf2_wrapper.h");

// ── Lifecycle ─────────────────────────────────────────────────────────────

extern "C" size_t ekf2_sizeof()
{
    return sizeof(Ekf);
}

extern "C" size_t ekf2_alignof()
{
    return alignof(Ekf);
}

extern "C" void* ekf2_create_heap()
{
    return new (std::nothrow) Ekf();
}

extern "C" void ekf2_destroy_heap(void* self)
{
    if (!self) {
        return;
    }
    delete as_ekf(self);
}

extern "C" void* ekf2_create(void* ekf_buf, size_t ekf_buf_size)
{
    if (ekf_buf_size < sizeof(Ekf)) {
        return nullptr;
    }
    return ::new (ekf_buf) Ekf();
}

extern "C" bool ekf2_init(void* self, uint64_t timestamp_us)
{
    return as_ekf(self)->init(timestamp_us);
}

extern "C" bool ekf2_update(void* self)
{
    return as_ekf(self)->update();
}

extern "C" void ekf2_destroy(void* self)
{
    as_ekf(self)->~Ekf();
}

// ── IMU (always present) ──────────────────────────────────────────────────

extern "C" void ekf2_set_imu_data(void* self, const EkfImuSample* s)
{
    imuSample cpp{};
    cpp.time_us        = s->time_us;
    cpp.delta_ang      = Vector3f{s->delta_ang[0], s->delta_ang[1], s->delta_ang[2]};
    cpp.delta_vel      = Vector3f{s->delta_vel[0], s->delta_vel[1], s->delta_vel[2]};
    cpp.delta_ang_dt   = s->delta_ang_dt;
    cpp.delta_vel_dt   = s->delta_vel_dt;
    cpp.delta_vel_clipping[0] = s->delta_vel_clipping[0] != 0;
    cpp.delta_vel_clipping[1] = s->delta_vel_clipping[1] != 0;
    cpp.delta_vel_clipping[2] = s->delta_vel_clipping[2] != 0;
    as_ekf(self)->setIMUData(cpp);
}

// ── GNSS ──────────────────────────────────────────────────────────────────

#ifdef CONFIG_EKF2_GNSS
extern "C" void ekf2_set_gps_data(void* self, const EkfGnssSample* s)
{
    gnssSample cpp{};
    cpp.time_us        = s->time_us;
    cpp.lat            = s->lat;
    cpp.lon            = s->lon;
    cpp.alt            = s->alt;
    cpp.yaw            = s->yaw;
    cpp.yaw_acc        = s->yaw_acc;
    cpp.yaw_offset     = s->yaw_offset;
    cpp.vel            = Vector3f{s->vel[0], s->vel[1], s->vel[2]};
    cpp.hacc           = s->hacc;
    cpp.vacc           = s->vacc;
    cpp.sacc           = s->sacc;
    cpp.pdop           = s->pdop;
    cpp.fix_type       = s->fix_type;
    cpp.nsats          = s->nsats;
    cpp.spoofed        = s->spoofed != 0;
    cpp.jammed         = s->jammed != 0;
    as_ekf(self)->setGpsData(cpp);
}
#endif /* CONFIG_EKF2_GNSS */

// ── Magnetometer ─────────────────────────────────────────────────────────

#ifdef CONFIG_EKF2_MAGNETOMETER
extern "C" void ekf2_set_mag_data(void* self, const EkfMagSample* s)
{
    magSample cpp{};
    cpp.time_us = s->time_us;
    cpp.mag     = Vector3f{s->mag[0], s->mag[1], s->mag[2]};
    cpp.reset   = s->reset != 0;
    as_ekf(self)->setMagData(cpp);
}
#endif /* CONFIG_EKF2_MAGNETOMETER */

// ── Barometer ─────────────────────────────────────────────────────────────

#ifdef CONFIG_EKF2_BAROMETER
extern "C" void ekf2_set_baro_data(void* self, const EkfBaroSample* s)
{
    baroSample cpp{};
    cpp.time_us = s->time_us;
    cpp.hgt     = s->hgt;
    cpp.reset   = s->reset != 0;
    as_ekf(self)->setBaroData(cpp);
}
#endif /* CONFIG_EKF2_BAROMETER */

// ── Airspeed ──────────────────────────────────────────────────────────────

#ifdef CONFIG_EKF2_AIRSPEED
extern "C" void ekf2_set_airspeed_data(void* self, const EkfAirspeedSample* s)
{
    airspeedSample cpp{};
    cpp.time_us        = s->time_us;
    cpp.true_airspeed  = s->true_airspeed;
    cpp.eas2tas        = s->eas2tas;
    as_ekf(self)->setAirspeedData(cpp);
}
#endif /* CONFIG_EKF2_AIRSPEED */

// ── Range finder ──────────────────────────────────────────────────────────

#ifdef CONFIG_EKF2_RANGE_FINDER
extern "C" void ekf2_set_range_data(void* self, const EkfRangeSample* s)
{
    estimator::sensor::rangeSample cpp{};
    cpp.time_us = s->time_us;
    cpp.rng     = s->rng;
    cpp.quality = s->quality;
    as_ekf(self)->setRangeData(cpp);
}
#endif /* CONFIG_EKF2_RANGE_FINDER */

// ── Optical flow ──────────────────────────────────────────────────────────

#ifdef CONFIG_EKF2_OPTICAL_FLOW
extern "C" void ekf2_set_optical_flow_data(void* self, const EkfFlowSample* s)
{
    flowSample cpp{};
    cpp.time_us   = s->time_us;
    cpp.flow_rate = Vector2f{s->flow_rate_xy[0], s->flow_rate_xy[1]};
    cpp.gyro_rate = Vector3f{s->gyro_rate_xyz[0], s->gyro_rate_xyz[1], s->gyro_rate_xyz[2]};
    cpp.quality   = s->quality;
    as_ekf(self)->setOpticalFlowData(cpp);
}
#endif /* CONFIG_EKF2_OPTICAL_FLOW */

// ── External vision ───────────────────────────────────────────────────────

#ifdef CONFIG_EKF2_EXTERNAL_VISION
extern "C" void ekf2_set_ev_data(void* self, const EkfExtVisionSample* s)
{
    extVisionSample cpp{};
    cpp.time_us = s->time_us;
    cpp.pos     = Vector3f{s->pos[0], s->pos[1], s->pos[2]};
    cpp.quat    = Quatf{s->quat[0], s->quat[1], s->quat[2], s->quat[3]};
    cpp.vel     = Vector3f{s->vel[0], s->vel[1], s->vel[2]};
    cpp.position_var    = Vector3f{s->pos_var[0], s->pos_var[1], s->pos_var[2]};
    cpp.velocity_var    = Vector3f{s->vel_var[0], s->vel_var[1], s->vel_var[2]};
    cpp.orientation_var = Vector3f{s->ang_var[0], s->ang_var[1], s->ang_var[2]};
    cpp.pos_frame       = static_cast<PositionFrame>(s->pos_frame);
    cpp.vel_frame       = static_cast<VelocityFrame>(s->vel_frame);
    cpp.reset_counter   = s->reset_counter;
    cpp.quality         = s->quality;
    as_ekf(self)->setExtVisionData(cpp);
}
#endif /* CONFIG_EKF2_EXTERNAL_VISION */

// ── Auxiliary velocity ────────────────────────────────────────────────────

#ifdef CONFIG_EKF2_AUXVEL
extern "C" void ekf2_set_aux_vel_data(void* self, const EkfAuxVelSample* s)
{
    auxVelSample cpp{};
    cpp.time_us = s->time_us;
    cpp.vel     = Vector2f{s->vel[0], s->vel[1]};
    cpp.velVar  = Vector2f{s->vel_var[0], s->vel_var[1]};
    as_ekf(self)->setAuxVelData(cpp);
}
#endif /* CONFIG_EKF2_AUXVEL */

// ── Vehicle/system flag inputs ────────────────────────────────────────────

extern "C" void ekf2_set_system_flag_data(void* self, const EkfSystemFlagUpdate* s)
{
    systemFlagUpdate cpp{};
    cpp.time_us             = s->time_us;
    cpp.at_rest             = s->at_rest;
    cpp.in_air              = s->in_air;
    cpp.is_fixed_wing       = s->is_fixed_wing;
    cpp.gnd_effect          = s->gnd_effect;
    cpp.constant_pos        = s->constant_pos;
    cpp.in_transition_to_fw = s->in_transition_to_fw;
    as_ekf(self)->setSystemFlagData(cpp);
}

extern "C" void ekf2_set_in_air_status(void* self, bool in_air)
{
    as_ekf(self)->set_in_air_status(in_air);
}

extern "C" void ekf2_set_vehicle_at_rest(void* self, bool at_rest)
{
    as_ekf(self)->set_vehicle_at_rest(at_rest);
}

extern "C" void ekf2_set_constant_pos(void* self, bool constant_pos)
{
    as_ekf(self)->set_constant_pos(constant_pos);
}

extern "C" void ekf2_set_is_fixed_wing(void* self, bool is_fixed_wing)
{
    as_ekf(self)->set_is_fixed_wing(is_fixed_wing);
}

extern "C" void ekf2_set_in_transition_to_fw(void* self, bool in_transition_to_fw)
{
    as_ekf(self)->set_in_transition_to_fw(in_transition_to_fw);
}

extern "C" void ekf2_set_gnd_effect(void* self)
{
    as_ekf(self)->set_gnd_effect();
}

extern "C" void ekf2_set_air_density(void* self, float air_density)
{
    as_ekf(self)->set_air_density(air_density);
}

#ifdef CONFIG_EKF2_GNSS
extern "C" void ekf2_set_gps_data_with_pps(void* self, const EkfGnssSample* s, bool pps_compensation)
{
    gnssSample cpp{};
    cpp.time_us        = s->time_us;
    cpp.lat            = s->lat;
    cpp.lon            = s->lon;
    cpp.alt            = s->alt;
    cpp.yaw            = s->yaw;
    cpp.yaw_acc        = s->yaw_acc;
    cpp.yaw_offset     = s->yaw_offset;
    cpp.vel            = Vector3f{s->vel[0], s->vel[1], s->vel[2]};
    cpp.hacc           = s->hacc;
    cpp.vacc           = s->vacc;
    cpp.sacc           = s->sacc;
    cpp.pdop           = s->pdop;
    cpp.fix_type       = s->fix_type;
    cpp.nsats          = s->nsats;
    cpp.spoofed        = s->spoofed != 0;
    cpp.jammed         = s->jammed != 0;
    as_ekf(self)->setGpsData(cpp, pps_compensation);
}
#endif

#ifdef CONFIG_EKF2_AIRSPEED
extern "C" void ekf2_set_synthetic_airspeed(void* self, bool synthetic_airspeed)
{
    as_ekf(self)->setSyntheticAirspeed(synthetic_airspeed);
}
#endif

#ifdef CONFIG_EKF2_RANGE_FINDER
extern "C" void ekf2_set_rangefinder_limits(void* self, float min_distance, float max_distance)
{
    as_ekf(self)->set_rangefinder_limits(min_distance, max_distance);
}
#endif

#ifdef CONFIG_EKF2_OPTICAL_FLOW
extern "C" void ekf2_set_optical_flow_limits(void* self, float max_flow_rate, float min_distance, float max_distance)
{
    as_ekf(self)->set_optical_flow_limits(max_flow_rate, min_distance, max_distance);
}
#endif

// ── State outputs ─────────────────────────────────────────────────────────

extern "C" void ekf2_get_quaternion(const void* self, float q[4])
{
    Quatf quat = as_ekf(self)->getQuaternion();
    q[0] = quat(0);  // w
    q[1] = quat(1);  // x
    q[2] = quat(2);  // y
    q[3] = quat(3);  // z
}

extern "C" void ekf2_get_velocity(const void* self, float v[3])
{
    Vector3f vel = as_ekf(self)->getVelocity();
    v[0] = vel(0); v[1] = vel(1); v[2] = vel(2);
}

extern "C" void ekf2_get_position(const void* self, float p[3])
{
    Vector3f pos = as_ekf(self)->getPosition();
    p[0] = pos(0); p[1] = pos(1); p[2] = pos(2);
}

extern "C" void ekf2_get_gyro_bias(const void* self, float b[3])
{
    Vector3f bias = as_ekf(self)->getGyroBias();
    b[0] = bias(0); b[1] = bias(1); b[2] = bias(2);
}

extern "C" void ekf2_get_accel_bias(const void* self, float b[3])
{
    Vector3f bias = as_ekf(self)->getAccelBias();
    b[0] = bias(0); b[1] = bias(1); b[2] = bias(2);
}

#ifdef CONFIG_EKF2_MAGNETOMETER
extern "C" void ekf2_get_mag_earth_field(const void* self, float mag[3])
{
    copy_vec3_to_array(as_ekf(self)->getMagEarthField(), mag);
}

extern "C" void ekf2_get_mag_bias(const void* self, float bias[3])
{
    copy_vec3_to_array(as_ekf(self)->getMagBias(), bias);
}

extern "C" void ekf2_get_mag_bias_variance(const void* self, float var[3])
{
    copy_vec3_to_array(as_ekf(self)->getMagBiasVariance(), var);
}
#endif

extern "C" void ekf2_get_angular_velocity_reset_accumulator(void* self, float w[3])
{
    Vector3f omega = as_ekf(self)->getAngularVelocityAndResetAccumulator();
    w[0] = omega(0); w[1] = omega(1); w[2] = omega(2);
}

extern "C" float ekf2_get_unaided_yaw(const void* self)
{
    return as_ekf(self)->getUnaidedYaw();
}

extern "C" bool ekf2_attitude_valid(const void* self)
{
    return as_ekf(self)->attitude_valid();
}

extern "C" bool ekf2_local_position_valid(const void* self)
{
    return as_ekf(self)->isLocalHorizontalPositionValid()
        && as_ekf(self)->isLocalVerticalPositionValid();
}

extern "C" bool ekf2_local_velocity_valid(const void* self)
{
    // PX4 does not expose a separate isLocalHorizontalVelocityValid(); it uses
    // isLocalHorizontalPositionValid() for both position and velocity validity
    // (see ekf_helper.cpp: soln_status.flags.velocity_horiz = isLocalHorizontalPositionValid()).
    // For vertical, velocity is valid if either the velocity OR position deadreckoning
    // timeout has not been exceeded, matching PX4's velocity_vert flag.
    return as_ekf(self)->isLocalHorizontalPositionValid()
        && (as_ekf(self)->isLocalVerticalVelocityValid()
            || as_ekf(self)->isLocalVerticalPositionValid());
}

extern "C" void ekf2_get_pos_variance(const void* self, float var[3])
{
    Vector3f v = as_ekf(self)->getPositionVariance();
    var[0] = v(0); var[1] = v(1); var[2] = v(2);
}

extern "C" void ekf2_get_vel_variance(const void* self, float var[3])
{
    Vector3f v = as_ekf(self)->getVelocityVariance();
    var[0] = v(0); var[1] = v(1); var[2] = v(2);
}

extern "C" bool ekf2_global_position_valid(const void* self)
{
    return as_ekf(self)->isGlobalHorizontalPositionValid()
        && as_ekf(self)->isGlobalVerticalPositionValid();
}

extern "C" bool ekf2_yaw_align_complete(const void* self)
{
    return as_ekf(self)->isYawFinalAlignComplete();
}

extern "C" void ekf2_get_gyro_bias_variance(const void* self, float var[3])
{
    Vector3f v = as_ekf(self)->getGyroBiasVariance();
    var[0] = v(0); var[1] = v(1); var[2] = v(2);
}

extern "C" void ekf2_get_accel_bias_variance(const void* self, float var[3])
{
    Vector3f v = as_ekf(self)->getAccelBiasVariance();
    var[0] = v(0); var[1] = v(1); var[2] = v(2);
}

extern "C" void ekf2_get_rot_var_body(const void* self, float var[3])
{
    copy_vec3_to_array(as_ekf(self)->getRotVarBody(), var);
}

extern "C" void ekf2_get_rot_var_ned(const void* self, float var[3])
{
    copy_vec3_to_array(as_ekf(self)->getRotVarNed(), var);
}

extern "C" float ekf2_get_yaw_variance(const void* self)
{
    return as_ekf(self)->getYawVar();
}

extern "C" float ekf2_get_tilt_variance(const void* self)
{
    return as_ekf(self)->getTiltVariance();
}

extern "C" size_t ekf2_get_state_size(void)
{
    return State::size;
}

extern "C" size_t ekf2_get_covariances_diagonal(const void* self, float* out, size_t capacity)
{
    const size_t required = State::size;

    if (out && capacity > 0) {
        const matrix::Vector<float, State::size> diag = as_ekf(self)->covariances_diagonal();
        const size_t count = (capacity < required) ? capacity : required;

        for (size_t i = 0; i < count; ++i) {
            out[i] = diag(i);
        }
    }

    return required;
}

extern "C" float ekf2_get_state_covariance(const void* self, uint32_t row, uint32_t col)
{
    if ((row >= State::size) || (col >= State::size)) {
        return NAN;
    }

    return as_ekf(self)->stateCovariance(row, col);
}

extern "C" void ekf2_get_velocity_derivative(const void* self, float vdot[3])
{
    Vector3f v = as_ekf(self)->getVelocityDerivative();
    vdot[0] = v(0); vdot[1] = v(1); vdot[2] = v(2);
}

extern "C" void ekf2_reset_velocity_derivative_accumulation(void* self)
{
    as_ekf(self)->resetVelocityDerivativeAccumulation();
}

extern "C" float ekf2_get_vertical_position_derivative(const void* self)
{
    return as_ekf(self)->getVerticalPositionDerivative();
}

extern "C" void ekf2_get_lat_lon_alt(const void* self, double* lat, double* lon, float* alt)
{
    const LatLonAlt lla = as_ekf(self)->getLatLonAlt();
    *lat = lla.latitude_deg();
    *lon = lla.longitude_deg();
    *alt = lla.altitude();
}

extern "C" void ekf2_get_output_tracking_error(const void* self, float err[3])
{
    const Vector3f &e = as_ekf(self)->getOutputTrackingError();
    err[0] = e(0); err[1] = e(1); err[2] = e(2);
}

extern "C" uint64_t ekf2_get_control_status(const void* self)
{
    return as_ekf(self)->control_status().value;
}

extern "C" uint32_t ekf2_get_fault_status(const void* self)
{
    return as_ekf(self)->fault_status().value;
}

extern "C" uint32_t ekf2_get_information_event_status(const void* self)
{
    return as_ekf(self)->information_event_status().value;
}

extern "C" void ekf2_clear_information_events(void* self)
{
    as_ekf(self)->clear_information_events();
}

// ── Innovation test ratios ────────────────────────────────────────────────

extern "C" float ekf2_get_heading_innov_test_ratio(const void* self)
{
    return as_ekf(self)->getHeadingInnovationTestRatio();
}

extern "C" float ekf2_get_horiz_vel_innov_test_ratio(const void* self)
{
    return as_ekf(self)->getHorizontalVelocityInnovationTestRatio();
}

extern "C" float ekf2_get_vert_vel_innov_test_ratio(const void* self)
{
    return as_ekf(self)->getVerticalVelocityInnovationTestRatio();
}

extern "C" float ekf2_get_horiz_pos_innov_test_ratio(const void* self)
{
    return as_ekf(self)->getHorizontalPositionInnovationTestRatio();
}

extern "C" float ekf2_get_vert_pos_innov_test_ratio(const void* self)
{
    return as_ekf(self)->getVerticalPositionInnovationTestRatio();
}

#ifdef CONFIG_EKF2_AIRSPEED
extern "C" float ekf2_get_airspeed_innov_test_ratio(const void* self)
{
    return as_ekf(self)->getAirspeedInnovationTestRatio();
}
#endif

#ifdef CONFIG_EKF2_SIDESLIP
extern "C" float ekf2_get_sideslip_innov_test_ratio(const void* self)
{
    return as_ekf(self)->getSyntheticSideslipInnovationTestRatio();
}
#endif

#ifdef CONFIG_EKF2_RANGE_FINDER
extern "C" float ekf2_get_hagl_rate_innov(const void* self)
{
    return as_ekf(self)->getHaglRateInnov();
}

extern "C" float ekf2_get_hagl_rate_innov_var(const void* self)
{
    return as_ekf(self)->getHaglRateInnovVar();
}

extern "C" float ekf2_get_hagl_rate_innov_ratio(const void* self)
{
    return as_ekf(self)->getHaglRateInnovRatio();
}
#endif

#if defined(CONFIG_EKF2_TERRAIN) || defined(CONFIG_EKF2_RANGE_FINDER) || defined(CONFIG_EKF2_OPTICAL_FLOW)
extern "C" float ekf2_get_hagl_innov_test_ratio(const void* self)
{
    return as_ekf(self)->getHeightAboveGroundInnovationTestRatio();
}
#endif

// ── Solution status and height sensor ref ─────────────────────────────────

extern "C" uint16_t ekf2_get_soln_status(const void* self)
{
    return as_ekf(self)->get_ekf_soln_status();
}

extern "C" uint8_t ekf2_get_height_sensor_ref(const void* self)
{
    return static_cast<uint8_t>(as_ekf(self)->getHeightSensorRef());
}

// ── Accuracy bounds ───────────────────────────────────────────────────────

extern "C" void ekf2_get_lpos_accuracy(const void* self, float* eph, float* epv)
{
    as_ekf(self)->get_ekf_lpos_accuracy(eph, epv);
}

extern "C" void ekf2_get_gpos_accuracy(const void* self, float* eph, float* epv)
{
    as_ekf(self)->get_ekf_gpos_accuracy(eph, epv);
}

extern "C" void ekf2_get_vel_accuracy(const void* self, float* evh, float* evv)
{
    as_ekf(self)->get_ekf_vel_accuracy(evh, evv);
}

extern "C" void ekf2_get_ctrl_limits(const void* self, float* vxy_max, float* vz_max,
                                      float* hagl_min, float* hagl_max_z, float* hagl_max_xy)
{
    as_ekf(self)->get_ekf_ctrl_limits(vxy_max, vz_max, hagl_min, hagl_max_z, hagl_max_xy);
}

// ── Global origin ─────────────────────────────────────────────────────────

extern "C" void ekf2_get_global_origin(const void* self, uint64_t* time_us,
                                        double* lat, double* lon, float* alt)
{
    as_ekf(self)->getEkfGlobalOrigin(*time_us, *lat, *lon, *alt);
}

extern "C" bool ekf2_set_global_origin(void* self, double lat, double lon, float alt,
                                        float hpos_var, float vpos_var)
{
    return as_ekf(self)->setEkfGlobalOrigin(lat, lon, alt, hpos_var, vpos_var);
}

extern "C" bool ekf2_reset_global_position(void* self, double lat, double lon, float alt,
                                            float hpos_var, float vpos_var)
{
    return as_ekf(self)->resetGlobalPositionTo(lat, lon, alt, hpos_var, vpos_var);
}

extern "C" bool ekf2_reset_global_position_to_external_observation(
    void* self,
    double lat,
    double lon,
    float alt,
    float eph,
    float epv,
    uint64_t timestamp_observation)
{
    return as_ekf(self)->resetGlobalPosToExternalObservation(lat, lon, alt, eph, epv, timestamp_observation);
}

// ── State reset tracking ──────────────────────────────────────────────────

extern "C" uint8_t ekf2_get_quat_reset_count(const void* self)   { return as_ekf(self)->get_quat_reset_count(); }
extern "C" uint8_t ekf2_get_pos_ne_reset_count(const void* self) { return as_ekf(self)->get_posNE_reset_count(); }
extern "C" uint8_t ekf2_get_vel_ne_reset_count(const void* self) { return as_ekf(self)->get_velNE_reset_count(); }
extern "C" uint8_t ekf2_get_pos_d_reset_count(const void* self)  { return as_ekf(self)->get_posD_reset_count(); }
extern "C" uint8_t ekf2_get_vel_d_reset_count(const void* self)  { return as_ekf(self)->get_velD_reset_count(); }
extern "C" uint8_t ekf2_get_hagl_reset_count(const void* self)   { return as_ekf(self)->get_hagl_reset_count(); }

extern "C" void ekf2_get_quat_reset(const void* self, float delta_quat[4], uint8_t* count)
{
    as_ekf(self)->get_quat_reset(delta_quat, count);
}

extern "C" void ekf2_get_pos_ne_reset(const void* self, float delta_ne[2], uint8_t* count)
{
    as_ekf(self)->get_posNE_reset(delta_ne, count);
}

extern "C" void ekf2_get_vel_ne_reset(const void* self, float delta_ne[2], uint8_t* count)
{
    as_ekf(self)->get_velNE_reset(delta_ne, count);
}

extern "C" void ekf2_get_pos_d_reset(const void* self, float* delta, uint8_t* count)
{
    as_ekf(self)->get_posD_reset(delta, count);
}

extern "C" void ekf2_get_vel_d_reset(const void* self, float* delta, uint8_t* count)
{
    as_ekf(self)->get_velD_reset(delta, count);
}

extern "C" void ekf2_get_hagl_reset(const void* self, float* delta, uint8_t* count)
{
    as_ekf(self)->get_hagl_reset(delta, count);
}

// ── Additional estimator diagnostics and control ──────────────────────────

extern "C" void ekf2_reset_gyro_bias(void* self)
{
    as_ekf(self)->resetGyroBias();
}

extern "C" void ekf2_reset_accel_bias(void* self)
{
    as_ekf(self)->resetAccelBias();
}

extern "C" void ekf2_reset_gyro_bias_cov(void* self)
{
    as_ekf(self)->resetGyroBiasCov();
}

extern "C" void ekf2_reset_accel_bias_cov(void* self)
{
    as_ekf(self)->resetAccelBiasCov();
}

extern "C" void ekf2_reset_heading_to_external_observation(void* self, float heading, float heading_accuracy)
{
    as_ekf(self)->resetHeadingToExternalObservation(heading, heading_accuracy);
}

extern "C" void ekf2_update_parameters(void* self)
{
    as_ekf(self)->updateParameters();
}

extern "C" bool ekf2_check_lat_lon_validity(void* self, double lat, double lon)
{
    return as_ekf(self)->checkLatLonValidity(lat, lon);
}

extern "C" bool ekf2_check_altitude_validity(void* self, float altitude)
{
    return as_ekf(self)->checkAltitudeValidity(altitude);
}

#ifdef CONFIG_EKF2_WIND
extern "C" void ekf2_reset_wind_to_external_observation(
    void* self,
    float wind_speed,
    float wind_direction,
    float wind_speed_accuracy,
    float wind_direction_accuracy)
{
    as_ekf(self)->resetWindToExternalObservation(wind_speed, wind_direction, wind_speed_accuracy, wind_direction_accuracy);
}
#endif

#ifdef CONFIG_EKF2_MAGNETOMETER
extern "C" bool ekf2_update_world_magnetic_model(void* self, double latitude_deg, double longitude_deg)
{
    return as_ekf(self)->updateWorldMagneticModel(latitude_deg, longitude_deg);
}
#endif

#ifdef CONFIG_EKF2_GNSS
extern "C" float ekf2_get_gps_horizontal_position_drift_rate(const void* self)
{
    return as_ekf(self)->gps_horizontal_position_drift_rate_m_s();
}

extern "C" float ekf2_get_gps_vertical_position_drift_rate(const void* self)
{
    return as_ekf(self)->gps_vertical_position_drift_rate_m_s();
}

extern "C" float ekf2_get_gps_filtered_horizontal_velocity(const void* self)
{
    return as_ekf(self)->gps_filtered_horizontal_velocity_m_s();
}

extern "C" bool ekf2_gps_checks_passed(const void* self)
{
    return as_ekf(self)->gps_checks_passed();
}

extern "C" uint16_t ekf2_get_gps_check_fail_status(const void* self)
{
    return as_ekf(self)->gps_check_fail_status().value;
}

extern "C" void ekf2_set_min_required_gps_health_time(void* self, uint32_t time_us)
{
    as_ekf(self)->set_min_required_gps_health_time(time_us);
}

extern "C" void ekf2_get_gps_hgt_bias_estimator_status(const void* self, EkfBiasEstimatorStatus* out)
{
    copy_bias_status_to_c(as_ekf(self)->getGpsHgtBiasEstimatorStatus(), out);
}

extern "C" bool ekf2_yaw_emergency_estimate_available(const void* self)
{
    return as_ekf(self)->isYawEmergencyEstimateAvailable();
}

extern "C" bool ekf2_get_ekfgsf_data(
    const void* self,
    float* yaw_composite,
    float* yaw_variance,
    float* yaw,
    float* innov_vn,
    float* innov_ve,
    float* weight,
    size_t capacity,
    size_t* count_out)
{
    if (count_out) {
        *count_out = N_MODELS_EKFGSF;
    }

    if (capacity < N_MODELS_EKFGSF) {
        return false;
    }

    return const_cast<Ekf*>(as_ekf(self))->getDataEKFGSF(
        yaw_composite, yaw_variance, yaw, innov_vn, innov_ve, weight);
}
#endif

// ── Terrain ───────────────────────────────────────────────────────────────

#if defined(CONFIG_EKF2_TERRAIN) || defined(CONFIG_EKF2_RANGE_FINDER) || defined(CONFIG_EKF2_OPTICAL_FLOW)
extern "C" bool  ekf2_terrain_valid(const void* self)          { return as_ekf(self)->isTerrainEstimateValid(); }
extern "C" float ekf2_get_terrain_vert_pos(const void* self)   { return as_ekf(self)->getTerrainVertPos(); }
extern "C" float ekf2_get_terrain_variance(const void* self)   { return as_ekf(self)->getTerrainVariance(); }
extern "C" float ekf2_get_hagl(const void* self)               { return as_ekf(self)->getHagl(); }
#endif

#ifdef CONFIG_EKF2_OPTICAL_FLOW
extern "C" void ekf2_get_flow_vel_body(const void* self, float v[2])
{
    copy_vec2_to_array(as_ekf(self)->getFlowVelBody(), v);
}

extern "C" void ekf2_get_flow_vel_ne(const void* self, float v[2])
{
    copy_vec2_to_array(as_ekf(self)->getFlowVelNE(), v);
}

extern "C" void ekf2_get_filtered_flow_vel_body(const void* self, float v[2])
{
    copy_vec2_to_array(as_ekf(self)->getFilteredFlowVelBody(), v);
}

extern "C" void ekf2_get_filtered_flow_vel_ne(const void* self, float v[2])
{
    copy_vec2_to_array(as_ekf(self)->getFilteredFlowVelNE(), v);
}

extern "C" void ekf2_get_flow_compensated(const void* self, float flow_rate[2])
{
    copy_vec2_to_array(as_ekf(self)->getFlowCompensated(), flow_rate);
}

extern "C" void ekf2_get_flow_uncompensated(const void* self, float flow_rate[2])
{
    copy_vec2_to_array(as_ekf(self)->getFlowUncompensated(), flow_rate);
}

extern "C" void ekf2_get_flow_gyro(const void* self, float gyro_rate[3])
{
    copy_vec3_to_array(as_ekf(self)->getFlowGyro(), gyro_rate);
}

extern "C" void ekf2_get_flow_gyro_bias(const void* self, float gyro_bias[3])
{
    copy_vec3_to_array(as_ekf(self)->getFlowGyroBias(), gyro_bias);
}

extern "C" void ekf2_get_flow_ref_body_rate(const void* self, float body_rate[3])
{
    copy_vec3_to_array(as_ekf(self)->getFlowRefBodyRate(), body_rate);
}
#endif

// ── Aid source diagnostics ────────────────────────────────────────────────
// Each function copies the PX4-internal struct into the caller's output
// buffer via memcpy.  The static_asserts at the top of this file guarantee
// that EkfAidSource{1,2,3}d is byte-for-byte identical to the corresponding
// estimator_aid_source{1,2,3}d_s type.

extern "C" void ekf2_get_aid_src_fake_hgt(const void* self, EkfAidSource1d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_fake_hgt(), sizeof(*out));
}

extern "C" void ekf2_get_aid_src_fake_pos(const void* self, EkfAidSource2d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_fake_pos(), sizeof(*out));
}

#ifdef CONFIG_EKF2_BAROMETER
extern "C" void ekf2_get_aid_src_baro_hgt(const void* self, EkfAidSource1d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_baro_hgt(), sizeof(*out));
}
#endif

#ifdef CONFIG_EKF2_GNSS
extern "C" void ekf2_get_aid_src_gnss_hgt(const void* self, EkfAidSource1d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_gnss_hgt(), sizeof(*out));
}

extern "C" void ekf2_get_aid_src_gnss_pos(const void* self, EkfAidSource2d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_gnss_pos(), sizeof(*out));
}

extern "C" void ekf2_get_aid_src_gnss_vel(const void* self, EkfAidSource3d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_gnss_vel(), sizeof(*out));
}

#  ifdef CONFIG_EKF2_GNSS_YAW
extern "C" void ekf2_get_aid_src_gnss_yaw(const void* self, EkfAidSource1d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_gnss_yaw(), sizeof(*out));
}
#  endif
#endif

#ifdef CONFIG_EKF2_MAGNETOMETER
extern "C" void ekf2_get_aid_src_mag(const void* self, EkfAidSource3d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_mag(), sizeof(*out));
}
#endif

#ifdef CONFIG_EKF2_AIRSPEED
extern "C" void ekf2_get_aid_src_airspeed(const void* self, EkfAidSource1d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_airspeed(), sizeof(*out));
}
#endif

#ifdef CONFIG_EKF2_SIDESLIP
extern "C" void ekf2_get_aid_src_sideslip(const void* self, EkfAidSource1d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_sideslip(), sizeof(*out));
}
#endif

#ifdef CONFIG_EKF2_RANGE_FINDER
extern "C" void ekf2_get_aid_src_rng_hgt(const void* self, EkfAidSource1d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_rng_hgt(), sizeof(*out));
}
#endif

#ifdef CONFIG_EKF2_OPTICAL_FLOW
extern "C" void ekf2_get_aid_src_optical_flow(const void* self, EkfAidSource2d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_optical_flow(), sizeof(*out));
}
#endif

#ifdef CONFIG_EKF2_EXTERNAL_VISION
extern "C" void ekf2_get_aid_src_ev_hgt(const void* self, EkfAidSource1d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_ev_hgt(), sizeof(*out));
}

extern "C" void ekf2_get_aid_src_ev_pos(const void* self, EkfAidSource2d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_ev_pos(), sizeof(*out));
}

extern "C" void ekf2_get_aid_src_ev_vel(const void* self, EkfAidSource3d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_ev_vel(), sizeof(*out));
}

extern "C" void ekf2_get_aid_src_ev_yaw(const void* self, EkfAidSource1d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_ev_yaw(), sizeof(*out));
}
#endif

#ifdef CONFIG_EKF2_DRAG_FUSION
extern "C" void ekf2_get_aid_src_drag(const void* self, EkfAidSource2d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_drag(), sizeof(*out));
}
#endif

#ifdef CONFIG_EKF2_GRAVITY_FUSION
extern "C" void ekf2_get_aid_src_gravity(const void* self, EkfAidSource3d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_gravity(), sizeof(*out));
}
#endif

#ifdef CONFIG_EKF2_AUXVEL
extern "C" void ekf2_get_aid_src_aux_vel(const void* self, EkfAidSource2d* out)
{
    memcpy(out, &as_ekf(self)->aid_src_aux_vel(), sizeof(*out));
}
#endif

#ifdef CONFIG_EKF2_WIND
extern "C" void ekf2_get_wind_velocity(const void* self, float w[2])
{
    copy_vec2_to_array(as_ekf(self)->getWindVelocity(), w);
}

extern "C" void ekf2_get_wind_variance(const void* self, float var[2])
{
    copy_vec2_to_array(as_ekf(self)->getWindVelocityVariance(), var);
}

extern "C" bool ekf2_wind_estimate_active(const void* self)
{
    return as_ekf(self)->get_wind_status();
}
#endif /* CONFIG_EKF2_WIND */

extern "C" bool ekf2_horizontal_aiding_active(const void* self)
{
    return as_ekf(self)->isHorizontalAidingActive();
}

extern "C" bool ekf2_horizontal_position_aiding_active(const void* self)
{
    return as_ekf(self)->isHorizontalPositionAidingActive();
}

extern "C" bool ekf2_vertical_aiding_active(const void* self)
{
    return as_ekf(self)->isVerticalAidingActive();
}

extern "C" bool ekf2_north_east_aiding_active(const void* self)
{
    return as_ekf(self)->isNorthEastAidingActive();
}

extern "C" int32_t ekf2_num_active_horizontal_aiding_sources(const void* self)
{
    return as_ekf(self)->getNumberOfActiveHorizontalAidingSources();
}

extern "C" int32_t ekf2_num_active_horizontal_position_aiding_sources(const void* self)
{
    return as_ekf(self)->getNumberOfActiveHorizontalPositionAidingSources();
}

extern "C" int32_t ekf2_num_active_horizontal_velocity_aiding_sources(const void* self)
{
    return as_ekf(self)->getNumberOfActiveHorizontalVelocityAidingSources();
}

extern "C" bool ekf2_vertical_position_aiding_active(const void* self)
{
    return as_ekf(self)->isVerticalPositionAidingActive();
}

extern "C" int32_t ekf2_num_active_vertical_position_aiding_sources(const void* self)
{
    return as_ekf(self)->getNumberOfActiveVerticalPositionAidingSources();
}

extern "C" bool ekf2_vertical_velocity_aiding_active(const void* self)
{
    return as_ekf(self)->isVerticalVelocityAidingActive();
}

extern "C" int32_t ekf2_num_active_vertical_velocity_aiding_sources(const void* self)
{
    return as_ekf(self)->getNumberOfActiveVerticalVelocityAidingSources();
}

extern "C" float ekf2_get_dt_ekf_avg(const void* self)
{
    return as_ekf(self)->get_dt_ekf_avg();
}

#ifdef CONFIG_EKF2_BAROMETER
extern "C" void ekf2_get_baro_bias_estimator_status(const void* self, EkfBiasEstimatorStatus* out)
{
    copy_bias_status_to_c(as_ekf(self)->getBaroBiasEstimatorStatus(), out);
}
#endif

#ifdef CONFIG_EKF2_EXTERNAL_VISION
extern "C" void ekf2_get_ev_hgt_bias_estimator_status(const void* self, EkfBiasEstimatorStatus* out)
{
    copy_bias_status_to_c(as_ekf(self)->getEvHgtBiasEstimatorStatus(), out);
}

extern "C" void ekf2_get_ev_pos_bias_estimator_status(const void* self, uint8_t axis, EkfBiasEstimatorStatus* out)
{
    if (axis > 1) {
        if (out) {
            out->bias = NAN;
            out->bias_var = NAN;
            out->innov = NAN;
            out->innov_var = NAN;
            out->innov_test_ratio = NAN;
        }
        return;
    }

    copy_bias_status_to_c(as_ekf(self)->getEvPosBiasEstimatorStatus(axis), out);
}
#endif

// ── Parameters ────────────────────────────────────────────────────────────

static inline parameters* params_mut(void* self)
{
    return as_ekf(self)->getParamHandle();
}

// SAFETY: getParamHandle() is a simple pointer accessor with no side effects.
// The const_cast is required because PX4 does not provide a const overload.
// The underlying Ekf object is always mutable (owned by Rust storage), so
// casting away const on the outer pointer does not violate the C++ object model.
static inline const parameters* params_const(const void* self)
{
    return as_ekf(const_cast<void*>(self))->getParamHandle();
}

extern "C" float ekf2_param_get_gyro_noise(const void* self) { return params_const(self)->ekf2_gyr_noise; }
extern "C" void  ekf2_param_set_gyro_noise(void* self, float v) { params_mut(self)->ekf2_gyr_noise = v; }
extern "C" float ekf2_param_get_accel_noise(const void* self) { return params_const(self)->ekf2_acc_noise; }
extern "C" void  ekf2_param_set_accel_noise(void* self, float v) { params_mut(self)->ekf2_acc_noise = v; }
extern "C" float ekf2_param_get_gyro_bias_noise(const void* self) { return params_const(self)->ekf2_gyr_b_noise; }
extern "C" void  ekf2_param_set_gyro_bias_noise(void* self, float v) { params_mut(self)->ekf2_gyr_b_noise = v; }
extern "C" float ekf2_param_get_accel_bias_noise(const void* self) { return params_const(self)->ekf2_acc_b_noise; }
extern "C" void  ekf2_param_set_accel_bias_noise(void* self, float v) { params_mut(self)->ekf2_acc_b_noise = v; }
extern "C" float ekf2_param_get_gyro_bias_init(const void* self) { return params_const(self)->ekf2_gbias_init; }
extern "C" void  ekf2_param_set_gyro_bias_init(void* self, float v) { params_mut(self)->ekf2_gbias_init = v; }
extern "C" float ekf2_param_get_accel_bias_init(const void* self) { return params_const(self)->ekf2_abias_init; }
extern "C" void  ekf2_param_set_accel_bias_init(void* self, float v) { params_mut(self)->ekf2_abias_init = v; }
extern "C" float ekf2_param_get_angerr_init(const void* self) { return params_const(self)->ekf2_angerr_init; }
extern "C" void  ekf2_param_set_angerr_init(void* self, float v) { params_mut(self)->ekf2_angerr_init = v; }
extern "C" float ekf2_param_get_delay_max(const void* self) { return params_const(self)->ekf2_delay_max; }
extern "C" void  ekf2_param_set_delay_max(void* self, float v) { params_mut(self)->ekf2_delay_max = v; }
extern "C" int32_t ekf2_param_get_predict_us(const void* self) { return params_const(self)->ekf2_predict_us; }
extern "C" void    ekf2_param_set_predict_us(void* self, int32_t v) { params_mut(self)->ekf2_predict_us = v; }
extern "C" int32_t ekf2_param_get_imu_ctrl(const void* self) { return params_const(self)->ekf2_imu_ctrl; }
extern "C" void    ekf2_param_set_imu_ctrl(void* self, int32_t v) { params_mut(self)->ekf2_imu_ctrl = v; }
extern "C" float   ekf2_param_get_vel_lim(const void* self) { return params_const(self)->ekf2_vel_lim; }
extern "C" void    ekf2_param_set_vel_lim(void* self, float v) { params_mut(self)->ekf2_vel_lim = v; }
extern "C" int32_t ekf2_param_get_position_sensor_ref(const void* self) { return params_const(self)->position_sensor_ref; }
extern "C" void    ekf2_param_set_position_sensor_ref(void* self, int32_t v) { params_mut(self)->position_sensor_ref = v; }
extern "C" int32_t ekf2_param_get_noaid_tout(const void* self) { return params_const(self)->ekf2_noaid_tout; }
extern "C" void    ekf2_param_set_noaid_tout(void* self, int32_t v) { params_mut(self)->ekf2_noaid_tout = v; }
extern "C" float ekf2_param_get_noaid_noise(const void* self) { return params_const(self)->ekf2_noaid_noise; }
extern "C" void  ekf2_param_set_noaid_noise(void* self, float v) { params_mut(self)->ekf2_noaid_noise = v; }
extern "C" float ekf2_param_get_hdg_gate(const void* self) { return params_const(self)->ekf2_hdg_gate; }
extern "C" void  ekf2_param_set_hdg_gate(void* self, float v) { params_mut(self)->ekf2_hdg_gate = v; }
extern "C" float ekf2_param_get_head_noise(const void* self) { return params_const(self)->ekf2_head_noise; }
extern "C" void  ekf2_param_set_head_noise(void* self, float v) { params_mut(self)->ekf2_head_noise = v; }
extern "C" float ekf2_param_get_abl_lim(const void* self) { return params_const(self)->ekf2_abl_lim; }
extern "C" void  ekf2_param_set_abl_lim(void* self, float v) { params_mut(self)->ekf2_abl_lim = v; }
extern "C" float ekf2_param_get_abl_acclim(const void* self) { return params_const(self)->ekf2_abl_acclim; }
extern "C" void  ekf2_param_set_abl_acclim(void* self, float v) { params_mut(self)->ekf2_abl_acclim = v; }
extern "C" float ekf2_param_get_abl_gyrlim(const void* self) { return params_const(self)->ekf2_abl_gyrlim; }
extern "C" void  ekf2_param_set_abl_gyrlim(void* self, float v) { params_mut(self)->ekf2_abl_gyrlim = v; }
extern "C" float ekf2_param_get_abl_tau(const void* self) { return params_const(self)->ekf2_abl_tau; }
extern "C" void  ekf2_param_set_abl_tau(void* self, float v) { params_mut(self)->ekf2_abl_tau = v; }
extern "C" float ekf2_param_get_gyr_b_lim(const void* self) { return params_const(self)->ekf2_gyr_b_lim; }
extern "C" void  ekf2_param_set_gyr_b_lim(void* self, float v) { params_mut(self)->ekf2_gyr_b_lim = v; }
extern "C" float ekf2_param_get_imu_pos_body_x(const void* self) { return params_const(self)->imu_pos_body(0); }
extern "C" void  ekf2_param_set_imu_pos_body_x(void* self, float v) { params_mut(self)->imu_pos_body(0) = v; }
extern "C" float ekf2_param_get_imu_pos_body_y(const void* self) { return params_const(self)->imu_pos_body(1); }
extern "C" void  ekf2_param_set_imu_pos_body_y(void* self, float v) { params_mut(self)->imu_pos_body(1) = v; }
extern "C" float ekf2_param_get_imu_pos_body_z(const void* self) { return params_const(self)->imu_pos_body(2); }
extern "C" void  ekf2_param_set_imu_pos_body_z(void* self, float v) { params_mut(self)->imu_pos_body(2) = v; }

#ifdef CONFIG_EKF2_BAROMETER
extern "C" int32_t ekf2_param_get_baro_ctrl(const void* self) { return params_const(self)->ekf2_baro_ctrl; }
extern "C" void    ekf2_param_set_baro_ctrl(void* self, int32_t v) { params_mut(self)->ekf2_baro_ctrl = v; }
extern "C" float ekf2_param_get_baro_delay(const void* self) { return params_const(self)->ekf2_baro_delay; }
extern "C" void  ekf2_param_set_baro_delay(void* self, float v) { params_mut(self)->ekf2_baro_delay = v; }
extern "C" float ekf2_param_get_baro_noise(const void* self) { return params_const(self)->ekf2_baro_noise; }
extern "C" void  ekf2_param_set_baro_noise(void* self, float v) { params_mut(self)->ekf2_baro_noise = v; }
extern "C" float ekf2_param_get_baro_gate(const void* self) { return params_const(self)->ekf2_baro_gate; }
extern "C" void  ekf2_param_set_baro_gate(void* self, float v) { params_mut(self)->ekf2_baro_gate = v; }
extern "C" float ekf2_param_get_baro_bias_nsd(const void* self) { return params_const(self)->baro_bias_nsd; }
extern "C" void  ekf2_param_set_baro_bias_nsd(void* self, float v) { params_mut(self)->baro_bias_nsd = v; }
extern "C" float ekf2_param_get_gnd_eff_dz(const void* self) { return params_const(self)->ekf2_gnd_eff_dz; }
extern "C" void  ekf2_param_set_gnd_eff_dz(void* self, float v) { params_mut(self)->ekf2_gnd_eff_dz = v; }
extern "C" float ekf2_param_get_gnd_max_hgt(const void* self) { return params_const(self)->ekf2_gnd_max_hgt; }
extern "C" void  ekf2_param_set_gnd_max_hgt(void* self, float v) { params_mut(self)->ekf2_gnd_max_hgt = v; }
#ifdef CONFIG_EKF2_BARO_COMPENSATION
extern "C" float ekf2_param_get_pcoef_xp(const void* self) { return params_const(self)->ekf2_pcoef_xp; }
extern "C" void  ekf2_param_set_pcoef_xp(void* self, float v) { params_mut(self)->ekf2_pcoef_xp = v; }
extern "C" float ekf2_param_get_pcoef_xn(const void* self) { return params_const(self)->ekf2_pcoef_xn; }
extern "C" void  ekf2_param_set_pcoef_xn(void* self, float v) { params_mut(self)->ekf2_pcoef_xn = v; }
extern "C" float ekf2_param_get_pcoef_yp(const void* self) { return params_const(self)->ekf2_pcoef_yp; }
extern "C" void  ekf2_param_set_pcoef_yp(void* self, float v) { params_mut(self)->ekf2_pcoef_yp = v; }
extern "C" float ekf2_param_get_pcoef_yn(const void* self) { return params_const(self)->ekf2_pcoef_yn; }
extern "C" void  ekf2_param_set_pcoef_yn(void* self, float v) { params_mut(self)->ekf2_pcoef_yn = v; }
extern "C" float ekf2_param_get_pcoef_z(const void* self) { return params_const(self)->ekf2_pcoef_z; }
extern "C" void  ekf2_param_set_pcoef_z(void* self, float v) { params_mut(self)->ekf2_pcoef_z = v; }
extern "C" float ekf2_param_get_aspd_max(const void* self) { return params_const(self)->ekf2_aspd_max; }
extern "C" void  ekf2_param_set_aspd_max(void* self, float v) { params_mut(self)->ekf2_aspd_max = v; }
#endif
#endif

#ifdef CONFIG_EKF2_GNSS
extern "C" int32_t ekf2_param_get_gps_mode(const void* self) { return params_const(self)->ekf2_gps_mode; }
extern "C" void    ekf2_param_set_gps_mode(void* self, int32_t v) { params_mut(self)->ekf2_gps_mode = v; }
extern "C" float ekf2_param_get_gps_delay(const void* self) { return params_const(self)->ekf2_gps_delay; }
extern "C" void  ekf2_param_set_gps_delay(void* self, float v) { params_mut(self)->ekf2_gps_delay = v; }
extern "C" float ekf2_param_get_gps_pos_body_x(const void* self) { return params_const(self)->gps_pos_body(0); }
extern "C" void  ekf2_param_set_gps_pos_body_x(void* self, float v) { params_mut(self)->gps_pos_body(0) = v; }
extern "C" float ekf2_param_get_gps_pos_body_y(const void* self) { return params_const(self)->gps_pos_body(1); }
extern "C" void  ekf2_param_set_gps_pos_body_y(void* self, float v) { params_mut(self)->gps_pos_body(1) = v; }
extern "C" float ekf2_param_get_gps_pos_body_z(const void* self) { return params_const(self)->gps_pos_body(2); }
extern "C" void  ekf2_param_set_gps_pos_body_z(void* self, float v) { params_mut(self)->gps_pos_body(2) = v; }
extern "C" float ekf2_param_get_gps_pos_noise(const void* self) { return params_const(self)->ekf2_gps_p_noise; }
extern "C" void  ekf2_param_set_gps_pos_noise(void* self, float v) { params_mut(self)->ekf2_gps_p_noise = v; }
extern "C" float ekf2_param_get_gps_vel_noise(const void* self) { return params_const(self)->ekf2_gps_v_noise; }
extern "C" void  ekf2_param_set_gps_vel_noise(void* self, float v) { params_mut(self)->ekf2_gps_v_noise = v; }
extern "C" float ekf2_param_get_gps_hgt_bias_nsd(const void* self) { return params_const(self)->gps_hgt_bias_nsd; }
extern "C" void  ekf2_param_set_gps_hgt_bias_nsd(void* self, float v) { params_mut(self)->gps_hgt_bias_nsd = v; }
extern "C" float ekf2_param_get_gps_pos_gate(const void* self) { return params_const(self)->ekf2_gps_p_gate; }
extern "C" void  ekf2_param_set_gps_pos_gate(void* self, float v) { params_mut(self)->ekf2_gps_p_gate = v; }
extern "C" float ekf2_param_get_gps_vel_gate(const void* self) { return params_const(self)->ekf2_gps_v_gate; }
extern "C" void  ekf2_param_set_gps_vel_gate(void* self, float v) { params_mut(self)->ekf2_gps_v_gate = v; }
extern "C" int32_t ekf2_param_get_gps_ctrl(const void* self) { return params_const(self)->ekf2_gps_ctrl; }
extern "C" void    ekf2_param_set_gps_ctrl(void* self, int32_t v) { params_mut(self)->ekf2_gps_ctrl = v; }
extern "C" int32_t ekf2_param_get_gps_check(const void* self) { return params_const(self)->ekf2_gps_check; }
extern "C" void    ekf2_param_set_gps_check(void* self, int32_t v) { params_mut(self)->ekf2_gps_check = v; }
extern "C" float ekf2_param_get_req_eph(const void* self) { return params_const(self)->ekf2_req_eph; }
extern "C" void  ekf2_param_set_req_eph(void* self, float v) { params_mut(self)->ekf2_req_eph = v; }
extern "C" float ekf2_param_get_req_epv(const void* self) { return params_const(self)->ekf2_req_epv; }
extern "C" void  ekf2_param_set_req_epv(void* self, float v) { params_mut(self)->ekf2_req_epv = v; }
extern "C" float ekf2_param_get_req_sacc(const void* self) { return params_const(self)->ekf2_req_sacc; }
extern "C" void  ekf2_param_set_req_sacc(void* self, float v) { params_mut(self)->ekf2_req_sacc = v; }
extern "C" int32_t ekf2_param_get_req_nsats(const void* self) { return params_const(self)->ekf2_req_nsats; }
extern "C" void    ekf2_param_set_req_nsats(void* self, int32_t v) { params_mut(self)->ekf2_req_nsats = v; }
extern "C" float ekf2_param_get_req_pdop(const void* self) { return params_const(self)->ekf2_req_pdop; }
extern "C" void  ekf2_param_set_req_pdop(void* self, float v) { params_mut(self)->ekf2_req_pdop = v; }
extern "C" float ekf2_param_get_req_hdrift(const void* self) { return params_const(self)->ekf2_req_hdrift; }
extern "C" void  ekf2_param_set_req_hdrift(void* self, float v) { params_mut(self)->ekf2_req_hdrift = v; }
extern "C" float ekf2_param_get_req_vdrift(const void* self) { return params_const(self)->ekf2_req_vdrift; }
extern "C" void  ekf2_param_set_req_vdrift(void* self, float v) { params_mut(self)->ekf2_req_vdrift = v; }
extern "C" int32_t ekf2_param_get_req_fix(const void* self) { return params_const(self)->ekf2_req_fix; }
extern "C" void    ekf2_param_set_req_fix(void* self, int32_t v) { params_mut(self)->ekf2_req_fix = v; }
extern "C" float ekf2_param_get_gsf_tas(const void* self) { return params_const(self)->ekf2_gsf_tas; }
extern "C" void  ekf2_param_set_gsf_tas(void* self, float v) { params_mut(self)->ekf2_gsf_tas = v; }
#ifdef CONFIG_EKF2_GNSS_YAW
extern "C" float ekf2_param_get_gnss_heading_noise(const void* self) { return params_const(self)->gnss_heading_noise; }
extern "C" void  ekf2_param_set_gnss_heading_noise(void* self, float v) { params_mut(self)->gnss_heading_noise = v; }
#endif
#endif

#ifdef CONFIG_EKF2_MAGNETOMETER
extern "C" float ekf2_param_get_mag_delay(const void* self) { return params_const(self)->ekf2_mag_delay; }
extern "C" void  ekf2_param_set_mag_delay(void* self, float v) { params_mut(self)->ekf2_mag_delay = v; }
extern "C" float ekf2_param_get_mag_noise(const void* self) { return params_const(self)->ekf2_mag_noise; }
extern "C" void  ekf2_param_set_mag_noise(void* self, float v) { params_mut(self)->ekf2_mag_noise = v; }
extern "C" float ekf2_param_get_mag_decl(const void* self) { return params_const(self)->ekf2_mag_decl; }
extern "C" void  ekf2_param_set_mag_decl(void* self, float v) { params_mut(self)->ekf2_mag_decl = v; }
extern "C" float ekf2_param_get_mag_gate(const void* self) { return params_const(self)->ekf2_mag_gate; }
extern "C" void  ekf2_param_set_mag_gate(void* self, float v) { params_mut(self)->ekf2_mag_gate = v; }
extern "C" int32_t ekf2_param_get_decl_type(const void* self) { return params_const(self)->ekf2_decl_type; }
extern "C" void    ekf2_param_set_decl_type(void* self, int32_t v) { params_mut(self)->ekf2_decl_type = v; }
extern "C" int32_t ekf2_param_get_mag_type(const void* self) { return params_const(self)->ekf2_mag_type; }
extern "C" void    ekf2_param_set_mag_type(void* self, int32_t v) { params_mut(self)->ekf2_mag_type = v; }
extern "C" int32_t ekf2_param_get_mag_check(const void* self) { return params_const(self)->ekf2_mag_check; }
extern "C" void    ekf2_param_set_mag_check(void* self, int32_t v) { params_mut(self)->ekf2_mag_check = v; }
extern "C" float ekf2_param_get_mag_chk_str(const void* self) { return params_const(self)->ekf2_mag_chk_str; }
extern "C" void  ekf2_param_set_mag_chk_str(void* self, float v) { params_mut(self)->ekf2_mag_chk_str = v; }
extern "C" float ekf2_param_get_mag_chk_inc(const void* self) { return params_const(self)->ekf2_mag_chk_inc; }
extern "C" void  ekf2_param_set_mag_chk_inc(void* self, float v) { params_mut(self)->ekf2_mag_chk_inc = v; }
extern "C" float ekf2_param_get_mag_e_noise(const void* self) { return params_const(self)->ekf2_mag_e_noise; }
extern "C" void  ekf2_param_set_mag_e_noise(void* self, float v) { params_mut(self)->ekf2_mag_e_noise = v; }
extern "C" float ekf2_param_get_mag_b_noise(const void* self) { return params_const(self)->ekf2_mag_b_noise; }
extern "C" void  ekf2_param_set_mag_b_noise(void* self, float v) { params_mut(self)->ekf2_mag_b_noise = v; }
extern "C" float ekf2_param_get_mag_acclim(const void* self) { return params_const(self)->ekf2_mag_acclim; }
extern "C" void  ekf2_param_set_mag_acclim(void* self, float v) { params_mut(self)->ekf2_mag_acclim = v; }
extern "C" int32_t ekf2_param_get_synt_mag_z(const void* self) { return params_const(self)->ekf2_synt_mag_z; }
extern "C" void    ekf2_param_set_synt_mag_z(void* self, int32_t v) { params_mut(self)->ekf2_synt_mag_z = v; }
#endif

extern "C" int32_t ekf2_param_get_hgt_ref(const void* self) { return params_const(self)->ekf2_hgt_ref; }
extern "C" void    ekf2_param_set_hgt_ref(void* self, int32_t v) { params_mut(self)->ekf2_hgt_ref = v; }

#ifdef CONFIG_EKF2_AIRSPEED
extern "C" float ekf2_param_get_asp_delay(const void* self) { return params_const(self)->ekf2_asp_delay; }
extern "C" void  ekf2_param_set_asp_delay(void* self, float v) { params_mut(self)->ekf2_asp_delay = v; }
extern "C" float ekf2_param_get_eas_noise(const void* self) { return params_const(self)->ekf2_eas_noise; }
extern "C" void  ekf2_param_set_eas_noise(void* self, float v) { params_mut(self)->ekf2_eas_noise = v; }
extern "C" float ekf2_param_get_tas_gate(const void* self) { return params_const(self)->ekf2_tas_gate; }
extern "C" void  ekf2_param_set_tas_gate(void* self, float v) { params_mut(self)->ekf2_tas_gate = v; }
extern "C" float ekf2_param_get_arsp_thr(const void* self) { return params_const(self)->ekf2_arsp_thr; }
extern "C" void  ekf2_param_set_arsp_thr(void* self, float v) { params_mut(self)->ekf2_arsp_thr = v; }
#endif

#ifdef CONFIG_EKF2_RANGE_FINDER
extern "C" int32_t ekf2_param_get_rng_ctrl(const void* self) { return params_const(self)->ekf2_rng_ctrl; }
extern "C" void    ekf2_param_set_rng_ctrl(void* self, int32_t v) { params_mut(self)->ekf2_rng_ctrl = v; }
extern "C" float ekf2_param_get_rng_delay(const void* self) { return params_const(self)->ekf2_rng_delay; }
extern "C" void  ekf2_param_set_rng_delay(void* self, float v) { params_mut(self)->ekf2_rng_delay = v; }
extern "C" float ekf2_param_get_rng_noise(const void* self) { return params_const(self)->ekf2_rng_noise; }
extern "C" void  ekf2_param_set_rng_noise(void* self, float v) { params_mut(self)->ekf2_rng_noise = v; }
extern "C" float ekf2_param_get_rng_gate(const void* self) { return params_const(self)->ekf2_rng_gate; }
extern "C" void  ekf2_param_set_rng_gate(void* self, float v) { params_mut(self)->ekf2_rng_gate = v; }
extern "C" float ekf2_param_get_rng_pitch(const void* self) { return params_const(self)->ekf2_rng_pitch; }
extern "C" void  ekf2_param_set_rng_pitch(void* self, float v) { params_mut(self)->ekf2_rng_pitch = v; }
extern "C" float ekf2_param_get_rng_sfe(const void* self) { return params_const(self)->ekf2_rng_sfe; }
extern "C" void  ekf2_param_set_rng_sfe(void* self, float v) { params_mut(self)->ekf2_rng_sfe = v; }
extern "C" float ekf2_param_get_rng_a_hmax(const void* self) { return params_const(self)->ekf2_rng_a_hmax; }
extern "C" void  ekf2_param_set_rng_a_hmax(void* self, float v) { params_mut(self)->ekf2_rng_a_hmax = v; }
extern "C" float ekf2_param_get_rng_a_vmax(const void* self) { return params_const(self)->ekf2_rng_a_vmax; }
extern "C" void  ekf2_param_set_rng_a_vmax(void* self, float v) { params_mut(self)->ekf2_rng_a_vmax = v; }
extern "C" float ekf2_param_get_rng_qlty_t(const void* self) { return params_const(self)->ekf2_rng_qlty_t; }
extern "C" void  ekf2_param_set_rng_qlty_t(void* self, float v) { params_mut(self)->ekf2_rng_qlty_t = v; }
extern "C" float ekf2_param_get_rng_k_gate(const void* self) { return params_const(self)->ekf2_rng_k_gate; }
extern "C" void  ekf2_param_set_rng_k_gate(void* self, float v) { params_mut(self)->ekf2_rng_k_gate = v; }
extern "C" float ekf2_param_get_rng_fog(const void* self) { return params_const(self)->ekf2_rng_fog; }
extern "C" void  ekf2_param_set_rng_fog(void* self, float v) { params_mut(self)->ekf2_rng_fog = v; }
extern "C" float ekf2_param_get_rng_pos_body_x(const void* self) { return params_const(self)->rng_pos_body(0); }
extern "C" void  ekf2_param_set_rng_pos_body_x(void* self, float v) { params_mut(self)->rng_pos_body(0) = v; }
extern "C" float ekf2_param_get_rng_pos_body_y(const void* self) { return params_const(self)->rng_pos_body(1); }
extern "C" void  ekf2_param_set_rng_pos_body_y(void* self, float v) { params_mut(self)->rng_pos_body(1) = v; }
extern "C" float ekf2_param_get_rng_pos_body_z(const void* self) { return params_const(self)->rng_pos_body(2); }
extern "C" void  ekf2_param_set_rng_pos_body_z(void* self, float v) { params_mut(self)->rng_pos_body(2) = v; }
#endif

#ifdef CONFIG_EKF2_EXTERNAL_VISION
extern "C" int32_t ekf2_param_get_ev_ctrl(const void* self) { return params_const(self)->ekf2_ev_ctrl; }
extern "C" void    ekf2_param_set_ev_ctrl(void* self, int32_t v) { params_mut(self)->ekf2_ev_ctrl = v; }
extern "C" float ekf2_param_get_ev_delay(const void* self) { return params_const(self)->ekf2_ev_delay; }
extern "C" void  ekf2_param_set_ev_delay(void* self, float v) { params_mut(self)->ekf2_ev_delay = v; }
extern "C" float ekf2_param_get_evv_noise(const void* self) { return params_const(self)->ekf2_evv_noise; }
extern "C" void  ekf2_param_set_evv_noise(void* self, float v) { params_mut(self)->ekf2_evv_noise = v; }
extern "C" float ekf2_param_get_evp_noise(const void* self) { return params_const(self)->ekf2_evp_noise; }
extern "C" void  ekf2_param_set_evp_noise(void* self, float v) { params_mut(self)->ekf2_evp_noise = v; }
extern "C" float ekf2_param_get_eva_noise(const void* self) { return params_const(self)->ekf2_eva_noise; }
extern "C" void  ekf2_param_set_eva_noise(void* self, float v) { params_mut(self)->ekf2_eva_noise = v; }
extern "C" int32_t ekf2_param_get_ev_qmin(const void* self) { return params_const(self)->ekf2_ev_qmin; }
extern "C" void    ekf2_param_set_ev_qmin(void* self, int32_t v) { params_mut(self)->ekf2_ev_qmin = v; }
extern "C" float ekf2_param_get_evv_gate(const void* self) { return params_const(self)->ekf2_evv_gate; }
extern "C" void  ekf2_param_set_evv_gate(void* self, float v) { params_mut(self)->ekf2_evv_gate = v; }
extern "C" float ekf2_param_get_evp_gate(const void* self) { return params_const(self)->ekf2_evp_gate; }
extern "C" void  ekf2_param_set_evp_gate(void* self, float v) { params_mut(self)->ekf2_evp_gate = v; }
extern "C" float ekf2_param_get_ev_hgt_bias_nsd(const void* self) { return params_const(self)->ev_hgt_bias_nsd; }
extern "C" void  ekf2_param_set_ev_hgt_bias_nsd(void* self, float v) { params_mut(self)->ev_hgt_bias_nsd = v; }
extern "C" float ekf2_param_get_ev_pos_body_x(const void* self) { return params_const(self)->ev_pos_body(0); }
extern "C" void  ekf2_param_set_ev_pos_body_x(void* self, float v) { params_mut(self)->ev_pos_body(0) = v; }
extern "C" float ekf2_param_get_ev_pos_body_y(const void* self) { return params_const(self)->ev_pos_body(1); }
extern "C" void  ekf2_param_set_ev_pos_body_y(void* self, float v) { params_mut(self)->ev_pos_body(1) = v; }
extern "C" float ekf2_param_get_ev_pos_body_z(const void* self) { return params_const(self)->ev_pos_body(2); }
extern "C" void  ekf2_param_set_ev_pos_body_z(void* self, float v) { params_mut(self)->ev_pos_body(2) = v; }
#endif

#ifdef CONFIG_EKF2_OPTICAL_FLOW
extern "C" int32_t ekf2_param_get_of_ctrl(const void* self) { return params_const(self)->ekf2_of_ctrl; }
extern "C" void    ekf2_param_set_of_ctrl(void* self, int32_t v) { params_mut(self)->ekf2_of_ctrl = v; }
extern "C" int32_t ekf2_param_get_of_gyr_src(const void* self) { return params_const(self)->ekf2_of_gyr_src; }
extern "C" void    ekf2_param_set_of_gyr_src(void* self, int32_t v) { params_mut(self)->ekf2_of_gyr_src = v; }
extern "C" float ekf2_param_get_of_delay(const void* self) { return params_const(self)->ekf2_of_delay; }
extern "C" void  ekf2_param_set_of_delay(void* self, float v) { params_mut(self)->ekf2_of_delay = v; }
extern "C" float ekf2_param_get_of_n_min(const void* self) { return params_const(self)->ekf2_of_n_min; }
extern "C" void  ekf2_param_set_of_n_min(void* self, float v) { params_mut(self)->ekf2_of_n_min = v; }
extern "C" float ekf2_param_get_of_n_max(const void* self) { return params_const(self)->ekf2_of_n_max; }
extern "C" void  ekf2_param_set_of_n_max(void* self, float v) { params_mut(self)->ekf2_of_n_max = v; }
extern "C" int32_t ekf2_param_get_of_qmin(const void* self) { return params_const(self)->ekf2_of_qmin; }
extern "C" void    ekf2_param_set_of_qmin(void* self, int32_t v) { params_mut(self)->ekf2_of_qmin = v; }
extern "C" int32_t ekf2_param_get_of_qmin_gnd(const void* self) { return params_const(self)->ekf2_of_qmin_gnd; }
extern "C" void    ekf2_param_set_of_qmin_gnd(void* self, int32_t v) { params_mut(self)->ekf2_of_qmin_gnd = v; }
extern "C" float ekf2_param_get_of_gate(const void* self) { return params_const(self)->ekf2_of_gate; }
extern "C" void  ekf2_param_set_of_gate(void* self, float v) { params_mut(self)->ekf2_of_gate = v; }
extern "C" float ekf2_param_get_flow_pos_body_x(const void* self) { return params_const(self)->flow_pos_body(0); }
extern "C" void  ekf2_param_set_flow_pos_body_x(void* self, float v) { params_mut(self)->flow_pos_body(0) = v; }
extern "C" float ekf2_param_get_flow_pos_body_y(const void* self) { return params_const(self)->flow_pos_body(1); }
extern "C" void  ekf2_param_set_flow_pos_body_y(void* self, float v) { params_mut(self)->flow_pos_body(1) = v; }
extern "C" float ekf2_param_get_flow_pos_body_z(const void* self) { return params_const(self)->flow_pos_body(2); }
extern "C" void  ekf2_param_set_flow_pos_body_z(void* self, float v) { params_mut(self)->flow_pos_body(2) = v; }
#endif

#ifdef CONFIG_EKF2_WIND
extern "C" float ekf2_param_get_wind_nsd(const void* self) { return params_const(self)->ekf2_wind_nsd; }
extern "C" void  ekf2_param_set_wind_nsd(void* self, float v) { params_mut(self)->ekf2_wind_nsd = v; }
#endif

#ifdef CONFIG_EKF2_SIDESLIP
extern "C" int32_t ekf2_param_get_fuse_beta(const void* self) { return params_const(self)->ekf2_fuse_beta; }
extern "C" void    ekf2_param_set_fuse_beta(void* self, int32_t v) { params_mut(self)->ekf2_fuse_beta = v; }
extern "C" float ekf2_param_get_beta_gate(const void* self) { return params_const(self)->ekf2_beta_gate; }
extern "C" void  ekf2_param_set_beta_gate(void* self, float v) { params_mut(self)->ekf2_beta_gate = v; }
extern "C" float ekf2_param_get_beta_noise(const void* self) { return params_const(self)->ekf2_beta_noise; }
extern "C" void  ekf2_param_set_beta_noise(void* self, float v) { params_mut(self)->ekf2_beta_noise = v; }
#endif

#ifdef CONFIG_EKF2_TERRAIN
extern "C" float ekf2_param_get_terr_noise(const void* self) { return params_const(self)->ekf2_terr_noise; }
extern "C" void  ekf2_param_set_terr_noise(void* self, float v) { params_mut(self)->ekf2_terr_noise = v; }
extern "C" float ekf2_param_get_terr_grad(const void* self) { return params_const(self)->ekf2_terr_grad; }
extern "C" void  ekf2_param_set_terr_grad(void* self, float v) { params_mut(self)->ekf2_terr_grad = v; }
#endif

#if defined(CONFIG_EKF2_TERRAIN) || defined(CONFIG_EKF2_OPTICAL_FLOW) || defined(CONFIG_EKF2_RANGE_FINDER)
extern "C" float ekf2_param_get_min_rng(const void* self) { return params_const(self)->ekf2_min_rng; }
extern "C" void  ekf2_param_set_min_rng(void* self, float v) { params_mut(self)->ekf2_min_rng = v; }
#endif

#ifdef CONFIG_EKF2_GRAVITY_FUSION
extern "C" float ekf2_param_get_grav_noise(const void* self) { return params_const(self)->ekf2_grav_noise; }
extern "C" void  ekf2_param_set_grav_noise(void* self, float v) { params_mut(self)->ekf2_grav_noise = v; }
#endif

#ifdef CONFIG_EKF2_DRAG_FUSION
extern "C" int32_t ekf2_param_get_drag_ctrl(const void* self) { return params_const(self)->ekf2_drag_ctrl; }
extern "C" void    ekf2_param_set_drag_ctrl(void* self, int32_t v) { params_mut(self)->ekf2_drag_ctrl = v; }
extern "C" float ekf2_param_get_drag_noise(const void* self) { return params_const(self)->ekf2_drag_noise; }
extern "C" void  ekf2_param_set_drag_noise(void* self, float v) { params_mut(self)->ekf2_drag_noise = v; }
extern "C" float ekf2_param_get_bcoef_x(const void* self) { return params_const(self)->ekf2_bcoef_x; }
extern "C" void  ekf2_param_set_bcoef_x(void* self, float v) { params_mut(self)->ekf2_bcoef_x = v; }
extern "C" float ekf2_param_get_bcoef_y(const void* self) { return params_const(self)->ekf2_bcoef_y; }
extern "C" void  ekf2_param_set_bcoef_y(void* self, float v) { params_mut(self)->ekf2_bcoef_y = v; }
extern "C" float ekf2_param_get_mcoef(const void* self) { return params_const(self)->ekf2_mcoef; }
extern "C" void  ekf2_param_set_mcoef(void* self, float v) { params_mut(self)->ekf2_mcoef = v; }
#endif

#ifdef CONFIG_EKF2_AUXVEL
extern "C" float ekf2_param_get_avel_delay(const void* self) { return params_const(self)->ekf2_avel_delay; }
extern "C" void  ekf2_param_set_avel_delay(void* self, float v) { params_mut(self)->ekf2_avel_delay = v; }
#endif
