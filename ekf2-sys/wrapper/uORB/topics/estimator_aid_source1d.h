/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Derived from PX4-Autopilot message definitions.
 * Copyright (c) 2012 - 2025 PX4 Development Team.
 */

/**
 * estimator_aid_source1d.h — plain C struct stub required for standalone EKF2 build.
 *
 * PX4's ekf.h hardcodes `#include <uORB/topics/estimator_aid_source1d.h>`, so this
 * file must live at that path under the include root.  It is NOT part of the uORB
 * pub/sub system — it is only the struct definition that PX4's EKF internals use.
 *
 * Generated from msg/EstimatorAidSource1d.msg (PX4-Autopilot main).
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

struct estimator_aid_source1d_s {
    uint64_t timestamp;               /**< [µs] system time */
    uint64_t timestamp_sample;        /**< [µs] raw sensor timestamp */
    uint8_t  estimator_instance;
    uint8_t  _padding0[3];
    uint32_t device_id;
    uint64_t time_last_fuse;          /**< [µs] time of last successful fusion */
    float    observation;
    float    observation_variance;
    float    innovation;
    float    innovation_filtered;
    float    innovation_variance;
    float    test_ratio;
    float    test_ratio_filtered;
    bool     innovation_rejected;
    bool     fused;
    uint8_t  _padding1[2];
};
