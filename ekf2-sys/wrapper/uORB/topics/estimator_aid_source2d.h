/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Derived from PX4-Autopilot message definitions.
 * Copyright (c) 2012 - 2025 PX4 Development Team.
 */

/**
 * estimator_aid_source2d.h — plain C struct stub required for standalone EKF2 build.
 *
 * PX4's ekf.h hardcodes `#include <uORB/topics/estimator_aid_source2d.h>`, so this
 * file must live at that path under the include root.  It is NOT part of the uORB
 * pub/sub system — it is only the struct definition that PX4's EKF internals use.
 *
 * Generated from msg/EstimatorAidSource2d.msg (PX4-Autopilot main).
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

struct estimator_aid_source2d_s {
    uint64_t timestamp;
    uint64_t timestamp_sample;
    uint8_t  estimator_instance;
    uint8_t  _padding0[7];           /**< pad to align double[2] */
    uint32_t device_id;
    uint8_t  _padding1[4];
    uint64_t time_last_fuse;
    double   observation[2];         /**< float64[2] as per .msg */
    float    observation_variance[2];
    float    innovation[2];
    float    innovation_filtered[2];
    float    innovation_variance[2];
    float    test_ratio[2];
    float    test_ratio_filtered[2];
    bool     innovation_rejected;
    bool     fused;
    uint8_t  _padding2[6];
};
