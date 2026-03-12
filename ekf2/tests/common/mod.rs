#![allow(dead_code)]

use ekf2::{types::ImuSample, Ekf, EkfError};
#[cfg(unix)]
use std::process::Command;

#[derive(Debug, Clone, Copy)]
pub struct Snapshot {
    pub q: [f32; 4],
    pub v: [f32; 3],
    pub p: [f32; 3],
    pub pos_var: [f32; 3],
    pub vel_var: [f32; 3],
    pub gyro_bias_var: [f32; 3],
    pub accel_bias_var: [f32; 3],
    pub soln_status_bits: u16,
    pub quat_reset_count: u8,
    pub pos_ne_reset_count: u8,
    pub vel_ne_reset_count: u8,
    pub pos_d_reset_count: u8,
    pub vel_d_reset_count: u8,
}

pub fn feed_imu(ekf: &mut Ekf, start_ts: u64, steps: u64, yaw_rate_z: f32) -> u32 {
    let dt_s = 0.01_f32;
    let mut update_failed = 0_u32;

    for i in 0..steps {
        let ts = start_ts + i * 10_000;
        let imu = ImuSample::new(
            ts,
            [0.0, 0.0, yaw_rate_z],
            [0.0, 0.0, -9.81 * dt_s],
            dt_s,
            dt_s,
        );
        ekf.set_imu_data(&imu);
        if let Err(EkfError::UpdateFailed) = ekf.update() {
            update_failed += 1;
        }
    }

    update_failed
}

pub fn yaw_from_quaternion(q: [f32; 4]) -> f32 {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let siny = 2.0 * (w * z + x * y);
    let cosy = 1.0 - 2.0 * (y * y + z * z);
    siny.atan2(cosy)
}

#[cfg(unix)]
pub fn current_rss_kib() -> Option<usize> {
    let pid = std::process::id().to_string();
    let output = Command::new("ps")
        .args(["-o", "rss=", "-p", &pid])
        .output()
        .ok()?;

    if !output.status.success() {
        return None;
    }

    let rss_text = String::from_utf8(output.stdout).ok()?;
    rss_text.trim().parse::<usize>().ok()
}

pub fn abs_max_diff<const N: usize>(a: [f32; N], b: [f32; N]) -> f32 {
    let mut m = 0.0_f32;
    for i in 0..N {
        m = m.max((a[i] - b[i]).abs());
    }
    m
}

pub fn quat_norm(q: [f32; 4]) -> f32 {
    (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt()
}

pub fn assert_non_negative_finite<const N: usize>(name: &str, v: [f32; N]) {
    for (idx, value) in v.iter().enumerate() {
        assert!(
            value.is_finite(),
            "{name}[{idx}] must be finite, got {value}"
        );
        assert!(
            *value >= -1e-6,
            "{name}[{idx}] must be non-negative, got {value}"
        );
    }
}

pub fn snapshot(ekf: &Ekf) -> Snapshot {
    Snapshot {
        q: ekf.quaternion(),
        v: ekf.velocity_ned(),
        p: ekf.position_ned(),
        pos_var: ekf.pos_variance(),
        vel_var: ekf.vel_variance(),
        gyro_bias_var: ekf.gyro_bias_variance(),
        accel_bias_var: ekf.accel_bias_variance(),
        soln_status_bits: ekf.soln_status().bits(),
        quat_reset_count: ekf.quat_reset_count(),
        pos_ne_reset_count: ekf.pos_ne_reset_count(),
        vel_ne_reset_count: ekf.vel_ne_reset_count(),
        pos_d_reset_count: ekf.pos_d_reset_count(),
        vel_d_reset_count: ekf.vel_d_reset_count(),
    }
}

pub fn assert_snapshot_sane(tag: &str, s: Snapshot) {
    assert!(
        s.q.iter().all(|v| v.is_finite()),
        "{tag}: quaternion must be finite"
    );
    assert!(
        s.v.iter().all(|v| v.is_finite()),
        "{tag}: velocity must be finite"
    );
    assert!(
        s.p.iter().all(|v| v.is_finite()),
        "{tag}: position must be finite"
    );
    let qn = quat_norm(s.q);
    assert!(
        (qn - 1.0).abs() < 0.2,
        "{tag}: quaternion norm out of bounds, got {qn}"
    );
    assert_non_negative_finite("pos_var", s.pos_var);
    assert_non_negative_finite("vel_var", s.vel_var);
    assert_non_negative_finite("gyro_bias_var", s.gyro_bias_var);
    assert_non_negative_finite("accel_bias_var", s.accel_bias_var);
}
