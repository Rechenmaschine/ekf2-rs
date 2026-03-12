#![cfg(all(feature = "gnss", feature = "magnetometer", feature = "barometer"))]

use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;

use ekf2_sys::{
    ekf2_create, ekf2_destroy, ekf2_init, ekf2_set_imu_data, ekf2_update, EkfBaroSample,
    EkfGnssSample, EkfImuSample, EkfMagSample,
};

fn fixture_path(file: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("ekf2")
        .join("tests")
        .join("data")
        .join(file)
}

fn parse_u64(parts: &[&str], idx: usize) -> u64 {
    parts[idx].parse::<u64>().expect("invalid u64 in fixture")
}

fn parse_f32(parts: &[&str], idx: usize) -> f32 {
    parts[idx].parse::<f32>().expect("invalid f32 in fixture")
}

fn parse_f64(parts: &[&str], idx: usize) -> f64 {
    parts[idx].parse::<f64>().expect("invalid f64 in fixture")
}

struct ReplayProbeExpectations {
    name: &'static str,
    replay_fixture: &'static str,
    min_imu_samples: usize,
    min_mag_samples: usize,
    min_baro_samples: usize,
    min_gps_samples: usize,
    min_successful_updates: usize,
}

fn run_raw_replay_probe(expectations: &ReplayProbeExpectations) {
    let replay_file =
        File::open(fixture_path(expectations.replay_fixture)).expect("missing replay fixture");

    let mut obj = [0u8; 65_536];

    let ptr = unsafe {
        let ptr = ekf2_create(obj.as_mut_ptr() as *mut _, obj.len());
        assert!(!ptr.is_null(), "create failed");
        let ok = ekf2_init(ptr, 0);
        assert!(ok, "init failed");
        ptr
    };

    let mut prev_imu_ts = None::<u64>;
    let mut last_mag_ts = 0_u64;
    let mut last_baro_ts = 0_u64;
    let mut imu_samples = 0_usize;
    let mut update_failures = 0_usize;
    let mut update_success = 0_usize;
    let mut gps_samples = 0_usize;
    let mut mag_samples = 0_usize;
    let mut baro_samples = 0_usize;

    for line in BufReader::new(replay_file).lines() {
        let line = line.expect("failed to read replay line");
        if line.trim().is_empty() {
            continue;
        }

        let parts: Vec<&str> = line.split(',').collect();
        let ts = parse_u64(&parts, 0);

        match parts[1] {
            "imu" => {
                imu_samples += 1;
                let ax = parse_f32(&parts, 2);
                let ay = parse_f32(&parts, 3);
                let az = parse_f32(&parts, 4);
                let gx = parse_f32(&parts, 5);
                let gy = parse_f32(&parts, 6);
                let gz = parse_f32(&parts, 7);

                let dt_s = prev_imu_ts
                    .map(|p| ((ts - p) as f32) * 1e-6)
                    .unwrap_or(0.004)
                    .clamp(0.001, 0.02);
                prev_imu_ts = Some(ts);

                let imu = EkfImuSample::new(
                    ts,
                    [gx * dt_s, gy * dt_s, gz * dt_s],
                    [ax * dt_s, ay * dt_s, az * dt_s],
                    dt_s,
                    dt_s,
                );

                unsafe {
                    ekf2_set_imu_data(ptr, &imu);
                    if ekf2_update(ptr) {
                        update_success += 1;
                    } else {
                        update_failures += 1;
                    }
                }
            }
            "mag" => {
                if ts >= last_mag_ts + 13_000 {
                    last_mag_ts = ts;
                    mag_samples += 1;
                    let mag = EkfMagSample::new(
                        ts,
                        [
                            parse_f32(&parts, 2),
                            parse_f32(&parts, 3),
                            parse_f32(&parts, 4),
                        ],
                    );
                    unsafe {
                        ekf2_sys::ekf2_set_mag_data(ptr, &mag);
                    }
                }
            }
            "baro" => {
                if ts >= last_baro_ts + 13_000 {
                    last_baro_ts = ts;
                    baro_samples += 1;
                    let baro = EkfBaroSample::new(ts, parse_f32(&parts, 2));
                    unsafe {
                        ekf2_sys::ekf2_set_baro_data(ptr, &baro);
                    }
                }
            }
            "gps" => {
                gps_samples += 1;
                let alt_m = parse_f32(&parts, 2) / 1000.0;
                let lat = parse_f64(&parts, 3) * 1e-7;
                let lon = parse_f64(&parts, 4) * 1e-7;
                let vel_ned = [
                    parse_f32(&parts, 5),
                    parse_f32(&parts, 6),
                    parse_f32(&parts, 7),
                ];
                let gps = EkfGnssSample::new(ts, lat, lon, alt_m, vel_ned, 0.5, 0.8, 0.2, 3, 16);
                unsafe {
                    ekf2_sys::ekf2_set_gps_data(ptr, &gps);
                }
            }
            _ => {}
        }
    }

    assert!(
        imu_samples > expectations.min_imu_samples,
        "{} replay expected substantial IMU coverage, got {}, expected > {}",
        expectations.name,
        imu_samples,
        expectations.min_imu_samples
    );
    assert!(
        mag_samples > expectations.min_mag_samples,
        "{} replay expected substantial MAG coverage, got {}, expected > {}",
        expectations.name,
        mag_samples,
        expectations.min_mag_samples
    );
    assert!(
        baro_samples > expectations.min_baro_samples,
        "{} replay expected substantial BARO coverage, got {}, expected > {}",
        expectations.name,
        baro_samples,
        expectations.min_baro_samples
    );
    assert!(
        gps_samples > expectations.min_gps_samples,
        "{} replay expected substantial GPS coverage, got {}, expected > {}",
        expectations.name,
        gps_samples,
        expectations.min_gps_samples
    );

    assert!(
        update_success > expectations.min_successful_updates,
        "{} replay had too few successful updates: {} (expected > {})",
        expectations.name,
        update_success,
        expectations.min_successful_updates
    );
    assert!(
        update_failures < imu_samples,
        "all updates failed: {update_failures}/{imu_samples}"
    );
    assert!(
        update_success + update_failures == imu_samples,
        "update accounting mismatch: success={update_success}, fail={update_failures}, imu={imu_samples}"
    );

    unsafe { ekf2_destroy(ptr) };
}

#[test]
fn raw_replay_probe_iris_processes_fixture_with_reasonable_signal() {
    let expectations = ReplayProbeExpectations {
        name: "iris_gps",
        replay_fixture: "iris_gps_replay.csv",
        min_imu_samples: 7_000,
        min_mag_samples: 1_500,
        min_baro_samples: 1_500,
        min_gps_samples: 100,
        min_successful_updates: 2_000,
    };
    run_raw_replay_probe(&expectations);
}

#[test]
fn raw_replay_probe_gsf_reset_processes_fixture_with_reasonable_signal() {
    let expectations = ReplayProbeExpectations {
        name: "ekf_gsf_reset",
        replay_fixture: "ekf_gsf_reset_replay.csv",
        min_imu_samples: 9_000,
        min_mag_samples: 1_200,
        min_baro_samples: 400,
        min_gps_samples: 80,
        min_successful_updates: 3_000,
    };
    run_raw_replay_probe(&expectations);
}
