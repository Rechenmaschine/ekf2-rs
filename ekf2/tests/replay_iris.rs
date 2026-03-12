#![cfg(all(feature = "gnss", feature = "magnetometer", feature = "barometer"))]

use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;

use ekf2::{
    types::{BaroSample, GnssSample, ImuSample, MagSample},
    AidSource2d, Ekf, EkfError, SolnStatus,
};

fn fixture_path(file: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
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

fn require_fields(parts: &[&str], min_len: usize, sensor: &str) {
    assert!(
        parts.len() >= min_len,
        "malformed {sensor} row: expected at least {min_len} fields, got {}",
        parts.len()
    );
}

struct ReplayOutcome {
    q: [f32; 4],
    v: [f32; 3],
    p: [f32; 3],
    attitude_valid: bool,
    local_velocity_valid: bool,
    local_position_valid: bool,
    soln: SolnStatus,
    heading_innov_ratio: f32,
    horiz_vel_innov_ratio: f32,
    horiz_pos_innov_ratio: f32,
    gnss_pos: AidSource2d,
    eph: f32,
    epv: f32,
    imu_samples: usize,
    mag_samples: usize,
    baro_samples: usize,
    gps_samples: usize,
    update_failures: usize,
}

struct ReplayExpectations {
    name: &'static str,
    replay_fixture: &'static str,
    reference_fixture: &'static str,
    min_imu_samples: usize,
    min_mag_samples: usize,
    min_baro_samples: usize,
    min_gps_samples: usize,
    min_successful_updates: usize,
    quaternion_tol: f32,
    position_tol: Option<[f32; 3]>,
    gps_vel_gate_override: Option<f32>,
    gps_pos_gate_override: Option<f32>,
}

fn replay_fixture_into_ekf(expectations: &ReplayExpectations) -> ReplayOutcome {
    let mut ekf = Ekf::new(0).expect("init should succeed");

    if expectations.gps_vel_gate_override.is_some() || expectations.gps_pos_gate_override.is_some()
    {
        let mut params = ekf.params_mut();
        if let Some(v) = expectations.gps_vel_gate_override {
            params.set_gps_vel_gate(v);
        }
        if let Some(v) = expectations.gps_pos_gate_override {
            params.set_gps_pos_gate(v);
        }
    }

    let replay_file =
        File::open(fixture_path(expectations.replay_fixture)).expect("missing replay fixture");

    let mut prev_imu_ts = None::<u64>;
    let mut last_mag_ts = 0_u64;
    let mut last_baro_ts = 0_u64;
    let mut imu_samples = 0_usize;
    let mut mag_samples = 0_usize;
    let mut baro_samples = 0_usize;
    let mut gps_samples = 0_usize;
    let mut update_failures = 0_usize;

    for line in BufReader::new(replay_file).lines() {
        let line = line.expect("failed to read replay line");
        if line.trim().is_empty() {
            continue;
        }

        let parts: Vec<&str> = line.split(',').collect();
        require_fields(&parts, 2, "replay");
        let ts = parse_u64(&parts, 0);

        match parts[1] {
            "imu" => {
                require_fields(&parts, 8, "imu");
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

                let imu = ImuSample::new(
                    ts,
                    [gx * dt_s, gy * dt_s, gz * dt_s],
                    [ax * dt_s, ay * dt_s, az * dt_s],
                    dt_s,
                    dt_s,
                );

                ekf.set_imu_data(&imu);
                if let Err(EkfError::UpdateFailed) = ekf.update() {
                    update_failures += 1;
                }
            }
            "mag" => {
                require_fields(&parts, 5, "mag");
                // Keep aiding rates close to EKF observation interval.
                if ts >= last_mag_ts + 13_000 {
                    last_mag_ts = ts;
                    mag_samples += 1;
                    let mag = MagSample::new(
                        ts,
                        [
                            parse_f32(&parts, 2),
                            parse_f32(&parts, 3),
                            parse_f32(&parts, 4),
                        ],
                    );
                    ekf.set_mag_data(&mag);
                }
            }
            "baro" => {
                require_fields(&parts, 3, "baro");
                if ts >= last_baro_ts + 13_000 {
                    last_baro_ts = ts;
                    baro_samples += 1;
                    let baro = BaroSample::new(ts, parse_f32(&parts, 2));
                    ekf.set_baro_data(&baro);
                }
            }
            "gps" => {
                require_fields(&parts, 8, "gps");
                gps_samples += 1;
                let alt_m = parse_f32(&parts, 2) / 1000.0;

                // Matches PX4-ECL replay fixture ordering.
                let lat = parse_f64(&parts, 3) * 1e-7;
                let lon = parse_f64(&parts, 4) * 1e-7;

                let vel_ned = [
                    parse_f32(&parts, 5),
                    parse_f32(&parts, 6),
                    parse_f32(&parts, 7),
                ];

                let gps = GnssSample::new(ts, lat, lon, alt_m, vel_ned, 0.5, 0.8, 0.2, 3, 16);
                ekf.set_gps_data(&gps);
            }
            _ => {}
        }
    }

    let q = ekf.quaternion();
    let v = ekf.velocity_ned();
    let p = ekf.position_ned();
    let attitude_valid = ekf.attitude_valid();
    let local_velocity_valid = ekf.local_velocity_valid();
    let local_position_valid = ekf.local_position_valid();
    let soln = ekf.soln_status();
    let heading_innov_ratio = ekf.heading_innov_test_ratio();
    let horiz_vel_innov_ratio = ekf.horiz_vel_innov_test_ratio();
    let horiz_pos_innov_ratio = ekf.horiz_pos_innov_test_ratio();
    let gnss_pos = ekf.aid_src_gnss_pos();
    let (eph, epv) = ekf.lpos_accuracy();

    ReplayOutcome {
        q,
        v,
        p,
        attitude_valid,
        local_velocity_valid,
        local_position_valid,
        soln,
        heading_innov_ratio,
        horiz_vel_innov_ratio,
        horiz_pos_innov_ratio,
        gnss_pos,
        eph,
        epv,
        imu_samples,
        mag_samples,
        baro_samples,
        gps_samples,
        update_failures,
    }
}

fn load_reference_last_row(reference_fixture: &str) -> ([f32; 4], [f32; 3]) {
    let reference_file =
        File::open(fixture_path(reference_fixture)).expect("missing reference fixture");

    let last = BufReader::new(reference_file)
        .lines()
        .map(|line| line.expect("failed to read reference line"))
        .filter(|line| !line.starts_with("Timestamp"))
        .last()
        .expect("reference file must contain at least one data row");

    let parts: Vec<&str> = last.split(',').collect();
    require_fields(&parts, 11, "reference");

    let q_ref = [
        parse_f32(&parts, 1),
        parse_f32(&parts, 2),
        parse_f32(&parts, 3),
        parse_f32(&parts, 4),
    ];

    let p_ref = [
        parse_f32(&parts, 8),
        parse_f32(&parts, 9),
        parse_f32(&parts, 10),
    ];

    (q_ref, p_ref)
}

fn assert_replay_matches_reference(expectations: &ReplayExpectations) {
    let outcome = replay_fixture_into_ekf(expectations);
    let (q_ref, p_ref) = load_reference_last_row(expectations.reference_fixture);

    let q = outcome.q;
    let v = outcome.v;
    let p = outcome.p;

    assert!(q.iter().all(|v| v.is_finite()));
    assert!(v.iter().all(|v| v.is_finite()));
    assert!(p.iter().all(|v| v.is_finite()));
    assert!(outcome.attitude_valid);
    assert!(outcome.local_velocity_valid);
    assert!(outcome.local_position_valid);

    assert!(
        outcome.imu_samples > expectations.min_imu_samples,
        "{} replay used too few IMU samples: got {}, expected > {}",
        expectations.name,
        outcome.imu_samples,
        expectations.min_imu_samples
    );
    assert!(
        outcome.mag_samples > expectations.min_mag_samples,
        "{} replay used too few mag samples: got {}, expected > {}",
        expectations.name,
        outcome.mag_samples,
        expectations.min_mag_samples
    );
    assert!(
        outcome.baro_samples > expectations.min_baro_samples,
        "{} replay used too few barometer samples: got {}, expected > {}",
        expectations.name,
        outcome.baro_samples,
        expectations.min_baro_samples
    );
    assert!(
        outcome.gps_samples > expectations.min_gps_samples,
        "{} replay used too few GPS samples: got {}, expected > {}",
        expectations.name,
        outcome.gps_samples,
        expectations.min_gps_samples
    );
    assert!(
        outcome.update_failures < outcome.imu_samples,
        "all EKF updates failed during replay: {} / {}",
        outcome.update_failures,
        outcome.imu_samples
    );
    assert!(
        outcome.imu_samples - outcome.update_failures > expectations.min_successful_updates,
        "{} replay had too few successful EKF updates: {} / {} (expected > {})",
        expectations.name,
        outcome.imu_samples - outcome.update_failures,
        outcome.imu_samples,
        expectations.min_successful_updates
    );

    // Solution status: attitude, horiz vel, vert vel, horiz pos should all be good.
    let soln = outcome.soln;
    assert!(
        soln.contains(SolnStatus::ATTITUDE),
        "soln_status: ATTITUDE bit not set"
    );
    assert!(
        soln.contains(SolnStatus::VELOCITY_HORIZ),
        "soln_status: VELOCITY_HORIZ bit not set"
    );
    assert!(
        soln.contains(SolnStatus::VELOCITY_VERT),
        "soln_status: VELOCITY_VERT bit not set"
    );
    assert!(
        soln.contains(SolnStatus::POS_HORIZ_REL),
        "soln_status: POS_HORIZ_REL bit not set"
    );

    // Innovation ratios should be finite (not NaN) after convergence.
    assert!(
        outcome.heading_innov_ratio.is_finite(),
        "heading innov ratio is NaN"
    );
    assert!(
        outcome.horiz_vel_innov_ratio.is_finite(),
        "horiz vel innov ratio is NaN"
    );
    assert!(
        outcome.horiz_pos_innov_ratio.is_finite(),
        "horiz pos innov ratio is NaN"
    );

    // Aid source: GNSS position should have been fused at least once.
    let gnss_pos = outcome.gnss_pos;
    assert!(
        gnss_pos.fused != 0 || gnss_pos.time_last_fuse > 0,
        "aid_src_gnss_pos: never fused"
    );

    // Local position accuracy should be reasonable after GPS lock.
    let (eph, epv) = (outcome.eph, outcome.epv);
    assert!(
        eph.is_finite() && eph < 50.0,
        "lpos EPH unreasonably large: {eph}"
    );
    assert!(
        epv.is_finite() && epv < 50.0,
        "lpos EPV unreasonably large: {epv}"
    );

    // Quaternion sign is ambiguous (q and -q represent the same rotation).
    let q_ref_sign = if q.iter().zip(q_ref.iter()).map(|(a, b)| a * b).sum::<f32>() < 0.0 {
        -1.0
    } else {
        1.0
    };

    for i in 0..4 {
        let err = (q[i] - q_ref[i] * q_ref_sign).abs();
        assert!(
            err < expectations.quaternion_tol,
            "quaternion component {i} too far from reference: got {}, ref {}, err {}",
            q[i],
            q_ref[i] * q_ref_sign,
            err
        );
    }

    if let Some(position_tol) = expectations.position_tol {
        for i in 0..3 {
            let err = (p[i] - p_ref[i]).abs();
            let tol = position_tol[i];
            assert!(
                err < tol,
                "position component {i} too far from reference: got {}, ref {}, err {}, tol {}",
                p[i],
                p_ref[i],
                err,
                tol
            );
        }
    }
}

#[test]
fn replay_iris_gps_matches_reference_state() {
    let expectations = ReplayExpectations {
        name: "iris_gps",
        replay_fixture: "iris_gps_replay.csv",
        reference_fixture: "iris_gps_reference.csv",
        min_imu_samples: 7_000,
        min_mag_samples: 1_500,
        min_baro_samples: 1_500,
        min_gps_samples: 100,
        min_successful_updates: 2_000,
        quaternion_tol: 0.02,
        position_tol: Some([0.35, 0.35, 0.70]),
        gps_vel_gate_override: None,
        gps_pos_gate_override: None,
    };

    assert_replay_matches_reference(&expectations);
}

#[test]
fn replay_ekf_gsf_reset_matches_reference_state() {
    let expectations = ReplayExpectations {
        name: "ekf_gsf_reset",
        replay_fixture: "ekf_gsf_reset_replay.csv",
        reference_fixture: "ekf_gsf_reset_reference.csv",
        min_imu_samples: 9_000,
        min_mag_samples: 1_200,
        min_baro_samples: 400,
        min_gps_samples: 80,
        min_successful_updates: 3_000,
        quaternion_tol: 0.06,
        position_tol: None,
        gps_vel_gate_override: Some(1.0),
        gps_pos_gate_override: Some(1.0),
    };

    assert_replay_matches_reference(&expectations);
}
