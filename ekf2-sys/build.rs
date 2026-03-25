use std::env;
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    let vendor_root = manifest_dir.join("vendor").join("PX4-Autopilot");
    let px4_src_dir = vendor_root.join("src");
    let ekf2_module_dir = px4_src_dir.join("modules").join("ekf2");
    let ekf_dir = ekf2_module_dir.join("EKF");
    let ekf_python_dir = ekf_dir.join("python");
    let lib_dir = px4_src_dir.join("lib");
    let matrix_dir = lib_dir.join("matrix");
    let geo_dir = lib_dir.join("geo");
    let atmosphere_dir = lib_dir.join("atmosphere");
    let lat_lon_alt_dir = lib_dir.join("lat_lon_alt");
    let world_magnetic_model_dir = lib_dir.join("world_magnetic_model");
    // include roots:
    //   px4_src_dir → resolves <lib/ringbuffer/TimestampedRingBuffer.hpp>
    //   lib_dir     → resolves <mathlib/math/Utilities.hpp>
    // The wrapper dir provides two include-path shims required by PX4 vendor headers:
    //   px4_platform_common/ — minimal platform-compat stubs (no PX4 OS dependency)
    //   uORB/topics/         — plain C struct definitions; NOT the uORB pub/sub system.
    //                          PX4's ekf.h hardcodes #include <uORB/topics/estimator_aid_sourceXd.h>
    //                          so the directory name is fixed by PX4 and cannot be changed.
    let wrapper_dir = manifest_dir.join("wrapper");

    // ── Vendor presence check ───────────────────────────────────────────
    // Check for a specific PX4 header rather than just the directory, since
    // the directory is created as a placeholder (.gitkeep) in the repository.
    let ekf_header = ekf_dir.join("ekf.h");
    if !ekf_header.exists() {
        panic!(
            "\n\n[ekf2-sys] PX4 vendor sources not found.\n\
             Expected: {}\n\n\
             If you are building from a git checkout, run `git submodule update --init --recursive`.\n\
             If you are building from a published crate, the package contents are incomplete and should be reported as a packaging bug.\n\
             This project stores the PX4 source subset at ekf2-sys/vendor/PX4-Autopilot.\n",
            ekf_header.display()
        );
    }

    // ── Feature → CONFIG define mapping ────────────────────────────────
    // Cargo normalises feature names: hyphens become underscores in env vars.
    let features: &[(&str, &str)] = &[
        ("GNSS", "CONFIG_EKF2_GNSS"),
        ("GNSS_YAW", "CONFIG_EKF2_GNSS_YAW"),
        ("MAGNETOMETER", "CONFIG_EKF2_MAGNETOMETER"),
        ("BAROMETER", "CONFIG_EKF2_BAROMETER"),
        ("BARO_COMPENSATION", "CONFIG_EKF2_BARO_COMPENSATION"),
        ("AIRSPEED", "CONFIG_EKF2_AIRSPEED"),
        ("RANGE_FINDER", "CONFIG_EKF2_RANGE_FINDER"),
        ("OPTICAL_FLOW", "CONFIG_EKF2_OPTICAL_FLOW"),
        ("EXTERNAL_VISION", "CONFIG_EKF2_EXTERNAL_VISION"),
        ("AUX_VEL", "CONFIG_EKF2_AUXVEL"),
        ("DRAG_FUSION", "CONFIG_EKF2_DRAG_FUSION"),
        ("WIND", "CONFIG_EKF2_WIND"),
        ("TERRAIN", "CONFIG_EKF2_TERRAIN"),
        ("GRAVITY_FUSION", "CONFIG_EKF2_GRAVITY_FUSION"),
        ("SIDESLIP", "CONFIG_EKF2_SIDESLIP"),
    ];

    let is_feature_enabled =
        |feat_upper: &str| -> bool { env::var(format!("CARGO_FEATURE_{}", feat_upper)).is_ok() };

    let mut active_defines: Vec<(&str, &str)> = Vec::new();
    for &(feat_upper, define) in features {
        if is_feature_enabled(feat_upper) {
            active_defines.push((define, "1"));
        }
    }

    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let is_baremetal = target_os == "none";

    // ── C++ compilation ─────────────────────────────────────────────────
    let mut build = cc::Build::new();
    build
        .cpp(true)
        .std("c++17")
        .include(&ekf2_module_dir) // resolves <EKF/ekf.h>
        .include(&ekf_dir)
        .include(&ekf_python_dir) // resolves <ekf_derivation/generated/...>
        .include(&matrix_dir)
        .include(&geo_dir)
        .include(&atmosphere_dir)
        .include(&lat_lon_alt_dir)
        .include(&world_magnetic_model_dir)
        .include(&px4_src_dir) // resolves <lib/ringbuffer/...>
        .include(&lib_dir) // resolves <mathlib/math/...>
        .include(&wrapper_dir) // resolves <px4_platform_common/...>, <uORB/topics/...>
        // Suppress warnings from vendored PX4 code
        .flag_if_supported("-Wno-unused-parameter")
        .flag_if_supported("-Wno-unused-variable")
        .flag_if_supported("-Wno-unused-but-set-variable")
        .flag_if_supported("-Wno-sign-compare")
        .flag_if_supported("-Wno-missing-field-initializers")
        .flag_if_supported("-Wno-deprecated-declarations")
        .flag_if_supported("-Wno-strict-aliasing")
        .flag_if_supported("-Wno-new-returns-null")
        // Needed for correct float behaviour
        .flag_if_supported("-fno-associative-math")
        // Keep C++ strictly non-throwing across the FFI boundary.
        .flag_if_supported("-fno-exceptions")
        .flag_if_supported("-fno-rtti")
        // Ensure caller code checks null returns from replacement operator new.
        .flag_if_supported("-fcheck-new");

    // Without MODULE_NAME: ECL_INFO/WARN/ERR fall through to printf/fprintf —
    // the client provides the implementation (e.g. via c-stubs on bare-metal).
    // With MODULE_NAME: ECL macros route to our no-op PX4_DEBUG stubs, silencing
    // all output. This is the default; enable the `logging` feature to opt out.
    if env::var("CARGO_FEATURE_LOGGING").is_err() {
        build.define("MODULE_NAME", "\"ekf2_rs\"");
    }

    // Bare-metal no_std extras.
    if is_baremetal {
        build
            // Disable C assert() to avoid pulling hosted libc assert runtime.
            .define("NDEBUG", None::<&str>)
            // In this mode we intentionally avoid auto-linking a host C++ stdlib.
            .cpp_link_stdlib(None);
    }

    for &(define, value) in &active_defines {
        build.define(define, value);
    }

    // ── Core EKF source files (unconditionally compiled) ────────────────
    // Matches PX4 EKF CMakeLists for v1.15-era layout.
    let core_sources = [
        "ekf.cpp",
        "ekf_helper.cpp",
        "estimator_interface.cpp",
        "height_control.cpp",
        "position_fusion.cpp",
        "velocity_fusion.cpp",
        "yaw_fusion.cpp",
        "control.cpp",
        "covariance.cpp",
        "imu_down_sampler/imu_down_sampler.cpp",
        "aid_sources/fake_height_control.cpp",
        "aid_sources/fake_pos_control.cpp",
        "aid_sources/ZeroGyroUpdate.cpp",
        "aid_sources/ZeroVelocityUpdate.cpp",
        "aid_sources/zero_innovation_heading_update.cpp",
        "output_predictor/output_predictor.cpp",
        "bias_estimator/bias_estimator.cpp",
    ];

    for src in &core_sources {
        let path = ekf_dir.join(src);
        if path.exists() {
            build.file(&path);
        } else {
            // Warn but don't hard-fail; file layout can vary by PX4 version.
            println!(
                "cargo:warning=[ekf2-sys] Core source not found: {}",
                path.display()
            );
        }
    }

    // ── Feature-gated aid source files ──────────────────────────────────
    // Paths below match PX4 v1.15-style EKF layout.
    let aid_sources: &[(&str, &str)] = &[
        ("GNSS", "aid_sources/gnss/gnss_checks.cpp"),
        ("GNSS", "aid_sources/gnss/gnss_height_control.cpp"),
        ("GNSS", "aid_sources/gnss/gps_control.cpp"),
        ("GNSS_YAW", "aid_sources/gnss/gnss_yaw_control.cpp"),
        ("GNSS", "yaw_estimator/EKFGSF_yaw.cpp"),
        ("MAGNETOMETER", "aid_sources/magnetometer/mag_control.cpp"),
        ("MAGNETOMETER", "aid_sources/magnetometer/mag_fusion.cpp"),
        ("BAROMETER", "aid_sources/barometer/baro_height_control.cpp"),
        ("AIRSPEED", "aid_sources/airspeed/airspeed_fusion.cpp"),
        (
            "RANGE_FINDER",
            "aid_sources/range_finder/range_finder_consistency_check.cpp",
        ),
        (
            "RANGE_FINDER",
            "aid_sources/range_finder/range_height_control.cpp",
        ),
        (
            "RANGE_FINDER",
            "aid_sources/range_finder/range_height_fusion.cpp",
        ),
        (
            "RANGE_FINDER",
            "aid_sources/range_finder/sensor_range_finder.cpp",
        ),
        (
            "OPTICAL_FLOW",
            "aid_sources/optical_flow/optical_flow_control.cpp",
        ),
        (
            "OPTICAL_FLOW",
            "aid_sources/optical_flow/optical_flow_fusion.cpp",
        ),
        (
            "EXTERNAL_VISION",
            "aid_sources/external_vision/ev_control.cpp",
        ),
        (
            "EXTERNAL_VISION",
            "aid_sources/external_vision/ev_height_control.cpp",
        ),
        (
            "EXTERNAL_VISION",
            "aid_sources/external_vision/ev_pos_control.cpp",
        ),
        (
            "EXTERNAL_VISION",
            "aid_sources/external_vision/ev_vel_control.cpp",
        ),
        (
            "EXTERNAL_VISION",
            "aid_sources/external_vision/ev_yaw_control.cpp",
        ),
        ("AUX_VEL", "aid_sources/auxvel/auxvel_fusion.cpp"),
        ("DRAG_FUSION", "aid_sources/drag/drag_fusion.cpp"),
        ("SIDESLIP", "aid_sources/sideslip/sideslip_fusion.cpp"),
        ("GRAVITY_FUSION", "aid_sources/gravity/gravity_fusion.cpp"),
        ("TERRAIN", "terrain_control.cpp"),
        ("WIND", "wind.cpp"),
    ];

    for &(feat_upper, src) in aid_sources {
        if is_feature_enabled(feat_upper) {
            let path = ekf_dir.join(src);
            if path.exists() {
                build.file(&path);
            } else {
                // Some files may not exist in older PX4 versions; warn only.
                println!(
                    "cargo:warning=[ekf2-sys] Aid source not found (may be OK for this PX4 version): {}",
                    path.display()
                );
            }
        }
    }

    // PX4 support libraries
    let geo_cpp = geo_dir.join("geo.cpp");
    if geo_cpp.exists() {
        build.file(&geo_cpp);
    }
    let atmosphere_cpp = atmosphere_dir.join("atmosphere.cpp");
    if atmosphere_cpp.exists() {
        build.file(&atmosphere_cpp);
    }
    let lat_lon_alt_cpp = lat_lon_alt_dir.join("lat_lon_alt.cpp");
    if lat_lon_alt_cpp.exists() {
        build.file(&lat_lon_alt_cpp);
    }
    let wmm_cpp = world_magnetic_model_dir.join("geo_mag_declination.cpp");
    if wmm_cpp.exists() {
        build.file(&wmm_cpp);
    }

    // Wrapper
    build.file(wrapper_dir.join("ekf2_wrapper.cpp"));
    build.file(wrapper_dir.join("allocator.cpp"));

    build.compile("ekf2_cpp");

    // ── Rebuild triggers ─────────────────────────────────────────────────
    println!("cargo:rerun-if-changed=wrapper/");
    println!("cargo:rerun-if-changed=vendor/PX4-Autopilot/");
    println!("cargo:rerun-if-changed=build.rs");

    // ── bindgen ──────────────────────────────────────────────────────────
    let mut clang_args: Vec<String> = vec![
        "-xc++".into(),
        format!("-I{}", ekf2_module_dir.display()),
        format!("-I{}", ekf_dir.display()),
        format!("-I{}", ekf_python_dir.display()),
        format!("-I{}", matrix_dir.display()),
        format!("-I{}", geo_dir.display()),
        format!("-I{}", atmosphere_dir.display()),
        format!("-I{}", lat_lon_alt_dir.display()),
        format!("-I{}", world_magnetic_model_dir.display()),
        format!("-I{}", px4_src_dir.display()),
        format!("-I{}", lib_dir.display()),
        format!("-I{}", wrapper_dir.display()),
        "-std=c++17".into(),
    ];

    for &(define, value) in &active_defines {
        clang_args.push(format!("-D{}={}", define, value));
    }
    let bindings = bindgen::Builder::default()
        .header(wrapper_dir.join("ekf2_wrapper.h").to_str().unwrap())
        .use_core()
        .ctypes_prefix("::core::ffi")
        .clang_args(&clang_args)
        // Only emit what we declared in the header (all Ekf-prefixed types and functions)
        .allowlist_function("ekf2_.*")
        .allowlist_type("Ekf.*")
        // Structs are #[repr(C)] by default via bindgen
        .derive_default(true)
        .derive_copy(true)
        .derive_debug(true)
        // Avoid pulling in std
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect(
            "bindgen failed to generate bindings — check submodule sources in vendor/PX4-Autopilot",
        );

    bindings
        .write_to_file(out_dir.join("bindings.rs"))
        .expect("Could not write bindings.rs");
}
