//! Memory-interface tests for `ekf2_create` / `ekf2_destroy`.
//!
//! Run with no sensor features so the object size exactly matches the
//! IMU-only configuration exercised by the test body:
//!
//!   cargo test -p ekf2-sys --no-default-features

// Refuse to compile if any sensor feature is active — the object size
// must match the IMU-only configuration exercised below.
#[cfg(any(
    feature = "gnss",
    feature = "magnetometer",
    feature = "barometer",
    feature = "airspeed",
    feature = "aux-vel",
    feature = "baro-compensation",
    feature = "drag-fusion",
    feature = "external-vision",
    feature = "gnss-yaw",
    feature = "gravity-fusion",
    feature = "optical-flow",
    feature = "range-finder",
    feature = "sideslip",
    feature = "terrain",
    feature = "wind",
))]
compile_error!(
    "memory_interface tests must be run with no sensor features \
     (`cargo test -p ekf2-sys --no-default-features`). \
     The object size must match the IMU-only configuration exercised by the test body."
);

use core::ffi::c_void;
use ekf2_sys::*;

const GUARD: usize = 128;

#[repr(C, align(64))]
struct Aligned<const N: usize>([u8; N]);

fn align_up(addr: usize, align: usize) -> usize {
    let mask = align - 1;
    (addr + mask) & !mask
}

fn assert_guard_intact(guard: &[u8], fill: u8, label: &str) {
    for (i, &b) in guard.iter().enumerate() {
        if b != fill {
            panic!("{label}: byte {i} was {b:#04x}, expected {fill:#04x}");
        }
    }
}

#[test]
fn create_rejects_zero_size() {
    let obj_align = unsafe { ekf2_alignof() }.max(1);

    let mut backing = Aligned::<4096>([0xAA; 4096]);
    let base = backing.0.as_mut_ptr() as usize;
    let obj_start = align_up(base + GUARD, obj_align);
    let obj_end = obj_start; // capacity = 0

    assert!(obj_end + GUARD <= base + backing.0.len());

    let obj_ptr = obj_start as *mut c_void;
    let prefix = &backing.0[..(obj_start - base)];
    let suffix = &backing.0[(obj_end - base)..];

    let ptr = unsafe { ekf2_create(obj_ptr, 0) };
    assert!(ptr.is_null(), "ekf2_create must return NULL for size=0");

    assert_guard_intact(prefix, 0xAA, "zero-size prefix");
    assert_guard_intact(suffix, 0xAA, "zero-size suffix");
}

#[test]
fn create_rejects_undersized_by_one() {
    let obj_size = unsafe { ekf2_sizeof() };
    let obj_align = unsafe { ekf2_alignof() }.max(1);

    let mut backing = Aligned::<262_144>([0xEE; 262_144]);
    let base = backing.0.as_mut_ptr() as usize;
    let obj_start = align_up(base + GUARD, obj_align);
    let obj_capacity = obj_size.saturating_sub(1);
    let obj_end = obj_start + obj_capacity;

    assert!(obj_end + GUARD <= base + backing.0.len());

    let obj_ptr = obj_start as *mut c_void;
    let prefix = &backing.0[..(obj_start - base)];
    let suffix = &backing.0[(obj_end - base)..];

    let ptr = unsafe { ekf2_create(obj_ptr, obj_capacity) };
    assert!(ptr.is_null(), "ekf2_create must return NULL for size=sizeof-1");

    assert_guard_intact(prefix, 0xEE, "undersized prefix");
    assert_guard_intact(suffix, 0xEE, "undersized suffix");
}

#[test]
fn create_accepts_exact_size_and_respects_bounds() {
    let obj_size = unsafe { ekf2_sizeof() };
    let obj_align = unsafe { ekf2_alignof() }.max(1);

    // Allocate exactly obj_size bytes — guards sit immediately adjacent.
    let total = GUARD + obj_align + obj_size + GUARD;
    let mut backing = vec![0xCDu8; total];
    let base = backing.as_mut_ptr() as usize;
    let obj_start = align_up(base + GUARD, obj_align);
    let obj_capacity = obj_size;
    let obj_end = obj_start + obj_capacity;

    assert!(obj_end + GUARD <= base + backing.len());
    assert_eq!(obj_start % obj_align, 0);

    let obj_ptr = obj_start as *mut c_void;

    let ekf = unsafe { ekf2_create(obj_ptr, obj_capacity) };
    assert!(!ekf.is_null(), "ekf2_create must succeed with exact-size buffer");

    let ok = unsafe { ekf2_init(ekf, 0) };
    assert!(ok, "ekf2_init should succeed");

    let dt_s = 0.01f32;
    for i in 0..200u64 {
        let imu = EkfImuSample::new(
            1_000_000 + i * 10_000,
            [0.0, 0.0, 0.0002],
            [0.0, 0.0, -9.81 * dt_s],
            dt_s,
            dt_s,
        );
        unsafe { ekf2_set_imu_data(ekf, &imu) };
        let _ = unsafe { ekf2_update(ekf) };
    }

    unsafe { ekf2_destroy(ekf) };

    let prefix = &backing[..(obj_start - base)];
    let suffix = &backing[(obj_end - base)..];

    assert_guard_intact(prefix, 0xCD, "exact-size prefix");
    assert_guard_intact(suffix, 0xCD, "exact-size suffix");
}
