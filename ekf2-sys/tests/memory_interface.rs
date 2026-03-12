use core::ffi::c_void;
use ekf2_sys::*;

const GUARD: usize = 128;

#[repr(C, align(64))]
struct Aligned<const N: usize>([u8; N]);

fn align_up(addr: usize, align: usize) -> usize {
    let mask = align - 1;
    (addr + mask) & !mask
}

fn assert_all_eq(slice: &[u8], expected: u8, what: &str) {
    assert!(
        slice.iter().all(|&b| b == expected),
        "{what} guard bytes were modified"
    );
}

#[test]
fn ffi_respects_object_boundaries() {
    let obj_size = unsafe { ekf2_sizeof() };
    let obj_align = unsafe { ekf2_alignof() }.max(1);

    // Extra slack allows runtime alignment adjustment while preserving guards.
    let mut obj_backing = Aligned::<131_072>([0xCD; 131_072]);
    let obj_base = obj_backing.0.as_mut_ptr() as usize;
    let obj_start = align_up(obj_base + GUARD, obj_align);
    let obj_capacity = 96_000usize;
    let obj_end = obj_start + obj_capacity;

    assert!(obj_size <= obj_capacity, "test object region is too small");
    assert!(obj_end + GUARD <= obj_base + obj_backing.0.len());
    assert_eq!(obj_start % obj_align, 0);

    let obj_ptr = obj_start as *mut c_void;
    let obj_prefix = &obj_backing.0[..(obj_start - obj_base)];
    let obj_suffix = &obj_backing.0[(obj_end - obj_base)..];

    let ekf = unsafe { ekf2_create(obj_ptr, obj_capacity) };
    assert!(
        !ekf.is_null(),
        "ekf2_create must succeed with ample object space"
    );

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

    assert_all_eq(obj_prefix, 0xCD, "object-prefix");
    assert_all_eq(obj_suffix, 0xCD, "object-suffix");
}

#[test]
fn ffi_rejects_too_small_object_storage_without_overwrite() {
    let obj_size = unsafe { ekf2_sizeof() };
    let obj_align = unsafe { ekf2_alignof() }.max(1);

    let mut obj_backing = Aligned::<262_144>([0xEE; 262_144]);
    let obj_base = obj_backing.0.as_mut_ptr() as usize;
    let obj_start = align_up(obj_base + GUARD, obj_align);
    let obj_capacity = obj_size.saturating_sub(1);
    let obj_end = obj_start + obj_capacity;

    assert!(obj_end + GUARD <= obj_base + obj_backing.0.len());

    let obj_ptr = obj_start as *mut c_void;
    let obj_prefix = &obj_backing.0[..(obj_start - obj_base)];
    let obj_suffix = &obj_backing.0[(obj_end - obj_base)..];

    let ptr = unsafe { ekf2_create(obj_ptr, obj_capacity) };
    assert!(
        ptr.is_null(),
        "ekf2_create must fail with undersized object buffer"
    );

    assert_all_eq(obj_prefix, 0xEE, "small-object-prefix");
    assert_all_eq(obj_suffix, 0xEE, "small-object-suffix");
}
