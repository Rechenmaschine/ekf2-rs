use allocator_api2::alloc::{AllocError, Allocator, Layout};
use ekf2::{types::ImuSample, types::SystemFlagUpdate, Ekf};
use std::alloc::{alloc, alloc_zeroed, dealloc};
use std::ptr::NonNull;
use std::sync::atomic::{AtomicUsize, Ordering};

struct CountingAllocator {
    allocations: AtomicUsize,
    deallocations: AtomicUsize,
}

unsafe impl Allocator for CountingAllocator {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        if layout.size() == 0 {
            return Ok(NonNull::slice_from_raw_parts(NonNull::dangling(), 0));
        }

        let ptr = NonNull::new(unsafe { alloc(layout) }).ok_or(AllocError)?;
        self.allocations.fetch_add(1, Ordering::Relaxed);
        Ok(NonNull::slice_from_raw_parts(ptr, layout.size()))
    }

    fn allocate_zeroed(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        if layout.size() == 0 {
            return Ok(NonNull::slice_from_raw_parts(NonNull::dangling(), 0));
        }

        let ptr = NonNull::new(unsafe { alloc_zeroed(layout) }).ok_or(AllocError)?;
        self.allocations.fetch_add(1, Ordering::Relaxed);
        Ok(NonNull::slice_from_raw_parts(ptr, layout.size()))
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
        self.deallocations.fetch_add(1, Ordering::Relaxed);
        unsafe { dealloc(ptr.as_ptr(), layout) };
    }
}

#[test]
fn cpp_storage_uses_the_allocator_bound_to_each_instance() {
    let allocator_a = CountingAllocator {
        allocations: AtomicUsize::new(0),
        deallocations: AtomicUsize::new(0),
    };
    let allocator_b = CountingAllocator {
        allocations: AtomicUsize::new(0),
        deallocations: AtomicUsize::new(0),
    };

    let mut ekf_a = Ekf::new_in(&allocator_a, 0).expect("custom allocator A should initialize");
    let mut ekf_b = Ekf::new_in(&allocator_b, 0).expect("custom allocator B should initialize");
    let a_allocations_after_init = allocator_a.allocations.load(Ordering::Relaxed);
    let b_allocations_after_init = allocator_b.allocations.load(Ordering::Relaxed);

    // This forces a lazily-created C++ ring buffer as well as the buffers
    // constructed during Ekf initialization.
    let imu = ImuSample::new(10_000, [0.0; 3], [0.0, 0.0, -0.0981], 0.01, 0.01);
    let flags = SystemFlagUpdate {
        time_us: 10_000,
        ..SystemFlagUpdate::default()
    };
    ekf_a.set_imu_data(&imu);
    ekf_a.set_system_flag_data(&flags);
    ekf_b.set_imu_data(&imu);
    ekf_b.set_system_flag_data(&flags);

    ekf_a
        .reset(20_000)
        .expect("reset must preserve allocator ownership");

    assert!(allocator_a.allocations.load(Ordering::Relaxed) > a_allocations_after_init);
    assert!(allocator_b.allocations.load(Ordering::Relaxed) > b_allocations_after_init);

    let b_deallocations_before_a_drop = allocator_b.deallocations.load(Ordering::Relaxed);

    drop(ekf_a);
    assert_eq!(
        allocator_a.allocations.load(Ordering::Relaxed),
        allocator_a.deallocations.load(Ordering::Relaxed),
        "instance A must return every C++ allocation to allocator A"
    );
    assert_eq!(
        allocator_b.deallocations.load(Ordering::Relaxed),
        b_deallocations_before_a_drop,
        "dropping instance A must not deallocate through allocator B"
    );

    drop(ekf_b);
    assert_eq!(
        allocator_b.allocations.load(Ordering::Relaxed),
        allocator_b.deallocations.load(Ordering::Relaxed),
        "instance B must return every C++ allocation to allocator B"
    );
}
