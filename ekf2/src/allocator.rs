use allocator_api2::alloc::{Allocator, Layout};
use core::ffi::c_void;
use core::ptr::NonNull;
use ekf2_sys as ffi;

unsafe extern "C" fn allocate<A: Allocator>(
    context: *mut c_void,
    size: usize,
    align: usize,
) -> *mut c_void {
    let allocator = unsafe { &*context.cast::<A>() };
    let Ok(layout) = Layout::from_size_align(size, align) else {
        return core::ptr::null_mut();
    };

    match allocator.allocate(layout) {
        Ok(block) => block.cast::<u8>().as_ptr().cast(),
        Err(_) => core::ptr::null_mut(),
    }
}

unsafe extern "C" fn allocate_zeroed<A: Allocator>(
    context: *mut c_void,
    size: usize,
    align: usize,
) -> *mut c_void {
    let allocator = unsafe { &*context.cast::<A>() };
    let Ok(layout) = Layout::from_size_align(size, align) else {
        return core::ptr::null_mut();
    };

    match allocator.allocate_zeroed(layout) {
        Ok(block) => block.cast::<u8>().as_ptr().cast(),
        Err(_) => core::ptr::null_mut(),
    }
}

unsafe extern "C" fn deallocate<A: Allocator>(
    context: *mut c_void,
    ptr: *mut c_void,
    size: usize,
    align: usize,
) {
    let Some(ptr) = NonNull::new(ptr.cast::<u8>()) else {
        return;
    };
    let Ok(layout) = Layout::from_size_align(size, align) else {
        return;
    };
    let allocator = unsafe { &*context.cast::<A>() };

    unsafe { allocator.deallocate(ptr, layout) };
}

pub(crate) fn raw<A: Allocator>(allocator: &A) -> ffi::EkfAllocator {
    ffi::EkfAllocator {
        context: allocator as *const A as *mut c_void,
        allocate: Some(allocate::<A>),
        allocate_zeroed: Some(allocate_zeroed::<A>),
        deallocate: Some(deallocate::<A>),
    }
}
