//! Bare-metal libc stubs

use core::ffi::{c_char, c_int, c_void};

#[no_mangle]
pub extern "C" fn printf(_fmt: *const c_char) -> c_int {
    0
}

#[no_mangle]
pub extern "C" fn fprintf(_stream: *mut c_void, _fmt: *const c_char) -> c_int {
    0
}

#[no_mangle]
pub extern "C" fn puts(_s: *const c_char) -> c_int {
    0
}

#[no_mangle]
pub extern "C" fn fwrite(
    _ptr: *const c_void,
    _size: usize,
    count: usize,
    _stream: *mut c_void,
) -> usize {
    count
}

#[no_mangle]
pub unsafe extern "C" fn qsort(
    base: *mut c_void,
    nmemb: usize,
    size: usize,
    compar: unsafe extern "C" fn(*const c_void, *const c_void) -> c_int,
) {
    if base.is_null() || size == 0 || nmemb < 2 {
        return;
    }
    let bytes = base as *mut u8;
    for i in 1..nmemb {
        let mut j = i;
        while j > 0 {
            let a = unsafe { bytes.add((j - 1) * size) };
            let b = unsafe { bytes.add(j * size) };
            if unsafe { compar(a as *const c_void, b as *const c_void) } <= 0 {
                break;
            }
            unsafe { core::ptr::swap_nonoverlapping(a, b, size) };
            j -= 1;
        }
    }
}
