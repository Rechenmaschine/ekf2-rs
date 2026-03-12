/**
 * allocator.cpp — C++ allocator bridge to Rust's global allocator
 *
 * C++ operator new/delete are always routed through Rust's allocator symbols,
 * so EKF C++ heap usage follows the active Rust global allocator.
 *
 * On bare-metal targets (target_os = "none"), we also provide libc allocation
 * shims (malloc/free/calloc/realloc), because no hosted libc is present.
 */

#include <cstddef>
#include <cstdint>
#include <new>

// Rust allocator shim symbols exported by ekf2-sys/src/lib.rs.
extern "C" {
    uint8_t* ekf2_rust_alloc       (size_t size, size_t align);
    void     ekf2_rust_dealloc     (uint8_t* ptr, size_t size, size_t align);
    uint8_t* ekf2_rust_alloc_zeroed(size_t size, size_t align);
}

// Minimum header/alignment: 16 bytes is enough for two size_t fields and
// meets max_align_t on supported targets.
static constexpr size_t kMinAlign = 16;

static_assert(2 * sizeof(size_t) <= kMinAlign,
    "kMinAlign must be large enough to hold two size_t fields");

// Allocate n bytes with alignment `align` (raised to kMinAlign if smaller).
// Stores {n, align} in the prefix before the returned pointer.
static void* cpp_alloc(size_t n, size_t align)
{
    if (n == 0) n = 1;
    if (align < kMinAlign) align = kMinAlign;
    if (n > static_cast<size_t>(-1) - align) return nullptr;

    const size_t total = align + n;
    uint8_t* raw = ekf2_rust_alloc(total, align);
    if (!raw) return nullptr;

    uint8_t* user = raw + align;
    reinterpret_cast<size_t*>(user)[-1] = n;
    reinterpret_cast<size_t*>(user)[-2] = align;
    return user;
}

// Free a pointer returned by cpp_alloc.
static void cpp_free(void* p)
{
    if (!p) return;
    uint8_t* user  = static_cast<uint8_t*>(p);
    const size_t n = reinterpret_cast<size_t*>(user)[-1];
    const size_t align = reinterpret_cast<size_t*>(user)[-2];
    uint8_t* raw = user - align;
    ekf2_rust_dealloc(raw, align + n, align);
}

// ── C++ allocator (all targets) ────────────────────────────────────────────

void* operator new  (size_t n)                     { return cpp_alloc(n, kMinAlign); }
void* operator new[](size_t n)                     { return cpp_alloc(n, kMinAlign); }
void* operator new  (size_t n, const std::nothrow_t&) noexcept { return cpp_alloc(n, kMinAlign); }
void* operator new[](size_t n, const std::nothrow_t&) noexcept { return cpp_alloc(n, kMinAlign); }
void* operator new  (size_t n, std::align_val_t a) { return cpp_alloc(n, static_cast<size_t>(a)); }
void* operator new[](size_t n, std::align_val_t a) { return cpp_alloc(n, static_cast<size_t>(a)); }

void operator delete  (void* p) noexcept                            { cpp_free(p); }
void operator delete[](void* p) noexcept                            { cpp_free(p); }
void operator delete  (void* p, const std::nothrow_t&) noexcept     { cpp_free(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept     { cpp_free(p); }
void operator delete  (void* p, size_t) noexcept                    { cpp_free(p); }
void operator delete[](void* p, size_t) noexcept                    { cpp_free(p); }
void operator delete  (void* p, std::align_val_t) noexcept          { cpp_free(p); }
void operator delete[](void* p, std::align_val_t) noexcept          { cpp_free(p); }
void operator delete  (void* p, size_t, std::align_val_t) noexcept  { cpp_free(p); }
void operator delete[](void* p, size_t, std::align_val_t) noexcept  { cpp_free(p); }

// ── Bare-metal libc allocator shims ────────────────────────────────────────

#if !defined(__unix__) && !defined(__APPLE__) && !defined(_WIN32)
extern "C" {

void* malloc(size_t n)
{
    return cpp_alloc(n, kMinAlign);
}

void free(void* p)
{
    cpp_free(p);
}

void* calloc(size_t nmemb, size_t size)
{
    if (nmemb != 0 && size > static_cast<size_t>(-1) / nmemb) {
        return nullptr;
    }
    size_t n = nmemb * size;
    if (n == 0) n = 1;
    if (n > static_cast<size_t>(-1) - kMinAlign) return nullptr;
    const size_t total = kMinAlign + n;

    uint8_t* raw = ekf2_rust_alloc_zeroed(total, kMinAlign);
    if (!raw) return nullptr;

    uint8_t* user = raw + kMinAlign;
    reinterpret_cast<size_t*>(user)[-1] = n;
    reinterpret_cast<size_t*>(user)[-2] = kMinAlign;
    return user;
}

void* realloc(void* old, size_t n)
{
    // EKF code does not use realloc; this is a safe fallback only.
    if (!old) return malloc(n);
    if (n == 0) {
        free(old);
        return malloc(1);
    }

    uint8_t* old_user = static_cast<uint8_t*>(old);
    const size_t old_n = reinterpret_cast<size_t*>(old_user)[-1];

    void* p = malloc(n);
    if (p) {
        const size_t to_copy = old_n < n ? old_n : n;
        __builtin_memcpy(p, old, to_copy);
        free(old);
    }
    return p;
}

} // extern "C"
#endif
