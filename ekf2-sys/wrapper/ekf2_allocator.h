/**
 * Per-instance allocation callbacks used by the EKF C++ object graph.
 *
 * The callbacks are deliberately raw C ABI so they can be supplied by Rust,
 * an embedded system, or another host application without making the C++ EKF
 * depend on a particular allocator implementation.
 */

#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void *(*EkfAllocateFn)(void *context, size_t size, size_t align);
typedef void (*EkfDeallocateFn)(void *context, void *ptr, size_t size, size_t align);

typedef struct EkfAllocator {
    void *context;
    EkfAllocateFn allocate;
    EkfAllocateFn allocate_zeroed;
    EkfDeallocateFn deallocate;
} EkfAllocator;

#ifdef __cplusplus
}
#endif
