/**
 * Explicit memory resource used by all allocations owned by one Ekf.
 *
 * This is intentionally a tiny C++ value type.  It erases the allocator at
 * the C ABI boundary, while keeping allocation/deallocation paired with the
 * same resource for the lifetime of the EKF instance.
 */

#pragma once

#include <cstddef>
#include <new>
#include <utility>

#include "ekf2_allocator.h"

class EkfMemory {
public:
    explicit EkfMemory(EkfAllocator allocator) : _allocator(allocator) {}

    bool valid() const
    {
        return _allocator.allocate != nullptr
            && _allocator.allocate_zeroed != nullptr
            && _allocator.deallocate != nullptr;
    }

    void *allocate(std::size_t size, std::size_t align) const
    {
        return valid() ? _allocator.allocate(_allocator.context, size, align) : nullptr;
    }

    void *allocate_zeroed(std::size_t size, std::size_t align) const
    {
        return valid() ? _allocator.allocate_zeroed(_allocator.context, size, align) : nullptr;
    }

    void deallocate(void *ptr, std::size_t size, std::size_t align) const
    {
        if (ptr != nullptr && valid()) {
            _allocator.deallocate(_allocator.context, ptr, size, align);
        }
    }

    EkfAllocator allocator() const { return _allocator; }

    template<typename T, typename... Args>
    T *create(Args&&... args) const
    {
        void *storage = allocate(sizeof(T), alignof(T));
        if (storage == nullptr) {
            return nullptr;
        }

        return ::new (storage) T(std::forward<Args>(args)...);
    }

    template<typename T>
    void destroy(T *object) const
    {
        if (object != nullptr) {
            object->~T();
            deallocate(object, sizeof(T), alignof(T));
        }
    }

    template<typename T>
    T *create_array(std::size_t count) const
    {
        if (count == 0 || count > static_cast<std::size_t>(-1) / sizeof(T)) {
            return nullptr;
        }

        T *objects = static_cast<T *>(allocate(sizeof(T) * count, alignof(T)));
        if (objects == nullptr) {
            return nullptr;
        }

        for (std::size_t i = 0; i < count; ++i) {
            ::new (static_cast<void *>(objects + i)) T{};
        }

        return objects;
    }

    template<typename T>
    void destroy_array(T *objects, std::size_t count) const
    {
        if (objects == nullptr) {
            return;
        }

        for (std::size_t i = count; i > 0; --i) {
            objects[i - 1].~T();
        }
        deallocate(objects, sizeof(T) * count, alignof(T));
    }

private:
    EkfAllocator _allocator;
};
