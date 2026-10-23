#pragma once

#include <cstddef>
#include <source_location>
#include <type_traits>

namespace rock::native_memory
{
    /*
     * Native scene and Havok pointers can become stale while FO4VR is updating
     * object trees. Guarded reads reject null and low addresses, then copy
     * under SEH: a fault returns false, costs nothing when no fault occurs and
     * may leave the target partially written. Writes and standalone range
     * checks (callers that dereference without SEH afterwards) query page
     * protection with VirtualQuery, which costs microseconds per call, so they
     * stay out of per-frame paths. Each guarded read reports its call site to
     * the performance profiler, which logs the hottest sites.
     */
    bool pointerLooksReadable(const void* ptr);
    bool pointerRangeLooksReadable(const void* ptr, std::size_t byteCount);
    bool pointerRangeLooksWritable(void* ptr, std::size_t byteCount);
    bool guardedCopyFromMemory(const void* source, void* target, std::size_t byteCount,
        const std::source_location& site = std::source_location::current());
    bool guardedCopyToMemory(void* target, const void* source, std::size_t byteCount);

    template <class T>
    bool tryReadValue(const T* address, T& out, const std::source_location& site = std::source_location::current())
    {
        static_assert(std::is_trivially_copyable_v<T>, "native reads must copy trivially copyable values");
        return guardedCopyFromMemory(address, &out, sizeof(T), site);
    }

    template <class T>
    bool tryReadField(const void* base, std::ptrdiff_t offset, T& out, const std::source_location& site = std::source_location::current())
    {
        if (!base) {
            return false;
        }

        const auto* address = reinterpret_cast<const T*>(reinterpret_cast<const char*>(base) + offset);
        return tryReadValue(address, out, site);
    }

    template <class T>
    bool tryWriteValue(T* address, const T& value)
    {
        static_assert(std::is_trivially_copyable_v<T>, "native writes must copy trivially copyable values");
        return guardedCopyToMemory(address, &value, sizeof(T));
    }
}
