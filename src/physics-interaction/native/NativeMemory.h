#pragma once

#include <cstddef>
#include <type_traits>

namespace rock::native_memory
{
    /*
     * Native scene and Havok pointers can become stale while FO4VR is updating
     * object trees. These helpers give callers a common fail-closed boundary:
     * VirtualQuery filters unreadable pages first, then SEH protects the final
     * copy so gameplay code never dereferences a small or stale native pointer.
     */
    bool pointerLooksReadable(const void* ptr);
    bool pointerRangeLooksReadable(const void* ptr, std::size_t byteCount);
    bool pointerRangeLooksWritable(void* ptr, std::size_t byteCount);
    bool guardedCopyFromMemory(const void* source, void* target, std::size_t byteCount);
    bool guardedCopyToMemory(void* target, const void* source, std::size_t byteCount);

    template <class T>
    bool tryReadValue(const T* address, T& out)
    {
        static_assert(std::is_trivially_copyable_v<T>, "native reads must copy trivially copyable values");
        return guardedCopyFromMemory(address, &out, sizeof(T));
    }

    template <class T>
    bool tryReadField(const void* base, std::ptrdiff_t offset, T& out)
    {
        if (!base) {
            return false;
        }

        const auto* address = reinterpret_cast<const T*>(reinterpret_cast<const char*>(base) + offset);
        return tryReadValue(address, out);
    }

    template <class T>
    bool tryWriteValue(T* address, const T& value)
    {
        static_assert(std::is_trivially_copyable_v<T>, "native writes must copy trivially copyable values");
        return guardedCopyToMemory(address, &value, sizeof(T));
    }
}
