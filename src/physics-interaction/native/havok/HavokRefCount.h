#pragma once

#include <cstdint>
#include <intrin.h>

namespace rock::havok_ref_count
{
    /*
     * FO4VR hknp objects use Havok's hkReferencedObject ref-count word rather
     * than Bethesda's NiRefObject release path. Generated ROCK bodies store
     * hknp shapes inside hknpPhysicsSystemData, so local references must be
     * balanced with the same low-word CAS and vtable +0x18 destroy callback
     * used by the executable. This helper keeps those ownership transitions
     * explicit and prevents body creation failures from leaking generated
     * shapes or system-data arrays.
     */
    inline void addRef(const void* object)
    {
        if (!object) {
            return;
        }

        auto* refCountDword = reinterpret_cast<volatile long*>(const_cast<char*>(reinterpret_cast<const char*>(object)) + 0x08);
        for (;;) {
            const long oldValue = *refCountDword;
            const auto flags = static_cast<std::uint16_t>((static_cast<std::uint32_t>(oldValue) >> 16) & 0xFFFFu);
            const auto refCount = static_cast<std::uint16_t>(oldValue & 0xFFFF);
            if (flags == 0 || refCount == 0xFFFF) {
                return;
            }

            const long newValue = (oldValue & static_cast<long>(0xFFFF0000u)) | static_cast<long>(static_cast<std::uint16_t>(refCount + 1));
            if (_InterlockedCompareExchange(refCountDword, newValue, oldValue) == oldValue) {
                return;
            }
        }
    }

    inline void release(const void* object)
    {
        if (!object) {
            return;
        }

        auto* mutableObject = const_cast<void*>(object);
        auto* refCountDword = reinterpret_cast<volatile long*>(reinterpret_cast<char*>(mutableObject) + 0x08);
        for (;;) {
            const long oldValue = *refCountDword;
            const auto flags = static_cast<std::uint16_t>((static_cast<std::uint32_t>(oldValue) >> 16) & 0xFFFFu);
            const auto refCount = static_cast<std::uint16_t>(oldValue & 0xFFFF);
            if (flags == 0 || refCount == 0 || refCount == 0xFFFF) {
                return;
            }

            const long newValue = (oldValue & static_cast<long>(0xFFFF0000u)) | static_cast<long>(static_cast<std::uint16_t>(refCount - 1));
            if (_InterlockedCompareExchange(refCountDword, newValue, oldValue) != oldValue) {
                continue;
            }

            if (refCount - 1 == 0) {
                auto** vtable = *reinterpret_cast<void***>(mutableObject);
                auto destroy = reinterpret_cast<void (*)(void*)>(vtable[3]);
                destroy(mutableObject);
            }
            return;
        }
    }
}
