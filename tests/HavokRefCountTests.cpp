#include "physics-interaction/native/HavokRefCount.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace
{
    struct NativeObject
    {
        void** vtable;
        long referenceWord;
        int destroyed = 0;
    };
    static_assert(offsetof(NativeObject, referenceWord) == 8);

    void destroy(void* pointer)
    {
        ++static_cast<NativeObject*>(pointer)->destroyed;
    }
}

int main()
{
    using namespace rock::havok_ref_count;
    std::array<void*, 4> vtable{ nullptr, nullptr, nullptr, reinterpret_cast<void*>(&destroy) };
    bool passed = true;
    const auto check = [&](bool condition, const char* label) {
        if (!condition) {
            std::printf("FAILED: %s\n", label);
            passed = false;
        }
    };

    // More acquisitions than a 16-bit count can represent must stay balanced
    // when each query releases its reference and the world retains its owner.
    NativeObject world{ vtable.data(), static_cast<long>(0xFFFF0001u) };
    for (int i = 0; i < 70'000; ++i) {
        addRef(&world);
        release(&world);
    }
    check(static_cast<std::uint32_t>(world.referenceWord) == 0xFFFF0001u && world.destroyed == 0,
        "repeated query ownership preserves the world reference");

    // A generated shape can have a local owner and a retained physics-system
    // owner. Releasing the former must not destroy the latter's live shape.
    NativeObject shape{ vtable.data(), static_cast<long>(0x00400002u) };
    release(&shape);
    check(shape.referenceWord == 0x00400001 && shape.destroyed == 0,
        "shared shape survives the local owner's release");
    release(&shape);
    check(shape.referenceWord == 0x00400000 && shape.destroyed == 1,
        "final shape reference invokes the native destroy callback");
    release(&shape);
    check(shape.destroyed == 1, "zero reference is not destroyed twice");

    NativeObject failedBuild{ vtable.data(), static_cast<long>(0x00400001u) };
    release(&failedBuild);
    check(failedBuild.referenceWord == 0x00400000 && failedBuild.destroyed == 1,
        "failed body creation releases its sole shape owner");

    NativeObject borrowed{ vtable.data(), 1 };
    addRef(&borrowed);
    release(&borrowed);
    check(borrowed.referenceWord == 1 && borrowed.destroyed == 0,
        "non-reference-counted storage is not released");
    addRef(nullptr);
    release(nullptr);
    return passed ? 0 : 1;
}
