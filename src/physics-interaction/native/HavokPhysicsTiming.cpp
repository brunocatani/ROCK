#include "physics-interaction/native/HavokPhysicsTiming.h"

#include "physics-interaction/native/HavokOffsets.h"

#include <REL/Relocation.h>

namespace rock::havok_physics_timing
{
    namespace
    {
        float readFloatGlobal(std::uintptr_t offset, float fallback)
        {
            REL::Relocation<float*> value{ REL::Offset(offset) };
            return value.address() ? *value : fallback;
        }

        std::uint32_t readUintGlobal(std::uintptr_t offset, std::uint32_t fallback)
        {
            REL::Relocation<std::uint32_t*> value{ REL::Offset(offset) };
            return value.address() ? *value : fallback;
        }
    }

    PhysicsTimingSample sampleCurrentTiming()
    {
        return makeTimingSample(
            readFloatGlobal(offsets::kData_BhkWorldRawDeltaSeconds, kFallbackPhysicsDeltaSeconds),
            readFloatGlobal(offsets::kData_BhkWorldSubstepDeltaSeconds, kFallbackPhysicsDeltaSeconds),
            readFloatGlobal(offsets::kData_BhkWorldRemainderDeltaSeconds, 0.0f),
            readFloatGlobal(offsets::kData_BhkWorldAccumulatedDeltaSeconds, kFallbackPhysicsDeltaSeconds),
            readUintGlobal(offsets::kData_BhkWorldSubstepCount, 1));
    }

    float sampleNativeFrameDeltaSeconds()
    {
        // Continuous engine frame dt (see kData_NativeFrameDeltaSeconds).
        // Returns 0 when unreadable or implausible so callers fail closed.
        const float value = readFloatGlobal(offsets::kData_NativeFrameDeltaSeconds, 0.0f);
        return isUsableDelta(value) ? value : 0.0f;
    }

}
