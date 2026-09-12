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
        /*
         * Unreadable globals report zero so makeTimingSample classifies the
         * sample as fallback; missing native timing must never look measured.
         */
        return makeTimingSample(
            readFloatGlobal(offsets::kData_BhkWorldRawDeltaSeconds, 0.0f),
            readFloatGlobal(offsets::kData_BhkWorldSubstepDeltaSeconds, 0.0f),
            readFloatGlobal(offsets::kData_BhkWorldRemainderDeltaSeconds, 0.0f),
            readFloatGlobal(offsets::kData_BhkWorldAccumulatedDeltaSeconds, 0.0f),
            readUintGlobal(offsets::kData_BhkWorldSubstepCount, 0));
    }

}
