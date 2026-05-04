#include "HavokPhysicsTiming.h"

#include "HavokOffsets.h"

#include <REL/Relocation.h>

namespace frik::rock::havok_physics_timing
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

}
