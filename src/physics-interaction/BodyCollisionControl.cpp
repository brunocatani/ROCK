#include "BodyCollisionControl.h"

#include "HavokOffsets.h"

#include "REL/Relocation.h"

namespace frik::rock::body_collision
{
    namespace
    {
        constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;

        bool isValidBodyId(RE::hknpBodyId bodyId)
        {
            return bodyId.value != kInvalidBodyId;
        }
    }

    bool tryReadFilterInfo(RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint32_t& outFilterInfo)
    {
        if (!world || !isValidBodyId(bodyId)) {
            return false;
        }

        auto* bodyArray = world->GetBodyArray();
        if (!bodyArray) {
            return false;
        }

        auto* bodyPtr = reinterpret_cast<char*>(&bodyArray[bodyId.value]);
        outFilterInfo = *reinterpret_cast<std::uint32_t*>(bodyPtr + offsets::kBody_CollisionFilterInfo);
        return true;
    }

    bool setFilterInfo(RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint32_t filterInfo)
    {
        if (!world || !isValidBodyId(bodyId)) {
            return false;
        }

        using SetCollisionFilter_t = void (*)(void*, std::uint32_t, std::uint32_t, std::uint32_t);
        static REL::Relocation<SetCollisionFilter_t> setBodyCollisionFilterInfo{ REL::Offset(offsets::kFunc_SetBodyCollisionFilterInfo) };
        setBodyCollisionFilterInfo(world, bodyId.value, filterInfo, 0);
        return true;
    }

    bool setBroadPhaseEnabled(RE::hknpWorld* world, RE::hknpBodyId bodyId, bool enabled)
    {
        if (!world || !isValidBodyId(bodyId)) {
            return false;
        }

        using SetBodyBroadPhaseEnabled_t = void (*)(void*, std::uint32_t, std::uint8_t);
        static REL::Relocation<SetBodyBroadPhaseEnabled_t> setBodyBroadPhaseEnabled{ REL::Offset(offsets::kFunc_SetBodyBroadPhaseEnabled) };
        setBodyBroadPhaseEnabled(world, bodyId.value, enabled ? std::uint8_t{ 1 } : std::uint8_t{ 0 });
        return true;
    }
}
