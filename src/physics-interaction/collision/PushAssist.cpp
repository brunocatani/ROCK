#include "physics-interaction/collision/PushAssist.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRuntime.h"

#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiPoint.h"

namespace frik::rock::push_assist
{
    bool applyLinearImpulse(void* collisionObject, const RE::NiPoint3& impulseHavok)
    {
        if (!collisionObject) {
            return false;
        }

        alignas(16) float impulse[4] = { impulseHavok.x, impulseHavok.y, impulseHavok.z, 0.0f };
        using ApplyLinearImpulse_t = std::uint8_t (*)(void*, float*);
        static REL::Relocation<ApplyLinearImpulse_t> applyImpulse{ REL::Offset(offsets::kFunc_CollisionObject_ApplyLinearImpulse) };
        return applyImpulse(collisionObject, impulse) != 0;
    }

    bool applyLinearVelocityDeltaDeferred(RE::hknpWorld* world, std::uint32_t bodyId, const RE::NiPoint3& velocityDeltaHavok)
    {
        return havok_runtime::applyLinearVelocityDeltaDeferred(world, bodyId, velocityDeltaHavok);
    }
}
