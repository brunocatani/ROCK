#include "PushAssist.h"

#include "HavokOffsets.h"

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
        if (!world || bodyId == 0x7FFF'FFFF) {
            return false;
        }

        auto* motion = world->GetBodyMotion(RE::hknpBodyId{ bodyId });
        if (!motion) {
            return false;
        }

        auto* motionFloats = reinterpret_cast<float*>(reinterpret_cast<char*>(motion) + 0x40);
        alignas(16) float linearVelocity[4] = {
            motionFloats[0] + velocityDeltaHavok.x,
            motionFloats[1] + velocityDeltaHavok.y,
            motionFloats[2] + velocityDeltaHavok.z,
            0.0f,
        };
        alignas(16) float angularVelocity[4] = {
            motionFloats[4],
            motionFloats[5],
            motionFloats[6],
            0.0f,
        };

        using SetVelocityDeferred_t = void (*)(void*, std::uint32_t, const float*, const float*);
        static REL::Relocation<SetVelocityDeferred_t> setVelocityDeferred{ REL::Offset(offsets::kFunc_SetBodyVelocityDeferred) };
        setVelocityDeferred(world, bodyId, linearVelocity, angularVelocity);
        return true;
    }
}
