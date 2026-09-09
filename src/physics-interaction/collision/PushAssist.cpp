#include "physics-interaction/collision/PushAssist.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/NativeMemory.h"

#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiPoint.h"

namespace rock::push_assist
{
    bool applyPointImpulse(RE::hknpWorld* world, std::uint32_t bodyId, std::uintptr_t expectedOwner,
        const RE::NiPoint3& impulseHavok, const RE::NiPoint3& pointHavok)
    {
        // 141E08640 -> 141DF5900 -> 14153A4D0: impulse is R8, point is R9.
        // The latter computes (point-COM) x impulse and updates angular motion.
        static const bool verified = [] {
            constexpr std::array<std::uint8_t, 5> expected{0x48, 0x89, 0x6C, 0x24, 0x18};
            std::array<std::uint8_t, 5> live{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Module::get().base() + 0x1E08640),
                live.data(), live.size()) && live == expected;
        }();
        if (!verified || !expectedOwner || !havok_runtime::bodySlotLooksReadable(world, RE::hknpBodyId{bodyId})) return false;
        auto* owner = havok_runtime::getCollisionObjectFromBody(world, RE::hknpBodyId{bodyId});
        RE::hknpWorld* ownerWorld = nullptr;
        RE::hknpBodyId ownerBody{0x7FFF'FFFF};
        if (reinterpret_cast<std::uintptr_t>(owner) != expectedOwner ||
            !havok_runtime::tryResolveCollisionObjectBody(owner, ownerWorld, ownerBody) || ownerWorld != world || ownerBody.value != bodyId) return false;
        alignas(16) float impulse[4]{impulseHavok.x, impulseHavok.y, impulseHavok.z, 0};
        alignas(16) float point[4]{pointHavok.x, pointHavok.y, pointHavok.z, 0};
        if (!havok_runtime::isFinite3(impulse) || !havok_runtime::isFinite3(point)) return false;
        using Apply = std::uint8_t (*)(void*, float*, float*);
        static REL::Relocation<Apply> apply{REL::Offset(offsets::kFunc_CollisionObject_ApplyPointImpulse)};
        return apply(owner, impulse, point) != 0;
    }

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
