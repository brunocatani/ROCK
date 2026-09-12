#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/input/NativeGrenadeThrowRuntime.h"

namespace rock
{
    namespace
    {
        constexpr float kNativeGrenadeCollisionGraceSeconds = 0.5f;
    }

    bool PhysicsInteraction::protectNativeGrenadeThrow()
    {
        auto* bhk = getPlayerBhkWorld();
        auto* world = bhk ? getHknpWorld(bhk) : nullptr;
        if (!_lifecycle.initialized.load(std::memory_order_acquire) ||
            bhk != _lifecycle.cachedBhkWorld || !physicsWritesAllowedForWorld(world)) return false;

        auto mutation = _generatedBodyStepDrive.callbackGate().pauseForMutation();
        const bool ready = refreshNativeGrenadeCollisionSuppression(world);
        if (!ready || !_suppression.nativeGrenadeLeases.beginDelayedRestore(kNativeGrenadeCollisionGraceSeconds)) {
            restoreNativeGrenadeCollisionSuppression(world);
            ROCK_LOG_WARN(Hand, "Native grenade blocked: collision protection could not cover current player colliders");
            return false;
        }
        ROCK_LOG_INFO(Hand, "Native grenade collision protection: bodies={} grace={:.2f}s after release",
            _suppression.nativeGrenadeLeases.size(), kNativeGrenadeCollisionGraceSeconds);
        return true;
    }

    bool PhysicsInteraction::refreshNativeGrenadeCollisionSuppression(RE::hknpWorld* world)
    {
        // Game thread under the existing callback mutation gate. Body IDs are
        // collected afresh; the shared registry verifies identity on restore.
        std::array<std::uint32_t, kGeneratedBodyContactRegistryCapacity + 3> desired{};
        std::size_t count = 0;
        bool complete = true;
        const auto append = [&](std::uint32_t id) {
            if (id == INVALID_CONTACT_BODY_ID) return;
            if (std::find(desired.begin(), desired.begin() + count, id) != desired.begin() + count) return;
            if (count == desired.size()) {complete = false; return;}
            desired[count++] = id;
        };
        for (const bool isLeft : {false, true}) {
            const auto& hand = isLeft ? _leftHand : _rightHand;
            const auto handCount = hand.getHandColliderBodyCount();
            complete = complete && handCount <= hand_collider_semantics::kHandColliderBodyCountPerHand;
            for (std::uint32_t i = 0; i < (std::min)(handCount,
                     static_cast<std::uint32_t>(hand_collider_semantics::kHandColliderBodyCountPerHand)); ++i)
                append(hand.getHandColliderBodyIdAtomic(i));
            if (!handCount) append(hand.getCollisionBodyId().value);
            append(_dynamicHandCollision.proxyBodyIdForDebug(isLeft, 0).value);
        }
        const auto bodyCount = _bodyBoneColliders.getBodyCount();
        const auto weaponCount = _weaponCollision.getWeaponBodyCount();
        complete = complete && bodyCount <= kBodyBoneColliderBodyCount && weaponCount <= MAX_WEAPON_COLLISION_BODIES;
        for (std::uint32_t i = 0; i < (std::min)(bodyCount,
                 static_cast<std::uint32_t>(kBodyBoneColliderBodyCount)); ++i)
            append(_bodyBoneColliders.getBodyIdAtomic(i));
        for (std::uint32_t i = 0; i < (std::min)(weaponCount,
                 static_cast<std::uint32_t>(MAX_WEAPON_COLLISION_BODIES)); ++i)
            append(_weaponCollision.getWeaponBodyIdAtomic(i));
        append(_dynamicWeaponCollision.proxyBodyIdForDebug().value);

        auto& leases = _suppression.nativeGrenadeLeases;
        const bool restorePending = leases.delayedRestorePending();
        const float remaining = leases.delayedRestoreRemainingSeconds();
        leases.releaseWhere(world, "native-grenade-stale",
            [&](std::uint32_t id) {return std::find(desired.begin(), desired.begin() + count, id) == desired.begin() + count;},
            [](std::uint32_t, const auto&) {});
        for (std::size_t i = 0; i < count; ++i) {
            const auto result = leases.acquire(world, desired[i], "native-grenade");
            std::uint32_t filter{};
            if (!result.valid || !body_collision::tryReadFilterInfo(world, RE::hknpBodyId{desired[i]}, filter) ||
                !(filter & collision_suppression_registry::kSuppressionNoCollideBit)) {
                complete = false;
                ROCK_LOG_SAMPLE_WARN(Hand, 1000, "Native grenade collision protection incomplete: body={}", desired[i]);
            }
        }
        if (restorePending && !leases.delayedRestorePending()) leases.beginDelayedRestore(remaining);
        return complete && count != 0;
    }

    void PhysicsInteraction::restoreNativeGrenadeCollisionSuppression(RE::hknpWorld* world)
    {
        auto& leases = _suppression.nativeGrenadeLeases;
        if (leases.empty()) return;
        auto mutation = _generatedBodyStepDrive.callbackGate().pauseForMutation();
        const auto count = leases.size();
        if (leases.releaseAll(world, "native-grenade-restore", [](std::uint32_t, const auto&) {})) {
            ROCK_LOG_INFO(Hand, "Native grenade collision protection restored: bodies={}", count);
        } else {
            ROCK_LOG_SAMPLE_WARN(Hand, 1000, "Native grenade collision restore pending: bodies={}", leases.size());
        }
    }

    void PhysicsInteraction::updateNativeGrenadeCollisionSuppression(RE::hknpWorld* world, float deltaSeconds)
    {
        auto& leases = _suppression.nativeGrenadeLeases;
        if (!world || leases.empty()) return;
        auto mutation = _generatedBodyStepDrive.callbackGate().pauseForMutation();
        if (!native_grenade_throw_runtime::active() &&
            (!leases.delayedRestorePending() || leases.advanceDelayedRestore(deltaSeconds))) {
            restoreNativeGrenadeCollisionSuppression(world);
        } else if (physicsWritesAllowedForWorld(world)) {
            static_cast<void>(refreshNativeGrenadeCollisionSuppression(world));
        }
    }
}
