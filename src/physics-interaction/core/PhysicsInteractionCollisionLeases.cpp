/*
 * COLLISION SUPPRESSION LEASES: turn hand and player collision off for a reason,
 * then put it back exactly as it was.
 *
 * Every lease family uses the same primitive pair, suppressHandCollisionLeases and
 * restoreHandCollisionLeases, so a new family cannot invent its own restore rule.
 * The three families are the dominant-weapon hand, the weapon-support hand, and
 * the post-drop hand.
 *
 * The native player suppression block at the bottom is the exception worth reading
 * carefully: it has a physics-thread twin. See the comments on those four functions
 * for the callback-gate rule that makes the unlocked read safe.
 */

#include "physics-interaction/core/PhysicsInteraction.h"

#include <algorithm>
#include <array>

#include "RockConfig.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/native/CharacterControllerRuntime.h"
#include "physics-interaction/native/havok/HavokRuntime.h"
#include "physics-interaction/native/query/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock
{
    namespace
    {
        constexpr float kNearbyCarCollisionRadiusGameUnits = 4096.0f;

        struct HandCollisionLeaseTarget
        {
            std::uint32_t bodyId = 0;
            const char* context = nullptr;
        };

        struct HandCollisionLeaseProfile
        {
            collision_suppression_registry::CollisionSuppressionOwner owner{};
            const char* logTag = nullptr;
            const char* handName = nullptr;
            const char* restoreContext = nullptr;
        };

        template <std::size_t Capacity>
        void appendHandCollisionLeaseTarget(
            std::array<HandCollisionLeaseTarget, Capacity>& targets,
            std::size_t& targetCount,
            std::uint32_t bodyId,
            const char* context)
        {
            if (targetCount >= targets.size()) {
                return;
            }
            targets[targetCount++] = HandCollisionLeaseTarget{
                .bodyId = bodyId,
                .context = context,
            };
        }

        template <std::size_t Capacity>
        void appendCurrentHandCollisionLeaseTargets(
            const Hand& hand,
            std::array<HandCollisionLeaseTarget, Capacity>& targets,
            std::size_t& targetCount,
            const char* colliderContext,
            const char* anchorContext)
        {
            const std::uint32_t colliderCount = hand.getHandColliderBodyCount();
            if (colliderCount > 0) {
                for (std::uint32_t i = 0; i < colliderCount; ++i) {
                    appendHandCollisionLeaseTarget(
                        targets,
                        targetCount,
                        hand.getHandColliderBodyIdAtomic(i),
                        colliderContext);
                }
                return;
            }

            appendHandCollisionLeaseTarget(
                targets,
                targetCount,
                hand.getCollisionBodyId().value,
                anchorContext);
        }

        template <std::size_t Capacity>
        void suppressHandCollisionLeases(
            RE::hknpWorld* world,
            hand_collision_suppression_math::SuppressionSet<Capacity>& suppressionSet,
            std::span<const HandCollisionLeaseTarget> targets,
            const HandCollisionLeaseProfile& profile,
            std::uint32_t invalidBodyId)
        {
            for (const auto& target : targets) {
                if (target.bodyId == invalidBodyId) {
                    continue;
                }

                std::uint32_t currentFilter = 0;
                if (!body_collision::tryReadFilterInfo(
                        world,
                        RE::hknpBodyId{ target.bodyId },
                        currentFilter)) {
                    continue;
                }

                const auto suppression =
                    hand_collision_suppression_math::beginSuppression(
                        suppressionSet,
                        target.bodyId,
                        currentFilter);
                if (!suppression.stored) {
                    ROCK_LOG_WARN(Weapon,
                        "{}: {} hand suppression set full; bodyId={} context={} left active",
                        profile.logTag,
                        profile.handName,
                        target.bodyId,
                        target.context ? target.context : "unknown");
                    continue;
                }

                const auto registryResult =
                    collision_suppression_registry::
                        globalCollisionSuppressionRegistry()
                            .acquire(
                                world,
                                target.bodyId,
                                profile.owner,
                                target.context);
                if (!registryResult.valid ||
                    (!registryResult.firstLeaseForBody &&
                        !registryResult.filterChanged)) {
                    continue;
                }

                ROCK_LOG_DEBUG(Weapon,
                    "{}: {} hand collision lease acquired bodyId={} context={} filter=0x{:08X}->0x{:08X} wasDisabledBefore={} leases={}",
                    profile.logTag,
                    profile.handName,
                    target.bodyId,
                    target.context ? target.context : "unknown",
                    registryResult.filterBefore,
                    registryResult.filterAfter,
                    registryResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    registryResult.activeLeaseCount);
            }
        }

        template <std::size_t Capacity>
        bool restoreHandCollisionLeases(
            RE::hknpWorld* world,
            hand_collision_suppression_math::SuppressionSet<Capacity>& suppressionSet,
            const HandCollisionLeaseProfile& profile,
            std::uint32_t invalidBodyId)
        {
            if (!hand_collision_suppression_math::hasActive(suppressionSet)) {
                return true;
            }
            if (!world) {
                ROCK_LOG_WARN(Weapon,
                    "{}: cannot restore {} hand collision yet (world=null); preserving suppression leases",
                    profile.logTag,
                    profile.handName);
                return false;
            }

            bool restoreDeferred = false;
            for (const auto& entry : suppressionSet.entries) {
                if (!entry.active || entry.bodyId == invalidBodyId) {
                    continue;
                }

                const auto releaseResult =
                    collision_suppression_registry::
                        globalCollisionSuppressionRegistry()
                            .release(
                                world,
                                entry.bodyId,
                                profile.owner,
                                profile.restoreContext);
                if (releaseResult.readFailed) {
                    restoreDeferred = true;
                    continue;
                }

                ROCK_LOG_DEBUG(Weapon,
                    "{}: {} hand collision lease released bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} fullyReleased={}",
                    profile.logTag,
                    profile.handName,
                    entry.bodyId,
                    releaseResult.filterBefore,
                    releaseResult.filterAfter,
                    releaseResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    releaseResult.bodyFullyReleased ? "yes" : "no");
            }
            if (restoreDeferred) {
                ROCK_LOG_WARN(Weapon,
                    "{}: {} hand collision restore deferred; suppression leases preserved",
                    profile.logTag,
                    profile.handName);
                return false;
            }

            hand_collision_suppression_math::clear(suppressionSet);
            return true;
        }

    }

    void PhysicsInteraction::suppressRightHandCollisionForDominantWeapon(RE::hknpWorld* world)
    {
        // The equipped weapon is the physical owner of the right hand.
        _rightDominantWeaponCollisionSuppressed.store(true, std::memory_order_release);
        if (!world || !_rightHand.hasCollisionBody()) {
            return;
        }

        std::array<
            HandCollisionLeaseTarget,
            hand_collider_semantics::kHandColliderBodyCountPerHand>
            targets{};
        std::size_t targetCount = 0;
        appendCurrentHandCollisionLeaseTargets(
            _rightHand,
            targets,
            targetCount,
            "dominant-weapon-hand",
            "dominant-weapon-hand");
        suppressHandCollisionLeases(
            world,
            _rightDominantWeaponCollisionSuppression,
            std::span(targets.data(), targetCount),
            HandCollisionLeaseProfile{
                .owner = collision_suppression_registry::
                    CollisionSuppressionOwner::WeaponDominantHand,
                .logTag = "DominantWeapon",
                .handName = "right",
                .restoreContext = "dominant-weapon-hand",
            },
            INVALID_CONTACT_BODY_ID);
    }

    void PhysicsInteraction::restoreRightHandCollisionAfterDominantWeapon(RE::hknpWorld* world)
    {
        const bool restored = restoreHandCollisionLeases(
            world,
            _rightDominantWeaponCollisionSuppression,
            HandCollisionLeaseProfile{
                .owner = collision_suppression_registry::
                    CollisionSuppressionOwner::WeaponDominantHand,
                .logTag = "DominantWeapon",
                .handName = "right",
                .restoreContext = "dominant-weapon-hand",
            },
            INVALID_CONTACT_BODY_ID);
        if (restored) {
            _rightDominantWeaponCollisionSuppressed.store(false, std::memory_order_release);
        }
    }

    void PhysicsInteraction::suppressHandCollisionForWeaponSupport(RE::hknpWorld* world, bool isLeft)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::SupportGripSuppression);

        /*
         * Layer 43 vs 44 is intentionally allowed so a free hand can physically
         * touch the equipped weapon before a part grip starts. Once the part
         * grip owns the transform, that hand body becomes a driver and must
         * stop solving against the weapon package just like held-object hand
         * collision suppression. This applies to the offhand support grip and
         * to the detached firing hand's part grips symmetrically.
         */
        Hand& hand = isLeft ? _leftHand : _rightHand;
        auto& suppressionSet = isLeft ? _leftWeaponSupportCollisionSuppression : _rightWeaponSupportCollisionSuppression;
        auto& suppressedFlag = isLeft ? _leftWeaponSupportCollisionSuppressed : _rightWeaponSupportCollisionSuppressed;
        suppressedFlag.store(true, std::memory_order_release);

        auto bodyAlreadySuppressed = [&](std::uint32_t bodyId) {
            return bodyId == INVALID_CONTACT_BODY_ID ||
                   hand_collision_suppression_math::findSuppressionState(suppressionSet, bodyId) != nullptr;
        };

        auto currentHandBodiesAlreadySuppressed = [&]() {
            if (!hand.hasCollisionBody()) {
                return false;
            }

            bool sawValidBody = false;
            const std::uint32_t colliderCount = hand.getHandColliderBodyCount();
            if (colliderCount > 0) {
                for (std::uint32_t i = 0; i < colliderCount; ++i) {
                    const std::uint32_t bodyId = hand.getHandColliderBodyIdAtomic(i);
                    if (bodyId == INVALID_CONTACT_BODY_ID) {
                        continue;
                    }
                    sawValidBody = true;
                    if (!bodyAlreadySuppressed(bodyId)) {
                        return false;
                    }
                }
                return sawValidBody;
            }

            const std::uint32_t bodyId = hand.getCollisionBodyId().value;
            return bodyId != INVALID_CONTACT_BODY_ID && bodyAlreadySuppressed(bodyId);
        };

        if (currentHandBodiesAlreadySuppressed()) {
            return;
        }

        if (!world || !hand.hasCollisionBody()) {
            return;
        }

        std::array<
            HandCollisionLeaseTarget,
            hand_collider_semantics::kHandColliderBodyCountPerHand>
            targets{};
        std::size_t targetCount = 0;
        appendCurrentHandCollisionLeaseTargets(
            hand,
            targets,
            targetCount,
            "weapon-support-hand",
            "weapon-support-hand");
        suppressHandCollisionLeases(
            world,
            suppressionSet,
            std::span(targets.data(), targetCount),
            HandCollisionLeaseProfile{
                .owner = collision_suppression_registry::
                    CollisionSuppressionOwner::WeaponSupportHand,
                .logTag = "TwoHandedGrip",
                .handName = isLeft ? "left" : "right",
                .restoreContext = "weapon-support-hand",
            },
            INVALID_CONTACT_BODY_ID);
    }

    void PhysicsInteraction::restoreHandCollisionAfterWeaponSupport(RE::hknpWorld* world, bool isLeft)
    {
        auto& suppressionSet = isLeft ? _leftWeaponSupportCollisionSuppression : _rightWeaponSupportCollisionSuppression;
        auto& suppressedFlag = isLeft ? _leftWeaponSupportCollisionSuppressed : _rightWeaponSupportCollisionSuppressed;
        const bool restored = restoreHandCollisionLeases(
            world,
            suppressionSet,
            HandCollisionLeaseProfile{
                .owner = collision_suppression_registry::
                    CollisionSuppressionOwner::WeaponSupportHand,
                .logTag = "TwoHandedGrip",
                .handName = isLeft ? "left" : "right",
                .restoreContext = "weapon-support-hand",
            },
            INVALID_CONTACT_BODY_ID);
        if (restored) {
            suppressedFlag.store(false, std::memory_order_release);
        }
    }

    void PhysicsInteraction::suppressHandCollisionAfterEquippedWeaponDrop(
        RE::hknpWorld* world,
        equipped_weapon_drop_policy::SourceHand sourceHand)
    {
        if (sourceHand != equipped_weapon_drop_policy::SourceHand::Right &&
            sourceHand != equipped_weapon_drop_policy::SourceHand::Left) {
            return;
        }

        const bool isLeft = equipped_weapon_drop_policy::isLeft(sourceHand);
        auto& hand = isLeft ? _leftHand : _rightHand;
        auto& suppressionSet = isLeft ? _leftEquippedWeaponDropCollisionSuppression : _rightEquippedWeaponDropCollisionSuppression;
        auto& suppressed = isLeft ? _leftEquippedWeaponDropCollisionSuppressed : _rightEquippedWeaponDropCollisionSuppressed;
        auto& delayedRestore = isLeft ? _leftEquippedWeaponDropDelayedRestore : _rightEquippedWeaponDropDelayedRestore;
        hand_collision_suppression_math::clear(delayedRestore);

        if (!world || !hand.hasCollisionBody()) {
            return;
        }

        std::array<
            HandCollisionLeaseTarget,
            kGrabCollisionSuppressionBodyCountPerHand>
            targets{};
        std::size_t targetCount = 0;
        appendCurrentHandCollisionLeaseTargets(
            hand,
            targets,
            targetCount,
            "equipped-weapon-drop-hand-suite",
            "equipped-weapon-drop-hand-anchor");

        std::array<std::uint32_t, kGrabCollisionSuppressionArmBodyCountPerHand> armBodyIds{};
        const auto armBodyCount = _bodyBoneColliders.copyGrabSuppressionArmBodyIdsAtomic(isLeft, armBodyIds.data(), armBodyIds.size());
        for (std::uint32_t i = 0; i < armBodyCount && i < armBodyIds.size(); ++i) {
            appendHandCollisionLeaseTarget(
                targets,
                targetCount,
                armBodyIds[i],
                "equipped-weapon-drop-arm-chain");
        }
        suppressHandCollisionLeases(
            world,
            suppressionSet,
            std::span(targets.data(), targetCount),
            HandCollisionLeaseProfile{
                .owner = collision_suppression_registry::
                    CollisionSuppressionOwner::EquippedWeaponDropHand,
                .logTag = "EquippedWeaponDrop",
                .handName = equipped_weapon_drop_policy::sourceHandName(sourceHand),
                .restoreContext = "equipped-weapon-drop-hand",
            },
            INVALID_CONTACT_BODY_ID);

        const bool hasSuppression = hand_collision_suppression_math::hasActive(suppressionSet);
        suppressed.store(hasSuppression, std::memory_order_release);
        if (!hasSuppression) {
            return;
        }

        if (hand_collision_suppression_math::beginDelayedRestore(
                delayedRestore,
                suppressionSet,
                g_rockConfig.rockGrabReleaseHandCollisionDelaySeconds)) {
            ROCK_LOG_DEBUG(Weapon,
                "EquippedWeaponDrop: {} hand post-drop collision restore delayed bodies={} firstBodyId={} seconds={:.3f}",
                equipped_weapon_drop_policy::sourceHandName(sourceHand),
                delayedRestore.bodyCount,
                delayedRestore.bodyId,
                delayedRestore.remainingSeconds);
        } else {
            restoreHandCollisionAfterEquippedWeaponDrop(world, isLeft);
        }
    }

    void PhysicsInteraction::restoreHandCollisionAfterEquippedWeaponDrop(RE::hknpWorld* world, bool isLeft)
    {
        auto& suppressionSet = isLeft ? _leftEquippedWeaponDropCollisionSuppression : _rightEquippedWeaponDropCollisionSuppression;
        auto& suppressed = isLeft ? _leftEquippedWeaponDropCollisionSuppressed : _rightEquippedWeaponDropCollisionSuppressed;
        auto& delayedRestore = isLeft ? _leftEquippedWeaponDropDelayedRestore : _rightEquippedWeaponDropDelayedRestore;

        const bool restored = restoreHandCollisionLeases(
            world,
            suppressionSet,
            HandCollisionLeaseProfile{
                .owner = collision_suppression_registry::
                    CollisionSuppressionOwner::EquippedWeaponDropHand,
                .logTag = "EquippedWeaponDrop",
                .handName = isLeft ? "left" : "right",
                .restoreContext = "equipped-weapon-drop-hand",
            },
            INVALID_CONTACT_BODY_ID);
        if (restored) {
            hand_collision_suppression_math::clear(delayedRestore);
            suppressed.store(false, std::memory_order_release);
        }
    }

    void PhysicsInteraction::updateEquippedWeaponPostDropCollisionSuppression(RE::hknpWorld* world, float deltaSeconds)
    {
        auto updateHand = [&](bool isLeft) {
            auto& suppressionSet = isLeft ? _leftEquippedWeaponDropCollisionSuppression : _rightEquippedWeaponDropCollisionSuppression;
            auto& suppressed = isLeft ? _leftEquippedWeaponDropCollisionSuppressed : _rightEquippedWeaponDropCollisionSuppressed;
            auto& delayedRestore = isLeft ? _leftEquippedWeaponDropDelayedRestore : _rightEquippedWeaponDropDelayedRestore;

            if (delayedRestore.pending && !hand_collision_suppression_math::advanceDelayedRestore(delayedRestore, suppressionSet, deltaSeconds)) {
                return;
            }

            if (hand_collision_suppression_math::hasActive(suppressionSet)) {
                restoreHandCollisionAfterEquippedWeaponDrop(world, isLeft);
                return;
            }

            hand_collision_suppression_math::clear(delayedRestore);
            suppressed.store(false, std::memory_order_release);
        };

        updateHand(false);
        updateHand(true);
    }

    void PhysicsInteraction::clearEquippedWeaponPostDropCollisionSuppressionState()
    {
        hand_collision_suppression_math::clear(_rightEquippedWeaponDropCollisionSuppression);
        hand_collision_suppression_math::clear(_leftEquippedWeaponDropCollisionSuppression);
        hand_collision_suppression_math::clear(_rightEquippedWeaponDropDelayedRestore);
        hand_collision_suppression_math::clear(_leftEquippedWeaponDropDelayedRestore);
        _rightEquippedWeaponDropCollisionSuppressed.store(false, std::memory_order_release);
        _leftEquippedWeaponDropCollisionSuppressed.store(false, std::memory_order_release);
    }

    void PhysicsInteraction::restoreAllHandCollisionLeases(RE::hknpWorld* world)
    {
        // Restore every lease family before a frame or lifecycle boundary.
        restoreRightHandCollisionAfterDominantWeapon(world);
        restoreHandCollisionAfterWeaponSupport(world, true);
        restoreHandCollisionAfterWeaponSupport(world, false);
        restoreHandCollisionAfterEquippedWeaponDrop(world, false);
        restoreHandCollisionAfterEquippedWeaponDrop(world, true);
    }

    void PhysicsInteraction::clearAllHandCollisionSuppressionState()
    {
        hand_collision_suppression_math::clear(
            _rightDominantWeaponCollisionSuppression);
        hand_collision_suppression_math::clear(
            _leftWeaponSupportCollisionSuppression);
        hand_collision_suppression_math::clear(
            _rightWeaponSupportCollisionSuppression);
        _rightDominantWeaponCollisionSuppressed.store(
            false,
            std::memory_order_release);
        _leftWeaponSupportCollisionSuppressed.store(
            false,
            std::memory_order_release);
        _rightWeaponSupportCollisionSuppressed.store(
            false,
            std::memory_order_release);
        clearEquippedWeaponPostDropCollisionSuppressionState();
    }


    bool PhysicsInteraction::shouldSuppressNativePlayerCollisionBody(RE::bhkWorld* bhk, RE::hknpWorld* hknp, std::uint32_t bodyId) const
    {
        if (!bhk || !hknp || !contact_pipeline_policy::isValidBodyId(bodyId)) {
            return false;
        }

        if (bodyId == _rightHand.getCollisionBodyId().value ||
            bodyId == _leftHand.getCollisionBodyId().value ||
            _rightHand.isHandColliderBodyId(bodyId) ||
            _leftHand.isHandColliderBodyId(bodyId) ||
            _rightHand.isHeldBodyId(bodyId) ||
            _leftHand.isHeldBodyId(bodyId) ||
            _weaponCollision.isWeaponBodyIdAtomic(bodyId) ||
            _bodyBoneColliders.isColliderBodyIdAtomic(bodyId) ||
            ::rock::provider::isExternalBodyId(bodyId)) {
            return false;
        }

        std::uint32_t filterInfo = 0;
        if (!body_collision::tryReadFilterInfo(hknp, RE::hknpBodyId{ bodyId }, filterInfo)) {
            return false;
        }

        const std::uint32_t layer = filterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
        if (!collision_layer_policy::isNativePlayerCollisionSuppressionLayer(layer)) {
            return false;
        }

        auto* resolvedRef = resolveBodyToRef(bhk, hknp, RE::hknpBodyId{ bodyId });
        auto* player = RE::PlayerCharacter::GetSingleton();
        return !resolvedRef || resolvedRef == player;
    }

    // Runs on the main thread. Takes the callback gate before it clears the
    // lease array, so the physics-step reader cannot see a half-cleared array.
    void PhysicsInteraction::restoreNativePlayerCollisionSuppression(RE::hknpWorld* hknp, const char* reason)
    {
        if (_nativePlayerCollisionSuppressedBodyCount == 0) {
            _nativePlayerCollisionSuppressionRefreshFrames = 0;
            return;
        }

        auto cacheMutation = _generatedBodyStepDrive.callbackGate().pauseForMutation();
        std::array<NativePlayerCollisionSuppressedBody, kNativePlayerCollisionSuppressionBodyCapacity> pending{};
        std::uint32_t pendingCount = 0;

        auto keepPending = [&](const NativePlayerCollisionSuppressedBody& body) {
            if (pendingCount < pending.size()) {
                pending[pendingCount++] = body;
            }
        };

        for (std::uint32_t i = 0; i < _nativePlayerCollisionSuppressedBodyCount && i < _nativePlayerCollisionSuppressedBodies.size(); ++i) {
            const auto& body = _nativePlayerCollisionSuppressedBodies[i];
            if (!contact_pipeline_policy::isValidBodyId(body.bodyId)) {
                continue;
            }

            const auto releaseResult = collision_suppression_registry::globalCollisionSuppressionRegistry().release(
                hknp,
                body.bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::NativePlayerBody,
                reason ? reason : "native-player-body");
            if (releaseResult.readFailed) {
                keepPending(body);
            }
        }

        _nativePlayerCollisionSuppressedBodies = pending;
        _nativePlayerCollisionSuppressedBodyCount = pendingCount;
        _nativePlayerCollisionSuppressionRefreshFrames = pendingCount == 0 ? 0 : 30;
    }

    // Runs on the main thread. The caller must already hold the callback gate:
    // this rewrites the lease array that the physics-step twin below reads.
    void PhysicsInteraction::refreshNativePlayerCollisionSuppression(RE::hknpWorld* hknp, const char* context)
    {
        if (!g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled || !hknp || _nativePlayerCollisionSuppressedBodyCount == 0) {
            return;
        }

        std::uint32_t retainedCount = 0;
        for (std::uint32_t i = 0; i < _nativePlayerCollisionSuppressedBodyCount && i < _nativePlayerCollisionSuppressedBodies.size(); ++i) {
            const auto body = _nativePlayerCollisionSuppressedBodies[i];
            if (!contact_pipeline_policy::isValidBodyId(body.bodyId)) {
                continue;
            }

            const auto refreshResult = collision_suppression_registry::globalCollisionSuppressionRegistry().refresh(
                hknp,
                body.bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::NativePlayerBody,
                context ? context : "native-player-body-refresh");
            if (refreshResult.readFailed || (refreshResult.valid && !refreshResult.staleLeaseDiscarded)) {
                _nativePlayerCollisionSuppressedBodies[retainedCount++] = body;
            }
        }

        for (std::uint32_t i = retainedCount; i < _nativePlayerCollisionSuppressedBodies.size(); ++i) {
            _nativePlayerCollisionSuppressedBodies[i] = {};
        }
        _nativePlayerCollisionSuppressedBodyCount = retainedCount;
    }

    // Runs on the physics step thread. This is the only physics-thread writer
    // outside PhysicsInteractionPhysicsStep.cpp and PhysicsInteractionContacts.cpp.
    // It stays with its main-thread family because all four functions share the
    // same lease array.
    //
    // The lease array is read here without a lock. That is safe only because
    // every main-thread mutator of _nativePlayerCollisionSuppressedBodies holds
    // _generatedBodyStepDrive.callbackGate().pauseForMutation() while it writes,
    // which keeps this callback out. Never mutate the array from the main thread
    // without that gate.
    void PhysicsInteraction::refreshNativePlayerCollisionSuppressionFromPhysicsSubstep(RE::hknpWorld* hknp, const char* context)
    {
        if (!g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled || !hknp || _nativePlayerCollisionSuppressedBodyCount == 0) {
            return;
        }

        for (std::uint32_t i = 0; i < _nativePlayerCollisionSuppressedBodyCount && i < _nativePlayerCollisionSuppressedBodies.size(); ++i) {
            const auto& leasedBody = _nativePlayerCollisionSuppressedBodies[i];
            if (!contact_pipeline_policy::isValidBodyId(leasedBody.bodyId)) {
                continue;
            }

            const auto snapshot = havok_runtime::snapshotBody(hknp, RE::hknpBodyId{ leasedBody.bodyId });
            if (!snapshot.valid) {
                ROCK_LOG_SAMPLE_WARN(Hand,
                    1000,
                    "Native player collision suppression physics refresh skipped: bodyId={} context={} cannot snapshot body",
                    leasedBody.bodyId,
                    context ? context : "");
                continue;
            }

            if (snapshot.motionIndex != leasedBody.motionIndex ||
                snapshot.collisionObject != leasedBody.collisionObject ||
                snapshot.ownerNode != leasedBody.ownerNode) {
                ROCK_LOG_SAMPLE_WARN(Hand,
                    1000,
                    "Native player collision suppression physics refresh rejected recycled body: bodyId={} context={} oldMotion={} newMotion={} oldOwner={} newOwner={} oldCollision={} newCollision={}",
                    leasedBody.bodyId,
                    context ? context : "",
                    leasedBody.motionIndex,
                    snapshot.motionIndex,
                    static_cast<const void*>(leasedBody.ownerNode),
                    static_cast<const void*>(snapshot.ownerNode),
                    static_cast<const void*>(leasedBody.collisionObject),
                    static_cast<const void*>(snapshot.collisionObject));
                continue;
            }

            std::uint32_t currentFilter = 0;
            if (!body_collision::tryReadFilterInfo(hknp, RE::hknpBodyId{ leasedBody.bodyId }, currentFilter)) {
                ROCK_LOG_SAMPLE_WARN(Hand,
                    1000,
                    "Native player collision suppression physics refresh skipped: bodyId={} context={} cannot read filter",
                    leasedBody.bodyId,
                    context ? context : "");
                continue;
            }

            const std::uint32_t refreshedFilter = currentFilter | collision_suppression_registry::kSuppressionNoCollideBit;
            if (refreshedFilter != currentFilter) {
                body_collision::setFilterInfo(hknp, RE::hknpBodyId{ leasedBody.bodyId }, refreshedFilter);
            }
        }
    }

    // Runs on the main thread, once per frame from update(). Owns the gate for
    // the whole refresh-and-rescan pass.
    void PhysicsInteraction::updateNativePlayerCollisionSuppression(RE::bhkWorld* bhk, RE::hknpWorld* hknp)
    {
        if (!g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled) {
            restoreNativePlayerCollisionSuppression(hknp, "native-player-filter-disabled");
            return;
        }

        if (!bhk || !hknp) {
            return;
        }

        auto cacheMutation = _generatedBodyStepDrive.callbackGate().pauseForMutation();
        refreshNativePlayerCollisionSuppression(hknp, "native-player-body-frame-refresh");

        if (_nativePlayerCollisionSuppressionRefreshFrames > 0) {
            --_nativePlayerCollisionSuppressionRefreshFrames;
            return;
        }
        _nativePlayerCollisionSuppressionRefreshFrames = 30;

        struct NativePlayerBodyScanContext
        {
            PhysicsInteraction* self = nullptr;
            RE::bhkWorld* bhk = nullptr;
            RE::hknpWorld* hknp = nullptr;
            std::array<std::uint32_t, PhysicsInteraction::kNativePlayerCollisionSuppressionBodyCapacity> bodyIds{};
            std::uint32_t bodyCount = 0;
            bool overflow = false;
            RE::NiPoint3 playerPositionGameUnits{};
            bool playerPositionValid = false;
            RE::TESObjectREFR* rightHeldRef = nullptr;
            RE::TESObjectREFR* leftHeldRef = nullptr;
            std::array<DynamicWorldCarTarget, DynamicWorldCarCollisionRuntime::kMaxTrackedTargets> nearbyCars{};
            std::uint32_t nearbyCarCount = 0;
            bool nearbyCarOverflow = false;

            bool contains(std::uint32_t bodyId) const
            {
                for (std::uint32_t i = 0; i < bodyCount && i < bodyIds.size(); ++i) {
                    if (bodyIds[i] == bodyId) {
                        return true;
                    }
                }
                return false;
            }

            void append(std::uint32_t bodyId)
            {
                appendNearbyCar(bodyId);
                if (!self || !self->shouldSuppressNativePlayerCollisionBody(bhk, hknp, bodyId) || contains(bodyId)) {
                    return;
                }
                if (bodyCount >= bodyIds.size()) {
                    overflow = true;
                    return;
                }
                bodyIds[bodyCount++] = bodyId;
            }

            void appendNearbyCar(std::uint32_t bodyId)
            {
                if (!self || !bhk || !hknp || !playerPositionValid ||
                    !contact_pipeline_policy::isValidBodyId(bodyId)) {
                    return;
                }

                std::uint32_t filterInfo = 0;
                if (!body_collision::tryReadFilterInfo(hknp, RE::hknpBodyId{ bodyId }, filterInfo)) {
                    return;
                }
                const auto layer = filterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
                if (layer != collision_layer_policy::FO4_LAYER_CLUTTER &&
                    layer != collision_layer_policy::FO4_LAYER_CLUTTER_LARGE &&
                    !collision_layer_policy::isDynamicWorldCarLayer(layer)) {
                    return;
                }

                RE::NiTransform bodyWorld{};
                if (!havok_runtime::tryGetBodyWorldTransform(hknp, RE::hknpBodyId{ bodyId }, bodyWorld)) {
                    return;
                }
                const float dx = bodyWorld.translate.x - playerPositionGameUnits.x;
                const float dy = bodyWorld.translate.y - playerPositionGameUnits.y;
                const float dz = bodyWorld.translate.z - playerPositionGameUnits.z;
                const float distanceSquared = dx * dx + dy * dy + dz * dz;
                constexpr float radiusSquared =
                    kNearbyCarCollisionRadiusGameUnits * kNearbyCarCollisionRadiusGameUnits;
                if (!std::isfinite(distanceSquared) || distanceSquared > radiusSquared) {
                    return;
                }

                auto* ref = resolveBodyToRef(bhk, hknp, RE::hknpBodyId{ bodyId });
                if (!ref || ref == rightHeldRef || ref == leftHeldRef || ref->IsDeleted() || ref->IsDisabled() ||
                    !fo4vr::isExplodableCar(ref->GetObjectReference())) {
                    return;
                }
                for (std::uint32_t index = 0; index < nearbyCarCount && index < nearbyCars.size(); ++index) {
                    if (nearbyCars[index].ref == ref) {
                        return;
                    }
                }
                if (nearbyCarCount >= nearbyCars.size()) {
                    nearbyCarOverflow = true;
                    return;
                }
                nearbyCars[nearbyCarCount++] = DynamicWorldCarTarget{
                    .ref = ref,
                    .seedBodyId = bodyId,
                };
            }
        } scanContext{ this, bhk, hknp };

        scanContext.playerPositionValid =
            character_controller_runtime::tryGetPlayerActorPositionGameUnits(scanContext.playerPositionGameUnits);
        scanContext.rightHeldRef = _rightHand.isHolding() ? _rightHand.getHeldRef() : nullptr;
        scanContext.leftHeldRef = _leftHand.isHolding() ? _leftHand.getHeldRef() : nullptr;

        auto visitBody = [](std::uint32_t bodyId, void* userData) {
            auto* context = static_cast<NativePlayerBodyScanContext*>(userData);
            if (!context) {
                return false;
            }
            context->append(bodyId);
            return true;
        };

        auto scanCollisionObject = [&](RE::NiCollisionObject* collisionObject) {
            havok_runtime::forEachPhysicsSystemBodyIdDetailed(collisionObject, hknp, 256, visitBody, &scanContext);
        };

        auto scanNode = [&](auto&& self, RE::NiAVObject* node, int depth) -> void {
            if (!node || depth <= 0) {
                return;
            }

            scanCollisionObject(node->collisionObject.get());
            if (auto* niNode = node->IsNode()) {
                auto& children = niNode->GetRuntimeData().children;
                for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                    if (auto* child = children[i].get()) {
                        self(self, child, depth - 1);
                    }
                }
            }
        };

        if (auto* player = RE::PlayerCharacter::GetSingleton()) {
            if (player->currentProcess && player->currentProcess->middleHigh && player->currentProcess->middleHigh->poseBound) {
                scanCollisionObject(player->currentProcess->middleHigh->poseBound.get());
            }
        }
        scanNode(scanNode, f4vr::getFirstPersonSkeleton(), 64);
        scanNode(scanNode, f4vr::getWorldRootNode(), 64);

        if (scanContext.playerPositionValid) {
            _dynamicWorldCarCollision.synchronizeNearbyTargets(
                bhk,
                hknp,
                std::span<const DynamicWorldCarTarget>{ scanContext.nearbyCars.data(), scanContext.nearbyCarCount });
        }

        if (scanContext.nearbyCarOverflow) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                5000,
                "Nearby car collision target capacity exceeded; keeping first {} cars within {:.0f} game units",
                scanContext.nearbyCars.size(),
                kNearbyCarCollisionRadiusGameUnits);
        }

        if (scanContext.overflow && !_nativePlayerCollisionSuppressionOverflowLogged) {
            _nativePlayerCollisionSuppressionOverflowLogged = true;
            ROCK_LOG_WARN(Hand,
                "Native player collision suppression body capacity exceeded; keeping first {} bodies",
                kNativePlayerCollisionSuppressionBodyCapacity);
        } else if (!scanContext.overflow) {
            _nativePlayerCollisionSuppressionOverflowLogged = false;
        }

        std::array<NativePlayerCollisionSuppressedBody, kNativePlayerCollisionSuppressionBodyCapacity> next{};
        std::uint32_t nextCount = 0;
        auto nextContains = [&](std::uint32_t bodyId) {
            for (std::uint32_t i = 0; i < nextCount && i < next.size(); ++i) {
                if (next[i].bodyId == bodyId) {
                    return true;
                }
            }
            return false;
        };
        auto appendNext = [&](const NativePlayerCollisionSuppressedBody& body) {
            if (!contact_pipeline_policy::isValidBodyId(body.bodyId) || nextContains(body.bodyId) || nextCount >= next.size()) {
                return;
            }
            next[nextCount++] = body;
        };

        for (std::uint32_t i = 0; i < scanContext.bodyCount && i < scanContext.bodyIds.size(); ++i) {
            const auto bodyId = scanContext.bodyIds[i];
            const auto acquireResult = collision_suppression_registry::globalCollisionSuppressionRegistry().acquire(
                hknp,
                bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::NativePlayerBody,
                "native-player-body");
            if (acquireResult.valid && acquireResult.leaseIdentityValid) {
                appendNext({
                    bodyId,
                    acquireResult.leaseMotionIndex,
                    acquireResult.leaseCollisionObject,
                    acquireResult.leaseOwnerNode,
                });
            } else if (acquireResult.valid) {
                collision_suppression_registry::globalCollisionSuppressionRegistry().release(
                    hknp,
                    bodyId,
                    collision_suppression_registry::CollisionSuppressionOwner::NativePlayerBody,
                    "native-player-body-missing-identity");
                ROCK_LOG_WARN(Hand,
                    "Native player collision suppression acquisition rejected: bodyId={} lease identity unavailable",
                    bodyId);
            }
        }

        for (std::uint32_t i = 0; i < _nativePlayerCollisionSuppressedBodyCount && i < _nativePlayerCollisionSuppressedBodies.size(); ++i) {
            const auto& body = _nativePlayerCollisionSuppressedBodies[i];
            if (nextContains(body.bodyId)) {
                continue;
            }

            const auto releaseResult = collision_suppression_registry::globalCollisionSuppressionRegistry().release(
                hknp,
                body.bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::NativePlayerBody,
                "native-player-body-stale");
            if (releaseResult.readFailed) {
                appendNext(body);
            }
        }

        _nativePlayerCollisionSuppressedBodies = next;
        _nativePlayerCollisionSuppressedBodyCount = nextCount;
    }

}
