#include "physics-interaction/hand/Hand.h"

/*
 * Collision suppression around a grab: hand against grabbed object while held,
 * and body against held loose weapon for weapons carried across the player body.
 *
 * Every suppress has exactly one restore, and both restore paths tolerate a call
 * when nothing was suppressed (hand_collision_suppression_math::hasActive gates
 * them). That is what lets abortGrabAcquisition run the complete teardown from
 * any failure exit. The delayed-restore path exists so a released object cannot
 * immediately re-collide with the hand that threw it.
 */

#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "RockConfig.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace rock
{
    void Hand::clearGrabHandCollisionSuppressionState()
    {
        hand_collision_suppression_math::clear(_grabHandCollisionSuppression);
        hand_collision_suppression_math::clear(_grabHandCollisionDelayedRestore);
    }
    
    void Hand::clearHeldLooseWeaponBodyCollisionSuppressionState()
    {
        hand_collision_suppression_math::clear(_heldLooseWeaponBodyCollisionSuppression);
    }
    
    void Hand::suppressHandCollisionForGrab(RE::hknpWorld* world, const BodyBoneColliderSet* bodyBoneColliders)
    {
        /*
         * Normal held-object grabs suppress the grabbing hand's immediate collision
         * authority while the object is constrained to that hand. The generated
         * hand suite covers palm/fingers, and BodyBoneColliderSet owns the adjacent
         * same-side forearm/wrist chain. Leasing both sets prevents held objects
         * from solving against their own driving arm without touching the separate
         * two-handed equipped-weapon suppression path.
         */
        hand_collision_suppression_math::clear(_grabHandCollisionDelayedRestore);
    
        if (!world || !hasCollisionBody())
            return;
    
        auto suppressBody = [&](std::uint32_t bodyId, const char* context) {
            if (bodyId == INVALID_BODY_ID) {
                return;
            }
    
            std::uint32_t currentFilter = 0;
            if (!body_collision::tryReadFilterInfo(world, RE::hknpBodyId{ bodyId }, currentFilter)) {
                return;
            }
    
            const auto suppression = hand_collision_suppression_math::beginSuppression(_grabHandCollisionSuppression, bodyId, currentFilter);
            if (!suppression.stored) {
                ROCK_LOG_WARN(Hand,
                    "{} hand: grab collision suppression set full; bodyId={} context={} left active",
                    handName(),
                    bodyId,
                    context ? context : "unknown");
                return;
            }
    
            const auto registryResult = collision_suppression_registry::globalCollisionSuppressionRegistry().acquire(
                world,
                bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::Grab,
                context);
    
            if (registryResult.valid && (registryResult.filterChanged || registryResult.firstLeaseForBody)) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand: grab collision lease acquired bodyId={} context={} filter=0x{:08X}->0x{:08X} wasDisabledBeforeGrab={} leases={}",
                    handName(),
                    bodyId,
                    context ? context : "unknown",
                    registryResult.filterBefore,
                    registryResult.filterAfter,
                    registryResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    registryResult.activeLeaseCount);
            }
        };
    
        const std::uint32_t colliderCount = _boneColliders.getBodyCount();
        if (colliderCount > 0) {
            for (std::uint32_t i = 0; i < colliderCount; ++i) {
                suppressBody(_boneColliders.getBodyIdAtomic(i), "held-grab-hand-suite");
            }
        } else {
            suppressBody(_handBody.getBodyId().value, "held-grab-hand-anchor");
        }
    
        if (bodyBoneColliders) {
            std::array<std::uint32_t, kGrabCollisionSuppressionArmBodyCountPerHand> armBodyIds{};
            const auto armBodyCount =
                bodyBoneColliders->copyGrabSuppressionArmBodyIdsAtomic(_isLeft, armBodyIds.data(), armBodyIds.size());
            for (std::uint32_t i = 0; i < armBodyCount && i < armBodyIds.size(); ++i) {
                suppressBody(armBodyIds[i], "held-grab-arm-chain");
            }
        }
    }
    
    void Hand::restoreHandCollisionAfterGrab(RE::hknpWorld* world)
    {
        if (!hand_collision_suppression_math::hasActive(_grabHandCollisionSuppression))
            return;
    
        if (!world) {
            ROCK_LOG_WARN(Hand,
                "{} hand: cannot restore grab hand collision yet (world={}); preserving suppression state",
                handName(),
                static_cast<const void*>(world));
            return;
        }
    
        bool restoreDeferred = false;
        for (const auto& entry : _grabHandCollisionSuppression.entries) {
            if (!entry.active || entry.bodyId == INVALID_BODY_ID) {
                continue;
            }
    
            const auto releaseResult = collision_suppression_registry::globalCollisionSuppressionRegistry().release(
                world,
                entry.bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::Grab,
                "held-grab-hand");
            if (releaseResult.readFailed) {
                restoreDeferred = true;
                continue;
            }
    
            ROCK_LOG_DEBUG(Hand,
                "{} hand: grab hand collision lease released bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} fullyReleased={}",
                handName(),
                entry.bodyId,
                releaseResult.filterBefore,
                releaseResult.filterAfter,
                releaseResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                releaseResult.bodyFullyReleased ? "yes" : "no");
        }
        if (restoreDeferred) {
            ROCK_LOG_WARN(Hand, "{} hand: grab hand collision restore deferred; suppression leases preserved", handName());
            return;
        }
        clearGrabHandCollisionSuppressionState();
    }
    
    void Hand::suppressBodyCollisionForHeldLooseWeapon(RE::hknpWorld* world, const BodyBoneColliderSet* bodyBoneColliders)
    {
        if (!world || !bodyBoneColliders || !bodyBoneColliders->hasBodies()) {
            return;
        }
    
        auto keepBodyColliderEnabled = [&](const BodyBoneColliderMetadata& metadata) {
            const auto otherSide = _isLeft ? body_zone::BodyZoneSide::Right : body_zone::BodyZoneSide::Left;
            if (metadata.side != otherSide) {
                return false;
            }
            return metadata.role == skeleton_bone_debug_math::BoneColliderRole::ForearmSegment ||
                   metadata.role == skeleton_bone_debug_math::BoneColliderRole::HandSegment;
        };
    
        auto roleName = [](skeleton_bone_debug_math::BoneColliderRole role) {
            using skeleton_bone_debug_math::BoneColliderRole;
            switch (role) {
            case BoneColliderRole::UpperArmSegment:
                return "UpperArmSegment";
            case BoneColliderRole::ForearmSegment:
                return "ForearmSegment";
            case BoneColliderRole::HandSegment:
                return "HandSegment";
            case BoneColliderRole::FingerSegment:
                return "FingerSegment";
            case BoneColliderRole::TorsoSegment:
                return "TorsoSegment";
            case BoneColliderRole::LegSegment:
                return "LegSegment";
            case BoneColliderRole::FootSegment:
                return "FootSegment";
            }
            return "Unknown";
        };
    
        auto suppressBody = [&](std::uint32_t bodyId) {
            if (bodyId == INVALID_BODY_ID) {
                return;
            }
    
            BodyBoneColliderMetadata metadata{};
            if (!bodyBoneColliders->tryGetBodyMetadataAtomic(bodyId, metadata) || keepBodyColliderEnabled(metadata)) {
                return;
            }
    
            std::uint32_t currentFilter = 0;
            if (!body_collision::tryReadFilterInfo(world, RE::hknpBodyId{ bodyId }, currentFilter)) {
                return;
            }
    
            const auto suppression = hand_collision_suppression_math::beginSuppression(_heldLooseWeaponBodyCollisionSuppression, bodyId, currentFilter);
            if (!suppression.stored) {
                ROCK_LOG_WARN(Hand,
                    "{} hand: held loose weapon body suppression set full; bodyId={} role={} zone={} side={} left active",
                    handName(),
                    bodyId,
                    roleName(metadata.role),
                    body_zone::bodyZoneName(metadata.zone),
                    body_zone::bodyZoneSideName(metadata.side));
                return;
            }
    
            const auto registryResult = collision_suppression_registry::globalCollisionSuppressionRegistry().acquire(
                world,
                bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::HeldLooseWeaponBody,
                "held-loose-weapon-body");
    
            if (registryResult.valid && (registryResult.filterChanged || registryResult.firstLeaseForBody)) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand: held loose weapon body collision lease acquired bodyId={} role={} zone={} side={} filter=0x{:08X}->0x{:08X} wasDisabledBefore={} leases={}",
                    handName(),
                    bodyId,
                    roleName(metadata.role),
                    body_zone::bodyZoneName(metadata.zone),
                    body_zone::bodyZoneSideName(metadata.side),
                    registryResult.filterBefore,
                    registryResult.filterAfter,
                    registryResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    registryResult.activeLeaseCount);
            }
        };
    
        const std::uint32_t bodyCount = bodyBoneColliders->getBodyCount();
        for (std::uint32_t i = 0; i < bodyCount; ++i) {
            suppressBody(bodyBoneColliders->getBodyIdAtomic(i));
        }
    }
    
    void Hand::restoreBodyCollisionAfterHeldLooseWeapon(RE::hknpWorld* world)
    {
        if (!hand_collision_suppression_math::hasActive(_heldLooseWeaponBodyCollisionSuppression)) {
            return;
        }
    
        if (!world) {
            ROCK_LOG_WARN(Hand,
                "{} hand: cannot restore held loose weapon body collision yet (world={}); preserving suppression state",
                handName(),
                static_cast<const void*>(world));
            return;
        }
    
        bool restoreDeferred = false;
        for (const auto& entry : _heldLooseWeaponBodyCollisionSuppression.entries) {
            if (!entry.active || entry.bodyId == INVALID_BODY_ID) {
                continue;
            }
    
            const auto releaseResult = collision_suppression_registry::globalCollisionSuppressionRegistry().release(
                world,
                entry.bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::HeldLooseWeaponBody,
                "held-loose-weapon-body");
            if (releaseResult.readFailed) {
                restoreDeferred = true;
                continue;
            }
    
            ROCK_LOG_DEBUG(Hand,
                "{} hand: held loose weapon body collision lease released bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} fullyReleased={}",
                handName(),
                entry.bodyId,
                releaseResult.filterBefore,
                releaseResult.filterAfter,
                releaseResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                releaseResult.bodyFullyReleased ? "yes" : "no");
        }
    
        if (restoreDeferred) {
            ROCK_LOG_WARN(Hand, "{} hand: held loose weapon body collision restore deferred; suppression leases preserved", handName());
            return;
        }
    
        clearHeldLooseWeaponBodyCollisionSuppressionState();
    }
    
    void Hand::updateDelayedGrabHandCollisionRestore(RE::hknpWorld* world, float deltaTime)
    {
        if (!hand_collision_suppression_math::advanceDelayedRestore(_grabHandCollisionDelayedRestore, _grabHandCollisionSuppression, deltaTime)) {
            return;
        }
    
        ROCK_LOG_DEBUG(Hand,
            "{} hand: delayed grab hand collision restore ready bodies={} firstBodyId={} delayRemaining={:.3f}",
            handName(),
            _grabHandCollisionDelayedRestore.bodyCount,
            _grabHandCollisionDelayedRestore.bodyId,
            _grabHandCollisionDelayedRestore.remainingSeconds);
        restoreHandCollisionAfterGrab(world);
    }
}

