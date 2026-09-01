#include "physics-interaction/core/PhysicsInteractionInternal.h"

// Hand and native-player collision suppression: contact evidence ownership, dominant-weapon and weapon-support hand suppression, post-drop suppression, and native player body suppression.

namespace rock
{
    void PhysicsInteraction::clearLeftWeaponContact()
    {
        _weaponContact.left.bodyId.store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
        _weaponContact.left.partKind.store(static_cast<std::uint32_t>(WeaponPartKind::Other), std::memory_order_release);
        _weaponContact.left.reloadRole.store(static_cast<std::uint32_t>(WeaponReloadRole::None), std::memory_order_release);
        _weaponContact.left.supportRole.store(static_cast<std::uint32_t>(WeaponSupportGripRole::None), std::memory_order_release);
        _weaponContact.left.socketRole.store(static_cast<std::uint32_t>(WeaponSocketRole::None), std::memory_order_release);
        _weaponContact.left.actionRole.store(static_cast<std::uint32_t>(WeaponActionRole::None), std::memory_order_release);
        _weaponContact.left.gripPose.store(static_cast<std::uint32_t>(WeaponGripPoseId::None), std::memory_order_release);
        _weaponContact.left.missedFrames.store(WEAPON_CONTACT_TIMEOUT_FRAMES + 1, std::memory_order_release);
        _equipped.weaponInteractionAcquisitionStates[0] = {};
    }

    void PhysicsInteraction::clearRightWeaponContact()
    {
        _weaponContact.right.bodyId.store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
        _weaponContact.right.partKind.store(static_cast<std::uint32_t>(WeaponPartKind::Other), std::memory_order_release);
        _weaponContact.right.reloadRole.store(static_cast<std::uint32_t>(WeaponReloadRole::None), std::memory_order_release);
        _weaponContact.right.supportRole.store(static_cast<std::uint32_t>(WeaponSupportGripRole::None), std::memory_order_release);
        _weaponContact.right.socketRole.store(static_cast<std::uint32_t>(WeaponSocketRole::None), std::memory_order_release);
        _weaponContact.right.actionRole.store(static_cast<std::uint32_t>(WeaponActionRole::None), std::memory_order_release);
        _weaponContact.right.gripPose.store(static_cast<std::uint32_t>(WeaponGripPoseId::None), std::memory_order_release);
        _weaponContact.right.missedFrames.store(WEAPON_CONTACT_TIMEOUT_FRAMES + 1, std::memory_order_release);
        _equipped.weaponInteractionAcquisitionStates[1] = {};
    }

    bool PhysicsInteraction::isHandContactEvidenceSuppressed(bool isLeft) const
    {
        /*
         * Native hknp contact callbacks can run on the physics boundary while
         * game-frame ownership is changing. Use only atomic state here: the
         * physics thread needs ROCK's "hand collision disabled while owned"
         * answer without reading Hand::_state directly.
         */
        const Hand& hand = isLeft ? _leftHand : _rightHand;
        return hand.hasContactEvidenceSuppressedAtomic() ||
               (!isLeft && _suppression.rightDominantSuppressed.load(std::memory_order_acquire)) ||
               (!isLeft && _suppression.rightWeaponSupportSuppressed.load(std::memory_order_acquire)) ||
               (isLeft && _suppression.leftWeaponSupportSuppressed.load(std::memory_order_acquire));
    }

    void PhysicsInteraction::clearContactEvidenceForHand(bool isLeft)
    {
        if (isLeft) {
            _leftHand.clearSemanticContactEvidence();
        } else {
            _rightHand.clearSemanticContactEvidence();
        }
    }

    void PhysicsInteraction::synchronizeContactEvidenceOwnership(bool rightHandWeaponAuthorityActive, bool leftSupportGripActive, bool rightPartGripActive)
    {
        /*
         * ROCK disables generated hand collision when a grab or two-hand/tool
         * owner has the hand. Clear semantic contact state at the same
         * authority transition so callbacks cannot leave a stale touch owner.
         */
        if (_rightHand.hasContactEvidenceSuppressedAtomic() || rightHandWeaponAuthorityActive || rightPartGripActive ||
            _suppression.rightDominantSuppressed.load(std::memory_order_acquire) ||
            _suppression.rightWeaponSupportSuppressed.load(std::memory_order_acquire)) {
            clearContactEvidenceForHand(false);
        }

        if (_leftHand.hasContactEvidenceSuppressedAtomic() || leftSupportGripActive ||
            _suppression.leftWeaponSupportSuppressed.load(std::memory_order_acquire)) {
            clearContactEvidenceForHand(true);
        }
    }

    void PhysicsInteraction::suppressRightHandCollisionForDominantWeapon(RE::hknpWorld* world)
    {
        /*
         * The equipped gun already owns the dominant-hand pose and weapon aim.
         * Letting the generated right-hand bodies keep colliding while that
         * authority is active creates a second physical owner: stale hand
         * contacts can push props or feed semantic touch. ROCK treats owned
         * tool states as collision-filter ownership, so it uses the shared
         * suppression lease here instead of a visual-only gate.
         */
        _suppression.rightDominantSuppressed.store(true, std::memory_order_release);

        if (!world) {
            return;
        }
        releaseStaleGeneratedHandSuppressionLeases(
            world,
            _rightHand,
            _suppression.rightDominantLeases,
            "dominant-weapon-hand-stale");
        if (!_rightHand.hasCollisionBody()) {
            return;
        }

        auto suppressBody = [&](std::uint32_t bodyId) {
            if (bodyId == INVALID_CONTACT_BODY_ID) {
                return;
            }

            if (!_suppression.rightDominantLeases.contains(bodyId) &&
                _suppression.rightDominantLeases.full()) {
                ROCK_LOG_WARN(Weapon, "DominantWeapon: right hand suppression set full; bodyId={} left active", bodyId);
                return;
            }

            const auto registryResult = _suppression.rightDominantLeases.acquire(
                world,
                bodyId,
                "dominant-weapon-hand");

            if (registryResult.valid && (registryResult.firstLeaseForBody || registryResult.filterChanged)) {
                ROCK_LOG_DEBUG(Weapon,
                    "DominantWeapon: right hand collision lease acquired bodyId={} filter=0x{:08X}->0x{:08X} wasDisabledBefore={} leases={}",
                    bodyId,
                    registryResult.filterBefore,
                    registryResult.filterAfter,
                    registryResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    registryResult.activeLeaseCount);
            }
        };

        const std::uint32_t colliderCount = _rightHand.getHandColliderBodyCount();
        if (colliderCount > 0) {
            for (std::uint32_t i = 0; i < colliderCount; ++i) {
                suppressBody(_rightHand.getHandColliderBodyIdAtomic(i));
            }
        } else {
            suppressBody(_rightHand.getCollisionBodyId().value);
        }
    }

    void PhysicsInteraction::restoreRightHandCollisionAfterDominantWeapon(RE::hknpWorld* world)
    {
        if (_suppression.rightDominantLeases.empty()) {
            _suppression.rightDominantSuppressed.store(false, std::memory_order_release);
            return;
        }

        if (!world) {
            ROCK_LOG_WARN(Weapon, "DominantWeapon: cannot restore right hand collision yet (world=null); preserving suppression leases");
            return;
        }

        const bool restored = _suppression.rightDominantLeases.releaseAll(
            world,
            "dominant-weapon-hand",
            [](std::uint32_t bodyId, const auto& releaseResult) {
                if (releaseResult.readFailed) {
                    return;
                }
                ROCK_LOG_DEBUG(Weapon,
                    "DominantWeapon: right hand collision lease released bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} fullyReleased={}",
                    bodyId,
                    releaseResult.filterBefore,
                    releaseResult.filterAfter,
                    releaseResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    releaseResult.bodyFullyReleased ? "yes" : "no");
            });

        if (!restored) {
            ROCK_LOG_WARN(Weapon, "DominantWeapon: right hand collision restore deferred; suppression leases preserved");
            return;
        }

        _suppression.rightDominantLeases.clearTracking();
        _suppression.rightDominantSuppressed.store(false, std::memory_order_release);
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
        auto& suppressionSet = isLeft ? _suppression.leftWeaponSupportLeases : _suppression.rightWeaponSupportLeases;
        auto& suppressedFlag = isLeft ? _suppression.leftWeaponSupportSuppressed : _suppression.rightWeaponSupportSuppressed;
        suppressionSet.cancelDelayedRestore();
        suppressedFlag.store(true, std::memory_order_release);

        if (!world) {
            return;
        }
        releaseStaleGeneratedHandSuppressionLeases(
            world,
            hand,
            suppressionSet,
            "weapon-support-hand-stale");

        auto bodyAlreadySuppressed = [&](std::uint32_t bodyId) {
            return bodyId == INVALID_CONTACT_BODY_ID ||
                   suppressionSet.contains(bodyId);
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

        if (!hand.hasCollisionBody()) {
            return;
        }

        auto suppressBody = [&](std::uint32_t bodyId) {
            if (bodyId == INVALID_CONTACT_BODY_ID) {
                return;
            }

            if (!suppressionSet.contains(bodyId) && suppressionSet.full()) {
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: {} hand support suppression set full; bodyId={} left active", isLeft ? "left" : "right", bodyId);
                return;
            }

            const auto registryResult = suppressionSet.acquire(
                world,
                bodyId,
                "weapon-support-hand");

            if (registryResult.valid && (registryResult.firstLeaseForBody || registryResult.filterChanged)) {
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: {} hand collision lease acquired bodyId={} filter=0x{:08X}->0x{:08X} wasDisabledBefore={} leases={}",
                    isLeft ? "left" : "right",
                    bodyId,
                    registryResult.filterBefore,
                    registryResult.filterAfter,
                    registryResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    registryResult.activeLeaseCount);
            }
        };

        const std::uint32_t colliderCount = hand.getHandColliderBodyCount();
        if (colliderCount > 0) {
            for (std::uint32_t i = 0; i < colliderCount; ++i) {
                suppressBody(hand.getHandColliderBodyIdAtomic(i));
            }
        } else {
            suppressBody(hand.getCollisionBodyId().value);
        }
    }

    void PhysicsInteraction::beginDelayedHandCollisionRestoreAfterWeaponSupport(
        RE::hknpWorld* world,
        const bool isLeft)
    {
        auto& suppressionSet = isLeft ? _suppression.leftWeaponSupportLeases : _suppression.rightWeaponSupportLeases;
        auto& suppressedFlag = isLeft ? _suppression.leftWeaponSupportSuppressed : _suppression.rightWeaponSupportSuppressed;
        if (suppressionSet.empty()) {
            suppressionSet.cancelDelayedRestore();
            suppressedFlag.store(false, std::memory_order_release);
            return;
        }

        if (suppressionSet.beginDelayedRestore(
                g_rockConfig.rockGrabReleaseHandCollisionDelaySeconds)) {
            suppressedFlag.store(true, std::memory_order_release);
            ROCK_LOG_DEBUG(
                Weapon,
                "TwoHandedGrip: {} hand support collision restore delayed bodies={} firstBodyId={} seconds={:.3f}",
                isLeft ? "left" : "right",
                suppressionSet.size(),
                suppressionSet.firstBodyId(),
                suppressionSet.delayedRestoreRemainingSeconds());
            return;
        }

        restoreHandCollisionAfterWeaponSupport(world, isLeft, true);
    }

    void PhysicsInteraction::restoreHandCollisionAfterWeaponSupport(
        RE::hknpWorld* world,
        const bool isLeft,
        const bool forceImmediate)
    {
        auto& suppressionSet = isLeft ? _suppression.leftWeaponSupportLeases : _suppression.rightWeaponSupportLeases;
        auto& suppressedFlag = isLeft ? _suppression.leftWeaponSupportSuppressed : _suppression.rightWeaponSupportSuppressed;
        if (suppressionSet.empty()) {
            suppressionSet.cancelDelayedRestore();
            suppressedFlag.store(false, std::memory_order_release);
            return;
        }

        if (suppressionSet.delayedRestorePending() && !forceImmediate) {
            suppressedFlag.store(true, std::memory_order_release);
            return;
        }

        if (forceImmediate) {
            suppressionSet.cancelDelayedRestore();
        }

        if (!world) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: cannot restore {} hand support collision yet (world=null); preserving suppression leases", isLeft ? "left" : "right");
            return;
        }

        const bool restored = suppressionSet.releaseAll(
            world,
            "weapon-support-hand",
            [isLeft](std::uint32_t bodyId, const auto& releaseResult) {
                if (releaseResult.readFailed) {
                    return;
                }
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: {} hand collision lease released bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} fullyReleased={}",
                    isLeft ? "left" : "right",
                    bodyId,
                    releaseResult.filterBefore,
                    releaseResult.filterAfter,
                    releaseResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    releaseResult.bodyFullyReleased ? "yes" : "no");
            });

        if (!restored) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: {} hand support collision restore deferred; suppression leases preserved", isLeft ? "left" : "right");
            return;
        }

        suppressionSet.clearTracking();
        suppressedFlag.store(false, std::memory_order_release);
    }

    void PhysicsInteraction::updateWeaponSupportCollisionSuppression(
        RE::hknpWorld* world,
        const float deltaSeconds)
    {
        auto updateHand = [&](const bool isLeft) {
            auto& suppressionSet = isLeft ? _suppression.leftWeaponSupportLeases : _suppression.rightWeaponSupportLeases;
            if (!suppressionSet.delayedRestorePending() ||
                !suppressionSet.advanceDelayedRestore(deltaSeconds)) {
                return;
            }

            restoreHandCollisionAfterWeaponSupport(world, isLeft, true);
        };

        updateHand(false);
        updateHand(true);
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
        auto& suppressionSet = isLeft ? _suppression.leftDropLeases : _suppression.rightDropLeases;
        auto& suppressed = isLeft ? _suppression.leftDropSuppressed : _suppression.rightDropSuppressed;
        suppressionSet.cancelDelayedRestore();

        if (!world || !hand.hasCollisionBody()) {
            return;
        }

        auto suppressBody = [&](std::uint32_t bodyId, const char* context) {
            if (bodyId == INVALID_CONTACT_BODY_ID) {
                return;
            }

            if (!suppressionSet.contains(bodyId) && suppressionSet.full()) {
                ROCK_LOG_WARN(Weapon,
                    "EquippedWeaponDrop: {} hand post-drop suppression set full; bodyId={} context={} left active",
                    equipped_weapon_drop_policy::sourceHandName(sourceHand),
                    bodyId,
                    context ? context : "unknown");
                return;
            }

            const auto registryResult = suppressionSet.acquire(
                world,
                bodyId,
                context);

            if (registryResult.valid && (registryResult.firstLeaseForBody || registryResult.filterChanged)) {
                ROCK_LOG_DEBUG(Weapon,
                    "EquippedWeaponDrop: {} hand post-drop collision lease acquired bodyId={} context={} filter=0x{:08X}->0x{:08X} wasDisabledBefore={} leases={}",
                    equipped_weapon_drop_policy::sourceHandName(sourceHand),
                    bodyId,
                    context ? context : "unknown",
                    registryResult.filterBefore,
                    registryResult.filterAfter,
                    registryResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    registryResult.activeLeaseCount);
            }
        };

        const std::uint32_t colliderCount = hand.getHandColliderBodyCount();
        if (colliderCount > 0) {
            for (std::uint32_t i = 0; i < colliderCount; ++i) {
                suppressBody(hand.getHandColliderBodyIdAtomic(i), "equipped-weapon-drop-hand-suite");
            }
        } else {
            suppressBody(hand.getCollisionBodyId().value, "equipped-weapon-drop-hand-anchor");
        }

        std::array<std::uint32_t, kGrabCollisionSuppressionArmBodyCountPerHand> armBodyIds{};
        const auto armBodyCount = _bodyBoneColliders.copyGrabSuppressionArmBodyIdsAtomic(isLeft, armBodyIds.data(), armBodyIds.size());
        for (std::uint32_t i = 0; i < armBodyCount && i < armBodyIds.size(); ++i) {
            suppressBody(armBodyIds[i], "equipped-weapon-drop-arm-chain");
        }

        const bool hasSuppression = !suppressionSet.empty();
        suppressed.store(hasSuppression, std::memory_order_release);
        if (!hasSuppression) {
            return;
        }

        if (suppressionSet.beginDelayedRestore(
                g_rockConfig.rockGrabReleaseHandCollisionDelaySeconds)) {
            ROCK_LOG_DEBUG(Weapon,
                "EquippedWeaponDrop: {} hand post-drop collision restore delayed bodies={} firstBodyId={} seconds={:.3f}",
                equipped_weapon_drop_policy::sourceHandName(sourceHand),
                suppressionSet.size(),
                suppressionSet.firstBodyId(),
                suppressionSet.delayedRestoreRemainingSeconds());
        } else {
            restoreHandCollisionAfterEquippedWeaponDrop(world, isLeft);
        }
    }

    void PhysicsInteraction::restoreHandCollisionAfterEquippedWeaponDrop(RE::hknpWorld* world, bool isLeft)
    {
        auto& suppressionSet = isLeft ? _suppression.leftDropLeases : _suppression.rightDropLeases;
        auto& suppressed = isLeft ? _suppression.leftDropSuppressed : _suppression.rightDropSuppressed;

        if (suppressionSet.empty()) {
            suppressionSet.cancelDelayedRestore();
            suppressed.store(false, std::memory_order_release);
            return;
        }

        if (!world) {
            ROCK_LOG_WARN(Weapon,
                "EquippedWeaponDrop: cannot restore {} hand post-drop collision yet (world=null); preserving suppression leases",
                isLeft ? "left" : "right");
            return;
        }

        const bool restored = suppressionSet.releaseAll(
            world,
            "equipped-weapon-drop-hand",
            [isLeft](std::uint32_t bodyId, const auto& releaseResult) {
                if (releaseResult.readFailed) {
                    return;
                }
                ROCK_LOG_DEBUG(Weapon,
                    "EquippedWeaponDrop: {} hand post-drop collision lease released bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} fullyReleased={}",
                    isLeft ? "left" : "right",
                    bodyId,
                    releaseResult.filterBefore,
                    releaseResult.filterAfter,
                    releaseResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    releaseResult.bodyFullyReleased ? "yes" : "no");
            });

        if (!restored) {
            ROCK_LOG_WARN(Weapon,
                "EquippedWeaponDrop: {} hand post-drop collision restore deferred; suppression leases preserved",
                isLeft ? "left" : "right");
            return;
        }

        suppressionSet.clearTracking();
        suppressed.store(false, std::memory_order_release);
    }

    void PhysicsInteraction::updateEquippedWeaponPostDropCollisionSuppression(RE::hknpWorld* world, float deltaSeconds)
    {
        auto updateHand = [&](bool isLeft) {
            auto& suppressionSet = isLeft ? _suppression.leftDropLeases : _suppression.rightDropLeases;
            auto& suppressed = isLeft ? _suppression.leftDropSuppressed : _suppression.rightDropSuppressed;

            if (suppressionSet.delayedRestorePending() &&
                !suppressionSet.advanceDelayedRestore(deltaSeconds)) {
                return;
            }

            if (!suppressionSet.empty()) {
                restoreHandCollisionAfterEquippedWeaponDrop(world, isLeft);
                return;
            }

            suppressionSet.cancelDelayedRestore();
            suppressed.store(false, std::memory_order_release);
        };

        updateHand(false);
        updateHand(true);
    }

    void PhysicsInteraction::clearEquippedWeaponPostDropCollisionSuppressionState()
    {
        _suppression.rightDropLeases.clearTracking();
        _suppression.leftDropLeases.clearTracking();
        _suppression.rightDropSuppressed.store(false, std::memory_order_release);
        _suppression.leftDropSuppressed.store(false, std::memory_order_release);
    }

    void PhysicsInteraction::updateHandCollisions(const PhysicsFrameContext& frame)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::HandColliderUpdate);

        if (!runtime_state::isLocalSkeletonReady()) {
            return;
        }

        auto* world = frame.hknpWorld;

        if (!_rightHand.hasCollisionBody() || !_leftHand.hasCollisionBody()) {
            if (frame.reloadBoundaryActive) {
                return;
            }
            if (_lifecycle.handColliderCreateRetryFrames > 0) {
                --_lifecycle.handColliderCreateRetryFrames;
                return;
            }

            ROCK_LOG_WARN(Hand,
                "Hand collider runtime missing generated bodies; recreating rightBody={} leftBody={}",
                _rightHand.hasCollisionBody() ? "yes" : "no",
                _leftHand.hasCollisionBody() ? "yes" : "no");
            destroyHandCollisions(frame.bhkWorld);
            if (!createHandCollisions(frame.hknpWorld, frame.bhkWorld)) {
                _lifecycle.handColliderCreateRetryFrames = 120;
            }
            return;
        }

        _rightHand.updateDelayedGrabHandCollisionRestore(world, frame.deltaSeconds);
        _leftHand.updateDelayedGrabHandCollisionRestore(world, frame.deltaSeconds);
        updateWeaponSupportCollisionSuppression(world, frame.deltaSeconds);
        updateEquippedWeaponPostDropCollisionSuppression(world, frame.deltaSeconds);

        if (!frame.right.disabled) {
            _rightHand.updateCollisionTransform(world, frame.right.rawHandWorld, frame.deltaSeconds);
        }
        if (!frame.left.disabled) {
            _leftHand.updateCollisionTransform(world, frame.left.rawHandWorld, frame.deltaSeconds);
        }
    }

    void PhysicsInteraction::updateBodyBoneCollisions(const PhysicsFrameContext& frame)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::BodyColliderUpdate);

        if (!runtime_state::isLocalSkeletonReady()) {
            return;
        }

        if (!g_rockConfig.rockBodyBoneCollidersEnabled) {
            if (_bodyBoneColliders.hasBodies()) {
                ROCK_LOG_INFO(Body, "Body bone collider config disabled; destroying generated body set");
                destroyBodyBoneCollisions(frame.bhkWorld);
                _contacts.bodyRuntime.reset();
            }
            return;
        }

        if (!_bodyBoneColliders.hasBodies()) {
            if (frame.reloadBoundaryActive) {
                return;
            }
            if (_lifecycle.bodyBoneColliderCreateRetryFrames > 0) {
                --_lifecycle.bodyBoneColliderCreateRetryFrames;
                return;
            }

            if (!createBodyBoneCollisions(frame.hknpWorld, frame.bhkWorld)) {
                _lifecycle.bodyBoneColliderCreateRetryFrames = 120;
            }
            return;
        }

        _bodyBoneColliders.update(frame.hknpWorld, frame.deltaSeconds);
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

    void PhysicsInteraction::restoreNativePlayerCollisionSuppression(RE::hknpWorld* hknp, const char* reason)
    {
        if (_suppression.nativePlayerBodyCount == 0) {
            _suppression.nativePlayerRefreshFrames = 0;
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

        for (std::uint32_t i = 0; i < _suppression.nativePlayerBodyCount && i < _suppression.nativePlayerBodies.size(); ++i) {
            const auto& body = _suppression.nativePlayerBodies[i];
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

        _suppression.nativePlayerBodies = pending;
        _suppression.nativePlayerBodyCount = pendingCount;
        _suppression.nativePlayerRefreshFrames = pendingCount == 0 ? 0 : 30;
    }

    void PhysicsInteraction::refreshNativePlayerCollisionSuppression(RE::hknpWorld* hknp, const char* context)
    {
        if (!g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled || !hknp || _suppression.nativePlayerBodyCount == 0) {
            return;
        }

        std::uint32_t retainedCount = 0;
        for (std::uint32_t i = 0; i < _suppression.nativePlayerBodyCount && i < _suppression.nativePlayerBodies.size(); ++i) {
            const auto body = _suppression.nativePlayerBodies[i];
            if (!contact_pipeline_policy::isValidBodyId(body.bodyId)) {
                continue;
            }

            const auto refreshResult = collision_suppression_registry::globalCollisionSuppressionRegistry().refresh(
                hknp,
                body.bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::NativePlayerBody,
                context ? context : "native-player-body-refresh");
            if (refreshResult.readFailed || (refreshResult.valid && !refreshResult.staleLeaseDiscarded)) {
                _suppression.nativePlayerBodies[retainedCount++] = body;
            }
        }

        for (std::uint32_t i = retainedCount; i < _suppression.nativePlayerBodies.size(); ++i) {
            _suppression.nativePlayerBodies[i] = {};
        }
        _suppression.nativePlayerBodyCount = retainedCount;
    }

    void PhysicsInteraction::refreshNativePlayerCollisionSuppressionFromPhysicsSubstep(RE::hknpWorld* hknp, const char* context)
    {
        if (!g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled || !hknp || _suppression.nativePlayerBodyCount == 0) {
            return;
        }

        for (std::uint32_t i = 0; i < _suppression.nativePlayerBodyCount && i < _suppression.nativePlayerBodies.size(); ++i) {
            const auto& leasedBody = _suppression.nativePlayerBodies[i];
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

        if (_suppression.nativePlayerRefreshFrames > 0) {
            --_suppression.nativePlayerRefreshFrames;
            return;
        }
        _suppression.nativePlayerRefreshFrames = 30;

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

        if (scanContext.overflow && !_suppression.nativePlayerOverflowLogged) {
            _suppression.nativePlayerOverflowLogged = true;
            ROCK_LOG_WARN(Hand,
                "Native player collision suppression body capacity exceeded; keeping first {} bodies",
                kNativePlayerCollisionSuppressionBodyCapacity);
        } else if (!scanContext.overflow) {
            _suppression.nativePlayerOverflowLogged = false;
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

        for (std::uint32_t i = 0; i < _suppression.nativePlayerBodyCount && i < _suppression.nativePlayerBodies.size(); ++i) {
            const auto& body = _suppression.nativePlayerBodies[i];
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

        _suppression.nativePlayerBodies = next;
        _suppression.nativePlayerBodyCount = nextCount;
    }
}
