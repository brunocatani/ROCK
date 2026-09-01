#include "physics-interaction/core/PhysicsInteractionInternal.h"

// Equipped weapon release capture and drop momentum handoff.

namespace rock
{
    void PhysicsInteraction::updateEquippedWeaponReleaseCapture(const PhysicsFrameContext& frame, RE::NiNode* weaponNode)
    {
        auto& capture = _equippedWeaponReleaseCapture;
        if (!_twoHandedGrip.isManualOwnershipActive()) {
            capture = {};
            return;
        }

        /*
         * Prefer the transform ROCK published this frame (part-carry and
         * two-handed solves own the weapon node); the live node world is the
         * FRIK/game-final pose otherwise (primary-only carry).
         */
        RE::NiTransform solvedWeaponWorld{};
        if (_twoHandedGrip.getSolvedWeaponTransform(solvedWeaponWorld) && finiteNiTransform(solvedWeaponWorld)) {
            capture.weaponWorld = solvedWeaponWorld;
            capture.hasWeaponWorld = true;
        } else if (weaponNode && finiteNiTransform(weaponNode->world)) {
            capture.weaponWorld = weaponNode->world;
            capture.hasWeaponWorld = true;
        }

        const bool usableDeltaTime = std::isfinite(frame.deltaSeconds) && frame.deltaSeconds > 0.000001f;

        for (std::size_t handIndex = 0; handIndex < 2; ++handIndex) {
            const auto& handInput = handIndex == 1 ? frame.left : frame.right;
            auto& history = capture.handHistories[handIndex];
            if (!finiteNiTransform(handInput.rawHandWorld)) {
                continue;
            }
            if (capture.hasPreviousHandWorld[handIndex] && usableDeltaTime) {
                const RE::NiPoint3 deltaGameUnits = handInput.rawHandWorld.translate - capture.previousHandWorld[handIndex].translate;
                const RE::NiPoint3 rawHandVelocityHavok = held_object_physics_math::gameUnitsDeltaToHavokVelocity(
                    deltaGameUnits,
                    frame.deltaSeconds,
                    physics_scale::havokToGame());
                const RE::NiPoint3 angularVelocity = held_object_physics_math::angularVelocityFromRotationDelta<RE::NiMatrix3, RE::NiPoint3>(
                    capture.previousHandWorld[handIndex].rotate,
                    handInput.rawHandWorld.rotate,
                    frame.deltaSeconds);
                history.push(rawHandVelocityHavok, angularVelocity);
            }
            capture.previousHandWorld[handIndex] = handInput.rawHandWorld;
            capture.hasPreviousHandWorld[handIndex] = true;
        }
    }

    bool PhysicsInteraction::hasAvailableEquippedWeaponDropHandoff() const
    {
        return std::any_of(
            _equippedWeaponDropMomentumHandoffs.begin(),
            _equippedWeaponDropMomentumHandoffs.end(),
            [](const EquippedWeaponDropMomentumHandoff& handoff) { return !handoff.active; });
    }

    void PhysicsInteraction::armEquippedWeaponDropMomentumHandoff(
        const RE::ObjectRefHandle& handle,
        std::uint32_t droppedFormId,
        equipped_weapon_drop_policy::SourceHand sourceHand,
        const WeaponCollision::ReleaseGeometrySnapshot& releaseGeometry)
    {
        if (!handle) {
            return;
        }
        if (!releaseGeometry.hasCapturedWeaponWorld || !finiteNiTransform(releaseGeometry.capturedWeaponWorld)) {
            ROCK_LOG_ERROR(Weapon,
                "Equipped weapon drop handoff rejected because the frozen release pose is invalid: dropped={:08X}",
                droppedFormId);
            return;
        }
        EquippedWeaponDropMomentumHandoff* handoff = nullptr;
        for (auto& candidate : _equippedWeaponDropMomentumHandoffs) {
            if (!candidate.active) {
                handoff = &candidate;
                break;
            }
        }
        if (!handoff) {
            ROCK_LOG_WARN(Weapon,
                "Equipped weapon drop handoff capacity exhausted after admission: dropped={:08X} capacity={}",
                droppedFormId,
                _equippedWeaponDropMomentumHandoffs.size());
            return;
        }

        // Unknown source (SourceHand::None) falls back to the right hand.
        const auto& history = _equippedWeaponReleaseCapture.handHistories[equipped_weapon_drop_policy::isLeft(sourceHand) ? 1u : 0u];
        // Histories are world-space hand velocities; no player-space addend exists anymore.
        const auto release = equipped_weapon_drop_momentum::composeReleaseVelocity(
            history,
            RE::NiPoint3{},
            equipped_weapon_drop_momentum::ReleaseVelocitySettings{
                .controllerDerivedEnabled = g_rockConfig.rockGrabControllerDerivedThrowVelocityEnabled,
                .throwMultiplier = g_rockConfig.rockThrowVelocityMultiplier,
                .maxLinearVelocityHavok = g_rockConfig.rockGrabThrowMaxVelocityHavok,
                .angularVelocityScale = g_rockConfig.rockGrabThrowAngularVelocityScale,
                .maxAngularVelocityRadiansPerSecond = g_rockConfig.rockGrabThrowMaxAngularVelocityRadiansPerSecond,
                .longObjectAngularScalingEnabled = g_rockConfig.rockGrabLongObjectAngularScalingEnabled,
                .longObjectLeverGameUnits = releaseGeometry.leverGameUnits,
                .longObjectReferenceLeverGameUnits = g_rockConfig.rockGrabLongObjectReferenceLeverGameUnits,
                .longObjectMinAngularScale = g_rockConfig.rockGrabLongObjectMinAngularScale,
            });

        *handoff = EquippedWeaponDropMomentumHandoff{
            .active = true,
            .hasReleaseVelocity = release.hasData,
            .handle = handle,
            .droppedFormId = droppedFormId,
            .linearVelocityHavok = release.linearVelocityHavok,
            .angularVelocityRadiansPerSecond = release.angularVelocityRadiansPerSecond,
            .hasReleaseWeaponWorld = releaseGeometry.hasCapturedWeaponWorld,
            .releaseWeaponWorld = releaseGeometry.capturedWeaponWorld,
            .progressSolveSequence = _completedPhysicsSolveSequence.load(std::memory_order_acquire),
        };
        ROCK_LOG_INFO(Weapon,
            "Equipped weapon drop handoff armed: dropped={:08X} sourceHand={} velocity={} lever={:.1f}gu angularScale={:.3f} angularCap={:.3f} "
            "linear=({:.3f},{:.3f},{:.3f}) angular=({:.3f},{:.3f},{:.3f})",
            droppedFormId,
            equipped_weapon_drop_policy::sourceHandName(sourceHand),
            release.hasData ? "captured" : "none",
            releaseGeometry.leverGameUnits,
            release.longObjectAngularScale,
            release.angularVelocityCapRadiansPerSecond,
            release.linearVelocityHavok.x,
            release.linearVelocityHavok.y,
            release.linearVelocityHavok.z,
            release.angularVelocityRadiansPerSecond.x,
            release.angularVelocityRadiansPerSecond.y,
            release.angularVelocityRadiansPerSecond.z);
    }

    void PhysicsInteraction::serviceEquippedWeaponDropMomentumHandoff(const PhysicsFrameContext& frame)
    {
        for (auto& handoff : _equippedWeaponDropMomentumHandoffs) {
            if (handoff.active) {
                serviceEquippedWeaponDropMomentumTransaction(handoff, frame);
            }
        }
    }

    void PhysicsInteraction::serviceEquippedWeaponDropMomentumTransaction(
        EquippedWeaponDropMomentumHandoff& handoff,
        const PhysicsFrameContext& frame)
    {
        // This bound applies only while an asynchronously published drop has
        // made no state progress. Paused physics does not consume the budget.
        constexpr std::uint64_t kPublicationStallSolveSteps = 180;

        if (!handoff.active || !frame.worldReady || !frame.hknpWorld) {
            return;
        }

        handoff.elapsedSeconds += (std::max)(0.0f, frame.deltaSeconds);
        const std::uint64_t completedSolveSequence =
            _completedPhysicsSolveSequence.load(std::memory_order_acquire);
        const auto publicationStalled = [&]() {
            return equipped_weapon_drop_momentum::publicationProgressStalled(
                handoff.progressSolveSequence,
                completedSolveSequence,
                kPublicationStallSolveSteps);
        };
        const auto endHandoff = [&](const char* reason, bool warn) {
            if (warn) {
                ROCK_LOG_WARN(Weapon,
                    "Equipped weapon drop handoff ended: dropped={:08X} reason={} stage={} elapsed={:.3f}s solveProgress={}->{} restarts={}",
                    handoff.droppedFormId,
                    reason ? reason : "unknown",
                    static_cast<std::uint32_t>(handoff.stage),
                    handoff.elapsedSeconds,
                    handoff.progressSolveSequence,
                    completedSolveSequence,
                    handoff.identityRestartCount);
            } else {
                ROCK_LOG_DEBUG(Weapon,
                    "Equipped weapon drop handoff ended: dropped={:08X} reason={} stage={} elapsed={:.3f}s",
                    handoff.droppedFormId,
                    reason ? reason : "unknown",
                    static_cast<std::uint32_t>(handoff.stage),
                    handoff.elapsedSeconds);
            }
            handoff = {};
        };

        const auto droppedRefPtr = handoff.handle.get();
        auto* droppedRef = droppedRefPtr.get();
        if (!droppedRef) {
            if (handoff.referenceResolvedOnce) {
                endHandoff("reference-unloaded", false);
            } else if (!handoff.handle || publicationStalled()) {
                endHandoff("reference-publication-stalled", true);
            }
            return;
        }
        if (!handoff.referenceResolvedOnce) {
            handoff.referenceResolvedOnce = true;
            handoff.progressSolveSequence = completedSolveSequence;
        }
        if (droppedRef->IsDeleted() || droppedRef->IsDisabled()) {
            endHandoff("reference-left-world", false);
            return;
        }
        if (handoff.droppedFormId == 0) {
            handoff.droppedFormId = droppedRef->GetFormID();
        }

        auto* scanWorld = frame.bhkWorld;
        if (!scanWorld) {
            auto* cell = droppedRef->GetParentCell();
            scanWorld = cell ? cell->GetbhkWorld() : nullptr;
        }
        auto* droppedRoot = droppedRef->Get3D();
        if (!scanWorld || !droppedRoot) {
            if (handoff.threeDResolvedOnce) {
                endHandoff("physics-3d-unloaded", false);
            } else if (publicationStalled()) {
                endHandoff("physics-3d-publication-stalled", true);
            }
            return;
        }
        if (!handoff.threeDResolvedOnce) {
            handoff.threeDResolvedOnce = true;
            handoff.progressSolveSequence = completedSolveSequence;
        }
        if (!handoff.hasReleaseWeaponWorld ||
            !finiteNiTransform(handoff.releaseWeaponWorld) ||
            !finiteNiTransform(droppedRoot->world)) {
            endHandoff("invalid-release-or-root-pose", true);
            return;
        }

        if (handoff.stage == EquippedWeaponDropHandoffStage::ResolvingBodies) {
            const bool collisionPrepared =
                physics_recursive_wrappers::enableCollisionRecursive(droppedRoot, true, true, true);
            if (!collisionPrepared) {
                if (publicationStalled()) {
                    endHandoff("collision-enable-stalled", true);
                }
                return;
            }
        }

        object_physics_body_set::BodySetScanOptions scanOptions{};
        scanOptions.mode = physics_body_classifier::InteractionMode::ActiveGrab;
        scanOptions.targetKind = grab_target::Kind::LooseObject;
        scanOptions.requireSameResolvedRef = true;
        scanOptions.allowUnresolvedRefBodies = true;
        scanOptions.allowWeaponRefExpansion = true;
        scanOptions.maxDepth = g_rockConfig.rockObjectPhysicsTreeMaxDepth;

        // The general exact-reference scanner owns temporary containers. It is
        // deliberately confined to this short publication/one-solve handoff;
        // no scan remains active during normal weapon flight.
        const auto bodySet = object_physics_body_set::scanObjectPhysicsBodySet(
            scanWorld,
            frame.hknpWorld,
            droppedRef,
            scanOptions);
        const bool bodyScanComplete =
            bodySet.diagnostics.scanFailures == 0 &&
            bodySet.diagnostics.invalidPhysicsSystems == 0 &&
            bodySet.diagnostics.depthLimitSkips == 0;
        if (!bodyScanComplete) {
            ROCK_LOG_SAMPLE_WARN(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Equipped weapon drop exact-reference scan incomplete; retaining transaction: dropped={:08X} stage={} failures={} invalidSystems={} depthSkips={}",
                handoff.droppedFormId,
                static_cast<std::uint32_t>(handoff.stage),
                bodySet.diagnostics.scanFailures,
                bodySet.diagnostics.invalidPhysicsSystems,
                bodySet.diagnostics.depthLimitSkips);
            if (publicationStalled()) {
                endHandoff("exact-reference-scan-stalled", true);
            }
            return;
        }

        using BodyRecord = object_physics_body_set::ObjectPhysicsBodyRecord;
        std::array<const BodyRecord*, kEquippedWeaponDropBodySnapshotCapacity> acceptedRecords{};
        std::array<const BodyRecord*, kEquippedWeaponDropBodySnapshotCapacity> uniqueMotionRecords{};
        std::size_t acceptedRecordCount = 0;
        std::size_t uniqueMotionRecordCount = 0;
        bool bodyCollectionOverflow = false;
        for (const auto& record : bodySet.records) {
            if (!record.accepted || record.bodyId == object_physics_body_set::INVALID_BODY_ID) {
                continue;
            }
            if (acceptedRecordCount >= acceptedRecords.size()) {
                bodyCollectionOverflow = true;
                break;
            }
            acceptedRecords[acceptedRecordCount++] = &record;

            bool motionSeen = false;
            for (std::size_t i = 0; i < uniqueMotionRecordCount; ++i) {
                if (uniqueMotionRecords[i]->motionId == record.motionId) {
                    motionSeen = true;
                    break;
                }
            }
            if (!motionSeen) {
                if (uniqueMotionRecordCount >= uniqueMotionRecords.size()) {
                    bodyCollectionOverflow = true;
                    break;
                }
                uniqueMotionRecords[uniqueMotionRecordCount++] = &record;
            }
        }
        if (bodyCollectionOverflow) {
            ROCK_LOG_SAMPLE_WARN(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Equipped weapon drop native body set exceeds fixed transaction capacity: dropped={:08X} capacity={}",
                handoff.droppedFormId,
                acceptedRecords.size());
            if (publicationStalled()) {
                endHandoff("native-body-capacity-stalled", true);
            }
            return;
        }
        if (acceptedRecordCount == 0 || uniqueMotionRecordCount == 0) {
            if (publicationStalled()) {
                endHandoff("native-body-publication-stalled", true);
            }
            return;
        }

        auto tryReadBodyIdentity = [&](std::uint32_t bodyId,
                                       equipped_weapon_drop_momentum::BodyIdentityKey& outIdentity) {
            const auto live = havok_runtime::snapshotBody(
                frame.hknpWorld,
                RE::hknpBodyId{ bodyId });
            if (!live.valid || !live.body || !live.motion || !live.body->shape ||
                !live.collisionObject || !live.ownerNode ||
                live.body->bodyId.value != bodyId) {
                return false;
            }
            auto* physicsSystem =
                havok_runtime::getPhysicsSystemFromCollisionObject(live.collisionObject);
            auto* physicsSystemInstance =
                havok_runtime::getPhysicsSystemInstance(physicsSystem);
            if (!physicsSystemInstance) {
                return false;
            }
            outIdentity = equipped_weapon_drop_momentum::BodyIdentityKey{
                .bodyId = bodyId,
                .motionId = live.motionIndex,
                .motionFirstBodyId = live.motion->firstBodyId,
                .shapeIdentity = reinterpret_cast<std::uintptr_t>(live.body->shape),
                .owningNodeIdentity = reinterpret_cast<std::uintptr_t>(live.ownerNode),
                .collisionObjectIdentity = reinterpret_cast<std::uintptr_t>(live.collisionObject),
                .physicsSystemInstanceIdentity =
                    reinterpret_cast<std::uintptr_t>(physicsSystemInstance),
            };
            return true;
        };
        auto tryBuildBodyIdentity = [&](const BodyRecord& record,
                                        equipped_weapon_drop_momentum::BodyIdentityKey& outIdentity) {
            if (!tryReadBodyIdentity(record.bodyId, outIdentity)) {
                return false;
            }
            return outIdentity.motionId == record.motionId &&
                   outIdentity.collisionObjectIdentity ==
                       reinterpret_cast<std::uintptr_t>(record.collisionObject) &&
                   outIdentity.owningNodeIdentity ==
                       reinterpret_cast<std::uintptr_t>(record.owningNode);
        };
        auto currentBodySetMatches = [&]() {
            if (handoff.bodySnapshotCount == 0 ||
                handoff.bodySnapshotCount != acceptedRecordCount) {
                return false;
            }
            for (std::size_t recordIndex = 0;
                 recordIndex < acceptedRecordCount;
                 ++recordIndex) {
                equipped_weapon_drop_momentum::BodyIdentityKey currentIdentity{};
                if (!acceptedRecords[recordIndex] ||
                    !tryBuildBodyIdentity(*acceptedRecords[recordIndex], currentIdentity)) {
                    return false;
                }
                bool found = false;
                for (std::size_t snapshotIndex = 0;
                     snapshotIndex < handoff.bodySnapshotCount;
                     ++snapshotIndex) {
                    const auto& expected = handoff.bodySnapshots[snapshotIndex];
                    if (expected.valid &&
                        equipped_weapon_drop_momentum::sameBodyIdentity(
                            expected.identity,
                            currentIdentity)) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    return false;
                }
            }
            return true;
        };

        if (handoff.stage == EquippedWeaponDropHandoffStage::ResolvingBodies) {
            std::array<EquippedWeaponDropBodySnapshot, kEquippedWeaponDropBodySnapshotCapacity>
                capturedIdentities{};
            for (std::size_t i = 0; i < acceptedRecordCount; ++i) {
                equipped_weapon_drop_momentum::BodyIdentityKey identity{};
                if (!acceptedRecords[i] ||
                    !tryBuildBodyIdentity(*acceptedRecords[i], identity)) {
                    if (publicationStalled()) {
                        endHandoff("native-identity-read-stalled", true);
                    }
                    return;
                }
                capturedIdentities[i] = EquippedWeaponDropBodySnapshot{
                    .valid = true,
                    .identity = identity,
                };
            }

            const auto currentRootInverse =
                transform_math::invertTransform(droppedRoot->world);
            if (!finiteNiTransform(currentRootInverse)) {
                if (publicationStalled()) {
                    endHandoff("native-root-inverse-stalled", true);
                }
                return;
            }
            std::array<std::uint32_t, kEquippedWeaponDropBodySnapshotCapacity>
                motionBodyIds{};
            std::array<RE::NiTransform, kEquippedWeaponDropBodySnapshotCapacity>
                originalMotionTransforms{};
            std::array<RE::NiTransform, kEquippedWeaponDropBodySnapshotCapacity>
                releaseMotionTargets{};
            for (std::size_t motionIndex = 0;
                 motionIndex < uniqueMotionRecordCount;
                 ++motionIndex) {
                const auto* record = uniqueMotionRecords[motionIndex];
                RE::NiTransform currentBodyWorld{};
                if (!record ||
                    !havok_runtime::tryGetBodyArrayWorldTransform(
                        frame.hknpWorld,
                        RE::hknpBodyId{ record->bodyId },
                        currentBodyWorld) ||
                    !finiteNiTransform(currentBodyWorld)) {
                    if (publicationStalled()) {
                        endHandoff("native-body-transform-read-stalled", true);
                    }
                    return;
                }
                const auto rootToBody = transform_math::composeTransforms(
                    currentRootInverse,
                    currentBodyWorld);
                const auto targetBodyWorld = transform_math::composeTransforms(
                    handoff.releaseWeaponWorld,
                    rootToBody);
                if (!finiteNiTransform(rootToBody) ||
                    !finiteNiTransform(targetBodyWorld)) {
                    if (publicationStalled()) {
                        endHandoff("native-release-target-stalled", true);
                    }
                    return;
                }
                motionBodyIds[motionIndex] = record->bodyId;
                originalMotionTransforms[motionIndex] = currentBodyWorld;
                releaseMotionTargets[motionIndex] = targetBodyWorld;
            }

            const RE::hkVector4f zeroVelocity{};
            std::size_t queuedMotions = 0;
            for (std::size_t motionIndex = 0;
                 motionIndex < uniqueMotionRecordCount;
                 ++motionIndex) {
                const bool transformQueued =
                    havok_runtime::setBodyTransformDeferred(
                        frame.hknpWorld,
                        motionBodyIds[motionIndex],
                        releaseMotionTargets[motionIndex],
                        1);
                const bool velocityQueued =
                    transformQueued &&
                    havok_runtime::setBodyVelocityDeferred(
                        frame.hknpWorld,
                        motionBodyIds[motionIndex],
                        zeroVelocity,
                        zeroVelocity);
                const bool activated =
                    velocityQueued &&
                    havok_runtime::activateBody(
                        frame.hknpWorld,
                        motionBodyIds[motionIndex]);
                if (!activated) {
                    const std::size_t rollbackCount =
                        queuedMotions + (transformQueued ? 1u : 0u);
                    for (std::size_t rollbackIndex = 0;
                         rollbackIndex < rollbackCount;
                         ++rollbackIndex) {
                        (void)havok_runtime::setBodyTransformDeferred(
                            frame.hknpWorld,
                            motionBodyIds[rollbackIndex],
                            originalMotionTransforms[rollbackIndex],
                            1);
                        (void)havok_runtime::setBodyVelocityDeferred(
                            frame.hknpWorld,
                            motionBodyIds[rollbackIndex],
                            zeroVelocity,
                            zeroVelocity);
                        (void)havok_runtime::activateBody(
                            frame.hknpWorld,
                            motionBodyIds[rollbackIndex]);
                    }
                    ROCK_LOG_SAMPLE_WARN(Weapon,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Equipped weapon drop release-pose placement write failed and queued writes were rolled back; retaining transaction: dropped={:08X} queued={} expected={}",
                        handoff.droppedFormId,
                        queuedMotions,
                        uniqueMotionRecordCount);
                    if (publicationStalled()) {
                        endHandoff("native-release-placement-write-stalled", true);
                    }
                    return;
                }
                ++queuedMotions;
            }

            handoff.bodySnapshots = capturedIdentities;
            handoff.bodySnapshotCount = acceptedRecordCount;
            handoff.bodyDiscoverySolveSequence = completedSolveSequence;
            handoff.progressSolveSequence = completedSolveSequence;
            handoff.stage = EquippedWeaponDropHandoffStage::WaitingForSettleStep;
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon drop native bodies placed at frozen release pose; waiting one solve: dropped={:08X} scanned={} bodies={} motions={} collisionEnabled=yes solveSeq={} restarts={}",
                handoff.droppedFormId,
                bodySet.records.size(),
                acceptedRecordCount,
                uniqueMotionRecordCount,
                handoff.bodyDiscoverySolveSequence,
                handoff.identityRestartCount);
            return;
        }

        if (handoff.stage != EquippedWeaponDropHandoffStage::WaitingForSettleStep ||
            !equipped_weapon_drop_momentum::completedSettleStep(
                handoff.bodyDiscoverySolveSequence,
                completedSolveSequence)) {
            return;
        }

        if (!currentBodySetMatches()) {
            ++handoff.identityRestartCount;
            handoff.stage = EquippedWeaponDropHandoffStage::ResolvingBodies;
            handoff.bodySnapshots = {};
            handoff.bodySnapshotCount = 0;
            handoff.progressSolveSequence = completedSolveSequence;
            ROCK_LOG_DEBUG(Weapon,
                "Equipped weapon drop native generation changed before momentum; restarting placement: dropped={:08X} restart={}",
                handoff.droppedFormId,
                handoff.identityRestartCount);
            return;
        }

        const RE::hkVector4f linearVelocity{
            handoff.linearVelocityHavok.x,
            handoff.linearVelocityHavok.y,
            handoff.linearVelocityHavok.z,
            0.0f,
        };
        const RE::hkVector4f angularVelocity{
            handoff.angularVelocityRadiansPerSecond.x,
            handoff.angularVelocityRadiansPerSecond.y,
            handoff.angularVelocityRadiansPerSecond.z,
            0.0f,
        };
        const RE::hkVector4f zeroVelocity{};
        std::size_t velocityWrites = 0;
        if (handoff.hasReleaseVelocity) {
            for (std::size_t motionIndex = 0;
                 motionIndex < uniqueMotionRecordCount;
                 ++motionIndex) {
                const auto* record = uniqueMotionRecords[motionIndex];
                if (!record ||
                    !havok_runtime::setBodyVelocityDeferred(
                        frame.hknpWorld,
                        record->bodyId,
                        linearVelocity,
                        angularVelocity)) {
                    break;
                }
                ++velocityWrites;
            }
        } else {
            velocityWrites = uniqueMotionRecordCount;
        }

        std::size_t completedMotions = 0;
        if (velocityWrites == uniqueMotionRecordCount) {
            for (std::size_t motionIndex = 0;
                 motionIndex < uniqueMotionRecordCount;
                 ++motionIndex) {
                const auto* record = uniqueMotionRecords[motionIndex];
                if (!record ||
                    !havok_runtime::activateBody(frame.hknpWorld, record->bodyId)) {
                    break;
                }
                ++completedMotions;
            }
        }
        if (completedMotions != uniqueMotionRecordCount) {
            if (handoff.hasReleaseVelocity) {
                for (std::size_t motionIndex = 0;
                     motionIndex < velocityWrites;
                     ++motionIndex) {
                    const auto* record = uniqueMotionRecords[motionIndex];
                    if (!record) {
                        continue;
                    }
                    (void)havok_runtime::setBodyVelocityDeferred(
                        frame.hknpWorld,
                        record->bodyId,
                        zeroVelocity,
                        zeroVelocity);
                    (void)havok_runtime::activateBody(
                        frame.hknpWorld,
                        record->bodyId);
                }
            }
            ROCK_LOG_SAMPLE_WARN(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Equipped weapon drop final momentum write incomplete and partial writes were zeroed; retaining transaction: dropped={:08X} velocityWrites={} activated={} expected={}",
                handoff.droppedFormId,
                velocityWrites,
                completedMotions,
                uniqueMotionRecordCount);
            if (publicationStalled()) {
                endHandoff("final-momentum-write-stalled", true);
            }
            return;
        }

        ROCK_LOG_INFO(Weapon,
            "Equipped weapon drop handoff complete after native contact solve: dropped={:08X} bodies={} motions={} velocity={} elapsed={:.3f}s linear=({:.3f},{:.3f},{:.3f}) angular=({:.3f},{:.3f},{:.3f})",
            handoff.droppedFormId,
            acceptedRecordCount,
            uniqueMotionRecordCount,
            handoff.hasReleaseVelocity ? "applied" : "none",
            handoff.elapsedSeconds,
            handoff.linearVelocityHavok.x,
            handoff.linearVelocityHavok.y,
            handoff.linearVelocityHavok.z,
            handoff.angularVelocityRadiansPerSecond.x,
            handoff.angularVelocityRadiansPerSecond.y,
            handoff.angularVelocityRadiansPerSecond.z);
        handoff = {};
    }
}
