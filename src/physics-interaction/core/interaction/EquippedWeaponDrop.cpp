#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/weapon/telemetry/VanillaWeaponAlignmentTelemetry.h"

// Shared native placement for equipped-to-loose Toggle Drop and Auto Drop.

namespace rock
{
    bool PhysicsInteraction::dropEquippedWeaponToWorld(const PhysicsFrameContext& frame,
        const EquippedWeaponManualDropRequest& request, equipped_weapon_drop_policy::Mode mode)
    {
        if (!frame.worldReady || !frame.hknpWorld || frame.menuBlocked ||
            mode == equipped_weapon_drop_policy::Mode::Off) return false;
        auto* hknp = frame.hknpWorld;
        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        const auto observedEquippedWeaponFormID = currentEquippedWeaponFormId();
        const auto sourceHand = request.sourceHand;
        const bool sourceHandKnown = sourceHand == equipped_weapon_drop_policy::SourceHand::Right ||
            sourceHand == equipped_weapon_drop_policy::SourceHand::Left;
        const auto dropLoc = equipped_weapon_drop_policy::isLeft(sourceHand) ?
            frame.left.grabAnchorWorld : frame.right.grabAnchorWorld;
        bool transferCommitted = false;
        const bool transferIsLeft = equipped_weapon_drop_policy::isLeft(sourceHand);
        const auto transferHandIndex = transferIsLeft ? 1u : 0u;
        Hand& transferHand = transferIsLeft ? _leftHand : _rightHand;
        const auto& transferInput = transferIsLeft ? frame.left : frame.right;
        const bool dropHandoffAvailable = sourceHandKnown && request.pose.valid() &&
            request.pose.weaponFormId == observedEquippedWeaponFormID && hasAvailableEquippedWeaponDropHandoff() &&
            (forceGrabHandBlockerMask(transferHand, transferIsLeft, transferInput.disabled, true) &
                ~static_cast<std::uint32_t>(force_grab_policy::HandBlocker::EquippedWeapon)) == 0;
        if (!dropHandoffAvailable) {
            ROCK_LOG_WARN(Weapon,
                "Equipped weapon transfer blocked: source hand unavailable or native handoff capacity exhausted capacity={}",
                _drop.nativeHandoffs.size());
            f4vr::showNotification("ROCK: Cannot take weapon - hand or transfer queue is busy.");
        }
        if (dropHandoffAvailable) {
            // Preserve the equipped pose while the native loose bodies
            // appear. Toggle Drop then seats the exact reference using
            // the same authored weapon resolver as a far-grab catch.
            RE::NiPoint3 releaseLoc = dropLoc;
            RE::NiPoint3 releaseRot{};
            const auto& releaseWeaponWorld = request.weaponWorld;
            const bool hasReleaseRot = finiteNiTransform(releaseWeaponWorld);
            if (hasReleaseRot) {
                releaseLoc = releaseWeaponWorld.translate;
                releaseRot = transform_math::matrixToReferenceEulerRadians<RE::NiMatrix3, RE::NiPoint3>(releaseWeaponWorld.rotate);
            }
            const std::size_t releaseHandIndex = equipped_weapon_drop_policy::isLeft(sourceHand) ? 1u : 0u;
            const auto& releaseHandInput = releaseHandIndex == 1u ? frame.left : frame.right;
            const RE::NiPoint3 releaseGripWorld = releaseHandInput.grabAnchorWorld;
            // Capture the native placement basis before retiring the
            // generated equipped representation.
            const auto releaseGeometry = hasReleaseRot ?
                                             _weaponCollision.getCurrentWeaponReleaseGeometry(releaseGripWorld, releaseWeaponWorld) :
                                             WeaponCollision::ReleaseGeometrySnapshot{};
            if (!releaseGeometry.hasCapturedWeaponWorld) {
                ROCK_LOG_WARN(Weapon,
                    "Equipped weapon physical drop blocked because no finite frozen release pose is available: sourceHand={}",
                    equipped_weapon_drop_policy::sourceHandName(sourceHand));
                f4vr::showNotification("ROCK: Cannot drop weapon - release pose is not ready.");
            } else {
                const bool toggleDrop = mode ==
                    equipped_weapon_drop_policy::Mode::ToggleDrop;
                const auto sourceVisual = toggleDrop ?
                    equipped_weapon_visual_state::observe(observedEquippedWeaponFormID) :
                    equipped_weapon_visual_state::Snapshot{};
                RE::NiPointer<RE::NiAVObject> dropVisualModel(
                    sourceVisual.ancestorPathVisible && sourceVisual.instanceLocallyVisible ?
                        sourceVisual.exactInstance : nullptr);
                const auto dropVisualInWeapon = dropVisualModel ?
                    transform_math::composeTransforms(transform_math::invertTransform(releaseWeaponWorld),
                        dropVisualModel->world) : RE::NiTransform{};
                if (toggleDrop) {
                    vanilla_weapon_alignment_telemetry::beginTransferTrace(
                        vanilla_weapon_alignment_telemetry::TransferKind::ToggleDrop,
                        transferIsLeft, observedEquippedWeaponFormID, weaponNode);
                }
                _twoHandedGrip.prepareEquippedWeaponDropCommit();
                const auto dropResult = weapon_equip_transfer::dropEquippedWeaponFromPlayer(weapon_equip_transfer::EquippedDropInput{
                    .dropLoc = releaseLoc,
                    .dropRot = releaseRot,
                    .hasDropLoc = true,
                    .hasDropRot = true,
                });
                const bool dropCommitted = equipped_weapon_drop_policy::physicalDropCommitted(
                    equipped_weapon_drop_policy::PhysicalDropCommitInput{
                        .dropSucceeded = dropResult.success,
                        .droppedReferenceUnavailable =
                            dropResult.reason == weapon_equip_transfer::DropReason::DroppedReferenceUnavailable,
                    });
                transferCommitted = dropCommitted;
                const auto& transfer = _equipped.transition.heldTransfer();
                const bool replacementDrop = dropCommitted && transfer.phase == held_weapon_transfer::Phase::AwaitEquip &&
                    transfer.request.retainOutgoing && !transfer.outgoingRemoved &&
                    transfer.request.previousForm == observedEquippedWeaponFormID && transfer.request.isLeft != transferIsLeft;
                if (replacementDrop) _equipped.transition.recordOutgoingRemoval(dropResult.droppedFormID);
                if (dropCommitted) {
                    enforceNoBareFistState(true);
                    /*
                     * RemoveItem creates the native layer-5 weapon at
                     * the last layer-44 equipped-collider pose. Retire
                     * ROCK's generated representation in this same
                     * transaction so no physics step can solve the two
                     * coincident weapon body sets before the native
                     * handoff takes ownership.
                     */
                    _weaponCollision.destroyWeaponBody(hknp);
                }
                if (dropCommitted && dropResult.handle) {
                    _forceGrab.pendingCommits[transferHandIndex] = PendingForceGrabCommit{
                        .active = true,
                        .isLeft = transferIsLeft,
                        .phase = dropResult.equippedSlotReleased ?
                            PendingForceGrabCommitPhase::WaitingForNativePlacement :
                            PendingForceGrabCommitPhase::EquippedSlotReleaseFailed,
                        .targetHandle = dropResult.handle,
                        .inventoryTransfer = true,
                        .weaponTransferSequence = replacementDrop ? transfer.sequence : 0,
                        .equippedWeaponDropMode = mode,
                        .weaponGripPose = request.pose,
                        .maxDistanceGame = 96.0f,
                    };
                    if (_forceGrab.pendingCommits[transferHandIndex].equippedWeaponDropMode ==
                        equipped_weapon_drop_policy::Mode::ToggleDrop) {
                        _forceGrab.retainedWeaponGrabs[transferHandIndex] = {
                            .inputState = transferred_weapon_grab_policy::State::AwaitInitialRelease,
                        };
                    }
                    vanilla_weapon_alignment_telemetry::recordTransferPose(
                        dropResult.droppedFormID, transferIsLeft, "release",
                        releaseGeometry.capturedWeaponWorld,
                        (transferIsLeft ? frame.left : frame.right).rawHandWorld);
                    if (dropResult.equippedSlotReleased) {
                        if (toggleDrop) {
                            _drop.visuals[transferHandIndex].begin(std::move(dropVisualModel),
                                dropVisualInWeapon, request.pose, dropResult.droppedFormID);
                        }
                        armEquippedWeaponNativeHandoff(
                            dropResult.handle,
                            dropResult.droppedFormID,
                            sourceHand,
                            releaseGeometry);
                    }
                }
                if (dropCommitted) {
                    ROCK_LOG_INFO(Weapon,
                        "Equipped weapon drop queued mode={} formID={:08X} dropped={:08X} reference={} sourceHand={} dropLoc=({:.1f},{:.1f},{:.1f}) lever={:.1f}gu stack={} instanceMatch={}",
                        static_cast<int>(mode),
                        dropResult.formID,
                        dropResult.droppedFormID,
                        dropResult.success ? "ready" : "pending",
                        equipped_weapon_drop_policy::sourceHandName(sourceHand),
                        releaseLoc.x,
                        releaseLoc.y,
                        releaseLoc.z,
                        releaseGeometry.leverGameUnits,
                        dropResult.stackID,
                        dropResult.matchedInstanceData ? "yes" : "no");
                } else {
                    ROCK_LOG_WARN(Weapon,
                        "Equipped weapon manual release drop failed formID={:08X} reason={} sourceHand={} attempted={} stack={} instanceMatch={}",
                        dropResult.formID,
                        weapon_equip_transfer::dropReasonName(dropResult.reason),
                        equipped_weapon_drop_policy::sourceHandName(sourceHand),
                        dropResult.attempted ? "yes" : "no",
                        dropResult.stackID,
                        dropResult.matchedInstanceData ? "yes" : "no");
                }
                if (sourceHandKnown && dropCommitted) {
                    suppressHandCollisionAfterEquippedWeaponDrop(hknp, sourceHand);
                }
            }
        }
        return transferCommitted;
    }

    void PhysicsInteraction::updateEquippedWeaponDropVisuals(const PhysicsFrameContext& frame)
    {
        for (std::size_t index = 0; index < _drop.visuals.size(); ++index) {
            auto& visual = _drop.visuals[index];
            if (!visual.active()) continue;
            const auto& commit = _forceGrab.pendingCommits[index];
            const auto& input = index == 1 ? frame.left : frame.right;
            const auto& hand = index == 1 ? _leftHand : _rightHand;
            RE::NiTransform handWorld{};
            const auto reference = commit.targetHandle.get();
            if (!commit.active || commit.equippedWeaponDropMode != equipped_weapon_drop_policy::Mode::ToggleDrop ||
                frame.menuBlocked || !frame.worldReady ||
                !canHandAcceptForceGrab(hand, index == 1, input.disabled) ||
                !_twoHandedGrip.tryGetPhysicalHandWorld(index == 1, handWorld)) {
                visual.release("handoff-finished-or-hand-unavailable");
                continue;
            }
            visual.update(handWorld, reference ? reference->Get3D() : nullptr);
        }
    }

    bool PhysicsInteraction::hasAvailableEquippedWeaponDropHandoff() const
    {
        return std::any_of(
            _drop.nativeHandoffs.begin(),
            _drop.nativeHandoffs.end(),
            [](const EquippedWeaponNativeHandoff& handoff) { return !handoff.active; });
    }

    void PhysicsInteraction::armEquippedWeaponNativeHandoff(
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
        EquippedWeaponNativeHandoff* handoff = nullptr;
        for (auto& candidate : _drop.nativeHandoffs) {
            if (!candidate.active) {
                handoff = &candidate;
                break;
            }
        }
        if (!handoff) {
            ROCK_LOG_WARN(Weapon,
                "Equipped weapon drop handoff capacity exhausted after admission: dropped={:08X} capacity={}",
                droppedFormId,
                _drop.nativeHandoffs.size());
            return;
        }

        *handoff = EquippedWeaponNativeHandoff{
            .active = true,
            .handle = handle,
            .droppedFormId = droppedFormId,
            .hasReleaseWeaponWorld = releaseGeometry.hasCapturedWeaponWorld,
            .releaseWeaponWorld = releaseGeometry.capturedWeaponWorld,
            .progressSolveSequence = _frame.completedPhysicsSolveSequence.load(std::memory_order_acquire),
        };
        ROCK_LOG_INFO(Weapon,
            "Equipped weapon native drop placement armed: ref={:08X} hand={}",
            droppedFormId, equipped_weapon_drop_policy::sourceHandName(sourceHand));
    }

    void PhysicsInteraction::serviceEquippedWeaponNativeHandoff(const PhysicsFrameContext& frame)
    {
        for (auto& handoff : _drop.nativeHandoffs) {
            if (handoff.active) {
                serviceEquippedWeaponNativeTransaction(handoff, frame);
            }
        }
    }

    void PhysicsInteraction::reportEquippedWeaponPlacementFailure(const EquippedWeaponNativeHandoff& handoff, const char* reason) const
    {
        ROCK_LOG_WARN(Weapon,
            "Native placement failure ref={:08X} reason={} waiting={} stage={} elapsed={:.3f}s refSeen={} rootSeen={} nodes={} collisionObjects={} scanned={} accepted={} motions={} scanFailures={} invalidSystems={} depthSkips={} foreignRefSkips={} rejectionMask={:X} body={} identityProofMask={:X} solve={}->{} restarts={}",
            handoff.droppedFormId, reason, handoff.waitReason, static_cast<unsigned>(handoff.stage), handoff.elapsedSeconds,
            handoff.referenceResolvedOnce, handoff.threeDResolvedOnce, handoff.visitedNodes, handoff.collisionObjects,
            handoff.scannedBodies, handoff.acceptedBodies, handoff.uniqueMotions, handoff.scanFailures, handoff.invalidSystems,
            handoff.depthSkips, handoff.foreignRefSkips, handoff.rejectionMask, handoff.inspectedBodyId, handoff.identityProofMask,
            handoff.progressSolveSequence, handoff.observedSolveSequence, handoff.identityRestartCount);
    }

    void PhysicsInteraction::serviceEquippedWeaponNativeTransaction(
        EquippedWeaponNativeHandoff& handoff,
        const PhysicsFrameContext& frame)
    {
        // This bound applies only while an asynchronously published drop has
        // made no state progress. Paused physics does not consume the budget.
        constexpr std::uint64_t kPublicationStallSolveSteps = 180;

        handoff.waitReason = "world-unavailable";
        if (!handoff.active || !frame.worldReady || !frame.hknpWorld) {
            return;
        }

        handoff.elapsedSeconds += (std::max)(0.0f, frame.deltaSeconds);
        const std::uint64_t completedSolveSequence =
            _frame.completedPhysicsSolveSequence.load(std::memory_order_acquire);
        handoff.observedSolveSequence = completedSolveSequence;
        const auto publicationStalled = [&]() {
            return equipped_weapon_drop_momentum::publicationProgressStalled(
                handoff.progressSolveSequence,
                completedSolveSequence,
                kPublicationStallSolveSteps);
        };
        const auto endHandoff = [&](const char* reason, bool warn, bool ready = false) {
            for (auto& commit : _forceGrab.pendingCommits) {
                if (commit.active && commit.isEquippedWeaponTransfer() && commit.targetHandle == handoff.handle) {
                    commit.phase = ready ? PendingForceGrabCommitPhase::AcquireAndCommitExactTarget :
                                           PendingForceGrabCommitPhase::NativePlacementFailed;
                }
            }
            if (warn) {
                reportEquippedWeaponPlacementFailure(handoff, reason);
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

        handoff.waitReason = "reference-unavailable";
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

        handoff.waitReason = "physics-world-or-3d-unavailable";
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
            handoff.waitReason = "collision-enable";
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
        handoff.waitReason = "exact-reference-scan";
        handoff.visitedNodes = bodySet.diagnostics.visitedNodes;
        handoff.collisionObjects = bodySet.diagnostics.collisionObjects;
        handoff.scannedBodies = static_cast<std::uint32_t>(bodySet.records.size());
        handoff.acceptedBodies = 0;
        handoff.uniqueMotions = 0;
        handoff.scanFailures = bodySet.diagnostics.scanFailures;
        handoff.invalidSystems = bodySet.diagnostics.invalidPhysicsSystems;
        handoff.depthSkips = bodySet.diagnostics.depthLimitSkips;
        handoff.foreignRefSkips = bodySet.diagnostics.foreignRefBodySkips;
        handoff.rejectionMask = 0;
        for (std::size_t i = 0; i < bodySet.diagnostics.rejectCounts.size(); ++i)
            if (bodySet.diagnostics.rejectCounts[i] != 0) handoff.rejectionMask |= std::uint64_t{1} << i;
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
        handoff.waitReason = "accepted-native-bodies";
        handoff.acceptedBodies = static_cast<std::uint32_t>(acceptedRecordCount);
        handoff.uniqueMotions = static_cast<std::uint32_t>(uniqueMotionRecordCount);
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
            handoff.inspectedBodyId = bodyId;
            const auto live = havok_runtime::snapshotBody(
                frame.hknpWorld,
                RE::hknpBodyId{ bodyId });
            // Bits: valid/body/motion/shape/collision/owner/bodyId/system-instance.
            handoff.identityProofMask = (live.valid ? 1u : 0u) | (live.body ? 2u : 0u) | (live.motion ? 4u : 0u) |
                (live.valid && live.body && live.body->shape ? 8u : 0u) | (live.collisionObject ? 16u : 0u) |
                (live.ownerNode ? 32u : 0u) | (live.valid && live.body && live.body->bodyId.value == bodyId ? 64u : 0u);
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
            handoff.identityProofMask |= 128u;
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
            // Additional bits identify which scan/live identity comparison failed.
            handoff.identityProofMask |= (outIdentity.motionId == record.motionId ? 256u : 0u) |
                (outIdentity.collisionObjectIdentity == reinterpret_cast<std::uintptr_t>(record.collisionObject) ? 512u : 0u) |
                (outIdentity.owningNodeIdentity == reinterpret_cast<std::uintptr_t>(record.owningNode) ? 1024u : 0u);
            auto* discoveredSystem = record.collisionObject ?
                havok_runtime::getPhysicsSystemFromCollisionObject(record.collisionObject) : nullptr;
            const auto discoveredInstance = reinterpret_cast<std::uintptr_t>(
                havok_runtime::getPhysicsSystemInstance(discoveredSystem));
            // Bits 11/12: discovery system available / same native system.
            handoff.identityProofMask |= (discoveredInstance != 0 ? 2048u : 0u) |
                (discoveredInstance != 0 && discoveredInstance == outIdentity.physicsSystemInstanceIdentity ? 4096u : 0u);
            return equipped_weapon_drop_momentum::matchesDiscoveredBody(
                outIdentity, record.bodyId, record.motionId, discoveredInstance);
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
            handoff.waitReason = "body-identity";
            auto placementWorld = handoff.releaseWeaponWorld;
            // Toggle Drop remains carried while native bodies are published.
            // Place those bodies on the discovery frame's hand, not the older
            // release frame. Auto Drop has already released and stays in world space.
            for (const auto& commit : _forceGrab.pendingCommits) {
                if (!commit.active || commit.targetHandle != handoff.handle ||
                    commit.equippedWeaponDropMode != equipped_weapon_drop_policy::Mode::ToggleDrop) continue;
                RE::NiTransform handWorld{};
                if (!commit.weaponGripPose.valid() || !_twoHandedGrip.tryGetPhysicalHandWorld(commit.isLeft, handWorld)) {
                    endHandoff("native-placement-hand-unavailable", true);
                    return;
                }
                placementWorld = transform_math::composeTransforms(handWorld,
                    transform_math::invertTransform(commit.weaponGripPose.placementHandWeaponLocal));
                if (!finiteNiTransform(placementWorld)) {
                    endHandoff("native-placement-hand-invalid", true);
                    return;
                }
                vanilla_weapon_alignment_telemetry::recordTransferTrace(
                    vanilla_weapon_alignment_telemetry::TransferKind::ToggleDrop, commit.isLeft,
                    "native-placement-target", droppedRoot, &placementWorld);
                break;
            }
            std::array<EquippedWeaponDropBodySnapshot, kEquippedWeaponDropBodySnapshotCapacity>
                capturedIdentities{};
            std::size_t sharedSystemOwners = 0;
            for (std::size_t i = 0; i < acceptedRecordCount; ++i) {
                equipped_weapon_drop_momentum::BodyIdentityKey identity{};
                if (!acceptedRecords[i] ||
                    !tryBuildBodyIdentity(*acceptedRecords[i], identity)) {
                    if (publicationStalled()) {
                        endHandoff("native-identity-read-stalled", true);
                    }
                    return;
                }
                if (identity.collisionObjectIdentity != reinterpret_cast<std::uintptr_t>(acceptedRecords[i]->collisionObject) ||
                    identity.owningNodeIdentity != reinterpret_cast<std::uintptr_t>(acceptedRecords[i]->owningNode)) {
                    ++sharedSystemOwners;
                }
                capturedIdentities[i] = EquippedWeaponDropBodySnapshot{
                    .valid = true,
                    .identity = identity,
                };
            }

            handoff.waitReason = "root-inverse";
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
                handoff.waitReason = "body-world-transform";
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
                handoff.waitReason = "release-target-transform";
                const auto rootToBody = transform_math::composeTransforms(
                    currentRootInverse,
                    currentBodyWorld);
                const auto targetBodyWorld = transform_math::composeTransforms(
                    placementWorld,
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
                handoff.waitReason = !transformQueued ? "release-transform-write" : !velocityQueued ? "release-velocity-write" : "body-activation";
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
            handoff.waitReason = "completed-physics-solve";
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon drop native bodies placed at handoff pose; waiting one solve: dropped={:08X} scanned={} bodies={} motions={} collisionEnabled=yes solveSeq={} restarts={} sharedSystemOwners={}",
                handoff.droppedFormId,
                bodySet.records.size(),
                acceptedRecordCount,
                uniqueMotionRecordCount,
                handoff.bodyDiscoverySolveSequence,
                handoff.identityRestartCount,
                sharedSystemOwners);
            return;
        }

        handoff.waitReason = "completed-physics-solve";
        if (handoff.stage != EquippedWeaponDropHandoffStage::WaitingForSettleStep ||
            !equipped_weapon_drop_momentum::completedSettleStep(
                handoff.bodyDiscoverySolveSequence,
                completedSolveSequence)) {
            return;
        }

        handoff.waitReason = "settled-body-identity";
        if (!currentBodySetMatches()) {
            ++handoff.identityRestartCount;
            handoff.stage = EquippedWeaponDropHandoffStage::ResolvingBodies;
            handoff.bodySnapshots = {};
            handoff.bodySnapshotCount = 0;
            handoff.progressSolveSequence = completedSolveSequence;
            ROCK_LOG_DEBUG(Weapon,
                "Equipped weapon drop native generation changed before placement completed; restarting placement: dropped={:08X} restart={}",
                handoff.droppedFormId,
                handoff.identityRestartCount);
            return;
        }

        handoff.waitReason = "settled-velocity-clear";
        // Both drop modes share native placement and clear spawn/contact
        // velocity. Toggle Drop continues into the retained grab afterward.
        const RE::hkVector4f zeroVelocity{};
        for (std::size_t i = 0; i < uniqueMotionRecordCount; ++i) {
            const auto* record = uniqueMotionRecords[i];
            if (!record || !havok_runtime::setBodyVelocityDeferred(
                    frame.hknpWorld, record->bodyId, zeroVelocity, zeroVelocity) ||
                !havok_runtime::activateBody(frame.hknpWorld, record->bodyId)) {
                if (publicationStalled()) {
                    endHandoff("native-velocity-clear-stalled", true);
                }
                return;
            }
        }
        ROCK_LOG_INFO(Weapon,
            "Equipped weapon native drop placement ready: ref={:08X} bodies={} motions={} elapsed={:.3f}s",
            handoff.droppedFormId, acceptedRecordCount, uniqueMotionRecordCount, handoff.elapsedSeconds);
        endHandoff("native-placement-ready", false, true);
    }
}
