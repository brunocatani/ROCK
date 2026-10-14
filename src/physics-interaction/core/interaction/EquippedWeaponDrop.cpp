#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/native/BipodFrozenRetirement.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/weapon/BipodRetentionPolicy.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/weapon/NativeHandEquipment.h"
#include "physics-interaction/weapon/NativeSecondaryPresentation.h"
#include "physics-interaction/weapon/NativeSecondaryActions.h"

// Shared native placement for equipped-to-loose Toggle Drop and Auto Drop.

namespace rock
{
    bool PhysicsInteraction::retireHandsFreeWeapon(const PhysicsFrameContext& frame, RE::NiNode* weaponNode)
    {
        if (!_twoHandedGrip.isHandsFreeRetained() || !weaponNode || frame.menuBlocked ||
            !frame.worldReady || !physicsWritesAllowedForWorld(frame.hknpWorld)) return false;
        RE::NiPoint3 anchor{};
        const bool hasAnchor = _dynamicWeaponCollision.tryGetSurfaceAnchor(anchor);
        const bool outside = frame.hasHmdFrame && hasAnchor &&
            bipod_retention_policy::outsideRadius(frame.hmdPositionWorld, anchor,
                physics_scale::gameToHavok(), g_rockConfig.rockBipodHandsFreeRadiusMeters);
        const bool supportLost = !_dynamicWeaponCollision.hasLatchedSurfaceSupport(
            reinterpret_cast<std::uintptr_t>(weaponNode), _weaponCollision.getCurrentWeaponGenerationKey());
        if (!outside && !_dynamicWeaponCollision.surfaceRetirementRequested() && !supportLost) return false;
        const auto pose = weaponNode->world;
        const auto form = currentEquippedWeaponFormId();
        if (!bipod_frozen_retirement::arm(form, frame.bhkWorld, pose)) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000, "Bipod retirement retained equipped item form={:08X} nativeGuard={} pending={}",
                form, bipod_frozen_retirement::available(), bipod_frozen_retirement::pending());
            return false;
        }
        const auto source = equipped_weapon_visual_state::observe(form);
        RE::NiPointer<RE::NiAVObject> model(source.ancestorPathVisible && source.instanceLocallyVisible ? source.exactInstance : nullptr);
        const auto modelInWeapon = model ? transform_math::composeTransforms(
            transform_math::invertTransform(pose), model->world) : RE::NiTransform{};
        RE::NiPointer<RE::NiNode> retainedNode(weaponNode);
        auto mutation = _generatedBodyStepDrive.callbackGate().pauseForMutation();
        _twoHandedGrip.prepareEquippedWeaponDropCommit();
        const auto drop = weapon_equip_transfer::dropEquippedWeaponFromPlayer({
            .dropLoc = pose.translate,
            .dropRot = transform_math::matrixToReferenceEulerRadians<RE::NiMatrix3, RE::NiPoint3>(pose.rotate),
            .hasDropLoc = true,
            .hasDropRot = true,
        });
        const bool committed = static_cast<bool>(drop.handle);
        bipod_frozen_retirement::finishRemoval(committed);
        if (!committed) {
            ROCK_LOG_WARN(Weapon, "Bipod retirement retained equipped item form={:08X} reason={}",
                form, weapon_equip_transfer::dropReasonName(drop.reason));
            return false;
        }
        _dynamicWeaponCollision.retireAll(frame.bhkWorld);
        _weaponCollision.destroyWeaponBody(frame.hknpWorld);
        _twoHandedGrip.finishHandsFreeRetirement();
        _bipodRetirementHandle = drop.handle;
        _bipodRetirementVisual.beginStationary(std::move(model), modelInWeapon, pose, drop.droppedFormID);
        _equipped.transition.pendingGrip() = {};
        clearEquippedWeaponFiringGripInputState();
        enforceNoBareFistState(true);
        ROCK_LOG_INFO(Weapon, "Bipod hands-free retirement committed form={:08X} ref={:08X} reason={} position=({:.3f},{:.3f},{:.3f})",
            form, drop.droppedFormID, outside ? "outside-radius" : "support-released",
            pose.translate.x, pose.translate.y, pose.translate.z);
        return true;
    }

    api::Status PhysicsInteraction::extractEquippedForStorage(bool left, std::uint32_t weapon, std::uint32_t& reference)
    {
        reference = 0;
        if (!isProviderReady() || !_lifecycle.cachedBhkWorld || !_lifecycle.cachedHknpWorld || currentEquippedWeaponFormId() != weapon)
            return api::Status::TargetUnavailable;
        const auto occupancy = equippedWeaponRoster().slotZeroGrips();
        const bool carries = left ? occupancy.left.carriesWeapon() : occupancy.right.carriesWeapon();
        const bool otherCarries = left ? occupancy.right.carriesWeapon() : occupancy.left.carriesWeapon();
        if (!carries || otherCarries || (left ? _leftHand : _rightHand).isHolding() || _forceGrab.pendingCommits[left ? 1 : 0].active)
            return api::Status::HandBusy;
        const auto frame = buildFrameContext(_lifecycle.cachedBhkWorld, _lifecycle.cachedHknpWorld);
        if (frame.menuBlocked) return api::Status::NotReady;
        const auto source = left ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right;
        if (!_twoHandedGrip.requestEquippedWeaponDrop("external-storage", source,
                equipped_weapon_drop_policy::Mode::AutoDrop, frame.deltaSeconds)) return api::Status::NotReady;
        const auto request = _twoHandedGrip.consumeEquippedWeaponDropRequest();
        RE::NiPointer<RE::NiNode> sourceNode(resolveEquippedWeaponInteractionNode());
        const bool dropped = dropEquippedWeaponToWorld(frame, request, equipped_weapon_drop_policy::Mode::AutoDrop);
        _twoHandedGrip.completeEquippedWeaponDrop(request, dropped);
        if (!dropped) return api::Status::TargetUnavailable;
        _equipped.transition.pendingGrip() = {}; clearEquippedWeaponFiringGripInputState();
        const auto ref = _forceGrab.pendingCommits[left ? 1 : 0].targetHandle.get();
        if (!ref) return api::Status::NotReady;
        reference = ref->formID;
        return api::Status::RequestQueued;
    }

    RE::NiAVObject* PhysicsInteraction::storageEquippedVisual(bool left, std::uint32_t expectedBase) const
    {
        if (currentEquippedWeaponFormId() != expectedBase) return nullptr;
        const auto occupancy = equippedWeaponRoster().slotZeroGrips();
        if (!(left ? occupancy.left.carriesWeapon() : occupancy.right.carriesWeapon())) return nullptr;
        return resolveEquippedWeaponInteractionNode();
    }
    bool PhysicsInteraction::storageHandoffPending(std::uint32_t reference) const
    {
        for (const auto& pending : _forceGrab.pendingCommits) if (pending.active) {
            const auto ref = pending.targetHandle.get();
            if (ref && ref->formID == reference) return true;
        }
        for (const auto& handoff : _drop.nativeHandoffs)
            if (handoff.active && handoff.droppedFormId == reference) return true;
        return false;
    }

    bool PhysicsInteraction::dropEquippedWeaponToWorld(const PhysicsFrameContext& frame,
        const EquippedWeaponManualDropRequest& request, equipped_weapon_drop_policy::Mode mode)
    {
        if (!frame.worldReady || !frame.hknpWorld || frame.menuBlocked ||
            mode == equipped_weapon_drop_policy::Mode::Off || mode != request.mode) return false;
        auto* hknp = frame.hknpWorld;
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
                // A second weapon outlives the primary: it is promoted to
                // index 0 in its own hand below, so the player stays armed.
                const auto secondary = native_hand_equipment::equippedViews()[1];
                const bool secondaryLeft = secondary.weapon && native_hand_equipment::secondaryHandIsLeft();
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
                if (dropCommitted && dropResult.equippedSlotReleased && secondary.weapon) {
                    promoteSecondaryWeapon(secondary.weapon, reinterpret_cast<std::uintptr_t>(secondary.instance), secondaryLeft);
                }
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
                    armTransitionSeamTrace("first-drop");
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
                if (sourceHandKnown && dropCommitted &&
                    (sourceHand == equipped_weapon_drop_policy::SourceHand::Right ||
                        sourceHand == equipped_weapon_drop_policy::SourceHand::Left)) {
                    // The releasing hand and arm stop colliding briefly so they
                    // do not knock the weapon they just let go of.
                    const bool dropIsLeft = equipped_weapon_drop_policy::isLeft(sourceHand);
                    setWeaponHandSuppression(hknp, dropIsLeft, WeaponHandReason::Drop, true);
                    setWeaponHandSuppression(hknp, dropIsLeft, WeaponHandReason::Drop, false,
                        g_rockConfig.rockGrabReleaseHandCollisionDelaySeconds);
                }
            }
        }
        return transferCommitted;
    }

    void PhysicsInteraction::promoteSecondaryWeapon(RE::TESObjectWEAP* weapon, std::uintptr_t instance, bool left)
    {
        /*
         * The re-equip into index 0 destroys this weapon's index-1 model and
         * draws a new one, which first renders in the native carry. Keep the
         * outgoing model, at its last placement and with the hand it
         * published, in the equip bridge until the new model is posed.
         */
        native_secondary_presentation::DropPose presented{};
        const char* unpresented = native_secondary_presentation::tryGetDropPose(presented);
        RE::NiPointer<RE::NiAVObject> outgoingModel(unpresented ? nullptr : presented.model);
        weapon_grip_transfer::HandGrip outgoingGrip{};
        if (outgoingModel) {
            outgoingGrip.handWeaponLocal = transform_math::composeTransforms(
                weapon_visual_authority_math::invertSceneTransform(outgoingModel->world), presented.handWorld);
            outgoingGrip.gripWeaponLocal = computeGrabPalmPointWorldFromHandBasis(outgoingGrip.handWeaponLocal, left);
            outgoingGrip.fingerLocals = presented.pose.fingerLocals;
            outgoingGrip.fingerMask = presented.pose.fingerMask;
            outgoingGrip.authoredRole = loose_weapon_authored_grab_policy::Role::Firing;
            outgoingGrip.hasFingerPose = true;
        } else {
            ROCK_LOG_WARN(Weapon, "AKIMBO promotion without outgoing model form={:08X}: {}", weapon->GetFormID(), unpresented);
        }
        armTransitionSeamTrace("promotion");
        native_secondary_actions::suspend();
        const auto formID = weapon->GetFormID();
        const auto equipped = weapon_equip_transfer::promoteSecondaryToPrimary(formID, instance);
        ROCK_LOG_INFO(Weapon, "AKIMBO second weapon promoted to primary form={:08X} hand={} success={} reason={} observed={:08X}",
            formID, left ? "left" : "right", equipped.success, weapon_equip_transfer::equipReasonName(equipped.reason),
            equipped.observedEquippedFormID);
        if (!equipped.success) return;
        // Same start as an inventory equip into that hand: its firing grip
        // and the native draw/visual continuity of the new index-0 weapon.
        native_hand_equipment::directPrimaryHand(formID, equipped.observedEquippedInstanceData, left);
        auto& coordinator = _equipped.transition;
        coordinator.pendingGrip() = {
            .pending = true, .isLeft = left,
            .targetWeaponFormID = formID,
            .targetWeaponInstanceData = equipped.observedEquippedInstanceData,
            .remainingSeconds = _equipped.handlingSettings.equipVisualBridgeTimeoutSeconds,
            .source = equipped_weapon_manual_ownership_policy::PrimaryOnlyStartSource::InventoryEquip,
        };
        coordinator.beginHeldTransition({ .formID = formID, .instanceData = equipped.observedEquippedInstanceData },
            EquippedWeaponTransitionCoordinator::Source::InventoryEquip, {
                .worldModel = std::move(outgoingModel),
                .weaponFormID = formID, .isLeftHand = left,
                .singleGrip = outgoingGrip.valid() ? &outgoingGrip : nullptr,
                .weapon = weapon,
                .timeoutSeconds = _equipped.handlingSettings.equipVisualBridgeTimeoutSeconds,
                .blendSeconds = _equipped.handlingSettings.equipVisualBridgeBlendSeconds,
            });
    }

    bool PhysicsInteraction::dropSecondaryWeaponToWorld(const PhysicsFrameContext& frame)
    {
        using equipped_weapon_drop_policy::Mode;
        const auto mode = equipped_weapon_drop_policy::fromSetting(g_rockConfig.rockWeaponDropMode);
        if (!frame.worldReady || !frame.hknpWorld || frame.menuBlocked || mode == Mode::Off) return false;
        native_secondary_presentation::DropPose drop{};
        if (const char* refused = native_secondary_presentation::tryGetDropPose(drop)) {
            ROCK_LOG_WARN(Weapon, "AKIMBO second weapon drop retained: {}", refused);
            return false;
        }
        const bool left = drop.left;
        const auto handIndex = left ? 1u : 0u;
        Hand& hand = left ? _leftHand : _rightHand;
        const auto& handInput = left ? frame.left : frame.right;
        const auto sourceHand = left ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right;
        // The hand carrying the second weapon is blocked for ordinary grabs by
        // that weapon itself; any other blocker refuses the handoff.
        const auto otherBlockers = forceGrabHandBlockerMask(hand, left, handInput.disabled, true) &
            ~(static_cast<std::uint32_t>(force_grab_policy::HandBlocker::EquippedWeapon) |
                static_cast<std::uint32_t>(force_grab_policy::HandBlocker::OffhandWeapon));
        if (otherBlockers != 0 || !hasAvailableEquippedWeaponDropHandoff() || _forceGrab.pendingCommits[handIndex].active) {
            ROCK_LOG_WARN(Weapon, "AKIMBO second weapon drop blocked: hand={} blockers={:#x} handoffCapacity={} pendingCommit={}",
                left ? "left" : "right", otherBlockers, hasAvailableEquippedWeaponDropHandoff(), _forceGrab.pendingCommits[handIndex].active);
            f4vr::showNotification("ROCK: Cannot drop weapon - hand or transfer queue is busy.");
            return false;
        }
        const bool toggleDrop = mode == Mode::ToggleDrop;
        RE::NiPointer<RE::NiAVObject> dropVisualModel(toggleDrop ? drop.model : nullptr);
        const auto dropVisualInWeapon = dropVisualModel ?
            transform_math::composeTransforms(transform_math::invertTransform(drop.weaponWorld), dropVisualModel->world) : RE::NiTransform{};
        // The second graph retires with its model; stop its actions first.
        native_secondary_actions::suspend();
        const auto dropResult = weapon_equip_transfer::dropEquippedWeaponFromPlayer(weapon_equip_transfer::EquippedDropInput{
            .dropLoc = drop.weaponWorld.translate,
            .dropRot = transform_math::matrixToReferenceEulerRadians<RE::NiMatrix3, RE::NiPoint3>(drop.weaponWorld.rotate),
            .hasDropLoc = true,
            .hasDropRot = true,
            .index = 1,
        });
        const bool committed = equipped_weapon_drop_policy::physicalDropCommitted(
            equipped_weapon_drop_policy::PhysicalDropCommitInput{
                .dropSucceeded = dropResult.success,
                .droppedReferenceUnavailable = dropResult.reason == weapon_equip_transfer::DropReason::DroppedReferenceUnavailable,
            });
        // The dropped native weapon appears at the last presented pose; retire
        // its hulls (53) and proxy (56) in this transaction, as the primary
        // drop does with 44 and 51.
        if (committed) {
            _secondaryDynamicWeaponCollision.retireAll(frame.bhkWorld);
            _secondaryWeaponCollision.destroyWeaponBody(frame.hknpWorld);
        }
        if (committed && dropResult.handle) {
            _forceGrab.pendingCommits[handIndex] = PendingForceGrabCommit{
                .active = true,
                .isLeft = left,
                .phase = dropResult.equippedSlotReleased ?
                    PendingForceGrabCommitPhase::WaitingForNativePlacement :
                    PendingForceGrabCommitPhase::EquippedSlotReleaseFailed,
                .targetHandle = dropResult.handle,
                .inventoryTransfer = true,
                .equippedWeaponDropMode = mode,
                .weaponGripPose = drop.pose,
                .maxDistanceGame = 96.0f,
            };
            if (toggleDrop) {
                _forceGrab.retainedWeaponGrabs[handIndex] = {
                    .inputState = transferred_weapon_grab_policy::State::AwaitInitialRelease,
                };
            }
            if (dropResult.equippedSlotReleased) {
                if (toggleDrop) {
                    _drop.visuals[handIndex].begin(std::move(dropVisualModel), dropVisualInWeapon, drop.pose, dropResult.droppedFormID);
                }
                armEquippedWeaponNativeHandoff(dropResult.handle, dropResult.droppedFormID, sourceHand,
                    WeaponCollision::ReleaseGeometrySnapshot{ .hasCapturedWeaponWorld = true, .capturedWeaponWorld = drop.weaponWorld });
            }
            // The releasing hand stops colliding briefly with what it let go of.
            setWeaponHandSuppression(frame.hknpWorld, left, WeaponHandReason::Drop, true);
            setWeaponHandSuppression(frame.hknpWorld, left, WeaponHandReason::Drop, false,
                g_rockConfig.rockGrabReleaseHandCollisionDelaySeconds);
        }
        if (committed) armTransitionSeamTrace("second-drop");
        ROCK_LOG_INFO(Weapon, "AKIMBO second weapon drop mode={} hand={} formID={:08X} dropped={:08X} committed={} slotReleased={} reason={}",
            static_cast<int>(mode), left ? "left" : "right", dropResult.formID, dropResult.droppedFormID, committed,
            dropResult.equippedSlotReleased, weapon_equip_transfer::dropReasonName(dropResult.reason));
        return committed;
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
        bipod_frozen_retirement::service();
        if (_bipodRetirementHandle) {
            const auto ref = _bipodRetirementHandle.get();
            if (!bipod_frozen_retirement::pending() ||
                (ref && bipod_frozen_retirement::frozen(ref->GetFormID()))) {
                _bipodRetirementVisual.release("bipod-native-placement-finished");
                _bipodRetirementHandle = {};
            } else {
                _bipodRetirementVisual.update({}, ref ? ref->Get3D() : nullptr);
            }
        }
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

        auto tryReadBodyIdentity = [&](const BodyRecord& record,
                                       equipped_weapon_drop_momentum::BodyIdentityKey& outIdentity) {
            const auto bodyId = record.bodyId;
            handoff.inspectedBodyId = bodyId;
            const auto live = havok_runtime::snapshotBody(
                frame.hknpWorld,
                RE::hknpBodyId{ bodyId });
            // Bits: valid/body/motion/shape/collision/owner/bodyId/system-instance.
            handoff.identityProofMask = (live.valid ? 1u : 0u) | (live.body ? 2u : 0u) | (live.motion ? 4u : 0u) |
                (live.valid && live.body && live.body->shape ? 8u : 0u) | (live.collisionObject ? 16u : 0u) |
                (live.ownerNode ? 32u : 0u) | (live.valid && live.body && live.body->bodyId.value == bodyId ? 64u : 0u);
            if (!live.valid || !live.body || !live.motion || !live.body->shape ||
                live.body->bodyId.value != bodyId) {
                return false;
            }
            auto* collisionObject = live.collisionObject;
            auto* ownerNode = live.ownerNode;
            if (!collisionObject && !ownerNode) {
                // Shared weapon systems include bodies with no reverse scene
                // link. This frame's exact-reference scan already enumerated
                // this body through its owning system; retain that provenance.
                // Never replace an existing but inconsistent native owner.
                collisionObject = record.collisionObject;
                ownerNode = havok_runtime::getOwnerNodeFromCollisionObject(collisionObject);
                if (ownerNode != record.owningNode) return false;
                handoff.identityProofMask |= 8192u;
            }
            if (!collisionObject || !ownerNode) return false;
            auto* physicsSystem =
                havok_runtime::getPhysicsSystemFromCollisionObject(collisionObject);
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
                .owningNodeIdentity = reinterpret_cast<std::uintptr_t>(ownerNode),
                .collisionObjectIdentity = reinterpret_cast<std::uintptr_t>(collisionObject),
                .physicsSystemInstanceIdentity =
                    reinterpret_cast<std::uintptr_t>(physicsSystemInstance),
            };
            return true;
        };
        auto tryBuildBodyIdentity = [&](const BodyRecord& record,
                                        equipped_weapon_drop_momentum::BodyIdentityKey& outIdentity) {
            if (!tryReadBodyIdentity(record, outIdentity)) {
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
        // Native bodies have settled and can complete the handoff now.
        // Saved-state geometry builds incrementally; LooseWeaponCollision
        // retains native collision until its generated replacements install.
        // Its work budget must not become a native placement timeout.
        ROCK_LOG_INFO(Weapon,
            "Equipped weapon native drop placement ready: ref={:08X} bodies={} motions={} elapsed={:.3f}s",
            handoff.droppedFormId, acceptedRecordCount, uniqueMotionRecordCount, handoff.elapsedSeconds);
        endHandoff("native-placement-ready", false, true);
    }
}
