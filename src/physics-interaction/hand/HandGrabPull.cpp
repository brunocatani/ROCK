// Far grabs: the dynamic pull that flies a selected object toward the hand
// (startDynamicPull, updateDynamicPull) and the pull-preparation state an
// object keeps while it is pulled, restored or handed to a physical drop when
// the pull ends without a grab.
#include "physics-interaction/hand/HandGrabInternal.h"

namespace rock
{

    namespace
    {
        std::uint32_t bodySetRejectCount(const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
            physics_body_classifier::BodyRejectReason reason)
        {
            const auto index = static_cast<std::size_t>(reason);
            if (index >= bodySet.diagnostics.rejectCounts.size()) {
                return 0;
            }
            return bodySet.diagnostics.rejectCounts[index];
        }

        held_object_drive_policy::HeldBodySetDriveDecision classifyHeldBodySetDrive(
            const object_physics_body_set::ObjectPhysicsBodySet& beforePrepBodySet,
            const object_physics_body_set::ObjectPhysicsBodySet& preparedBodySet, bool incompleteNativeScan,
            bool carriedRigidAssembly)
        {
            const auto uniqueMotionCount = preparedBodySet.forEachUniqueAcceptedMotion([](const auto&) {});
            return held_object_drive_policy::evaluateHeldBodySetDrive(held_object_drive_policy::HeldBodySetDriveInput{
                .acceptedBodyCount = static_cast<std::uint32_t>(preparedBodySet.acceptedCount()),
                .uniqueMotionCount = static_cast<std::uint32_t>(uniqueMotionCount),
                .rejectedFixedOrNonDynamicCount =
                    bodySetRejectCount(preparedBodySet, physics_body_classifier::BodyRejectReason::StaticMotion) +
                    bodySetRejectCount(preparedBodySet,
                        physics_body_classifier::BodyRejectReason::NotDynamicAfterActivePrep),
                .scanFailureCount =
                    beforePrepBodySet.diagnostics.scanFailures + preparedBodySet.diagnostics.scanFailures,
                .invalidPhysicsSystemCount = beforePrepBodySet.diagnostics.invalidPhysicsSystems +
                    preparedBodySet.diagnostics.invalidPhysicsSystems,
                .incompleteNativeScan = incompleteNativeScan,
                .carriedRigidAssembly = carriedRigidAssembly,
            });
        }

        void setHeldLinearVelocity(RE::hknpWorld* world, RE::hknpBodyId primaryBodyId,
            const std::vector<std::uint32_t>& heldBodyIds, const RE::NiPoint3& linearVelocity,
            float angularVelocityKeep = 1.0f, bool includeConnectedBodies = true)
        {
            setHeldVelocity(world, primaryBodyId, heldBodyIds, linearVelocity, RE::NiPoint3{}, false,
                angularVelocityKeep, includeConnectedBodies, false);
        }
    }

    void Hand::clearPullPrepTracking()
    {
        _pullActiveLifecycle.clear();
        _pullPrepHknpWorld = nullptr;
        _pullPrepRootNode = nullptr;
        _pullPrepRefr = nullptr;
        _pullPrepTargetKind = grab_target::Kind::LooseObject;
        _pullPrepOriginalMotionPropsId = 1;
        _pullPrepRestoreArmed = false;
    }

    void Hand::restorePullPrepIfActive(const char* context)
    {
        if (!_pullPrepRestoreArmed) {
            return;
        }

        const std::uint32_t primaryBodyId =
            _pulledPrimaryBodyId != INVALID_BODY_ID ? _pulledPrimaryBodyId : active_grab_body_lifecycle::kInvalidBodyId;

        restoreActiveGrabLifecycle(_pullPrepHknpWorld, _pullActiveLifecycle,
            _pullActiveLifecycle.restorePlanForFailure(), primaryBodyId, handName(),
            context ? context : "pull-prep-abandoned");
        if (_pullActiveLifecycle.hasIncompleteNativeScan()) {
            restoreIncompleteActivePrepRoot(_pullPrepRootNode, _pullPrepOriginalMotionPropsId, handName(),
                context ? context : "pull-prep-abandoned-incomplete-scan");
        }

        clearPullPrepTracking();
    }

    void Hand::finishPullPrepAsPhysicalDropIfActive(const char* context)
    {
        if (!_pullPrepRestoreArmed) {
            return;
        }

        const std::uint32_t primaryBodyId =
            _pulledPrimaryBodyId != INVALID_BODY_ID ? _pulledPrimaryBodyId : active_grab_body_lifecycle::kInvalidBodyId;
        const auto releaseRestorePolicy =
            active_grab_body_lifecycle::releaseRestorePolicyForTargetKind(_pullPrepTargetKind);
        const auto releasePlan = _pullActiveLifecycle.restorePlanForRelease(releaseRestorePolicy, _pullPrepTargetKind,
            active_grab_body_lifecycle::BodyReleaseIntent::PhysicalDrop);

        restoreActiveGrabLifecycle(_pullPrepHknpWorld, _pullActiveLifecycle, releasePlan, primaryBodyId, handName(),
            context ? context : "pull-physical-drop");

        if (_pullActiveLifecycle.hasIncompleteNativeScan()) {
            if (active_grab_body_lifecycle::shouldSkipIncompleteScanRootRestore(releasePlan,
                    _pullPrepOriginalMotionPropsId)) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand {}: skipped recursive root restore for converted loose-object physical pull drop "
                    "root='{}' motionProps={} preservedMotion={}",
                    handName(), context ? context : "pull-physical-drop", nodeDebugName(_pullPrepRootNode),
                    _pullPrepOriginalMotionPropsId, releasePlan.preservedConvertedMotionCount);
            } else {
                restoreIncompleteActivePrepRoot(_pullPrepRootNode, _pullPrepOriginalMotionPropsId, handName(),
                    context ? context : "pull-physical-drop-incomplete-scan");
            }
        }

        if (_pullPrepHknpWorld && primaryBodyId != INVALID_BODY_ID) {
            const auto releaseActivation = activateHeldObjectBodySet(_pullPrepHknpWorld, primaryBodyId, _pulledBodyIds);
            if (releaseActivation.failedActivationCount > 0) {
                ROCK_LOG_WARN(Hand,
                    "{} hand PULL physical-drop activation incomplete: primaryBody={} bodies={} activated={} failed={}",
                    handName(), primaryBodyId, releaseActivation.bodyCount, releaseActivation.activatedCount,
                    releaseActivation.failedActivationCount);
            }
        }

        clearPullPrepTracking();
    }

    bool Hand::consumePullPrepLifecycleForActiveGrab(RE::TESObjectREFR* refr,
        active_grab_body_lifecycle::BodyLifecycleSnapshot& outLifecycle)
    {
        if (!_pullPrepRestoreArmed || !refr || refr != _pullPrepRefr) {
            return false;
        }

        outLifecycle = _pullActiveLifecycle;
        ROCK_LOG_DEBUG(Hand,
            "{} hand consumed pull prep lifecycle for held grab: formID={:08X} bodies={} incompleteScan={}", handName(),
            refr->GetFormID(), outLifecycle.size(), outLifecycle.hasIncompleteNativeScan() ? "yes" : "no");
        clearPullPrepTracking();
        return true;
    }

    bool Hand::startDynamicPull(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform)
    {
        /*
         * Long-range pull remains a dynamic-object operation. ROCK promotes the
         * selected object tree through FO4VR's recursive wrappers, scans the
         * resulting dynamic body set, and then applies short-lived predicted
         * velocity to the accepted motions. It avoids a keyframed object path
         * that would conflict with the dynamic grab constraint used after arrival.
         */
        if (!world || _state != HandState::SelectionLocked || !_currentSelection.isValid() ||
            !_currentSelection.isFarSelection) {
            return false;
        }
        _pullDriveDecision = {};

        auto* selectedRef = _currentSelection.refr;
        if (!selectedRef || selectedRef->IsDeleted() || selectedRef->IsDisabled()) {
            clearSelectionState(true);
            return false;
        }

        if (!grab_target::canUseRockDynamicPull(_currentSelection.targetKind)) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL blocked: targetKind={} formID={:08X}; actor targets require the loot/SCISSORS path",
                handName(), grab_target::name(_currentSelection.targetKind), selectedRef->GetFormID());
            clearSelectionState(true);
            return false;
        }

        auto* rootNode = selectedRef->Get3D();
        if (!rootNode) {
            ROCK_LOG_WARN(Hand, "{} hand PULL failed: selected ref has no 3D root", handName());
            clearSelectionState(true);
            return false;
        }

        restorePullPrepIfActive("new-pull");

        auto* ownerCell = selectedRef->GetParentCell();
        auto* bhkWorld = ownerCell ? ownerCell->GetbhkWorld() : nullptr;
        if (!bhkWorld) {
            ROCK_LOG_WARN(Hand, "{} hand PULL failed: selected ref has no bhkWorld", handName());
            clearSelectionState(true);
            return false;
        }

        auto* selectedBase = selectedRef->GetObjectReference();
        const char* selectedType = selectedBase ? selectedBase->GetFormTypeString() : "???";
        auto selectedNameView = selectedBase ? RE::TESFullName::GetFullName(*selectedBase, false) : std::string_view{};
        const std::string selectedName =
            selectedNameView.empty() ? std::string("(unnamed)") : std::string(selectedNameView);
        const bool selectedIsWeapon = selectedType && std::string_view(selectedType) == "WEAP";

        std::uint16_t selectedOriginalMotionPropsId = 1;
        auto selectedOriginalMotionType = physics_body_classifier::BodyMotionType::Unknown;
        if (_currentSelection.bodyId.value != INVALID_BODY_ID) {
            havok_runtime::tryReadBodyMotionPropertiesId(world, _currentSelection.bodyId,
                selectedOriginalMotionPropsId);
            if (auto* selectedBody = havok_runtime::getBody(world, _currentSelection.bodyId)) {
                selectedOriginalMotionType = physics_body_classifier::motionTypeFromBodyFlags(selectedBody->flags);
            }
        }

        const auto scanOptions = makeActiveGrabBodyScanOptions(_currentSelection);

        object_physics_body_set::ObjectPhysicsBodySet beforePrepBodySet;
        bool beforePrepScanCacheHit =
            tryUseGrabAcquisitionBeforePrepCache(bhkWorld, world, _currentSelection, scanOptions, beforePrepBodySet);
        if (!beforePrepScanCacheHit) {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAcquisitionBodyScan);
            beforePrepBodySet =
                object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, selectedRef, scanOptions);
        }
        active_grab_body_lifecycle::BodyLifecycleSnapshot pullLifecycle;
        pullLifecycle.captureBeforeActivePrep(beforePrepBodySet);
        bool motionConverted = false;
        bool collisionEnabled = false;
        {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAcquisitionActivePrep);
            motionConverted = physics_recursive_wrappers::setMotionRecursive(rootNode,
                physics_recursive_wrappers::MotionPreset::Dynamic, true, true, true);
            collisionEnabled = physics_recursive_wrappers::enableCollisionRecursive(rootNode, true, true, true);
        }
        auto restoreFailedPullPrep = [&]() {
            restoreActiveGrabLifecycle(world, pullLifecycle, pullLifecycle.restorePlanForFailure(),
                _pulledPrimaryBodyId != INVALID_BODY_ID ? _pulledPrimaryBodyId : _currentSelection.bodyId.value,
                handName(), "failed-pull-setup");
            if (pullLifecycle.hasIncompleteNativeScan()) {
                restoreIncompleteActivePrepRoot(rootNode, selectedOriginalMotionPropsId, handName(),
                    "failed-pull-setup-incomplete-scan");
            } else if (active_object_prep_policy::shouldRestoreMotionAfterFailedActivePrep(motionConverted,
                           selectedOriginalMotionPropsId)) {
                physics_recursive_wrappers::setMotionRecursive(rootNode,
                    motionPresetFromMotionType(selectedOriginalMotionType, selectedOriginalMotionPropsId), true, true,
                    false);
            }
        };
        object_physics_body_set::ObjectPhysicsBodySet preparedBodySet;
        bool preparedBodySetPostPrepComplete = false;
        bool preparedScanCacheHit = tryBuildGrabAcquisitionPreparedBodySetFromCache(bhkWorld, world, _currentSelection,
            scanOptions, preparedBodySet, preparedBodySetPostPrepComplete);
        if (!preparedScanCacheHit || preparedBodySet.acceptedCount() == 0) {
            preparedScanCacheHit = false;
            preparedBodySetPostPrepComplete = true;
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAcquisitionBodyScan);
            preparedBodySet =
                object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, selectedRef, scanOptions);
        }
        pullLifecycle.markPreparedBodies(preparedBodySet);
        if (preparedScanCacheHit && !preparedBodySetPostPrepComplete) {
            pullLifecycle.markIncompleteNativeScan();
        }
        // Count-based loose ammo flies as one rigid object led by its visible
        // branch body; the arrival grab restores the relation again.
        const bool carriedAmmoAssembly = restoreCountBasedAmmoAssembly(world, count_based_ammo::resolve(selectedRef),
            preparedBodySet, "pull");
        _pullDriveDecision = classifyHeldBodySetDrive(beforePrepBodySet, preparedBodySet,
            pullLifecycle.hasIncompleteNativeScan(), carriedAmmoAssembly);
        ROCK_LOG_DEBUG(Hand,
            "{} hand PULL scan: type={} weapon={} name='{}' formID={:08X} seedBody={} beforeBodies={} afterBodies={} "
            "accepted={} rejected={} "
            "seeded={} scanSource={}/{} preparedComplete={} cachedBodyIds={} cacheHits={} scanFailures={} "
            "invalidSystems={} benignSkips={} foreignSkips={} unresolvedAccepted={} unresolvedSkips={} "
            "collisionObjects={} visitedNodes={} driveMode={} driveReason={} linearScope={} angularScope={}",
            handName(), selectedType ? selectedType : "???", selectedIsWeapon ? "yes" : "no", selectedName,
            selectedRef->GetFormID(), scanOptions.seedBodyId, beforePrepBodySet.records.size(),
            preparedBodySet.records.size(), preparedBodySet.acceptedCount(), preparedBodySet.rejectedCount(),
            preparedBodySet.diagnostics.seedBodiesAdded, beforePrepScanCacheHit ? "cache" : "direct",
            preparedScanCacheHit ? "cache" : "direct", preparedBodySetPostPrepComplete ? "yes" : "no",
            preparedBodySet.diagnostics.cachedBodyIds, preparedBodySet.diagnostics.cachedScanHits,
            preparedBodySet.diagnostics.scanFailures, preparedBodySet.diagnostics.invalidPhysicsSystems,
            preparedBodySet.diagnostics.benignScanSkips, preparedBodySet.diagnostics.foreignRefBodySkips,
            preparedBodySet.diagnostics.unresolvedRefBodiesAccepted, preparedBodySet.diagnostics.unresolvedRefBodySkips,
            preparedBodySet.diagnostics.collisionObjects, preparedBodySet.diagnostics.visitedNodes,
            held_object_drive_policy::modeName(_pullDriveDecision.mode), _pullDriveDecision.reason,
            _pullDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
            _pullDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly");
        const RE::NiPoint3 grabPivotAForPrimaryChoice = computeGrabPivotAWorld(world, handWorldTransform);
        const RE::NiPoint3 primaryChoiceTarget =
            _currentSelection.hasHitPoint ? _currentSelection.hitPointWorld : grabPivotAForPrimaryChoice;
        const auto primaryChoice = carriedAmmoAssembly
            ? object_physics_body_set::PrimaryBodyChoice{ .bodyId = _ammoCarry.primaryBodyId,
                  .reason = object_physics_body_set::PrimaryBodyChoiceReason::SurfaceOwnerAccepted }
            : preparedBodySet.choosePrimaryBody(_currentSelection.bodyId.value,
                  object_physics_body_set::PurePoint3{ primaryChoiceTarget });

        if (primaryChoice.bodyId == INVALID_BODY_ID) {
            const auto* rejectedBody = diagnosticRejectedBodyRecord(preparedBodySet, _currentSelection.bodyId.value);
            ROCK_LOG_WARN(Hand,
                "{} hand PULL failed: no accepted dynamic body after recursive prep formID={:08X} beforeBodies={} "
                "afterBodies={} accepted={} rejected={} "
                "seeded={} scanFailures={} invalidSystems={} benignSkips={} foreignSkips={} unresolvedAccepted={} "
                "unresolvedSkips={} rejectReason={} rejectBody={} rejectLayer={} rejectMotion={} rejectFlags=0x{:08X} "
                "rejectMotionProps={} setMotion={} enableCollision={}",
                handName(), selectedRef->GetFormID(), beforePrepBodySet.records.size(), preparedBodySet.records.size(),
                preparedBodySet.acceptedCount(), preparedBodySet.rejectedCount(),
                preparedBodySet.diagnostics.seedBodiesAdded, preparedBodySet.diagnostics.scanFailures,
                preparedBodySet.diagnostics.invalidPhysicsSystems, preparedBodySet.diagnostics.benignScanSkips,
                preparedBodySet.diagnostics.foreignRefBodySkips,
                preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
                preparedBodySet.diagnostics.unresolvedRefBodySkips,
                rejectedBody ? physics_body_classifier::rejectReasonName(rejectedBody->rejectReason) : "none",
                rejectedBody ? rejectedBody->bodyId : INVALID_BODY_ID, rejectedBody ? rejectedBody->collisionLayer : 0,
                rejectedBody ? bodyMotionTypeName(rejectedBody->motionType) : "none",
                rejectedBody ? rejectedBody->bodyFlags : 0, rejectedBody ? rejectedBody->motionPropertiesId : 0,
                motionConverted ? "ok" : "failed", collisionEnabled ? "ok" : "failed");
            restoreFailedPullPrep();
            clearSelectionState(true);
            _pullDriveDecision = {};
            return false;
        }

        _pulledPrimaryBodyId = primaryChoice.bodyId;
        _pulledBodyIds = preparedBodySet.acceptedBodyIds();
        if (_pulledBodyIds.empty()) {
            _pulledBodyIds.push_back(_pulledPrimaryBodyId);
        }
        armPullCatchIntent(selectedRef, _pulledPrimaryBodyId, _currentSelection.targetKind);

        for (const auto bodyId : _pulledBodyIds) {
            physics_recursive_wrappers::activateBody(world, bodyId);
        }

        _currentSelection.bodyId = RE::hknpBodyId{ _pulledPrimaryBodyId };
        if (!_currentSelection.visualNode) {
            _currentSelection.visualNode = rootNode;
        }
        if (!_currentSelection.hitNode) {
            _currentSelection.hitNode = getOwnerNodeFromBody(world, RE::hknpBodyId{ _pulledPrimaryBodyId });
        }

        auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ _pulledPrimaryBodyId });
        if (!motion) {
            ROCK_LOG_WARN(Hand, "{} hand PULL failed: primary dynamic body has no motion bodyId={}", handName(),
                _pulledPrimaryBodyId);
            restoreFailedPullPrep();
            clearPullRuntimeState();
            clearSelectionState(true);
            return false;
        }

        const auto grabPivotWorld = computeGrabPivotAWorld(world, handWorldTransform);
        const auto grabPivotHavok = niPointToHkVector(grabPivotWorld);
        RE::NiTransform primaryBodyWorld{};
        const bool hasPrimaryBodyWorld =
            tryGetBodyWorldTransform(world, RE::hknpBodyId{ _pulledPrimaryBodyId }, primaryBodyWorld);
        RE::NiPoint3 ownerNodePosition{};
        bool hasOwnerNode = false;
        if (auto* ownerNode = getOwnerNodeFromBody(world, RE::hknpBodyId{ _pulledPrimaryBodyId })) {
            ownerNodePosition = ownerNode->world.translate;
            hasOwnerNode = true;
        }
        const auto selectedPointAnchor = body_frame::chooseSelectionDistanceAnchor(_currentSelection.hasHitPoint,
            _currentSelection.hitPointWorld, hasPrimaryBodyWorld, primaryBodyWorld.translate, hasOwnerNode,
            ownerNodePosition, true, hkVectorToNiPoint(motion->position), grabPivotWorld);
        const auto selectedPointWorld = selectedPointAnchor.position;
        const auto selectedPointHavok = niPointToHkVector(selectedPointWorld);
        _pullPointOffsetHavok = RE::NiPoint3{
            selectedPointHavok.x - motion->position.x,
            selectedPointHavok.y - motion->position.y,
            selectedPointHavok.z - motion->position.z,
        };
        /*
         * Far grabs pull the object CENTER: a ray hit at selection range is aim
         * noise, not grip intent, and a fixed world offset cannot rotate with
         * the presentation servo anyway. Tracking the COM (zero offset) drifts
         * with nothing. Arrival then overwrites the selection hit point with
         * the tracked center, so capture seeds from the middle of the object
         * and the seat (mesh point nearest the palm, seat depth stop) settles
         * the surface onto the palm.
         */
        if (g_rockConfig.rockPullToObjectCenterEnabled) {
            _pullPointOffsetHavok = {};
        }

        /*
         * Long-object presentation capture: one mesh extraction + PCA at pull
         * start. The axis is frozen in primary-body local space so the flight
         * servo in updateDynamicPull can re-derive it from the live body pose
         * without touching nodes per frame.
         */
        _pullPresentationAxisBodyLocal = {};
        _pullPresentationElongationRatio = 0.0f;
        _pullPresentationValid = false;
        if (g_rockConfig.rockPullLongAxisPresentationEnabled && hasPrimaryBodyWorld) {
            RE::NiAVObject* presentationMeshNode =
                _currentSelection.visualNode ? _currentSelection.visualNode : rootNode;
            if (presentationMeshNode) {
                std::vector<TriangleData> presentationTriangles;
                std::vector<GrabSurfaceTriangleData> presentationSurfaceTriangles;
                MeshExtractionStats presentationMeshStats;
                extractAllSurfaceTriangles(presentationMeshNode, presentationTriangles, presentationSurfaceTriangles,
                    (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth), &presentationMeshStats, false);
                const auto longAxis = computeGrabMeshLongAxis(presentationTriangles);
                if (longAxis.valid && longAxis.elongationRatio >= g_rockConfig.rockPullPresentationMinElongationRatio) {
                    const RE::NiPoint3 axisBodyLocal =
                        normalizeOrZero(transform_math::worldVectorToLocal(primaryBodyWorld, longAxis.axisWorld));
                    if (lengthSquared(axisBodyLocal) > 0.000001f) {
                        _pullPresentationAxisBodyLocal = axisBodyLocal;
                        _pullPresentationElongationRatio = longAxis.elongationRatio;
                        _pullPresentationValid = true;
                    }
                }
                ROCK_LOG_DEBUG(Hand,
                    "{} hand PULL presentation axis: valid={} elongation={:.2f} tris={} reason={} "
                    "axisLocal=({:.3f},{:.3f},{:.3f})",
                    handName(), _pullPresentationValid ? "yes" : "no", longAxis.elongationRatio, longAxis.triangleCount,
                    longAxis.reason, _pullPresentationAxisBodyLocal.x, _pullPresentationAxisBodyLocal.y,
                    _pullPresentationAxisBodyLocal.z);
            }
        }

        const RE::NiPoint3 objectPointHavok{
            motion->position.x + _pullPointOffsetHavok.x,
            motion->position.y + _pullPointOffsetHavok.y,
            motion->position.z + _pullPointOffsetHavok.z,
        };
        const RE::NiPoint3 grabPivotPointHavok{ grabPivotHavok.x, grabPivotHavok.y, grabPivotHavok.z };
        const float pullDistance = (grabPivotPointHavok - objectPointHavok).Length();

        _pullElapsedSeconds = 0.0f;
        _pullDurationSeconds = pull_motion_math::computePullDurationSeconds(pullDistance);
        _pullTargetHavok = {};
        _pullHasTarget = false;
        _pullHasPreviousObjectPoint = false;
        const auto transition = applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::BeginPull });
        if (!transition.accepted) {
            clearPullRuntimeState(false, "begin-pull-rejected");
            clearPullCatchIntent("beginPullRejected");
            restoreFailedPullPrep();
            return false;
        }
        _pullActiveLifecycle = pullLifecycle;
        _pullPrepHknpWorld = world;
        _pullPrepRootNode = rootNode;
        _pullPrepRefr = selectedRef;
        _pullPrepTargetKind = _currentSelection.targetKind;
        _pullPrepOriginalMotionPropsId = selectedOriginalMotionPropsId;
        _pullPrepRestoreArmed = true;
        stopSelectionHighlight();

        ROCK_LOG_INFO(Hand,
            "{} hand PULL start: type={} weapon={} formID={:08X} primaryBody={} bodyCount={} driveMode={} "
            "linearScope={} angularScope={} seeded={} scanFailures={} invalidSystems={} benignSkips={} "
            "unresolvedAccepted={} distanceHk={:.3f} duration={:.3f}s setMotion={} enableCollision={}",
            handName(), selectedType ? selectedType : "???", selectedIsWeapon ? "yes" : "no", selectedRef->GetFormID(),
            _pulledPrimaryBodyId, _pulledBodyIds.size(), held_object_drive_policy::modeName(_pullDriveDecision.mode),
            _pullDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
            _pullDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly",
            preparedBodySet.diagnostics.seedBodiesAdded, preparedBodySet.diagnostics.scanFailures,
            preparedBodySet.diagnostics.invalidPhysicsSystems, preparedBodySet.diagnostics.benignScanSkips,
            preparedBodySet.diagnostics.unresolvedRefBodiesAccepted, pullDistance, _pullDurationSeconds,
            motionConverted ? "ok" : "failed", collisionEnabled ? "ok" : "failed");
        return true;
    }

    bool Hand::updateDynamicPull(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, float deltaTime)
    {
        if (_state != HandState::Pulled || !world || _pulledPrimaryBodyId == INVALID_BODY_ID ||
            !_currentSelection.isValid()) {
            return false;
        }

        auto* selectedRef = _currentSelection.refr;
        if (!selectedRef || selectedRef->IsDeleted() || selectedRef->IsDisabled()) {
            ROCK_LOG_DEBUG(Hand, "{} hand PULL invalidated before arrival: selected ref unavailable", handName());
            clearSelectionState(true);
            return false;
        }

        auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ _pulledPrimaryBodyId });
        if (!motion) {
            ROCK_LOG_DEBUG(Hand, "{} hand PULL invalidated before arrival: primary motion missing bodyId={}",
                handName(), _pulledPrimaryBodyId);
            clearSelectionState(true);
            return false;
        }

        const auto grabPivotWorld = computeGrabPivotAWorld(world, handWorldTransform);
        const auto grabPivotHavokVector = niPointToHkVector(grabPivotWorld);
        const RE::NiPoint3 grabPivotHavok{ grabPivotHavokVector.x, grabPivotHavokVector.y, grabPivotHavokVector.z };
        const RE::NiPoint3 objectPointHavok{
            motion->position.x + _pullPointOffsetHavok.x,
            motion->position.y + _pullPointOffsetHavok.y,
            motion->position.z + _pullPointOffsetHavok.z,
        };

        const float distanceGameUnits = (grabPivotHavok - objectPointHavok).Length() * havokToGameScale();
        _currentSelection.distance = distanceGameUnits;
        const float configuredAutoGrabDistance = pull_motion_math::kAutoGrabDistanceGameUnits;
        const float nearConvergeDistance =
            (std::max)(configuredAutoGrabDistance, g_rockConfig.rockGrabNearConvergeDistanceGameUnits);
        const float pocketBand = (std::max)(0.0f, g_rockConfig.rockGrabPocketRadiusGameUnits);
        const float arrivalDistance = (std::max)(configuredAutoGrabDistance,
            (std::min)(nearConvergeDistance, configuredAutoGrabDistance + pocketBand));
        const RE::NiPoint3 closestApproachHavok = _pullHasPreviousObjectPoint
            ? pull_motion_math::closestPointOnSegment(_pullPreviousObjectPointHavok, objectPointHavok, grabPivotHavok)
            : objectPointHavok;
        const float closestApproachGameUnits = (grabPivotHavok - closestApproachHavok).Length() * havokToGameScale();
        _pullPreviousObjectPointHavok = objectPointHavok;
        _pullHasPreviousObjectPoint = true;
        RE::NiPoint3 arrivalPointHavok = objectPointHavok;
        if (distanceGameUnits > arrivalDistance && closestApproachGameUnits <= arrivalDistance) {
            // The object crossed the catch sphere between frames. Put it back at
            // its closest approach, where a faster frame would have caught it.
            auto passBack = makeIdentityTransform();
            passBack.translate = (closestApproachHavok - objectPointHavok) * havokToGameScale();
            const auto moved = moveBodySetRigidly(world, _pulledPrimaryBodyId, _pulledBodyIds, passBack);
            if (moved != 0)
                arrivalPointHavok = closestApproachHavok;
            ROCK_LOG_INFO(Hand,
                "{} hand PULL crossed the catch sphere between frames dist={:.1f} closest={:.1f} arrival={:.1f} "
                "movedBack={}",
                handName(), distanceGameUnits, closestApproachGameUnits, arrivalDistance, moved);
        }
        if (distanceGameUnits <= arrivalDistance || closestApproachGameUnits <= arrivalDistance) {
            _currentSelection.isFarSelection = false;
            _currentSelection.hitPointWorld = RE::NiPoint3{
                arrivalPointHavok.x * havokToGameScale(),
                arrivalPointHavok.y * havokToGameScale(),
                arrivalPointHavok.z * havokToGameScale(),
            };
            _currentSelection.hasHitPoint = true;
            markPullCatchIntentArrived();
            _pullTargetHavok = {};
            _pullElapsedSeconds = 0.0f;
            _pullDurationSeconds = 0.0f;
            _pullHasTarget = false;
            applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::PullArrivedClose });
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL arrived -> close grab window dist={:.1f} arrival={:.1f} configuredAuto={:.1f} "
                "near={:.1f} pocketBand={:.1f}",
                handName(), distanceGameUnits, arrivalDistance, configuredAutoGrabDistance, nearConvergeDistance,
                pocketBand);
            return true;
        }

        const auto motionResult =
            pull_motion_math::computePullMotion<RE::NiPoint3>(pull_motion_math::PullMotionInput<RE::NiPoint3>{
                .handHavok = grabPivotHavok,
                .objectPointHavok = objectPointHavok,
                .previousTargetHavok = _pullTargetHavok,
                .elapsedSeconds = _pullElapsedSeconds,
                .durationSeconds = _pullDurationSeconds,
                .hasPreviousTarget = _pullHasTarget,
            });

        _pullElapsedSeconds += (std::max)(0.0f, deltaTime);
        if (motionResult.refreshTarget) {
            _pullTargetHavok = motionResult.targetHavok;
            _pullHasTarget = true;
        }

        if (motionResult.expired) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL owner expired before arrival dist={:.1f} arrival={:.1f} elapsed={:.3f}s duration={:.3f}s "
                "ownerGrace={:.3f}s",
                handName(), distanceGameUnits, arrivalDistance, _pullElapsedSeconds, _pullDurationSeconds,
                pull_motion_math::kOwnerGraceSeconds);
            finishPullPrepAsPhysicalDropIfActive("pull-owner-expired");
            clearSelectionState(true);
            return false;
        }

        if (!motionResult.applyVelocity) {
            ROCK_LOG_SAMPLE_DEBUG(Hand, g_rockConfig.rockLogSampleMilliseconds,
                "{} hand PULL holding owner after velocity window dist={:.1f} arrival={:.1f} elapsed={:.3f}s "
                "duration={:.3f}s ownerGrace={:.3f}s",
                handName(), distanceGameUnits, arrivalDistance, _pullElapsedSeconds, _pullDurationSeconds,
                pull_motion_math::kOwnerGraceSeconds);
            for (const auto bodyId : _pulledBodyIds) {
                physics_recursive_wrappers::activateBody(world, bodyId);
            }
            return false;
        }

        /*
         * Long-object presentation (flight only): servo the mesh principal
         * axis toward the hand's cross-palm (thumb->pinky) line while the pull
         * drive owns the object, so long props arrive oriented for the grab.
         * The angular velocity is SET each frame from the remaining angle
         * (kinematic servo with exponential decay), so it cannot overshoot.
         * This path ends at pull arrival, before capture freezes the relation;
         * held objects keep the no-rotate rule.
         */
        RE::NiPoint3 presentationAngularVelocity{};
        bool presentationActive = false;
        if (_pullPresentationValid && g_rockConfig.rockPullLongAxisPresentationEnabled) {
            RE::NiTransform pulledBodyWorld{};
            if (tryGetBodyWorldTransform(world, RE::hknpBodyId{ _pulledPrimaryBodyId }, pulledBodyWorld)) {
                const RE::NiPoint3 currentAxisWorld = normalizeOrZero(
                    transform_math::localVectorToWorld(pulledBodyWorld, _pullPresentationAxisBodyLocal));
                /*
                 * The grip line is not pure cross-palm Z: the thumb sits in the
                 * way, so the natural long-object hold tilts a few degrees
                 * toward the fingers-forward X axis (Bruno-tuned via INI).
                 */
                const RE::NiPoint3 crossPalmWorld =
                    transformHandspaceDirection(handWorldTransform, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }, _isLeft);
                const RE::NiPoint3 fingerForwardWorld =
                    transformHandspaceDirection(handWorldTransform, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, _isLeft);
                RE::NiPoint3 targetAxisWorld = grab_three_phase::buildGripPresentationAxisTowardFingertips(
                    crossPalmWorld, fingerForwardWorld, g_rockConfig.rockPullPresentationGripAxisTiltDegrees);
                if (lengthSquared(currentAxisWorld) > 0.000001f && lengthSquared(targetAxisWorld) > 0.000001f) {
                    // The mesh axis has no sign: always rotate toward the nearest hemisphere.
                    if (dotProduct(currentAxisWorld, targetAxisWorld) < 0.0f) {
                        targetAxisWorld = RE::NiPoint3{ -targetAxisWorld.x, -targetAxisWorld.y, -targetAxisWorld.z };
                    }
                    const RE::NiPoint3 rotationAxis = crossProduct(currentAxisWorld, targetAxisWorld);
                    const float sinAngle = std::sqrt((std::max)(0.0f, lengthSquared(rotationAxis)));
                    const float cosAngle = std::clamp(dotProduct(currentAxisWorld, targetAxisWorld), -1.0f, 1.0f);
                    const float angleRadians = std::atan2(sinAngle, cosAngle);
                    if (sinAngle > 0.000001f && angleRadians > 0.005f) {
                        const float angularSpeed = (std::min)(angleRadians *
                                (std::max)(0.0f, g_rockConfig.rockPullPresentationAngularGainPerSecond),
                            (std::max)(0.0f, g_rockConfig.rockPullPresentationMaxAngularSpeedRadiansPerSecond));
                        const float invSin = 1.0f / sinAngle;
                        presentationAngularVelocity = RE::NiPoint3{
                            rotationAxis.x * invSin * angularSpeed,
                            rotationAxis.y * invSin * angularSpeed,
                            rotationAxis.z * invSin * angularSpeed,
                        };
                        presentationActive = angularSpeed > 0.0f;
                    }
                }
            }
        }
        if (presentationActive) {
            setHeldVelocity(world, RE::hknpBodyId{ _pulledPrimaryBodyId }, _pulledBodyIds, motionResult.velocityHavok,
                presentationAngularVelocity, true, 1.0f, _pullDriveDecision.includeConnectedLinearVelocity,
                _pullDriveDecision.includeConnectedAngularVelocity);
        } else {
            setHeldLinearVelocity(world, RE::hknpBodyId{ _pulledPrimaryBodyId }, _pulledBodyIds,
                motionResult.velocityHavok, pull_motion_math::angularVelocityKeepForDamping(deltaTime),
                _pullDriveDecision.includeConnectedLinearVelocity);
        }
        for (const auto bodyId : _pulledBodyIds) {
            physics_recursive_wrappers::activateBody(world, bodyId);
        }

        return false;
    }

}
