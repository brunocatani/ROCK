#include "physics-interaction/hand/Hand.h"

/*
 * Dynamic PULL: the flight phase that brings a distant object to the hand before
 * a grab can seat. Lifecycle order in this file is prep, start, update, then
 * either catch (acquisition takes over) or restore.
 *
 * Pull prep and grab acquisition share the same body-set scan and active-prep
 * sequence, so an abandoned pull must restore exactly what it changed.
 * restorePullPrepIfActive and finishPullPrepAsPhysicalDropIfActive are the two
 * exits, and both are safe to call when no prep is active.
 *
 * nativeVRGrabDrop is a native FO4VR entry point. Its RVA lives in
 * native/havok/HavokOffsets.h, not inline here.
 */

#include "physics-interaction/hand/grab/HandGrabBodySetRuntime.h"
#include "physics-interaction/hand/grab/HandGrabMath.h"
#include "physics-interaction/hand/grab/HandGrabPivotAuthority.h"
#include "physics-interaction/hand/grab/HandGrabSupportModel.h"
#include "physics-interaction/hand/grab/HandGrabTrace.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/native/query/PhysicsRecursiveWrappers.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/TransformMath.h"
#include "RockConfig.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace rock
{
    using namespace hand_grab_detail;

    void hand_grab_detail::nativeVRGrabDrop(void* playerChar, int handIndex)
    {
        typedef void func_t(void*, int, std::uint64_t);
        static REL::Relocation<func_t> func{ REL::Offset(offsets::kFunc_NativeVRGrabDrop) };
        func(playerChar, handIndex, 0);
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
    
        restoreActiveGrabLifecycle(_pullPrepHknpWorld,
            _pullActiveLifecycle,
            _pullActiveLifecycle.restorePlanForFailure(),
            primaryBodyId,
            handName(),
            context ? context : "pull-prep-abandoned");
        if (_pullActiveLifecycle.hasIncompleteNativeScan()) {
            restoreIncompleteActivePrepRoot(
                _pullPrepRootNode,
                _pullPrepOriginalMotionPropsId,
                handName(),
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
        const auto releasePlan = _pullActiveLifecycle.restorePlanForRelease(
            releaseRestorePolicy,
            _pullPrepTargetKind,
            active_grab_body_lifecycle::BodyReleaseIntent::PhysicalDrop);
    
        restoreActiveGrabLifecycle(_pullPrepHknpWorld,
            _pullActiveLifecycle,
            releasePlan,
            primaryBodyId,
            handName(),
            context ? context : "pull-physical-drop");
    
        if (_pullActiveLifecycle.hasIncompleteNativeScan()) {
            if (active_grab_body_lifecycle::shouldSkipIncompleteScanRootRestore(releasePlan, _pullPrepOriginalMotionPropsId)) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand {}: skipped recursive root restore for converted loose-object physical pull drop root='{}' motionProps={} preservedMotion={}",
                    handName(),
                    context ? context : "pull-physical-drop",
                    nodeDebugName(_pullPrepRootNode),
                    _pullPrepOriginalMotionPropsId,
                    releasePlan.preservedConvertedMotionCount);
            } else {
                restoreIncompleteActivePrepRoot(
                    _pullPrepRootNode,
                    _pullPrepOriginalMotionPropsId,
                    handName(),
                    context ? context : "pull-physical-drop-incomplete-scan");
            }
        }
    
        if (_pullPrepHknpWorld && primaryBodyId != INVALID_BODY_ID) {
            const auto releaseActivation = activateHeldObjectBodySet(_pullPrepHknpWorld, primaryBodyId, _pulledBodyIds);
            if (releaseActivation.failedActivationCount > 0) {
                ROCK_LOG_WARN(Hand,
                    "{} hand PULL physical-drop activation incomplete: primaryBody={} bodies={} activated={} failed={}",
                    handName(),
                    primaryBodyId,
                    releaseActivation.bodyCount,
                    releaseActivation.activatedCount,
                    releaseActivation.failedActivationCount);
            }
        }
    
        clearPullPrepTracking();
    }
    
    bool Hand::consumePullPrepLifecycleForActiveGrab(RE::TESObjectREFR* refr, active_grab_body_lifecycle::BodyLifecycleSnapshot& outLifecycle)
    {
        if (!_pullPrepRestoreArmed || !refr || refr != _pullPrepRefr) {
            return false;
        }
    
        outLifecycle = _pullActiveLifecycle;
        ROCK_LOG_DEBUG(Hand,
            "{} hand consumed pull prep lifecycle for held grab: formID={:08X} bodies={} incompleteScan={}",
            handName(),
            refr->GetFormID(),
            outLifecycle.size(),
            outLifecycle.hasIncompleteNativeScan() ? "yes" : "no");
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
        if (!world || _state != HandState::SelectionLocked || !_currentSelection.isValid() || !_currentSelection.isFarSelection) {
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
                handName(),
                grab_target::name(_currentSelection.targetKind),
                selectedRef->GetFormID());
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
        const std::string selectedName = selectedNameView.empty() ? std::string("(unnamed)") : std::string(selectedNameView);
        const bool selectedIsWeapon = selectedType && std::string_view(selectedType) == "WEAP";
    
        std::uint16_t selectedOriginalMotionPropsId = 1;
        auto selectedOriginalMotionType = physics_body_classifier::BodyMotionType::Unknown;
        if (_currentSelection.bodyId.value != INVALID_BODY_ID) {
            havok_runtime::tryReadBodyMotionPropertiesId(world, _currentSelection.bodyId, selectedOriginalMotionPropsId);
            if (auto* selectedBody = havok_runtime::getBody(world, _currentSelection.bodyId)) {
                selectedOriginalMotionType = physics_body_classifier::motionTypeFromBodyFlags(selectedBody->flags);
            }
        }
    
        active_grab_body_lifecycle::BodyLifecycleSnapshot pullLifecycle;
        ActiveGrabBodySetPrep bodySetPrep{};
        prepareActiveGrabBodySet(
            bhkWorld,
            world,
            _currentSelection,
            rootNode,
            false,
            true,
            true,
            pullLifecycle,
            bodySetPrep);
        const auto& beforePrepBodySet = bodySetPrep.beforePrepBodySet;
        const auto& preparedBodySet = bodySetPrep.preparedBodySet;
        const bool beforePrepScanCacheHit = bodySetPrep.beforePrepScanCacheHit;
        const bool preparedScanCacheHit = bodySetPrep.preparedScanCacheHit;
        const bool preparedBodySetPostPrepComplete = bodySetPrep.preparedBodySetPostPrepComplete;
        const bool motionConverted = bodySetPrep.motionConverted;
        const bool collisionEnabled = bodySetPrep.collisionEnabled;
        _pullDriveDecision = bodySetPrep.driveDecision;
        auto restoreFailedPullPrep = [&]() {
            restoreActiveGrabLifecycle(world,
                pullLifecycle,
                pullLifecycle.restorePlanForFailure(),
                _pulledPrimaryBodyId != INVALID_BODY_ID ? _pulledPrimaryBodyId : _currentSelection.bodyId.value,
                handName(),
                "failed-pull-setup");
            if (pullLifecycle.hasIncompleteNativeScan()) {
                restoreIncompleteActivePrepRoot(rootNode, selectedOriginalMotionPropsId, handName(), "failed-pull-setup-incomplete-scan");
            } else if (active_object_prep_policy::shouldRestoreMotionAfterFailedActivePrep(motionConverted, selectedOriginalMotionPropsId)) {
                physics_recursive_wrappers::setMotionRecursive(
                    rootNode,
                    motionPresetFromMotionType(selectedOriginalMotionType, selectedOriginalMotionPropsId),
                    true,
                    true,
                    false);
            }
        };
        ROCK_LOG_DEBUG(Hand,
            "{} hand PULL scan: type={} weapon={} name='{}' formID={:08X} seedBody={} beforeBodies={} afterBodies={} accepted={} rejected={} "
            "seeded={} scanSource={}/{} preparedComplete={} cachedBodyIds={} cacheHits={} scanFailures={} invalidSystems={} benignSkips={} foreignSkips={} unresolvedAccepted={} unresolvedSkips={} collisionObjects={} visitedNodes={} driveMode={} driveReason={} linearScope={} angularScope={}",
            handName(),
            selectedType ? selectedType : "???",
            selectedIsWeapon ? "yes" : "no",
            selectedName,
            selectedRef->GetFormID(),
            bodySetPrep.seedBodyId,
            beforePrepBodySet.records.size(),
            preparedBodySet.records.size(),
            preparedBodySet.acceptedCount(),
            preparedBodySet.rejectedCount(),
            preparedBodySet.diagnostics.seedBodiesAdded,
            beforePrepScanCacheHit ? "cache" : "direct",
            preparedScanCacheHit ? "cache" : "direct",
            preparedBodySetPostPrepComplete ? "yes" : "no",
            preparedBodySet.diagnostics.cachedBodyIds,
            preparedBodySet.diagnostics.cachedScanHits,
            preparedBodySet.diagnostics.scanFailures,
            preparedBodySet.diagnostics.invalidPhysicsSystems,
            preparedBodySet.diagnostics.benignScanSkips,
            preparedBodySet.diagnostics.foreignRefBodySkips,
            preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
            preparedBodySet.diagnostics.unresolvedRefBodySkips,
            preparedBodySet.diagnostics.collisionObjects,
            preparedBodySet.diagnostics.visitedNodes,
            held_object_drive_policy::modeName(_pullDriveDecision.mode),
            _pullDriveDecision.reason,
            _pullDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
            _pullDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly");
        const RE::NiPoint3 grabPivotAForPrimaryChoice = computeGrabPivotAWorld(world, handWorldTransform);
        const RE::NiPoint3 primaryChoiceTarget = _currentSelection.hasHitPoint ? _currentSelection.hitPointWorld : grabPivotAForPrimaryChoice;
        const auto primaryChoice = preparedBodySet.choosePrimaryBody(_currentSelection.bodyId.value, object_physics_body_set::PurePoint3{ primaryChoiceTarget });
    
        if (primaryChoice.bodyId == INVALID_BODY_ID) {
            const auto* rejectedBody = diagnosticRejectedBodyRecord(preparedBodySet, _currentSelection.bodyId.value);
            ROCK_LOG_WARN(Hand,
                "{} hand PULL failed: no accepted dynamic body after recursive prep formID={:08X} beforeBodies={} afterBodies={} accepted={} rejected={} "
                "seeded={} scanFailures={} invalidSystems={} benignSkips={} foreignSkips={} unresolvedAccepted={} unresolvedSkips={} rejectReason={} rejectBody={} rejectLayer={} rejectMotion={} rejectFlags=0x{:08X} rejectMotionProps={} setMotion={} enableCollision={}",
                handName(),
                selectedRef->GetFormID(),
                beforePrepBodySet.records.size(),
                preparedBodySet.records.size(),
                preparedBodySet.acceptedCount(),
                preparedBodySet.rejectedCount(),
                preparedBodySet.diagnostics.seedBodiesAdded,
                preparedBodySet.diagnostics.scanFailures,
                preparedBodySet.diagnostics.invalidPhysicsSystems,
                preparedBodySet.diagnostics.benignScanSkips,
                preparedBodySet.diagnostics.foreignRefBodySkips,
                preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
                preparedBodySet.diagnostics.unresolvedRefBodySkips,
                rejectedBody ? physics_body_classifier::rejectReasonName(rejectedBody->rejectReason) : "none",
                rejectedBody ? rejectedBody->bodyId : INVALID_BODY_ID,
                rejectedBody ? rejectedBody->collisionLayer : 0,
                rejectedBody ? bodyMotionTypeName(rejectedBody->motionType) : "none",
                rejectedBody ? rejectedBody->bodyFlags : 0,
                rejectedBody ? rejectedBody->motionPropertiesId : 0,
                motionConverted ? "ok" : "failed",
                collisionEnabled ? "ok" : "failed");
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
            ROCK_LOG_WARN(Hand, "{} hand PULL failed: primary dynamic body has no motion bodyId={}", handName(), _pulledPrimaryBodyId);
            restoreFailedPullPrep();
            clearPullRuntimeState();
            clearSelectionState(true);
            return false;
        }
    
        const auto grabPivotWorld = computeGrabPivotAWorld(world, handWorldTransform);
        const auto grabPivotHavok = niPointToHkVector(grabPivotWorld);
        RE::NiTransform primaryBodyWorld{};
        const bool hasPrimaryBodyWorld = tryGetBodyWorldTransform(world, RE::hknpBodyId{ _pulledPrimaryBodyId }, primaryBodyWorld);
        RE::NiPoint3 ownerNodePosition{};
        bool hasOwnerNode = false;
        if (auto* ownerNode = getOwnerNodeFromBody(world, RE::hknpBodyId{ _pulledPrimaryBodyId })) {
            ownerNodePosition = ownerNode->world.translate;
            hasOwnerNode = true;
        }
        const auto selectedPointAnchor = body_frame::chooseSelectionDistanceAnchor(_currentSelection.hasHitPoint,
            _currentSelection.hitPointWorld,
            hasPrimaryBodyWorld,
            primaryBodyWorld.translate,
            hasOwnerNode,
            ownerNodePosition,
            true,
            hkVectorToNiPoint(motion->position),
            grabPivotWorld);
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
         * and the seat machinery (support model, seated reacquire, seat depth
         * stop) settles the surface onto the palm.
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
            RE::NiAVObject* presentationMeshNode = _currentSelection.visualNode ? _currentSelection.visualNode : rootNode;
            if (presentationMeshNode) {
                std::vector<TriangleData> presentationTriangles;
                std::vector<GrabSurfaceTriangleData> presentationSurfaceTriangles;
                MeshExtractionStats presentationMeshStats;
                extractAllSurfaceTriangles(presentationMeshNode,
                    presentationTriangles,
                    presentationSurfaceTriangles,
                    (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth),
                    &presentationMeshStats,
                    g_rockConfig.rockGrabNodeNameBlacklist,
                    false);
                const auto longAxis = computeGrabMeshLongAxis(presentationTriangles);
                if (longAxis.valid && longAxis.elongationRatio >= g_rockConfig.rockPullPresentationMinElongationRatio) {
                    const RE::NiPoint3 axisBodyLocal = normalizeOrZero(
                        transform_math::worldVectorToLocal(primaryBodyWorld, longAxis.axisWorld));
                    if (lengthSquared(axisBodyLocal) > 0.000001f) {
                        _pullPresentationAxisBodyLocal = axisBodyLocal;
                        _pullPresentationElongationRatio = longAxis.elongationRatio;
                        _pullPresentationValid = true;
                    }
                }
                ROCK_LOG_DEBUG(Hand,
                    "{} hand PULL presentation axis: valid={} elongation={:.2f} tris={} reason={} axisLocal=({:.3f},{:.3f},{:.3f})",
                    handName(),
                    _pullPresentationValid ? "yes" : "no",
                    longAxis.elongationRatio,
                    longAxis.triangleCount,
                    longAxis.reason,
                    _pullPresentationAxisBodyLocal.x,
                    _pullPresentationAxisBodyLocal.y,
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
        _pullDurationSeconds = pull_motion_math::computePullDurationSeconds(pullDistance, g_rockConfig.rockPullDurationA, g_rockConfig.rockPullDurationB, g_rockConfig.rockPullDurationC);
        _pullTargetHavok = {};
        _pullHasTarget = false;
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
            "{} hand PULL start: type={} weapon={} formID={:08X} primaryBody={} bodyCount={} driveMode={} linearScope={} angularScope={} seeded={} scanFailures={} invalidSystems={} benignSkips={} unresolvedAccepted={} distanceHk={:.3f} duration={:.3f}s setMotion={} enableCollision={}",
            handName(),
            selectedType ? selectedType : "???",
            selectedIsWeapon ? "yes" : "no",
            selectedRef->GetFormID(),
            _pulledPrimaryBodyId,
            _pulledBodyIds.size(),
            held_object_drive_policy::modeName(_pullDriveDecision.mode),
            _pullDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
            _pullDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly",
            preparedBodySet.diagnostics.seedBodiesAdded,
            preparedBodySet.diagnostics.scanFailures,
            preparedBodySet.diagnostics.invalidPhysicsSystems,
            preparedBodySet.diagnostics.benignScanSkips,
            preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
            pullDistance,
            _pullDurationSeconds,
            motionConverted ? "ok" : "failed",
            collisionEnabled ? "ok" : "failed");
        return true;
    }
    
    bool Hand::updateDynamicPull(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, float deltaTime)
    {
        if (_state != HandState::Pulled || !world || _pulledPrimaryBodyId == INVALID_BODY_ID || !_currentSelection.isValid()) {
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
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL invalidated before arrival: primary motion missing bodyId={}",
                handName(),
                _pulledPrimaryBodyId);
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
        const float configuredAutoGrabDistance = (std::max)(0.1f, g_rockConfig.rockPullAutoGrabDistanceGameUnits);
        const float nearConvergeDistance = (std::max)(configuredAutoGrabDistance, g_rockConfig.rockGrabNearConvergeDistanceGameUnits);
        const float pocketBand = (std::max)(0.0f, g_rockConfig.rockGrabPocketRadiusGameUnits);
        const float arrivalDistance = (std::max)(configuredAutoGrabDistance, (std::min)(nearConvergeDistance, configuredAutoGrabDistance + pocketBand));
        if (distanceGameUnits <= arrivalDistance) {
            _currentSelection.isFarSelection = false;
            _currentSelection.hitPointWorld = RE::NiPoint3{
                objectPointHavok.x * havokToGameScale(),
                objectPointHavok.y * havokToGameScale(),
                objectPointHavok.z * havokToGameScale(),
            };
            _currentSelection.hasHitPoint = true;
            markPullCatchIntentArrived();
            _pullTargetHavok = {};
            _pullElapsedSeconds = 0.0f;
            _pullDurationSeconds = 0.0f;
            _pullHasTarget = false;
            applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::PullArrivedClose });
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL arrived -> close grab window dist={:.1f} arrival={:.1f} configuredAuto={:.1f} near={:.1f} pocketBand={:.1f}",
                handName(),
                distanceGameUnits,
                arrivalDistance,
                configuredAutoGrabDistance,
                nearConvergeDistance,
                pocketBand);
            return true;
        }
    
        const auto motionResult = pull_motion_math::computePullMotion<RE::NiPoint3>(
            pull_motion_math::PullMotionInput<RE::NiPoint3>{
                .handHavok = grabPivotHavok,
                .objectPointHavok = objectPointHavok,
                .previousTargetHavok = _pullTargetHavok,
                .elapsedSeconds = _pullElapsedSeconds,
                .durationSeconds = _pullDurationSeconds,
                .applyVelocitySeconds = g_rockConfig.rockPullApplyVelocityTime,
                .ownerGraceSeconds = g_rockConfig.rockPullOwnerGraceSeconds,
                .trackHandSeconds = g_rockConfig.rockPullTrackHandTime,
                .destinationOffsetHavok = g_rockConfig.rockPullDestinationZOffsetHavok,
                .maxVelocityHavok = g_rockConfig.rockPullMaxVelocityHavok,
                .hasPreviousTarget = _pullHasTarget,
            });
    
        _pullElapsedSeconds += (std::max)(0.0f, deltaTime);
        if (motionResult.refreshTarget) {
            _pullTargetHavok = motionResult.targetHavok;
            _pullHasTarget = true;
        }
    
        if (motionResult.expired) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL owner expired before arrival dist={:.1f} arrival={:.1f} elapsed={:.3f}s duration={:.3f}s ownerGrace={:.3f}s",
                handName(),
                distanceGameUnits,
                arrivalDistance,
                _pullElapsedSeconds,
                _pullDurationSeconds,
                g_rockConfig.rockPullOwnerGraceSeconds);
            finishPullPrepAsPhysicalDropIfActive("pull-owner-expired");
            clearSelectionState(true);
            return false;
        }
    
        if (!motionResult.applyVelocity) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} hand PULL holding owner after velocity window dist={:.1f} arrival={:.1f} elapsed={:.3f}s duration={:.3f}s ownerGrace={:.3f}s",
                handName(),
                distanceGameUnits,
                arrivalDistance,
                _pullElapsedSeconds,
                _pullDurationSeconds,
                g_rockConfig.rockPullOwnerGraceSeconds);
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
                const RE::NiPoint3 currentAxisWorld =
                    normalizeOrZero(transform_math::localVectorToWorld(pulledBodyWorld, _pullPresentationAxisBodyLocal));
                /*
                 * The grip line is not pure cross-palm Z: the thumb sits in the
                 * way, so the natural long-object hold tilts a few degrees
                 * toward the fingers-forward X axis (Bruno-tuned via INI).
                 */
                const float gripAxisTiltRadians = gripAxisTiltRadiansForHand(_isLeft);
                RE::NiPoint3 targetAxisWorld = normalizeOrZero(
                    transformHandspaceDirection(handWorldTransform,
                        RE::NiPoint3{ std::sin(gripAxisTiltRadians), 0.0f, std::cos(gripAxisTiltRadians) },
                        _isLeft));
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
                        const float angularSpeed = (std::min)(
                            angleRadians * (std::max)(0.0f, g_rockConfig.rockPullPresentationAngularGainPerSecond),
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
                presentationAngularVelocity,
                true,
                1.0f,
                _pullDriveDecision.includeConnectedLinearVelocity,
                _pullDriveDecision.includeConnectedAngularVelocity);
        } else {
            setHeldLinearVelocity(world, RE::hknpBodyId{ _pulledPrimaryBodyId }, _pulledBodyIds, motionResult.velocityHavok,
                pull_motion_math::angularVelocityKeepForDamping(g_rockConfig.rockPulledAngularDamping, deltaTime),
                _pullDriveDecision.includeConnectedLinearVelocity);
        }
        for (const auto bodyId : _pulledBodyIds) {
            physics_recursive_wrappers::activateBody(world, bodyId);
        }
    
        return false;
    }
}
