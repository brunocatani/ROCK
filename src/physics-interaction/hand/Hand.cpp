#include "physics-interaction/hand/Hand.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/hand/HandSelection.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/TransformMath.h"
#include "RockUtils.h"
#include "RE/Havok/hknpMotion.h"

namespace rock
{
    namespace
    {
        constexpr const char* GRAB_HAND_POSE_TAG = "ROCK_Grab";
        constexpr const char* LEFT_GRAB_PRIMARY_POSE_BLOCK_TAG = "ROCK_GrabPrimaryPoseLeft";
        constexpr const char* RIGHT_GRAB_PRIMARY_POSE_BLOCK_TAG = "ROCK_GrabPrimaryPoseRight";
        constexpr const char* GRAB_EXTERNAL_HAND_TAG = "ROCK_GrabVisual";
        constexpr object_physics_body_set::ObjectPhysicsBodyScanBudget kGrabAcquisitionPrewarmBudget{
            .maxVisitedNodes = 64,
            .maxCollisionObjects = 16,
            .maxBodyIds = 128,
        };

        void clearGrabHandPose(bool isLeft)
        {
            (void)frik_visual_authority::clearHandPose(GRAB_HAND_POSE_TAG, handFromBool(isLeft));
            (void)frik_visual_authority::blockPrimaryHandWeaponPose(isLeft ? LEFT_GRAB_PRIMARY_POSE_BLOCK_TAG : RIGHT_GRAB_PRIMARY_POSE_BLOCK_TAG, false);
        }

        void clearGrabExternalHandWorldTransform(bool isLeft)
        {
            (void)frik_visual_authority::clearExternalHandWorldTransform(GRAB_EXTERNAL_HAND_TAG, handFromBool(isLeft));
        }

        RE::NiTransform getLiveBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId)
        {
            RE::NiTransform result = transform_math::makeIdentityTransform<RE::NiTransform>();
            tryResolveLiveBodyWorldTransform(world, bodyId, result);
            return result;
        }

        float pointDistanceGameUnits(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            const RE::NiPoint3 delta = lhs - rhs;
            return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
        }

        RE::NiPoint3 normalizeOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
        {
            const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
            if (lengthSquared <= 1.0e-6f) {
                return fallback;
            }

            const float inverseLength = 1.0f / std::sqrt(lengthSquared);
            return RE::NiPoint3(value.x * inverseLength, value.y * inverseLength, value.z * inverseLength);
        }

        const char* primaryBodyChoiceReasonName(object_physics_body_set::PrimaryBodyChoiceReason reason)
        {
            using object_physics_body_set::PrimaryBodyChoiceReason;
            switch (reason) {
            case PrimaryBodyChoiceReason::PreferredHitAccepted:
                return "preferredHitAccepted";
            case PrimaryBodyChoiceReason::SurfaceOwnerAccepted:
                return "surfaceOwnerAccepted";
            case PrimaryBodyChoiceReason::NearestAcceptedFallback:
                return "nearestAcceptedFallback";
            case PrimaryBodyChoiceReason::NoAcceptedBody:
                return "noAcceptedBody";
            case PrimaryBodyChoiceReason::None:
            default:
                return "none";
            }
        }

        constexpr const char* SELECTED_CLOSE_FINGER_TAG = "ROCK_SelectedClose";

        const char* handStateName(HandState state)
        {
            switch (state) {
            case HandState::Idle:
                return "Idle";
            case HandState::SelectedClose:
                return "SelectedClose";
            case HandState::SelectedFar:
                return "SelectedFar";
            case HandState::SelectionLocked:
                return "SelectionLocked";
            case HandState::PreGrabItem:
                return "PreGrabItem";
            case HandState::PrePullItem:
                return "PrePullItem";
            case HandState::HeldInit:
                return "HeldInit";
            case HandState::HeldBody:
                return "HeldBody";
            case HandState::Pulled:
                return "Pulled";
            case HandState::GrabFromOtherHand:
                return "GrabFromOtherHand";
            case HandState::GrabExternal:
                return "GrabExternal";
            case HandState::LootOtherHand:
                return "LootOtherHand";
            case HandState::SelectedTwoHand:
                return "SelectedTwoHand";
            case HandState::HeldTwoHanded:
                return "HeldTwoHanded";
            case HandState::StashCandidate:
                return "StashCandidate";
            case HandState::ConsumeCandidate:
                return "ConsumeCandidate";
            }
            return "Unknown";
        }

        const char* handEventName(HandInteractionEvent event)
        {
            switch (event) {
            case HandInteractionEvent::Initialize:
                return "Initialize";
            case HandInteractionEvent::SelectionFoundClose:
                return "SelectionFoundClose";
            case HandInteractionEvent::SelectionFoundFar:
                return "SelectionFoundFar";
            case HandInteractionEvent::SelectionLost:
                return "SelectionLost";
            case HandInteractionEvent::LockFarSelection:
                return "LockFarSelection";
            case HandInteractionEvent::BeginPreGrabItem:
                return "BeginPreGrabItem";
            case HandInteractionEvent::BeginPrePullItem:
                return "BeginPrePullItem";
            case HandInteractionEvent::BeginExternalGrab:
                return "BeginExternalGrab";
            case HandInteractionEvent::BeginLootOtherHand:
                return "BeginLootOtherHand";
            case HandInteractionEvent::SpawnedItemReady:
                return "SpawnedItemReady";
            case HandInteractionEvent::BeginPull:
                return "BeginPull";
            case HandInteractionEvent::PullArrivedClose:
                return "PullArrivedClose";
            case HandInteractionEvent::BeginGrabCommit:
                return "BeginGrabCommit";
            case HandInteractionEvent::GrabCommitSucceeded:
                return "GrabCommitSucceeded";
            case HandInteractionEvent::HeldFadeComplete:
                return "HeldFadeComplete";
            case HandInteractionEvent::BeginStashCandidate:
                return "BeginStashCandidate";
            case HandInteractionEvent::BeginConsumeCandidate:
                return "BeginConsumeCandidate";
            case HandInteractionEvent::CancelGameplayCandidate:
                return "CancelGameplayCandidate";
            case HandInteractionEvent::CommitStash:
                return "CommitStash";
            case HandInteractionEvent::CommitConsume:
                return "CommitConsume";
            case HandInteractionEvent::CompleteLoot:
                return "CompleteLoot";
            case HandInteractionEvent::ReleaseRequested:
                return "ReleaseRequested";
            case HandInteractionEvent::ObjectInvalidated:
                return "ObjectInvalidated";
            case HandInteractionEvent::WorldInvalidated:
                return "WorldInvalidated";
            case HandInteractionEvent::BeginOtherHandTransfer:
                return "BeginOtherHandTransfer";
            case HandInteractionEvent::CompleteOtherHandTransfer:
                return "CompleteOtherHandTransfer";
            case HandInteractionEvent::BeginTwoHandSelection:
                return "BeginTwoHandSelection";
            case HandInteractionEvent::BeginTwoHandHold:
                return "BeginTwoHandHold";
            case HandInteractionEvent::EndTwoHandHold:
                return "EndTwoHandHold";
            }
            return "Unknown";
        }
    }

    void Hand::reset()
    {
        const bool suppressionActive = hand_collision_suppression_math::hasActive(_grabHandCollisionSuppression);
        const bool cleanupRequired = hand_lifecycle_policy::requiresHavokCleanupBeforeReset(
            _activeConstraint.isValid() || _grabAuthorityProxy.isValid(), suppressionActive, _heldBodyIds.size(), _savedObjectState.isValid(), hasCollisionBody());
        if (cleanupRequired) {
            ROCK_LOG_ERROR(Hand,
                "{} hand reset blocked: Havok state still needs cleanup (constraint={} proxy={} suppression={} heldBodies={} savedState={} handBody={})",
                handName(),
                _activeConstraint.isValid() ? "yes" : "no",
                _grabAuthorityProxy.isValid() ? "yes" : "no",
                suppressionActive ? "yes" : "no",
                _heldBodyIds.size(),
                _savedObjectState.isValid() ? "yes" : "no",
                hasCollisionBody() ? "yes" : "no");
            return;
        }

        stopSelectionHighlight();
        _selectionBeam.shutdown();
        _isHoldingFlag.store(false, std::memory_order_release);
        _heldBodyIdsCount.store(0, std::memory_order_release);
        clearHeldBodyContactSnapshot();
        _state = HandState::Idle;
        _prevState = HandState::Idle;
        _stateAtomic.store(HandState::Idle, std::memory_order_release);
        _idleDesired = false;
        _grabRequested = false;
        _releaseRequested = false;
        _boneColliders.reset();
        _handBody.reset();
        _currentSelection.clear();
        _cachedFarCandidate.clear();
        clearGrabAcquisitionCache("reset");
        _farDetectCounter = 0;
        _selectionHoldFrames = 0;
        _deselectCooldown = 0;
        _lastDeselectedRef = nullptr;
        _lastTouchedRef = nullptr;
        _lastTouchedFormID = 0;
        _lastTouchedLayer = 0;
        _touchActiveFrames = 100;
        {
            std::scoped_lock writeLock(_semanticContactWriteMutex);
            _semanticContactFrameCounter.store(0, std::memory_order_release);
            _semanticContactValid.store(0, std::memory_order_release);
            _semanticContactSequence.store(0, std::memory_order_release);
            _semanticContactRole.store(static_cast<std::uint32_t>(hand_collider_semantics::HandColliderRole::PalmAnchor), std::memory_order_release);
            _semanticContactFinger.store(static_cast<std::uint32_t>(hand_collider_semantics::HandFinger::None), std::memory_order_release);
            _semanticContactSegment.store(static_cast<std::uint32_t>(hand_collider_semantics::HandFingerSegment::None), std::memory_order_release);
            _semanticContactHandBodyId.store(hand_semantic_contact_state::kInvalidBodyId, std::memory_order_release);
            _semanticContactOtherBodyId.store(hand_semantic_contact_state::kInvalidBodyId, std::memory_order_release);
            _semanticContactFrames.store(0xFFFF'FFFFu, std::memory_order_release);
            for (std::size_t i = 0; i < hand_semantic_contact_state::kMaxSemanticContactRecords; ++i) {
                _semanticContactSetValid[i].store(0, std::memory_order_release);
                _semanticContactSetRole[i].store(static_cast<std::uint32_t>(hand_collider_semantics::HandColliderRole::PalmAnchor), std::memory_order_release);
                _semanticContactSetFinger[i].store(static_cast<std::uint32_t>(hand_collider_semantics::HandFinger::None), std::memory_order_release);
                _semanticContactSetSegment[i].store(static_cast<std::uint32_t>(hand_collider_semantics::HandFingerSegment::None), std::memory_order_release);
                _semanticContactSetHandBodyId[i].store(hand_semantic_contact_state::kInvalidBodyId, std::memory_order_release);
                _semanticContactSetOtherBodyId[i].store(hand_semantic_contact_state::kInvalidBodyId, std::memory_order_release);
                _semanticContactSetFrames[i].store(0xFFFF'FFFFu, std::memory_order_release);
                _semanticContactSetRunStartFrames[i].store(0xFFFF'FFFFu, std::memory_order_release);
                _semanticContactSetSequence[i].store(0, std::memory_order_release);
            }
        }
        _activeConstraint.clear();
        abandonGrabAuthorityProxy();
        _savedObjectState.clear();
        _activeGrabLifecycle.clear();
        _grabStartTime = 0.0f;
        _heldLogCounter = 0;
        _notifCounter = 0;
        _heldBodyIds.clear();
        clearPullRuntimeState(false, "reset");
        clearPullCatchIntent("reset");
        clearActorEquipmentDropHandoff("reset");
        if (_nearbyGrabDamping.active || !_nearbyGrabDamping.motions.empty()) {
            ROCK_LOG_WARN(Hand, "{} hand reset cleared nearby velocity-damping state without a world; no native damping fields were modified", handName());
        }
        _nearbyGrabDamping.clear();
        _grabFrame.clear();
        _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::Idle;
        _grabObjectGripAtGrab = {};
        _heldObjectIsLooseWeapon = false;
        _grabFingerPosePublished = false;
        _grabConvergeStableInsidePocketFrames = 0;
        _grabConvergePreviousGripErrorGameUnits = std::numeric_limits<float>::max();
        clearGrabHandPose(_isLeft);
        clearGrabExternalHandWorldTransform(_isLeft);
        clearGrabVisualReturn("reset", false);
        _grabDeviationExceededSeconds = 0.0f;
        _grabVisualHandTransform = {};
        _hasGrabVisualHandTransform = false;
        _lastPublishedGrabVisualHandTransform = {};
        _hasLastPublishedGrabVisualHandTransform = false;
        _grabVisualDeviationExceededSeconds = 0.0f;
        _grabAuthorityProxyReleasePending.store(false, std::memory_order_release);
        _grabFingerProbeStart = {};
        _grabFingerProbeEnd = {};
        _hasGrabFingerProbeDebug = false;
        _grabFingerSweepDebugCapture = {};
        _grabFingerSweepDebugObjectWorld = {};
        _hasGrabFingerSweepDebug = false;
        _grabFingerPadProbeStart = {};
        _grabFingerPadProbeEnd = {};
        _grabFingerPadProbeHit = {};
        _grabFingerPadProbeHitValid = {};
        _hasGrabFingerPadProbeDebug = false;
        _grabFingerSurfaceTarget = {};
        _grabFingerSurfaceTargetValid = {};
        _hasGrabFingerSurfaceTargetDebug = false;
        _grabFingerJointPose = {};
        _grabFingerLocalTransforms = {};
        _grabFingerLocalTransformMask = 0;
        _grabFingerPose = {};
        _grabFingerTriangleIndex.clear();
        _hasGrabFingerJointPose = false;
        _hasGrabFingerLocalTransforms = false;
        _hasGrabFingerPose = false;
        clearSelectedCloseFingerPose();
        _lastSelectedCloseOrigin = {};
        _hasLastSelectedCloseOrigin = false;
        _selectedCloseHandSpeedMetersPerSecond = 0.0f;
        _heldLocalLinearVelocityHistory = {};
        _heldLocalLinearVelocityHistoryCount = 0;
        _heldLocalLinearVelocityHistoryNext = 0;
        _heldLocalHandVelocityHistory = {};
        _heldHandAngularVelocityHistory = {};
        _heldHandVelocityHistoryCount = 0;
        _heldHandVelocityHistoryNext = 0;
        _lastHeldObjectLocalLinearVelocityHavok = {};
        _hasLastHeldObjectLocalLinearVelocityHavok = false;
        _previousHeldRawHandWorld = {};
        _previousHeldHandPositionHavok = {};
        _lastHeldHandPositionHavok = {};
        _hasPreviousHeldRawHandWorld = false;
        _hasLastHeldHandPositionHavok = false;
        clearGrabHandCollisionSuppressionState();
        clearHeldLooseWeaponBodyCollisionSuppressionState();
        // reset() is blocked above while held bodies still need cleanup, so any
        // lease reaching here has already been restored through the release path.
        clearHeldObjectCollisionLayerState();
    }

    void Hand::abandonHavokStateAfterWorldLoss()
    {
        _selectionBeam.abandonSceneGraph();

        /*
         * World-loss teardown cannot safely restore old body flags, filters, or
         * recursive motion through the stale hknp world. Normal release remains
         * the required path while the world is valid; this path only abandons
         * ROCK-owned handles so module shutdown/reset can complete after the
         * game has already replaced or destroyed the physics world.
         */
        const bool hadConstraint = _activeConstraint.isValid();
        const bool hadProxy = _grabAuthorityProxy.isValid();
        const bool hadSuppression = hand_collision_suppression_math::hasActive(_grabHandCollisionSuppression);
        const bool hadLooseWeaponBodySuppression = hand_collision_suppression_math::hasActive(_heldLooseWeaponBodyCollisionSuppression);
        const auto heldBodyCount = _heldBodyIds.size();
        const bool hadSavedState = _savedObjectState.isValid();
        const bool hadHandBody = hasCollisionBody();

        if (!hadConstraint && !hadProxy && !hadSuppression && !hadLooseWeaponBodySuppression && heldBodyCount == 0 && !hadSavedState && !hadHandBody) {
            return;
        }

        ROCK_LOG_WARN(Hand,
            "{} hand abandoning Havok state after world loss: constraint={} proxy={} suppression={} looseWeaponBodySuppression={} heldBodies={} savedState={} handBody={}",
            handName(),
            hadConstraint ? "yes" : "no",
            hadProxy ? "yes" : "no",
            hadSuppression ? "yes" : "no",
            hadLooseWeaponBodySuppression ? "yes" : "no",
            heldBodyCount,
            hadSavedState ? "yes" : "no",
            hadHandBody ? "yes" : "no");

        _activeConstraint.clear();
        abandonGrabAuthorityProxy();
        _savedObjectState.clear();
        _activeGrabLifecycle.clear();
        _heldBodyIds.clear();
        clearGrabAcquisitionCache("worldLoss");
        clearPullRuntimeState(false, "worldLoss");
        clearPullCatchIntent("worldLoss");
        clearActorEquipmentDropHandoff("worldLoss");
        _heldBodyIdsCount.store(0, std::memory_order_release);
        clearHeldBodyContactSnapshot();
        _isHoldingFlag.store(false, std::memory_order_release);
        _nearbyGrabDamping.clear();
        _grabFrame.clear();
        _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::Idle;
        _grabObjectGripAtGrab = {};
        _heldObjectIsLooseWeapon = false;
        _grabFingerPosePublished = false;
        _grabConvergeStableInsidePocketFrames = 0;
        _grabConvergePreviousGripErrorGameUnits = std::numeric_limits<float>::max();
        clearGrabHandCollisionSuppressionState();
        clearHeldLooseWeaponBodyCollisionSuppressionState();
        // The held-object layer cannot be written back through a dead world, for
        // the same reason the filters above cannot; those bodies are destroyed
        // with the world being abandoned.
        clearHeldObjectCollisionLayerState();
        _boneColliders.reset();
        _handBody.reset();
        _grabAuthorityProxyReleasePending.store(false, std::memory_order_release);
        clearGrabHandPose(_isLeft);
        clearGrabExternalHandWorldTransform(_isLeft);
        clearGrabVisualReturn("world-loss", false);
        _grabVisualHandTransform = {};
        _hasGrabVisualHandTransform = false;
        _lastPublishedGrabVisualHandTransform = {};
        _hasLastPublishedGrabVisualHandTransform = false;
        _grabVisualDeviationExceededSeconds = 0.0f;
        clearSelectedCloseFingerPose();
        _grabFingerSweepDebugCapture = {};
        _grabFingerSweepDebugObjectWorld = {};
        _hasGrabFingerSweepDebug = false;
        _grabFingerJointPose = {};
        _grabFingerLocalTransforms = {};
        _grabFingerLocalTransformMask = 0;
        _grabFingerPose = {};
        _grabFingerTriangleIndex.clear();
        _hasGrabFingerJointPose = false;
        _hasGrabFingerLocalTransforms = false;
        _hasGrabFingerPose = false;
        _heldLocalHandVelocityHistory = {};
        _heldHandAngularVelocityHistory = {};
        _heldHandVelocityHistoryCount = 0;
        _heldHandVelocityHistoryNext = 0;
        _lastHeldObjectLocalLinearVelocityHavok = {};
        _hasLastHeldObjectLocalLinearVelocityHavok = false;
        _previousHeldRawHandWorld = {};
        _previousHeldHandPositionHavok = {};
        _lastHeldHandPositionHavok = {};
        _hasPreviousHeldRawHandWorld = false;
        _hasLastHeldHandPositionHavok = false;
    }

    HandTransitionResult Hand::applyTransition(const HandTransitionRequest& request)
    {
        HandTransitionRequest evaluatedRequest = request;
        evaluatedRequest.current = _state;
        const auto result = evaluateHandTransition(evaluatedRequest);
        if (!result.accepted) {
            ROCK_LOG_TRACE(Hand,
                "{} hand state transition rejected: state={} event={} reason={}",
                handName(),
                handStateName(_state),
                handEventName(evaluatedRequest.event),
                result.reason ? result.reason : "");
            return result;
        }

        const auto oldState = _state;
        if (result.next != oldState) {
            _prevState = oldState;
            _state = result.next;
            _stateAtomic.store(result.next, std::memory_order_release);
            if (suppressesGeneratedHandContactEvidence(oldState) || suppressesGeneratedHandContactEvidence(result.next)) {
                clearSemanticContactEvidence();
            }
            ROCK_LOG_DEBUG(Hand,
                "{} hand state {} -> {} via {}",
                handName(),
                handStateName(oldState),
                handStateName(result.next),
                handEventName(evaluatedRequest.event));
        }

        return result;
    }

    bool Hand::beginStashCandidate()
    {
        return applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::BeginStashCandidate }).accepted;
    }

    bool Hand::cancelStashCandidate()
    {
        if (_state != HandState::StashCandidate) {
            return false;
        }
        return applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::CancelGameplayCandidate }).accepted;
    }

    bool Hand::beginConsumeCandidate()
    {
        return applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::BeginConsumeCandidate }).accepted;
    }

    bool Hand::cancelConsumeCandidate()
    {
        if (_state != HandState::ConsumeCandidate) {
            return false;
        }
        return applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::CancelGameplayCandidate }).accepted;
    }

    void Hand::clearPullRuntimeState(bool restorePreparedObject, const char* context)
    {
        if (restorePreparedObject) {
            restorePullPrepIfActive(context);
        } else {
            clearPullPrepTracking();
        }
        _pulledBodyIds.clear();
        _pullDriveDecision = {};
        _pulledPrimaryBodyId = INVALID_BODY_ID;
        _pullPointOffsetHavok = {};
        _pullTargetHavok = {};
        _pullElapsedSeconds = 0.0f;
        _pullDurationSeconds = 0.0f;
        _pullHasTarget = false;
        _pullPresentationAxisBodyLocal = {};
        _pullPresentationElongationRatio = 0.0f;
        _pullPresentationValid = false;
    }

    object_physics_body_set::BodySetScanOptions Hand::makeActiveGrabBodyScanOptions(const SelectedObject& selection) const
    {
        object_physics_body_set::BodySetScanOptions scanOptions{};
        scanOptions.mode = physics_body_classifier::InteractionMode::ActiveGrab;
        scanOptions.rightHandBodyId = _isLeft ? INVALID_BODY_ID : _handBody.getBodyId().value;
        scanOptions.leftHandBodyId = _isLeft ? _handBody.getBodyId().value : INVALID_BODY_ID;
        scanOptions.sourceBodyId = _handBody.getBodyId().value;
        scanOptions.seedBodyId = selection.bodyId.value;
        scanOptions.targetKind = selection.targetKind;
        scanOptions.seedHitNode = selection.hitNode;
        scanOptions.requireSameResolvedRef = true;
        scanOptions.allowWeaponRefExpansion = true;
        scanOptions.heldBySameHand = &_heldBodyIds;
        scanOptions.maxDepth = g_rockConfig.rockObjectPhysicsTreeMaxDepth;
        return scanOptions;
    }

    void Hand::clearGrabAcquisitionCache(const char* reason)
    {
        if (_grabAcquisitionCache.valid) {
            performance_profiler::addCounter(performance_profiler::Counter::GrabAcquisitionCacheInvalidated);
            ROCK_LOG_TRACE(Hand,
                "{} hand cleared grab acquisition cache: reason={} formID={:08X} body={} cachedBodies={}",
                handName(),
                reason ? reason : "unknown",
                _grabAcquisitionCache.formId,
                _grabAcquisitionCache.selectedBodyId,
                _grabAcquisitionCache.scanCache.bodyIdCount);
        }
        _grabAcquisitionCache.clear();
    }

    bool Hand::grabAcquisitionCacheMatches(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        const SelectedObject& selection,
        const object_physics_body_set::BodySetScanOptions& options) const
    {
        if (!_grabAcquisitionCache.valid || !bhkWorld || !hknpWorld || !selection.isValid() || !selection.refr) {
            return false;
        }
        if (selection.refr->IsDeleted() || selection.refr->IsDisabled()) {
            return false;
        }

        auto* rootNode = selection.refr->Get3D();
        return rootNode &&
               _grabAcquisitionCache.refr == selection.refr &&
               _grabAcquisitionCache.formId == selection.refr->GetFormID() &&
               _grabAcquisitionCache.rootNode.get() == rootNode &&
               _grabAcquisitionCache.bhkWorld == bhkWorld &&
               _grabAcquisitionCache.hknpWorld == hknpWorld &&
               _grabAcquisitionCache.selectedBodyId == options.seedBodyId &&
               _grabAcquisitionCache.rightHandBodyId == options.rightHandBodyId &&
               _grabAcquisitionCache.leftHandBodyId == options.leftHandBodyId &&
               _grabAcquisitionCache.sourceBodyId == options.sourceBodyId &&
               _grabAcquisitionCache.targetKind == options.targetKind &&
               _grabAcquisitionCache.maxDepth == options.maxDepth;
    }

    void Hand::updateGrabAcquisitionCache(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld)
    {
        if (!_currentSelection.isValid() || !_currentSelection.refr || !bhkWorld || !hknpWorld || !_handBody.isValid()) {
            clearGrabAcquisitionCache("selection-unavailable");
            return;
        }
        if (_currentSelection.refr->IsDeleted() || _currentSelection.refr->IsDisabled()) {
            clearGrabAcquisitionCache("selection-invalid");
            return;
        }
        if (!grab_target::canUseRockActiveGrab(_currentSelection.targetKind) &&
            !grab_target::canUseRockDynamicPull(_currentSelection.targetKind)) {
            clearGrabAcquisitionCache("target-kind-not-physical");
            return;
        }

        const auto scanOptions = makeActiveGrabBodyScanOptions(_currentSelection);
        const bool matchingCache = grabAcquisitionCacheMatches(bhkWorld, hknpWorld, _currentSelection, scanOptions);
        if (matchingCache && _grabAcquisitionCache.stage == GrabAcquisitionCache::Stage::PreScanReady) {
            return;
        }

        if (!matchingCache) {
            clearGrabAcquisitionCache("selection-identity-changed");
            auto* rootNode = _currentSelection.refr->Get3D();
            if (!rootNode) {
                return;
            }

            _grabAcquisitionCache.refr = _currentSelection.refr;
            _grabAcquisitionCache.rootNode.reset(rootNode);
            _grabAcquisitionCache.hitNode.reset(_currentSelection.hitNode);
            _grabAcquisitionCache.bhkWorld = bhkWorld;
            _grabAcquisitionCache.hknpWorld = hknpWorld;
            _grabAcquisitionCache.formId = _currentSelection.refr->GetFormID();
            _grabAcquisitionCache.selectedBodyId = scanOptions.seedBodyId;
            _grabAcquisitionCache.rightHandBodyId = scanOptions.rightHandBodyId;
            _grabAcquisitionCache.leftHandBodyId = scanOptions.leftHandBodyId;
            _grabAcquisitionCache.sourceBodyId = scanOptions.sourceBodyId;
            _grabAcquisitionCache.targetKind = scanOptions.targetKind;
            _grabAcquisitionCache.maxDepth = scanOptions.maxDepth;
            _grabAcquisitionCache.stage = GrabAcquisitionCache::Stage::PreScanRunning;
            _grabAcquisitionCache.valid = true;

            if (!object_physics_body_set::beginObjectPhysicsBodyScanCache(
                    _currentSelection.refr,
                    scanOptions,
                    _grabAcquisitionCache.scanCursor,
                    _grabAcquisitionCache.scanCache)) {
                clearGrabAcquisitionCache("prewarm-begin-failed");
                performance_profiler::addCounter(performance_profiler::Counter::GrabAcquisitionCacheMiss);
                return;
            }

            performance_profiler::addCounter(performance_profiler::Counter::GrabAcquisitionCachePrewarm);
        }

        if (_grabAcquisitionCache.stage != GrabAcquisitionCache::Stage::PreScanRunning) {
            return;
        }

        object_physics_body_set::ObjectPhysicsBodyScanStepResult step{};
        {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAcquisitionBodyScan);
            step = object_physics_body_set::advanceObjectPhysicsBodyScanCache(
                hknpWorld,
                scanOptions,
                kGrabAcquisitionPrewarmBudget,
                _grabAcquisitionCache.scanCursor,
                _grabAcquisitionCache.scanCache);
        }

        if (step.invalidated) {
            clearGrabAcquisitionCache("prewarm-invalidated");
            return;
        }

        if (!step.progressed && !step.finished) {
            return;
        }

        performance_profiler::observeValue(performance_profiler::ValueMetric::GrabAcquisitionVisitedNodes, step.visitedNodes);
        performance_profiler::observeValue(performance_profiler::ValueMetric::GrabAcquisitionCollisionObjects, step.collisionObjects);
        performance_profiler::observeValue(performance_profiler::ValueMetric::GrabAcquisitionBodyIds, step.bodyIds);

        if (!step.finished) {
            return;
        }

        _grabAcquisitionCache.beforePrepBodySet =
            object_physics_body_set::buildObjectPhysicsBodySetFromScanCache(bhkWorld, hknpWorld, _currentSelection.refr, scanOptions, _grabAcquisitionCache.scanCache);
        if (!_grabAcquisitionCache.scanCache.valid || _grabAcquisitionCache.beforePrepBodySet.records.empty()) {
            clearGrabAcquisitionCache("prewarm-empty");
            performance_profiler::addCounter(performance_profiler::Counter::GrabAcquisitionCacheMiss);
            return;
        }

        _grabAcquisitionCache.stage = GrabAcquisitionCache::Stage::PreScanReady;

        ROCK_LOG_SAMPLE_DEBUG(Hand,
            g_rockConfig.rockLogSampleMilliseconds,
            "{} hand prewarmed grab acquisition cache: formID={:08X} body={} targetKind={} records={} accepted={} visitedNodes={} collisionObjects={} cachedBodyIds={} invalidSystems={} benignSkips={} staleSkips={}",
            handName(),
            _grabAcquisitionCache.formId,
            _grabAcquisitionCache.selectedBodyId,
            grab_target::name(_grabAcquisitionCache.targetKind),
            _grabAcquisitionCache.beforePrepBodySet.records.size(),
            _grabAcquisitionCache.beforePrepBodySet.acceptedCount(),
            _grabAcquisitionCache.scanCache.diagnostics.visitedNodes,
            _grabAcquisitionCache.scanCache.diagnostics.collisionObjects,
            _grabAcquisitionCache.scanCache.bodyIdCount,
            _grabAcquisitionCache.scanCache.diagnostics.invalidPhysicsSystems,
            _grabAcquisitionCache.scanCache.diagnostics.benignScanSkips,
            _grabAcquisitionCache.scanCache.diagnostics.staleCacheEntrySkips);
    }

    bool Hand::tryUseGrabAcquisitionBeforePrepCache(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        const SelectedObject& selection,
        const object_physics_body_set::BodySetScanOptions& options,
        object_physics_body_set::ObjectPhysicsBodySet& outBodySet) const
    {
        if (!grabAcquisitionCacheMatches(bhkWorld, hknpWorld, selection, options) ||
            _grabAcquisitionCache.stage != GrabAcquisitionCache::Stage::PreScanReady ||
            !_grabAcquisitionCache.scanCache.valid) {
            performance_profiler::addCounter(performance_profiler::Counter::GrabAcquisitionCacheMiss);
            return false;
        }
        outBodySet = object_physics_body_set::buildObjectPhysicsBodySetFromScanCache(
            bhkWorld,
            hknpWorld,
            selection.refr,
            options,
            _grabAcquisitionCache.scanCache);
        if (outBodySet.records.empty()) {
            performance_profiler::addCounter(performance_profiler::Counter::GrabAcquisitionCacheMiss);
            return false;
        }
        performance_profiler::addCounter(performance_profiler::Counter::GrabAcquisitionCacheHit);
        return true;
    }

    bool Hand::tryBuildGrabAcquisitionPreparedBodySetFromCache(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        const SelectedObject& selection,
        const object_physics_body_set::BodySetScanOptions& options,
        object_physics_body_set::ObjectPhysicsBodySet& outBodySet,
        bool& outPostPrepComplete) const
    {
        outPostPrepComplete = false;
        if (!grabAcquisitionCacheMatches(bhkWorld, hknpWorld, selection, options) ||
            _grabAcquisitionCache.stage != GrabAcquisitionCache::Stage::PreScanReady ||
            !_grabAcquisitionCache.scanCache.valid) {
            performance_profiler::addCounter(performance_profiler::Counter::GrabAcquisitionCacheMiss);
            return false;
        }

        const auto& sourceCache = _grabAcquisitionCache.postPrepComplete ? _grabAcquisitionCache.postPrepScanCache : _grabAcquisitionCache.scanCache;
        outBodySet = _grabAcquisitionCache.postPrepComplete ?
                         _grabAcquisitionCache.postPrepBodySet :
                         object_physics_body_set::buildObjectPhysicsBodySetFromScanCache(
                             bhkWorld,
                             hknpWorld,
                             selection.refr,
                             options,
                             sourceCache);
        if (outBodySet.records.empty()) {
            performance_profiler::addCounter(performance_profiler::Counter::GrabAcquisitionCacheMiss);
            return false;
        }
        outPostPrepComplete = _grabAcquisitionCache.postPrepComplete;
        performance_profiler::addCounter(performance_profiler::Counter::GrabAcquisitionCacheHit);
        return true;
    }

    void Hand::armPullCatchIntent(RE::TESObjectREFR* refr, std::uint32_t primaryBodyId, grab_target::Kind targetKind)
    {
        _pullCatchIntent = PullCatchIntent{
            .active = refr != nullptr && primaryBodyId != INVALID_BODY_ID,
            .commitPending = false,
            .refr = refr,
            .formId = refr ? refr->GetFormID() : 0,
            .primaryBodyId = primaryBodyId,
            .targetKind = targetKind,
            .commitElapsedSeconds = 0.0f,
            .failedCommitAttempts = 0,
        };

        if (_pullCatchIntent.active) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL catch intent armed formID={:08X} primaryBody={}",
                handName(),
                _pullCatchIntent.formId,
                _pullCatchIntent.primaryBodyId);
        }
    }

    void Hand::markPullCatchIntentArrived()
    {
        if (!_pullCatchIntent.active) {
            return;
        }

        _pullCatchIntent.commitPending = true;
        _pullCatchIntent.commitElapsedSeconds = 0.0f;
        _pullCatchIntent.failedCommitAttempts = 0;
        ROCK_LOG_DEBUG(Hand,
            "{} hand PULL catch intent arrived formID={:08X} primaryBody={}",
            handName(),
            _pullCatchIntent.formId,
            _pullCatchIntent.primaryBodyId);
    }

    bool Hand::pullCatchIntentMatchesSelection() const
    {
        return _pullCatchIntent.active &&
               _pullCatchIntent.commitPending &&
               _state == HandState::SelectedClose &&
               _currentSelection.isValid() &&
               !_currentSelection.isFarSelection &&
               _currentSelection.refr == _pullCatchIntent.refr &&
               (!_currentSelection.refr || _currentSelection.refr->GetFormID() == _pullCatchIntent.formId) &&
               _currentSelection.bodyId.value == _pullCatchIntent.primaryBodyId;
    }

    bool Hand::hasActivePullCatchIntent() const
    {
        return _pullCatchIntent.active;
    }

    bool Hand::hasArrivedPullCatchIntent() const
    {
        return _pullCatchIntent.active && _pullCatchIntent.commitPending;
    }

    bool Hand::hasPendingPullCatchCommit() const
    {
        return pullCatchIntentMatchesSelection();
    }

    bool Hand::advancePullCatchCommit(float deltaTime, float maxCommitSeconds)
    {
        if (!hasPendingPullCatchCommit()) {
            return false;
        }

        if (_pullCatchIntent.failedCommitAttempts == 0) {
            return true;
        }

        _pullCatchIntent.commitElapsedSeconds += (std::max)(0.0f, std::isfinite(deltaTime) ? deltaTime : 0.0f);
        const float retryWindow = (std::max)(0.0f, std::isfinite(maxCommitSeconds) ? maxCommitSeconds : 0.0f);
        return retryWindow <= 0.0f || _pullCatchIntent.commitElapsedSeconds <= retryWindow;
    }

    void Hand::notePullCatchCommitAttemptFailed()
    {
        if (!hasPendingPullCatchCommit()) {
            return;
        }

        ++_pullCatchIntent.failedCommitAttempts;
    }

    RE::TESObjectREFR* Hand::getPullCatchIntentRef() const
    {
        return _pullCatchIntent.refr;
    }

    bool Hand::reacquirePullCatchCloseSelection(RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        const RE::NiPoint3& selectionOrigin,
        const RE::NiPoint3& palmNormal,
        float radiusGameUnits,
        float maxBodyDistanceGameUnits)
    {
        /*
         * ROCK gives the pulled object a wider target-specific close grab query
         * after arrival instead of falling back to normal near/far selection.
         * This path can only restore the already-claimed ref/body stored in
         * PullCatchIntent, never switch hands to a fresh object or restart far
         * selection.
         */
        if (!_pullCatchIntent.active || !_pullCatchIntent.commitPending || !hknpWorld || !_pullCatchIntent.refr ||
            _pullCatchIntent.primaryBodyId == INVALID_BODY_ID) {
            return false;
        }

        auto* refr = _pullCatchIntent.refr;
        if (refr->IsDeleted() || refr->IsDisabled() || refr->GetFormID() != _pullCatchIntent.formId) {
            return false;
        }

        RE::NiTransform bodyWorld{};
        const auto bodyId = RE::hknpBodyId{ _pullCatchIntent.primaryBodyId };
        if (!havok_runtime::tryGetBodyArrayWorldTransform(hknpWorld, bodyId, bodyWorld) &&
            !tryResolveLiveBodyWorldTransform(hknpWorld, bodyId, bodyWorld)) {
            return false;
        }

        const float distance = pointDistanceGameUnits(selectionOrigin, bodyWorld.translate);
        const float radius = std::isfinite(radiusGameUnits) ? (std::max)(0.0f, radiusGameUnits) : 0.0f;
        const float maxBodyDistance = std::isfinite(maxBodyDistanceGameUnits) ? (std::max)(0.0f, maxBodyDistanceGameUnits) : radius;
        const float acceptedDistance = (std::max)(radius, maxBodyDistance);
        if (acceptedDistance <= 0.0f || distance > acceptedDistance) {
            return false;
        }

        SelectedObject selection{};
        selection.setReference(refr);
        selection.bodyId = bodyId;
        selection.hitPointWorld = bodyWorld.translate;
        selection.hitNormalWorld = normalizeOrFallback(selectionOrigin - bodyWorld.translate, palmNormal);
        selection.distance = distance;
        selection.signedAlongDistance = distance;
        selection.lateralDistance = 0.0f;
        selection.hitFraction = 0.0f;
        selection.targetKind = _pullCatchIntent.targetKind;
        selection.isFarSelection = false;
        selection.hasHitPoint = true;
        selection.hasHitNormal = true;
        selection.visualNode = refr->Get3D();
        selection.hitNode = selection.visualNode;
        if (bhkWorld) {
            auto bodyHandle = bodyId;
            if (auto* collisionObject = RE::bhkNPCollisionObject::Getbhk(bhkWorld, bodyHandle)) {
                selection.hitNode = collisionObject->sceneObject ? collisionObject->sceneObject : selection.hitNode;
            }
        }

        if (!selection.isValid()) {
            return false;
        }

        stopSelectionHighlight();
        _currentSelection = selection;
        applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::SelectionFoundClose });
        _selectionHoldFrames = 0;
        clearSelectedCloseFingerPose();
        playSelectionHighlight(_currentSelection);

        ROCK_LOG_DEBUG(Hand,
            "{} hand pull-catch wide reacquired close selection: formID={:08X} body={} dist={:.1f} acceptedDistance={:.1f}",
            handName(),
            _pullCatchIntent.formId,
            _pullCatchIntent.primaryBodyId,
            distance,
            acceptedDistance);
        return true;
    }

    bool Hand::beginActorEquipmentDropHandoff(
        const actor_equipment_grab::DropResult& dropResult,
        const RE::NiPoint3& sourceHitPointWorld)
    {
        if (dropResult.status != actor_equipment_grab::DropStatus::Success || !dropResult.handle) {
            return false;
        }
        if (!_currentSelection.isValid() || !_currentSelection.isFarSelection || _currentSelection.targetKind != grab_target::Kind::ActorEquipment) {
            return false;
        }

        _actorEquipmentDropHandoff = ActorEquipmentDropHandoff{
            .active = true,
            .handle = dropResult.handle,
            .droppedRef = dropResult.droppedRef,
            .sourceHitPointWorld = sourceHitPointWorld,
            .elapsedSeconds = 0.0f,
            .attempts = 0,
            .actorFormId = dropResult.actorFormId,
            .itemFormId = dropResult.itemFormId,
            .droppedFormId = dropResult.droppedFormId,
        };

        ROCK_LOG_DEBUG(Hand,
            "{} hand actor-equipment drop handoff armed: actor={:08X} item={:08X} dropped={:08X}",
            handName(),
            _actorEquipmentDropHandoff.actorFormId,
            _actorEquipmentDropHandoff.itemFormId,
            _actorEquipmentDropHandoff.droppedFormId);
        return true;
    }

    bool Hand::hasPendingActorEquipmentDropHandoff() const
    {
        return _actorEquipmentDropHandoff.active;
    }

    Hand::ActorEquipmentDropHandoffStatus Hand::advanceActorEquipmentDropHandoff(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        float deltaSeconds,
        float maxHandoffSeconds)
    {
        if (!_actorEquipmentDropHandoff.active) {
            return ActorEquipmentDropHandoffStatus::None;
        }

        if (!_currentSelection.isValid() || !_currentSelection.isFarSelection || _currentSelection.targetKind != grab_target::Kind::ActorEquipment) {
            clearActorEquipmentDropHandoff("invalid-selection");
            return ActorEquipmentDropHandoffStatus::InvalidSelection;
        }

        _actorEquipmentDropHandoff.elapsedSeconds += (std::max)(0.0f, std::isfinite(deltaSeconds) ? deltaSeconds : 0.0f);
        const float handoffWindow = (std::max)(0.0f, std::isfinite(maxHandoffSeconds) ? maxHandoffSeconds : 0.0f);

        auto droppedRef = _actorEquipmentDropHandoff.droppedRef;
        if (!droppedRef && _actorEquipmentDropHandoff.handle) {
            droppedRef = _actorEquipmentDropHandoff.handle.get();
            _actorEquipmentDropHandoff.droppedRef = droppedRef;
            _actorEquipmentDropHandoff.droppedFormId = droppedRef ? droppedRef->GetFormID() : _actorEquipmentDropHandoff.droppedFormId;
        }

        if (!droppedRef || droppedRef->IsDeleted() || droppedRef->IsDisabled()) {
            if (handoffWindow > 0.0f && _actorEquipmentDropHandoff.elapsedSeconds > handoffWindow) {
                clearActorEquipmentDropHandoff("missing-dropped-reference");
                return ActorEquipmentDropHandoffStatus::MissingDroppedReference;
            }
            return ActorEquipmentDropHandoffStatus::Pending;
        }

        ++_actorEquipmentDropHandoff.attempts;
        if (replaceFarActorEquipmentSelectionWithDroppedObject(
                bhkWorld,
                hknpWorld,
                droppedRef.get(),
                _actorEquipmentDropHandoff.sourceHitPointWorld)) {
            clearActorEquipmentDropHandoff("ready");
            return ActorEquipmentDropHandoffStatus::Ready;
        }

        if (handoffWindow > 0.0f && _actorEquipmentDropHandoff.elapsedSeconds > handoffWindow) {
            clearActorEquipmentDropHandoff("timed-out");
            return ActorEquipmentDropHandoffStatus::TimedOut;
        }

        return ActorEquipmentDropHandoffStatus::Pending;
    }

    bool Hand::replaceFarActorEquipmentSelectionWithDroppedObject(RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        RE::TESObjectREFR* droppedRef,
        const RE::NiPoint3& sourceHitPointWorld)
    {
        /*
         * The clothing drop path deliberately hands off to a normal loose-object
         * selection instead of letting actor-equipment reach dynamic pull. That
         * keeps the far-only actor resolver isolated while all physics prep,
         * pull ownership, and later two-hand grab behavior remain on the same
         * code path as any other spawned item.
         */
        if (!hknpWorld || !droppedRef || droppedRef->IsDeleted() || droppedRef->IsDisabled()) {
            return false;
        }
        if (!_currentSelection.isValid() || !_currentSelection.isFarSelection || _currentSelection.targetKind != grab_target::Kind::ActorEquipment) {
            return false;
        }

        auto* scanWorld = bhkWorld;
        if (!scanWorld) {
            auto* cell = droppedRef->GetParentCell();
            scanWorld = cell ? cell->GetbhkWorld() : nullptr;
        }
        if (!scanWorld) {
            return false;
        }

        object_physics_body_set::BodySetScanOptions scanOptions{};
        scanOptions.mode = physics_body_classifier::InteractionMode::ActiveGrab;
        scanOptions.rightHandBodyId = _isLeft ? INVALID_BODY_ID : _handBody.getBodyId().value;
        scanOptions.leftHandBodyId = _isLeft ? _handBody.getBodyId().value : INVALID_BODY_ID;
        scanOptions.sourceBodyId = _handBody.getBodyId().value;
        scanOptions.targetKind = grab_target::Kind::LooseObject;
        scanOptions.requireSameResolvedRef = true;
        scanOptions.allowUnresolvedRefBodies = true;
        scanOptions.allowWeaponRefExpansion = true;
        scanOptions.heldBySameHand = &_heldBodyIds;
        scanOptions.maxDepth = g_rockConfig.rockObjectPhysicsTreeMaxDepth;

        const auto bodySet = object_physics_body_set::scanObjectPhysicsBodySet(scanWorld, hknpWorld, droppedRef, scanOptions);
        const auto primaryChoice = bodySet.choosePrimaryBody(object_physics_body_set::INVALID_BODY_ID, object_physics_body_set::PurePoint3{ sourceHitPointWorld });
        if (primaryChoice.bodyId == object_physics_body_set::INVALID_BODY_ID) {
            ROCK_LOG_WARN(Hand,
                "{} hand actor-equipment drop could not resolve a loose-object body: dropped={:08X} scanned={} accepted={} rejected={} collisions={} visited={}",
                handName(),
                droppedRef->GetFormID(),
                bodySet.records.size(),
                bodySet.acceptedCount(),
                bodySet.rejectedCount(),
                bodySet.diagnostics.collisionObjects,
                bodySet.diagnostics.visitedNodes);
            return false;
        }

        auto previousSelection = _currentSelection;
        const auto* primaryRecord = bodySet.findRecord(primaryChoice.bodyId);
        const RE::NiPoint3 bodyPoint = primaryRecord ?
                                           RE::NiPoint3(primaryRecord->positionGame.x, primaryRecord->positionGame.y, primaryRecord->positionGame.z) :
                                           sourceHitPointWorld;

        SelectedObject replacement{};
        replacement.setReference(droppedRef);
        replacement.bodyId = RE::hknpBodyId{ primaryChoice.bodyId };
        replacement.hitNode = primaryRecord && primaryRecord->owningNode ? primaryRecord->owningNode : droppedRef->Get3D();
        replacement.visualNode = droppedRef->Get3D();
        replacement.hitPointWorld = bodyPoint;
        replacement.hitNormalWorld = previousSelection.hasHitNormal ? previousSelection.hitNormalWorld : RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
        replacement.distance = previousSelection.distance;
        replacement.signedAlongDistance = previousSelection.signedAlongDistance;
        replacement.lateralDistance = previousSelection.lateralDistance;
        replacement.hitFraction = previousSelection.hitFraction;
        replacement.hitShapeKey = previousSelection.hitShapeKey;
        replacement.hitShapeCollisionFilterInfo = previousSelection.hitShapeCollisionFilterInfo;
        replacement.targetKind = grab_target::Kind::LooseObject;
        replacement.isFarSelection = true;
        replacement.hasHitPoint = true;
        replacement.hasHitNormal = previousSelection.hasHitNormal;
        replacement.hasHitShapeKey = previousSelection.hasHitShapeKey;

        stopSelectionHighlight();
        _currentSelection = replacement;
        _cachedFarCandidate = replacement;
        applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::SelectionFoundFar });
        _selectionHoldFrames = 0;
        playSelectionHighlight(_currentSelection);

        ROCK_LOG_DEBUG(Hand,
            "{} hand actor-equipment selection replaced with dropped object: dropped={:08X} body={} reason={} previousActor={:08X} previousItem={:08X}",
            handName(),
            droppedRef->GetFormID(),
            primaryChoice.bodyId,
            primaryBodyChoiceReasonName(primaryChoice.reason),
            previousSelection.refr ? previousSelection.refr->GetFormID() : 0,
            previousSelection.actorEquipment.itemFormId);
        return true;
    }

    bool Hand::acquireForceGrabLooseSelection(RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        RE::TESObjectREFR* targetRef,
        const RE::NiPoint3& sourcePointWorld,
        std::uint32_t preferredBodyId,
        float maxDistanceGame)
    {
        if (!hknpWorld || !targetRef || targetRef->IsDeleted() || targetRef->IsDisabled()) {
            return false;
        }

        auto* scanWorld = bhkWorld;
        if (!scanWorld) {
            auto* cell = targetRef->GetParentCell();
            scanWorld = cell ? cell->GetbhkWorld() : nullptr;
        }
        if (!scanWorld) {
            return false;
        }

        object_physics_body_set::BodySetScanOptions scanOptions{};
        scanOptions.mode = physics_body_classifier::InteractionMode::ActiveGrab;
        scanOptions.rightHandBodyId = _isLeft ? INVALID_BODY_ID : _handBody.getBodyId().value;
        scanOptions.leftHandBodyId = _isLeft ? _handBody.getBodyId().value : INVALID_BODY_ID;
        scanOptions.sourceBodyId = _handBody.getBodyId().value;
        scanOptions.targetKind = grab_target::Kind::LooseObject;
        scanOptions.requireSameResolvedRef = true;
        scanOptions.allowUnresolvedRefBodies = true;
        scanOptions.allowWeaponRefExpansion = true;
        scanOptions.heldBySameHand = &_heldBodyIds;
        scanOptions.maxDepth = g_rockConfig.rockObjectPhysicsTreeMaxDepth;

        const auto bodySet = object_physics_body_set::scanObjectPhysicsBodySet(scanWorld, hknpWorld, targetRef, scanOptions);
        const auto preferred = preferredBodyId != INVALID_BODY_ID ? preferredBodyId : object_physics_body_set::INVALID_BODY_ID;
        const auto primaryChoice = bodySet.choosePrimaryBody(preferred, object_physics_body_set::PurePoint3{ sourcePointWorld });
        if (primaryChoice.bodyId == object_physics_body_set::INVALID_BODY_ID) {
            ROCK_LOG_WARN(Hand,
                "{} hand force-grab could not resolve a loose-object body: target={:08X} scanned={} accepted={} rejected={} collisions={} visited={}",
                handName(),
                targetRef->GetFormID(),
                bodySet.records.size(),
                bodySet.acceptedCount(),
                bodySet.rejectedCount(),
                bodySet.diagnostics.collisionObjects,
                bodySet.diagnostics.visitedNodes);
            return false;
        }

        const auto* primaryRecord = bodySet.findRecord(primaryChoice.bodyId);
        const RE::NiPoint3 bodyPoint = primaryRecord ?
                                           RE::NiPoint3(primaryRecord->positionGame.x, primaryRecord->positionGame.y, primaryRecord->positionGame.z) :
                                           sourcePointWorld;
        const float distance = pointDistanceGameUnits(sourcePointWorld, bodyPoint);
        const float acceptedDistance = std::isfinite(maxDistanceGame) && maxDistanceGame > 0.0f ? maxDistanceGame : g_rockConfig.rockNearDetectionRange;
        if (acceptedDistance > 0.0f && distance > acceptedDistance) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand force-grab selection rejected by distance: target={:08X} body={} distance={:.1f} max={:.1f}",
                handName(),
                targetRef->GetFormID(),
                primaryChoice.bodyId,
                distance,
                acceptedDistance);
            return false;
        }

        SelectedObject selection{};
        selection.setReference(targetRef);
        selection.bodyId = RE::hknpBodyId{ primaryChoice.bodyId };
        selection.hitNode = primaryRecord && primaryRecord->owningNode ? primaryRecord->owningNode : targetRef->Get3D();
        selection.visualNode = targetRef->Get3D();
        selection.hitPointWorld = bodyPoint;
        selection.hitNormalWorld = normalizeOrFallback(sourcePointWorld - bodyPoint, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
        selection.distance = distance;
        selection.signedAlongDistance = distance;
        selection.lateralDistance = 0.0f;
        selection.hitFraction = 0.0f;
        selection.targetKind = grab_target::Kind::LooseObject;
        selection.isFarSelection = false;
        selection.hasHitPoint = true;
        selection.hasHitNormal = true;
        selection.forcedArrival = true;

        if (!selection.isValid()) {
            return false;
        }

        stopSelectionHighlight();
        _currentSelection = selection;
        applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::SelectionFoundClose });
        _selectionHoldFrames = 0;
        clearSelectedCloseFingerPose();

        ROCK_LOG_DEBUG(Hand,
            "{} hand force-grab selection acquired: target={:08X} body={} reason={} distance={:.1f}",
            handName(),
            targetRef->GetFormID(),
            primaryChoice.bodyId,
            primaryBodyChoiceReasonName(primaryChoice.reason),
            distance);
        return true;
    }

    void Hand::clearActorEquipmentDropHandoff(const char* reason)
    {
        if (_actorEquipmentDropHandoff.active) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand actor-equipment drop handoff cleared reason={} actor={:08X} item={:08X} dropped={:08X} attempts={} elapsed={:.3f}s",
                handName(),
                reason ? reason : "cleared",
                _actorEquipmentDropHandoff.actorFormId,
                _actorEquipmentDropHandoff.itemFormId,
                _actorEquipmentDropHandoff.droppedFormId,
                _actorEquipmentDropHandoff.attempts,
                _actorEquipmentDropHandoff.elapsedSeconds);
        }
        _actorEquipmentDropHandoff = {};
    }

    void Hand::clearPullCatchIntent(const char* reason)
    {
        if (_pullCatchIntent.active) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL catch intent cleared reason={} formID={:08X} primaryBody={} pending={} elapsed={:.3f}s failedAttempts={}",
                handName(),
                reason ? reason : "cleared",
                _pullCatchIntent.formId,
                _pullCatchIntent.primaryBodyId,
                _pullCatchIntent.commitPending ? "yes" : "no",
                _pullCatchIntent.commitElapsedSeconds,
                _pullCatchIntent.failedCommitAttempts);
        }
        _pullCatchIntent = {};
    }

    void Hand::collectHeldBodyIds(RE::TESObjectREFR* refr)
    {
        _heldBodyIds.clear();
        if (!refr)
            return;
        auto* node3D = refr->Get3D();
        if (!node3D)
            return;
        collectBodyIdsRecursive(node3D);
    }

    void Hand::collectBodyIdsRecursive(RE::NiAVObject* node, int maxDepth)
    {
        if (!node || maxDepth <= 0)
            return;

        auto* collObj = node->collisionObject.get();
        struct HeldBodyCollector
        {
            std::vector<std::uint32_t>* ids = nullptr;
        } collector{ &_heldBodyIds };

        auto appendBodyId = [](std::uint32_t bodyId, void* userData) {
            auto* state = static_cast<HeldBodyCollector*>(userData);
            if (!state || !state->ids) {
                return false;
            }
            state->ids->push_back(bodyId);
            return true;
        };
        havok_runtime::forEachPhysicsSystemBodyId(collObj, nullptr, 64, appendBodyId, &collector);

        auto* niNode = node->IsNode();
        if (niNode) {
            auto& kids = niNode->GetRuntimeData().children;
            for (auto i = decltype(kids.size()){ 0 }; i < kids.size(); i++) {
                auto* kid = kids[i].get();
                if (kid)
                    collectBodyIdsRecursive(kid, maxDepth - 1);
            }
        }
    }

    bool Hand::tryResolveLivePalmAnchorReference(RE::hknpWorld* world, LivePalmAnchorReference& outReference) const
    {
        outReference = {};
        outReference.world = transform_math::makeIdentityTransform<RE::NiTransform>();

        if (!world || !_handBody.isValid() || _handBody.getBodyId().value == INVALID_BODY_ID) {
            return false;
        }

        body_frame::BodyFrameSource source = body_frame::BodyFrameSource::Fallback;
        std::uint32_t motionIndex = body_frame::kFreeMotionIndex;
        RE::NiTransform livePalmWorld{};
        if (!tryResolveLiveBodyWorldTransform(world, _handBody.getBodyId(), livePalmWorld, &source, &motionIndex)) {
            return false;
        }

        outReference.valid = true;
        outReference.world = livePalmWorld;
        outReference.source = source;
        outReference.motionIndex = motionIndex;

        const auto snapshot = havok_runtime::snapshotBody(world, _handBody.getBodyId());
        if (snapshot.valid && snapshot.motion) {
            outReference.linearVelocityHavok = RE::NiPoint3{
                snapshot.motion->linearVelocity.x,
                snapshot.motion->linearVelocity.y,
                snapshot.motion->linearVelocity.z,
            };
            outReference.angularVelocityRadiansPerSecond = RE::NiPoint3{
                snapshot.motion->angularVelocity.x,
                snapshot.motion->angularVelocity.y,
                snapshot.motion->angularVelocity.z,
            };
            outReference.hasMotionVelocity = true;
        }

        return true;
    }

    RE::NiPoint3 Hand::computeGrabPivotAWorld(RE::hknpWorld* world, const RE::NiTransform& fallbackHandWorldTransform) const
    {
        RE::NiTransform proxyFrameWorld{};
        if (tryComputeGrabProxyLocalPalmPocketFrameWorld(world, proxyFrameWorld)) {
            return proxyFrameWorld.translate;
        }

        return fallbackHandWorldTransform.translate;
    }

    bool Hand::tryComputeGrabProxyLocalPalmPocketFrameWorld(RE::hknpWorld* world, RE::NiTransform& outFrameWorld) const
    {
        outFrameWorld = {};
        LivePalmAnchorReference palmReference{};
        if (tryResolveLivePalmAnchorReference(world, palmReference) &&
            std::isfinite(palmReference.world.translate.x) &&
            std::isfinite(palmReference.world.translate.y) &&
            std::isfinite(palmReference.world.translate.z)) {
            const RE::NiTransform palmAuthorityBaseWorld =
                hand_bone_collider_geometry_math::generatedColliderFrameToGrabAuthorityFrame(palmReference.world);
            outFrameWorld = applyGrabAuthorityProxyLocalOffsetToFrame(palmAuthorityBaseWorld, _isLeft);
            bool rotationFinite = true;
            for (std::uint32_t row = 0; row < 3; ++row) {
                for (std::uint32_t column = 0; column < 3; ++column) {
                    rotationFinite = rotationFinite && std::isfinite(outFrameWorld.rotate.entry[row][column]);
                }
            }
            return rotationFinite &&
                   std::isfinite(outFrameWorld.translate.x) &&
                   std::isfinite(outFrameWorld.translate.y) &&
                   std::isfinite(outFrameWorld.translate.z) &&
                   std::isfinite(outFrameWorld.scale) &&
                   outFrameWorld.scale > 0.0001f;
        }

        return false;
    }

    bool Hand::tryComputeGrabProxyLocalPalmPocketPivotAWorld(RE::hknpWorld* world, RE::NiPoint3& outPivotWorld) const
    {
        outPivotWorld = {};
        RE::NiTransform proxyFrameWorld{};
        if (!tryComputeGrabProxyLocalPalmPocketFrameWorld(world, proxyFrameWorld)) {
            return false;
        }

        outPivotWorld = proxyFrameWorld.translate;
        return true;
    }

    bool Hand::tryGetLiveGrabFingerPoseSnapshot(GrabFingerPoseSnapshot& outSnapshot) const
    {
        outSnapshot = {};
        if (!_hasGrabFingerPose || !_grabFingerPose.solved) {
            return false;
        }

        outSnapshot.values = _grabFingerPose.values;
        outSnapshot.hasJointValues = _hasGrabFingerJointPose;
        if (_hasGrabFingerJointPose) {
            outSnapshot.jointValues = _grabFingerJointPose;
        }
        return true;
    }

    bool Hand::tryBuildSavedGrabCapture(RE::hknpWorld* world, const RE::NiTransform& proxyWorld, saved_grab_capture::HandCapture& outCapture) const
    {
        outCapture = {};
        auto* refr = getHeldRef();
        auto* rootNode = refr ? refr->Get3D() : nullptr;
        if (!rootNode || !grab_three_phase::isFinite(proxyWorld) || !grab_three_phase::isFinite(rootNode->world)) {
            return false;
        }

        auto storeFrame = [](const RE::NiTransform& transform) {
            saved_grab_capture::Frame frame{};
            frame.valid = true;
            frame.translate[0] = transform.translate.x;
            frame.translate[1] = transform.translate.y;
            frame.translate[2] = transform.translate.z;
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    frame.rotate[row * 3 + column] = transform.rotate.entry[row][column];
                }
            }
            frame.scale = transform.scale;
            return frame;
        };
        // Same helper the saved offset itself uses, so every frame in the
        // capture lands in exactly the space the label is expressed in.
        auto inProxyLocal = [&](const RE::NiTransform& world) {
            return storeFrame(grab_frame_math::objectInGeneratedProxyLocalSpace(proxyWorld, world));
        };
        auto pointInProxyLocal = [&](const RE::NiPoint3& world, float (&out)[3]) {
            const auto local = hand_bone_collider_geometry_math::generatedColliderWorldPointToLocal(proxyWorld, world);
            out[0] = local.x;
            out[1] = local.y;
            out[2] = local.z;
        };

        outCapture.present = true;
        outCapture.acquisition = _grabFrame.seatDiagnostics.acquisitionMode;
        outCapture.objectProxyLocal = inProxyLocal(rootNode->world);
        outCapture.objectScale = std::isfinite(rootNode->world.scale) && rootNode->world.scale > 0.0f ? rootNode->world.scale : 1.0f;

        /*
         * Mesh: the cached triangles the grab machinery scored, in the local
         * space of the node they were extracted from. nodeInObjectRoot ties
         * that space to the object root the saved pose is expressed for -
         * without it a nested collidable node would silently misalign the
         * mesh against the label.
         */
        auto* meshNode = _grabFrame.heldNode ? _grabFrame.heldNode : rootNode;
        const auto& triangles = _grabFrame.localMeshTriangles;
        auto& mesh = outCapture.mesh;
        mesh.valid = !triangles.empty();
        mesh.triangleCount = static_cast<std::uint32_t>(triangles.size());
        mesh.nodeInObjectRoot = storeFrame(grab_frame_math::objectInGeneratedProxyLocalSpace(rootNode->world, meshNode->world));
        if (mesh.valid) {
            mesh.verticesObjectLocal.reserve(triangles.size() * 9);
            RE::NiPoint3 aabbMin{ std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
            RE::NiPoint3 aabbMax{ std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };
            double areaSum = 0.0;
            double centroidAccum[3]{};
            for (const auto& triangle : triangles) {
                const RE::NiPoint3 vertices[3]{ triangle.v0, triangle.v1, triangle.v2 };
                for (const auto& vertex : vertices) {
                    mesh.verticesObjectLocal.push_back(vertex.x);
                    mesh.verticesObjectLocal.push_back(vertex.y);
                    mesh.verticesObjectLocal.push_back(vertex.z);
                    aabbMin.x = (std::min)(aabbMin.x, vertex.x);
                    aabbMin.y = (std::min)(aabbMin.y, vertex.y);
                    aabbMin.z = (std::min)(aabbMin.z, vertex.z);
                    aabbMax.x = (std::max)(aabbMax.x, vertex.x);
                    aabbMax.y = (std::max)(aabbMax.y, vertex.y);
                    aabbMax.z = (std::max)(aabbMax.z, vertex.z);
                }
                const RE::NiPoint3 edge0 = triangle.v1 - triangle.v0;
                const RE::NiPoint3 edge1 = triangle.v2 - triangle.v0;
                const RE::NiPoint3 cross{
                    edge0.y * edge1.z - edge0.z * edge1.y,
                    edge0.z * edge1.x - edge0.x * edge1.z,
                    edge0.x * edge1.y - edge0.y * edge1.x,
                };
                const double area = 0.5 * std::sqrt((std::max)(0.0f, cross.x * cross.x + cross.y * cross.y + cross.z * cross.z));
                if (!(area > 0.0)) {
                    continue;
                }
                areaSum += area;
                centroidAccum[0] += area * (triangle.v0.x + triangle.v1.x + triangle.v2.x) / 3.0;
                centroidAccum[1] += area * (triangle.v0.y + triangle.v1.y + triangle.v2.y) / 3.0;
                centroidAccum[2] += area * (triangle.v0.z + triangle.v1.z + triangle.v2.z) / 3.0;
            }
            mesh.aabbMinObjectLocal[0] = aabbMin.x;
            mesh.aabbMinObjectLocal[1] = aabbMin.y;
            mesh.aabbMinObjectLocal[2] = aabbMin.z;
            mesh.aabbMaxObjectLocal[0] = aabbMax.x;
            mesh.aabbMaxObjectLocal[1] = aabbMax.y;
            mesh.aabbMaxObjectLocal[2] = aabbMax.z;
            if (areaSum > 0.0) {
                mesh.areaCentroidObjectLocal[0] = static_cast<float>(centroidAccum[0] / areaSum);
                mesh.areaCentroidObjectLocal[1] = static_cast<float>(centroidAccum[1] / areaSum);
                mesh.areaCentroidObjectLocal[2] = static_cast<float>(centroidAccum[2] / areaSum);
            }
        }

        /*
         * Centre of mass from the held body's Havok motion. Offset authority is
         * this binary's own reflection table: hknpMotion member
         * 'centerOfMassAndMassFactor' at +0x00 (verified 2026-07-25). Mass
         * distribution cannot be recovered from the render mesh, so this is
         * captured as a first-class field rather than inferred later from the
         * body frame.
         *
         * Gated, not trusted: a rigid body's centre of mass is always inside
         * its own convex hull, so a COM outside the mesh bounds means the read
         * is wrong. That flags the record instead of quietly feeding a bad
         * label into a fit.
         */
        if (world && mesh.valid) {
            if (auto* motion = havok_runtime::getBodyMotion(world, getSavedObjectState().bodyId)) {
                const RE::NiPoint3 comWorld = hkVectorToNiPoint(motion->position);
                if (std::isfinite(comWorld.x) && std::isfinite(comWorld.y) && std::isfinite(comWorld.z)) {
                    const RE::NiPoint3 comLocal = transform_math::worldPointToLocal(meshNode->world, comWorld);
                    auto& physics = outCapture.physics;
                    physics.hasCenterOfMass = true;
                    physics.comObjectLocal[0] = comLocal.x;
                    physics.comObjectLocal[1] = comLocal.y;
                    physics.comObjectLocal[2] = comLocal.z;

                    const float comAxis[3]{ comLocal.x, comLocal.y, comLocal.z };
                    bool insideBounds = true;
                    for (std::size_t axis = 0; axis < 3; ++axis) {
                        const float minBound = mesh.aabbMinObjectLocal[axis];
                        const float maxBound = mesh.aabbMaxObjectLocal[axis];
                        const float margin = (std::max)(0.25f * (maxBound - minBound), 0.5f);
                        if (comAxis[axis] < minBound - margin || comAxis[axis] > maxBound + margin) {
                            insideBounds = false;
                            break;
                        }
                    }
                    physics.comTrusted = insideBounds;
                    if (!insideBounds) {
                        ROCK_LOG_WARN(Hand,
                            "{} saved grab capture: motion centre of mass ({:.2f},{:.2f},{:.2f}) falls outside the mesh bounds "
                            "([{:.2f},{:.2f}] [{:.2f},{:.2f}] [{:.2f},{:.2f}]) -> recorded untrusted",
                            handName(),
                            comLocal.x, comLocal.y, comLocal.z,
                            mesh.aabbMinObjectLocal[0], mesh.aabbMaxObjectLocal[0],
                            mesh.aabbMinObjectLocal[1], mesh.aabbMaxObjectLocal[1],
                            mesh.aabbMinObjectLocal[2], mesh.aabbMaxObjectLocal[2]);
                    }
                }
            }
        }

        /*
         * Hand geometry: the palm and fingertip frames the collider set is
         * actually driving this frame, not a reconstruction. These are the
         * volumes the object must stay out of, so a solver fitted here is
         * fitted against the real hand. Mid-finger segments are not published
         * by the twin targets; the fingertip contacts below carry the wrap
         * evidence those segments would otherwise be needed for.
         */
        const auto& twins = dynamicTwinTargets();
        if (twins.palm.valid) {
            outCapture.palm.valid = true;
            outCapture.palm.role = "palm";
            outCapture.palm.frameProxyLocal = inProxyLocal(twins.palm.target);
            outCapture.palm.length = twins.palm.length;
            outCapture.palm.radius = twins.palm.radius;
            outCapture.palm.convexRadius = twins.palm.convexRadius;
        }
        // Every driven segment, not only the fingertips: the non-penetration
        // constraint a solver is fitted under applies to the whole hand volume.
        for (const auto& segment : _boneColliders.segmentColliderFrames()) {
            if (!segment.valid) {
                continue;
            }
            saved_grab_capture::ColliderSlot capture{};
            capture.valid = true;
            capture.role = hand_collider_semantics::roleName(segment.role);
            capture.frameProxyLocal = inProxyLocal(segment.target);
            capture.length = segment.length;
            capture.radius = segment.radius;
            capture.convexRadius = segment.convexRadius;
            outCapture.fingerSegments.push_back(std::move(capture));
        }

        GrabFingerPoseSnapshot fingerSnapshot{};
        if (tryGetLiveGrabFingerPoseSnapshot(fingerSnapshot)) {
            for (std::size_t finger = 0; finger < saved_grab_capture::kFingerCount; ++finger) {
                outCapture.fingerCurls[finger] = fingerSnapshot.values[finger];
            }
            outCapture.hasFingerJointValues = fingerSnapshot.hasJointValues;
            if (fingerSnapshot.hasJointValues) {
                for (std::size_t joint = 0; joint < saved_grab_capture::kFingerJointValueCount; ++joint) {
                    outCapture.fingerJointValues[joint] = fingerSnapshot.jointValues[joint];
                }
            }
        }

        /*
         * Contacts: where each fingertip collider actually reaches the mesh in
         * the saved pose. This is the numeric definition of "properly held"
         * for this object - which fingers found surface, at what point, and
         * against which normal - and it is the label a wrap-quality term gets
         * fitted against.
         */
        if (mesh.valid && grab_three_phase::isFinite(meshNode->world)) {
            const float meshScale = std::isfinite(meshNode->world.scale) && meshNode->world.scale > 0.0f ? meshNode->world.scale : 1.0f;
            constexpr float kContactSkinGameUnits = 0.5f;
            for (std::size_t finger = 0; finger < twins.fingertips.size() && finger < saved_grab_capture::kFingerCount; ++finger) {
                const auto& slot = twins.fingertips[finger];
                if (!slot.valid || !grab_three_phase::isFinite(slot.target)) {
                    continue;
                }
                const RE::NiPoint3 tipLocal = transform_math::worldPointToLocal(meshNode->world, slot.target.translate);
                float bestDistanceSquared = std::numeric_limits<float>::max();
                RE::NiPoint3 bestPoint{};
                RE::NiPoint3 bestNormal{};
                for (const auto& triangle : triangles) {
                    const TriangleData candidateTriangle{ triangle.v0, triangle.v1, triangle.v2 };
                    float distanceSquared = 0.0f;
                    const RE::NiPoint3 candidate = closestPointOnTriangleToPoint(tipLocal, candidateTriangle, distanceSquared);
                    if (!std::isfinite(distanceSquared) || distanceSquared >= bestDistanceSquared) {
                        continue;
                    }
                    bestDistanceSquared = distanceSquared;
                    bestPoint = candidate;
                    const RE::NiPoint3 edge0 = triangle.v1 - triangle.v0;
                    const RE::NiPoint3 edge1 = triangle.v2 - triangle.v0;
                    bestNormal = RE::NiPoint3{
                        edge0.y * edge1.z - edge0.z * edge1.y,
                        edge0.z * edge1.x - edge0.x * edge1.z,
                        edge0.x * edge1.y - edge0.y * edge1.x,
                    };
                }
                if (bestDistanceSquared == std::numeric_limits<float>::max()) {
                    continue;
                }
                const float normalLength = std::sqrt((std::max)(0.0f,
                    bestNormal.x * bestNormal.x + bestNormal.y * bestNormal.y + bestNormal.z * bestNormal.z));
                auto& contact = outCapture.fingerContacts[finger];
                contact.curl = outCapture.fingerCurls[finger];
                contact.pointObjectLocal[0] = bestPoint.x;
                contact.pointObjectLocal[1] = bestPoint.y;
                contact.pointObjectLocal[2] = bestPoint.z;
                if (normalLength > 0.000001f) {
                    contact.normalObjectLocal[0] = bestNormal.x / normalLength;
                    contact.normalObjectLocal[1] = bestNormal.y / normalLength;
                    contact.normalObjectLocal[2] = bestNormal.z / normalLength;
                }
                const float gapGameUnits = std::sqrt((std::max)(0.0f, bestDistanceSquared)) * meshScale;
                contact.touching = gapGameUnits <= slot.radius + kContactSkinGameUnits;
            }
        }

        const auto& telemetry = _grabFrame.captureTelemetry;
        const auto& diagnostics = telemetry.seatDiagnostics;
        auto& seat = outCapture.seat;
        seat.valid = _grabFrame.hasTelemetryCapture;
        seat.objectProxyLocal = inProxyLocal(telemetry.desiredObjectWorld);
        seat.shapeClass = diagnostics.shapeClass;
        seat.elongationRatio = diagnostics.elongationRatio;
        seat.secondElongationRatio = diagnostics.secondElongationRatio;
        seat.alignmentAngleDegrees = diagnostics.alignmentAngleDegrees;
        seat.alignmentReason = diagnostics.alignmentReason;
        seat.rollAngleDegrees = diagnostics.rollAngleDegrees;
        seat.rollReason = diagnostics.rollReason;
        seat.depthGameUnits = diagnostics.depthGameUnits;
        seat.depthOffsetGameUnits = diagnostics.depthOffsetGameUnits;
        seat.depthReason = diagnostics.depthReason;
        seat.penetrationBackstopGameUnits = diagnostics.penetrationBackstopGameUnits;
        seat.penetrationBackstopReason = diagnostics.penetrationBackstopReason;
        seat.gripPointObjectLocal[0] = telemetry.gripPointLocal.x;
        seat.gripPointObjectLocal[1] = telemetry.gripPointLocal.y;
        seat.gripPointObjectLocal[2] = telemetry.gripPointLocal.z;
        pointInProxyLocal(telemetry.grabPivotWorld, seat.pivotProxyLocal);
        seat.seatMode = grabSeatModeName(_grabFrame.seatMode);
        seat.pivotAuthoritySource = telemetry.pivotAuthoritySource ? telemetry.pivotAuthoritySource : "none";

        // Havok body relative to the object node, both frozen at the same
        // instant at capture. Raw data only - see PhysicsCapture for why this
        // must not be read as a centre of mass until the layout is verified.
        outCapture.physics.bodyInObjectNode =
            storeFrame(grab_frame_math::objectInGeneratedProxyLocalSpace(telemetry.objectNodeWorld, telemetry.bodyWorld));

        auto& tuning = outCapture.tuning;
        tuning.seatDepthMaxGameUnits = g_rockConfig.rockGrabSeatDepthMaxGameUnits;
        tuning.seatDepthFootprintRadiusGameUnits = g_rockConfig.rockGrabSeatDepthFootprintRadiusGameUnits;
        tuning.seatPenetrationBackstopFootprintRadiusGameUnits = g_rockConfig.rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits;
        tuning.seatDepthSkinGameUnits = g_rockConfig.rockGrabSeatDepthSkinGameUnits;
        tuning.gripInsetGameUnits = g_rockConfig.rockGrabGripInsetGameUnits;
        tuning.pullPresentationMinElongationRatio = g_rockConfig.rockPullPresentationMinElongationRatio;
        tuning.pullPresentationGripAxisTiltDegrees = g_rockConfig.rockPullPresentationGripAxisTiltDegrees;
        tuning.seatRollMinSecondElongationRatio = g_rockConfig.rockGrabSeatRollMinSecondElongationRatio;
        tuning.pocketDepthGameUnits = g_rockConfig.rockGrabPocketDepthGameUnits;
        tuning.pocketRadiusGameUnits = g_rockConfig.rockGrabPocketRadiusGameUnits;
        return true;
    }

    bool Hand::getGrabAuthorityProxyDebugSnapshot(RE::hknpWorld* world, const RE::NiTransform& rawHandWorld, GrabAuthorityProxyDebugSnapshot& out) const
    {
        /*
         * Offset tuning needs the proxy-local seat point before a grab creates
         * the real no-contact proxy body. This marker intentionally follows the
         * generated palm authority frame used by active proxy readback.
         */
        out = {};
        (void)rawHandWorld;

        LivePalmAnchorReference palmReference{};
        if (!tryResolveLivePalmAnchorReference(world, palmReference) ||
            !std::isfinite(palmReference.world.translate.x) ||
            !std::isfinite(palmReference.world.translate.y) ||
            !std::isfinite(palmReference.world.translate.z)) {
            return false;
        }

        const RE::NiTransform palmAuthorityBaseWorld =
            hand_bone_collider_geometry_math::generatedColliderFrameToGrabAuthorityFrame(palmReference.world);
        out.palmAuthorityBaseWorld = palmAuthorityBaseWorld;
        out.proxyTargetWorld = applyGrabAuthorityProxyLocalOffsetToFrame(palmAuthorityBaseWorld, _isLeft);
        out.localOffsetGameUnits = computeGrabAuthorityProxyOffsetLocalGame(_isLeft);
        out.palmSource = palmReference.source;
        out.palmMotionIndex = palmReference.motionIndex;
        return true;
    }

    void Hand::recordSemanticContact(const HandColliderBodyMetadata& metadata,
        std::uint32_t otherBodyId,
        const hand_semantic_contact_state::SemanticContactVector* contactPointGame,
        const hand_semantic_contact_state::SemanticContactVector* contactNormalGame)
    {
        if (!metadata.valid || metadata.bodyId == hand_semantic_contact_state::kInvalidBodyId || otherBodyId == hand_semantic_contact_state::kInvalidBodyId) {
            return;
        }
        if (suppressesGeneratedHandContactEvidence(getStateAtomic())) {
            return;
        }

        /*
         * Semantic contact readers are lock-free sequence snapshots, but the
         * physics callback producer and the game-frame ownership clearer are two
         * different writers. Serialize only those writes so an already-dispatched
         * callback cannot republish a stale contact after the owner transition
         * invalidated the slots.
         */
        std::scoped_lock writeLock(_semanticContactWriteMutex);
        if (suppressesGeneratedHandContactEvidence(getStateAtomic())) {
            return;
        }

        const bool hasContactPointGame =
            contactPointGame && hand_semantic_contact_state::isFiniteVector(*contactPointGame);
        const bool hasContactNormalGame =
            contactNormalGame && hand_semantic_contact_state::isFiniteVector(*contactNormalGame) &&
            (contactNormalGame->x * contactNormalGame->x +
                contactNormalGame->y * contactNormalGame->y +
                contactNormalGame->z * contactNormalGame->z) > 1.0e-6f;
        const hand_semantic_contact_state::SemanticContactVector emptyVector{};
        const auto& storedPoint = hasContactPointGame ? *contactPointGame : emptyVector;
        const auto& storedNormal = hasContactNormalGame ? *contactNormalGame : emptyVector;

        const std::uint32_t contactFrame = _semanticContactFrameCounter.load(std::memory_order_acquire);
        _semanticContactValid.store(0, std::memory_order_release);
        std::uint32_t contactSequence = _semanticContactSequence.fetch_add(1, std::memory_order_acq_rel) + 1;
        if ((contactSequence & 1u) == 0) {
            contactSequence = _semanticContactSequence.fetch_add(1, std::memory_order_acq_rel) + 1;
        }
        _semanticContactRole.store(static_cast<std::uint32_t>(metadata.role), std::memory_order_release);
        _semanticContactFinger.store(static_cast<std::uint32_t>(metadata.finger), std::memory_order_release);
        _semanticContactSegment.store(static_cast<std::uint32_t>(metadata.segment), std::memory_order_release);
        _semanticContactHandBodyId.store(metadata.bodyId, std::memory_order_release);
        _semanticContactOtherBodyId.store(otherBodyId, std::memory_order_release);
        _semanticContactFrames.store(contactFrame, std::memory_order_release);
        _semanticContactPointGameX.store(storedPoint.x, std::memory_order_release);
        _semanticContactPointGameY.store(storedPoint.y, std::memory_order_release);
        _semanticContactPointGameZ.store(storedPoint.z, std::memory_order_release);
        _semanticContactNormalGameX.store(storedNormal.x, std::memory_order_release);
        _semanticContactNormalGameY.store(storedNormal.y, std::memory_order_release);
        _semanticContactNormalGameZ.store(storedNormal.z, std::memory_order_release);
        _semanticContactHasPointGame.store(hasContactPointGame ? 1u : 0u, std::memory_order_release);
        _semanticContactHasNormalGame.store(hasContactNormalGame ? 1u : 0u, std::memory_order_release);
        _semanticContactSequence.store(contactSequence + 1, std::memory_order_release);
        _semanticContactValid.store(1, std::memory_order_release);

        const std::size_t slot = hand_semantic_contact_state::semanticContactSlotForRole(metadata.role);
        const bool continuesContactRun =
            hand_semantic_contact_state::semanticContactContinuesRun(
                _semanticContactSetValid[slot].load(std::memory_order_acquire) != 0,
                _semanticContactSetOtherBodyId[slot].load(std::memory_order_acquire),
                otherBodyId,
                contactFrame,
                _semanticContactSetFrames[slot].load(std::memory_order_acquire));
        const std::uint32_t contactRunStartFrame = continuesContactRun ?
            _semanticContactSetRunStartFrames[slot].load(std::memory_order_acquire) :
            contactFrame;
        _semanticContactSetValid[slot].store(0, std::memory_order_release);
        std::uint32_t slotSequence = _semanticContactSetSequence[slot].fetch_add(1, std::memory_order_acq_rel) + 1;
        if ((slotSequence & 1u) == 0) {
            slotSequence = _semanticContactSetSequence[slot].fetch_add(1, std::memory_order_acq_rel) + 1;
        }
        _semanticContactSetRole[slot].store(static_cast<std::uint32_t>(metadata.role), std::memory_order_release);
        _semanticContactSetFinger[slot].store(static_cast<std::uint32_t>(metadata.finger), std::memory_order_release);
        _semanticContactSetSegment[slot].store(static_cast<std::uint32_t>(metadata.segment), std::memory_order_release);
        _semanticContactSetHandBodyId[slot].store(metadata.bodyId, std::memory_order_release);
        _semanticContactSetOtherBodyId[slot].store(otherBodyId, std::memory_order_release);
        _semanticContactSetFrames[slot].store(contactFrame, std::memory_order_release);
        _semanticContactSetRunStartFrames[slot].store(contactRunStartFrame, std::memory_order_release);
        _semanticContactSetPointGameX[slot].store(storedPoint.x, std::memory_order_release);
        _semanticContactSetPointGameY[slot].store(storedPoint.y, std::memory_order_release);
        _semanticContactSetPointGameZ[slot].store(storedPoint.z, std::memory_order_release);
        _semanticContactSetNormalGameX[slot].store(storedNormal.x, std::memory_order_release);
        _semanticContactSetNormalGameY[slot].store(storedNormal.y, std::memory_order_release);
        _semanticContactSetNormalGameZ[slot].store(storedNormal.z, std::memory_order_release);
        _semanticContactSetHasPointGame[slot].store(hasContactPointGame ? 1u : 0u, std::memory_order_release);
        _semanticContactSetHasNormalGame[slot].store(hasContactNormalGame ? 1u : 0u, std::memory_order_release);
        _semanticContactSetSequence[slot].store(slotSequence + 1, std::memory_order_release);
        _semanticContactSetValid[slot].store(1, std::memory_order_release);
    }

    void Hand::clearSemanticContactEvidence()
    {
        /*
         * Contact events may arrive on the physics callback boundary while the
         * game-frame hand is entering or leaving grab ownership. Invalidate the
         * semantic contact slots atomically instead of waiting for age-based
         * expiry so stale hand-body evidence cannot seed the next grab frame.
         */
        std::scoped_lock writeLock(_semanticContactWriteMutex);
        _semanticContactValid.store(0, std::memory_order_release);
        _semanticContactHandBodyId.store(hand_semantic_contact_state::kInvalidBodyId, std::memory_order_release);
        _semanticContactOtherBodyId.store(hand_semantic_contact_state::kInvalidBodyId, std::memory_order_release);
        _semanticContactFrames.store(0xFFFF'FFFFu, std::memory_order_release);
        _semanticContactHasPointGame.store(0, std::memory_order_release);
        _semanticContactHasNormalGame.store(0, std::memory_order_release);
        _semanticContactSequence.fetch_add(2, std::memory_order_acq_rel);

        for (std::size_t i = 0; i < hand_semantic_contact_state::kMaxSemanticContactRecords; ++i) {
            _semanticContactSetValid[i].store(0, std::memory_order_release);
            _semanticContactSetHandBodyId[i].store(hand_semantic_contact_state::kInvalidBodyId, std::memory_order_release);
            _semanticContactSetOtherBodyId[i].store(hand_semantic_contact_state::kInvalidBodyId, std::memory_order_release);
            _semanticContactSetFrames[i].store(0xFFFF'FFFFu, std::memory_order_release);
            _semanticContactSetRunStartFrames[i].store(0xFFFF'FFFFu, std::memory_order_release);
            _semanticContactSetHasPointGame[i].store(0, std::memory_order_release);
            _semanticContactSetHasNormalGame[i].store(0, std::memory_order_release);
            _semanticContactSetSequence[i].fetch_add(2, std::memory_order_acq_rel);
        }
    }

    void Hand::tickSemanticContactState()
    {
        _semanticContactFrameCounter.fetch_add(1, std::memory_order_acq_rel);
    }

    bool Hand::getLastSemanticContact(hand_semantic_contact_state::SemanticContactRecord& outContact) const
    {
        outContact = {};
        for (int attempt = 0; attempt < 3; ++attempt) {
            const auto sequenceBefore = _semanticContactSequence.load(std::memory_order_acquire);
            if ((sequenceBefore & 1u) != 0) {
                continue;
            }
            if (_semanticContactValid.load(std::memory_order_acquire) == 0) {
                return false;
            }

            hand_semantic_contact_state::SemanticContactRecord contact{};
            contact.valid = true;
            contact.isLeft = _isLeft;
            contact.role = static_cast<hand_collider_semantics::HandColliderRole>(_semanticContactRole.load(std::memory_order_acquire));
            contact.finger = static_cast<hand_collider_semantics::HandFinger>(_semanticContactFinger.load(std::memory_order_acquire));
            contact.segment = static_cast<hand_collider_semantics::HandFingerSegment>(_semanticContactSegment.load(std::memory_order_acquire));
            contact.handBodyId = _semanticContactHandBodyId.load(std::memory_order_acquire);
            contact.otherBodyId = _semanticContactOtherBodyId.load(std::memory_order_acquire);
            const auto contactFrame = _semanticContactFrames.load(std::memory_order_acquire);
            contact.contactFrame = contactFrame;
            contact.sequence = sequenceBefore;
            contact.framesSinceContact = hand_semantic_contact_state::semanticFramesSinceContact(
                _semanticContactFrameCounter.load(std::memory_order_acquire),
                contactFrame);
            contact.hasContactPointGame = _semanticContactHasPointGame.load(std::memory_order_acquire) != 0;
            contact.hasContactNormalGame = _semanticContactHasNormalGame.load(std::memory_order_acquire) != 0;
            contact.contactPointGame = hand_semantic_contact_state::SemanticContactVector{
                _semanticContactPointGameX.load(std::memory_order_acquire),
                _semanticContactPointGameY.load(std::memory_order_acquire),
                _semanticContactPointGameZ.load(std::memory_order_acquire),
            };
            contact.contactNormalGame = hand_semantic_contact_state::SemanticContactVector{
                _semanticContactNormalGameX.load(std::memory_order_acquire),
                _semanticContactNormalGameY.load(std::memory_order_acquire),
                _semanticContactNormalGameZ.load(std::memory_order_acquire),
            };
            const auto sequenceAfter = _semanticContactSequence.load(std::memory_order_acquire);
            if (!hand_semantic_contact_state::semanticContactSequenceSnapshotStable(sequenceBefore, sequenceAfter)) {
                continue;
            }
            if (contact.handBodyId == hand_semantic_contact_state::kInvalidBodyId || contact.otherBodyId == hand_semantic_contact_state::kInvalidBodyId) {
                return false;
            }
            if (contact.hasContactPointGame && !hand_semantic_contact_state::isFiniteVector(contact.contactPointGame)) {
                contact.hasContactPointGame = false;
            }
            if (contact.hasContactNormalGame && !hand_semantic_contact_state::hasUsableContactNormal(contact)) {
                contact.hasContactNormalGame = false;
            }
            outContact = contact;
            return true;
        }
        return false;
    }

    bool Hand::getFreshSemanticContactForRole(
        hand_collider_semantics::HandColliderRole role,
        std::uint32_t maxFramesSinceContact,
        hand_semantic_contact_state::SemanticContactRecord& outContact) const
    {
        outContact = {};
        const std::size_t slot = hand_semantic_contact_state::semanticContactSlotForRole(role);
        for (int attempt = 0; attempt < 3; ++attempt) {
            const auto sequenceBefore = _semanticContactSetSequence[slot].load(std::memory_order_acquire);
            if ((sequenceBefore & 1u) != 0) {
                continue;
            }
            if (_semanticContactSetValid[slot].load(std::memory_order_acquire) == 0) {
                return false;
            }

            hand_semantic_contact_state::SemanticContactRecord contact{};
            contact.valid = true;
            contact.isLeft = _isLeft;
            contact.role = static_cast<hand_collider_semantics::HandColliderRole>(_semanticContactSetRole[slot].load(std::memory_order_acquire));
            contact.finger = static_cast<hand_collider_semantics::HandFinger>(_semanticContactSetFinger[slot].load(std::memory_order_acquire));
            contact.segment = static_cast<hand_collider_semantics::HandFingerSegment>(_semanticContactSetSegment[slot].load(std::memory_order_acquire));
            contact.handBodyId = _semanticContactSetHandBodyId[slot].load(std::memory_order_acquire);
            contact.otherBodyId = _semanticContactSetOtherBodyId[slot].load(std::memory_order_acquire);
            const auto contactFrame = _semanticContactSetFrames[slot].load(std::memory_order_acquire);
            contact.contactFrame = contactFrame;
            contact.contactRunStartFrame = _semanticContactSetRunStartFrames[slot].load(std::memory_order_acquire);
            contact.sequence = sequenceBefore;
            contact.framesSinceContact = hand_semantic_contact_state::semanticFramesSinceContact(
                _semanticContactFrameCounter.load(std::memory_order_acquire),
                contactFrame);
            contact.hasContactPointGame = _semanticContactSetHasPointGame[slot].load(std::memory_order_acquire) != 0;
            contact.hasContactNormalGame = _semanticContactSetHasNormalGame[slot].load(std::memory_order_acquire) != 0;
            contact.contactPointGame = hand_semantic_contact_state::SemanticContactVector{
                _semanticContactSetPointGameX[slot].load(std::memory_order_acquire),
                _semanticContactSetPointGameY[slot].load(std::memory_order_acquire),
                _semanticContactSetPointGameZ[slot].load(std::memory_order_acquire),
            };
            contact.contactNormalGame = hand_semantic_contact_state::SemanticContactVector{
                _semanticContactSetNormalGameX[slot].load(std::memory_order_acquire),
                _semanticContactSetNormalGameY[slot].load(std::memory_order_acquire),
                _semanticContactSetNormalGameZ[slot].load(std::memory_order_acquire),
            };
            const auto sequenceAfter = _semanticContactSetSequence[slot].load(std::memory_order_acquire);
            if (!hand_semantic_contact_state::semanticContactSequenceSnapshotStable(sequenceBefore, sequenceAfter)) {
                continue;
            }

            if (contact.role != role ||
                contact.handBodyId == hand_semantic_contact_state::kInvalidBodyId ||
                contact.otherBodyId == hand_semantic_contact_state::kInvalidBodyId ||
                contact.framesSinceContact > maxFramesSinceContact) {
                return false;
            }
            if (contact.hasContactPointGame && !hand_semantic_contact_state::isFiniteVector(contact.contactPointGame)) {
                contact.hasContactPointGame = false;
            }
            if (contact.hasContactNormalGame && !hand_semantic_contact_state::hasUsableContactNormal(contact)) {
                contact.hasContactNormalGame = false;
            }
            outContact = contact;
            return true;
        }
        return false;
    }

    hand_semantic_contact_state::SemanticContactCollection Hand::collectFreshSemanticContacts(
        const std::uint32_t maxFramesSinceContact) const
    {
        hand_semantic_contact_state::SemanticContactCollection contacts{};

        for (std::size_t i = 0; i < hand_semantic_contact_state::kMaxSemanticContactRecords; ++i) {
            for (int attempt = 0; attempt < 3; ++attempt) {
                const auto sequenceBefore = _semanticContactSetSequence[i].load(std::memory_order_acquire);
                if ((sequenceBefore & 1u) != 0) {
                    continue;
                }
                if (_semanticContactSetValid[i].load(std::memory_order_acquire) == 0) {
                    break;
                }

                hand_semantic_contact_state::SemanticContactRecord record{};
                record.valid = true;
                record.isLeft = _isLeft;
                record.role = static_cast<hand_collider_semantics::HandColliderRole>(_semanticContactSetRole[i].load(std::memory_order_acquire));
                record.finger = static_cast<hand_collider_semantics::HandFinger>(_semanticContactSetFinger[i].load(std::memory_order_acquire));
                record.segment = static_cast<hand_collider_semantics::HandFingerSegment>(_semanticContactSetSegment[i].load(std::memory_order_acquire));
                record.handBodyId = _semanticContactSetHandBodyId[i].load(std::memory_order_acquire);
                record.otherBodyId = _semanticContactSetOtherBodyId[i].load(std::memory_order_acquire);
                const auto contactFrame = _semanticContactSetFrames[i].load(std::memory_order_acquire);
                record.contactFrame = contactFrame;
                record.contactRunStartFrame = _semanticContactSetRunStartFrames[i].load(std::memory_order_acquire);
                record.framesSinceContact = hand_semantic_contact_state::semanticFramesSinceContact(
                    _semanticContactFrameCounter.load(std::memory_order_acquire),
                    contactFrame);
                record.sequence = sequenceBefore;
                record.hasContactPointGame = _semanticContactSetHasPointGame[i].load(std::memory_order_acquire) != 0;
                record.hasContactNormalGame = _semanticContactSetHasNormalGame[i].load(std::memory_order_acquire) != 0;
                record.contactPointGame = hand_semantic_contact_state::SemanticContactVector{
                    _semanticContactSetPointGameX[i].load(std::memory_order_acquire),
                    _semanticContactSetPointGameY[i].load(std::memory_order_acquire),
                    _semanticContactSetPointGameZ[i].load(std::memory_order_acquire),
                };
                record.contactNormalGame = hand_semantic_contact_state::SemanticContactVector{
                    _semanticContactSetNormalGameX[i].load(std::memory_order_acquire),
                    _semanticContactSetNormalGameY[i].load(std::memory_order_acquire),
                    _semanticContactSetNormalGameZ[i].load(std::memory_order_acquire),
                };
                const auto sequenceAfter = _semanticContactSetSequence[i].load(std::memory_order_acquire);
                if (!hand_semantic_contact_state::semanticContactSequenceSnapshotStable(sequenceBefore, sequenceAfter)) {
                    continue;
                }

                if (record.handBodyId == hand_semantic_contact_state::kInvalidBodyId ||
                    record.otherBodyId == hand_semantic_contact_state::kInvalidBodyId) {
                    break;
                }
                if (record.hasContactPointGame && !hand_semantic_contact_state::isFiniteVector(record.contactPointGame)) {
                    record.hasContactPointGame = false;
                }
                if (record.hasContactNormalGame && !hand_semantic_contact_state::hasUsableContactNormal(record)) {
                    record.hasContactNormalGame = false;
                }
                if (record.framesSinceContact <= maxFramesSinceContact) {
                    contacts.add(record);
                }
                break;
            }
        }

        return contacts;
    }

    hand_semantic_contact_state::SemanticContactCollection Hand::collectFreshSemanticContactsForBody(
        const std::uint32_t targetBodyId,
        const std::uint32_t maxFramesSinceContact) const
    {
        hand_semantic_contact_state::SemanticContactCollection matching{};
        if (targetBodyId == hand_semantic_contact_state::kInvalidBodyId) {
            return matching;
        }

        const auto contacts =
            collectFreshSemanticContacts(maxFramesSinceContact);
        for (std::size_t index = 0; index < contacts.count; ++index) {
            if (contacts.records[index].otherBodyId == targetBodyId) {
                matching.add(contacts.records[index]);
            }
        }
        return matching;
    }

    bool Hand::tryGetHandColliderMetadataForRole(hand_collider_semantics::HandColliderRole role, HandColliderBodyMetadata& outMetadata) const
    {
        outMetadata = {};
        const auto count = getHandColliderBodyCount();
        for (std::uint32_t i = 0; i < count; ++i) {
            const std::uint32_t bodyId = getHandColliderBodyIdAtomic(i);
            if (bodyId == hand_collider_semantics::kInvalidBodyId) {
                continue;
            }
            HandColliderBodyMetadata metadata{};
            if (!tryGetHandColliderMetadata(bodyId, metadata) || !metadata.valid) {
                continue;
            }
            if (metadata.role == role) {
                outMetadata = metadata;
                return true;
            }
        }
        return false;
    }

    bool Hand::isFingerTouching(hand_collider_semantics::HandFinger finger) const
    {
        hand_semantic_contact_state::SemanticContactRecord contact{};
        return getLastSemanticContact(contact) && contact.framesSinceContact < 5 && contact.finger == finger;
    }

    bool Hand::isFingerTipTouching(hand_collider_semantics::HandFinger finger) const
    {
        hand_semantic_contact_state::SemanticContactRecord contact{};
        return getLastSemanticContact(contact) && contact.framesSinceContact < 5 && contact.finger == finger &&
               contact.segment == hand_collider_semantics::HandFingerSegment::Tip;
    }

    bool Hand::lockFarSelection()
    {
        if (_state != HandState::SelectedFar || !_currentSelection.isValid() || !_currentSelection.isFarSelection) {
            return false;
        }

        const auto transition = applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::LockFarSelection });
        if (!transition.accepted) {
            return false;
        }
        _selectionHoldFrames = 0;
        ROCK_LOG_DEBUG(Hand, "{} hand locked far selection formID={:08X} dist={:.1f}", handName(), _currentSelection.refr ? _currentSelection.refr->GetFormID() : 0,
            _currentSelection.distance);
        return true;
    }

    void Hand::clearSelectionState(bool rememberDeselect)
    {
        stopSelectionHighlight();
        clearSelectedCloseFingerPose();
        if (rememberDeselect) {
            _lastDeselectedRef = _currentSelection.refr;
            _deselectCooldown = 10;
        }
        _currentSelection.clear();
        _cachedFarCandidate.clear();
        clearGrabAcquisitionCache(rememberDeselect ? "selection-cleared-remembered" : "selection-cleared");
        clearPullRuntimeState(true, rememberDeselect ? "selection-cleared-remembered" : "selection-cleared");
        clearPullCatchIntent(rememberDeselect ? "selectionClearedRemembered" : "selectionCleared");
        clearActorEquipmentDropHandoff(rememberDeselect ? "selectionClearedRemembered" : "selectionCleared");
        _lastSelectedCloseOrigin = {};
        _hasLastSelectedCloseOrigin = false;
        _selectedCloseHandSpeedMetersPerSecond = 0.0f;
        const auto event =
            (_state == HandState::SelectedClose || _state == HandState::SelectedFar) ? HandInteractionEvent::SelectionLost :
            (_state == HandState::Idle) ? HandInteractionEvent::Initialize :
                                          HandInteractionEvent::ObjectInvalidated;
        applyTransition(HandTransitionRequest{ .event = event });
        _selectionHoldFrames = 0;
    }

    void Hand::preloadSelectionBeam()
    {
        (void)_selectionBeam.preload(handName());
    }

    void Hand::updateSelectionBeam(RE::hknpWorld* hknpWorld, const RE::NiPoint3& selectionOrigin)
    {
        const bool stateCanShowBeam = _state == HandState::SelectedFar || _state == HandState::SelectionLocked;
        if (!g_rockConfig.rockSelectionBeamEnabled || !stateCanShowBeam || !_currentSelection.isValid() || !_currentSelection.isFarSelection) {
            _selectionBeam.hide();
            return;
        }

        RE::NiPoint3 targetWorld{};
        if (!resolveFarSelectionHmdConeAnchor(hknpWorld, _currentSelection, targetWorld)) {
            _selectionBeam.hide();
            return;
        }

        (void)_selectionBeam.update(selection_beam_policy::Frame{
                                       .active = true,
                                       .startWorld = selectionOrigin,
                                       .endWorld = targetWorld,
                                       .config =
                                           selection_beam_policy::Config{
                                               .enabled = g_rockConfig.rockSelectionBeamEnabled,
                                               .segmentSizeGameUnits = g_rockConfig.rockSelectionBeamSegmentSizeGameUnits,
                                               .curveLiftGameUnits = g_rockConfig.rockSelectionBeamCurveLiftGameUnits,
                                               .alpha = g_rockConfig.rockSelectionBeamAlpha,
                                           },
                                   },
            handName());
    }

    void Hand::stopSelectionBeam()
    {
        _selectionBeam.hide();
    }

    void Hand::updateSelection(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& selectionOrigin, const RE::NiPoint3& closeSelectionDirection,
        const RE::NiPoint3& farSelectionDirection, const RE::NiPoint3& pinchOrigin, const RE::NiPoint3& pinchDirection, bool hasPinchOrigin,
        const FarSelectionHmdConeGate& farHmdConeGate, float nearRange, float farRange, float deltaTime, const OtherHandSelectionContext& otherHandContext)
    {
        if (!selection_state_policy::canUpdateSelectionFromState(_state))
            return;

        if (hasArrivedPullCatchIntent()) {
            /*
             * Once pull arrives, the original ref/body is the owner until close
             * commit succeeds, release cancels, or the retry window expires.
             * Letting normal near/far queries refresh this selection can orphan
             * the claimed pulled object or restart the far-pull path.
             */
            if (_currentSelection.isValid()) {
                _selectionHoldFrames++;
                refreshSelectionHighlight(_currentSelection);
                updateGrabAcquisitionCache(bhkWorld, hknpWorld);
            }
            return;
        }

        if (hasPendingActorEquipmentDropHandoff()) {
            /*
             * The dropped wearable is not selectable as a normal loose object
             * until its scene tree/physics bodies exist. Keep the original
             * far actor-equipment selection stable while updateGrabInput polls
             * the handoff, otherwise the hand can flicker back to the corpse
             * root and lose the far-only clothing authority.
             */
            if (_currentSelection.isValid()) {
                _selectionHoldFrames++;
                refreshSelectionHighlight(_currentSelection);
                updateGrabAcquisitionCache(bhkWorld, hknpWorld);
            }
            return;
        }

        RE::NiPoint3 resolvedPinchOrigin = pinchOrigin;
        bool resolvedHasPinchOrigin = hasPinchOrigin;
        auto resolvePinchOriginIfNeeded = [&]() {
            if (resolvedHasPinchOrigin) {
                return true;
            }

            root_flattened_finger_skeleton_runtime::Snapshot fingerSnapshot{};
            if (!root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(_isLeft, fingerSnapshot) ||
                !fingerSnapshot.valid ||
                !fingerSnapshot.fingers[0].valid ||
                !fingerSnapshot.fingers[1].valid) {
                return false;
            }

            const RE::NiPoint3 thumbPad = fingerSnapshot.fingers[0].points[2];
            const RE::NiPoint3 indexPad = fingerSnapshot.fingers[1].points[2];
            resolvedPinchOrigin = (thumbPad + indexPad) * 0.5f;
            resolvedHasPinchOrigin = true;
            return true;
        };

        auto nearCandidate = findCloseObject(bhkWorld, hknpWorld, selectionOrigin, closeSelectionDirection, nearRange, _isLeft, otherHandContext);
        if (!nearCandidate.isValid() &&
            g_rockConfig.rockGrabPinchPocketEnabled &&
            g_rockConfig.rockGrabPinchCloseSelectionEnabled &&
            resolvePinchOriginIfNeeded()) {
            nearCandidate = findCloseObject(bhkWorld,
                hknpWorld,
                resolvedPinchOrigin,
                pinchDirection,
                nearRange,
                _isLeft,
                otherHandContext,
                _isLeft ? "pinch-near-L" : "pinch-near-R");
            if (nearCandidate.isValid()) {
                nearCandidate.pinchCloseSelectionFallback = true;
            }
        }
        if (_currentSelection.pinchCloseSelectionFallback) {
            (void)resolvePinchOriginIfNeeded();
        }
        auto currentCloseSelectionOrigin = [&]() -> const RE::NiPoint3& {
            return _currentSelection.pinchCloseSelectionFallback && resolvedHasPinchOrigin ? resolvedPinchOrigin : selectionOrigin;
        };

        const bool farSelectionQueryReady = !farHmdConeGate.enabled || farHmdConeGate.hasHmdFrame;
        if (!farSelectionQueryReady) {
            _cachedFarCandidate.clear();
            _farDetectCounter = 0;
        }

        SelectedObject farCandidate;
        if (farSelectionQueryReady) {
            _farDetectCounter++;
            if (_farDetectCounter >= 3) {
                _farDetectCounter = 0;
                farCandidate = findFarObject(bhkWorld, hknpWorld, selectionOrigin, farSelectionDirection, farRange, farHmdConeGate, otherHandContext);
                _cachedFarCandidate = farCandidate;
            } else {
                farCandidate = _cachedFarCandidate;
                if (!selectedObjectPassesFarHmdCone(hknpWorld, farCandidate, farHmdConeGate)) {
                    farCandidate.clear();
                    _cachedFarCandidate.clear();
                }
            }
        }

        if (_deselectCooldown > 0) {
            _deselectCooldown--;
            if (nearCandidate.refr == _lastDeselectedRef)
                nearCandidate.clear();
            if (farCandidate.refr == _lastDeselectedRef)
                farCandidate.clear();
            if (_deselectCooldown == 0)
                _lastDeselectedRef = nullptr;
        }

        if (_currentSelection.isValid() && _currentSelection.isFarSelection) {
            float hmdConeDot = -1.0f;
            if (!selectedObjectPassesFarHmdCone(hknpWorld, _currentSelection, farHmdConeGate, &hmdConeDot)) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand cleared far selection outside HMD cone formID={:08X} hmdDot={:.3f} minDot={:.3f}",
                    handName(),
                    _currentSelection.refr ? _currentSelection.refr->GetFormID() : 0,
                    hmdConeDot,
                    farHmdConeGate.minDot);
                clearSelectionState(true);
                return;
            }
        }

        SelectedObject best = nearCandidate.isValid() ? nearCandidate : farCandidate;

        if (best.isValid() && _currentSelection.isValid() && best.refr != _currentSelection.refr && !best.isFarSelection && !_currentSelection.isFarSelection) {
            const float currentScore = _currentSelection.hasSelectionScore ? _currentSelection.selectionScore : (std::numeric_limits<float>::infinity)();
            const float candidateScore = best.hasSelectionScore ? best.selectionScore : (std::numeric_limits<float>::infinity)();
            if (selection_query_policy::shouldKeepCurrentCloseSelectionAgainstCandidate(currentScore, candidateScore, _currentSelection.distance, best.distance)) {
                _selectionHoldFrames++;
                refreshSelectionHighlight(_currentSelection);
                updateGrabAcquisitionCache(bhkWorld, hknpWorld);
                return;
            }
        }

        if (best.refr == _currentSelection.refr && best.isValid()) {
            const bool refreshedSource = selection_query_policy::shouldReplaceSelectionForSameRef(
                _currentSelection.isFarSelection, best.isFarSelection, _currentSelection.bodyId.value, best.bodyId.value);
            _currentSelection = best;
            applyTransition(
                HandTransitionRequest{ .event = best.isFarSelection ? HandInteractionEvent::SelectionFoundFar : HandInteractionEvent::SelectionFoundClose });
            if (refreshedSource) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand refreshed selection source -> {} formID={:08X} body={} dist={:.1f} signedAlong={:.1f} lateral={:.1f}",
                    handName(),
                    best.isFarSelection ? "far" : "near",
                    best.refr ? best.refr->GetFormID() : 0,
                    best.bodyId.value,
                    best.distance,
                    best.signedAlongDistance,
                    best.lateralDistance);
            }
            _selectionHoldFrames++;
            if (refreshedSource) {
                playSelectionHighlight(_currentSelection);
            } else {
                refreshSelectionHighlight(_currentSelection);
            }
        } else if (best.isValid()) {
            auto* baseObj = best.refr->GetObjectReference();
            const char* typeName = baseObj ? baseObj->GetFormTypeString() : "???";

            auto objName = baseObj ? RE::TESFullName::GetFullName(*baseObj, false) : std::string_view{};
            const std::string nameStr = objName.empty() ? std::string("(unnamed)") : std::string(objName);

            if (_currentSelection.isValid()) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand switched -> {} [{}] '{}' formID={:08X} dist={:.1f} signedAlong={:.1f} lateral={:.1f}",
                    handName(),
                    best.isFarSelection ? "far" : "near",
                    typeName,
                    nameStr,
                    best.refr->GetFormID(),
                    best.distance,
                    best.signedAlongDistance,
                    best.lateralDistance);
            } else {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand selected {} [{}] '{}' formID={:08X} dist={:.1f} signedAlong={:.1f} lateral={:.1f}",
                    handName(),
                    best.isFarSelection ? "far" : "near",
                    typeName,
                    nameStr,
                    best.refr->GetFormID(),
                    best.distance,
                    best.signedAlongDistance,
                    best.lateralDistance);
            }

            stopSelectionHighlight();

            _currentSelection = best;

            applyTransition(
                HandTransitionRequest{ .event = best.isFarSelection ? HandInteractionEvent::SelectionFoundFar : HandInteractionEvent::SelectionFoundClose });
            _selectionHoldFrames = 0;

            playSelectionHighlight(_currentSelection);
        } else if (_currentSelection.isValid()) {
            constexpr int MIN_HOLD_FRAMES = 15;

            if (_selectionHoldFrames < MIN_HOLD_FRAMES) {
                _selectionHoldFrames++;
                refreshSelectionHighlight(_currentSelection);
                return;
            }

            float hysteresisRange = _currentSelection.isFarSelection ? farRange * 2.5f : nearRange * 2.5f;

            if (_currentSelection.bodyId.value != 0x7FFF'FFFF && hknpWorld) {
                RE::NiTransform bodyWorld{};
                const bool hasBodyTransform = tryGetBodyWorldTransform(hknpWorld, _currentSelection.bodyId, bodyWorld);

                RE::NiPoint3 ownerNodePosition{};
                bool hasOwnerNode = false;
                if (auto* ownerNode = getOwnerNodeFromBody(hknpWorld, _currentSelection.bodyId)) {
                    ownerNodePosition = ownerNode->world.translate;
                    hasOwnerNode = true;
                }

                RE::NiPoint3 motionCenterOfMass{};
                bool hasMotionCenterOfMass = false;
                if (auto* motion = havok_runtime::getBodyMotion(hknpWorld, _currentSelection.bodyId)) {
                    motionCenterOfMass = hkVectorToNiPoint(motion->position);
                    hasMotionCenterOfMass = true;
                }

                const auto anchor = body_frame::chooseSelectionDistanceAnchor(false,
                    _currentSelection.hitPointWorld,
                    hasBodyTransform,
                    bodyWorld.translate,
                    hasOwnerNode,
                    ownerNodePosition,
                    hasMotionCenterOfMass,
                    motionCenterOfMass,
                    currentCloseSelectionOrigin());
                if (anchor.source != body_frame::BodyFrameSource::Fallback) {
                    _currentSelection.distance = body_frame::distance(anchor.position, currentCloseSelectionOrigin());
                } else {
                    stopSelectionHighlight();
                    _currentSelection.clear();
                    clearGrabAcquisitionCache("selection-anchor-lost");
                    applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::SelectionLost });
                    _selectionHoldFrames = 0;
                    clearSelectedCloseFingerPose();
                    return;
                }
            }

            bool refInvalid = !_currentSelection.refr || _currentSelection.refr->IsDeleted() || _currentSelection.refr->IsDisabled();

            const bool keepAfterMiss = selection_query_policy::shouldKeepSelectionAfterMiss(
                _currentSelection.isFarSelection,
                _selectionHoldFrames,
                MIN_HOLD_FRAMES,
                _currentSelection.distance,
                hysteresisRange);

            if (refInvalid || !keepAfterMiss) {
                ROCK_LOG_DEBUG(Hand, "{} hand cleared (formID={:08X}, dist={:.1f}, held={}f)", handName(), _currentSelection.refr ? _currentSelection.refr->GetFormID() : 0,
                    _currentSelection.distance, _selectionHoldFrames);
                clearSelectionState(true);
            } else {
                refreshSelectionHighlight(_currentSelection);
            }
        } else {
            if (_state != HandState::Idle) {
                applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::SelectionLost });
            }
        }

        if (_state == HandState::SelectedClose && _currentSelection.isValid()) {
            const RE::NiPoint3& selectedCloseOrigin = currentCloseSelectionOrigin();
            if (_hasLastSelectedCloseOrigin) {
                const float dx = selectedCloseOrigin.x - _lastSelectedCloseOrigin.x;
                const float dy = selectedCloseOrigin.y - _lastSelectedCloseOrigin.y;
                const float dz = selectedCloseOrigin.z - _lastSelectedCloseOrigin.z;
                const float distanceGameUnits = std::sqrt(dx * dx + dy * dy + dz * dz);
                _selectedCloseHandSpeedMetersPerSecond =
                    selected_close_finger_policy::estimateHandSpeedMetersPerSecond(distanceGameUnits, deltaTime, havokToGameScale());
            } else {
                _selectedCloseHandSpeedMetersPerSecond = 0.0f;
            }
            _lastSelectedCloseOrigin = selectedCloseOrigin;
            _hasLastSelectedCloseOrigin = true;
        } else {
            _lastSelectedCloseOrigin = {};
            _hasLastSelectedCloseOrigin = false;
            _selectedCloseHandSpeedMetersPerSecond = 0.0f;
        }

        updateSelectedCloseFingerPose();
        updateGrabAcquisitionCache(bhkWorld, hknpWorld);
    }

    bool Hand::acquirePeerHeldCloseSelection(RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        const SavedObjectState& peerSavedObjectState,
        const std::vector<std::uint32_t>& peerHeldBodyIds,
        const RE::NiPoint3& selectionOrigin,
        const RE::NiPoint3& palmNormal,
        float nearRange,
        const char** outRefusalReason)
    {
        /*
         * ROCK lets the second hand close-grab the dynamic body already held by
         * the peer hand. Normal swept selection can miss after ROCK prepares
         * the held body's dynamic collision state, so a grip press gets this narrow
         * peer-held fallback: it only considers the peer's current held ref and
         * only promotes it when one of that ref's active held bodies is inside
         * close reach. Far pulls and unrelated selections remain exclusive.
         */
        auto refuse = [&](const char* reason) {
            if (outRefusalReason) {
                *outRefusalReason = reason ? reason : "unknown";
            }
            return false;
        };

        if (outRefusalReason) {
            *outRefusalReason = "not-evaluated";
        }

        if (!selection_state_policy::canUpdateSelectionFromState(_state)) {
            return refuse("state-not-updatable");
        }
        if (!hknpWorld) {
            return refuse("missing-hknp-world");
        }
        if (!peerSavedObjectState.isValid()) {
            return refuse("invalid-peer-held-state");
        }

        auto* peerRef = peerSavedObjectState.refr;
        if (!peerRef || peerRef->IsDeleted() || peerRef->IsDisabled()) {
            return refuse("invalid-peer-ref");
        }

        if (_currentSelection.isValid() && _currentSelection.refr != peerRef) {
            return refuse(_currentSelection.isFarSelection ? "unrelated-far-selection" : "unrelated-close-selection");
        }

        const float closeReach = (std::max)(
            (std::max)(nearRange, g_rockConfig.rockNearCastDistanceGameUnits),
            g_rockConfig.rockNearDetectionRange) +
                                 (std::max)(g_rockConfig.rockNearCastRadiusGameUnits, g_rockConfig.rockGrabTouchAcquireDistanceGameUnits);

        std::vector<std::uint32_t> candidateBodyIds;
        candidateBodyIds.reserve(peerHeldBodyIds.size() + 1);
        auto appendUniqueBody = [&](std::uint32_t bodyId) {
            if (bodyId == INVALID_BODY_ID) {
                return;
            }
            if (std::find(candidateBodyIds.begin(), candidateBodyIds.end(), bodyId) == candidateBodyIds.end()) {
                candidateBodyIds.push_back(bodyId);
            }
        };
        appendUniqueBody(peerSavedObjectState.bodyId.value);
        for (const auto bodyId : peerHeldBodyIds) {
            appendUniqueBody(bodyId);
        }
        if (candidateBodyIds.empty()) {
            return refuse("no-peer-held-bodies");
        }

        SelectedObject best{};
        float bestDistance = std::numeric_limits<float>::max();
        const char* bestSource = "none";
        const char* lastMissReason = "no-close-evidence";
        auto toNiPoint = [](const hand_semantic_contact_state::SemanticContactVector& value) {
            return RE::NiPoint3{ value.x, value.y, value.z };
        };
        for (const auto bodyId : candidateBodyIds) {
            RE::NiTransform bodyWorld{};
            if (!havok_runtime::tryGetBodyArrayWorldTransform(hknpWorld, RE::hknpBodyId{ bodyId }, bodyWorld) &&
                !tryResolveLiveBodyWorldTransform(hknpWorld, RE::hknpBodyId{ bodyId }, bodyWorld)) {
                lastMissReason = "no-body-transform";
                continue;
            }

            auto* hitNode = peerRef->Get3D();
            if (bhkWorld) {
                auto bodyHandle = RE::hknpBodyId{ bodyId };
                if (auto* collisionObject = RE::bhkNPCollisionObject::Getbhk(bhkWorld, bodyHandle)) {
                    hitNode = collisionObject->sceneObject ? collisionObject->sceneObject : hitNode;
                }
            }

            auto considerHitPoint = [&](const RE::NiPoint3& hitPointWorld, const RE::NiPoint3& hitNormalWorld, const char* source) {
                const float distance = pointDistanceGameUnits(selectionOrigin, hitPointWorld);
                if (distance > closeReach || distance >= bestDistance) {
                    lastMissReason = distance > closeReach ? "outside-close-reach" : lastMissReason;
                    return;
                }

                bestDistance = distance;
                bestSource = source;
                best.setReference(peerRef);
                best.bodyId = RE::hknpBodyId{ bodyId };
                best.hitPointWorld = hitPointWorld;
                best.hitNormalWorld = normalizeOrFallback(hitNormalWorld, normalizeOrFallback(selectionOrigin - hitPointWorld, palmNormal));
                best.distance = distance;
                best.signedAlongDistance = distance;
                best.lateralDistance = 0.0f;
                best.hitFraction = 0.0f;
                best.targetKind = peerSavedObjectState.targetKind;
                best.isFarSelection = false;
                best.hasHitPoint = true;
                best.hasHitNormal = true;
                best.visualNode = peerRef->Get3D();
                best.hitNode = hitNode ? hitNode : best.visualNode;
            };

            const auto semanticContacts = collectFreshSemanticContactsForBody(
                bodyId,
                static_cast<std::uint32_t>((std::max)(0, g_rockConfig.rockGrabOppositionContactMaxAgeFrames)));
            for (std::size_t i = 0; i < semanticContacts.count && i < semanticContacts.records.size(); ++i) {
                const auto& contact = semanticContacts.records[i];
                const auto semanticDecision = hand_semantic_contact_state::evaluateSemanticPivotCandidate(
                    true,
                    contact,
                    bodyId,
                    static_cast<std::uint32_t>((std::max)(0, g_rockConfig.rockGrabOppositionContactMaxAgeFrames)));
                if (!semanticDecision.accept) {
                    lastMissReason = semanticDecision.reason;
                    continue;
                }

                if (hand_semantic_contact_state::hasUsableContactPoint(contact)) {
                    const RE::NiPoint3 contactPointWorld = toNiPoint(contact.contactPointGame);
                    const RE::NiPoint3 contactNormalWorld =
                        hand_semantic_contact_state::hasUsableContactNormal(contact) ?
                            toNiPoint(contact.contactNormalGame) :
                            normalizeOrFallback(selectionOrigin - contactPointWorld, palmNormal);
                    considerHitPoint(contactPointWorld, contactNormalWorld, "semanticContactPoint");
                    continue;
                }

                RE::NiTransform handContactWorld{};
                if (tryResolveLiveBodyWorldTransform(hknpWorld, RE::hknpBodyId{ contact.handBodyId }, handContactWorld)) {
                    considerHitPoint(handContactWorld.translate,
                        normalizeOrFallback(selectionOrigin - handContactWorld.translate, palmNormal),
                        "semanticHandBodyOriginFallback");
                } else {
                    lastMissReason = "semantic-hand-body-missing";
                }
            }

            considerHitPoint(bodyWorld.translate,
                normalizeOrFallback(selectionOrigin - bodyWorld.translate, palmNormal),
                "peerHeldBodyOriginFallback");
        }

        if (!best.isValid()) {
            return refuse(lastMissReason);
        }

        const auto transition = applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::SelectionFoundClose });
        if (!transition.accepted) {
            return refuse("state-transition-rejected");
        }

        stopSelectionHighlight();
        _currentSelection = best;
        _selectionHoldFrames = 0;
        clearSelectedCloseFingerPose();

        ROCK_LOG_DEBUG(Hand,
            "{} hand peer-held close selection acquired: formID={:08X} body={} dist={:.1f} reach={:.1f} source={}",
            handName(),
            peerRef->GetFormID(),
            best.bodyId.value,
            best.distance,
            closeReach,
            bestSource);
        if (outRefusalReason) {
            *outRefusalReason = bestSource;
        }
        return true;
    }

    void Hand::updateSelectedCloseFingerPose()
    {
        const bool shouldApply = selected_close_finger_policy::shouldApplyPreCurl(
            g_rockConfig.rockSelectedCloseFingerCurlEnabled,
            _state == HandState::SelectedClose,
            _currentSelection.isValid(),
            _selectedCloseHandSpeedMetersPerSecond,
            g_rockConfig.rockSelectedCloseFingerAnimMaxHandSpeed);
        if (!shouldApply) {
            clearSelectedCloseFingerPose();
            return;
        }

        const float value = std::clamp(g_rockConfig.rockSelectedCloseFingerAnimValue, 0.0f, 1.0f);
        const auto hand = handFromBool(_isLeft);
        if (frik_visual_authority::setHandPoseCustomWithPriority(
                SELECTED_CLOSE_FINGER_TAG,
                hand,
                frik_visual_authority::makeUniformHandPoseData(value, value, value, value, value),
                10)) {
            _selectedCloseFingerPoseActive = true;
        }
    }

    void Hand::clearSelectedCloseFingerPose()
    {
        if (!_selectedCloseFingerPoseActive) {
            return;
        }

        (void)frik_visual_authority::clearHandPose(SELECTED_CLOSE_FINGER_TAG, handFromBool(_isLeft));
        _selectedCloseFingerPoseActive = false;
    }

    bool Hand::createCollision(RE::hknpWorld* world, void* bhkWorld, const RE::NiTransform& rollAuthorityWorld)
    {
        if (hasCollisionBody()) {
            ROCK_LOG_WARN(Hand, "{} hand already has collision body -- skipping create", handName());
            return false;
        }

        if (!world || !bhkWorld) {
            ROCK_LOG_ERROR(Hand, "{} hand createCollision: world={} bhkWorld={}", handName(), (void*)world, bhkWorld);
            return false;
        }

        if (g_rockConfig.rockHandColliderRuntimeMode == 0) {
            ROCK_LOG_WARN(Hand, "{} bone-derived hand collision disabled by iHandColliderRuntimeMode=0", handName());
            return false;
        }

        if (!_boneColliders.create(world, bhkWorld, _isLeft, rollAuthorityWorld, _handBody)) {
            ROCK_LOG_ERROR(Hand, "{} bone-derived hand collision create failed", handName());
            return false;
        }

        ROCK_LOG_INFO(Hand,
            "{} hand collision created from live skeleton bones — palmAnchorBody={} generatedBodies={}",
            handName(),
            _handBody.getBodyId().value,
            _boneColliders.getBodyCount());

        return true;
    }

    void Hand::destroyCollision(void* bhkWorld)
    {
        if (!hasCollisionBody() && !_boneColliders.hasBodies())
            return;

        ROCK_LOG_DEBUG(Hand, "{} hand collision destroying — palmAnchorBody={}", handName(), _handBody.getBodyId().value);
        _boneColliders.destroy(bhkWorld, _handBody);
        clearGrabHandCollisionSuppressionState();
        clearHeldLooseWeaponBodyCollisionSuppressionState();
    }

    void Hand::updateCollisionTransform(
        RE::hknpWorld* world,
        const RE::NiTransform& rollAuthorityWorld,
        float deltaTime)
    {
        if (!hasCollisionBody() || !world)
            return;

        _boneColliders.update(world, _isLeft, rollAuthorityWorld, _handBody, deltaTime);
    }

    void Hand::flushPendingCollisionPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!hasCollisionBody() || !world) {
            return;
        }

        _boneColliders.flushPendingPhysicsDrive(world, timing, _handBody);
    }

}
