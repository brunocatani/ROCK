#include "physics-interaction/hand/Hand.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/hand/HandSelection.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/TransformMath.h"
#include "RockUtils.h"
#include "api/FRIKApi.h"
#include "RE/Havok/hknpMotion.h"

namespace rock
{
    namespace
    {
        constexpr const char* GRAB_HAND_POSE_TAG = "ROCK_Grab";
        constexpr const char* GRAB_EXTERNAL_HAND_TAG = "ROCK_GrabVisual";

        void clearGrabHandPose(bool isLeft)
        {
            auto* api = frik::api::FRIKApi::inst;
            if (!api || !api->clearHandPose) {
                return;
            }

            api->clearHandPose(GRAB_HAND_POSE_TAG, handFromBool(isLeft));
        }

        void clearGrabExternalHandWorldTransform(bool isLeft)
        {
            auto* api = frik::api::FRIKApi::inst;
            if (!api || !api->clearExternalHandWorldTransform) {
                return;
            }

            api->clearExternalHandWorldTransform(GRAB_EXTERNAL_HAND_TAG, handFromBool(isLeft));
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
        _isHoldingFlag.store(false, std::memory_order_release);
        _heldBodyIdsCount.store(0, std::memory_order_release);
        _heldBodyContactFrame.store(100, std::memory_order_release);
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
        _grabDeviationExceededSeconds = 0.0f;
        _grabVisualHandTransform = {};
        _hasGrabVisualHandTransform = false;
        _grabVisualDeviationExceededSeconds = 0.0f;
        _grabAuthorityProxyReleasePending.store(false, std::memory_order_release);
        _grabFingerProbeStart = {};
        _grabFingerProbeEnd = {};
        _hasGrabFingerProbeDebug = false;
        _grabFingerJointPose = {};
        _grabFingerLocalTransforms = {};
        _grabFingerLocalTransformMask = 0;
        _grabFingerPose = {};
        _hasGrabFingerJointPose = false;
        _hasGrabFingerLocalTransforms = false;
        _hasGrabFingerPose = false;
        clearSelectedCloseFingerPose();
        _lastSelectedCloseOrigin = {};
        _hasLastSelectedCloseOrigin = false;
        _selectedCloseHandSpeedMetersPerSecond = 0.0f;
        _grabFingerPoseFrameCounter = 0;
        _grabFingerPoseAccumulatedDeltaTime = 0.0f;
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
        _lastPlayerSpaceVelocityHavok = {};
        clearGrabHandCollisionSuppressionState();
    }

    void Hand::abandonHavokStateAfterWorldLoss()
    {
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
        const auto heldBodyCount = _heldBodyIds.size();
        const bool hadSavedState = _savedObjectState.isValid();
        const bool hadHandBody = hasCollisionBody();

        if (!hadConstraint && !hadProxy && !hadSuppression && heldBodyCount == 0 && !hadSavedState && !hadHandBody) {
            return;
        }

        ROCK_LOG_WARN(Hand,
            "{} hand abandoning Havok state after world loss: constraint={} proxy={} suppression={} heldBodies={} savedState={} handBody={}",
            handName(),
            hadConstraint ? "yes" : "no",
            hadProxy ? "yes" : "no",
            hadSuppression ? "yes" : "no",
            heldBodyCount,
            hadSavedState ? "yes" : "no",
            hadHandBody ? "yes" : "no");

        _activeConstraint.clear();
        abandonGrabAuthorityProxy();
        _savedObjectState.clear();
        _activeGrabLifecycle.clear();
        _heldBodyIds.clear();
        clearPullRuntimeState(false, "worldLoss");
        clearPullCatchIntent("worldLoss");
        clearActorEquipmentDropHandoff("worldLoss");
        _heldBodyIdsCount.store(0, std::memory_order_release);
        _heldBodyContactFrame.store(100, std::memory_order_release);
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
        _boneColliders.reset();
        _handBody.reset();
        _grabAuthorityProxyReleasePending.store(false, std::memory_order_release);
        clearGrabHandPose(_isLeft);
        clearGrabExternalHandWorldTransform(_isLeft);
        _grabVisualHandTransform = {};
        _hasGrabVisualHandTransform = false;
        _grabVisualDeviationExceededSeconds = 0.0f;
        clearSelectedCloseFingerPose();
        _grabFingerJointPose = {};
        _grabFingerLocalTransforms = {};
        _grabFingerLocalTransformMask = 0;
        _grabFingerPose = {};
        _hasGrabFingerJointPose = false;
        _hasGrabFingerLocalTransforms = false;
        _hasGrabFingerPose = false;
        _grabFingerPoseFrameCounter = 0;
        _grabFingerPoseAccumulatedDeltaTime = 0.0f;
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

    void Hand::clearPullRuntimeState(bool restorePreparedObject, const char* context)
    {
        if (restorePreparedObject) {
            restorePullPrepIfActive(context);
        } else {
            clearPullPrepTracking();
        }
        _pulledBodyIds.clear();
        _pulledPrimaryBodyId = INVALID_BODY_ID;
        _pullPointOffsetHavok = {};
        _pullTargetHavok = {};
        _pullElapsedSeconds = 0.0f;
        _pullDurationSeconds = 0.0f;
        _pullHasTarget = false;
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
        selection.refr = refr;
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
        replacement.refr = droppedRef;
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
        LivePalmAnchorReference palmReference{};
        if (tryResolveLivePalmAnchorReference(world, palmReference) &&
            std::isfinite(palmReference.world.translate.x) &&
            std::isfinite(palmReference.world.translate.y) &&
            std::isfinite(palmReference.world.translate.z)) {
            const RE::NiTransform proxyBaseWorld =
                hand_bone_collider_geometry_math::generatedColliderFrameToGrabAuthorityFrame(palmReference.world);
            return applyGrabAuthorityProxyLocalOffsetToFrame(proxyBaseWorld, _isLeft).translate;
        }

        return fallbackHandWorldTransform.translate;
    }

    void Hand::recordSemanticContact(const HandColliderBodyMetadata& metadata, std::uint32_t otherBodyId)
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
        _semanticContactSequence.store(contactSequence + 1, std::memory_order_release);
        _semanticContactValid.store(1, std::memory_order_release);

        const std::size_t slot = hand_semantic_contact_state::semanticContactSlotForRole(metadata.role);
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
        _semanticContactSequence.fetch_add(2, std::memory_order_acq_rel);

        for (std::size_t i = 0; i < hand_semantic_contact_state::kMaxSemanticContactRecords; ++i) {
            _semanticContactSetValid[i].store(0, std::memory_order_release);
            _semanticContactSetHandBodyId[i].store(hand_semantic_contact_state::kInvalidBodyId, std::memory_order_release);
            _semanticContactSetOtherBodyId[i].store(hand_semantic_contact_state::kInvalidBodyId, std::memory_order_release);
            _semanticContactSetFrames[i].store(0xFFFF'FFFFu, std::memory_order_release);
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
            contact.sequence = sequenceBefore;
            contact.framesSinceContact = hand_semantic_contact_state::semanticFramesSinceContact(
                _semanticContactFrameCounter.load(std::memory_order_acquire),
                contactFrame);
            const auto sequenceAfter = _semanticContactSequence.load(std::memory_order_acquire);
            if (!hand_semantic_contact_state::semanticContactSequenceSnapshotStable(sequenceBefore, sequenceAfter)) {
                continue;
            }
            if (contact.handBodyId == hand_semantic_contact_state::kInvalidBodyId || contact.otherBodyId == hand_semantic_contact_state::kInvalidBodyId) {
                return false;
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
            contact.sequence = sequenceBefore;
            contact.framesSinceContact = hand_semantic_contact_state::semanticFramesSinceContact(
                _semanticContactFrameCounter.load(std::memory_order_acquire),
                contactFrame);
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
            outContact = contact;
            return true;
        }
        return false;
    }

    hand_semantic_contact_state::SemanticContactCollection Hand::collectFreshSemanticContactsForBody(std::uint32_t targetBodyId, std::uint32_t maxFramesSinceContact) const
    {
        hand_semantic_contact_state::SemanticContactCollection contacts{};
        if (targetBodyId == hand_semantic_contact_state::kInvalidBodyId) {
            return contacts;
        }

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
                record.framesSinceContact = hand_semantic_contact_state::semanticFramesSinceContact(
                    _semanticContactFrameCounter.load(std::memory_order_acquire),
                    contactFrame);
                record.sequence = sequenceBefore;
                const auto sequenceAfter = _semanticContactSetSequence[i].load(std::memory_order_acquire);
                if (!hand_semantic_contact_state::semanticContactSequenceSnapshotStable(sequenceBefore, sequenceAfter)) {
                    continue;
                }

                if (record.handBodyId == hand_semantic_contact_state::kInvalidBodyId || record.otherBodyId != targetBodyId) {
                    break;
                }
                if (record.framesSinceContact <= maxFramesSinceContact) {
                    contacts.add(record);
                }
                break;
            }
        }

        return contacts;
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

    void Hand::updateSelection(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& selectionOrigin, const RE::NiPoint3& palmNormal,
        const RE::NiPoint3& pointingDirection, const FarSelectionHmdConeGate& farHmdConeGate, float nearRange, float farRange, float deltaTime,
        const OtherHandSelectionContext& otherHandContext)
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
            }
            return;
        }

        auto nearCandidate = findCloseObject(bhkWorld, hknpWorld, selectionOrigin, palmNormal, nearRange, _isLeft, otherHandContext);

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
                farCandidate = findFarObject(bhkWorld, hknpWorld, selectionOrigin, pointingDirection, farRange, farHmdConeGate, otherHandContext);
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
            float stickyThreshold = _currentSelection.distance * 0.7f;
            if (best.distance > stickyThreshold) {
                _currentSelection.distance = _currentSelection.distance;
                _selectionHoldFrames++;
                refreshSelectionHighlight(_currentSelection);
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
                    selectionOrigin);
                if (anchor.source != body_frame::BodyFrameSource::Fallback) {
                    _currentSelection.distance = body_frame::distance(anchor.position, selectionOrigin);
                } else {
                    stopSelectionHighlight();
                    _currentSelection.clear();
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
            if (_hasLastSelectedCloseOrigin) {
                const float dx = selectionOrigin.x - _lastSelectedCloseOrigin.x;
                const float dy = selectionOrigin.y - _lastSelectedCloseOrigin.y;
                const float dz = selectionOrigin.z - _lastSelectedCloseOrigin.z;
                const float distanceGameUnits = std::sqrt(dx * dx + dy * dy + dz * dz);
                _selectedCloseHandSpeedMetersPerSecond =
                    selected_close_finger_policy::estimateHandSpeedMetersPerSecond(distanceGameUnits, deltaTime, havokToGameScale());
            } else {
                _selectedCloseHandSpeedMetersPerSecond = 0.0f;
            }
            _lastSelectedCloseOrigin = selectionOrigin;
            _hasLastSelectedCloseOrigin = true;
        } else {
            _lastSelectedCloseOrigin = {};
            _hasLastSelectedCloseOrigin = false;
            _selectedCloseHandSpeedMetersPerSecond = 0.0f;
        }

        updateSelectedCloseFingerPose();
    }

    bool Hand::acquirePeerHeldCloseSelection(RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        const SavedObjectState& peerSavedObjectState,
        const std::vector<std::uint32_t>& peerHeldBodyIds,
        const RE::NiPoint3& selectionOrigin,
        const RE::NiPoint3& palmNormal,
        float nearRange)
    {
        /*
         * ROCK lets the second hand close-grab the dynamic body already held by
         * the peer hand. Normal swept selection can miss after ROCK prepares
         * the held body's dynamic collision state, so a grip press gets this narrow
         * peer-held fallback: it only considers the peer's current held ref and
         * only promotes it when one of that ref's active held bodies is inside
         * close reach. Far pulls and unrelated selections remain exclusive.
         */
        if (!selection_state_policy::canUpdateSelectionFromState(_state) || !hknpWorld || !peerSavedObjectState.isValid()) {
            return false;
        }

        auto* peerRef = peerSavedObjectState.refr;
        if (!peerRef || peerRef->IsDeleted() || peerRef->IsDisabled()) {
            return false;
        }

        if (_currentSelection.isValid() && !_currentSelection.isFarSelection && _currentSelection.refr != peerRef) {
            return false;
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

        SelectedObject best{};
        float bestDistance = std::numeric_limits<float>::max();
        const char* bestSource = "none";
        for (const auto bodyId : candidateBodyIds) {
            RE::NiTransform bodyWorld{};
            if (!havok_runtime::tryGetBodyArrayWorldTransform(hknpWorld, RE::hknpBodyId{ bodyId }, bodyWorld) &&
                !tryResolveLiveBodyWorldTransform(hknpWorld, RE::hknpBodyId{ bodyId }, bodyWorld)) {
                continue;
            }

            auto* hitNode = peerRef->Get3D();
            if (bhkWorld) {
                auto bodyHandle = RE::hknpBodyId{ bodyId };
                if (auto* collisionObject = RE::bhkNPCollisionObject::Getbhk(bhkWorld, bodyHandle)) {
                    hitNode = collisionObject->sceneObject ? collisionObject->sceneObject : hitNode;
                }
            }

            auto considerHitPoint = [&](const RE::NiPoint3& hitPointWorld, const char* source) {
                const float distance = pointDistanceGameUnits(selectionOrigin, hitPointWorld);
                if (distance > closeReach || distance >= bestDistance) {
                    return;
                }

                bestDistance = distance;
                bestSource = source;
                best.refr = peerRef;
                best.bodyId = RE::hknpBodyId{ bodyId };
                best.hitPointWorld = hitPointWorld;
                best.hitNormalWorld = normalizeOrFallback(selectionOrigin - hitPointWorld, palmNormal);
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
                    continue;
                }

                RE::NiTransform handContactWorld{};
                if (tryResolveLiveBodyWorldTransform(hknpWorld, RE::hknpBodyId{ contact.handBodyId }, handContactWorld)) {
                    considerHitPoint(handContactWorld.translate, semanticDecision.reason);
                }
            }

            considerHitPoint(bodyWorld.translate, "bodyOriginFallback");
        }

        if (!best.isValid()) {
            return false;
        }

        const auto transition = applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::SelectionFoundClose });
        if (!transition.accepted) {
            return false;
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
        return true;
    }

    void Hand::updateSelectedCloseFingerPose()
    {
        auto* api = frik::api::FRIKApi::inst;
        const bool shouldApply = selected_close_finger_policy::shouldApplyPreCurl(
            g_rockConfig.rockSelectedCloseFingerCurlEnabled,
            _state == HandState::SelectedClose,
            _currentSelection.isValid(),
            _selectedCloseHandSpeedMetersPerSecond,
            g_rockConfig.rockSelectedCloseFingerAnimMaxHandSpeed);
        if (!api || !shouldApply) {
            clearSelectedCloseFingerPose();
            return;
        }

        const float value = std::clamp(g_rockConfig.rockSelectedCloseFingerAnimValue, 0.0f, 1.0f);
        const auto hand = handFromBool(_isLeft);
        if (api->setHandPoseCustomFingerPositionsWithPriority) {
            api->setHandPoseCustomFingerPositionsWithPriority(SELECTED_CLOSE_FINGER_TAG, hand, value, value, value, value, value, 10);
            _selectedCloseFingerPoseActive = true;
        }
    }

    void Hand::clearSelectedCloseFingerPose()
    {
        if (!_selectedCloseFingerPoseActive) {
            return;
        }

        if (auto* api = frik::api::FRIKApi::inst) {
            if (api->clearHandPose) {
                api->clearHandPose(SELECTED_CLOSE_FINGER_TAG, handFromBool(_isLeft));
            }
        }
        _selectedCloseFingerPoseActive = false;
    }

    bool Hand::createCollision(RE::hknpWorld* world, void* bhkWorld)
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

        if (!_boneColliders.create(world, bhkWorld, _isLeft, _handBody)) {
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
    }

    void Hand::updateCollisionTransform(RE::hknpWorld* world, float deltaTime)
    {
        if (!hasCollisionBody() || !world)
            return;

        _boneColliders.update(world, _isLeft, _handBody, deltaTime);
    }

    void Hand::flushPendingCollisionPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!hasCollisionBody() || !world) {
            return;
        }

        _boneColliders.flushPendingPhysicsDrive(world, timing, _handBody);
    }

}
