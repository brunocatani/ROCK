#include "Hand.h"

#include <cmath>
#include <string>

#include "GrabVisualAuthorityPolicy.h"
#include "HandLifecyclePolicy.h"
#include "HandVisualAuthorityMath.h"
#include "HavokOffsets.h"
#include "PalmTransform.h"
#include "PhysicsBodyFrame.h"
#include "SelectionQueryPolicy.h"
#include "SelectionStatePolicy.h"
#include "SelectedCloseFingerPolicy.h"
#include "TransformMath.h"
#include "RockUtils.h"
#include "api/FRIKApi.h"

namespace frik::rock
{
    namespace
    {
        constexpr const char* GRAB_EXTERNAL_HAND_TAG = "ROCK_GrabVisual";

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
            case HandState::HeldInit:
                return "HeldInit";
            case HandState::HeldBody:
                return "HeldBody";
            case HandState::Pulled:
                return "Pulled";
            case HandState::GrabFromOtherHand:
                return "GrabFromOtherHand";
            case HandState::SelectedTwoHand:
                return "SelectedTwoHand";
            case HandState::HeldTwoHanded:
                return "HeldTwoHanded";
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

    RE::hknpMaterialId registerHandMaterial(RE::hknpWorld* world)
    {
        static void* cachedMaterialLibrary = nullptr;
        static RE::hknpMaterialId cachedId{ 0xFFFF };

        if (!world)
            return { 0 };

        auto* matLibPtr = reinterpret_cast<void**>(reinterpret_cast<char*>(world) + 0x5C8);
        auto* matLib = *matLibPtr;
        if (!matLib) {
            ROCK_LOG_WARN(Hand, "Material library is null -- using default material 0");
            return { 0 };
        }
        if (cachedMaterialLibrary == matLib && cachedId.value != 0xFFFF) {
            return cachedId;
        }

        alignas(16) char matBuffer[0x50];
        memset(matBuffer, 0, 0x50);

        typedef void (*matCtor_t)(void*);
        static REL::Relocation<matCtor_t> matCtor{ REL::Offset(offsets::kFunc_MaterialCtor) };
        matCtor(matBuffer);

        *reinterpret_cast<std::uint8_t*>(matBuffer + 0x11) = 200;

        *reinterpret_cast<std::uint16_t*>(matBuffer + 0x12) = 0x3C00;

        *reinterpret_cast<std::uint16_t*>(matBuffer + 0x28) = 0x0000;

        *reinterpret_cast<std::uint8_t*>(matBuffer + 0x18) = 2;

        *reinterpret_cast<std::uint8_t*>(matBuffer + 0x10) = 0;

        typedef void (*addMat_t)(void*, std::uint16_t*, void*);
        static REL::Relocation<addMat_t> addMaterial{ REL::Offset(offsets::kFunc_MaterialLibrary_AddMaterial) };

        std::uint16_t newId = 0xFFFF;
        addMaterial(matLib, &newId, matBuffer);

        if (newId != 0xFFFF) {
            cachedId.value = newId;
            cachedMaterialLibrary = matLib;
            ROCK_LOG_DEBUG(Hand, "Registered ROCK_Hand material ID={} (dynFriction=200, staticFriction=1.0, restitution=0.0)", newId);
        } else {
            ROCK_LOG_WARN(Hand, "Failed to register ROCK_Hand material -- using default 0");
            return { 0 };
        }

        return cachedId;
    }

    void Hand::reset()
    {
        const bool suppressionActive = hand_collision_suppression_math::hasActive(_grabHandCollisionSuppression);
        const bool cleanupRequired = hand_lifecycle_policy::requiresHavokCleanupBeforeReset(
            _activeConstraint.isValid(), suppressionActive, _heldBodyIds.size(), _savedObjectState.isValid(), hasCollisionBody());
        if (cleanupRequired) {
            ROCK_LOG_ERROR(Hand,
                "{} hand reset blocked: native Havok state still needs cleanup (constraint={} suppression={} heldBodies={} savedState={} handBody={})",
                handName(),
                _activeConstraint.isValid() ? "yes" : "no",
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
        _activeConstraint.clear();
        _savedObjectState.clear();
        _activeGrabLifecycle.clear();
        _grabStartTime = 0.0f;
        _heldLogCounter = 0;
        _notifCounter = 0;
        _heldBodyIds.clear();
        clearPullRuntimeState();
        if (_nearbyGrabDamping.active || !_nearbyGrabDamping.motions.empty()) {
            ROCK_LOG_WARN(Hand, "{} hand reset cleared nearby velocity-damping state without a world; no native damping fields were modified", handName());
        }
        _nearbyGrabDamping.clear();
        _grabFrame.clear();
        _adjustedHandTransform = RE::NiTransform();
        _hasAdjustedHandTransform = false;
        clearGrabExternalHandWorldTransform(_isLeft);
        _grabVisualLerpElapsed = 0.0f;
        _grabVisualLerpDuration = g_rockConfig.rockGrabLerpMaxTime;
        _grabDeviationExceededSeconds = 0.0f;
        _grabFingerProbeStart = {};
        _grabFingerProbeEnd = {};
        _hasGrabFingerProbeDebug = false;
        _grabFingerJointPose = {};
        _hasGrabFingerJointPose = false;
        clearSelectedCloseFingerPose();
        _lastSelectedCloseOrigin = {};
        _hasLastSelectedCloseOrigin = false;
        _selectedCloseHandSpeedMetersPerSecond = 0.0f;
        _grabFingerPoseFrameCounter = 0;
        _heldLocalLinearVelocityHistory = {};
        _heldLocalLinearVelocityHistoryCount = 0;
        _heldLocalLinearVelocityHistoryNext = 0;
        _lastPlayerSpaceVelocityHavok = {};
        clearGrabHandCollisionSuppressionState();
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
            ROCK_LOG_DEBUG(Hand,
                "{} hand state {} -> {} via {}",
                handName(),
                handStateName(oldState),
                handStateName(result.next),
                handEventName(evaluatedRequest.event));
        }

        return result;
    }

    void Hand::clearPullRuntimeState()
    {
        _pulledBodyIds.clear();
        _pulledPrimaryBodyId = INVALID_BODY_ID;
        _pullPointOffsetHavok = {};
        _pullTargetHavok = {};
        _pullElapsedSeconds = 0.0f;
        _pullDurationSeconds = 0.0f;
        _pullHasTarget = false;
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
        if (collObj) {
            constexpr std::uintptr_t kMinValidPointer = 0x10000;
            auto* fieldAt20 = *reinterpret_cast<void**>(reinterpret_cast<char*>(collObj) + offsets::kCollisionObject_PhysSystemPtr);
            if (fieldAt20 && reinterpret_cast<std::uintptr_t>(fieldAt20) > kMinValidPointer) {
                auto* physSystem = reinterpret_cast<RE::bhkPhysicsSystem*>(fieldAt20);
                auto* inst = physSystem->instance;
                if (inst && reinterpret_cast<std::uintptr_t>(inst) > kMinValidPointer) {
                    for (std::int32_t i = 0; i < inst->bodyCount && i < 64; i++) {
                        std::uint32_t bid = inst->bodyIds[i];
                        if (bid != 0x7FFF'FFFF) {
                            _heldBodyIds.push_back(bid);
                        }
                    }
                }
            }
        }

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

    bool Hand::computeAdjustedHandTransformTarget(RE::NiTransform& outTransform) const
    {
        return computeAdjustedHandTransformTarget(nullptr, outTransform);
    }

    bool Hand::computeAdjustedHandTransformTarget(RE::hknpWorld* world, RE::NiTransform& outTransform) const
    {
        (void)world;

        if (!isHolding())
            return false;

        RE::NiAVObject* node = nullptr;
        if (_savedObjectState.refr && !_savedObjectState.refr->IsDeleted() && !_savedObjectState.refr->IsDisabled()) {
            node = _grabFrame.heldNode ? _grabFrame.heldNode : _savedObjectState.refr->Get3D();
        }
        if (!node)
            return false;
        if (!_grabFrame.visualAuthorityContactValid)
            return false;

        /*
         * Visual hand authority follows HIGGS' object-owned visual-node solve:
         * adjustedHand = heldVisualNodeWorld * inverse(rawObjectHandSpace).
         * The hand-body/constraint frame remains the physics source of truth, but
         * feeding that body-derived frame back into FRIK visual IK can reverse the
         * rendered wrist when the body frame and the visual node disagree.
         */
        outTransform = hand_visual_authority_math::buildAppliedVisualAuthorityHandWorld(
            node->world,
            _grabFrame.rawHandSpace,
            transform_math::makeIdentityTransform<RE::NiTransform>(),
            _grabFrame.constraintHandSpace,
            _grabFrame.handBodyToRawHandAtGrab);
        return true;
    }

    bool Hand::getAdjustedHandTransform(RE::NiTransform& outTransform) const
    {
        if (!isHolding())
            return false;

        if (!grab_visual_authority_policy::shouldApplyObjectReverseAlignedExternalHandTransform(
                g_rockConfig.rockGrabObjectVisualHandAuthorityEnabled, true, _grabFrame.visualAuthorityContactValid)) {
            return false;
        }

        if (_hasAdjustedHandTransform) {
            outTransform = _adjustedHandTransform;
            return true;
        }

        return computeAdjustedHandTransformTarget(outTransform);
    }

    RE::NiPoint3 Hand::computeGrabPivotAWorld(RE::hknpWorld* world, const RE::NiTransform& fallbackHandWorldTransform) const
    {
        /*
         * Bone-derived collision makes the palm anchor body the authoritative
         * hand-side constraint actor. Pivot A must therefore come from the live
         * anchor body when it exists; the INI-authored pivot remains available
         * only as diagnostic/fallback data when bone collision is disabled or
         * the anchor is unavailable during transitions.
         */
        if (g_rockConfig.rockGrabUseBoneDerivedPalmPivot && g_rockConfig.rockHandColliderRuntimeMode != 0 && world && _handBody.isValid()) {
            RE::NiTransform bodyWorld{};
            if (tryResolveLiveBodyWorldTransform(world, _handBody.getBodyId(), bodyWorld)) {
                return bodyWorld.translate;
            }
        }

        return computeGrabPivotAPositionFromHandBasis(fallbackHandWorldTransform, _isLeft);
    }

    void Hand::recordSemanticContact(const HandColliderBodyMetadata& metadata, std::uint32_t otherBodyId)
    {
        if (!metadata.valid || metadata.bodyId == hand_semantic_contact_state::kInvalidBodyId || otherBodyId == hand_semantic_contact_state::kInvalidBodyId) {
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
        clearPullRuntimeState();
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
        const RE::NiPoint3& pointingDirection, float nearRange, float farRange, float deltaTime, RE::TESObjectREFR* otherHandRef)
    {
        if (!selection_state_policy::canUpdateSelectionFromState(_state))
            return;

        auto nearCandidate = findCloseObject(bhkWorld, hknpWorld, selectionOrigin, palmNormal, nearRange, _isLeft, otherHandRef);

        SelectedObject farCandidate;
        _farDetectCounter++;
        if (_farDetectCounter >= 3) {
            _farDetectCounter = 0;
            farCandidate = findFarObject(bhkWorld, hknpWorld, selectionOrigin, pointingDirection, farRange, otherHandRef);
            _cachedFarCandidate = farCandidate;
        } else {
            farCandidate = _cachedFarCandidate;
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
                    "{} hand refreshed selection source -> {} formID={:08X} body={} dist={:.1f}",
                    handName(),
                    best.isFarSelection ? "far" : "near",
                    best.refr ? best.refr->GetFormID() : 0,
                    best.bodyId.value,
                    best.distance);
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
                ROCK_LOG_DEBUG(Hand, "{} hand switched -> {} [{}] '{}' formID={:08X} dist={:.1f}", handName(), best.isFarSelection ? "far" : "near", typeName, nameStr,
                    best.refr->GetFormID(), best.distance);
            } else {
                ROCK_LOG_DEBUG(Hand, "{} hand selected {} [{}] '{}' formID={:08X} dist={:.1f}", handName(), best.isFarSelection ? "far" : "near", typeName, nameStr,
                    best.refr->GetFormID(), best.distance);
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
