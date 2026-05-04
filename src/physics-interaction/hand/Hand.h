#pragma once

#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabTelemetry.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/hand/HandBoneColliderSet.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/hand/HandInteractionStateMachine.h"
#include "physics-interaction/grab/NearbyGrabDamping.h"
#include "physics-interaction/object/ObjectDetection.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "RockConfig.h"
#include "physics-interaction/hand/HandSelection.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Bethesda/bhkPhysicsSystem.h"
#include "RE/Havok/hkReferencedObject.h"
#include "RE/Havok/hknpBodyCinfo.h"
#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpWorld.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <vector>

namespace rock
{

    constexpr std::uint32_t ROCK_HAND_LAYER = 43;

    constexpr std::uint32_t INVALID_BODY_ID = 0x7FFF'FFFF;

    RE::hknpMaterialId registerHandMaterial(RE::hknpWorld* world);

    struct GrabPivotDebugSnapshot
    {
        RE::NiPoint3 handPivotWorld{};
        RE::NiPoint3 objectPivotWorld{};
        RE::NiPoint3 handBodyWorld{};
        RE::NiPoint3 objectBodyWorld{};
        float pivotErrorGameUnits = 0.0f;
    };

    struct GrabSurfaceFrameDebugSnapshot
    {
        RE::NiPoint3 contactPointWorld{};
        RE::NiPoint3 normalEndWorld{};
        RE::NiPoint3 tangentEndWorld{};
        RE::NiPoint3 bitangentEndWorld{};
        bool hasTangent = false;
        bool hasBitangent = false;
    };

    struct GrabContactPatchDebugSnapshot
    {
        std::array<RE::NiPoint3, kMaxGrabContactPatchSamples> samplePointsWorld{};
        std::uint32_t sampleCount = 0;
    };

    struct HeldObjectPlayerSpaceFrame
    {
        RE::NiPoint3 deltaGameUnits{};
        RE::NiPoint3 velocityHavok{};
        RE::NiTransform previousPlayerSpaceWorld{};
        RE::NiTransform currentPlayerSpaceWorld{};
        float rotationDeltaDegrees = 0.0f;
        const char* source = "none";
        bool enabled = false;
        bool warp = false;
        bool warpByDistance = false;
        bool warpByRotation = false;
        bool hasWarpTransforms = false;
    };

    enum class GrabReleaseCollisionRestoreMode : std::uint8_t
    {
        Delayed,
        Immediate
    };

    class Hand
    {
    public:
        explicit Hand(bool isLeft) : _isLeft(isLeft) {}

        bool isLeft() const { return _isLeft; }
        HandState getState() const { return _state; }
        const char* handName() const { return _isLeft ? "Left" : "Right"; }

        void reset();

        void collectHeldBodyIds(RE::TESObjectREFR* refr);

    private:
        void collectBodyIdsRecursive(RE::NiAVObject* node, int maxDepth = 10);
        void suppressHandCollisionForGrab(RE::hknpWorld* world);
        void restoreHandCollisionAfterGrab(RE::hknpWorld* world);
        void clearGrabHandCollisionSuppressionState();
        void clearPullRuntimeState();
        void updateSelectedCloseFingerPose();
        void clearSelectedCloseFingerPose();
        bool computeAdjustedHandTransformTarget(RE::NiTransform& outTransform) const;
        bool computeAdjustedHandTransformTarget(RE::hknpWorld* world, RE::NiTransform& outTransform) const;

    public:
        const std::vector<std::uint32_t>& getHeldBodyIds() const { return _heldBodyIds; }

        bool isHeldBodyId(std::uint32_t bodyId) const
        {
            int count = _heldBodyIdsCount.load(std::memory_order_acquire);
            for (int i = 0; i < count; i++) {
                if (_heldBodyIdsSnapshot[i] == bodyId)
                    return true;
            }
            return false;
        }

        bool isHoldingAtomic() const { return _isHoldingFlag.load(std::memory_order_acquire); }

        const SelectedObject& getSelection() const { return _currentSelection; }
        bool hasSelection() const { return _currentSelection.isValid(); }

        bool isTouching() const { return _touchActiveFrames < 5; }
        RE::TESObjectREFR* getLastTouchedRef() const { return _lastTouchedRef; }
        std::uint32_t getLastTouchedFormID() const { return _lastTouchedFormID; }
        std::uint32_t getLastTouchedLayer() const { return _lastTouchedLayer; }

        void setTouchState(RE::TESObjectREFR* refr, std::uint32_t formID, std::uint32_t layer)
        {
            _lastTouchedRef = refr;
            _lastTouchedFormID = formID;
            _lastTouchedLayer = layer;
            _touchActiveFrames = 0;
        }

        bool isHolding() const { return _state == HandState::HeldInit || _state == HandState::HeldBody; }
        RE::TESObjectREFR* getHeldRef() const { return _savedObjectState.refr; }
        const ActiveConstraint& getActiveConstraint() const { return _activeConstraint; }
        const SavedObjectState& getSavedObjectState() const { return _savedObjectState; }
        bool getGrabPivotDebugSnapshot(RE::hknpWorld* world, GrabPivotDebugSnapshot& out) const;
        bool getGrabSurfaceFrameDebugSnapshot(RE::hknpWorld* world, GrabSurfaceFrameDebugSnapshot& out) const;
        bool getGrabContactPatchDebugSnapshot(RE::hknpWorld* world, GrabContactPatchDebugSnapshot& out) const;
        bool getGrabTransformTelemetrySnapshot(RE::hknpWorld* world,
            const RE::NiTransform& rawHandWorld,
            bool visualAuthorityEnabled,
            grab_transform_telemetry::RuntimeSample& out) const;

        bool getAdjustedHandTransform(RE::NiTransform& outTransform) const;
        bool getGrabFingerProbeDebug(std::array<RE::NiPoint3, 5>& outStart, std::array<RE::NiPoint3, 5>& outEnd) const
        {
            if (!_hasGrabFingerProbeDebug)
                return false;
            outStart = _grabFingerProbeStart;
            outEnd = _grabFingerProbeEnd;
            return true;
        }

        bool grabSelectedObject(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, float tau, float damping, float maxForce, float proportionalRecovery,
            float constantRecovery);

        void updateHeldObject(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, const HeldObjectPlayerSpaceFrame& playerSpaceFrame, float deltaTime,
            float forceFadeInTime, float tauMin);

        void releaseGrabbedObject(RE::hknpWorld* world, GrabReleaseCollisionRestoreMode collisionRestoreMode = GrabReleaseCollisionRestoreMode::Delayed);
        void updateDelayedGrabHandCollisionRestore(RE::hknpWorld* world, float deltaTime);

        bool lockFarSelection();
        bool startDynamicPull(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform);
        bool updateDynamicPull(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, float deltaTime);
        void clearSelectionState(bool rememberDeselect);

        void tickTouchState() { _touchActiveFrames++; }

        void updateSelection(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& selectionOrigin, const RE::NiPoint3& palmNormal,
            const RE::NiPoint3& pointingDirection, float nearRange, float farRange, float deltaTime, RE::TESObjectREFR* otherHandRef);

        RE::hknpBodyId getCollisionBodyId() const { return _handBody.getBodyId(); }
        bool hasCollisionBody() const { return _handBody.isValid(); }
        BethesdaPhysicsBody& getHandBody() { return _handBody; }
        const BethesdaPhysicsBody& getHandBody() const { return _handBody; }
        RE::NiPoint3 computeGrabPivotAWorld(RE::hknpWorld* world, const RE::NiTransform& fallbackHandWorldTransform) const;
        std::uint32_t getHandColliderBodyCount() const { return _boneColliders.getBodyCount(); }
        std::uint32_t getHandColliderBodyIdAtomic(std::size_t index) const { return _boneColliders.getBodyIdAtomic(index); }
        bool isHandColliderBodyId(std::uint32_t bodyId) const { return _boneColliders.isColliderBodyIdAtomic(bodyId); }
        bool tryGetHandColliderMetadata(std::uint32_t bodyId, HandColliderBodyMetadata& outMetadata) const { return _boneColliders.tryGetBodyMetadataAtomic(bodyId, outMetadata); }
        void recordSemanticContact(const HandColliderBodyMetadata& metadata, std::uint32_t otherBodyId);
        void tickSemanticContactState();
        bool getLastSemanticContact(hand_semantic_contact_state::SemanticContactRecord& outContact) const;
        bool getFreshSemanticContactForRole(
            hand_collider_semantics::HandColliderRole role,
            std::uint32_t maxFramesSinceContact,
            hand_semantic_contact_state::SemanticContactRecord& outContact) const;
        hand_semantic_contact_state::SemanticContactCollection collectFreshSemanticContactsForBody(std::uint32_t targetBodyId, std::uint32_t maxFramesSinceContact) const;
        bool isFingerTouching(hand_collider_semantics::HandFinger finger) const;
        bool isFingerTipTouching(hand_collider_semantics::HandFinger finger) const;
        bool tryGetHandColliderMetadataForRole(hand_collider_semantics::HandColliderRole role, HandColliderBodyMetadata& outMetadata) const;

        bool createCollision(RE::hknpWorld* world, void* bhkWorld);

        void destroyCollision(void* bhkWorld);

        void updateCollisionTransform(RE::hknpWorld* world, float deltaTime);

        void flushPendingCollisionPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);

    private:
        HandTransitionResult applyTransition(const HandTransitionRequest& request);

        bool _isLeft;
        HandState _state = HandState::Idle;
        HandState _prevState = HandState::Idle;

        bool _idleDesired = false;
        bool _grabRequested = false;
        bool _releaseRequested = false;

        BethesdaPhysicsBody _handBody;
        HandBoneColliderSet _boneColliders;

        SelectedObject _currentSelection;
        SelectedObject _cachedFarCandidate;
        int _farDetectCounter = 0;
        int _selectionHoldFrames = 0;
        int _selectionHighlightRefreshFrames = 0;
        int _deselectCooldown = 0;
        RE::TESObjectREFR* _lastDeselectedRef = nullptr;

        RE::TESObjectREFR* _lastTouchedRef = nullptr;
        std::uint32_t _lastTouchedFormID = 0;
        std::uint32_t _lastTouchedLayer = 0;
        int _touchActiveFrames = 100;

        std::atomic<int> _heldBodyContactFrame{ 100 };

        hand_collision_suppression_math::SuppressionSet<hand_collider_semantics::kHandColliderBodyCountPerHand> _grabHandCollisionSuppression{};
        hand_collision_suppression_math::DelayedRestoreState _grabHandCollisionDelayedRestore{};
        std::atomic<std::uint32_t> _semanticContactFrameCounter{ 0 };
        std::atomic<std::uint32_t> _semanticContactValid{ 0 };
        std::atomic<std::uint32_t> _semanticContactSequence{ 0 };
        std::atomic<std::uint32_t> _semanticContactRole{ static_cast<std::uint32_t>(hand_collider_semantics::HandColliderRole::PalmAnchor) };
        std::atomic<std::uint32_t> _semanticContactFinger{ static_cast<std::uint32_t>(hand_collider_semantics::HandFinger::None) };
        std::atomic<std::uint32_t> _semanticContactSegment{ static_cast<std::uint32_t>(hand_collider_semantics::HandFingerSegment::None) };
        std::atomic<std::uint32_t> _semanticContactHandBodyId{ hand_semantic_contact_state::kInvalidBodyId };
        std::atomic<std::uint32_t> _semanticContactOtherBodyId{ hand_semantic_contact_state::kInvalidBodyId };
        std::atomic<std::uint32_t> _semanticContactFrames{ 0xFFFF'FFFFu };
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetValid{};
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetRole{};
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetFinger{};
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetSegment{};
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetHandBodyId{};
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetOtherBodyId{};
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetFrames{};
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetSequence{};

    public:
        bool isHeldBodyColliding() const { return _heldBodyContactFrame.load(std::memory_order_acquire) < 5; }

        void notifyHeldBodyContact() { _heldBodyContactFrame.store(0, std::memory_order_release); }

        void tickHeldBodyContact()
        {
            int current = _heldBodyContactFrame.load(std::memory_order_acquire);
            if (current < 100) {
                _heldBodyContactFrame.compare_exchange_weak(current, current + 1, std::memory_order_release, std::memory_order_relaxed);
            }
        }

    private:
        ActiveConstraint _activeConstraint;
        SavedObjectState _savedObjectState;
        active_grab_body_lifecycle::BodyLifecycleSnapshot _activeGrabLifecycle;
        float _grabStartTime = 0.0f;
        int _heldLogCounter = 0;
        int _notifCounter = 0;

        CanonicalGrabFrame _grabFrame;
        nearby_grab_damping::NearbyGrabDampingState _nearbyGrabDamping;
        RE::NiTransform _adjustedHandTransform;
        bool _hasAdjustedHandTransform = false;
        float _grabVisualLerpElapsed = 0.0f;
        float _grabVisualLerpDuration = 0.5f;
        float _grabDeviationExceededSeconds = 0.0f;
        std::array<RE::NiPoint3, 5> _grabFingerProbeStart{};
        std::array<RE::NiPoint3, 5> _grabFingerProbeEnd{};
        bool _hasGrabFingerProbeDebug = false;
        std::array<float, 15> _grabFingerJointPose{};
        std::array<RE::NiTransform, 15> _grabFingerLocalTransforms{};
        std::uint16_t _grabFingerLocalTransformMask = 0;
        bool _hasGrabFingerJointPose = false;
        bool _hasGrabFingerLocalTransforms = false;
        bool _selectedCloseFingerPoseActive = false;
        RE::NiPoint3 _lastSelectedCloseOrigin{};
        bool _hasLastSelectedCloseOrigin = false;
        float _selectedCloseHandSpeedMetersPerSecond = 0.0f;
        int _grabFingerPoseFrameCounter = 0;

        static constexpr std::size_t GRAB_RELEASE_VELOCITY_HISTORY = 5;
        std::array<RE::NiPoint3, GRAB_RELEASE_VELOCITY_HISTORY> _heldLocalLinearVelocityHistory{};
        std::size_t _heldLocalLinearVelocityHistoryCount = 0;
        std::size_t _heldLocalLinearVelocityHistoryNext = 0;
        RE::NiPoint3 _lastPlayerSpaceVelocityHavok{};

        std::vector<std::uint32_t> _heldBodyIds;
        std::vector<std::uint32_t> _pulledBodyIds;
        std::uint32_t _pulledPrimaryBodyId = INVALID_BODY_ID;
        RE::NiPoint3 _pullPointOffsetHavok{};
        RE::NiPoint3 _pullTargetHavok{};
        float _pullElapsedSeconds = 0.0f;
        float _pullDurationSeconds = 0.0f;
        bool _pullHasTarget = false;

        static constexpr int MAX_HELD_BODIES = 64;
        std::uint32_t _heldBodyIdsSnapshot[MAX_HELD_BODIES] = {};
        std::atomic<int> _heldBodyIdsCount{ 0 };

        std::atomic<bool> _isHoldingFlag{ false };

        VatsSelectionHighlight _selectionHighlight;

    public:
        selection_highlight_policy::VatsHighlightTargetChoice chooseSelectionHighlightTarget(const SelectedObject& selection) const
        {
            auto* referenceRoot = selection.refr ? selection.refr->Get3D() : nullptr;
            return selection_highlight_policy::chooseVatsHighlightTarget(selection.hitNode, selection.visualNode, referenceRoot);
        }

        bool playSelectionHighlightCandidate(RE::TESObjectREFR* refr, RE::NiAVObject* node, selection_highlight_policy::VatsHighlightSource source)
        {
            if (!node) {
                return false;
            }

            return _selectionHighlight.play(refr, node, g_rockConfig.rockHighlightEnabled, handName(), selection_highlight_policy::vatsHighlightSourceName(source));
        }

        bool playSelectionHighlight(const SelectedObject& selection)
        {
            if (!selection_highlight_policy::shouldUseVatsHighlightForSelection(selection.isFarSelection)) {
                stopSelectionHighlight();
                return false;
            }

            auto* referenceRoot = selection.refr ? selection.refr->Get3D() : nullptr;
            _selectionHighlightRefreshFrames = 0;
            if (playSelectionHighlightCandidate(selection.refr, referenceRoot, selection_highlight_policy::VatsHighlightSource::ReferenceRoot)) {
                return true;
            }
            if (selection.visualNode != selection.hitNode &&
                playSelectionHighlightCandidate(selection.refr, selection.visualNode, selection_highlight_policy::VatsHighlightSource::VisualNode)) {
                return true;
            }
            if (selection.hitNode != referenceRoot && playSelectionHighlightCandidate(selection.refr, selection.hitNode, selection_highlight_policy::VatsHighlightSource::HitNode)) {
                return true;
            }

            return false;
        }

        void refreshSelectionHighlight(const SelectedObject& selection)
        {
            if (!g_rockConfig.rockHighlightEnabled || !selection.isValid() ||
                !selection_highlight_policy::shouldUseVatsHighlightForSelection(selection.isFarSelection)) {
                stopSelectionHighlight();
                return;
            }

            ++_selectionHighlightRefreshFrames;
            if (!selection_highlight_policy::shouldRefreshVatsHighlightAttempt(g_rockConfig.rockHighlightEnabled,
                    selection.refr != nullptr,
                    _selectionHighlightRefreshFrames,
                    selection_highlight_policy::kVatsHighlightRefreshIntervalFrames)) {
                return;
            }

            if (_selectionHighlight.isActiveForReference(selection.refr)) {
                _selectionHighlight.refreshActive(g_rockConfig.rockHighlightEnabled, handName());
            } else {
                playSelectionHighlight(selection);
            }
            _selectionHighlightRefreshFrames = 0;
        }

        void stopSelectionHighlight()
        {
            _selectionHighlightRefreshFrames = 0;
            _selectionHighlight.stop();
        }

    };
}
