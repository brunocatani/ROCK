#pragma once

#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabTelemetry.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/hand/HandBoneColliderSet.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/hand/HandInteractionStateMachine.h"
#include "physics-interaction/hand/SelectionBeamEffect.h"
#include "physics-interaction/grab/NearbyGrabDamping.h"
#include "physics-interaction/object/ObjectDetection.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
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
#include <cstdint>
#include <limits>
#include <mutex>
#include <vector>

namespace rock
{
    class BodyBoneColliderSet;

    constexpr std::uint32_t ROCK_HAND_LAYER = 43;

    constexpr std::uint32_t INVALID_BODY_ID = 0x7FFF'FFFF;
    inline constexpr std::size_t kGrabCollisionSuppressionArmBodyCountPerHand = 3;
    inline constexpr std::size_t kGrabCollisionSuppressionBodyCountPerHand =
        hand_collider_semantics::kHandColliderBodyCountPerHand + kGrabCollisionSuppressionArmBodyCountPerHand;

    struct GrabPivotDebugSnapshot
    {
        RE::NiPoint3 handPivotWorld{};
        RE::NiPoint3 objectPivotWorld{};
        RE::NiPoint3 handBodyWorld{};
        RE::NiPoint3 objectBodyWorld{};
        float pivotErrorGameUnits = 0.0f;
    };

    struct GrabPocketNormalDebugSnapshot
    {
        RE::NiPoint3 contactPointWorld{};
        RE::NiPoint3 normalEndWorld{};
    };

    struct GrabAuthorityProxyDebugSnapshot
    {
        RE::NiTransform palmAuthorityBaseWorld{};
        RE::NiTransform proxyTargetWorld{};
        RE::NiPoint3 localOffsetGameUnits{};
        body_frame::BodyFrameSource palmSource{ body_frame::BodyFrameSource::Fallback };
        std::uint32_t palmMotionIndex{ body_frame::kFreeMotionIndex };
    };

    struct GrabContactPatchDebugSnapshot
    {
        std::array<RE::NiPoint3, kMaxGrabContactPatchSamples> samplePointsWorld{};
        std::uint32_t sampleCount = 0;
    };

    struct GrabSupportFrameDebugSnapshot
    {
        std::array<RE::NiPoint3, 3> pivotTriangleWorld{};
        RE::NiPoint3 pivotWorld{};
        RE::NiPoint3 normalEndWorld{};
        RE::NiPoint3 supportAxisEndWorld{};
        RE::NiPoint3 binormalEndWorld{};
        float axisLengthGameUnits = 0.0f;
        const char* pivotAuthoritySource = "none";
        const char* activeGrabPointMode = "none";
        const char* supportKind = "none";
        const char* supportReason = "none";
        bool hasNormal = false;
        bool hasSupportAxis = false;
        bool hasBinormal = false;
        bool hasPivotTriangle = false;
        bool authoredSupportPivot = false;
        bool positionOnlyPivot = false;
        bool normalTrusted = false;
    };

    struct GrabForceTorqueDebugSnapshot
    {
        std::array<RE::NiPoint3, kMaxGrabContactPatchSamples> contactSamplePointsWorld{};
        std::array<RE::NiPoint3, 3> pivotTriangleWorld{};
        RE::hknpBodyId pivotSourceBodyId{ INVALID_BODY_ID };
        RE::NiTransform liveBodyWorld{};
        RE::NiTransform desiredBodyWorld{};
        RE::NiTransform motorConstraintAWorld{};
        RE::NiTransform motorConstraintBWorld{};
        RE::NiTransform motorAtomTargetBodyWorld{};
        RE::NiTransform motorColumnTargetBodyWorld{};
        RE::NiTransform motorRelationInputBodyWorld{};
        RE::NiTransform motorRelationInverseBodyWorld{};
        RE::NiTransform motorSolverEffectiveBodyWorld{};
        RE::NiPoint3 targetPivotWorld{};
        RE::NiPoint3 livePivotWorld{};
        RE::NiPoint3 motorAnchorAWorld{};
        RE::NiPoint3 motorAnchorBWorld{};
        RE::NiPoint3 motorAtomTargetPivotWorld{};
        RE::NiPoint3 motorRelationPivotWorld{};
        RE::NiPoint3 motorAngularAxisEndWorld{};
        RE::NiPoint3 motorTargetBodyDeltaEndWorld{};
        RE::NiPoint3 activePivotBLiveBodyWorld{};
        RE::NiPoint3 activePivotBDesiredBodyWorld{};
        RE::NiPoint3 activePivotBVisualNodeWorld{};
        RE::NiPoint3 correctionEndWorld{};
        RE::NiPoint3 leverArmEndWorld{};
        RE::NiPoint3 torqueAxisEndWorld{};
        RE::NiPoint3 meshGripPointWorld{};
        RE::NiPoint3 visualMeshGripPointWorld{};
        RE::NiPoint3 captureMeshGripPointBodyWorld{};
        RE::NiPoint3 captureMeshGripPointVisualWorld{};
        RE::NiPoint3 contactPatchPointWorld{};
        float pivotErrorGameUnits = 0.0f;
        float pivotTrackingErrorGameUnits = 0.0f;
        float bodyVisualMeshLockErrorGameUnits = 0.0f;
        float activePivotBVisualLockErrorGameUnits = 0.0f;
        float captureGripLocalDeltaGameUnits = 0.0f;
        float captureFreezeBodyShiftGameUnits = 0.0f;
        float captureFreezeBodyRotationDegrees = 0.0f;
        float captureFreezePivotGapBeforeGameUnits = 0.0f;
        float captureFreezePivotGapAfterGameUnits = 0.0f;
        float captureFreezeShiftDot = 0.0f;
        float captureFreezePivotLeverGameUnits = 0.0f;
        float rotationErrorDegrees = 0.0f;
        float leverLengthGameUnits = 0.0f;
        float correctionLengthGameUnits = 0.0f;
        float torqueWitnessGameUnitsSquared = 0.0f;
        float motorTargetBodyDeltaGameUnits = 0.0f;
        float motorTargetBodyDeltaDegrees = 0.0f;
        float motorColumnTargetBodyDeltaDegrees = 0.0f;
        float motorRelationInverseBodyDeltaGameUnits = 0.0f;
        float motorRelationInverseBodyDeltaDegrees = 0.0f;
        float motorAtomToRelationInverseDeltaDegrees = 0.0f;
        float motorSolverEffectiveBodyDeltaGameUnits = 0.0f;
        float motorSolverEffectiveBodyDeltaDegrees = 0.0f;
        float motorSolverEffectiveToAtomDeltaDegrees = 0.0f;
        float motorSolverEffectiveLiveABodyDeltaDegrees = 0.0f;
        float motorSolverEffectiveTargetALiveADeltaDegrees = 0.0f;
        float motorPhysicsProxyToLiveProxyDeltaDegrees = 0.0f;
        float motorPhysicsProxyToLiveProxyDeltaGameUnits = 0.0f;
        float motorRelationToConstraintATargetDegrees = 0.0f;
        float motorRelationToConstraintALiveDegrees = 0.0f;
        float motorTransformARawMaxDelta = 0.0f;
        float motorTransformBRawMaxDelta = 0.0f;
        float motorTargetBRcaRawMaxDelta = 0.0f;
        float motorTargetProxyToLiveProxyDeltaDegrees = 0.0f;
        float motorTargetProxyToLiveProxyDeltaGameUnits = 0.0f;
        float motorTransformBRelationLocalDeltaGameUnits = 0.0f;
        float motorTransformBPivotToAnchorAGameUnits = 0.0f;
        float pocketDistanceGameUnits = 0.0f;
        float selectionDistanceGameUnits = 0.0f;
        float longLeverGameUnits = 0.0f;
        float positionConfidence = 0.0f;
        const char* pivotAuthoritySource = "none";
        const char* activeGrabPointMode = "none";
        const char* authorityFrameSource = "none";
        const char* acquisitionPhase = "none";
        const char* capturePivotAuthoritySource = "none";
        const char* captureGrabPointMode = "none";
        const char* lastSeatedPivotReacquireReason = "none";
        std::uint32_t contactSampleCount = 0;
        std::uint32_t seatedPivotReacquireCount = 0;
        bool hasTorqueAxis = false;
        bool hasMotorConstraintFrames = false;
        bool hasMotorColumnTargetBody = false;
        bool hasMotorRelationFrames = false;
        bool hasMotorSolverEffectiveBody = false;
        bool hasMotorAngularCommand = false;
        bool hasMotorTargetBodyDelta = false;
        bool hasPivotTriangle = false;
        bool hasMeshGripPoint = false;
        bool hasVisualMeshGripPoint = false;
        bool hasActivePivotBVisualNode = false;
        bool hasCaptureMeshGripPoint = false;
        bool gripPointMutatedAfterCapture = false;
        bool hasContactPatchPoint = false;
        bool positionOnlyPivot = false;
        bool normalTrusted = false;
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

    enum class GrabReleaseDisposition : std::uint8_t
    {
        PhysicalDrop,
        PendingInventoryTransfer,
        TransferToInventory,
        PendingConsumeTransfer,
        OwnershipHandoff,
    };

    struct GrabSharedObjectContext
    {
        bool joiningPeerHeldObject = false;
        const SavedObjectState* peerSavedObjectState = nullptr;
        const active_grab_body_lifecycle::BodyLifecycleSnapshot* peerActiveGrabLifecycle = nullptr;
        const std::vector<std::uint32_t>* peerHeldBodyIds = nullptr;

        [[nodiscard]] bool hasPeerState() const noexcept
        {
            return joiningPeerHeldObject && peerSavedObjectState && peerSavedObjectState->isValid();
        }
    };

    struct GrabReleaseContext
    {
        bool finalObjectRelease = true;
        bool peerHandStillHolding = false;
        GrabReleaseDisposition disposition = GrabReleaseDisposition::PhysicalDrop;
        const char* reason = "single-hand";
    };

    struct GrabReleaseOutcome
    {
        struct VelocitySnapshot
        {
            static constexpr std::size_t kMaxBodyIds = 64;

            bool available = false;
            RE::hknpBodyId primaryBodyId{ INVALID_BODY_ID };
            std::array<std::uint32_t, kMaxBodyIds> bodyIds{};
            std::uint32_t bodyCount = 0;
            RE::NiPoint3 linearVelocityHavok{};
            RE::NiPoint3 angularVelocityRadiansPerSecond{};
            bool overrideAngularVelocity = false;
        };

        bool released = false;
        RE::TESObjectREFR* refr = nullptr;
        std::uint32_t formID = 0;
        bool finalObjectRelease = true;
        VelocitySnapshot velocity{};
    };

    class Hand
    {
    public:
        enum class ActorEquipmentDropHandoffStatus : std::uint8_t
        {
            None,
            Pending,
            Ready,
            InvalidSelection,
            MissingDroppedReference,
            TimedOut,
        };

        explicit Hand(bool isLeft) : _isLeft(isLeft) {}

        bool isLeft() const { return _isLeft; }
        HandState getState() const { return _state; }
        HandState getStateAtomic() const { return _stateAtomic.load(std::memory_order_acquire); }
        bool hasContactEvidenceSuppressedAtomic() const { return suppressesGeneratedHandContactEvidence(getStateAtomic()); }
        const char* handName() const { return _isLeft ? "Left" : "Right"; }

        void reset();

        void collectHeldBodyIds(RE::TESObjectREFR* refr);

    private:
        void collectBodyIdsRecursive(RE::NiAVObject* node, int maxDepth = 10);
        void suppressHandCollisionForGrab(RE::hknpWorld* world, const BodyBoneColliderSet* bodyBoneColliders);
        void restoreHandCollisionAfterGrab(RE::hknpWorld* world);
        void clearGrabHandCollisionSuppressionState();
        void clearPullRuntimeState(bool restorePreparedObject = true, const char* context = "clear-pull-runtime");
        void clearPullPrepTracking();
        void restorePullPrepIfActive(const char* context);
        bool consumePullPrepLifecycleForActiveGrab(RE::TESObjectREFR* refr, active_grab_body_lifecycle::BodyLifecycleSnapshot& outLifecycle);
        object_physics_body_set::BodySetScanOptions makeActiveGrabBodyScanOptions(const SelectedObject& selection) const;
        void clearGrabAcquisitionCache(const char* reason);
        void updateGrabAcquisitionCache(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld);
        bool grabAcquisitionCacheMatches(
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            const SelectedObject& selection,
            const object_physics_body_set::BodySetScanOptions& options) const;
        bool tryUseGrabAcquisitionBeforePrepCache(
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            const SelectedObject& selection,
            const object_physics_body_set::BodySetScanOptions& options,
            object_physics_body_set::ObjectPhysicsBodySet& outBodySet) const;
        bool tryBuildGrabAcquisitionPreparedBodySetFromCache(
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            const SelectedObject& selection,
            const object_physics_body_set::BodySetScanOptions& options,
            object_physics_body_set::ObjectPhysicsBodySet& outBodySet,
            bool& outPostPrepComplete) const;
        void updateSelectedCloseFingerPose();
        void clearSelectedCloseFingerPose();
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

        bool isHolding() const { return isHoldingState(_state); }
        RE::TESObjectREFR* getHeldRef() const { return _savedObjectState.refr; }
        const ActiveConstraint& getActiveConstraint() const { return _activeConstraint; }
        const SavedObjectState& getSavedObjectState() const { return _savedObjectState; }
        const active_grab_body_lifecycle::BodyLifecycleSnapshot& getActiveGrabLifecycle() const { return _activeGrabLifecycle; }
        bool tryGetHeldObjectGrabPivotWorld(RE::hknpWorld* world, RE::NiPoint3& outPivotWorld) const;
        bool getGrabPivotDebugSnapshot(RE::hknpWorld* world, GrabPivotDebugSnapshot& out) const;
        bool getGrabPocketNormalDebugSnapshot(RE::hknpWorld* world, GrabPocketNormalDebugSnapshot& out) const;
        bool getGrabAuthorityProxyDebugSnapshot(RE::hknpWorld* world, const RE::NiTransform& rawHandWorld, GrabAuthorityProxyDebugSnapshot& out) const;
        bool getGrabContactPatchDebugSnapshot(RE::hknpWorld* world, GrabContactPatchDebugSnapshot& out) const;
        bool getGrabSupportFrameDebugSnapshot(RE::hknpWorld* world, GrabSupportFrameDebugSnapshot& out) const;
        bool getGrabForceTorqueDebugSnapshot(RE::hknpWorld* world, const RE::NiTransform& rawHandWorld, GrabForceTorqueDebugSnapshot& out) const;
        bool getGrabTransformTelemetrySnapshot(RE::hknpWorld* world,
            const RE::NiTransform& rawHandWorld,
            grab_transform_telemetry::RuntimeSample& out) const;

        bool getGrabFingerProbeDebug(std::array<RE::NiPoint3, 5>& outStart, std::array<RE::NiPoint3, 5>& outEnd) const
        {
            if (!_hasGrabFingerProbeDebug)
                return false;
            outStart = _grabFingerProbeStart;
            outEnd = _grabFingerProbeEnd;
            return true;
        }

        bool getGrabFingerPadProbeDebug(
            std::array<RE::NiPoint3, 5>& outStart,
            std::array<RE::NiPoint3, 5>& outEnd,
            std::array<RE::NiPoint3, 5>& outHit,
            std::array<std::uint8_t, 5>& outHitValid) const
        {
            if (!_hasGrabFingerPadProbeDebug)
                return false;
            outStart = _grabFingerPadProbeStart;
            outEnd = _grabFingerPadProbeEnd;
            outHit = _grabFingerPadProbeHit;
            outHitValid = _grabFingerPadProbeHitValid;
            return true;
        }

        bool getGrabFingerSurfaceTargetDebug(
            std::array<RE::NiPoint3, 5>& outTarget,
            std::array<std::uint8_t, 5>& outTargetValid) const
        {
            if (!_hasGrabFingerSurfaceTargetDebug)
                return false;
            outTarget = _grabFingerSurfaceTarget;
            outTargetValid = _grabFingerSurfaceTargetValid;
            return true;
        }

        bool grabSelectedObject(RE::hknpWorld* world,
            const RE::NiTransform& handWorldTransform,
            float tau,
            float damping,
            float maxForce,
            float proportionalRecovery,
            float constantRecovery,
            const BodyBoneColliderSet* bodyBoneColliders,
            const GrabSharedObjectContext& sharedContext = {});

        bool acquirePeerHeldCloseSelection(RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            const SavedObjectState& peerSavedObjectState,
            const std::vector<std::uint32_t>& peerHeldBodyIds,
            const RE::NiPoint3& selectionOrigin,
            const RE::NiPoint3& palmNormal,
            float nearRange,
            const char** outRefusalReason = nullptr);

        bool promoteHeldObjectToConstraintDrive(RE::bhkWorld* bhkWorld,
            RE::hknpWorld* world,
            const RE::NiTransform& handWorldTransform,
            float tau,
            float damping,
            float maxForce,
            float proportionalRecovery,
            float constantRecovery,
            const char* reason);

        void updateHeldObject(RE::hknpWorld* world,
            const RE::NiTransform& handWorldTransform,
            const HeldObjectPlayerSpaceFrame& playerSpaceFrame,
            float deltaTime,
            float forceFadeInTime,
            float tauMin,
            const BodyBoneColliderSet* bodyBoneColliders,
            const GrabReleaseContext& releaseContext = {});
        void captureHeldReleaseMotion(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, const HeldObjectPlayerSpaceFrame& playerSpaceFrame, float deltaTime);
        void applyReleaseVelocitySnapshot(RE::hknpWorld* world, const GrabReleaseOutcome::VelocitySnapshot& snapshot) const;

        GrabReleaseOutcome releaseGrabbedObject(
            RE::hknpWorld* world,
            GrabReleaseCollisionRestoreMode collisionRestoreMode = GrabReleaseCollisionRestoreMode::Delayed,
            const GrabReleaseContext& releaseContext = {});
        void abandonHavokStateAfterWorldLoss();
        void updateDelayedGrabHandCollisionRestore(RE::hknpWorld* world, float deltaTime);

        bool lockFarSelection();
        bool startDynamicPull(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform);
        bool updateDynamicPull(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, float deltaTime);
        void finishPullPrepAsPhysicalDropIfActive(const char* context);
        bool hasActivePullCatchIntent() const;
        bool hasArrivedPullCatchIntent() const;
        bool hasPendingPullCatchCommit() const;
        bool advancePullCatchCommit(float deltaTime, float maxCommitSeconds);
        void notePullCatchCommitAttemptFailed();
        RE::TESObjectREFR* getPullCatchIntentRef() const;
        bool reacquirePullCatchCloseSelection(RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            const RE::NiPoint3& selectionOrigin,
            const RE::NiPoint3& palmNormal,
            float radiusGameUnits,
            float maxBodyDistanceGameUnits);
        bool beginActorEquipmentDropHandoff(
            const actor_equipment_grab::DropResult& dropResult,
            const RE::NiPoint3& sourceHitPointWorld);
        bool hasPendingActorEquipmentDropHandoff() const;
        ActorEquipmentDropHandoffStatus advanceActorEquipmentDropHandoff(
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            float deltaSeconds,
            float maxHandoffSeconds);
        bool replaceFarActorEquipmentSelectionWithDroppedObject(RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            RE::TESObjectREFR* droppedRef,
            const RE::NiPoint3& sourceHitPointWorld);
        void clearActorEquipmentDropHandoff(const char* reason = "cleared");
        void clearPullCatchIntent(const char* reason = "cleared");
        void clearSelectionState(bool rememberDeselect);

        void tickTouchState() { _touchActiveFrames++; }

        void updateSelection(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& selectionOrigin, const RE::NiPoint3& closeSelectionDirection,
            const RE::NiPoint3& farSelectionDirection, const RE::NiPoint3& pinchOrigin, const RE::NiPoint3& pinchDirection, bool hasPinchOrigin,
            const FarSelectionHmdConeGate& farHmdConeGate, float nearRange, float farRange, float deltaTime, const OtherHandSelectionContext& otherHandContext);
        void preloadSelectionBeam();
        void updateSelectionBeam(RE::hknpWorld* hknpWorld, const RE::NiPoint3& selectionOrigin);
        void stopSelectionBeam();

        struct LivePalmAnchorReference
        {
            bool valid = false;
            RE::NiTransform world{};
            body_frame::BodyFrameSource source{ body_frame::BodyFrameSource::Fallback };
            std::uint32_t motionIndex{ body_frame::kFreeMotionIndex };
            bool hasMotionVelocity = false;
            RE::NiPoint3 linearVelocityHavok{};
            RE::NiPoint3 angularVelocityRadiansPerSecond{};
        };

        RE::hknpBodyId getCollisionBodyId() const { return _handBody.getBodyId(); }
        RE::hknpBodyId getGrabAuthorityProxyBodyId() const { return _grabAuthorityProxy.getBodyId(); }
        bool hasCollisionBody() const { return _handBody.isValid(); }
        BethesdaPhysicsBody& getHandBody() { return _handBody; }
        const BethesdaPhysicsBody& getHandBody() const { return _handBody; }
        bool tryResolveLivePalmAnchorReference(RE::hknpWorld* world, LivePalmAnchorReference& outReference) const;
        RE::NiPoint3 computeGrabPivotAWorld(RE::hknpWorld* world, const RE::NiTransform& fallbackHandWorldTransform) const;
        bool tryComputeGrabProxyLocalPalmPocketFrameWorld(RE::hknpWorld* world, RE::NiTransform& outFrameWorld) const;
        bool tryComputeGrabProxyLocalPalmPocketPivotAWorld(RE::hknpWorld* world, RE::NiPoint3& outPivotWorld) const;
        std::uint32_t getHandColliderBodyCount() const { return _boneColliders.getBodyCount(); }
        std::uint32_t getHandColliderBodyIdAtomic(std::size_t index) const { return _boneColliders.getBodyIdAtomic(index); }
        bool isHandColliderBodyId(std::uint32_t bodyId) const { return _boneColliders.isColliderBodyIdAtomic(bodyId); }
        bool tryGetHandColliderMetadata(std::uint32_t bodyId, HandColliderBodyMetadata& outMetadata) const { return _boneColliders.tryGetBodyMetadataAtomic(bodyId, outMetadata); }
        bool tryGetPalmAnchorTarget(RE::NiTransform& outTarget) const { return _boneColliders.tryGetPalmAnchorTarget(outTarget); }
        void recordSemanticContact(const HandColliderBodyMetadata& metadata,
            std::uint32_t otherBodyId,
            const hand_semantic_contact_state::SemanticContactVector* contactPointGame = nullptr,
            const hand_semantic_contact_state::SemanticContactVector* contactNormalGame = nullptr);
        void clearSemanticContactEvidence();
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

        bool createCollision(RE::hknpWorld* world, void* bhkWorld, const RE::NiTransform& rollAuthorityWorld);

        void destroyCollision(void* bhkWorld);

        void updateCollisionTransform(
            RE::hknpWorld* world,
            const RE::NiTransform& rollAuthorityWorld,
            float deltaTime,
            const RE::NiPoint3& authorityTranslationOffsetGame = RE::NiPoint3{});

        void flushPendingCollisionPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        void flushPendingCustomGrabAuthority(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        void observeCustomGrabAuthorityAfterSolve(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        bool beginStashCandidate();
        bool cancelStashCandidate();
        bool beginConsumeCandidate();
        bool cancelConsumeCandidate();

    private:
        HandTransitionResult applyTransition(const HandTransitionRequest& request);

        bool createProxyConstraintGrabDrive(RE::bhkWorld* bhkWorld,
            RE::hknpWorld* world,
            RE::hknpBodyId objectBodyId,
            const RE::NiTransform& proxyWorldTransform,
            const RE::NiTransform& rawHandWorldTransform,
            const RE::NiPoint3& grabPivotAWorld,
            float tau,
            float damping,
            float maxForce,
            float authorityForceScale,
            float proportionalRecovery,
            float constantRecovery,
            bool looseWeaponGrab,
            const char* reason);
        bool updateProxyConstraintGrabDriveTarget(RE::hknpWorld* world,
            const RE::NiTransform& proxyWorldTransform,
            RE::NiTransform& outDesiredObjectWorld,
            RE::NiTransform& outDesiredBodyWorld,
            RE::NiPoint3& outDesiredTargetPointWorld,
            RE::NiPoint3& outActivePivotBBodyLocalGame);
        enum class GrabAuthorityProxyFramePolicy : std::uint8_t
        {
            LivePalmOnly = 0,
            PreferQueuedPalmTarget
        };
        bool resolveGrabAuthorityProxyFrame(RE::hknpWorld* world,
            const RE::NiTransform& rawHandWorld,
            const RE::NiTransform* fallbackPalmAnchorWorld,
            RE::NiTransform& outProxyWorld,
            const char*& outSource,
            GrabAuthorityProxyFramePolicy policy = GrabAuthorityProxyFramePolicy::LivePalmOnly) const;
        bool resolveActiveGrabAuthorityPivotAWorld(
            const RE::NiTransform& proxyWorldTransform,
            RE::NiPoint3& outPivotWorld) const;
        void updateConstraintGrabDriveMotors(RE::hknpWorld* world,
            float deltaTime,
            float forceFadeInTime,
            float tauMin,
            float authorityForceScale,
            bool heldBodyColliding,
            const grab_motion_controller::HeldAuthorityState& heldAuthority);
        void queueProxyGrabAuthorityTarget(const RE::NiTransform& proxyWorldTransform,
            const RE::NiTransform& rawHandWorldTransform,
            const char* proxyFrameSource,
            float deltaTime,
            float forceFadeInTime,
            float tauMin,
            float grabPositionErrorGameUnits,
            float grabRotationErrorDegrees,
            float authorityForceScale,
            bool heldBodyColliding);
        void destroyGrabAuthorityProxy(RE::bhkWorld* bhkWorld);
        void abandonGrabAuthorityProxy();
        void clearGrabAuthorityProxyRuntime();
        void destroyGrabAuthorityProxyLocked(RE::bhkWorld* bhkWorld);
        void abandonGrabAuthorityProxyLocked();
        void clearGrabAuthorityProxyRuntimeLocked();
        bool tryGetGrabDriveObjectWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId, RE::NiTransform& outTransform) const;
        RE::NiPoint3 activeProxyConstraintPivotBLocalGame() const;

        /*
         * Far-pull arrival needs explicit ownership separate from button edges.
         * ROCK keeps a held grab request alive while the object moves into the
         * hand, but release still cancels and arrival retries only the close
         * commit, not the dynamic pull itself. This prevents one failed arrival
         * frame from forcing a second grip press.
         */
        struct PullCatchIntent
        {
            bool active = false;
            bool commitPending = false;
            RE::TESObjectREFR* refr = nullptr;
            std::uint32_t formId = 0;
            std::uint32_t primaryBodyId = INVALID_BODY_ID;
            grab_target::Kind targetKind = grab_target::Kind::LooseObject;
            float commitElapsedSeconds = 0.0f;
            std::uint32_t failedCommitAttempts = 0;
        };

        /*
         * ROCK stages actor-equipment drops through a pre-pull/pre-grab item
         * state because the spawned reference can exist before its scene tree and
         * hknp bodies are queryable. The selected actor equipment remains the
         * authority until the dropped ref scans as a normal loose object, then
         * the existing far-pull code takes over unchanged.
         */
        struct ActorEquipmentDropHandoff
        {
            bool active = false;
            RE::ObjectRefHandle handle{};
            RE::NiPointer<RE::TESObjectREFR> droppedRef;
            RE::NiPoint3 sourceHitPointWorld{};
            float elapsedSeconds = 0.0f;
            std::uint32_t attempts = 0;
            std::uint32_t actorFormId = 0;
            std::uint32_t itemFormId = 0;
            std::uint32_t droppedFormId = 0;
        };

        struct GrabAcquisitionCache
        {
            enum class Stage : std::uint8_t
            {
                Empty,
                PreScanRunning,
                PreScanReady,
            };

            RE::TESObjectREFR* refr = nullptr;
            RE::NiPointer<RE::NiAVObject> rootNode;
            RE::NiPointer<RE::NiAVObject> hitNode;
            RE::bhkWorld* bhkWorld = nullptr;
            RE::hknpWorld* hknpWorld = nullptr;
            std::uint32_t formId = 0;
            std::uint32_t selectedBodyId = INVALID_BODY_ID;
            std::uint32_t rightHandBodyId = INVALID_BODY_ID;
            std::uint32_t leftHandBodyId = INVALID_BODY_ID;
            std::uint32_t sourceBodyId = INVALID_BODY_ID;
            grab_target::Kind targetKind = grab_target::Kind::LooseObject;
            int maxDepth = 0;
            Stage stage = Stage::Empty;
            object_physics_body_set::ObjectPhysicsBodyScanCursor scanCursor;
            object_physics_body_set::ObjectPhysicsBodyScanCache scanCache;
            object_physics_body_set::ObjectPhysicsBodySet beforePrepBodySet;
            object_physics_body_set::ObjectPhysicsBodyScanCache postPrepScanCache;
            object_physics_body_set::ObjectPhysicsBodySet postPrepBodySet;
            bool postPrepComplete = false;
            bool valid = false;

            void clear() noexcept
            {
                refr = nullptr;
                rootNode.reset();
                hitNode.reset();
                bhkWorld = nullptr;
                hknpWorld = nullptr;
                formId = 0;
                selectedBodyId = INVALID_BODY_ID;
                rightHandBodyId = INVALID_BODY_ID;
                leftHandBodyId = INVALID_BODY_ID;
                sourceBodyId = INVALID_BODY_ID;
                targetKind = grab_target::Kind::LooseObject;
                maxDepth = 0;
                stage = Stage::Empty;
                scanCursor.clear();
                scanCache.clear();
                beforePrepBodySet = {};
                postPrepScanCache.clear();
                postPrepBodySet = {};
                postPrepComplete = false;
                valid = false;
            }
        };

        void armPullCatchIntent(RE::TESObjectREFR* refr, std::uint32_t primaryBodyId, grab_target::Kind targetKind);
        void markPullCatchIntentArrived();
        bool pullCatchIntentMatchesSelection() const;

        bool _isLeft;
        HandState _state = HandState::Idle;
        HandState _prevState = HandState::Idle;
        std::atomic<HandState> _stateAtomic{ HandState::Idle };

        bool _idleDesired = false;
        bool _grabRequested = false;
        bool _releaseRequested = false;

        BethesdaPhysicsBody _handBody;
        HandBoneColliderSet _boneColliders;

        SelectedObject _currentSelection;
        SelectedObject _cachedFarCandidate;
        GrabAcquisitionCache _grabAcquisitionCache;
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
        std::atomic<std::uint32_t> _heldContactHeldBodyId{ INVALID_BODY_ID };
        std::atomic<std::uint32_t> _heldContactOtherBodyId{ INVALID_BODY_ID };
        std::atomic<std::uint32_t> _heldContactOtherLayer{ 0xFFFF'FFFFu };
        std::atomic<std::uint32_t> _heldContactHasNormal{ 0 };
        std::atomic<float> _heldContactPointHavokX{ 0.0f };
        std::atomic<float> _heldContactPointHavokY{ 0.0f };
        std::atomic<float> _heldContactPointHavokZ{ 0.0f };
        std::atomic<float> _heldContactNormalHavokX{ 0.0f };
        std::atomic<float> _heldContactNormalHavokY{ 0.0f };
        std::atomic<float> _heldContactNormalHavokZ{ 0.0f };

        hand_collision_suppression_math::SuppressionSet<kGrabCollisionSuppressionBodyCountPerHand> _grabHandCollisionSuppression{};
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
        std::atomic<std::uint32_t> _semanticContactHasPointGame{ 0 };
        std::atomic<std::uint32_t> _semanticContactHasNormalGame{ 0 };
        std::atomic<float> _semanticContactPointGameX{ 0.0f };
        std::atomic<float> _semanticContactPointGameY{ 0.0f };
        std::atomic<float> _semanticContactPointGameZ{ 0.0f };
        std::atomic<float> _semanticContactNormalGameX{ 0.0f };
        std::atomic<float> _semanticContactNormalGameY{ 0.0f };
        std::atomic<float> _semanticContactNormalGameZ{ 0.0f };
        std::mutex _semanticContactWriteMutex;
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetValid{};
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetRole{};
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetFinger{};
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetSegment{};
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetHandBodyId{};
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetOtherBodyId{};
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetFrames{};
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetSequence{};
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetHasPointGame{};
        std::array<std::atomic<std::uint32_t>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetHasNormalGame{};
        std::array<std::atomic<float>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetPointGameX{};
        std::array<std::atomic<float>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetPointGameY{};
        std::array<std::atomic<float>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetPointGameZ{};
        std::array<std::atomic<float>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetNormalGameX{};
        std::array<std::atomic<float>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetNormalGameY{};
        std::array<std::atomic<float>, hand_semantic_contact_state::kMaxSemanticContactRecords> _semanticContactSetNormalGameZ{};

    public:
        bool isHeldBodyColliding() const { return _heldBodyContactFrame.load(std::memory_order_acquire) < 5; }

        struct HeldBodyContactSnapshot
        {
            RE::NiPoint3 contactPointHavok{};
            RE::NiPoint3 contactNormalHavok{};
            std::uint32_t heldBodyId = INVALID_BODY_ID;
            std::uint32_t otherBodyId = INVALID_BODY_ID;
            std::uint32_t otherLayer = 0xFFFF'FFFFu;
            bool recent = false;
            bool hasNormal = false;
        };

        void clearHeldBodyContactSnapshot()
        {
            _heldBodyContactFrame.store(100, std::memory_order_release);
            _heldContactHeldBodyId.store(INVALID_BODY_ID, std::memory_order_release);
            _heldContactOtherBodyId.store(INVALID_BODY_ID, std::memory_order_release);
            _heldContactOtherLayer.store(0xFFFF'FFFFu, std::memory_order_release);
            _heldContactHasNormal.store(0, std::memory_order_release);
        }

        void notifyHeldBodyContact()
        {
            _heldContactHeldBodyId.store(INVALID_BODY_ID, std::memory_order_relaxed);
            _heldContactOtherBodyId.store(INVALID_BODY_ID, std::memory_order_relaxed);
            _heldContactOtherLayer.store(0xFFFF'FFFFu, std::memory_order_relaxed);
            _heldContactHasNormal.store(0, std::memory_order_relaxed);
            _heldBodyContactFrame.store(0, std::memory_order_release);
        }

        void notifyHeldBodyContact(std::uint32_t heldBodyId,
            std::uint32_t otherBodyId,
            std::uint32_t otherLayer,
            const RE::NiPoint3& contactPointHavok,
            const RE::NiPoint3& contactNormalHavok,
            bool hasNormal)
        {
            _heldContactHeldBodyId.store(heldBodyId, std::memory_order_relaxed);
            _heldContactOtherBodyId.store(otherBodyId, std::memory_order_relaxed);
            _heldContactOtherLayer.store(otherLayer, std::memory_order_relaxed);
            _heldContactPointHavokX.store(contactPointHavok.x, std::memory_order_relaxed);
            _heldContactPointHavokY.store(contactPointHavok.y, std::memory_order_relaxed);
            _heldContactPointHavokZ.store(contactPointHavok.z, std::memory_order_relaxed);
            _heldContactNormalHavokX.store(contactNormalHavok.x, std::memory_order_relaxed);
            _heldContactNormalHavokY.store(contactNormalHavok.y, std::memory_order_relaxed);
            _heldContactNormalHavokZ.store(contactNormalHavok.z, std::memory_order_relaxed);
            _heldContactHasNormal.store(hasNormal ? 1u : 0u, std::memory_order_relaxed);
            _heldBodyContactFrame.store(0, std::memory_order_release);
        }

        HeldBodyContactSnapshot readHeldBodyContactSnapshot() const
        {
            HeldBodyContactSnapshot snapshot{};
            snapshot.recent = isHeldBodyColliding();
            if (!snapshot.recent) {
                return snapshot;
            }
            snapshot.heldBodyId = _heldContactHeldBodyId.load(std::memory_order_acquire);
            snapshot.otherBodyId = _heldContactOtherBodyId.load(std::memory_order_acquire);
            snapshot.otherLayer = _heldContactOtherLayer.load(std::memory_order_acquire);
            snapshot.hasNormal = _heldContactHasNormal.load(std::memory_order_acquire) != 0;
            snapshot.contactPointHavok = RE::NiPoint3{
                _heldContactPointHavokX.load(std::memory_order_acquire),
                _heldContactPointHavokY.load(std::memory_order_acquire),
                _heldContactPointHavokZ.load(std::memory_order_acquire),
            };
            snapshot.contactNormalHavok = RE::NiPoint3{
                _heldContactNormalHavokX.load(std::memory_order_acquire),
                _heldContactNormalHavokY.load(std::memory_order_acquire),
                _heldContactNormalHavokZ.load(std::memory_order_acquire),
            };
            return snapshot;
        }

        void tickHeldBodyContact()
        {
            int current = _heldBodyContactFrame.load(std::memory_order_acquire);
            if (current < 100) {
                _heldBodyContactFrame.compare_exchange_weak(current, current + 1, std::memory_order_release, std::memory_order_relaxed);
            }
        }

    private:
        struct HeldHandMotionSample
        {
            RE::NiPoint3 localLinearVelocityHavok{};
            RE::NiPoint3 angularVelocityRadiansPerSecond{};
            bool hasLocalLinearVelocity = false;
            bool hasAngularVelocity = false;
        };

        HeldHandMotionSample recordHeldControllerMotionSample(const RE::NiTransform& handWorldTransform, const HeldObjectPlayerSpaceFrame& playerSpaceFrame, float deltaTime);
        void recordHeldObjectVelocitySample(RE::hknpWorld* world, const HeldObjectPlayerSpaceFrame& playerSpaceFrame);

        ActiveConstraint _activeConstraint;
        std::atomic<float> _lastGrabPhysicsHz{ 90.0f };
        std::atomic<float> _lastGrabPhysicsRateForceScale{ 1.0f };
        BethesdaPhysicsBody _grabAuthorityProxy;
        RE::bhkWorld* _grabAuthorityProxyBhkWorld = nullptr;
        RE::hknpWorld* _grabAuthorityProxyHknpWorld = nullptr;
        RE::NiPoint3 _grabAuthorityPivotAProxyLocalGame{};
        RE::NiPoint3 _grabAuthorityPivotBConstraintLocalGame{};
        bool _grabAuthorityProxyFrameValid = false;
        struct GrabAuthorityProxyPendingTarget
        {
            RE::NiTransform proxyWorld{};
            RE::NiTransform rawHandWorld{};
            const char* proxyFrameSource = "unknown";
            float deltaTime = 0.0f;
            float forceFadeInTime = 0.0f;
            float tauMin = 0.0f;
            float grabPositionErrorGameUnits = 0.0f;
            float grabRotationErrorDegrees = 0.0f;
            float authorityForceScale = 1.0f;
            bool heldBodyColliding = false;
            bool valid = false;
        };
        GrabAuthorityProxyPendingTarget _grabAuthorityPendingTarget{};
        RE::NiTransform _lastAppliedGrabAuthorityProxyWorld{};
        RE::NiTransform _lastAppliedGrabAuthorityRawHandWorld{};
        bool _hasLastAppliedGrabAuthorityProxyWorld = false;
        struct RagdollAngularProbePreSolve
        {
            RE::hknpBodyId objectBodyId{ INVALID_BODY_ID };
            RE::NiTransform desiredBodyWorld{};
            RE::NiTransform bodyAWorldBefore{};
            RE::NiTransform bodyWorldBefore{};
            RE::NiTransform targetRelationAWorld{};
            RE::NiTransform liveRelationAWorld{};
            RE::NiTransform targetConstraintAWorld{};
            RE::NiTransform liveConstraintAWorld{};
            RE::NiTransform relationInverseBodyWorld{};
            RE::NiTransform atomRowsBodyWorld{};
            RE::NiTransform solverEffectiveBodyWorld{};
            RE::NiTransform solverEffectiveLiveAWorld{};
            RE::NiMatrix3 transformARotation{};
            RE::NiMatrix3 transformBRotation{};
            std::array<float, 12> targetBRcaRaw{};
            RE::NiPoint3 requiredAxisWorld{};
            RE::NiPoint3 requiredAxisProxyLocal{};
            RE::NiPoint3 linearCorrectionWorld{};
            RE::NiPoint3 linearLeverWorld{};
            RE::NiPoint3 linearTorqueWitnessWorld{};
            RE::NiPoint3 linearTorqueAxisProxyLocal{};
            RE::NiPoint3 angularVelocityBeforeRadians{};
            float beforeErrorDegrees = -1.0f;
            float beforeGripErrorGameUnits = -1.0f;
            float pivotLeverGameUnits = -1.0f;
            float linearTorqueWitnessGameUnitsSquared = -1.0f;
            float linearTorqueAxisDotRequired = 0.0f;
            float angularMotorTau = 0.0f;
            float angularMotorDamping = 0.0f;
            float angularMotorMaxForce = 0.0f;
            float linearMotorMaxForce = 0.0f;
            float targetToHiggsRelationDegrees = -1.0f;
            float transformBFrozenDeltaDegrees = -1.0f;
            float pivotBRelationDeltaGameUnits = -1.0f;
            float transformAPivotRoundTripDeltaGameUnits = -1.0f;
            float targetProxyToLiveProxyDeltaGameUnits = -1.0f;
            float targetProxyToLiveProxyDeltaDegrees = -1.0f;
            float targetRelationAToLiveRelationADegrees = -1.0f;
            float targetConstraintAToLiveConstraintADegrees = -1.0f;
            float targetRelationAToTargetConstraintADegrees = -1.0f;
            float liveRelationAToLiveConstraintADegrees = -1.0f;
            float transformARawMaxDelta = -1.0f;
            float transformBRawMaxDelta = -1.0f;
            float targetBRcaRawMaxDelta = -1.0f;
            float relationInverseBodyDeltaGameUnits = -1.0f;
            float relationInverseBodyDeltaDegrees = -1.0f;
            float atomRowsBodyDeltaGameUnits = -1.0f;
            float atomRowsBodyDeltaDegrees = -1.0f;
            float atomRowsToRelationInverseDegrees = -1.0f;
            float targetRowsToProxyInBodyDegrees = -1.0f;
            float solverEffectiveBodyDeltaGameUnits = -1.0f;
            float solverEffectiveBodyDeltaDegrees = -1.0f;
            float solverEffectiveToAtomDegrees = -1.0f;
            float solverEffectiveLiveABodyDeltaDegrees = -1.0f;
            float solverEffectiveTargetALiveADeltaDegrees = -1.0f;
            float ragdollBRcaRowsErrorDegrees = -1.0f;
            float ragdollBRcaColumnsErrorDegrees = -1.0f;
            float ragdollARcbRowsInverseErrorDegrees = -1.0f;
            float ragdollARcbColumnsInverseErrorDegrees = -1.0f;
            float solverCompositionSignedAngleDegrees = 0.0f;
            float solverCompositionSignedAxisX = 0.0f;
            float solverCompositionSignedAxisY = 0.0f;
            float solverCompositionSignedAxisZ = 0.0f;
            float solverCompositionRelationDeterminant = 0.0f;
            float solverCompositionBestDegrees = -1.0f;
            float solverCompositionCurrentDegrees = -1.0f;
            float solverCompositionRowsBDegrees = -1.0f;
            float solverCompositionRowsNeutralDegrees = -1.0f;
            float solverCompositionColsBDegrees = -1.0f;
            float solverCompositionColsNeutralDegrees = -1.0f;
            float solverCompositionInvRowsBDegrees = -1.0f;
            float solverCompositionInvRowsNeutralDegrees = -1.0f;
            float solverCompositionInvColsBDegrees = -1.0f;
            float solverCompositionInvColsNeutralDegrees = -1.0f;
            float solverCompositionForced0Degrees = -1.0f;
            float solverCompositionForced1Degrees = -1.0f;
            float solverCompositionForced0TransformBDeltaDegrees = -1.0f;
            float solverCompositionForced1TransformBDeltaDegrees = -1.0f;
            float solverCompositionColumnDeltaDegrees = -1.0f;
            const char* solverCompositionSignedAxis = "none";
            const char* solverCompositionBestName = "none";
            int ragdollDecompositionConfigMode = -1;
            int ragdollDecompositionMode = -1;
            std::uint64_t traceId = 0;
            std::uint64_t targetWriteSequence = 0;
            std::uint64_t flushSequence = 0;
            bool bodyAWorldValid = false;
            bool ragdollMotorEnabled = false;
            bool valid = false;
        };
        RagdollAngularProbePreSolve _ragdollAngularProbePreSolve{};
        std::array<float, 12> _lastGrabProbeTransformARaw{};
        std::array<float, 12> _lastGrabProbeTransformBRaw{};
        std::array<float, 12> _lastGrabProbeTargetBRcaRaw{};
        std::uint64_t _lastGrabProbeAtomTraceId = 0;
        bool _hasLastGrabProbeAtomBytes = false;
        GeneratedKeyframedBodyDriveState _grabAuthorityProxyDriveState{};
        std::uint64_t _grabAuthorityProxyQueuedSequence = 0;
        std::uint64_t _grabAuthorityProxyFlushSequence = 0;
        std::uint64_t _grabAuthorityProxyFailedFlushes = 0;
        float _grabAuthorityProxyLastFlushDeltaSeconds = 0.0f;
        int _grabAuthorityProxyLogCounter = 0;
        std::uint64_t _grabAuthorityProxyAfterSolveLogCounter = 0;
        std::atomic<bool> _grabAuthorityProxyReleasePending{ false };
        std::mutex _grabAuthorityProxyMutex;
        SavedObjectState _savedObjectState;
        active_grab_body_lifecycle::BodyLifecycleSnapshot _activeGrabLifecycle;
        float _grabStartTime = 0.0f;
        int _heldLogCounter = 0;
        int _notifCounter = 0;

        CanonicalGrabFrame _grabFrame;
        grab_three_phase::AcquisitionPhase _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::Idle;
        grab_three_phase::ObjectGripArea _grabObjectGripAtGrab{};
        held_object_drive_policy::HeldBodySetDriveDecision _heldDriveDecision{};
        held_object_drive_policy::HeldBodySetDriveDecision _pullDriveDecision{};
        bool _heldObjectIsLooseWeapon = false;
        bool _grabFingerPosePublished = false;
        int _grabConvergeStableInsidePocketFrames = 0;
        float _grabConvergePreviousGripErrorGameUnits = std::numeric_limits<float>::max();
        nearby_grab_damping::NearbyGrabDampingState _nearbyGrabDamping;
        float _grabDeviationExceededSeconds = 0.0f;
        std::array<float, 5> _grabDeviationHistory{};
        std::size_t _grabDeviationHistoryCount = 0;
        std::size_t _grabDeviationHistoryNext = 0;
        RE::NiTransform _grabVisualHandTransform{};
        bool _hasGrabVisualHandTransform = false;
        RE::NiTransform _grabVisualHandLerpStartTransform{};
        float _grabVisualHandLerpElapsedSeconds = 0.0f;
        float _grabVisualHandLerpDurationSeconds = 0.0f;
        float _grabVisualDeviationExceededSeconds = 0.0f;
        std::array<float, 5> _grabVisualDeviationHistory{};
        std::size_t _grabVisualDeviationHistoryCount = 0;
        std::size_t _grabVisualDeviationHistoryNext = 0;
        std::array<RE::NiPoint3, 5> _grabFingerProbeStart{};
        std::array<RE::NiPoint3, 5> _grabFingerProbeEnd{};
        bool _hasGrabFingerProbeDebug = false;
        std::array<RE::NiPoint3, 5> _grabFingerPadProbeStart{};
        std::array<RE::NiPoint3, 5> _grabFingerPadProbeEnd{};
        std::array<RE::NiPoint3, 5> _grabFingerPadProbeHit{};
        std::array<std::uint8_t, 5> _grabFingerPadProbeHitValid{};
        bool _hasGrabFingerPadProbeDebug = false;
        std::array<RE::NiPoint3, 5> _grabFingerSurfaceTarget{};
        std::array<std::uint8_t, 5> _grabFingerSurfaceTargetValid{};
        bool _hasGrabFingerSurfaceTargetDebug = false;
        std::array<float, 15> _grabFingerJointPose{};
        std::array<RE::NiTransform, 15> _grabFingerLocalTransforms{};
        std::uint16_t _grabFingerLocalTransformMask = 0;
        grab_finger_pose_runtime::SolvedGrabFingerPose _grabFingerPose{};
        bool _hasGrabFingerJointPose = false;
        bool _hasGrabFingerLocalTransforms = false;
        bool _hasGrabFingerPose = false;
        bool _selectedCloseFingerPoseActive = false;
        RE::NiPoint3 _lastSelectedCloseOrigin{};
        bool _hasLastSelectedCloseOrigin = false;
        float _selectedCloseHandSpeedMetersPerSecond = 0.0f;
        int _grabFingerPoseFrameCounter = 0;
        float _grabFingerPoseAccumulatedDeltaTime = 0.0f;

        static constexpr std::size_t GRAB_RELEASE_VELOCITY_HISTORY = 5;
        std::array<RE::NiPoint3, GRAB_RELEASE_VELOCITY_HISTORY> _heldLocalLinearVelocityHistory{};
        std::size_t _heldLocalLinearVelocityHistoryCount = 0;
        std::size_t _heldLocalLinearVelocityHistoryNext = 0;
        std::array<RE::NiPoint3, GRAB_RELEASE_VELOCITY_HISTORY> _heldLocalHandVelocityHistory{};
        std::array<RE::NiPoint3, GRAB_RELEASE_VELOCITY_HISTORY> _heldHandAngularVelocityHistory{};
        std::size_t _heldHandVelocityHistoryCount = 0;
        std::size_t _heldHandVelocityHistoryNext = 0;
        RE::NiPoint3 _lastHeldObjectLocalLinearVelocityHavok{};
        bool _hasLastHeldObjectLocalLinearVelocityHavok = false;
        RE::NiTransform _previousHeldRawHandWorld{};
        RE::NiPoint3 _previousHeldHandPositionHavok{};
        RE::NiPoint3 _lastHeldHandPositionHavok{};
        bool _hasPreviousHeldRawHandWorld = false;
        bool _hasLastHeldHandPositionHavok = false;
        RE::NiPoint3 _lastPlayerSpaceVelocityHavok{};

        std::vector<std::uint32_t> _heldBodyIds;
        std::vector<std::uint32_t> _pulledBodyIds;
        active_grab_body_lifecycle::BodyLifecycleSnapshot _pullActiveLifecycle;
        RE::hknpWorld* _pullPrepHknpWorld = nullptr;
        RE::NiAVObject* _pullPrepRootNode = nullptr;
        RE::TESObjectREFR* _pullPrepRefr = nullptr;
        grab_target::Kind _pullPrepTargetKind = grab_target::Kind::LooseObject;
        std::uint16_t _pullPrepOriginalMotionPropsId = 1;
        bool _pullPrepRestoreArmed = false;
        std::uint32_t _pulledPrimaryBodyId = INVALID_BODY_ID;
        RE::NiPoint3 _pullPointOffsetHavok{};
        RE::NiPoint3 _pullTargetHavok{};
        float _pullElapsedSeconds = 0.0f;
        float _pullDurationSeconds = 0.0f;
        bool _pullHasTarget = false;
        PullCatchIntent _pullCatchIntent{};
        ActorEquipmentDropHandoff _actorEquipmentDropHandoff{};

        static constexpr int MAX_HELD_BODIES = 64;
        std::uint32_t _heldBodyIdsSnapshot[MAX_HELD_BODIES] = {};
        std::atomic<int> _heldBodyIdsCount{ 0 };

        std::atomic<bool> _isHoldingFlag{ false };

        VatsSelectionHighlight _selectionHighlight;
        SelectionBeamEffect _selectionBeam;

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
            if (selection.targetKind == grab_target::Kind::ActorEquipment) {
                if (playSelectionHighlightCandidate(selection.refr, selection.visualNode, selection_highlight_policy::VatsHighlightSource::VisualNode)) {
                    return true;
                }
                if (selection.hitNode != selection.visualNode &&
                    playSelectionHighlightCandidate(selection.refr, selection.hitNode, selection_highlight_policy::VatsHighlightSource::HitNode)) {
                    return true;
                }
                if (playSelectionHighlightCandidate(selection.refr, referenceRoot, selection_highlight_policy::VatsHighlightSource::ReferenceRoot)) {
                    return true;
                }
                return false;
            }
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
            stopSelectionBeam();
            _selectionHighlight.stop();
        }

    };
}
