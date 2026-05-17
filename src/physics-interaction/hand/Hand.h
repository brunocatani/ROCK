#pragma once

#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabTelemetry.h"
#include "physics-interaction/grab/GrabThreePhase.h"
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
#include <cstdint>
#include <limits>
#include <mutex>
#include <vector>

namespace rock
{

    constexpr std::uint32_t ROCK_HAND_LAYER = 43;

    constexpr std::uint32_t INVALID_BODY_ID = 0x7FFF'FFFF;

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

    struct GrabContactPatchDebugSnapshot
    {
        std::array<RE::NiPoint3, kMaxGrabContactPatchSamples> samplePointsWorld{};
        std::uint32_t sampleCount = 0;
    };

    struct GrabForceTorqueDebugSnapshot
    {
        std::array<RE::NiPoint3, kMaxGrabContactPatchSamples> contactSamplePointsWorld{};
        std::array<RE::NiPoint3, 3> pivotTriangleWorld{};
        RE::hknpBodyId pivotSourceBodyId{ INVALID_BODY_ID };
        RE::NiTransform liveBodyWorld{};
        RE::NiTransform desiredBodyWorld{};
        RE::NiPoint3 targetPivotWorld{};
        RE::NiPoint3 livePivotWorld{};
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
        float captureGripLocalDeltaGameUnits = 0.0f;
        float rotationErrorDegrees = 0.0f;
        float leverLengthGameUnits = 0.0f;
        float correctionLengthGameUnits = 0.0f;
        float torqueWitnessGameUnitsSquared = 0.0f;
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
        bool hasPivotTriangle = false;
        bool hasMeshGripPoint = false;
        bool hasVisualMeshGripPoint = false;
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
        void suppressHandCollisionForGrab(RE::hknpWorld* world);
        void restoreHandCollisionAfterGrab(RE::hknpWorld* world);
        void clearGrabHandCollisionSuppressionState();
        void clearPullRuntimeState(bool restorePreparedObject = true, const char* context = "clear-pull-runtime");
        void clearPullPrepTracking();
        void restorePullPrepIfActive(const char* context);
        bool consumePullPrepLifecycleForActiveGrab(RE::TESObjectREFR* refr, active_grab_body_lifecycle::BodyLifecycleSnapshot& outLifecycle);
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
        bool getGrabContactPatchDebugSnapshot(RE::hknpWorld* world, GrabContactPatchDebugSnapshot& out) const;
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

        bool grabSelectedObject(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, float tau, float damping, float maxForce, float proportionalRecovery,
            float constantRecovery, const GrabSharedObjectContext& sharedContext = {});

        bool acquirePeerHeldCloseSelection(RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            const SavedObjectState& peerSavedObjectState,
            const std::vector<std::uint32_t>& peerHeldBodyIds,
            const RE::NiPoint3& selectionOrigin,
            const RE::NiPoint3& palmNormal,
            float nearRange);

        bool promoteHeldObjectToConstraintDrive(RE::bhkWorld* bhkWorld,
            RE::hknpWorld* world,
            const RE::NiTransform& handWorldTransform,
            float tau,
            float damping,
            float maxForce,
            float proportionalRecovery,
            float constantRecovery,
            const char* reason);

        void updateHeldObject(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, const HeldObjectPlayerSpaceFrame& playerSpaceFrame, float deltaTime,
            float forceFadeInTime, float tauMin, const GrabReleaseContext& releaseContext = {});
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

        void updateSelection(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& selectionOrigin, const RE::NiPoint3& palmNormal,
            const RE::NiPoint3& pointingDirection, const FarSelectionHmdConeGate& farHmdConeGate, float nearRange, float farRange, float deltaTime,
            const OtherHandSelectionContext& otherHandContext);

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
        std::uint32_t getHandColliderBodyCount() const { return _boneColliders.getBodyCount(); }
        std::uint32_t getHandColliderBodyIdAtomic(std::size_t index) const { return _boneColliders.getBodyIdAtomic(index); }
        bool isHandColliderBodyId(std::uint32_t bodyId) const { return _boneColliders.isColliderBodyIdAtomic(bodyId); }
        bool tryGetHandColliderMetadata(std::uint32_t bodyId, HandColliderBodyMetadata& outMetadata) const { return _boneColliders.tryGetBodyMetadataAtomic(bodyId, outMetadata); }
        void recordSemanticContact(const HandColliderBodyMetadata& metadata, std::uint32_t otherBodyId);
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

        bool createCollision(RE::hknpWorld* world, void* bhkWorld);

        void destroyCollision(void* bhkWorld);

        void updateCollisionTransform(RE::hknpWorld* world, float deltaTime);

        void flushPendingCollisionPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        void flushPendingCustomGrabAuthority(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        void observeCustomGrabAuthorityAfterSolve(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        bool beginStashCandidate();
        bool cancelStashCandidate();

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
            const RE::NiTransform& rawHandWorldTransform,
            RE::NiTransform& outDesiredObjectWorld,
            RE::NiTransform& outDesiredBodyWorld,
            RE::NiPoint3& outDesiredTargetPointWorld,
            RE::NiPoint3& outActivePivotBBodyLocalGame);
        RE::NiTransform resolveProxyConstraintAngularDriveTargetWorld(const RE::NiTransform& proxyWorldTransform,
            const RE::NiTransform& desiredObjectWorld,
            const RE::NiTransform& desiredBodyWorld) const;
        bool resolveGrabAuthorityProxyFrame(RE::hknpWorld* world,
            const RE::NiTransform& rawHandWorld,
            const RE::NiTransform* fallbackPalmAnchorWorld,
            RE::NiTransform& outProxyWorld,
            const char*& outSource) const;
        void updateConstraintGrabDriveMotors(RE::hknpWorld* world,
            float deltaTime,
            float forceFadeInTime,
            float tauMin,
            float grabPositionErrorGameUnits,
            float grabRotationErrorDegrees,
            float authorityForceScale,
            bool heldBodyColliding);
        bool applyProxyConstraintAngularVelocityDrive(RE::hknpWorld* world,
            const RE::NiTransform& desiredBodyWorld,
            float deltaTime,
            float& outRawAngularSpeedRadiansPerSecond,
            float& outAppliedAngularSpeedRadiansPerSecond,
            float& outMaxAngularSpeedRadiansPerSecond,
            float& outLongObjectAngularScale,
            std::uint32_t& outAppliedAngularBodyCount,
            RE::NiPoint3& outRawAngularVelocityRadiansPerSecond,
            RE::NiPoint3& outAppliedAngularVelocityRadiansPerSecond);
        void queueProxyGrabAuthorityTarget(const RE::NiTransform& proxyWorldTransform,
            const RE::NiTransform& rawHandWorldTransform,
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
        std::mutex _semanticContactWriteMutex;
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
        float _grabVisualDeviationExceededSeconds = 0.0f;
        std::array<float, 5> _grabVisualDeviationHistory{};
        std::size_t _grabVisualDeviationHistoryCount = 0;
        std::size_t _grabVisualDeviationHistoryNext = 0;
        std::array<RE::NiPoint3, 5> _grabFingerProbeStart{};
        std::array<RE::NiPoint3, 5> _grabFingerProbeEnd{};
        bool _hasGrabFingerProbeDebug = false;
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
            _selectionHighlight.stop();
        }

    };
}
