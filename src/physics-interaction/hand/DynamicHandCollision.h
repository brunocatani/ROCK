#pragma once

#include "api/ProviderRuntimeTypes.h"

#include "physics-interaction/hand/DynamicHandCollisionFeedbackPolicy.h"
#include "physics-interaction/hand/DynamicHandSurfaceContactState.h"
#include "physics-interaction/hand/DynamicHandCollisionTransitionPolicy.h"
#include "physics-interaction/hand/DynamicHandCollisionTelemetry.h"
#include "physics-interaction/hand/DynamicHandTwinTargets.h"
#include "physics-interaction/hand/SurfaceFingerCollisionPolicy.h"
#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/GeneratedKeyframedBodyDrive.h"
#include "physics-interaction/native/HavokCompoundShapeBuilder.h"
#include "physics-interaction/native/HavokPhysicsTiming.h"
#include "physics-interaction/native/HavokPairCollisionFilter.h"
#include "physics-interaction/native/PhysicsCallbackQuiescenceGate.h"

#include "RE/Havok/hknpShape.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>

namespace RE
{
    class NiCollisionObject;
}

namespace rock
{
    class BodyBoneColliderSet;
    class Hand;
    enum class HandState : std::uint8_t;
    struct PhysicsFrameContext;
    struct HandFrameInput;

    /*
     * Dynamic world collision uses one palm-rooted animated compound per hand.
     * Its 17 semantic children are the palm, all 15 finger segments, and one
     * merged ForeArm1->Hand proxy. The current ROCK/FRIK presentation contract
     * remains unchanged: the compound supplies per-child position deviations,
     * and the normal game-frame path publishes the existing combined position
     * correction. No additional FRIK phase or transform convention exists.
     * Finger-segment residuals independently drive bounded anatomical flexion
     * or extension, choosing the direction that best moves all contacted
     * phalanxes toward their solver-safe positions. During ordinary tracking
     * authority is strictly one-directional
     * (wand/skeleton targets -> twins -> render): twin targets come from the
     * same HandBoneColliderSet/BodyBoneColliderSet role-frame publications the
     * keyframed colliders are driven with. A fixed-surface latch captures one
     * solved-pose readback, then drives both twins and rendering from immutable
     * target-local relationships so no render-to-physics feedback loop exists.
     * The twins are not ordinary gameplay contact evidence and collide only
     * with static world-surface layers plus the dedicated rows used by
     * explicitly identified car bodies. Palm/fingertip callbacks publish into
     * a separate bounded channel consumed exclusively by provider-registered
     * fixed-surface grabs.
     *
     * Threading: updateFrame runs on the main game thread; the drive flush runs
     * on the physics step thread and publishes fixed per-body telemetry through
     * atomics that updateFrame consumes one substep later (~1/270 s).
     */
    class DynamicHandCollisionRuntime
    {
    public:
        static constexpr std::size_t kPalmSlot = dynamic_hand_collision_telemetry::kPalmSlot;
        static constexpr std::size_t kFirstForearmSlot = dynamic_hand_collision_telemetry::kFirstForearmSlot;
        static constexpr std::size_t kBodiesPerHand = dynamic_hand_collision_telemetry::kBodiesPerHand;

        void setPhysicsCallbackGate(PhysicsCallbackQuiescenceGate* gate) { _physicsCallbackGate = gate; }

        void updateFrame(const PhysicsFrameContext& frame,
            bool physicsWritesAllowed,
            const Hand& rightHand,
            const Hand& leftHand,
            const BodyBoneColliderSet& bodyBoneColliders,
            bool rightHandWeaponOwned,
            bool leftHandWeaponOwned,
            std::uint32_t dynamicWeaponBodyId,
            bool rightVisualReturnActive,
            bool leftVisualReturnActive);
        void flushPendingPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        void finalizePose(const PhysicsFrameContext& frame, const Hand& rightHand,
            const Hand& leftHand, const BodyBoneColliderSet& bodyBoneColliders);
        /*
         * Post-solve deviation sampling (physics step thread, after-solve
         * phase). Two-stage measurement against the SAME substep's targets:
         * the residual vs the COMMANDED (velocity-limited) target detects
         * contact — an unobstructed hard-keyframe drive lands exactly on it,
         * so tracking motion (hand, locomotion, room scale) produces exactly
         * zero — and only in contact is the render deviation published,
         * measured vs the REQUESTED (pre-limit) target so it equals the true
         * blocked depth. Sampling pre-collide leaks one substep of tracking
         * lag (sessions 1-2 twitch/drag); rendering the commanded residual
         * saturates at the dt-dependent limiter distance (sessions 3-5
         * milli-punch pulsing).
         */
        void samplePostSolveDeviations(
            RE::hknpWorld* world,
            const havok_physics_timing::PhysicsTimingSample& timing);
        void retireAll(void* bhkWorld);
        void reset();
        // Main-thread snapshot/event access. Future provider adapters must copy
        // from here on the main thread rather than retain runtime-owned state.
        [[nodiscard]] bool getTelemetrySnapshot(dynamic_hand_collision_telemetry::Snapshot& outSnapshot) const;
        [[nodiscard]] bool isTransitionCollisionSuppressedAtomic() const
        {
            return _transitionCollisionSuppressedAtomic.load(
                std::memory_order_acquire);
        }
        [[nodiscard]] dynamic_hand_collision_telemetry::HapticEvents consumeHapticEvents();

        struct DynamicBodyContactSource
        {
            bool valid{ false };
            bool isLeft{ false };
            std::uint8_t slot{ 0 };
            std::uint32_t bodyId{
                hand_semantic_contact_state::kInvalidBodyId
            };
        };

        [[nodiscard]] bool tryClassifyDynamicBodyContactSourceAtomic(
            std::uint32_t bodyId,
            std::uint32_t shapeKey,
            DynamicBodyContactSource& outSource) const noexcept;
        void recordDynamicBodyContactCallback(
            const DynamicBodyContactSource& source,
            std::uint32_t otherBodyId,
            bool otherIsHand,
            bool otherIsWeapon) noexcept;

        [[nodiscard]] static bool classifySurfaceContactSource(
            const DynamicBodyContactSource& bodySource,
            dynamic_hand_surface_contact_state::ContactSource& outSource) noexcept;
        void recordSurfaceContactCallback(
            const dynamic_hand_surface_contact_state::ContactSource& source,
            std::uint32_t otherBodyId,
            const hand_semantic_contact_state::SemanticContactVector* contactPointGame,
            const hand_semantic_contact_state::SemanticContactVector* contactNormalGame) noexcept;
        void recordSurfaceManifoldProcessedCallback(
            const dynamic_hand_surface_contact_state::ContactSource& source,
            std::uint32_t otherBodyId,
            bool otherLayerRead,
            std::uint32_t otherLayer,
            const hand_semantic_contact_state::SemanticContactVector* contactPointGame = nullptr,
            const hand_semantic_contact_state::SemanticContactVector* contactNormalGame = nullptr) noexcept;
        [[nodiscard]] hand_semantic_contact_state::SemanticContactCollection collectFreshSurfaceContacts(
            bool isLeft,
            std::uint32_t maximumAgeFrames,
            float maximumAgeSeconds = -1.0f) const noexcept;

        enum class SurfaceLatchFailure : std::uint8_t
        {
            None,
            PrerequisiteUnavailable,
            ContactSourceMismatch,
            HandTransformUnavailable,
            TargetTransformUnavailable,
            SourceProxyUnavailable,
        };

        struct SurfaceLatchPresentation
        {
            RE::NiTransform handWorld{};
            RE::NiPoint3 meshAnchorWorld{};
            RE::NiPoint3 meshNormalWorld{};
            float shellToMeshDistanceGameUnits = 0.0f;
            bool valid = false;
            std::uint32_t animatedReferenceFormId = 0;
            std::uint32_t animatedReferenceNativeHandle = 0;
            provider::RockProviderPowerArmorPointV1 animatedPoint{};
        };

        /*
         * Fixed-surface ownership does not constrain or mutate the target.
         * Instead the hand and all live dynamic twins retain their transforms
         * relative to that target until TouchGrabRuntime ends the latch.
         */
        [[nodiscard]] bool beginSurfaceLatch(
            const dynamic_hand_surface_contact_state::ContactSource& source,
            std::uint32_t targetBodyId,
            RE::hknpWorld* world,
            const SurfaceLatchPresentation* presentation = nullptr,
            SurfaceLatchFailure* outFailure = nullptr);
        void endSurfaceLatch(bool isLeft) noexcept;
        [[nodiscard]] bool isSurfaceLatchActive(bool isLeft) const noexcept;
        [[nodiscard]] bool isSurfaceLatchMeshAuthoritative(
            bool isLeft) const noexcept;
        [[nodiscard]] bool getLastPresentedHandWorld(
            bool isLeft,
            RE::NiTransform& outHandWorld) const noexcept;

        /*
         * Debug-overlay accessor; main thread only (creation/retire happen on
         * the same thread as the overlay publish).
         */
        [[nodiscard]] RE::hknpBodyId proxyBodyIdForDebug(bool isLeft, std::size_t bodyIndex) const
        {
            if (bodyIndex != 0) {
                return RE::hknpBodyId{ 0x7FFF'FFFF };
            }
            const auto& slot = _hands[isLeft ? 1u : 0u].bodies[0];
            return slot.created ? slot.body.getBodyId() : RE::hknpBodyId{ 0x7FFF'FFFF };
        }
        // Main-thread debug publication only. Returns the current compound-root
        // target queued for the next generated-body physics callback.
        [[nodiscard]] bool tryGetBodyTargetForDebug(
            bool isLeft,
            std::size_t bodyIndex,
            RE::NiTransform& outTarget) const;

    private:
        struct PhysicsTelemetrySample
        {
            RE::NiPoint3 requestedTargetWorldGame{};
            RE::NiPoint3 commandedTargetWorldGame{};
            RE::NiPoint3 liveBodyWorldGame{};
            RE::NiPoint3 targetVelocityWorldGameUnitsPerSecond{};
            float approachSpeedGameUnitsPerSecond = 0.0f;
            float physicsDeltaSeconds = 0.0f;
            bool valid = false;
            bool targetVelocityValid = false;
            bool contactActive = false;
            bool worldContactActive = false;
            bool recoveryTeleport = false;
            std::uint64_t sourceSequence = 0;
            std::uint64_t solveSequence = 0;
        };

        /*
         * Physics writes and the main thread reads these fields. Components
         * stay atomic to avoid a C++ data race; the odd/even sequence is a
         * bounded seqlock that prevents accepting a mixed-substep sample.
         */
        struct AtomicPhysicsTelemetry
        {
            std::atomic<std::uint64_t> sequence{ 0 };
            std::atomic<float> requestedX{ 0.0f };
            std::atomic<float> requestedY{ 0.0f };
            std::atomic<float> requestedZ{ 0.0f };
            std::atomic<float> commandedX{ 0.0f };
            std::atomic<float> commandedY{ 0.0f };
            std::atomic<float> commandedZ{ 0.0f };
            std::atomic<float> liveX{ 0.0f };
            std::atomic<float> liveY{ 0.0f };
            std::atomic<float> liveZ{ 0.0f };
            std::atomic<float> targetVelocityX{ 0.0f };
            std::atomic<float> targetVelocityY{ 0.0f };
            std::atomic<float> targetVelocityZ{ 0.0f };
            std::atomic<float> approachSpeed{ 0.0f };
            std::atomic<float> physicsDeltaSeconds{ 0.0f };
            std::atomic<bool> valid{ false };
            std::atomic<bool> targetVelocityValid{ false };
            std::atomic<bool> contactActive{ false };
            std::atomic<bool> worldContactActive{ false };
            std::atomic<bool> recoveryTeleport{ false };
            std::atomic<std::uint64_t> sourceSequence{ 0 };
            std::atomic<std::uint64_t> solveSequence{ 0 };
        };

        struct ProxySlot
        {
            BethesdaPhysicsBody body{};
            RE::hknpShape* shape = nullptr;
            GeneratedKeyframedBodyDriveState driveState{};
            RE::hknpWorld* createdWorld = nullptr;
            void* createdBhkWorld = nullptr;
            std::uint64_t createdGeometryGeneration = 0;
            bool created = false;
            std::atomic<std::uint32_t> bodyIdAtomic{
                hand_semantic_contact_state::kInvalidBodyId
            };
            /*
             * Physics-thread-only handshake between the pre-collide drive and
             * the after-solve deviation sample of the same substep. Two targets
             * with different jobs: the COMMANDED (velocity-limited) target
             * detects contact — an unobstructed drive lands exactly on it, so
             * any residual means the solver blocked the body — while the
             * REQUESTED (pre-limit wand intent) target measures how deep the
             * blocked intent is. Rendering the commanded residual instead
             * saturates the deviation at maxLinearVelocity * driveDt, a value
             * that steps with every substep-count/framerate change: the
             * in-and-out "milli-punch" pulsing of in-game sessions 3-5.
             */
            bool droveThisSubstep = false;
            RE::NiTransform commandedTargetWorld{};
            RE::NiTransform requestedTargetWorld{};
            RE::NiPoint3 commandedTargetGame{};
            RE::NiPoint3 requestedTargetGame{};
            // Physics-thread copy of the last post-solve CONTACT deviation
            // (zero while tracking freely); feeds the next substep's contact
            // press cap (drive must lean on an established contact, not slam
            // the full deficit into it). The hysteresis flag keeps a grazing
            // contact from flapping around the enter threshold.
            RE::NiPoint3 lastPostSolveDeviationGame{};
            bool lastPostSolveDeviationValid = false;
            bool lastPostSolveContact = false;
            RE::NiPoint3 droveTargetVelocityGameUnitsPerSecond{};
            float drovePhysicsDeltaSeconds = 0.0f;
            bool droveTargetVelocityValid = false;
            bool droveRecoveryTeleport = false;
            /*
             * Divergence must PERSIST before a recovery teleport fires
             * (physics thread only). Without the dwell, a hand fighting a wall
             * near the divergence threshold limit-cycles: teleport into the
             * geometry, solver ejection, divergence again — the harsh
             * position-reset stutter of the third in-game session. The dwell
             * resets after each teleport, so it doubles as the re-fire
             * cooldown.
            */
            float divergenceDwellSeconds = 0.0f;
            std::atomic<bool> teleportedAtomic{ false };
            std::atomic<bool> rebuildRequestedAtomic{ false };
            AtomicPhysicsTelemetry physicsTelemetry{};
        };

        struct HandSlots
        {
            struct SurfaceFingerResponse
            {
                std::array<RE::NiPoint3,
                    hand_collider_semantics::kHandFingerRoleCount>
                    baselineCentersInHand{};
                std::array<RE::NiPoint3,
                    hand_collider_semantics::kHandFingerRoleCount>
                    closingProbeTravelInHand{};
                std::array<RE::NiPoint3,
                    hand_collider_semantics::kHandFingerRoleCount>
                    openingProbeTravelInHand{};
                std::array<float, hand_collider_semantics::kHandFingerCount>
                    baselineOpenValues{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
                std::array<float, hand_collider_semantics::kHandFingerCount>
                    currentOpenValues{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
                std::array<std::int8_t,
                    hand_collider_semantics::kHandFingerCount>
                    lastDirections{};
                std::uint32_t lastHelpfulDynamicSlotMask = 0;
                float noContactSeconds = 0.0f;
                bool active = false;
                bool posePublished = false;
            };

            struct SurfaceLatch
            {
                bool active = false;
                std::uint32_t animatedReferenceFormId = 0;
                std::uint32_t animatedReferenceNativeHandle = 0;
                provider::RockProviderPowerArmorPointV1 animatedPoint{};
                RE::NiAVObject* animatedRootIdentity = nullptr;
                std::uint32_t targetBodyId =
                    hand_semantic_contact_state::kInvalidBodyId;
                RE::hknpBody* targetBodyIdentity = nullptr;
                RE::NiCollisionObject* targetCollisionIdentity = nullptr;
                RE::NiTransform handInTargetBody{};
                RE::NiTransform lastHandWorld{};
                std::array<RE::NiTransform, kBodiesPerHand> proxyInTargetBody{};
                std::array<RE::NiTransform, kBodiesPerHand> lastProxyWorld{};
                std::array<bool, kBodiesPerHand> proxyRelationshipValid{};
                RE::NiPoint3 meshAnchorWorld{};
                RE::NiPoint3 meshNormalWorld{};
                RE::NiPoint3 meshAnchorInTargetBody{};
                RE::NiPoint3 meshNormalInTargetBody{};
                float shellToMeshDistanceGameUnits = 0.0f;
                bool meshAuthoritative = false;
                bool posePublished = false;
            };

            std::array<ProxySlot, kBodiesPerHand> bodies{};
            havok_compound_shape_builder::DynamicCompoundShape compoundShape{};
            std::mutex compoundPoseMutex{};
            std::array<havok_compound_shape_builder::ChildTransform,
                kBodiesPerHand> pendingCompoundChildTransforms{};
            std::array<RE::NiTransform, kBodiesPerHand>
                childInCompound{};
            std::array<RE::NiTransform, kBodiesPerHand>
                consumedChildInCompound{};
            std::uint64_t queuedCompoundPoseSequence = 0;
            std::uint64_t consumedCompoundPoseSequence = 0;
            std::uint64_t compoundGeometryGeneration = 0;
            RE::NiPoint3 appliedDeviation{};
            bool visualActive = false;
            // Lines emitted for the current contact episode (debug trace).
            std::uint32_t debugTraceLines = 0;
            // Diagnostics only. Peer ID/kind is one atomic witness, not ownership.
            std::atomic<std::uint64_t> tracePeer{ 0x7FFF'FFFFu };
            std::uint64_t traceQueuedSequence = 0; // game thread
            std::uint64_t poseFrame = 0; // Early decision awaiting final articulation.
            std::uint64_t traceSourceSequence = 0; // physics thread
            std::uint64_t traceSourceJumpCount = 0;
            std::uint64_t traceDivergenceCount = 0;
            std::uint64_t traceCapCount = 0;
            RE::NiTransform lastPresentedHandWorld{};
            bool lastPresentedHandWorldValid = false;
            SurfaceLatch surfaceLatch{};
            SurfaceFingerResponse surfaceFingerResponse{};
            /*
             * Post-teleport visual recovery: while this window is open the
             * render-side filter uses a slow eased glide instead of the snappy
             * contact smoothing, so a divergence recovery reads as the hand
             * smoothly rejoining the controller instead of a position snap.
             */
            float teleportRecoverySecondsRemaining = 0.0f;
            bool physicsContactActive = false;
            std::atomic<std::uint64_t> contactEntrySequenceAtomic{ 0 };
            std::atomic<float> contactEntryApproachSpeedAtomic{ 0.0f };
            std::atomic<std::uint32_t> contactEntryMaskAtomic{ 0 };
            std::atomic<std::uint32_t> pendingOtherHandContactMaskAtomic{ 0 };
            std::atomic<std::uint32_t> pendingWeaponContactMaskAtomic{ 0 };
            std::atomic<std::uint32_t> pendingSolverContactMaskAtomic{ 0 };
            std::atomic<std::uint32_t> pendingWorldContactMaskAtomic{ 0 };
            std::uint32_t retainedSolverContactMask = 0;
            std::uint32_t retainedWorldContactMask = 0;
            std::array<float, kBodiesPerHand>
                solverContactRetentionSeconds{};
            std::array<float, kBodiesPerHand>
                worldContactRetentionSeconds{};
            std::uint32_t otherHandContactMask{ 0 };
            std::uint32_t weaponContactMask{ 0 };
            std::uint8_t otherHandContactGraceFrames{ 0 };
            std::uint8_t weaponContactGraceFrames{ 0 };
            dynamic_hand_collision_feedback::ContactPulseState hapticState{};
        };

        bool ensureHandCreated(HandSlots& handSlots,
            bool isLeft,
            const PhysicsFrameContext& frame,
            const Hand& hand,
            const BodyBoneColliderSet& bodyBoneColliders,
            const std::array<const dynamic_hand_twin::TwinSlotFrame*,
                kBodiesPerHand>& twinFrames,
            const RE::NiTransform& compoundRootTarget,
            const std::array<RE::NiTransform, kBodiesPerHand>& driveTargets,
            std::uint64_t geometryGeneration);
        [[nodiscard]] bool queueCompoundPose(
            HandSlots& handSlots,
            const RE::NiTransform& compoundRootTarget,
            const std::array<RE::NiTransform, kBodiesPerHand>& driveTargets);
        void retireSlot(ProxySlot& slot, void* bhkWorld);
        void retireHand(HandSlots& handSlots, void* bhkWorld, bool isLeft);
        void clearVisual(HandSlots& handSlots, bool isLeft);
        void clearSurfaceFingerResponse(HandSlots& handSlots, bool isLeft);
        void applySurfaceLatchPose(HandSlots& handSlots, bool isLeft);
        void clearSurfaceLatchPose(HandSlots& handSlots, bool isLeft);
        [[nodiscard]] bool captureSurfaceFingerResponse(
            HandSlots& handSlots,
            bool isLeft,
            HandState handState,
            const RE::NiTransform& rawHandWorld,
            const dynamic_hand_twin::TwinTargets& handTwins);
        [[nodiscard]] std::uint32_t updateSurfaceFingerResponse(
            HandSlots& handSlots,
            bool isLeft,
            HandState handState,
            const RE::NiTransform& rawHandWorld,
            const dynamic_hand_twin::TwinTargets& handTwins,
            const dynamic_hand_collision_telemetry::HandSample& handTelemetry,
            float deltaSeconds);
        void applyWeaponOwnershipCollisionSuppression(
            RE::hknpWorld* world,
            bool rightHandWeaponOwned,
            bool leftHandWeaponOwned);
        void applyTransitionCollisionSuppression(RE::hknpWorld* world, bool suppressCollision);
        static void publishPhysicsTelemetry(ProxySlot& slot, const PhysicsTelemetrySample& sample);
        [[nodiscard]] static bool readPhysicsTelemetry(const ProxySlot& slot, PhysicsTelemetrySample& outSample, std::uint64_t& outSequence);
        static void clearPhysicsContactState(ProxySlot& slot);
        void updateHandHaptic(
            HandSlots& handSlots,
            dynamic_hand_collision_telemetry::HandSample& handTelemetry,
            bool authorityAllowsFeedback,
            float deltaSeconds);

        std::array<HandSlots, 2> _hands{};
        dynamic_hand_surface_contact_state::State _surfaceContacts{};
        std::atomic<std::uint64_t> _surfaceImpulsePairSequenceAtomic{ 0 };
        std::atomic<std::uint64_t> _surfaceProcessedPairSequenceAtomic{ 0 };
        std::atomic<std::uint64_t> _surfaceEligiblePairSequenceAtomic{ 0 };
        std::atomic<std::uint64_t> _surfaceContactPublishSequenceAtomic{ 0 };
        dynamic_hand_collision_telemetry::Snapshot _telemetrySnapshot{};
        dynamic_hand_collision_telemetry::HapticEvents _pendingHapticEvents{};
        std::uint64_t _telemetryUpdateSequence = 0;
        std::uint32_t _logCounter = 0;
        PhysicsCallbackQuiescenceGate* _physicsCallbackGate = nullptr;
        dynamic_hand_collision_transition::State _transitionState{};
        bool _transitionCollisionSuppressed = false;
        std::atomic<bool> _transitionCollisionSuppressedAtomic{ false };
        std::array<bool, 2> _weaponOwnershipCollisionSuppressed{};
        std::atomic<std::uint32_t> _desiredWeaponBodyIdAtomic{
            hand_semantic_contact_state::kInvalidBodyId
        };
        std::array<std::atomic<bool>, 2> _weaponOwnedAtomic{};
        std::atomic<bool> _pairFilterReadyAtomic{ false };
        std::array<std::atomic<std::uint32_t>, 2>
            _suppressedWeaponPairCountAtomic{};
        HavokPairCollisionLeaseSet _weaponPairLeases{};
    };
}
