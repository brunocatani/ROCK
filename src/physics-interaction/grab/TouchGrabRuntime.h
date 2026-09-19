#pragma once

#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/PhysicsCallbackQuiescenceGate.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <cstdint>

namespace RE
{
    class NiCollisionObject;
    class bhkNPCollisionObject;
    class bhkWorld;
    class hknpWorld;
}

namespace rock
{
    class DynamicHandCollisionRuntime;

    /*
     * TouchGrabRuntime is intentionally separate from Hand's loose-object
     * state machine. Provider targets retain first authority, then a selected
     * native PA point and a selected close object, while the optional built-in surface policy supplies only a
     * FixedAnchor wildcard fallback.
     * Mechanisms receive one stock limited joint plus finite hand attachment,
     * while FixedAnchor latches the rendered hand and dynamic hand twins
     * relative to the touched target without changing that target. This keeps
     * ordinary static/keyframed selection rejection unchanged.
     */
    class TouchGrabRuntime
    {
    public:
        enum class TargetClass : std::uint8_t
        {
            Explicit,
            Wildcard,
            Fallback,
        };

        enum class ContactSource : std::uint8_t
        {
            SemanticHand,
            DynamicSurface,
        };

        struct HandReport
        {
            bool globalSurface = false;
            bool commandOwned = false;
            bool hasSurfaceAnchor = false;
            provider::RockProviderTouchGrabKindV1 kind{
                provider::RockProviderTouchGrabKindV1::FixedAnchor
            };
            provider::RockProviderSurfaceGripModeV1 surfaceGripMode{
                provider::RockProviderSurfaceGripModeV1::CollisionAnchor
            };
            std::uint32_t bodyId = 0x7FFF'FFFFu;
            std::uint32_t referenceFormId = 0;
            std::uint32_t referenceNativeHandle = 0;
            RE::NiPoint3 surfaceAnchorGame{};
            bool hasNormal = false;
            RE::NiPoint3 normalGame{};
            provider::RockProviderPowerArmorPointV1 powerArmorPoint{};
            std::uint32_t sourceTriangleIndex = 0xFFFF'FFFFu;
            char meshPartName[64]{};
        };

        struct PowerArmorCandidate
        {
            bool valid = false;
            std::uint32_t referenceFormId = 0;
            std::uint32_t referenceNativeHandle = 0;
            std::uint32_t bodyId = 0x7FFF'FFFFu;
            provider::RockProviderPowerArmorPointV1 point{};
            RE::NiPoint3 positionGame{};
        };
        static constexpr float kPowerArmorProximityRadiusGame = 8.0f;
        struct PowerArmorProbeDiagnostics
        {
            // 0 not attempted, 1 query failed, 2 no PA, 3 no bone,
            // 4 outside radius, 5 eligible point.
            std::uint32_t stage = 0;
            std::uint32_t hits = 0;
            std::uint32_t references = 0;
            std::uint32_t armorReferences = 0;
            std::uint32_t bones = 0;
            float nearestDistanceGame = -1.0f;
        };
        PowerArmorCandidate findPowerArmorCandidate(RE::hknpWorld* world,
            const RE::NiPoint3& handPosition, std::uint32_t frameFormId = 0,
            provider::RockProviderPowerArmorPointV1 point = {},
            float radius = kPowerArmorProximityRadiusGame,
            PowerArmorProbeDiagnostics* diagnostics = nullptr) const;
        bool tryAcquirePowerArmor(bool isLeft, const PowerArmorCandidate& candidate,
            RE::bhkWorld* bhkWorld, RE::hknpWorld* world,
            std::uint32_t worldGeneration, std::uint32_t skeletonGeneration,
            std::uint32_t providerGeneration, std::uint32_t collisionGeneration,
            std::uint64_t commandOwnerToken = 0);

        enum class AttemptFailure : std::uint8_t
        {
            NoCandidates,
            InvalidInput,
            SnapshotUnavailable,
            TargetUnavailable,
            YieldRequested,
            MotionUnsupported,
            ContactKindMismatch,
            TargetClassMismatch,
            WorldMismatch,
            TargetConflict,
            CapacityFull,
            MechanismCreationFailed,
            HandAttachmentFailed,
            CloseObjectPriority,
        };

        struct AttemptReport
        {
            AttemptFailure failure{ AttemptFailure::NoCandidates };
            TargetClass targetClass{ TargetClass::Explicit };
            ContactSource contactSource{ ContactSource::SemanticHand };
            provider::TouchGrabMotionClassV1 motionClass{
                provider::TouchGrabMotionClassV1::Other
            };
            std::uint32_t bodyId = 0x7FFF'FFFFu;
            std::uint32_t collisionLayer = 0xFFFF'FFFFu;
            std::uint32_t motionIndex = 0xFFFF'FFFFu;
            std::uint16_t motionPropertiesId = 0xFFFFu;
            std::uint8_t surfaceLatchFailure = 0;
            std::uint8_t surfaceMeshFailure = 0;
        };

        void setPhysicsCallbackGate(
            PhysicsCallbackQuiescenceGate* gate) noexcept;
        void setDynamicHandCollisionRuntime(
            DynamicHandCollisionRuntime* runtime) noexcept
        {
            _dynamicHandCollision = runtime;
        }
        void setGlobalSurfaceGrabEnabled(const bool enabled) noexcept
        {
            _globalSurfaceGrabEnabled = enabled;
        }

        [[nodiscard]] bool isHandActive(bool isLeft) const noexcept;
        [[nodiscard]] bool getHandReport(
            bool isLeft,
            HandReport& outReport) const noexcept;
        void beginAttemptDiagnostics() noexcept
        {
            _lastAttemptReport = {};
        }
        [[nodiscard]] AttemptReport getAttemptReport() const noexcept
        {
            return _lastAttemptReport;
        }

        [[nodiscard]] bool tryAcquire(
            bool isLeft,
            const hand_semantic_contact_state::SemanticContactRecord& contact,
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            std::uint32_t worldGeneration,
            std::uint32_t skeletonGeneration,
            std::uint32_t providerGeneration,
            std::uint32_t collisionGeneration,
            TargetClass targetClass,
            ContactSource contactSource,
            bool closeObjectCandidate,
            provider::RockProviderPowerArmorPointV1 powerArmorPoint = {});

        void service(
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            float deltaSeconds,
            std::uint32_t worldGeneration,
            std::uint32_t skeletonGeneration,
            std::uint32_t providerGeneration,
            std::uint32_t collisionGeneration);

        void releaseCommandOwner(std::uint64_t ownerToken,
            RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld,
            std::uint32_t collisionGeneration);

        void releaseHand(
            bool isLeft,
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            provider::RockProviderTouchGrabReleaseReasonV1 reason,
            std::uint32_t collisionGeneration);

        void releaseAll(
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            provider::RockProviderTouchGrabReleaseReasonV1 reason,
            std::uint32_t collisionGeneration);

        // World loss means native world teardown owns constraints already.
        // This path releases only ROCK wrapper references and never dereferences
        // the no-longer-authoritative target body.
        void abandonAll(
            provider::RockProviderTouchGrabReleaseReasonV1 reason) noexcept;

    private:
        static constexpr std::uint32_t kInvalidId = 0x7FFF'FFFFu;
        static constexpr std::size_t kMaximumActiveTargets = 2;
        static constexpr std::size_t kHandsPerTarget = 2;

        struct HandAttachment
        {
            bool active = false;
            bool isLeft = false;
            std::uint64_t commandOwnerToken = 0;
            std::uint32_t handBodyId = kInvalidId;
            std::uint32_t constraintId = kInvalidId;
            bool hasContactPoint = false;
            bool hasContactNormal = false;
            bool surfaceLatch = false;
            bool contactRelativeToTarget = false;
            provider::RockProviderSurfaceGripModeV1 surfaceGripMode{
                provider::RockProviderSurfaceGripModeV1::CollisionAnchor
            };
            RE::NiPoint3 contactPointGame{};
            RE::NiPoint3 contactNormalGame{};
            RE::NiPoint3 contactPointInTargetBody{};
            RE::NiPoint3 contactNormalInTargetBody{};
            float shellToMeshDistanceGameUnits = 0.0f;
            provider::RockProviderPowerArmorPointV1 powerArmorPoint{};
            std::uint32_t sourceTriangleIndex = 0xFFFF'FFFFu;
            char meshPartName[64]{};
        };

        struct ActiveTarget
        {
            bool active = false;
            bool globalSurface = false;
            std::uint64_t ownerToken = 0;
            std::uint64_t scopeToken = 0;
            provider::RockProviderTouchGrabTargetV1 target{};
            std::uint32_t bodyId = kInvalidId;
            provider::TouchGrabMotionClassV1 originalMotionClass{
                provider::TouchGrabMotionClassV1::Other
            };
            RE::NiCollisionObject* collisionIdentity = nullptr;
            RE::bhkNPCollisionObject* collisionObject = nullptr;
            BethesdaPhysicsBody anchor{};
            std::uint32_t mechanismConstraintId = kInvalidId;
            RE::NiTransform initialBodyWorld{};
            RE::NiPoint3 axisWorld{};
            RE::NiPoint3 initialHingeWitnessWorld{};
            RE::NiPoint3 hingeWitnessBodyLocal{};
            float initialCoordinate = 0.0f;
            float lastCoordinate = 0.0f;
            std::array<HandAttachment, kHandsPerTarget> hands{};
        };

        [[nodiscard]] ActiveTarget* findTarget(
            std::uint64_t ownerToken,
            std::uint64_t scopeToken,
            std::uint64_t targetId,
            std::uint32_t targetGeneration) noexcept;
        [[nodiscard]] const ActiveTarget* findTargetForHand(
            bool isLeft) const noexcept;
        [[nodiscard]] ActiveTarget* findTargetForHand(
            bool isLeft) noexcept;
        [[nodiscard]] ActiveTarget* firstFreeTarget() noexcept;
        void resetTarget(ActiveTarget& active) noexcept;

        [[nodiscard]] bool attachHand(
            ActiveTarget& active,
            bool isLeft,
            const hand_semantic_contact_state::SemanticContactRecord& contact,
            RE::hknpWorld* world,
            ContactSource contactSource,
            provider::RockProviderPowerArmorPointV1 powerArmorPoint = {});
        [[nodiscard]] bool createMechanism(
            ActiveTarget& active,
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* world);

        void releaseTarget(
            ActiveTarget& active,
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* world,
            provider::RockProviderTouchGrabReleaseReasonV1 reason,
            std::uint32_t collisionGeneration,
            bool restoreTarget);
        void publishState(
            ActiveTarget& active,
            provider::RockProviderTouchGrabPhaseV1 phase,
            provider::RockProviderTouchGrabReleaseReasonV1 reason,
            float coordinateVelocity,
            std::uint32_t collisionGeneration);

        [[nodiscard]] float sampleCoordinate(
            ActiveTarget& active,
            RE::hknpWorld* world) const;
        [[nodiscard]] std::uint32_t activeHandMask(
            const ActiveTarget& active) const noexcept;

        std::array<ActiveTarget, kMaximumActiveTargets> _targets{};
        PhysicsCallbackQuiescenceGate* _physicsCallbackGate = nullptr;
        DynamicHandCollisionRuntime* _dynamicHandCollision = nullptr;
        bool _globalSurfaceGrabEnabled = false;
        RE::bhkWorld* _activeBhkWorld = nullptr;
        RE::hknpWorld* _activeHknpWorld = nullptr;
        std::uint32_t _worldGeneration = 0;
        std::uint32_t _skeletonGeneration = 0;
        std::uint32_t _providerGeneration = 0;
        AttemptReport _lastAttemptReport{};
    };
}
