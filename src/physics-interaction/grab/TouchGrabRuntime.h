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
    /*
     * TouchGrabRuntime is intentionally separate from Hand's loose-object
     * state machine. Only provider-registered targets enter this path:
     * mechanisms receive one stock limited joint plus finite hand attachment,
     * while FixedAnchor records touch ownership without changing the target.
     * This keeps ordinary static/keyframed selection rejection unchanged.
     */
    class TouchGrabRuntime
    {
    public:
        enum class TargetClass : std::uint8_t
        {
            Explicit,
            Wildcard,
        };

        void setPhysicsCallbackGate(
            PhysicsCallbackQuiescenceGate* gate) noexcept;

        [[nodiscard]] bool isHandActive(bool isLeft) const noexcept;

        [[nodiscard]] bool tryAcquire(
            bool isLeft,
            const hand_semantic_contact_state::SemanticContactRecord& contact,
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            std::uint32_t worldGeneration,
            std::uint32_t skeletonGeneration,
            std::uint32_t providerGeneration,
            std::uint32_t collisionGeneration,
            TargetClass targetClass);

        void service(
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            float deltaSeconds,
            std::uint32_t worldGeneration,
            std::uint32_t skeletonGeneration,
            std::uint32_t providerGeneration,
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
            std::uint32_t handBodyId = kInvalidId;
            std::uint32_t constraintId = kInvalidId;
            bool hasContactPoint = false;
            bool hasContactNormal = false;
            RE::NiPoint3 contactPointGame{};
            RE::NiPoint3 contactNormalGame{};
        };

        struct ActiveTarget
        {
            bool active = false;
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
            RE::hknpWorld* world);
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
        RE::bhkWorld* _activeBhkWorld = nullptr;
        RE::hknpWorld* _activeHknpWorld = nullptr;
        std::uint32_t _worldGeneration = 0;
        std::uint32_t _skeletonGeneration = 0;
        std::uint32_t _providerGeneration = 0;
    };
}
