#pragma once

#include "physics-interaction/hand/Hand.h"

#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/native/PhysicsRecursiveWrappers.h"

#include <cstdint>
#include <vector>

namespace rock::hand_grab_detail
{
    [[nodiscard]] const char* bodyMotionTypeName(physics_body_classifier::BodyMotionType motionType);
    [[nodiscard]] const object_physics_body_set::ObjectPhysicsBodyRecord* diagnosticRejectedBodyRecord(
        const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
        std::uint32_t preferredBodyId);
    bool restoreIncompleteActivePrepRoot(
        RE::NiAVObject* rootNode,
        std::uint16_t originalMotionPropertiesId,
        const char* handName,
        const char* context);
    void nativeVRGrabDrop(void* playerCharacter, int handIndex);
    [[nodiscard]] float looseWeaponMultiplier(bool looseWeaponGrab, float multiplier);
    [[nodiscard]] float scaleDriveValue(float value, float multiplier);
    [[nodiscard]] float sharedGrabAuthorityForceScale(bool peerHandStillHolding);
    [[nodiscard]] GrabConstraintMotorTuning buildProxyConstraintMotorTuning(
        float tau,
        float damping,
        float maxForce,
        float authorityForceScale,
        float proportionalRecovery,
        float constantRecovery,
        bool looseWeaponGrab,
        float mass,
        float forceToMassRatio);

    struct HeldBodyActivationSummary
    {
        std::uint32_t bodyCount = 0;
        std::uint32_t activatedCount = 0;
        std::uint32_t failedActivationCount = 0;
    };

    struct HeldBodyFlagLeaseSummary
    {
        std::uint32_t bodyCount = 0;
        std::uint32_t collisionLeaseCount = 0;
        std::uint32_t authorityLeaseCount = 0;
        std::uint32_t failedLeaseCount = 0;
    };

    struct HeldObjectMotionSample
    {
        RE::NiPoint3 primaryLocalLinearVelocity{};
        bool hasPrimaryVelocity = false;
    };

    struct HeldBodyMassSummary
    {
        float primaryMass = 0.0f;
        float aggregateMass = 0.0f;
        std::uint32_t sampledBodies = 0;
        std::uint32_t uniqueMotions = 0;

        [[nodiscard]] float motorMass() const noexcept;
    };

    [[nodiscard]] float effectiveGrabMotorMass(float mass);
    [[nodiscard]] std::uintptr_t heldBodyFlagLeaseOwner(const Hand* hand);
    HeldBodyActivationSummary activateHeldObjectBodySet(
        RE::hknpWorld* world,
        std::uint32_t primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds);
    HeldBodyFlagLeaseSummary acquireHeldObjectBodyFlagLeases(
        RE::hknpWorld* world,
        std::uint32_t primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds,
        std::uintptr_t ownerToken);
    HeldBodyFlagLeaseSummary releaseHeldObjectBodyFlagLeases(
        RE::hknpWorld* world,
        std::uint32_t primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds,
        std::uintptr_t ownerToken,
        bool restoreOnFinalLease);
    [[nodiscard]] std::vector<std::uint32_t> buildCommittedHeldBodyIds(
        std::uint32_t primaryBodyId,
        const std::vector<std::uint32_t>& mechanicalScopeBodyIds,
        const GrabSharedObjectContext& sharedContext,
        bool& adoptedPeerHeldBodyIds);
    [[nodiscard]] physics_recursive_wrappers::MotionPreset motionPresetFromMotionType(
        physics_body_classifier::BodyMotionType motionType,
        std::uint16_t fallbackMotionPropertiesId);
    active_grab_body_lifecycle::BodyLifecycleAudit restoreActiveGrabLifecycle(
        RE::hknpWorld* world,
        const active_grab_body_lifecycle::BodyLifecycleSnapshot& snapshot,
        const active_grab_body_lifecycle::BodyRestorePlan& plan,
        std::uint32_t primaryBodyId,
        const char* handName,
        const char* context);
    [[nodiscard]] HeldObjectMotionSample sampleHeldObjectMotion(
        RE::hknpWorld* world,
        RE::hknpBodyId primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds,
        bool includeConnectedBodies = true);
    void setHeldVelocity(
        RE::hknpWorld* world,
        RE::hknpBodyId primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds,
        const RE::NiPoint3& linearVelocity,
        const RE::NiPoint3& angularVelocity,
        bool overrideAngularVelocity,
        float angularVelocityKeep = 1.0f,
        bool includeConnectedLinearVelocity = true,
        bool includeConnectedAngularVelocity = true);
    void setHeldLinearVelocity(
        RE::hknpWorld* world,
        RE::hknpBodyId primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds,
        const RE::NiPoint3& linearVelocity,
        float angularVelocityKeep = 1.0f,
        bool includeConnectedBodies = true);
    [[nodiscard]] float readBodyMass(RE::hknpWorld* world, RE::hknpBodyId bodyId);
    [[nodiscard]] HeldBodyMassSummary readHeldBodyMassSummary(
        RE::hknpWorld* world,
        RE::hknpBodyId primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds,
        bool includeConnectedBodies = true);
    [[nodiscard]] held_object_contact_policy::HeldContactOtherMotion classifyHeldContactOtherMotion(
        RE::hknpWorld* world,
        std::uint32_t bodyId);
}
