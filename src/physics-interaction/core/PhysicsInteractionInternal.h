#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

#include "api/ROCKProviderApi.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/collision/ContactPipelinePolicy.h"
#include "physics-interaction/stash/ShoulderStashTransfer.h"
#include "physics-interaction/native/havok/HavokPhysicsTiming.h"
#include "physics-interaction/weapon/WeaponTypes.h"

namespace rock::physics_interaction_detail
{
    inline constexpr std::uint32_t kInvalidAtomicBodyId = 0xFFFF'FFFFu;
    inline constexpr std::uint64_t kInvalidHeldImpactPair = 0xFFFF'FFFF'FFFF'FFFFull;

    struct GrabButtonState
    {
        bool held{ false };
        bool pressed{ false };
        bool released{ false };
        bool syntheticPressed{ false };
    };

    struct TransformDelta
    {
        float position = 0.0f;
        float rotationDegrees = 0.0f;
    };

    [[nodiscard]] constexpr std::uint32_t claimOwnerBit(PhysicsObjectClaimOwner owner)
    {
        return 1u << static_cast<std::uint32_t>(owner);
    }

    [[nodiscard]] constexpr PhysicsObjectClaimOwner claimOwnerForHand(bool isLeft)
    {
        return isLeft ? PhysicsObjectClaimOwner::LeftHand : PhysicsObjectClaimOwner::RightHand;
    }

    [[nodiscard]] std::uint32_t claimOwnerCount(std::uint32_t ownerMask);

    [[nodiscard]] FarSelectionHmdConeGate makeFarSelectionHmdConeGate(
        const PhysicsFrameContext& frame);

    [[nodiscard]] shoulder_stash::DetectorConfig makeShoulderStashDetectorConfig();
    [[nodiscard]] shoulder_stash::DetectorConfig makeEquippedWeaponStashDetectorConfig(
        bool enabled);
    [[nodiscard]] shoulder_stash::Probe makeShoulderStashObjectProbe(
        RE::hknpWorld* world,
        const Hand& hand,
        const HandFrameInput& handInput);
    [[nodiscard]] shoulder_stash::Probe makeShoulderStashHmdProbe(
        const HandFrameInput& handInput);
    [[nodiscard]] bool shouldEmitShoulderStashCandidatePulse(
        const shoulder_stash::Decision& decision,
        shoulder_stash::RuntimeState& state,
        float elapsedSeconds);
    void showShoulderStashCollectedNotification(
        const shoulder_stash::TransferResult& transferResult,
        std::uint32_t fallbackFormID);
    [[nodiscard]] std::string_view shoulderStashItemName(RE::TESBoundObject* baseForm);

    [[nodiscard]] mouth_consume::DetectorConfig makeMouthConsumeDetectorConfig();
    [[nodiscard]] mouth_consume::Probe makeMouthConsumeObjectProbe(
        RE::hknpWorld* world,
        const Hand& hand,
        const HandFrameInput& handInput);
    [[nodiscard]] mouth_consume::Probe makeMouthConsumeHandProbe(
        const HandFrameInput& handInput);

    [[nodiscard]] bool isInvalidGrabBodyId(std::uint32_t bodyId);
    [[nodiscard]] std::uint64_t packHeldImpactPair(
        std::uint32_t heldBodyId,
        std::uint32_t otherBodyId);
    [[nodiscard]] bool unpackHeldImpactPair(
        std::uint64_t packedPair,
        std::uint32_t& heldBodyId,
        std::uint32_t& otherBodyId);
    [[nodiscard]] float readGrabEventBodyMass(
        RE::hknpWorld* world,
        std::uint32_t bodyId);

    [[nodiscard]] GrabButtonState readGrabButtonState(bool isLeft, int buttonId);
    [[nodiscard]] bool readGrabButtonHeld(bool isLeft, int buttonId);
    [[nodiscard]] bool readGrabButtonPressedEdge(bool isLeft, int buttonId);
    [[nodiscard]] bool readHeldWeaponEquipTriggerPressedEdge(bool isLeft);

    [[nodiscard]] TransformDelta measureTransformDelta(
        const RE::NiTransform& a,
        const RE::NiTransform& b);
    [[nodiscard]] float measurePointDelta(
        const RE::NiPoint3& a,
        const RE::NiPoint3& b);

    // Main and physics threads can call this diagnostic. It only reads Hand state.
    void logPalmClockSampleForHand(
        const char* stage,
        const Hand& hand,
        RE::hknpWorld* world,
        const RE::NiTransform* rawHandWorld,
        std::uint64_t gameFrameIndex,
        float gameDeltaSeconds,
        const havok_physics_timing::PhysicsTimingSample* timing);

    [[nodiscard]] RE::TESObjectWEAP* currentEquippedWeaponForm();
    [[nodiscard]] RE::TBO_InstanceData* currentEquippedWeaponInstanceData(
        const RE::TESObjectWEAP* expectedWeapon);
    [[nodiscard]] std::uint32_t currentEquippedWeaponFormId();

    void fillProviderTransform(
        const RE::NiTransform& source,
        ::rock::provider::RockProviderTransform& target);
    [[nodiscard]] RE::NiTransform providerTransformToNi(
        const ::rock::provider::RockProviderTransform& source);
    [[nodiscard]] std::uint32_t providerHandStateFlags(const Hand& hand, bool isLeft);
    void copyProviderString(char* target, std::size_t targetSize, const std::string& source);
    void copyProviderString(char* target, std::size_t targetSize, const char* source);
    [[nodiscard]] ::rock::provider::RockProviderPoint3 makeProviderPoint(
        const WeaponEvidencePoint3& point);
    [[nodiscard]] ::rock::provider::RockProviderPoint3 makeProviderPoint(
        const RE::NiPoint3& point);
    [[nodiscard]] ::rock::provider::RockProviderBodyContactTargetKind
        providerBodyContactTargetKind(contact_pipeline_policy::ContactEndpointKind kind);

    [[nodiscard]] RE::NiAVObject* getEquippedProjectileNode();
    [[nodiscard]] RE::EquippedWeaponData* getValidatedEquippedWeaponData();
    [[nodiscard]] RE::NiNode* resolveEquippedWeaponInteractionNode();

    bool ensureWeaponCollisionWorkbenchExitMenuSinkRegistered();
    void clearEquippedWeaponFiringGripInputState();
}
