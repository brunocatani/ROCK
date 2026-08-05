#pragma once

#include "api/ROCKProviderApi.h"
#include "physics-interaction/melee/PhysicalMeleePolicy.h"

#include <cstdint>

namespace RE
{
    class Actor;
    class BGSBodyPart;
    class bhkWorld;
}

namespace rock::physical_melee
{
    enum class NativeMeleeHitFailure : std::uint32_t
    {
        None = 0,
        BridgeNotInstalled,
        InvalidInput,
        InvalidScale,
        NonFiniteContact,
        MissingTargetCollisionObject,
        MissingTargetBodyPart,
        EvidenceInjectionFailed,
        WeaponWitnessMismatch,
        MissingEquippedWeapon,
        EquippedWeaponMismatch,
        PendingQueueFull,
        HitDataConstructorFailed,
        InitializeWeaponHitDataFailed,
        ActorHitMeFailed,
        HitDataDestructorFailed,
    };

    struct NativeMeleeHitInput
    {
        RE::bhkWorld* bhkWorld{ nullptr };
        RE::Actor* target{ nullptr };
        RE::Actor* aggressor{ nullptr };
        RE::BGSBodyPart* targetBodyPart{ nullptr };
        std::uint32_t targetNativeDamageLimb{ 0xFFFF'FFFFu };
        const provider::RockProviderExternalContactRecordV1* contact{ nullptr };
        WeaponIdentityWitness expectedWeapon{};
        WeaponIdentityWitness currentWeapon{};
        std::uint64_t submissionFrameIndex{ 0 };
        float nativeDamageMultiplier{ 1.0f };
        float closingSpeedGame{ 0.0f };
        float havokToGameScale{ 0.0f };
    };

    struct NativeMeleeHitResult
    {
        bool submitted{ false };
        NativeMeleeHitFailure failure{ NativeMeleeHitFailure::None };
        std::uintptr_t targetCollisionObject{ 0 };
        float contactPointGame[3]{};
        float contactNormal[3]{};
        float relativeVelocityGame[3]{};
        provider::RockProviderImpactOutcomeV1 outcome{};
    };

    [[nodiscard]] bool installNativeMeleeHitBridge();
    void resetNativeMeleeHitBridge();
    [[nodiscard]] std::uint32_t drainNativeMeleeHitOutcomes(
        std::uint64_t currentFrameIndex,
        provider::RockProviderImpactOutcomeV1* outOutcomes,
        std::uint32_t maxOutcomes);
    [[nodiscard]] NativeMeleeHitResult applyNativeMeleeHit(const NativeMeleeHitInput& input);
    [[nodiscard]] const char* nativeMeleeHitFailureName(NativeMeleeHitFailure failure);
}
