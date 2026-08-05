#pragma once

#include "api/ROCKProviderApi.h"

#include <cstdint>

namespace rock::physical_melee
{
    struct Settings
    {
        bool enabled{ true };
        float minSourceSpeedGame{ 400.0f };
        float virtualWeaponMass{ 12.0f };
        float damageMultiplier{ 1.0f };
        float maxNativeDamageMultiplier{ 10.0f };
        float sourceTargetCooldownSeconds{ 0.20f };
        std::uint32_t maxDamageEventsPerFrame{ 32 };
    };

    enum class DecisionReason : std::uint32_t
    {
        Accepted = 0,
        Disabled,
        InvalidBodyId,
        UnsupportedSource,
        ContactPointNotVerified,
        MissingRawManifold,
        MissingRelativeVelocity,
        MissingWeaponIdentity,
        MissingSurfaceClassification,
        MissingTargetAnatomy,
        AmbiguousTargetAnatomy,
        ContinuedEpisode,
        SourceTooSlow,
        NonFiniteInput,
        InvalidScale,
        CooldownActive,
        FrameBudgetExceeded,
        MissingTargetActor,
        NativeSubmissionFailed,
        CollisionUnavailable,
        TransitionSuppressed,
        NonDamagingSource,
        DamageMultiplierZero,
        RuntimeUnavailable,
        WeaponWitnessMismatch,
        SupersededCandidate,
        RegisteredTargetMismatch,
    };

    struct WeaponIdentityWitness
    {
        std::uint64_t bodyGenerationKey{ 0 };
        std::uint64_t identityKey{ 0 };
        std::uint64_t ownershipKey{ 0 };
        std::uintptr_t instanceDataAddress{ 0 };
        std::uint32_t weaponFormId{ 0 };
        std::uint32_t collisionGeneration{ 0 };
        std::uint32_t equipIndex{ 0 };
    };

    struct ImpactCandidateScore
    {
        float nativeDamageMultiplier{ 0.0f };
        float positiveImpulseSum{ 0.0f };
        std::uint32_t surfaceConfidencePermille{ 0 };
        std::uint32_t sourceBodyId{ 0 };
        std::uint32_t descriptorIndex{ 0 };
    };

    struct Decision
    {
        bool accepted{ false };
        DecisionReason reason{ DecisionReason::InvalidBodyId };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        float closingSpeedHavok{ 0.0f };
        float closingSpeedGame{ 0.0f };
        float tangentSpeedHavok{ 0.0f };
        float relativePointSpeedHavok{ 0.0f };
        float relativePointSpeedGame{ 0.0f };
        float virtualMass{ 0.0f };
        float sourceSurfaceDamageCoefficient{ 1.0f };
        float nativeDamageMultiplier{ 0.0f };
    };

    [[nodiscard]] Settings sanitizeSettings(Settings settings);
    [[nodiscard]] bool weaponWitnessMatches(
        const WeaponIdentityWitness& expected,
        const WeaponIdentityWitness& current) noexcept;
    [[nodiscard]] bool isBetterImpactCandidate(
        const ImpactCandidateScore& candidate,
        const ImpactCandidateScore& incumbent) noexcept;
    [[nodiscard]] Decision evaluate(
        const provider::RockProviderExternalContactRecordV1& contact,
        const Settings& settings,
        float gameToHavokScale);
    [[nodiscard]] const char* decisionReasonName(DecisionReason reason);
}
