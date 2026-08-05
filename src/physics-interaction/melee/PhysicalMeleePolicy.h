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
    };

    struct Decision
    {
        bool accepted{ false };
        DecisionReason reason{ DecisionReason::InvalidBodyId };
        std::uint32_t targetBodyId{ 0x7FFF'FFFF };
        float closingSpeedHavok{ 0.0f };
        float closingSpeedGame{ 0.0f };
        float virtualMass{ 0.0f };
        float sourceSurfaceDamageCoefficient{ 1.0f };
        float nativeDamageMultiplier{ 0.0f };
    };

    [[nodiscard]] Settings sanitizeSettings(Settings settings);
    [[nodiscard]] Decision evaluate(
        const provider::RockProviderExternalContactRecordV1& contact,
        const Settings& settings,
        float gameToHavokScale);
    [[nodiscard]] const char* decisionReasonName(DecisionReason reason);
}
