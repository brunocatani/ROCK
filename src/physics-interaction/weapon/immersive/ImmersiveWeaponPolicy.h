#pragma once

#include <string_view>

namespace rock::immersive_weapon_policy
{
    enum class DetachAuthority
    {
        None,
        IntegratedPhysicalRight,
        ExternalProvider,
    };

    struct Config
    {
        // Runtime snapshots fail closed until RockConfig publishes the loaded
        // compiled/production setting for the current frame.
        bool physicalRightFiringGripDetachEnabled{ false };
        float physicalRightFiringGripReattachRadiusGameUnits{ 3.0f };
        float physicalRightFiringGripHapticDurationSeconds{ 0.10f };
        float physicalRightFiringGripAttachHapticIntensity{ 0.85f };
        float physicalRightFiringGripDetachHapticIntensity{ 0.30f };
    };

    struct ResolveInput
    {
        Config integrated{};
        bool firingHandIsLeft{ false };
        bool firingGripOwnershipEnabled{ false };
        bool externalPrimaryDetachEnabled{ false };
        float externalReattachRadiusGameUnits{ 3.0f };
        float externalGripHapticDurationSeconds{ 0.10f };
        float externalGripAttachHapticIntensity{ 0.85f };
        float externalGripDetachHapticIntensity{ 0.30f };
    };

    struct Decision
    {
        DetachAuthority authority{ DetachAuthority::None };
        bool firingGripOwnershipEnabled{ false };
        bool primaryDetachEnabled{ false };
        float reattachRadiusGameUnits{ 3.0f };
        float gripHapticDurationSeconds{ 0.10f };
        float gripAttachHapticIntensity{ 0.85f };
        float gripDetachHapticIntensity{ 0.30f };
    };

    [[nodiscard]] inline constexpr bool appliesToPhysicalHand(
        const bool enabled,
        const bool handIsLeft) noexcept
    {
        return enabled && !handIsLeft;
    }

    [[nodiscard]] inline constexpr bool appliesToPhysicalHand(
        const Config& config,
        const bool handIsLeft) noexcept
    {
        return appliesToPhysicalHand(
            config.physicalRightFiringGripDetachEnabled,
            handIsLeft);
    }

    [[nodiscard]] inline constexpr Decision resolve(
        const ResolveInput& input) noexcept
    {
        Decision decision{};
        decision.firingGripOwnershipEnabled =
            input.firingGripOwnershipEnabled;

        if (input.externalPrimaryDetachEnabled) {
            decision.authority = DetachAuthority::ExternalProvider;
            decision.primaryDetachEnabled = true;
            decision.reattachRadiusGameUnits =
                input.externalReattachRadiusGameUnits;
            decision.gripHapticDurationSeconds =
                input.externalGripHapticDurationSeconds;
            decision.gripAttachHapticIntensity =
                input.externalGripAttachHapticIntensity;
            decision.gripDetachHapticIntensity =
                input.externalGripDetachHapticIntensity;
        } else if (appliesToPhysicalHand(
                       input.integrated,
                       input.firingHandIsLeft)) {
            decision.authority = DetachAuthority::IntegratedPhysicalRight;
            decision.primaryDetachEnabled = true;
            decision.reattachRadiusGameUnits = input.integrated.
                physicalRightFiringGripReattachRadiusGameUnits;
            decision.gripHapticDurationSeconds = input.integrated.
                physicalRightFiringGripHapticDurationSeconds;
            decision.gripAttachHapticIntensity = input.integrated.
                physicalRightFiringGripAttachHapticIntensity;
            decision.gripDetachHapticIntensity = input.integrated.
                physicalRightFiringGripDetachHapticIntensity;
        }

        decision.firingGripOwnershipEnabled =
            decision.firingGripOwnershipEnabled ||
            decision.primaryDetachEnabled;
        return decision;
    }

    [[nodiscard]] inline constexpr std::string_view authorityName(
        const DetachAuthority authority) noexcept
    {
        switch (authority) {
        case DetachAuthority::IntegratedPhysicalRight:
            return "integrated-physical-right";
        case DetachAuthority::ExternalProvider:
            return "external-provider";
        case DetachAuthority::None:
        default:
            return "none";
        }
    }
}
