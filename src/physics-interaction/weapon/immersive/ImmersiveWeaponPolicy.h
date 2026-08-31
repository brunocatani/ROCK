#pragma once

#include <string_view>

namespace rock::immersive_weapon_policy
{
    enum class DetachAuthority
    {
        None,
        IntegratedImmersive,
        ExternalProvider,
    };

    enum class DetachedFiringHandPartGrabSelection
    {
        Standard,
        ExactProviderTarget,
        Reject,
    };

    struct DetachedFiringHandPartGrabInput
    {
        DetachAuthority partCarryAuthority{ DetachAuthority::None };
        bool authoredOnlySupportGrabsEnabled{ false };
        bool exactProviderPartTargetActive{ false };
    };

    struct Config
    {
        // Runtime snapshots fail closed until RockConfig publishes the loaded
        // compiled/production setting for the current frame.
        bool firingGripDetachEnabled{ false };
        bool firingGripDetachPosePreservationEnabled{ false };
        float firingGripReattachRadiusGameUnits{ 3.0f };
        float firingGripHapticDurationSeconds{ 0.10f };
        float firingGripAttachHapticIntensity{ 0.85f };
        float firingGripDetachHapticIntensity{ 0.30f };
    };

    struct ResolveInput
    {
        Config integrated{};
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
        bool preserveWeaponPoseOnDetach{ false };
        float reattachRadiusGameUnits{ 3.0f };
        float gripHapticDurationSeconds{ 0.10f };
        float gripAttachHapticIntensity{ 0.85f };
        float gripDetachHapticIntensity{ 0.30f };
    };

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
        } else if (input.integrated.firingGripDetachEnabled) {
            decision.authority = DetachAuthority::IntegratedImmersive;
            decision.primaryDetachEnabled = true;
            decision.preserveWeaponPoseOnDetach = input.integrated.
                firingGripDetachPosePreservationEnabled;
            decision.reattachRadiusGameUnits = input.integrated.
                firingGripReattachRadiusGameUnits;
            decision.gripHapticDurationSeconds = input.integrated.
                firingGripHapticDurationSeconds;
            decision.gripAttachHapticIntensity = input.integrated.
                firingGripAttachHapticIntensity;
            decision.gripDetachHapticIntensity = input.integrated.
                firingGripDetachHapticIntensity;
        }

        decision.firingGripOwnershipEnabled =
            decision.firingGripOwnershipEnabled ||
            decision.primaryDetachEnabled;
        return decision;
    }

    /*
     * Integrated immersive detach is an authored firing-grip contract.
     * PartCarry tests that firing grip before this policy runs. With ROCK's
     * authored-only switch enabled, the detached firing hand may therefore
     * capture another weapon part only when the current contact has exact
     * provider authority. External detach and mode-off operation retain the
     * established support-grab selector.
     */
    [[nodiscard]] inline constexpr DetachedFiringHandPartGrabSelection
    resolveDetachedFiringHandPartGrab(
        const DetachedFiringHandPartGrabInput& input) noexcept
    {
        if (input.partCarryAuthority !=
                DetachAuthority::IntegratedImmersive ||
            !input.authoredOnlySupportGrabsEnabled) {
            return DetachedFiringHandPartGrabSelection::Standard;
        }

        return input.exactProviderPartTargetActive ?
                   DetachedFiringHandPartGrabSelection::ExactProviderTarget :
                   DetachedFiringHandPartGrabSelection::Reject;
    }

    [[nodiscard]] inline constexpr std::string_view authorityName(
        const DetachAuthority authority) noexcept
    {
        switch (authority) {
        case DetachAuthority::IntegratedImmersive:
            return "integrated-immersive";
        case DetachAuthority::ExternalProvider:
            return "external-provider";
        case DetachAuthority::None:
        default:
            return "none";
        }
    }
}
