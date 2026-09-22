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
        bool authoredSupportSeatAvailable{ false };
    };

    struct Config
    {
        // Runtime snapshots fail closed until RockConfig publishes the loaded
        // compiled/production setting for the current frame.
        bool firingGripDetachEnabled{ false };
        bool firingGripDetachPosePreservationEnabled{ false };
        float firingGripReattachRadiusGameUnits{ 12.0f };
        float firingGripHapticDurationSeconds{ 0.10f };
        float firingGripAttachHapticIntensity{ 0.85f };
        float firingGripDetachHapticIntensity{ 0.30f };
    };

    struct ResolveInput
    {
        Config integrated{};
        bool firingGripOwnershipEnabled{ false };
        bool externalPrimaryDetachEnabled{ false };
        float externalReattachRadiusGameUnits{ 12.0f };
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
        float reattachRadiusGameUnits{ 12.0f };
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
     * PartCarry tests the firing grip first. Either physical hand may then
     * acquire its own validated authored support seat. Other parts still
     * require exact provider authority in integrated authored-only mode.
     */
    [[nodiscard]] inline constexpr DetachedFiringHandPartGrabSelection
    resolveDetachedFiringHandPartGrab(
        const DetachedFiringHandPartGrabInput& input) noexcept
    {
        if (input.partCarryAuthority !=
                DetachAuthority::IntegratedImmersive ||
            !input.authoredOnlySupportGrabsEnabled ||
            input.authoredSupportSeatAvailable) {
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
