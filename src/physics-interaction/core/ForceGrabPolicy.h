#pragma once

#include <cstdint>

namespace rock::force_grab_policy
{
    enum class HandChoice : std::uint8_t
    {
        None,
        Right,
        Left,
    };

    enum class HandBlocker : std::uint32_t
    {
        None = 0,
        Disabled = 1u << 0,
        InteractionState = 1u << 1,
        Holding = 1u << 2,
        PullCatch = 1u << 3,
        ActorEquipmentHandoff = 1u << 4,
        PendingForceGrab = 1u << 5,
        EquippedWeapon = 1u << 6,
        TouchGrab = 1u << 7,
    };

    struct HandAvailabilityInput
    {
        bool disabled{ false };
        bool openInteractionState{ true };
        bool holding{ false };
        bool activePullCatch{ false };
        bool actorEquipmentHandoff{ false };
        bool pendingForceGrab{ false };
        bool equippedWeaponOccupiesHand{ false };
        bool touchGrabActive{ false };
    };

    [[nodiscard]] inline constexpr std::uint32_t blockerMask(const HandAvailabilityInput& input) noexcept
    {
        std::uint32_t result = 0;
        const auto add = [&](bool blocked, HandBlocker blocker) {
            if (blocked) {
                result |= static_cast<std::uint32_t>(blocker);
            }
        };

        add(input.disabled, HandBlocker::Disabled);
        add(!input.openInteractionState, HandBlocker::InteractionState);
        add(input.holding, HandBlocker::Holding);
        add(input.activePullCatch, HandBlocker::PullCatch);
        add(input.actorEquipmentHandoff, HandBlocker::ActorEquipmentHandoff);
        add(input.pendingForceGrab, HandBlocker::PendingForceGrab);
        add(input.equippedWeaponOccupiesHand, HandBlocker::EquippedWeapon);
        add(input.touchGrabActive, HandBlocker::TouchGrab);
        return result;
    }

    [[nodiscard]] inline constexpr bool isAvailable(const HandAvailabilityInput& input) noexcept
    {
        return blockerMask(input) == 0;
    }

    /*
     * Part grips are physical occupancy regardless of role. The firing hand is
     * otherwise occupied while an equipped weapon is present, except during
     * part-carry after that firing grip has detached. Keeping this role-driven
     * makes grenade hand choice mirror correctly when left-hand weapon support
     * becomes authoritative.
     */
    [[nodiscard]] inline constexpr bool equippedWeaponOccupiesHand(
        bool handIsLeft,
        bool equippedWeaponPresent,
        bool partCarryActive,
        bool firingHandIsLeft,
        bool partGripActiveForHand) noexcept
    {
        return partGripActiveForHand ||
               (equippedWeaponPresent && !partCarryActive && handIsLeft == firingHandIsLeft);
    }

    enum class GrenadeSelectionFailure : std::uint8_t
    {
        None,
        GrenadeAlreadyHeld,
        HandsBlocked,
    };

    struct GrenadeHandSelection
    {
        HandChoice hand{ HandChoice::None };
        GrenadeSelectionFailure failure{ GrenadeSelectionFailure::None };
    };

    [[nodiscard]] inline constexpr GrenadeHandSelection selectGrenadeHand(
        bool grenadeAlreadyHeld,
        bool rightAvailable,
        bool leftAvailable) noexcept
    {
        if (grenadeAlreadyHeld) {
            return { .failure = GrenadeSelectionFailure::GrenadeAlreadyHeld };
        }
        if (rightAvailable) {
            return { .hand = HandChoice::Right };
        }
        if (leftAvailable) {
            return { .hand = HandChoice::Left };
        }
        return { .failure = GrenadeSelectionFailure::HandsBlocked };
    }

    [[nodiscard]] inline constexpr GrenadeHandSelection selectGrenadeHand(
        bool grenadeAlreadyHeld,
        const HandAvailabilityInput& right,
        const HandAvailabilityInput& left) noexcept
    {
        return selectGrenadeHand(grenadeAlreadyHeld, isAvailable(right), isAvailable(left));
    }
}
