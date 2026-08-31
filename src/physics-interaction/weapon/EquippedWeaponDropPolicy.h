#pragma once

#include <cstdint>

namespace rock::equipped_weapon_drop_policy
{
    enum class SourceHand : std::uint8_t
    {
        None = 0,
        Right,
        Left,
    };

    [[nodiscard]] inline constexpr bool isLeft(SourceHand hand) noexcept
    {
        return hand == SourceHand::Left;
    }

    [[nodiscard]] inline constexpr const char* sourceHandName(SourceHand hand) noexcept
    {
        switch (hand) {
        case SourceHand::Right:
            return "right";
        case SourceHand::Left:
            return "left";
        case SourceHand::None:
            break;
        }
        return "none";
    }

    [[nodiscard]] inline constexpr SourceHand sourceForSupportRelease(
        const bool primaryReleasedThisFrame,
        const bool firingHandIsLeft) noexcept
    {
        const bool sourceHandIsLeft = primaryReleasedThisFrame ?
            firingHandIsLeft :
            !firingHandIsLeft;
        return sourceHandIsLeft ? SourceHand::Left : SourceHand::Right;
    }

    [[nodiscard]] inline constexpr bool equippedWeaponShoulderStashAvailable(
        bool shoulderStashConfigured) noexcept
    {
        return shoulderStashConfigured;
    }

    /*
     * Provider/manual carry resolves to a shoulder source only when one hand
     * currently owns the weapon's drop action. With two active holds no single
     * release drops, so neither hand is a manual carry candidate until one grip
     * lets go. The attached native firing-hand fallback is selected by the
     * coordinator adapter, outside this drop policy.
     */
    [[nodiscard]] inline constexpr SourceHand resolveEquippedWeaponStashCarryHand(
        bool primaryOnlyActive,
        bool partCarryActive,
        bool leftPartGripActive,
        bool rightPartGripActive,
        bool firingHandIsLeft) noexcept
    {
        if (primaryOnlyActive) {
            return firingHandIsLeft ? SourceHand::Left : SourceHand::Right;
        }
        if (partCarryActive && leftPartGripActive != rightPartGripActive) {
            return leftPartGripActive ? SourceHand::Left : SourceHand::Right;
        }
        return SourceHand::None;
    }

    [[nodiscard]] inline constexpr bool shouldAttemptPhysicalDrop(bool stashCommitSelected) noexcept
    {
        // A selected stash is a fail-closed inventory action. If unequip fails,
        // restoring the equipped weapon is safer than converting the same
        // gesture into a destructive world drop.
        return !stashCommitSelected;
    }

    struct PhysicalDropCommitInput
    {
        bool dropSucceeded{ false };
        bool droppedReferenceUnavailable{ false };
    };

    [[nodiscard]] inline constexpr bool physicalDropCommitted(const PhysicalDropCommitInput& input) noexcept
    {
        /*
         * RemoveItem has already transferred the inventory stack once it
         * returns a handle. The handle can be valid before its reference is
         * immediately resolvable, so DroppedReferenceUnavailable is still a
         * committed physical drop for collider-lifecycle cleanup.
         */
        return input.dropSucceeded || input.droppedReferenceUnavailable;
    }

}
