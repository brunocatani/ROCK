#pragma once

#include <cmath>
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

    [[nodiscard]] inline constexpr SourceHand sourceForSupportRelease(bool primaryReleasedThisFrame) noexcept
    {
        return primaryReleasedThisFrame ? SourceHand::Right : SourceHand::Left;
    }

    [[nodiscard]] inline constexpr bool equippedWeaponShoulderStashAvailable(
        bool shoulderStashConfigured) noexcept
    {
        return shoulderStashConfigured;
    }

    struct NativeShoulderSheathInput
    {
        bool handlingEnabled{ false };
        bool primaryDetachEnabled{ false };
        bool weaponAvailable{ false };
        bool menuInputActive{ false };
        bool handDisabled{ false };
        bool handEmpty{ false };
        bool detectorConfirmed{ false };
        bool explicitInputIntent{ false };
    };

    [[nodiscard]] inline constexpr bool resolveShoulderSheathInputIntent(
        const bool toggleGrabEnabled,
        const bool gripPhysicallyPressed,
        const bool gripPhysicallyReleased) noexcept
    {
        // Spatial confirmation only identifies the shoulder; it never supplies
        // user intent. Toggle mode accepts either edge of an explicit tap so a
        // press begun near the dwell boundary can still complete on release.
        // Hold mode retains its ordinary let-go gesture.
        return toggleGrabEnabled ?
            gripPhysicallyPressed || gripPhysicallyReleased :
            gripPhysicallyReleased;
    }

    [[nodiscard]] inline constexpr bool hasToggleShoulderTapIntent(
        const bool toggleGrabEnabled,
        const bool detectorCandidate,
        const bool gripPhysicallyPressed,
        const bool gripPhysicallyReleased) noexcept
    {
        return toggleGrabEnabled &&
               detectorCandidate &&
               (gripPhysicallyPressed || gripPhysicallyReleased);
    }

    struct ShoulderInputGuardState
    {
        bool retrievalGestureActive{ false };
    };

    [[nodiscard]] inline constexpr bool shouldBlockSheathForRetrievalGesture(
        ShoulderInputGuardState& state,
        const bool gripPhysicallyHeld) noexcept
    {
        if (!state.retrievalGestureActive) {
            return false;
        }

        // Consume the entire gesture that drew the weapon. The release frame
        // rearms future sheathing but remains owned by retrieval, so a tap
        // cannot draw and immediately sheathe while still in the back volume.
        if (!gripPhysicallyHeld) {
            state.retrievalGestureActive = false;
        }
        return true;
    }

    [[nodiscard]] inline constexpr bool canCommitNativeShoulderSheath(
        const NativeShoulderSheathInput& input) noexcept
    {
        // This is the ROCK-owned native-attached gesture. Provider-owned
        // realistic detach continues through the manual carry/drop request so
        // the two modes can never interpret the same ordinary release twice.
        return input.handlingEnabled &&
               !input.primaryDetachEnabled &&
               input.weaponAvailable &&
               !input.menuInputActive &&
               !input.handDisabled &&
               input.handEmpty &&
               input.detectorConfirmed &&
               input.explicitInputIntent;
    }

    /*
     * Equipped-weapon shoulder stash only tracks a single carrying hand: the
     * gesture is "carry the weapon over the shoulder and let go", which
     * requires that releasing this hand is what would drop the weapon. With
     * two active holds no single release drops, so no hand is a stash carry
     * candidate until one grip lets go.
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

    struct ShoulderRetrievalInput
    {
        bool stashActive{ false };
        bool handlingEnabled{ false };
        bool identityMatches{ false };
        bool nativePresentationRetrievable{ false };
        bool menuInputActive{ false };
        bool handDisabled{ false };
        bool handEmpty{ false };
        bool handCanOwnFiringGrip{ false };
        bool toggleGrabEnabled{ false };
        bool detectorCandidate{ false };
        bool detectorConfirmed{ false };
        bool sameShoulderZone{ false };
        bool gripPhysicallyHeld{ false };
        bool gripPhysicallyPressed{ false };
        bool gripPhysicallyReleased{ false };
    };

    [[nodiscard]] inline constexpr bool canRetrieveShoulderStashedWeapon(
        const ShoulderRetrievalInput& input) noexcept
    {
        const bool holdOrPullIntent =
            input.detectorConfirmed && input.gripPhysicallyHeld;
        const bool tapIntent = hasToggleShoulderTapIntent(
            input.toggleGrabEnabled,
            input.detectorCandidate,
            input.gripPhysicallyPressed,
            input.gripPhysicallyReleased);
        return input.stashActive &&
               input.handlingEnabled &&
               input.identityMatches &&
               input.nativePresentationRetrievable &&
               !input.menuInputActive &&
               !input.handDisabled &&
               input.handEmpty &&
               input.handCanOwnFiringGrip &&
               input.sameShoulderZone &&
               (holdOrPullIntent || tapIntent);
    }

    struct ShoulderRetrievalCandidate
    {
        bool eligible{ false };
        float confidence{ 0.0f };
    };

    [[nodiscard]] inline SourceHand selectShoulderRetrievalHand(
        const ShoulderRetrievalCandidate& right,
        const ShoulderRetrievalCandidate& left,
        const bool stashedByLeftHand) noexcept
    {
        if (right.eligible != left.eligible) {
            return left.eligible ? SourceHand::Left : SourceHand::Right;
        }
        if (!right.eligible) {
            return SourceHand::None;
        }

        const float rightConfidence = std::isfinite(right.confidence) ? right.confidence : 0.0f;
        const float leftConfidence = std::isfinite(left.confidence) ? left.confidence : 0.0f;
        if (rightConfidence != leftConfidence) {
            return leftConfidence > rightConfidence ? SourceHand::Left : SourceHand::Right;
        }

        // A same-frame tie is rare but must remain deterministic. Favor the
        // hand that physically placed the weapon instead of a fixed side.
        return stashedByLeftHand ? SourceHand::Left : SourceHand::Right;
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
