#pragma once

#include "physics-interaction/weapon/HeldWeaponEquipStatePolicy.h"

#include <algorithm>
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
        bool gripReleased{ false };
    };

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
               input.gripReleased;
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
        bool detectorConfirmed{ false };
        bool sameShoulderZone{ false };
        bool gripPhysicallyHeld{ false };
    };

    [[nodiscard]] inline constexpr bool canRetrieveShoulderStashedWeapon(
        const ShoulderRetrievalInput& input) noexcept
    {
        return input.stashActive &&
               input.handlingEnabled &&
               input.identityMatches &&
               input.nativePresentationRetrievable &&
               !input.menuInputActive &&
               !input.handDisabled &&
               input.handEmpty &&
               input.handCanOwnFiringGrip &&
               input.detectorConfirmed &&
               input.sameShoulderZone &&
               input.gripPhysicallyHeld;
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

    inline constexpr float kShoulderDrawRetryIntervalSeconds = 0.10f;
    inline constexpr float kShoulderDrawAcknowledgementDeadlineSeconds = 1.00f;

    enum class ShoulderDrawAction : std::uint8_t
    {
        Wait = 0,
        SubmitDraw,
        CommitRetrieval,
        RestartGesture,
    };

    struct ShoulderDrawState
    {
        float elapsedSeconds{ 0.0f };
        float nextRequestAtSeconds{ 0.0f };
        std::uint32_t requestCount{ 0 };
    };

    inline constexpr void beginShoulderDrawWait(
        ShoulderDrawState& state) noexcept
    {
        state = ShoulderDrawState{
            .nextRequestAtSeconds = kShoulderDrawRetryIntervalSeconds,
            .requestCount = 1,
        };
    }

    [[nodiscard]] inline constexpr bool nativeStateAcknowledgesShoulderDraw(
        const std::uint32_t nativeState) noexcept
    {
        using NativeWeaponState =
            held_weapon_equip_state_policy::NativeWeaponState;
        switch (static_cast<NativeWeaponState>(nativeState)) {
        case NativeWeaponState::WantToDraw:
        case NativeWeaponState::Drawing:
        case NativeWeaponState::Drawn:
            return true;
        default:
            return false;
        }
    }

    [[nodiscard]] inline ShoulderDrawAction advanceShoulderDrawWait(
        ShoulderDrawState& state,
        const std::uint32_t nativeState,
        const float deltaSeconds) noexcept
    {
        if (!held_weapon_equip_state_policy::isValidNativeWeaponState(
                nativeState)) {
            return ShoulderDrawAction::RestartGesture;
        }
        if (nativeStateAcknowledgesShoulderDraw(nativeState)) {
            return ShoulderDrawAction::CommitRetrieval;
        }

        const float elapsed =
            std::isfinite(deltaSeconds) && deltaSeconds > 0.0f ?
            std::clamp(deltaSeconds, 0.0f, 0.1f) :
            0.0f;
        state.elapsedSeconds += elapsed;
        if (state.elapsedSeconds >=
            kShoulderDrawAcknowledgementDeadlineSeconds) {
            return ShoulderDrawAction::RestartGesture;
        }
        if (state.elapsedSeconds < state.nextRequestAtSeconds) {
            return ShoulderDrawAction::Wait;
        }

        state.nextRequestAtSeconds =
            state.elapsedSeconds + kShoulderDrawRetryIntervalSeconds;
        ++state.requestCount;
        return ShoulderDrawAction::SubmitDraw;
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
