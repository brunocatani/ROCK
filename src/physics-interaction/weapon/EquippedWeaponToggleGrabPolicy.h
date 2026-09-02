#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::equipped_weapon_toggle_grab_policy
{
    enum class HandState : std::uint8_t
    {
        Open,
        Latched,
        ReleasePending,
        BlockedUntilRelease,
    };

    struct ButtonState
    {
        bool held{ false };
        bool pressed{ false };
        bool released{ false };
    };

    struct GripOccupancy
    {
        bool left{ false };
        bool right{ false };
    };

    struct RuntimeState
    {
        std::uint64_t weaponOwnershipKey{ 0 };
        // ROCK uses index 0 for the right hand and index 1 for the left hand.
        std::array<HandState, 2> hands{
            HandState::Open,
            HandState::Open,
        };
    };

    struct Input
    {
        bool enabled{ false };
        bool inputAllowed{ false };
        std::uint64_t weaponOwnershipKey{ 0 };
        GripOccupancy occupancy{};
        ButtonState left{};
        ButtonState right{};
    };

    struct Decision
    {
        ButtonState left{};
        ButtonState right{};
        bool leftReleasePressConsumed{ false };
        bool rightReleasePressConsumed{ false };
    };

    struct GripReleaseRetention
    {
        // The weapon state machine refused this hand's logical release this
        // update because the hand is the weapon's last carrier and the
        // last-grip drop is disabled. The release press is spent.
        bool left{ false };
        bool right{ false };
    };

    struct ReconcileDecision
    {
        // The caller must drain the acquisition press for each newly engaged
        // hand. Otherwise the still-pending edge can look like the next press.
        bool leftGripAcquired{ false };
        bool rightGripAcquired{ false };
    };

    [[nodiscard]] inline constexpr std::size_t handIndex(const bool isLeft) noexcept
    {
        return isLeft ? 1u : 0u;
    }

    inline constexpr void reset(RuntimeState& state) noexcept
    {
        state = {};
    }

    namespace detail
    {
        struct HandDecision
        {
            ButtonState button{};
            bool releasePressConsumed{ false };
        };

        [[nodiscard]] inline constexpr HandDecision prepareHand(
            HandState& state,
            const ButtonState& physical,
            const bool occupied,
            const bool inputAllowed,
            const bool synchronizeOccupancy) noexcept
        {
            if (synchronizeOccupancy) {
                if (occupied && state == HandState::Open) {
                    // A programmatic or held-button acquisition becomes a
                    // normal latched grip on the next input evaluation.
                    state = HandState::Latched;
                } else if (!occupied && state == HandState::Latched) {
                    state = HandState::Open;
                } else if (!occupied && state == HandState::ReleasePending) {
                    state = HandState::BlockedUntilRelease;
                }
            }

            switch (state) {
            case HandState::Open:
                return inputAllowed ? HandDecision{ .button = physical } : HandDecision{};

            case HandState::Latched:
                if (inputAllowed && physical.pressed) {
                    state = HandState::ReleasePending;
                    return HandDecision{
                        .button = ButtonState{
                            .held = false,
                            .pressed = false,
                            .released = true,
                        },
                        .releasePressConsumed = true,
                    };
                }
                return HandDecision{
                    .button = ButtonState{
                        .held = true,
                        .pressed = false,
                        .released = false,
                    },
                };

            case HandState::ReleasePending:
                // Keep the logical grip open until the existing primary-grip
                // release debounce and the weapon state machine finish. The
                // release edge is one-shot; only the open state persists.
                return HandDecision{
                    .button = ButtonState{
                        .held = false,
                        .pressed = false,
                        .released = false,
                    },
                };

            case HandState::BlockedUntilRelease:
                if (!physical.held) {
                    state = HandState::Open;
                }
                // Do not pass the release edge or the tail of the release
                // press into a new weapon or world grab.
                return {};
            }

            return {};
        }

        [[nodiscard]] inline constexpr bool reconcileHand(
            HandState& state,
            const bool occupied,
            const bool releaseRetained) noexcept
        {
            if (occupied) {
                if (state == HandState::Open) {
                    state = HandState::Latched;
                    return true;
                } else if (state == HandState::BlockedUntilRelease) {
                    state = HandState::ReleasePending;
                } else if (state == HandState::ReleasePending &&
                           releaseRetained) {
                    // A refused last-carrier release re-latches in place: the
                    // next press is again a release request, never a
                    // re-acquisition, and the pending open state cannot leak
                    // into a later detach or reattach decision.
                    state = HandState::Latched;
                }
                return false;
            }

            if (state == HandState::ReleasePending) {
                state = HandState::BlockedUntilRelease;
            } else if (state == HandState::Latched) {
                state = HandState::Open;
            }
            return false;
        }
    }

    [[nodiscard]] inline constexpr Decision prepare(
        RuntimeState& state,
        const Input& input) noexcept
    {
        if (!input.enabled || input.weaponOwnershipKey == 0) {
            reset(state);
            return Decision{
                .left = input.left,
                .right = input.right,
            };
        }

        const bool identityChanged =
            state.weaponOwnershipKey != input.weaponOwnershipKey;
        if (identityChanged) {
            reset(state);
            state.weaponOwnershipKey = input.weaponOwnershipKey;
        }

        auto left = detail::prepareHand(
            state.hands[handIndex(true)],
            input.left,
            input.occupancy.left,
            input.inputAllowed,
            !identityChanged);
        auto right = detail::prepareHand(
            state.hands[handIndex(false)],
            input.right,
            input.occupancy.right,
            input.inputAllowed,
            !identityChanged);
        return Decision{
            .left = left.button,
            .right = right.button,
            .leftReleasePressConsumed = left.releasePressConsumed,
            .rightReleasePressConsumed = right.releasePressConsumed,
        };
    }

    [[nodiscard]] inline constexpr ReconcileDecision reconcile(
        RuntimeState& state,
        const bool enabled,
        const std::uint64_t weaponOwnershipKey,
        const GripOccupancy& occupancy,
        const GripReleaseRetention& releaseRetained) noexcept
    {
        if (!enabled || weaponOwnershipKey == 0 ||
            state.weaponOwnershipKey != weaponOwnershipKey) {
            reset(state);
            return {};
        }

        return ReconcileDecision{
            .leftGripAcquired = detail::reconcileHand(
                state.hands[handIndex(true)],
                occupancy.left,
                releaseRetained.left),
            .rightGripAcquired = detail::reconcileHand(
                state.hands[handIndex(false)],
                occupancy.right,
                releaseRetained.right),
        };
    }
}
