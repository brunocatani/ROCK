#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::held_weapon_instant_transition_policy
{
    constexpr std::size_t kMaximumActionTraceEntries = 8;

    enum class NativeAction : std::uint8_t
    {
        Sheathe,
        Draw,
    };

    enum class HookDecision : std::uint8_t
    {
        PassThroughInactive,
        SuppressSheathe,
        SuppressDraw,
        PassThroughPlayerMismatch,
        PassThroughUnexpectedCaller,
    };

    struct ActionTrace
    {
        std::array<NativeAction, kMaximumActionTraceEntries> actions{};
        std::uint8_t count{ 0 };
        bool playerMismatch{ false };
        bool unexpectedCaller{ false };
        bool nestedScope{ false };
        bool overflow{ false };
    };

    [[nodiscard]] inline constexpr HookDecision classifyHookCall(
        const bool scopeActive,
        const bool playerMatches,
        const bool draw,
        const std::uintptr_t returnAddress,
        const std::uintptr_t drawReturnAddress,
        const std::uintptr_t sheatheReturnAddress) noexcept
    {
        if (!scopeActive) {
            return HookDecision::PassThroughInactive;
        }
        if (!playerMatches) {
            return HookDecision::PassThroughPlayerMismatch;
        }
        if (draw && returnAddress == drawReturnAddress) {
            return HookDecision::SuppressDraw;
        }
        if (!draw && returnAddress == sheatheReturnAddress) {
            return HookDecision::SuppressSheathe;
        }
        return HookDecision::PassThroughUnexpectedCaller;
    }

    inline constexpr void recordAction(ActionTrace& trace, const NativeAction action) noexcept
    {
        if (trace.count >= trace.actions.size()) {
            trace.overflow = true;
            return;
        }
        trace.actions[trace.count++] = action;
    }

    [[nodiscard]] inline constexpr std::uint8_t actionCount(
        const ActionTrace& trace,
        const NativeAction action) noexcept
    {
        std::uint8_t count = 0;
        for (std::uint8_t index = 0; index < trace.count; ++index) {
            if (trace.actions[index] == action) {
                ++count;
            }
        }
        return count;
    }

    [[nodiscard]] inline constexpr bool isValidCompletionTrace(const ActionTrace& trace) noexcept
    {
        if (trace.playerMismatch || trace.unexpectedCaller || trace.nestedScope ||
            trace.overflow || trace.count == 0 ||
            trace.count > trace.actions.size()) {
            return false;
        }

        if (trace.actions[trace.count - 1] != NativeAction::Draw ||
            actionCount(trace, NativeAction::Draw) != 1) {
            return false;
        }

        for (std::uint8_t index = 0; index + 1 < trace.count; ++index) {
            if (trace.actions[index] != NativeAction::Sheathe) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] inline constexpr bool isStableWeaponState(
        const std::uint32_t nativeWeaponState) noexcept
    {
        return nativeWeaponState == 0 || nativeWeaponState == 3;
    }
}
