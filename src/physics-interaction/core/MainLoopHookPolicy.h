#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

/*
 * Pure decode and decision rules for ROCK's two main-loop hooks.
 *
 * FRIK installs its main-loop hook at kGameLoaded, after every plugin's Load,
 * so it displaces ROCK's load-time hook and becomes the outer hook: FRIK's
 * frame runs first, then ROCK's inner hook, then ROCK's frame. FRIK API v2
 * consumes hand world claims in FRIK's next skeleton frame, so ROCK needs one
 * bounded pass before FRIK to rebase active claims by the controller motion
 * since they were published. ROCK gets that pass by re-wrapping the same call
 * site once FRIK owns it. Everything that reads or decides about the site is
 * here so it can be tested without the game.
 */
namespace rock::main_loop_hook_policy
{
    // E8 rel32
    inline constexpr std::size_t kRelativeCallSize = 5;
    // CommonLibF4 5-byte branch stub: FF 25 00000000 + imm64 (jmp [rip])
    inline constexpr std::size_t kCommonLibAbsoluteJumpThunkSize = 14;
    // FRIK hooks in the same kGameLoaded dispatch ROCK first sees, so a few
    // frames normally suffice; the cap only bounds a FRIK that never hooks.
    inline constexpr std::uint32_t kMaxOuterHookAttempts = 900;

    [[nodiscard]] inline bool decodeRelativeCallTarget(
        const std::uint8_t* bytes,
        const std::uintptr_t siteAddress,
        std::uintptr_t& outTarget) noexcept
    {
        outTarget = 0;
        if (!bytes || bytes[0] != 0xE8) {
            return false;
        }
        std::int32_t displacement = 0;
        std::memcpy(&displacement, bytes + 1, sizeof(displacement));
        outTarget = siteAddress + kRelativeCallSize +
            static_cast<std::uintptr_t>(static_cast<std::intptr_t>(displacement));
        return outTarget != 0;
    }

    [[nodiscard]] inline bool isCommonLibAbsoluteJumpThunk(const std::uint8_t* bytes) noexcept
    {
        return bytes &&
               bytes[0] == 0xFF &&
               bytes[1] == 0x25 &&
               bytes[2] == 0x00 &&
               bytes[3] == 0x00 &&
               bytes[4] == 0x00 &&
               bytes[5] == 0x00;
    }

    [[nodiscard]] inline bool decodeCommonLibAbsoluteJumpTarget(
        const std::uint8_t* bytes,
        std::uintptr_t& outTarget) noexcept
    {
        outTarget = 0;
        if (!isCommonLibAbsoluteJumpThunk(bytes)) {
            return false;
        }
        std::uint64_t target = 0;
        std::memcpy(&target, bytes + 6, sizeof(target));
        outTarget = static_cast<std::uintptr_t>(target);
        return outTarget != 0;
    }

    /*
     * What the call site currently resolves to, after following at most one
     * CommonLib thunk. The caller fills the identity flags because only it
     * knows its own function addresses and which module owns the target.
     */
    struct OuterHookProbe
    {
        bool readable = false;
        bool immediateIsOuterHook = false;
        bool terminalIsOuterHook = false;
        bool terminalIsInnerHook = false;
        bool terminalOwnedByFrik = false;
    };

    enum class OuterHookDecision : std::uint8_t
    {
        // The site already points at ROCK's outer hook.
        AlreadyInstalled,
        // FRIK owns the site: write the outer hook now.
        Install,
        // The site still points at ROCK's inner hook: FRIK has not hooked yet.
        RetryLater,
        // Unreadable site or an owner that is neither ROCK nor FRIK.
        Refuse,
        // FRIK never hooked within the attempt budget.
        GiveUp,
    };

    struct OuterHookAttemptState
    {
        std::uint32_t attempts = 0;
        bool refused = false;
    };

    [[nodiscard]] inline OuterHookDecision decideOuterHook(
        OuterHookAttemptState& state,
        const OuterHookProbe& probe,
        const std::uint32_t maxAttempts = kMaxOuterHookAttempts) noexcept
    {
        if (state.refused) {
            return OuterHookDecision::Refuse;
        }
        if (probe.readable && (probe.immediateIsOuterHook || probe.terminalIsOuterHook)) {
            return OuterHookDecision::AlreadyInstalled;
        }
        if (!probe.readable) {
            state.refused = true;
            return OuterHookDecision::Refuse;
        }
        if (probe.terminalIsInnerHook) {
            ++state.attempts;
            if (state.attempts >= maxAttempts) {
                state.refused = true;
                return OuterHookDecision::GiveUp;
            }
            return OuterHookDecision::RetryLater;
        }
        if (!probe.terminalOwnedByFrik) {
            state.refused = true;
            return OuterHookDecision::Refuse;
        }
        return OuterHookDecision::Install;
    }

    /*
     * Monotonic, never-zero scheduler sequence. Zero is reserved for "no
     * pre-FRIK pass has run", so the wraparound skips it.
     */
    [[nodiscard]] inline std::uint64_t nextSchedulerSequence(const std::uint64_t current) noexcept
    {
        const std::uint64_t next = current + 1;
        return next == 0 ? 1 : next;
    }
}
