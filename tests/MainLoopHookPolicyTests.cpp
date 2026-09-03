#include "physics-interaction/core/MainLoopHookPolicy.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace
{
    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }
        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectEqualU64(const char* label, std::uint64_t actual, std::uint64_t expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %llu got %llu\n", label, static_cast<unsigned long long>(expected), static_cast<unsigned long long>(actual));
        return false;
    }

    template <class Enum>
    bool expectDecision(const char* label, Enum actual, Enum expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %d got %d\n", label, static_cast<int>(expected), static_cast<int>(actual));
        return false;
    }

    std::array<std::uint8_t, 5> makeCall(std::int32_t displacement)
    {
        std::array<std::uint8_t, 5> bytes{ 0xE8, 0, 0, 0, 0 };
        std::memcpy(bytes.data() + 1, &displacement, sizeof(displacement));
        return bytes;
    }

    std::array<std::uint8_t, 14> makeThunk(std::uint64_t target)
    {
        std::array<std::uint8_t, 14> bytes{ 0xFF, 0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        std::memcpy(bytes.data() + 6, &target, sizeof(target));
        return bytes;
    }
}

int main()
{
    using namespace rock::main_loop_hook_policy;
    bool ok = true;

    // Relative call decode: forward and backward displacements, and rejection of a non-CALL opcode.
    {
        const std::uintptr_t site = 0x140D8405Eull;
        std::uintptr_t target = 0;
        ok &= expectTrue("forward call decodes", decodeRelativeCallTarget(makeCall(0x1000).data(), site, target));
        ok &= expectEqualU64("forward call target", target, site + 5 + 0x1000);
        ok &= expectTrue("backward call decodes", decodeRelativeCallTarget(makeCall(-0x2000).data(), site, target));
        ok &= expectEqualU64("backward call target", target, site + 5 - 0x2000);
        std::array<std::uint8_t, 5> notCall{ 0xE9, 0, 0, 0, 0 };
        ok &= expectFalse("jmp is not a call", decodeRelativeCallTarget(notCall.data(), site, target));
        ok &= expectEqualU64("rejected call leaves zero", target, 0);
        ok &= expectFalse("null bytes rejected", decodeRelativeCallTarget(nullptr, site, target));
    }

    // CommonLib thunk detection and decode.
    {
        const auto thunk = makeThunk(0x00007FF6ABCDEF00ull);
        ok &= expectTrue("thunk recognized", isCommonLibAbsoluteJumpThunk(thunk.data()));
        std::uintptr_t target = 0;
        ok &= expectTrue("thunk target decodes", decodeCommonLibAbsoluteJumpTarget(thunk.data(), target));
        ok &= expectEqualU64("thunk target value", target, 0x00007FF6ABCDEF00ull);
        auto notThunk = thunk;
        notThunk[2] = 0x10;
        ok &= expectFalse("non-zero displacement is not the CommonLib thunk", isCommonLibAbsoluteJumpThunk(notThunk.data()));
        ok &= expectFalse("non-thunk decode fails", decodeCommonLibAbsoluteJumpTarget(notThunk.data(), target));
        ok &= expectEqualU64("failed decode leaves zero", target, 0);
        const auto zeroThunk = makeThunk(0);
        ok &= expectFalse("zero target rejected", decodeCommonLibAbsoluteJumpTarget(zeroThunk.data(), target));
    }

    // Decision: already installed (direct or through a thunk).
    {
        OuterHookAttemptState state{};
        ok &= expectDecision("immediate outer", decideOuterHook(state, OuterHookProbe{ .readable = true, .immediateIsOuterHook = true }), OuterHookDecision::AlreadyInstalled);
        ok &= expectDecision("terminal outer", decideOuterHook(state, OuterHookProbe{ .readable = true, .terminalIsOuterHook = true }), OuterHookDecision::AlreadyInstalled);
        ok &= expectEqualU64("no attempts consumed", state.attempts, 0);
        ok &= expectFalse("not refused", state.refused);
    }

    // Decision: FRIK owns the site -> install.
    {
        OuterHookAttemptState state{};
        ok &= expectDecision("frik owner installs", decideOuterHook(state, OuterHookProbe{ .readable = true, .terminalOwnedByFrik = true }), OuterHookDecision::Install);
        ok &= expectFalse("install does not refuse", state.refused);
    }

    // Decision: inner hook still owns the site -> retry, bounded.
    {
        OuterHookAttemptState state{};
        const OuterHookProbe innerProbe{ .readable = true, .terminalIsInnerHook = true };
        ok &= expectDecision("first retry", decideOuterHook(state, innerProbe, 3), OuterHookDecision::RetryLater);
        ok &= expectDecision("second retry", decideOuterHook(state, innerProbe, 3), OuterHookDecision::RetryLater);
        ok &= expectDecision("budget exhausted gives up", decideOuterHook(state, innerProbe, 3), OuterHookDecision::GiveUp);
        ok &= expectTrue("give up refuses further work", state.refused);
        ok &= expectDecision("refused stays refused", decideOuterHook(state, OuterHookProbe{ .readable = true, .terminalOwnedByFrik = true }, 3), OuterHookDecision::Refuse);
    }

    // Decision: unreadable site and foreign owner refuse permanently.
    {
        OuterHookAttemptState state{};
        ok &= expectDecision("unreadable refuses", decideOuterHook(state, OuterHookProbe{ .readable = false }), OuterHookDecision::Refuse);
        ok &= expectTrue("unreadable marks refused", state.refused);

        OuterHookAttemptState foreign{};
        ok &= expectDecision("foreign owner refuses", decideOuterHook(foreign, OuterHookProbe{ .readable = true }), OuterHookDecision::Refuse);
        ok &= expectTrue("foreign marks refused", foreign.refused);
    }

    // Scheduler sequence never returns zero.
    {
        ok &= expectEqualU64("sequence increments", nextSchedulerSequence(41), 42);
        ok &= expectEqualU64("sequence skips zero", nextSchedulerSequence(~0ull), 1);
    }

    if (!ok) {
        std::printf("MainLoopHookPolicyTests FAILED\n");
        return 1;
    }
    std::printf("MainLoopHookPolicyTests passed\n");
    return 0;
}
