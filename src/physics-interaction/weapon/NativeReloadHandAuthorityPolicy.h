#pragma once

#include <cstdint>

namespace rock::native_reload_hand_authority_policy
{
    inline constexpr std::uint32_t kReloadingGunState = 4;
    inline constexpr std::uint32_t kFiringGunState = 7;
    inline constexpr std::uint32_t kSightedFiringGunState = 8;
    inline constexpr std::uint32_t kInvalidGunState = 0xFFFF'FFFFu;
    // Some first-person firearm graphs submit their state-4 follow-through
    // well after the recoil sample. Two seconds at the 90 Hz target covers
    // that graph delay; explicit reload and empty-magazine evidence override
    // the exclusion throughout this window.
    inline constexpr std::uint64_t kRecentFireFrameWindow = 180;
    inline constexpr std::uint64_t kReloadRequestFrameWindow = 45;

    struct State
    {
        std::uint32_t previousGunState{ kInvalidGunState };
        std::uint64_t lastFiringTriggerFrame{ 0 };
        std::uint64_t observedReloadDispatchSequence{ 0 };
        std::uint64_t reloadRequestExpiresAtFrame{ 0 };
        bool rawReloadOwnsSupportHand{ false };
    };

    struct Input
    {
        std::uint32_t gunState{ kInvalidGunState };
        std::uint64_t frameIndex{ 0 };
        std::uint64_t reloadDispatchSequence{ 0 };
        bool providerOwnsArmsOrHands{ false };
        bool firingTriggerHeld{ false };
        bool magazineCountKnown{ false };
        bool magazineEmpty{ false };
    };

    [[nodiscard]] inline constexpr bool isFiringState(
        const std::uint32_t gunState) noexcept
    {
        return gunState == kFiringGunState ||
               gunState == kSightedFiringGunState;
    }

    [[nodiscard]] inline constexpr bool frameWithinWindow(
        const std::uint64_t frameIndex,
        const std::uint64_t referenceFrame,
        const std::uint64_t windowFrames) noexcept
    {
        return referenceFrame != 0 &&
               frameIndex >= referenceFrame &&
               frameIndex - referenceFrame <= windowFrames;
    }

    /*
     * FO4VR uses gun state 4 for native reload animation, but a right-hand
     * firearm can also enter the same state directly after a shot. A state-4
     * sample alone is therefore not sufficient authority to remove ROCK's
     * deferred support-hand target.
     *
     * Explicit provider arms/hands authority always wins. For native-only
     * reloads, a routed reload request or an empty magazine confirms reload.
     * State 4 entered without recent firing evidence remains a valid native
     * reload fallback for script-driven and unobserved reload starts.
     */
    [[nodiscard]] inline bool update(
        State& state,
        const Input& input) noexcept
    {
        if (input.firingTriggerHeld) {
            state.lastFiringTriggerFrame = input.frameIndex;
        }

        if (input.reloadDispatchSequence != 0 &&
            input.reloadDispatchSequence !=
                state.observedReloadDispatchSequence) {
            state.observedReloadDispatchSequence =
                input.reloadDispatchSequence;
            state.reloadRequestExpiresAtFrame =
                input.frameIndex + kReloadRequestFrameWindow;
        }

        const bool explicitReloadPending =
            state.reloadRequestExpiresAtFrame != 0 &&
            input.frameIndex <= state.reloadRequestExpiresAtFrame;
        const bool nativeReloadState =
            input.gunState == kReloadingGunState;

        if (!nativeReloadState) {
            state.rawReloadOwnsSupportHand = false;
            if (state.reloadRequestExpiresAtFrame != 0 &&
                input.frameIndex > state.reloadRequestExpiresAtFrame) {
                state.reloadRequestExpiresAtFrame = 0;
            }
        } else if (state.previousGunState != kReloadingGunState) {
            const bool enteredFromFire =
                isFiringState(state.previousGunState) ||
                frameWithinWindow(
                    input.frameIndex,
                    state.lastFiringTriggerFrame,
                    kRecentFireFrameWindow);
            const bool automaticEmptyReload =
                input.magazineCountKnown && input.magazineEmpty;
            state.rawReloadOwnsSupportHand =
                explicitReloadPending ||
                automaticEmptyReload ||
                !enteredFromFire;
            if (explicitReloadPending) {
                state.reloadRequestExpiresAtFrame = 0;
            }
        } else {
            const bool automaticEmptyReload =
                input.magazineCountKnown && input.magazineEmpty;
            if (explicitReloadPending || automaticEmptyReload) {
                // A reload request or final-round transition can arrive while
                // a post-fire state-4 interval is already active. Promote that
                // interval to real reload authority.
                state.rawReloadOwnsSupportHand = true;
                if (explicitReloadPending) {
                    state.reloadRequestExpiresAtFrame = 0;
                }
            }
        }

        state.previousGunState = input.gunState;
        return input.providerOwnsArmsOrHands ||
               state.rawReloadOwnsSupportHand;
    }
}
