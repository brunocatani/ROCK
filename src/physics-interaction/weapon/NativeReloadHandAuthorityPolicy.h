#pragma once

#include <cstdint>

namespace rock::native_reload_hand_authority_policy
{
    inline constexpr std::uint32_t kReloadingGunState = 4;
    inline constexpr std::uint32_t kInvalidGunState = 0xFFFF'FFFFu;
    inline constexpr std::uint64_t kReloadRequestFrameWindow = 45;
    inline constexpr std::uint64_t kReloadAuthorityFrameWindow = 900;

    struct State
    {
        std::uint64_t weaponGenerationKey = 0;
        std::uint64_t observedReloadDispatchSequence = 0;
        std::uint64_t reloadRequestExpiresAtFrame = 0;
        std::uint64_t reloadAuthorityExpiresAtFrame = 0;
        std::uint32_t previousGunState = kInvalidGunState;
        bool rawReloadOwnsSupportHand = false;
    };

    struct Input
    {
        std::uint64_t weaponGenerationKey = 0;
        std::uint64_t frameIndex = 0;
        std::uint64_t reloadDispatchSequence = 0;
        std::uint32_t gunState = kInvalidGunState;
        bool magazineCountKnown = false;
        bool magazineEmpty = false;
    };

    /*
     * Native gun state 4 is necessary reload evidence, but it is not sufficient
     * authority by itself. Some firearm graphs enter the same state after a
     * shot. Explicit ROCK reload dispatch, an empty-magazine transition, or a
     * provider arm/hand lease can establish the corresponding independent
     * authority. This policy owns only the native reload transaction: raw
     * state 4 without positive evidence always fails closed.
     */
    [[nodiscard]] inline bool update(
        State& state,
        const Input& input) noexcept
    {
        if (input.weaponGenerationKey == 0 ||
            state.weaponGenerationKey != input.weaponGenerationKey) {
            const std::uint64_t observedReloadDispatchSequence =
                state.observedReloadDispatchSequence;
            state = {};
            state.weaponGenerationKey = input.weaponGenerationKey;
            state.observedReloadDispatchSequence =
                observedReloadDispatchSequence;
        }

        if (input.weaponGenerationKey == 0) {
            state.rawReloadOwnsSupportHand = false;
            state.previousGunState = input.gunState;
            return false;
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
        const bool automaticEmptyReload =
            input.magazineCountKnown && input.magazineEmpty;
        const bool nativeReloadState =
            input.gunState == kReloadingGunState;

        if (!nativeReloadState) {
            state.rawReloadOwnsSupportHand = false;
            state.reloadAuthorityExpiresAtFrame = 0;
            if (state.reloadRequestExpiresAtFrame != 0 &&
                input.frameIndex > state.reloadRequestExpiresAtFrame) {
                state.reloadRequestExpiresAtFrame = 0;
            }
        } else if (state.previousGunState != kReloadingGunState) {
            state.rawReloadOwnsSupportHand =
                explicitReloadPending ||
                automaticEmptyReload;
            state.reloadAuthorityExpiresAtFrame =
                state.rawReloadOwnsSupportHand ?
                    input.frameIndex + kReloadAuthorityFrameWindow :
                    0;
            if (explicitReloadPending) {
                state.reloadRequestExpiresAtFrame = 0;
            }
        } else if (!state.rawReloadOwnsSupportHand &&
                   (explicitReloadPending || automaticEmptyReload)) {
            state.rawReloadOwnsSupportHand = true;
            state.reloadAuthorityExpiresAtFrame =
                input.frameIndex + kReloadAuthorityFrameWindow;
            if (explicitReloadPending) {
                state.reloadRequestExpiresAtFrame = 0;
            }
        } else if (state.rawReloadOwnsSupportHand &&
                   state.reloadAuthorityExpiresAtFrame != 0 &&
                   input.frameIndex > state.reloadAuthorityExpiresAtFrame) {
            state.rawReloadOwnsSupportHand = false;
            state.reloadAuthorityExpiresAtFrame = 0;
        }

        state.previousGunState = input.gunState;
        return state.rawReloadOwnsSupportHand;
    }
}
