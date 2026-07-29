#pragma once

#include "physics-interaction/weapon/HeldWeaponEquipStatePolicy.h"

#include <cstdint>

namespace rock::equipped_weapon_transition_policy
{
    constexpr std::uint8_t kStableFramesBeforeNativeHandoff = 3;
    constexpr std::uint8_t kMissingFramesBeforeRepair = 2;
    constexpr std::uint8_t kAttachSettleFrames = 6;
    constexpr std::uint8_t kMaximumLocalVisibilityAttempts = 2;
    constexpr std::uint8_t kMaximumAttachAttempts = 2;
    constexpr float kDrawRetryIntervalSeconds = 0.10f;
    constexpr float kWantToDrawStallSeconds = 0.50f;
    constexpr float kDrawRecoveryDeadlineSeconds = 1.00f;

    enum class RepairAction : std::uint8_t
    {
        None,
        RequestDraw,
        DrawExhausted,
        RestoreLocalVisibility,
        QueueNativeAttach,
        Exhausted,
    };

    struct State
    {
        std::uint8_t stableFrames{ 0 };
        std::uint8_t missingFrames{ 0 };
        std::uint8_t attachSettleFramesRemaining{ 0 };
        std::uint8_t localVisibilityAttempts{ 0 };
        std::uint8_t attachAttempts{ 0 };
        float drawRecoveryWindowStartedAtSeconds{ 0.0f };
        float nextDrawRequestAtSeconds{ 0.0f };
        std::uint32_t drawRequests{ 0 };
        bool drawRecoveryWindowActive{ false };
        bool wantToDrawObserved{ false };
        bool drawRecoveryExhausted{ false };
        bool nativeHandoffObserved{ false };
    };

    struct FrameInput
    {
        float drawRecoveryElapsedSeconds{ 0.0f };
        bool mutationAllowed{ false };
        bool identityMatches{ false };
        bool weaponExactlyDrawn{ false };
        std::uint32_t nativeWeaponState{ 0 };
        bool bridgeModelAvailable{ false };
        bool nativeInstanceFound{ false };
        bool nativeAncestorPathVisible{ false };
        bool nativeInstanceLocallyVisible{ false };
        bool bridgeOwnsNativeInstanceCull{ false };
    };

    struct Decision
    {
        bool presentBridgeModel{ false };
        bool handoffBridgeToNative{ false };
        RepairAction repair{ RepairAction::None };
    };

    [[nodiscard]] inline constexpr bool matchesExpectedIdentity(
        const std::uint32_t currentFormID,
        const std::uintptr_t currentInstanceData,
        const std::uint32_t expectedFormID,
        const std::uintptr_t expectedInstanceData,
        const std::uint32_t previousFormID,
        const std::uintptr_t previousInstanceData) noexcept
    {
        if (currentFormID == 0 || currentFormID != expectedFormID) {
            return false;
        }
        if (expectedInstanceData == 0 ||
            currentInstanceData == expectedInstanceData) {
            return true;
        }

        // Some native equip paths clone the instance-data payload instead of
        // publishing the exact inventory-stack pointer submitted to
        // EquipObject. Accept that clone only after the current identity has
        // moved away from the pre-request weapon. This prevents a same-base
        // equip from accidentally binding to the old stack.
        return currentFormID != previousFormID ||
               currentInstanceData != previousInstanceData;
    }

    [[nodiscard]] inline constexpr Decision advance(State& state, const FrameInput& input) noexcept
    {
        Decision decision{};

        if (!input.identityMatches) {
            state.stableFrames = 0;
            state.missingFrames = 0;
            state.drawRecoveryWindowStartedAtSeconds = 0.0f;
            state.nextDrawRequestAtSeconds = 0.0f;
            state.drawRecoveryWindowActive = false;
            state.wantToDrawObserved = false;
            state.drawRecoveryExhausted = false;
            return decision;
        }

        if (!input.weaponExactlyDrawn) {
            state.stableFrames = 0;
            state.missingFrames = 0;
            // Before the first native handoff, WantToDraw/Drawing may leave a
            // real gap which the bridge must cover. After a completed handoff,
            // a later non-drawn state belongs to a new engine/menu transition;
            // never resurrect the completed equip's loose model over it.
            decision.presentBridgeModel =
                input.bridgeModelAvailable && !state.nativeHandoffObserved;

            if (state.nativeHandoffObserved || !input.mutationAllowed) {
                return decision;
            }

            using NativeWeaponState = held_weapon_equip_state_policy::NativeWeaponState;
            const auto nativeState = static_cast<NativeWeaponState>(input.nativeWeaponState);
            if (!held_weapon_equip_state_policy::isValidNativeWeaponState(
                    input.nativeWeaponState)) {
                state.drawRecoveryExhausted = true;
                decision.repair = RepairAction::DrawExhausted;
                return decision;
            }

            // Drawing is an explicit native acknowledgment. Do not age it
            // against the request window: the asynchronous animation owns
            // progress until it reaches Drawn or returns to a retryable state.
            if (nativeState == NativeWeaponState::Drawing) {
                state.drawRecoveryWindowStartedAtSeconds = 0.0f;
                state.nextDrawRequestAtSeconds =
                    input.drawRecoveryElapsedSeconds;
                state.drawRecoveryWindowActive = false;
                state.wantToDrawObserved = false;
                state.drawRecoveryExhausted = false;
                return decision;
            }

            if (nativeState == NativeWeaponState::WantToDraw) {
                if (!state.wantToDrawObserved) {
                    state.drawRecoveryWindowStartedAtSeconds =
                        input.drawRecoveryElapsedSeconds;
                    state.nextDrawRequestAtSeconds =
                        input.drawRecoveryElapsedSeconds +
                        kWantToDrawStallSeconds;
                    state.drawRecoveryWindowActive = true;
                    state.wantToDrawObserved = true;
                    state.drawRecoveryExhausted = false;
                    return decision;
                }

                const float wantToDrawElapsedSeconds =
                    input.drawRecoveryElapsedSeconds -
                    state.drawRecoveryWindowStartedAtSeconds;
                if (state.drawRecoveryExhausted ||
                    wantToDrawElapsedSeconds >=
                        kDrawRecoveryDeadlineSeconds) {
                    state.drawRecoveryExhausted = true;
                    decision.repair = RepairAction::DrawExhausted;
                    return decision;
                }
                if (input.drawRecoveryElapsedSeconds <
                    state.nextDrawRequestAtSeconds) {
                    return decision;
                }

                ++state.drawRequests;
                state.nextDrawRequestAtSeconds =
                    input.drawRecoveryElapsedSeconds +
                    kWantToDrawStallSeconds;
                decision.repair = RepairAction::RequestDraw;
                return decision;
            }

            const bool returnedFromWantToDraw =
                state.wantToDrawObserved;
            state.wantToDrawObserved = false;
            if (!held_weapon_equip_state_policy::shouldSubmitDrawFollowup(
                    input.nativeWeaponState)) {
                return decision;
            }
            if (returnedFromWantToDraw ||
                !state.drawRecoveryWindowActive) {
                state.drawRecoveryWindowStartedAtSeconds =
                    input.drawRecoveryElapsedSeconds;
                state.nextDrawRequestAtSeconds =
                    input.drawRecoveryElapsedSeconds;
                state.drawRecoveryWindowActive = true;
                state.drawRecoveryExhausted = false;
            }

            const float unacknowledgedSeconds =
                input.drawRecoveryElapsedSeconds -
                state.drawRecoveryWindowStartedAtSeconds;
            if (state.drawRecoveryExhausted ||
                unacknowledgedSeconds >= kDrawRecoveryDeadlineSeconds) {
                state.drawRecoveryExhausted = true;
                decision.repair = RepairAction::DrawExhausted;
                return decision;
            }
            if (input.drawRecoveryElapsedSeconds <
                state.nextDrawRequestAtSeconds) {
                return decision;
            }

            // DrawWeaponMagicHands(true) is a void submission. Requests are
            // deliberately not counted as progress; only a native state
            // advance to WantToDraw, Drawing, or Drawn resets this window.
            ++state.drawRequests;
            state.nextDrawRequestAtSeconds =
                input.drawRecoveryElapsedSeconds +
                kDrawRetryIntervalSeconds;
            decision.repair = RepairAction::RequestDraw;
            return decision;
        }

        state.drawRecoveryWindowStartedAtSeconds = 0.0f;
        state.nextDrawRequestAtSeconds = 0.0f;
        state.drawRecoveryWindowActive = false;
        state.wantToDrawObserved = false;
        state.drawRecoveryExhausted = false;

        const bool exactInstanceRenderable =
            input.nativeInstanceFound &&
            input.nativeAncestorPathVisible &&
            (input.nativeInstanceLocallyVisible || input.bridgeOwnsNativeInstanceCull);
        if (exactInstanceRenderable) {
            state.missingFrames = 0;
            if (state.stableFrames < kStableFramesBeforeNativeHandoff) {
                ++state.stableFrames;
            }
            if (state.attachSettleFramesRemaining > 0) {
                --state.attachSettleFramesRemaining;
            }

            if (state.stableFrames >= kStableFramesBeforeNativeHandoff) {
                state.nativeHandoffObserved = true;
                decision.handoffBridgeToNative = input.bridgeModelAvailable;
            } else {
                decision.presentBridgeModel = input.bridgeModelAvailable;
            }
            return decision;
        }

        state.stableFrames = 0;
        if (state.missingFrames < kMissingFramesBeforeRepair) {
            ++state.missingFrames;
        }
        if (state.attachSettleFramesRemaining > 0) {
            --state.attachSettleFramesRemaining;
        }
        decision.presentBridgeModel = input.bridgeModelAvailable;

        if (!input.mutationAllowed || state.missingFrames < kMissingFramesBeforeRepair ||
            state.attachSettleFramesRemaining > 0) {
            return decision;
        }

        if (input.nativeInstanceFound &&
            state.localVisibilityAttempts < kMaximumLocalVisibilityAttempts) {
            ++state.localVisibilityAttempts;
            decision.repair = RepairAction::RestoreLocalVisibility;
            state.attachSettleFramesRemaining = 1;
            return decision;
        }

        if (state.attachAttempts < kMaximumAttachAttempts) {
            ++state.attachAttempts;
            state.attachSettleFramesRemaining = kAttachSettleFrames;
            decision.repair = RepairAction::QueueNativeAttach;
            return decision;
        }

        decision.repair = RepairAction::Exhausted;
        return decision;
    }
}
