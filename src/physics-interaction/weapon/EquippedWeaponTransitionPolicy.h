#pragma once

#include "physics-interaction/weapon/HeldWeaponEquipStatePolicy.h"

#include <cstdint>

namespace rock::equipped_weapon_transition_policy
{
    /*
     * Renderability confirmation uses consecutive observations. Draw,
     * visibility, and native-attach recovery use elapsed seconds because the
     * engine can publish many game frames before its queued equipment work has
     * settled. A frame-count repair budget can otherwise be consumed in a few
     * milliseconds at VR frame rates.
     */
    constexpr std::uint8_t kStableFramesBeforeNativeHandoff = 3;
    constexpr std::uint8_t kMissingFramesBeforeRepair = 2;
    constexpr std::uint8_t kMaximumLocalVisibilityAttempts = 3;
    constexpr std::uint8_t kMaximumAttachAttempts = 3;
    constexpr float kDrawRetryIntervalSeconds = 0.50f;
    constexpr float kWantToDrawStallSeconds = 1.00f;
    constexpr float kDrawRecoveryDeadlineSeconds = 3.00f;
    constexpr float kPartialDrawCompletionDeadlineSeconds = 1.00f;
    constexpr float kPresentationRecoveryGraceSeconds = 0.35f;
    constexpr float kLocalVisibilitySettleSeconds = 0.15f;
    constexpr float kNativeAttachSettleSeconds = 0.35f;
    constexpr float kPresentationRecoveryDeadlineSeconds = 2.50f;

    enum class RepairAction : std::uint8_t
    {
        None,
        RequestDraw,
        RequestPreparedDraw,
        FinalizePartialDraw,
        DrawExhausted,
        RestoreLocalVisibility,
        QueueNativeAttach,
        Exhausted,
    };

    struct State
    {
        std::uint8_t stableFrames{ 0 };
        std::uint8_t missingFrames{ 0 };
        std::uint8_t localVisibilityAttempts{ 0 };
        std::uint8_t attachAttempts{ 0 };
        float drawRecoveryWindowStartedAtSeconds{ 0.0f };
        float nextDrawRequestAtSeconds{ 0.0f };
        float partialDrawRecoveryStartedAtSeconds{ 0.0f };
        float presentationRecoveryWindowStartedAtSeconds{ 0.0f };
        float nextPresentationRepairAtSeconds{ 0.0f };
        std::uint32_t drawRequests{ 0 };
        bool drawRecoveryWindowActive{ false };
        bool partialDrawRecoveryActive{ false };
        bool presentationRecoveryWindowActive{ false };
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

    inline constexpr void resetDrawRecoveryWindow(State& state) noexcept
    {
        state.drawRecoveryWindowStartedAtSeconds = 0.0f;
        state.nextDrawRequestAtSeconds = 0.0f;
        state.drawRecoveryWindowActive = false;
        state.wantToDrawObserved = false;
        state.drawRecoveryExhausted = false;
    }

    inline constexpr void resetPresentationRecoveryWindow(State& state) noexcept
    {
        state.presentationRecoveryWindowStartedAtSeconds = 0.0f;
        state.nextPresentationRepairAtSeconds = 0.0f;
        state.localVisibilityAttempts = 0;
        state.attachAttempts = 0;
        state.presentationRecoveryWindowActive = false;
    }

    inline constexpr void resetPartialDrawRecovery(State& state) noexcept
    {
        state.partialDrawRecoveryStartedAtSeconds = 0.0f;
        state.partialDrawRecoveryActive = false;
    }

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

    struct FiringHandReservation
    {
        bool pending{ false };
        bool isLeft{ false };
        std::uint32_t formID{ 0 };
        std::uintptr_t instanceData{ 0 };
        std::uint32_t previousFormID{ 0 };
        std::uintptr_t previousInstanceData{ 0 };
    };

    [[nodiscard]] inline constexpr bool resolveFiringHandIsLeft(
        bool currentFiringHandIsLeft, std::uint32_t currentFormID,
        std::uintptr_t currentInstanceData, const FiringHandReservation& reservation) noexcept
    {
        // Native draw can finish before the requested physical grip is ready.
        // Reserve its hand only for the accepted weapon identity; an expired
        // or replaced transfer must not override the live firing role.
        return reservation.pending && matchesExpectedIdentity(currentFormID, currentInstanceData,
                   reservation.formID, reservation.instanceData,
                   reservation.previousFormID, reservation.previousInstanceData) ?
            reservation.isLeft : currentFiringHandIsLeft;
    }

    [[nodiscard]] inline constexpr Decision advance(State& state, const FrameInput& input) noexcept
    {
        Decision decision{};

        if (!input.identityMatches) {
            state.stableFrames = 0;
            state.missingFrames = 0;
            state.drawRequests = 0;
            resetDrawRecoveryWindow(state);
            resetPartialDrawRecovery(state);
            resetPresentationRecoveryWindow(state);
            return decision;
        }

        if (!input.weaponExactlyDrawn) {
            state.stableFrames = 0;
            state.missingFrames = 0;
            resetPresentationRecoveryWindow(state);
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

            if (nativeState != NativeWeaponState::Drawing) {
                resetPartialDrawRecovery(state);
            }

            // Ordinary Drawing is an explicit native acknowledgment and owns
            // its progress. A partial-action recovery is different: ROCK
            // supplied Drawing after verified clip activation, so it retains
            // a bounded completion deadline until the graph reaches Drawn.
            if (nativeState == NativeWeaponState::Drawing) {
                if (!state.partialDrawRecoveryActive) {
                    resetDrawRecoveryWindow(state);
                    return decision;
                }

                state.drawRecoveryWindowStartedAtSeconds = 0.0f;
                state.nextDrawRequestAtSeconds = 0.0f;
                state.drawRecoveryWindowActive = false;
                state.wantToDrawObserved = false;
                const float partialRecoveryElapsedSeconds =
                    input.drawRecoveryElapsedSeconds -
                    state.partialDrawRecoveryStartedAtSeconds;
                if (state.drawRecoveryExhausted) {
                    decision.repair = RepairAction::DrawExhausted;
                } else if (partialRecoveryElapsedSeconds >=
                           kPartialDrawCompletionDeadlineSeconds) {
                    decision.repair = RepairAction::FinalizePartialDraw;
                }
                return decision;
            }

            /*
             * A menu weapon swap first completes the old weapon's holster.
             * FO4VR can reject ActionDraw while that transition owns the graph.
             * Wait for stable Sheathed, then repeat the engine's equip-draw
             * preparation before requesting the new weapon.
             */
            if (nativeState == NativeWeaponState::WantToSheathe ||
                nativeState == NativeWeaponState::Sheathing) {
                resetDrawRecoveryWindow(state);
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
            if (nativeState != NativeWeaponState::Sheathed) {
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

            // The coordinator repeats FO4VR's equip-draw preparation before
            // this request. Only native state acknowledgment counts as progress.
            ++state.drawRequests;
            state.nextDrawRequestAtSeconds =
                input.drawRecoveryElapsedSeconds +
                kDrawRetryIntervalSeconds;
            decision.repair = RepairAction::RequestPreparedDraw;
            return decision;
        }

        resetDrawRecoveryWindow(state);
        resetPartialDrawRecovery(state);

        const bool exactInstanceRenderable =
            input.nativeInstanceFound &&
            input.nativeAncestorPathVisible &&
            (input.nativeInstanceLocallyVisible || input.bridgeOwnsNativeInstanceCull);
        if (exactInstanceRenderable) {
            state.missingFrames = 0;
            resetPresentationRecoveryWindow(state);
            if (state.stableFrames < kStableFramesBeforeNativeHandoff) {
                ++state.stableFrames;
            }

            if (state.stableFrames >= kStableFramesBeforeNativeHandoff) {
                if (!state.nativeHandoffObserved) {
                    state.nativeHandoffObserved = true;
                    decision.handoffBridgeToNative = input.bridgeModelAvailable;
                }
            } else {
                decision.presentBridgeModel =
                    input.bridgeModelAvailable && !state.nativeHandoffObserved;
            }
            return decision;
        }

        state.stableFrames = 0;
        if (state.missingFrames < kMissingFramesBeforeRepair) {
            ++state.missingFrames;
        }
        // Native recovery remains active after the equip handoff, but the
        // loose-model bridge is equip-only and may never be resurrected.
        decision.presentBridgeModel =
            input.bridgeModelAvailable && !state.nativeHandoffObserved;

        if (!state.presentationRecoveryWindowActive) {
            state.presentationRecoveryWindowStartedAtSeconds =
                input.drawRecoveryElapsedSeconds;
            state.nextPresentationRepairAtSeconds =
                input.drawRecoveryElapsedSeconds +
                kPresentationRecoveryGraceSeconds;
            state.presentationRecoveryWindowActive = true;
        }

        if (!input.mutationAllowed ||
            state.missingFrames < kMissingFramesBeforeRepair ||
            input.drawRecoveryElapsedSeconds <
                state.nextPresentationRepairAtSeconds) {
            return decision;
        }

        const float presentationRecoveryElapsedSeconds =
            input.drawRecoveryElapsedSeconds -
            state.presentationRecoveryWindowStartedAtSeconds;
        if (presentationRecoveryElapsedSeconds >=
            kPresentationRecoveryDeadlineSeconds) {
            decision.repair = RepairAction::Exhausted;
            return decision;
        }

        const bool localVisibilityRepairAvailable =
            input.nativeInstanceFound &&
            state.localVisibilityAttempts < kMaximumLocalVisibilityAttempts;
        if (localVisibilityRepairAvailable &&
            state.localVisibilityAttempts <= state.attachAttempts) {
            ++state.localVisibilityAttempts;
            decision.repair = RepairAction::RestoreLocalVisibility;
            state.nextPresentationRepairAtSeconds =
                input.drawRecoveryElapsedSeconds +
                kLocalVisibilitySettleSeconds;
            return decision;
        }

        if (state.attachAttempts < kMaximumAttachAttempts) {
            ++state.attachAttempts;
            decision.repair = RepairAction::QueueNativeAttach;
            state.nextPresentationRepairAtSeconds =
                input.drawRecoveryElapsedSeconds +
                kNativeAttachSettleSeconds;
            return decision;
        }

        if (localVisibilityRepairAvailable) {
            ++state.localVisibilityAttempts;
            decision.repair = RepairAction::RestoreLocalVisibility;
            state.nextPresentationRepairAtSeconds =
                input.drawRecoveryElapsedSeconds +
                kLocalVisibilitySettleSeconds;
        }
        return decision;
    }
}
