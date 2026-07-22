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
    constexpr std::uint8_t kDrawSettleFrames = 6;
    constexpr std::uint8_t kMaximumDrawAttempts = 3;
    constexpr std::uint8_t kWantToDrawStallFrames = 45;

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
        std::uint8_t drawSettleFramesRemaining{ 0 };
        std::uint8_t drawAttempts{ 0 };
        std::uint8_t wantToDrawFrames{ 0 };
        bool nativeHandoffObserved{ false };
    };

    struct FrameInput
    {
        bool mutationAllowed{ false };
        bool identityMatches{ false };
        bool presentationExpected{ false };
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

    [[nodiscard]] inline constexpr bool presentationExpectedFromNativeState(
        const std::uint32_t nativeWeaponState,
        const bool priorPresentationExpected) noexcept
    {
        using NativeWeaponState = held_weapon_equip_state_policy::NativeWeaponState;
        switch (static_cast<NativeWeaponState>(nativeWeaponState)) {
        case NativeWeaponState::WantToDraw:
        case NativeWeaponState::Drawing:
        case NativeWeaponState::Drawn:
            return true;
        case NativeWeaponState::WantToSheathe:
        case NativeWeaponState::Sheathing:
            return priorPresentationExpected;
        case NativeWeaponState::Sheathed:
        default:
            return false;
        }
    }

    [[nodiscard]] inline constexpr bool presentationExpectedAfterMenu(
        const bool menuEntryPresentationExpected,
        const bool identityMutatedWhileMenuOpen,
        const bool currentIdentityDiffersFromMenuEntry,
        const std::uint32_t nativeWeaponState,
        const bool priorPresentationExpected) noexcept
    {
        // Native menu code may fully holster before ROCK sees the close. An
        // equipped-identity mutation is the durable evidence that the player
        // selected or rebuilt a weapon and expects it to be presented again.
        return menuEntryPresentationExpected ||
               identityMutatedWhileMenuOpen ||
               currentIdentityDiffersFromMenuEntry ||
               presentationExpectedFromNativeState(
                   nativeWeaponState,
                   priorPresentationExpected);
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

    [[nodiscard]] inline constexpr Decision advance(State& state, const FrameInput& input) noexcept
    {
        Decision decision{};

        if (!input.identityMatches) {
            state.stableFrames = 0;
            state.missingFrames = 0;
            state.drawSettleFramesRemaining = 0;
            state.wantToDrawFrames = 0;
            return decision;
        }

        if (!input.presentationExpected) {
            state.stableFrames = 0;
            state.missingFrames = 0;
            state.drawSettleFramesRemaining = 0;
            state.wantToDrawFrames = 0;
            return decision;
        }

        if (!input.weaponExactlyDrawn) {
            state.stableFrames = 0;
            state.missingFrames = 0;
            // Before the first native handoff, WantToDraw/Drawing may leave a
            // real gap which the bridge must cover. After a native handoff,
            // however, a non-drawn state is a real holster transition; never
            // resurrect the loose model over it.
            decision.presentBridgeModel =
                input.bridgeModelAvailable && !state.nativeHandoffObserved;

            if (state.nativeHandoffObserved || !input.mutationAllowed) {
                return decision;
            }

            if (state.drawSettleFramesRemaining > 0) {
                --state.drawSettleFramesRemaining;
            }

            using NativeWeaponState = held_weapon_equip_state_policy::NativeWeaponState;
            const auto nativeState = static_cast<NativeWeaponState>(input.nativeWeaponState);
            if (nativeState == NativeWeaponState::WantToDraw) {
                if (state.wantToDrawFrames < kWantToDrawStallFrames) {
                    ++state.wantToDrawFrames;
                }
                if (state.wantToDrawFrames < kWantToDrawStallFrames ||
                    state.drawSettleFramesRemaining > 0) {
                    return decision;
                }
            } else if (nativeState == NativeWeaponState::Drawing) {
                state.wantToDrawFrames = 0;
                return decision;
            } else {
                state.wantToDrawFrames = 0;
                if (!held_weapon_equip_state_policy::shouldSubmitDrawFollowup(
                        input.nativeWeaponState) ||
                    state.drawSettleFramesRemaining > 0) {
                    if (!held_weapon_equip_state_policy::isValidNativeWeaponState(
                            input.nativeWeaponState)) {
                        decision.repair = RepairAction::DrawExhausted;
                    }
                    return decision;
                }
            }

            if (state.drawAttempts < kMaximumDrawAttempts) {
                ++state.drawAttempts;
                state.drawSettleFramesRemaining = kDrawSettleFrames;
                state.wantToDrawFrames = 0;
                decision.repair = RepairAction::RequestDraw;
            } else {
                decision.repair = RepairAction::DrawExhausted;
            }
            return decision;
        }

        state.drawSettleFramesRemaining = 0;
        state.wantToDrawFrames = 0;

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
