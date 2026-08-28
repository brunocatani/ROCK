#include "physics-interaction/weapon/EquippedWeaponTransitionPolicy.h"
#include "physics-interaction/weapon/EquipVisualBridgePolicy.h"

#include <cstdio>

namespace
{
    bool expect(const char* label, const bool condition)
    {
        if (condition) {
            return true;
        }
        std::printf("%s\n", label);
        return false;
    }
}

int main()
{
    using namespace rock::equipped_weapon_transition_policy;

    bool ok = true;

    ok &= expect("the bridge must honor a shorter positive presentation lease",
        rock::equip_visual_bridge_policy::effectivePresentationLeaseSeconds(0.4f) ==
            0.4f);
    ok &= expect("the bridge must clamp every long presentation lease to one second",
        rock::equip_visual_bridge_policy::effectivePresentationLeaseSeconds(12.0f) ==
            rock::equip_visual_bridge_policy::kMaximumPresentationLeaseSeconds);
    ok &= expect("an invalid presentation lease must fail safe to the one-second maximum",
        rock::equip_visual_bridge_policy::effectivePresentationLeaseSeconds(0.0f) ==
            rock::equip_visual_bridge_policy::kMaximumPresentationLeaseSeconds);
    ok &= expect("the bridge must remain available before its absolute lease expires",
        !rock::equip_visual_bridge_policy::presentationLeaseExpired(0.999f, 12.0f));
    ok &= expect("the bridge must expire at the one-second absolute maximum",
        rock::equip_visual_bridge_policy::presentationLeaseExpired(1.0f, 12.0f));

    ok &= expect("an exact requested instance must match",
        matchesExpectedIdentity(0x1234, 0x2222, 0x1234, 0x2222, 0x1234, 0x1111));
    ok &= expect("a same-base old instance must not satisfy a deferred equip",
        !matchesExpectedIdentity(0x1234, 0x1111, 0x1234, 0x2222, 0x1234, 0x1111));
    ok &= expect("a native-cloned instance may match only after leaving the old baseline",
        matchesExpectedIdentity(0x1234, 0x3333, 0x1234, 0x2222, 0x1234, 0x1111));
    ok &= expect("another weapon form must never satisfy the target",
        !matchesExpectedIdentity(0x5678, 0x3333, 0x1234, 0x2222, 0x1234, 0x1111));

    State transitionState{};
    const auto drawing = advance(transitionState, FrameInput{
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 2,
        .bridgeModelAvailable = true,
    });
    ok &= expect("drawing must keep the bridge without repairing native state",
        drawing.presentBridgeModel &&
            !drawing.handoffBridgeToNative &&
            drawing.repair == RepairAction::None);

    State stableState{};
    const FrameInput nativeVisible{
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = true,
        .nativeWeaponState = 3,
        .bridgeModelAvailable = true,
        .nativeInstanceFound = true,
        .nativeAncestorPathVisible = true,
        .nativeInstanceLocallyVisible = true,
    };
    const auto stableOne = advance(stableState, nativeVisible);
    const auto stableTwo = advance(stableState, nativeVisible);
    const auto stableThree = advance(stableState, nativeVisible);
    ok &= expect("native presentation must be consecutive before handoff",
        stableOne.presentBridgeModel &&
            stableTwo.presentBridgeModel &&
            !stableOne.handoffBridgeToNative &&
            !stableTwo.handoffBridgeToNative &&
            stableThree.handoffBridgeToNative &&
            stableState.nativeHandoffObserved);

    const auto holsteredAfterHandoff = advance(stableState, FrameInput{
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 0,
        .bridgeModelAvailable = true,
    });
    ok &= expect("a later engine transition must not resurrect a completed equip bridge",
        !holsteredAfterHandoff.presentBridgeModel &&
            !holsteredAfterHandoff.handoffBridgeToNative &&
            holsteredAfterHandoff.repair == RepairAction::None);

    const auto lateDetachOne = advance(stableState, FrameInput{
        .drawRecoveryElapsedSeconds = 0.0f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = true,
        .nativeWeaponState = 3,
        .bridgeModelAvailable = true,
    });
    const auto lateDetachTwo = advance(stableState, FrameInput{
        .drawRecoveryElapsedSeconds =
            kPresentationRecoveryGraceSeconds - 0.001f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = true,
        .nativeWeaponState = 3,
        .bridgeModelAvailable = true,
    });
    ok &= expect("a late native detach must not resurrect the completed equip bridge",
        !lateDetachOne.presentBridgeModel &&
            lateDetachOne.repair == RepairAction::None);
    const auto lateDetachRepair = advance(stableState, FrameInput{
        .drawRecoveryElapsedSeconds = kPresentationRecoveryGraceSeconds,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = true,
        .nativeWeaponState = 3,
        .bridgeModelAvailable = true,
    });
    ok &= expect("a late detach must honor native grace before attachment repair",
        !lateDetachTwo.presentBridgeModel &&
            lateDetachTwo.repair == RepairAction::None &&
            lateDetachRepair.repair == RepairAction::QueueNativeAttach &&
            stableState.attachAttempts == 1);

    State hiddenState{};
    const FrameInput hiddenNative{
        .drawRecoveryElapsedSeconds = 0.0f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = true,
        .nativeWeaponState = 3,
        .nativeInstanceFound = true,
        .nativeAncestorPathVisible = false,
        .nativeInstanceLocallyVisible = true,
    };
    (void)advance(hiddenState, hiddenNative);
    auto hiddenAfterGrace = hiddenNative;
    hiddenAfterGrace.drawRecoveryElapsedSeconds =
        kPresentationRecoveryGraceSeconds;
    const auto hiddenRepair = advance(hiddenState, hiddenAfterGrace);
    ok &= expect("an existing hidden exact instance must receive timed visibility repair before reattachment",
        hiddenRepair.repair == RepairAction::RestoreLocalVisibility &&
            hiddenState.localVisibilityAttempts == 1 &&
            hiddenState.attachAttempts == 0);

    State ownedCullState{};
    const auto ownedCull = advance(ownedCullState, FrameInput{
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = true,
        .nativeWeaponState = 3,
        .bridgeModelAvailable = true,
        .nativeInstanceFound = true,
        .nativeAncestorPathVisible = true,
        .nativeInstanceLocallyVisible = false,
        .bridgeOwnsNativeInstanceCull = true,
    });
    ok &= expect("the bridge's exact-child cull must count as structurally stable",
        ownedCull.presentBridgeModel &&
            ownedCullState.stableFrames == 1 &&
            ownedCull.repair == RepairAction::None);

    State blockedState{ .missingFrames = kMissingFramesBeforeRepair };
    const auto blocked = advance(blockedState, FrameInput{
        .mutationAllowed = false,
        .identityMatches = true,
        .weaponExactlyDrawn = true,
        .nativeWeaponState = 3,
    });
    ok &= expect("menu or compatibility blocking must prevent native mutation",
        blocked.repair == RepairAction::None);

    State exhaustedState{
        .missingFrames = kMissingFramesBeforeRepair,
        .localVisibilityAttempts = kMaximumLocalVisibilityAttempts,
        .attachAttempts = kMaximumAttachAttempts,
        .presentationRecoveryWindowStartedAtSeconds = 0.0f,
        .nextPresentationRepairAtSeconds = 0.0f,
        .presentationRecoveryWindowActive = true,
    };
    const auto exhausted = advance(exhaustedState, FrameInput{
        .drawRecoveryElapsedSeconds =
            kPresentationRecoveryDeadlineSeconds,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = true,
        .nativeWeaponState = 3,
    });
    ok &= expect("native attachment recovery must be bounded",
        exhausted.repair == RepairAction::Exhausted &&
            exhaustedState.localVisibilityAttempts ==
                kMaximumLocalVisibilityAttempts &&
            exhaustedState.attachAttempts == kMaximumAttachAttempts);

    State drawState{};
    const auto waitForWantToSheathe = advance(drawState, FrameInput{
        .drawRecoveryElapsedSeconds = 10.0f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 4,
        .bridgeModelAvailable = true,
    });
    const auto waitForSheathing = advance(drawState, FrameInput{
        .drawRecoveryElapsedSeconds = 10.2f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 5,
        .bridgeModelAvailable = true,
    });
    ok &= expect("menu-time holstering must finish before equip draw recovery",
        waitForWantToSheathe.presentBridgeModel &&
            waitForWantToSheathe.repair == RepairAction::None &&
            waitForSheathing.repair == RepairAction::None &&
            drawState.drawRequests == 0 &&
            !drawState.drawRecoveryWindowActive);

    const auto firstPreparedDraw = advance(drawState, FrameInput{
        .drawRecoveryElapsedSeconds = 10.25f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 0,
        .bridgeModelAvailable = true,
    });
    const auto beforePreparedRetry = advance(drawState, FrameInput{
        .drawRecoveryElapsedSeconds =
            10.25f + kDrawRetryIntervalSeconds - 0.001f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 0,
        .bridgeModelAvailable = true,
    });
    const auto preparedRetry = advance(drawState, FrameInput{
        .drawRecoveryElapsedSeconds =
            10.25f + kDrawRetryIntervalSeconds,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 0,
        .bridgeModelAvailable = true,
    });
    ok &= expect("stable sheathed recovery must repeat native equip preparation at timed intervals",
        firstPreparedDraw.repair == RepairAction::RequestPreparedDraw &&
            beforePreparedRetry.repair == RepairAction::None &&
            preparedRetry.repair == RepairAction::RequestPreparedDraw &&
            drawState.drawRequests == 2 &&
            drawState.drawRecoveryWindowActive);

    const auto preparedDrawExhausted = advance(drawState, FrameInput{
        .drawRecoveryElapsedSeconds =
            10.25f + kDrawRecoveryDeadlineSeconds,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 0,
    });
    ok &= expect("prepared draw recovery must remain bounded by acknowledgment time",
        preparedDrawExhausted.repair == RepairAction::DrawExhausted &&
            drawState.drawRecoveryExhausted);

    State blockedDrawState{};
    const auto blockedDraw = advance(blockedDrawState, FrameInput{
        .drawRecoveryElapsedSeconds = 20.0f,
        .mutationAllowed = false,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 0,
        .bridgeModelAvailable = true,
    });
    ok &= expect("menu blocking must preserve the bridge without submitting a draw",
        blockedDraw.presentBridgeModel &&
            blockedDraw.repair == RepairAction::None &&
            blockedDrawState.drawRequests == 0 &&
            !blockedDrawState.drawRecoveryWindowActive);

    State stalledWantToDraw{};
    const auto wantToDrawAcknowledged = advance(stalledWantToDraw, FrameInput{
        .drawRecoveryElapsedSeconds = 0.0f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 1,
    });
    const auto waiting = advance(stalledWantToDraw, FrameInput{
        .drawRecoveryElapsedSeconds = kWantToDrawStallSeconds - 0.001f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 1,
    });
    const auto stalledRetry = advance(stalledWantToDraw, FrameInput{
        .drawRecoveryElapsedSeconds = kWantToDrawStallSeconds,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 1,
    });
    ok &= expect("want-to-draw acknowledgment must receive one time-spaced refresh only after stalling",
        wantToDrawAcknowledged.repair == RepairAction::None &&
            waiting.repair == RepairAction::None &&
        stalledRetry.repair == RepairAction::RequestDraw &&
            stalledWantToDraw.drawRequests == 1);

    const auto stalledWantToDrawExhausted =
        advance(stalledWantToDraw, FrameInput{
            .drawRecoveryElapsedSeconds = kDrawRecoveryDeadlineSeconds,
            .mutationAllowed = true,
            .identityMatches = true,
            .weaponExactlyDrawn = false,
            .nativeWeaponState = 1,
        });
    ok &= expect("a want-to-draw state that never advances must remain bounded",
        stalledWantToDrawExhausted.repair == RepairAction::DrawExhausted &&
            stalledWantToDraw.drawRecoveryExhausted);

    const auto lateDrawingAcknowledgment =
        advance(stalledWantToDraw, FrameInput{
            .drawRecoveryElapsedSeconds = 5.0f,
            .mutationAllowed = true,
            .identityMatches = true,
            .weaponExactlyDrawn = false,
            .nativeWeaponState = 2,
        });
    ok &= expect("drawing must acknowledge recovery even after an earlier timeout",
        lateDrawingAcknowledgment.repair == RepairAction::None &&
            !stalledWantToDraw.drawRecoveryExhausted &&
            !stalledWantToDraw.drawRecoveryWindowActive);

    State partialDrawState{
        .partialDrawRecoveryStartedAtSeconds = 4.0f,
        .partialDrawRecoveryActive = true,
    };
    const auto partialDrawPending = advance(partialDrawState, FrameInput{
        .drawRecoveryElapsedSeconds =
            4.0f + kPartialDrawCompletionDeadlineSeconds - 0.001f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 2,
    });
    const auto partialDrawFinalize = advance(partialDrawState, FrameInput{
        .drawRecoveryElapsedSeconds =
            4.0f + kPartialDrawCompletionDeadlineSeconds,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 2,
    });
    ok &= expect("a clip-proven partial draw must receive one bounded native finalization",
        partialDrawPending.repair == RepairAction::None &&
            partialDrawFinalize.repair == RepairAction::FinalizePartialDraw &&
            partialDrawState.partialDrawRecoveryActive);

    partialDrawState.drawRecoveryExhausted = true;
    const auto failedPartialDraw = advance(partialDrawState, FrameInput{
        .drawRecoveryElapsedSeconds =
            4.0f + kPartialDrawCompletionDeadlineSeconds + 0.01f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 2,
    });
    ok &= expect("a failed partial draw finalization must fail closed",
        failedPartialDraw.repair == RepairAction::DrawExhausted);

    State acknowledgedThenReturned{};
    (void)advance(acknowledgedThenReturned, FrameInput{
        .drawRecoveryElapsedSeconds = 0.25f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 1,
    });
    const auto returnedToSheathed =
        advance(acknowledgedThenReturned, FrameInput{
            .drawRecoveryElapsedSeconds = 5.0f,
            .mutationAllowed = true,
            .identityMatches = true,
            .weaponExactlyDrawn = false,
            .nativeWeaponState = 0,
        });
    ok &= expect("a native acknowledgment followed by sheathed must start a fresh recovery window",
        returnedToSheathed.repair == RepairAction::RequestPreparedDraw &&
            acknowledgedThenReturned.drawRequests == 1 &&
            !acknowledgedThenReturned.drawRecoveryExhausted);

    State invalidDrawState{};
    const auto invalidDraw = advance(invalidDrawState, FrameInput{
        .drawRecoveryElapsedSeconds = 0.0f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 99,
    });
    ok &= expect("an invalid native weapon state must fail closed immediately",
        invalidDraw.repair == RepairAction::DrawExhausted &&
            invalidDrawState.drawRecoveryExhausted &&
            invalidDrawState.drawRequests == 0);

    return ok ? 0 : 1;
}
