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
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = true,
        .nativeWeaponState = 3,
        .bridgeModelAvailable = true,
    });
    const auto lateDetachTwo = advance(stableState, FrameInput{
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = true,
        .nativeWeaponState = 3,
        .bridgeModelAvailable = true,
    });
    ok &= expect("a late native detach must immediately restore the standby bridge",
        lateDetachOne.presentBridgeModel &&
            lateDetachOne.repair == RepairAction::None);
    ok &= expect("two missing frames must request one exact native attach",
        lateDetachTwo.presentBridgeModel &&
            lateDetachTwo.repair == RepairAction::QueueNativeAttach &&
            stableState.attachAttempts == 1 &&
            stableState.attachSettleFramesRemaining == kAttachSettleFrames);

    State hiddenState{};
    const FrameInput hiddenNative{
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = true,
        .nativeWeaponState = 3,
        .nativeInstanceFound = true,
        .nativeAncestorPathVisible = false,
        .nativeInstanceLocallyVisible = true,
    };
    (void)advance(hiddenState, hiddenNative);
    const auto hiddenRepair = advance(hiddenState, hiddenNative);
    ok &= expect("an existing hidden exact instance must be unculled before reattachment",
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
        .attachAttempts = kMaximumAttachAttempts,
    };
    const auto exhausted = advance(exhaustedState, FrameInput{
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = true,
        .nativeWeaponState = 3,
    });
    ok &= expect("native attachment recovery must be bounded",
        exhausted.repair == RepairAction::Exhausted &&
            exhaustedState.attachAttempts == kMaximumAttachAttempts);

    State drawState{};
    const auto firstDraw = advance(drawState, FrameInput{
        .drawRecoveryElapsedSeconds = 0.0f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 0,
        .bridgeModelAvailable = true,
    });
    ok &= expect("every bound sheathed weapon must request an immediate draw",
        firstDraw.presentBridgeModel &&
            firstDraw.repair == RepairAction::RequestDraw &&
            drawState.drawRequests == 1 &&
            drawState.drawRecoveryWindowActive);

    const auto highRateFrame = advance(drawState, FrameInput{
        .drawRecoveryElapsedSeconds = 0.001f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 4,
        .bridgeModelAvailable = true,
    });
    const auto highRateFrameTwo = advance(drawState, FrameInput{
        .drawRecoveryElapsedSeconds = 0.016f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 4,
        .bridgeModelAvailable = true,
    });
    const auto beforeRetryDeadline = advance(drawState, FrameInput{
        .drawRecoveryElapsedSeconds = 0.099f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 5,
        .bridgeModelAvailable = true,
    });
    ok &= expect("high frame rates must not consume a draw retry budget",
        highRateFrame.presentBridgeModel &&
            highRateFrame.repair == RepairAction::None &&
            highRateFrameTwo.repair == RepairAction::None &&
            beforeRetryDeadline.repair == RepairAction::None &&
            drawState.drawRequests == 1);

    const auto timedRetry = advance(drawState, FrameInput{
        .drawRecoveryElapsedSeconds = kDrawRetryIntervalSeconds,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 0,
        .bridgeModelAvailable = true,
    });
    const auto variableRateRetry = advance(drawState, FrameInput{
        .drawRecoveryElapsedSeconds = 0.347f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 0,
        .bridgeModelAvailable = true,
    });
    const auto noCatchUpBurst = advance(drawState, FrameInput{
        .drawRecoveryElapsedSeconds = 0.351f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 0,
        .bridgeModelAvailable = true,
    });
    ok &= expect("draw retries must follow elapsed time without catch-up bursts",
        timedRetry.repair == RepairAction::RequestDraw &&
            variableRateRetry.repair == RepairAction::RequestDraw &&
            noCatchUpBurst.repair == RepairAction::None &&
            drawState.drawRequests == 3);

    const auto delayedNativeReadinessRetry = advance(drawState, FrameInput{
        .drawRecoveryElapsedSeconds = 0.451f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 0,
        .bridgeModelAvailable = true,
    });
    ok &= expect("a void draw submission must not exhaust recovery after three requests",
        delayedNativeReadinessRetry.repair == RepairAction::RequestDraw &&
            drawState.drawRequests == 4 &&
            !drawState.drawRecoveryExhausted);

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
        .drawRecoveryElapsedSeconds = 0.36f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 1,
    });
    const auto waiting = advance(stalledWantToDraw, FrameInput{
        .drawRecoveryElapsedSeconds = 0.859f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 1,
    });
    const auto stalledRetry = advance(stalledWantToDraw, FrameInput{
        .drawRecoveryElapsedSeconds = 0.86f,
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
            .drawRecoveryElapsedSeconds = 1.37f,
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
        returnedToSheathed.repair == RepairAction::RequestDraw &&
            acknowledgedThenReturned.drawRequests == 1 &&
            !acknowledgedThenReturned.drawRecoveryExhausted);

    State holsterReversal{};
    const auto reverseHolster = advance(holsterReversal, FrameInput{
        .drawRecoveryElapsedSeconds = 10.0f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 5,
        .bridgeModelAvailable = true,
    });
    ok &= expect("a bound weapon must reverse a pre-handoff sheathing state",
        reverseHolster.repair == RepairAction::RequestDraw);

    State exhaustedDrawState{};
    const auto initialLateClockDraw = advance(exhaustedDrawState, FrameInput{
        .drawRecoveryElapsedSeconds = 10.0f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 0,
    });
    const auto exhaustedDraw = advance(exhaustedDrawState, FrameInput{
        .drawRecoveryElapsedSeconds = 11.01f,
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = false,
        .nativeWeaponState = 0,
    });
    ok &= expect("native draw recovery must use a bounded elapsed window, not absolute transition age",
        initialLateClockDraw.repair == RepairAction::RequestDraw &&
            exhaustedDraw.repair == RepairAction::DrawExhausted &&
            exhaustedDrawState.drawRequests == 1 &&
            exhaustedDrawState.drawRecoveryExhausted);

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
