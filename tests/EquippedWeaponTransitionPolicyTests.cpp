#include "physics-interaction/weapon/EquippedWeaponTransitionPolicy.h"

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
        .bridgeModelAvailable = true,
    });
    ok &= expect("a completed native handoff must not resurrect the bridge while holstering",
        !holsteredAfterHandoff.presentBridgeModel &&
            !holsteredAfterHandoff.handoffBridgeToNative &&
            holsteredAfterHandoff.repair == RepairAction::None);

    const auto lateDetachOne = advance(stableState, FrameInput{
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = true,
        .bridgeModelAvailable = true,
    });
    const auto lateDetachTwo = advance(stableState, FrameInput{
        .mutationAllowed = true,
        .identityMatches = true,
        .weaponExactlyDrawn = true,
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
    });
    ok &= expect("native attachment recovery must be bounded",
        exhausted.repair == RepairAction::Exhausted &&
            exhaustedState.attachAttempts == kMaximumAttachAttempts);

    return ok ? 0 : 1;
}
