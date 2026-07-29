#include "physics-interaction/api/InteractionCommandPolicy.h"
#include "physics-interaction/core/ForceGrabPolicy.h"
#include "physics-interaction/weapon/BareFistGuardPolicy.h"
#include "physics-interaction/weapon/HeldWeaponEquipStatePolicy.h"

#include <cstdio>

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

    template <class T>
    bool expectEqual(const char* label, T actual, T expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %u got %u\n", label, static_cast<unsigned>(expected), static_cast<unsigned>(actual));
        return false;
    }
}

int main()
{
    using namespace rock::force_grab_policy;
    using namespace rock::provider;
    using namespace rock::provider::interaction_command_policy;

    bool ok = true;
    const HandAvailabilityInput freeHand{};
    const HandAvailabilityInput blockedHand{ .holding = true };

    ok &= expectEqual("grenade prefers right", selectGrenadeHand(false, freeHand, freeHand).hand, HandChoice::Right);
    ok &= expectEqual("grenade falls back left", selectGrenadeHand(false, blockedHand, freeHand).hand, HandChoice::Left);
    ok &= expectEqual("blocked hands reject", selectGrenadeHand(false, blockedHand, blockedHand).failure, GrenadeSelectionFailure::HandsBlocked);
    ok &= expectEqual("held grenade rejects globally", selectGrenadeHand(true, freeHand, freeHand).failure, GrenadeSelectionFailure::GrenadeAlreadyHeld);

    HandAvailabilityInput pending = freeHand;
    pending.pendingForceGrab = true;
    ok &= expectEqual("pending right force grab selects left", selectGrenadeHand(false, pending, freeHand).hand, HandChoice::Left);

    HandAvailabilityInput selected = freeHand;
    selected.openInteractionState = true;
    ok &= expectTrue("selected close/far state stays replaceable", isAvailable(selected));
    selected.openInteractionState = false;
    ok &= expectFalse("locked transition state blocks", isAvailable(selected));

    HandAvailabilityInput disabled = freeHand;
    disabled.disabled = true;
    ok &= expectFalse("disabled hand blocks", isAvailable(disabled));
    HandAvailabilityInput pulling = freeHand;
    pulling.activePullCatch = true;
    ok &= expectFalse("pull catch blocks", isAvailable(pulling));
    HandAvailabilityInput handoff = freeHand;
    handoff.actorEquipmentHandoff = true;
    ok &= expectFalse("actor equipment handoff blocks", isAvailable(handoff));
    HandAvailabilityInput weaponOccupied = freeHand;
    weaponOccupied.equippedWeaponOccupiesHand = true;
    ok &= expectFalse("equipped weapon role blocks", isAvailable(weaponOccupied));
    HandAvailabilityInput touchGrabbed = freeHand;
    touchGrabbed.touchGrabActive = true;
    ok &= expectFalse("touch grab blocks replacement", isAvailable(touchGrabbed));

    ok &= expectTrue("current right firing hand occupied", equippedWeaponOccupiesHand(false, true, false, false, false));
    ok &= expectFalse("current left offhand remains free", equippedWeaponOccupiesHand(true, true, false, false, false));
    ok &= expectFalse("detached firing hand remains free", equippedWeaponOccupiesHand(false, true, true, false, false));
    ok &= expectTrue("part grip always occupies hand", equippedWeaponOccupiesHand(false, true, true, false, true));
    ok &= expectTrue("future left firing hand occupied", equippedWeaponOccupiesHand(true, true, false, true, false));
    ok &= expectFalse("future right offhand remains free", equippedWeaponOccupiesHand(false, true, false, true, false));

    using rock::bare_fist_guard_policy::Witness;
    using rock::bare_fist_guard_policy::shouldHolster;
    ok &= expectTrue("drawn bare fists are holstered", shouldHolster(Witness{
        .weaponDrawn = true,
        .actorUsingMelee = true,
        .realMeleeWeaponEquipped = false,
    }));
    ok &= expectFalse("real hand-to-hand weapon is preserved", shouldHolster(Witness{
        .weaponDrawn = true,
        .actorUsingMelee = true,
        .realMeleeWeaponEquipped = true,
    }));
    ok &= expectFalse("holstered player is unchanged", shouldHolster(Witness{
        .weaponDrawn = false,
        .actorUsingMelee = true,
        .realMeleeWeaponEquipped = false,
    }));

    using rock::bare_fist_guard_policy::RecheckState;
    using rock::bare_fist_guard_policy::shouldRefreshWitness;
    const RecheckState initialWitness{};
    ok &= expectTrue("bare fist witness initializes", shouldRefreshWitness(initialWitness, false, false, 0, false));
    const RecheckState stableWitness{ .initialized = true, .weaponDrawn = false, .equippedWeaponFormId = 42 };
    ok &= expectFalse("unchanged bare fist witness is cached", shouldRefreshWitness(stableWitness, false, false, 42, false));
    ok &= expectTrue("menu close forces bare fist witness", shouldRefreshWitness(stableWitness, true, false, 42, false));
    ok &= expectTrue("draw transition refreshes bare fist witness", shouldRefreshWitness(stableWitness, false, true, 42, false));
    ok &= expectTrue("equip transition refreshes bare fist witness", shouldRefreshWitness(stableWitness, false, false, 43, false));
    ok &= expectTrue("melee-state transition refreshes bare fist witness", shouldRefreshWitness(stableWitness, false, false, 42, true));

    using rock::held_weapon_equip_state_policy::EquipReadiness;
    using rock::held_weapon_equip_state_policy::classifyForEquip;
    using rock::held_weapon_equip_state_policy::isValidNativeWeaponState;
    using rock::held_weapon_equip_state_policy::shouldRearmTrigger;
    using rock::held_weapon_equip_state_policy::shouldSubmitDrawFollowup;
    ok &= expectEqual("sheathed state permits equip", classifyForEquip(0), EquipReadiness::Stable);
    ok &= expectEqual("drawn state permits replacement equip", classifyForEquip(3), EquipReadiness::Stable);
    ok &= expectEqual("want-draw state defers equip", classifyForEquip(1), EquipReadiness::Transitioning);
    ok &= expectEqual("drawing state defers equip", classifyForEquip(2), EquipReadiness::Transitioning);
    ok &= expectEqual("want-sheathe state defers equip", classifyForEquip(4), EquipReadiness::Transitioning);
    ok &= expectEqual("sheathing state defers equip", classifyForEquip(5), EquipReadiness::Transitioning);
    ok &= expectEqual("unknown state blocks equip", classifyForEquip(6), EquipReadiness::Invalid);
    ok &= expectTrue("transitioning trigger request rearms", shouldRearmTrigger(4, true));
    ok &= expectFalse("grip-zone request does not need trigger lease", shouldRearmTrigger(4, false));
    ok &= expectFalse("stable trigger request does not rearm", shouldRearmTrigger(0, true));
    ok &= expectTrue("sheathed native weapon accepts draw recovery", shouldSubmitDrawFollowup(0));
    ok &= expectTrue("want-draw native weapon accepts a bounded stalled retry", shouldSubmitDrawFollowup(1));
    ok &= expectFalse("drawing native weapon rejects duplicate draw recovery", shouldSubmitDrawFollowup(2));
    ok &= expectFalse("drawn native weapon rejects duplicate draw recovery", shouldSubmitDrawFollowup(3));
    ok &= expectTrue("want-sheathe native weapon accepts draw reversal", shouldSubmitDrawFollowup(4));
    ok &= expectTrue("sheathing native weapon accepts draw reversal", shouldSubmitDrawFollowup(5));
    ok &= expectFalse("unknown native weapon state rejects draw recovery", shouldSubmitDrawFollowup(6));
    ok &= expectTrue("last known native weapon state is valid", isValidNativeWeaponState(5));
    ok &= expectFalse("state outside the FO4VR weapon enum is invalid", isValidNativeWeaponState(6));

    ForceGrabReservations reservations;
    ok &= expectFalse("invalid API hand cannot reserve", reservations.reserve(RockProviderHand::None, 11, 100));
    ok &= expectTrue("first right API force grab reserves", reservations.reserve(RockProviderHand::Right, 11, 101));
    ok &= expectFalse("duplicate right API force grab is rejected", reservations.reserve(RockProviderHand::Right, 11, 102));
    ok &= expectTrue("independent left API force grab is allowed", reservations.reserve(RockProviderHand::Left, 22, 201));
    ok &= expectTrue("dequeue does not implicitly release right", reservations.isReserved(RockProviderHand::Right));
    reservations.release(11, 101);
    ok &= expectFalse("terminal result releases matching right", reservations.isReserved(RockProviderHand::Right));
    ok &= expectTrue("terminal right does not release left", reservations.isReserved(RockProviderHand::Left));
    reservations.clearOwner(22);
    ok &= expectFalse("owner loss clears active left", reservations.isReserved(RockProviderHand::Left));
    ok &= expectFalse("queued state is non-terminal", isTerminal(RockProviderInteractionCommandStateV1::Queued));
    ok &= expectTrue("rejected state is terminal", isTerminal(RockProviderInteractionCommandStateV1::Rejected));

    RockProviderInteractionCommandResultV1 accepted{};
    accepted.state = RockProviderInteractionCommandStateV1::Queued;
    accepted.stage = RockProviderCommandStageV1::Committed;
    accepted.acceptedFrame = 17;
    accepted.committedFrame = 19;
    RockProviderInteractionCommandResultV1 completed{};
    completed.state = RockProviderInteractionCommandStateV1::Succeeded;
    completed.stage = RockProviderCommandStageV1::Queued;
    completed.acceptedFrame = 23;
    const auto mergedCompletion = mergeResultHistory(accepted, completed, 29);
    ok &= expectEqual("completion preserves acceptance frame", mergedCompletion.acceptedFrame, std::uint64_t{ 17 });
    ok &= expectEqual("completion preserves committed frame", mergedCompletion.committedFrame, std::uint64_t{ 19 });
    ok &= expectEqual("completion records terminal frame", mergedCompletion.frameIndex, std::uint64_t{ 29 });
    ok &= expectEqual("successful completion records applied frame", mergedCompletion.appliedFrame, std::uint64_t{ 29 });
    ok &= expectEqual("completion normalizes terminal stage", mergedCompletion.stage, RockProviderCommandStageV1::Terminal);

    completed.state = RockProviderInteractionCommandStateV1::Rejected;
    completed.failure = RockProviderInteractionFailureV1::TargetUnavailable;
    completed.failureStage = RockProviderInteractionFailureV1::None;
    completed.frameIndex = 31;
    completed.appliedFrame = 0;
    const auto mergedRejection = mergeResultHistory(accepted, completed, 37);
    ok &= expectEqual("rejection preserves explicit terminal frame", mergedRejection.frameIndex, std::uint64_t{ 31 });
    ok &= expectEqual("rejection records failure stage", mergedRejection.failureStage, RockProviderInteractionFailureV1::TargetUnavailable);
    ok &= expectEqual("rejection does not invent applied frame", mergedRejection.appliedFrame, std::uint64_t{ 0 });

    return ok ? 0 : 1;
}
