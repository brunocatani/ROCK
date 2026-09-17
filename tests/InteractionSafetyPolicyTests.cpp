#include "physics-interaction/api/InteractionCommandPolicy.h"
#include "physics-interaction/core/ForceGrabPolicy.h"
#include "physics-interaction/grenade/LooseThrowablePolicy.h"
#include "physics-interaction/object/PhysicsBodyClassifier.h"
#include "physics-interaction/weapon/BareFistGuardPolicy.h"
#include "physics-interaction/weapon/HeldWeaponEquipStatePolicy.h"
#include "physics-interaction/input/BareFistGesturePolicy.h"

#include <cstdio>
#include <limits>

namespace
{
    enum class TestWeaponType : std::uint8_t
    {
        Gun = 9,
        Grenade = 10,
        Mine = 11,
    };

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
    {
        using namespace rock::bare_fist_gesture;
        // Exercise complete physical cycles, including releases that occur
        // between game-frame observations and leave the buttons held again.
        std::uint64_t cycle = 2;
        cycle = observe(cycle, true, true, false, false);
        ok &= expectEqual("held-at-start fists require release", capture(cycle), Capture::Rearming);
        cycle = observe(cycle, true, false, true, false);
        cycle = observe(cycle, true, false, false, false);
        ok &= expectEqual("partial chord cannot capture fists", capture(cycle), Capture::Idle);
        cycle = observe(cycle, true, true, false, false);
        State state{};
        ok &= expectEqual("capture starts qualification without draw", update(state, {cycle, true, false, 0.1f}), Action::None);
        ok &= expectEqual("short fist hold does not draw", update(state, {cycle, true, false, 0.9f}), Action::None);
        ok &= expectEqual("full second draws fists once", update(state, {cycle, true, false, 0.11f}), Action::Draw);
        ok &= expectEqual("drawing waits for native readiness", update(state, {cycle, true, false, 0.1f}), Action::None);
        ok &= expectEqual("unobserved draw has no active fists", state.phase, Phase::Drawing);
        static_cast<void>(update(state, {cycle, true, true, 0.1f}));
        ok &= expectEqual("observed unarmed draw enables fists", state.phase, Phase::Active);
        const auto firstCycle = cycle;
        cycle = observe(cycle, true, true, false, true);
        ok &= expectEqual("release and repress cancels captured chord", capture(cycle), Capture::Draining);
        ok &= expectEqual("first release ends active fists", update(state, {cycle, true, true, 0.01f}), Action::Cancel);
        ok &= expectEqual("held tail stays consumed", observe(cycle, true, false, false, false), cycle);
        cycle = observe(cycle, true, false, true, false);
        cycle = observe(cycle, true, true, false, false);
        ok &= expectTrue("fresh capture has new identity", cycle != firstCycle);
        static_cast<void>(update(state, {cycle, true, false, 0.1f}));
        ok &= expectEqual("new cycle cannot reuse qualification", state.phase, Phase::Qualifying);
        ok &= expectEqual("occupancy or gameplay loss cancels qualification", update(state, {cycle, false, false, 1.0f}), Action::Cancel);
        static_cast<void>(update(state, {cycle, true, false, 0.0f}));
        static_cast<void>(update(state, {cycle, true, false, 1.0f}));
        ok &= expectEqual("failed draw retires instead of retrying", update(state, {cycle, true, false, 2.0f}), Action::Cancel);
        state = {Phase::Active, firstCycle, 0};
        ok &= expectEqual("new capture cannot inherit old attack permission", update(state, {cycle, true, true, 0.0f}), Action::Cancel);
        ok &= expectEqual("stale samples cancel held capture", capture(observe(cycle, false, true, false, false)), Capture::Draining);
        ok &= expectFalse("reserved fist input blocks force grab", isAvailable({.inputReserved = true}));
        state = {};
        static_cast<void>(update(state, {cycle, true, false, 0.0f, 2.0f}));
        ok &= expectEqual("custom timer holds qualification past one second",
            update(state, {cycle, true, false, 1.0f, 2.0f}), Action::None);
        ok &= expectEqual("shorter reloaded timer cannot complete the current hold",
            update(state, {cycle, true, false, 0.5f, 0.25f}), Action::None);
        ok &= expectEqual("current hold completes at its captured timer",
            update(state, {cycle, true, false, 0.5f, 0.25f}), Action::Draw);
        static_cast<void>(update(state, {cycle, true, true, 0.0f, 0.25f}));
        ok &= expectEqual("disabling eligibility ends an active Rocky session",
            update(state, {cycle, false, true, 0.0f, 0.25f}), Action::Cancel);
        static_cast<void>(update(state, {cycle, true, false, 0.0f, 0.25f}));
        ok &= expectEqual("next hold uses the new fractional timer",
            update(state, {cycle, true, false, 0.25f, 0.25f}), Action::Draw);
    }
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

    using rock::loose_throwable_policy::DetonationMode;
    using rock::loose_throwable_policy::classifyDetonationMode;
    using rock::loose_throwable_policy::isSupportedWeaponType;
    using rock::loose_throwable_policy::isWithinProximity;
    using rock::loose_throwable_policy::preservesReferenceAfterDetonation;
    ok &= expectTrue("grenade is a supported throwable", isSupportedWeaponType(TestWeaponType::Grenade, TestWeaponType::Grenade, TestWeaponType::Mine));
    ok &= expectTrue("mine is a supported throwable", isSupportedWeaponType(TestWeaponType::Mine, TestWeaponType::Grenade, TestWeaponType::Mine));
    ok &= expectFalse("gun is not a supported throwable", isSupportedWeaponType(TestWeaponType::Gun, TestWeaponType::Grenade, TestWeaponType::Mine));
    ok &= expectEqual("generic grenade keeps timed fuse", classifyDetonationMode(TestWeaponType::Grenade, TestWeaponType::Grenade, TestWeaponType::Mine, false, true, 0.0f), DetonationMode::TimedFuse);
    ok &= expectEqual("Molotov uses impact", classifyDetonationMode(TestWeaponType::Grenade, TestWeaponType::Grenade, TestWeaponType::Mine, true, true, 0.0f), DetonationMode::Impact);
    ok &= expectEqual("placed mine uses proximity", classifyDetonationMode(TestWeaponType::Mine, TestWeaponType::Grenade, TestWeaponType::Mine, false, true, 100.0f), DetonationMode::Proximity);
    ok &= expectEqual("projectile-style mine uses impact", classifyDetonationMode(TestWeaponType::Mine, TestWeaponType::Grenade, TestWeaponType::Mine, false, true, 0.0f), DetonationMode::Impact);
    ok &= expectEqual("mine with invalid proximity fails closed", classifyDetonationMode(TestWeaponType::Mine, TestWeaponType::Grenade, TestWeaponType::Mine, false, true, (std::numeric_limits<float>::quiet_NaN)()), DetonationMode::Unsupported);
    ok &= expectEqual("throwable without explosion fails closed", classifyDetonationMode(TestWeaponType::Mine, TestWeaponType::Grenade, TestWeaponType::Mine, false, false, 0.0f), DetonationMode::Unsupported);
    ok &= expectTrue("pickup impact throwable remains recoverable", preservesReferenceAfterDetonation(DetonationMode::Impact, rock::loose_throwable_policy::kProjectileCanBePickedUp, false));
    ok &= expectFalse("authored placed-object recovery consumes source prop", preservesReferenceAfterDetonation(DetonationMode::Impact, rock::loose_throwable_policy::kProjectileCanBePickedUp, true));
    ok &= expectFalse("explosive impact throwable is consumed", preservesReferenceAfterDetonation(DetonationMode::Impact, 0, false));
    ok &= expectTrue("actor inside mine radius triggers", isWithinProximity(100.0f, 60.0f, 60.0f, 0.0f));
    ok &= expectFalse("actor outside mine radius does not trigger", isWithinProximity(100.0f, 80.0f, 80.0f, 0.0f));

    using rock::physics_body_classifier::BodyClassificationInput;
    using rock::physics_body_classifier::BodyMotionType;
    using rock::physics_body_classifier::InteractionMode;
    using rock::physics_body_classifier::classifyBody;
    BodyClassificationInput projectileLayerThrowable{
        .bodyId = 1,
        .motionId = 1,
        .layer = rock::collision_layer_policy::FO4_LAYER_PROJECTILE,
        .motionType = BodyMotionType::Dynamic,
        .targetKind = rock::grab_target::Kind::LooseObject,
    };
    ok &= expectFalse("organic projectile-layer object remains blocked", classifyBody(projectileLayerThrowable, InteractionMode::ActiveGrab).accepted);
    projectileLayerThrowable.allowProjectileLayerForExactTarget = true;
    ok &= expectTrue("exact quick-draw projectile-layer body is admitted", classifyBody(projectileLayerThrowable, InteractionMode::ActiveGrab).accepted);
    ok &= expectFalse("projectile-layer passive push remains blocked", classifyBody(projectileLayerThrowable, InteractionMode::PassivePush).accepted);
    projectileLayerThrowable.targetKind = rock::grab_target::Kind::ActorEquipment;
    ok &= expectFalse("projectile exception cannot escape loose-object target", classifyBody(projectileLayerThrowable, InteractionMode::ActiveGrab).accepted);

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
    using rock::held_weapon_equip_state_policy::isShoulderStashedPresentationState;
    using rock::held_weapon_equip_state_policy::shouldRearmTrigger;
    using rock::held_weapon_equip_state_policy::shouldSubmitDrawFollowup;
    using rock::held_weapon_equip_state_policy::shouldSubmitSheatheFollowup;
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
    ok &= expectFalse("sheathed native weapon rejects duplicate sheathe", shouldSubmitSheatheFollowup(0));
    ok &= expectTrue("want-draw native weapon accepts sheathe reversal", shouldSubmitSheatheFollowup(1));
    ok &= expectTrue("drawing native weapon accepts sheathe reversal", shouldSubmitSheatheFollowup(2));
    ok &= expectTrue("drawn native weapon accepts sheathe", shouldSubmitSheatheFollowup(3));
    ok &= expectTrue("want-sheathe native weapon accepts final sheathe", shouldSubmitSheatheFollowup(4));
    ok &= expectFalse("sheathing native weapon rejects duplicate sheathe", shouldSubmitSheatheFollowup(5));
    ok &= expectFalse("unknown native weapon state rejects sheathe", shouldSubmitSheatheFollowup(6));
    ok &= expectTrue("sheathed presentation is shoulder retrievable", isShoulderStashedPresentationState(0));
    ok &= expectFalse("want-draw presentation is not shoulder retrievable", isShoulderStashedPresentationState(1));
    ok &= expectFalse("drawing presentation is not shoulder retrievable", isShoulderStashedPresentationState(2));
    ok &= expectFalse("drawn presentation is not shoulder retrievable", isShoulderStashedPresentationState(3));
    ok &= expectTrue("want-sheathe presentation is shoulder retrievable", isShoulderStashedPresentationState(4));
    ok &= expectTrue("sheathing presentation is shoulder retrievable", isShoulderStashedPresentationState(5));
    ok &= expectFalse("unknown presentation is not shoulder retrievable", isShoulderStashedPresentationState(6));
    ok &= expectTrue("last known native weapon state is valid", isValidNativeWeaponState(5));
    ok &= expectFalse("state outside the FO4VR weapon enum is invalid", isValidNativeWeaponState(6));

    ForceGrabReservations reservations;
    ok &= expectFalse("invalid API hand cannot reserve", reservations.reserve(static_cast<RockProviderHand>(99), 11, 100));
    ok &= expectFalse("auto hand rejects missing owner", reservations.reserve(RockProviderHand::None, 0, 100));
    ok &= expectTrue("auto hand reserves both hands", reservations.reserve(RockProviderHand::None, 11, 100));
    ok &= expectTrue("auto hand reserves right", reservations.isReserved(RockProviderHand::Right));
    ok &= expectTrue("auto hand reserves left", reservations.isReserved(RockProviderHand::Left));
    ok &= expectFalse("auto hand blocks overtaking request", reservations.reserve(RockProviderHand::Left, 22, 201));
    reservations.release(22, 100);
    ok &= expectTrue("different owner cannot release auto hand", reservations.matches(11, 100));
    reservations.release(11, 100);
    ok &= expectFalse("auto hand completion releases both", reservations.isReserved(RockProviderHand::None));
    ok &= expectTrue("first right API force grab reserves", reservations.reserve(RockProviderHand::Right, 11, 101));
    ok &= expectFalse("auto hand cannot overtake reserved right", reservations.reserve(RockProviderHand::None, 22, 200));
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
