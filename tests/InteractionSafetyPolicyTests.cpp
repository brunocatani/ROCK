#include "physics-interaction/api/InteractionCommandPolicy.h"
#include "physics-interaction/core/ForceGrabPolicy.h"
#include "physics-interaction/weapon/BareFistGuardPolicy.h"

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

    return ok ? 0 : 1;
}
