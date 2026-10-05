#include "physics-interaction/native/HeldWeaponInstantTransitionPolicy.h"
#include "physics-interaction/native/NativeWeaponEquipAudio.h"
#include "physics-interaction/native/NativeWeaponAnimationAudio.h"

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
    using namespace rock::held_weapon_instant_transition_policy;

    constexpr std::uintptr_t drawReturn = 0x1000;
    constexpr std::uintptr_t sheatheReturn = 0x2000;
    bool ok = true;
    using rock::native_weapon_equip_audio::isPlayerSource;
    ok &= expect("weapon equip audio matches the player's position identity",
        isPlayerSource(0x1000, 0x10D0));
    ok &= expect("NPC and unrelated positions keep native playback",
        !isPlayerSource(0x1000, 0x20D0) && !isPlayerSource(0x1000, 0x1000));
    ok &= expect("missing player or position cannot suppress sound",
        !isPlayerSource(0, 0) && !isPlayerSource(0, 0xD0) && !isPlayerSource(0x1000, 0));
    using rock::native_weapon_equip_audio::isPlayerWeaponItem;
    ok &= expect("player weapon inventory sounds are suppressed",
        isPlayerWeaponItem(0x1000, 0x1000, true));
    ok &= expect("other items, NPCs and missing players retain inventory audio",
        !isPlayerWeaponItem(0x1000, 0x1000, false) && !isPlayerWeaponItem(0x1000, 0x2000, true) &&
        !isPlayerWeaponItem(0, 0, true));
    using rock::native_weapon_animation_audio::isEquipClip;
    using rock::native_weapon_animation_audio::isSoundStart;
    ok &= expect("authored equip clips qualify regardless of weapon directory or case",
        isEquipClip("Meshes/Actors/Character/Animations/RU556Anims/WPNEquip.hkx") &&
        isEquipClip("Pistol\\WPNEQUIPFAST.HKX") && isEquipClip("WPNUnEquip.hkx"));
    // Exact clip identities in the 2026-10-05 runtime trace. These events
    // reached the filter but previously escaped its HKX/WPN-only matcher.
    ok &= expect("captured firearm fast-equip clip suppresses its sound",
        isEquipClip("Animations\\44Pistol\\WPNEquipFast.hkt"));
    ok &= expect("captured firearm unequip clip suppresses its sound",
        isEquipClip("Animations\\44Pistol\\WPNUnEquip.hkt"));
    ok &= expect("captured melee fast-equip clip suppresses its sound",
        isEquipClip("Animations\\1HM\\EquipFast.hkt"));
    ok &= expect("regular and unprefixed equipment clips accept both native representations",
        isEquipClip("Animations/44Pistol/WPNEquip.hkt") && isEquipClip("Animations/1HM/Equip.hkt") &&
        isEquipClip("Animations/1HM/UnEquip.hkt") && isEquipClip("Equip.hkx") &&
        isEquipClip("EquipFast.hkx") && isEquipClip("UnEquip.hkx") && isEquipClip("WPNEQUIP.HKT"));
    ok &= expect("reload, fire, melee and unrelated equip names retain audio",
        !isEquipClip("WPNReload.hkx") && !isEquipClip("WPNFireSingleReady.hkx") &&
        !isEquipClip("WPNMelee.hkx") && !isEquipClip("WPNEquip.hkx/idle.hkx") &&
        !isEquipClip("NotWPNEquip.hkx") && !isEquipClip(""));
    ok &= expect("runtime clip matching keeps action and filename boundaries",
        !isEquipClip("Animations/1HM/Attack.hkt") && !isEquipClip("Animations/44Pistol/WPNReload.hkt") &&
        !isEquipClip("Animations/44Pistol/WPNFireSingleReady.hkt") && !isEquipClip("WPNMelee.hkt") &&
        !isEquipClip("EquipFurniture.hkt") && !isEquipClip("NotEquip.hkt") && !isEquipClip("Equip") &&
        !isEquipClip("WPNEquip.hkt.bak") && !isEquipClip("WPNEquip.txt") && !isEquipClip("WPNEquip.hkt/Idle.hkt"));
    ok &= expect("only sound start notifications qualify",
        isSoundStart("SoundPlay") && isSoundStart("soundplayat") &&
        !isSoundStart("SoundStop") && !isSoundStart("SoundRelease") &&
        !isSoundStart("weaponDraw") && !isSoundStart("weaponSheathe") && !isSoundStart("reloadComplete"));

    ok &= expect("inactive hook calls must pass through",
        classifyHookCall(false, true, true, drawReturn, drawReturn, sheatheReturn) ==
            HookDecision::PassThroughInactive);
    ok &= expect("another player must pass through and invalidate the scope",
        classifyHookCall(true, false, true, drawReturn, drawReturn, sheatheReturn) ==
            HookDecision::PassThroughPlayerMismatch);
    ok &= expect("only the verified equip draw caller may be suppressed",
        classifyHookCall(true, true, true, drawReturn, drawReturn, sheatheReturn) ==
            HookDecision::SuppressDraw);
    ok &= expect("only the verified conflict sheathe caller may be suppressed",
        classifyHookCall(true, true, false, sheatheReturn, drawReturn, sheatheReturn) ==
            HookDecision::SuppressSheathe);
    ok &= expect("a direction mismatch must pass through and invalidate the scope",
        classifyHookCall(true, true, false, drawReturn, drawReturn, sheatheReturn) ==
            HookDecision::PassThroughUnexpectedCaller);
    ok &= expect("an unknown scoped caller must pass through and invalidate the scope",
        classifyHookCall(true, true, true, 0x3000, drawReturn, sheatheReturn) ==
            HookDecision::PassThroughUnexpectedCaller);

    ActionTrace drawOnly{};
    recordAction(drawOnly, NativeAction::Draw);
    ok &= expect("one intercepted draw is a valid new-weapon equip trace",
        isValidEquipActionTrace(drawOnly));

    ActionTrace replacement{};
    recordAction(replacement, NativeAction::Sheathe);
    recordAction(replacement, NativeAction::Sheathe);
    recordAction(replacement, NativeAction::Draw);
    ok &= expect("bounded replacement sheathes followed by one draw are a valid equip trace",
        isValidEquipActionTrace(replacement) &&
            actionCount(replacement, NativeAction::Sheathe) == 2 &&
            actionCount(replacement, NativeAction::Draw) == 1);

    ActionTrace empty{};
    ok &= expect("an empty trace cannot validate an intercepted equip",
        !isValidEquipActionTrace(empty));

    ActionTrace sheatheOnly{};
    recordAction(sheatheOnly, NativeAction::Sheathe);
    ok &= expect("a sheathe-only trace cannot validate an intercepted equip",
        !isValidEquipActionTrace(sheatheOnly));

    ActionTrace drawNotLast{};
    recordAction(drawNotLast, NativeAction::Draw);
    recordAction(drawNotLast, NativeAction::Sheathe);
    ok &= expect("draw must be the final intercepted action",
        !isValidEquipActionTrace(drawNotLast));

    ActionTrace multipleDraws{};
    recordAction(multipleDraws, NativeAction::Draw);
    recordAction(multipleDraws, NativeAction::Draw);
    ok &= expect("multiple draw requests are ambiguous and rejected",
        !isValidEquipActionTrace(multipleDraws));

    ActionTrace faulted = drawOnly;
    faulted.unexpectedCaller = true;
    ok &= expect("an unexpected scoped caller invalidates an otherwise valid trace",
        !isValidEquipActionTrace(faulted));
    faulted = drawOnly;
    faulted.playerMismatch = true;
    ok &= expect("a scoped player mismatch invalidates the equip trace",
        !isValidEquipActionTrace(faulted));
    faulted = drawOnly;
    faulted.nestedScope = true;
    ok &= expect("a nested transaction invalidates the equip trace",
        !isValidEquipActionTrace(faulted));

    ActionTrace overflow{};
    for (std::size_t index = 0; index < kMaximumActionTraceEntries + 1; ++index) {
        recordAction(overflow, NativeAction::Sheathe);
    }
    ok &= expect("the fixed trace must fail closed on overflow",
        overflow.overflow && !isValidEquipActionTrace(overflow));

    return ok ? 0 : 1;
}
