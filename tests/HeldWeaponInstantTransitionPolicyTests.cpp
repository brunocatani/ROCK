#include "physics-interaction/native/HeldWeaponInstantTransitionPolicy.h"

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
