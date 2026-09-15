#include "physics-interaction/weapon/WeaponInventoryStackSelectionPolicy.h"

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
    using namespace rock::weapon_inventory_stack_selection_policy;

    bool ok = true;

    const StackWitness requested{ 0x1000, 0xA000, 1 };
    ok &= expect("equip may rebuild or move a stack while preserving the exact instance",
        matchesEquippedStack(requested, { 0x2000, 0xA000, 1 }));
    ok &= expect("a reused stack node must not admit a different weapon instance",
        !matchesEquippedStack(requested, { 0x1000, 0xB000, 1 }));
    ok &= expect("missing equipped stack must fail closed",
        !matchesEquippedStack(requested, {}));
    ok &= expect("base-data stack identity survives a list reorder",
        matchesEquippedStack({ 0x1000, 0, 1 }, { 0x1000, 0, 1 }));
    ok &= expect("base-data weapons cannot borrow another stack",
        !matchesEquippedStack({ 0x1000, 0, 1 }, { 0x2000, 0, 1 }));

    Snapshot duplicateBefore{};
    duplicateBefore.count = 2;
    duplicateBefore.stacks[0] = StackWitness{ .stackAddress = 0x1000, .instanceDataAddress = 0xA000, .count = 1 };
    duplicateBefore.stacks[1] = StackWitness{ .stackAddress = 0x2000, .instanceDataAddress = 0xB000, .count = 1 };

    Snapshot newStackAfter = duplicateBefore;
    newStackAfter.count = 3;
    newStackAfter.stacks[2] = StackWitness{ .stackAddress = 0x3000, .instanceDataAddress = 0xC000, .count = 1 };
    const auto newStack = selectTransferredStack(duplicateBefore, newStackAfter, 0xDEAD);
    ok &= expect("a newly inserted stack must win among same-base duplicates",
        newStack.found &&
            newStack.postIndex == 2 &&
            newStack.evidence == Evidence::NewStack);

    Snapshot mergedAfter = duplicateBefore;
    mergedAfter.stacks[1].count = 2;
    mergedAfter.stacks[1].instanceDataAddress = 0xC000;
    const auto merged = selectTransferredStack(duplicateBefore, mergedAfter, 0xDEAD);
    ok &= expect("the only increased stack must identify a merged pickup",
        merged.found &&
            merged.postIndex == 1 &&
            merged.evidence == Evidence::ExistingStackCountIncrease);

    const auto exact = selectTransferredStack(duplicateBefore, newStackAfter, 0xB000);
    ok &= expect("an exact surviving instance pointer must remain authoritative",
        exact.found &&
            exact.postIndex == 1 &&
            exact.evidence == Evidence::ExactInstance);

    Snapshot ambiguousAfter = duplicateBefore;
    ambiguousAfter.stacks[0].count = 2;
    ambiguousAfter.stacks[1].count = 2;
    const auto ambiguous = selectTransferredStack(duplicateBefore, ambiguousAfter, 0);
    ok &= expect("multiple mutated stacks must fail closed",
        !ambiguous.found &&
            ambiguous.mutationCandidateCount == 2 &&
            ambiguous.evidence == Evidence::AmbiguousMutation);

    Snapshot soleBefore{};
    Snapshot soleAfter{};
    soleAfter.count = 1;
    soleAfter.stacks[0] = StackWitness{ .stackAddress = 0x4000, .instanceDataAddress = 0xD000, .count = 1 };
    const auto sole = selectTransferredStack(soleBefore, soleAfter, 0);
    ok &= expect("a sole post-transfer stack must remain a safe fallback",
        sole.found &&
            sole.postIndex == 0 &&
            sole.evidence == Evidence::NewStack);

    const auto soleUnchanged = selectTransferredStack(soleAfter, soleAfter, 0);
    ok &= expect("a sole unchanged stack remains an unambiguous fallback",
        soleUnchanged.found &&
            soleUnchanged.postIndex == 0 &&
            soleUnchanged.evidence == Evidence::SolePostTransferStack);

    Snapshot rebuiltBefore{};
    rebuiltBefore.count = 1;
    rebuiltBefore.stacks[0] = StackWitness{ .stackAddress = 0x5000, .instanceDataAddress = 0xE000, .count = 1 };
    Snapshot rebuiltAfter{};
    rebuiltAfter.count = 2;
    rebuiltAfter.stacks[0] = StackWitness{ .stackAddress = 0x6000, .instanceDataAddress = 0xE000, .count = 1 };
    rebuiltAfter.stacks[1] = StackWitness{ .stackAddress = 0x7000, .instanceDataAddress = 0xF000, .count = 1 };
    const auto rebuilt = selectTransferredStack(rebuiltBefore, rebuiltAfter, 0);
    ok &= expect("a stable instance pointer must survive inventory stack-node rebuilding",
        rebuilt.found &&
            rebuilt.postIndex == 1 &&
            rebuilt.evidence == Evidence::NewStack);

    Snapshot incompleteBefore = duplicateBefore;
    incompleteBefore.complete = false;
    const auto incomplete = selectTransferredStack(incompleteBefore, duplicateBefore, 0);
    ok &= expect("a capacity-truncated differential must fail closed",
        !incomplete.found &&
            incomplete.evidence == Evidence::SnapshotCapacityExceeded);

    const auto incompleteExact = selectTransferredStack(incompleteBefore, duplicateBefore, 0xA000);
    ok &= expect("an exact pointer remains safe even if the differential was truncated",
        incompleteExact.found &&
            incompleteExact.postIndex == 0 &&
            incompleteExact.evidence == Evidence::ExactInstance);

    return ok ? 0 : 1;
}
