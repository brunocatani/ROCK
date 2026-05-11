#include "physics-interaction/hand/HandInteractionStateMachine.h"
#include "physics-interaction/hand/HandSelection.h"

#include <cstdio>
#include <cstdint>
#include <initializer_list>

namespace
{
    using rock::HandInteractionEvent;
    using rock::HandState;
    using rock::HandTransitionEffect;
    using rock::HandTransitionRequest;

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

    bool expectState(const char* label, HandState actual, HandState expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected state %u got %u\n", label, static_cast<unsigned>(expected), static_cast<unsigned>(actual));
        return false;
    }

    bool hasEffect(std::uint32_t effects, HandTransitionEffect effect)
    {
        return (effects & static_cast<std::uint32_t>(effect)) != 0;
    }

    bool expectEffect(const char* label, std::uint32_t effects, HandTransitionEffect effect)
    {
        if (hasEffect(effects, effect)) {
            return true;
        }

        std::printf("%s missing effect 0x%08X in 0x%08X\n", label, static_cast<unsigned>(effect), effects);
        return false;
    }

    bool expectNoEffect(const char* label, std::uint32_t effects, HandTransitionEffect effect)
    {
        if (!hasEffect(effects, effect)) {
            return true;
        }

        std::printf("%s unexpected effect 0x%08X in 0x%08X\n", label, static_cast<unsigned>(effect), effects);
        return false;
    }

    HandTransitionRequest request(HandState current, HandInteractionEvent event)
    {
        return HandTransitionRequest{ .current = current, .event = event };
    }

    bool expectAccepted(const char* label, HandState current, HandInteractionEvent event, HandState next, std::uint32_t requiredEffects = 0)
    {
        const auto result = rock::evaluateHandTransition(request(current, event));
        bool ok = true;
        ok &= expectTrue(label, result.accepted);
        ok &= expectState(label, result.next, next);
        ok &= expectTrue("accepted transition has a reason", result.reason && result.reason[0] != '\0');
        ok &= expectTrue("accepted transition contains requested effects", (result.effects & requiredEffects) == requiredEffects);
        return ok;
    }

    bool expectRejected(const char* label, HandState current, HandInteractionEvent event)
    {
        const auto result = rock::evaluateHandTransition(request(current, event));
        bool ok = true;
        ok &= expectFalse(label, result.accepted);
        ok &= expectState(label, result.next, current);
        ok &= expectTrue("rejected transition has a reason", result.reason && result.reason[0] != '\0');
        ok &= expectTrue("rejected transition has no effects", result.effects == 0);
        return ok;
    }
}

int main()
{
    using namespace rock;

    bool ok = true;

    const auto clearAll = static_cast<std::uint32_t>(HandTransitionEffect::ClearSelection) |
                          static_cast<std::uint32_t>(HandTransitionEffect::ClearHeldRuntime) |
                          static_cast<std::uint32_t>(HandTransitionEffect::ClearFingerPose);
    const auto releaseAll = static_cast<std::uint32_t>(HandTransitionEffect::ReleaseHeld) |
                            static_cast<std::uint32_t>(HandTransitionEffect::ClearSelection) |
                            static_cast<std::uint32_t>(HandTransitionEffect::ClearHeldRuntime) |
                            static_cast<std::uint32_t>(HandTransitionEffect::ClearFingerPose);
    const auto enterGameplay = static_cast<std::uint32_t>(HandTransitionEffect::EnterGameplayScaffold);
    const auto exitGameplay = static_cast<std::uint32_t>(HandTransitionEffect::ExitGameplayScaffold);

    ok &= expectAccepted("initialize from idle", HandState::Idle, HandInteractionEvent::Initialize, HandState::Idle, clearAll);
    ok &= expectAccepted("initialize from held body", HandState::HeldBody, HandInteractionEvent::Initialize, HandState::Idle, clearAll);

    ok &= expectAccepted("idle finds close selection", HandState::Idle, HandInteractionEvent::SelectionFoundClose, HandState::SelectedClose);
    ok &= expectAccepted("close refreshes close selection", HandState::SelectedClose, HandInteractionEvent::SelectionFoundClose, HandState::SelectedClose);
    ok &= expectAccepted("far switches to close selection", HandState::SelectedFar, HandInteractionEvent::SelectionFoundClose, HandState::SelectedClose);
    ok &= expectAccepted("idle finds far selection", HandState::Idle, HandInteractionEvent::SelectionFoundFar, HandState::SelectedFar);
    ok &= expectAccepted("close switches to far selection", HandState::SelectedClose, HandInteractionEvent::SelectionFoundFar, HandState::SelectedFar);
    ok &= expectAccepted("far refreshes far selection", HandState::SelectedFar, HandInteractionEvent::SelectionFoundFar, HandState::SelectedFar);

    auto result = evaluateHandTransition(request(HandState::SelectedClose, HandInteractionEvent::SelectionLost));
    ok &= expectTrue("close selection can be lost", result.accepted);
    ok &= expectState("close selection lost returns idle", result.next, HandState::Idle);
    ok &= expectEffect("selection lost clears selection", result.effects, HandTransitionEffect::ClearSelection);
    ok &= expectEffect("selection lost remembers deselect", result.effects, HandTransitionEffect::RememberDeselect);

    result = evaluateHandTransition(request(HandState::SelectedFar, HandInteractionEvent::LockFarSelection));
    ok &= expectTrue("far selection can lock", result.accepted);
    ok &= expectState("far selection locks", result.next, HandState::SelectionLocked);
    ok &= expectEffect("far selection lock records lock effect", result.effects, HandTransitionEffect::LockSelection);

    ok &= expectAccepted("locked selection begins pull",
        HandState::SelectionLocked,
        HandInteractionEvent::BeginPull,
        HandState::Pulled,
        static_cast<std::uint32_t>(HandTransitionEffect::StartPull));
    ok &= expectAccepted("pulled object arrives close",
        HandState::Pulled,
        HandInteractionEvent::PullArrivedClose,
        HandState::SelectedClose,
        static_cast<std::uint32_t>(HandTransitionEffect::RestorePulledCollision));

    ok &= expectAccepted("locked selection begins pre-pull item scaffold",
        HandState::SelectionLocked,
        HandInteractionEvent::BeginPrePullItem,
        HandState::PrePullItem,
        static_cast<std::uint32_t>(HandTransitionEffect::StartPull) | enterGameplay);
    ok &= expectAccepted("close selection begins pre-grab item scaffold",
        HandState::SelectedClose,
        HandInteractionEvent::BeginPreGrabItem,
        HandState::PreGrabItem,
        static_cast<std::uint32_t>(HandTransitionEffect::LockSelection) | enterGameplay);
    ok &= expectAccepted("idle begins external grab scaffold",
        HandState::Idle,
        HandInteractionEvent::BeginExternalGrab,
        HandState::GrabExternal,
        enterGameplay);
    ok &= expectAccepted("close selection begins loot-other-hand scaffold",
        HandState::SelectedClose,
        HandInteractionEvent::BeginLootOtherHand,
        HandState::LootOtherHand,
        static_cast<std::uint32_t>(HandTransitionEffect::LockSelection) | enterGameplay);
    ok &= expectAccepted("spawned pre-grab item can commit",
        HandState::PreGrabItem,
        HandInteractionEvent::SpawnedItemReady,
        HandState::HeldInit,
        static_cast<std::uint32_t>(HandTransitionEffect::CommitGrab) | exitGameplay);
    ok &= expectAccepted("spawned pre-pull item can become pulled",
        HandState::PrePullItem,
        HandInteractionEvent::SpawnedItemReady,
        HandState::Pulled,
        static_cast<std::uint32_t>(HandTransitionEffect::StartPull) | exitGameplay);
    ok &= expectAccepted("loot-other-hand transitions through pre-grab item",
        HandState::LootOtherHand,
        HandInteractionEvent::CompleteLoot,
        HandState::PreGrabItem,
        enterGameplay);

    ok &= expectAccepted("close begins grab commit",
        HandState::SelectedClose,
        HandInteractionEvent::BeginGrabCommit,
        HandState::SelectedClose,
        static_cast<std::uint32_t>(HandTransitionEffect::CommitGrab));
    ok &= expectAccepted("far begins grab commit",
        HandState::SelectedFar,
        HandInteractionEvent::BeginGrabCommit,
        HandState::SelectedFar,
        static_cast<std::uint32_t>(HandTransitionEffect::CommitGrab));
    ok &= expectAccepted("locked begins grab commit",
        HandState::SelectionLocked,
        HandInteractionEvent::BeginGrabCommit,
        HandState::SelectionLocked,
        static_cast<std::uint32_t>(HandTransitionEffect::CommitGrab));

    ok &= expectAccepted("close grab commit succeeds", HandState::SelectedClose, HandInteractionEvent::GrabCommitSucceeded, HandState::HeldInit);
    ok &= expectAccepted("far grab commit succeeds", HandState::SelectedFar, HandInteractionEvent::GrabCommitSucceeded, HandState::HeldInit);
    ok &= expectAccepted("locked grab commit succeeds", HandState::SelectionLocked, HandInteractionEvent::GrabCommitSucceeded, HandState::HeldInit);
    ok &= expectAccepted("held init fade completes", HandState::HeldInit, HandInteractionEvent::HeldFadeComplete, HandState::HeldBody);

    ok &= expectAccepted("held body begins stash candidate",
        HandState::HeldBody,
        HandInteractionEvent::BeginStashCandidate,
        HandState::StashCandidate,
        enterGameplay);
    ok &= expectAccepted("held body begins consume candidate",
        HandState::HeldBody,
        HandInteractionEvent::BeginConsumeCandidate,
        HandState::ConsumeCandidate,
        enterGameplay);
    ok &= expectRejected("held init cannot begin stash candidate before fade completes", HandState::HeldInit, HandInteractionEvent::BeginStashCandidate);
    ok &= expectRejected("held init cannot begin consume candidate before fade completes", HandState::HeldInit, HandInteractionEvent::BeginConsumeCandidate);
    ok &= expectRejected("stash candidate cannot switch directly to consume candidate", HandState::StashCandidate, HandInteractionEvent::BeginConsumeCandidate);
    ok &= expectRejected("consume candidate cannot switch directly to stash candidate", HandState::ConsumeCandidate, HandInteractionEvent::BeginStashCandidate);
    ok &= expectAccepted("stash candidate can cancel back to held",
        HandState::StashCandidate,
        HandInteractionEvent::CancelGameplayCandidate,
        HandState::HeldBody,
        exitGameplay);
    ok &= expectAccepted("consume candidate can cancel back to held",
        HandState::ConsumeCandidate,
        HandInteractionEvent::CancelGameplayCandidate,
        HandState::HeldBody,
        exitGameplay);
    ok &= expectAccepted("stash candidate commits as release",
        HandState::StashCandidate,
        HandInteractionEvent::CommitStash,
        HandState::Idle,
        releaseAll | exitGameplay);
    ok &= expectAccepted("consume candidate commits as release",
        HandState::ConsumeCandidate,
        HandInteractionEvent::CommitConsume,
        HandState::Idle,
        releaseAll | exitGameplay);

    for (const auto state : { HandState::HeldInit,
             HandState::HeldBody,
             HandState::Pulled,
             HandState::SelectionLocked,
             HandState::PreGrabItem,
             HandState::PrePullItem,
             HandState::GrabExternal,
             HandState::LootOtherHand,
             HandState::StashCandidate,
             HandState::ConsumeCandidate,
             HandState::SelectedClose,
             HandState::SelectedFar }) {
        ok &= expectAccepted("release requested returns idle", state, HandInteractionEvent::ReleaseRequested, HandState::Idle, releaseAll);
        ok &= expectAccepted("object invalidated returns idle", state, HandInteractionEvent::ObjectInvalidated, HandState::Idle, releaseAll);
    }

    const auto worldInvalidatedEffects = releaseAll | static_cast<std::uint32_t>(HandTransitionEffect::ExitTwoHandAuthority);
    for (const auto state :
        { HandState::HeldInit,
            HandState::HeldBody,
            HandState::Pulled,
            HandState::SelectionLocked,
            HandState::PreGrabItem,
            HandState::PrePullItem,
            HandState::GrabExternal,
            HandState::LootOtherHand,
            HandState::StashCandidate,
            HandState::ConsumeCandidate,
            HandState::SelectedClose,
            HandState::SelectedFar,
            HandState::HeldTwoHanded }) {
        ok &= expectAccepted("world invalidation returns non-idle state to idle", state, HandInteractionEvent::WorldInvalidated, HandState::Idle, worldInvalidatedEffects);
    }

    ok &= expectAccepted("close selection begins other hand transfer", HandState::SelectedClose, HandInteractionEvent::BeginOtherHandTransfer, HandState::GrabFromOtherHand);
    ok &= expectAccepted("held body begins other hand transfer", HandState::HeldBody, HandInteractionEvent::BeginOtherHandTransfer, HandState::GrabFromOtherHand);
    ok &= expectAccepted("other hand transfer completes",
        HandState::GrabFromOtherHand,
        HandInteractionEvent::CompleteOtherHandTransfer,
        HandState::HeldInit,
        static_cast<std::uint32_t>(HandTransitionEffect::CommitGrab));

    ok &= expectAccepted("idle begins two-hand selection", HandState::Idle, HandInteractionEvent::BeginTwoHandSelection, HandState::SelectedTwoHand);
    ok &= expectAccepted("close begins two-hand selection", HandState::SelectedClose, HandInteractionEvent::BeginTwoHandSelection, HandState::SelectedTwoHand);
    ok &= expectAccepted("two-hand selection begins hold",
        HandState::SelectedTwoHand,
        HandInteractionEvent::BeginTwoHandHold,
        HandState::HeldTwoHanded,
        static_cast<std::uint32_t>(HandTransitionEffect::EnterTwoHandAuthority));

    result = evaluateHandTransition(request(HandState::HeldTwoHanded, HandInteractionEvent::EndTwoHandHold));
    ok &= expectTrue("held two-hand hold can end", result.accepted);
    ok &= expectState("held two-hand hold returns idle", result.next, HandState::Idle);
    ok &= expectEffect("ending two-hand hold exits authority", result.effects, HandTransitionEffect::ExitTwoHandAuthority);
    ok &= expectEffect("ending two-hand hold clears finger pose", result.effects, HandTransitionEffect::ClearFingerPose);
    ok &= expectNoEffect("ending two-hand hold does not release held object", result.effects, HandTransitionEffect::ReleaseHeld);

    ok &= expectAccepted("selected two-hand can end", HandState::SelectedTwoHand, HandInteractionEvent::EndTwoHandHold, HandState::Idle);

    ok &= expectRejected("idle cannot begin pull", HandState::Idle, HandInteractionEvent::BeginPull);
    ok &= expectRejected("close cannot complete held fade", HandState::SelectedClose, HandInteractionEvent::HeldFadeComplete);
    ok &= expectRejected("held body ignores far selection updates", HandState::HeldBody, HandInteractionEvent::SelectionFoundFar);
    ok &= expectRejected("pulled cannot begin two-hand hold", HandState::Pulled, HandInteractionEvent::BeginTwoHandHold);

    ok &= expectTrue("held init is holding", isHoldingState(HandState::HeldInit));
    ok &= expectTrue("held body is holding", isHoldingState(HandState::HeldBody));
    ok &= expectTrue("stash candidate still owns held object", isHoldingState(HandState::StashCandidate));
    ok &= expectTrue("consume candidate still owns held object", isHoldingState(HandState::ConsumeCandidate));
    ok &= expectFalse("idle is not holding", isHoldingState(HandState::Idle));

    ok &= expectTrue("idle can update selection", canUpdateSelectionFromState(HandState::Idle));
    ok &= expectTrue("close can update selection", canUpdateSelectionFromState(HandState::SelectedClose));
    ok &= expectTrue("far can update selection", canUpdateSelectionFromState(HandState::SelectedFar));
    ok &= expectFalse("held body cannot update selection", canUpdateSelectionFromState(HandState::HeldBody));

    ok &= expectTrue("close can process selected state", canProcessSelectedState(HandState::SelectedClose));
    ok &= expectTrue("far can process selected state", canProcessSelectedState(HandState::SelectedFar));
    ok &= expectFalse("idle cannot process selected state", canProcessSelectedState(HandState::Idle));

    ok &= expectTrue("locked selection is exclusive", hasExclusiveObjectSelection(HandState::SelectionLocked));
    ok &= expectTrue("pulled selection is exclusive", hasExclusiveObjectSelection(HandState::Pulled));
    ok &= expectTrue("held init is exclusive", hasExclusiveObjectSelection(HandState::HeldInit));
    ok &= expectTrue("held body is exclusive", hasExclusiveObjectSelection(HandState::HeldBody));
    ok &= expectTrue("pre-grab item is exclusive", hasExclusiveObjectSelection(HandState::PreGrabItem));
    ok &= expectTrue("pre-pull item is exclusive", hasExclusiveObjectSelection(HandState::PrePullItem));
    ok &= expectTrue("loot-other-hand is exclusive", hasExclusiveObjectSelection(HandState::LootOtherHand));
    ok &= expectTrue("stash candidate is exclusive", hasExclusiveObjectSelection(HandState::StashCandidate));
    ok &= expectFalse("close hover selection is not exclusive", hasExclusiveObjectSelection(HandState::SelectedClose));
    ok &= expectFalse("far hover selection is not exclusive", hasExclusiveObjectSelection(HandState::SelectedFar));

    ok &= expectTrue("pre-grab item is gameplay scaffold", isGameplayScaffoldState(HandState::PreGrabItem));
    ok &= expectTrue("consume candidate is gameplay scaffold", isGameplayScaffoldState(HandState::ConsumeCandidate));
    ok &= expectFalse("held body is not gameplay scaffold", isGameplayScaffoldState(HandState::HeldBody));

    auto* refA = reinterpret_cast<RE::TESObjectREFR*>(static_cast<std::uintptr_t>(0x1000u));
    auto* refB = reinterpret_cast<RE::TESObjectREFR*>(static_cast<std::uintptr_t>(0x2000u));
    OtherHandSelectionContext lockedPeer{};
    lockedPeer.exclusiveRef = refA;
    ok &= expectTrue("peer locked ref blocks same object", lockedPeer.blocksReference(refA));
    ok &= expectFalse("peer locked ref does not block different object", lockedPeer.blocksReference(refB));

    OtherHandSelectionContext heldPeer{};
    heldPeer.shareableHeldRef = refA;
    ok &= expectTrue("peer held ref is marked shareable", heldPeer.allowsSharedHeldReference(refA));
    ok &= expectFalse("peer held ref does not block shared object", heldPeer.blocksReference(refA));

    return ok ? 0 : 1;
}
