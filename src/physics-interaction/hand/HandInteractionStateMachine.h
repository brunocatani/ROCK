#pragma once

#include "physics-interaction/hand/HandLifecycle.h"

#include <cstdint>

namespace rock
{
    /*
     * Hand state transitions are kept as a pure policy layer so the current grab,
     * pull, and selection behavior can be preserved while removing scattered
     * direct state writes from runtime code. HIGGS uses an explicit hand lifecycle
     * but mixes transitions with side effects; ROCK keeps side effects in Hand and
     * makes transition legality independently testable.
     */
    enum class HandInteractionEvent : std::uint8_t
    {
        Initialize,
        SelectionFoundClose,
        SelectionFoundFar,
        SelectionLost,
        LockFarSelection,
        BeginPull,
        PullArrivedClose,
        BeginGrabCommit,
        GrabCommitSucceeded,
        HeldFadeComplete,
        ReleaseRequested,
        ObjectInvalidated,
        WorldInvalidated,
        BeginOtherHandTransfer,
        CompleteOtherHandTransfer,
        BeginTwoHandSelection,
        BeginTwoHandHold,
        EndTwoHandHold,
    };

    enum class HandTransitionEffect : std::uint32_t
    {
        None = 0,
        ClearSelection = 1u << 0,
        RememberDeselect = 1u << 1,
        LockSelection = 1u << 2,
        StartPull = 1u << 3,
        RestorePulledCollision = 1u << 4,
        CommitGrab = 1u << 5,
        ReleaseHeld = 1u << 6,
        ClearHeldRuntime = 1u << 7,
        ClearFingerPose = 1u << 8,
        EnterTwoHandAuthority = 1u << 9,
        ExitTwoHandAuthority = 1u << 10,
    };

    struct HandTransitionRequest
    {
        HandState current = HandState::Idle;
        HandInteractionEvent event = HandInteractionEvent::Initialize;
        bool hasSelection = false;
        bool selectionIsFar = false;
        bool hasHeldBody = false;
        bool otherHandTransferPending = false;
        bool twoHandCandidateValid = false;
    };

    struct HandTransitionResult
    {
        bool accepted = false;
        HandState next = HandState::Idle;
        std::uint32_t effects = 0;
        const char* reason = "";
    };

    [[nodiscard]] constexpr std::uint32_t transitionEffectMask(HandTransitionEffect effect)
    {
        return static_cast<std::uint32_t>(effect);
    }

    [[nodiscard]] constexpr std::uint32_t transitionEffectMask(HandTransitionEffect first, HandTransitionEffect second)
    {
        return transitionEffectMask(first) | transitionEffectMask(second);
    }

    template <class... Rest>
    [[nodiscard]] constexpr std::uint32_t transitionEffectMask(HandTransitionEffect first, HandTransitionEffect second, Rest... rest)
    {
        return transitionEffectMask(first) | transitionEffectMask(second, rest...);
    }

    [[nodiscard]] HandTransitionResult evaluateHandTransition(const HandTransitionRequest& request);

    [[nodiscard]] constexpr bool isHoldingState(HandState state)
    {
        return state == HandState::HeldInit || state == HandState::HeldBody;
    }

    [[nodiscard]] constexpr bool canUpdateSelectionFromState(HandState state)
    {
        return state == HandState::Idle || state == HandState::SelectedClose || state == HandState::SelectedFar;
    }

    [[nodiscard]] constexpr bool canProcessSelectedState(HandState state)
    {
        return state == HandState::SelectedClose || state == HandState::SelectedFar;
    }

    [[nodiscard]] constexpr bool hasExclusiveObjectSelection(HandState state)
    {
        return state == HandState::SelectionLocked || state == HandState::Pulled || state == HandState::HeldInit || state == HandState::HeldBody;
    }
}
