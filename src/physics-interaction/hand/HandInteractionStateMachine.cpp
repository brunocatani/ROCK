#include "physics-interaction/hand/HandInteractionStateMachine.h"

namespace frik::rock
{
    namespace
    {
        constexpr std::uint32_t kClearAllEffects = transitionEffectMask(
            HandTransitionEffect::ClearSelection, HandTransitionEffect::ClearHeldRuntime, HandTransitionEffect::ClearFingerPose);

        constexpr std::uint32_t kReleaseAllEffects = transitionEffectMask(HandTransitionEffect::ReleaseHeld,
            HandTransitionEffect::ClearSelection,
            HandTransitionEffect::ClearHeldRuntime,
            HandTransitionEffect::ClearFingerPose);

        [[nodiscard]] constexpr bool isSelectionUpdateState(HandState state)
        {
            return state == HandState::Idle || state == HandState::SelectedClose || state == HandState::SelectedFar;
        }

        [[nodiscard]] constexpr bool isCommitCandidateState(HandState state)
        {
            return state == HandState::SelectedClose || state == HandState::SelectedFar || state == HandState::SelectionLocked;
        }

        [[nodiscard]] constexpr bool isReleaseCandidateState(HandState state)
        {
            return state == HandState::HeldInit || state == HandState::HeldBody || state == HandState::Pulled || state == HandState::SelectionLocked ||
                   state == HandState::SelectedClose || state == HandState::SelectedFar;
        }

        [[nodiscard]] constexpr HandTransitionResult accept(HandState next, std::uint32_t effects, const char* reason)
        {
            return HandTransitionResult{ .accepted = true, .next = next, .effects = effects, .reason = reason };
        }

        [[nodiscard]] constexpr HandTransitionResult reject(HandState current, const char* reason)
        {
            return HandTransitionResult{ .accepted = false, .next = current, .effects = 0, .reason = reason };
        }
    }

    HandTransitionResult evaluateHandTransition(const HandTransitionRequest& request)
    {
        const auto current = request.current;

        switch (request.event) {
        case HandInteractionEvent::Initialize:
            return accept(HandState::Idle, kClearAllEffects, "initialize");

        case HandInteractionEvent::SelectionFoundClose:
            if (isSelectionUpdateState(current)) {
                return accept(HandState::SelectedClose, 0, "selectionFoundClose");
            }
            break;

        case HandInteractionEvent::SelectionFoundFar:
            if (isSelectionUpdateState(current)) {
                return accept(HandState::SelectedFar, 0, "selectionFoundFar");
            }
            break;

        case HandInteractionEvent::SelectionLost:
            if (current == HandState::SelectedClose || current == HandState::SelectedFar) {
                return accept(HandState::Idle,
                    transitionEffectMask(HandTransitionEffect::ClearSelection, HandTransitionEffect::RememberDeselect),
                    "selectionLost");
            }
            break;

        case HandInteractionEvent::LockFarSelection:
            if (current == HandState::SelectedFar) {
                return accept(HandState::SelectionLocked, transitionEffectMask(HandTransitionEffect::LockSelection), "lockFarSelection");
            }
            break;

        case HandInteractionEvent::BeginPull:
            if (current == HandState::SelectionLocked) {
                return accept(HandState::Pulled, transitionEffectMask(HandTransitionEffect::StartPull), "beginPull");
            }
            break;

        case HandInteractionEvent::PullArrivedClose:
            if (current == HandState::Pulled) {
                return accept(HandState::SelectedClose, transitionEffectMask(HandTransitionEffect::RestorePulledCollision), "pullArrivedClose");
            }
            break;

        case HandInteractionEvent::BeginGrabCommit:
            if (isCommitCandidateState(current)) {
                return accept(current, transitionEffectMask(HandTransitionEffect::CommitGrab), "beginGrabCommit");
            }
            break;

        case HandInteractionEvent::GrabCommitSucceeded:
            if (isCommitCandidateState(current)) {
                return accept(HandState::HeldInit, 0, "grabCommitSucceeded");
            }
            break;

        case HandInteractionEvent::HeldFadeComplete:
            if (current == HandState::HeldInit) {
                return accept(HandState::HeldBody, 0, "heldFadeComplete");
            }
            break;

        case HandInteractionEvent::ReleaseRequested:
            if (isReleaseCandidateState(current)) {
                return accept(HandState::Idle, kReleaseAllEffects, "releaseRequested");
            }
            break;

        case HandInteractionEvent::ObjectInvalidated:
            if (isReleaseCandidateState(current)) {
                return accept(HandState::Idle, kReleaseAllEffects, "objectInvalidated");
            }
            break;

        case HandInteractionEvent::WorldInvalidated:
            if (current != HandState::Idle) {
                return accept(
                    HandState::Idle, kReleaseAllEffects | transitionEffectMask(HandTransitionEffect::ExitTwoHandAuthority), "worldInvalidated");
            }
            break;

        case HandInteractionEvent::BeginOtherHandTransfer:
            if (current == HandState::SelectedClose || current == HandState::HeldBody) {
                return accept(HandState::GrabFromOtherHand, 0, "beginOtherHandTransfer");
            }
            break;

        case HandInteractionEvent::CompleteOtherHandTransfer:
            if (current == HandState::GrabFromOtherHand) {
                return accept(HandState::HeldInit, transitionEffectMask(HandTransitionEffect::CommitGrab), "completeOtherHandTransfer");
            }
            break;

        case HandInteractionEvent::BeginTwoHandSelection:
            if (current == HandState::Idle || current == HandState::SelectedClose) {
                return accept(HandState::SelectedTwoHand, 0, "beginTwoHandSelection");
            }
            break;

        case HandInteractionEvent::BeginTwoHandHold:
            if (current == HandState::SelectedTwoHand) {
                return accept(HandState::HeldTwoHanded, transitionEffectMask(HandTransitionEffect::EnterTwoHandAuthority), "beginTwoHandHold");
            }
            break;

        case HandInteractionEvent::EndTwoHandHold:
            if (current == HandState::HeldTwoHanded || current == HandState::SelectedTwoHand) {
                return accept(HandState::Idle,
                    transitionEffectMask(HandTransitionEffect::ExitTwoHandAuthority, HandTransitionEffect::ClearFingerPose),
                    "endTwoHandHold");
            }
            break;
        }

        return reject(current, "transitionRejected");
    }

}
