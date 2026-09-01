#pragma once

#include "physics-interaction/body/BodyZone.h"

#include <cstdint>

namespace rock::equipped_weapon_shoulder
{
    enum class Hand : std::uint8_t
    {
        None = 0,
        Right,
        Left,
    };

    enum class NativePresentation : std::uint8_t
    {
        Invalid = 0,
        StableSheathed,
        WantDraw,
        Drawing,
        StableDrawn,
        WantSheathe,
        Sheathing,
    };

    enum class Phase : std::uint8_t
    {
        Disabled = 0,
        NativeTransitionBlocked,
        DrawnReady,
        DrawnShoulderArmed,
        SheathSubmitted,
        StoredWaitForGestureEnd,
        StoredReady,
        RetrieveSubmitted,
        DrawnWaitForGestureEnd,
    };

    enum class Action : std::uint8_t
    {
        None = 0,
        SubmitSheath,
        SubmitRetrieve,
    };

    enum class SheathInputMode : std::uint8_t
    {
        Tap = 0,
        HoldRelease,
    };

    enum class Reason : std::uint8_t
    {
        None = 0,
        Disabled,
        InputBlocked,
        InvalidIdentity,
        InvalidPresentation,
        NativeTransition,
        GestureConsumed,
        SheathTap,
        HoldRelease,
        RetrievalTap,
        RetrievalHeld,
    };

    struct ButtonState
    {
        bool held{ false };
        bool pressed{ false };
        bool released{ false };
    };

    struct DetectorDecision
    {
        bool candidate{ false };
        bool confirmed{ false };
        body_zone::BodyZoneKind zone{ body_zone::BodyZoneKind::Unknown };
        float confidence{ 0.0f };
    };

    struct HandInput
    {
        bool disabled{ false };
        bool eligible{ false };
        bool carriesWeapon{ false };
        ButtonState button{};
        DetectorDecision detector{};
    };

    struct FrameInput
    {
        bool enabled{ false };
        bool inputAllowed{ false };
        SheathInputMode sheathInputMode{ SheathInputMode::Tap };
        bool storedActive{ false };
        bool stashedByLeftHand{ false };
        std::uint64_t weaponOwnershipKey{ 0 };
        NativePresentation presentation{ NativePresentation::Invalid };
        body_zone::BodyZoneKind storedZone{ body_zone::BodyZoneKind::Unknown };
        HandInput right{};
        HandInput left{};
    };

    struct Decision
    {
        Action action{ Action::None };
        Action gestureAction{ Action::None };
        Reason reason{ Reason::None };
        Hand hand{ Hand::None };
        body_zone::BodyZoneKind zone{ body_zone::BodyZoneKind::Unknown };
        float confidence{ 0.0f };
        std::uint64_t gestureSerial{ 0 };
        bool consumeRightInput{ false };
        bool consumeLeftInput{ false };
        bool suppressEquippedDrop{ false };
    };

    struct GestureRuntime
    {
        bool active{ false };
        bool blockedUntilRelease{ false };
        std::uint64_t serial{ 0 };
    };

    struct RuntimeState
    {
        Phase phase{ Phase::Disabled };
        std::uint64_t weaponOwnershipKey{ 0 };
        GestureRuntime rightGesture{};
        GestureRuntime leftGesture{};
        Action activeAction{ Action::None };
        Hand actionHand{ Hand::None };
        std::uint64_t actionGestureSerial{ 0 };
        Hand armedHand{ Hand::None };
        std::uint64_t armedGestureSerial{ 0 };
        body_zone::BodyZoneKind armedZone{ body_zone::BodyZoneKind::Unknown };
        float armedConfidence{ 0.0f };
    };

    struct GestureFrame
    {
        bool valid{ false };
        bool ended{ false };
        bool blocked{ false };
        std::uint64_t serial{ 0 };
    };

    [[nodiscard]] inline constexpr bool isLeft(const Hand hand) noexcept
    {
        return hand == Hand::Left;
    }

    [[nodiscard]] inline constexpr const char* handName(const Hand hand) noexcept
    {
        switch (hand) {
        case Hand::Right:
            return "right";
        case Hand::Left:
            return "left";
        case Hand::None:
            break;
        }
        return "none";
    }

    [[nodiscard]] inline constexpr const char* phaseName(const Phase phase) noexcept
    {
        switch (phase) {
        case Phase::Disabled:
            return "disabled";
        case Phase::NativeTransitionBlocked:
            return "native-transition-blocked";
        case Phase::DrawnReady:
            return "drawn-ready";
        case Phase::DrawnShoulderArmed:
            return "drawn-shoulder-armed";
        case Phase::SheathSubmitted:
            return "sheath-submitted";
        case Phase::StoredWaitForGestureEnd:
            return "stored-wait-gesture-end";
        case Phase::StoredReady:
            return "stored-ready";
        case Phase::RetrieveSubmitted:
            return "retrieve-submitted";
        case Phase::DrawnWaitForGestureEnd:
            return "drawn-wait-gesture-end";
        }
        return "unknown";
    }

    [[nodiscard]] inline constexpr const char* actionName(const Action action) noexcept
    {
        switch (action) {
        case Action::SubmitSheath:
            return "submit-sheath";
        case Action::SubmitRetrieve:
            return "submit-retrieve";
        case Action::None:
            break;
        }
        return "none";
    }

    [[nodiscard]] inline constexpr SheathInputMode resolveSheathInputMode(
        const bool immersiveWeaponsEnabled,
        const bool toggleGrabEnabled) noexcept
    {
        return immersiveWeaponsEnabled && !toggleGrabEnabled ?
            SheathInputMode::HoldRelease : SheathInputMode::Tap;
    }

    [[nodiscard]] inline constexpr const char* sheathInputModeName(
        const SheathInputMode mode) noexcept
    {
        return mode == SheathInputMode::HoldRelease ?
            "hold-release" : "tap";
    }

    [[nodiscard]] inline constexpr const char* reasonName(const Reason reason) noexcept
    {
        switch (reason) {
        case Reason::Disabled:
            return "disabled";
        case Reason::InputBlocked:
            return "input-blocked";
        case Reason::InvalidIdentity:
            return "invalid-identity";
        case Reason::InvalidPresentation:
            return "invalid-presentation";
        case Reason::NativeTransition:
            return "native-transition";
        case Reason::GestureConsumed:
            return "gesture-consumed";
        case Reason::SheathTap:
            return "sheath-tap";
        case Reason::HoldRelease:
            return "hold-release";
        case Reason::RetrievalTap:
            return "retrieval-tap";
        case Reason::RetrievalHeld:
            return "retrieval-held";
        case Reason::None:
            break;
        }
        return "none";
    }

    inline constexpr void reset(RuntimeState& state) noexcept
    {
        state = {};
    }

    namespace detail
    {
        [[nodiscard]] inline constexpr const HandInput& inputFor(
            const FrameInput& input,
            const Hand hand) noexcept
        {
            return hand == Hand::Left ? input.left : input.right;
        }

        [[nodiscard]] inline constexpr GestureFrame prepareGesture(
            GestureRuntime& runtime,
            const ButtonState& button) noexcept
        {
            GestureFrame frame{};
            if (runtime.blockedUntilRelease) {
                frame.blocked = true;
                if (!button.held) {
                    runtime.blockedUntilRelease = false;
                }
                return frame;
            }

            const bool beginsNow =
                !runtime.active && (button.pressed || button.held);
            if (beginsNow) {
                ++runtime.serial;
                if (runtime.serial == 0) {
                    ++runtime.serial;
                }
                runtime.active = true;
            }

            if (!runtime.active) {
                return frame;
            }

            frame.valid = true;
            frame.serial = runtime.serial;
            frame.ended = button.released ||
                          (!button.held && !button.pressed);
            return frame;
        }

        inline constexpr void finishGesture(
            GestureRuntime& runtime,
            const GestureFrame& frame) noexcept
        {
            if (frame.valid && frame.ended) {
                runtime.active = false;
            }
        }

        [[nodiscard]] inline constexpr bool gestureStillActive(
            const RuntimeState& state,
            const Hand hand,
            const std::uint64_t serial) noexcept
        {
            if (hand == Hand::None || serial == 0) {
                return false;
            }
            const auto& gesture = hand == Hand::Left ?
                state.leftGesture : state.rightGesture;
            return gesture.active && gesture.serial == serial;
        }

        inline constexpr void consumeActionGesture(
            const RuntimeState& state,
            const GestureFrame& right,
            const GestureFrame& left,
            Decision& decision) noexcept
        {
            const auto& frame = state.actionHand == Hand::Left ? left : right;
            if (state.actionHand == Hand::None ||
                !frame.valid ||
                frame.serial != state.actionGestureSerial) {
                return;
            }
            decision.consumeLeftInput = state.actionHand == Hand::Left;
            decision.consumeRightInput = state.actionHand == Hand::Right;
            decision.suppressEquippedDrop = true;
            decision.gestureAction = state.activeAction;
            decision.hand = state.actionHand;
            decision.gestureSerial = state.actionGestureSerial;
            decision.reason = Reason::GestureConsumed;
        }

        [[nodiscard]] inline constexpr Phase stablePhaseFor(
            const FrameInput& input) noexcept
        {
            if (input.storedActive &&
                input.presentation == NativePresentation::StableSheathed) {
                return Phase::StoredReady;
            }
            if (!input.storedActive &&
                input.presentation == NativePresentation::StableDrawn) {
                return Phase::DrawnReady;
            }
            return Phase::NativeTransitionBlocked;
        }

        inline constexpr void clearAction(RuntimeState& state) noexcept
        {
            state.activeAction = Action::None;
            state.actionHand = Hand::None;
            state.actionGestureSerial = 0;
        }

        inline constexpr void clearArm(RuntimeState& state) noexcept
        {
            state.armedHand = Hand::None;
            state.armedGestureSerial = 0;
            state.armedZone = body_zone::BodyZoneKind::Unknown;
            state.armedConfidence = 0.0f;
        }

        [[nodiscard]] inline constexpr bool sameStoredShoulder(
            const FrameInput& input,
            const HandInput& hand) noexcept
        {
            return hand.detector.candidate &&
                   hand.detector.zone == input.storedZone;
        }

        [[nodiscard]] inline constexpr Hand selectRetrievalHand(
            const FrameInput& input,
            const GestureFrame& rightGesture,
            const GestureFrame& leftGesture,
            Reason& outReason) noexcept
        {
            const auto eligible = [&](const HandInput& hand,
                                      const GestureFrame& gesture) {
                if (hand.disabled || !hand.eligible || !gesture.valid ||
                    gesture.blocked || !sameStoredShoulder(input, hand)) {
                    return false;
                }
                const bool tap = hand.button.pressed;
                const bool heldPull =
                    input.sheathInputMode == SheathInputMode::HoldRelease &&
                    hand.button.held &&
                    hand.detector.confirmed;
                return tap || heldPull;
            };

            const bool rightEligible = eligible(input.right, rightGesture);
            const bool leftEligible = eligible(input.left, leftGesture);
            if (rightEligible != leftEligible) {
                const Hand selected = leftEligible ? Hand::Left : Hand::Right;
                const auto& selectedInput = inputFor(input, selected);
                outReason = selectedInput.button.pressed ?
                    Reason::RetrievalTap : Reason::RetrievalHeld;
                return selected;
            }
            if (!rightEligible) {
                return Hand::None;
            }

            if (input.right.detector.confidence !=
                input.left.detector.confidence) {
                const Hand selected =
                    input.left.detector.confidence >
                            input.right.detector.confidence ?
                        Hand::Left : Hand::Right;
                const auto& selectedInput = inputFor(input, selected);
                outReason = selectedInput.button.pressed ?
                    Reason::RetrievalTap : Reason::RetrievalHeld;
                return selected;
            }

            const Hand selected = input.stashedByLeftHand ?
                Hand::Left : Hand::Right;
            const auto& selectedInput = inputFor(input, selected);
            outReason = selectedInput.button.pressed ?
                Reason::RetrievalTap : Reason::RetrievalHeld;
            return selected;
        }

        inline constexpr void beginAction(
            RuntimeState& state,
            Decision& decision,
            const Action action,
            const Reason reason,
            const Hand hand,
            const GestureFrame& gesture,
            const DetectorDecision& detector) noexcept
        {
            state.activeAction = action;
            state.actionHand = hand;
            state.actionGestureSerial = gesture.serial;
            state.phase = action == Action::SubmitSheath ?
                Phase::SheathSubmitted : Phase::RetrieveSubmitted;
            clearArm(state);

            decision.action = action;
            decision.gestureAction = action;
            decision.reason = reason;
            decision.hand = hand;
            decision.zone = detector.zone;
            decision.confidence = detector.confidence;
            decision.gestureSerial = gesture.serial;
            decision.consumeLeftInput = hand == Hand::Left;
            decision.consumeRightInput = hand == Hand::Right;
            decision.suppressEquippedDrop = true;
        }
    }

    [[nodiscard]] inline constexpr Decision advance(
        RuntimeState& state,
        const FrameInput& input) noexcept
    {
        Decision decision{};
        if (!input.enabled) {
            reset(state);
            decision.reason = Reason::Disabled;
            return decision;
        }
        if (!input.inputAllowed) {
            reset(state);
            decision.reason = Reason::InputBlocked;
            return decision;
        }
        if (input.weaponOwnershipKey == 0) {
            reset(state);
            decision.reason = Reason::InvalidIdentity;
            return decision;
        }
        if (input.presentation == NativePresentation::Invalid) {
            reset(state);
            decision.reason = Reason::InvalidPresentation;
            return decision;
        }

        const bool identityChanged =
            state.weaponOwnershipKey != input.weaponOwnershipKey;
        if (identityChanged) {
            reset(state);
            state.weaponOwnershipKey = input.weaponOwnershipKey;
            state.rightGesture.blockedUntilRelease =
                input.right.button.held && !input.right.button.pressed;
            state.leftGesture.blockedUntilRelease =
                input.left.button.held && !input.left.button.pressed;
            state.phase = detail::stablePhaseFor(input);
        }

        const GestureFrame rightGesture = detail::prepareGesture(
            state.rightGesture,
            input.right.button);
        const GestureFrame leftGesture = detail::prepareGesture(
            state.leftGesture,
            input.left.button);
        if (state.activeAction != Action::None) {
            decision.gestureAction = state.activeAction;
            decision.suppressEquippedDrop = true;
            const auto& actionFrame = state.actionHand == Hand::Left ?
                leftGesture : rightGesture;
            if (state.actionHand != Hand::None &&
                actionFrame.valid &&
                actionFrame.serial != state.actionGestureSerial) {
                decision.consumeLeftInput =
                    state.actionHand == Hand::Left;
                decision.consumeRightInput =
                    state.actionHand == Hand::Right;
                decision.hand = state.actionHand;
                decision.gestureSerial = actionFrame.serial;
                decision.reason = Reason::GestureConsumed;
                // A second press during the native transition is part of the
                // existing transaction, never a new opposite-direction tap.
                // Extend ownership through that newer cycle's release.
                state.actionGestureSerial = actionFrame.serial;
            }
        }

        bool pendingOrConsumed = false;
        if ((state.phase == Phase::DrawnReady ||
                state.phase == Phase::DrawnShoulderArmed) &&
            (input.storedActive ||
                input.presentation != NativePresentation::StableDrawn)) {
            detail::clearArm(state);
            state.phase = detail::stablePhaseFor(input);
            pendingOrConsumed = true;
        } else if (state.phase == Phase::StoredReady &&
                   (!input.storedActive ||
                       input.presentation !=
                           NativePresentation::StableSheathed)) {
            state.phase = detail::stablePhaseFor(input);
            pendingOrConsumed = true;
        }

        switch (state.phase) {
        case Phase::SheathSubmitted:
            pendingOrConsumed = true;
            if (input.storedActive &&
                input.presentation == NativePresentation::StableSheathed) {
                if (detail::gestureStillActive(
                        state,
                        state.actionHand,
                        state.actionGestureSerial)) {
                    state.phase = Phase::StoredWaitForGestureEnd;
                } else {
                    state.phase = Phase::StoredReady;
                    detail::clearAction(state);
                }
            }
            break;
        case Phase::RetrieveSubmitted:
            pendingOrConsumed = true;
            if (!input.storedActive &&
                input.presentation == NativePresentation::StableDrawn) {
                if (detail::gestureStillActive(
                        state,
                        state.actionHand,
                        state.actionGestureSerial)) {
                    state.phase = Phase::DrawnWaitForGestureEnd;
                } else {
                    state.phase = Phase::DrawnReady;
                    detail::clearAction(state);
                }
            }
            break;
        case Phase::StoredWaitForGestureEnd:
            pendingOrConsumed = true;
            if (!input.storedActive ||
                input.presentation !=
                    NativePresentation::StableSheathed) {
                state.phase = detail::stablePhaseFor(input);
                detail::clearAction(state);
            }
            break;
        case Phase::DrawnWaitForGestureEnd:
            pendingOrConsumed = true;
            if (input.storedActive ||
                input.presentation != NativePresentation::StableDrawn) {
                state.phase = detail::stablePhaseFor(input);
                detail::clearAction(state);
            }
            break;
        case Phase::NativeTransitionBlocked:
            pendingOrConsumed = true;
            state.phase = detail::stablePhaseFor(input);
            break;
        default:
            break;
        }

        if (pendingOrConsumed) {
            detail::consumeActionGesture(
                state,
                rightGesture,
                leftGesture,
                decision);
            if (decision.reason == Reason::None) {
                decision.reason = Reason::NativeTransition;
            }
            detail::finishGesture(state.rightGesture, rightGesture);
            detail::finishGesture(state.leftGesture, leftGesture);

            if (state.phase == Phase::StoredWaitForGestureEnd &&
                !detail::gestureStillActive(
                    state,
                    state.actionHand,
                    state.actionGestureSerial)) {
                state.phase = Phase::StoredReady;
                detail::clearAction(state);
            } else if (state.phase == Phase::DrawnWaitForGestureEnd &&
                       !detail::gestureStillActive(
                           state,
                           state.actionHand,
                           state.actionGestureSerial)) {
                state.phase = Phase::DrawnReady;
                detail::clearAction(state);
            }
            return decision;
        }

        if (state.phase == Phase::StoredReady) {
            Reason retrievalReason = Reason::None;
            const Hand retrievalHand = detail::selectRetrievalHand(
                input,
                rightGesture,
                leftGesture,
                retrievalReason);
            if (retrievalHand != Hand::None) {
                const auto& handInput = detail::inputFor(
                    input,
                    retrievalHand);
                const auto& gesture = retrievalHand == Hand::Left ?
                    leftGesture : rightGesture;
                detail::beginAction(
                    state,
                    decision,
                    Action::SubmitRetrieve,
                    retrievalReason,
                    retrievalHand,
                    gesture,
                    handInput.detector);
            }
        } else if (state.phase == Phase::DrawnReady) {
            const Hand carryHand = input.left.carriesWeapon ?
                Hand::Left : input.right.carriesWeapon ?
                Hand::Right : Hand::None;
            if (carryHand != Hand::None) {
                const auto& handInput = detail::inputFor(input, carryHand);
                const auto& gesture = carryHand == Hand::Left ?
                    leftGesture : rightGesture;
                if (!handInput.disabled && handInput.eligible &&
                    gesture.valid && !gesture.blocked) {
                    if (input.sheathInputMode == SheathInputMode::Tap &&
                        handInput.button.pressed &&
                        handInput.detector.candidate) {
                        detail::beginAction(
                            state,
                            decision,
                            Action::SubmitSheath,
                            Reason::SheathTap,
                            carryHand,
                            gesture,
                            handInput.detector);
                    } else if (input.sheathInputMode ==
                                   SheathInputMode::HoldRelease &&
                               handInput.button.held &&
                               handInput.detector.confirmed) {
                        state.phase = Phase::DrawnShoulderArmed;
                        state.armedHand = carryHand;
                        state.armedGestureSerial = gesture.serial;
                        state.armedZone = handInput.detector.zone;
                        state.armedConfidence =
                            handInput.detector.confidence;
                    }
                }
            }
        } else if (state.phase == Phase::DrawnShoulderArmed) {
            const Hand armedHand = state.armedHand;
            const auto& handInput = detail::inputFor(input, armedHand);
            const auto& gesture = armedHand == Hand::Left ?
                leftGesture : rightGesture;
            const bool sameGesture = gesture.valid &&
                gesture.serial == state.armedGestureSerial;
            if (!sameGesture || handInput.disabled ||
                !handInput.eligible || !handInput.carriesWeapon ||
                input.sheathInputMode != SheathInputMode::HoldRelease) {
                state.phase = Phase::DrawnReady;
                detail::clearArm(state);
            } else if (handInput.button.released) {
                const DetectorDecision armedDetector{
                    .candidate = true,
                    .confirmed = true,
                    .zone = state.armedZone,
                    .confidence = state.armedConfidence,
                };
                detail::beginAction(
                    state,
                    decision,
                    Action::SubmitSheath,
                    Reason::HoldRelease,
                    armedHand,
                    gesture,
                    armedDetector);
            } else if (gesture.ended) {
                state.phase = Phase::DrawnReady;
                detail::clearArm(state);
            }
        }

        detail::finishGesture(state.rightGesture, rightGesture);
        detail::finishGesture(state.leftGesture, leftGesture);
        return decision;
    }

    inline constexpr void reportExecutionResult(
        RuntimeState& state,
        const Action action,
        const bool accepted) noexcept
    {
        const bool matchingSheath =
            action == Action::SubmitSheath &&
            state.phase == Phase::SheathSubmitted;
        const bool matchingRetrieve =
            action == Action::SubmitRetrieve &&
            state.phase == Phase::RetrieveSubmitted;
        if (!matchingSheath && !matchingRetrieve) {
            return;
        }
        if (accepted) {
            return;
        }

        state.phase = matchingSheath ?
            Phase::DrawnWaitForGestureEnd :
            Phase::StoredWaitForGestureEnd;
    }
}
