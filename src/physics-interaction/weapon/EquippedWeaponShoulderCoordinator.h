#pragma once

#include "physics-interaction/body/BodyZone.h"

#include <cstdint>

/*
 * Shoulder stash gesture for the primary equipped weapon. The hand carrying
 * the weapon taps (toggle grab) or holds and releases (hold grab) inside the
 * HMD-relative back volume, and the weapon returns to the inventory. The
 * coordinator only decides the gesture; the frame owner executes the stash.
 * A stashed weapon leaves the equipment, so its ownership key changes and
 * the coordinator resets with the still-held buttons blocked until release.
 */
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
        StashSubmitted,
        WaitForGestureEnd,
    };

    enum class Action : std::uint8_t
    {
        None = 0,
        SubmitStash,
    };

    enum class StashInputMode : std::uint8_t
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
        StashTap,
        HoldRelease,
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
        StashInputMode stashInputMode{ StashInputMode::Tap };
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
        std::uint64_t weaponOwnershipKey{ 0 };
        NativePresentation presentation{ NativePresentation::Invalid };
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
        case Phase::StashSubmitted:
            return "stash-submitted";
        case Phase::WaitForGestureEnd:
            return "wait-gesture-end";
        }
        return "unknown";
    }

    [[nodiscard]] inline constexpr const char* actionName(const Action action) noexcept
    {
        return action == Action::SubmitStash ? "submit-stash" : "none";
    }

    [[nodiscard]] inline constexpr StashInputMode resolveStashInputMode(
        const bool immersiveWeaponsEnabled,
        const bool toggleGrabEnabled) noexcept
    {
        return immersiveWeaponsEnabled && !toggleGrabEnabled ?
            StashInputMode::HoldRelease : StashInputMode::Tap;
    }

    [[nodiscard]] inline constexpr const char* stashInputModeName(
        const StashInputMode mode) noexcept
    {
        return mode == StashInputMode::HoldRelease ?
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
        case Reason::StashTap:
            return "stash-tap";
        case Reason::HoldRelease:
            return "hold-release";
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
            return input.presentation == NativePresentation::StableDrawn ?
                Phase::DrawnReady : Phase::NativeTransitionBlocked;
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

        inline constexpr void beginAction(
            RuntimeState& state,
            Decision& decision,
            const Reason reason,
            const Hand hand,
            const GestureFrame& gesture,
            const DetectorDecision& detector) noexcept
        {
            state.activeAction = Action::SubmitStash;
            state.actionHand = hand;
            state.actionGestureSerial = gesture.serial;
            state.phase = Phase::StashSubmitted;
            clearArm(state);

            decision.action = Action::SubmitStash;
            decision.gestureAction = Action::SubmitStash;
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

        // A new weapon (the stashed one left, or another equipped) never
        // inherits a held button: it must be released first.
        if (state.weaponOwnershipKey != input.weaponOwnershipKey) {
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

        bool pendingOrConsumed = false;
        if ((state.phase == Phase::DrawnReady ||
                state.phase == Phase::DrawnShoulderArmed) &&
            input.presentation != NativePresentation::StableDrawn) {
            detail::clearArm(state);
            state.phase = Phase::NativeTransitionBlocked;
            pendingOrConsumed = true;
        }

        switch (state.phase) {
        case Phase::StashSubmitted:
        case Phase::WaitForGestureEnd:
            // The stash leaves within the frame it is accepted; until the
            // identity changes (or a refused stash's gesture ends) the
            // gesture stays consumed and never becomes a drop or a grab.
            pendingOrConsumed = true;
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

            if (state.phase == Phase::WaitForGestureEnd &&
                !detail::gestureStillActive(
                    state,
                    state.actionHand,
                    state.actionGestureSerial)) {
                state.phase = detail::stablePhaseFor(input);
                detail::clearAction(state);
            }
            return decision;
        }

        if (state.phase == Phase::DrawnReady) {
            const Hand carryHand = input.left.carriesWeapon ?
                Hand::Left : input.right.carriesWeapon ?
                Hand::Right : Hand::None;
            if (carryHand != Hand::None) {
                const auto& handInput = detail::inputFor(input, carryHand);
                const auto& gesture = carryHand == Hand::Left ?
                    leftGesture : rightGesture;
                if (!handInput.disabled && handInput.eligible &&
                    gesture.valid && !gesture.blocked) {
                    if (handInput.stashInputMode == StashInputMode::Tap &&
                        handInput.button.pressed &&
                        handInput.detector.candidate) {
                        detail::beginAction(
                            state,
                            decision,
                            Reason::StashTap,
                            carryHand,
                            gesture,
                            handInput.detector);
                    } else if (handInput.stashInputMode ==
                                   StashInputMode::HoldRelease &&
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
                handInput.stashInputMode != StashInputMode::HoldRelease) {
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

    // A refused stash keeps the weapon; its gesture is consumed until it ends.
    inline constexpr void reportStashResult(
        RuntimeState& state,
        const bool accepted) noexcept
    {
        if (state.phase != Phase::StashSubmitted || accepted) {
            return;
        }
        state.phase = Phase::WaitForGestureEnd;
    }
}
