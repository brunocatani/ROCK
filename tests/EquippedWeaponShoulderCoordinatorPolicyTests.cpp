#include "physics-interaction/weapon/EquippedWeaponShoulderCoordinator.h"

#include <cstdio>

namespace
{
    template <class T>
    bool expectEqual(const char* label, const T actual, const T expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s mismatch\n", label);
        return false;
    }

    bool expectTrue(const char* label, const bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, const bool value)
    {
        if (!value) {
            return true;
        }
        std::printf("%s expected false\n", label);
        return false;
    }
}

int main()
{
    using namespace rock::equipped_weapon_shoulder;
    using rock::body_zone::BodyZoneKind;

    bool ok = true;

    const auto rightCandidate = [](
                                    const bool confirmed = true,
                                    const float confidence = 0.8f) {
        return DetectorDecision{
            .candidate = true,
            .confirmed = confirmed,
            .zone = BodyZoneKind::RightShoulder,
            .confidence = confidence,
        };
    };
    const auto leftCandidate = [](
                                   const bool confirmed = true,
                                   const float confidence = 0.8f) {
        return DetectorDecision{
            .candidate = true,
            .confirmed = confirmed,
            .zone = BodyZoneKind::LeftShoulder,
            .confidence = confidence,
        };
    };
    const auto drawnInput = [](
                                const SheathInputMode sheathInputMode =
                                    SheathInputMode::Tap) {
        FrameInput input{
            .enabled = true,
            .inputAllowed = true,
            .weaponOwnershipKey = 0xAAu,
            .presentation = NativePresentation::StableDrawn,
        };
        input.right.sheathInputMode = sheathInputMode;
        input.left.sheathInputMode = sheathInputMode;
        input.right.eligible = true;
        input.right.carriesWeapon = true;
        return input;
    };
    const auto storedInput = [](
                                 const SheathInputMode sheathInputMode =
                                     SheathInputMode::Tap) {
        FrameInput input{
            .enabled = true,
            .inputAllowed = true,
            .storedActive = true,
            .weaponOwnershipKey = 0xAAu,
            .presentation = NativePresentation::StableSheathed,
            .storedZone = BodyZoneKind::RightShoulder,
        };
        input.right.sheathInputMode = sheathInputMode;
        input.left.sheathInputMode = sheathInputMode;
        input.right.eligible = true;
        return input;
    };

    ok &= expectEqual("immersive toggle mode uses tap sheath input",
        resolveSheathInputMode(true, true),
        SheathInputMode::Tap);
    ok &= expectEqual("immersive hold mode uses release sheath input",
        resolveSheathInputMode(true, false),
        SheathInputMode::HoldRelease);
    ok &= expectEqual("non-immersive toggle mode uses tap sheath input",
        resolveSheathInputMode(false, true),
        SheathInputMode::Tap);
    ok &= expectEqual("non-immersive hold setting still uses tap sheath input",
        resolveSheathInputMode(false, false),
        SheathInputMode::Tap);

    {
        RuntimeState state{};
        auto input = drawnInput(SheathInputMode::Tap);
        input.right.detector = rightCandidate(false);
        auto decision = advance(state, input);
        ok &= expectEqual("proximity alone emits no action",
            decision.action,
            Action::None);
        ok &= expectEqual("drawn proximity remains ready",
            state.phase,
            Phase::DrawnReady);
    }

    {
        RuntimeState state{};
        auto input = drawnInput(SheathInputMode::Tap);
        input.right.eligible = false;
        input.right.carriesWeapon = false;
        input.left.eligible = true;
        input.left.carriesWeapon = true;
        input.left.detector = leftCandidate(false);
        input.left.button = { .held = true, .pressed = true };
        const auto decision = advance(state, input);
        ok &= expectEqual("left tap mode submits sheath",
            decision.action,
            Action::SubmitSheath);
        ok &= expectEqual("left tap mode selects left carry hand",
            decision.hand,
            Hand::Left);
        ok &= expectTrue("left tap mode consumes left input",
            decision.consumeLeftInput);
        ok &= expectFalse("left tap mode does not consume right input",
            decision.consumeRightInput);
    }

    {
        RuntimeState state{};
        auto input = drawnInput(SheathInputMode::HoldRelease);
        input.right.eligible = false;
        input.right.carriesWeapon = false;
        input.left.eligible = true;
        input.left.carriesWeapon = true;
        input.left.detector = leftCandidate(true);
        input.left.button = { .held = true, .pressed = true };
        auto decision = advance(state, input);
        ok &= expectEqual("left hold mode arms in shoulder zone",
            state.phase,
            Phase::DrawnShoulderArmed);
        ok &= expectEqual("left hold mode does not sheath before release",
            decision.action,
            Action::None);

        input.left.button = { .released = true };
        decision = advance(state, input);
        ok &= expectEqual("left hold release submits sheath",
            decision.action,
            Action::SubmitSheath);
        ok &= expectEqual("left hold release selects left carry hand",
            decision.hand,
            Hand::Left);
        ok &= expectTrue("left hold release consumes left input",
            decision.consumeLeftInput);
    }

    {
        RuntimeState state{};
        auto input = drawnInput(SheathInputMode::Tap);
        input.right.detector = rightCandidate(false);
        input.right.button = {
            .held = true,
            .pressed = true,
        };
        auto decision = advance(state, input);
        ok &= expectEqual("toggle down submits one sheath",
            decision.action,
            Action::SubmitSheath);
        ok &= expectEqual("toggle sheath uses tap reason",
            decision.reason,
            Reason::SheathTap);
        ok &= expectEqual("toggle sheath owns the gesture as a sheath",
            decision.gestureAction,
            Action::SubmitSheath);
        ok &= expectTrue("toggle sheath consumes right input",
            decision.consumeRightInput);
        ok &= expectTrue("toggle sheath suppresses drop",
            decision.suppressEquippedDrop);
        const auto sheathGesture = decision.gestureSerial;
        reportExecutionResult(state, decision.action, true);

        input = storedInput(SheathInputMode::Tap);
        input.right.detector = rightCandidate(true);
        input.right.button = { .held = true };
        decision = advance(state, input);
        ok &= expectEqual("held sheath tap cannot retrieve",
            decision.action,
            Action::None);
        ok &= expectEqual("stable stored state waits for sheath gesture release",
            state.phase,
            Phase::StoredWaitForGestureEnd);
        ok &= expectTrue("held sheath tap remains consumed",
            decision.consumeRightInput);
        ok &= expectEqual("held sheath tap stays direction locked",
            decision.gestureAction,
            Action::SubmitSheath);
        ok &= expectEqual("held sheath tap keeps one gesture",
            decision.gestureSerial,
            sheathGesture);

        input.right.button = { .released = true };
        decision = advance(state, input);
        ok &= expectEqual("sheath release performs no retrieval",
            decision.action,
            Action::None);
        ok &= expectTrue("sheath release remains consumed",
            decision.consumeRightInput);
        ok &= expectEqual("sheath release rearms stored state",
            state.phase,
            Phase::StoredReady);
        ok &= expectEqual("sheath release clears transaction direction",
            state.activeAction,
            Action::None);

        input.right.button = {
            .held = true,
            .pressed = true,
        };
        decision = advance(state, input);
        ok &= expectEqual("fresh stored tap retrieves once",
            decision.action,
            Action::SubmitRetrieve);
        ok &= expectEqual("fresh stored tap uses retrieval reason",
            decision.reason,
            Reason::RetrievalTap);
        ok &= expectTrue("retrieval tap has a fresh gesture",
            decision.gestureSerial != sheathGesture);
        const auto retrievalGesture = decision.gestureSerial;
        reportExecutionResult(state, decision.action, true);

        input = drawnInput(SheathInputMode::Tap);
        input.right.detector = rightCandidate(true);
        input.right.button = { .held = true };
        decision = advance(state, input);
        ok &= expectEqual("held retrieval tap cannot sheathe",
            decision.action,
            Action::None);
        ok &= expectEqual("stable drawn state waits for retrieval gesture release",
            state.phase,
            Phase::DrawnWaitForGestureEnd);
        ok &= expectEqual("held retrieval retains its gesture",
            decision.gestureSerial,
            retrievalGesture);
        ok &= expectEqual("held retrieval stays direction locked",
            decision.gestureAction,
            Action::SubmitRetrieve);

        input.right.button = { .released = true };
        decision = advance(state, input);
        ok &= expectEqual("retrieval release performs no sheath",
            decision.action,
            Action::None);
        ok &= expectTrue("retrieval release remains consumed",
            decision.consumeRightInput);
        ok &= expectEqual("retrieval release rearms drawn state",
            state.phase,
            Phase::DrawnReady);
        ok &= expectEqual("retrieval release clears transaction direction",
            state.activeAction,
            Action::None);
    }

    {
        RuntimeState state{};
        auto input = drawnInput(SheathInputMode::Tap);
        input.right.detector = rightCandidate(false);
        input.right.button = {
            .held = true,
            .pressed = true,
        };
        auto decision = advance(state, input);
        const auto firstGesture = decision.gestureSerial;
        reportExecutionResult(state, decision.action, true);

        input = storedInput(SheathInputMode::Tap);
        input.presentation = NativePresentation::WantSheathe;
        input.right.detector = rightCandidate(true);
        input.right.button = { .released = true };
        decision = advance(state, input);
        ok &= expectEqual("first tap release cannot reverse during sheathing",
            decision.action,
            Action::None);

        input.right.button = {
            .held = true,
            .pressed = true,
        };
        decision = advance(state, input);
        ok &= expectEqual("second tap during sheathing emits no action",
            decision.action,
            Action::None);
        ok &= expectTrue("second tap during sheathing is consumed",
            decision.consumeRightInput);
        ok &= expectTrue("second tap receives a distinct gesture",
            decision.gestureSerial != firstGesture);
        const auto reentrantGesture = decision.gestureSerial;

        input.presentation = NativePresentation::StableSheathed;
        input.right.button = { .held = true };
        decision = advance(state, input);
        ok &= expectEqual("stable sheath cannot turn second tap into retrieval",
            decision.action,
            Action::None);
        ok &= expectEqual("stable sheath retains second tap ownership",
            decision.gestureSerial,
            reentrantGesture);
        ok &= expectEqual("stable sheath waits for second tap release",
            state.phase,
            Phase::StoredWaitForGestureEnd);

        input.right.button = { .released = true };
        decision = advance(state, input);
        ok &= expectEqual("second tap release only rearms after sheathing",
            decision.action,
            Action::None);
        ok &= expectEqual("second tap release returns stored ready",
            state.phase,
            Phase::StoredReady);
    }

    {
        RuntimeState state{};
        auto input = storedInput(SheathInputMode::Tap);
        input.right.button = {
            .held = true,
            .pressed = true,
        };
        auto decision = advance(state, input);
        ok &= expectEqual("toggle press outside stored shoulder does not retrieve",
            decision.action,
            Action::None);

        input.right.detector = rightCandidate(true);
        input.right.button = { .held = true };
        decision = advance(state, input);
        ok &= expectEqual("toggle hold entering stored shoulder is not a new tap",
            decision.action,
            Action::None);

        input.right.button = { .released = true };
        decision = advance(state, input);
        ok &= expectEqual("toggle release after outside press only rearms",
            decision.action,
            Action::None);

        input.right.button = {
            .held = true,
            .pressed = true,
        };
        decision = advance(state, input);
        ok &= expectEqual("fresh toggle tap inside stored shoulder retrieves",
            decision.action,
            Action::SubmitRetrieve);
    }

    {
        RuntimeState state{};
        auto input = drawnInput(SheathInputMode::HoldRelease);
        input.right.detector = rightCandidate(true);
        input.right.button = {
            .held = true,
            .pressed = true,
        };
        auto decision = advance(state, input);
        ok &= expectEqual("hold mode confirmed hold arms without sheathing",
            decision.action,
            Action::None);
        ok &= expectEqual("hold mode enters armed state",
            state.phase,
            Phase::DrawnShoulderArmed);

        input.right.detector = {};
        input.right.button = { .held = true };
        decision = advance(state, input);
        ok &= expectEqual("confirmed hold stays armed through release motion",
            state.phase,
            Phase::DrawnShoulderArmed);
        ok &= expectEqual("moving forward while held does not sheathe",
            decision.action,
            Action::None);

        input.right.button = { .released = true };
        decision = advance(state, input);
        ok &= expectEqual("armed hold release submits sheath despite release motion",
            decision.action,
            Action::SubmitSheath);
        ok &= expectEqual("hold release has explicit reason",
            decision.reason,
            Reason::HoldRelease);
    }

    {
        RuntimeState state{};
        auto input = drawnInput(SheathInputMode::Tap);
        input.right.button = {
            .held = true,
            .pressed = true,
        };
        auto decision = advance(state, input);
        ok &= expectEqual("toggle press outside detector does not sheath",
            decision.action,
            Action::None);

        input.right.detector = rightCandidate(true);
        input.right.button = { .released = true };
        decision = advance(state, input);
        ok &= expectEqual("toggle release entering detector does not complete a tap",
            decision.action,
            Action::None);
    }

    {
        RuntimeState state{};
        auto input = storedInput(SheathInputMode::HoldRelease);
        input.right.button = {
            .held = true,
            .pressed = true,
        };
        auto decision = advance(state, input);
        ok &= expectEqual("stored press outside detector does not retrieve",
            decision.action,
            Action::None);

        input.right.detector = rightCandidate(true);
        input.right.button = { .held = true };
        decision = advance(state, input);
        ok &= expectEqual("held grab retrieves after existing detector confirms",
            decision.action,
            Action::SubmitRetrieve);
        ok &= expectEqual("held grab retrieval has explicit reason",
            decision.reason,
            Reason::RetrievalHeld);
    }

    {
        RuntimeState state{};
        auto input = storedInput(SheathInputMode::HoldRelease);
        input.right.detector = rightCandidate(false);
        input.right.button = { .pressed = true };
        auto decision = advance(state, input);
        ok &= expectEqual("hold mode quick tap retrieves on button down",
            decision.action,
            Action::SubmitRetrieve);
        ok &= expectEqual("hold mode quick tap uses retrieval tap reason",
            decision.reason,
            Reason::RetrievalTap);
        reportExecutionResult(state, decision.action, true);

        input = drawnInput(SheathInputMode::HoldRelease);
        input.right.detector = rightCandidate(true);
        input.right.button = { .released = true };
        decision = advance(state, input);
        ok &= expectEqual("hold mode retrieval tap release cannot sheathe",
            decision.action,
            Action::None);
        ok &= expectTrue("hold mode retrieval tap release remains consumed",
            decision.consumeRightInput);
        ok &= expectEqual("hold mode retrieval tap release remains drawn ready",
            state.phase,
            Phase::DrawnReady);
    }

    {
        RuntimeState state{};
        auto input = storedInput(SheathInputMode::Tap);
        input.stashedByLeftHand = false;
        input.right.eligible = true;
        input.left.eligible = true;
        input.right.detector = rightCandidate(true, 0.55f);
        input.left.detector = DetectorDecision{
            .candidate = true,
            .confirmed = true,
            .zone = BodyZoneKind::RightShoulder,
            .confidence = 0.85f,
        };
        input.right.button = { .held = true, .pressed = true };
        input.left.button = { .held = true, .pressed = true };
        auto decision = advance(state, input);
        ok &= expectEqual("higher-confidence eligible hand wins retrieval",
            decision.hand,
            Hand::Left);
    }

    {
        RuntimeState state{};
        auto input = storedInput(SheathInputMode::Tap);
        input.stashedByLeftHand = true;
        input.right.eligible = true;
        input.left.eligible = true;
        input.right.detector = rightCandidate(true, 0.8f);
        input.left.detector = DetectorDecision{
            .candidate = true,
            .confirmed = true,
            .zone = BodyZoneKind::RightShoulder,
            .confidence = 0.8f,
        };
        input.right.button = { .held = true, .pressed = true };
        input.left.button = { .held = true, .pressed = true };
        auto decision = advance(state, input);
        ok &= expectEqual("retrieval confidence tie favors stashing hand",
            decision.hand,
            Hand::Left);
    }

    {
        RuntimeState state{};
        auto input = drawnInput(SheathInputMode::Tap);
        input.right.detector = rightCandidate(true);
        input.right.button = { .held = true, .pressed = true };
        auto decision = advance(state, input);
        reportExecutionResult(state, decision.action, false);
        ok &= expectEqual("rejected sheath waits for gesture end",
            state.phase,
            Phase::DrawnWaitForGestureEnd);

        input.right.button = { .held = true };
        decision = advance(state, input);
        ok &= expectEqual("rejected sheath hold cannot retry",
            decision.action,
            Action::None);
        ok &= expectTrue("rejected sheath hold stays consumed",
            decision.consumeRightInput);
        ok &= expectEqual("rejected sheath keeps sheath direction",
            decision.gestureAction,
            Action::SubmitSheath);
        ok &= expectTrue("rejected sheath keeps deferred drop suppressed",
            decision.suppressEquippedDrop);

        input.right.button = { .released = true };
        decision = advance(state, input);
        ok &= expectEqual("rejected sheath release only rearms",
            decision.action,
            Action::None);
        ok &= expectEqual("rejected sheath returns to drawn ready",
            state.phase,
            Phase::DrawnReady);
        ok &= expectTrue("rejected sheath release still suppresses drop",
            decision.suppressEquippedDrop);
    }

    {
        RuntimeState state{};
        auto input = drawnInput(SheathInputMode::Tap);
        input.presentation = NativePresentation::Sheathing;
        input.right.detector = rightCandidate(true);
        input.right.button = { .held = true, .pressed = true };
        auto decision = advance(state, input);
        ok &= expectEqual("unowned native transition blocks sheath",
            decision.action,
            Action::None);
        ok &= expectEqual("unowned transition has blocked phase",
            state.phase,
            Phase::NativeTransitionBlocked);
    }

    {
        RuntimeState state{};
        auto input = drawnInput(SheathInputMode::Tap);
        input.inputAllowed = false;
        input.right.detector = rightCandidate(true);
        input.right.button = { .held = true, .pressed = true };
        const auto decision = advance(state, input);
        ok &= expectEqual("menu/input block emits no action",
            decision.action,
            Action::None);
        ok &= expectEqual("menu/input block resets coordinator",
            state.phase,
            Phase::Disabled);
    }

    {
        RuntimeState state{};
        auto input = drawnInput(SheathInputMode::Tap);
        input.right.detector = rightCandidate(true);
        input.right.button = { .held = true, .pressed = true };
        auto decision = advance(state, input);
        reportExecutionResult(state, decision.action, true);

        input.presentation = NativePresentation::Invalid;
        decision = advance(state, input);
        ok &= expectEqual("invalid native presentation cancels action",
            decision.action,
            Action::None);
        ok &= expectEqual("invalid native presentation has explicit reason",
            decision.reason,
            Reason::InvalidPresentation);
        ok &= expectEqual("invalid native presentation resets phase",
            state.phase,
            Phase::Disabled);
        ok &= expectEqual("invalid native presentation clears direction",
            state.activeAction,
            Action::None);
    }

    return ok ? 0 : 1;
}
