#include "physics-interaction/api/InteractionCommandPolicy.h"
#include "physics-interaction/core/ForceGrabPolicy.h"
#include "physics-interaction/input/BareFistGesturePolicy.h"

#include <cstdio>
#include <limits>

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
    {
        namespace fists = rock::bare_fist_gesture;
        const auto sample = [](std::uint64_t tick, unsigned buttons) { return (tick << 3) | buttons; };
        const auto buttons = [&](std::uint64_t now, unsigned left, unsigned right) {
            return fists::readButtons(sample(now, left), sample(now, right), now);
        };
        // The player may stagger the presses by much more than 100 ms. The
        // controllers keep reporting held levels throughout that interval.
        auto cycle = fists::observe(2, true, buttons(1000, 3, 1), false);
        ok &= expectEqual("partial initial chord rearms with grips held", fists::capture(cycle), fists::Capture::Idle);
        cycle = fists::observe(cycle, true, buttons(1500, 3, 1), false);
        ok &= expectEqual("staggered buttons do not capture early", fists::capture(cycle), fists::Capture::Idle);
        cycle = fists::observe(cycle, true, buttons(1800, 3, 3), false);
        ok &= expectEqual("late fourth button completes Rocky chord", fists::capture(cycle), fists::Capture::Holding);
        const auto firstCycle = cycle;

        // Releasing any one of the four buttons revokes the session. A fresh
        // broken chord rearms without requiring the other three to come up.
        for (unsigned hand = 0; hand < 2; ++hand) {
            for (unsigned remaining : { 1u, 2u }) {
                const auto partial = buttons(2000, hand == 0 ? remaining : 3u, hand == 1 ? remaining : 3u);
                auto released = fists::observe(firstCycle, true, partial, true);
                ok &= expectEqual("one physical release cancels capture", fists::capture(released), fists::Capture::Draining);
                released = fists::observe(released, true, partial, false);
                ok &= expectEqual("one released button rearms Rocky", fists::capture(released), fists::Capture::ReadyToRetry);
                ok &= expectTrue("remaining buttons stay consumed while retry is armed", (released & 1u) != 0);
                const auto recaptured = fists::observe(released, true, buttons(2100, 3, 3), false);
                ok &= expectEqual("repress captures Rocky again", fists::capture(recaptured), fists::Capture::Holding);
                ok &= expectTrue("repress starts a distinct physical cycle", recaptured != firstCycle);
            }
        }

        // On recovery from a stalled game frame, OpenVR refreshes the two
        // controllers separately. Unknown input must not become button-up.
        const auto stalePeer = fists::readButtons(sample(2300, 3), sample(2100, 3), 2300);
        ok &= expectFalse("stale peer never authorizes drawing or damage", stalePeer.held);
        ok &= expectFalse("stale peer is not a physical chord break", stalePeer.chordBroken);
        ok &= expectEqual("first returning sample preserves capture", fists::observe(firstCycle, true, stalePeer, false), firstCycle);
        const auto refreshed = fists::readButtons(sample(2300, 3), sample(2301, 3), 2301);
        ok &= expectTrue("both returning samples prove held input", refreshed.held);
        ok &= expectEqual("second returning sample does not need full release", fists::observe(firstCycle, true, refreshed, false), firstCycle);
        ok &= expectEqual("stale samples cannot start a new capture", fists::observe(0, true, stalePeer, false), std::uint64_t{0});
        ok &= expectEqual("observed release wins over a stale peer", fists::capture(fists::observe(firstCycle, true, stalePeer, true)), fists::Capture::Draining);
        ok &= expectEqual("explicit admission loss still cancels capture", fists::capture(fists::observe(firstCycle, false, stalePeer, false)), fists::Capture::Draining);
        ok &= expectFalse("missing controller sample is not fresh", fists::readButtons(0, sample(2300, 3), 2300).fresh);
        ok &= expectFalse("future sample timestamp is not fresh", fists::readButtons(sample(2301, 3), sample(2300, 3), 2300).fresh);
        ok &= expectTrue("100 ms input boundary remains valid", fists::readButtons(sample(2200, 3), sample(2300, 3), 2300).held);
        ok &= expectFalse("older input still fails closed", fists::readButtons(sample(2199, 3), sample(2300, 3), 2300).held);

        const auto uiHeld = buttons(2400, 7, 3);
        const auto cancelled = fists::observe(firstCycle, true, uiHeld, false);
        ok &= expectEqual("UI capture cancels Rocky", fists::capture(cancelled), fists::Capture::Draining);
        ok &= expectEqual("UI blocked release cannot rearm Rocky", fists::observe(cancelled, true, buttons(2401, 5, 3), false), cancelled);
        ok &= expectEqual("held chord cannot replay after cancellation", fists::observe(cancelled, true, buttons(2402, 3, 3), false), cancelled);
        const auto retry = fists::observe(cancelled, true, buttons(2410, 1, 3), false);
        ok &= expectEqual("partial release keeps native input owned", fists::capture(retry), fists::Capture::ReadyToRetry);
        const auto retryCancelled = retry | 2u; // Runtime's atomic cancellation.
        ok &= expectEqual("dispatch admission loss revokes armed retry", fists::observe(retry, false, buttons(2411, 3, 3), false), retryCancelled);
        ok &= expectEqual("a cancellation revokes a previously armed retry", fists::capture(retryCancelled), fists::Capture::CancelledRetry);
        ok &= expectEqual("cancelled retry cannot replay a complete held chord", fists::observe(retryCancelled, true, buttons(2411, 3, 3), false), retryCancelled);
        const auto drained = fists::observe(retryCancelled, true, buttons(2420, 0, 0), false);
        ok &= expectEqual("full release ends all spent-input ownership", fists::capture(drained), fists::Capture::Idle);
        ok &= expectFalse("idle returns input to normal consumers", (drained & 1u) != 0);
        ok &= expectEqual("unknown samples cannot activate an armed retry", fists::observe(retry, true, stalePeer, false), retry);

        fists::State state{};
        ok &= expectEqual("qualification starts without an immediate draw", fists::update(state, {firstCycle, true, false, 0.2f, 0.3f}), fists::Action::None);
        ok &= expectEqual("partial hold does not draw", fists::update(state, {firstCycle, true, false, 0.2f, 0.3f}), fists::Action::None);
        ok &= expectEqual("continuous complete hold draws once", fists::update(state, {firstCycle, true, false, 0.11f, 0.3f}), fists::Action::Draw);
        ok &= expectEqual("native draw acknowledgement does not redraw", fists::update(state, {firstCycle, true, true, 0.01f, 0.3f}), fists::Action::None);
        ok &= expectEqual("acknowledged fists become active", state.phase, fists::Phase::Active);
        ok &= expectEqual("freshness loss on a game update cancels activity", fists::update(state, {firstCycle, false, true, 0.01f, 0.3f}), fists::Action::Cancel);
        ok &= expectEqual("cancelled fists return idle", state.phase, fists::Phase::Idle);
    }

    ForceGrabReservations reservations;
    ok &= expectFalse("invalid API hand cannot reserve", reservations.reserve(static_cast<rock::api::Hand>(99), 11, 100));
    ok &= expectFalse("auto hand rejects missing owner", reservations.reserve(rock::api::Hand::None, 0, 100));
    ok &= expectTrue("auto hand reserves both hands", reservations.reserve(rock::api::Hand::None, 11, 100));
    ok &= expectTrue("auto hand reserves right", reservations.isReserved(rock::api::Hand::Right));
    ok &= expectTrue("auto hand reserves left", reservations.isReserved(rock::api::Hand::Left));
    ok &= expectFalse("auto hand blocks overtaking request", reservations.reserve(rock::api::Hand::Left, 22, 201));
    reservations.release(22, 100);
    ok &= expectTrue("different owner cannot release auto hand", reservations.matches(11, 100));
    reservations.release(11, 100);
    ok &= expectFalse("auto hand completion releases both", reservations.isReserved(rock::api::Hand::None));
    ok &= expectTrue("first right API force grab reserves", reservations.reserve(rock::api::Hand::Right, 11, 101));
    ok &= expectFalse("auto hand cannot overtake reserved right", reservations.reserve(rock::api::Hand::None, 22, 200));
    ok &= expectFalse("duplicate right API force grab is rejected", reservations.reserve(rock::api::Hand::Right, 11, 102));
    ok &= expectTrue("independent left API force grab is allowed", reservations.reserve(rock::api::Hand::Left, 22, 201));
    ok &= expectTrue("dequeue does not implicitly release right", reservations.isReserved(rock::api::Hand::Right));
    reservations.release(11, 101);
    ok &= expectFalse("terminal result releases matching right", reservations.isReserved(rock::api::Hand::Right));
    ok &= expectTrue("terminal right does not release left", reservations.isReserved(rock::api::Hand::Left));
    reservations.clearOwner(22);
    ok &= expectFalse("owner loss clears active left", reservations.isReserved(rock::api::Hand::Left));
    ok &= expectFalse("queued state is non-terminal", isTerminal(rock::api::grab::InteractionCommandState::Queued));
    ok &= expectTrue("rejected state is terminal", isTerminal(rock::api::grab::InteractionCommandState::Rejected));

    RockProviderInteractionCommandResultV1 accepted{};
    accepted.state = rock::api::grab::InteractionCommandState::Queued;
    accepted.stage = rock::api::grab::CommandStage::Committed;
    accepted.acceptedFrame = 17;
    accepted.committedFrame = 19;
    RockProviderInteractionCommandResultV1 completed{};
    completed.state = rock::api::grab::InteractionCommandState::Succeeded;
    completed.stage = rock::api::grab::CommandStage::Queued;
    completed.acceptedFrame = 23;
    const auto mergedCompletion = mergeResultHistory(accepted, completed, 29);
    ok &= expectEqual("completion preserves acceptance frame", mergedCompletion.acceptedFrame, std::uint64_t{ 17 });
    ok &= expectEqual("completion preserves committed frame", mergedCompletion.committedFrame, std::uint64_t{ 19 });
    ok &= expectEqual("completion records terminal frame", mergedCompletion.frameIndex, std::uint64_t{ 29 });
    ok &= expectEqual("successful completion records applied frame", mergedCompletion.appliedFrame, std::uint64_t{ 29 });
    ok &= expectEqual("completion normalizes terminal stage", mergedCompletion.stage, rock::api::grab::CommandStage::Terminal);

    completed.state = rock::api::grab::InteractionCommandState::Rejected;
    completed.failure = rock::api::grab::InteractionFailure::TargetUnavailable;
    completed.failureStage = rock::api::grab::InteractionFailure::None;
    completed.frameIndex = 31;
    completed.appliedFrame = 0;
    const auto mergedRejection = mergeResultHistory(accepted, completed, 37);
    ok &= expectEqual("rejection preserves explicit terminal frame", mergedRejection.frameIndex, std::uint64_t{ 31 });
    ok &= expectEqual("rejection records failure stage", mergedRejection.failureStage, rock::api::grab::InteractionFailure::TargetUnavailable);
    ok &= expectEqual("rejection does not invent applied frame", mergedRejection.appliedFrame, std::uint64_t{ 0 });

    return ok ? 0 : 1;
}
