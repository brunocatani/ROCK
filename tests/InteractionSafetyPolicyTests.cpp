#include "physics-interaction/api/InteractionCommandPolicy.h"
#include "physics-interaction/core/ForceGrabPolicy.h"

#include <cstdio>

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

    rock::api::grab::InteractionCommandResult accepted{};
    accepted.state = rock::api::grab::InteractionCommandState::Queued;
    accepted.stage = rock::api::grab::CommandStage::Committed;
    accepted.acceptedFrame = 17;
    accepted.committedFrame = 19;
    rock::api::grab::InteractionCommandResult completed{};
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
