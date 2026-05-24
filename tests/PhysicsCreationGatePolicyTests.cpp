#include <cstdint>
#include <cstdio>

#include "physics-interaction/core/PhysicsCreationGatePolicy.h"

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

        std::printf("%s expected equality\n", label);
        return false;
    }
}

int main()
{
    using namespace rock::physics_creation_gate_policy;

    bool ok = true;

    WorldStabilityState state{};
    CreationGateInput input{
        .rockEnabled = true,
        .providerAvailable = true,
        .skeletonReady = true,
        .bhkWorld = 0x1000,
        .hknpWorld = 0x2000,
    };

    auto decision = evaluateCreationGate(state, input);
    ok &= expectFalse("first world frame does not create", decision.canCreate);
    ok &= expectEqual("first world frame is unstable", decision.blockReason, CreationBlockReason::WorldUnstable);
    ok &= expectEqual("first world frame counted", decision.stableWorldFrames, 1u);

    decision = evaluateCreationGate(state, input);
    ok &= expectTrue("second stable world frame creates", decision.canCreate);
    ok &= expectEqual("second stable world frame is unblocked", decision.blockReason, CreationBlockReason::None);
    ok &= expectEqual("second stable world frame counted", decision.stableWorldFrames, 2u);

    input.readyDeferralFrames = 1;
    decision = evaluateCreationGate(state, input);
    ok &= expectFalse("ready deferral blocks creation", decision.canCreate);
    ok &= expectEqual("ready deferral reports reason", decision.blockReason, CreationBlockReason::ReadyEventDeferred);
    ok &= expectEqual("ready deferral resets stability", state.stableFrames, 0u);

    input.readyDeferralFrames = 0;
    input.runtimeMenuBlocking = true;
    decision = evaluateCreationGate(state, input);
    ok &= expectFalse("runtime menu gate blocks creation", decision.canCreate);
    ok &= expectEqual("runtime menu gate reports menu reason", decision.blockReason, CreationBlockReason::MenuBlocked);

    input.runtimeMenuBlocking = false;
    input.compatibilityConfigBlocking = true;
    decision = evaluateCreationGate(state, input);
    ok &= expectFalse("compatibility config gate blocks creation", decision.canCreate);
    ok &= expectEqual("compatibility config gate reports menu reason", decision.blockReason, CreationBlockReason::MenuBlocked);

    input.compatibilityConfigBlocking = false;
    input.bhkWorld = 0;
    decision = evaluateCreationGate(state, input);
    ok &= expectFalse("missing world blocks creation", decision.canCreate);
    ok &= expectEqual("missing world reports unavailable", decision.blockReason, CreationBlockReason::WorldUnavailable);

    input.bhkWorld = 0x3000;
    input.hknpWorld = 0x4000;
    decision = evaluateCreationGate(state, input);
    ok &= expectFalse("changed world restarts settle count", decision.canCreate);
    ok &= expectEqual("changed world first frame is unstable", decision.stableWorldFrames, 1u);
    decision = evaluateCreationGate(state, input);
    ok &= expectTrue("changed world creates after settle", decision.canCreate);

    input.rockEnabled = false;
    decision = evaluateCreationGate(state, input);
    ok &= expectFalse("disabled ROCK blocks creation", decision.canCreate);
    ok &= expectFalse("disabled ROCK clears pending request", decision.keepRequestPending);
    ok &= expectEqual("disabled ROCK reports disabled", decision.blockReason, CreationBlockReason::RockDisabled);

    input.rockEnabled = true;
    input.skeletonReady = false;
    decision = evaluateCreationGate(state, input);
    ok &= expectFalse("missing skeleton blocks creation", decision.canCreate);
    ok &= expectTrue("missing skeleton keeps request pending", decision.keepRequestPending);
    ok &= expectEqual("missing skeleton reports reason", decision.blockReason, CreationBlockReason::SkeletonNotReady);

    return ok ? 0 : 1;
}
