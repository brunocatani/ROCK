#include <cstdio>

#include "physics-interaction/core/RockRuntimeStatePolicy.h"

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
    using namespace rock::runtime_state_policy;

    bool ok = true;

    ok &= expectEqual("positive frame delta preserved", sanitizeFrameDelta(1.0f / 72.0f), 1.0f / 72.0f);
    ok &= expectEqual("zero frame delta falls back", sanitizeFrameDelta(0.0f), kFallbackDeltaSeconds);
    ok &= expectEqual("negative frame delta falls back", sanitizeFrameDelta(-0.01f), kFallbackDeltaSeconds);
    ok &= expectEqual("huge frame delta falls back", sanitizeFrameDelta(0.5f), kFallbackDeltaSeconds);

    PlayerSpaceTrackerState tracker{};
    auto decision = updatePlayerSpaceTracker(tracker, PlayerSpaceTrackerInput{
        .positionValid = true,
        .currentPosition = Vec3{ 1.0f, 2.0f, 3.0f },
    });
    ok &= expectTrue("first player-space sample valid", decision.valid);
    ok &= expectFalse("first player-space sample not moving", decision.moving);

    decision = updatePlayerSpaceTracker(tracker, PlayerSpaceTrackerInput{
        .positionValid = true,
        .currentPosition = Vec3{ 1.01f, 2.0f, 3.0f },
    });
    ok &= expectFalse("small player-space delta is not moving", decision.moving);

    decision = updatePlayerSpaceTracker(tracker, PlayerSpaceTrackerInput{
        .positionValid = true,
        .currentPosition = Vec3{ 2.0f, 2.0f, 3.0f },
    });
    ok &= expectTrue("large player-space delta is moving", decision.moving);
    ok &= expectEqual("player-space delta x tracked", decision.deltaGameUnits.x, 0.99f);

    decision = updatePlayerSpaceTracker(tracker, PlayerSpaceTrackerInput{});
    ok &= expectFalse("invalid player-space sample clears validity", decision.valid);
    ok &= expectFalse("invalid player-space sample clears tracker", tracker.hasPrevious);

    ok &= expectTrue("complete local skeleton readiness passes", evaluateSkeletonReadiness(SkeletonReadinessInput{
                                                               .playerAvailable = true,
                                                               .rootNodeAvailable = true,
                                                               .rootParentAttached = true,
                                                               .flattenedTreeValid = true,
                                                               .requiredHandBonesResolved = true,
                                                           }));
    ok &= expectFalse("missing player blocks local skeleton readiness", evaluateSkeletonReadiness(SkeletonReadinessInput{
                                                                         .playerAvailable = false,
                                                                         .rootNodeAvailable = true,
                                                                         .rootParentAttached = true,
                                                                         .flattenedTreeValid = true,
                                                                         .requiredHandBonesResolved = true,
                                                                     }));
    ok &= expectFalse("detached root blocks local skeleton readiness", evaluateSkeletonReadiness(SkeletonReadinessInput{
                                                                        .playerAvailable = true,
                                                                        .rootNodeAvailable = true,
                                                                        .rootParentAttached = false,
                                                                        .flattenedTreeValid = true,
                                                                        .requiredHandBonesResolved = true,
                                                                    }));
    ok &= expectFalse("missing hand bones block local skeleton readiness", evaluateSkeletonReadiness(SkeletonReadinessInput{
                                                                            .playerAvailable = true,
                                                                            .rootNodeAvailable = true,
                                                                            .rootParentAttached = true,
                                                                            .flattenedTreeValid = true,
                                                                            .requiredHandBonesResolved = false,
                                                                        }));
    ok &= expectFalse("required body bones block local skeleton readiness", evaluateSkeletonReadiness(SkeletonReadinessInput{
                                                                            .playerAvailable = true,
                                                                            .rootNodeAvailable = true,
                                                                            .rootParentAttached = true,
                                                                            .flattenedTreeValid = true,
                                                                            .requiredHandBonesResolved = true,
                                                                            .bodyBonesRequired = true,
                                                                            .requiredBodyBonesResolved = false,
                                                                        }));

    return ok ? 0 : 1;
}
