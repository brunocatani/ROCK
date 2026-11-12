#include "physics-interaction/grab/PullFlightPolicy.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
    namespace pull = rock::pull_motion_math;

    struct Vec3
    {
        float x = 0.0f, y = 0.0f, z = 0.0f;
    };

    bool check(bool value, const char* label)
    {
        if (!value)
            std::printf("FAILED: %s\n", label);
        return value;
    }

    float distance(const Vec3& a, const Vec3& b)
    {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
    }

    struct FlightOutcome
    {
        bool arrived = false;
        bool stalled = false;
        float seconds = 0.0f;
    };

    /*
     * The flight as the game runs it: each frame the velocity is SET on the
     * body, then one physics step integrates gravity. The target moves at a
     * constant velocity (hand plus locomotion). Arrival is the catch sphere,
     * 18 game units (0.257 Havok metres).
     */
    FlightOutcome fly(Vec3 object, Vec3 target, Vec3 targetVelocity, float seconds = 3.0f)
    {
        constexpr float dt = 1.0f / 90.0f;
        constexpr float arrival = 18.0f / 69.99125f;
        const float duration = pull::computePullDurationSeconds(distance(object, target));
        pull::TargetVelocityEstimate<Vec3> estimate{};
        pull::PullProgress progress{};
        FlightOutcome outcome{};
        float elapsed = 0.0f;
        for (int frame = 0; elapsed < seconds; ++frame) {
            pull::updateTargetVelocity(estimate, target, frame == 0 ? 0.0f : dt);
            if (distance(object, target) <= arrival) {
                outcome.arrived = true;
                outcome.seconds = elapsed;
                return outcome;
            }
            if (pull::updatePullProgress(progress, distance(object, target), dt)) {
                outcome.stalled = true;
                outcome.seconds = elapsed;
                return outcome;
            }
            const auto flight = pull::computePullFlight<Vec3>({ .targetHavok = target,
                .targetVelocityHavok = estimate.velocityHavok,
                .objectPointHavok = object,
                .elapsedSeconds = elapsed,
                .durationSeconds = duration });
            if (!flight.valid)
                return outcome;
            Vec3 velocity = flight.velocityHavok;
            velocity.z -= pull::kGravityHavok * dt;
            object.x += velocity.x * dt;
            object.y += velocity.y * dt;
            object.z += velocity.z * dt;
            target.x += targetVelocity.x * dt;
            target.y += targetVelocity.y * dt;
            target.z += targetVelocity.z * dt;
            elapsed += dt;
        }
        outcome.seconds = elapsed;
        return outcome;
    }
}

int main()
{
    bool ok = true;
    const Vec3 hand{ 0.0f, 0.0f, 1.2f };
    const Vec3 farObject{ 0.0f, 4.5f, 0.1f };

    // A still hand: the re-solved arc is the planned one and lands on time.
    {
        const float duration = pull::computePullDurationSeconds(distance(farObject, hand));
        const auto outcome = fly(farObject, hand, {});
        ok &= check(outcome.arrived, "still hand: arrives");
        ok &= check(outcome.seconds <= duration + 0.05f, "still hand: arrives within the planned flight time");
    }

    // Walking toward, away from and across the flight, and turning (hand sweeping sideways fast).
    const Vec3 motions[] = { { 0.0f, 3.0f, 0.0f }, { 0.0f, -3.0f, 0.0f }, { 3.0f, 0.0f, 0.0f }, { -6.0f, 0.0f, 0.0f } };
    for (const auto& motion : motions) {
        const auto outcome = fly(farObject, hand, motion);
        ok &= check(outcome.arrived && !outcome.stalled, "moving hand: object still arrives");
        ok &= check(outcome.seconds < 1.5f, "moving hand: arrives without a long chase");
    }

    // A hand that moves after launch, then stops: the late object homes in.
    {
        const auto outcome = fly(Vec3{ 2.0f, 2.0f, 0.1f }, hand, Vec3{ -2.0f, -1.0f, 0.3f }, 3.0f);
        ok &= check(outcome.arrived, "hand moving the whole flight: arrives");
    }

    // A snap turn or teleport moves the target faster than any hand: no throw.
    {
        pull::TargetVelocityEstimate<Vec3> estimate{};
        pull::updateTargetVelocity(estimate, Vec3{ 0, 0, 0 }, 0.0f);
        pull::updateTargetVelocity(estimate, Vec3{ 0.02f, 0, 0 }, 1.0f / 90.0f);
        ok &= check(estimate.velocityHavok.x > 0.0f, "plausible motion is measured");
        pull::updateTargetVelocity(estimate, Vec3{ 1.5f, 0, 0 }, 1.0f / 90.0f);
        ok &= check(pull::vectorLength(estimate.velocityHavok) == 0.0f, "teleport jump zeroes the estimate");
        pull::updateTargetVelocity(estimate, Vec3{ 9.0f, 0, 0 }, 0.0f);
        ok &= check(pull::vectorLength(estimate.velocityHavok) == 0.0f && estimate.lastTargetHavok.x == 9.0f,
            "unmeasured frame re-bases without a velocity");
    }

    // Blocked flight: no progress ends it; steady progress never does.
    {
        pull::PullProgress progress{};
        bool stalled = false;
        for (int frame = 0; frame < 90 && !stalled; ++frame)
            stalled = pull::updatePullProgress(progress, 2.0f, 1.0f / 90.0f);
        ok &= check(stalled, "no progress stalls");
        pull::PullProgress moving{};
        bool movingStalled = false;
        for (int frame = 0; frame < 270; ++frame)
            movingStalled |= pull::updatePullProgress(moving, 4.0f - frame * 0.01f, 1.0f / 90.0f);
        ok &= check(!movingStalled, "closing in never stalls");
    }

    // Non-finite input fails closed.
    {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const auto flight = pull::computePullFlight<Vec3>({ .targetHavok = Vec3{ nan, 0, 0 },
            .objectPointHavok = farObject,
            .durationSeconds = 0.5f });
        ok &= check(!flight.valid, "non-finite target is refused");
    }

    return ok ? 0 : 1;
}
