#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

/*
 * Far-pull flight and catch, engine-free so tests compile it.
 *
 * A pulled object flies a ballistic arc to the hand, re-solved every frame
 * from the object's live position to where the hand will be when it lands:
 * the live target plus the hand's measured velocity times the time to go. With
 * a still hand the re-solved arc is the arc planned at the start; a hand that
 * walks, turns or moves keeps the object coming to it. Once the planned
 * flight time has run out the solve keeps a short fixed time to go, so a late
 * object homes on the hand instead of coasting past it. A flight that stops
 * closing in (blocked by the world) ends as a physical drop.
 *
 * The catch: holding grab when the object arrives catches it. An object that
 * arrives without grab held falls free and a press within the catch window
 * catches it while its pulled point is within the catch radius of the palm.
 */
namespace rock::pull_motion_math
{
    // Planned flight time by start distance (Havok metres).
    inline constexpr float kDurationA = 0.715619f;
    inline constexpr float kDurationB = -0.415619f;
    inline constexpr float kDurationC = 0.656256f;
    // The arc aims this far above the target so it does not land short.
    inline constexpr float kDestinationZOffsetHavok = 0.01f;
    inline constexpr float kGravityHavok = 9.81f;
    inline constexpr float kMaximumVelocityHavok = 10.0f;
    // The shortest time to go a solve uses: the homing gain of a late object.
    inline constexpr float kMinimumTimeToGoSeconds = 0.1f;
    // Hand velocity estimate: smoothing, and the speed above which a target
    // jump is a teleport or snap turn rather than motion.
    inline constexpr float kTargetVelocityTimeConstantSeconds = 0.05f;
    inline constexpr float kMaximumPlausibleTargetSpeedHavok = 15.0f;
    // A flight that has not closed this much in this long is blocked.
    inline constexpr float kStallProgressHavok = 0.05f;
    inline constexpr float kStallSeconds = 0.75f;
    inline constexpr float kAutoGrabDistanceGameUnits = 18.0f;
    inline constexpr float kCatchRetryMaximumTimeSeconds = 0.65f;
    // After an arrival without grab held, a press within this time catches.
    inline constexpr float kCatchWindowSeconds = 1.0f;
    // Pulled point to palm distance within which a press catches (0.9 m).
    inline constexpr float kCatchRadiusGameUnits = 63.0f;
    // Actor-equipment materialization is a separate lifecycle even though its
    // current timeout matches pull-catch retry tuning.
    inline constexpr float kExtractionHandoffMaximumTimeSeconds = 0.65f;
    inline constexpr float kAngularDamping = 8.0f;

    template <class Vec3>
    struct PullFlightInput
    {
        Vec3 targetHavok{};
        Vec3 targetVelocityHavok{};
        Vec3 objectPointHavok{};
        float elapsedSeconds = 0.0f;
        float durationSeconds = 0.0f;
    };

    template <class Vec3>
    struct PullFlightResult
    {
        Vec3 velocityHavok{};
        float timeToGoSeconds = 0.0f;
        bool valid = false;
    };

    template <class Vec3>
    struct TargetVelocityEstimate
    {
        Vec3 lastTargetHavok{};
        Vec3 velocityHavok{};
        bool hasLastTarget = false;
    };

    struct PullProgress
    {
        float closestDistanceHavok = (std::numeric_limits<float>::max)();
        float secondsWithoutProgress = 0.0f;
    };

    inline float computePullDurationSeconds(float distanceHavok)
    {
        const float clampedDistance = std::isfinite(distanceHavok) ? (std::max)(0.0f, distanceHavok) : 0.0f;
        return (std::max)(0.001f, kDurationA + kDurationB * std::exp(-kDurationC * clampedDistance));
    }

    inline float angularVelocityKeepForDamping(float deltaTime)
    {
        if (!std::isfinite(deltaTime) || deltaTime <= 0.0f) {
            // Unmeasured frame: no elapsed time, no damping decay.
            return 1.0f;
        }
        return 1.0f / (1.0f + kAngularDamping * deltaTime);
    }

    template <class Vec3>
    float vectorLength(const Vec3& value)
    {
        return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
    }

    template <class Vec3>
    bool isFiniteVector(const Vec3& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    /*
     * Point of the segment start->end closest to point. Pull arrival tests the
     * path travelled since the previous frame, not only the current position:
     * at pull speed a slow frame moves the object farther than the catch
     * sphere is wide, so a per-frame point test lets it pass the hand unseen.
     */
    template <class Vec3>
    Vec3 closestPointOnSegment(const Vec3& start, const Vec3& end, const Vec3& point)
    {
        const Vec3 segment{ end.x - start.x, end.y - start.y, end.z - start.z };
        const float lengthSquared = segment.x * segment.x + segment.y * segment.y + segment.z * segment.z;
        if (!std::isfinite(lengthSquared) || lengthSquared <= 1.0e-12f)
            return end;
        const float t =
            ((point.x - start.x) * segment.x + (point.y - start.y) * segment.y + (point.z - start.z) * segment.z) /
            lengthSquared;
        if (!std::isfinite(t))
            return end;
        const float clamped = (std::clamp)(t, 0.0f, 1.0f);
        return Vec3{ start.x + segment.x * clamped, start.y + segment.y * clamped, start.z + segment.z * clamped };
    }

    template <class Vec3>
    Vec3 clampLength(Vec3 value, float maxLength)
    {
        if (maxLength <= 0.0f) {
            return value;
        }

        const float length = vectorLength(value);
        if (length <= maxLength || length <= 0.0001f) {
            return value;
        }

        const float scale = maxLength / length;
        value.x *= scale;
        value.y *= scale;
        value.z *= scale;
        return value;
    }

    /*
     * Measured target velocity, low-pass filtered. An unmeasured frame keeps
     * the estimate and re-bases on the new target; a jump faster than any hand
     * moves (snap turn, teleport) zeroes it instead of throwing the object.
     */
    template <class Vec3>
    void updateTargetVelocity(TargetVelocityEstimate<Vec3>& estimate, const Vec3& targetHavok, float deltaSeconds)
    {
        if (!isFiniteVector(targetHavok)) {
            return;
        }
        const bool measured = std::isfinite(deltaSeconds) && deltaSeconds > 0.0f;
        if (!estimate.hasLastTarget || !measured) {
            estimate.lastTargetHavok = targetHavok;
            estimate.hasLastTarget = true;
            return;
        }
        const Vec3 raw{ (targetHavok.x - estimate.lastTargetHavok.x) / deltaSeconds,
            (targetHavok.y - estimate.lastTargetHavok.y) / deltaSeconds,
            (targetHavok.z - estimate.lastTargetHavok.z) / deltaSeconds };
        estimate.lastTargetHavok = targetHavok;
        if (!isFiniteVector(raw) || vectorLength(raw) > kMaximumPlausibleTargetSpeedHavok) {
            estimate.velocityHavok = Vec3{};
            return;
        }
        const float blend = (std::min)(1.0f, deltaSeconds / kTargetVelocityTimeConstantSeconds);
        estimate.velocityHavok.x += (raw.x - estimate.velocityHavok.x) * blend;
        estimate.velocityHavok.y += (raw.y - estimate.velocityHavok.y) * blend;
        estimate.velocityHavok.z += (raw.z - estimate.velocityHavok.z) * blend;
    }

    // True once the flight has gone kStallSeconds without closing kStallProgressHavok.
    inline bool updatePullProgress(PullProgress& progress, float distanceHavok, float deltaSeconds)
    {
        if (!std::isfinite(distanceHavok)) {
            return false;
        }
        if (distanceHavok < progress.closestDistanceHavok - kStallProgressHavok) {
            progress.closestDistanceHavok = distanceHavok;
            progress.secondsWithoutProgress = 0.0f;
            return false;
        }
        if (std::isfinite(deltaSeconds) && deltaSeconds > 0.0f) {
            progress.secondsWithoutProgress += deltaSeconds;
        }
        return progress.secondsWithoutProgress >= kStallSeconds;
    }

    // The velocity to SET this frame on the pulled body (Havok units).
    template <class Vec3>
    PullFlightResult<Vec3> computePullFlight(const PullFlightInput<Vec3>& input)
    {
        PullFlightResult<Vec3> result{};
        if (!isFiniteVector(input.targetHavok) || !isFiniteVector(input.objectPointHavok)) {
            return result;
        }
        const float elapsed = std::isfinite(input.elapsedSeconds) ? (std::max)(0.0f, input.elapsedSeconds) : 0.0f;
        const float duration = std::isfinite(input.durationSeconds) ? (std::max)(0.0f, input.durationSeconds) : 0.0f;
        const float timeToGo = (std::max)(duration - elapsed, kMinimumTimeToGoSeconds);
        const Vec3 targetVelocity = isFiniteVector(input.targetVelocityHavok) ? input.targetVelocityHavok : Vec3{};

        const Vec3 aim{ input.targetHavok.x + targetVelocity.x * timeToGo,
            input.targetHavok.y + targetVelocity.y * timeToGo,
            input.targetHavok.z + targetVelocity.z * timeToGo + kDestinationZOffsetHavok };
        Vec3 velocity{ (aim.x - input.objectPointHavok.x) / timeToGo, (aim.y - input.objectPointHavok.y) / timeToGo,
            0.5f * kGravityHavok * timeToGo + (aim.z - input.objectPointHavok.z) / timeToGo };
        if (!isFiniteVector(velocity)) {
            return result;
        }
        result.velocityHavok = clampLength(velocity, kMaximumVelocityHavok);
        result.timeToGoSeconds = timeToGo;
        result.valid = true;
        return result;
    }
}
