#pragma once

#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/native/HavokPhysicsTiming.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/TransformMath.h"

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"
#include "RE/Havok/hknpWorld.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>

namespace rock
{
    namespace generated_keyframed_body_drive_math
    {
        inline constexpr bool kPredictionEnabled = false;
        inline constexpr bool kVelocityHardSyncEnabled = false;
        inline constexpr float kMaxPredictionSeconds = 1.0f / 30.0f;
        inline constexpr float kMaxPredictionSourceFrames = 2.0f;
        inline constexpr float kMaxStaleSeconds = 0.10f;
        inline constexpr float kTinyDistanceGameUnits = 0.0001f;
        inline constexpr float kTinyRotationRadians = 0.000001f;

        inline float sanitizeSourceDeltaSeconds(float value)
        {
            return havok_physics_timing::isUsableDelta(value) ? value : havok_physics_timing::kFallbackPhysicsDeltaSeconds;
        }

        inline float predictionLeadSeconds(float secondsSinceSourceSample, float sourceDeltaSeconds, float driveDeltaSeconds)
        {
            if constexpr (!kPredictionEnabled) {
                (void)secondsSinceSourceSample;
                (void)sourceDeltaSeconds;
                (void)driveDeltaSeconds;
                return 0.0f;
            }

            if (!std::isfinite(secondsSinceSourceSample) || secondsSinceSourceSample < 0.0f) {
                secondsSinceSourceSample = 0.0f;
            }
            const float sourceDelta = sanitizeSourceDeltaSeconds(sourceDeltaSeconds);
            const float driveDelta = havok_physics_timing::isUsableDelta(driveDeltaSeconds) ? driveDeltaSeconds : havok_physics_timing::kFallbackPhysicsDeltaSeconds;
            if (secondsSinceSourceSample > kMaxStaleSeconds) {
                return 0.0f;
            }

            const float requestedLead = secondsSinceSourceSample + driveDelta;
            const float maxLead = (std::min)(kMaxPredictionSeconds, sourceDelta * kMaxPredictionSourceFrames);
            return (std::max)(0.0f, (std::min)(requestedLead, maxLead));
        }

        inline bool sourceIsStale(float secondsSinceSourceSample)
        {
            return std::isfinite(secondsSinceSourceSample) && secondsSinceSourceSample > kMaxStaleSeconds;
        }

        template <class Point>
        inline bool tryComputeSampledLinearVelocityHavok(
            const Point& previousTargetGame,
            const Point& currentTargetGame,
            float sourceDeltaSeconds,
            float gameToHavokScale,
            Point& outVelocityHavok)
        {
            outVelocityHavok = {};
            if (!std::isfinite(gameToHavokScale) || gameToHavokScale <= 0.0f ||
                !std::isfinite(previousTargetGame.x) || !std::isfinite(previousTargetGame.y) || !std::isfinite(previousTargetGame.z) ||
                !std::isfinite(currentTargetGame.x) || !std::isfinite(currentTargetGame.y) || !std::isfinite(currentTargetGame.z)) {
                return false;
            }

            const float sourceDelta = sanitizeSourceDeltaSeconds(sourceDeltaSeconds);
            const float invDelta = 1.0f / sourceDelta;
            outVelocityHavok.x = (currentTargetGame.x - previousTargetGame.x) * gameToHavokScale * invDelta;
            outVelocityHavok.y = (currentTargetGame.y - previousTargetGame.y) * gameToHavokScale * invDelta;
            outVelocityHavok.z = (currentTargetGame.z - previousTargetGame.z) * gameToHavokScale * invDelta;
            return std::isfinite(outVelocityHavok.x) && std::isfinite(outVelocityHavok.y) && std::isfinite(outVelocityHavok.z);
        }

        inline bool shouldUseSubstepInterpolatedTarget(
            const havok_physics_timing::PhysicsTimingSample& timing,
            bool hasPendingTarget,
            bool hasPreviousTarget,
            bool immediatePlacement)
        {
            return !immediatePlacement && timing.phase == havok_physics_timing::PhysicsStepPhase::SubstepPreCollide && hasPendingTarget && hasPreviousTarget;
        }

        inline bool shouldFinalizePlacedTargetForNextSource(const havok_physics_timing::PhysicsTimingSample& timing, bool immediatePlacement)
        {
            if (immediatePlacement || timing.phase != havok_physics_timing::PhysicsStepPhase::SubstepPreCollide) {
                return true;
            }
            if (timing.substepCount <= 1) {
                return true;
            }
            return timing.substepIndex + 1 >= timing.substepCount || timing.substepProgress >= 0.999f;
        }

        template <class Point>
        inline float pointDistance(const Point& lhs, const Point& rhs)
        {
            const float dx = rhs.x - lhs.x;
            const float dy = rhs.y - lhs.y;
            const float dz = rhs.z - lhs.z;
            const float squared = dx * dx + dy * dy + dz * dz;
            return std::isfinite(squared) && squared >= 0.0f ? std::sqrt(squared) : (std::numeric_limits<float>::infinity)();
        }

        template <class Point>
        inline Point lerpPoint(const Point& from, const Point& to, float alpha)
        {
            return Point{
                from.x + (to.x - from.x) * alpha,
                from.y + (to.y - from.y) * alpha,
                from.z + (to.z - from.z) * alpha,
            };
        }

        struct Quaternion
        {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            float w = 1.0f;
        };

        inline Quaternion normalize(Quaternion q)
        {
            const float length = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (length <= 0.000001f || !std::isfinite(length)) {
                return {};
            }

            const float invLength = 1.0f / length;
            q.x *= invLength;
            q.y *= invLength;
            q.z *= invLength;
            q.w *= invLength;
            return q;
        }

        inline float dot(const Quaternion& lhs, const Quaternion& rhs)
        {
            return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
        }

        inline Quaternion matrixToQuaternion(const RE::NiMatrix3& matrix)
        {
            float values[4]{};
            transform_math::niRowsToHavokQuaternion(matrix, values);
            return normalize(Quaternion{ values[0], values[1], values[2], values[3] });
        }

        inline RE::NiMatrix3 quaternionToMatrix(const Quaternion& q)
        {
            const Quaternion n = normalize(q);
            const float values[4] = { n.x, n.y, n.z, n.w };
            return transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(values);
        }

        inline Quaternion slerp(Quaternion from, Quaternion to, float alpha)
        {
            alpha = std::clamp(std::isfinite(alpha) ? alpha : 1.0f, 0.0f, 1.0f);
            from = normalize(from);
            to = normalize(to);

            float cosine = dot(from, to);
            if (cosine < 0.0f) {
                to.x = -to.x;
                to.y = -to.y;
                to.z = -to.z;
                to.w = -to.w;
                cosine = -cosine;
            }

            if (cosine > 0.9995f) {
                return normalize(Quaternion{
                    from.x + (to.x - from.x) * alpha,
                    from.y + (to.y - from.y) * alpha,
                    from.z + (to.z - from.z) * alpha,
                    from.w + (to.w - from.w) * alpha,
                });
            }

            cosine = std::clamp(cosine, -1.0f, 1.0f);
            const float theta = std::acos(cosine);
            const float sinTheta = std::sin(theta);
            if (std::abs(sinTheta) <= 0.000001f) {
                return from;
            }

            const float fromScale = std::sin((1.0f - alpha) * theta) / sinTheta;
            const float toScale = std::sin(alpha * theta) / sinTheta;
            return normalize(Quaternion{
                from.x * fromScale + to.x * toScale,
                from.y * fromScale + to.y * toScale,
                from.z * fromScale + to.z * toScale,
                from.w * fromScale + to.w * toScale,
            });
        }

        inline RE::NiTransform interpolateTargetTransform(const RE::NiTransform& from, const RE::NiTransform& to, float alpha)
        {
            RE::NiTransform result = to;
            alpha = std::clamp(std::isfinite(alpha) ? alpha : 1.0f, 0.0f, 1.0f);
            result.translate = lerpPoint(from.translate, to.translate, alpha);
            result.rotate = quaternionToMatrix(slerp(matrixToQuaternion(from.rotate), matrixToQuaternion(to.rotate), alpha));
            result.scale = from.scale + (to.scale - from.scale) * alpha;
            return result;
        }

        inline float matrixDot(const RE::NiMatrix3& a, const RE::NiMatrix3& b)
        {
            return a.entry[0][0] * b.entry[0][0] + a.entry[0][1] * b.entry[0][1] + a.entry[0][2] * b.entry[0][2] +
                   a.entry[1][0] * b.entry[1][0] + a.entry[1][1] * b.entry[1][1] + a.entry[1][2] * b.entry[1][2] +
                   a.entry[2][0] * b.entry[2][0] + a.entry[2][1] * b.entry[2][1] + a.entry[2][2] * b.entry[2][2];
        }

        inline float rotationAngleRadians(const RE::NiMatrix3& previous, const RE::NiMatrix3& current)
        {
            const float trace = matrixDot(previous, current);
            const float cosine = std::clamp((trace - 1.0f) * 0.5f, -1.0f, 1.0f);
            return std::acos(cosine);
        }

        struct TargetVelocityLimit
        {
            float alpha = 1.0f;
            bool linearLimitExceeded = false;
            bool angularLimitExceeded = false;
        };

        inline TargetVelocityLimit computeTargetVelocityLimit(
            float linearDistanceGameUnits,
            float angularDistanceRadians,
            float driveDeltaSeconds,
            float gameToHavokScale,
            float maxLinearVelocityHavok,
            float maxAngularVelocityRadians)
        {
            TargetVelocityLimit limit{};
            const float driveDelta = havok_physics_timing::isUsableDelta(driveDeltaSeconds) ? driveDeltaSeconds : havok_physics_timing::kFallbackPhysicsDeltaSeconds;
            const float scale = physics_scale::isUsableScale(gameToHavokScale) ? gameToHavokScale : physics_scale::kFallbackGameToHavok;

            if (std::isfinite(maxLinearVelocityHavok) && maxLinearVelocityHavok > 0.0f &&
                std::isfinite(linearDistanceGameUnits) && linearDistanceGameUnits > kTinyDistanceGameUnits) {
                const float requiredLinearVelocityHavok = linearDistanceGameUnits * scale / driveDelta;
                if (std::isfinite(requiredLinearVelocityHavok) && requiredLinearVelocityHavok > maxLinearVelocityHavok) {
                    limit.linearLimitExceeded = true;
                    limit.alpha = (std::min)(limit.alpha, maxLinearVelocityHavok / requiredLinearVelocityHavok);
                }
            }

            if (std::isfinite(maxAngularVelocityRadians) && maxAngularVelocityRadians > 0.0f &&
                std::isfinite(angularDistanceRadians) && angularDistanceRadians > kTinyRotationRadians) {
                const float requiredAngularVelocityRadians = angularDistanceRadians / driveDelta;
                if (std::isfinite(requiredAngularVelocityRadians) && requiredAngularVelocityRadians > maxAngularVelocityRadians) {
                    limit.angularLimitExceeded = true;
                    limit.alpha = (std::min)(limit.alpha, maxAngularVelocityRadians / requiredAngularVelocityRadians);
                }
            }

            limit.alpha = std::clamp(std::isfinite(limit.alpha) ? limit.alpha : 1.0f, 0.0f, 1.0f);
            return limit;
        }

        inline TargetVelocityLimit computeTargetVelocityLimit(
            const RE::NiTransform& from,
            const RE::NiTransform& requestedTarget,
            float driveDeltaSeconds,
            float gameToHavokScale,
            float maxLinearVelocityHavok,
            float maxAngularVelocityRadians)
        {
            return computeTargetVelocityLimit(
                pointDistance(from.translate, requestedTarget.translate),
                rotationAngleRadians(from.rotate, requestedTarget.rotate),
                driveDeltaSeconds,
                gameToHavokScale,
                maxLinearVelocityHavok,
                maxAngularVelocityRadians);
        }

        struct LimitedTarget
        {
            RE::NiTransform target{};
            TargetVelocityLimit limit{};
        };

        inline LimitedTarget limitGeneratedDriveTarget(
            const RE::NiTransform& from,
            const RE::NiTransform& requestedTarget,
            float driveDeltaSeconds,
            float gameToHavokScale,
            float maxLinearVelocityHavok,
            float maxAngularVelocityRadians)
        {
            LimitedTarget limited{};
            limited.limit = computeTargetVelocityLimit(
                from,
                requestedTarget,
                driveDeltaSeconds,
                gameToHavokScale,
                maxLinearVelocityHavok,
                maxAngularVelocityRadians);
            limited.target =
                (limited.limit.linearLimitExceeded || limited.limit.angularLimitExceeded) ?
                    interpolateTargetTransform(from, requestedTarget, limited.limit.alpha) :
                    requestedTarget;
            return limited;
        }
    }

    /*
     * Generated hand and weapon colliders are solver inputs driven from sampled
     * game transforms. Runtime validation showed the bounded prediction and
     * velocity hard-sync experiment did not address the current stutter and added
     * extra movement modes that obscure transform-convention debugging. The state
     * keeps source metadata for telemetry. Normal drive holds the last exact
     * queued target across Havok substeps so generated bodies do not coast for a
     * physics tick when render/skeleton sampling runs at a different cadence.
     * It still does not predict or hard-sync body velocities. It does keep the
     * sampled target-to-target linear velocity separately so contact consumers
     * can use the commanded collider motion, because Bethesda keyframed bodies
     * can report little or no hknp motion velocity even while driveToKeyFrame is
     * moving them through contacts.
     */
    struct GeneratedKeyframedBodyDriveState
    {
        mutable std::mutex mutex;
        RE::NiTransform pendingTarget{};
        RE::NiTransform previousTarget{};
        RE::NiPoint3 sampledLinearVelocityHavok{};
        float sourceDeltaSeconds = havok_physics_timing::kFallbackPhysicsDeltaSeconds;
        float secondsSinceSourceSample = generated_keyframed_body_drive_math::kMaxStaleSeconds;
        float teleportDistanceGameUnits = 1000.0f;
        std::uint32_t stepsWithoutSource = 0;
        std::uint64_t queuedSequence = 0;
        std::uint64_t consumedSequence = 0;
        bool hasPendingTarget = false;
        bool hasPreviousTarget = false;
        bool pendingTeleport = false;
        bool hasSampledLinearVelocityHavok = false;
    };

    [[nodiscard]] inline bool hasGeneratedKeyframedBodyDriveTargetUnlocked(const GeneratedKeyframedBodyDriveState& state)
    {
        return state.hasPendingTarget || state.hasPreviousTarget;
    }

    [[nodiscard]] inline bool hasGeneratedKeyframedBodyDriveTarget(const GeneratedKeyframedBodyDriveState& state)
    {
        std::scoped_lock lock(state.mutex);
        return hasGeneratedKeyframedBodyDriveTargetUnlocked(state);
    }

    [[nodiscard]] inline bool generatedKeyframedBodyDriveHasFreshSourceUnlocked(const GeneratedKeyframedBodyDriveState& state)
    {
        return state.queuedSequence != state.consumedSequence;
    }

    [[nodiscard]] inline bool generatedKeyframedBodyDriveHasFreshSource(const GeneratedKeyframedBodyDriveState& state)
    {
        std::scoped_lock lock(state.mutex);
        return generatedKeyframedBodyDriveHasFreshSourceUnlocked(state);
    }

    inline void refreshGeneratedKeyframedBodySourceClockForDriveUnlocked(GeneratedKeyframedBodyDriveState& state, float driveDeltaSeconds)
    {
        if (generatedKeyframedBodyDriveHasFreshSourceUnlocked(state)) {
            state.secondsSinceSourceSample = 0.0f;
            state.stepsWithoutSource = 0;
            return;
        }

        const float driveDelta =
            havok_physics_timing::isUsableDelta(driveDeltaSeconds) ? driveDeltaSeconds : havok_physics_timing::kFallbackPhysicsDeltaSeconds;
        const float currentAge =
            (std::isfinite(state.secondsSinceSourceSample) && state.secondsSinceSourceSample >= 0.0f) ? state.secondsSinceSourceSample : 0.0f;
        state.secondsSinceSourceSample = currentAge + driveDelta;
        if (state.stepsWithoutSource < (std::numeric_limits<std::uint32_t>::max)()) {
            ++state.stepsWithoutSource;
        }
    }

    inline void refreshGeneratedKeyframedBodySourceClockForDrive(GeneratedKeyframedBodyDriveState& state, float driveDeltaSeconds)
    {
        std::scoped_lock lock(state.mutex);
        refreshGeneratedKeyframedBodySourceClockForDriveUnlocked(state, driveDeltaSeconds);
    }

    struct GeneratedKeyframedBodyDriveResult
    {
        bool attempted = false;
        bool driven = false;
        bool teleported = false;
        bool skippedStale = false;
        bool missingBody = false;
        bool placementFailed = false;
        bool nativeDriveFailed = false;
        bool predicted = false;
        bool sourceStale = false;
        bool hardSynced = false;
        bool linearLimitExceeded = false;
        bool angularLimitExceeded = false;
        bool hasLiveBodyTransform = false;
        RE::NiPoint3 targetGamePosition{};
        RE::NiPoint3 targetHavokPosition{};
        RE::NiPoint3 liveBodyGamePosition{};
        RE::NiPoint3 targetAxisXWorld{};
        RE::NiPoint3 targetAxisYWorld{};
        RE::NiPoint3 targetAxisZWorld{};
        RE::NiPoint3 liveBodyAxisXWorld{};
        RE::NiPoint3 liveBodyAxisYWorld{};
        RE::NiPoint3 liveBodyAxisZWorld{};
        float bodyDeltaGameUnits = 0.0f;
        float targetToBodyRotationDegrees = 0.0f;
        float targetToBodyAxisXDegrees = 0.0f;
        float targetToBodyAxisYDegrees = 0.0f;
        float targetToBodyAxisZDegrees = 0.0f;
        std::uint32_t motionIndex = body_frame::kFreeMotionIndex;
        body_frame::BodyFrameSource liveBodyFrameSource = body_frame::BodyFrameSource::Fallback;
        float driveDeltaSeconds = havok_physics_timing::kFallbackPhysicsDeltaSeconds;
        float sourceDeltaSeconds = havok_physics_timing::kFallbackPhysicsDeltaSeconds;
        float sourceAgeSeconds = 0.0f;
        float predictionLeadSeconds = 0.0f;
        float requiredLinearVelocityHavok = 0.0f;
        float requiredAngularVelocityRadians = 0.0f;
        float uncappedRequiredLinearVelocityHavok = 0.0f;
        float uncappedRequiredAngularVelocityRadians = 0.0f;
        float targetLimitAlpha = 1.0f;
        std::uint32_t stepsWithoutSource = 0;

        [[nodiscard]] bool shouldRequestRebuild() const
        {
            return attempted && !driven && (missingBody || placementFailed || nativeDriveFailed);
        }
    };

    struct GeneratedKeyframedBodyDriveSampledVelocity
    {
        bool valid = false;
        RE::NiPoint3 velocityHavok{};
    };

    void clearGeneratedKeyframedBodyDriveState(GeneratedKeyframedBodyDriveState& state);
    void initializeGeneratedKeyframedBodyDriveState(GeneratedKeyframedBodyDriveState& state, const RE::NiTransform& target);
    void queueGeneratedKeyframedBodyTarget(
        GeneratedKeyframedBodyDriveState& state,
        const RE::NiTransform& target,
        float sourceDeltaSeconds,
        float teleportDistanceGameUnits);
    GeneratedKeyframedBodyDriveSampledVelocity snapshotGeneratedKeyframedBodyDriveSampledVelocity(const GeneratedKeyframedBodyDriveState& state);
    bool placeGeneratedKeyframedBodyImmediately(BethesdaPhysicsBody& body, const RE::NiTransform& target);
    inline void markGeneratedKeyframedBodyDrivePlacedUnlocked(
        GeneratedKeyframedBodyDriveState& state,
        float driveDeltaSeconds = 0.0f,
        bool finalizeTargetForNextSource = true)
    {
        (void)driveDeltaSeconds;
        if (!hasGeneratedKeyframedBodyDriveTargetUnlocked(state)) {
            return;
        }

        if (generatedKeyframedBodyDriveHasFreshSourceUnlocked(state)) {
            state.secondsSinceSourceSample = 0.0f;
            state.stepsWithoutSource = 0;
        }

        if (finalizeTargetForNextSource) {
            state.previousTarget = state.hasPendingTarget ? state.pendingTarget : state.previousTarget;
            state.hasPreviousTarget = true;
        }
        state.consumedSequence = state.queuedSequence;
        state.hasPendingTarget = true;
        state.pendingTeleport = false;
    }
    inline void markGeneratedKeyframedBodyDrivePlaced(
        GeneratedKeyframedBodyDriveState& state,
        float driveDeltaSeconds = 0.0f,
        bool finalizeTargetForNextSource = true)
    {
        std::scoped_lock lock(state.mutex);
        markGeneratedKeyframedBodyDrivePlacedUnlocked(state, driveDeltaSeconds, finalizeTargetForNextSource);
    }
    GeneratedKeyframedBodyDriveResult driveGeneratedKeyframedBody(
        RE::hknpWorld* world,
        BethesdaPhysicsBody& body,
        GeneratedKeyframedBodyDriveState& state,
        const havok_physics_timing::PhysicsTimingSample& timing,
        const char* ownerName,
        std::uint32_t bodyIndex,
        float maxLinearVelocityHavok = 0.0f,
        float maxAngularVelocityRadians = 0.0f);
}
