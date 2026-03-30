#pragma once

#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/native/HavokPhysicsTiming.h"

#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

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
        float bodyDeltaGameUnits = 0.0f;
        std::uint32_t motionIndex = body_frame::kFreeMotionIndex;
        body_frame::BodyFrameSource liveBodyFrameSource = body_frame::BodyFrameSource::Fallback;
        float driveDeltaSeconds = havok_physics_timing::kFallbackPhysicsDeltaSeconds;
        float sourceDeltaSeconds = havok_physics_timing::kFallbackPhysicsDeltaSeconds;
        float sourceAgeSeconds = 0.0f;
        float predictionLeadSeconds = 0.0f;
        float requiredLinearVelocityHavok = 0.0f;
        float requiredAngularVelocityRadians = 0.0f;
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
