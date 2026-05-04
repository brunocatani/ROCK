#pragma once

#include "HeldObjectDampingMath.h"
#include "PhysicsScale.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace frik::rock::held_object_physics_math
{
    /*
     * Held-object motion needs to preserve player-space movement separately from
     * solver residuals. HIGGS adds the room/player velocity to carried bodies and
     * damps only the motion left over from hand/object correction. Keeping these
     * formulas in a pure helper makes the constraint code and the release path use
     * one convention instead of accumulating one-off scale and damping rules.
     */
    template <class Vec3>
    inline Vec3 makeVector(float x, float y, float z)
    {
        Vec3 result{};
        result.x = x;
        result.y = y;
        result.z = z;
        return result;
    }

    template <class Vec3>
    inline float lengthSquared(const Vec3& value)
    {
        return value.x * value.x + value.y * value.y + value.z * value.z;
    }

    template <class Vec3>
    inline float length(const Vec3& value)
    {
        return std::sqrt(lengthSquared(value));
    }

    inline float safeDeltaTime(float deltaTime)
    {
        return (std::isfinite(deltaTime) && deltaTime > 0.00001f) ? deltaTime : (1.0f / 90.0f);
    }

    template <class Vec3>
    inline Vec3 gameUnitsDeltaToHavokVelocity(const Vec3& deltaGameUnits, float deltaTime, float havokToGameScale = physics_scale::kFallbackHavokToGame)
    {
        const float unitsPerHavok = physics_scale::isUsableScale(havokToGameScale) ? havokToGameScale : physics_scale::kFallbackHavokToGame;
        const float scale = 1.0f / (unitsPerHavok * safeDeltaTime(deltaTime));
        return makeVector<Vec3>(deltaGameUnits.x * scale, deltaGameUnits.y * scale, deltaGameUnits.z * scale);
    }

    template <class Vec3>
    inline bool shouldWarpPlayerSpaceDelta(const Vec3& deltaGameUnits, float warpDistanceGameUnits)
    {
        if (!std::isfinite(warpDistanceGameUnits) || warpDistanceGameUnits <= 0.0f) {
            return false;
        }

        return lengthSquared(deltaGameUnits) > (warpDistanceGameUnits * warpDistanceGameUnits);
    }

    template <class Vec3>
    inline Vec3 applyResidualVelocityDamping(const Vec3& currentVelocity, const Vec3& playerSpaceVelocity, float damping)
    {
        const float keep = held_object_damping_math::velocityKeepFactor(damping);
        return makeVector<Vec3>(
            playerSpaceVelocity.x + (currentVelocity.x - playerSpaceVelocity.x) * keep,
            playerSpaceVelocity.y + (currentVelocity.y - playerSpaceVelocity.y) * keep,
            playerSpaceVelocity.z + (currentVelocity.z - playerSpaceVelocity.z) * keep);
    }

    template <class Vec3>
    inline Vec3 composeReleaseVelocity(const Vec3& localVelocity, const Vec3& playerSpaceVelocity, float throwMultiplier)
    {
        const float multiplier = (std::isfinite(throwMultiplier) && throwMultiplier > 0.0f) ? throwMultiplier : 1.0f;
        return makeVector<Vec3>(
            playerSpaceVelocity.x + localVelocity.x * multiplier,
            playerSpaceVelocity.y + localVelocity.y * multiplier,
            playerSpaceVelocity.z + localVelocity.z * multiplier);
    }

    template <class Vec3, std::size_t Count>
    inline Vec3 maxMagnitudeVelocity(const std::array<Vec3, Count>& history, std::size_t validCount)
    {
        validCount = (std::min)(validCount, Count);
        if (validCount == 0) {
            return Vec3{};
        }

        std::size_t largestIndex = 0;
        float largestMagnitude = lengthSquared(history[0]);
        for (std::size_t i = 1; i < validCount; ++i) {
            const float magnitude = lengthSquared(history[i]);
            if (magnitude > largestMagnitude) {
                largestMagnitude = magnitude;
                largestIndex = i;
            }
        }

        if (validCount < 3 || largestIndex == 0 || largestIndex == validCount - 1) {
            return history[largestIndex];
        }

        return makeVector<Vec3>(
            (history[largestIndex - 1].x + history[largestIndex].x + history[largestIndex + 1].x) / 3.0f,
            (history[largestIndex - 1].y + history[largestIndex].y + history[largestIndex + 1].y) / 3.0f,
            (history[largestIndex - 1].z + history[largestIndex].z + history[largestIndex + 1].z) / 3.0f);
    }

    inline float computeHandLerpDuration(float distanceGameUnits, float minTime, float maxTime, float minDistanceGameUnits, float maxDistanceGameUnits)
    {
        if (!std::isfinite(minTime) || minTime < 0.0f) {
            minTime = 0.0f;
        }
        if (!std::isfinite(maxTime) || maxTime < minTime) {
            maxTime = minTime;
        }
        if (!std::isfinite(distanceGameUnits)) {
            return maxTime;
        }
        if (!std::isfinite(minDistanceGameUnits)) {
            minDistanceGameUnits = 0.0f;
        }
        if (!std::isfinite(maxDistanceGameUnits) || maxDistanceGameUnits <= minDistanceGameUnits) {
            return maxTime;
        }

        const float t = std::clamp((distanceGameUnits - minDistanceGameUnits) / (maxDistanceGameUnits - minDistanceGameUnits), 0.0f, 1.0f);
        return minTime + (maxTime - minTime) * t;
    }

    inline float advanceDeviationSeconds(float currentSeconds, float deviationGameUnits, float maxDeviationGameUnits, float deltaTime)
    {
        if (!std::isfinite(maxDeviationGameUnits) || maxDeviationGameUnits <= 0.0f) {
            return 0.0f;
        }
        if (!std::isfinite(deviationGameUnits) || deviationGameUnits <= maxDeviationGameUnits) {
            return 0.0f;
        }

        const float current = std::isfinite(currentSeconds) && currentSeconds > 0.0f ? currentSeconds : 0.0f;
        return current + safeDeltaTime(deltaTime);
    }

    inline bool deviationExceeded(float accumulatedSeconds, float allowedSeconds)
    {
        if (!std::isfinite(allowedSeconds) || allowedSeconds <= 0.0f) {
            return false;
        }
        return std::isfinite(accumulatedSeconds) && accumulatedSeconds >= allowedSeconds;
    }

    inline float advanceToward(float current, float target, float speed, float deltaTime)
    {
        if (!std::isfinite(current)) {
            current = target;
        }
        if (!std::isfinite(target)) {
            return current;
        }
        if (!std::isfinite(speed) || speed <= 0.0f) {
            return target;
        }

        const float step = speed * safeDeltaTime(deltaTime);
        const float delta = target - current;
        if (std::abs(delta) <= step) {
            return target;
        }
        return current + (delta > 0.0f ? step : -step);
    }

    inline float capForceByMassRatio(float force, float mass, float forceToMassRatio)
    {
        if (!std::isfinite(force) || force < 0.0f) {
            return 0.0f;
        }
        if (!std::isfinite(mass) || mass <= 0.0f || !std::isfinite(forceToMassRatio) || forceToMassRatio <= 0.0f) {
            return force;
        }
        return (std::min)(force, mass * forceToMassRatio);
    }

    inline float angularForceFromRatio(float linearForce, float angularToLinearRatio)
    {
        if (!std::isfinite(linearForce) || linearForce <= 0.0f) {
            return 0.0f;
        }
        if (!std::isfinite(angularToLinearRatio) || angularToLinearRatio <= 0.001f) {
            return linearForce;
        }
        return linearForce / angularToLinearRatio;
    }
}
