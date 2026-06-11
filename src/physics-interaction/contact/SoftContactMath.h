#pragma once

/*
 * ROCK soft contact is the visual-authority half of the interaction system.
 * Generated hknp hand bodies can provide native world evidence, but keyframed
 * bodies still do not move the rendered FRIK hand by themselves. The math here
 * turns native world evidence or proxy/query fallback hits into a bounded
 * current-frame external hand transform, while ownership gates decide when
 * another system has stronger authority over posing. The tracked generated hand
 * body remains the magnet; contact may project it out of penetration but never
 * becomes a resting target.
 */

#include "RE/NetImmerse/NiPoint.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace rock::soft_contact_math
{
    enum class ContactState : std::uint8_t
    {
        Inactive = 0,
        Touching,
        Penetrating,
        Suppressed
    };

    enum class ContactKind : std::uint8_t
    {
        None = 0,
        WorldStatic
    };

    struct CapsuleContact
    {
        bool active = false;
        RE::NiPoint3 movablePoint{};
        RE::NiPoint3 targetPoint{};
        RE::NiPoint3 normal{};
        float distance = 0.0f;
        float penetration = 0.0f;
        std::uint32_t movableId = 0;
        std::uint32_t targetId = 0;
    };

    struct CorrectionStep
    {
        RE::NiPoint3 correction{};
        float alpha = 0.0f;
    };

    struct HapticEdgeConfig
    {
        bool enabled = true;
        float baseIntensity = 0.18f;
        float maxIntensity = 0.55f;
        float speedScale = 0.006f;
        float minApproachSpeed = 3.0f;
        float cooldownSeconds = 0.12f;
    };

    struct HapticEdgeState
    {
        bool wasActive = false;
        float cooldownRemainingSeconds = 0.0f;
    };

    struct HapticEdgeDecision
    {
        bool fire = false;
        float intensity = 0.0f;
    };

    inline RE::NiPoint3 add(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return RE::NiPoint3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }

    inline RE::NiPoint3 sub(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return RE::NiPoint3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    inline RE::NiPoint3 mul(const RE::NiPoint3& value, float scalar)
    {
        return RE::NiPoint3(value.x * scalar, value.y * scalar, value.z * scalar);
    }

    inline float dot(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    inline float lengthSquared(const RE::NiPoint3& value)
    {
        return dot(value, value);
    }

    inline float length(const RE::NiPoint3& value)
    {
        return std::sqrt(lengthSquared(value));
    }

    inline bool isFinite(const RE::NiPoint3& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    inline RE::NiPoint3 normalizeOr(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
    {
        const float lenSq = lengthSquared(value);
        if (!std::isfinite(lenSq) || lenSq <= 1.0e-6f) {
            const float fallbackLenSq = lengthSquared(fallback);
            if (!std::isfinite(fallbackLenSq) || fallbackLenSq <= 1.0e-6f) {
                return RE::NiPoint3(0.0f, 0.0f, 1.0f);
            }
            return mul(fallback, 1.0f / std::sqrt(fallbackLenSq));
        }

        return mul(value, 1.0f / std::sqrt(lenSq));
    }

    inline RE::NiPoint3 negate(const RE::NiPoint3& value)
    {
        return RE::NiPoint3(-value.x, -value.y, -value.z);
    }

    inline RE::NiPoint3 orientNormalTowardPoint(
        const RE::NiPoint3& surfacePoint,
        const RE::NiPoint3& normal,
        const RE::NiPoint3& point,
        const RE::NiPoint3& fallback)
    {
        const RE::NiPoint3 normalized = normalizeOr(normal, fallback);
        return dot(sub(point, surfacePoint), normalized) < 0.0f ? negate(normalized) : normalized;
    }

    inline RE::NiPoint3 clampLength(const RE::NiPoint3& value, float maxLength)
    {
        if (maxLength <= 0.0f || !std::isfinite(maxLength)) {
            return RE::NiPoint3();
        }

        const float len = length(value);
        if (!std::isfinite(len) || len <= maxLength || len <= 1.0e-6f) {
            return value;
        }

        return mul(value, maxLength / len);
    }

    inline float sanitizePositive(float value, float fallback)
    {
        return std::isfinite(value) && value > 0.0f ? value : fallback;
    }

    inline float sanitizeNonNegative(float value, float fallback)
    {
        return std::isfinite(value) && value >= 0.0f ? value : fallback;
    }

    inline float smoothingAlpha(float smoothingSpeed, float deltaSeconds)
    {
        smoothingSpeed = sanitizeNonNegative(smoothingSpeed, 0.0f);
        if (smoothingSpeed <= 0.0f) {
            return 1.0f;
        }

        deltaSeconds = std::clamp(std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f), 0.0f, 0.1f);
        return std::clamp(1.0f - std::exp(-smoothingSpeed * deltaSeconds), 0.0f, 1.0f);
    }

    inline CorrectionStep smoothCorrection(
        const RE::NiPoint3& previous,
        const RE::NiPoint3& desired,
        float smoothingSpeed,
        float deltaSeconds,
        float maxCorrection)
    {
        const float alpha = smoothingAlpha(smoothingSpeed, deltaSeconds);
        const RE::NiPoint3 blended = add(previous, mul(sub(desired, previous), alpha));
        return CorrectionStep{
            .correction = clampLength(blended, sanitizePositive(maxCorrection, 0.0f)),
            .alpha = alpha,
        };
    }

    inline RE::NiPoint3 projectTrackedMagnetCorrection(
        const RE::NiPoint3& contactNormal,
        float penetration,
        float maxCorrection)
    {
        /*
         * The tracked generated hand body is the magnet. Contact evidence may
         * only remove the illegal inward component for the current tracked pose;
         * it must never become a resting target that the rendered hand follows.
         * ROCK drives hand collision from the real hand transform; contact is a
         * constraint/evidence result.
         */
        if (!std::isfinite(penetration) || penetration <= 0.0f) {
            return RE::NiPoint3{};
        }

        return clampLength(
            mul(normalizeOr(contactNormal, RE::NiPoint3(0.0f, 0.0f, 1.0f)), penetration),
            sanitizePositive(maxCorrection, 0.0f));
    }

    inline bool preferStrongerContactResponse(
        float candidateResponseLength,
        float candidatePenetration,
        float currentResponseLength,
        float currentPenetration,
        float epsilon = 0.0001f)
    {
        const float safeCandidateResponse = std::isfinite(candidateResponseLength) ? (std::max)(0.0f, candidateResponseLength) : 0.0f;
        const float safeCurrentResponse = std::isfinite(currentResponseLength) ? (std::max)(0.0f, currentResponseLength) : 0.0f;
        const float safeEpsilon = std::isfinite(epsilon) && epsilon > 0.0f ? epsilon : 0.0001f;
        if (safeCandidateResponse > safeCurrentResponse + safeEpsilon) {
            return true;
        }
        if (safeCandidateResponse + safeEpsilon < safeCurrentResponse) {
            return false;
        }

        const float safeCandidatePenetration = std::isfinite(candidatePenetration) ? candidatePenetration : 0.0f;
        const float safeCurrentPenetration = std::isfinite(currentPenetration) ? currentPenetration : 0.0f;
        return safeCandidatePenetration > safeCurrentPenetration;
    }

    inline CapsuleContact solvePointPlaneContact(
        const RE::NiPoint3& movablePoint,
        const RE::NiPoint3& surfacePoint,
        const RE::NiPoint3& surfaceNormal,
        float probeRadius,
        float skin,
        std::uint32_t movableId,
        std::uint32_t targetId)
    {
        CapsuleContact result{};
        if (!isFinite(movablePoint) || !isFinite(surfacePoint) || !isFinite(surfaceNormal)) {
            return result;
        }

        result.normal = normalizeOr(surfaceNormal, sub(movablePoint, surfacePoint));
        result.movablePoint = movablePoint;
        result.distance = dot(sub(movablePoint, surfacePoint), result.normal);
        result.targetPoint = sub(movablePoint, mul(result.normal, result.distance));
        result.penetration = sanitizeNonNegative(probeRadius, 0.0f) + sanitizeNonNegative(skin, 0.0f) - result.distance;
        result.movableId = movableId;
        result.targetId = targetId;
        result.active = result.penetration > 0.0f;
        return result;
    }

    inline float effectiveQueryPadding(float queryPadding, float contactPadding, float fallbackQueryPadding, float fallbackContactPadding)
    {
        return std::max(
            sanitizeNonNegative(queryPadding, fallbackQueryPadding),
            sanitizeNonNegative(contactPadding, fallbackContactPadding));
    }

    inline float tangentDistanceFromAnchor(
        const RE::NiPoint3& projectedPoint,
        const RE::NiPoint3& anchorPoint,
        const RE::NiPoint3& surfaceNormal)
    {
        if (!isFinite(projectedPoint) || !isFinite(anchorPoint) || !isFinite(surfaceNormal)) {
            return std::numeric_limits<float>::max();
        }

        const float normalLenSq = lengthSquared(surfaceNormal);
        if (!std::isfinite(normalLenSq) || normalLenSq <= 1.0e-6f) {
            return std::numeric_limits<float>::max();
        }

        const RE::NiPoint3 normal = mul(surfaceNormal, 1.0f / std::sqrt(normalLenSq));
        const RE::NiPoint3 delta = sub(projectedPoint, anchorPoint);
        const RE::NiPoint3 tangent = sub(delta, mul(normal, dot(delta, normal)));
        return length(tangent);
    }

    inline bool withinTangentDriftLimit(
        const RE::NiPoint3& projectedPoint,
        const RE::NiPoint3& anchorPoint,
        const RE::NiPoint3& surfaceNormal,
        float maxTangentDrift)
    {
        const float limit = sanitizePositive(maxTangentDrift, 0.0f);
        return tangentDistanceFromAnchor(projectedPoint, anchorPoint, surfaceNormal) <= limit;
    }

    inline bool withinClearDistanceLimit(
        const RE::NiPoint3& point,
        const RE::NiPoint3& anchorPoint,
        float maxClearDistance)
    {
        if (!isFinite(point) || !isFinite(anchorPoint)) {
            return false;
        }

        const float limit = sanitizePositive(maxClearDistance, 0.0f);
        if (limit <= 0.0f) {
            return false;
        }

        return length(sub(point, anchorPoint)) <= limit;
    }

    inline bool shouldAllowPostReleaseReentrySweep(
        bool releasedCachedPlane,
        const RE::NiPoint3& sweepDelta,
        const RE::NiPoint3& surfaceNormal,
        float minApproachDistance)
    {
        if (!releasedCachedPlane) {
            return true;
        }
        if (!isFinite(sweepDelta) || !isFinite(surfaceNormal)) {
            return false;
        }

        const float normalLenSq = lengthSquared(surfaceNormal);
        if (!std::isfinite(normalLenSq) || normalLenSq <= 1.0e-6f) {
            return false;
        }

        const RE::NiPoint3 normal = mul(surfaceNormal, 1.0f / std::sqrt(normalLenSq));
        const float approachDistance = -dot(sweepDelta, normal);
        return approachDistance > sanitizeNonNegative(minApproachDistance, 0.0f);
    }

    inline HapticEdgeDecision updateHapticEdge(
        HapticEdgeState& state,
        bool active,
        float approachSpeed,
        float deltaSeconds,
        const HapticEdgeConfig& config)
    {
        const float dt = std::clamp(std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f), 0.0f, 0.1f);
        state.cooldownRemainingSeconds = std::max(0.0f, state.cooldownRemainingSeconds - dt);

        HapticEdgeDecision decision{};
        const float sanitizedApproachSpeed = std::isfinite(approachSpeed) ? std::max(0.0f, approachSpeed) : 0.0f;
        const float minApproachSpeed = sanitizeNonNegative(config.minApproachSpeed, 0.0f);
        const bool enteringContact = active && !state.wasActive;
        if (config.enabled && enteringContact && state.cooldownRemainingSeconds <= 0.0f && sanitizedApproachSpeed >= minApproachSpeed) {
            const float base = std::clamp(std::isfinite(config.baseIntensity) ? config.baseIntensity : 0.18f, 0.0f, 1.0f);
            const float maxIntensity = std::clamp(std::isfinite(config.maxIntensity) ? config.maxIntensity : 0.55f, base, 1.0f);
            const float speedScale = sanitizeNonNegative(config.speedScale, 0.0f);
            decision.fire = true;
            decision.intensity = std::clamp(base + sanitizedApproachSpeed * speedScale, base, maxIntensity);
            state.cooldownRemainingSeconds = sanitizeNonNegative(config.cooldownSeconds, 0.12f);
        }

        state.wasActive = active;
        return decision;
    }

}
