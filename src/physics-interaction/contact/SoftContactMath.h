#pragma once

/*
 * ROCK soft contact is the visual-authority half of the interaction system.
 * Generated hknp hand/weapon bodies can now provide native contact evidence
 * for walls, tables, weapons, held objects, and future body colliders, but
 * keyframed bodies still do not move the rendered FRIK hand by themselves.
 * The math here turns native evidence or proxy/query fallback hits into a
 * bounded current-frame external hand transform, while ownership gates decide
 * when another system has stronger authority over posing. The tracked generated
 * hand body remains the magnet; contact may project it out of penetration but
 * never becomes a resting target.
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
        HandHand,
        WeaponHand,
        Body,
        WorldStatic
    };

    struct Capsule
    {
        RE::NiPoint3 start{};
        RE::NiPoint3 end{};
        float radius = 0.0f;
        std::uint32_t id = 0;
        bool valid = false;
    };

    struct Aabb
    {
        RE::NiPoint3 min{};
        RE::NiPoint3 max{};
        std::uint32_t id = 0;
        bool valid = false;
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

    inline RE::NiPoint3 pointMin(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return RE::NiPoint3((std::min)(lhs.x, rhs.x), (std::min)(lhs.y, rhs.y), (std::min)(lhs.z, rhs.z));
    }

    inline RE::NiPoint3 pointMax(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return RE::NiPoint3((std::max)(lhs.x, rhs.x), (std::max)(lhs.y, rhs.y), (std::max)(lhs.z, rhs.z));
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

    inline float axisValue(const RE::NiPoint3& point, int axis)
    {
        switch (axis) {
        case 0:
            return point.x;
        case 1:
            return point.y;
        default:
            return point.z;
        }
    }

    inline RE::NiPoint3 setAxisValue(RE::NiPoint3 point, int axis, float value)
    {
        switch (axis) {
        case 0:
            point.x = value;
            break;
        case 1:
            point.y = value;
            break;
        default:
            point.z = value;
            break;
        }
        return point;
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

    inline float compliantHardStopResponseScale(float penetration, float correctionScale, float hardStopPenetration)
    {
        if (!std::isfinite(penetration) || penetration <= 0.0f) {
            return 0.0f;
        }

        const float safeScale = std::clamp(std::isfinite(correctionScale) ? correctionScale : 1.0f, 0.0f, 1.0f);
        const float safeHardStop = sanitizePositive(hardStopPenetration, 0.0f);
        if (safeHardStop <= 0.0f || penetration >= safeHardStop) {
            return 1.0f;
        }

        const float t = std::clamp(penetration / safeHardStop, 0.0f, 1.0f);
        const float ramp = t * t * (3.0f - 2.0f * t);
        return safeScale + (1.0f - safeScale) * ramp;
    }

    inline RE::NiPoint3 projectCompliantTrackedMagnetCorrection(
        const RE::NiPoint3& contactNormal,
        float penetration,
        float maxCorrection,
        float correctionScale,
        float hardStopPenetration)
    {
        /*
         * Weapon-hand contact needs compliance without stale visual ownership:
         * shallow penetration only receives part of the current-frame projection,
         * while deep penetration ramps back to the same hard-stop correction used
         * by regular tracked-magnet contact. No previous frame target is retained.
         */
        const auto hardStopCorrection = projectTrackedMagnetCorrection(contactNormal, penetration, maxCorrection);
        return mul(hardStopCorrection, compliantHardStopResponseScale(penetration, correctionScale, hardStopPenetration));
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

    inline RE::NiPoint3 closestPointOnSegment(const RE::NiPoint3& point, const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        const RE::NiPoint3 ab = sub(b, a);
        const float lenSq = lengthSquared(ab);
        if (!std::isfinite(lenSq) || lenSq <= 1.0e-6f) {
            return a;
        }

        const float t = std::clamp(dot(sub(point, a), ab) / lenSq, 0.0f, 1.0f);
        return add(a, mul(ab, t));
    }

    inline RE::NiPoint3 clampPointToAabb(const RE::NiPoint3& point, const Aabb& bounds)
    {
        return RE::NiPoint3(
            std::clamp(point.x, bounds.min.x, bounds.max.x),
            std::clamp(point.y, bounds.min.y, bounds.max.y),
            std::clamp(point.z, bounds.min.z, bounds.max.z));
    }

    inline bool sanitizeAabb(const Aabb& input, Aabb& output)
    {
        output = {};
        if (!input.valid || !isFinite(input.min) || !isFinite(input.max)) {
            return false;
        }

        output.min = pointMin(input.min, input.max);
        output.max = pointMax(input.min, input.max);
        output.id = input.id;
        output.valid = true;
        return true;
    }

    inline bool segmentIntersectsAabb(const RE::NiPoint3& start, const RE::NiPoint3& end, const Aabb& bounds, float& outEnterT, float& outExitT)
    {
        float enterT = 0.0f;
        float exitT = 1.0f;
        const RE::NiPoint3 direction = sub(end, start);

        for (int axis = 0; axis < 3; ++axis) {
            const float startAxis = axisValue(start, axis);
            const float directionAxis = axisValue(direction, axis);
            const float minAxis = axisValue(bounds.min, axis);
            const float maxAxis = axisValue(bounds.max, axis);
            if (std::abs(directionAxis) <= 1.0e-6f) {
                if (startAxis < minAxis || startAxis > maxAxis) {
                    return false;
                }
                continue;
            }

            float axisEnter = (minAxis - startAxis) / directionAxis;
            float axisExit = (maxAxis - startAxis) / directionAxis;
            if (axisEnter > axisExit) {
                std::swap(axisEnter, axisExit);
            }

            enterT = (std::max)(enterT, axisEnter);
            exitT = (std::min)(exitT, axisExit);
            if (enterT > exitT) {
                return false;
            }
        }

        outEnterT = std::clamp(enterT, 0.0f, 1.0f);
        outExitT = std::clamp(exitT, 0.0f, 1.0f);
        return true;
    }

    struct AabbFaceContact
    {
        RE::NiPoint3 insidePoint{};
        RE::NiPoint3 point{};
        RE::NiPoint3 normal{};
        float insideDistance = 0.0f;
    };

    struct AabbFaceDistanceLine
    {
        float distanceAtStart = 0.0f;
        float slope = 0.0f;
    };

    inline AabbFaceContact nearestAabbFaceContact(const RE::NiPoint3& pointInside, const Aabb& bounds, const RE::NiPoint3& fallbackNormal)
    {
        AabbFaceContact best{};
        best.insidePoint = pointInside;
        best.point = pointInside;
        best.normal = normalizeOr(fallbackNormal, RE::NiPoint3(0.0f, 0.0f, 1.0f));
        best.insideDistance = std::numeric_limits<float>::max();

        auto considerFace = [&](int axis, float faceValue, const RE::NiPoint3& normal) {
            const float distance = std::abs(axisValue(pointInside, axis) - faceValue);
            if (distance < best.insideDistance) {
                best.insidePoint = pointInside;
                best.point = setAxisValue(pointInside, axis, faceValue);
                best.normal = normal;
                best.insideDistance = distance;
            }
        };

        considerFace(0, bounds.min.x, RE::NiPoint3(-1.0f, 0.0f, 0.0f));
        considerFace(0, bounds.max.x, RE::NiPoint3(1.0f, 0.0f, 0.0f));
        considerFace(1, bounds.min.y, RE::NiPoint3(0.0f, -1.0f, 0.0f));
        considerFace(1, bounds.max.y, RE::NiPoint3(0.0f, 1.0f, 0.0f));
        considerFace(2, bounds.min.z, RE::NiPoint3(0.0f, 0.0f, -1.0f));
        considerFace(2, bounds.max.z, RE::NiPoint3(0.0f, 0.0f, 1.0f));

        if (!std::isfinite(best.insideDistance) || best.insideDistance == std::numeric_limits<float>::max()) {
            best.insideDistance = 0.0f;
            best.normal = normalizeOr(fallbackNormal, RE::NiPoint3(0.0f, 0.0f, 1.0f));
        }

        return best;
    }

    inline AabbFaceDistanceLine makeAabbFaceDistanceLine(
        const RE::NiPoint3& start,
        const RE::NiPoint3& segment,
        const Aabb& bounds,
        int axis,
        bool maxFace)
    {
        const float startAxis = axisValue(start, axis);
        const float segmentAxis = axisValue(segment, axis);
        const float faceAxis = axisValue(maxFace ? bounds.max : bounds.min, axis);
        if (maxFace) {
            return AabbFaceDistanceLine{
                .distanceAtStart = faceAxis - startAxis,
                .slope = -segmentAxis,
            };
        }

        return AabbFaceDistanceLine{
            .distanceAtStart = startAxis - faceAxis,
            .slope = segmentAxis,
        };
    }

    inline AabbFaceContact deepestAabbFaceContactOnSegment(
        const RE::NiPoint3& start,
        const RE::NiPoint3& segment,
        const Aabb& bounds,
        float enterT,
        float exitT,
        const RE::NiPoint3& fallbackNormal)
    {
        /*
         * A weapon box can be crossed completely by the hand capsule centerline
         * in one frame. ROCK needs the deepest illegal point along the clipped
         * interval, not the first entry face, so the compliant weapon response
         * still reaches its hard stop when penetration is actually deep.
         */
        const float minT = std::clamp((std::min)(enterT, exitT), 0.0f, 1.0f);
        const float maxT = std::clamp((std::max)(enterT, exitT), 0.0f, 1.0f);
        AabbFaceContact best = nearestAabbFaceContact(add(start, mul(segment, minT)), bounds, fallbackNormal);

        auto considerT = [&](float t) {
            if (!std::isfinite(t) || t < minT - 1.0e-5f || t > maxT + 1.0e-5f) {
                return;
            }

            const float clampedT = std::clamp(t, minT, maxT);
            const AabbFaceContact contact = nearestAabbFaceContact(add(start, mul(segment, clampedT)), bounds, fallbackNormal);
            if (std::isfinite(contact.insideDistance) && contact.insideDistance > best.insideDistance) {
                best = contact;
            }
        };

        considerT(maxT);
        considerT((minT + maxT) * 0.5f);

        const AabbFaceDistanceLine lines[6]{
            makeAabbFaceDistanceLine(start, segment, bounds, 0, false),
            makeAabbFaceDistanceLine(start, segment, bounds, 0, true),
            makeAabbFaceDistanceLine(start, segment, bounds, 1, false),
            makeAabbFaceDistanceLine(start, segment, bounds, 1, true),
            makeAabbFaceDistanceLine(start, segment, bounds, 2, false),
            makeAabbFaceDistanceLine(start, segment, bounds, 2, true),
        };

        for (int i = 0; i < 6; ++i) {
            for (int j = i + 1; j < 6; ++j) {
                const float slopeDelta = lines[i].slope - lines[j].slope;
                if (std::abs(slopeDelta) <= 1.0e-6f) {
                    continue;
                }
                considerT((lines[j].distanceAtStart - lines[i].distanceAtStart) / slopeDelta);
            }
        }

        return best;
    }

    inline CapsuleContact solveCapsulePair(const Capsule& movable, const Capsule& target, const RE::NiPoint3& fallbackNormal, float radiusPadding)
    {
        CapsuleContact result{};
        if (!movable.valid || !target.valid || movable.radius < 0.0f || target.radius < 0.0f ||
            !isFinite(movable.start) || !isFinite(movable.end) || !isFinite(target.start) || !isFinite(target.end)) {
            return result;
        }

        const RE::NiPoint3 u = sub(movable.end, movable.start);
        const RE::NiPoint3 v = sub(target.end, target.start);
        const RE::NiPoint3 w = sub(movable.start, target.start);
        const float a = dot(u, u);
        const float b = dot(u, v);
        const float c = dot(v, v);
        const float d = dot(u, w);
        const float e = dot(v, w);
        const float denom = a * c - b * b;

        float s = 0.0f;
        float t = 0.0f;
        if (a <= 1.0e-6f && c <= 1.0e-6f) {
            s = 0.0f;
            t = 0.0f;
        } else if (a <= 1.0e-6f) {
            s = 0.0f;
            t = std::clamp(e / c, 0.0f, 1.0f);
        } else if (c <= 1.0e-6f) {
            t = 0.0f;
            s = std::clamp(-d / a, 0.0f, 1.0f);
        } else {
            s = denom > 1.0e-6f ? std::clamp((b * e - c * d) / denom, 0.0f, 1.0f) : 0.0f;
            t = (b * s + e) / c;
            if (t < 0.0f) {
                t = 0.0f;
                s = std::clamp(-d / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = std::clamp((b - d) / a, 0.0f, 1.0f);
            }
        }

        result.movablePoint = add(movable.start, mul(u, s));
        result.targetPoint = add(target.start, mul(v, t));
        const RE::NiPoint3 separation = sub(result.movablePoint, result.targetPoint);
        result.distance = length(separation);
        const float combinedRadius = movable.radius + target.radius + sanitizeNonNegative(radiusPadding, 0.0f);
        result.penetration = combinedRadius - result.distance;
        result.normal = normalizeOr(separation, fallbackNormal);
        result.movableId = movable.id;
        result.targetId = target.id;
        result.active = result.penetration > 0.0f;
        return result;
    }

    inline CapsuleContact solveCapsuleAabb(const Capsule& movable, const Aabb& target, const RE::NiPoint3& fallbackNormal, float radiusPadding)
    {
        CapsuleContact result{};
        Aabb bounds{};
        if (!movable.valid || movable.radius < 0.0f || !isFinite(movable.start) || !isFinite(movable.end) || !sanitizeAabb(target, bounds)) {
            return result;
        }

        const float combinedRadius = movable.radius + sanitizeNonNegative(radiusPadding, 0.0f);
        if (!std::isfinite(combinedRadius) || combinedRadius <= 0.0f) {
            return result;
        }

        const RE::NiPoint3 segment = sub(movable.end, movable.start);
        float enterT = 0.0f;
        float exitT = 1.0f;
        if (segmentIntersectsAabb(movable.start, movable.end, bounds, enterT, exitT)) {
            const AabbFaceContact face = deepestAabbFaceContactOnSegment(movable.start, segment, bounds, enterT, exitT, fallbackNormal);
            result.active = true;
            result.movablePoint = face.insidePoint;
            result.targetPoint = face.point;
            result.normal = normalizeOr(face.normal, fallbackNormal);
            result.distance = 0.0f;
            result.penetration = combinedRadius + (std::max)(0.0f, face.insideDistance);
            result.movableId = movable.id;
            result.targetId = bounds.id;
            return result;
        }

        bool initialized = false;
        float bestDistance = std::numeric_limits<float>::max();
        RE::NiPoint3 bestMovable{};
        RE::NiPoint3 bestTarget{};
        auto considerT = [&](float t) {
            if (!std::isfinite(t)) {
                return;
            }

            const float clampedT = std::clamp(t, 0.0f, 1.0f);
            const RE::NiPoint3 movablePoint = add(movable.start, mul(segment, clampedT));
            const RE::NiPoint3 targetPoint = clampPointToAabb(movablePoint, bounds);
            const float distance = length(sub(movablePoint, targetPoint));
            if (!std::isfinite(distance)) {
                return;
            }
            if (!initialized || distance < bestDistance) {
                initialized = true;
                bestDistance = distance;
                bestMovable = movablePoint;
                bestTarget = targetPoint;
            }
        };

        considerT(0.0f);
        considerT(0.5f);
        considerT(1.0f);

        const float segmentLenSq = lengthSquared(segment);
        if (segmentLenSq > 1.0e-6f && std::isfinite(segmentLenSq)) {
            for (int axis = 0; axis < 3; ++axis) {
                const float directionAxis = axisValue(segment, axis);
                if (std::abs(directionAxis) > 1.0e-6f) {
                    considerT((axisValue(bounds.min, axis) - axisValue(movable.start, axis)) / directionAxis);
                    considerT((axisValue(bounds.max, axis) - axisValue(movable.start, axis)) / directionAxis);
                }
            }

            for (int x = 0; x < 2; ++x) {
                for (int y = 0; y < 2; ++y) {
                    for (int z = 0; z < 2; ++z) {
                        const RE::NiPoint3 corner(
                            x == 0 ? bounds.min.x : bounds.max.x,
                            y == 0 ? bounds.min.y : bounds.max.y,
                            z == 0 ? bounds.min.z : bounds.max.z);
                        considerT(dot(sub(corner, movable.start), segment) / segmentLenSq);
                    }
                }
            }
        }

        if (!initialized) {
            return result;
        }

        const RE::NiPoint3 separation = sub(bestMovable, bestTarget);
        result.movablePoint = bestMovable;
        result.targetPoint = bestTarget;
        result.normal = normalizeOr(separation, fallbackNormal);
        result.distance = bestDistance;
        result.penetration = combinedRadius - bestDistance;
        result.movableId = movable.id;
        result.targetId = bounds.id;
        result.active = result.penetration > 0.0f;
        return result;
    }
}
