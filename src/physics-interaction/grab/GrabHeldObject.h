#pragma once

/*
 * Held-object helpers are grouped here because body-set policy, damping, physics math, player-space math, and character-controller contact policy shape the same held-object behavior.
 */


// ---- HeldObjectBodySetPolicy.h ----

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace rock::held_object_body_set_policy
{
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;

    inline std::vector<std::uint32_t> makePrimaryFirstUniqueBodyList(std::uint32_t primaryBodyId, const std::vector<std::uint32_t>& heldBodyIds)
    {
        std::vector<std::uint32_t> result;
        std::unordered_set<std::uint32_t> seen;
        result.reserve(heldBodyIds.size() + 1);

        auto append = [&](std::uint32_t bodyId) {
            if (bodyId == kInvalidBodyId || !seen.insert(bodyId).second) {
                return;
            }
            result.push_back(bodyId);
        };

        append(primaryBodyId);
        for (const auto bodyId : heldBodyIds) {
            append(bodyId);
        }
        return result;
    }

    inline bool containsBody(const std::vector<std::uint32_t>& bodyIds, std::uint32_t bodyId)
    {
        if (bodyId == kInvalidBodyId) {
            return false;
        }
        for (const auto heldBodyId : bodyIds) {
            if (heldBodyId == bodyId) {
                return true;
            }
        }
        return false;
    }

    inline bool containsAnyBody(const std::vector<std::uint32_t>& first,
        const std::vector<std::uint32_t>& second,
        std::uint32_t bodyId)
    {
        return containsBody(first, bodyId) || containsBody(second, bodyId);
    }
}

// ---- HeldObjectDrivePolicy.h ----

namespace rock::held_object_drive_policy
{
    enum class HeldBodySetDriveMode : std::uint8_t
    {
        SingleDynamic,
        ConnectedDynamic,
        FixedAttached,
        ComplexArticulated,
        IncompleteNativeScan,
    };

    inline const char* modeName(HeldBodySetDriveMode mode)
    {
        switch (mode) {
        case HeldBodySetDriveMode::SingleDynamic:
            return "singleDynamic";
        case HeldBodySetDriveMode::ConnectedDynamic:
            return "connectedDynamic";
        case HeldBodySetDriveMode::FixedAttached:
            return "fixedAttached";
        case HeldBodySetDriveMode::ComplexArticulated:
            return "complexArticulated";
        case HeldBodySetDriveMode::IncompleteNativeScan:
            return "incompleteNativeScan";
        }
        return "unknown";
    }

    struct HeldBodySetDriveInput
    {
        std::uint32_t acceptedBodyCount = 0;
        std::uint32_t uniqueMotionCount = 0;
        std::uint32_t rejectedFixedOrNonDynamicCount = 0;
        std::uint32_t scanFailureCount = 0;
        std::uint32_t invalidPhysicsSystemCount = 0;
        bool incompleteNativeScan = false;
    };

    struct HeldBodySetDriveDecision
    {
        HeldBodySetDriveMode mode = HeldBodySetDriveMode::SingleDynamic;
        float motorAuthorityScale = 1.0f;
        bool includeConnectedLinearVelocity = true;
        bool includeConnectedAngularVelocity = true;
        bool includeConnectedMass = true;
        const char* reason = "single-dynamic";
    };

    inline HeldBodySetDriveDecision evaluateHeldBodySetDrive(const HeldBodySetDriveInput& input)
    {
        /*
         * The held body list is the ownership/cleanup scope. Drive scope is
         * narrower: fixed-attached and incomplete scans should not receive broad
         * velocity writes that can energize unknown or anchored object parts.
         */
        const std::uint32_t acceptedBodies = input.acceptedBodyCount;
        const std::uint32_t uniqueMotions = input.uniqueMotionCount > 0 ? input.uniqueMotionCount : (acceptedBodies > 0 ? 1u : 0u);
        if (acceptedBodies == 0 || input.incompleteNativeScan || input.scanFailureCount > 0 || input.invalidPhysicsSystemCount > 0) {
            return HeldBodySetDriveDecision{
                .mode = HeldBodySetDriveMode::IncompleteNativeScan,
                .motorAuthorityScale = 0.55f,
                .includeConnectedLinearVelocity = false,
                .includeConnectedAngularVelocity = false,
                .includeConnectedMass = false,
                .reason = acceptedBodies == 0 ? "no-accepted-dynamic-body" : "incomplete-native-scan",
            };
        }

        if (input.rejectedFixedOrNonDynamicCount > 0) {
            return HeldBodySetDriveDecision{
                .mode = HeldBodySetDriveMode::FixedAttached,
                .motorAuthorityScale = 0.65f,
                .includeConnectedLinearVelocity = false,
                .includeConnectedAngularVelocity = false,
                .includeConnectedMass = false,
                .reason = "fixed-or-nondynamic-body",
            };
        }

        if (acceptedBodies <= 1 || uniqueMotions <= 1) {
            return acceptedBodies <= 1 ?
                       HeldBodySetDriveDecision{} :
                       HeldBodySetDriveDecision{
                           .mode = HeldBodySetDriveMode::ConnectedDynamic,
                           .reason = "connected-rigid-dynamic",
                       };
        }

        return HeldBodySetDriveDecision{
            .mode = HeldBodySetDriveMode::ComplexArticulated,
            .motorAuthorityScale = 0.80f,
            .includeConnectedLinearVelocity = true,
            .includeConnectedAngularVelocity = false,
            .includeConnectedMass = true,
            .reason = "multiple-independent-motions",
        };
    }

    inline float combineMotorAuthorityScale(float baseScale, const HeldBodySetDriveDecision& decision)
    {
        const float base = std::clamp(std::isfinite(baseScale) && baseScale > 0.0f ? baseScale : 1.0f, 0.05f, 1.0f);
        const float drive = std::clamp(std::isfinite(decision.motorAuthorityScale) && decision.motorAuthorityScale > 0.0f ? decision.motorAuthorityScale : 1.0f, 0.05f, 1.0f);
        return std::clamp(base * drive, 0.05f, 1.0f);
    }
}

// ---- HeldObjectContactPolicy.h ----

namespace rock::held_object_contact_policy
{
    /*
     * Multipart held refs need one external-impact signal for the whole held
     * object, not a feedback loop where two bodies from that same object keep
     * telling grab code that the object is colliding with the world. This policy
     * keeps the HIGGS connected-component rule at the ROCK contact boundary:
     * same-held-object contacts are internal, while held-to-world/object/body
     * contacts still notify the active grab.
     */
    struct HeldExternalContactInput
    {
        bool handHolding = false;
        bool bodyAIsHeld = false;
        bool bodyBIsHeld = false;
        bool otherIsRightHand = false;
        bool otherIsLeftHand = false;
        bool otherIsRightPalmBody = false;
        bool otherIsLeftPalmBody = false;
        bool otherIsBodyCollider = false;
        bool otherIsExternalProvider = false;
    };

    struct HeldExternalContactDecision
    {
        bool notify = false;
        bool sameHeldObject = false;
        const char* reason = "not-held-contact";
    };

    inline HeldExternalContactDecision evaluateHeldExternalContact(const HeldExternalContactInput& input)
    {
        if (!input.handHolding) {
            return HeldExternalContactDecision{ .reason = "hand-not-holding" };
        }
        if (!input.bodyAIsHeld && !input.bodyBIsHeld) {
            return HeldExternalContactDecision{ .reason = "no-held-endpoint" };
        }
        if (input.bodyAIsHeld && input.bodyBIsHeld) {
            return HeldExternalContactDecision{
                .sameHeldObject = true,
                .reason = "same-held-object",
            };
        }
        if (input.otherIsRightHand || input.otherIsLeftHand || input.otherIsRightPalmBody || input.otherIsLeftPalmBody) {
            return HeldExternalContactDecision{ .reason = "hand-endpoint" };
        }
        if (input.otherIsBodyCollider) {
            return HeldExternalContactDecision{ .reason = "body-collider-endpoint" };
        }
        if (input.otherIsExternalProvider) {
            return HeldExternalContactDecision{ .reason = "external-provider-endpoint" };
        }
        return HeldExternalContactDecision{
            .notify = true,
            .reason = "external-held-impact",
        };
    }

    enum class HeldContactOtherMotion : std::uint8_t
    {
        Unknown,
        FixedOrStatic,
        Dynamic
    };

    template <class Vec3>
    struct HeldContactMotorSofteningInput
    {
        bool recentContact = false;
        bool hasCorrectionVector = false;
        bool hasHeldToOtherVector = false;
        bool hasContactNormal = false;
        HeldContactOtherMotion otherMotion = HeldContactOtherMotion::Unknown;
        Vec3 correctionTowardTarget{};
        Vec3 heldToOther{};
        Vec3 contactNormal{};
        float pushDotThreshold = 0.15f;
    };

    struct HeldContactMotorSofteningDecision
    {
        bool soften = false;
        bool directional = false;
        const char* reason = "no-recent-contact";
    };

    template <class Vec3>
    inline float dot(const Vec3& lhs, const Vec3& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    template <class Vec3>
    inline float lengthSquared(const Vec3& value)
    {
        return dot(value, value);
    }

    template <class Vec3>
    inline Vec3 normalizeOrZero(const Vec3& value)
    {
        const float lenSq = lengthSquared(value);
        if (!std::isfinite(lenSq) || lenSq <= 1.0e-8f) {
            return Vec3{};
        }
        const float invLen = 1.0f / std::sqrt(lenSq);
        return Vec3{ value.x * invLen, value.y * invLen, value.z * invLen };
    }

    template <class Vec3>
    inline Vec3 negate(const Vec3& value)
    {
        return Vec3{ -value.x, -value.y, -value.z };
    }

    template <class Vec3>
    inline HeldContactMotorSofteningDecision evaluateHeldContactMotorSoftening(
        const HeldContactMotorSofteningInput<Vec3>& input)
    {
        /*
         * A held contact should weaken the grab motor only when the requested
         * correction is pushing the held object deeper into a fixed/static
         * obstacle. Sliding along the contact or pulling away should keep
         * authority; otherwise balls, watches, and long handles feel sticky.
         */
        if (!input.recentContact) {
            return {};
        }
        if (input.otherMotion == HeldContactOtherMotion::Dynamic) {
            return HeldContactMotorSofteningDecision{ .reason = "dynamic-other-body" };
        }
        if (!input.hasCorrectionVector) {
            return HeldContactMotorSofteningDecision{ .soften = true, .reason = "contact-no-correction-vector" };
        }

        const Vec3 correction = normalizeOrZero(input.correctionTowardTarget);
        if (lengthSquared(correction) <= 0.0f) {
            return HeldContactMotorSofteningDecision{ .reason = "zero-correction" };
        }

        Vec3 outward{};
        if (input.hasHeldToOtherVector) {
            outward = normalizeOrZero(input.heldToOther);
        }
        if (lengthSquared(outward) <= 0.0f && input.hasContactNormal) {
            outward = normalizeOrZero(input.contactNormal);
        }
        if (lengthSquared(outward) <= 0.0f) {
            return HeldContactMotorSofteningDecision{ .soften = true, .reason = "contact-no-direction" };
        }

        if (input.hasContactNormal) {
            Vec3 normal = normalizeOrZero(input.contactNormal);
            if (lengthSquared(normal) > 0.0f && dot(normal, outward) < 0.0f) {
                normal = negate(normal);
            }
            if (lengthSquared(normal) > 0.0f) {
                outward = normal;
            }
        }

        const float threshold = std::clamp(std::isfinite(input.pushDotThreshold) ? input.pushDotThreshold : 0.15f, -1.0f, 1.0f);
        const float pushDot = dot(correction, outward);
        if (pushDot > threshold) {
            return HeldContactMotorSofteningDecision{
                .soften = true,
                .directional = true,
                .reason = input.otherMotion == HeldContactOtherMotion::FixedOrStatic ? "pushing-into-fixed-contact" : "pushing-into-unknown-contact",
            };
        }
        return HeldContactMotorSofteningDecision{
            .directional = true,
            .reason = pushDot < -threshold ? "pulling-away-from-contact" : "tangent-contact-correction",
        };
    }
}

// ---- HeldObjectDampingMath.h ----

#include <algorithm>
#include <cmath>

namespace rock::held_object_damping_math
{
    // Held-object smoothing is applied as a velocity attenuation on the already constrained
    // body instead of changing hand/body frames. The grab frame math and pivots remain the
    // source of truth; this only removes carried solver velocity that makes held objects
    // visually chase or snap around the target from one physics update to the next.
    inline float clampVelocityDamping(float damping)
    {
        if (!std::isfinite(damping)) {
            return 0.0f;
        }

        return std::clamp(damping, 0.0f, 1.0f);
    }

    inline float velocityKeepFactor(float damping)
    {
        return 1.0f - clampVelocityDamping(damping);
    }

    template <class Vec3>
    inline Vec3 applyVelocityDamping(const Vec3& velocity, float damping)
    {
        const float keep = velocityKeepFactor(damping);
        Vec3 result = velocity;
        result.x *= keep;
        result.y *= keep;
        result.z *= keep;
        return result;
    }
}

// ---- HeldObjectPhysicsMath.h ----

#include "physics-interaction/native/PhysicsScale.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace rock::held_object_physics_math
{
    inline constexpr float kMaxGrabAuthorityTargetDeltaSeconds = 0.05f;

    /*
     * Held-object motion needs to preserve player-space movement separately from
     * solver residuals. ROCK adds the room/player velocity to carried bodies and
     * damps only the motion left over from hand/object correction. Keeping these
     * formulas in a pure helper makes the constraint code and the release path
     * use one convention instead of accumulating one-off scale and damping rules.
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

    inline float finitePositiveOrZero(float value)
    {
        return std::isfinite(value) && value > 0.0f ? value : 0.0f;
    }

    inline bool shouldQueueGrabAuthorityTargetForDelta(float deltaTime)
    {
        return std::isfinite(deltaTime) && deltaTime > 0.0f && deltaTime <= kMaxGrabAuthorityTargetDeltaSeconds;
    }

    inline float instantDeviationReleaseThreshold(float maxDeviationGameUnits)
    {
        const float configuredThreshold = (std::isfinite(maxDeviationGameUnits) && maxDeviationGameUnits > 0.0f) ? maxDeviationGameUnits * 2.0f : 0.0f;
        return (std::max)(configuredThreshold, 100.0f);
    }

    inline bool instantDeviationExceeded(float deviationGameUnits, float maxDeviationGameUnits)
    {
        if (!std::isfinite(deviationGameUnits)) {
            return true;
        }
        return deviationGameUnits > instantDeviationReleaseThreshold(maxDeviationGameUnits);
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

// ---- GrabHeldResponse.h ----

#include <algorithm>
#include <cmath>

namespace rock::grab_held_response
{
    /*
     * ROCK dynamic grab authority uses proxy-backed finite motors for ordinary
     * loose-object holds. These helpers derive bounded release velocity from
     * controller motion, held-body lag, player motion, and COM-relative angular
     * swing without making COM a grip pivot or keeping a second drive authority.
     */
    inline float finiteOr(float value, float fallback)
    {
        return std::isfinite(value) ? value : fallback;
    }

    inline float safePositive(float value, float fallback)
    {
        return std::isfinite(value) && value > 0.0f ? value : fallback;
    }

    inline float clamp01(float value)
    {
        if (!std::isfinite(value)) {
            return 0.0f;
        }
        return std::clamp(value, 0.0f, 1.0f);
    }

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
    inline Vec3 add(const Vec3& lhs, const Vec3& rhs)
    {
        return makeVector<Vec3>(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }

    template <class Vec3>
    inline Vec3 sub(const Vec3& lhs, const Vec3& rhs)
    {
        return makeVector<Vec3>(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    template <class Vec3>
    inline Vec3 scale(const Vec3& value, float scalar)
    {
        return makeVector<Vec3>(value.x * scalar, value.y * scalar, value.z * scalar);
    }

    template <class Vec3>
    inline float dot(const Vec3& lhs, const Vec3& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    template <class Vec3>
    inline Vec3 cross(const Vec3& lhs, const Vec3& rhs)
    {
        return makeVector<Vec3>(
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x);
    }

    template <class Vec3>
    inline float lengthSquared(const Vec3& value)
    {
        return dot(value, value);
    }

    template <class Vec3>
    inline float length(const Vec3& value)
    {
        return std::sqrt(lengthSquared(value));
    }

    template <class Vec3>
    inline Vec3 normalizeOrZero(const Vec3& value)
    {
        const float len = length(value);
        if (!std::isfinite(len) || len <= 0.000001f) {
            return Vec3{};
        }
        return scale(value, 1.0f / len);
    }

    template <class Vec3>
    inline Vec3 clampMagnitude(const Vec3& value, float maxMagnitude)
    {
        if (!std::isfinite(maxMagnitude) || maxMagnitude <= 0.0f) {
            return value;
        }

        const float magnitude = length(value);
        if (!std::isfinite(magnitude) || magnitude <= maxMagnitude || magnitude <= 0.000001f) {
            return value;
        }

        return scale(value, maxMagnitude / magnitude);
    }

    template <class Vec3>
    struct ReleaseVelocityInput
    {
        bool controllerDerivedEnabled = true;
        bool hasHandLocalVelocity = false;
        bool hasObjectLocalVelocity = false;
        bool hasTangentialVelocity = false;
        Vec3 handLocalVelocityHavok{};
        Vec3 objectLocalVelocityHavok{};
        Vec3 playerVelocityHavok{};
        Vec3 tangentialVelocityHavok{};
        float objectVelocityBlend = 0.35f;
        float tangentialVelocityScale = 1.0f;
        float throwMultiplier = 1.5f;
        float maxVelocityHavok = 12.0f;
    };

    template <class Vec3>
    struct ReleaseAngularVelocityInput
    {
        bool controllerDerivedEnabled = true;
        bool hasHandAngularVelocity = false;
        Vec3 handAngularVelocityRadiansPerSecond{};
        float angularVelocityScale = 1.0f;
        float maxAngularVelocityRadiansPerSecond = 18.0f;
    };

    template <class Vec3>
    inline Vec3 composeControllerReleaseVelocity(const ReleaseVelocityInput<Vec3>& input)
    {
        Vec3 localVelocity{};
        if (input.controllerDerivedEnabled && input.hasHandLocalVelocity) {
            localVelocity = input.handLocalVelocityHavok;
            if (input.hasTangentialVelocity) {
                localVelocity = add(localVelocity, scale(input.tangentialVelocityHavok, finiteOr(input.tangentialVelocityScale, 1.0f)));
            }
            if (input.hasObjectLocalVelocity) {
                localVelocity = add(localVelocity, scale(input.objectLocalVelocityHavok, clamp01(input.objectVelocityBlend)));
            }
        } else if (input.hasObjectLocalVelocity) {
            localVelocity = input.objectLocalVelocityHavok;
        }

        localVelocity = clampMagnitude(localVelocity, input.maxVelocityHavok);
        const float multiplier = safePositive(input.throwMultiplier, 1.0f);
        return clampMagnitude(add(input.playerVelocityHavok, scale(localVelocity, multiplier)), input.maxVelocityHavok);
    }

    template <class Vec3>
    inline Vec3 composeControllerReleaseAngularVelocity(const ReleaseAngularVelocityInput<Vec3>& input)
    {
        if (!input.controllerDerivedEnabled || !input.hasHandAngularVelocity) {
            return Vec3{};
        }

        const float scaleFactor = std::max(0.0f, finiteOr(input.angularVelocityScale, 1.0f));
        const float maxAngularVelocity = std::max(0.0f, finiteOr(input.maxAngularVelocityRadiansPerSecond, 0.0f));
        if (scaleFactor <= 0.0f || maxAngularVelocity <= 0.0f) {
            return Vec3{};
        }

        return clampMagnitude(scale(input.handAngularVelocityRadiansPerSecond, scaleFactor), maxAngularVelocity);
    }

    template <class Vec3>
    inline Vec3 computeTangentialVelocityFromAngularSwing(
        const Vec3& angularVelocityRadiansPerSecond,
        const Vec3& handPositionHavok,
        const Vec3& centerOfMassHavok)
    {
        const float angularSpeed = length(angularVelocityRadiansPerSecond);
        if (!std::isfinite(angularSpeed) || angularSpeed <= 0.000001f) {
            return Vec3{};
        }

        const Vec3 axis = normalizeOrZero(angularVelocityRadiansPerSecond);
        const Vec3 handToCenter = sub(centerOfMassHavok, handPositionHavok);
        const Vec3 radial = sub(handToCenter, scale(axis, dot(handToCenter, axis)));
        if (lengthSquared(radial) <= 0.000001f) {
            return Vec3{};
        }

        return scale(cross(axis, radial), angularSpeed);
    }
}

// ---- HeldPlayerSpaceMath.h ----

#include "physics-interaction/TransformMath.h"

#include <algorithm>
#include <cmath>

namespace rock::held_player_space_math
{
    /*
     * ROCK treats smooth/snap turning as a room-space warp instead of a large
     * velocity. The runtime can either write the verified FO4VR body transform or
     * only log the warp decision, but the math for preserving the held body's
     * room-local pose stays independent of native Havok calls.
     */

    template <class Matrix>
    inline float rotationDeltaDegrees(const Matrix& previous, const Matrix& current)
    {
        const float trace =
            previous.entry[0][0] * current.entry[0][0] + previous.entry[0][1] * current.entry[0][1] + previous.entry[0][2] * current.entry[0][2] +
            previous.entry[1][0] * current.entry[1][0] + previous.entry[1][1] * current.entry[1][1] + previous.entry[1][2] * current.entry[1][2] +
            previous.entry[2][0] * current.entry[2][0] + previous.entry[2][1] * current.entry[2][1] + previous.entry[2][2] * current.entry[2][2];
        const float cosAngle = std::clamp((trace - 1.0f) * 0.5f, -1.0f, 1.0f);
        return std::acos(cosAngle) * (180.0f / 3.14159265358979323846f);
    }

    template <class Matrix>
    inline bool shouldWarpPlayerSpaceRotation(const Matrix& previous, const Matrix& current, float minRotationDegrees)
    {
        if (!std::isfinite(minRotationDegrees) || minRotationDegrees <= 0.0f) {
            return false;
        }
        return rotationDeltaDegrees(previous, current) > minRotationDegrees;
    }

    inline constexpr bool shouldApplyRuntimeTransformWarp(bool enabled, bool diagnosticWarp, bool hasWarpTransforms)
    {
        return enabled && diagnosticWarp && hasWarpTransforms;
    }

    template <class Transform>
    inline Transform warpBodyWorldThroughPlayerSpace(
        const Transform& previousPlayerSpaceWorld,
        const Transform& currentPlayerSpaceWorld,
        const Transform& bodyWorld)
    {
        const Transform bodyInPreviousPlayerSpace =
            transform_math::composeTransforms(transform_math::invertTransform(previousPlayerSpaceWorld), bodyWorld);
        return transform_math::composeTransforms(currentPlayerSpaceWorld, bodyInPreviousPlayerSpace);
    }
}

// ---- HeldGrabCharacterControllerPolicy.h ----

/*
 * Held grabs must not have a hidden character-controller authority fighting the
 * grab motor. In FO4VR, the verified character-proxy callback exposes paired
 * manifold/contact rows before ROCK calls the original listener, so ROCK can
 * compact held-body rows out before the original callback sees them instead of
 * using a broad world-cast hook. The rejected alternative was a post-callback
 * surface-velocity scrub, because it leaves the engine-generated held-body
 * contact alive and adds a late writer against the same body the grab constraint
 * is already controlling.
 */

#include <cstdint>
#include <cstring>

namespace rock::held_grab_cc_policy
{
    inline constexpr int kGeneratedContactStride = 0x40;
    inline constexpr int kGeneratedContactBodyIdOffset = 0x28;
    inline constexpr int kGeneratedConstraintRowsOffset = 0x48;
    inline constexpr int kGeneratedConstraintCountOffset = 0x50;

    enum class HeldGrabContactIntervention : std::uint8_t
    {
        None = 0,
        PreOriginalFilter = 1,
    };

    struct HeldGrabContactPolicyInput
    {
        bool hooksEnabled = false;
        bool holdingHeldObject = false;
        bool diagnosticsEnabled = false;
    };

    struct HeldGrabContactPolicyDecision
    {
        HeldGrabContactIntervention intervention = HeldGrabContactIntervention::None;
        bool mayFilterBeforeOriginal = false;
        bool mayInspectGeneratedConstraint = false;
        bool mayMutateGeneratedConstraint = false;
        const char* reason = "postConstraintScrubDisabled";
    };

    struct GeneratedContactBufferView
    {
        bool valid = false;
        char* manifoldEntries = nullptr;
        char* constraintEntries = nullptr;
        int* manifoldCountPtr = nullptr;
        int* constraintCountPtr = nullptr;
        int manifoldCount = 0;
        int constraintCount = 0;
        int pairCount = 0;
        const char* reason = "uninitialized";
    };

    struct GeneratedContactFilterResult
    {
        bool valid = false;
        int originalPairCount = 0;
        int keptPairCount = 0;
        int removedPairCount = 0;
        const char* reason = "uninitialized";
    };

    inline HeldGrabContactPolicyDecision evaluateHeldGrabContactPolicy(const HeldGrabContactPolicyInput& input)
    {
        if (!input.hooksEnabled) {
            return HeldGrabContactPolicyDecision{
                .intervention = HeldGrabContactIntervention::None,
                .mayFilterBeforeOriginal = false,
                .mayInspectGeneratedConstraint = false,
                .mayMutateGeneratedConstraint = false,
                .reason = "hooksDisabled",
            };
        }

        if (!input.holdingHeldObject) {
            return HeldGrabContactPolicyDecision{
                .intervention = HeldGrabContactIntervention::None,
                .mayFilterBeforeOriginal = false,
                .mayInspectGeneratedConstraint = false,
                .mayMutateGeneratedConstraint = false,
                .reason = "notHolding",
            };
        }

        return HeldGrabContactPolicyDecision{
            .intervention = HeldGrabContactIntervention::PreOriginalFilter,
            .mayFilterBeforeOriginal = true,
            .mayInspectGeneratedConstraint = input.diagnosticsEnabled,
            .mayMutateGeneratedConstraint = false,
            .reason = "preOriginalHeldBodyFilter",
        };
    }

    inline GeneratedContactBufferView makeGeneratedContactBufferView(void* manifold, void* simplexInput)
    {
        if (!manifold) {
            return GeneratedContactBufferView{ .valid = false, .reason = "missingManifold" };
        }
        if (!simplexInput) {
            return GeneratedContactBufferView{ .valid = false, .reason = "missingSimplexInput" };
        }

        auto* manifoldBytes = static_cast<char*>(manifold);
        auto* simplexBytes = static_cast<char*>(simplexInput);
        char* manifoldEntries = *reinterpret_cast<char**>(manifoldBytes);
        int* manifoldCountPtr = reinterpret_cast<int*>(manifoldBytes + sizeof(char*));
        char* constraintEntries = *reinterpret_cast<char**>(simplexBytes + kGeneratedConstraintRowsOffset);
        int* constraintCountPtr = reinterpret_cast<int*>(simplexBytes + kGeneratedConstraintCountOffset);
        const int manifoldCount = *manifoldCountPtr;
        const int constraintCount = *constraintCountPtr;

        if (!manifoldEntries) {
            return GeneratedContactBufferView{
                .valid = false,
                .constraintEntries = constraintEntries,
                .manifoldCountPtr = manifoldCountPtr,
                .constraintCountPtr = constraintCountPtr,
                .manifoldCount = manifoldCount,
                .constraintCount = constraintCount,
                .reason = "missingManifoldEntries",
            };
        }
        if (!constraintEntries) {
            return GeneratedContactBufferView{
                .valid = false,
                .manifoldEntries = manifoldEntries,
                .manifoldCountPtr = manifoldCountPtr,
                .constraintCountPtr = constraintCountPtr,
                .manifoldCount = manifoldCount,
                .constraintCount = constraintCount,
                .reason = "missingConstraintEntries",
            };
        }
        if (manifoldCount <= 0 || constraintCount <= 0) {
            return GeneratedContactBufferView{
                .valid = false,
                .manifoldEntries = manifoldEntries,
                .constraintEntries = constraintEntries,
                .manifoldCountPtr = manifoldCountPtr,
                .constraintCountPtr = constraintCountPtr,
                .manifoldCount = manifoldCount,
                .constraintCount = constraintCount,
                .reason = "emptyContactBuffers",
            };
        }

        return GeneratedContactBufferView{
            .valid = true,
            .manifoldEntries = manifoldEntries,
            .constraintEntries = constraintEntries,
            .manifoldCountPtr = manifoldCountPtr,
            .constraintCountPtr = constraintCountPtr,
            .manifoldCount = manifoldCount,
            .constraintCount = constraintCount,
            .pairCount = manifoldCount < constraintCount ? manifoldCount : constraintCount,
            .reason = "ok",
        };
    }

    inline GeneratedContactFilterResult clearGeneratedConstraintOnlyContacts(const GeneratedContactBufferView& view)
    {
        if (view.manifoldEntries || !view.constraintEntries || !view.constraintCountPtr || view.constraintCount <= 0) {
            return GeneratedContactFilterResult{
                .valid = false,
                .originalPairCount = view.constraintCount,
                .reason = "notConstraintOnlyContacts",
            };
        }

        const int originalCount = view.constraintCount;
        if (view.manifoldCountPtr) {
            *view.manifoldCountPtr = 0;
        }
        *view.constraintCountPtr = 0;

        return GeneratedContactFilterResult{
            .valid = true,
            .originalPairCount = originalCount,
            .keptPairCount = 0,
            .removedPairCount = originalCount,
            .reason = "clearedConstraintOnlyContacts",
        };
    }

    template <class IsHeldBody>
    inline GeneratedContactFilterResult filterGeneratedContactBuffers(const GeneratedContactBufferView& view, IsHeldBody&& isHeldBody)
    {
        if (!view.valid) {
            return GeneratedContactFilterResult{
                .valid = false,
                .reason = view.reason,
            };
        }

        if (!view.manifoldEntries || !view.constraintEntries || !view.manifoldCountPtr || !view.constraintCountPtr) {
            return GeneratedContactFilterResult{
                .valid = false,
                .originalPairCount = view.pairCount,
                .reason = "missingFilterBuffer",
            };
        }

        int writeIndex = 0;
        for (int readIndex = 0; readIndex < view.pairCount; ++readIndex) {
            char* manifoldEntry = view.manifoldEntries + readIndex * kGeneratedContactStride;
            const auto bodyId = *reinterpret_cast<std::uint32_t*>(manifoldEntry + kGeneratedContactBodyIdOffset);
            if (isHeldBody(bodyId)) {
                continue;
            }

            if (writeIndex != readIndex) {
                std::memmove(view.manifoldEntries + writeIndex * kGeneratedContactStride,
                    manifoldEntry,
                    kGeneratedContactStride);
                std::memmove(view.constraintEntries + writeIndex * kGeneratedContactStride,
                    view.constraintEntries + readIndex * kGeneratedContactStride,
                    kGeneratedContactStride);
            }
            ++writeIndex;
        }

        const int removedCount = view.pairCount - writeIndex;
        if (removedCount > 0) {
            *view.manifoldCountPtr = writeIndex;
            *view.constraintCountPtr = writeIndex;
        }

        return GeneratedContactFilterResult{
            .valid = true,
            .originalPairCount = view.pairCount,
            .keptPairCount = writeIndex,
            .removedPairCount = removedCount,
            .reason = removedCount > 0 ? "filteredGeneratedContactRows" : "noGeneratedContactRowsFiltered",
        };
    }
}
