#pragma once

/*
 * Held-object helpers are grouped here because body-set policy, damping, physics math, player-space math, and character-controller contact policy shape the same held-object behavior.
 */


// ---- HeldObjectBodySetPolicy.h ----

#include <cstdint>
#include <unordered_set>
#include <vector>

namespace frik::rock::held_object_body_set_policy
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

// ---- HeldObjectDampingMath.h ----

#include <algorithm>
#include <cmath>

namespace frik::rock::held_object_damping_math
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

// ---- HeldPlayerSpaceMath.h ----

#include "physics-interaction/TransformMath.h"

#include <algorithm>
#include <cmath>

namespace frik::rock::held_player_space_math
{
    /*
     * HIGGS treats smooth/snap turning as a room-space warp instead of a large
     * velocity. ROCK keeps that convention in a pure helper first: the runtime
     * can either write the verified FO4VR body transform or only log the warp
     * decision, but the math for preserving the held body's room-local pose stays
     * independent of native Havok calls.
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
 * grab motor. HIGGS removes held/ignored bodies from the player-proxy collector
 * before Havok builds constraints. In FO4VR, the verified character-proxy
 * callback exposes paired manifold/contact rows before ROCK calls the original
 * listener, so ROCK can apply the same ownership rule without a broad world-cast
 * hook: compact held-body rows out before the original callback sees them. The
 * rejected alternative was a post-callback surface-velocity scrub, because it
 * leaves the engine-generated held-body contact alive and adds a late writer
 * against the same body the grab constraint is already controlling.
 */

#include <cstdint>
#include <cstring>

namespace frik::rock::held_grab_cc_policy
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
            .reason = removedCount > 0 ? "filteredHeldBodyContacts" : "noHeldBodyContacts",
        };
    }
}
