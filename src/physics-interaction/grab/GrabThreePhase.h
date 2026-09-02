#pragma once

/*
 * Palm-pocket acquisition policy. The generated/proxy authority frame builds
 * one stable pocket in front of the palm; the pocket gate below is the single
 * rule that decides when a grip point is "in the hand". The seat gate at
 * commit, the pull arrival test, and the force-grab warp decision all use it.
 * Nothing here creates state: every function is a pure geometric predicate.
 */

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/VectorMath.h"
#include "physics-interaction/hand/HandFrame.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace rock::grab_three_phase
{
    inline float lengthSquared(const RE::NiPoint3& value)
    {
        return vector_math::lengthSquared(value);
    }

    inline float length(const RE::NiPoint3& value)
    {
        return std::sqrt((std::max)(0.0f, lengthSquared(value)));
    }

    inline float dot(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        return vector_math::dot(a, b);
    }

    inline bool isFinite(const RE::NiPoint3& value)
    {
        return vector_math::hasFiniteComponents(value);
    }

    inline bool isFinite(const RE::NiTransform& transform)
    {
        if (!isFinite(transform.translate) || !std::isfinite(transform.scale)) {
            return false;
        }
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(transform.rotate.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    inline RE::NiPoint3 normalizeOrZero(const RE::NiPoint3& value)
    {
        const float lenSq = lengthSquared(value);
        if (!std::isfinite(lenSq) || lenSq <= 0.000001f) {
            return RE::NiPoint3{};
        }

        const float invLen = 1.0f / std::sqrt(lenSq);
        return RE::NiPoint3{ value.x * invLen, value.y * invLen, value.z * invLen };
    }

    inline RE::NiPoint3 normalizeOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
    {
        const float lenSq = lengthSquared(value);
        if (!std::isfinite(lenSq) || lenSq <= 0.000001f) {
            const float fallbackLenSq = lengthSquared(fallback);
            if (!std::isfinite(fallbackLenSq) || fallbackLenSq <= 0.000001f) {
                return RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
            }
            const float invFallbackLen = 1.0f / std::sqrt(fallbackLenSq);
            return RE::NiPoint3{ fallback.x * invFallbackLen, fallback.y * invFallbackLen, fallback.z * invFallbackLen };
        }

        const float invLen = 1.0f / std::sqrt(lenSq);
        return RE::NiPoint3{ value.x * invLen, value.y * invLen, value.z * invLen };
    }

    inline RE::NiPoint3 cross(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return vector_math::cross(lhs, rhs);
    }

    inline RE::NiPoint3 rejectFromAxis(const RE::NiPoint3& value, const RE::NiPoint3& axis)
    {
        return value - axis * dot(value, axis);
    }

    inline RE::NiPoint3 orientToward(const RE::NiPoint3& value, const RE::NiPoint3& reference)
    {
        return dot(value, reference) < 0.0f ? RE::NiPoint3{ -value.x, -value.y, -value.z } : value;
    }

    struct GrabPocketFrame
    {
        RE::NiTransform basisWorld{};
        RE::NiPoint3 palmCenterWorld{};
        RE::NiPoint3 palmNormalWorld{};
        RE::NiPoint3 fingerForwardWorld{};
        RE::NiPoint3 crossPalmWorld{};
        RE::NiPoint3 oppositeCrossPalmWorld{};
        RE::NiPoint3 pocketCenterWorld{};
        float pocketRadiusGameUnits = 0.0f;
        float pocketDepthGameUnits = 0.0f;
        bool valid = false;
    };

    inline GrabPocketFrame buildGrabPocketFrameWithPalmCenter(
        const RE::NiTransform& basisWorld,
        bool isLeft,
        const RE::NiPoint3& palmCenterWorld,
        float pocketDepthGameUnits,
        float pocketRadiusGameUnits)
    {
        /*
         * Dynamic grab callers pass the generated/proxy authority relation
         * frame here. That row-view preserves the generated collider's local
         * axes for pocket roll without letting the raw controller hand basis
         * pull the pocket around a separate rotation origin. Legacy callers
         * that still need authored handspace use buildGrabPocketFrame below.
         */
        GrabPocketFrame frame{};
        frame.basisWorld = basisWorld;
        frame.palmCenterWorld = palmCenterWorld;
        frame.palmNormalWorld = normalizeOrFallback(computePalmNormalFromHandBasis(basisWorld, isLeft), RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
        const RE::NiPoint3 basisFingerForwardWorld =
            normalizeOrFallback(transformHandspaceDirection(basisWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, isLeft), RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
        const RE::NiPoint3 basisCrossPalmWorld =
            normalizeOrFallback(transformHandspaceDirection(basisWorld, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }, isLeft), RE::NiPoint3{ 0.0f, 0.0f, 1.0f });

        const RE::NiPoint3 fallbackFingerForwardWorld =
            orientToward(normalizeOrFallback(cross(basisCrossPalmWorld, frame.palmNormalWorld), basisFingerForwardWorld), basisFingerForwardWorld);
        frame.fingerForwardWorld =
            orientToward(normalizeOrFallback(rejectFromAxis(basisFingerForwardWorld, frame.palmNormalWorld), fallbackFingerForwardWorld), basisFingerForwardWorld);

        RE::NiPoint3 crossPalmCandidate = rejectFromAxis(basisCrossPalmWorld, frame.palmNormalWorld);
        crossPalmCandidate = rejectFromAxis(crossPalmCandidate, frame.fingerForwardWorld);
        const RE::NiPoint3 fallbackCrossPalmWorld =
            orientToward(normalizeOrFallback(cross(frame.palmNormalWorld, frame.fingerForwardWorld), basisCrossPalmWorld), basisCrossPalmWorld);
        frame.crossPalmWorld = orientToward(normalizeOrFallback(crossPalmCandidate, fallbackCrossPalmWorld), basisCrossPalmWorld);
        frame.oppositeCrossPalmWorld = RE::NiPoint3{ -frame.crossPalmWorld.x, -frame.crossPalmWorld.y, -frame.crossPalmWorld.z };
        frame.pocketDepthGameUnits = (std::max)(0.0f, std::isfinite(pocketDepthGameUnits) ? pocketDepthGameUnits : 0.0f);
        frame.pocketRadiusGameUnits = (std::max)(0.1f, std::isfinite(pocketRadiusGameUnits) ? pocketRadiusGameUnits : 9.0f);
        frame.pocketCenterWorld = frame.palmCenterWorld + frame.palmNormalWorld * frame.pocketDepthGameUnits;
        frame.valid = isFinite(basisWorld) && isFinite(frame.palmCenterWorld) && isFinite(frame.pocketCenterWorld) && lengthSquared(frame.palmNormalWorld) > 0.000001f;
        return frame;
    }

    inline GrabPocketFrame buildGrabPocketFrame(const RE::NiTransform& handWorld, bool isLeft, float pocketDepthGameUnits, float pocketRadiusGameUnits)
    {
        /*
         * Legacy helper for non-dynamic-grab callers that still consume the old
         * authored handspace palm position. Dynamic grab must use
         * buildGrabPocketFrameWithPalmCenter and pass an already-resolved palm
         * authority point so the old INI pivot cannot become runtime authority.
         */
        return buildGrabPocketFrameWithPalmCenter(
            handWorld,
            isLeft,
            computeGrabLegacyPalmPivotAWorldFromHandBasis(handWorld, isLeft),
            pocketDepthGameUnits,
            pocketRadiusGameUnits);
    }

    /*
     * The pocket gate. A grip point is inside the pocket when it lies within the
     * pocket radius of the palm centre and is not behind the palm plane by more
     * than the tolerance. radiusMarginGameUnits shrinks both limits; the pull
     * arrival test uses it so one frame of hand motion between the arrival
     * check and the commit-time seat gate cannot flip the answer.
     */
    struct PocketGateResult
    {
        float gripToPalmDistanceGameUnits = std::numeric_limits<float>::max();
        float signedPalmDistanceGameUnits = 0.0f;
        const char* reason = "invalidPocketOrGripPoint";
        bool inside = false;
    };

    inline PocketGateResult evaluatePocketGate(
        const GrabPocketFrame& pocket,
        const RE::NiPoint3& gripPointWorld,
        float behindPalmToleranceGameUnits,
        float radiusMarginGameUnits = 0.0f)
    {
        PocketGateResult result{};
        if (!pocket.valid || !isFinite(gripPointWorld)) {
            return result;
        }

        const RE::NiPoint3 toGrip = gripPointWorld - pocket.palmCenterWorld;
        result.gripToPalmDistanceGameUnits = length(toGrip);
        result.signedPalmDistanceGameUnits = dot(toGrip, pocket.palmNormalWorld);

        const float behindTolerance =
            (std::max)(0.0f, std::isfinite(behindPalmToleranceGameUnits) ? behindPalmToleranceGameUnits : 0.0f);
        const float margin = (std::max)(0.0f, std::isfinite(radiusMarginGameUnits) ? radiusMarginGameUnits : 0.0f);
        if (result.signedPalmDistanceGameUnits < -behindTolerance + margin) {
            result.reason = "behindPalm";
            return result;
        }
        if (result.gripToPalmDistanceGameUnits > (std::max)(0.0f, pocket.pocketRadiusGameUnits - margin)) {
            result.reason = "outsidePocketRadius";
            return result;
        }

        result.inside = true;
        result.reason = "insidePocket";
        return result;
    }
}
