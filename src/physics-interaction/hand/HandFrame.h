#pragma once

/*
 * Hand frame helpers are grouped here because handspace convention, palm transform, and pointing direction define the same coordinate authority.
 */


// ---- HandspaceConvention.h ----

namespace rock::handspace_convention
{
    /*
     * ROCK root-flattened game hand frames use the active authored handspace convention:
     * authored X = fingers, authored Y = cross-palm, authored Z = palm thickness.
     * The resolved game bone transform already carries handedness, so left/right
     * conversion uses the same X/-Z/Y basis and does not apply an extra mirror.
     */
    template <class Vector>
    inline Vector makeVector(float x, float y, float z)
    {
        Vector result{};
        result.x = x;
        result.y = y;
        result.z = z;
        return result;
    }

    template <class Vector>
    inline Vector authoredToRaw(Vector value)
    {
        return makeVector<Vector>(value.x, value.z, -value.y);
    }

    template <class Vector>
    inline Vector authoredToRawForHand(Vector value, bool isLeft)
    {
        (void)isLeft;
        return authoredToRaw(value);
    }
}

// ---- PointingDirectionMath.h ----

namespace rock::pointing_direction_math
{
    /*
     * ROCK keeps close palm selection and far pointing selection as separate
     * handspace directions so debugging the blue far selection ray never changes
     * the yellow palm-normal line or close-grab direction. This helper keeps the
     * optional far-ray reversal shared between runtime selection and the
     * visualizer.
     */
    template <class Vector>
    inline Vector applyFarGrabNormalReversal(Vector direction, bool reverse)
    {
        if (reverse) {
            direction.x *= -1.0f;
            direction.y *= -1.0f;
            direction.z *= -1.0f;
        }

        return direction;
    }
}

// ---- PalmTransform.h ----

#include <cmath>

#include "RockConfig.h"
#include "physics-interaction/TransformMath.h"

namespace rock
{
    inline RE::NiPoint3 authoredHandspaceToRawHandspace(RE::NiPoint3 value)
    {
        return handspace_convention::authoredToRaw(value);
    }

    inline RE::NiPoint3 authoredHandspaceToRawHandspaceForHand(RE::NiPoint3 value, bool isLeft)
    {
        return handspace_convention::authoredToRawForHand(value, isLeft);
    }

    inline RE::NiPoint3 normalizeDirection(RE::NiPoint3 value)
    {
        const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
        if (lengthSquared <= 1.0e-8f) {
            return RE::NiPoint3(0.0f, 0.0f, 1.0f);
        }

        const float inverseLength = 1.0f / std::sqrt(lengthSquared);
        value.x *= inverseLength;
        value.y *= inverseLength;
        value.z *= inverseLength;
        return value;
    }

    inline RE::NiPoint3 transformHandspaceLocalToWorld(const RE::NiTransform& handTransform, const RE::NiPoint3& localVector)
    {
        // FO4VR binary verification: NiAVObject world updates call the Bethesda transform
        // compose helper at 0x1401A8D60, which applies child-local vectors through the
        // parent's column basis. CommonLib's direct matrix-vector operator exposes the
        // inverse direction for this use, so handspace offsets and directions must use
        // the transposed matrix to match engine-authored nodes and ROCK's NiTransform path.
        return handTransform.rotate.Transpose() * localVector;
    }

    inline RE::NiPoint3 transformHandspacePosition(const RE::NiTransform& handTransform, const RE::NiPoint3& localPosition, bool isLeft)
    {
        return handTransform.translate + transformHandspaceLocalToWorld(handTransform, authoredHandspaceToRawHandspaceForHand(localPosition, isLeft) * handTransform.scale);
    }

    inline RE::NiPoint3 transformHandspaceDirection(const RE::NiTransform& handTransform, const RE::NiPoint3& localDirection, bool isLeft)
    {
        return normalizeDirection(transformHandspaceLocalToWorld(handTransform, authoredHandspaceToRawHandspaceForHand(localDirection, isLeft)));
    }

    inline RE::NiPoint3 computeGrabPivotAHandspacePosition(bool isLeft)
    {
        return isLeft ? g_rockConfig.rockLeftGrabPivotAHandspace : g_rockConfig.rockRightGrabPivotAHandspace;
    }

    inline RE::NiPoint3 computePalmPositionFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
    {
        return transformHandspacePosition(handTransform, computeGrabPivotAHandspacePosition(isLeft), isLeft);
    }

    inline RE::NiPoint3 computePalmNormalFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
    {
        /*
         * Close selection uses the same palm-face convention proven by the working
         * far ray: authored Z is the collider's palm-thickness normal. The live
         * ROCK root-flattened game hand frame already carries left/right handedness,
         * so applying a separate authored Y mirror sends close detection across the palm.
         */
        const RE::NiPoint3 authoredNormal = g_rockConfig.rockPalmNormalHandspace;
        RE::NiPoint3 normal = transformHandspaceDirection(handTransform, authoredNormal, isLeft);
        if (g_rockConfig.rockReversePalmNormal) {
            normal.x *= -1.0f;
            normal.y *= -1.0f;
            normal.z *= -1.0f;
        }
        return normal;
    }

    inline RE::NiPoint3 computePointingVectorFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
    {
        return pointing_direction_math::applyFarGrabNormalReversal(
            transformHandspaceDirection(handTransform, g_rockConfig.rockPointingVectorHandspace, isLeft), g_rockConfig.rockReverseFarGrabNormal);
    }

    inline RE::NiPoint3 computePinchDetectionDirectionFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
    {
        return transformHandspaceDirection(handTransform, g_rockConfig.rockGrabPinchDetectionDirectionHandspace, isLeft);
    }

    inline RE::NiPoint3 computeGrabPivotAPositionFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
    {
        return transformHandspacePosition(handTransform, computeGrabPivotAHandspacePosition(isLeft), isLeft);
    }

    inline RE::NiPoint3 computeGrabAuthorityProxyOffsetLocalGame(bool isLeft)
    {
        return isLeft ? g_rockConfig.rockLeftGrabAuthorityProxyOffsetGameUnits : g_rockConfig.rockRightGrabAuthorityProxyOffsetGameUnits;
    }

    inline float grabAuthorityFrameLengthSquared(const RE::NiPoint3& value)
    {
        return value.x * value.x + value.y * value.y + value.z * value.z;
    }

    inline float grabAuthorityFrameDot(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    inline RE::NiPoint3 grabAuthorityFrameCross(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return RE::NiPoint3{
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x,
        };
    }

    inline RE::NiPoint3 grabAuthorityFrameNormalizeOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
    {
        const float lengthSquared = grabAuthorityFrameLengthSquared(value);
        if (std::isfinite(lengthSquared) && lengthSquared > 1.0e-6f) {
            const float inverseLength = 1.0f / std::sqrt(lengthSquared);
            return RE::NiPoint3{ value.x * inverseLength, value.y * inverseLength, value.z * inverseLength };
        }

        const float fallbackLengthSquared = grabAuthorityFrameLengthSquared(fallback);
        if (std::isfinite(fallbackLengthSquared) && fallbackLengthSquared > 1.0e-6f) {
            const float inverseLength = 1.0f / std::sqrt(fallbackLengthSquared);
            return RE::NiPoint3{ fallback.x * inverseLength, fallback.y * inverseLength, fallback.z * inverseLength };
        }

        return RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
    }

    inline RE::NiPoint3 grabAuthorityFrameRejectFromAxis(const RE::NiPoint3& value, const RE::NiPoint3& axis)
    {
        return value - axis * grabAuthorityFrameDot(value, axis);
    }

    inline RE::NiPoint3 grabAuthorityFrameOrientToward(const RE::NiPoint3& value, const RE::NiPoint3& reference)
    {
        return grabAuthorityFrameDot(value, reference) < 0.0f ? RE::NiPoint3{ -value.x, -value.y, -value.z } : value;
    }

    struct GrabAuthoritySemanticPalmFrame
    {
        RE::NiTransform rawRotationPalmTranslationWorld{};
        RE::NiPoint3 palmFaceWorld{};
        RE::NiPoint3 fingerForwardWorld{};
        RE::NiPoint3 acrossPalmWorld{};
        bool valid = false;
    };

    inline bool isFiniteGrabAuthorityPoint(const RE::NiPoint3& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    inline GrabAuthoritySemanticPalmFrame buildGrabAuthoritySemanticPalmFrame(
        const RE::NiTransform& rawHandWorld,
        const RE::NiPoint3& palmAnchorWorld,
        bool isLeft)
    {
        /*
         * Hidden grab authority uses the generated palm anchor only as a
         * translation source. Rotation and authored offset semantics come from
         * the root-flattened raw hand frame, using the same projected palm frame
         * as palm-pocket acquisition: X=fingers, Y=palm depth, Z=across palm.
         */
        GrabAuthoritySemanticPalmFrame frame{};
        frame.rawRotationPalmTranslationWorld = rawHandWorld;
        frame.rawRotationPalmTranslationWorld.translate = palmAnchorWorld;

        frame.palmFaceWorld =
            grabAuthorityFrameNormalizeOrFallback(computePalmNormalFromHandBasis(rawHandWorld, isLeft), RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
        const RE::NiPoint3 rawFingerForwardWorld =
            grabAuthorityFrameNormalizeOrFallback(transformHandspaceDirection(rawHandWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, isLeft), RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
        const RE::NiPoint3 rawAcrossPalmWorld =
            grabAuthorityFrameNormalizeOrFallback(transformHandspaceDirection(rawHandWorld, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }, isLeft), RE::NiPoint3{ 0.0f, 1.0f, 0.0f });

        const RE::NiPoint3 fallbackFingerForwardWorld =
            grabAuthorityFrameOrientToward(
                grabAuthorityFrameNormalizeOrFallback(grabAuthorityFrameCross(rawAcrossPalmWorld, frame.palmFaceWorld), rawFingerForwardWorld),
                rawFingerForwardWorld);
        frame.fingerForwardWorld =
            grabAuthorityFrameOrientToward(
                grabAuthorityFrameNormalizeOrFallback(grabAuthorityFrameRejectFromAxis(rawFingerForwardWorld, frame.palmFaceWorld), fallbackFingerForwardWorld),
                rawFingerForwardWorld);

        RE::NiPoint3 acrossCandidate = grabAuthorityFrameRejectFromAxis(rawAcrossPalmWorld, frame.palmFaceWorld);
        acrossCandidate = grabAuthorityFrameRejectFromAxis(acrossCandidate, frame.fingerForwardWorld);
        const RE::NiPoint3 fallbackAcrossPalmWorld =
            grabAuthorityFrameOrientToward(
                grabAuthorityFrameNormalizeOrFallback(grabAuthorityFrameCross(frame.palmFaceWorld, frame.fingerForwardWorld), rawAcrossPalmWorld),
                rawAcrossPalmWorld);
        frame.acrossPalmWorld =
            grabAuthorityFrameOrientToward(grabAuthorityFrameNormalizeOrFallback(acrossCandidate, fallbackAcrossPalmWorld), rawAcrossPalmWorld);

        frame.valid =
            isFiniteGrabAuthorityPoint(frame.rawRotationPalmTranslationWorld.translate) &&
            isFiniteGrabAuthorityPoint(frame.palmFaceWorld) &&
            isFiniteGrabAuthorityPoint(frame.fingerForwardWorld) &&
            isFiniteGrabAuthorityPoint(frame.acrossPalmWorld) &&
            std::isfinite(frame.rawRotationPalmTranslationWorld.scale);
        return frame;
    }

    inline RE::NiTransform makeGrabAuthoritySemanticPalmBaseFrame(
        const RE::NiTransform& rawHandWorld,
        const RE::NiPoint3& palmAnchorWorld,
        bool isLeft)
    {
        return buildGrabAuthoritySemanticPalmFrame(rawHandWorld, palmAnchorWorld, isLeft).rawRotationPalmTranslationWorld;
    }

    inline RE::NiTransform makeGrabAuthorityProxyFrameFromSemanticPalmPocket(
        const RE::NiTransform& rawHandWorld,
        const RE::NiPoint3& palmAnchorWorld,
        bool isLeft)
    {
        /*
         * This moves only the hidden grab authority proxy/seat point. The real
         * generated palm/finger colliders remain bound to the flattened bone tree
         * so contact evidence and collision behavior are not retuned by visual
         * hand placement experiments.
         */
        const GrabAuthoritySemanticPalmFrame frame = buildGrabAuthoritySemanticPalmFrame(rawHandWorld, palmAnchorWorld, isLeft);
        RE::NiTransform result = frame.rawRotationPalmTranslationWorld;
        const RE::NiPoint3 localOffset = computeGrabAuthorityProxyOffsetLocalGame(isLeft);
        if (frame.valid && std::isfinite(localOffset.x) && std::isfinite(localOffset.y) && std::isfinite(localOffset.z)) {
            const float scale = std::isfinite(rawHandWorld.scale) ? rawHandWorld.scale : 1.0f;
            const RE::NiPoint3 palmDepthWorld{
                -frame.palmFaceWorld.x,
                -frame.palmFaceWorld.y,
                -frame.palmFaceWorld.z,
            };
            const RE::NiPoint3 offsetWorld =
                (frame.fingerForwardWorld * localOffset.x + palmDepthWorld * localOffset.y + frame.acrossPalmWorld * localOffset.z) * scale;
            result.translate = result.translate + offsetWorld;
        }
        return result;
    }
}
