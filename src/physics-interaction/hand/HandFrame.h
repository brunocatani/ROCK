#pragma once

/*
 * LEGACY HANDSPACE WARNING
 *
 * This file contains the old authored handspace conversion where authored
 * X=fingers, authored Y=cross-palm, and authored Z=palm thickness. That is NOT
 * the production grab/palm/pinch/finger-pose authority anymore.
 *
 * New runtime hand code must use the unified root-flattened skeleton convention
 * from HandSkeleton.h / SemanticHandFrame:
 *   X  = fingers
 *   Y  = palm depth/back
 *   -Y = palm face
 *   Z  = across palm
 *
 * Do not use these legacy helpers for dynamic grab, palm pocket, pinch pocket,
 * finger curl planes, public palm API output, soft-contact palm normals, or any
 * new hand-space feature. They remain only as a quarantined compatibility
 * surface for old authored handspace diagnostics and the known two-handed
 * weapon handspace exception if that path ever needs to be restored or replaced.
 */


// ---- HandspaceConvention.h ----

namespace rock::handspace_convention
{
    /*
     * LEGACY CONVERSION ONLY.
     *
     * This authored Y/Z swap is intentionally not the unified flatroot skeleton
     * convention. Treat any new caller as suspicious unless it is explicitly
     * maintaining the old two-handed weapon handspace exception or legacy
     * telemetry comparing old configured offsets against the real generated
     * palm/proxy frame.
     *
     * Legacy root-flattened handspace callers use this authored convention:
     * authored X = fingers, authored Y = cross-palm, authored Z = palm thickness.
     * The resolved game bone transform already carries handedness, so left/right
     * conversion uses the same X/-Z/Y basis and does not apply an extra mirror.
     *
     * Dynamic grab authority does not use this authored Y/Z swap. It consumes
     * generated palm proxy space directly: X=fingers, Y=palm depth, Z=across palm.
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

    inline RE::NiPoint3 transformGrabAuthorityLocalDirection(const RE::NiTransform& proxyFrameWorld, const RE::NiPoint3& localDirection)
    {
        return normalizeDirection(transform_math::localVectorToWorld(proxyFrameWorld, localDirection));
    }

    inline RE::NiPoint3 computeGrabAuthorityFingerForwardWorld(const RE::NiTransform& proxyFrameWorld)
    {
        return transformGrabAuthorityLocalDirection(proxyFrameWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
    }

    inline RE::NiPoint3 computeGrabAuthorityPalmFaceWorld(const RE::NiTransform& proxyFrameWorld)
    {
        /*
         * Generated palm authority frames use local Y as palm depth. In the
         * working pivot tuning, -Y is palm-face for both hands; keep pocket and
         * pinch math on that same axis instead of the legacy authored Y/Z swap.
         */
        return transformGrabAuthorityLocalDirection(proxyFrameWorld, RE::NiPoint3{ 0.0f, -1.0f, 0.0f });
    }

    inline RE::NiPoint3 computeGrabAuthorityAcrossPalmWorld(const RE::NiTransform& proxyFrameWorld)
    {
        return transformGrabAuthorityLocalDirection(proxyFrameWorld, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
    }

    inline RE::NiPoint3 transformGrabAuthorityTuningDirection(const RE::NiTransform& proxyFrameWorld, const RE::NiPoint3& localDirection)
    {
        return transformGrabAuthorityLocalDirection(proxyFrameWorld, localDirection);
    }

    inline RE::NiTransform makeGrabStartupCaptureAuthorityFrame(const RE::NiTransform& rawHandWorld, const RE::NiTransform& palmAnchorWorld)
    {
        /*
         * Grab startup has a narrower problem than runtime held motion: the
         * configured proxy-local seat offset must be interpreted in controller
         * space when choosing and freezing the first pivot point. The actual
         * hidden proxy body still follows the existing generated palm frame
         * after the grab is live, so do not use this helper for held updates.
         */
        RE::NiTransform result = palmAnchorWorld;
        result.rotate = rawHandWorld.rotate;
        result.scale = rawHandWorld.scale;
        return result;
    }

    inline RE::NiTransform applyGrabAuthorityProxyLocalOffsetToFrame(const RE::NiTransform& proxyFrameWorld, bool isLeft)
    {
        /*
         * This moves only the hidden grab authority proxy/seat point. The real
         * generated palm/finger colliders remain bound to the flattened bone tree
         * so contact evidence and collision behavior are not retuned by visual
         * hand placement experiments.
         */
        RE::NiTransform result = proxyFrameWorld;
        const RE::NiPoint3 localOffset = computeGrabAuthorityProxyOffsetLocalGame(isLeft);
        if (std::isfinite(localOffset.x) && std::isfinite(localOffset.y) && std::isfinite(localOffset.z)) {
            result.translate = result.translate + transform_math::localVectorToWorld(proxyFrameWorld, localOffset);
        }
        return result;
    }
}
