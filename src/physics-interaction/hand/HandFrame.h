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
     * Authored Z maps to raw/proxy local Y, so runtime proxy local -Y is the
     * palm-face direction after bReversePalmNormal and local +Y is the back of
     * the hand. Keep this bridge explicit; treating runtime Z as palm depth was
     * the Y/Z swap that made frozen grab points depend on object orientation.
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

    inline RE::NiPoint3 normalizeDirectionOrFallback(RE::NiPoint3 value, RE::NiPoint3 fallback)
    {
        const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
        if (lengthSquared > 1.0e-8f && std::isfinite(lengthSquared)) {
            const float inverseLength = 1.0f / std::sqrt(lengthSquared);
            value.x *= inverseLength;
            value.y *= inverseLength;
            value.z *= inverseLength;
            return value;
        }

        const float fallbackLengthSquared = fallback.x * fallback.x + fallback.y * fallback.y + fallback.z * fallback.z;
        if (fallbackLengthSquared <= 1.0e-8f || !std::isfinite(fallbackLengthSquared)) {
            return RE::NiPoint3(0.0f, -1.0f, 0.0f);
        }

        const float inverseFallbackLength = 1.0f / std::sqrt(fallbackLengthSquared);
        fallback.x *= inverseFallbackLength;
        fallback.y *= inverseFallbackLength;
        fallback.z *= inverseFallbackLength;
        return fallback;
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

    inline RE::NiPoint3 computePalmNormalFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
    {
        /*
         * Close selection uses the same palm-face convention proven by the working
         * far ray: authored Z is the collider's palm-thickness normal. The live
         * ROCK root-flattened game hand frame already carries left/right handedness,
         * so applying a separate authored Y mirror sends close detection across the palm.
         */
        const RE::NiPoint3 authoredNormal = g_rockConfig.rockPalmNormalHandspace;
        const RE::NiPoint3 rawNormal = authoredHandspaceToRawHandspaceForHand(authoredNormal, isLeft);
        RE::NiPoint3 normal = transformHandspaceLocalToWorld(handTransform, rawNormal);
        if (g_rockConfig.rockReversePalmNormal) {
            normal.x *= -1.0f;
            normal.y *= -1.0f;
            normal.z *= -1.0f;
        }
        const RE::NiPoint3 fallbackPalmFace =
            normalizeDirectionOrFallback(transformHandspaceLocalToWorld(handTransform, RE::NiPoint3(0.0f, -1.0f, 0.0f)), RE::NiPoint3(0.0f, -1.0f, 0.0f));
        return normalizeDirectionOrFallback(normal, fallbackPalmFace);
    }

    inline RE::NiPoint3 computePointingVectorFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
    {
        return pointing_direction_math::applyFarGrabNormalReversal(
            transformHandspaceDirection(handTransform, g_rockConfig.rockPointingVectorHandspace, isLeft), g_rockConfig.rockReverseFarGrabNormal);
    }

    inline RE::NiPoint3 computeGrabAuthorityProxyOffsetLocalGame(bool isLeft)
    {
        return isLeft ? g_rockConfig.rockLeftGrabAuthorityProxyOffsetGameUnits : g_rockConfig.rockRightGrabAuthorityProxyOffsetGameUnits;
    }

    inline RE::NiTransform applyGrabAuthorityProxyLocalOffsetToFrame(const RE::NiTransform& proxyFrameWorld, bool isLeft)
    {
        /*
         * This moves only the hidden grab authority proxy/seat point. The
         * generated palm/finger colliders remain bound to the flattened bone
         * tree, while the explicit grab-authority adapter supplies a proxy frame
         * where local -Y is the palm-face seat direction.
         */
        RE::NiTransform result = proxyFrameWorld;
        const RE::NiPoint3 localOffset = computeGrabAuthorityProxyOffsetLocalGame(isLeft);
        if (std::isfinite(localOffset.x) && std::isfinite(localOffset.y) && std::isfinite(localOffset.z)) {
            result.translate = result.translate + transform_math::localVectorToWorld(proxyFrameWorld, localOffset);
        }
        return result;
    }

    inline RE::NiPoint3 computeFallbackGrabAuthorityProxySeatWorld(const RE::NiTransform& rawHandWorld, bool isLeft)
    {
        /*
         * When the live generated palm body is unavailable, callers still use the
         * active proxy-seat offset rather than resurrecting the retired authored
         * hand-space pivot. This keeps fallback diagnostics and API consumers on
         * the same local -Y palm-seat convention as the real hidden proxy.
         */
        return applyGrabAuthorityProxyLocalOffsetToFrame(rawHandWorld, isLeft).translate;
    }
}
