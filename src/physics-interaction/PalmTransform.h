#pragma once

#include <cmath>

#include "HandspaceConvention.h"
#include "PointingDirectionMath.h"
#include "RockConfig.h"

namespace frik::rock
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
        // the transposed matrix to match engine-authored nodes and HIGGS' NiTransform path.
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
         * FO4/FRIK hand node already carries left/right handedness, so applying a
         * separate authored Y mirror sends close detection across the palm.
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

    inline RE::NiPoint3 computeGrabPivotAPositionFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
    {
        return transformHandspacePosition(handTransform, computeGrabPivotAHandspacePosition(isLeft), isLeft);
    }
}
