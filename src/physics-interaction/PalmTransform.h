#pragma once

#include <cmath>

#include "GrabAnchorMath.h"
#include "HandspaceConvention.h"
#include "PhysicsUtils.h"
#include "PointingDirectionMath.h"
#include "RockConfig.h"

namespace frik::rock
{
    inline RE::NiPoint3 authoredHandspaceToRawHandspace(RE::NiPoint3 value)
    {
        return handspace_convention::authoredToRaw(value, g_rockConfig.rockHandspaceBasisMode);
    }

    inline RE::NiPoint3 authoredHandspaceToRawHandspaceForHand(RE::NiPoint3 value, bool isLeft)
    {
        return handspace_convention::authoredToRawForHand(value, isLeft, g_rockConfig.rockHandspaceBasisMode);
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

    inline void setRotationColumns(RE::NiMatrix3& matrix, const RE::NiPoint3& xAxis, const RE::NiPoint3& yAxis, const RE::NiPoint3& zAxis)
    {
        matrix.entry[0][0] = xAxis.x;
        matrix.entry[1][0] = xAxis.y;
        matrix.entry[2][0] = xAxis.z;
        matrix.entry[0][1] = yAxis.x;
        matrix.entry[1][1] = yAxis.y;
        matrix.entry[2][1] = yAxis.z;
        matrix.entry[0][2] = zAxis.x;
        matrix.entry[1][2] = zAxis.y;
        matrix.entry[2][2] = zAxis.z;
        matrix.entry[0][3] = 0.0f;
        matrix.entry[1][3] = 0.0f;
        matrix.entry[2][3] = 0.0f;
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

    inline RE::NiMatrix3 computeAuthoredHandspaceRotation(const RE::NiTransform& handTransform)
    {
        const RE::NiPoint3 xAxis = normalizeDirection(transformHandspaceLocalToWorld(handTransform, authoredHandspaceToRawHandspace(RE::NiPoint3(1.0f, 0.0f, 0.0f))));
        const RE::NiPoint3 yAxis = normalizeDirection(transformHandspaceLocalToWorld(handTransform, authoredHandspaceToRawHandspace(RE::NiPoint3(0.0f, 1.0f, 0.0f))));
        const RE::NiPoint3 zAxis = normalizeDirection(transformHandspaceLocalToWorld(handTransform, authoredHandspaceToRawHandspace(RE::NiPoint3(0.0f, 0.0f, 1.0f))));

        RE::NiMatrix3 result{};
        setRotationColumns(result, xAxis, yAxis, zAxis);
        return result;
    }

    inline RE::NiTransform computeHandCollisionTransformFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
    {
        RE::NiTransform result = handTransform;
        result.rotate = computeAuthoredHandspaceRotation(handTransform);
        result.translate = transformHandspacePosition(handTransform, g_rockConfig.rockHandCollisionOffsetHandspace * kHavokToGameScale, isLeft);
        return result;
    }

    inline RE::NiPoint3 computePalmPositionFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
    {
        return transformHandspacePosition(handTransform, g_rockConfig.rockPalmPositionHandspace, isLeft);
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

    inline RE::NiPoint3 computeGrabPivotAHandspacePosition(bool isLeft)
    {
        return grab_anchor_math::makeGrabAnchorHandspaceForHand(
            g_rockConfig.rockPalmPositionHandspace, g_rockConfig.rockGrabPivotAOffsetHandspace, g_rockConfig.rockReverseGrabPivotAOffset, isLeft);
    }

    inline RE::NiPoint3 computeGrabPivotAPositionFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
    {
        return transformHandspacePosition(handTransform, computeGrabPivotAHandspacePosition(isLeft), isLeft);
    }
}
