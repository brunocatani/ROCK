#pragma once

#include <cmath>

#include "PhysicsUtils.h"
#include "RockConfig.h"

namespace frik::rock
{
    inline RE::NiPoint3 mirrorHandspaceZ(RE::NiPoint3 value, bool isLeft)
    {
        if (isLeft) {
            value.z *= -1.0f;
        }
        return value;
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

    inline RE::NiMatrix3 authoredHandspaceRotationFromRawHandBasis(const RE::NiMatrix3& rawRotation)
    {
        RE::NiMatrix3 result{};

        result.entry[0][0] = rawRotation.entry[0][1];
        result.entry[1][0] = rawRotation.entry[1][1];
        result.entry[2][0] = rawRotation.entry[2][1];

        result.entry[0][1] = rawRotation.entry[0][2];
        result.entry[1][1] = rawRotation.entry[1][2];
        result.entry[2][1] = rawRotation.entry[2][2];

        result.entry[0][2] = rawRotation.entry[0][0];
        result.entry[1][2] = rawRotation.entry[1][0];
        result.entry[2][2] = rawRotation.entry[2][0];
        return result;
    }

    inline RE::NiPoint3 transformHandspacePosition(const RE::NiTransform& handTransform, const RE::NiPoint3& localPosition, bool isLeft)
    {
        const RE::NiMatrix3 rigidRotation = authoredHandspaceRotationFromRawHandBasis(handTransform.rotate);
        return handTransform.translate + rigidRotation * (mirrorHandspaceZ(localPosition, isLeft) * handTransform.scale);
    }

    inline RE::NiPoint3 transformHandspaceDirection(const RE::NiTransform& handTransform, const RE::NiPoint3& localDirection, bool isLeft)
    {
        const RE::NiMatrix3 rigidRotation = authoredHandspaceRotationFromRawHandBasis(handTransform.rotate);
        return normalizeDirection(rigidRotation * mirrorHandspaceZ(localDirection, isLeft));
    }

    inline RE::NiTransform computeHandCollisionTransformFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
    {
        RE::NiTransform result = handTransform;
        result.rotate = authoredHandspaceRotationFromRawHandBasis(handTransform.rotate);
        result.translate = transformHandspacePosition(handTransform, g_rockConfig.rockHandCollisionOffsetHandspace * kHavokToGameScale, isLeft);
        return result;
    }

    inline RE::NiPoint3 computePalmPositionFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
    {
        return transformHandspacePosition(handTransform, g_rockConfig.rockPalmPositionHandspace, isLeft);
    }

    inline RE::NiPoint3 computePalmNormalFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
    {
        return transformHandspaceDirection(handTransform, g_rockConfig.rockPalmNormalHandspace, isLeft);
    }

    inline RE::NiPoint3 computePointingVectorFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
    {
        return transformHandspaceDirection(handTransform, g_rockConfig.rockPointingVectorHandspace, isLeft);
    }

    inline RE::NiPoint3 extractWandForward(const RE::NiMatrix3& rot) { return RE::NiPoint3(rot.entry[0][1], rot.entry[1][1], rot.entry[2][1]); }

    inline RE::NiPoint3 extractWandRight(const RE::NiMatrix3& rot) { return RE::NiPoint3(rot.entry[0][0], rot.entry[1][0], rot.entry[2][0]); }

    inline RE::NiPoint3 extractWandUp(const RE::NiMatrix3& rot) { return RE::NiPoint3(rot.entry[0][2], rot.entry[1][2], rot.entry[2][2]); }

    inline RE::NiPoint3 computePalmPosition(const RE::NiTransform& handTransform, bool isLeft)
    {
        RE::NiPoint3 forward = extractWandForward(handTransform.rotate);
        RE::NiPoint3 right = extractWandRight(handTransform.rotate);
        RE::NiPoint3 up = extractWandUp(handTransform.rotate);

        float rightSign = isLeft ? -1.0f : 1.0f;

        return handTransform.translate + forward * g_rockConfig.rockPalmOffsetForward + up * g_rockConfig.rockPalmOffsetUp + right * (g_rockConfig.rockPalmOffsetRight * rightSign);
    }

    inline RE::NiMatrix3 computePalmRotation(const RE::NiTransform& handTransform, bool isLeft)
    {
        (void)isLeft;
        return handTransform.rotate;
    }

    inline RE::NiPoint3 computePalmForward(const RE::NiTransform& handTransform, bool isLeft)
    {
        (void)isLeft;
        return extractWandForward(handTransform.rotate);
    }
}
