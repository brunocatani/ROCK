#pragma once

/*
 * HIGGS dynamic grabs keep physics authority on the constraint and visual hand
 * authority on the reverse object solve: adjustedHand = heldNodeWorld *
 * inverse(desiredNodeTransformHandSpace). ROCK has a split authority model,
 * where the object is constrained through the live hand-body/palm frame while
 * FRIK consumes a raw visual hand-bone target. The split-frame reverse solve
 * recovers the hand-body target from the held object, then reapplies the
 * grab-start hand-body-to-raw correction so visual rotation follows the same
 * live basis that owns the constraint instead of reversing through a stale raw
 * object frame.
 */

#include "TransformMath.h"

#include <cmath>

namespace frik::rock::hand_visual_authority_math
{
    template <class Transform>
    inline Transform buildHiggsReverseAlignedHandWorld(const Transform& heldNodeWorld, const Transform& rawHandSpace)
    {
        return transform_math::composeTransforms(heldNodeWorld, transform_math::invertTransform(rawHandSpace));
    }

    template <class Transform>
    inline Transform buildAppliedVisualAuthorityHandWorld(const Transform& heldVisualNodeWorld,
        const Transform& rawHandSpace,
        const Transform&,
        const Transform&,
        const Transform&)
    {
        /*
         * The visible FRIK hand is a visual-node contract, not a physics-body
         * contract. HIGGS reverse-solves the hand from the held visual node and
         * the raw object-in-hand frame captured at grab time. ROCK keeps the
         * body-derived split frame for constraints and diagnostics, but the
         * applied visual IK target must ignore that body-derived frame so object
         * physics cannot invert or counter-rotate the rendered hand.
         */
        return buildHiggsReverseAlignedHandWorld(heldVisualNodeWorld, rawHandSpace);
    }

    template <class Transform>
    inline Transform buildSplitFrameReverseAlignedHandWorld(const Transform& heldNodeWorld,
        const Transform& constraintHandSpace,
        const Transform& handBodyToRawHandAtGrab)
    {
        const Transform handBodyTargetWorld = transform_math::composeTransforms(heldNodeWorld, transform_math::invertTransform(constraintHandSpace));
        return transform_math::composeTransforms(handBodyTargetWorld, handBodyToRawHandAtGrab);
    }

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
    inline Vector add(const Vector& lhs, const Vector& rhs)
    {
        return makeVector<Vector>(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }

    template <class Vector>
    inline Vector sub(const Vector& lhs, const Vector& rhs)
    {
        return makeVector<Vector>(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    template <class Vector>
    inline Vector mul(const Vector& value, float scale)
    {
        return makeVector<Vector>(value.x * scale, value.y * scale, value.z * scale);
    }

    template <class Vector>
    inline Vector negate(const Vector& value)
    {
        return makeVector<Vector>(-value.x, -value.y, -value.z);
    }

    template <class Vector>
    inline float dot(const Vector& lhs, const Vector& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    template <class Vector>
    inline Vector cross(const Vector& lhs, const Vector& rhs)
    {
        return makeVector<Vector>(
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x);
    }

    template <class Vector>
    inline float lengthSquared(const Vector& value)
    {
        return dot(value, value);
    }

    template <class Vector>
    inline Vector normalizeOrZero(const Vector& value)
    {
        const float lenSq = lengthSquared(value);
        if (lenSq <= 1.0e-8f || !std::isfinite(lenSq)) {
            return {};
        }

        const float invLen = 1.0f / std::sqrt(lenSq);
        return mul(value, invLen);
    }

    template <class Vector>
    inline Vector projectOntoPlane(const Vector& value, const Vector& normal)
    {
        return sub(value, mul(normal, dot(value, normal)));
    }

    template <class Vector>
    inline Vector stablePerpendicular(const Vector& normal)
    {
        const Vector xAxis = makeVector<Vector>(1.0f, 0.0f, 0.0f);
        const Vector yAxis = makeVector<Vector>(0.0f, 1.0f, 0.0f);
        const Vector zAxis = makeVector<Vector>(0.0f, 0.0f, 1.0f);
        const Vector hint = std::fabs(dot(normal, xAxis)) < 0.85f ? xAxis : (std::fabs(dot(normal, yAxis)) < 0.85f ? yAxis : zAxis);
        return normalizeOrZero(projectOntoPlane(hint, normal));
    }

    template <class Vector>
    struct PalmContactFrame
    {
        Vector normal{};
        Vector tangent{};
        Vector bitangent{};
        bool valid = false;
    };

    template <class Vector>
    inline PalmContactFrame<Vector> makePalmContactFrame(const Vector& normalInput, const Vector& tangentHint)
    {
        PalmContactFrame<Vector> frame{};
        frame.normal = normalizeOrZero(normalInput);
        if (lengthSquared(frame.normal) <= 0.0f) {
            return frame;
        }

        frame.tangent = normalizeOrZero(projectOntoPlane(tangentHint, frame.normal));
        if (lengthSquared(frame.tangent) <= 0.0f) {
            frame.tangent = stablePerpendicular(frame.normal);
        }

        frame.bitangent = normalizeOrZero(cross(frame.normal, frame.tangent));
        if (lengthSquared(frame.bitangent) <= 0.0f) {
            return frame;
        }

        frame.tangent = normalizeOrZero(cross(frame.bitangent, frame.normal));
        frame.valid = lengthSquared(frame.tangent) > 0.0f;
        return frame;
    }

    template <class Vector>
    inline Vector mapBetweenPalmContactFrames(const Vector& localVector, const PalmContactFrame<Vector>& localFrame, const PalmContactFrame<Vector>& worldFrame)
    {
        const float normalCoord = dot(localVector, localFrame.normal);
        const float tangentCoord = dot(localVector, localFrame.tangent);
        const float bitangentCoord = dot(localVector, localFrame.bitangent);
        return add(add(mul(worldFrame.normal, normalCoord), mul(worldFrame.tangent, tangentCoord)), mul(worldFrame.bitangent, bitangentCoord));
    }

    template <class Matrix, class Vector>
    inline void setNiRowsFromLocalToWorldBasis(Matrix& matrix, const Vector& row0, const Vector& row1, const Vector& row2)
    {
        matrix.entry[0][0] = row0.x;
        matrix.entry[0][1] = row0.y;
        matrix.entry[0][2] = row0.z;
        matrix.entry[0][3] = 0.0f;
        matrix.entry[1][0] = row1.x;
        matrix.entry[1][1] = row1.y;
        matrix.entry[1][2] = row1.z;
        matrix.entry[1][3] = 0.0f;
        matrix.entry[2][0] = row2.x;
        matrix.entry[2][1] = row2.y;
        matrix.entry[2][2] = row2.z;
        matrix.entry[2][3] = 0.0f;
    }

    template <class Transform, class Vector>
    inline Transform buildHandBoneWorldFromContactFrame(const Vector& rawPalmPivotLocal,
        const Vector& rawPalmContactNormalLocal,
        const Vector& rawFingerTangentLocal,
        const Vector& contactPointWorld,
        const Vector& surfaceNormalWorld,
        const Vector& surfaceTangentWorld,
        float handScale)
    {
        Transform result = transform_math::makeIdentityTransform<Transform>();
        result.scale = (std::isfinite(handScale) && std::fabs(handScale) > 0.0001f) ? handScale : 1.0f;

        const auto localFrame = makePalmContactFrame(rawPalmContactNormalLocal, rawFingerTangentLocal);
        const auto worldFrame = makePalmContactFrame(negate(surfaceNormalWorld), surfaceTangentWorld);
        if (!localFrame.valid || !worldFrame.valid) {
            result.translate = contactPointWorld;
            return result;
        }

        const Vector row0 = mapBetweenPalmContactFrames(makeVector<Vector>(1.0f, 0.0f, 0.0f), localFrame, worldFrame);
        const Vector row1 = mapBetweenPalmContactFrames(makeVector<Vector>(0.0f, 1.0f, 0.0f), localFrame, worldFrame);
        const Vector row2 = mapBetweenPalmContactFrames(makeVector<Vector>(0.0f, 0.0f, 1.0f), localFrame, worldFrame);
        setNiRowsFromLocalToWorldBasis(result.rotate, row0, row1, row2);

        const Vector pivotOffsetWorld = transform_math::localVectorToWorld(result, rawPalmPivotLocal);
        result.translate = sub(contactPointWorld, pivotOffsetWorld);
        return result;
    }
}
