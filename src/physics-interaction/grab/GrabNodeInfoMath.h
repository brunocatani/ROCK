#pragma once

/*
 * ROCK's authored grab-node diagnostics mirror HIGGS' NifSkope workflow: once
 * a grab has a good hand/object relationship, compute the object-local marker
 * transform that would reproduce that relationship as ROCK:GrabR/L. Keeping
 * this as pure transform math lets the logger validate the same convention used
 * by authored nodes without coupling tests to runtime Havok or FRIK state.
 */

#include "physics-interaction/TransformMath.h"

#include <algorithm>
#include <cmath>

namespace rock::grab_node_info_math
{
    template <class Transform, class Vector>
    inline Transform buildPalmTargetFrame(const Transform& handWorldTransform, const Vector& grabPivotWorld)
    {
        Transform target = handWorldTransform;
        target.translate = grabPivotWorld;
        return target;
    }

    template <class Transform>
    inline Transform buildDesiredObjectWorldFromAuthoredGrabNode(const Transform& objectWorldTransform,
        const Transform& grabNodeWorld,
        const Transform& handWorldTransform,
        const decltype(objectWorldTransform.translate)& grabPivotWorld)
    {
        const Transform grabNodeLocal = transform_math::composeTransforms(transform_math::invertTransform(objectWorldTransform), grabNodeWorld);
        const Transform palmTargetWorld = buildPalmTargetFrame(handWorldTransform, grabPivotWorld);
        Transform desiredObjectWorld = transform_math::composeTransforms(palmTargetWorld, transform_math::invertTransform(grabNodeLocal));
        desiredObjectWorld.scale = objectWorldTransform.scale;
        return desiredObjectWorld;
    }

    template <class Transform>
    inline Transform computeGrabNodeLocalTransformForCurrentGrab(const Transform& desiredObjectWorld,
        const Transform& handWorldTransform,
        const decltype(desiredObjectWorld.translate)& grabPivotWorld)
    {
        const Transform palmTargetWorld = buildPalmTargetFrame(handWorldTransform, grabPivotWorld);
        Transform local = transform_math::composeTransforms(transform_math::invertTransform(desiredObjectWorld), palmTargetWorld);
        local.scale = 1.0f;
        return local;
    }

    template <class Vector>
    inline Vector radiansToDegrees(const Vector& radians)
    {
        constexpr float kRadiansToDegrees = 57.29577951308232f;
        return Vector{ radians.x * kRadiansToDegrees, radians.y * kRadiansToDegrees, radians.z * kRadiansToDegrees };
    }

	inline float sanitizeZero(float value)
	{
		return std::abs(value) < 0.00001f ? 0.0f : value;
	}

    template <class Matrix, class Vector>
    inline Vector nifskopeMatrixToEulerRadians(const Matrix& matrix)
    {
        Vector out{};
        const float m02 = std::clamp(matrix.entry[0][2], -1.0f, 1.0f);

        if (m02 < 1.0f) {
            if (m02 > -1.0f) {
                out.x = std::atan2(-matrix.entry[1][2], matrix.entry[2][2]);
                out.y = std::asin(m02);
                out.z = std::atan2(-matrix.entry[0][1], matrix.entry[0][0]);
            } else {
                out.x = -std::atan2(-matrix.entry[1][0], matrix.entry[1][1]);
                out.y = -1.5707963267948966f;
                out.z = 0.0f;
            }
        } else {
            out.x = std::atan2(matrix.entry[1][0], matrix.entry[1][1]);
            out.y = 1.5707963267948966f;
            out.z = 0.0f;
        }

        out.x = sanitizeZero(out.x);
        out.y = sanitizeZero(out.y);
        out.z = sanitizeZero(out.z);
        return out;
    }

    template <class Matrix>
    inline auto nifskopeMatrixToEulerDegrees(const Matrix& matrix)
    {
        struct Vector
        {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
        };
        return radiansToDegrees(nifskopeMatrixToEulerRadians<Matrix, Vector>(matrix));
    }
}
