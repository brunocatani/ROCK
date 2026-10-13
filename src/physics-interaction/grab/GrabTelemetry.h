#pragma once

/*
 * Grab telemetry helpers are grouped here to keep sampled transform diagnostics and overlay formatting separate from grab authority.
 */

// ---- GrabTransformTelemetry.h ----

/*
 * Grab transform telemetry is diagnostic-only: it compares ROCK's live grab
 * frames against captured object/hand-space relationships without changing the
 * active constraint, held-object target, or finger pose path. Keeping the math and stamped
 * formatting here lets the VR overlay and log rows report the same values, so an
 * in-game screenshot can be matched to the exact numeric frame in ROCK.log.
 */

#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/TransformMath.h"

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>

namespace rock::grab_transform_telemetry
{

    struct OrientationBasis
    {
        RE::NiPoint3 x{ 1.0f, 0.0f, 0.0f };
        RE::NiPoint3 y{ 0.0f, 1.0f, 0.0f };
        RE::NiPoint3 z{ 0.0f, 0.0f, 1.0f };
    };

    template <class Vector>
    inline float vectorLength(const Vector& value)
    {
        return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
    }

    inline RE::NiPoint3 normalizeDirectionOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
    {
        const float length = vectorLength(value);
        if (!std::isfinite(length) || length <= 1.0e-6f) {
            const float fallbackLength = vectorLength(fallback);
            if (!std::isfinite(fallbackLength) || fallbackLength <= 1.0e-6f) {
                return RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
            }
            return RE::NiPoint3{ fallback.x / fallbackLength, fallback.y / fallbackLength, fallback.z / fallbackLength };
        }

        return RE::NiPoint3{ value.x / length, value.y / length, value.z / length };
    }

    inline OrientationBasis makeOrientationBasis(const RE::NiTransform& transform)
    {
        /*
         * This is the native-mesh axis view: local X/Y/Z transformed through the
         * same verified FO4VR NiTransform convention used by collision/grab math.
         * It intentionally does not use matrix columns, because those are useful
         * low-level diagnostics but not the engine-authored "where local axis
         * points in world" view the grab snap needs.
         */
        OrientationBasis result{};
        result.x = normalizeDirectionOrFallback(
            transform_math::rotateLocalVectorToWorld(transform.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }),
            RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
        result.y = normalizeDirectionOrFallback(
            transform_math::rotateLocalVectorToWorld(transform.rotate, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }),
            RE::NiPoint3{ 0.0f, 1.0f, 0.0f });
        result.z = normalizeDirectionOrFallback(
            transform_math::rotateLocalVectorToWorld(transform.rotate, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }),
            RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
        return result;
    }

    template <class Vector>
    inline float directionDot(const Vector& a, const Vector& b)
    {
        const float aLength = vectorLength(a);
        const float bLength = vectorLength(b);
        if (aLength <= 1.0e-8f || bLength <= 1.0e-8f) {
            return 0.0f;
        }

        return (std::clamp)((a.x * b.x + a.y * b.y + a.z * b.z) / (aLength * bLength), -1.0f, 1.0f);
    }

    inline const RE::NiPoint3& basisAxis(const OrientationBasis& basis, int axis)
    {
        switch (axis) {
        case 1:
            return basis.y;
        case 2:
            return basis.z;
        default:
            return basis.x;
        }
    }

    inline const char* axisName(int axis)
    {
        switch (axis) {
        case 1:
            return "Y";
        case 2:
            return "Z";
        default:
            return "X";
        }
    }

    struct AxisBestMatch
    {
        int axis = 0;
        float dot = 0.0f;
        float absDot = 0.0f;
    };

    inline AxisBestMatch bestAxisMatch(const RE::NiPoint3& fromAxis, const OrientationBasis& to)
    {
        AxisBestMatch result{};
        for (int axis = 0; axis < 3; ++axis) {
            const float dot = directionDot(fromAxis, basisAxis(to, axis));
            const float absDot = std::fabs(dot);
            if (axis == 0 || absDot > result.absDot) {
                result.axis = axis;
                result.dot = dot;
                result.absDot = absDot;
            }
        }
        return result;
    }

    inline std::string formatBasisBestMap(const char* label, const OrientationBasis& from, const OrientationBasis& to)
    {
        const AxisBestMatch x = bestAxisMatch(from.x, to);
        const AxisBestMatch y = bestAxisMatch(from.y, to);
        const AxisBestMatch z = bestAxisMatch(from.z, to);
        char buffer[128]{};
        std::snprintf(buffer,
            sizeof(buffer),
            "%s X->%s%s %.2f Y->%s%s %.2f Z->%s%s %.2f",
            label ? label : "axisMap",
            x.dot < 0.0f ? "-" : "+",
            axisName(x.axis),
            x.absDot,
            y.dot < 0.0f ? "-" : "+",
            axisName(y.axis),
            y.absDot,
            z.dot < 0.0f ? "-" : "+",
            axisName(z.axis),
            z.absDot);
        return std::string(buffer);
    }

}
