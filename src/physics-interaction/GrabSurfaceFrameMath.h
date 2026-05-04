#pragma once

/*
 * Palm-face grab alignment is intentionally solved as pure transform math.
 * HIGGS uses graphics triangles to choose coherent contact/body ownership, then
 * keeps a captured object-in-hand frame. ROCK extends that by optionally rotating
 * the touched surface into a palm-facing frame, but the solver must stay separate
 * from Havok body memory and FRIK runtime state so it can be tested and reused.
 */

#include <algorithm>
#include <cmath>
#include <limits>

#include "TransformMath.h"

namespace frik::rock::grab_surface_frame_math
{
    enum class GrabOrientationMode
    {
        PreserveObjectRotation = 0,
        SurfaceNormalAuto = 1,
        AuthoredOnly = 2
    };

    enum class GrabSurfaceFaceKind
    {
        Side = 0,
        CapTopBottom = 1,
        NarrowEdge = 2,
        Ambiguous = 3
    };

    enum class GrabSurfaceTangentSource
    {
        TriangleLongestEdge = 0,
        PreservedObjectRoll = 1,
        AuthoredFrame = 2,
        ObjectLongAxis = 3,
        Fallback = 4,
        ContactPatchPrincipal = 5
    };

    enum class GrabSurfaceAlignmentDecision
    {
        Accepted = 0,
        RejectedMode = 1,
        RejectedMissingSelectionHit = 2,
        RejectedOwnerMismatch = 3,
        RejectedPivotDistance = 4,
        RejectedSelectionDistance = 5,
        RejectedLowConfidence = 6,
        RejectedAmbiguousTangent = 7
    };

    template <class Vector>
    struct GrabSurfaceFrame
    {
        Vector normal{};
        Vector tangent{};
        Vector bitangent{};
        GrabSurfaceFaceKind faceKind{ GrabSurfaceFaceKind::Ambiguous };
        GrabSurfaceTangentSource tangentSource{ GrabSurfaceTangentSource::Fallback };
        float confidence = 0.0f;
        const char* fallbackReason = "uninitialized";

        bool usable() const { return confidence > 0.0f && faceKind != GrabSurfaceFaceKind::Ambiguous; }
    };

    template <class Transform>
    struct DesiredObjectFrame
    {
        Transform transform{};
        GrabOrientationMode modeUsed{ GrabOrientationMode::PreserveObjectRotation };
        GrabSurfaceFaceKind faceKind{ GrabSurfaceFaceKind::Ambiguous };
        GrabSurfaceTangentSource tangentSource{ GrabSurfaceTangentSource::Fallback };
        GrabSurfaceAlignmentDecision alignmentDecision{ GrabSurfaceAlignmentDecision::RejectedMode };
        float confidence = 0.0f;
        const char* fallbackReason = "uninitialized";
    };

    template <class Vector>
    inline Vector add(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
    }

    template <class Vector>
    inline Vector sub(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
    }

    template <class Vector>
    inline Vector mul(const Vector& value, float scale)
    {
        return Vector{ value.x * scale, value.y * scale, value.z * scale };
    }

    template <class Vector>
    inline float dot(const Vector& lhs, const Vector& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    template <class Vector>
    inline Vector cross(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x };
    }

    template <class Vector>
    inline float lengthSquared(const Vector& value)
    {
        return dot(value, value);
    }

    template <class Vector>
    inline float length(const Vector& value)
    {
        return std::sqrt(lengthSquared(value));
    }

    template <class Vector>
    inline Vector normalizeOrZero(const Vector& value)
    {
        const float len = length(value);
        if (len <= 1.0e-6f || !std::isfinite(len)) {
            return Vector{};
        }
        return mul(value, 1.0f / len);
    }

    template <class Vector>
    inline Vector negate(const Vector& value)
    {
        return Vector{ -value.x, -value.y, -value.z };
    }

    template <class Vector>
    inline Vector projectOntoPlane(const Vector& value, const Vector& normal)
    {
        return sub(value, mul(normal, dot(value, normal)));
    }

    template <class Vector>
    inline Vector stablePerpendicular(const Vector& normal)
    {
        const Vector xAxis{ 1.0f, 0.0f, 0.0f };
        const Vector yAxis{ 0.0f, 1.0f, 0.0f };
        const Vector zAxis{ 0.0f, 0.0f, 1.0f };
        const Vector hint = std::fabs(dot(normal, xAxis)) < 0.85f ? xAxis : (std::fabs(dot(normal, yAxis)) < 0.85f ? yAxis : zAxis);
        return normalizeOrZero(projectOntoPlane(hint, normal));
    }

    template <class Vector>
    inline Vector chooseObjectDerivedTangentHint(const Vector& normal, const Vector& objectLongAxis)
    {
        Vector tangent = normalizeOrZero(projectOntoPlane(objectLongAxis, normal));
        if (lengthSquared(tangent) > 0.0f) {
            return tangent;
        }

        const Vector axes[3] = {
            Vector{ 1.0f, 0.0f, 0.0f },
            Vector{ 0.0f, 1.0f, 0.0f },
            Vector{ 0.0f, 0.0f, 1.0f },
        };
        const Vector n = normalizeOrZero(normal);
        int best = 0;
        float bestDot = (std::numeric_limits<float>::max)();
        for (int i = 0; i < 3; ++i) {
            const float axisDot = std::fabs(dot(axes[i], n));
            if (axisDot < bestDot) {
                bestDot = axisDot;
                best = i;
            }
        }
        return normalizeOrZero(projectOntoPlane(axes[best], normal));
    }

    template <class Vector>
    inline Vector chooseLongestEdgeTangent(
        const Vector& v0,
        const Vector& v1,
        const Vector& v2,
        const Vector& normal,
        float& outLongest,
        float& outSecondLongest)
    {
        const Vector edges[3] = { sub(v1, v0), sub(v2, v1), sub(v0, v2) };
        float lengths[3] = { lengthSquared(edges[0]), lengthSquared(edges[1]), lengthSquared(edges[2]) };
        int best = 0;
        int second = 1;
        if (lengths[1] > lengths[best]) {
            best = 1;
            second = 0;
        }
        if (lengths[2] > lengths[best]) {
            second = best;
            best = 2;
        } else if (lengths[2] > lengths[second]) {
            second = 2;
        }

        outLongest = std::sqrt((std::max)(0.0f, lengths[best]));
        outSecondLongest = std::sqrt((std::max)(0.0f, lengths[second]));
        return normalizeOrZero(projectOntoPlane(edges[best], normal));
    }

    template <class Vector>
    inline GrabSurfaceFrame<Vector> buildSurfaceFrameFromTriangle(const Vector& v0,
        const Vector& v1,
        const Vector& v2,
        const Vector& preservedTangent,
        const Vector& objectLongAxis,
        float capNormalDotThreshold = 0.85f,
        bool preserveRollForCaps = false)
    {
        GrabSurfaceFrame<Vector> result{};
        const Vector normal = normalizeOrZero(cross(sub(v1, v0), sub(v2, v1)));
        if (lengthSquared(normal) <= 0.0f) {
            result.normal = normalizeOrZero(objectLongAxis);
            result.tangent = stablePerpendicular(result.normal);
            result.bitangent = normalizeOrZero(cross(result.normal, result.tangent));
            result.faceKind = GrabSurfaceFaceKind::Ambiguous;
            result.tangentSource = GrabSurfaceTangentSource::Fallback;
            result.confidence = 0.0f;
            result.fallbackReason = "degenerateTriangle";
            return result;
        }

        result.normal = normal;
        const Vector longAxis = normalizeOrZero(objectLongAxis);
        const bool capFace = lengthSquared(longAxis) > 0.0f && std::fabs(dot(normal, longAxis)) >= capNormalDotThreshold;
        result.faceKind = capFace ? GrabSurfaceFaceKind::CapTopBottom : GrabSurfaceFaceKind::Side;

        Vector tangent{};
        if (capFace && preserveRollForCaps) {
            tangent = normalizeOrZero(projectOntoPlane(preservedTangent, normal));
            result.tangentSource = GrabSurfaceTangentSource::PreservedObjectRoll;
            result.confidence = 0.50f;
        }

        if (!capFace && lengthSquared(longAxis) > 0.0f) {
            tangent = normalizeOrZero(projectOntoPlane(longAxis, normal));
            if (lengthSquared(tangent) > 0.0f) {
                result.tangentSource = GrabSurfaceTangentSource::ObjectLongAxis;
                result.confidence = 1.0f;
            }
        }

        if (lengthSquared(tangent) <= 0.0f) {
            float longest = 0.0f;
            float secondLongest = 0.0f;
            tangent = chooseLongestEdgeTangent(v0, v1, v2, normal, longest, secondLongest);
            result.tangentSource = GrabSurfaceTangentSource::TriangleLongestEdge;
            result.confidence = 1.0f;
            if (secondLongest > 0.0f && longest / secondLongest < 1.15f) {
                result.faceKind = capFace ? GrabSurfaceFaceKind::CapTopBottom : GrabSurfaceFaceKind::NarrowEdge;
                result.confidence = capFace ? 0.50f : 0.65f;
            }
        }

        if (lengthSquared(tangent) <= 0.0f) {
            tangent = normalizeOrZero(projectOntoPlane(preservedTangent, normal));
            result.tangentSource = GrabSurfaceTangentSource::PreservedObjectRoll;
            result.confidence = 0.35f;
        }
        if (lengthSquared(tangent) <= 0.0f) {
            tangent = stablePerpendicular(normal);
            result.tangentSource = GrabSurfaceTangentSource::Fallback;
            result.confidence = 0.20f;
            result.fallbackReason = "noStableTriangleTangent";
        } else {
            result.fallbackReason = "none";
        }

        result.tangent = tangent;
        result.bitangent = normalizeOrZero(cross(result.normal, result.tangent));
        if (lengthSquared(result.bitangent) <= 0.0f) {
            result.faceKind = GrabSurfaceFaceKind::Ambiguous;
            result.tangentSource = GrabSurfaceTangentSource::Fallback;
            result.confidence = 0.0f;
            result.fallbackReason = "invalidFrame";
        }
        return result;
    }

    template <class Vector>
    inline GrabSurfaceFrame<Vector> buildSurfaceFrameFromNormal(const Vector& surfaceNormal,
        const Vector& preservedTangent,
        const Vector& objectLongAxis,
        float capNormalDotThreshold = 0.85f,
        bool preserveRollForCaps = false)
    {
        GrabSurfaceFrame<Vector> result{};
        const Vector normal = normalizeOrZero(surfaceNormal);
        if (lengthSquared(normal) <= 0.0f) {
            result.faceKind = GrabSurfaceFaceKind::Ambiguous;
            result.tangentSource = GrabSurfaceTangentSource::Fallback;
            result.confidence = 0.0f;
            result.fallbackReason = "invalidCollisionNormal";
            return result;
        }

        result.normal = normal;
        const Vector longAxis = normalizeOrZero(objectLongAxis);
        const bool capFace = lengthSquared(longAxis) > 0.0f && std::fabs(dot(normal, longAxis)) >= capNormalDotThreshold;
        result.faceKind = capFace ? GrabSurfaceFaceKind::CapTopBottom : GrabSurfaceFaceKind::Side;

        Vector tangent{};
        if (capFace && preserveRollForCaps) {
            tangent = normalizeOrZero(projectOntoPlane(preservedTangent, normal));
            result.tangentSource = GrabSurfaceTangentSource::PreservedObjectRoll;
            result.confidence = 0.50f;
        }

        if (!capFace && lengthSquared(longAxis) > 0.0f) {
            tangent = normalizeOrZero(projectOntoPlane(longAxis, normal));
            if (lengthSquared(tangent) > 0.0f) {
                result.tangentSource = GrabSurfaceTangentSource::ObjectLongAxis;
                result.confidence = 1.0f;
            }
        }

        if (lengthSquared(tangent) <= 0.0f) {
            tangent = normalizeOrZero(projectOntoPlane(preservedTangent, normal));
            result.tangentSource = GrabSurfaceTangentSource::PreservedObjectRoll;
            result.confidence = 0.65f;
        }
        if (lengthSquared(tangent) <= 0.0f) {
            tangent = stablePerpendicular(normal);
            result.tangentSource = GrabSurfaceTangentSource::Fallback;
            result.confidence = 0.20f;
            result.fallbackReason = "noStableCollisionTangent";
        } else {
            result.fallbackReason = "none";
        }

        result.tangent = tangent;
        result.bitangent = normalizeOrZero(cross(result.normal, result.tangent));
        if (lengthSquared(result.bitangent) <= 0.0f) {
            result.faceKind = GrabSurfaceFaceKind::Ambiguous;
            result.tangentSource = GrabSurfaceTangentSource::Fallback;
            result.confidence = 0.0f;
            result.fallbackReason = "invalidFrame";
        }
        return result;
    }

    template <class Vector>
    inline GrabSurfaceAlignmentDecision evaluateSurfaceAlignmentGate(const GrabSurfaceFrame<Vector>& localSurfaceFrame,
        GrabOrientationMode mode,
        bool hasSelectionHit,
        bool resolvedOwnerMatches,
        float pivotToSurfaceDistanceGameUnits,
        float selectionToMeshDistanceGameUnits,
        float maxPivotToSurfaceDistanceGameUnits,
        float maxSelectionToMeshDistanceGameUnits,
        bool requireResolvedOwnerMatch,
        float minConfidence = 0.20f)
    {
        if (mode != GrabOrientationMode::SurfaceNormalAuto) {
            return GrabSurfaceAlignmentDecision::RejectedMode;
        }
        if (!hasSelectionHit) {
            return GrabSurfaceAlignmentDecision::RejectedMissingSelectionHit;
        }
        if (requireResolvedOwnerMatch && !resolvedOwnerMatches) {
            return GrabSurfaceAlignmentDecision::RejectedOwnerMismatch;
        }
        if (maxPivotToSurfaceDistanceGameUnits >= 0.0f && pivotToSurfaceDistanceGameUnits > maxPivotToSurfaceDistanceGameUnits) {
            return GrabSurfaceAlignmentDecision::RejectedPivotDistance;
        }
        if (maxSelectionToMeshDistanceGameUnits >= 0.0f && selectionToMeshDistanceGameUnits > maxSelectionToMeshDistanceGameUnits) {
            return GrabSurfaceAlignmentDecision::RejectedSelectionDistance;
        }
        if (!localSurfaceFrame.usable() || localSurfaceFrame.confidence < minConfidence) {
            return GrabSurfaceAlignmentDecision::RejectedLowConfidence;
        }
        if (localSurfaceFrame.tangentSource == GrabSurfaceTangentSource::Fallback ||
            (localSurfaceFrame.faceKind != GrabSurfaceFaceKind::Side && localSurfaceFrame.faceKind != GrabSurfaceFaceKind::CapTopBottom)) {
            return GrabSurfaceAlignmentDecision::RejectedAmbiguousTangent;
        }
        return GrabSurfaceAlignmentDecision::Accepted;
    }

    template <class Matrix, class Vector>
    inline void setRows(Matrix& matrix, const Vector& row0, const Vector& row1, const Vector& row2)
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

    template <class Matrix, class Vector>
    inline Matrix buildRotationMappingLocalFrameToWorldFrame(const GrabSurfaceFrame<Vector>& localFrame,
        const Vector& targetNormal,
        const Vector& targetTangent,
        const Vector& targetBitangent)
    {
        Matrix result{};
        const Vector row0 = add(add(mul(targetNormal, localFrame.normal.x), mul(targetTangent, localFrame.tangent.x)), mul(targetBitangent, localFrame.bitangent.x));
        const Vector row1 = add(add(mul(targetNormal, localFrame.normal.y), mul(targetTangent, localFrame.tangent.y)), mul(targetBitangent, localFrame.bitangent.y));
        const Vector row2 = add(add(mul(targetNormal, localFrame.normal.z), mul(targetTangent, localFrame.tangent.z)), mul(targetBitangent, localFrame.bitangent.z));
        setRows(result, row0, row1, row2);
        return result;
    }

    template <class Transform, class Vector>
    inline Transform translateObjectToPutLocalPointAtWorldPoint(Transform objectWorld, const Vector& localPoint, const Vector& worldPoint)
    {
        const Vector localOffset = transform_math::localVectorToWorld(objectWorld, localPoint);
        objectWorld.translate = sub(worldPoint, localOffset);
        return objectWorld;
    }

    template <class Transform, class Vector>
    inline DesiredObjectFrame<Transform> buildDesiredObjectWorldFromSurfaceFrame(const Transform& objectWorld,
        const Vector& localSurfacePoint,
        const GrabSurfaceFrame<Vector>& localSurfaceFrame,
        const Vector& grabPivotWorld,
        const Vector& palmNormalWorld,
        const Vector& palmTangentWorld,
        GrabOrientationMode mode,
        float minConfidence = 0.20f)
    {
        DesiredObjectFrame<Transform> result{};
        result.transform = translateObjectToPutLocalPointAtWorldPoint(objectWorld, localSurfacePoint, grabPivotWorld);
        result.modeUsed = mode == GrabOrientationMode::AuthoredOnly ? GrabOrientationMode::AuthoredOnly : GrabOrientationMode::PreserveObjectRotation;
        result.faceKind = localSurfaceFrame.faceKind;
        result.tangentSource = localSurfaceFrame.tangentSource;
        result.confidence = localSurfaceFrame.confidence;
        result.fallbackReason = "preserveObjectRotation";
        result.alignmentDecision = mode == GrabOrientationMode::SurfaceNormalAuto ? GrabSurfaceAlignmentDecision::Accepted : GrabSurfaceAlignmentDecision::RejectedMode;

        if (mode != GrabOrientationMode::SurfaceNormalAuto) {
            result.alignmentDecision = GrabSurfaceAlignmentDecision::RejectedMode;
            return result;
        }
        if (!localSurfaceFrame.usable() || localSurfaceFrame.confidence < minConfidence) {
            result.modeUsed = GrabOrientationMode::PreserveObjectRotation;
            result.alignmentDecision = GrabSurfaceAlignmentDecision::RejectedLowConfidence;
            result.fallbackReason = localSurfaceFrame.fallbackReason ? localSurfaceFrame.fallbackReason : "surfaceFrameUnusable";
            return result;
        }

        const Vector targetNormal = normalizeOrZero(negate(palmNormalWorld));
        if (lengthSquared(targetNormal) <= 0.0f) {
            result.modeUsed = GrabOrientationMode::PreserveObjectRotation;
            result.alignmentDecision = GrabSurfaceAlignmentDecision::RejectedLowConfidence;
            result.fallbackReason = "invalidPalmNormal";
            return result;
        }

        Vector targetTangent{};
        if (localSurfaceFrame.faceKind == GrabSurfaceFaceKind::CapTopBottom && localSurfaceFrame.tangentSource == GrabSurfaceTangentSource::PreservedObjectRoll) {
            const Vector preservedWorldTangent = transform_math::rotateLocalVectorToWorld(objectWorld.rotate, localSurfaceFrame.tangent);
            targetTangent = normalizeOrZero(projectOntoPlane(preservedWorldTangent, targetNormal));
        }
        if (lengthSquared(targetTangent) <= 0.0f) {
            targetTangent = normalizeOrZero(projectOntoPlane(palmTangentWorld, targetNormal));
        }
        if (lengthSquared(targetTangent) <= 0.0f) {
            targetTangent = stablePerpendicular(targetNormal);
            result.tangentSource = GrabSurfaceTangentSource::Fallback;
        }

        const Vector targetBitangent = normalizeOrZero(cross(targetNormal, targetTangent));
        if (lengthSquared(targetBitangent) <= 0.0f) {
            result.modeUsed = GrabOrientationMode::PreserveObjectRotation;
            result.alignmentDecision = GrabSurfaceAlignmentDecision::RejectedLowConfidence;
            result.fallbackReason = "invalidPalmFrame";
            return result;
        }

        result.transform = objectWorld;
        result.transform.rotate = buildRotationMappingLocalFrameToWorldFrame<decltype(objectWorld.rotate), Vector>(
            localSurfaceFrame, targetNormal, targetTangent, targetBitangent);
        result.transform = translateObjectToPutLocalPointAtWorldPoint(result.transform, localSurfacePoint, grabPivotWorld);
        result.modeUsed = GrabOrientationMode::SurfaceNormalAuto;
        result.alignmentDecision = GrabSurfaceAlignmentDecision::Accepted;
        result.fallbackReason = "none";
        return result;
    }
}
