#include "physics-interaction/hand/grab/HandGrabSupportModel.h"

#include "physics-interaction/hand/grab/HandGrabMath.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/TransformMath.h"
#include "RockConfig.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rock::hand_grab_detail
{
    namespace
    {
        struct LocalMeshLongAxis
        {
            RE::NiPoint3 axisWorld{};
            float spanGameUnits = 0.0f;
            bool valid = false;
        };
    }

    LocalMeshLongAxis computeLocalMeshLongAxis(
        const std::vector<GrabLocalTriangle>& localTriangles,
        const RE::NiTransform& objectWorldTransform)
    {
        LocalMeshLongAxis result{};
        if (localTriangles.empty() || !grab_three_phase::isFinite(objectWorldTransform)) {
            return result;
        }
    
        RE::NiPoint3 minPoint{
            (std::numeric_limits<float>::max)(),
            (std::numeric_limits<float>::max)(),
            (std::numeric_limits<float>::max)(),
        };
        RE::NiPoint3 maxPoint{
            -(std::numeric_limits<float>::max)(),
            -(std::numeric_limits<float>::max)(),
            -(std::numeric_limits<float>::max)(),
        };
    
        auto visit = [&](const RE::NiPoint3& point) {
            minPoint.x = (std::min)(minPoint.x, point.x);
            minPoint.y = (std::min)(minPoint.y, point.y);
            minPoint.z = (std::min)(minPoint.z, point.z);
            maxPoint.x = (std::max)(maxPoint.x, point.x);
            maxPoint.y = (std::max)(maxPoint.y, point.y);
            maxPoint.z = (std::max)(maxPoint.z, point.z);
        };
        for (const auto& triangle : localTriangles) {
            visit(triangle.v0);
            visit(triangle.v1);
            visit(triangle.v2);
        }
    
        const RE::NiPoint3 extents = maxPoint - minPoint;
        if (!std::isfinite(extents.x) || !std::isfinite(extents.y) || !std::isfinite(extents.z)) {
            return result;
        }
    
        RE::NiPoint3 localAxis{ 1.0f, 0.0f, 0.0f };
        float spanLocal = extents.x;
        if (extents.y > spanLocal) {
            localAxis = RE::NiPoint3{ 0.0f, 1.0f, 0.0f };
            spanLocal = extents.y;
        }
        if (extents.z > spanLocal) {
            localAxis = RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
            spanLocal = extents.z;
        }
    
        const float objectScale =
            std::isfinite(objectWorldTransform.scale) && objectWorldTransform.scale > 0.0f ? objectWorldTransform.scale : 1.0f;
        result.axisWorld = normalizeOrZero(transform_math::localVectorToWorld(objectWorldTransform, localAxis));
        result.spanGameUnits = spanLocal * objectScale;
        result.valid = result.spanGameUnits > 0.001f && lengthSquared(result.axisWorld) > 0.000001f;
        return result;
    }
    
    bool supportHitMatchesResolvedBody(const SelectedObject& selection,
        const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
        std::uint32_t resolvedBodyId,
        const GrabSurfaceHit& hit)
    {
        if (!hit.valid || !hit.sourceNode) {
            return false;
        }
    
        const auto* ownerRecord = bodySet.findAcceptedRecordByOwnerNode(hit.sourceNode);
        return (ownerRecord && ownerRecord->bodyId == resolvedBodyId) ||
               acceptsSelectedMultibodyOwnerlessVisualMesh(selection,
                   bodySet,
                   resolvedBodyId,
                   hit.sourceNode,
                   ownerRecord);
    }
    
    void appendRuntimeGripSupportSample(RuntimeGripSupportModel& support,
        const RE::NiPoint3& point,
        const RE::NiPoint3& normal,
        grab_support_model_math::GripSupportRole role)
    {
        if (support.sampleCount >= support.samples.size() || !grab_three_phase::isFinite(point)) {
            return;
        }
    
        support.samples[support.sampleCount++] = grab_support_model_math::GripSupportSample<RE::NiPoint3>{
            .point = point,
            .normal = normalizeOrZero(normal),
            .role = role,
            .valid = true,
        };
    }
    
    void appendRuntimeGripSupportMeshProbe(RuntimeGripSupportModel& support,
        const SelectedObject& selection,
        const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
        std::uint32_t resolvedBodyId,
        const std::vector<GrabSurfaceTriangleData>& surfaceTriangles,
        const RE::NiPoint3& queryPointWorld,
        const RE::NiPoint3& preferredNormalWorld,
        float maxDistanceGameUnits,
        grab_support_model_math::GripSupportRole role)
    {
        if (surfaceTriangles.empty() || maxDistanceGameUnits <= 0.0f || !std::isfinite(maxDistanceGameUnits)) {
            return;
        }
    
        GrabSurfaceHit hit{};
        if (!findClosestGrabSurfaceHitToPointPositionOnly(surfaceTriangles,
                queryPointWorld,
                preferredNormalWorld,
                maxDistanceGameUnits,
                hit)) {
            ++support.rejectedDistanceCount;
            return;
        }
        if (!supportHitMatchesResolvedBody(selection, bodySet, resolvedBodyId, hit)) {
            ++support.rejectedOwnerCount;
            return;
        }
    
        ++support.meshProbeHitCount;
        /*
         * findClosestGrabSurfaceHitToPointPositionOnly orients the mesh
         * normal to the probe direction. Do not replace it with raw
         * triangle winding here; per-face winding is mesh data, not support
         * authority, and can manufacture false opposed support on one face.
         */
        appendRuntimeGripSupportSample(support, hit.position, hit.normal, role);
    }
    
    const char* gripSupportActivePointMode(grab_support_model_math::GripSupportKind kind)
    {
        switch (kind) {
        case grab_support_model_math::GripSupportKind::SinglePoint:
            return "gripSupportForcedSinglePoint";
        case grab_support_model_math::GripSupportKind::SameSurface:
            return "gripSupportForcedSameSurface";
        case grab_support_model_math::GripSupportKind::OpposedPinch:
            return "gripSupportOpposedPinch";
        case grab_support_model_math::GripSupportKind::LongHandleAxis:
            return "gripSupportLongHandleAxis";
        case grab_support_model_math::GripSupportKind::PalmWrap:
            return "gripSupportPalmWrap";
        default:
            return "gripSupportEvidenceOnly";
        }
    }
    
    bool forceRuntimeGripSupportAuthority(RuntimeGripSupportModel& support,
        const RE::NiPoint3& activeGripPointWorld,
        const RE::NiPoint3& activeGripNormalWorld,
        const RE::NiPoint3& fallbackNormalWorld,
        const RE::NiPoint3& fallbackAxisWorld,
        float confidenceFloor,
        const char* reason)
    {
        if (support.model.canAuthorPivot) {
            return true;
        }
        if (!grab_three_phase::isFinite(activeGripPointWorld)) {
            support.model.reason = "forcedSupportInvalidAnchor";
            return false;
        }
    
        auto supportNormal = normalizeOrZero(activeGripNormalWorld);
        if (lengthSquared(supportNormal) <= 0.000001f) {
            supportNormal = normalizeOrZero(fallbackNormalWorld);
        }
        if (lengthSquared(supportNormal) <= 0.000001f) {
            supportNormal = RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
        }
    
        auto supportAxis = normalizeOrZero(fallbackAxisWorld);
        if (lengthSquared(supportAxis) <= 0.000001f) {
            supportAxis = grab_contact_patch_math::stablePerpendicular(supportNormal);
        }
    
        if (support.model.kind == grab_support_model_math::GripSupportKind::None) {
            support.model.kind = support.sampleCount > 0 ?
                grab_support_model_math::GripSupportKind::SameSurface :
                grab_support_model_math::GripSupportKind::SinglePoint;
        }
        support.model.pivotPoint = activeGripPointWorld;
        support.model.supportNormal = supportNormal;
        support.model.supportAxis = supportAxis;
        support.model.pivotShiftGameUnits = 0.0f;
        support.model.acceptedSampleCount = (std::max)(support.model.acceptedSampleCount, static_cast<std::size_t>(support.sampleCount + 1));
        support.model.confidence = (std::max)(support.model.confidence, confidenceFloor);
        support.model.reason = reason ? reason : "forcedSupportGroupUpgrade";
        support.model.valid = true;
        support.model.canAuthorPivot = true;
        support.valid = true;
        return true;
    }
    
    RuntimeGripSupportModel buildRuntimeGripSupportModel(
        const SelectedObject& selection,
        const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
        std::uint32_t resolvedBodyId,
        const RE::NiTransform& objectWorldTransform,
        const std::vector<GrabSurfaceTriangleData>& surfaceTriangles,
        const std::vector<GrabLocalTriangle>& localMeshTriangles,
        const RuntimeGrabContactPatch& contactPatch,
        const RuntimePinchPocketCandidate& pinchPocket,
        const RE::NiPoint3& activeGripPointWorld,
        const RE::NiPoint3& activeGripNormalWorld,
        const RE::NiPoint3& grabPivotAWorld,
        const RE::NiPoint3& palmNormalWorld,
        const RE::NiPoint3& fingerAxisWorld,
        const RE::NiPoint3& acrossPalmAxisWorld,
        float longObjectLeverGameUnits)
    {
        RuntimeGripSupportModel support{};
        const RE::NiPoint3 palmNormal = normalizeOrZero(palmNormalWorld);
        const RE::NiPoint3 fingerAxis = normalizeOrZero(fingerAxisWorld);
        const RE::NiPoint3 acrossAxis = normalizeOrZero(acrossPalmAxisWorld);
        if (resolvedBodyId == INVALID_BODY_ID || !grab_three_phase::isFinite(activeGripPointWorld)) {
            support.model.reason = "invalidGripSupportInput";
            return support;
        }
    
        const std::uint32_t contactSampleCount =
            (std::min)(contactPatch.sampleCount, static_cast<std::uint32_t>(contactPatch.samples.size()));
        for (std::uint32_t i = 0; i < contactSampleCount; ++i) {
            const auto& sample = contactPatch.samples[i];
            if (!sample.accepted) {
                continue;
            }
            appendRuntimeGripSupportSample(support,
                sample.point,
                sample.normal,
                grab_support_model_math::GripSupportRole::ContactPatch);
        }
    
        const float configuredSpacing =
            std::isfinite(contactPatch.probeSpacingGameUnits) && contactPatch.probeSpacingGameUnits > 0.0f ?
                contactPatch.probeSpacingGameUnits :
                (std::max)(1.0f, g_rockConfig.rockGrabContactPatchProbeSpacingGameUnits);
        const float supportOffset = std::clamp(configuredSpacing * 1.25f, 1.0f, 6.0f);
        const float supportMaxDistance = (std::max)(
            supportOffset + (std::max)(1.0f, g_rockConfig.rockGrabContactPatchMeshSnapMaxDistanceGameUnits),
            (std::max)(g_rockConfig.rockGrabPocketRadiusGameUnits, supportOffset * 2.0f));
        auto appendOffsetProbe = [&](const RE::NiPoint3& axis,
                                  float sign,
                                  grab_support_model_math::GripSupportRole role) {
            if (lengthSquared(axis) <= 0.000001f) {
                return;
            }
            const RE::NiPoint3 query = grabPivotAWorld + axis * (supportOffset * sign);
            RE::NiPoint3 preferred = normalizeOrZero(activeGripPointWorld - query);
            if (lengthSquared(preferred) <= 0.000001f) {
                preferred = palmNormal;
            }
            appendRuntimeGripSupportMeshProbe(support,
                selection,
                bodySet,
                resolvedBodyId,
                surfaceTriangles,
                query,
                preferred,
                supportMaxDistance,
                role);
        };
    
        appendOffsetProbe(acrossAxis, 1.0f, grab_support_model_math::GripSupportRole::AcrossPositive);
        appendOffsetProbe(acrossAxis, -1.0f, grab_support_model_math::GripSupportRole::AcrossNegative);
        appendOffsetProbe(fingerAxis, 1.0f, grab_support_model_math::GripSupportRole::FingerForward);
        appendOffsetProbe(fingerAxis, -1.0f, grab_support_model_math::GripSupportRole::FingerBack);
    
        if (pinchPocket.valid) {
            const float pinchProbeDistance = (std::max)(
                currentPinchPocketConfig().maxPocketDistanceGameUnits,
                supportMaxDistance);
            appendRuntimeGripSupportMeshProbe(support,
                selection,
                bodySet,
                resolvedBodyId,
                surfaceTriangles,
                pinchPocket.thumbPadWorld,
                RE::NiPoint3{ -pinchPocket.pinchAxisWorld.x, -pinchPocket.pinchAxisWorld.y, -pinchPocket.pinchAxisWorld.z },
                pinchProbeDistance,
                grab_support_model_math::GripSupportRole::ThumbPad);
            appendRuntimeGripSupportMeshProbe(support,
                selection,
                bodySet,
                resolvedBodyId,
                surfaceTriangles,
                pinchPocket.indexPadWorld,
                pinchPocket.pinchAxisWorld,
                pinchProbeDistance,
                grab_support_model_math::GripSupportRole::IndexPad);
        }
    
        const auto longAxis = computeLocalMeshLongAxis(localMeshTriangles, objectWorldTransform);
        const float maxPivotShift = (std::max)(1.0f, supportMaxDistance);
        support.model = grab_support_model_math::buildGripSupportModel(grab_support_model_math::GripSupportModelInput<RE::NiPoint3>{
            .anchorPoint = activeGripPointWorld,
            .anchorNormal = activeGripNormalWorld,
            .palmNormal = palmNormal,
            .acrossPalmAxis = acrossAxis,
            .fingerAxis = fingerAxis,
            .pinchAxis = pinchPocket.valid ? pinchPocket.pinchAxisWorld : acrossAxis,
            .objectLongAxis = longAxis.valid ? longAxis.axisWorld : RE::NiPoint3{},
            .samples = support.samples.data(),
            .sampleCount = support.sampleCount,
            .longObjectLeverGameUnits = longObjectLeverGameUnits,
            .objectLongAxisSpanGameUnits = longAxis.valid ? longAxis.spanGameUnits : 0.0f,
            .smallObjectReferenceLeverGameUnits = g_rockConfig.rockGrabSmallObjectReferenceLeverGameUnits,
            .longObjectReferenceLeverGameUnits = g_rockConfig.rockGrabLongObjectReferenceLeverGameUnits,
            .maxPivotShiftGameUnits = maxPivotShift,
            .minOpposedSpanGameUnits = 0.75f,
            .pinchSeat = pinchPocket.valid,
        });
        support.valid =
            support.model.valid &&
            support.model.kind != grab_support_model_math::GripSupportKind::SinglePoint;
        return support;
    }
    GrabMeshLongAxisResult computeGrabMeshLongAxis(const std::vector<TriangleData>& worldTriangles)
    {
        GrabMeshLongAxisResult result{};
        result.triangleCount = static_cast<std::uint32_t>(worldTriangles.size());
        if (worldTriangles.empty()) {
            result.reason = "noTriangles";
            return result;
        }
    
        double weightSum = 0.0;
        double meanAccum[3] = {};
        auto isFinitePoint = [](const RE::NiPoint3& p) {
            return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
        };
        auto triangleArea = [](const TriangleData& tri) {
            const RE::NiPoint3 e0 = tri.v1 - tri.v0;
            const RE::NiPoint3 e1 = tri.v2 - tri.v0;
            const RE::NiPoint3 n{
                e0.y * e1.z - e0.z * e1.y,
                e0.z * e1.x - e0.x * e1.z,
                e0.x * e1.y - e0.y * e1.x,
            };
            return 0.5f * std::sqrt((std::max)(0.0f, n.x * n.x + n.y * n.y + n.z * n.z));
        };
        for (const auto& tri : worldTriangles) {
            if (!isFinitePoint(tri.v0) || !isFinitePoint(tri.v1) || !isFinitePoint(tri.v2)) {
                continue;
            }
            const float area = triangleArea(tri);
            if (!std::isfinite(area) || area <= 0.000001f) {
                continue;
            }
            weightSum += area;
            meanAccum[0] += static_cast<double>(area) * (tri.v0.x + tri.v1.x + tri.v2.x) / 3.0;
            meanAccum[1] += static_cast<double>(area) * (tri.v0.y + tri.v1.y + tri.v2.y) / 3.0;
            meanAccum[2] += static_cast<double>(area) * (tri.v0.z + tri.v1.z + tri.v2.z) / 3.0;
        }
        if (weightSum <= 0.000001) {
            result.reason = "degenerateMeshArea";
            return result;
        }
        const double mean[3] = { meanAccum[0] / weightSum, meanAccum[1] / weightSum, meanAccum[2] / weightSum };
    
        // Symmetric covariance: [xx, xy, xz, yy, yz, zz]
        double cov[6] = {};
        for (const auto& tri : worldTriangles) {
            if (!isFinitePoint(tri.v0) || !isFinitePoint(tri.v1) || !isFinitePoint(tri.v2)) {
                continue;
            }
            const float area = triangleArea(tri);
            if (!std::isfinite(area) || area <= 0.000001f) {
                continue;
            }
            const double vertexWeight = static_cast<double>(area) / 3.0;
            const RE::NiPoint3* vertices[3] = { &tri.v0, &tri.v1, &tri.v2 };
            for (const auto* vertex : vertices) {
                const double d[3] = { vertex->x - mean[0], vertex->y - mean[1], vertex->z - mean[2] };
                cov[0] += vertexWeight * d[0] * d[0];
                cov[1] += vertexWeight * d[0] * d[1];
                cov[2] += vertexWeight * d[0] * d[2];
                cov[3] += vertexWeight * d[1] * d[1];
                cov[4] += vertexWeight * d[1] * d[2];
                cov[5] += vertexWeight * d[2] * d[2];
            }
        }
        for (double& entry : cov) {
            entry /= weightSum;
        }
    
        auto covMultiply = [](const double m[6], const double v[3], double out[3]) {
            out[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
            out[1] = m[1] * v[0] + m[3] * v[1] + m[4] * v[2];
            out[2] = m[2] * v[0] + m[4] * v[1] + m[5] * v[2];
        };
        auto dominantEigen = [&covMultiply](const double m[6], double outAxis[3]) {
            // Deterministic start: the coordinate axis with the largest diagonal.
            double v[3] = {};
            if (m[0] >= m[3] && m[0] >= m[5]) {
                v[0] = 1.0;
            } else if (m[3] >= m[5]) {
                v[1] = 1.0;
            } else {
                v[2] = 1.0;
            }
            for (int iteration = 0; iteration < 48; ++iteration) {
                double next[3];
                covMultiply(m, v, next);
                const double lenSq = next[0] * next[0] + next[1] * next[1] + next[2] * next[2];
                if (!(lenSq > 1e-18)) {
                    break;
                }
                const double invLen = 1.0 / std::sqrt(lenSq);
                v[0] = next[0] * invLen;
                v[1] = next[1] * invLen;
                v[2] = next[2] * invLen;
            }
            double mv[3];
            covMultiply(m, v, mv);
            const double eigenvalue = mv[0] * v[0] + mv[1] * v[1] + mv[2] * v[2];
            outAxis[0] = v[0];
            outAxis[1] = v[1];
            outAxis[2] = v[2];
            return eigenvalue;
        };
    
        double axis1[3];
        const double lambda1 = dominantEigen(cov, axis1);
        if (!(lambda1 > 1e-9)) {
            result.reason = "degenerateCovariance";
            return result;
        }
        double deflated[6] = {
            cov[0] - lambda1 * axis1[0] * axis1[0],
            cov[1] - lambda1 * axis1[0] * axis1[1],
            cov[2] - lambda1 * axis1[0] * axis1[2],
            cov[3] - lambda1 * axis1[1] * axis1[1],
            cov[4] - lambda1 * axis1[1] * axis1[2],
            cov[5] - lambda1 * axis1[2] * axis1[2],
        };
        double axis2[3];
        const double lambda2 = (std::max)(0.0, dominantEigen(deflated, axis2));
    
        result.axisWorld = RE::NiPoint3{
            static_cast<float>(axis1[0]),
            static_cast<float>(axis1[1]),
            static_cast<float>(axis1[2]),
        };
        result.elongationRatio = static_cast<float>((std::min)(100.0, std::sqrt(lambda1 / (std::max)(lambda2, lambda1 * 1e-4))));
        if (lambda2 > 1e-9) {
            // Re-orthogonalize against axis1: deflation leaves numerical drift.
            const double axis12dot = axis2[0] * axis1[0] + axis2[1] * axis1[1] + axis2[2] * axis1[2];
            double axis2Ortho[3] = {
                axis2[0] - axis12dot * axis1[0],
                axis2[1] - axis12dot * axis1[1],
                axis2[2] - axis12dot * axis1[2],
            };
            const double axis2LenSq =
                axis2Ortho[0] * axis2Ortho[0] + axis2Ortho[1] * axis2Ortho[1] + axis2Ortho[2] * axis2Ortho[2];
            if (axis2LenSq > 1e-12) {
                const double invLen = 1.0 / std::sqrt(axis2LenSq);
                result.secondAxisWorld = RE::NiPoint3{
                    static_cast<float>(axis2Ortho[0] * invLen),
                    static_cast<float>(axis2Ortho[1] * invLen),
                    static_cast<float>(axis2Ortho[2] * invLen),
                };
                // Eigenvalues of a symmetric matrix sum to its trace, so
                // lambda3 needs no third power iteration.
                const double trace = cov[0] + cov[3] + cov[5];
                const double lambda3 = (std::max)(0.0, trace - lambda1 - lambda2);
                result.secondElongationRatio =
                    static_cast<float>((std::min)(100.0, std::sqrt(lambda2 / (std::max)(lambda3, lambda2 * 1e-4))));
            }
        }
        result.reason = "meshPrincipalAxis";
        result.valid = true;
        return result;
    }
}
