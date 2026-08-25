#pragma once

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/VectorMath.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/MeshGrab.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rock::surface_mesh_grab_policy
{
    enum class Failure : std::uint8_t
    {
        None = 0,
        Disabled,
        ContactPointUnavailable,
        TargetTransformUnavailable,
        OwnerNodeUnavailable,
        ExtractionEmpty,
        ExtractionBudgetExceeded,
        ProjectionMiss,
        ProjectionTooFar,
        PatchEmpty,
        FingerPoseUnavailable,
    };

    struct ProjectionInput
    {
        RE::NiTransform handWorld{};
        RE::NiPoint3 collisionPointWorld{};
        RE::NiPoint3 collisionNormalWorld{};
        RE::NiPoint3 meshPointWorld{};
        RE::NiPoint3 meshNormalWorld{};
        float maximumProjectionDistanceGameUnits = 48.0f;
        bool hasCollisionNormal = false;
        bool hasMeshNormal = false;
    };

    struct ProjectionResult
    {
        RE::NiTransform correctedHandWorld{};
        RE::NiPoint3 meshPointWorld{};
        RE::NiPoint3 meshNormalWorld{};
        RE::NiPoint3 shellToMeshDeltaWorld{};
        float shellToMeshDistanceGameUnits = 0.0f;
        bool valid = false;
    };

    [[nodiscard]] inline bool finitePoint(const RE::NiPoint3& point)
    {
        return vector_math::hasFiniteComponents(point);
    }

    [[nodiscard]] inline bool finiteTransform(const RE::NiTransform& value)
    {
        if (!finitePoint(value.translate) || !std::isfinite(value.scale) ||
            value.scale <= 0.0001f) {
            return false;
        }
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(value.rotate.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    [[nodiscard]] inline float lengthSquared(const RE::NiPoint3& value)
    {
        return vector_math::lengthSquared(value);
    }

    [[nodiscard]] inline RE::NiPoint3 normalizedOrZero(
        const RE::NiPoint3& value)
    {
        const float squared = lengthSquared(value);
        if (!std::isfinite(squared) || squared <= 1.0e-6f) {
            return {};
        }
        return value * (1.0f / std::sqrt(squared));
    }

    [[nodiscard]] inline ProjectionResult projectHandToMesh(
        const ProjectionInput& input)
    {
        ProjectionResult result{};
        if (!finiteTransform(input.handWorld) ||
            !finitePoint(input.collisionPointWorld) ||
            !finitePoint(input.meshPointWorld)) {
            return result;
        }
        const float maximumDistance = std::clamp(
            std::isfinite(input.maximumProjectionDistanceGameUnits) ?
                input.maximumProjectionDistanceGameUnits :
                48.0f,
            1.0f,
            128.0f);
        const RE::NiPoint3 delta =
            input.meshPointWorld - input.collisionPointWorld;
        const float distanceSquared = lengthSquared(delta);
        if (!std::isfinite(distanceSquared) ||
            distanceSquared > maximumDistance * maximumDistance) {
            return result;
        }

        result.correctedHandWorld = input.handWorld;
        result.correctedHandWorld.translate =
            input.handWorld.translate + delta;
        result.meshPointWorld = input.meshPointWorld;
        result.meshNormalWorld = input.hasMeshNormal ?
            normalizedOrZero(input.meshNormalWorld) :
            (input.hasCollisionNormal ?
                    normalizedOrZero(input.collisionNormalWorld) :
                    RE::NiPoint3{});
        result.shellToMeshDeltaWorld = delta;
        result.shellToMeshDistanceGameUnits = std::sqrt(distanceSquared);
        result.valid = finiteTransform(result.correctedHandWorld) &&
                       finitePoint(result.meshNormalWorld);
        return result;
    }

    struct RankedTriangle
    {
        std::size_t index = 0;
        float distanceSquared = 0.0f;
    };

    [[nodiscard]] inline std::vector<GrabLocalTriangle>
        buildTargetLocalPatch(
            const std::vector<TriangleData>& worldTriangles,
            const RE::NiTransform& targetWorld,
            const RE::NiPoint3& meshPointWorld,
            std::size_t maximumPatchTriangles)
    {
        std::vector<GrabLocalTriangle> patch;
        if (worldTriangles.empty() || !finiteTransform(targetWorld) ||
            !finitePoint(meshPointWorld) || maximumPatchTriangles == 0) {
            return patch;
        }

        const std::size_t retained = (std::min)(
            worldTriangles.size(),
            maximumPatchTriangles);
        std::vector<RankedTriangle> ranked;
        ranked.reserve(worldTriangles.size());
        for (std::size_t index = 0; index < worldTriangles.size(); ++index) {
            const auto& triangle = worldTriangles[index];
            float distanceSquared = 0.0f;
            (void)closestPointOnTriangleToPoint(
                meshPointWorld,
                triangle,
                distanceSquared);
            if (!std::isfinite(distanceSquared)) {
                continue;
            }
            ranked.push_back(RankedTriangle{
                .index = index,
                .distanceSquared = distanceSquared,
            });
        }
        if (ranked.empty()) {
            return patch;
        }
        const std::size_t rankedRetained = (std::min)(retained, ranked.size());
        std::partial_sort(
            ranked.begin(),
            ranked.begin() + rankedRetained,
            ranked.end(),
            [](const RankedTriangle& left, const RankedTriangle& right) {
                if (left.distanceSquared == right.distanceSquared) {
                    return left.index < right.index;
                }
                return left.distanceSquared < right.distanceSquared;
            });

        patch.reserve(rankedRetained);
        for (std::size_t rank = 0; rank < rankedRetained; ++rank) {
            const auto& triangle = worldTriangles[ranked[rank].index];
            const GrabLocalTriangle local{
                .v0 = transform_math::worldPointToLocal(
                    targetWorld,
                    triangle.v0),
                .v1 = transform_math::worldPointToLocal(
                    targetWorld,
                    triangle.v1),
                .v2 = transform_math::worldPointToLocal(
                    targetWorld,
                    triangle.v2),
            };
            if (!finitePoint(local.v0) || !finitePoint(local.v1) ||
                !finitePoint(local.v2)) {
                continue;
            }
            patch.push_back(local);
        }
        return patch;
    }
}
