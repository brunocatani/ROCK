#pragma once

#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponHierarchy.h"
#include "physics-interaction/weapon/WeaponTriangleIndex.h"

#include <span>

namespace rock::weapon_interaction_query
{
    // Borrowed immutable geometry and one current pose. The caller owns this
    // snapshot only for consecutive queries with no intervening weapon update.
    struct Part
    {
        const std::vector<TriangleData>* triangles = nullptr;
        const WeaponTriangleIndex* index = nullptr;
        RE::NiTransform world{};
        RE::NiPoint3 boundsMin{}, boundsMax{};
        float absoluteScale = 0;
        float diagonalSquaredGame = 0;
        std::uint8_t priority = 0;
    };

    inline bool finitePoint(const RE::NiPoint3& point)
    {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    }

    inline bool prepare(const std::vector<TriangleData>& triangles, const WeaponTriangleIndex& index,
        const RE::NiTransform& world, const RE::NiPoint3& boundsMin, const RE::NiPoint3& boundsMax,
        std::uint8_t priority, Part& out)
    {
        out = {};
        if (triangles.empty() || !weapon_hierarchy::weaponTransformFinite(world) ||
            std::abs(world.scale) <= 0.000001f || !finitePoint(boundsMin) || !finitePoint(boundsMax) ||
            boundsMin.x > boundsMax.x || boundsMin.y > boundsMax.y || boundsMin.z > boundsMax.z) return false;
        out.triangles = &triangles;
        out.index = &index;
        out.world = world;
        out.boundsMin = boundsMin;
        out.boundsMax = boundsMax;
        out.absoluteScale = std::abs(world.scale);
        out.diagonalSquaredGame = weapon_interaction_probe_math::aabbDiagonalSquared(boundsMin, boundsMax) *
            (out.absoluteScale * out.absoluteScale);
        out.priority = priority;
        return true;
    }

    struct Selection
    {
        std::size_t part = (std::numeric_limits<std::size_t>::max)();
        weapon_interaction_probe_math::ProbeCandidateRank rank{};
        std::uint32_t boundsCandidates = 0;
        std::uint32_t surfaceCandidates = 0;
        bool valid() const { return part != (std::numeric_limits<std::size_t>::max)(); }
    };

    inline Selection find(std::span<const Part> parts, const RE::NiPoint3& point, float radiusGame)
    {
        Selection result{};
        if (!finitePoint(point) || !std::isfinite(radiusGame) || radiusGame <= 0.0f) return result;
        for (std::size_t i = 0; i < parts.size(); ++i) {
            const auto& part = parts[i];
            if (!part.triangles || !part.index) continue;
            const float localRadius = radiusGame / part.absoluteScale;
            const auto localPoint = weapon_collision_geometry_math::worldPointToLocal(
                part.world.rotate, part.world.translate, part.world.scale, point);
            if (!finitePoint(localPoint)) continue;
            const auto boundsDistance = weapon_interaction_probe_math::pointAabbDistanceSquared(
                localPoint, part.boundsMin, part.boundsMax);
            if (!std::isfinite(boundsDistance) ||
                !weapon_interaction_probe_math::isWithinProbeRadiusSquared(boundsDistance, localRadius)) continue;
            ++result.boundsCandidates;
            const auto distance = part.index->nearestDistanceSquared(*part.triangles, localPoint, localRadius * localRadius,
                [](const RE::NiPoint3& p, const TriangleData& triangle) {
                    float squared = (std::numeric_limits<float>::infinity)();
                    (void)closestPointOnTriangleToPoint(p, triangle, squared);
                    return squared;
                });
            if (!std::isfinite(distance) ||
                !weapon_interaction_probe_math::isWithinProbeRadiusSquared(distance, localRadius)) continue;
            ++result.surfaceCandidates;
            const weapon_interaction_probe_math::ProbeCandidateRank rank{
                .distanceSquaredGame = distance * (part.absoluteScale * part.absoluteScale),
                .aabbDiagonalSquaredGame = part.diagonalSquaredGame,
                .semanticPriority = part.priority,
            };
            if (result.valid() && !weapon_interaction_probe_math::isBetterProbeCandidate(rank, result.rank)) continue;
            result.part = i;
            result.rank = rank;
        }
        return result;
    }
}
