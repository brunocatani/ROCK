#pragma once

#include <cstdint>

namespace rock::debug_overlay_policy
{
    /*
     * Debug body rendering is accurate but expensive because shape support
     * vertices are triangulated and uploaded on the VR submit path. Keep the
     * visualizer explicit-ID based, then cap the high-cardinality weapon hulls
     * and first-time shape generation so diagnostics stay useful without making
     * every generated weapon part a per-frame cost spike.
     */
    inline bool shouldDrawHandBody(bool drawRockBodies, bool drawHandColliders)
    {
        return drawRockBodies && drawHandColliders;
    }

    inline bool shouldDrawWeaponBody(bool drawRockBodies, bool drawWeaponColliders, std::uint32_t weaponIndex, int maxWeaponBodiesDrawn)
    {
        if (!drawRockBodies || !drawWeaponColliders || maxWeaponBodiesDrawn <= 0) {
            return false;
        }
        return weaponIndex < static_cast<std::uint32_t>(maxWeaponBodiesDrawn);
    }

    inline std::uint32_t clampShapeGenerationsPerFrame(int requested)
    {
        if (requested <= 0) {
            return 0;
        }
        if (requested > 32) {
            return 32;
        }
        return static_cast<std::uint32_t>(requested);
    }

    inline std::uint32_t clampMaxConvexSupportVertices(int requested)
    {
        if (requested < 4) {
            return 4;
        }
        if (requested > 256) {
            return 256;
        }
        return static_cast<std::uint32_t>(requested);
    }

    inline bool shouldUseBoundsForHeavyConvex(std::uint32_t supportVertexCount, int maxDetailedSupportVertices, bool boundsFallbackEnabled)
    {
        if (!boundsFallbackEnabled) {
            return false;
        }
        return supportVertexCount > clampMaxConvexSupportVertices(maxDetailedSupportVertices);
    }

    inline std::uint64_t mixOverlaySettingsKey(std::uint64_t seed, std::uint64_t value)
    {
        value += 0x9e3779b97f4a7c15ull;
        value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ull;
        value = (value ^ (value >> 27)) * 0x94d049bb133111ebull;
        value = value ^ (value >> 31);
        return seed ^ (value + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2));
    }

    /*
     * Overlay cache invalidation depends on every renderer-shape setting having
     * a distinct contribution. The older manual bit packing overlapped wide
     * fields, so some cap changes could leave cached hull meshes alive with the
     * wrong draw policy. Use a small deterministic hash instead of packed bit
     * ranges so adding body settings cannot collide by construction.
     */
    inline std::uint64_t makeOverlaySettingsKey(bool drawRockBodies,
        bool drawTargetBodies,
        bool drawAxes,
        bool drawMarkers,
        bool drawSkeleton,
        bool drawText,
        bool drawHandColliders,
        bool drawHandBoneColliders,
        bool drawWeaponColliders,
        int maxHandBoneBodiesDrawn,
        int maxWeaponBodiesDrawn,
        int maxShapeGenerationsPerFrame,
        int maxConvexSupportVertices,
        bool useBoundsForHeavyConvex)
    {
        std::uint64_t key = 0xcbf29ce484222325ull;
        key = mixOverlaySettingsKey(key, drawRockBodies ? 1ull : 0ull);
        key = mixOverlaySettingsKey(key, drawTargetBodies ? 1ull : 0ull);
        key = mixOverlaySettingsKey(key, drawAxes ? 1ull : 0ull);
        key = mixOverlaySettingsKey(key, drawMarkers ? 1ull : 0ull);
        key = mixOverlaySettingsKey(key, drawSkeleton ? 1ull : 0ull);
        key = mixOverlaySettingsKey(key, drawText ? 1ull : 0ull);
        key = mixOverlaySettingsKey(key, drawHandColliders ? 1ull : 0ull);
        key = mixOverlaySettingsKey(key, drawHandBoneColliders ? 1ull : 0ull);
        key = mixOverlaySettingsKey(key, drawWeaponColliders ? 1ull : 0ull);
        key = mixOverlaySettingsKey(key, static_cast<std::uint32_t>(maxHandBoneBodiesDrawn < 0 ? 0 : maxHandBoneBodiesDrawn));
        key = mixOverlaySettingsKey(key, static_cast<std::uint32_t>(maxWeaponBodiesDrawn < 0 ? 0 : maxWeaponBodiesDrawn));
        key = mixOverlaySettingsKey(key, clampShapeGenerationsPerFrame(maxShapeGenerationsPerFrame));
        key = mixOverlaySettingsKey(key, clampMaxConvexSupportVertices(maxConvexSupportVertices));
        key = mixOverlaySettingsKey(key, useBoundsForHeavyConvex ? 1ull : 0ull);
        return key;
    }
}
