#pragma once

#include <cstdint>

namespace frik::rock::debug_overlay_policy
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
}
