#pragma once

#include <cstdint>

namespace rock::debug_overlay_policy
{
    inline constexpr std::uint32_t kDefaultLineVertexBudget = 8192;
    inline constexpr std::uint32_t kMaxLineVertexBudget = 65536;
    inline constexpr std::uint32_t kDefaultShapeCacheBudget = 512;
    inline constexpr std::uint32_t kMaxShapeCacheBudget = 4096;
    inline constexpr std::uint32_t kMaxDetailedConvexSupportVertices = 32;
    inline constexpr std::uint32_t kDefaultMaxCompoundChildren = 256;
    inline constexpr std::uint32_t kMaxCompoundChildren = 1024;
    inline constexpr std::uint32_t kDefaultMaxCompoundDepth = 4;
    inline constexpr std::uint32_t kMaxCompoundDepth = 8;

    enum class ShapeDecodeMode : std::uint8_t
    {
        Detailed,
        Proxy,
        Unsupported
    };

    /*
     * Debug body rendering is accurate but expensive because shape support
     * vertices are triangulated and uploaded on the VR submit path. Keep the
     * visualizer explicit-ID based, then cap the high-cardinality weapon hulls
     * and first-time shape capture so diagnostics stay useful without making
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

    inline constexpr std::uint32_t clampMaxConvexSupportVertices(int requested)
    {
        if (requested < 4) {
            return 4;
        }
        if (requested > static_cast<int>(kMaxDetailedConvexSupportVertices)) {
            return kMaxDetailedConvexSupportVertices;
        }
        return static_cast<std::uint32_t>(requested);
    }

    inline constexpr std::uint32_t clampLineVertexBudget(int requested)
    {
        if (requested <= 0) {
            return 0;
        }
        if (requested > static_cast<int>(kMaxLineVertexBudget)) {
            return kMaxLineVertexBudget;
        }
        return static_cast<std::uint32_t>(requested);
    }

    inline constexpr std::uint32_t clampShapeCacheBudget(int requested)
    {
        if (requested < 16) {
            return 16;
        }
        if (requested > static_cast<int>(kMaxShapeCacheBudget)) {
            return kMaxShapeCacheBudget;
        }
        return static_cast<std::uint32_t>(requested);
    }

    inline constexpr std::uint32_t clampMaxCompoundChildren(int requested)
    {
        if (requested < 1) {
            return 1;
        }
        if (requested > static_cast<int>(kMaxCompoundChildren)) {
            return kMaxCompoundChildren;
        }
        return static_cast<std::uint32_t>(requested);
    }

    inline constexpr std::uint32_t clampMaxCompoundDepth(int requested)
    {
        if (requested < 1) {
            return 1;
        }
        if (requested > static_cast<int>(kMaxCompoundDepth)) {
            return kMaxCompoundDepth;
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

    inline bool isSupportedDetailedShapeType(int shapeType)
    {
        switch (shapeType) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 7:
        case 8:
        case 11:
            return true;
        default:
            return false;
        }
    }

    inline bool isConvexSupportShapeType(int shapeType)
    {
        return shapeType == 0 || shapeType == 1 || shapeType == 4;
    }

    inline ShapeDecodeMode chooseShapeDecodeMode(int shapeType, std::uint32_t supportVertexCount, int maxDetailedSupportVertices, bool proxyFallbackEnabled)
    {
        if (!isSupportedDetailedShapeType(shapeType)) {
            return proxyFallbackEnabled ? ShapeDecodeMode::Proxy : ShapeDecodeMode::Unsupported;
        }
        if (isConvexSupportShapeType(shapeType) && shouldUseBoundsForHeavyConvex(supportVertexCount, maxDetailedSupportVertices, proxyFallbackEnabled)) {
            return ShapeDecodeMode::Proxy;
        }
        return ShapeDecodeMode::Detailed;
    }

    inline std::uint64_t mixOverlaySettingsKey(std::uint64_t seed, std::uint64_t value)
    {
        value += 0x9e3779b97f4a7c15ull;
        value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ull;
        value = (value ^ (value >> 27)) * 0x94d049bb133111ebull;
        value = value ^ (value >> 31);
        return seed ^ (value + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2));
    }

    inline std::uint64_t makeShapeDecodeSettingsKey(
        int maxConvexSupportVertices,
        bool useBoundsForHeavyConvex,
        int maxCompoundChildren = static_cast<int>(kDefaultMaxCompoundChildren),
        int maxCompoundDepth = static_cast<int>(kDefaultMaxCompoundDepth))
    {
        std::uint64_t key = 0xcbf29ce484222325ull;
        key = mixOverlaySettingsKey(key, clampMaxConvexSupportVertices(maxConvexSupportVertices));
        key = mixOverlaySettingsKey(key, useBoundsForHeavyConvex ? 1ull : 0ull);
        key = mixOverlaySettingsKey(key, clampMaxCompoundChildren(maxCompoundChildren));
        key = mixOverlaySettingsKey(key, clampMaxCompoundDepth(maxCompoundDepth));
        return key;
    }

}
