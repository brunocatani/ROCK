#pragma once

#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiPoint.h"

#include <cstdint>

/*
 * The collision triangles of visible surfaces near a point (Collision 2.1
 * queryWorldSurfaceTriangles). hknpWorld::queryAabb reports one hit per leaf
 * of every body in the box; each mesh, compound or heightfield leaf is decoded
 * by its shape's own leaf decoder (vtable +0xA8, getLeafShapes) into a
 * ROCK-owned collector, and convex leaves and convex bodies become their hulls
 * from their support vertices. Bodies keep only visible surface layers, as the
 * Surfaces raycast does.
 *
 * Evidence: DOCS_STORAGE/SCISSORS/17-11-2026/FO4VR_World_Triangle_Query_Research.md
 * and its audit (triangle_query_audit/Wave1_QueryAabb.md, Wave2_LeafDecoders.md).
 * The query function's prologue and each decoder's vtable slot are checked
 * once before first use; a mismatch refuses every query.
 *
 * Thread: the frame thread. The world read lock is held across the query and
 * every decode, because queryAabb takes only hknp's internal lock and shapes
 * stay alive only under the world lock.
 */
namespace rock::physics_surface_triangles
{
    struct Triangle
    {
        RE::NiPoint3 a;
        RE::NiPoint3 b;
        RE::NiPoint3 c;
        RE::NiPoint3 normal;
        std::uint32_t layer = 0;
    };

    struct Result
    {
        std::uint32_t count = 0;
        bool truncated = false;
    };

    // Game units. false when the world, the binary check or a decode fails.
    bool collect(RE::hknpWorld* world, const RE::NiPoint3& centreGame, float radiusGame, Triangle* out,
        std::uint32_t capacity, Result& result);
}
