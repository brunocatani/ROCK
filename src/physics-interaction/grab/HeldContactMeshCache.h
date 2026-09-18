#pragma once

#include "physics-interaction/weapon/WeaponTriangleIndex.h"

namespace rock
{
    // Value-only acceleration owned by the same grab frame as the triangles.
    // Every replacement/clear of that mesh invalidates this lazy preparation.
    struct HeldContactMeshCache
    {
        static constexpr std::size_t maximumTriangles = 16384;
        bool prepared = false;
        bool valid = false;
        bool indexed = false;
        RE::NiPoint3 minimum{}, maximum{};
        WeaponTriangleIndex index;

        void clear()
        {
            prepared = valid = indexed = false;
            minimum = maximum = {};
            index.clear();
        }

        template <class Triangles>
        void prepare(const Triangles& triangles)
        {
            if (prepared) return;
            prepared = true;
            if (triangles.empty() || triangles.size() > maximumTriangles) return;
            minimum = { (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::max)() };
            maximum = { -minimum.x, -minimum.y, -minimum.z };
            for (const auto& triangle : triangles) {
                for (const auto& p : { triangle.v0, triangle.v1, triangle.v2 }) {
                    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
                    minimum = { (std::min)(minimum.x, p.x), (std::min)(minimum.y, p.y), (std::min)(minimum.z, p.z) };
                    maximum = { (std::max)(maximum.x, p.x), (std::max)(maximum.y, p.y), (std::max)(maximum.z, p.z) };
                }
            }
            valid = true;
            // Tiny meshes use the same exact loop without paying tree construction.
            indexed = triangles.size() >= 64;
            if (indexed) index.build(triangles);
        }
    };
}
