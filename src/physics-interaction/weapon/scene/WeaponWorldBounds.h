// World bounds of a weapon model ROCK has moved after the engine's update.
//
// ROCK and FRIK place the weapon after the engine's scene update and refresh
// world transforms only (UpdateWorldData), so each shape's world bound stays
// where the engine last put the weapon. The renderer culls by those bounds,
// hierarchically, so a gun drawn at the hand was tested at its old place and
// vanished whenever that place was out of view or occluded: more often near
// the face and where a location carries heavy occlusion data. refresh()
// recomputes the model's bounds from its final transforms and widens each
// ancestor's bound just enough to contain it, stopping at the first ancestor
// that already does; the engine rebuilds those ancestors itself next frame.
#pragma once

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"

#include <ROCK/TransformMath.h>

#include <cmath>

namespace rock::weapon_world_bounds
{
    namespace detail
    {
        inline constexpr int kMaximumDepth = 32;
        inline constexpr int kMaximumObjects = 1024;
        inline constexpr int kMaximumAncestors = 48;

        [[nodiscard]] inline bool usable(const RE::NiBound& bound) noexcept
        {
            return std::isfinite(bound.center.x) && std::isfinite(bound.center.y) && std::isfinite(bound.center.z) &&
                std::isfinite(bound.fRadius) && bound.fRadius > 0.0f;
        }

        [[nodiscard]] inline float distance(const RE::NiPoint3& a, const RE::NiPoint3& b) noexcept
        {
            const float x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
            return std::sqrt(x * x + y * y + z * z);
        }

        [[nodiscard]] inline bool contains(const RE::NiBound& outer, const RE::NiBound& inner) noexcept
        {
            return distance(outer.center, inner.center) + inner.fRadius <= outer.fRadius;
        }

        // Smallest sphere holding both.
        [[nodiscard]] inline RE::NiBound merge(const RE::NiBound& a, const RE::NiBound& b) noexcept
        {
            if (!usable(a))
                return b;
            if (!usable(b) || contains(a, b))
                return a;
            if (contains(b, a))
                return b;
            const float d = distance(a.center, b.center);
            RE::NiBound result{};
            result.fRadius = 0.5f * (d + a.fRadius + b.fRadius);
            const float t = (result.fRadius - a.fRadius) / d;
            result.center = RE::NiPoint3{ a.center.x + (b.center.x - a.center.x) * t,
                a.center.y + (b.center.y - a.center.y) * t, a.center.z + (b.center.z - a.center.z) * t };
            return result;
        }

        inline void refreshSubtree(RE::NiAVObject* object, int depth, int& budget) noexcept
        {
            if (!object || depth > kMaximumDepth || --budget < 0)
                return;
            if (auto* node = object->IsNode()) {
                RE::NiBound bound{};
                for (const auto& child : node->children) {
                    if (!child)
                        continue;
                    refreshSubtree(child.get(), depth + 1, budget);
                    bound = merge(bound, child->worldBound);
                }
                if (usable(bound))
                    node->worldBound = bound;
                return;
            }
            auto* geometry = object->IsGeometry();
            if (!geometry)
                geometry = object->IsTriShape();
            // A skinned shape is drawn from its bones; its bound is the
            // engine's to compute and is kept.
            if (!geometry || geometry->GetRuntimeData().skinInstance)
                return;
            const auto& model = geometry->GetRuntimeData().modelBound;
            RE::NiBound bound{};
            bound.center = transform_math::localPointToWorld(object->world, model.center);
            bound.fRadius = model.fRadius * object->world.scale;
            if (usable(bound))
                object->worldBound = bound;
        }
    }

    // Frame thread, after the model's final world transforms for this frame.
    inline void refresh(RE::NiAVObject* root) noexcept
    {
        if (!root)
            return;
        int budget = detail::kMaximumObjects;
        detail::refreshSubtree(root, 0, budget);
        const RE::NiAVObject* child = root;
        auto* ancestor = root->parent;
        for (int i = 0; ancestor && i < detail::kMaximumAncestors; ++i) {
            if (!detail::usable(child->worldBound) ||
                (detail::usable(ancestor->worldBound) && detail::contains(ancestor->worldBound, child->worldBound)))
                return;
            ancestor->worldBound = detail::merge(ancestor->worldBound, child->worldBound);
            child = ancestor;
            ancestor = ancestor->parent;
        }
    }
}
