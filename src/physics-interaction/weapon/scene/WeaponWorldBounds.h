// World bounds of a weapon model ROCK has moved after the engine's update.
//
// ROCK and FRIK place the weapon after the engine's scene update and refresh
// world transforms only (UpdateWorldData), so every bound the renderer culls by
// stays where the engine last put the weapon. A gun ROCK holds in the off hand
// still hangs under the firing hand's bone, so it was culled with that hand:
// it vanished whenever the firing hand left the view, and sights near the face
// were tested at the wrong place.
//
// The renderer culls a BSFadeNode's shapes from the node's flattened geometry
// list, one precomputed bound per shape, not from the shapes themselves. The
// first-person skeleton's fade node lists the weapon's shapes beside the body's,
// and the weapon root's fade node lists them again. refresh() recomputes the
// shapes' world bounds from their final transforms, copies them into every
// fade-node list entry of those shapes (in the model and above it), and widens
// each ancestor's sphere bound until one already contains the model. The engine
// rebuilds all of these itself in its next update.
#pragma once

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"
#include "REL/Relocation.h"

#include <ROCK/TransformMath.h>

#include "physics-interaction/VectorMath.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace rock::weapon_world_bounds
{
    namespace detail
    {
        inline constexpr int kMaximumDepth = 32;
        inline constexpr int kMaximumObjects = 1024;
        inline constexpr int kMaximumAncestors = 48;
        inline constexpr std::size_t kMaximumShapes = 256;
        inline constexpr std::uint32_t kMaximumListEntries = 4096;

        /*
         * FO4VR 1.2.72 BSFadeNode, raw disassembly of its constructor
         * (0x1427A7720): it stores vtable 0x1430A5F08 and initialises the
         * flattened geometry BSTArray at +0x1A8 and the merged-bound array at
         * +0x1C0. The same vtable was read live from the equipped weapon root,
         * which the game's RTTI names BSFadeNode, and that root's list held one
         * entry per weapon shape: bound (+0x00), shape pointer (+0x10), flags
         * (+0x18), 0x20 bytes each; the array keeps data at +0x00, capacity at
         * +0x08 and size at +0x10. Only exact BSFadeNodes are touched.
         */
        inline constexpr std::uintptr_t kBSFadeNodeVTableRva = 0x30A5F08;
        inline constexpr std::ptrdiff_t kFlattenedGeometryArrayOffset = 0x1A8;

        struct FlattenedGeometryEntry
        {
            RE::NiBound bound;
            RE::NiAVObject* geometry;
            std::uint32_t flags;
            std::uint32_t pad1C;
        };
        static_assert(sizeof(FlattenedGeometryEntry) == 0x20);
        static_assert(offsetof(FlattenedGeometryEntry, geometry) == 0x10);

        struct FlattenedGeometryArray
        {
            FlattenedGeometryEntry* data;
            std::uint32_t capacity;
            std::uint32_t pad0C;
            std::uint32_t size;
            std::uint32_t pad14;
        };
        static_assert(sizeof(FlattenedGeometryArray) == 0x18);

        struct Shapes
        {
            std::array<const RE::NiAVObject*, kMaximumShapes> objects{};
            std::size_t count = 0;

            [[nodiscard]] bool contains(const RE::NiAVObject* object) const noexcept
            {
                for (std::size_t i = 0; i < count; ++i) {
                    if (objects[i] == object)
                        return true;
                }
                return false;
            }
        };

        [[nodiscard]] inline bool usable(const RE::NiBound& bound) noexcept
        {
            return std::isfinite(bound.center.x) && std::isfinite(bound.center.y) && std::isfinite(bound.center.z) &&
                std::isfinite(bound.fRadius) && bound.fRadius > 0.0f;
        }

        using vector_math::distance;

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

        [[nodiscard]] inline bool isFadeNode(const RE::NiAVObject* object) noexcept
        {
            static const std::uintptr_t vtable = REL::Module::get().base() + kBSFadeNodeVTableRva;
            return object && *reinterpret_cast<const std::uintptr_t*>(object) == vtable;
        }

        // Copies the refreshed bound of each listed shape of the model into the
        // fade node's flattened list. A list that is not plausible is left alone.
        inline void refreshFlattenedList(RE::NiAVObject* fadeNode, const Shapes& shapes) noexcept
        {
            auto& list = *reinterpret_cast<FlattenedGeometryArray*>(
                reinterpret_cast<std::byte*>(fadeNode) + kFlattenedGeometryArrayOffset);
            if (!list.data || list.size > list.capacity || list.capacity > kMaximumListEntries)
                return;
            for (std::uint32_t i = 0; i < list.size; ++i) {
                auto& entry = list.data[i];
                if (entry.geometry && shapes.contains(entry.geometry) && usable(entry.geometry->worldBound))
                    entry.bound = entry.geometry->worldBound;
            }
        }

        inline void refreshSubtree(RE::NiAVObject* object, int depth, int& budget, Shapes& shapes,
            Shapes& fadeNodes) noexcept
        {
            if (!object || depth > kMaximumDepth || --budget < 0)
                return;
            if (auto* node = object->IsNode()) {
                if (isFadeNode(object) && fadeNodes.count < fadeNodes.objects.size())
                    fadeNodes.objects[fadeNodes.count++] = object;
                RE::NiBound bound{};
                for (const auto& child : node->children) {
                    if (!child)
                        continue;
                    refreshSubtree(child.get(), depth + 1, budget, shapes, fadeNodes);
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
            // engine's to compute and is kept, here and in the lists.
            if (!geometry || geometry->GetRuntimeData().skinInstance)
                return;
            const auto& model = geometry->GetRuntimeData().modelBound;
            RE::NiBound bound{};
            bound.center = transform_math::localPointToWorld(object->world, model.center);
            bound.fRadius = model.fRadius * object->world.scale;
            if (!usable(bound))
                return;
            object->worldBound = bound;
            if (shapes.count < shapes.objects.size())
                shapes.objects[shapes.count++] = object;
        }
    }

    // Frame thread, after the model's final world transforms for this frame.
    inline void refresh(RE::NiAVObject* root) noexcept
    {
        if (!root)
            return;
        int budget = detail::kMaximumObjects;
        detail::Shapes shapes{};
        detail::Shapes fadeNodes{};
        detail::refreshSubtree(root, 0, budget, shapes, fadeNodes);
        if (shapes.count == 0)
            return;
        for (std::size_t i = 0; i < fadeNodes.count; ++i)
            detail::refreshFlattenedList(const_cast<RE::NiAVObject*>(fadeNodes.objects[i]), shapes);

        // Ancestors: every fade node's list, and the sphere bounds until one
        // already contains the model.
        const RE::NiAVObject* child = root;
        bool widening = true;
        auto* ancestor = root->parent;
        for (int i = 0; ancestor && i < detail::kMaximumAncestors; ++i) {
            if (detail::isFadeNode(ancestor))
                detail::refreshFlattenedList(ancestor, shapes);
            if (widening) {
                if (!detail::usable(child->worldBound) ||
                    (detail::usable(ancestor->worldBound) && detail::contains(ancestor->worldBound, child->worldBound)))
                    widening = false;
                else
                    ancestor->worldBound = detail::merge(ancestor->worldBound, child->worldBound);
            }
            child = ancestor;
            ancestor = ancestor->parent;
        }
    }
}
