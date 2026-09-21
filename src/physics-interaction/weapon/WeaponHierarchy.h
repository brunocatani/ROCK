#pragma once
#include "RE/NetImmerse/NiAVObject.h"
#include "physics-interaction/TransformMath.h"
#include <array>
#include <cstdint>

namespace rock::weapon_hierarchy {
        [[nodiscard]] inline bool weaponTransformFinite(const RE::NiTransform& transform)
        {
            if (!std::isfinite(transform.translate.x) || !std::isfinite(transform.translate.y) ||
                !std::isfinite(transform.translate.z) || !std::isfinite(transform.scale)) {
                return false;
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return true;
        }

        inline constexpr std::size_t kMaximumWeaponLocalHierarchyDepth = 64;

        [[nodiscard]] inline bool weaponSceneNodePointerPlausible(const RE::NiAVObject* node) noexcept
        {
            const auto address = reinterpret_cast<std::uintptr_t>(node);
            return address >= 0x10000 && address <= 0x0000'7FFF'FFFF'FFFFull;
        }

        [[nodiscard]] inline bool tryResolveDescendantLocalTransform(
            const RE::NiAVObject* ancestor,
            const RE::NiAVObject* descendant,
            RE::NiTransform& outDescendantLocal)
        {
            /*
             * Descendant world transforms can belong to a different scene-graph
             * propagation epoch than the current weapon root. Compose the
             * bounded parent path so callers receive one coherent local frame.
             * Shoulder draw can also leave that coherent frame in presentation
             * space; post-undraw callers remove that separately with a validated
             * source-frame correction.
             */
            outDescendantLocal = transform_math::makeIdentityTransform<RE::NiTransform>();
            if (!ancestor || !descendant) {
                return false;
            }

            std::array<const RE::NiAVObject*, kMaximumWeaponLocalHierarchyDepth> reversePath{};
            std::size_t pathLength = 0;
            auto* cursor = descendant;
            while (cursor && cursor != ancestor) {
                if (!weaponSceneNodePointerPlausible(cursor) ||
                    pathLength >= reversePath.size() ||
                    !weaponTransformFinite(cursor->local)) {
                    outDescendantLocal = {};
                    return false;
                }
                reversePath[pathLength++] = cursor;
                cursor = cursor->parent;
            }
            if (cursor != ancestor) {
                outDescendantLocal = {};
                return false;
            }

            while (pathLength != 0) {
                outDescendantLocal = transform_math::composeTransforms(
                    outDescendantLocal,
                    reversePath[--pathLength]->local);
            }
            if (!weaponTransformFinite(outDescendantLocal)) {
                outDescendantLocal = {};
                return false;
            }
            return true;
        }

        [[nodiscard]] inline bool tryResolveDescendantWorldTransform(
            const RE::NiAVObject* ancestor,
            const RE::NiTransform& ancestorWorld,
            const RE::NiAVObject* descendant,
            RE::NiTransform& outDescendantWorld)
        {
            RE::NiTransform descendantLocal{};
            if (!weaponTransformFinite(ancestorWorld) ||
                !tryResolveDescendantLocalTransform(ancestor, descendant, descendantLocal)) {
                outDescendantWorld = {};
                return false;
            }

            outDescendantWorld = transform_math::composeTransforms(ancestorWorld, descendantLocal);
            if (!weaponTransformFinite(outDescendantWorld)) {
                outDescendantWorld = {};
                return false;
            }
            return true;
        }

}
