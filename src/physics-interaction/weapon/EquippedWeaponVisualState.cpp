#include "physics-interaction/weapon/EquippedWeaponVisualState.h"

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"

#include "rock_support/Fo4VrRuntime.h"
#include "physics-interaction/performance/PerformanceProfiler.h"

#include <array>
#include <cstdio>
#include <cstring>

namespace rock::equipped_weapon_visual_state
{
    namespace
    {
        constexpr std::uint64_t kAppCulledFlag = 0x1ull;
        constexpr int kSubtreeWalkBudget = 2048;
        constexpr int kMaximumInstanceSearchDepth =
            static_cast<int>(kMaximumObservedPathNodes) - 1;

        struct SearchContext
        {
            const char* token{ nullptr };
            int budget{ kSubtreeWalkBudget };
            std::uintptr_t excludedInstanceAddress{ 0 };
            std::array<RE::NiAVObject*, kMaximumObservedPathNodes> path{};
            Snapshot result{};
        };

        bool findExactInstance(
            RE::NiAVObject* node,
            const int depth,
            const std::size_t pathDepth,
            SearchContext& context) noexcept
        {
            if (!node || depth < 0 || context.budget <= 0 ||
                pathDepth >= context.path.size()) {
                return false;
            }

            --context.budget;
            context.path[pathDepth] = node;
            const char* name = node->name.c_str();
            if (name && *name && std::strstr(name, context.token) &&
                reinterpret_cast<std::uintptr_t>(node) !=
                    context.excludedInstanceAddress) {
                context.result.exactInstance = node;
                context.result.pathNodeCount = pathDepth + 1;
                for (std::size_t index = 0; index <= pathDepth; ++index) {
                    context.result.pathNodes[index] = context.path[index];
                }

                context.result.ancestorPathVisible = true;
                for (std::size_t index = 0; index < pathDepth; ++index) {
                    if (!isLocallyVisible(context.path[index])) {
                        context.result.ancestorPathVisible = false;
                        break;
                    }
                }
                context.result.instanceLocallyVisible = isLocallyVisible(node);
                return true;
            }

            auto* asNode = node->IsNode();
            if (!asNode) {
                return false;
            }
            const auto& children = asNode->children;
            for (decltype(children.capacity()) slot = 0; slot < children.capacity() && context.budget > 0; ++slot) {
                const auto& child = children[slot];
                if (child && findExactInstance(child.get(), depth - 1, pathDepth + 1, context)) {
                    return true;
                }
            }
            return false;
        }
    }

    Snapshot observe(
        const std::uint32_t weaponBaseFormID,
        const std::uintptr_t excludedInstanceAddress,
        ObservationCache* cache) noexcept
    {
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::WeaponVisualObservation);
        Snapshot snapshot{};
        if (weaponBaseFormID == 0) {
            return snapshot;
        }

        auto* firstPersonSkeleton = f4vr::getFirstPersonSkeleton();
        auto* cachedRoot = cache ? cache->weaponPath.resolve(firstPersonSkeleton) : nullptr;
        if (cachedRoot && (!cachedRoot->name.c_str() || std::strcmp(cachedRoot->name.c_str(), "Weapon") != 0)) cachedRoot = nullptr;
        snapshot.weaponRoot = cachedRoot ? cachedRoot->IsNode() :
            (firstPersonSkeleton ? f4vr::findNode(firstPersonSkeleton, "Weapon") : nullptr);
        if (!snapshot.weaponRoot) {
            if (cache) *cache = {};
            return snapshot;
        }
        if (cache && !cachedRoot) {
            cache->weaponPath.capture(firstPersonSkeleton, snapshot.weaponRoot);
            cache->instancePath.clear();
        }

        char token[16]{};
        std::snprintf(token, sizeof(token), "(%08X)", weaponBaseFormID);
        if (cache && cache->formID == weaponBaseFormID && cache->excludedInstance == excludedInstanceAddress) {
            snapshot.ancestorPathVisible = true;
            auto* instance = cache->instancePath.resolve(snapshot.weaponRoot, [&](RE::NiAVObject* node) {
                if (snapshot.pathNodeCount < snapshot.pathNodes.size()) snapshot.pathNodes[snapshot.pathNodeCount++] = node;
            });
            if (instance && instance->name.c_str() && std::strstr(instance->name.c_str(), token) &&
                reinterpret_cast<std::uintptr_t>(instance) != excludedInstanceAddress) {
                snapshot.exactInstance = instance;
                for (std::size_t i = 0; i + 1 < snapshot.pathNodeCount; ++i)
                    snapshot.ancestorPathVisible &= isLocallyVisible(snapshot.pathNodes[i]);
                snapshot.instanceLocallyVisible = isLocallyVisible(instance);
                return snapshot;
            }
            snapshot.pathNodeCount = 0;
            snapshot.ancestorPathVisible = false;
        }
        SearchContext context{};
        context.token = token;
        context.excludedInstanceAddress = excludedInstanceAddress;
        context.result.weaponRoot = snapshot.weaponRoot;
        if (findExactInstance(
                snapshot.weaponRoot,
                kMaximumInstanceSearchDepth,
                0,
                context)) {
            if (cache) {
                cache->formID = weaponBaseFormID;
                cache->excludedInstance = excludedInstanceAddress;
                cache->instancePath.capture(snapshot.weaponRoot, context.result.exactInstance);
            }
            return context.result;
        }
        if (cache) cache->instancePath.clear();
        return snapshot;
    }

    bool isLocallyVisible(const RE::NiAVObject* node) noexcept
    {
        return node && (node->flags.flags & kAppCulledFlag) == 0;
    }

    void setLocallyVisible(RE::NiAVObject* node, const bool visible) noexcept
    {
        if (!node) {
            return;
        }
        if (visible) {
            node->flags.flags &= ~kAppCulledFlag;
        } else {
            node->flags.flags |= kAppCulledFlag;
        }
    }

    bool restoreExactInstancePathVisibility(const Snapshot& snapshot) noexcept
    {
        if (!snapshot.exactInstance || snapshot.pathNodeCount == 0) {
            return false;
        }

        bool changed = false;
        for (std::size_t index = 0; index < snapshot.pathNodeCount; ++index) {
            auto* node = snapshot.pathNodes[index];
            if (node && !isLocallyVisible(node)) {
                setLocallyVisible(node, true);
                changed = true;
            }
        }
        if (changed && snapshot.weaponRoot) {
            f4vr::updateDown(snapshot.weaponRoot, true);
        }
        return changed;
    }
}
