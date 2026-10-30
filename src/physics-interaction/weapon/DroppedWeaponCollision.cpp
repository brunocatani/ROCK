#include "physics-interaction/weapon/DroppedWeaponCollision.h"

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"

#include <algorithm>
#include <cstring>

namespace rock
{
    namespace
    {
        constexpr std::size_t kMaxPathDepth = 64;

        const char* nameOf(const RE::NiAVObject* node)
        {
            const char* name = node->name.c_str();
            return name ? name : "";
        }

        template <class Visitor>
        void forEachChild(const RE::NiNode* parent, Visitor&& visit)
        {
            const auto& children = parent->children;
            for (decltype(children.capacity()) slot = 0; slot < children.capacity(); ++slot) {
                auto* child = children[slot].get();
                if (child && visit(child)) return;
            }
        }
    }

    bool DroppedWeaponCollision::capturePath(const RE::NiAVObject* root, const RE::NiAVObject* leaf, std::vector<Step>& out)
    {
        out.clear();
        if (!root || !leaf) return false;
        for (const auto* node = leaf; node != root; node = node->parent) {
            const RE::NiNode* parent = node->parent;
            if (!parent || out.size() >= kMaxPathDepth) {
                out.clear();
                return false;
            }
            Step step{ nameOf(node), 0, node->IsNode() != nullptr };
            bool found = false;
            forEachChild(parent, [&](RE::NiAVObject* child) {
                if (child == node) return found = true;
                if (step.name == nameOf(child)) ++step.ordinal;
                return false;
            });
            if (!found) {
                out.clear();
                return false;
            }
            out.push_back(std::move(step));
        }
        std::reverse(out.begin(), out.end());
        return true;
    }

    RE::NiAVObject* DroppedWeaponCollision::resolvePath(RE::NiAVObject* root, const std::vector<Step>& path)
    {
        auto* current = root;
        for (const auto& step : path) {
            const auto* parent = current ? current->IsNode() : nullptr;
            if (!parent) return nullptr;
            RE::NiAVObject* next = nullptr;
            std::uint32_t seen = 0;
            forEachChild(parent, [&](RE::NiAVObject* child) {
                if (step.name != nameOf(child) || seen++ != step.ordinal) return false;
                next = child;
                return true;
            });
            if (!next || (next->IsNode() != nullptr) != step.node) return nullptr;
            current = next;
        }
        return current;
    }
}
