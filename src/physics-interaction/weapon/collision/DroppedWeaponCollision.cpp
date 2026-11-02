// Captures the equipped weapon's generated hulls as node-name paths below the
// model root, and resolves those paths again on the dropped reference's own
// model clone.
#include "physics-interaction/weapon/collision/DroppedWeaponCollision.h"

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
                if (child && visit(child))
                    return;
            }
        }
    }

    bool DroppedWeaponCollision::capturePath(const RE::NiAVObject* root, const RE::NiAVObject* leaf,
        std::vector<Step>& out)
    {
        out.clear();
        if (!root || !leaf)
            return false;
        for (const auto* node = leaf; node != root; node = node->parent) {
            const RE::NiNode* parent = node->parent;
            if (!parent || out.size() >= kMaxPathDepth) {
                out.clear();
                return false;
            }
            Step step{ nameOf(node), 0, node->IsNode() != nullptr };
            bool found = false;
            forEachChild(parent, [&](RE::NiAVObject* child) {
                if (child == node)
                    return found = true;
                if (step.name == nameOf(child))
                    ++step.ordinal;
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
            if (!parent)
                return nullptr;
            RE::NiAVObject* next = nullptr;
            std::uint32_t seen = 0;
            forEachChild(parent, [&](RE::NiAVObject* child) {
                if (step.name != nameOf(child) || seen++ != step.ordinal)
                    return false;
                next = child;
                return true;
            });
            if (!next || (next->IsNode() != nullptr) != step.node)
                return nullptr;
            current = next;
        }
        return current;
    }

    std::string DroppedWeaponCollision::describeUnresolved(RE::NiAVObject* root, const std::vector<Step>& path)
    {
        std::string text;
        for (std::size_t i = 0; i < path.size(); ++i) {
            text += (i ? "/" : "") + path[i].name + "#" + std::to_string(path[i].ordinal);
        }
        auto* current = root;
        for (std::size_t i = 0; i < path.size(); ++i) {
            const auto& step = path[i];
            const auto* parent = current ? current->IsNode() : nullptr;
            RE::NiAVObject* next = nullptr;
            std::uint32_t seen = 0;
            std::string siblings;
            std::size_t listed = 0;
            if (parent) {
                forEachChild(parent, [&](RE::NiAVObject* child) {
                    if (listed++ < 24)
                        siblings += std::string(listed > 1 ? "," : "") + nameOf(child);
                    if (!next && step.name == nameOf(child) && seen++ == step.ordinal)
                        next = child;
                    return false;
                });
            }
            if (!next || (next->IsNode() != nullptr) != step.node) {
                return "path=" + text + " failedStep=" + std::to_string(i) + " parent='" +
                    (current ? nameOf(current) : "") + "' wanted='" + step.name + "'#" + std::to_string(step.ordinal) +
                    (step.node ? " node" : " leaf") + (next ? " typeMismatch" : "") + " children=[" + siblings + "]";
            }
            current = next;
        }
        return "path=" + text + " resolved";
    }
}
