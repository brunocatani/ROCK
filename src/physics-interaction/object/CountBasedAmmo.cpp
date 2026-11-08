#include "physics-interaction/object/CountBasedAmmo.h"

#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"

namespace rock::count_based_ammo
{
    namespace
    {
        constexpr std::string_view kSingleNodeName = "AmmoSingle";
        constexpr std::string_view kMultipleNodeName = "AmmoMultiple";

        // The engine searches by name; the branches are root children in
        // every vanilla and replacer model seen, so a shallow bounded walk
        // finds them without a virtual name lookup.
        RE::NiAVObject* findNamedBranch(RE::NiAVObject* node, std::string_view name, int depth)
        {
            if (!node || depth < 0) {
                return nullptr;
            }
            const char* nodeName = node->name.c_str();
            if (nodeName && name == nodeName) {
                return node;
            }
            auto* branch = node->IsNode();
            if (!branch) {
                return nullptr;
            }
            for (decltype(branch->children.capacity()) i = 0; i < branch->children.capacity(); ++i) {
                if (auto* found = findNamedBranch(branch->children[i].get(), name, depth - 1)) {
                    return found;
                }
            }
            return nullptr;
        }
    }

    Assembly resolve(RE::TESObjectREFR* refr)
    {
        auto* base = refr ? refr->GetObjectReference() : nullptr;
        const auto* ammo = base ? base->As<RE::TESAmmo>() : nullptr;
        if (!ammo || (static_cast<std::uint8_t>(ammo->data.flags) & kCountBased3DFlag) == 0) {
            return {};
        }
        auto* root = refr->Get3D();
        if (!root) {
            return {};
        }

        auto* single = findNamedBranch(root, kSingleNodeName, 3);
        auto* multiple = findNamedBranch(root, kMultipleNodeName, 3);
        // Exactly one branch stays attached after the engine's count-based
        // setup. Both or neither means a model this does not understand.
        if ((single != nullptr) == (multiple != nullptr)) {
            return {};
        }

        Assembly assembly{};
        assembly.root = root;
        assembly.variantNode = single ? single : multiple;
        assembly.variant = single ? Variant::Single : Variant::Multiple;
        return assembly;
    }
}
