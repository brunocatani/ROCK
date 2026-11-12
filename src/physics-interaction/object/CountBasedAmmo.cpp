#include "physics-interaction/object/CountBasedAmmo.h"

#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"
#include "physics-interaction/native/havok/HavokRuntime.h"

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

        // The body a node's own collision object binds in this world.
        std::uint32_t ownBody(RE::hknpWorld* world, RE::NiAVObject* node)
        {
            auto* collisionObject = node ? node->collisionObject.get() : nullptr;
            RE::hknpWorld* bodyWorld = nullptr;
            RE::hknpBodyId bodyId{ AssemblyBodies::kNoBody };
            if (!collisionObject || !havok_runtime::tryResolveCollisionObjectBody(collisionObject, bodyWorld, bodyId) ||
                bodyWorld != world) {
                return AssemblyBodies::kNoBody;
            }
            return bodyId.value;
        }

        // The branch's collision object sits on the branch node in every model
        // seen; a shallow walk also accepts one on a direct child.
        std::uint32_t branchBody(RE::hknpWorld* world, RE::NiAVObject* branch)
        {
            const auto own = ownBody(world, branch);
            if (own != AssemblyBodies::kNoBody) {
                return own;
            }
            auto* node = branch ? branch->IsNode() : nullptr;
            if (!node) {
                return AssemblyBodies::kNoBody;
            }
            for (decltype(node->children.capacity()) i = 0; i < node->children.capacity(); ++i) {
                const auto child = ownBody(world, node->children[i].get());
                if (child != AssemblyBodies::kNoBody) {
                    return child;
                }
            }
            return AssemblyBodies::kNoBody;
        }
    }

    bool isCountBasedAmmo(RE::TESObjectREFR* refr)
    {
        auto* base = refr ? refr->GetObjectReference() : nullptr;
        const auto* ammo = base ? base->As<RE::TESAmmo>() : nullptr;
        return ammo && (static_cast<std::uint8_t>(ammo->data.flags) & kCountBased3DFlag) != 0;
    }

    Assembly resolve(RE::TESObjectREFR* refr)
    {
        if (!isCountBasedAmmo(refr)) {
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

    AssemblyBodies resolveBodies(RE::hknpWorld* world, const Assembly& assembly)
    {
        if (!world || !assembly.valid()) {
            return {};
        }
        AssemblyBodies bodies{};
        bodies.root = ownBody(world, assembly.root);
        bodies.branch = branchBody(world, assembly.variantNode);
        return bodies.valid() ? bodies : AssemblyBodies{};
    }

    bool isRootBodyNode(RE::TESObjectREFR* refr, const RE::NiAVObject* hitNode)
    {
        if (!hitNode || !isCountBasedAmmo(refr) || refr->Get3D() != hitNode) {
            return false;
        }
        return resolve(refr).valid();
    }
}
