#include "physics-interaction/native/SceneRelation.h"
#include "physics-interaction/grab/SkinnedBoneOwner.h"
#include "RE/Bethesda/BSGeometry.h"
#include "RE/Bethesda/BSSkin.h"
#include "RE/NetImmerse/NiNode.h"
#include <array>
#include <unordered_set>

namespace rock::scene_relation {
    constexpr int kMaxNodeSearchDepth=48;
    constexpr std::uint32_t kMaxSkinBoneCount=512;
        void collectDownstreamNodesNoCollision(RE::NiAVObject* root, std::unordered_set<RE::NiAVObject*>& targets, int depth)
        {
            if (!root || depth < 0) {
                return;
            }

            targets.insert(root);
            auto* node = root->IsNode();
            if (!node) {
                return;
            }

            auto& children = node->GetRuntimeData().children;
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                auto* child = children[i].get();
                if (!child || child->collisionObject.get()) {
                    continue;
                }
                collectDownstreamNodesNoCollision(child, targets, depth - 1);
            }
        }

        [[nodiscard]] bool skinnedGeometryUsesTargetBones(RE::BSGeometry* geometry, const std::unordered_set<RE::NiAVObject*>& targetNodes)
        {
            if (!geometry || targetNodes.empty()) {
                return false;
            }

            auto* skinInstance = geometry->GetRuntimeData().skinInstance.get();
            RE::NiAVObject** boneNodes = nullptr;
            std::uint32_t boneCount = 0;
            if (!skinInstance || !nativeSkinLayoutVerified() ||
                !native_memory::tryReadField(skinInstance, 0x10, boneNodes) ||
                !native_memory::tryReadField(skinInstance, 0x20, boneCount) ||
                !boneNodes || boneCount == 0 || boneCount > kMaxSkinBoneCount) {
                return false;
            }

            std::array<RE::NiAVObject*, kMaxSkinBoneCount> bones{};
            if (!native_memory::guardedCopyFromMemory(boneNodes, bones.data(), boneCount * sizeof(bones[0]))) return false;
            for (std::uint32_t i = 0; i < boneCount; ++i) {
                auto* bone = bones[i];
                if (bone && targetNodes.find(bone) != targetNodes.end()) {
                    return true;
                }
            }

            return false;
        }

        [[nodiscard]] bool isSkinnedToTargetNodes(RE::NiAVObject* visualRoot, const std::unordered_set<RE::NiAVObject*>& targetNodes, int depth)
        {
            if (!visualRoot || depth < 0) {
                return false;
            }

            if (auto* geometry = visualRoot->IsGeometry()) {
                return skinnedGeometryUsesTargetBones(geometry, targetNodes);
            }

            auto* node = visualRoot->IsNode();
            if (!node) {
                return false;
            }

            auto& children = node->GetRuntimeData().children;
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                auto* child = children[i].get();
                if (child && isSkinnedToTargetNodes(child, targetNodes, depth - 1)) {
                    return true;
                }
            }

            return false;
        }

        bool isSkinnedToNode(RE::NiAVObject* visualRoot, RE::NiAVObject* hitNode)
        {
            if (!visualRoot || !hitNode) {
                return false;
            }

            std::unordered_set<RE::NiAVObject*> targetNodes;
            targetNodes.reserve(32);
            collectDownstreamNodesNoCollision(hitNode, targetNodes, kMaxNodeSearchDepth);
            return isSkinnedToTargetNodes(visualRoot, targetNodes, kMaxNodeSearchDepth);
        }

    bool nodeContainsNode(RE::NiAVObject* root, RE::NiAVObject* target, int maxDepth)
    {
        if (!root || !target || maxDepth < 0) {
            return false;
        }
        if (root == target) {
            return true;
        }

        auto* node = root->IsNode();
        if (!node) {
            return false;
        }

        auto& children = node->GetRuntimeData().children;
        for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
            auto* child = children[i].get();
            if (child && nodeContainsNode(child, target, maxDepth - 1)) {
                return true;
            }
        }
        return false;
    }


}
