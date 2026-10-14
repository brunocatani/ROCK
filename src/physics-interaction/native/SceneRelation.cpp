#include "physics-interaction/native/SceneRelation.h"
#include "RE/NetImmerse/NiNode.h"

namespace rock::scene_relation {
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
