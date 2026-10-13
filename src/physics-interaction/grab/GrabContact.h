#pragma once

#include "physics-interaction/VectorMath.h"

/*
 * Physics collision identifies the object and body; the rendered mesh supplies
 * the grabbed point when one is found, and the collision point is the fallback.
 */

namespace rock::grab_contact_source_policy
{
    // A weapon may have one native collision owner and several visual sibling
    // branches. Prove both branches belong to that reference without crossing
    // another collision owner. No node is retained beyond this acquisition.
    template <class Node>
    [[nodiscard]] bool isSingleBodyOwnerlessVisualBranch(
        const Node* root, const Node* bodyOwner, const Node* visualNode)
    {
        if (!root || !bodyOwner || !bodyOwner->collisionObject || !visualNode) {
            return false;
        }
        const Node* ownerAncestor = bodyOwner;
        for (unsigned depth = 0; ownerAncestor && depth < 64; ++depth) {
            if (ownerAncestor == root) {
                for (unsigned visualDepth = 0; visualNode && visualDepth < 64; ++visualDepth) {
                    if (visualNode == root) return true;
                    if (visualNode->collisionObject) return false;
                    visualNode = visualNode->parent;
                }
                return false;
            }
            ownerAncestor = ownerAncestor->parent;
        }
        return false;
    }

}


#include "physics-interaction/hand/HandColliderTypes.h"
#include "physics-interaction/TransformMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>
