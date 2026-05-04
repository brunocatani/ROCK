#pragma once

#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTransform.h"

namespace frik::rock
{
    struct HandFrame
    {
        RE::NiTransform transform{};
        RE::NiNode* node = nullptr;
        const char* label = "none";
        bool valid = false;
    };

    class HandFrameResolver
    {
    public:
        /*
         * ROCK's collision, palm selection, grab math, and debug axes use one
         * root flattened hand-frame convention. Scene nodes from another tree
         * are not returned as authority because mixing node conventions makes
         * grab frames disagree with the generated collider bodies.
         */
        HandFrame resolve(bool isLeft, bool hasRootFlattenedHand, const RE::NiTransform& rootFlattenedHandWorld) const
        {
            if (!hasRootFlattenedHand) {
                return {};
            }

            return HandFrame{
                rootFlattenedHandWorld,
                nullptr,
                isLeft ? "left-root-flattened-hand-bone" : "right-root-flattened-hand-bone",
                true
            };
        }
    };
}
