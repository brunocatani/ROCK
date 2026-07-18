#include "physics-interaction/hand/DynamicHandTwinTargets.h"

#include <cassert>

int main()
{
    rock::dynamic_hand_twin::TwinTargets canonicalHand{};
    canonicalHand.palm.valid = true;
    canonicalHand.palm.length = 4.0f;
    canonicalHand.palm.radius = 1.5f;
    canonicalHand.palm.convexRadius = 0.2f;
    rock::dynamic_hand_twin::TwinTargets liveHand{};
    liveHand.palm.valid = true;
    liveHand.palm.target.translate.y = 17.0f;
    liveHand.palm.length = 7.0f;
    liveHand.palm.radius = 2.0f;
    liveHand.palm.convexRadius = 0.4f;
    rock::dynamic_hand_twin::applyCanonicalHandDimensions(liveHand, canonicalHand);
    assert(liveHand.palm.target.translate.y == 17.0f);
    assert(liveHand.palm.length == 4.0f);
    assert(liveHand.palm.radius == 1.5f);
    assert(liveHand.palm.convexRadius == 0.2f);

    rock::dynamic_hand_twin::ForearmTwinTargets canonical{};
    canonical.right[0].valid = true;
    canonical.right[0].length = 18.5f;
    canonical.right[0].radius = 2.0f;
    canonical.right[0].convexRadius = 0.25f;

    rock::dynamic_hand_twin::ForearmTwinTargets live{};
    live.right[0].valid = true;
    live.right[0].target.translate.x = 42.0f;
    live.right[0].length = 31.0f;
    live.right[0].radius = 3.5f;
    live.right[0].convexRadius = 0.5f;
    live.right[0].handTargetResponseScale = 0.75f;

    rock::dynamic_hand_twin::applyCanonicalForearmDimensions(live, canonical);
    assert(live.right[0].valid);
    assert(live.right[0].target.translate.x == 42.0f);
    assert(live.right[0].length == 18.5f);
    assert(live.right[0].radius == 2.0f);
    assert(live.right[0].convexRadius == 0.25f);
    assert(live.right[0].handTargetResponseScale == 0.75f);

    live.left[0].valid = true;
    rock::dynamic_hand_twin::applyCanonicalForearmDimensions(live, canonical);
    assert(!live.left[0].valid);
    return 0;
}
