#include "physics-interaction/native/LooseWeaponLayerRepair.h"
#include "physics-interaction/grab/GrabInteractionPolicy.h"

#include <cassert>

int main()
{
    using namespace rock;
    using namespace physics_body_classifier;
    using loose_weapon_layer_repair::correctedFilter;

    // 2026-09-24 14:37:05.915: SR-25 FF004E7A, P-PistolGrip,
    // body 56, motion 53, filter 00230001, flags 00041082, exact owner.
    BodyClassificationInput captured{};
    captured.bodyId = 56;
    captured.motionId = 53;
    captured.filterInfo = 0x00230001;
    captured.layer = captured.filterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
    captured.bodyFlags = 0x00041082;
    captured.motionType = motionTypeFromBodyFlags(captured.bodyFlags);
    assert(classifyBody(captured).reason == BodyRejectReason::UnsupportedLayer);
    assert(grab_interaction_policy::evaluateLooseObjectSelectionCandidate("WEAP", true, 6, true, captured.layer).blocked);

    const auto fixed = correctedFilter(captured.filterInfo, captured.bodyFlags, captured.motionId, true);
    assert(fixed && *fixed == 0x00230005);
    captured.filterInfo = *fixed;
    captured.layer = *fixed & collision_layer_policy::FO4_LAYER_FILTER_MASK;
    assert(classifyBody(captured).accepted);
    assert(!grab_interaction_policy::evaluateLooseObjectSelectionCandidate("WEAP", true, 6, true, captured.layer).blocked);
    // A repeat initializer performs no further filter mutation.
    assert(!correctedFilter(*fixed, captured.bodyFlags, captured.motionId, true));

    // Preserve an independent no-collide lease and group bits. Retirement can
    // park the same movable body; correction must not require waking it up.
    const auto parked = correctedFilter(0x00234001, 0x00041086, 53, true);
    assert(parked && *parked == 0x00234005);
    assert(((*parked ^ 0x00234001u) & ~collision_layer_policy::FO4_LAYER_FILTER_MASK) == 0);

    // Ownership loss/recycled motion slots cannot authorize this captured write.
    assert(!correctedFilter(0x00230001, 0x00041082, 53, false));
    assert(!correctedFilter(0x00230001, 0x00041082, 0, true));
    // The same layer on genuine static geometry must remain untouched.
    assert(!correctedFilter(0x00070001, 0x00040001, 0, true));
    assert(!correctedFilter(0x00230001, 0x00040001, 53, true));
}
