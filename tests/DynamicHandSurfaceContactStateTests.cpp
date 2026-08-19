#include "physics-interaction/grab/GlobalSurfaceGrabPolicy.h"
#include "physics-interaction/hand/collision/DynamicHandSurfaceContactState.h"

#include <limits>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

int main()
{
    using namespace rock;
    using namespace rock::dynamic_hand_surface_contact_state;

    State state{};
    const ContactSource palm{
        .valid = true,
        .isLeft = false,
        .slot = 0,
        .role = hand_collider_semantics::HandColliderRole::PalmAnchor,
        .finger = hand_collider_semantics::HandFinger::None,
        .segment = hand_collider_semantics::HandFingerSegment::None,
        .bodyId = 101,
    };
    const hand_semantic_contact_state::SemanticContactVector point{
        10.0f,
        20.0f,
        30.0f,
    };
    const hand_semantic_contact_state::SemanticContactVector normal{
        0.0f,
        0.0f,
        1.0f,
    };
    assert(state.record(palm, 501, &point, &normal));

    auto contacts = state.collectFresh(false, 0);
    assert(contacts.count == 1);
    assert(contacts.records[0].valid);
    assert(!contacts.records[0].isLeft);
    assert(contacts.records[0].handBodyId == 101);
    assert(contacts.records[0].otherBodyId == 501);
    assert(contacts.records[0].hasContactPointGame);
    assert(contacts.records[0].hasContactNormalGame);
    assert(contacts.records[0].contactPointGame.y == 20.0f);

    ContactSource leftIndex = palm;
    leftIndex.isLeft = true;
    leftIndex.slot = 2;
    leftIndex.role = hand_collider_semantics::HandColliderRole::IndexTip;
    leftIndex.finger = hand_collider_semantics::HandFinger::Index;
    leftIndex.segment = hand_collider_semantics::HandFingerSegment::Tip;
    leftIndex.bodyId = 202;
    const hand_semantic_contact_state::SemanticContactVector invalidNormal{
        std::numeric_limits<float>::quiet_NaN(),
        0.0f,
        0.0f,
    };
    assert(state.record(leftIndex, 502, nullptr, &invalidNormal));
    contacts = state.collectFresh(true, 0);
    assert(contacts.count == 1);
    assert(contacts.records[0].role ==
           hand_collider_semantics::HandColliderRole::IndexTip);
    assert(!contacts.records[0].hasContactPointGame);
    assert(!contacts.records[0].hasContactNormalGame);

    state.advanceFrame();
    assert(state.collectFresh(false, 0).count == 0);
    assert(state.collectFresh(false, 1).count == 1);

    state.clear();
    assert(state.collectFresh(false, 10).count == 0);
    assert(state.collectFresh(true, 10).count == 0);

    ContactSource invalid = palm;
    invalid.slot = kContactRolesPerHand;
    assert(!state.record(invalid, 503, &point, &normal));
    assert(!state.record(palm, hand_semantic_contact_state::kInvalidBodyId, &point, &normal));
    assert(!state.record(palm, palm.bodyId, &point, &normal));

    using namespace rock::collision_layer_policy;
    using namespace rock::global_surface_grab_policy;
    assert(shouldUseFallback(
        true,
        false,
        true,
        true,
        FO4_LAYER_STATIC));
    assert(shouldUseFallback(
        true,
        false,
        true,
        true,
        FO4_LAYER_ANIMSTATIC));
    assert(shouldUseFallback(
        true,
        false,
        true,
        true,
        FO4_LAYER_TREES));
    assert(!shouldUseFallback(
        false,
        false,
        true,
        true,
        FO4_LAYER_STATIC));
    assert(!shouldUseFallback(
        true,
        true,
        true,
        true,
        FO4_LAYER_STATIC));
    assert(!shouldUseFallback(
        true,
        false,
        false,
        true,
        FO4_LAYER_STATIC));
    assert(!shouldUseFallback(
        true,
        false,
        true,
        false,
        FO4_LAYER_STATIC));
    assert(!shouldUseFallback(
        true,
        false,
        true,
        true,
        FO4_LAYER_CLUTTER));
    assert(canFollowUnclassifiedMotion(true, true));
    assert(!canFollowUnclassifiedMotion(false, true));
    assert(!canFollowUnclassifiedMotion(true, false));
    assert(maskEnablesLayer(allowedLayerMask(), FO4_LAYER_STATIC));
    assert(maskEnablesLayer(allowedLayerMask(), FO4_LAYER_TERRAIN));
    assert(maskEnablesLayer(
        allowedLayerMask(),
        ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER));
    assert(!maskEnablesLayer(allowedLayerMask(), FO4_LAYER_CLUTTER));
    assert(targetIdForHand(false) != targetIdForHand(true));
    return 0;
}
