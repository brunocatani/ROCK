#include "physics-interaction/grab/DecorationModePolicy.h"
#include <cassert>

int main()
{
    using namespace rock::decoration_mode;
    assert(singleObject(0,0)==0);
    assert(singleObject(17,0)==17 && singleObject(0,17)==17);
    assert(singleObject(17,17)==17);
    assert(singleObject(17,18)==0);
    // A full live scan accepts the single-body bottle and a multipart object
    // independently of the old grab cache. Partial/stale discovery still fails.
    assert(completeBodyScan(true,1,1,0));
    assert(completeBodyScan(true,5,5,0));
    assert(!completeBodyScan(false,1,1,0));
    assert(!completeBodyScan(true,0,0,0));
    assert(!completeBodyScan(true,65,65,0));
    assert(!completeBodyScan(true,5,4,0));
    assert(!completeBodyScan(true,1,1,1));
    using rock::physics_body_classifier::BodyMotionType;
    using rock::physics_body_classifier::BodyRejectReason;
    // Timberwolf: five driven bodies and one owned static part at motion 0.
    const bool fixedPart=preserveStaticBody(BodyRejectReason::InvalidMotionId,BodyMotionType::Static,0,true);
    assert(fixedPart && completeBodyScan(true,6,5+fixedPart,0));
    assert(!preserveStaticBody(BodyRejectReason::InvalidMotionId,BodyMotionType::Dynamic,0,true));
    assert(!preserveStaticBody(BodyRejectReason::InvalidMotionId,BodyMotionType::Unknown,0,true));
    assert(!preserveStaticBody(BodyRejectReason::InvalidMotionId,BodyMotionType::Static,1,true));
    assert(!preserveStaticBody(BodyRejectReason::InvalidMotionId,BodyMotionType::Static,0,false));
    assert(!preserveStaticBody(BodyRejectReason::InvalidBodyId,BodyMotionType::Static,0,true));
    assert(anchoredMotion(BodyMotionType::Dynamic,BodyMotionType::Keyframed));
    assert(anchoredMotion(BodyMotionType::Static,BodyMotionType::Static));
    assert(!anchoredMotion(BodyMotionType::Static,BodyMotionType::Keyframed));
    assert(!anchoredMotion(BodyMotionType::Dynamic,BodyMotionType::Static));
    assert(!anchoredMotion(BodyMotionType::Dynamic,BodyMotionType::Dynamic));
    SurfaceContactState contact;
    assert(!contact.read().recent);
    contact.publish(152,2); // A bottle touching the table needs no impact event.
    assert(contact.read().recent && contact.read().heldBodyId==152 && contact.read().otherBodyId==2);
    for (int i=0;i<4;++i) contact.tick();
    assert(contact.read().recent);
    contact.tick();
    assert(!contact.read().recent);
    contact.publish(149,3);
    assert(contact.read().recent && contact.read().heldBodyId==149 && contact.read().otherBodyId==3);
    contact.clear(); // Release/world cleanup cannot retain placement eligibility.
    assert(!contact.read().recent);
    ClickState state;
    state.update(true,17,true,false,false,0);
    assert(state.reserved && !state.request);
    state.update(true,17,true,true,true,0);
    assert(state.request==17 && state.draining);
    state.update(true,17,true,true,false,0);
    assert(!state.request && state.reserved);
    // Anchoring releases the object. Its spent click must not open PALM.
    state.update(false,0,true,true,false,0);
    assert(!state.request && state.reserved);
    state.update(false,0,true,false,false,0);
    assert(state.reserved && !state.draining);
    state.update(false,0,true,false,false,0);
    assert(!state.reserved);
    // A press spent away from a supporting surface cannot anchor on arrival.
    state.update(true,0,true,true,true,0);
    state.update(true,17,true,true,false,0);
    assert(!state.request);
    state.update(true,17,true,false,false,0);
    state.update(true,17,true,false,true,0);
    assert(state.request==17); // A whole short click can fit between frames.
    state.update(true,17,false,true,true,0);
    state.update(true,17,true,true,true,0);
    assert(!state.request); // Menu/provider loss needs release to rearm.
    state.update(true,17,true,false,false,0);
    state.update(true,17,true,true,true,101);
    assert(!state.request && !state.reserved);
    state.update(false,17,true,false,false,0);
    state.update(false,17,true,true,true,0);
    assert(!state.request && !state.reserved);
    // Enabling the mode cannot adopt a click that began while it was off.
    state.update(true,17,true,false,true,0);
    assert(!state.request);
    state.update(true,17,true,true,true,0);
    assert(state.request==17);
}
