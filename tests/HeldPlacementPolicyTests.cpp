#include "physics-interaction/grab/HeldPlacementPolicy.h"
#include <cassert>

int main()
{
    using namespace rock::held_placement_policy;
    FrameLease lease;
    using Admission=FrameLease::Admission;
    const auto firstToken=lease.token();
    assert(lease.submit(0,firstToken)==Admission::InvalidOwner);
    assert(lease.submit(17,firstToken)==Admission::Accepted);
    assert(lease.submit(17,firstToken)==Admission::Accepted);
    assert(lease.submit(18,firstToken)==Admission::Busy);
    lease.clear(18); // One consumer cannot clear another's request.
    assert(lease.owner()==17);
    assert(lease.consume()==17);
    assert(lease.consume()==0); // No callback means no retained input/action.
    assert(lease.submit(17,firstToken)==Admission::Stale);
    assert(lease.submit(18,lease.token())==Admission::Accepted);
    lease.clear(18);
    assert(lease.consume()==0);
    const auto beforeWorldLoss=lease.token();
    assert(lease.submit(17,beforeWorldLoss)==Admission::Accepted);
    lease.invalidate();
    assert(lease.consume()==0);
    assert(lease.submit(17,beforeWorldLoss)==Admission::Stale);
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
}
