#include "api/WeaponPartCollisionLease.h"
#include <cassert>
#include <array>
int main() {
    using namespace rock;using api::Status;
    api::Sample sample{100,4,1,2,3,4};
    api::weaponparts::v1_2::CollisionSuppression request;request.sample=sample;
    request.weaponGenerationKey=40;request.sourceCount=2;request.sourceKeys[0]=11;request.sourceKeys[1]=12;
    weapon_part_collision::Lease lease;
    assert(lease.set(1,request,sample)==Status::Ok&&lease.current(sample));
    assert(lease.set(2,request,sample)==Status::OwnerConflict);
    assert(lease.clear(2,sample)==Status::OwnerConflict);
    request.sample.frameIndex=99;assert(lease.set(1,request,sample)==Status::Ok);
    request.sample.frameIndex=98;assert(lease.set(1,request,sample)==Status::GenerationMismatch);
    assert(lease.owner==1&&lease.expires==102);
    auto next=sample;next.frameIndex=103;assert(!lease.current(next));request.sample=next;
    assert(lease.set(2,request,next)==Status::Ok);
    for(int field=0;field<4;++field) {
        auto changed=next;
        switch(field){case 0:++changed.worldGeneration;break;case 1:++changed.skeletonGeneration;break;case 2:++changed.providerGeneration;break;case 3:++changed.collisionGeneration;break;}
        assert(!lease.current(changed));assert(lease.set(2,request,changed)==Status::GenerationMismatch);
    }
    for(const auto frames:{0u,3u}){request.leaseFrames=frames;assert(lease.set(2,request,next)==Status::InvalidArgument);}
    request.leaseFrames=2;request.sourceCount=17;assert(lease.set(2,request,next)==Status::InvalidArgument);
    request.sourceCount=0;assert(lease.set(2,request,next)==Status::InvalidArgument);
    request.sourceCount=1;request.sourceKeys[0]=0;assert(lease.set(2,request,next)==Status::InvalidArgument);
    assert(lease.clear(2,next)==Status::Ok&&!lease.current(next));
    // Full and empty magazine subtrees match all their compound bodies while
    // frame, slide, a different weapon, and the spawned loose copy stay active.
    struct Node {const Node* parent{};};
    Node weapon{},full{&weapon},empty{&weapon},shell{&full},geometry{&shell},frameNode{&weapon},slide{&weapon},loose{},otherWeapon{},otherMag{&otherWeapon};
    const std::array<const Node*,2> roots{&full,&empty};
    const auto selected=[&](const Node& n){return weapon_part_collision::selectedSource(&n,std::span<const Node* const>(roots),&weapon);};
    for(const auto* n:{&full,&empty,&shell,&geometry})assert(selected(*n));
    for(const auto* n:{&weapon,&frameNode,&slide,&loose,&otherMag})assert(!selected(*n));
    Node cycle{};cycle.parent=&cycle;assert(!selected(cycle));
}
