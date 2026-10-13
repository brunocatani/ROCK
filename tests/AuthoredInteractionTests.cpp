#include "api/AuthoredInteractionLease.h"
#include "physics-interaction/collision/NativePlayerCollisionPolicy.h"
#include <cassert>
#include <limits>
int main() {
    using namespace rock;
    api::SampleV1 now{};now.frameIndex=10;now.worldGeneration=1;now.skeletonGeneration=2;now.providerGeneration=3;now.collisionGeneration=4;
    authored_interaction::Request r{};r.sample=now;r.hand=api::Hand::Left;r.referenceFormId=100;r.flags=3;
    authored_interaction::Lease lease;
    assert(lease.set(7,r,now)==api::Status::Ok&&lease.current(now));
    r.flags=6;assert(lease.set(7,r,now)==api::Status::Ok);
    assert(lease.set(8,r,now)==api::Status::OwnerConflict);
    assert(lease.clear(8,now)==api::Status::OwnerConflict);
    r.flags=4;assert(lease.set(7,r,now)==api::Status::InvalidArgument);
    r.flags=3;r.contactWorldGame.x=std::numeric_limits<float>::quiet_NaN();assert(lease.set(7,r,now)==api::Status::InvalidArgument);
    r.contactWorldGame.x=0;r.sample.worldGeneration++;assert(lease.set(7,r,now)==api::Status::GenerationMismatch);
    r.sample=now;r.sample.frameIndex-=2;assert(lease.set(7,r,now)==api::Status::GenerationMismatch);
    auto changed=now;++changed.collisionGeneration;assert(!lease.current(changed));
    now.frameIndex=13;assert(!lease.current(now));r.sample=now;assert(lease.set(8,r,now)==api::Status::Ok);
    assert(lease.clear(8,now)==api::Status::Ok&&!lease.current(now));
    // Only the selected live proxy and world surfaces are suppressed. Hand,
    // weapon, actor, dynamic-car and clutter contacts remain native.
    using namespace collision_layer_policy;
    native_player_collision::BodyIdentity original{10,2,123,456},reused{10,2,789,456};
    using native_player_collision::suppressAuthoredHandWorldPair;
    assert(suppressAuthoredHandWorldPair(original,original,FO4_LAYER_STATIC));
    assert(suppressAuthoredHandWorldPair(original,original,FO4_LAYER_ANIMSTATIC));
    for(auto layer:{ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY,ROCK_LAYER_WEAPON,FO4_LAYER_BIPED_NO_CC,ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER,FO4_LAYER_CLUTTER})assert(!suppressAuthoredHandWorldPair(original,original,layer));
    assert(!suppressAuthoredHandWorldPair(original,reused,FO4_LAYER_STATIC));
    assert(!suppressAuthoredHandWorldPair({},original,FO4_LAYER_STATIC));
}
