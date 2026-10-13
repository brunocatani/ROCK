#include "WeaponPartCollisionService.h"
#include "WeaponPartCollisionLease.h"
#include "ProviderRuntimeServices.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/PhysicsLog.h"
#include <algorithm>
#include <array>

namespace rock::weapon_part_collision {
namespace {
    namespace collision=collision_suppression_registry;
    Lease lease;
    std::array<std::uint32_t,MAX_WEAPON_COLLISION_BODIES> selected{};
    std::uint32_t selectedCount{};
    collision::SuppressionLeaseSet<MAX_WEAPON_COLLISION_BODIES> bodies{collision::CollisionSuppressionOwner::ConsumerWeaponPart};
    RE::hknpWorld* trackedWorld{}; // Identity only; never dereferenced after world loss.
}
api::Status set(api::OwnerToken owner,const api::weaponparts::v1_2::CollisionSuppression& request) {
    const auto sample=provider::runtime::sample();
    auto next=lease;
    if(const auto status=next.set(owner,request,sample);status!=api::Status::Ok)return status;
    auto* physics=PhysicsInteraction::s_instance.load(std::memory_order_acquire);
    if(!physics||!physics->isProviderReady())return api::Status::NotReady;
    std::array<std::uint32_t,MAX_WEAPON_COLLISION_BODIES> matched{};std::uint32_t count{};
    physics->refreshProviderWeaponSources();
    const auto status=physics->collectProviderWeaponCollisionBodies(request.weaponGenerationKey,
        std::span(request.sourceKeys).first(request.sourceCount),matched,count);
    if(status!=api::Status::Ok)return status;
    if(!lease.current(sample)||lease.owner!=owner||selectedCount!=count||!std::equal(matched.begin(),matched.begin()+count,selected.begin()))
        ROCK_LOG_INFO(Weapon,"Consumer weapon-part collision suppressed owner={} generation={} sources={} bodies={}",owner,request.weaponGenerationKey,request.sourceCount,count);
    lease=next;selected=matched;selectedCount=count;return api::Status::Ok;
}
api::Status clear(api::OwnerToken owner) {
    return lease.clear(owner,provider::runtime::sample());
}
void update(PhysicsInteraction& physics,const PhysicsFrameContext& frame) {
    if(trackedWorld&&trackedWorld!=frame.hknpWorld)bodies.clearTracking();
    trackedWorld=frame.hknpWorld;
    const auto sample=provider::runtime::sample();
    if(!frame.worldReady||frame.menuBlocked||!lease.current(sample)||
        provider::runtime::authorize(lease.owner,api::InterfaceId::WeaponParts,2,true)!=api::Status::Ok||
        std::any_of(selected.begin(),selected.begin()+selectedCount,[&](auto id){return !physics.isProviderWeaponBodyCurrentV1(lease.request.weaponGenerationKey,id);}))lease={};
    if(!lease.owner)selectedCount=0;
    if(!frame.worldReady)return;
    bodies.releaseWhere(frame.hknpWorld,"consumer-weapon-part-clear",
        [&](auto id){return std::find(selected.begin(),selected.begin()+selectedCount,id)==selected.begin()+selectedCount;},
        [](auto id,const auto& result){if(result.readFailed)ROCK_LOG_SAMPLE_WARN(Weapon,1000,"Consumer weapon-part collision restore pending body={}",id);});
    for(std::uint32_t i=0;i<selectedCount;++i) {
        const auto result=bodies.acquire(frame.hknpWorld,selected[i],"consumer-weapon-part");
        if(!result.valid)ROCK_LOG_SAMPLE_WARN(Weapon,1000,"Consumer weapon-part collision suppression unavailable body={}",selected[i]);
        else if(result.firstLeaseForBody)ROCK_LOG_INFO(Weapon,"Consumer weapon-part collision filter body={} before={:08X} after={:08X}",selected[i],result.filterBefore,result.filterAfter);
    }
}
void shutdown(RE::hknpWorld* world) {
    lease={};selectedCount=0;
    if(world)(void)bodies.releaseAll(world,"consumer-weapon-part-shutdown",[](auto,const auto&){});
    else bodies.clearTracking();
    if(bodies.empty())trackedWorld=nullptr;
}
}
