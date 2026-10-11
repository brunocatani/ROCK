#include "physics-interaction/core/PhysicsInteraction.h"
#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/native/NativeObjectPlacement.h"
#include "physics-interaction/native/NativePlayerCollisionFilter.h"
#include "physics-interaction/native/MagazineEjectionSnapshot.h"
#include "RE/Bethesda/TESBoundObjects.h"
namespace rock {
namespace {
struct WriteScope {
    RE::hknpWorld* world{};
    explicit WriteScope(RE::hknpWorld* value) {
        if(havok_world_lock::detail::currentThreadInsidePhysicsStep())return;
        reinterpret_cast<void(*)(RE::hknpWorld*)>(REL::Offset(0x1DF5FB0).address())(value);world=value;
    }
    ~WriteScope(){if(world)reinterpret_cast<void(*)(RE::hknpWorld*)>(REL::Offset(0x1DF5FD0).address())(world);}
};
struct Bodies {std::array<std::uint32_t,4> ids{};std::size_t count{};};
bool body(std::uint32_t id,void* data) {
    auto& out=*static_cast<Bodies*>(data);
    if(std::find(out.ids.begin(),out.ids.begin()+out.count,id)!=out.ids.begin()+out.count)return true;
    if(out.count==out.ids.size())return false;out.ids[out.count++]=id;return true;
}
bool collect(RE::NiAVObject* root,RE::hknpWorld* world,Bodies& out) {
    std::array<RE::NiAVObject*,128> nodes{};nodes[0]=root;std::size_t count=1;
    for(std::size_t i=0;i<count;++i) {
        auto* node=nodes[i];if(!node)return false;
        if(node->collisionObject) {
            const auto scan=havok_runtime::forEachPhysicsSystemBodyIdDetailed(node->collisionObject.get(),world,4,&body,&out);
            if(!scan.enumerated())return false;
        }
        if(auto* parent=node->IsNode())for(const auto& child:parent->children)if(child){if(count==nodes.size()||child->parent!=parent)return false;nodes[count++]=child.get();}
    }
    return out.count!=0;
}
}
void PhysicsInteraction::clearEjectionContacts(RE::hknpWorld* world) {
    if(world&&native_object_placement::available()) {WriteScope lock(world);native_player_collision::clearMagazineEjection(world);}
    else native_player_collision::clearMagazineEjection(nullptr);
    _ejectionContacts.entries={};_ejectionContacts.hadPairs=false;_ejectionContacts.handContactReported=false;
    (void)native_player_collision::takeMagazineEjectionHandPair();
}
void PhysicsInteraction::updateEjectionContacts(const PhysicsFrameContext& frame) {
    auto& service=_ejectionContacts;
    if(!service.hadPairs&&std::none_of(service.entries.begin(),service.entries.end(),[](const auto& e){return e.owner&&!e.finished;}))return;
    if(!frame.hknpWorld||!native_object_placement::available())return;
    const auto sample=provider::runtime::sample();const auto weapon=_weaponCollision.getWeaponBodySnapshotAtomic();
    const float dt=std::isfinite(frame.deltaSeconds)?std::clamp(frame.deltaSeconds,0.f,.1f):0.f;
    std::array<std::uint32_t,32> magazines{};std::size_t count=0;std::array<bool,8> prepared{};
    std::array<std::uint32_t,128> weapons{};std::size_t weaponCount=0;
    std::array<std::uint32_t,32> handProtected{};std::size_t protectedCount=0;
    std::array<std::uint32_t,128> hands{};std::size_t handCount=0;
    static_assert(2*(hand_collider_semantics::kHandColliderBodyCountPerHand+2)<=128);
    const auto addHand=[&](std::uint32_t id){if(id!=0x7fffffff&&handCount<hands.size())hands[handCount++]=id;};
    for(bool left:{false,true}) {
        const auto& hand=left?_leftHand:_rightHand;
        const auto n=std::min(hand.getHandColliderBodyCount(),static_cast<std::uint32_t>(hand_collider_semantics::kHandColliderBodyCountPerHand));
        for(std::uint32_t i=0;i<n;++i)addHand(hand.getHandColliderBodyIdAtomic(i));
        if(!n)addHand(hand.getCollisionBodyId().value);
        addHand(_dynamicHandCollision.proxyBodyIdForDebug(left,0).value);
    }
    const auto handPair=native_player_collision::takeMagazineEjectionHandPair();
    if(handPair&&!service.handContactReported){service.handContactReported=true;ROCK_LOG_INFO(Weapon,"Ejection hand contact rejected: magazineBody={} handBody={}",handPair>>32,std::uint32_t(handPair));}
    static_assert(std::tuple_size_v<decltype(weapon.bodyIds)> < 128);
    for(std::size_t i=0;i<weapon.count&&i<weapon.bodyIds.size();++i)if(weapon.bodyIds[i]!=0x7fffffff)weapons[weaponCount++]=weapon.bodyIds[i];
    const auto proxy=_dynamicWeaponCollision.proxyBodyIdForDebug();
    if(proxy.value!=0x7fffffff)weapons[weaponCount++]=proxy.value;
    WriteScope lock(frame.hknpWorld);
    for(auto& entry:service.entries)if(entry.owner&&!entry.finished) {
        const auto& request=entry.request;const auto& key=request.reference;
        if(entry.cancelled||provider::runtime::authorize(entry.owner,api::InterfaceId::Grab,3,true)!=api::Status::Ok||
           key.worldGeneration!=sample.worldGeneration||key.skeletonGeneration!=sample.skeletonGeneration||key.providerGeneration!=sample.providerGeneration||
           weapon.generationKey!=request.weaponGenerationKey) {entry={};continue;}
        auto* ref=RE::TESForm::GetFormByID<RE::TESObjectREFR>(key.referenceFormId);auto* base=ref?ref->GetObjectReference():nullptr;
        if(!ref||ref->GetFormType()!=RE::ENUM_FORM_ID::kREFR||ref->IsDeleted()||ref->IsDisabled()||!base||base->formID!=request.expectedBase||
            (key.referenceNativeHandle&&ref->GetHandle().native_handle()!=key.referenceNativeHandle)){entry={};continue;}
        if(entry.active){entry.age+=dt;if(entry.age>=1.f){entry.finished=true;entry.active=false;ROCK_LOG_INFO(Weapon,"Ejection contact lease expired ref={:08X} totalRejectedPairs={}",key.referenceFormId,native_player_collision::magazineEjectionRejectedCount());continue;}}
        else {entry.waiting+=dt;if(entry.waiting>1.5f){ROCK_LOG_WARN(Weapon,"Ejection contact lease activation timed out ref={:08X}",key.referenceFormId);entry={};continue;}}
        Bodies bodies;auto* root=ref->Get3D();if(!root||!weapon.count||weapon.count>weapon.bodyIds.size()||!collect(root,frame.hknpWorld,bodies))continue;
        const auto start=count;
        for(std::size_t a=0;a<bodies.count;++a)if(count<magazines.size()){
            magazines[count++]=bodies.ids[a];if(entry.hands)handProtected[protectedCount++]=bodies.ids[a];
        }
        prepared[&entry-service.entries.data()]=count>start;
    }
    const auto unique=[](auto& ids,std::size_t size) {std::sort(ids.begin(),ids.begin()+size);return static_cast<std::size_t>(std::unique(ids.begin(),ids.begin()+size)-ids.begin());};
    count=unique(magazines,count);weaponCount=unique(weapons,weaponCount);
    protectedCount=unique(handProtected,protectedCount);handCount=unique(hands,handCount);
    if(!protectedCount)handCount=0;
    if(!count)service.handContactReported=false;
    const bool applied=native_player_collision::publishMagazineEjection(frame.hknpWorld,{magazines.data(),count},{weapons.data(),weaponCount},{handProtected.data(),protectedCount},{hands.data(),handCount});
    if(applied)service.hadPairs=count!=0&&weaponCount!=0;
    if(applied&&weaponCount)for(std::size_t i=0;i<prepared.size();++i)if(prepared[i]&&!service.entries[i].active) {
        auto& entry=service.entries[i];entry.active=true;entry.age=0;
        ROCK_LOG_INFO(Weapon,"Ejection contact lease active ref={:08X} weapon={:08X} duration=1s backend=simulation hands={} handBodies={}",entry.request.reference.referenceFormId,entry.request.weaponForm,entry.hands,entry.hands?handCount:0);
    }
    if(count&&!applied)ROCK_LOG_SAMPLE_WARN(Weapon,2000,"Ejection simulation publication pending magazines={} weaponBodies={} hook={}",count,weaponCount,native_player_collision::installed());
}
}
