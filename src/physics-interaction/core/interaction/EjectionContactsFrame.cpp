#include "physics-interaction/core/PhysicsInteraction.h"
#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/native/NativeObjectPlacement.h"
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
    if(world&&native_object_placement::available()) {WriteScope lock(world);(void)_ejectionContacts.pairs.reconcile(world,nullptr,0);}
    _ejectionContacts.pairs.abandonWorld();_ejectionContacts.entries={};_ejectionContacts.hadPairs=false;
}
void PhysicsInteraction::updateEjectionContacts(const PhysicsFrameContext& frame) {
    auto& service=_ejectionContacts;
    if(!service.hadPairs&&std::none_of(service.entries.begin(),service.entries.end(),[](const auto& e){return e.owner&&!e.finished;}))return;
    if(!frame.hknpWorld||!native_object_placement::available())return;
    const auto sample=provider::runtime::sample();const auto weapon=_weaponCollision.getWeaponBodySnapshotAtomic();
    const float dt=std::isfinite(frame.deltaSeconds)?std::clamp(frame.deltaSeconds,0.f,.1f):0.f;
    std::array<BasicHavokPairCollisionLeaseSet<4096,1>::DesiredPair,4096> desired{};std::size_t count=0;std::array<bool,8> prepared{};
    WriteScope lock(frame.hknpWorld);
    for(auto& entry:service.entries)if(entry.owner&&!entry.finished) {
        const auto& request=entry.request;const auto& key=request.reference;
        if(entry.cancelled||provider::runtime::authorize(entry.owner,api::InterfaceId::Grab,3,true)!=api::Status::Ok||
           key.worldGeneration!=sample.worldGeneration||key.skeletonGeneration!=sample.skeletonGeneration||key.providerGeneration!=sample.providerGeneration||
           weapon.generationKey!=request.weaponGenerationKey) {entry={};continue;}
        auto* ref=RE::TESForm::GetFormByID<RE::TESObjectREFR>(key.referenceFormId);auto* base=ref?ref->GetObjectReference():nullptr;
        if(!ref||ref->GetFormType()!=RE::ENUM_FORM_ID::kREFR||ref->IsDeleted()||ref->IsDisabled()||!base||base->formID!=request.expectedBase||
            (key.referenceNativeHandle&&ref->GetHandle().native_handle()!=key.referenceNativeHandle)){entry={};continue;}
        if(entry.active){entry.age+=dt;if(entry.age>=1.f){entry.finished=true;entry.active=false;ROCK_LOG_INFO(Weapon,"Ejection contact lease expired ref={:08X}",key.referenceFormId);continue;}}
        else {entry.waiting+=dt;if(entry.waiting>1.5f){ROCK_LOG_WARN(Weapon,"Ejection contact lease body wait timed out ref={:08X}",key.referenceFormId);entry={};continue;}}
        Bodies bodies;auto* root=ref->Get3D();if(!root||!weapon.count||weapon.count>weapon.bodyIds.size()||!collect(root,frame.hknpWorld,bodies))continue;
        const auto start=count;
        for(std::size_t a=0;a<bodies.count;++a) {
            for(std::size_t b=0;b<weapon.count;++b)if(weapon.bodyIds[b]!=0x7fffffff&&weapon.bodyIds[b]!=bodies.ids[a]&&count<desired.size())desired[count++]={bodies.ids[a],weapon.bodyIds[b],0};
            const auto proxy=_dynamicWeaponCollision.proxyBodyIdForDebug();
            if(proxy.value!=0x7fffffff&&proxy.value!=bodies.ids[a]&&count<desired.size())desired[count++]={bodies.ids[a],proxy.value,0};
        }
        prepared[&entry-service.entries.data()]=count>start;
    }
    for(std::size_t i=0;i<count;++i)if(desired[i].bodyB<desired[i].bodyA)std::swap(desired[i].bodyA,desired[i].bodyB);
    const auto less=[](const auto& a,const auto& b){return a.bodyA<b.bodyA||(a.bodyA==b.bodyA&&a.bodyB<b.bodyB);};
    std::sort(desired.begin(),desired.begin()+count,less);
    count=std::unique(desired.begin(),desired.begin()+count,[](const auto& a,const auto& b){return a.bodyA==b.bodyA&&a.bodyB==b.bodyB;})-desired.begin();
    const auto result=service.pairs.reconcile(frame.hknpWorld,desired.data(),count);service.hadPairs=result.activePairCount!=0;
    if(result.filterAvailable&&result.activePairCount==count)for(std::size_t i=0;i<prepared.size();++i)if(prepared[i]&&!service.entries[i].active) {
        auto& entry=service.entries[i];entry.active=true;entry.age=0;
        ROCK_LOG_INFO(Weapon,"Ejection contact lease active ref={:08X} weapon={:08X} duration=1s",entry.request.reference.referenceFormId,entry.request.weaponForm);
    }
    if(count&&result.activePairCount!=count)ROCK_LOG_SAMPLE_WARN(Weapon,2000,"Ejection contact lease pairs pending requested={} active={}",count,result.activePairCount);
}
}
