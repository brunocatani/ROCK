#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "api/ProviderRuntimeServices.h"
#include "api/ProviderFrameClock.h"
#include <cassert>
#include <limits>
namespace { rock::provider::ProviderFrameClock eventClock; }
namespace rock {
#include "physics-interaction/core/interaction/ProviderEventDispatch.inl"
}
namespace rock::provider::runtime {
    // The old event producer used this stale snapshot. The current one must not.
    std::uint64_t currentGameFrameIndex(){return eventClock.current();}
    api::SampleV1 sample(){return {100,1,2,3,4,5};}
    void deferRevoke(std::uint64_t){assert(false);}
}
namespace {
    template<class T=RE::NiAVObject> auto child(const std::shared_ptr<RE::NiNode>& parent,const char* name) {
        auto node=std::make_shared<T>();node->parent=parent.get();node->name=name;
        parent->data.children.push_back(node);return node;
    }
    std::uint64_t key(rock::PhysicsInteraction& pi,RE::NiAVObject* node){return pi.providerWeaponSourceKey(pi._weaponCollision.generation,reinterpret_cast<std::uintptr_t>(node));}
    void sourceTests() {
        using rock::api::Status;
        auto pi=std::make_unique<rock::PhysicsInteraction>();
        auto root=std::make_shared<RE::NiNode>();root->name="Weapon";pi->weaponRoot=root.get();
        auto slide=child(root,"Slide");slide->local.translate.x=5;slide->world.translate.x=5;
        pi->refreshProviderWeaponSources();const auto slideKey=key(*pi,slide.get());const auto rootKey=key(*pi,root.get());
        assert(slideKey && rootKey);
        auto duplicate=child(root,"Slide");pi->refreshProviderWeaponSources();
        std::uint32_t copied{},total{};
        assert(pi->copyProviderWeaponSources(17,0,nullptr,0,copied,total)==Status::Ok && total==3);
        assert(!pi->resolveProviderWeaponSourceName(17,"Slide"));
        assert(key(*pi,slide.get())==slideKey && key(*pi,root.get())==rootKey);
        duplicate->name="Other";pi->refreshProviderWeaponSources();
        assert(pi->resolveProviderWeaponSourceName(17,"Slide")==reinterpret_cast<std::uintptr_t>(slide.get()));
        root->world.translate.x=100;root->world.scale=2;
        rock::provider::WeaponSourcePose pose{};
        assert(pi->queryProviderWeaponSourcePose(17,slideKey,pose)==Status::Ok);
        assert(pose.sourceParentLocal.translate[0]==5 && pose.weaponRootLocal.translate[0]==5 && pose.world.translate[0]==110);
        auto parent=child<RE::NiNode>(root,"Parent");parent->local.translate.x=3;
        root->data.children.erase(root->data.children.begin());parent->data.children.push_back(slide);slide->parent=parent.get();
        pi->refreshProviderWeaponSources();
        assert(key(*pi,slide.get())==slideKey);
        std::uint64_t parentKey{};std::uint32_t index{};
        assert(pi->queryProviderWeaponSourcePath(17,slideKey,parentKey,index)==Status::Ok && parentKey==key(*pi,parent.get()) && index==0);
        assert(pi->queryProviderWeaponSourcePose(17,slideKey,pose)==Status::Ok && pose.weaponRootLocal.translate[0]==8 && pose.world.translate[0]==116);
        parent->local.translate.x=std::numeric_limits<float>::quiet_NaN();
        assert(pi->queryProviderWeaponSourcePose(17,slideKey,pose)==Status::NotReady && pose.sourceKey==0);
        parent->local.translate.x=3;
        std::weak_ptr<RE::NiAVObject> retired=slide;parent->data.children.clear();slide.reset();
        assert(!retired.expired()); // Catalog pin prevents allocation reuse.
        pi->refreshProviderWeaponSources();assert(retired.expired());
        assert(pi->queryProviderWeaponSourcePose(17,slideKey,pose)==Status::GenerationMismatch);
        const auto previousRootKey=key(*pi,root.get());++pi->_weaponCollision.generation;pi->refreshProviderWeaponSources();
        assert(key(*pi,root.get())!=previousRootKey && !pi->resolveProviderWeaponSource(17,previousRootKey));
        pi->_providerSources.clear();assert(pi->_providerSources.count==0);
    }
    void boundsTests() {
        using rock::api::Status;
        auto pi=std::make_unique<rock::PhysicsInteraction>();auto root=std::make_shared<RE::NiNode>();pi->weaponRoot=root.get();
        for(std::size_t i=1;i<rock::provider::WeaponSourceCatalog::Capacity;++i)child(root,"Node");
        pi->refreshProviderWeaponSources();std::uint32_t copied{},total{};
        assert(pi->copyProviderWeaponSources(17,0,nullptr,0,copied,total)==Status::Ok && total==4096);
        root->data.children.clear();for(std::size_t i=1;i<4096;++i)child(root,"Replacement");
        pi->refreshProviderWeaponSources();assert(pi->_providerSources.count==4096);
        child(root,"Overflow");pi->refreshProviderWeaponSources();
        assert(pi->copyProviderWeaponSources(17,0,nullptr,0,copied,total)==Status::CapacityFull && total==0);
        root->data.children.clear();auto cursor=root;
        for(int i=0;i<64;++i)cursor=child<RE::NiNode>(cursor,"Deep");
        pi->refreshProviderWeaponSources();assert(!pi->_providerSources.overflow && key(*pi,cursor.get()));
        child(cursor,"TooDeep");pi->refreshProviderWeaponSources();assert(pi->_providerSources.overflow);
        root->data.children.clear();pi->refreshProviderWeaponSources();assert(!pi->_providerSources.overflow && pi->_providerSources.count==1);
    }
    void eventTests() {
        using namespace rock;
        auto pi=std::make_unique<PhysicsInteraction>();
        eventClock.beginFrame(101);pi->_frame.palmClockGameFrameIndex=0; // Initialization precedes the first physics update.
        assert(provider::events::bind(77,api::InterfaceId::Core)==api::Status::Ok);
        assert(provider::events::bind(77,api::InterfaceId::Grab)==api::Status::Ok);
        pi->dispatchPhysicsMessage(104,false,nullptr,0,0);
        api::core::EventV1 event{};api::StreamV1 stream{};
        assert(provider::events::copy(77,0,&event,1,stream)==api::Status::Ok && stream.copiedCount==1);
        assert(event.frameIndex==101 && event.worldGeneration==6 && event.skeletonGeneration==7 && event.providerGeneration==8);
        GrabEventData grab{};grab.type=GrabEventType::Released;RE::TESObjectREFR reference;grab.refr=&reference;
        pi->dispatchGrabEvent(grab);api::grab::EventV1 released{};
        assert(pi->hapticsHandled);
        assert(provider::events::copy(77,0,&released,1,stream)==api::Status::Ok && stream.copiedCount==1);
        assert(released.frameIndex==101 && released.worldGeneration==6 && released.skeletonGeneration==7 && released.providerGeneration==8 && released.formId==123);
        eventClock.beginFrame(102);pi->_frame.palmClockGameFrameIndex=102;pi->_lifecycle.worldGenerationAtomic=10;
        pi->dispatchPhysicsMessage(105,false,nullptr,0,0);
        assert(provider::events::copy(77,1,&event,1,stream)==api::Status::Ok && event.frameIndex==102 && event.worldGeneration==10);
        provider::events::remove(77);
    }
}
int main(){sourceTests();boundsTests();eventTests();}
