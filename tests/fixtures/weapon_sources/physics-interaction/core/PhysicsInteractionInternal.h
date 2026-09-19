#pragma once
#include "api/WeaponSourceCatalogRuntime.h"
#include "api/EventStreams.h"
#include "physics-interaction/weapon/WeaponHierarchy.h"
namespace rock {
    inline bool finiteNiTransform(const RE::NiTransform& value) {return weapon_hierarchy::weaponTransformFinite(value);}
    struct SourceTestDescriptor {bool valid{};std::uintptr_t sourceRootAddress{};std::uint32_t bodyId{};};
    struct SourceTestEvidence: std::vector<SourceTestDescriptor> {
        const SourceTestDescriptor* find(std::uint32_t body)const {for(auto& d:*this)if(d.bodyId==body)return &d;return nullptr;}
    };
    class PhysicsInteraction {
    public:
        RE::NiAVObject* weaponRoot{};
        struct Weapon {
            std::uint64_t generation{17}; SourceTestEvidence evidence;
            std::uint64_t getCurrentWeaponGenerationKey()const{return generation;}
            const SourceTestEvidence& getProfileEvidenceDescriptors()const{return evidence;}
        } _weaponCollision;
        provider::WeaponSourceCatalog _providerSources;
        struct Frame {std::atomic<std::uint64_t> palmClockGameFrameIndex{101};} _frame;
        struct Lifecycle {
            std::atomic<std::uint32_t> worldGenerationAtomic{6},skeletonGenerationAtomic{7},providerGenerationAtomic{8},collisionGenerationAtomic{9};
        } _lifecycle;
        struct GrabEvents {std::uint64_t frameCounter{};} _grabEvents;
        bool hapticsHandled{};
        void handleGrabEventHaptics(const GrabEventData&){hapticsHandled=true;}
        RE::NiAVObject* resolveEquippedWeaponInteractionNode()const{return weaponRoot;}
        void refreshProviderWeaponSources();
        std::uintptr_t resolveProviderWeaponSource(std::uint64_t,std::uint64_t)const;
        std::uint64_t providerWeaponSourceKey(std::uint64_t,std::uintptr_t)const;
        std::uint64_t providerWeaponSourceKeyForBody(std::uint64_t,std::uint32_t)const;
        std::uintptr_t resolveProviderWeaponSourceName(std::uint64_t,const char*)const;
        api::Status copyProviderWeaponSources(std::uint64_t,std::uint32_t,provider::WeaponSourceRecord*,std::uint32_t,std::uint32_t&,std::uint32_t&)const;
        api::Status queryProviderWeaponSourcePose(std::uint64_t,std::uint64_t,provider::WeaponSourcePose&)const;
        api::Status queryProviderWeaponSourcePath(std::uint64_t,std::uint64_t,std::uint64_t&,std::uint32_t&)const;
        api::SampleV1 providerEventSample()const;
        void dispatchPhysicsMessage(std::uint32_t,bool,RE::TESObjectREFR*,std::uint32_t,std::uint32_t);
        void dispatchGrabEvent(GrabEventData);
    };
}
