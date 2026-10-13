#pragma once
#include "NativeInteractionService.h"
#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/PhysicsCallbackQuiescenceGate.h"
#include <array>
#include <atomic>
#include <memory>

namespace rock::native_interaction_service::detail {
    namespace ni = api::native_interaction;
    struct Extension {
        Owner owner{};
        ni::Callbacks callbacks{};
        std::uint64_t obstacleLayers{};
        std::array<ni::BodyVisual, ni::kMaxVisualBodies> bodies{};
        std::uint32_t bodyCount{};
        std::uint64_t visualFrame{};
    };
    struct BodySlot {
        Owner owner{};
        std::uint32_t serial{};
        BethesdaPhysicsBody body;
        RE::hknpWorld* world{};
        void* bhk{};
        std::array<char,48> name{};
        ni::PoseBasis poseBasis{ni::PoseBasis::NiRows};
        bool privateAnchor{};
    };
    struct HavokReferenceDeleter { void operator()(void*) const noexcept; };
    struct SliderSlot {
        Owner owner{};
        std::uint32_t serial{}, id{ni::kInvalidBody};
        std::unique_ptr<void,HavokReferenceDeleter> data;
        RE::hknpWorld* world{};
        ni::Resource anchor{};
    };
    struct State {
        // Single game owner mutates registrations/resources while callbacks are
        // drained. Native step/contact readers hold this gate, never a mutex.
        PhysicsCallbackQuiescenceGate gate;
        PhysicsCallbackQuiescenceGate* rockGate{};
        RetireWeapon retireWeapon{};
        RefreshLayers refreshLayers{};
        void* retireContext{};
        std::array<Extension, ni::kMaxExtensions> extensions{};
        std::array<std::atomic<Owner>,ni::kMaxExtensions> registeredOwners{};
        std::array<BodySlot, ni::kMaxBodies> bodies{};
        std::array<SliderSlot, 32> sliders{};
        std::atomic<std::uint64_t> obstacleLayers{};
        std::atomic<std::uint32_t> faultedExtensions{};
        std::array<std::atomic<Owner>,ni::kMaxExtensions> pendingRemoval{};
        std::atomic<bool> removeAll{};
        ni::Frame frame{};
        Owner driver{};
        std::uint32_t weaponBody{ni::kInvalidBody};
        std::uint32_t excludedTarget{ni::kInvalidBody};
        ni::WeaponCommand command{};
        const WeaponCollision* collision{}; // only during WeaponUpdate
        RE::NiNode* weaponNode{}; // only during WeaponUpdate
        std::uint64_t generation{};
        std::uint32_t nextSerial{1};
    };
    State& state();
    struct CallContext { Owner owner{}; ni::Frame frame{}; };
    extern thread_local const CallContext* current;
    struct ContextScope {
        CallContext value;
        const CallContext* previous;
        ContextScope(Owner owner,const ni::Frame& frame):value{owner,frame},previous(current){current=&value;}
        ~ContextScope(){current=previous;}
    };
    api::Status access(Owner,bool structural=false,bool write=false);
    Extension* extension(Owner);
    template<class F> api::Status guarded(Owner owner,F&& operation) noexcept {
        try { return operation(); }
        catch (...) {
            if(auto* slot=extension(owner))state().faultedExtensions.fetch_or(1u<<static_cast<unsigned>(slot-state().extensions.data()),std::memory_order_release);
            return api::Status::InternalError;
        }
    }
    api::Status readBody(Owner,std::uint32_t,ni::BodySnapshot*) noexcept;
    api::Status resolveCollisionBody(Owner,std::uintptr_t,std::uint32_t*) noexcept;
    api::Status resolveBodyReference(Owner,std::uint32_t,std::uintptr_t*) noexcept;
    api::Status queryNodeRelation(Owner,std::uintptr_t,std::uintptr_t,ni::NodeRelation,std::uint32_t,std::uint32_t*) noexcept;
    api::Status createBody(Owner,const ni::BodyCreate*,ni::Resource*,std::uint32_t*) noexcept;
    api::Status retireBody(Owner,ni::Resource) noexcept;
    api::Status placeBody(Owner,ni::Resource,const api::Transform*) noexcept;
    api::Status driveBody(Owner,ni::Resource,const api::Transform*,float) noexcept;
    api::Status setBodyFilter(Owner,ni::Resource,std::uint32_t) noexcept;
    api::Status activateWeapon(Owner) noexcept;
    api::Status createSlider(Owner,const ni::SliderCreate*,ni::Resource*) noexcept;
    api::Status setSliderLimits(Owner,ni::Resource,float,float) noexcept;
    api::Status destroySlider(Owner,ni::Resource) noexcept;
    api::Status setWeaponContactExclusion(Owner,const ni::BodyIdentity*) noexcept;
    api::Status clearWeaponContactExclusion(Owner) noexcept;
    api::Status hasWeaponContactExclusion(Owner,const ni::BodyIdentity*,std::uint32_t*) noexcept;
    void releaseResources(Owner,RE::hknpWorld*,void*);
    void releaseWeaponResources(Owner,RE::hknpWorld*,void*);
    RE::NiTransform fromApi(const api::Transform&);
    api::Transform toApi(const RE::NiTransform&);
    RE::NiPoint3 fromApi(const api::Point3&);
    api::Point3 toApi(const RE::NiPoint3&);
    bool finite(const api::Transform&);
    bool matches(RE::hknpWorld*,const ni::BodyIdentity&);
    Extension* extension(Owner);
}
