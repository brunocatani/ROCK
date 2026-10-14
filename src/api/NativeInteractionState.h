#pragma once
#include "NativeInteractionService.h"
#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/PhysicsCallbackQuiescenceGate.h"
#include <array>
#include <atomic>

namespace rock::native_interaction_service::detail {
    namespace ni = api::native_interaction;
    struct Extension {
        Owner owner{};
        ni::Callbacks callbacks{};
        std::uint64_t obstacleLayers{};
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
    api::Status activateWeapon(Owner,std::uint32_t) noexcept;
    api::Status setWeaponContactExclusion(Owner,std::uint32_t,const ni::BodyIdentity*) noexcept;
    api::Status clearWeaponContactExclusion(Owner,std::uint32_t) noexcept;
    api::Status hasWeaponContactExclusion(Owner,std::uint32_t,const ni::BodyIdentity*,std::uint32_t*) noexcept;
    void releaseWeaponResources(Owner,RE::hknpWorld*);
    RE::NiTransform fromApi(const api::Transform&);
    api::Transform toApi(const RE::NiTransform&);
    RE::NiPoint3 fromApi(const api::Point3&);
    api::Point3 toApi(const RE::NiPoint3&);
    bool finite(const api::Transform&);
}
