#pragma once

#include "physics-interaction/weapon/LooseWeaponCollisionPolicy.h"
#include "RE/NetImmerse/NiTransform.h"
#include <array>
#include <atomic>
#include <memory>

namespace RE { class TESObjectREFR; class hknpWorld; }
namespace rock {
    struct PhysicsFrameContext;
    class PhysicsCallbackQuiescenceGate;

    // The interaction thread owns references, shapes and all native mutations.
    // Contact callbacks read atomic body IDs and publish only a value witness.
    // Parked objects retain native saved state and leave this bounded pool.
    class LooseWeaponCollision
    {
    public:
        LooseWeaponCollision();
        ~LooseWeaponCollision();
        void update(const PhysicsFrameContext& frame, RE::TESObjectREFR* right,
            RE::TESObjectREFR* left, PhysicsCallbackQuiescenceGate& gate,bool admitWeapons=true);
        enum class NativeRest { Watching, Resting, Unavailable, CapacityFull };
        NativeRest watchNative(RE::TESObjectREFR*,const PhysicsFrameContext&,PhysicsCallbackQuiescenceGate&);
        void clear(RE::hknpWorld* liveWorld);
        bool prepareRelease(RE::TESObjectREFR* reference, RE::hknpWorld* liveWorld,
            PhysicsCallbackQuiescenceGate& gate, bool physicalDrop);
        bool releaseReference(RE::TESObjectREFR* reference, RE::hknpWorld* liveWorld,
            PhysicsCallbackQuiescenceGate& gate);
        bool watches(std::uint32_t body) const noexcept;
        void contact(RE::hknpWorld* world, std::uint32_t a, std::uint32_t b,
            const RE::NiPoint3& point, const RE::NiPoint3& normal) noexcept;

    private:
        struct State;
        struct Slot
        {
            Slot();
            ~Slot();
            void admit(RE::TESObjectREFR* reference, const PhysicsFrameContext& frame);
            void update(const PhysicsFrameContext& frame, RE::TESObjectREFR* right,
                RE::TESObjectREFR* left, PhysicsCallbackQuiescenceGate& gate, bool prepare);
            void clear(RE::hknpWorld* liveWorld);
            void publishBodies();
            bool watches(std::uint32_t body) const noexcept;
            void contact(RE::hknpWorld* world, std::uint32_t a, std::uint32_t b,
                const RE::NiPoint3& point, const RE::NiPoint3& normal) noexcept;

            std::unique_ptr<State> _state;
            std::uint32_t _declined{};
            std::uint32_t _rested{};
            std::atomic<RE::hknpWorld*> _world{};
            std::array<std::atomic<std::uint32_t>, loose_weapon_collision_policy::kMaxNativeBodies> _bodies{};
            std::atomic<unsigned> _count{};
            std::atomic<std::uint64_t> _generation{1};
            struct Support
            {
                std::uint64_t generation{};
                std::uint32_t weaponBody{}, surfaceBody{};
                std::uintptr_t surfaceCollision{};
                RE::NiTransform weaponPose{}, surfacePose{};
                bool valid{};
            };
            std::atomic_flag _supportBusy = ATOMIC_FLAG_INIT;
            Support _support{};
        };
        void publishWatchMask() noexcept;
        std::array<Slot, loose_weapon_collision_policy::kMaxWeapons> _slots;
        std::atomic<std::uint32_t> _watchMask{};
        std::size_t _nextPreparationSlot{};
    };
}
