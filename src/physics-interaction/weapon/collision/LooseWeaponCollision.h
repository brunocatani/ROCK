// LooseWeaponCollision: generated collision for loose references in the
// world, kept in a small bounded pool (see LooseWeaponCollision.cpp). One
// class serves three pools: loose weapons, object rest watches and the
// experimental generated collision for held loose objects.
#pragma once

#include <array>
#include <atomic>
#include <memory>
#include "RE/Bethesda/BSPointerHandle.h"
#include "RE/NetImmerse/NiTransform.h"
#include "physics-interaction/weapon/collision/DroppedWeaponCollision.h"
#include "physics-interaction/weapon/collision/LooseWeaponCollisionPolicy.h"

namespace RE
{
    class TESObjectREFR;
    class hknpWorld;
}

namespace rock
{
    struct PhysicsFrameContext;
    class PhysicsCallbackQuiescenceGate;

    // The interaction thread owns references, shapes and all native mutations.
    // Contact callbacks read atomic body IDs and publish only a value witness.
    // Parked objects retain native saved state and leave this bounded pool.
    class LooseWeaponCollision
    {
    public:
        enum class Pool
        {
            // Held loose weapons get generated hulls; at rest they are frozen
            // in place and their native collision is restored.
            Weapons,
            // Native rest watches only (Collision API watchObjectRest).
            ObjectRest,
            // Held single-body loose objects get generated hulls, keep them
            // while thrown or dropped, and get their native collision back
            // after two seconds of supported rest. They are never frozen.
            Objects
        };

        explicit LooseWeaponCollision(Pool pool);
        ~LooseWeaponCollision();
        // right/left: each hand's held reference, or null. The Objects pool's
        // caller passes only held loose objects it may admit.
        void update(const PhysicsFrameContext& frame, RE::TESObjectREFR* right, RE::TESObjectREFR* left,
            PhysicsCallbackQuiescenceGate& gate);
        bool empty() const noexcept;
        bool tracks(const RE::TESObjectREFR* reference) const noexcept;
        enum class NativeRest
        {
            Watching,
            Resting,
            Unavailable,
            CapacityFull
        };
        NativeRest watchNative(RE::TESObjectREFR*, const PhysicsFrameContext&, PhysicsCallbackQuiescenceGate&);
        bool takeNativeRestCompletion(std::uint32_t reference) noexcept;

        // Equipped drops: the dropped reference keeps the equipped hulls from
        // the first frame its native bodies exist until rest retirement.
        // Returns false while its native bodies are not yet published; the
        // drop handoff calls it before its placement solve. A declined or
        // empty set leaves native collision under the same rest watch.
        bool adoptDropped(RE::TESObjectREFR* reference, DroppedWeaponCollision& collision,
            const PhysicsFrameContext& frame, PhysicsCallbackQuiescenceGate& gate);
        // Placement finished before the bodies were published: retry each update.
        void queueDropped(const RE::ObjectRefHandle& handle, DroppedWeaponCollision collision);

        void clear(RE::hknpWorld* liveWorld);
        bool prepareRelease(RE::TESObjectREFR* reference, RE::hknpWorld* liveWorld, PhysicsCallbackQuiescenceGate& gate,
            bool physicalDrop);
        bool releaseReference(RE::TESObjectREFR* reference, RE::hknpWorld* liveWorld,
            PhysicsCallbackQuiescenceGate& gate);
        bool watches(std::uint32_t body) const noexcept;
        void contact(RE::hknpWorld* world, std::uint32_t a, std::uint32_t b, const RE::NiPoint3& point,
            const RE::NiPoint3& normal) noexcept;

    private:
        struct State;

        struct Slot
        {
            Slot();
            ~Slot();
            void admit(RE::TESObjectREFR* reference, const PhysicsFrameContext& frame, Pool pool);
            void update(const PhysicsFrameContext& frame, RE::TESObjectREFR* right, RE::TESObjectREFR* left,
                PhysicsCallbackQuiescenceGate& gate, bool prepare);
            void clear(RE::hknpWorld* liveWorld);
            void publishBodies();
            bool watches(std::uint32_t body) const noexcept;
            void contact(RE::hknpWorld* world, std::uint32_t a, std::uint32_t b, const RE::NiPoint3& point,
                const RE::NiPoint3& normal) noexcept;

            std::unique_ptr<State> _state;
            std::uint32_t _declined{};
            std::uint32_t _rested{};
            std::atomic<RE::hknpWorld*> _world{};
            std::array<std::atomic<std::uint32_t>, loose_weapon_collision_policy::kMaxNativeBodies> _bodies{};
            std::atomic<unsigned> _count{};
            std::atomic<std::uint64_t> _generation{ 1 };

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
        const Pool _pool;
        std::array<Slot, loose_weapon_collision_policy::kMaxWeapons> _slots;
        // Settled stateful weapons retain their exact generated shapes outside
        // the active preparation/contact pool. Released on pickup, model/world
        // replacement or consumer withdrawal, never evicted onto default shapes.
        std::array<std::unique_ptr<State>, 256> _parked;

        struct DroppedHandoff
        {
            RE::ObjectRefHandle handle;
            DroppedWeaponCollision collision;
            float waitedSeconds{};
        };

        // One entry per concurrent equipped drop handoff.
        std::array<DroppedHandoff, 4> _dropped;
        std::atomic<std::uint32_t> _watchMask{};
        std::size_t _nextPreparationSlot{};
    };
}
