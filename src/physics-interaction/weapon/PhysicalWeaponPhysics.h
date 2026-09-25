#pragma once

#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/weapon/DynamicWeaponCollision.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/collision/PushContact.h"

namespace rock
{
    class Hand;
    struct PhysicsFrameContext;

    // Same generated hull bank and dynamic contact solver as equipped weapons.
    // The old reference bodies retain their grip constraints, but surrender
    // collision only while this complete replacement is ready.
    class PhysicalWeaponPhysics
    {
    public:
        WeaponCollision collision;
        DynamicWeaponCollisionRuntime dynamic;
        push_assist::ContactChannel push;
        unsigned heldHandsAtomic() const noexcept { return _heldHands.load(std::memory_order_acquire); }
        void setPhysicsCallbackGate(PhysicsCallbackQuiescenceGate* gate);
        bool update(const PhysicsFrameContext& frame, RE::TESObjectREFR* reference,
            RE::EquippedWeaponData* data, Hand& owner, Hand* support);
        bool clear(bool worldAvailable);
        bool ready() const noexcept { return _ready; }
        void finishParts(const PhysicsFrameContext& frame);
    private:
        void retireProxy(const char* reason);
        PhysicsCallbackQuiescenceGate* _gate{};
        RE::hknpWorld* _world{};
        RE::bhkWorld* _bhk{};
        RE::NiPointer<RE::TESObjectREFR> _reference{};
        RE::NiPointer<RE::NiNode> _root{};
        std::uint64_t _reportedGeneration{};
        collision_suppression_registry::SuppressionLeaseSet<128> _nativeBodies{
            collision_suppression_registry::CollisionSuppressionOwner::PhysicalWeaponSession};
        bool _ready{}, _presenterLeft{};
        std::atomic<unsigned> _heldHands{0};
    };
}
