#pragma once

#include "physics-interaction/weapon/BladePenetrationPolicy.h"
#include "physics-interaction/weapon/WeaponSurfaceSupport.h"
#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/HavokPairCollisionFilter.h"

namespace rock
{
    class WeaponCollision;
    class PhysicsCallbackQuiescenceGate;
    struct PhysicsFrameContext;

    // Structural state is written only by the game thread with callbacks
    // quiesced. Physics drives the private guide anchor, never the NPC body.
    class BladePenetrationRuntime
    {
    public:
        bool update(const PhysicsFrameContext& frame, const WeaponCollision& collision,
            RE::NiNode* weaponNode, BethesdaPhysicsBody& weaponBody,
            const RE::NiPoint3& centerWeaponLocal, std::uint64_t generation,
            PhysicsCallbackQuiescenceGate* gate, RE::NiTransform& requestedWeapon,
            bool surfaceSupportActive);
        void recordContact(RE::hknpWorld* world, std::uint32_t weaponBody,
            std::uint32_t targetBody, std::uint32_t targetLayer, const RE::NiPoint3& point);
        bool prePhysics(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        void retire(RE::hknpWorld* world, void* bhkWorld);
        bool active() const { return _constraintId != 0x7FFF'FFFFu; }
        const RE::NiTransform& presentation() const { return _presentation; }

    private:
        bool resolveBlade(const WeaponCollision& collision, RE::NiNode* weaponNode,
            std::uint64_t generation, blade_penetration::Blade& blade);
        bool targetWorld(RE::hknpWorld* world, RE::NiTransform& result) const;
        void release(RE::hknpWorld* world, const char* reason);

        std::uint64_t _profileGeneration{ 0 };
        std::uint32_t _sourceBodyId{ 0x7FFF'FFFFu };
        RE::NiPoint3 _sourceTip{};
        blade_penetration::Blade _blade{};
        RE::NiPoint3 _centerWeaponLocal{};
        float _weaponScale{ 1.0f };
        RE::hknpWorld* _world{ nullptr };
        std::uint32_t _weaponBodyId{ 0x7FFF'FFFFu };
        weapon_surface_support::ContactChannel _contacts;
        weapon_surface_support::Contact _contact{};
        BethesdaPhysicsBody _anchor;
        HavokPairCollisionLeaseSet _pairs;
        std::uint32_t _constraintId{ 0x7FFF'FFFFu };
        RE::NiTransform _entryWeaponInTarget{};
        RE::NiTransform _presentation{};
        RE::NiTransform _lastTargetWorld{};
        std::atomic<bool> _physicsFailed{ false };
        std::uint64_t _lastContactReport{ 0 };
        bool _acquisitionUnavailable{ false };
    };
}
