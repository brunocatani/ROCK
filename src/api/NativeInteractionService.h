#pragma once
#include <ROCK/NativeInteractionV1_1.h>
#include "physics-interaction/native/HavokPhysicsTiming.h"
#include "RE/NetImmerse/NiTransform.h"

namespace RE { class hknpWorld; class NiNode; }
namespace rock {
    struct PhysicsFrameContext;
    class PhysicsCallbackQuiescenceGate;
    class BethesdaPhysicsBody;
    class WeaponCollision;
}
namespace rock::debug { struct BodyOverlayFrame; }
namespace rock::native_interaction_service {
    using Owner = api::OwnerToken;
    using RetireWeapon = void(*)(void*,void*);
    using RefreshLayers = void(*)(void*,RE::hknpWorld*);
    void initialize(PhysicsCallbackQuiescenceGate*,RetireWeapon,RefreshLayers,void*);
    void detach(PhysicsCallbackQuiescenceGate*);
    void update(const PhysicsFrameContext&);
    void preCollide(RE::hknpWorld*,const havok_physics_timing::PhysicsTimingSample&);
    void render(const PhysicsFrameContext&);
    void appendVisuals(debug::BodyOverlayFrame&);
    bool hasVisuals();
    bool hasVisualLabels();
    void clear(RE::hknpWorld*,void* bhkWorld);
    void clearOwner(Owner);
    api::Status canUnregisterOwner(Owner);
    std::uint64_t dynamicObstacleLayers();

    bool updateWeapon(const PhysicsFrameContext&,const WeaponCollision&,RE::NiNode*,BethesdaPhysicsBody&,
        const RE::NiPoint3& center,std::uint64_t generation,RE::NiTransform& requested,
        bool supportOwnsPose,const RE::NiPoint3* primaryGrip);
    bool weaponControlled();
    RE::NiTransform weaponPresentation();
    bool preCollideWeapon(RE::hknpWorld*,const havok_physics_timing::PhysicsTimingSample&);
    void weaponContact(RE::hknpWorld*,std::uint32_t weapon,std::uint32_t target,std::uint32_t layer,const RE::NiPoint3&,const RE::NiPoint3&,std::uint32_t pointCount);
    void retireWeapon(RE::hknpWorld*,void* bhkWorld);

    api::native_interaction::SelectionReply classify(const api::native_interaction::SelectionQuery&);
    bool validateSelection(const api::native_interaction::SelectionQuery&,const api::native_interaction::SelectionReply&);
    api::native_interaction::ProximityResult proximityTarget(std::uintptr_t reference,float radiusGame);
    std::uint32_t impactEffect(std::uintptr_t source,std::uintptr_t target,float damage);
    api::Status extract(const api::native_interaction::SelectionQuery&,
        const api::native_interaction::SelectionReply&,api::native_interaction::ExtractedReference&);
}
