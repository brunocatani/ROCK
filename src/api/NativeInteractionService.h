#pragma once
#include <ROCK/NativeInteraction.h>
#include "RE/NetImmerse/NiTransform.h"
#include "physics-interaction/native/HavokPhysicsTiming.h"

namespace RE
{
    class hknpWorld;
    class NiNode;
}

namespace rock
{
    struct PhysicsFrameContext;
    class PhysicsCallbackQuiescenceGate;
    class BethesdaPhysicsBody;
    class WeaponCollision;
}

namespace rock::debug
{
    struct BodyOverlayFrame;
}

namespace rock::native_interaction_service
{
    using Owner = api::OwnerToken;
    // Retires the dynamic weapon proxy of one weapon slot (context, slot, bhkWorld).
    using RetireWeapon = void (*)(void*, std::uint32_t, void*);
    using RefreshLayers = void (*)(void*, RE::hknpWorld*);
    void initialize(PhysicsCallbackQuiescenceGate*, RetireWeapon, RefreshLayers, void*);
    void detach(PhysicsCallbackQuiescenceGate*);
    void update(const PhysicsFrameContext&);
    void preCollide(RE::hknpWorld*, const havok_physics_timing::PhysicsTimingSample&);
    void clear(RE::hknpWorld*, void* bhkWorld);
    void clearOwner(Owner);
    api::Status canUnregisterOwner(Owner);
    std::uint64_t dynamicObstacleLayers();

    // Weapon slot 0 is the primary weapon, 1 the second equipped weapon.
    bool updateWeapon(std::uint32_t slot, const PhysicsFrameContext&, const WeaponCollision&, RE::NiNode*,
        BethesdaPhysicsBody&, const RE::NiPoint3& center, std::uint64_t generation, RE::NiTransform& requested,
        bool supportOwnsPose, const RE::NiPoint3* primaryGrip);
    bool weaponControlled(std::uint32_t slot);
    RE::NiTransform weaponPresentation(std::uint32_t slot);
    bool preCollideWeapon(std::uint32_t slot, RE::hknpWorld*, const havok_physics_timing::PhysicsTimingSample&);
    void weaponContact(std::uint32_t slot, RE::hknpWorld*, std::uint32_t weapon, std::uint32_t target,
        std::uint32_t layer, const RE::NiPoint3&, const RE::NiPoint3&, std::uint32_t pointCount);
    void retireWeapon(std::uint32_t slot, RE::hknpWorld*, void* bhkWorld);
    // Physics thread: one solved manifold of a dynamic hand body (point and
    // normal in game space, normal from the target toward the hand).
    void handContact(RE::hknpWorld*, bool left, api::native_interaction::HandPart part, std::uint32_t handBody,
        std::uint32_t target, std::uint32_t layer, const RE::NiPoint3& point, const RE::NiPoint3& normal,
        std::uint32_t pointCount);

    api::native_interaction::SelectionReply classify(const api::native_interaction::SelectionQuery&);
    bool validateSelection(const api::native_interaction::SelectionQuery&,
        const api::native_interaction::SelectionReply&);
    api::native_interaction::ProximityResult proximityTarget(std::uintptr_t reference, float radiusGame);
    std::uint32_t impactEffect(std::uintptr_t source, std::uintptr_t target, float damage);
    api::Status extract(const api::native_interaction::SelectionQuery&, const api::native_interaction::SelectionReply&,
        api::native_interaction::ExtractedReference&);
}
