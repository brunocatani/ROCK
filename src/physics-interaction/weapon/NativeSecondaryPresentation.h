#pragma once

#include "physics-interaction/weapon/AuthoredWeaponGripPose.h"
#include "RE/NetImmerse/NiAVObject.h"

namespace RE { class PlayerCharacter; class NiNode; }

namespace rock::native_secondary_presentation
{
    // Game-frame owner only. The native index-1 model is placed in the hand
    // that holds it, using its own authored firing seat; the primary weapon
    // keeps its existing owner.
    void beginFrame() noexcept;
    void update() noexcept;
    void afterWeaponPosition() noexcept;
    void afterWorldFinal() noexcept;
    // The return value belongs to this exact native call, including nested calls.
    // Only the secondary offset's arm pass leaves the left branch in world space.
    [[nodiscard]] bool beforeNativeArm(const RE::PlayerCharacter* player, const RE::NiNode* weapon, const RE::NiNode* offset) noexcept;
    void afterNativeArm(const RE::PlayerCharacter* player, bool secondaryPass) noexcept;
    void endFrame(bool skeletonUpdated) noexcept;

    // The latest placement of the second weapon (this or the previous
    // frame), for a drop from its hand:
    // the firing grip pose (authored seat and fingers, physical placement)
    // and the placed weapon world. `model` is the borrowed native model.
    struct DropPose
    {
        bool left = true;
        AuthoredWeaponGripPose pose{};
        RE::NiTransform weaponWorld{};
        RE::NiAVObject* model = nullptr;
    };
    // Null when `out` is valid; otherwise the reason it is not.
    [[nodiscard]] const char* tryGetDropPose(DropPose& out) noexcept;

    // The index-1 model this frame placed in its hand (borrowed, frame
    // owner only), or null before this frame's placement or without one.
    [[nodiscard]] RE::NiAVObject* presentedModel() noexcept;

    // The collision-free world pose of presentedModel() this frame: the
    // rendered pose with this frame's collision correction removed. This is
    // the second weapon's dynamic-proxy intent.
    [[nodiscard]] bool tryGetModelIntentWorld(RE::NiTransform& out) noexcept;

    /*
     * Weapon collision correction for the second weapon, called once per
     * placement on the frame thread with the model's collision-free world pose.
     * Returns true with the resolved pose to present; the gun and the hand
     * holding it then move together by that correction.
     */
    using CollisionResolver = bool (*)(void* context, const RE::NiTransform& modelIntentWorld,
        RE::NiTransform& resolvedModelWorld);
    void setCollisionResolver(void* context, CollisionResolver resolver) noexcept;
}
