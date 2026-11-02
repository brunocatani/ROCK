#pragma once

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiSmartPointer.h"

#include <array>
#include <cstdint>

#include "RE/NetImmerse/NiAVObject.h"
#include "physics-interaction/weapon/AuthoredWeaponGripPose.h"
#include "physics-interaction/weapon/HoldingGripMath.h"

namespace RE
{
    class PlayerCharacter;
    class NiNode;
}

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
    [[nodiscard]] bool beforeNativeArm(const RE::PlayerCharacter* player, const RE::NiNode* weapon,
        const RE::NiNode* offset) noexcept;
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
        // The hand world this placement published.
        RE::NiTransform handWorld{};
        RE::NiAVObject* model = nullptr;
    };

    // Null when `out` is valid; otherwise the reason it is not.
    [[nodiscard]] const char* tryGetDropPose(DropPose& out) noexcept;

    /*
     * Each frame the native update first returns the index-1 attachment to
     * its own biped pose (in the left hand, a rotated carry); this
     * presentation re-places it later in the frame. A caller that takes the
     * model before that (promotion) re-applies the last completed placement
     * first, so the model is captured as it was rendered. Frame owner only.
     */
    [[nodiscard]] bool restoreLastPlacement() noexcept;

    /*
     * A loose weapon equipped into index 1 loses its 3D inside ActivateRef;
     * the equipped model is placed here no earlier than the next frame, and
     * a first-ever weapon waits for its idle harvest. Keeps the loose model
     * in the hand, with the hand and fingers the loose grab presented, until
     * this presentation places the equipped model; the swap is one frame.
     * Frame owner only. Takes the reference retained before ActivateRef.
     */
    void bridgeIncoming(RE::NiPointer<RE::NiAVObject> model, std::uint32_t weaponFormID, bool left,
        const RE::NiTransform& handInModel, const std::array<RE::NiTransform, 15>& fingerLocals,
        std::uint16_t fingerMask) noexcept;
    // Transition seam diagnostic: the incoming bridge's model, or null.
    [[nodiscard]] const RE::NiAVObject* incomingModelForTrace(bool& left) noexcept;

    // The index-1 model this frame placed in its hand (borrowed, frame
    // owner only), or null before this frame's placement or without one.
    [[nodiscard]] RE::NiAVObject* presentedModel() noexcept;

    // The presented model this frame is seated and scaled by a player FRIK
    // weapon offset. Frame owner only.
    [[nodiscard]] bool usesUserWeaponOffset() noexcept;

    // This frame's firing seat of the second weapon in its attachment's
    // frame, and that attachment as presented. Frame owner only.
    [[nodiscard]] bool tryGetFiringSeat(RE::NiTransform& outHandWeaponLocal, RE::NiTransform& outWeaponWorld) noexcept;

    // The collision-free world pose of presentedModel() this frame: the
    // controller placement before collision correction. This is the second
    // weapon's dynamic-proxy intent.
    [[nodiscard]] bool tryGetModelIntentWorld(RE::NiTransform& out) noexcept;

    /*
     * The second weapon's model world and projectile-node world for this
     * frame, for provider readers that run before this frame's placement
     * (frame callbacks). A firearm carries the last completed placement (at
     * most one frame old) to the current weapon wand, which its placement
     * follows; a melee weapon keeps that placement. outProjectileValid is
     * false when the model has no projectile node. Frame owner only.
     */
    [[nodiscard]] bool tryGetAimFrame(RE::NiTransform& outModelWorld, RE::NiTransform& outProjectileWorld,
        bool& outProjectileValid) noexcept;

    /*
     * Weapon collision correction for the second weapon, called once per
     * placement on the frame thread with the model's collision-free world pose.
     * Returns true with the resolved pose to present; the gun and the hand
     * holding it then move together by that correction.
     */
    // The second weapon's geometry mirror plane, from its generated
    // collision (0 until generated). The left seat mirrors across it.
    void setMirrorOffsets(holding_grip_math::MirrorOffsets offsets) noexcept;

    using CollisionResolver = bool (*)(void* context, const RE::NiTransform& modelIntentWorld,
        RE::NiTransform& resolvedModelWorld);
    void setCollisionResolver(void* context, CollisionResolver resolver) noexcept;
}
