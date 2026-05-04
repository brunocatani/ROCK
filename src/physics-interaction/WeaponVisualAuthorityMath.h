#pragma once

#include "TransformMath.h"

#include <array>

namespace frik::rock::weapon_visual_authority_math
{
    /*
     * ROCK-owned equipped weapon authority must write the same final visual
     * weapon frame that later drives generated weapon collision bodies. Keeping
     * world-target to parent-local conversion as pure math avoids duplicating
     * FRIK-style node write conventions across two-handed grip, one-handed mesh
     * grip, and debug verification paths. Two-handed support also needs a
     * HIGGS-style locked hand frame: controllers guide the weapon solve, while
     * visible hands are recomposed from stored weapon-local frames so the mesh
     * contact point cannot slide along the gun.
     */

    enum class TwoHandedExternalAuthorityStep
    {
        ApplyWeaponVisual,
        PublishHandPose,
        ApplyLockedHandVisual
    };

    enum class LockedHandRole
    {
        Primary,
        Support
    };

    inline constexpr std::array<TwoHandedExternalAuthorityStep, 3> twoHandedExternalAuthorityOrder()
    {
        /*
         * The order is part of the cross-mod contract with FRIK. The weapon must
         * be in its final ROCK-owned frame before hand targets are composed, and
         * the hand pose must be published before FRIK applies/finalizes the
         * external wrist target so right-hand fingers inherit the pivoted wrist.
         */
        return {
            TwoHandedExternalAuthorityStep::ApplyWeaponVisual,
            TwoHandedExternalAuthorityStep::PublishHandPose,
            TwoHandedExternalAuthorityStep::ApplyLockedHandVisual
        };
    }

    inline constexpr int authorityOrderIndex(const TwoHandedExternalAuthorityStep step)
    {
        const auto order = twoHandedExternalAuthorityOrder();
        for (int index = 0; index < static_cast<int>(order.size()); ++index) {
            if (order[static_cast<std::size_t>(index)] == step) {
                return index;
            }
        }
        return -1;
    }

    inline constexpr bool handPosePrecedesLockedHandAuthority()
    {
        return authorityOrderIndex(TwoHandedExternalAuthorityStep::PublishHandPose) <
               authorityOrderIndex(TwoHandedExternalAuthorityStep::ApplyLockedHandVisual);
    }

    inline constexpr bool weaponVisualPrecedesLockedHandAuthority()
    {
        return authorityOrderIndex(TwoHandedExternalAuthorityStep::ApplyWeaponVisual) <
               authorityOrderIndex(TwoHandedExternalAuthorityStep::ApplyLockedHandVisual);
    }

    inline constexpr bool shouldPublishTwoHandedGripPose(const LockedHandRole role)
    {
        /*
         * The primary/right hand already has FRIK's native weapon grip pose. ROCK
         * only needs to move that wrist with the final weapon frame. Publishing a
         * primary mesh pose would replace the user's tuned FRIK grip pose, while
         * the support hand still needs ROCK's mesh/contact pose.
         */
        return role == LockedHandRole::Support;
    }

    inline constexpr bool shouldUseMeshGripFrameRotationAtGrabStart(const LockedHandRole)
    {
        /*
         * Mesh semantics may choose the grip point, but locked hand authority
         * must preserve the wrist rotation already produced by FRIK/INI/native
         * weapon setup at the moment support grip starts. Replacing the right
         * wrist with the mesh-derived grip frame is what causes the visible
         * hand to snap up or down on grab activation.
         */
        return false;
    }

    inline constexpr bool shouldSelectMeshGripPointAtGrabStart(const LockedHandRole role)
    {
        /*
         * The support hand is allowed to select the mesh contact it actually
         * touched. The primary/right hand is not reselected from weapon part
         * names or bounds when support grip starts; the current FRIK-configured
         * hand-to-weapon relationship is already the correct primary grip.
         */
        return role == LockedHandRole::Support;
    }

    template <class Transform>
    inline Transform worldTargetToParentLocal(const Transform& parentWorld, const Transform& targetWorld)
    {
        return transform_math::composeTransforms(transform_math::invertTransform(parentWorld), targetWorld);
    }

    template <class Transform>
    inline Transform weaponLocalFrameToWorld(const Transform& weaponWorld, const Transform& weaponLocalFrame)
    {
        return transform_math::composeTransforms(weaponWorld, weaponLocalFrame);
    }
}
