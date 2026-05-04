#pragma once

/*
 * Two-handed equipped-weapon support grip uses the HIGGS mesh probe to decide
 * whether the thumb should follow the alternate thumb curve. FRIK's existing
 * scalar pose API can bend the thumb, but it cannot switch the local rotation
 * basis the way HIGGS does. Keep the publication decision isolated here so the
 * runtime only sends local thumb transforms when the mesh solve selected that
 * alternate path and the provider can actually accept those transforms.
 */

namespace frik::rock::weapon_support_thumb_pose_policy
{
    [[nodiscard]] constexpr bool shouldPublishAlternateThumbLocalOverride(
        const bool solvedFingerPose,
        const bool usedAlternateThumbCurve,
        const bool hasLocalTransformApi)
    {
        return solvedFingerPose && usedAlternateThumbCurve && hasLocalTransformApi;
    }

    template <class Vector>
    [[nodiscard]] constexpr Vector predictThumbNodeWorldForGripFrame(
        const Vector& liveThumbNodeWorld,
        const Vector& currentSupportGripPivotWorld,
        const Vector& targetGripPointWorld)
    {
        const Vector supportHandOffset{
            targetGripPointWorld.x - currentSupportGripPivotWorld.x,
            targetGripPointWorld.y - currentSupportGripPivotWorld.y,
            targetGripPointWorld.z - currentSupportGripPivotWorld.z,
        };
        return Vector{
            liveThumbNodeWorld.x + supportHandOffset.x,
            liveThumbNodeWorld.y + supportHandOffset.y,
            liveThumbNodeWorld.z + supportHandOffset.z,
        };
    }

    template <class Vector>
    [[nodiscard]] constexpr Vector vectorToGripFromPredictedThumbNode(
        const Vector& liveThumbNodeWorld,
        const Vector& currentSupportGripPivotWorld,
        const Vector& targetGripPointWorld)
    {
        const Vector predictedNodeWorld =
            predictThumbNodeWorldForGripFrame(liveThumbNodeWorld, currentSupportGripPivotWorld, targetGripPointWorld);
        return Vector{
            targetGripPointWorld.x - predictedNodeWorld.x,
            targetGripPointWorld.y - predictedNodeWorld.y,
            targetGripPointWorld.z - predictedNodeWorld.z,
        };
    }
}
