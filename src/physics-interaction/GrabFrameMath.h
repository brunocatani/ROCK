#pragma once

/*
 * ROCK's calibrated grab pivot is authored in semantic handspace, but the live
 * grab has two different consumers: the rendered FRIK hand, and the keyframed
 * physics hand body. HIGGS keeps one coherent palm-to-object relation; ROCK needs
 * the same coherence while preserving its separate visual and collision frames.
 * These helpers keep that split explicit so pivot A, pivot B, reverse visual
 * alignment, and debug measurements cannot silently drift onto different frames.
 */

#include "TransformMath.h"

#include <utility>

namespace frik::rock::grab_frame_math
{
    template <class Transform>
    struct SplitGrabFrame
    {
        using Vector = decltype(std::declval<Transform>().translate);

        Transform shiftedObjectWorld{};
        Transform rawHandSpace{};
        Transform constraintHandSpace{};
        Transform handBodyToRawHandAtGrab{};
        Transform desiredBodyHandBodySpace{};
        Vector pivotAHandBodyLocal{};
        Vector pivotBBodyLocal{};
    };

    template <class Transform, class Vector>
    inline Transform shiftObjectToAlignSurfaceWithPivot(Transform objectWorld, const Vector& grabPivotWorld, const Vector& surfacePointWorld)
    {
        objectWorld.translate.x += grabPivotWorld.x - surfacePointWorld.x;
        objectWorld.translate.y += grabPivotWorld.y - surfacePointWorld.y;
        objectWorld.translate.z += grabPivotWorld.z - surfacePointWorld.z;
        return objectWorld;
    }

    template <class Transform>
    inline Transform objectInFrameSpace(const Transform& frameWorld, const Transform& objectWorld)
    {
        return transform_math::composeTransforms(transform_math::invertTransform(frameWorld), objectWorld);
    }

    template <class Transform>
    inline Transform desiredBodyInHandBodySpace(const Transform& constraintHandSpace, const Transform& bodyLocal)
    {
        return transform_math::composeTransforms(constraintHandSpace, bodyLocal);
    }

    template <class Transform, class Vector>
    inline Vector computePivotAHandBodyLocal(const Transform& handBodyWorld, const Vector& grabPivotWorld)
    {
        return transform_math::worldPointToLocal(handBodyWorld, grabPivotWorld);
    }

    template <class Transform, class Vector>
    inline Vector computePivotBBodyLocal(const Transform& desiredBodyTransformHandBodySpace, const Vector& pivotAHandBodyLocal)
    {
        return transform_math::localPointToWorld(transform_math::invertTransform(desiredBodyTransformHandBodySpace), pivotAHandBodyLocal);
    }

    template <class Transform>
    inline Transform computeVisualHandFromHeldNode(const Transform& heldNodeWorld, const Transform& rawHandSpace)
    {
        return transform_math::composeTransforms(heldNodeWorld, transform_math::invertTransform(rawHandSpace));
    }

    template <class Transform, class Vector>
    inline SplitGrabFrame<Transform> buildSplitGrabFrame(const Transform& rawHandWorld,
        const Transform& handBodyWorld,
        const Transform& objectNodeWorld,
        const Transform& bodyLocal,
        const Vector& grabPivotWorld,
        const Vector& surfacePointWorld)
    {
        SplitGrabFrame<Transform> result{};
        result.shiftedObjectWorld = shiftObjectToAlignSurfaceWithPivot(objectNodeWorld, grabPivotWorld, surfacePointWorld);
        result.rawHandSpace = objectInFrameSpace(rawHandWorld, result.shiftedObjectWorld);
        result.constraintHandSpace = objectInFrameSpace(handBodyWorld, result.shiftedObjectWorld);
        result.handBodyToRawHandAtGrab = objectInFrameSpace(handBodyWorld, rawHandWorld);
        result.desiredBodyHandBodySpace = desiredBodyInHandBodySpace(result.constraintHandSpace, bodyLocal);
        result.pivotAHandBodyLocal = computePivotAHandBodyLocal(handBodyWorld, grabPivotWorld);
        result.pivotBBodyLocal = computePivotBBodyLocal(result.desiredBodyHandBodySpace, result.pivotAHandBodyLocal);
        return result;
    }

    template <class Transform, class Vector>
    inline SplitGrabFrame<Transform> buildSplitGrabFrameFromDesiredObject(const Transform& rawHandWorld,
        const Transform& handBodyWorld,
        const Transform& desiredObjectWorld,
        const Transform& bodyLocal,
        const Vector& grabPivotWorld)
    {
        SplitGrabFrame<Transform> result{};
        result.shiftedObjectWorld = desiredObjectWorld;
        result.rawHandSpace = objectInFrameSpace(rawHandWorld, desiredObjectWorld);
        result.constraintHandSpace = objectInFrameSpace(handBodyWorld, desiredObjectWorld);
        result.handBodyToRawHandAtGrab = objectInFrameSpace(handBodyWorld, rawHandWorld);
        result.desiredBodyHandBodySpace = desiredBodyInHandBodySpace(result.constraintHandSpace, bodyLocal);
        result.pivotAHandBodyLocal = computePivotAHandBodyLocal(handBodyWorld, grabPivotWorld);
        result.pivotBBodyLocal = computePivotBBodyLocal(result.desiredBodyHandBodySpace, result.pivotAHandBodyLocal);
        return result;
    }
}
