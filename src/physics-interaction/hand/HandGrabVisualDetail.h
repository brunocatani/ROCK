#pragma once

#include "RockUtils.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock::hand_grab_detail
{
    inline constexpr const char* kGrabExternalHandTag = "ROCK_GrabVisual";
    inline constexpr int kGrabExternalHandPriority = 90;
    inline constexpr const char* kGrabReturnHandTag = "ROCK_GrabReturn";
    inline constexpr int kGrabReturnHandPriority = 85;

    inline bool applyGrabExternalHandWorldTransform(bool isLeft, const RE::NiTransform& adjustedHandTransform)
    {
        return frik_visual_authority::applyExternalHandWorldTransform(
            kGrabExternalHandTag,
            handFromBool(isLeft),
            adjustedHandTransform,
            kGrabExternalHandPriority);
    }

    inline void clearGrabExternalHandWorldTransform(bool isLeft)
    {
        (void)frik_visual_authority::clearExternalHandWorldTransform(kGrabExternalHandTag, handFromBool(isLeft));
    }

    inline bool applyGrabReturnHandWorldTransform(bool isLeft, const RE::NiTransform& handTransform)
    {
        return frik_visual_authority::applyExternalHandWorldTransform(
            kGrabReturnHandTag,
            handFromBool(isLeft),
            handTransform,
            kGrabReturnHandPriority);
    }

    inline void clearGrabReturnHandWorldTransform(bool isLeft)
    {
        (void)frik_visual_authority::clearExternalHandWorldTransform(kGrabReturnHandTag, handFromBool(isLeft));
    }

    /*
     * Both callers must use the same scene-graph write mechanism. Keep this as
     * one inline definition on purpose - both the pre-FRIK refresh and the
     * held-update publish must write the scene graph identically.
     */
    inline bool applyHeldVisualNodeWorldTransform(RE::NiAVObject* node, const RE::NiTransform& desiredWorld)
    {
        if (!node) {
            return false;
        }
        if (auto* parent = node->parent) {
            node->local = transform_math::composeTransforms(
                transform_math::invertTransform(parent->world),
                desiredWorld);
        } else {
            node->local = desiredWorld;
        }
        f4vr::updateTransformsDown(node, true);
        return true;
    }
}
