#pragma once

#include "RockUtils.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "rock_support/Fo4VrRuntime.h"

#include <cmath>

namespace rock::hand_grab_detail
{
    inline bool isUsableGrabVisualTransform(const RE::NiTransform& transform)
    {
        if (!std::isfinite(transform.translate.x) || !std::isfinite(transform.translate.y) ||
            !std::isfinite(transform.translate.z) || !std::isfinite(transform.scale) ||
            std::abs(transform.scale) <= 0.0001f) {
            return false;
        }
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(transform.rotate.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

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
