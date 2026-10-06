#pragma once

#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/object/PhysicsBodyClassifier.h"
#include "RE/Havok/hknpBody.h"
#include <cmath>

namespace rock::weapon_rest_support
{
    // Shared by loose references and the retained equipped-drop compound.
    inline bool sameSurfacePose(const RE::NiTransform& a, const RE::NiTransform& b)
    {
        const auto delta = a.translate - b.translate;
        const auto distance = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
        if (!std::isfinite(distance) || distance > 0.1f) return false;
        for (unsigned r = 0; r < 3; ++r) for (unsigned c = 0; c < 3; ++c)
            if (!std::isfinite(a.rotate.entry[r][c]) || !std::isfinite(b.rotate.entry[r][c]) ||
                std::abs(a.rotate.entry[r][c] - b.rotate.entry[r][c]) > 0.002f) return false;
        return true;
    }

    inline bool parkingSurface(const havok_runtime::BodySnapshot& surface)
    {
        if (!surface.valid || !surface.body || !surface.body->shape ||
            (surface.collisionFilterInfo & collision_suppression_registry::kSuppressionNoCollideBit)) return false;
        const auto layer = surface.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
        if (!collision_layer_policy::isWorldSurfaceLayer(layer)) return false;
        const auto type = physics_body_classifier::motionTypeFromBodyFlags(surface.body->flags);
        using Motion = physics_body_classifier::BodyMotionType;
        return type == Motion::Static || type == Motion::Keyframed;
    }
}
