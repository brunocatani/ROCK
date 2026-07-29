#pragma once

/*
 * Dynamic grab needs a hand-owned solver anchor that can be driven in the
 * same physics phase as the custom finite-force constraint. This proxy is not
 * a hand collider and is not gameplay contact evidence; it is a hidden,
 * no-contact, keyframed body whose only job is to give the grab constraint a
 * stable body A while the held object remains dynamic body B.
 */

#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/grab/GrabAuthorityProxyMotion.h"
#include "physics-interaction/native/HavokConvexShapeBuilder.h"
#include "physics-interaction/native/PhysicsUtils.h"

#include "RE/Havok/hknpShape.h"
#include "RE/NetImmerse/NiTransform.h"

#include <cmath>
#include <cstdint>
#include <vector>

namespace rock::grab_authority_proxy
{
    inline constexpr float kProxyConvexRadiusHavok = 0.001f;
    inline constexpr float kProxyHullHalfExtentHavok = 0.0125f;
    inline constexpr std::uint32_t kProxyCollisionGroup = 0x000B;

    inline std::vector<RE::NiPoint3> makeProxyHullPoints()
    {
        const float s = kProxyHullHalfExtentHavok;
        return {
            RE::NiPoint3{ s, s, s },
            RE::NiPoint3{ -s, -s, s },
            RE::NiPoint3{ -s, s, -s },
            RE::NiPoint3{ s, -s, -s },
        };
    }

    inline RE::hknpShape* buildProxyShape()
    {
        return havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(makeProxyHullPoints(), kProxyConvexRadiusHavok);
    }

    inline std::uint32_t noContactFilterInfo()
    {
        return (kProxyCollisionGroup << 16) |
               (collision_layer_policy::FO4_LAYER_NONCOLLIDABLE & collision_layer_policy::FO4_LAYER_FILTER_MASK) |
               collision_suppression_registry::kSuppressionNoCollideBit;
    }

    inline bool hasNoContactFilterInfo(std::uint32_t filterInfo)
    {
        const std::uint32_t layer = filterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
        return layer == collision_layer_policy::FO4_LAYER_NONCOLLIDABLE &&
               (filterInfo & collision_suppression_registry::kSuppressionNoCollideBit) != 0;
    }

    inline const char* filterPolicyName()
    {
        return "nonCollidable+bit14";
    }

    inline RE::hkTransformf makeHavokTransform(const RE::NiTransform& target)
    {
        RE::hkTransformf transform{};
        transform.rotation = niRotToHkTransformRotation(target.rotate);
        transform.translation = RE::NiPoint4(
            target.translate.x * gameToHavokScale(),
            target.translate.y * gameToHavokScale(),
            target.translate.z * gameToHavokScale(),
            0.0f);
        return transform;
    }

    inline void computeLinearVelocityHavok(const RE::NiTransform& previous, const RE::NiTransform& current, float deltaSeconds, float outVelocity[4])
    {
        grab_authority_proxy_motion::computeLinearVelocityHavok(previous, current, deltaSeconds, gameToHavokScale(), outVelocity);
    }

}
