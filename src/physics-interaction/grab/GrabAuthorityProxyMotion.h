#pragma once

/*
 * Hidden grab authority proxies are keyframed solver anchors, not contact
 * evidence. Linear telemetry can be derived from the queued palm-target delta,
 * but angular velocity must stay on FO4VR's native hard-keyframe boundary. The
 * removed manual matrix-column angular helper was one of the stale convention
 * paths that could reintroduce world-direction-dependent rotation.
 */

#include "physics-interaction/native/HavokPhysicsTiming.h"

#include "RE/NetImmerse/NiTransform.h"

namespace rock::grab_authority_proxy_motion
{
    inline void computeLinearVelocityHavok(
        const RE::NiTransform& previous,
        const RE::NiTransform& current,
        float deltaSeconds,
        float gameToHavokScale,
        float outVelocity[4])
    {
        outVelocity[0] = 0.0f;
        outVelocity[1] = 0.0f;
        outVelocity[2] = 0.0f;
        outVelocity[3] = 0.0f;
        if (!havok_physics_timing::isUsableDelta(deltaSeconds)) {
            return;
        }

        const float scale = gameToHavokScale / deltaSeconds;
        outVelocity[0] = (current.translate.x - previous.translate.x) * scale;
        outVelocity[1] = (current.translate.y - previous.translate.y) * scale;
        outVelocity[2] = (current.translate.z - previous.translate.z) * scale;
    }
}
