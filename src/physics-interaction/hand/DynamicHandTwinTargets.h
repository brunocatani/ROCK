#pragma once

#include "physics-interaction/hand/HandColliderTypes.h"

#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <cstdint>

namespace rock::dynamic_hand_twin
{
    inline constexpr std::size_t kForearmSegmentCountPerHand = 1;

    /*
     * Per-frame publication from HandBoneColliderSet for the stage A dynamic
     * hand twins: the EXACT role frames and dimensions the keyframed palm
     * anchor and fingertip (Tip segment) colliders are driven with, so the
     * dynamic proxies mirror the production collider conventions by
     * construction instead of re-deriving hand geometry. Main-thread only:
     * written by HandBoneColliderSet::update and consumed by
     * DynamicHandCollisionRuntime::updateFrame in the same frame loop.
     */
    struct TwinSlotFrame
    {
        bool valid = false;
        RE::NiTransform target{};
        float length = 0.0f;
        float radius = 0.0f;
        float convexRadius = 0.0f;
        // Palm/fingertip proxies map 1:1 to the hand target. The merged
        // forearm proxy publishes its pose-derived IK leverage correction.
        float handTargetResponseScale = 1.0f;
    };

    struct TwinTargets
    {
        TwinSlotFrame palm{};
        std::array<TwinSlotFrame, hand_collider_semantics::kHandFingerCount> fingertips{};
        std::uint64_t updateCounter = 0;
    };

    /*
     * Main-thread publication from BodyBoneColliderSet. These are the exact
     * merged ForeArm1->Hand frame derived from all three tuned production
     * keyframed segments (ForeArm1->2, ForeArm2->3, and ForeArm3->Hand).
     * DynamicHandCollisionRuntime consumes it after BodyBoneColliderSet::update
     * in the same frame, so the forearm twin never reads dynamic Havok bodies
     * back into intent.
     */
    struct ForearmTwinTargets
    {
        std::array<TwinSlotFrame, kForearmSegmentCountPerHand> right{};
        std::array<TwinSlotFrame, kForearmSegmentCountPerHand> left{};
        std::uint64_t updateCounter = 0;

        [[nodiscard]] const std::array<TwinSlotFrame, kForearmSegmentCountPerHand>& forHand(bool isLeft) const
        {
            return isLeft ? left : right;
        }
    };
}
