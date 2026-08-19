#pragma once

#include "physics-interaction/hand/collision/HandColliderTypes.h"

#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <cstdint>

namespace rock::dynamic_hand_twin
{
    inline constexpr std::size_t kForearmSegmentCountPerHand = 1;

    /*
     * Per-frame publication from HandBoneColliderSet for the stage A dynamic
     * hand twins: the EXACT role frames and dimensions the keyframed palm
     * anchor and all 15 finger-segment colliders are driven with, so the
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
        // Palm/finger proxies map 1:1 to the hand target. The merged
        // forearm proxy publishes its pose-derived IK leverage correction.
        float handTargetResponseScale = 1.0f;
    };

    struct TwinTargets
    {
        TwinSlotFrame palm{};
        std::array<
            std::array<TwinSlotFrame, hand_collider_semantics::kHandFingerSegmentCount>,
            hand_collider_semantics::kHandFingerCount>
            fingers{};
        std::uint64_t updateCounter = 0;
        // Changes only when the owning keyframed collider set is rebuilt for
        // a real source/tuning/lifetime reason; never changes for live pose.
        std::uint64_t geometryGeneration = 0;
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
        std::uint64_t geometryGeneration = 0;

        [[nodiscard]] const std::array<TwinSlotFrame, kForearmSegmentCountPerHand>& forHand(bool isLeft) const
        {
            return isLeft ? left : right;
        }
    };

    /*
     * hFRIK changes live forearm node spacing while solving animation/IK.
     * That spacing is pose, not collider topology. Preserve the current rigid
     * target and response scale, but always publish the dimensions captured by
     * the owning BodyBoneColliderSet generation.
     */
    inline void applyCanonicalSlotDimensions(
        TwinSlotFrame& live,
        const TwinSlotFrame& canonical)
    {
        if (!live.valid) {
            return;
        }
        if (!canonical.valid || canonical.length <= 0.0f || canonical.radius <= 0.0f || canonical.convexRadius < 0.0f) {
            live.valid = false;
            return;
        }
        live.length = canonical.length;
        live.radius = canonical.radius;
        live.convexRadius = canonical.convexRadius;
    }

    inline void applyCanonicalHandDimensions(
        TwinTargets& liveTargets,
        const TwinTargets& canonicalTargets)
    {
        applyCanonicalSlotDimensions(liveTargets.palm, canonicalTargets.palm);
        for (std::size_t finger = 0; finger < liveTargets.fingers.size(); ++finger) {
            for (std::size_t segment = 0;
                 segment < liveTargets.fingers[finger].size();
                 ++segment) {
                applyCanonicalSlotDimensions(
                    liveTargets.fingers[finger][segment],
                    canonicalTargets.fingers[finger][segment]);
            }
        }
    }

    inline void applyCanonicalForearmDimensions(
        ForearmTwinTargets& liveTargets,
        const ForearmTwinTargets& canonicalTargets)
    {
        for (std::size_t index = 0; index < kForearmSegmentCountPerHand; ++index) {
            applyCanonicalSlotDimensions(liveTargets.right[index], canonicalTargets.right[index]);
            applyCanonicalSlotDimensions(liveTargets.left[index], canonicalTargets.left[index]);
        }
    }
}
