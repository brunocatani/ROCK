#pragma once

#include "physics-interaction/hand/HandColliderTypes.h"

#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::dynamic_hand_twin
{
    inline constexpr std::size_t kPalmSlot = 0;
    inline constexpr std::size_t kBodiesPerHand = 1 + hand_collider_semantics::kHandFingerCount;

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
    };

    struct TwinTargets
    {
        TwinSlotFrame palm{};
        std::array<TwinSlotFrame, hand_collider_semantics::kHandFingerCount> fingertips{};
        // Changes only when collider construction inputs (power armor or
        // tuning overrides) change. Per-frame poses use updateCounter instead.
        std::uint64_t geometrySignature = 0;
        std::uint64_t updateCounter = 0;
    };

    [[nodiscard]] inline const TwinSlotFrame* frameForBodyIndex(
        const TwinTargets& targets,
        std::size_t bodyIndex)
    {
        if (bodyIndex == kPalmSlot) {
            return &targets.palm;
        }
        const std::size_t fingerIndex = bodyIndex - 1;
        return fingerIndex < targets.fingertips.size() ? &targets.fingertips[fingerIndex] : nullptr;
    }
}
