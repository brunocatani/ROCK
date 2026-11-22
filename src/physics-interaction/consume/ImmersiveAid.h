#pragma once

#include "physics-interaction/consume/ImmersiveAidPolicy.h"
#include "physics-interaction/consume/MouthConsumeDetector.h"

namespace RE
{
    class TESBoundObject;
    class hknpWorld;
}

namespace rock
{
    class Hand;
    class BodyBoneColliderSet;
}

namespace rock::immersive_aid
{
    [[nodiscard]] Injector classify(RE::TESBoundObject* base);

    // The body zones whose colliders the mesh of the loose object hand holds
    // overlaps (bit 1 << zone; never the holding hand's own zone). available
    // is false without a held contact mesh or ready body colliders.
    // stopAtFirst ends at the first touched zone.
    struct HeldMeshBodyTouch
    {
        bool available = false;
        std::uint32_t zoneMask = 0;
        body_zone::BodyZoneKind firstZone = body_zone::BodyZoneKind::Unknown;
        RE::NiPoint3 firstBodyCenterGame{};
    };

    [[nodiscard]] HeldMeshBodyTouch findHeldMeshBodyTouch(RE::hknpWorld* world,
        const BodyBoneColliderSet& bodyColliders, const Hand& hand, bool isLeft, bool stopAtFirst);

    [[nodiscard]] mouth_consume::Decision evaluate(RE::hknpWorld* world, const BodyBoneColliderSet& bodyColliders,
        const Hand& hand, bool isLeft, std::uint64_t collisionGeneration,
        const game_frame_timing_policy::GameFrameTiming& timing, mouth_consume::RuntimeState& state);
}
