#pragma once

#include "physics-interaction/consume/ImmersiveAidPolicy.h"
#include "physics-interaction/consume/MouthConsumeDetector.h"

namespace RE { class TESBoundObject; class hknpWorld; }
namespace rock { class Hand; class BodyBoneColliderSet; }

namespace rock::immersive_aid
{
    [[nodiscard]] Injector classify(RE::TESBoundObject* base);

    [[nodiscard]] mouth_consume::Decision evaluate(RE::hknpWorld* world,
        const BodyBoneColliderSet& bodyColliders, const Hand& hand, bool isLeft,
        std::uint64_t collisionGeneration, const game_frame_timing_policy::GameFrameTiming& timing,
        mouth_consume::RuntimeState& state);
}
