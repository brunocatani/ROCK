#pragma once
#include "Boundary.h"
namespace rock::api::boundary {
    inline Sample sampleFromSnapshot(const provider::RockProviderFrameSnapshot& frame) noexcept {
        return {frame.frameIndex, frame.stateSequence, frame.worldGeneration,
            frame.skeletonGeneration, frame.providerGeneration, frame.collisionGeneration};
    }
}
