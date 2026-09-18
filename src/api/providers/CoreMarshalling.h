#pragma once
#include <ROCK/Core.h>
#include "CommonMarshalling.h"
namespace rock::api::boundary {
    inline void convert(rock::api::core::RegistrationV1& out, const rock::provider::RockProviderConsumerRegistrationV1& in) {
        for (std::size_t i=0; i<std::size(out.modName); ++i) out.modName[i] = in.modName[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderConsumerRegistrationV1& out, const rock::api::core::RegistrationV1& in) {
        for (std::size_t i=0; i<std::size(out.modName); ++i) out.modName[i] = in.modName[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::core::OwnerV1& out, const rock::provider::RockProviderConsumerHandleV1& in) {
        out.ownerToken = static_cast<decltype(out.ownerToken)>(in.ownerToken);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderConsumerHandleV1& out, const rock::api::core::OwnerV1& in) {
        out.ownerToken = static_cast<decltype(out.ownerToken)>(in.ownerToken);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::core::SnapshotV1& out, const rock::provider::RockProviderFrameSnapshot& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.frikSkeletonReady = static_cast<decltype(out.frikSkeletonReady)>(in.frikSkeletonReady);
        out.menuBlocking = static_cast<decltype(out.menuBlocking)>(in.menuBlocking);
        out.configBlocking = static_cast<decltype(out.configBlocking)>(in.configBlocking);
        out.providerReady = static_cast<decltype(out.providerReady)>(in.providerReady);
        out.lifecycleFlags = static_cast<decltype(out.lifecycleFlags)>(in.lifecycleFlags);
        out.lastLifecycleReason = static_cast<decltype(out.lastLifecycleReason)>(in.lastLifecycleReason);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.stableFrameCount = static_cast<decltype(out.stableFrameCount)>(in.stableFrameCount);
        out.deltaSeconds = static_cast<decltype(out.deltaSeconds)>(in.deltaSeconds);
        out.stateSequence = static_cast<decltype(out.stateSequence)>(in.stateSequence);
    }
    inline void convert(rock::provider::RockProviderFrameSnapshot& out, const rock::api::core::SnapshotV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.frikSkeletonReady = static_cast<decltype(out.frikSkeletonReady)>(in.frikSkeletonReady);
        out.menuBlocking = static_cast<decltype(out.menuBlocking)>(in.menuBlocking);
        out.configBlocking = static_cast<decltype(out.configBlocking)>(in.configBlocking);
        out.providerReady = static_cast<decltype(out.providerReady)>(in.providerReady);
        out.lifecycleFlags = static_cast<decltype(out.lifecycleFlags)>(in.lifecycleFlags);
        out.lastLifecycleReason = static_cast<decltype(out.lastLifecycleReason)>(in.lastLifecycleReason);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.stableFrameCount = static_cast<decltype(out.stableFrameCount)>(in.stableFrameCount);
        out.deltaSeconds = static_cast<decltype(out.deltaSeconds)>(in.deltaSeconds);
        out.stateSequence = static_cast<decltype(out.stateSequence)>(in.stateSequence);
    }
    inline void convert(rock::api::core::AnimationPhaseContextV1& out, const rock::provider::RockProviderAnimationPhaseContextV1& in) {
        out.phase = static_cast<decltype(out.phase)>(in.phase);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.deltaSeconds = static_cast<decltype(out.deltaSeconds)>(in.deltaSeconds);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderAnimationPhaseContextV1& out, const rock::api::core::AnimationPhaseContextV1& in) {
        out.phase = static_cast<decltype(out.phase)>(in.phase);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.deltaSeconds = static_cast<decltype(out.deltaSeconds)>(in.deltaSeconds);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
}
