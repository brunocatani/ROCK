#pragma once
#include <ROCK/Hands.h>
#include "CommonMarshalling.h"
namespace rock::api::boundary {
    inline void convert(rock::api::hands::HandFrameV1& out, const rock::provider::RockProviderHandFrameV1& in) {
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        convert(out.transform, in.transform);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.stateSequence = static_cast<decltype(out.stateSequence)>(in.stateSequence);
        out.flags &= ~((1u<<2)|(1u<<3));
    }
    inline void convert(rock::provider::RockProviderHandFrameV1& out, const rock::api::hands::HandFrameV1& in) {
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        convert(out.transform, in.transform);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.stateSequence = static_cast<decltype(out.stateSequence)>(in.stateSequence);
    }
    inline void convert(rock::api::hands::PresentedHandPoseV1& out, const rock::provider::RockProviderPresentedHandPoseV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        convert(out.handWorld, in.handWorld);
        out.fingerLocalTransformMask = static_cast<decltype(out.fingerLocalTransformMask)>(in.fingerLocalTransformMask);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        for (std::size_t i=0; i<std::size(out.fingerLocalTransforms); ++i) convert(out.fingerLocalTransforms[i], in.fingerLocalTransforms[i]);
        out.presentationSequence = static_cast<decltype(out.presentationSequence)>(in.presentationSequence);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderPresentedHandPoseV1& out, const rock::api::hands::PresentedHandPoseV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        convert(out.handWorld, in.handWorld);
        out.fingerLocalTransformMask = static_cast<decltype(out.fingerLocalTransformMask)>(in.fingerLocalTransformMask);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        for (std::size_t i=0; i<std::size(out.fingerLocalTransforms); ++i) convert(out.fingerLocalTransforms[i], in.fingerLocalTransforms[i]);
        out.presentationSequence = static_cast<decltype(out.presentationSequence)>(in.presentationSequence);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
}
