#pragma once
#include <ROCK/Animation.h>
#include "CommonMarshalling.h"
namespace rock::api::boundary {
    inline void convert(rock::api::animation::NativeAnimationAuthorityRequestV1& out, const rock::provider::RockProviderNativeAnimationAuthorityRequestV1& in) {
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderNativeAnimationAuthorityRequestV1& out, const rock::api::animation::NativeAnimationAuthorityRequestV1& in) {
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::animation::NativeAnimationAuthorityStateV1& out, const rock::provider::RockProviderNativeAnimationAuthorityStateV1& in) {
        out.activeFlags = static_cast<decltype(out.activeFlags)>(in.activeFlags);
        out.statusFlags = static_cast<decltype(out.statusFlags)>(in.statusFlags);
        out.activeOwnerCount = static_cast<decltype(out.activeOwnerCount)>(in.activeOwnerCount);
        out.capturedTransformCount = static_cast<decltype(out.capturedTransformCount)>(in.capturedTransformCount);
        out.captureSequence = static_cast<decltype(out.captureSequence)>(in.captureSequence);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderNativeAnimationAuthorityStateV1& out, const rock::api::animation::NativeAnimationAuthorityStateV1& in) {
        out.activeFlags = static_cast<decltype(out.activeFlags)>(in.activeFlags);
        out.statusFlags = static_cast<decltype(out.statusFlags)>(in.statusFlags);
        out.activeOwnerCount = static_cast<decltype(out.activeOwnerCount)>(in.activeOwnerCount);
        out.capturedTransformCount = static_cast<decltype(out.capturedTransformCount)>(in.capturedTransformCount);
        out.captureSequence = static_cast<decltype(out.captureSequence)>(in.captureSequence);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::animation::HandVisualAuthorityRequestV1& out, const rock::provider::RockProviderHandVisualAuthorityRequestV1& in) {
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.priority = static_cast<decltype(out.priority)>(in.priority);
        out.fingerLocalTransformMask = static_cast<decltype(out.fingerLocalTransformMask)>(in.fingerLocalTransformMask);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        convert(out.worldTransform, in.worldTransform);
        for (std::size_t i=0; i<std::size(out.fingerLocalTransforms); ++i) convert(out.fingerLocalTransforms[i], in.fingerLocalTransforms[i]);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderHandVisualAuthorityRequestV1& out, const rock::api::animation::HandVisualAuthorityRequestV1& in) {
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.priority = static_cast<decltype(out.priority)>(in.priority);
        out.fingerLocalTransformMask = static_cast<decltype(out.fingerLocalTransformMask)>(in.fingerLocalTransformMask);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        convert(out.worldTransform, in.worldTransform);
        for (std::size_t i=0; i<std::size(out.fingerLocalTransforms); ++i) convert(out.fingerLocalTransforms[i], in.fingerLocalTransforms[i]);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::animation::NativeAnimationRuntimePublicationV1& out, const rock::provider::RockProviderNativeAnimationRuntimePublicationV1& in) {
        out.statusFlags = static_cast<decltype(out.statusFlags)>(in.statusFlags);
        out.capturedTransformCount = static_cast<decltype(out.capturedTransformCount)>(in.capturedTransformCount);
        out.captureSequence = static_cast<decltype(out.captureSequence)>(in.captureSequence);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderNativeAnimationRuntimePublicationV1& out, const rock::api::animation::NativeAnimationRuntimePublicationV1& in) {
        out.statusFlags = static_cast<decltype(out.statusFlags)>(in.statusFlags);
        out.capturedTransformCount = static_cast<decltype(out.capturedTransformCount)>(in.capturedTransformCount);
        out.captureSequence = static_cast<decltype(out.captureSequence)>(in.captureSequence);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
}
