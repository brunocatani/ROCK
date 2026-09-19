#pragma once
#include <ROCK/Input.h>
#include "CommonMarshalling.h"
namespace rock::api::boundary {
    inline void convert(rock::api::input::HandInputSuppressionRequestV1& out, const rock::provider::RockProviderHandInputSuppressionRequestV1& in) {
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.chordButtonsLow); ++i) out.chordButtonsLow[i] = in.chordButtonsLow[i];
        for (std::size_t i=0; i<std::size(out.chordButtonsHigh); ++i) out.chordButtonsHigh[i] = in.chordButtonsHigh[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderHandInputSuppressionRequestV1& out, const rock::api::input::HandInputSuppressionRequestV1& in) {
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.chordButtonsLow); ++i) out.chordButtonsLow[i] = in.chordButtonsLow[i];
        for (std::size_t i=0; i<std::size(out.chordButtonsHigh); ++i) out.chordButtonsHigh[i] = in.chordButtonsHigh[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::input::RawWandButtonStateV1& out, const rock::provider::RockProviderRawWandButtonStateV1& in) {
        out.available = static_cast<decltype(out.available)>(in.available);
        out.held = static_cast<decltype(out.held)>(in.held);
        out.sampleSequence = static_cast<decltype(out.sampleSequence)>(in.sampleSequence);
        out.sampleAgeMilliseconds = static_cast<decltype(out.sampleAgeMilliseconds)>(in.sampleAgeMilliseconds);
        out.availabilityReason = static_cast<decltype(out.availabilityReason)>(in.availabilityReason);
    }
    inline void convert(rock::provider::RockProviderRawWandButtonStateV1& out, const rock::api::input::RawWandButtonStateV1& in) {
        out.available = static_cast<decltype(out.available)>(in.available);
        out.held = static_cast<decltype(out.held)>(in.held);
        out.sampleSequence = static_cast<decltype(out.sampleSequence)>(in.sampleSequence);
        out.sampleAgeMilliseconds = static_cast<decltype(out.sampleAgeMilliseconds)>(in.sampleAgeMilliseconds);
        out.availabilityReason = static_cast<decltype(out.availabilityReason)>(in.availabilityReason);
    }
    inline void convert(rock::api::input::LogicalInputActionStateV1& out, const rock::provider::RockProviderLogicalInputActionStateV1& in) {
        out.action = static_cast<decltype(out.action)>(in.action);
        out.available = static_cast<decltype(out.available)>(in.available);
        out.held = static_cast<decltype(out.held)>(in.held);
        out.availabilityReason = static_cast<decltype(out.availabilityReason)>(in.availabilityReason);
        out.sampleSequence = static_cast<decltype(out.sampleSequence)>(in.sampleSequence);
        out.pressSequence = static_cast<decltype(out.pressSequence)>(in.pressSequence);
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.sampleAgeMilliseconds = static_cast<decltype(out.sampleAgeMilliseconds)>(in.sampleAgeMilliseconds);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderLogicalInputActionStateV1& out, const rock::api::input::LogicalInputActionStateV1& in) {
        out.action = static_cast<decltype(out.action)>(in.action);
        out.available = static_cast<decltype(out.available)>(in.available);
        out.held = static_cast<decltype(out.held)>(in.held);
        out.availabilityReason = static_cast<decltype(out.availabilityReason)>(in.availabilityReason);
        out.sampleSequence = static_cast<decltype(out.sampleSequence)>(in.sampleSequence);
        out.pressSequence = static_cast<decltype(out.pressSequence)>(in.pressSequence);
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.sampleAgeMilliseconds = static_cast<decltype(out.sampleAgeMilliseconds)>(in.sampleAgeMilliseconds);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::input::HandInputSuppressionStateV1& out, const rock::provider::RockProviderHandInputSuppressionStateV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.callerFlags = static_cast<decltype(out.callerFlags)>(in.callerFlags);
        out.effectiveFlags = static_cast<decltype(out.effectiveFlags)>(in.effectiveFlags);
        out.callerLeaseActive = static_cast<decltype(out.callerLeaseActive)>(in.callerLeaseActive);
        out.callerExpiresAfterFrame = static_cast<decltype(out.callerExpiresAfterFrame)>(in.callerExpiresAfterFrame);
        out.callerRemainingFrames = static_cast<decltype(out.callerRemainingFrames)>(in.callerRemainingFrames);
        out.lastInvalidationReason = static_cast<decltype(out.lastInvalidationReason)>(in.lastInvalidationReason);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderHandInputSuppressionStateV1& out, const rock::api::input::HandInputSuppressionStateV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.callerFlags = static_cast<decltype(out.callerFlags)>(in.callerFlags);
        out.effectiveFlags = static_cast<decltype(out.effectiveFlags)>(in.effectiveFlags);
        out.callerLeaseActive = static_cast<decltype(out.callerLeaseActive)>(in.callerLeaseActive);
        out.callerExpiresAfterFrame = static_cast<decltype(out.callerExpiresAfterFrame)>(in.callerExpiresAfterFrame);
        out.callerRemainingFrames = static_cast<decltype(out.callerRemainingFrames)>(in.callerRemainingFrames);
        out.lastInvalidationReason = static_cast<decltype(out.lastInvalidationReason)>(in.lastInvalidationReason);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
}
