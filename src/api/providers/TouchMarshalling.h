#pragma once
#include <ROCK/Touch.h>
#include "CommonMarshalling.h"
namespace rock::api::boundary {
    inline void convert(rock::api::touch::TouchGrabTargetV1& out, const rock::provider::RockProviderTouchGrabTargetV1& in) {
        out.targetId = static_cast<decltype(out.targetId)>(in.targetId);
        out.targetGeneration = static_cast<decltype(out.targetGeneration)>(in.targetGeneration);
        out.kind = static_cast<decltype(out.kind)>(in.kind);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.referenceFormId = static_cast<decltype(out.referenceFormId)>(in.referenceFormId);
        out.referenceNativeHandle = static_cast<decltype(out.referenceNativeHandle)>(in.referenceNativeHandle);
        out.allowedLayerMask = static_cast<decltype(out.allowedLayerMask)>(in.allowedLayerMask);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        convert(out.pivotWorldGame, in.pivotWorldGame);
        convert(out.axisWorldGame, in.axisWorldGame);
        out.minimumCoordinate = static_cast<decltype(out.minimumCoordinate)>(in.minimumCoordinate);
        out.maximumCoordinate = static_cast<decltype(out.maximumCoordinate)>(in.maximumCoordinate);
        out.currentCoordinate = static_cast<decltype(out.currentCoordinate)>(in.currentCoordinate);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderTouchGrabTargetV1& out, const rock::api::touch::TouchGrabTargetV1& in) {
        out.targetId = static_cast<decltype(out.targetId)>(in.targetId);
        out.targetGeneration = static_cast<decltype(out.targetGeneration)>(in.targetGeneration);
        out.kind = static_cast<decltype(out.kind)>(in.kind);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.referenceFormId = static_cast<decltype(out.referenceFormId)>(in.referenceFormId);
        out.referenceNativeHandle = static_cast<decltype(out.referenceNativeHandle)>(in.referenceNativeHandle);
        out.allowedLayerMask = static_cast<decltype(out.allowedLayerMask)>(in.allowedLayerMask);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        convert(out.pivotWorldGame, in.pivotWorldGame);
        convert(out.axisWorldGame, in.axisWorldGame);
        out.minimumCoordinate = static_cast<decltype(out.minimumCoordinate)>(in.minimumCoordinate);
        out.maximumCoordinate = static_cast<decltype(out.maximumCoordinate)>(in.maximumCoordinate);
        out.currentCoordinate = static_cast<decltype(out.currentCoordinate)>(in.currentCoordinate);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::touch::TouchGrabStateV1& out, const rock::provider::RockProviderTouchGrabStateV1& in) {
        out.targetId = static_cast<decltype(out.targetId)>(in.targetId);
        out.targetGeneration = static_cast<decltype(out.targetGeneration)>(in.targetGeneration);
        out.kind = static_cast<decltype(out.kind)>(in.kind);
        out.phase = static_cast<decltype(out.phase)>(in.phase);
        out.releaseReason = static_cast<decltype(out.releaseReason)>(in.releaseReason);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.referenceFormId = static_cast<decltype(out.referenceFormId)>(in.referenceFormId);
        out.referenceNativeHandle = static_cast<decltype(out.referenceNativeHandle)>(in.referenceNativeHandle);
        out.activeHandMask = static_cast<decltype(out.activeHandMask)>(in.activeHandMask);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.surfaceGripMode = static_cast<decltype(out.surfaceGripMode)>(in.surfaceGripMode);
        out.currentCoordinate = static_cast<decltype(out.currentCoordinate)>(in.currentCoordinate);
        out.coordinateVelocity = static_cast<decltype(out.coordinateVelocity)>(in.coordinateVelocity);
        convert(out.contactPointGame, in.contactPointGame);
        convert(out.contactNormalGame, in.contactNormalGame);
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.sequence = static_cast<decltype(out.sequence)>(in.sequence);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.collisionGeneration = static_cast<decltype(out.collisionGeneration)>(in.collisionGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderTouchGrabStateV1& out, const rock::api::touch::TouchGrabStateV1& in) {
        out.targetId = static_cast<decltype(out.targetId)>(in.targetId);
        out.targetGeneration = static_cast<decltype(out.targetGeneration)>(in.targetGeneration);
        out.kind = static_cast<decltype(out.kind)>(in.kind);
        out.phase = static_cast<decltype(out.phase)>(in.phase);
        out.releaseReason = static_cast<decltype(out.releaseReason)>(in.releaseReason);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.referenceFormId = static_cast<decltype(out.referenceFormId)>(in.referenceFormId);
        out.referenceNativeHandle = static_cast<decltype(out.referenceNativeHandle)>(in.referenceNativeHandle);
        out.activeHandMask = static_cast<decltype(out.activeHandMask)>(in.activeHandMask);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.surfaceGripMode = static_cast<decltype(out.surfaceGripMode)>(in.surfaceGripMode);
        out.currentCoordinate = static_cast<decltype(out.currentCoordinate)>(in.currentCoordinate);
        out.coordinateVelocity = static_cast<decltype(out.coordinateVelocity)>(in.coordinateVelocity);
        convert(out.contactPointGame, in.contactPointGame);
        convert(out.contactNormalGame, in.contactNormalGame);
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.sequence = static_cast<decltype(out.sequence)>(in.sequence);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.collisionGeneration = static_cast<decltype(out.collisionGeneration)>(in.collisionGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
}
