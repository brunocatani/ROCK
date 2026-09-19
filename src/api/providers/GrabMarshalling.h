#pragma once
#include <ROCK/Grab.h>
#include "CommonMarshalling.h"
namespace rock::api::boundary {
    inline void convert(rock::api::grab::ForceGrabRequestV1& out, const rock::provider::RockProviderForceGrabRequestV1& in) {
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.targetFormId = static_cast<decltype(out.targetFormId)>(in.targetFormId);
        out.targetBodyId = static_cast<decltype(out.targetBodyId)>(in.targetBodyId);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.maxDistanceGame = static_cast<decltype(out.maxDistanceGame)>(in.maxDistanceGame);
        for (std::size_t i=0; i<std::size(out.preferredGrabPointGame); ++i) out.preferredGrabPointGame[i] = in.preferredGrabPointGame[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderForceGrabRequestV1& out, const rock::api::grab::ForceGrabRequestV1& in) {
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.targetFormId = static_cast<decltype(out.targetFormId)>(in.targetFormId);
        out.targetBodyId = static_cast<decltype(out.targetBodyId)>(in.targetBodyId);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.maxDistanceGame = static_cast<decltype(out.maxDistanceGame)>(in.maxDistanceGame);
        for (std::size_t i=0; i<std::size(out.preferredGrabPointGame); ++i) out.preferredGrabPointGame[i] = in.preferredGrabPointGame[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::grab::ForceReleaseRequestV1& out, const rock::provider::RockProviderForceReleaseRequestV1& in) {
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.targetFormId = static_cast<decltype(out.targetFormId)>(in.targetFormId);
        out.targetBodyId = static_cast<decltype(out.targetBodyId)>(in.targetBodyId);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.linearVelocityHavok); ++i) out.linearVelocityHavok[i] = in.linearVelocityHavok[i];
        for (std::size_t i=0; i<std::size(out.angularVelocityRadiansPerSecond); ++i) out.angularVelocityRadiansPerSecond[i] = in.angularVelocityRadiansPerSecond[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderForceReleaseRequestV1& out, const rock::api::grab::ForceReleaseRequestV1& in) {
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.targetFormId = static_cast<decltype(out.targetFormId)>(in.targetFormId);
        out.targetBodyId = static_cast<decltype(out.targetBodyId)>(in.targetBodyId);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.linearVelocityHavok); ++i) out.linearVelocityHavok[i] = in.linearVelocityHavok[i];
        for (std::size_t i=0; i<std::size(out.angularVelocityRadiansPerSecond); ++i) out.angularVelocityRadiansPerSecond[i] = in.angularVelocityRadiansPerSecond[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::grab::ThrownDropRequestV1& out, const rock::provider::RockProviderThrownDropRequestV1& in) {
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.targetFormId = static_cast<decltype(out.targetFormId)>(in.targetFormId);
        out.targetBodyId = static_cast<decltype(out.targetBodyId)>(in.targetBodyId);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        for (std::size_t i=0; i<std::size(out.linearVelocityHavok); ++i) out.linearVelocityHavok[i] = in.linearVelocityHavok[i];
        for (std::size_t i=0; i<std::size(out.angularVelocityRadiansPerSecond); ++i) out.angularVelocityRadiansPerSecond[i] = in.angularVelocityRadiansPerSecond[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderThrownDropRequestV1& out, const rock::api::grab::ThrownDropRequestV1& in) {
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.targetFormId = static_cast<decltype(out.targetFormId)>(in.targetFormId);
        out.targetBodyId = static_cast<decltype(out.targetBodyId)>(in.targetBodyId);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        for (std::size_t i=0; i<std::size(out.linearVelocityHavok); ++i) out.linearVelocityHavok[i] = in.linearVelocityHavok[i];
        for (std::size_t i=0; i<std::size(out.angularVelocityRadiansPerSecond); ++i) out.angularVelocityRadiansPerSecond[i] = in.angularVelocityRadiansPerSecond[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::grab::InteractionCommandResultV1& out, const rock::provider::RockProviderInteractionCommandResultV1& in) {
        out.ownerToken = static_cast<decltype(out.ownerToken)>(in.ownerToken);
        out.commandId = static_cast<decltype(out.commandId)>(in.commandId);
        out.kind = static_cast<decltype(out.kind)>(in.kind);
        out.state = static_cast<decltype(out.state)>(in.state);
        out.failure = static_cast<decltype(out.failure)>(in.failure);
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.targetFormId = static_cast<decltype(out.targetFormId)>(in.targetFormId);
        out.targetBodyId = static_cast<decltype(out.targetBodyId)>(in.targetBodyId);
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.stage = static_cast<decltype(out.stage)>(in.stage);
        out.failureStage = static_cast<decltype(out.failureStage)>(in.failureStage);
        out.acceptedFrame = static_cast<decltype(out.acceptedFrame)>(in.acceptedFrame);
        out.committedFrame = static_cast<decltype(out.committedFrame)>(in.committedFrame);
        out.appliedFrame = static_cast<decltype(out.appliedFrame)>(in.appliedFrame);
        out.reserved = static_cast<decltype(out.reserved)>(in.reserved);
    }
    inline void convert(rock::provider::RockProviderInteractionCommandResultV1& out, const rock::api::grab::InteractionCommandResultV1& in) {
        out.ownerToken = static_cast<decltype(out.ownerToken)>(in.ownerToken);
        out.commandId = static_cast<decltype(out.commandId)>(in.commandId);
        out.kind = static_cast<decltype(out.kind)>(in.kind);
        out.state = static_cast<decltype(out.state)>(in.state);
        out.failure = static_cast<decltype(out.failure)>(in.failure);
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.targetFormId = static_cast<decltype(out.targetFormId)>(in.targetFormId);
        out.targetBodyId = static_cast<decltype(out.targetBodyId)>(in.targetBodyId);
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.stage = static_cast<decltype(out.stage)>(in.stage);
        out.failureStage = static_cast<decltype(out.failureStage)>(in.failureStage);
        out.acceptedFrame = static_cast<decltype(out.acceptedFrame)>(in.acceptedFrame);
        out.committedFrame = static_cast<decltype(out.committedFrame)>(in.committedFrame);
        out.appliedFrame = static_cast<decltype(out.appliedFrame)>(in.appliedFrame);
        out.reserved = static_cast<decltype(out.reserved)>(in.reserved);
    }
    inline void convert(rock::api::grab::HandInteractionStateV1& out, const rock::provider::RockProviderHandInteractionStateV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.phase = static_cast<decltype(out.phase)>(in.phase);
        out.targetKind = static_cast<decltype(out.targetKind)>(in.targetKind);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.reservedTargetIdentity = static_cast<decltype(out.reservedTargetIdentity)>(in.reservedTargetIdentity);
        out.targetFormId = static_cast<decltype(out.targetFormId)>(in.targetFormId);
        out.primaryBodyId = static_cast<decltype(out.primaryBodyId)>(in.primaryBodyId);
        out.heldBodyCount = static_cast<decltype(out.heldBodyCount)>(in.heldBodyCount);
        for (std::size_t i=0; i<std::size(out.heldBodyIds); ++i) out.heldBodyIds[i] = in.heldBodyIds[i];
        out.stateSequence = static_cast<decltype(out.stateSequence)>(in.stateSequence);
        out.targetSequence = static_cast<decltype(out.targetSequence)>(in.targetSequence);
        out.gripSequence = static_cast<decltype(out.gripSequence)>(in.gripSequence);
        out.releaseSequence = static_cast<decltype(out.releaseSequence)>(in.releaseSequence);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.collisionGeneration = static_cast<decltype(out.collisionGeneration)>(in.collisionGeneration);
        convert(out.surfaceAnchorGame, in.surfaceAnchorGame);
        out.surfaceGripMode = static_cast<decltype(out.surfaceGripMode)>(in.surfaceGripMode);
    }
    inline void convert(rock::provider::RockProviderHandInteractionStateV1& out, const rock::api::grab::HandInteractionStateV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.phase = static_cast<decltype(out.phase)>(in.phase);
        out.targetKind = static_cast<decltype(out.targetKind)>(in.targetKind);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.reservedTargetIdentity = static_cast<decltype(out.reservedTargetIdentity)>(in.reservedTargetIdentity);
        out.targetFormId = static_cast<decltype(out.targetFormId)>(in.targetFormId);
        out.primaryBodyId = static_cast<decltype(out.primaryBodyId)>(in.primaryBodyId);
        out.heldBodyCount = static_cast<decltype(out.heldBodyCount)>(in.heldBodyCount);
        for (std::size_t i=0; i<std::size(out.heldBodyIds); ++i) out.heldBodyIds[i] = in.heldBodyIds[i];
        out.stateSequence = static_cast<decltype(out.stateSequence)>(in.stateSequence);
        out.targetSequence = static_cast<decltype(out.targetSequence)>(in.targetSequence);
        out.gripSequence = static_cast<decltype(out.gripSequence)>(in.gripSequence);
        out.releaseSequence = static_cast<decltype(out.releaseSequence)>(in.releaseSequence);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.collisionGeneration = static_cast<decltype(out.collisionGeneration)>(in.collisionGeneration);
        convert(out.surfaceAnchorGame, in.surfaceAnchorGame);
        out.surfaceGripMode = static_cast<decltype(out.surfaceGripMode)>(in.surfaceGripMode);
    }
    inline void convert(rock::api::grab::OffhandReservationRequestV1& out, const rock::provider::RockProviderOffhandReservationRequestV1& in) {
        out.reservation = static_cast<decltype(out.reservation)>(in.reservation);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderOffhandReservationRequestV1& out, const rock::api::grab::OffhandReservationRequestV1& in) {
        out.reservation = static_cast<decltype(out.reservation)>(in.reservation);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::grab::OffhandReservationStateV1& out, const rock::provider::RockProviderOffhandReservationStateV1& in) {
        out.reservation = static_cast<decltype(out.reservation)>(in.reservation);
        out.active = static_cast<decltype(out.active)>(in.active);
        out.ownerToken = static_cast<decltype(out.ownerToken)>(in.ownerToken);
        out.expiresAfterFrame = static_cast<decltype(out.expiresAfterFrame)>(in.expiresAfterFrame);
        out.remainingFrames = static_cast<decltype(out.remainingFrames)>(in.remainingFrames);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderOffhandReservationStateV1& out, const rock::api::grab::OffhandReservationStateV1& in) {
        out.reservation = static_cast<decltype(out.reservation)>(in.reservation);
        out.active = static_cast<decltype(out.active)>(in.active);
        out.ownerToken = static_cast<decltype(out.ownerToken)>(in.ownerToken);
        out.expiresAfterFrame = static_cast<decltype(out.expiresAfterFrame)>(in.expiresAfterFrame);
        out.remainingFrames = static_cast<decltype(out.remainingFrames)>(in.remainingFrames);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::grab::PowerArmorGrabRequestV1& out, const rock::provider::RockProviderPowerArmorGrabRequestV1& in) {
        out.target.referenceFormId = in.target.referenceFormId;
        out.target.referenceNativeHandle = in.target.referenceNativeHandle;
        out.target.worldGeneration = in.target.worldGeneration;
        out.target.skeletonGeneration = in.target.skeletonGeneration;
        out.target.providerGeneration = in.target.providerGeneration;
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.point = static_cast<decltype(out.point)>(in.point);
        out.maxDistanceGame = static_cast<decltype(out.maxDistanceGame)>(in.maxDistanceGame);
    }
    inline void convert(rock::provider::RockProviderPowerArmorGrabRequestV1& out, const rock::api::grab::PowerArmorGrabRequestV1& in) {
        out.target.referenceFormId = in.target.referenceFormId;
        out.target.referenceNativeHandle = in.target.referenceNativeHandle;
        out.target.worldGeneration = in.target.worldGeneration;
        out.target.skeletonGeneration = in.target.skeletonGeneration;
        out.target.providerGeneration = in.target.providerGeneration;
        out.hand = static_cast<decltype(out.hand)>(in.hand);
        out.point = static_cast<decltype(out.point)>(in.point);
        out.maxDistanceGame = static_cast<decltype(out.maxDistanceGame)>(in.maxDistanceGame);
    }
    inline void convert(rock::api::grab::HandTargetDetailsV1& out, const rock::provider::RockProviderHandTargetDetailsV1& in) {
        convert(out.handState, in.handState);
        out.referenceFormId = in.reference.referenceFormId;
        out.referenceNativeHandle = in.reference.referenceNativeHandle;
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.collisionLayer = static_cast<decltype(out.collisionLayer)>(in.collisionLayer);
        convert(out.anchorGame, in.anchorGame);
        convert(out.normalGame, in.normalGame);
        out.powerArmorPoint = static_cast<decltype(out.powerArmorPoint)>(in.powerArmorPoint);
        out.sourceTriangleIndex = static_cast<decltype(out.sourceTriangleIndex)>(in.sourceTriangleIndex);
        for (std::size_t i=0; i<std::size(out.collisionNodeName); ++i) out.collisionNodeName[i] = in.collisionNodeName[i];
        for (std::size_t i=0; i<std::size(out.meshPartName); ++i) out.meshPartName[i] = in.meshPartName[i];
    }
    inline void convert(rock::provider::RockProviderHandTargetDetailsV1& out, const rock::api::grab::HandTargetDetailsV1& in) {
        convert(out.handState, in.handState);
        out.reference.referenceFormId = in.referenceFormId;
        out.reference.referenceNativeHandle = in.referenceNativeHandle;
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.collisionLayer = static_cast<decltype(out.collisionLayer)>(in.collisionLayer);
        convert(out.anchorGame, in.anchorGame);
        convert(out.normalGame, in.normalGame);
        out.powerArmorPoint = static_cast<decltype(out.powerArmorPoint)>(in.powerArmorPoint);
        out.sourceTriangleIndex = static_cast<decltype(out.sourceTriangleIndex)>(in.sourceTriangleIndex);
        for (std::size_t i=0; i<std::size(out.collisionNodeName); ++i) out.collisionNodeName[i] = in.collisionNodeName[i];
        for (std::size_t i=0; i<std::size(out.meshPartName); ++i) out.meshPartName[i] = in.meshPartName[i];
    }
}
