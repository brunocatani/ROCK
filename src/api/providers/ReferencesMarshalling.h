#pragma once
#include <ROCK/References.h>
#include "CommonMarshalling.h"
namespace rock::api::boundary {
    inline void convert(rock::api::references::ReferenceQueryV1& out, const rock::provider::RockProviderReferenceQueryV1& in) {
        out.referenceFormId = static_cast<decltype(out.referenceFormId)>(in.referenceFormId);
        out.referenceNativeHandle = static_cast<decltype(out.referenceNativeHandle)>(in.referenceNativeHandle);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.furnitureMarkerIndex = static_cast<decltype(out.furnitureMarkerIndex)>(in.furnitureMarkerIndex);
    }
    inline void convert(rock::provider::RockProviderReferenceQueryV1& out, const rock::api::references::ReferenceQueryV1& in) {
        out.referenceFormId = static_cast<decltype(out.referenceFormId)>(in.referenceFormId);
        out.referenceNativeHandle = static_cast<decltype(out.referenceNativeHandle)>(in.referenceNativeHandle);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.furnitureMarkerIndex = static_cast<decltype(out.furnitureMarkerIndex)>(in.furnitureMarkerIndex);
    }
    inline void convert(rock::api::references::ReferenceInteractionV1& out, const rock::provider::RockProviderReferenceInteractionV1& in) {
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.referenceFormId = static_cast<decltype(out.referenceFormId)>(in.referenceFormId);
        out.referenceNativeHandle = static_cast<decltype(out.referenceNativeHandle)>(in.referenceNativeHandle);
        out.baseFormId = static_cast<decltype(out.baseFormId)>(in.baseFormId);
        out.baseFormType = static_cast<decltype(out.baseFormType)>(in.baseFormType);
        out.activationBlocked = static_cast<decltype(out.activationBlocked)>(in.activationBlocked);
        out.openState = static_cast<decltype(out.openState)>(in.openState);
        out.furnitureInUse = static_cast<decltype(out.furnitureInUse)>(in.furnitureInUse);
        out.furnitureInUseIncludingReservations = static_cast<decltype(out.furnitureInUseIncludingReservations)>(in.furnitureInUseIncludingReservations);
        out.furnitureMarkerIndex = static_cast<decltype(out.furnitureMarkerIndex)>(in.furnitureMarkerIndex);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
    }
    inline void convert(rock::provider::RockProviderReferenceInteractionV1& out, const rock::api::references::ReferenceInteractionV1& in) {
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.referenceFormId = static_cast<decltype(out.referenceFormId)>(in.referenceFormId);
        out.referenceNativeHandle = static_cast<decltype(out.referenceNativeHandle)>(in.referenceNativeHandle);
        out.baseFormId = static_cast<decltype(out.baseFormId)>(in.baseFormId);
        out.baseFormType = static_cast<decltype(out.baseFormType)>(in.baseFormType);
        out.activationBlocked = static_cast<decltype(out.activationBlocked)>(in.activationBlocked);
        out.openState = static_cast<decltype(out.openState)>(in.openState);
        out.furnitureInUse = static_cast<decltype(out.furnitureInUse)>(in.furnitureInUse);
        out.furnitureInUseIncludingReservations = static_cast<decltype(out.furnitureInUseIncludingReservations)>(in.furnitureInUseIncludingReservations);
        out.furnitureMarkerIndex = static_cast<decltype(out.furnitureMarkerIndex)>(in.furnitureMarkerIndex);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
    }
    inline void convert(rock::api::references::PowerArmorPointPoseV1& out, const rock::provider::RockProviderPowerArmorPointPoseV1& in) {
        out.point = static_cast<decltype(out.point)>(in.point);
        out.valid = static_cast<decltype(out.valid)>(in.valid);
        convert(out.world, in.world);
        convert(out.frameLocal, in.frameLocal);
    }
    inline void convert(rock::provider::RockProviderPowerArmorPointPoseV1& out, const rock::api::references::PowerArmorPointPoseV1& in) {
        out.point = static_cast<decltype(out.point)>(in.point);
        out.valid = static_cast<decltype(out.valid)>(in.valid);
        convert(out.world, in.world);
        convert(out.frameLocal, in.frameLocal);
    }
    inline void convert(rock::api::references::PowerArmorTargetV1& out, const rock::provider::RockProviderPowerArmorTargetV1& in) {
        convert(out.touchedReference, in.touchedReference);
        convert(out.frameReference, in.frameReference);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.actorFormId = static_cast<decltype(out.actorFormId)>(in.actorFormId);
        for (std::size_t i=0; i<std::size(out.points); ++i) convert(out.points[i], in.points[i]);
    }
    inline void convert(rock::provider::RockProviderPowerArmorTargetV1& out, const rock::api::references::PowerArmorTargetV1& in) {
        convert(out.touchedReference, in.touchedReference);
        convert(out.frameReference, in.frameReference);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.actorFormId = static_cast<decltype(out.actorFormId)>(in.actorFormId);
        for (std::size_t i=0; i<std::size(out.points); ++i) convert(out.points[i], in.points[i]);
    }
}
