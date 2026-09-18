#pragma once
#include <ROCK/PlayerController.h>
#include "CommonMarshalling.h"
namespace rock::api::boundary {
    inline void convert(rock::api::playercontroller::PlayerControllerStateV1& out, const rock::provider::RockProviderPlayerControllerStateV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.implementation = static_cast<decltype(out.implementation)>(in.implementation);
        out.supportState = static_cast<decltype(out.supportState)>(in.supportState);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        convert(out.positionGame, in.positionGame);
        convert(out.velocityGame, in.velocityGame);
        convert(out.supportNormalGame, in.supportNormalGame);
        out.radiusGame = static_cast<decltype(out.radiusGame)>(in.radiusGame);
        out.heightGame = static_cast<decltype(out.heightGame)>(in.heightGame);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderPlayerControllerStateV1& out, const rock::api::playercontroller::PlayerControllerStateV1& in) {
        out.frameIndex = static_cast<decltype(out.frameIndex)>(in.frameIndex);
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.implementation = static_cast<decltype(out.implementation)>(in.implementation);
        out.supportState = static_cast<decltype(out.supportState)>(in.supportState);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        convert(out.positionGame, in.positionGame);
        convert(out.velocityGame, in.velocityGame);
        convert(out.supportNormalGame, in.supportNormalGame);
        out.radiusGame = static_cast<decltype(out.radiusGame)>(in.radiusGame);
        out.heightGame = static_cast<decltype(out.heightGame)>(in.heightGame);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::playercontroller::PlayerControllerJumpRequestV1& out, const rock::provider::RockProviderPlayerControllerJumpRequestV1& in) {
        out.heightGameUnits = static_cast<decltype(out.heightGameUnits)>(in.heightGameUnits);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderPlayerControllerJumpRequestV1& out, const rock::api::playercontroller::PlayerControllerJumpRequestV1& in) {
        out.heightGameUnits = static_cast<decltype(out.heightGameUnits)>(in.heightGameUnits);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
}
