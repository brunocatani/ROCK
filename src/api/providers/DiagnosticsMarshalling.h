#pragma once
#include <ROCK/Diagnostics.h>
#include "CommonMarshalling.h"
namespace rock::api::boundary {
    inline void convert(rock::api::diagnostics::ColliderVisualizationRequestV1& out, const rock::provider::RockProviderColliderVisualizationRequestV1& in) {
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.partKind = static_cast<decltype(out.partKind)>(in.partKind);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderColliderVisualizationRequestV1& out, const rock::api::diagnostics::ColliderVisualizationRequestV1& in) {
        out.weaponGenerationKey = static_cast<decltype(out.weaponGenerationKey)>(in.weaponGenerationKey);
        out.bodyId = static_cast<decltype(out.bodyId)>(in.bodyId);
        out.partKind = static_cast<decltype(out.partKind)>(in.partKind);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::diagnostics::DebugOverlayLineV1& out, const rock::provider::RockProviderDebugOverlayLineV1& in) {
        for (std::size_t i=0; i<std::size(out.startGame); ++i) out.startGame[i] = in.startGame[i];
        for (std::size_t i=0; i<std::size(out.endGame); ++i) out.endGame[i] = in.endGame[i];
        for (std::size_t i=0; i<std::size(out.color); ++i) out.color[i] = in.color[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderDebugOverlayLineV1& out, const rock::api::diagnostics::DebugOverlayLineV1& in) {
        for (std::size_t i=0; i<std::size(out.startGame); ++i) out.startGame[i] = in.startGame[i];
        for (std::size_t i=0; i<std::size(out.endGame); ++i) out.endGame[i] = in.endGame[i];
        for (std::size_t i=0; i<std::size(out.color); ++i) out.color[i] = in.color[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::diagnostics::DebugOverlayTextV1& out, const rock::provider::RockProviderDebugOverlayTextV1& in) {
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        for (std::size_t i=0; i<std::size(out.text); ++i) out.text[i] = in.text[i];
        out.x = static_cast<decltype(out.x)>(in.x);
        out.y = static_cast<decltype(out.y)>(in.y);
        out.textSize = static_cast<decltype(out.textSize)>(in.textSize);
        for (std::size_t i=0; i<std::size(out.color); ++i) out.color[i] = in.color[i];
        for (std::size_t i=0; i<std::size(out.worldAnchorGame); ++i) out.worldAnchorGame[i] = in.worldAnchorGame[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderDebugOverlayTextV1& out, const rock::api::diagnostics::DebugOverlayTextV1& in) {
        out.flags = static_cast<decltype(out.flags)>(in.flags);
        out.reserved0 = static_cast<decltype(out.reserved0)>(in.reserved0);
        for (std::size_t i=0; i<std::size(out.text); ++i) out.text[i] = in.text[i];
        out.x = static_cast<decltype(out.x)>(in.x);
        out.y = static_cast<decltype(out.y)>(in.y);
        out.textSize = static_cast<decltype(out.textSize)>(in.textSize);
        for (std::size_t i=0; i<std::size(out.color); ++i) out.color[i] = in.color[i];
        for (std::size_t i=0; i<std::size(out.worldAnchorGame); ++i) out.worldAnchorGame[i] = in.worldAnchorGame[i];
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::api::diagnostics::DebugOverlayPublicationV1& out, const rock::provider::RockProviderDebugOverlayPublicationV1& in) {
        out.lineCount = static_cast<decltype(out.lineCount)>(in.lineCount);
        out.textCount = static_cast<decltype(out.textCount)>(in.textCount);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
    inline void convert(rock::provider::RockProviderDebugOverlayPublicationV1& out, const rock::api::diagnostics::DebugOverlayPublicationV1& in) {
        out.lineCount = static_cast<decltype(out.lineCount)>(in.lineCount);
        out.textCount = static_cast<decltype(out.textCount)>(in.textCount);
        out.worldGeneration = static_cast<decltype(out.worldGeneration)>(in.worldGeneration);
        out.skeletonGeneration = static_cast<decltype(out.skeletonGeneration)>(in.skeletonGeneration);
        out.providerGeneration = static_cast<decltype(out.providerGeneration)>(in.providerGeneration);
        out.leaseFrames = static_cast<decltype(out.leaseFrames)>(in.leaseFrames);
        for (std::size_t i=0; i<std::size(out.reserved); ++i) out.reserved[i] = in.reserved[i];
    }
}
