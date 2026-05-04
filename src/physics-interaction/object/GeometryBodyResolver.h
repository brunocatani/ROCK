#pragma once

/*
 * Geometry body resolution is separated from mesh extraction because HIGGS keeps
 * the contact geometry and chosen rigid body coherent before creating the held
 * relationship. ROCK has several evidence sources (authored grab nodes, visual
 * triangle owners, selected hknp body, nearest fallback). This policy fixes the
 * priority order without depending on unverified shape-key or skinned-weight
 * decoding; dynamic skinned ownership remains an explicit fallback reason until
 * its runtime layout is verified.
 */

#include <cstdint>

namespace rock::geometry_body_resolver
{
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;

    enum class GeometryBodyResolutionSource : std::uint8_t
    {
        None,
        AuthoredNode,
        TriangleOwner,
        SelectedBody,
        NearestAccepted,
    };

    struct GeometryBodyResolutionInput
    {
        std::uint32_t authoredBodyId = kInvalidBodyId;
        std::uint32_t triangleOwnerBodyId = kInvalidBodyId;
        std::uint32_t selectedBodyId = kInvalidBodyId;
        std::uint32_t nearestBodyId = kInvalidBodyId;
        bool authoredUsable = false;
        bool triangleOwnerUsable = false;
        bool selectedUsable = true;
        bool nearestUsable = true;
    };

    struct GeometryBodyResolution
    {
        std::uint32_t bodyId = kInvalidBodyId;
        GeometryBodyResolutionSource source = GeometryBodyResolutionSource::None;
        const char* reason = "noAcceptedBody";
    };

    inline bool validBody(std::uint32_t bodyId) { return bodyId != kInvalidBodyId; }

    inline GeometryBodyResolution resolveGeometryBody(const GeometryBodyResolutionInput& input)
    {
        if (input.authoredUsable && validBody(input.authoredBodyId)) {
            return GeometryBodyResolution{ .bodyId = input.authoredBodyId, .source = GeometryBodyResolutionSource::AuthoredNode, .reason = "authoredNode" };
        }
        if (input.triangleOwnerUsable && validBody(input.triangleOwnerBodyId)) {
            return GeometryBodyResolution{ .bodyId = input.triangleOwnerBodyId, .source = GeometryBodyResolutionSource::TriangleOwner, .reason = "triangleOwner" };
        }
        if (input.selectedUsable && validBody(input.selectedBodyId)) {
            return GeometryBodyResolution{ .bodyId = input.selectedBodyId, .source = GeometryBodyResolutionSource::SelectedBody, .reason = "selectedBody" };
        }
        if (input.nearestUsable && validBody(input.nearestBodyId)) {
            return GeometryBodyResolution{ .bodyId = input.nearestBodyId, .source = GeometryBodyResolutionSource::NearestAccepted, .reason = "nearestAccepted" };
        }
        return GeometryBodyResolution{};
    }

    inline const char* dynamicSkinnedResolutionFallbackReason(bool verifiedWeightedBoneLayout)
    {
        return verifiedWeightedBoneLayout ? "weightedBoneOwner" : "dynamicSkinnedWeightsUnverified";
    }
}
