#pragma once

/*
 * Geometry body resolution is separated from mesh extraction so ROCK keeps the
 * contact geometry and chosen rigid body coherent before creating the held
 * relationship. Visual triangle owners, the selected hknp body, and the nearest
 * accepted fallback can nominate the body. This
 * policy fixes the priority order without depending on unverified shape-key or
 * skinned-weight decoding; dynamic skinned ownership remains an explicit
 * fallback reason until its runtime layout is verified.
 */

#include <cstdint>

namespace rock::geometry_body_resolver
{
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;

    enum class GeometryBodyResolutionSource : std::uint8_t
    {
        None,
        TriangleOwner,
        SelectedBody,
        NearestAccepted,
    };

    struct GeometryBodyResolutionInput
    {
        std::uint32_t triangleOwnerBodyId = kInvalidBodyId;
        std::uint32_t selectedBodyId = kInvalidBodyId;
        std::uint32_t nearestBodyId = kInvalidBodyId;
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
