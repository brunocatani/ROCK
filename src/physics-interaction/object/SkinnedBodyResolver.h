#pragma once

#include <cstdint>

#include "physics-interaction/object/GeometryBodyResolver.h"
#include "physics-interaction/object/GrabTargetKind.h"

namespace rock::skinned_body_resolver
{
    inline constexpr std::uint32_t kInvalidBodyId = geometry_body_resolver::kInvalidBodyId;

    enum class ResolutionSource : std::uint8_t
    {
        None,
        WeightedSkinOwner,
        TriangleOwner,
        SelectedBody,
        NearestAccepted,
    };

    struct ResolutionInput
    {
        grab_target::Kind targetKind = grab_target::Kind::LooseObject;
        std::uint32_t surfaceOwnerBodyId = kInvalidBodyId;
        std::uint32_t selectedBodyId = kInvalidBodyId;
        std::uint32_t nearestBodyId = kInvalidBodyId;
        bool surfaceOwnerUsable = false;
        bool selectedUsable = false;
        bool nearestUsable = false;
        bool surfaceIsSkinned = false;
        bool hasSkinInfluences = false;
    };

    struct Resolution
    {
        std::uint32_t bodyId = kInvalidBodyId;
        ResolutionSource source = ResolutionSource::None;
        const char* reason = "noAcceptedBody";
        bool usedSkinInfluences = false;
        bool relaxedMechanicalAuthority = false;
    };

    [[nodiscard]] inline const char* sourceName(ResolutionSource source) noexcept
    {
        switch (source) {
        case ResolutionSource::WeightedSkinOwner:
            return "weightedSkinOwner";
        case ResolutionSource::TriangleOwner:
            return "triangleOwner";
        case ResolutionSource::SelectedBody:
            return "selectedBody";
        case ResolutionSource::NearestAccepted:
            return "nearestAccepted";
        case ResolutionSource::None:
        default:
            return "none";
        }
    }

    [[nodiscard]] inline bool validBody(std::uint32_t bodyId) noexcept { return bodyId != kInvalidBodyId; }

    [[nodiscard]] inline Resolution resolvePrimaryBody(const ResolutionInput& input) noexcept
    {
        if (input.surfaceIsSkinned && input.hasSkinInfluences && input.surfaceOwnerUsable && validBody(input.surfaceOwnerBodyId)) {
            return Resolution{
                .bodyId = input.surfaceOwnerBodyId,
                .source = ResolutionSource::WeightedSkinOwner,
                .reason = "weightedSkinOwner",
                .usedSkinInfluences = true,
                .relaxedMechanicalAuthority = grab_target::relaxesStrictPocketAuthorityForMechanicalGrab(input.targetKind),
            };
        }

        const auto base = geometry_body_resolver::resolveGeometryBody(geometry_body_resolver::GeometryBodyResolutionInput{
            .triangleOwnerBodyId = input.surfaceOwnerBodyId,
            .selectedBodyId = input.selectedBodyId,
            .nearestBodyId = input.nearestBodyId,
            .triangleOwnerUsable = input.surfaceOwnerUsable,
            .selectedUsable = input.selectedUsable,
            .nearestUsable = input.nearestUsable,
        });

        switch (base.source) {
        case geometry_body_resolver::GeometryBodyResolutionSource::TriangleOwner:
            return Resolution{ .bodyId = base.bodyId, .source = ResolutionSource::TriangleOwner, .reason = base.reason };
        case geometry_body_resolver::GeometryBodyResolutionSource::SelectedBody:
            return Resolution{ .bodyId = base.bodyId, .source = ResolutionSource::SelectedBody, .reason = base.reason };
        case geometry_body_resolver::GeometryBodyResolutionSource::NearestAccepted:
            return Resolution{ .bodyId = base.bodyId, .source = ResolutionSource::NearestAccepted, .reason = base.reason };
        default:
            return Resolution{};
        }
    }
}
