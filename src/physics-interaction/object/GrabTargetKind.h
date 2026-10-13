#pragma once

#include <cstdint>

#include "physics-interaction/collision/CollisionLayerPolicy.h"

namespace rock::grab_target
{
    // Mechanical acquisition categories. Extensions own target eligibility and
    // opaque extraction tickets; core owns hand/pull/constraint execution.
    enum class Kind : std::uint8_t
    {
        None = 0,
        LooseObject,
        ExtractableItem,
        DetachedBody,
        DynamicMovableStatic,
        ConnectedBody,

        BlockedExtension,
    };

    enum class GrabAcquisitionMode : std::uint8_t
    {
        Default,
        HandPocketOnly,
    };

    [[nodiscard]] inline constexpr const char* name(Kind kind) noexcept
    {
        switch (kind) {
        case Kind::LooseObject:
            return "loose-object";
        case Kind::ExtractableItem:
            return "extractable-item";
        case Kind::DetachedBody:
            return "detached-body";
        case Kind::DynamicMovableStatic:
            return "dynamic-movable-static";
        case Kind::ConnectedBody:
            return "connected-body";
        case Kind::BlockedExtension:
            return "blocked-extension";
        default:
            return "none";
        }
    }

    [[nodiscard]] inline constexpr bool isPhysicalRockObject(Kind kind) noexcept
    {
        return kind == Kind::LooseObject || kind == Kind::DetachedBody || kind == Kind::DynamicMovableStatic || kind == Kind::ConnectedBody;
    }

    [[nodiscard]] inline constexpr GrabAcquisitionMode acquisitionMode(Kind kind) noexcept
    {
        switch (kind) {
        case Kind::DynamicMovableStatic:
        case Kind::DetachedBody:
        case Kind::ConnectedBody:
            return GrabAcquisitionMode::HandPocketOnly;
        default:
            return GrabAcquisitionMode::Default;
        }
    }

    [[nodiscard]] inline constexpr bool requiresHandPocketGrab(Kind kind) noexcept
    {
        return acquisitionMode(kind) == GrabAcquisitionMode::HandPocketOnly;
    }

    [[nodiscard]] inline constexpr bool canUseFarSelection(Kind kind) noexcept
    {
        return kind == Kind::ExtractableItem || (isPhysicalRockObject(kind) && !requiresHandPocketGrab(kind));
    }

    [[nodiscard]] inline constexpr bool canUseRockDynamicPull(Kind kind) noexcept
    {
        return isPhysicalRockObject(kind) && !requiresHandPocketGrab(kind);
    }

    [[nodiscard]] inline constexpr bool canUseRockActiveGrab(Kind kind) noexcept
    {
        return isPhysicalRockObject(kind);
    }

    [[nodiscard]] inline constexpr bool requiresNativeBodyOwnership(Kind kind) noexcept
    {
        return kind == Kind::ConnectedBody || kind == Kind::DetachedBody;
    }

    [[nodiscard]] inline constexpr bool prefersMechanicalScope(Kind kind) noexcept
    {
        return kind == Kind::ConnectedBody || kind == Kind::DynamicMovableStatic;
    }

    [[nodiscard]] inline constexpr bool relaxesStrictPocketAuthorityForMechanicalGrab(Kind kind) noexcept
    {
        return kind == Kind::ConnectedBody;
    }

    [[nodiscard]] inline constexpr bool isExtensionSelection(Kind kind) noexcept
    {
        return kind == Kind::ExtractableItem || kind == Kind::BlockedExtension;
    }

    [[nodiscard]] inline constexpr bool isNativeDetachedBodyLayer(std::uint32_t layer) noexcept
    {
        return layer == collision_layer_policy::FO4_LAYER_DEADBIP;
    }

}
