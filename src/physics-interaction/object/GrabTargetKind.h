#pragma once

#include <cstdint>

#include "physics-interaction/collision/CollisionLayerPolicy.h"

namespace rock::grab_target
{
    /*
     * Actor visuals, gore pieces, and loose objects can all arrive through the
     * same hknp selection query, but they do not share authority. This target
     * kind keeps ROCK's gameplay split explicit: whole NPC bodies are not loose
     * pull-grab objects, worn gear is an actor-loot candidate, and detached gore
     * parts are physical objects only when the runtime body evidence proves it.
     */
    enum class Kind : std::uint8_t
    {
        None = 0,
        LooseObject,
        ActorEquipment,
        DetachedGore,
        DynamicMovableStatic,
        DeadActorBody,
        LiveActorScissors,
        BlockedWholeActorBody,
    };

    enum class GrabAcquisitionMode : std::uint8_t
    {
        Default,
        HandPocketOnly,
    };

    enum class HandlingProfile : std::uint8_t
    {
        OrdinaryLooseObject,
        CloseSimplePhysical,
        CloseMechanicalPreferred,
        ActorDriven,
        Blocked,
    };

    [[nodiscard]] inline constexpr const char* name(Kind kind) noexcept
    {
        switch (kind) {
        case Kind::LooseObject:
            return "loose-object";
        case Kind::ActorEquipment:
            return "actor-equipment";
        case Kind::DetachedGore:
            return "detached-gore";
        case Kind::DynamicMovableStatic:
            return "dynamic-movable-static";
        case Kind::DeadActorBody:
            return "dead-actor-body";
        case Kind::LiveActorScissors:
            return "live-actor-scissors";
        case Kind::BlockedWholeActorBody:
            return "blocked-whole-actor-body";
        default:
            return "none";
        }
    }

    [[nodiscard]] inline constexpr bool isPhysicalRockObject(Kind kind) noexcept
    {
        return kind == Kind::LooseObject || kind == Kind::DetachedGore || kind == Kind::DynamicMovableStatic || kind == Kind::DeadActorBody;
    }

    [[nodiscard]] inline constexpr HandlingProfile handlingProfile(Kind kind) noexcept
    {
        switch (kind) {
        case Kind::LooseObject:
            return HandlingProfile::OrdinaryLooseObject;
        case Kind::DetachedGore:
            return HandlingProfile::CloseSimplePhysical;
        case Kind::DynamicMovableStatic:
            return HandlingProfile::CloseMechanicalPreferred;
        case Kind::DeadActorBody:
            return HandlingProfile::CloseMechanicalPreferred;
        case Kind::ActorEquipment:
        case Kind::LiveActorScissors:
            return HandlingProfile::ActorDriven;
        case Kind::BlockedWholeActorBody:
            return HandlingProfile::Blocked;
        default:
            return HandlingProfile::Blocked;
        }
    }

    [[nodiscard]] inline constexpr GrabAcquisitionMode acquisitionMode(Kind kind) noexcept
    {
        switch (kind) {
        case Kind::DynamicMovableStatic:
        case Kind::DetachedGore:
        case Kind::DeadActorBody:
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
        return kind == Kind::ActorEquipment || (isPhysicalRockObject(kind) && !requiresHandPocketGrab(kind));
    }

    [[nodiscard]] inline constexpr bool canUseRockDynamicPull(Kind kind) noexcept
    {
        return isPhysicalRockObject(kind) && !requiresHandPocketGrab(kind);
    }

    [[nodiscard]] inline constexpr bool canUseRockActiveGrab(Kind kind) noexcept
    {
        return isPhysicalRockObject(kind);
    }

    [[nodiscard]] inline constexpr bool prefersMechanicalScope(Kind kind) noexcept
    {
        return kind == Kind::DeadActorBody || kind == Kind::DynamicMovableStatic;
    }

    [[nodiscard]] inline constexpr bool relaxesStrictPocketAuthorityForMechanicalGrab(Kind kind) noexcept
    {
        return kind == Kind::DeadActorBody;
    }

    [[nodiscard]] inline constexpr bool isActorDriven(Kind kind) noexcept
    {
        return kind == Kind::ActorEquipment || kind == Kind::LiveActorScissors || kind == Kind::BlockedWholeActorBody;
    }

    [[nodiscard]] inline constexpr bool isDetachedGoreLayer(std::uint32_t layer) noexcept
    {
        return layer == collision_layer_policy::FO4_LAYER_DEADBIP;
    }
}
