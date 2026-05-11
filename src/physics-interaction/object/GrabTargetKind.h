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
        LiveActorScissors,
        BlockedWholeActorBody,
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
        return kind == Kind::LooseObject || kind == Kind::DetachedGore;
    }

    [[nodiscard]] inline constexpr bool canUseRockDynamicPull(Kind kind) noexcept
    {
        return isPhysicalRockObject(kind);
    }

    [[nodiscard]] inline constexpr bool canUseRockActiveGrab(Kind kind) noexcept
    {
        return isPhysicalRockObject(kind);
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
