#pragma once

#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/object/PhysicsBodyClassifier.h"

#include <cstdint>
#include <optional>

namespace RE { class TESObjectREFR; }

namespace rock::loose_weapon_layer_repair
{
    inline constexpr std::optional<std::uint32_t> correctedFilter(
        std::uint32_t filter, std::uint32_t flags, std::uint32_t motion, bool exactLooseOwner)
    {
        using namespace collision_layer_policy;
        using namespace physics_body_classifier;
        const auto type = motionTypeFromBodyFlags(flags);
        // A native save may park a movable weapon as keyframed. Repair only
        // its category; leave its motion, shape, group and suppression bits alone.
        if (!exactLooseOwner || !motion || (filter & FO4_LAYER_FILTER_MASK) != FO4_LAYER_STATIC ||
            (type != BodyMotionType::Dynamic && type != BodyMotionType::Keyframed)) return std::nullopt;
        return (filter & ~FO4_LAYER_FILTER_MASK) | FO4_LAYER_WEAPON;
    }

    // Borrowed initializer scope only. The existing native initialization hook
    // calls this before either normal simulation or retirement qualification.
    // The corrected layer is the final state of this loose body's lifetime;
    // recreated bodies are qualified anew. No registry or frame polling.
    void initialize(RE::TESObjectREFR* reference, void (*original)(RE::TESObjectREFR*));
}
