#pragma once

#include "physics-interaction/native/HavokCompoundShapeBuilder.h"
#include "RE/NetImmerse/NiPoint.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace RE
{
    class NiAVObject;
}

namespace rock
{
    /*
     * Equipped generated hulls handed to the reference an equipped drop
     * creates. Equipped and loose replacement collision come from the same
     * source-local hull builder, so the dropped weapon reuses these exact
     * shapes instead of extracting its mesh again. The dropped reference owns
     * its own clone of the model, so each part names its source node by a
     * name path below the model root rather than by node identity.
     */
    struct DroppedWeaponCollision
    {
        struct Step
        {
            std::string name;
            std::uint32_t ordinal{};  // among same-named siblings, in slot order
            bool node{};
        };

        struct Part
        {
            std::unique_ptr<RE::hknpShape, havok_compound_shape_builder::HavokShapeRelease> shape;
            std::vector<Step> path;
            RE::NiPoint3 sourceCenter{};
            // Hull points carry the equipped source's world scale.
            float sourceWorldScale{ 1.0f };
        };

        std::vector<Part> parts;

        static bool capturePath(const RE::NiAVObject* root, const RE::NiAVObject* leaf, std::vector<Step>& out);
        static RE::NiAVObject* resolvePath(RE::NiAVObject* root, const std::vector<Step>& path);
    };
}
