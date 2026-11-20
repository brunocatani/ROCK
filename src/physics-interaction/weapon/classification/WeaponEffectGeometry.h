// Reads from a live weapon shape the evidence WeaponEffectGeometryPolicy
// classifies. Generated weapon collision and the far pull both use it, so they
// agree on which geometry is the weapon's solid body.
#pragma once

#include "RE/Fallout.h"
#include "physics-interaction/weapon/classification/WeaponEffectGeometryPolicy.h"

#include <cstring>

namespace rock::weapon_effect_geometry
{
    [[nodiscard]] inline bool niObjectRttiChainContains(const RE::NiObject* object, const char* typeName)
    {
        const RE::NiRTTI* rtti = object && typeName ? object->GetRTTI() : nullptr;
        for (int depth = 0; rtti && depth < 16; ++depth, rtti = rtti->GetBaseRTTI()) {
            const char* rttiName = rtti->GetName();
            if (rttiName && std::strcmp(rttiName, typeName) == 0) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] inline bool shapeHasEffectShaderProperty(const RE::BSTriShape* triShape)
    {
        if (!triShape) {
            return false;
        }
        for (const auto& property : triShape->GetRuntimeData().properties) {
            if (niObjectRttiChainContains(property.get(), "BSEffectShaderProperty")) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] inline bool shapeHasBillboardAncestor(const RE::NiAVObject* node)
    {
        // Parent links are frame-scoped engine references; nothing from this walk is retained.
        for (auto* ancestor = node ? node->parent : nullptr; ancestor; ancestor = ancestor->parent) {
            if (niObjectRttiChainContains(ancestor, "NiBillboardNode")) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] inline weapon_effect_geometry_policy::ExclusionReason classify(const RE::BSTriShape* triShape)
    {
        const char* name = triShape ? triShape->name.c_str() : nullptr;
        return weapon_effect_geometry_policy::classify({
            .hasEffectShaderProperty = shapeHasEffectShaderProperty(triShape),
            .hasBillboardAncestor = shapeHasBillboardAncestor(triShape),
            .geometryName = name ? name : "",
        });
    }

    // Laser and flashlight beams, dots, reticles and glows: drawn with the
    // weapon, never part of what a hand or the world touches.
    [[nodiscard]] inline bool isEffectShape(const RE::BSTriShape* triShape)
    {
        return classify(triShape) != weapon_effect_geometry_policy::ExclusionReason::None;
    }
}
