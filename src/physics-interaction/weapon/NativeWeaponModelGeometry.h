#pragma once

#include "RE/NetImmerse/NiNode.h"
#include <cstdint>

namespace rock::native_weapon_model_geometry
{
    struct Census
    {
        std::uint32_t objects = 0, shapes = 0;
        bool complete = true;
    };

    inline void count(RE::NiAVObject* object, Census& result, std::uint32_t depth = 0) noexcept
    {
        if (!object || !result.complete) return;
        if (depth >= 128 || result.objects >= 8192) {
            result.complete = false;
            return;
        }
        ++result.objects;
        if (object->IsGeometry() || object->IsTriShape()) ++result.shapes;
        if (auto* node = object->IsNode()) {
            for (const auto& child : node->GetRuntimeData().children)
                if (child) count(child.get(), result, depth + 1);
        }
    }

    // Construction/identity transition only, not a per-frame full tree scan.
    inline Census inspect(RE::NiAVObject* root) noexcept
    {
        Census result{};
        count(root, result);
        return result;
    }
}
