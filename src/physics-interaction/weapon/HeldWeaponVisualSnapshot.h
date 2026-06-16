#pragma once

#include <cstdint>

#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTransform.h"

namespace RE
{
    class TESObjectREFR;
}

namespace rock
{
    struct HeldWeaponVisualSnapshot
    {
        RE::TESObjectREFR* heldRef = nullptr;
        RE::NiNode* cloneSourceNode = nullptr;
        RE::NiNode* parent = nullptr;
        RE::NiTransform sourceWorld{};
        std::uint32_t formID = 0;
        const char* source = "none";
        bool isLeft = false;

        [[nodiscard]] bool isValid() const noexcept
        {
            return heldRef != nullptr && cloneSourceNode != nullptr && parent != nullptr && formID != 0;
        }
    };
}
