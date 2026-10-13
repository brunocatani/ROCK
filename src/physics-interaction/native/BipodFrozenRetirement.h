#pragma once

#include "RE/NetImmerse/NiTransform.h"
#include <cstdint>

namespace RE { class bhkWorld; class hknpWorld; class TESObjectREFR; }

namespace rock::bipod_frozen_retirement
{
    // One exact equipped-item transfer at a time. The game thread arms and
    // finishes it; native reference initialization may run on another thread.
    // No native pointer is retained across that initializer's borrowed scope.
    bool install();
    bool available();
    bool arm(std::uint32_t weaponForm, RE::bhkWorld* world, const RE::NiTransform& pose);
    void finishRemoval(bool committed);
    void service();
    bool pending();
    bool frozen(std::uint32_t reference);
    void cancelForWorldLoss();
}
