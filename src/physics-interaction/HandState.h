#pragma once

#include <cstdint>

namespace frik::rock
{
    enum class HandState : std::uint8_t
    {
        Idle,
        SelectedClose,
        SelectedFar,
        SelectionLocked,
        HeldInit,
        HeldBody,
        Pulled,
        GrabFromOtherHand,
        SelectedTwoHand,
        HeldTwoHanded,
    };
}
