#pragma once

#include "RE/NetImmerse/NiTransform.h"
#include <cstdint>

namespace rock::native_secondary_pose_transport
{
    // Intermediate native arm passes can overwrite NiAVObject::previousWorld
    // several times in one frame. Retain the prior final presentation, while
    // allowing later corrections in the current frame to replace its sample.
    class PresentedHistory
    {
        RE::NiTransform _current{},_previous{};
        std::uint64_t _frame{};
    public:
        const RE::NiTransform& publish(std::uint64_t frame,const RE::NiTransform& world) noexcept
        {
            if (frame!=_frame) {
                _previous=_frame && frame==_frame+1?_current:world;
                _frame=frame;
            }
            _current=world;
            return _previous;
        }
    };
}
