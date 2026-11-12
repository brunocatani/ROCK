#pragma once

#include "physics-interaction/input/QueuedHandInput.h"

namespace rock::native_pipboy_hand_input
{
    [[nodiscard]] bool install() noexcept;
    [[nodiscard]] queued_hand_input::Origin current() noexcept;
}
