#pragma once

#include "physics-interaction/weapon/TextureAlpha.h"
#include <string_view>

namespace rock::weapon_texture_alpha
{
    // One game-thread owner, started/stopped with WeaponCollision. The worker
    // owns its resource streams and DDS buffers; only copied paths and atomic
    // results cross threads. No scene/material/texture pointers leave the frame.
    void start();
    void stop();
    [[nodiscard]] texture_alpha::Result query(std::string_view sourcePath);
}
