#pragma once

#include "physics-interaction/weapon/TextureAlpha.h"

namespace rock::weapon_texture_source
{
    struct Trace
    {
        const char* stage = "resource-open";
        std::size_t bytes = 0;
        unsigned error = 0;
        unsigned archive = 0, chunks = 0, format = 0, width = 0, height = 0;
    };

    // Called only by the texture-alpha worker. All streams and buffers are owned
    // by this call; no renderer or scene object is retained or touched here.
    texture_alpha::Result inspect(const char* path, const std::atomic<bool>& stopping, Trace& trace);
}
