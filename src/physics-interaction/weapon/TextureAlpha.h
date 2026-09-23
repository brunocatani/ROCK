#pragma once

#include <cstdint>
#include <span>
#include <atomic>

namespace rock::texture_alpha
{
    enum class Result : std::uint8_t
    {
        Pending,
        Transparent,
        Visible,
        Unavailable
    };

    // CPU-only DDS inspection. Every texel of every mip must have exactly zero
    // alpha. Compressed images are decoded a block row at a time, bounding the
    // scratch allocation without losing small opaque features or tiny alpha.
    [[nodiscard]] Result inspect(std::span<const std::uint8_t> dds, const std::atomic<bool>* stopping = nullptr);
}
