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

    // Archive texture records contain the mip payload without a DDS header.
    // format is DXGI_FORMAT; bytes must contain the complete, tightly packed chain.
    [[nodiscard]] Result inspectMipData(std::span<const std::uint8_t> bytes, std::uint32_t format,
        std::uint32_t width, std::uint32_t height, std::uint32_t mipCount,
        const std::atomic<bool>* stopping = nullptr);
}
