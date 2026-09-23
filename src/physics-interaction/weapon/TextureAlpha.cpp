#include "physics-interaction/weapon/TextureAlpha.h"

#include <DirectXTex.h>
#include <algorithm>
#include <cmath>

namespace rock::texture_alpha
{
    namespace
    {
        bool zeroAlpha(const DirectX::Image& image)
        {
            bool zero = true;
            const auto status = DirectX::EvaluateImage(image,
                [&](const DirectX::XMVECTOR* pixels, std::size_t width, std::size_t) {
                    for (std::size_t i = 0; zero && i < width; ++i) {
                        const float alpha = DirectX::XMVectorGetW(pixels[i]);
                        zero = std::isfinite(alpha) && alpha == 0.0f;
                    }
                });
            return SUCCEEDED(status) && zero;
        }
    }

    Result inspect(std::span<const std::uint8_t> dds, const std::atomic<bool>* stopping)
    {
        DirectX::TexMetadata metadata{};
        if (FAILED(DirectX::GetMetadataFromDDSMemory(dds.data(), dds.size(), DirectX::DDS_FLAGS_NONE, metadata)) ||
            metadata.dimension != DirectX::TEX_DIMENSION_TEXTURE2D || metadata.arraySize != 1 ||
            metadata.width == 0 || metadata.height == 0 || metadata.width > 16384 || metadata.height > 16384) {
            return Result::Unavailable;
        }
        DirectX::ScratchImage source;
        if (FAILED(DirectX::LoadFromDDSMemory(dds.data(), dds.size(), DirectX::DDS_FLAGS_NONE, &metadata, source)))
            return Result::Unavailable;
        if (!DirectX::HasAlpha(metadata.format)) return Result::Visible;
        DirectX::ScratchImage decoded;
        for (std::size_t i = 0; i < source.GetImageCount(); ++i) {
            if (stopping && stopping->load(std::memory_order_acquire)) return Result::Unavailable;
            const auto& image = source.GetImages()[i];
            if (!DirectX::IsCompressed(image.format)) {
                if (!zeroAlpha(image)) return Result::Visible;
                continue;
            }
            for (std::size_t y = 0; y < image.height; y += 4) {
                if (stopping && stopping->load(std::memory_order_acquire)) return Result::Unavailable;
                auto strip = image;
                strip.height = (std::min)(std::size_t{4}, image.height - y);
                strip.pixels += (y / 4) * image.rowPitch;
                strip.slicePitch = image.rowPitch;
                if (FAILED(DirectX::Decompress(strip, DXGI_FORMAT_R32G32B32A32_FLOAT, decoded)))
                    return Result::Unavailable;
                if (!zeroAlpha(*decoded.GetImage(0, 0, 0))) return Result::Visible;
            }
        }
        return Result::Transparent;
    }
}
