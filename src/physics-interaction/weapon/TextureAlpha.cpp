#include "physics-interaction/weapon/TextureAlpha.h"

#include <DirectXTex.h>
#include <algorithm>
#include <array>
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
        Result inspectImages(std::span<const DirectX::Image> images, const std::atomic<bool>* stopping)
        {
            DirectX::ScratchImage decoded;
            for (const auto& image : images) {
                if (stopping && stopping->load(std::memory_order_acquire)) return Result::Unavailable;
                if (!DirectX::HasAlpha(image.format)) return Result::Visible;
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
        return inspectImages({ source.GetImages(), source.GetImageCount() }, stopping);
    }

    Result inspectMipData(std::span<const std::uint8_t> bytes, std::uint32_t formatValue,
        std::uint32_t width, std::uint32_t height, std::uint32_t mipCount, const std::atomic<bool>* stopping)
    {
        const auto format = static_cast<DXGI_FORMAT>(formatValue);
        if (!width || !height || width > 16384 || height > 16384 || !mipCount || mipCount > 15 ||
            !DirectX::IsValid(format) || DirectX::IsTypeless(format) || DirectX::IsPlanar(format))
            return Result::Unavailable;
        std::array<DirectX::Image, 15> images{};
        std::size_t offset = 0;
        for (std::uint32_t mip = 0; mip < mipCount; ++mip) {
            std::size_t row = 0, slice = 0;
            if (FAILED(DirectX::ComputePitch(format, width, height, row, slice)) || slice > bytes.size() - offset)
                return Result::Unavailable;
            images[mip] = { width, height, format, row, slice, const_cast<std::uint8_t*>(bytes.data() + offset) };
            offset += slice;
            if (width == 1 && height == 1 && mip + 1 < mipCount) return Result::Unavailable;
            width = (std::max)(width / 2, 1u);
            height = (std::max)(height / 2, 1u);
        }
        if (offset != bytes.size()) return Result::Unavailable;
        return inspectImages({ images.data(), mipCount }, stopping);
    }
}
