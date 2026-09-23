#include "physics-interaction/weapon/TextureAlpha.h"

#include <DirectXTex.h>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <vector>

namespace
{
    using rock::texture_alpha::Result;

    Result classify(const DirectX::ScratchImage& image, const std::atomic<bool>* stopping = nullptr)
    {
        DirectX::Blob dds;
        if (FAILED(DirectX::SaveToDDSMemory(image.GetImages(), image.GetImageCount(), image.GetMetadata(),
                DirectX::DDS_FLAGS_NONE, dds))) return Result::Unavailable;
        const auto result = rock::texture_alpha::inspect({ static_cast<const std::uint8_t*>(dds.GetBufferPointer()), dds.GetBufferSize() }, stopping);
        const auto& info = image.GetMetadata();
        const auto raw = rock::texture_alpha::inspectMipData({ image.GetPixels(), image.GetPixelsSize() },
            info.format, static_cast<std::uint32_t>(info.width), static_cast<std::uint32_t>(info.height),
            static_cast<std::uint32_t>(info.mipLevels), stopping);
        return raw == result ? result : Result::Unavailable;
    }
}

int main(int argc, char** argv)
{
    DirectX::ScratchImage image;
    if (FAILED(image.Initialize2D(DXGI_FORMAT_R8G8B8A8_UNORM, 8, 8, 1, 4))) return 1;
    std::memset(image.GetPixels(), 0, image.GetPixelsSize());
    if (classify(image) != Result::Transparent) return 2;
    const std::atomic<bool> stopped{ true };
    if (classify(image, &stopped) != Result::Unavailable) return 14;

    // One visible pixel is enough; mip averages and thumbnail sampling would
    // incorrectly erase fine wire, glass markings, or attachment decals.
    image.GetImage(0, 0, 0)->pixels[63 * 4 + 3] = 1;
    if (classify(image) != Result::Visible) return 3;
    std::memset(image.GetPixels(), 0, image.GetPixelsSize());
    image.GetImage(3, 0, 0)->pixels[3] = 255;
    if (classify(image) != Result::Visible) return 4;
    std::memset(image.GetPixels(), 0, image.GetPixelsSize());

    for (auto format : { DXGI_FORMAT_BC1_UNORM, DXGI_FORMAT_BC2_UNORM, DXGI_FORMAT_BC3_UNORM, DXGI_FORMAT_BC7_UNORM }) {
        DirectX::ScratchImage compressed;
        if (FAILED(DirectX::Compress(image.GetImages(), image.GetImageCount(), image.GetMetadata(), format,
                DirectX::TEX_COMPRESS_DEFAULT, 0.5f, compressed))) return 5;
        if (classify(compressed) != Result::Transparent) return 6;
        image.GetImage(0, 0, 0)->pixels[3] = 255;
        if (FAILED(DirectX::Compress(image.GetImages(), image.GetImageCount(), image.GetMetadata(), format,
                DirectX::TEX_COMPRESS_DEFAULT, 0.5f, compressed))) return 7;
        if (classify(compressed) != Result::Visible) return 8;
        image.GetImage(0, 0, 0)->pixels[3] = 0;
    }

    DirectX::ScratchImage floating;
    if (FAILED(floating.Initialize2D(DXGI_FORMAT_R32G32B32A32_FLOAT, 4, 4, 1, 1))) return 9;
    std::memset(floating.GetPixels(), 0, floating.GetPixelsSize());
    reinterpret_cast<float*>(floating.GetPixels())[3] = 1e-8f;
    if (classify(floating) != Result::Visible) return 10;
    if (rock::texture_alpha::inspect({}) != Result::Unavailable) return 11;
    const std::uint8_t invalid[]{ 'D', 'D', 'S', ' ' };
    if (rock::texture_alpha::inspect(invalid) != Result::Unavailable) return 12;
    if (rock::texture_alpha::inspectMipData(invalid, DXGI_FORMAT_R8G8B8A8_UNORM, 8, 8, 1) != Result::Unavailable) return 15;
    const std::uint8_t transparentPixel[4]{};
    if (rock::texture_alpha::inspectMipData(transparentPixel, DXGI_FORMAT_R8G8B8A8_UNORM, 1, 1, 2) != Result::Unavailable) return 16;
    if (rock::texture_alpha::inspectMipData(transparentPixel, DXGI_FORMAT_R8G8B8A8_UNORM, 32768, 1, 1) != Result::Unavailable) return 17;
    if (rock::texture_alpha::inspectMipData(transparentPixel, DXGI_FORMAT_R8G8B8A8_UNORM, 1, 1, 0) != Result::Unavailable) return 18;

    // Optional local regression inputs: real DDS files, with no mod or path
    // assumptions in the decoder and no weapon assets distributed by ROCK.
    for (int i = 1; i < argc; ++i) {
        std::ifstream input(argv[i], std::ios::binary);
        const std::vector<std::uint8_t> bytes{ std::istreambuf_iterator<char>(input), {} };
        const auto result = rock::texture_alpha::inspect(bytes);
        std::printf("%s: %u\n", argv[i], static_cast<unsigned>(result));
        if (result != Result::Transparent) return 13;
        DirectX::ScratchImage actual;
        if (FAILED(DirectX::LoadFromDDSMemory(bytes.data(), bytes.size(), DirectX::DDS_FLAGS_NONE, nullptr, actual)) ||
            classify(actual) != Result::Transparent) return 19;
    }
    return 0;
}
