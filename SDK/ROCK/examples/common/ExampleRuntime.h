#pragma once

#include "ROCKProviderApi.h"

#include <cstdint>
#include <string_view>

namespace rock::sdk::example
{
    struct Definition
    {
        const char* pluginName{ nullptr };
        std::uint32_t pluginVersion{ 1 };
        std::uint32_t requestedCapabilities{ 0 };
        std::uint32_t minimumTableBytes{ 0 };
        bool (*onStart)(std::uint64_t ownerToken) noexcept{ nullptr };
        void (*onStop)(std::uint64_t ownerToken) noexcept{ nullptr };
        void (*onFrame)(
            std::uint64_t ownerToken,
            const provider::RockProviderFrameSnapshot& snapshot) noexcept{ nullptr };
    };

    [[nodiscard]] const Definition& definition() noexcept;
    void logInfo(std::string_view message) noexcept;
    void logWarning(std::string_view message) noexcept;
}
