#pragma once

#include <cstdint>

namespace rock::configuration_api
{
    enum class Group : std::uint32_t { Consumer, Developer };
    enum class ValueType : std::uint32_t { Boolean, Integer, Float, String };

    struct SettingV1
    {
        const char* section;
        const char* key;
        const char* value;
        const char* defaultValue;
        const char* category;
        const char* description;
        ValueType type;
        std::uint32_t overridden;
    };

    using VisitorV1 = void (*)(const SettingV1*, void*) noexcept;

    // revision() is safe on any thread and performs no I/O. Other calls belong
    // on the game task thread, outside physics/input callbacks. Visitor strings
    // are borrowed for that callback only. A successful write is applied by
    // ROCK's next configuration reload; revision changes after application.
    struct ApiV1
    {
        std::uint32_t version;
        std::uint32_t byteSize;
        std::uint64_t (*revision)() noexcept;
        bool (*visit)(Group, VisitorV1, void*) noexcept;
        bool (*setValue)(Group, const char* section, const char* key,
            const char* value, char* error, std::uint32_t errorCapacity) noexcept;
    };

    using GetApiV1 = const ApiV1* (*)(std::uint32_t version) noexcept;
    inline constexpr auto kExportName = "GetROCKConfigurationApi";
    inline constexpr std::uint32_t kVersion = 1;

    static_assert(sizeof(SettingV1) == 56);
    static_assert(sizeof(ApiV1) == 32);
}
