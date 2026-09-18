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

    static_assert(sizeof(SettingV1) == 56);
}
