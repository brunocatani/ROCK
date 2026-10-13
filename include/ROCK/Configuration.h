#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::configuration {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Configuration;
    inline constexpr std::uint32_t kMajor = 1;
    inline constexpr std::uint32_t kMinor = 0;
    enum class PermissionV1 : std::uint32_t { Read=1, Write=2 };
    inline constexpr std::uint32_t kSupportedPermissions = 3;
    enum class Group : std::uint32_t { Consumer, Developer };
    enum class ValueType : std::uint32_t { Boolean, Integer, Float, String };
    struct SettingV1 {
        const char* section;
        const char* key;
        const char* value;
        const char* defaultValue;
        const char* category;
        const char* description;
        ValueType type;
        std::uint32_t overridden;
    };
    // Strings and the setting record are borrowed for this synchronous callback only.
    using VisitorV1 = void(ROCK_CALL*)(const SettingV1*, void*);
    struct ApiV1 {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor;
        static constexpr std::uint32_t minorVersion=kMinor;
        Status(ROCK_CALL* revision)(OwnerToken, std::uint64_t* outRevision) noexcept;
        Status(ROCK_CALL* visit)(OwnerToken, Group, VisitorV1, void*) noexcept;
        Status(ROCK_CALL* setValue)(OwnerToken, Group, const char* section, const char* key,
            const char* value, char* error, std::uint32_t errorCapacity) noexcept;
    };
    static_assert(sizeof(SettingV1)==56);
}

#pragma pack(pop)
