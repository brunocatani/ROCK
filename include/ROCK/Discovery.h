#pragma once
#include "Abi.h"
#pragma pack(push, 8)
namespace rock::api {
    struct InterfaceDescriptorV1 {
        std::uint32_t size{sizeof(InterfaceDescriptorV1)};
        InterfaceId interfaceId{};
        std::uint32_t major{};
        std::uint32_t minor{};
        std::uint32_t tableByteSize{};
        std::uint32_t requiredCoreMajor{1};
        std::uint32_t requiredCoreMinor{};
        std::uint32_t reserved{};
        const void* table{};
    };
    using QueryInterfaceV1 = Status(ROCK_CALL*)(InterfaceId, std::uint32_t exactMajor,
        std::uint32_t minimumMinor, std::uint32_t minimumTableBytes,
        const InterfaceDescriptorV1** outDescriptor) noexcept;
    inline constexpr char kQueryExportName[] = "ROCKAPI_QueryInterfaceV1";
    static_assert(sizeof(InterfaceDescriptorV1)==40);
}

#pragma pack(pop)
