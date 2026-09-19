#pragma once
#include <ROCK/Discovery.h>
#include <ROCK/Core.h>
#include <span>

namespace rock::api::discovery {
    struct RegisteredInterface {
        InterfaceDescriptorV1 descriptor;
        std::uint32_t permissions;
    };

    template<class Table> RegisteredInterface registration(const Table& table, std::uint32_t permissions) noexcept {
        return {{sizeof(InterfaceDescriptorV1), Table::interfaceId, Table::majorVersion, Table::minorVersion,
            sizeof(Table), core::kMajor, core::kMinor, 0, &table}, permissions};
    }

    // Discovery and owner binding share the same installed contracts.
    std::span<const RegisteredInterface> registeredInterfaces() noexcept;

    inline const RegisteredInterface* findRegistration(std::span<const RegisteredInterface> entries,
        InterfaceId id, std::uint32_t major) noexcept {
        for (const auto& entry : entries)
            if (entry.descriptor.interfaceId == id && entry.descriptor.major == major) return &entry;
        return nullptr;
    }

    inline Status query(std::span<const RegisteredInterface> entries,
        InterfaceId id, std::uint32_t major, std::uint32_t minor,
        std::uint32_t bytes, const InterfaceDescriptorV1** output) noexcept
    {
        if (!output) return Status::InvalidArgument;
        *output = nullptr;
        if (!major || !bytes) return Status::InvalidArgument;
        bool foundId = false;
        for (const auto& entry : entries) {
            const auto& descriptor = entry.descriptor;
            if (descriptor.interfaceId != id) continue;
            foundId = true;
            if (descriptor.major != major) continue;
            if (descriptor.minor < minor) return Status::UnsupportedMinor;
            if (descriptor.tableByteSize < bytes) return Status::TableTooSmall;
            if (!descriptor.table || descriptor.size != sizeof(InterfaceDescriptorV1)) return Status::InternalError;
            *output = &descriptor;
            return Status::Ok;
        }
        return foundId ? Status::UnsupportedMajor : Status::UnknownInterface;
    }
}
