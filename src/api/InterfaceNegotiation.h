#pragma once
#include <ROCK/Discovery.h>
#include <span>

namespace rock::api::discovery {
    inline Status query(std::span<const InterfaceDescriptorV1> descriptors,
        InterfaceId id, std::uint32_t major, std::uint32_t minor,
        std::uint32_t bytes, const InterfaceDescriptorV1** output) noexcept
    {
        if (!output) return Status::InvalidArgument;
        *output = nullptr;
        if (!major || !bytes) return Status::InvalidArgument;
        bool foundId = false;
        for (const auto& descriptor : descriptors) {
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
