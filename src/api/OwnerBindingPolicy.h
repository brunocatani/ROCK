#pragma once
#include <ROCK/Abi.h>

namespace rock::provider {
    struct InterfaceBinding {
        std::uint32_t major{};
        std::uint32_t permissions{};
    };
    inline api::Status bindInterface(InterfaceBinding& binding,std::uint32_t major,
        std::uint32_t permissions,std::uint32_t supportedPermissions) noexcept
    {
        if (!major || !permissions || (permissions & ~supportedPermissions)) return api::Status::InvalidArgument;
        if (binding.major && binding.major!=major) return api::Status::Busy;
        if ((binding.permissions & permissions)!=binding.permissions) return api::Status::Busy;
        binding={major,permissions};
        return api::Status::Ok;
    }
}
