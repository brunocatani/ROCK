#pragma once
#include <ROCK/Abi.h>

namespace rock::provider {
    struct InterfaceBinding {
        std::uint32_t major{};
        std::uint32_t permissions{};
    };
    enum class OwnerAccess { Existing, Active };

    inline api::Status authorizeBinding(const InterfaceBinding& binding, bool revoked,
        std::uint32_t permission, OwnerAccess access = OwnerAccess::Existing) noexcept
    {
        if (revoked && (access == OwnerAccess::Active || (permission != 0 && permission != 1)))
            return api::Status::OwnerRevoked;
        return (binding.permissions & permission) == permission ?
            api::Status::Ok : api::Status::PermissionDenied;
    }
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
