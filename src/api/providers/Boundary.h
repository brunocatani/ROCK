#pragma once
#include "api/ProviderRuntimeServices.h"
#include <algorithm>
#include <array>
namespace rock::api::boundary {
    template<class F> Status invoke(OwnerToken owner, InterfaceId family, std::uint32_t permission, bool gameThread, F&& operation) noexcept {
        try {
            const auto status = provider::runtime::authorize(owner, family, permission, gameThread);
            if (status != Status::Ok) return status;
            return operation();
        } catch (...) { provider::runtime::reportBoundaryFailure(owner,family); return Status::InternalError; }
    }
    template<class T> Status checkInput(const T* value) noexcept {
        if (!value) return Status::InvalidArgument;
        if constexpr (requires { value->size; }) if (value->size != sizeof(T)) return Status::InvalidSize;
        if constexpr (requires { value->version; }) if (value->version != 1) return Status::UnsupportedVersion;
        return Status::Ok;
    }
    template<class T> Status checkOutput(T* value) noexcept {
        const auto status = checkInput(value);
        if (status == Status::Ok) *value = {};
        return status;
    }
}
