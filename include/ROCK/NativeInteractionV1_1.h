#pragma once
#include "NativeInteraction.h"
#include <cstddef>

namespace rock::api::native_interaction::v1_1 {
    struct ContactDetails {
        std::uint32_t size{sizeof(ContactDetails)}, valid{};
        // Unit normal of the target surface pointing toward the weapon.
        Point3 normalWorld{};
        std::uint32_t pointCount{};
    };
    struct Api {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=1,minorVersion=1;
        ApiV1 v1;
        // Only during the owner's existing weaponContact callback. Copied
        // narrow-phase manifold details; no pointer outlives the callback.
        // V1 callback layout, emission order and point selection are unchanged.
        Status(ROCK_CALL* readContactDetails)(OwnerToken,ContactDetails*) noexcept;
    };
    static_assert(offsetof(Api,v1)==0&&offsetof(Api,readContactDetails)==sizeof(ApiV1));
    static_assert(sizeof(Api)==23*sizeof(void*)&&sizeof(ContactDetails)==24);
}
