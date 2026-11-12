#include <ROCK/Animation.h>
#include <ROCK/Collision.h>
#include <ROCK/Configuration.h>
#include <ROCK/Core.h>
#include <ROCK/Grab.h>
#include <ROCK/Hands.h>
#include <ROCK/Input.h>
#include <ROCK/NativeInteraction.h>
#include <ROCK/PlayerController.h>
#include <ROCK/Storage.h>
#include <ROCK/Touch.h>
#include <ROCK/Weapon.h>
#include <ROCK/WeaponParts.h>
#include <array>
#include <intrin.h>
#include "InterfaceNegotiation.h"
#include "OutdatedConsumers.h"

namespace rock::api::core
{
    const Api& table() noexcept;
}

namespace rock::api::hands
{
    const Api& table() noexcept;
}

namespace rock::api::collision
{
    const Api& table() noexcept;
}

namespace rock::api::grab
{
    const Api& table() noexcept;
}

namespace rock::api::touch
{
    const Api& table() noexcept;
}

namespace rock::api::weapon
{
    const Api& table() noexcept;
}

namespace rock::api::weaponparts
{
    const Api& table() noexcept;
}

namespace rock::api::animation
{
    const Api& table() noexcept;
}

namespace rock::api::input
{
    const Api& table() noexcept;
}

namespace rock::api::playercontroller
{
    const Api& table() noexcept;
}

namespace rock::api::configuration
{
    const Api& table() noexcept;
}

namespace rock::api::storage
{
    const Api& table() noexcept;
}

namespace rock::native_interaction_service
{
    const api::native_interaction::Api& table() noexcept;
}

namespace rock::api::discovery
{
    std::span<const RegisteredInterface> registeredInterfaces() noexcept
    {
        static const std::array entries{
            registration(core::table(), core::kSupportedPermissions),
            registration(hands::table(), hands::kSupportedPermissions),
            registration(collision::table(), collision::kSupportedPermissions),
            registration(grab::table(), grab::kSupportedPermissions),
            registration(touch::table(), touch::kSupportedPermissions),
            registration(weapon::table(), weapon::kSupportedPermissions),
            registration(weaponparts::table(), weaponparts::kSupportedPermissions),
            registration(animation::table(), animation::kSupportedPermissions),
            registration(input::table(), input::kSupportedPermissions),
            registration(playercontroller::table(), playercontroller::kSupportedPermissions),
            registration(configuration::table(), configuration::kSupportedPermissions),
            registration(native_interaction_service::table(), native_interaction::kSupportedPermissions),
            registration(storage::table(), storage::kSupportedPermissions),
        };
        return entries;
    }
}

extern "C" __declspec(dllexport) rock::api::Status ROCK_CALL ROCKAPI_QueryInterfaceV1(rock::api::InterfaceId id,
    std::uint32_t exactMajor, std::uint32_t minimumMinor, std::uint32_t minimumTableBytes,
    const rock::api::InterfaceDescriptorV1** output) noexcept
{
    const auto status = rock::api::discovery::query(rock::api::discovery::registeredInterfaces(), id, exactMajor,
        minimumMinor, minimumTableBytes, output);
    if (status != rock::api::Status::Ok)
        rock::api::discovery::noteRefusal(status, id, exactMajor, _ReturnAddress());
    return status;
}
