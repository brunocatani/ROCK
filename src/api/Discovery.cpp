#include "InterfaceNegotiation.h"
#include <array>
#include <ROCK/Core.h>
#include <ROCK/Hands.h>
#include <ROCK/Collision.h>
#include <ROCK/Grab.h>
#include <ROCK/Touch.h>
#include <ROCK/Weapon.h>
#include <ROCK/WeaponParts.h>
#include <ROCK/Animation.h>
#include <ROCK/Input.h>
#include <ROCK/References.h>
#include <ROCK/PlayerController.h>
#include <ROCK/Diagnostics.h>
#include <ROCK/Configuration.h>

namespace rock::api::core { const ApiV1& table() noexcept; }
namespace rock::api::hands { const ApiV1& table() noexcept; }
namespace rock::api::collision { const ApiV1& table() noexcept; }
namespace rock::api::grab { const ApiV1& table() noexcept; }
namespace rock::api::touch { const ApiV1& table() noexcept; }
namespace rock::api::weapon { const ApiV1& table() noexcept; }
namespace rock::api::weaponparts { const ApiV1& table() noexcept; }
namespace rock::api::animation { const ApiV1& table() noexcept; }
namespace rock::api::input { const ApiV1& table() noexcept; }
namespace rock::api::references { const ApiV1& table() noexcept; }
namespace rock::api::playercontroller { const ApiV1& table() noexcept; }
namespace rock::api::diagnostics { const ApiV1& table() noexcept; }
namespace rock::api::configuration { const ApiV1& table() noexcept; }

namespace rock::api::discovery {
std::span<const RegisteredInterface> registeredInterfaces() noexcept {
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
        registration(references::table(), references::kSupportedPermissions),
        registration(playercontroller::table(), playercontroller::kSupportedPermissions),
        registration(diagnostics::table(), diagnostics::kSupportedPermissions),
        registration(configuration::table(), configuration::kSupportedPermissions),
    };
    return entries;
}
}

extern "C" __declspec(dllexport) rock::api::Status ROCK_CALL ROCKAPI_QueryInterfaceV1(
    rock::api::InterfaceId id, std::uint32_t exactMajor, std::uint32_t minimumMinor,
    std::uint32_t minimumTableBytes, const rock::api::InterfaceDescriptorV1** output) noexcept {
    return rock::api::discovery::query(rock::api::discovery::registeredInterfaces(),
        id, exactMajor, minimumMinor, minimumTableBytes, output);
}
