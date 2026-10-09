#include "InterfaceNegotiation.h"
#include <array>
#include <ROCK/Core.h>
#include <ROCK/Hands.h>
#include <ROCK/Collision.h>
#include <ROCK/Grab.h>
#include <ROCK/GrabV1_3.h>
#include <ROCK/Touch.h>
#include <ROCK/TouchV1_1.h>
#include <ROCK/Weapon.h>
#include <ROCK/WeaponV1_1.h>
#include <ROCK/WeaponPartsV1_6.h>
#include <ROCK/Animation.h>
#include <ROCK/Input.h>
#include <ROCK/InputV1_1.h>
#include <ROCK/InputV1_2.h>
#include <ROCK/InputV1_5.h>
#include <ROCK/References.h>
#include <ROCK/PlayerController.h>
#include <ROCK/Diagnostics.h>
#include <ROCK/Configuration.h>

namespace rock::api::core { const ApiV1& table() noexcept; }
namespace rock::api::hands { const ApiV1& table() noexcept; }
namespace rock::api::collision { const ApiV1& table() noexcept; }
namespace rock::api::grab { const v1_3::Api& tableV1_3() noexcept; }
namespace rock::api::touch { const v1_1::Api& tableV1_1() noexcept; }
namespace rock::api::weapon { const ApiV1& table() noexcept; }
namespace rock::api::weapon { const v1_1::Api& tableV1_1() noexcept; }
namespace rock::api::weaponparts { const v1_6::Api& tableV1_6() noexcept; }
namespace rock::api::animation { const ApiV1& table() noexcept; }
namespace rock::api::input { const ApiV1& table() noexcept; }
namespace rock::api::input { const v1_1::Api& tableV1_1() noexcept; }
namespace rock::api::input { const v1_2::Api& tableV1_2() noexcept; }
namespace rock::api::input { const v1_5::Api& tableV1_5() noexcept; }
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
        registration(grab::tableV1_3(), grab::kSupportedPermissions),
        registration(touch::tableV1_1(), touch::kSupportedPermissions),
        registration(weapon::tableV1_1(), weapon::kSupportedPermissions),
        registration(weaponparts::tableV1_6(), weaponparts::kSupportedPermissions),
        registration(animation::table(), animation::kSupportedPermissions),
        registration(input::tableV1_5(), input::kSupportedPermissions),
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
