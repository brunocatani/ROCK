#include "InterfaceNegotiation.h"
#include "physics-interaction/PhysicsLog.h"
#include <array>
#include <atomic>
#include <ROCK/Core.h>
#include <ROCK/Hands.h>
#include <ROCK/Collision.h>
#include <ROCK/Grab.h>
#include <ROCK/GrabV1_3.h>
#include <ROCK/Touch.h>
#include <ROCK/Weapon.h>
#include <ROCK/WeaponV1_1.h>
#include <ROCK/WeaponPartsV1_7.h>
#include <ROCK/Animation.h>
#include <ROCK/Input.h>
#include <ROCK/References.h>
#include <ROCK/PlayerController.h>
#include <ROCK/Diagnostics.h>
#include <ROCK/Configuration.h>
#include <ROCK/NativeInteractionV1_1.h>

namespace rock::api::core { const Api& table() noexcept; }
namespace rock::api::hands { const Api& table() noexcept; }
namespace rock::api::collision { const ApiV1& table() noexcept; }
namespace rock::api::grab { const v1_3::Api& tableV1_3() noexcept; }
namespace rock::api::touch { const Api& table() noexcept; }
namespace rock::api::weapon { const ApiV1& table() noexcept; }
namespace rock::api::weapon { const v1_1::Api& tableV1_1() noexcept; }
namespace rock::api::weaponparts { const v1_7::Api& tableV1_7() noexcept; }
namespace rock::api::animation { const Api& table() noexcept; }
namespace rock::api::input { const Api& table() noexcept; }
namespace rock::api::references { const ApiV1& table() noexcept; }
namespace rock::api::playercontroller { const Api& table() noexcept; }
namespace rock::api::diagnostics { const ApiV1& table() noexcept; }
namespace rock::api::configuration { const Api& table() noexcept; }
namespace rock::native_interaction_service { const api::native_interaction::ApiV1& table() noexcept; const api::native_interaction::v1_1::Api& tableV1_1() noexcept; }

namespace rock::api::discovery {
std::span<const RegisteredInterface> registeredInterfaces() noexcept {
    static const std::array entries{
        registration(core::table(), core::kSupportedPermissions),
        registration(hands::table(), hands::kSupportedPermissions),
        registration(collision::table(), collision::kSupportedPermissions),
        registration(grab::tableV1_3(), grab::kSupportedPermissions),
        registration(touch::table(), touch::kSupportedPermissions),
        registration(weapon::tableV1_1(), weapon::kSupportedPermissions),
        registration(weaponparts::tableV1_7(), weaponparts::kSupportedPermissions),
        registration(animation::table(), animation::kSupportedPermissions),
        registration(input::table(), input::kSupportedPermissions),
        registration(references::table(), references::kSupportedPermissions),
        registration(playercontroller::table(), playercontroller::kSupportedPermissions),
        registration(diagnostics::table(), diagnostics::kSupportedPermissions),
        registration(configuration::table(), configuration::kSupportedPermissions),
        registration(native_interaction_service::tableV1_1(), native_interaction::kSupportedPermissions),
    };
    return entries;
}
}

namespace {
    // An outdated consumer usually retries discovery; each refused
    // (interface, major) pair is logged once.
    std::array<std::atomic<std::uint32_t>, 32> s_refusedMajors{};

    void logRefusedMajor(rock::api::InterfaceId id, std::uint32_t major) noexcept {
        const auto index = static_cast<std::uint32_t>(id);
        if (index >= s_refusedMajors.size() || major >= 32) return;
        const auto bit = 1u << major;
        if (s_refusedMajors[index].fetch_or(bit, std::memory_order_relaxed) & bit) return;
        try {
            ROCK_LOG_WARN(Api, "Discovery refused interface {} major {}; this ROCK serves only its current provider majors, so the consumer must be updated",
                index, major);
        } catch (...) {
        }
    }
}

extern "C" __declspec(dllexport) rock::api::Status ROCK_CALL ROCKAPI_QueryInterfaceV1(
    rock::api::InterfaceId id, std::uint32_t exactMajor, std::uint32_t minimumMinor,
    std::uint32_t minimumTableBytes, const rock::api::InterfaceDescriptorV1** output) noexcept {
    const auto status = rock::api::discovery::query(rock::api::discovery::registeredInterfaces(),
        id, exactMajor, minimumMinor, minimumTableBytes, output);
    if (status == rock::api::Status::UnsupportedMajor) logRefusedMajor(id, exactMajor);
    return status;
}
