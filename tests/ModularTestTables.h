#pragma once
// The production catalog is linked below. Empty tables avoid native runtime
// startup; discovery tests inspect their addresses and real SDK table extents.
#include <ROCK/Core.h>
#include <ROCK/Hands.h>
#include <ROCK/Collision.h>
#include <ROCK/Grab.h>
#include <ROCK/Touch.h>
#include <ROCK/Weapon.h>
#include <ROCK/WeaponV1_1.h>
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
namespace rock::api::weapon { const ApiV1& table() noexcept; const v1_1::Api& tableV1_1() noexcept; }
namespace rock::api::weaponparts { const ApiV1& table() noexcept; }
namespace rock::api::animation { const ApiV1& table() noexcept; }
namespace rock::api::input { const ApiV1& table() noexcept; }
namespace rock::api::references { const ApiV1& table() noexcept; }
namespace rock::api::playercontroller { const ApiV1& table() noexcept; }
namespace rock::api::diagnostics { const ApiV1& table() noexcept; }
namespace rock::api::configuration { const ApiV1& table() noexcept; }
extern "C" rock::api::Status ROCK_CALL ROCKAPI_QueryInterfaceV1(
    rock::api::InterfaceId,std::uint32_t,std::uint32_t,std::uint32_t,
    const rock::api::InterfaceDescriptorV1**) noexcept;
