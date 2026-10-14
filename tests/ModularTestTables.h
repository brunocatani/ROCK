#pragma once
// The production catalog is linked below. Empty tables avoid native runtime
// startup; discovery tests inspect their addresses and real SDK table extents.
#include <ROCK/Core.h>
#include <ROCK/Hands.h>
#include <ROCK/Collision.h>
#include <ROCK/Grab.h>
#include <ROCK/Touch.h>
#include <ROCK/Touch.h>
#include <ROCK/Weapon.h>
#include <ROCK/WeaponParts.h>
#include <ROCK/Animation.h>
#include <ROCK/Input.h>
#include <ROCK/Input.h>
#include <ROCK/PlayerController.h>
#include <ROCK/Configuration.h>
#include <ROCK/Storage.h>
namespace rock::api::core { const Api& table() noexcept; }
namespace rock::api::hands { const Api& table() noexcept; }
namespace rock::api::collision { const Api& table() noexcept; }
namespace rock::api::grab { const Api& table() noexcept; }
namespace rock::api::touch { const Api& table() noexcept; }
namespace rock::api::weapon { const Api& table() noexcept; }
namespace rock::api::weaponparts { const Api& table() noexcept; }
namespace rock::api::animation { const Api& table() noexcept; }
namespace rock::api::input { const Api& table() noexcept; }
namespace rock::api::playercontroller { const Api& table() noexcept; }
namespace rock::api::configuration { const Api& table() noexcept; }
namespace rock::api::storage { const Api& table() noexcept; }
extern "C" rock::api::Status ROCK_CALL ROCKAPI_QueryInterfaceV1(
    rock::api::InterfaceId,std::uint32_t,std::uint32_t,std::uint32_t,
    const rock::api::InterfaceDescriptorV1**) noexcept;
#include <ROCK/NativeInteraction.h>
namespace rock::native_interaction_service { const api::native_interaction::Api& table() noexcept; }
