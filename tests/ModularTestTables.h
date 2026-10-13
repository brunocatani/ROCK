#pragma once
// The production catalog is linked below. Empty tables avoid native runtime
// startup; discovery tests inspect their addresses and real SDK table extents.
#include <ROCK/Core.h>
#include <ROCK/Hands.h>
#include <ROCK/Collision.h>
#include <ROCK/Grab.h>
#include <ROCK/GrabV1_2.h>
#include <ROCK/GrabV1_3.h>
#include <ROCK/Touch.h>
#include <ROCK/Touch.h>
#include <ROCK/Weapon.h>
#include <ROCK/WeaponV1_1.h>
#include <ROCK/WeaponParts.h>
#include <ROCK/WeaponPartsV1_2.h>
#include <ROCK/WeaponPartsV1_7.h>
#include <ROCK/Animation.h>
#include <ROCK/Input.h>
#include <ROCK/InputV1_1.h>
#include <ROCK/InputV1_2.h>
#include <ROCK/InputV1_3.h>
#include <ROCK/InputV1_4.h>
#include <ROCK/InputV1_5.h>
#include <ROCK/References.h>
#include <ROCK/PlayerController.h>
#include <ROCK/Diagnostics.h>
#include <ROCK/Configuration.h>
namespace rock::api::core { const Api& table() noexcept; }
namespace rock::api::grab { const v1_3::Api& tableV1_3() noexcept; }
namespace rock::api::hands { const Api& table() noexcept; }
namespace rock::api::collision { const ApiV1& table() noexcept; }
namespace rock::api::grab { const ApiV1& table() noexcept; const v1_1::Api& tableV1_1() noexcept; const v1_2::Api& tableV1_2() noexcept; }
namespace rock::api::touch { const Api& table() noexcept; }
namespace rock::api::weapon { const ApiV1& table() noexcept; const v1_1::Api& tableV1_1() noexcept; }
namespace rock::api::weaponparts { const ApiV1& table() noexcept; const v1_2::Api& tableV1_2() noexcept; const v1_3::Api& tableV1_3() noexcept; const v1_4::Api& tableV1_4() noexcept; const v1_5::Api& tableV1_5() noexcept; const v1_6::Api& tableV1_6() noexcept; }
namespace rock::api::weaponparts { const v1_7::Api& tableV1_7() noexcept; }
namespace rock::api::animation { const Api& table() noexcept; }
namespace rock::api::input { const v1_5::Api& tableV1_5() noexcept; const ApiV1& table() noexcept; const v1_1::Api& tableV1_1() noexcept; const v1_2::Api& tableV1_2() noexcept; const v1_3::Api& tableV1_3() noexcept; const v1_4::Api& tableV1_4() noexcept; }
namespace rock::api::references { const ApiV1& table() noexcept; }
namespace rock::api::playercontroller { const Api& table() noexcept; }
namespace rock::api::diagnostics { const ApiV1& table() noexcept; }
namespace rock::api::configuration { const Api& table() noexcept; }
extern "C" rock::api::Status ROCK_CALL ROCKAPI_QueryInterfaceV1(
    rock::api::InterfaceId,std::uint32_t,std::uint32_t,std::uint32_t,
    const rock::api::InterfaceDescriptorV1**) noexcept;
#include <ROCK/NativeInteractionV1_1.h>
namespace rock::native_interaction_service { const api::native_interaction::ApiV1& table() noexcept; const api::native_interaction::v1_1::Api& tableV1_1() noexcept; }
