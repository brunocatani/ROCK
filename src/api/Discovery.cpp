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

extern "C" __declspec(dllexport) rock::api::Status ROCK_CALL ROCKAPI_QueryInterfaceV1(
    rock::api::InterfaceId id, std::uint32_t exactMajor, std::uint32_t minimumMinor,
    std::uint32_t minimumTableBytes, const rock::api::InterfaceDescriptorV1** output) noexcept {
    using namespace rock::api;
    static const std::array descriptors{
        InterfaceDescriptorV1{sizeof(InterfaceDescriptorV1),InterfaceId::Core,1,0,sizeof(core::ApiV1),1,0,0,&core::table()},
        InterfaceDescriptorV1{sizeof(InterfaceDescriptorV1),InterfaceId::Hands,1,0,sizeof(hands::ApiV1),1,0,0,&hands::table()},
        InterfaceDescriptorV1{sizeof(InterfaceDescriptorV1),InterfaceId::Collision,1,0,sizeof(collision::ApiV1),1,0,0,&collision::table()},
        InterfaceDescriptorV1{sizeof(InterfaceDescriptorV1),InterfaceId::Grab,1,0,sizeof(grab::ApiV1),1,0,0,&grab::table()},
        InterfaceDescriptorV1{sizeof(InterfaceDescriptorV1),InterfaceId::Touch,1,0,sizeof(touch::ApiV1),1,0,0,&touch::table()},
        InterfaceDescriptorV1{sizeof(InterfaceDescriptorV1),InterfaceId::Weapon,1,0,sizeof(weapon::ApiV1),1,0,0,&weapon::table()},
        InterfaceDescriptorV1{sizeof(InterfaceDescriptorV1),InterfaceId::WeaponParts,1,0,sizeof(weaponparts::ApiV1),1,0,0,&weaponparts::table()},
        InterfaceDescriptorV1{sizeof(InterfaceDescriptorV1),InterfaceId::Animation,1,0,sizeof(animation::ApiV1),1,0,0,&animation::table()},
        InterfaceDescriptorV1{sizeof(InterfaceDescriptorV1),InterfaceId::Input,1,0,sizeof(input::ApiV1),1,0,0,&input::table()},
        InterfaceDescriptorV1{sizeof(InterfaceDescriptorV1),InterfaceId::References,1,0,sizeof(references::ApiV1),1,0,0,&references::table()},
        InterfaceDescriptorV1{sizeof(InterfaceDescriptorV1),InterfaceId::PlayerController,1,0,sizeof(playercontroller::ApiV1),1,0,0,&playercontroller::table()},
        InterfaceDescriptorV1{sizeof(InterfaceDescriptorV1),InterfaceId::Diagnostics,1,0,sizeof(diagnostics::ApiV1),1,0,0,&diagnostics::table()},
        InterfaceDescriptorV1{sizeof(InterfaceDescriptorV1),InterfaceId::Configuration,1,0,sizeof(configuration::ApiV1),1,0,0,&configuration::table()},
    };
    return discovery::query(descriptors,id,exactMajor,minimumMinor,minimumTableBytes,output);
}
