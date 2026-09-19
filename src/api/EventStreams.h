#pragma once
#include "ProviderRuntimeTypes.h"
#include "physics-interaction/grab/GrabEvent.h"
#include <ROCK/Core.h>
#include <ROCK/Collision.h>
#include <ROCK/Grab.h>
#include <ROCK/Touch.h>
#include <ROCK/Weapon.h>
#include <ROCK/WeaponParts.h>
#include <ROCK/Animation.h>
#include <ROCK/Input.h>
#include <ROCK/Diagnostics.h>
namespace rock::provider::events {
    api::Status bind(api::OwnerToken,api::InterfaceId);
    void remove(api::OwnerToken);
    void publish(const RockProviderEventV1&);
    void publishGrab(const rock::GrabEventData&, const api::SampleV1&);
    void publishPhysics(std::uint32_t kind,bool left,std::uint32_t form,std::uint32_t layer,const api::SampleV1&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::core::EventV1*,std::uint32_t,api::StreamV1&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::collision::EventV1*,std::uint32_t,api::StreamV1&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::grab::EventV1*,std::uint32_t,api::StreamV1&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::touch::EventV1*,std::uint32_t,api::StreamV1&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::weapon::EventV1*,std::uint32_t,api::StreamV1&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::weaponparts::EventV1*,std::uint32_t,api::StreamV1&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::animation::EventV1*,std::uint32_t,api::StreamV1&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::input::EventV1*,std::uint32_t,api::StreamV1&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::diagnostics::EventV1*,std::uint32_t,api::StreamV1&);
    bool inSynchronousCallback() noexcept;
    api::Status setGrabCallback(api::OwnerToken,api::grab::EventCallbackV1,void*);
    void clearGrabCallback(api::OwnerToken);
    void publishTouch(std::uint64_t owner,std::uint64_t scope,const RockProviderTouchGrabStateV1&,std::uint64_t frame);
}
