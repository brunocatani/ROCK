#pragma once
#include "physics-interaction/grab/GrabEvent.h"
#include <ROCK/Core.h>
#include <ROCK/Collision.h>
#include <ROCK/Grab.h>
#include <ROCK/Touch.h>
#include <ROCK/Weapon.h>
#include <ROCK/WeaponParts.h>
#include <ROCK/Animation.h>
#include <ROCK/Input.h>
namespace rock::provider::events {
    api::Status bind(api::OwnerToken,api::InterfaceId);
    void remove(api::OwnerToken);
    // Each family's event, stamped by the caller; recipient 0 reaches every bound owner.
    void publish(const api::core::Event&, api::OwnerToken recipient = 0);
    void publish(const api::grab::Event&, api::OwnerToken recipient = 0);
    void publish(const api::weapon::Event&, api::OwnerToken recipient = 0);
    void publish(const api::weaponparts::Event&, api::OwnerToken recipient = 0);
    void publish(const api::animation::Event&, api::OwnerToken recipient = 0);
    void publish(const api::input::Event&, api::OwnerToken recipient = 0);
    void publishGrab(const rock::GrabEventData&, const api::Sample&);
    void publishPhysics(std::uint32_t kind,bool left,std::uint32_t form,std::uint32_t layer,const api::Sample&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::core::Event*,std::uint32_t,api::Stream&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::collision::Event*,std::uint32_t,api::Stream&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::grab::Event*,std::uint32_t,api::Stream&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::touch::Event*,std::uint32_t,api::Stream&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::weapon::Event*,std::uint32_t,api::Stream&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::weaponparts::Event*,std::uint32_t,api::Stream&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::animation::Event*,std::uint32_t,api::Stream&);
    api::Status copy(api::OwnerToken,std::uint64_t,api::input::Event*,std::uint32_t,api::Stream&);
    bool inSynchronousCallback() noexcept;
    api::Status setGrabCallback(api::OwnerToken,api::grab::EventCallback,void*);
    void clearGrabCallback(api::OwnerToken);
    void publishTouch(std::uint64_t owner,std::uint64_t scope,const ::rock::api::touch::TouchGrabState&,std::uint64_t frame);
}
