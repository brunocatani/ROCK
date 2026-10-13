#pragma once
#include <cstdint>
#include <ROCK/Storage.h>
#include "physics-interaction/native/HavokPhysicsTiming.h"
namespace RE { class hknpWorld; }
namespace rock { class PhysicsInteraction; struct PhysicsFrameContext; class DynamicHandCollisionRuntime; class PhysicsCallbackQuiescenceGate; }
namespace rock::storage_interaction
{
    void update(PhysicsInteraction&,const PhysicsFrameContext&,const DynamicHandCollisionRuntime&,PhysicsCallbackQuiescenceGate&);
    void flush(RE::hknpWorld*,const havok_physics_timing::PhysicsTimingSample&);
    void shutdown(RE::hknpWorld* validWorld);
    bool blocksShoulder(bool isLeft);
    bool allowsPair(RE::hknpWorld*,std::uint32_t a,std::uint32_t b) noexcept;
    void contact(RE::hknpWorld*,std::uint32_t otherBody,bool isLeft) noexcept;
    // Storage displays and shoulder routing.
    api::Status ROCK_CALL setDisplay(api::OwnerToken,const api::storage::Display*) noexcept;
    api::Status ROCK_CALL clearDisplay(api::OwnerToken,std::uint64_t displayId) noexcept;
    api::Status ROCK_CALL getDisplayContact(api::OwnerToken,api::Hand,api::storage::Contact*) noexcept;
    api::Status ROCK_CALL getAuthoredGrip(api::OwnerToken,const api::storage::GripRequest*,api::storage::GripPose*) noexcept;
    api::Status ROCK_CALL setShoulderRouting(api::OwnerToken,const api::storage::ShoulderRouting*) noexcept;
    api::Status ROCK_CALL clearDisplaysAndRouting(api::OwnerToken) noexcept;
}
