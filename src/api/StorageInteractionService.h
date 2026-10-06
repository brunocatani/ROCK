#pragma once
#include <cstdint>
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
}
