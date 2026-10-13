#pragma once
#include <ROCK/WeaponPartsV1_2.h>
namespace RE { class hknpWorld; }
namespace rock { class PhysicsInteraction; struct PhysicsFrameContext; }
namespace rock::weapon_part_collision {
    api::Status set(api::OwnerToken,const api::weaponparts::v1_2::CollisionSuppression&);
    api::Status clear(api::OwnerToken);
    void update(PhysicsInteraction&,const PhysicsFrameContext&);
    void shutdown(RE::hknpWorld* validWorld);
}
