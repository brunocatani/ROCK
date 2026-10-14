#pragma once

#include <ROCK/Hands.h>
#include <ROCK/Collision.h>
#include <ROCK/Weapon.h>
#include <ROCK/WeaponParts.h>

// Copied publications only. These endpoints authorize observation from tasks
// without granting access to live scene objects or gameplay mutation.
namespace rock::api::hands {
    Status ROCK_CALL getHandFrame(OwnerToken, Hand, HandFrame*) noexcept;
    Status ROCK_CALL getHeadPose(OwnerToken, HeadPose*) noexcept;
    Status ROCK_CALL getRoles(OwnerToken, Roles*) noexcept;
}
namespace rock::api::collision {
    Status ROCK_CALL getEnvironment(OwnerToken, Environment*) noexcept;
}
namespace rock::api::weaponparts {
    Status ROCK_CALL getGripState(OwnerToken, Hand, GripState*) noexcept;
}
