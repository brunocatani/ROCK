#pragma once

#include <ROCK/Hands.h>
#include <ROCK/Collision.h>
#include <ROCK/Weapon.h>
#include <ROCK/WeaponParts.h>

// Copied publications only. These endpoints authorize observation from tasks
// without granting access to live scene objects or gameplay mutation.
namespace rock::api::hands {
    Status ROCK_CALL getHandFrameV1(OwnerToken, Hand, HandFrameV1*) noexcept;
    Status ROCK_CALL getHeadPose(OwnerToken, HeadPoseV1*) noexcept;
    Status ROCK_CALL getRoles(OwnerToken, RolesV1*) noexcept;
}
namespace rock::api::collision {
    Status ROCK_CALL getEnvironment(OwnerToken, EnvironmentV1*) noexcept;
}
namespace rock::api::weapon {
    Status ROCK_CALL getEquippedWeaponStateV1(OwnerToken, EquippedWeaponStateV1*) noexcept;
}
namespace rock::api::weaponparts {
    Status ROCK_CALL getWeaponPartGripStateV1(OwnerToken, Hand, WeaponPartGripStateV1*) noexcept;
}
