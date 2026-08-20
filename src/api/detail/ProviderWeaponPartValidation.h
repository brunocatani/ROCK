#pragma once

#include "api/detail/ProviderApiState.h"
#include "physics-interaction/weapon/parts/WeaponPartRuntime.h"

namespace rock::provider::detail
{
    [[nodiscard]] bool hasValidWeaponPartMatcher(
        std::uint32_t flags,
        std::uint32_t bodyId,
        std::uintptr_t sourceRoot,
        const char* sourceName);
    [[nodiscard]] bool hasConcreteWeaponPartDriveMatcher(
        std::uint32_t flags,
        std::uint32_t bodyId,
        std::uintptr_t sourceRoot,
        const char* sourceName);
    [[nodiscard]] bool isValidWeaponPartKindValue(std::uint32_t value);
    [[nodiscard]] bool isValidWeaponReloadRoleValue(std::uint32_t value);
    [[nodiscard]] bool isValidWeaponSupportRoleValue(std::uint32_t value);
    [[nodiscard]] bool isValidWeaponSocketRoleValue(std::uint32_t value);
    [[nodiscard]] bool isValidWeaponActionRoleValue(std::uint32_t value);
    [[nodiscard]] bool hasValidWeaponPartTargetSemantics(
        const RockProviderWeaponPartTargetV1& target);
    [[nodiscard]] bool isValidWeaponPartGrabMode(
        RockProviderWeaponPartGrabModeV1 mode);
    [[nodiscard]] bool isValidWeaponPartDriveSpace(
        RockProviderWeaponPartDriveSpaceV1 space);
    [[nodiscard]] weapon_part_runtime::GrabMode toRuntimeGrabMode(
        RockProviderWeaponPartGrabModeV1 mode);
    [[nodiscard]] RockProviderWeaponPartGrabModeV1 fromRuntimeGrabMode(
        weapon_part_runtime::GrabMode mode);
    [[nodiscard]] weapon_part_runtime::Target toRuntimeTarget(
        const WeaponPartTargetSlot& slot);
}
