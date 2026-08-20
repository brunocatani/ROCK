#define ROCK_API_EXPORTS
/*
 * Build the DLL side of the public provider ABI.
 *
 * WEAPON PART VALIDATION: matcher and semantic checks for consumer-supplied weapon
 * part targets, plus the conversion into the ROCK runtime target type.
 *
 * Pure functions over public structs. They fail closed: an unparseable or
 * non-finite target is rejected, never clamped into something plausible.
 */
#include "api/detail/ProviderWeaponPartValidation.h"

#include "api/detail/ProviderTransformMath.h"

#include <cstring>

namespace rock::provider::detail
{
    constexpr std::uint32_t kWeaponPartTargetMatcherFlagsV1 =
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchBodyId) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceRoot) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceName) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchPartKind) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchReloadRole) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSupportRole) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSocketRole) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchActionRole);
    constexpr std::uint32_t kImplementedWeaponPartTargetFlagsV1 =
        kWeaponPartTargetMatcherFlagsV1 |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::NonExclusive);
    constexpr std::uint32_t kImplementedWeaponPartDriveMatcherFlagsV1 =
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchBodyId) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceRoot) |
        static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceName);
    bool hasValidWeaponPartMatcher(std::uint32_t flags, std::uint32_t bodyId, std::uintptr_t sourceRoot, const char* sourceName)
    {
        // NonExclusive is a semantics flag, not a matcher: at least one match
        // flag must still be present for the target to select anything.
        if ((flags & ~kImplementedWeaponPartTargetFlagsV1) != 0 || (flags & kWeaponPartTargetMatcherFlagsV1) == 0) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchBodyId)) != 0 && bodyId == kProviderInvalidBodyId) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceRoot)) != 0 && sourceRoot == 0) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceName)) != 0 &&
            boundedStringLength(sourceName, ROCK_PROVIDER_MAX_EVIDENCE_NAME) == 0) {
            return false;
        }
        return true;
    }

    bool hasConcreteWeaponPartDriveMatcher(std::uint32_t flags, std::uint32_t bodyId, std::uintptr_t sourceRoot, const char* sourceName)
    {
        if ((flags & ~kImplementedWeaponPartDriveMatcherFlagsV1) != 0 || (flags & kImplementedWeaponPartDriveMatcherFlagsV1) == 0) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchBodyId)) != 0 && bodyId == kProviderInvalidBodyId) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceRoot)) != 0 && sourceRoot == 0) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceName)) != 0 &&
            boundedStringLength(sourceName, ROCK_PROVIDER_MAX_EVIDENCE_NAME) == 0) {
            return false;
        }
        return ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceRoot)) != 0 && sourceRoot != 0) ||
               ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchBodyId)) != 0 && bodyId != kProviderInvalidBodyId) ||
               ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSourceName)) != 0 &&
                   boundedStringLength(sourceName, ROCK_PROVIDER_MAX_EVIDENCE_NAME) != 0);
    }

    bool isValidWeaponPartKindValue(std::uint32_t value)
    {
        return value < static_cast<std::uint32_t>(WeaponPartKind::Count);
    }

    bool isValidWeaponReloadRoleValue(std::uint32_t value)
    {
        return value <= static_cast<std::uint32_t>(WeaponReloadRole::Receiver);
    }

    bool isValidWeaponSupportRoleValue(std::uint32_t value)
    {
        return value <= static_cast<std::uint32_t>(WeaponSupportGripRole::ReceiverSupport);
    }

    bool isValidWeaponSocketRoleValue(std::uint32_t value)
    {
        return value <= static_cast<std::uint32_t>(WeaponSocketRole::LoadingGate);
    }

    bool isValidWeaponActionRoleValue(std::uint32_t value)
    {
        return value <= static_cast<std::uint32_t>(WeaponActionRole::Latch);
    }

    bool hasValidWeaponPartTargetSemantics(const RockProviderWeaponPartTargetV1& target)
    {
        const auto flags = target.flags;
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchPartKind)) != 0 &&
            !isValidWeaponPartKindValue(target.partKind)) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchReloadRole)) != 0 &&
            !isValidWeaponReloadRoleValue(target.reloadRole)) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSupportRole)) != 0 &&
            !isValidWeaponSupportRoleValue(target.supportRole)) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchSocketRole)) != 0 &&
            !isValidWeaponSocketRoleValue(target.socketRole)) {
            return false;
        }
        if ((flags & static_cast<std::uint32_t>(RockProviderWeaponPartTargetFlagV1::MatchActionRole)) != 0 &&
            !isValidWeaponActionRoleValue(target.actionRole)) {
            return false;
        }
        return true;
    }

    bool isValidWeaponPartGrabMode(RockProviderWeaponPartGrabModeV1 mode)
    {
        return mode == RockProviderWeaponPartGrabModeV1::FullTwoHandAuthority ||
               mode == RockProviderWeaponPartGrabModeV1::AttachOnly;
    }

    bool isValidWeaponPartDriveSpace(RockProviderWeaponPartDriveSpaceV1 space)
    {
        return space == RockProviderWeaponPartDriveSpaceV1::WeaponRootLocal ||
               space == RockProviderWeaponPartDriveSpaceV1::SourceParentLocal;
    }

    weapon_part_runtime::GrabMode toRuntimeGrabMode(RockProviderWeaponPartGrabModeV1 mode)
    {
        switch (mode) {
        case RockProviderWeaponPartGrabModeV1::FullTwoHandAuthority:
            return weapon_part_runtime::GrabMode::FullTwoHandAuthority;
        case RockProviderWeaponPartGrabModeV1::AttachOnly:
            return weapon_part_runtime::GrabMode::AttachOnly;
        case RockProviderWeaponPartGrabModeV1::None:
        default:
            return weapon_part_runtime::GrabMode::None;
        }
    }

    RockProviderWeaponPartGrabModeV1 fromRuntimeGrabMode(weapon_part_runtime::GrabMode mode)
    {
        switch (mode) {
        case weapon_part_runtime::GrabMode::FullTwoHandAuthority:
            return RockProviderWeaponPartGrabModeV1::FullTwoHandAuthority;
        case weapon_part_runtime::GrabMode::AttachOnly:
            return RockProviderWeaponPartGrabModeV1::AttachOnly;
        case weapon_part_runtime::GrabMode::None:
        default:
            return RockProviderWeaponPartGrabModeV1::None;
        }
    }

    weapon_part_runtime::Target toRuntimeTarget(const WeaponPartTargetSlot& slot)
    {
        weapon_part_runtime::Target target{};
        if (!slot.active) {
            return target;
        }

        target.active = true;
        target.ownerToken = slot.ownerToken;
        target.weaponGenerationKey = slot.target.weaponGenerationKey;
        target.flags = slot.target.flags;
        target.grabMode = toRuntimeGrabMode(slot.target.grabMode);
        target.bodyId = slot.target.bodyId;
        target.sourceRoot = slot.target.sourceRoot;
        std::memcpy(target.sourceName.data(), slot.target.sourceName, target.sourceName.size());
        target.sourceName[target.sourceName.size() - 1] = '\0';
        target.partKind = static_cast<WeaponPartKind>(slot.target.partKind);
        target.reloadRole = static_cast<WeaponReloadRole>(slot.target.reloadRole);
        target.supportRole = static_cast<WeaponSupportGripRole>(slot.target.supportRole);
        target.socketRole = static_cast<WeaponSocketRole>(slot.target.socketRole);
        target.actionRole = static_cast<WeaponActionRole>(slot.target.actionRole);
        target.groupId = slot.target.groupId;
        target.priority = slot.target.priority;
        return target;
    }
}
