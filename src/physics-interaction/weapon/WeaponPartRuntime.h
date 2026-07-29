#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "physics-interaction/weapon/WeaponTypes.h"

namespace rock::weapon_part_runtime
{
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;
    inline constexpr std::size_t kMaxSourceName = 64;

    enum class GrabMode : std::uint32_t
    {
        None = 0,
        FullTwoHandAuthority = 1,
        AttachOnly = 2,
    };

    enum TargetFlag : std::uint32_t
    {
        MatchBodyId = 1u << 0,
        MatchSourceRoot = 1u << 1,
        MatchSourceName = 1u << 2,
        MatchPartKind = 1u << 3,
        MatchReloadRole = 1u << 4,
        MatchSupportRole = 1u << 5,
        MatchSocketRole = 1u << 6,
        MatchActionRole = 1u << 7,
        /*
         * A non-exclusive target grants its grab mode on match but does not
         * activate whitelist gating: contacts that match no target keep their
         * normal grip behavior instead of being blocked. Exclusive targets
         * (flag absent) preserve the original session semantics where an
         * active whitelist rejects every unmatched part grip.
         */
        NonExclusive = 1u << 8,
    };

    inline constexpr std::uint32_t kMatcherTargetFlags =
        MatchBodyId |
        MatchSourceRoot |
        MatchSourceName |
        MatchPartKind |
        MatchReloadRole |
        MatchSupportRole |
        MatchSocketRole |
        MatchActionRole;

    inline constexpr std::uint32_t kImplementedTargetFlags = kMatcherTargetFlags | NonExclusive;

    struct Contact
    {
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ kInvalidBodyId };
        std::uintptr_t sourceRoot{ 0 };
        std::string_view sourceName{};
        WeaponPartKind partKind{ WeaponPartKind::Other };
        WeaponReloadRole reloadRole{ WeaponReloadRole::None };
        WeaponSupportGripRole supportRole{ WeaponSupportGripRole::None };
        WeaponSocketRole socketRole{ WeaponSocketRole::None };
        WeaponActionRole actionRole{ WeaponActionRole::None };
    };

    struct Target
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t flags{ 0 };
        GrabMode grabMode{ GrabMode::None };
        std::uint32_t bodyId{ kInvalidBodyId };
        std::uintptr_t sourceRoot{ 0 };
        std::array<char, kMaxSourceName> sourceName{};
        WeaponPartKind partKind{ WeaponPartKind::Other };
        WeaponReloadRole reloadRole{ WeaponReloadRole::None };
        WeaponSupportGripRole supportRole{ WeaponSupportGripRole::None };
        WeaponSocketRole socketRole{ WeaponSocketRole::None };
        WeaponActionRole actionRole{ WeaponActionRole::None };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
    };

    struct Resolution
    {
        bool whitelistActive{ false };
        bool matched{ false };
        GrabMode grabMode{ GrabMode::None };
        std::uint64_t ownerToken{ 0 };
        std::uint32_t groupId{ 0 };
        std::uint32_t priority{ 0 };
    };

    inline std::size_t boundedStringLength(const char* value, std::size_t capacity)
    {
        if (!value) {
            return 0;
        }

        for (std::size_t i = 0; i < capacity; ++i) {
            if (value[i] == '\0') {
                return i;
            }
        }
        return capacity;
    }

    inline std::string_view fixedStringView(const std::array<char, kMaxSourceName>& value)
    {
        return std::string_view(value.data(), boundedStringLength(value.data(), value.size()));
    }

    inline bool targetAppliesToGeneration(const Target& target, std::uint64_t weaponGenerationKey)
    {
        return target.active &&
               target.ownerToken != 0 &&
               target.grabMode != GrabMode::None &&
               (target.weaponGenerationKey == 0 || target.weaponGenerationKey == weaponGenerationKey);
    }

    inline bool targetHasUsableMatcher(const Target& target)
    {
        if ((target.flags & ~kImplementedTargetFlags) != 0) {
            return false;
        }
        if ((target.flags & MatchBodyId) != 0 && target.bodyId == kInvalidBodyId) {
            return false;
        }
        if ((target.flags & MatchSourceRoot) != 0 && target.sourceRoot == 0) {
            return false;
        }
        if ((target.flags & MatchSourceName) != 0 && fixedStringView(target.sourceName).empty()) {
            return false;
        }
        return (target.flags & kMatcherTargetFlags) != 0;
    }

    inline bool targetIsExclusive(const Target& target)
    {
        return (target.flags & NonExclusive) == 0;
    }

    inline bool sourceNamesMatch(std::string_view lhs, std::string_view rhs)
    {
        return !lhs.empty() && lhs == rhs;
    }

    inline bool targetMatchesContact(const Target& target, const Contact& contact)
    {
        if (!targetHasUsableMatcher(target)) {
            return false;
        }
        if ((target.flags & MatchBodyId) != 0 && target.bodyId != contact.bodyId) {
            return false;
        }
        if ((target.flags & MatchSourceRoot) != 0 && target.sourceRoot != contact.sourceRoot) {
            return false;
        }
        if ((target.flags & MatchSourceName) != 0 && !sourceNamesMatch(fixedStringView(target.sourceName), contact.sourceName)) {
            return false;
        }
        if ((target.flags & MatchPartKind) != 0 && target.partKind != contact.partKind) {
            return false;
        }
        if ((target.flags & MatchReloadRole) != 0 && target.reloadRole != contact.reloadRole) {
            return false;
        }
        if ((target.flags & MatchSupportRole) != 0 && target.supportRole != contact.supportRole) {
            return false;
        }
        if ((target.flags & MatchSocketRole) != 0 && target.socketRole != contact.socketRole) {
            return false;
        }
        if ((target.flags & MatchActionRole) != 0 && target.actionRole != contact.actionRole) {
            return false;
        }
        return true;
    }

    inline bool preferCandidate(const Target& candidate, const Target* currentBest)
    {
        if (!currentBest) {
            return true;
        }
        if (candidate.priority != currentBest->priority) {
            return candidate.priority > currentBest->priority;
        }
        if (candidate.grabMode != currentBest->grabMode) {
            return candidate.grabMode == GrabMode::FullTwoHandAuthority;
        }
        return candidate.ownerToken < currentBest->ownerToken;
    }

    template <std::size_t Capacity>
    inline Resolution resolveTarget(const std::array<Target, Capacity>& targets, const Contact& contact)
    {
        Resolution result{};
        const Target* best = nullptr;
        for (const auto& target : targets) {
            if (!targetAppliesToGeneration(target, contact.weaponGenerationKey)) {
                continue;
            }
            if (!targetHasUsableMatcher(target)) {
                continue;
            }

            if (targetIsExclusive(target)) {
                result.whitelistActive = true;
            }
            if (!targetMatchesContact(target, contact)) {
                continue;
            }
            if (preferCandidate(target, best)) {
                best = &target;
            }
        }

        if (best) {
            result.matched = true;
            result.grabMode = best->grabMode;
            result.ownerToken = best->ownerToken;
            result.groupId = best->groupId;
            result.priority = best->priority;
        }
        return result;
    }
}
