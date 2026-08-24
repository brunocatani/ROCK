#pragma once

#include "physics-interaction/weapon/WeaponTypes.h"

#include <cstdint>

namespace rock::weapon_classification_policy
{
    inline constexpr std::uint32_t kRightHandEquipSlotFormID = 0x00013F42u;
    inline constexpr std::uint32_t kBothHandsEquipSlotFormID = 0x00013F45u;

    struct Input
    {
        std::uint64_t keywordFlags{ 0 };
        std::uint32_t effectiveEquipSlotFormID{ 0 };
        bool nativeMeleeType{ false };
    };

    struct Result
    {
        WeaponSizeClass sizeClass{ WeaponSizeClass::Rifle };
        WeaponClassificationSource source{ WeaponClassificationSource::None };
        bool resolved{ false };
    };

    [[nodiscard]] constexpr Result classify(const Input& input) noexcept
    {
        const auto has = [&](const WeaponKeywordFlag flag) {
            return hasWeaponKeywordFlag(input.keywordFlags, flag);
        };

        if (input.nativeMeleeType) {
            return {
                .sizeClass = WeaponSizeClass::Melee,
                .source = WeaponClassificationSource::WeaponData,
                .resolved = true,
            };
        }

        if (has(WeaponKeywordFlag::Melee1H) ||
            has(WeaponKeywordFlag::Melee2H) ||
            has(WeaponKeywordFlag::Unarmed) ||
            has(WeaponKeywordFlag::HandToHand) ||
            has(WeaponKeywordFlag::Ripper) ||
            has(WeaponKeywordFlag::Shishkebab)) {
            return {
                .sizeClass = WeaponSizeClass::Melee,
                .source = WeaponClassificationSource::Keyword,
                .resolved = true,
            };
        }

        if (has(WeaponKeywordFlag::HeavyGun) ||
            has(WeaponKeywordFlag::Minigun) ||
            has(WeaponKeywordFlag::Fatman) ||
            has(WeaponKeywordFlag::MissileLauncher) ||
            has(WeaponKeywordFlag::GatlingLaser) ||
            has(WeaponKeywordFlag::Flamer) ||
            has(WeaponKeywordFlag::Cryolater) ||
            has(WeaponKeywordFlag::JunkJet) ||
            has(WeaponKeywordFlag::Broadsider)) {
            return {
                .sizeClass = WeaponSizeClass::Heavy,
                .source = WeaponClassificationSource::Keyword,
                .resolved = true,
            };
        }

        const bool pistolKeyword =
            has(WeaponKeywordFlag::Pistol) ||
            has(WeaponKeywordFlag::FlareGun) ||
            has(WeaponKeywordFlag::GammaGun) ||
            has(WeaponKeywordFlag::AlienBlaster);
        const bool rifleKeyword =
            has(WeaponKeywordFlag::Rifle) ||
            has(WeaponKeywordFlag::Shotgun) ||
            has(WeaponKeywordFlag::AssaultRifle) ||
            has(WeaponKeywordFlag::Sniper) ||
            has(WeaponKeywordFlag::GaussRifle) ||
            has(WeaponKeywordFlag::LaserMusket) ||
            has(WeaponKeywordFlag::RailwayRifle) ||
            has(WeaponKeywordFlag::Syringer);

        if (pistolKeyword && rifleKeyword) {
            if (input.effectiveEquipSlotFormID == kRightHandEquipSlotFormID) {
                return {
                    .sizeClass = WeaponSizeClass::Pistol,
                    .source = WeaponClassificationSource::EquipSlot,
                    .resolved = true,
                };
            }
            if (input.effectiveEquipSlotFormID == kBothHandsEquipSlotFormID) {
                return {
                    .sizeClass = WeaponSizeClass::Rifle,
                    .source = WeaponClassificationSource::EquipSlot,
                    .resolved = true,
                };
            }
        }

        if (pistolKeyword) {
            return {
                .sizeClass = WeaponSizeClass::Pistol,
                .source = WeaponClassificationSource::Keyword,
                .resolved = true,
            };
        }

        if (rifleKeyword) {
            return {
                .sizeClass = WeaponSizeClass::Rifle,
                .source = WeaponClassificationSource::Keyword,
                .resolved = true,
            };
        }

        return {};
    }
}
