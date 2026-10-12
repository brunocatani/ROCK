#pragma once

#include "physics-interaction/weapon/WeaponClassificationPolicy.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include <array>

namespace rock::weapon_instance_classification
{
    struct WeaponKeywordFormEntry
    {
        std::uint32_t formId;
        WeaponKeywordFlag flag;
    };

    inline constexpr WeaponKeywordFormEntry kWeaponKeywordForms[] = {
        { 0x0004A0A0, WeaponKeywordFlag::Pistol },
        { 0x0004A0A1, WeaponKeywordFlag::Rifle },
        { 0x00226454, WeaponKeywordFlag::Shotgun },
        { 0x00226455, WeaponKeywordFlag::AssaultRifle },
        { 0x001E325D, WeaponKeywordFlag::Sniper },
        { 0x00226456, WeaponKeywordFlag::GaussRifle },
        { 0x00226452, WeaponKeywordFlag::LaserMusket },
        { 0x0004A0A3, WeaponKeywordFlag::HeavyGun },
        { 0x00226453, WeaponKeywordFlag::HandToHand },
        { 0x0004A0A4, WeaponKeywordFlag::Melee1H },
        { 0x0004A0A5, WeaponKeywordFlag::Melee2H },
        { 0x0005240E, WeaponKeywordFlag::Unarmed },
        { 0x0022575D, WeaponKeywordFlag::Minigun },
        { 0x0022575C, WeaponKeywordFlag::Fatman },
        { 0x0022575B, WeaponKeywordFlag::MissileLauncher },
        { 0x0022575E, WeaponKeywordFlag::GatlingLaser },
        { 0x00225760, WeaponKeywordFlag::Flamer },
        { 0x0022575F, WeaponKeywordFlag::Cryolater },
        { 0x00225763, WeaponKeywordFlag::JunkJet },
        { 0x00225764, WeaponKeywordFlag::RailwayRifle },
        { 0x00225766, WeaponKeywordFlag::Broadsider },
        { 0x00225765, WeaponKeywordFlag::Syringer },
        { 0x00225761, WeaponKeywordFlag::FlareGun },
        { 0x00225762, WeaponKeywordFlag::GammaGun },
        { 0x0016968B, WeaponKeywordFlag::AlienBlaster },
        { 0x00225767, WeaponKeywordFlag::Ripper },
        { 0x00225768, WeaponKeywordFlag::Shishkebab },
        { 0x00092A84, WeaponKeywordFlag::Laser },
        { 0x00092A85, WeaponKeywordFlag::Plasma },
        { 0x00092A86, WeaponKeywordFlag::Ballistic },
        { 0x0004A0A6, WeaponKeywordFlag::Thrown },
        { 0x0010C415, WeaponKeywordFlag::Grenade },
        { 0x0010C414, WeaponKeywordFlag::Mine },
        { 0x0004C922, WeaponKeywordFlag::Explosive },
        { 0x0004A0A2, WeaponKeywordFlag::Automatic },
    };

    struct ResolvedWeaponKeywordEntry
    {
        const RE::BGSKeyword* keyword{ nullptr };
        WeaponKeywordFlag flag{ WeaponKeywordFlag::None };
    };

    inline const std::array<ResolvedWeaponKeywordEntry, std::size(kWeaponKeywordForms)>& resolvedWeaponKeywordForms()
    {
        /*
         * Lazily resolved on first use (function-local static, thread-safe
         * magic-static init) because RE::TESForm::GetFormByID requires the
         * game's form table to be populated, which is not guaranteed at
         * static-initialization time. No static initialization-order
         * dependency: this runs on first equipped-weapon identity read,
         * well after data load.
         */
        static const std::array<ResolvedWeaponKeywordEntry, std::size(kWeaponKeywordForms)> resolved = [] {
            std::array<ResolvedWeaponKeywordEntry, std::size(kWeaponKeywordForms)> table{};
            for (std::size_t i = 0; i < std::size(kWeaponKeywordForms); ++i) {
                table[i].keyword = RE::TESForm::GetFormByID<RE::BGSKeyword>(kWeaponKeywordForms[i].formId);
                table[i].flag = kWeaponKeywordForms[i].flag;
            }
            return table;
        }();
        return resolved;
    }

    inline std::uint64_t computeWeaponKeywordFlags(
        const RE::TESObjectWEAP* weapon,
        const RE::TBO_InstanceData* instanceData)
    {
        std::uint64_t flags = 0;
        if (!weapon) {
            return flags;
        }
        for (const auto& entry : resolvedWeaponKeywordForms()) {
            if (entry.keyword && weapon->HasKeyword(entry.keyword, instanceData)) {
                flags |= static_cast<std::uint64_t>(entry.flag);
            }
        }
        return flags;
    }

    struct WeaponClassificationResult
    {
        WeaponSizeClass sizeClass{ WeaponSizeClass::Rifle };
        WeaponClassificationSource source{ WeaponClassificationSource::None };
        std::uint64_t keywordFlags{ 0 };
        bool usedEffectiveInstanceKeywordData{ false };
        bool resolved{ false };
    };

    /*
     * FO4VR 1.2.72 BGSKeywordForm::HasKeyword at 0x140147F50 selects
     * TBO_InstanceData::GetKeywordData() when available. This reads the
     * engine-assembled keyword set after installed OMOD property changes.
     * Native instance weapon data resolves melee weapons. The effective
     * equip slot only disambiguates conflicting pistol and rifle tags.
     * Inconclusive weapons remain unclassified.
     */
    inline WeaponClassificationResult classifyEquippedWeapon(
        const RE::TESObjectWEAP* weapon,
        const RE::TBO_InstanceData* instanceData,
        const std::uint32_t effectiveEquipSlotFormID)
    {
        WeaponClassificationResult result{};
        if (!weapon) {
            return result;
        }

        const auto* effectiveData = instanceData ?
            static_cast<const RE::TESObjectWEAP::InstanceData*>(instanceData) :
            static_cast<const RE::TESObjectWEAP::InstanceData*>(&weapon->weaponData);
        result.keywordFlags = computeWeaponKeywordFlags(weapon, instanceData);
        const auto policyResult = weapon_classification_policy::classify({
            .keywordFlags = result.keywordFlags,
            .effectiveEquipSlotFormID = effectiveEquipSlotFormID,
            .nativeMeleeType = effectiveData &&
                (effectiveData->type == RE::WEAPON_TYPE::kHandToHand ||
                 weapon_type_policy::isMelee(effectiveData->type.get())),
        });
        result.sizeClass = policyResult.sizeClass;
        result.source = policyResult.source;
        result.resolved = policyResult.resolved;
        result.usedEffectiveInstanceKeywordData =
            result.keywordFlags != 0 && instanceData &&
            instanceData->GetKeywordData();
        return result;
    }

}
