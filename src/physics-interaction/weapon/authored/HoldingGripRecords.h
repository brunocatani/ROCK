// Holding-grip records: per-weapon hand poses that override the firing seat
// (see HoldingGripRecords.cpp).
#pragma once
#include <filesystem>
#include <string>
#include <vector>
#include "physics-interaction/weapon/authored/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/authored/HoldingGripAdapter.h"

namespace rock::holding_grips
{
    struct Record
    {
        std::string weapon, gripNode;
        bool left{}, powerArmor{};
        RE::NiTransform wrist;
        authored_weapon_grip_library::FiringFingerPose fingers;
    };

    void load(const std::filesystem::path& directory);
    const Record* find(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* root, bool left, std::uint64_t generation,
        bool powerArmor);
}
