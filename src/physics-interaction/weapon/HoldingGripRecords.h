#pragma once
#include "physics-interaction/weapon/HoldingGripAdapter.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include <filesystem>
#include <string>
#include <vector>
namespace rock::holding_grips {
struct Record {std::string weapon,gripNode;bool left{},powerArmor{};RE::NiTransform wrist;authored_weapon_grip_library::FiringFingerPose fingers;};
void load(const std::filesystem::path& directory);
const Record* find(const RE::TESObjectWEAP* weapon,const RE::NiAVObject* root,bool left,std::uint64_t generation,bool powerArmor);
}
