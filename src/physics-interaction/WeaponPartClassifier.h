#pragma once

#include "WeaponSemanticTypes.h"

#include <algorithm>
#include <cctype>
#include <string_view>

namespace frik::rock
{
    /*
     * The generated mesh path sees weapon parts through FO4VR NIF node names,
     * not authored ROCK data yet. This classifier is deliberately conservative:
     * it only assigns high-value semantic roles when names match common firearm
     * part terms, leaving unknown geometry as Other instead of forcing reload or
     * grip behavior onto decorative mesh.
     */

    inline bool weaponPartNameContains(std::string_view haystack, std::string_view needle)
    {
        if (needle.empty() || haystack.size() < needle.size()) {
            return false;
        }

        return std::search(haystack.begin(), haystack.end(), needle.begin(), needle.end(), [](char lhs, char rhs) {
            return std::tolower(static_cast<unsigned char>(lhs)) == std::tolower(static_cast<unsigned char>(rhs));
        }) != haystack.end();
    }

    inline WeaponPartClassification classifyWeaponPartKind(WeaponPartKind kind)
    {
        WeaponPartClassification result{};
        result.partKind = kind;

        switch (kind) {
        case WeaponPartKind::Foregrip:
            result.supportGripRole = WeaponSupportGripRole::Foregrip;
            result.fallbackGripPose = WeaponGripPoseId::VerticalForegrip;
            result.priority = 95;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Pump:
            result.supportGripRole = WeaponSupportGripRole::PumpGrip;
            result.actionRole = WeaponActionRole::Pump;
            result.fallbackGripPose = WeaponGripPoseId::PumpGrip;
            result.priority = 92;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Handguard:
            result.supportGripRole = WeaponSupportGripRole::SupportSurface;
            result.fallbackGripPose = WeaponGripPoseId::HandguardClamp;
            result.priority = 88;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Barrel:
            result.supportGripRole = WeaponSupportGripRole::SupportSurface;
            result.fallbackGripPose = WeaponGripPoseId::BarrelWrap;
            result.priority = 84;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Magwell:
            result.supportGripRole = WeaponSupportGripRole::MagwellHold;
            result.socketRole = WeaponSocketRole::Magwell;
            result.fallbackGripPose = WeaponGripPoseId::MagwellHold;
            result.priority = 82;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Magazine:
            result.reloadRole = WeaponReloadRole::MagazineBody;
            result.priority = 80;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Bolt:
            result.actionRole = WeaponActionRole::Bolt;
            result.priority = 78;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Slide:
            result.actionRole = WeaponActionRole::Slide;
            result.priority = 78;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::ChargingHandle:
            result.actionRole = WeaponActionRole::ChargingHandle;
            result.priority = 78;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::BreakAction:
            result.actionRole = WeaponActionRole::BreakAction;
            result.priority = 76;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Cylinder:
            result.actionRole = WeaponActionRole::Cylinder;
            result.socketRole = WeaponSocketRole::Cylinder;
            result.priority = 76;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Chamber:
            result.socketRole = WeaponSocketRole::Chamber;
            result.priority = 74;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::LaserCell:
            result.socketRole = WeaponSocketRole::LaserCell;
            result.reloadRole = WeaponReloadRole::AmmoPiece;
            result.priority = 74;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Lever:
            result.actionRole = WeaponActionRole::Lever;
            result.socketRole = WeaponSocketRole::LoadingGate;
            result.priority = 72;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Stock:
            result.supportGripRole = WeaponSupportGripRole::StockForward;
            result.fallbackGripPose = WeaponGripPoseId::ReceiverSupport;
            result.priority = 66;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Receiver:
            result.reloadRole = WeaponReloadRole::Receiver;
            result.supportGripRole = WeaponSupportGripRole::ReceiverSupport;
            result.fallbackGripPose = WeaponGripPoseId::ReceiverSupport;
            result.priority = 62;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Grip:
            result.priority = 56;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Shell:
        case WeaponPartKind::Round:
            result.reloadRole = WeaponReloadRole::AmmoPiece;
            result.priority = 46;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Sight:
        case WeaponPartKind::Accessory:
            result.priority = 28;
            break;
        case WeaponPartKind::CosmeticAmmo:
            result.reloadRole = WeaponReloadRole::CosmeticAmmo;
            result.cosmetic = true;
            result.priority = 12;
            break;
        case WeaponPartKind::Other:
        default:
            result.priority = 10;
            break;
        }

        return result;
    }

    inline WeaponPartClassification classifyWeaponPartName(std::string_view sourceName)
    {
        if (weaponPartNameContains(sourceName, "magwell")) {
            return classifyWeaponPartKind(WeaponPartKind::Magwell);
        }
        if (weaponPartNameContains(sourceName, "foregrip") || weaponPartNameContains(sourceName, "verticalgrip") ||
            weaponPartNameContains(sourceName, "angledgrip")) {
            return classifyWeaponPartKind(WeaponPartKind::Foregrip);
        }
        if (weaponPartNameContains(sourceName, "handguard") || weaponPartNameContains(sourceName, "forearm")) {
            return classifyWeaponPartKind(WeaponPartKind::Handguard);
        }
        if (weaponPartNameContains(sourceName, "pump")) {
            return classifyWeaponPartKind(WeaponPartKind::Pump);
        }
        if (weaponPartNameContains(sourceName, "magazine") || weaponPartNameContains(sourceName, " clip")) {
            return classifyWeaponPartKind(WeaponPartKind::Magazine);
        }
        if (weaponPartNameContains(sourceName, "charging")) {
            return classifyWeaponPartKind(WeaponPartKind::ChargingHandle);
        }
        if (weaponPartNameContains(sourceName, "bolt")) {
            return classifyWeaponPartKind(WeaponPartKind::Bolt);
        }
        if (weaponPartNameContains(sourceName, "slide")) {
            return classifyWeaponPartKind(WeaponPartKind::Slide);
        }
        if (weaponPartNameContains(sourceName, "break") || weaponPartNameContains(sourceName, "hinge")) {
            return classifyWeaponPartKind(WeaponPartKind::BreakAction);
        }
        if (weaponPartNameContains(sourceName, "cylinder")) {
            return classifyWeaponPartKind(WeaponPartKind::Cylinder);
        }
        if (weaponPartNameContains(sourceName, "chamber")) {
            return classifyWeaponPartKind(WeaponPartKind::Chamber);
        }
        if (weaponPartNameContains(sourceName, "fusioncell") || weaponPartNameContains(sourceName, "fusion cell") ||
            weaponPartNameContains(sourceName, "lasercell") || weaponPartNameContains(sourceName, "battery")) {
            return classifyWeaponPartKind(WeaponPartKind::LaserCell);
        }
        if (weaponPartNameContains(sourceName, "lever")) {
            return classifyWeaponPartKind(WeaponPartKind::Lever);
        }
        if (weaponPartNameContains(sourceName, "stock") || weaponPartNameContains(sourceName, "butt")) {
            return classifyWeaponPartKind(WeaponPartKind::Stock);
        }
        if (weaponPartNameContains(sourceName, "barrel") || weaponPartNameContains(sourceName, "muzzle") ||
            weaponPartNameContains(sourceName, "suppressor") || weaponPartNameContains(sourceName, "silencer")) {
            return classifyWeaponPartKind(WeaponPartKind::Barrel);
        }
        if (weaponPartNameContains(sourceName, "shell")) {
            return classifyWeaponPartKind(WeaponPartKind::Shell);
        }
        if (weaponPartNameContains(sourceName, "round") || weaponPartNameContains(sourceName, "cartridge")) {
            return classifyWeaponPartKind(WeaponPartKind::Round);
        }
        if (weaponPartNameContains(sourceName, "casing") || weaponPartNameContains(sourceName, "bullet") ||
            weaponPartNameContains(sourceName, "ammo")) {
            return classifyWeaponPartKind(WeaponPartKind::CosmeticAmmo);
        }
        if (weaponPartNameContains(sourceName, "sight") || weaponPartNameContains(sourceName, "scope")) {
            return classifyWeaponPartKind(WeaponPartKind::Sight);
        }
        if (weaponPartNameContains(sourceName, "accessory") || weaponPartNameContains(sourceName, "rail")) {
            return classifyWeaponPartKind(WeaponPartKind::Accessory);
        }
        if (weaponPartNameContains(sourceName, "receiver") || weaponPartNameContains(sourceName, "frame") ||
            weaponPartNameContains(sourceName, "body") || weaponPartNameContains(sourceName, "rifle") ||
            weaponPartNameContains(sourceName, "pistol") || weaponPartNameContains(sourceName, "weapon")) {
            return classifyWeaponPartKind(WeaponPartKind::Receiver);
        }
        if (weaponPartNameContains(sourceName, "grip") || weaponPartNameContains(sourceName, "handle")) {
            return classifyWeaponPartKind(WeaponPartKind::Grip);
        }

        return classifyWeaponPartKind(WeaponPartKind::Other);
    }
}
