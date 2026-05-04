#pragma once

#include "WeaponSemanticTypes.h"

namespace frik::rock::weapon_support_grip_policy
{
    /*
     * HIGGS gates two-handing by whether the held item is two-handable and by
     * the hand's live contact with the equipped weapon, not by a narrow list of
     * approved mesh part names. PAPER owns semantic reload/action routing
     * through the provider API; ROCK's support grip remains a generic equipped-weapon contact so
     * stocks, magazines, accessories, unknown NIF chunks, and melee blades can
     * all become the locked support point the player actually touched.
     */

    inline WeaponGripPoseId fallbackPoseForPart(WeaponPartKind partKind)
    {
        switch (partKind) {
        case WeaponPartKind::Foregrip:
            return WeaponGripPoseId::VerticalForegrip;
        case WeaponPartKind::Pump:
            return WeaponGripPoseId::PumpGrip;
        case WeaponPartKind::Handguard:
            return WeaponGripPoseId::HandguardClamp;
        case WeaponPartKind::Barrel:
            return WeaponGripPoseId::BarrelWrap;
        case WeaponPartKind::Magazine:
        case WeaponPartKind::Magwell:
            return WeaponGripPoseId::MagwellHold;
        case WeaponPartKind::Stock:
        case WeaponPartKind::Receiver:
        case WeaponPartKind::Grip:
        case WeaponPartKind::Bolt:
        case WeaponPartKind::Slide:
        case WeaponPartKind::ChargingHandle:
        case WeaponPartKind::BreakAction:
        case WeaponPartKind::Cylinder:
        case WeaponPartKind::Chamber:
        case WeaponPartKind::LaserCell:
        case WeaponPartKind::Lever:
        case WeaponPartKind::Sight:
        case WeaponPartKind::Accessory:
            return WeaponGripPoseId::ReceiverSupport;
        case WeaponPartKind::Shell:
        case WeaponPartKind::Round:
        case WeaponPartKind::CosmeticAmmo:
        case WeaponPartKind::Other:
        default:
            return WeaponGripPoseId::BarrelWrap;
        }
    }

    inline WeaponGripPoseId resolveSupportGripPose(const WeaponInteractionContact& contact)
    {
        return contact.fallbackGripPose != WeaponGripPoseId::None ? contact.fallbackGripPose : fallbackPoseForPart(contact.partKind);
    }

    inline bool canUseContactForSupportGrip(const WeaponInteractionContact& contact, const WeaponInteractionRuntimeState& runtimeState)
    {
        return contact.valid && runtimeState.supportGripAllowed;
    }
}
