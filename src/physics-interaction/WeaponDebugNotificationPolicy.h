#pragma once

#include "WeaponInteractionRouter.h"

#include <cstdio>
#include <cstdint>
#include <string>

namespace frik::rock::weapon_debug_notification_policy
{
    /*
     * Weapon debug notifications are intentionally split from the live
     * PhysicsInteraction loop. The runtime loop owns contact sampling and UI
     * dispatch, while this pure policy owns naming, message shape, and
     * edge-trigger suppression. That keeps troubleshooting output useful in VR
     * without coupling HUD spam control to HIGGS-style weapon authority logic.
     */

    enum class WeaponContactSource : std::uint8_t
    {
        None,
        Contact,
        Probe
    };

    enum class WeaponGripNotificationEvent : std::uint8_t
    {
        None,
        Started,
        Ended
    };

    struct WeaponNotificationKey
    {
        bool valid{ false };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        WeaponPartKind partKind{ WeaponPartKind::Other };
        WeaponInteractionKind interactionKind{ WeaponInteractionKind::None };
        WeaponGripPoseId gripPose{ WeaponGripPoseId::None };
        WeaponContactSource source{ WeaponContactSource::None };

        bool operator==(const WeaponNotificationKey& rhs) const
        {
            return valid == rhs.valid && bodyId == rhs.bodyId && partKind == rhs.partKind && interactionKind == rhs.interactionKind &&
                   gripPose == rhs.gripPose && source == rhs.source;
        }

        bool operator!=(const WeaponNotificationKey& rhs) const { return !(*this == rhs); }
    };

    struct WeaponNotificationState
    {
        WeaponNotificationKey lastContact{};
        bool hasLastContact{ false };
        bool supportGripActive{ false };
    };

    inline const char* nameOf(WeaponContactSource source)
    {
        switch (source) {
        case WeaponContactSource::Contact:
            return "contact";
        case WeaponContactSource::Probe:
            return "probe";
        case WeaponContactSource::None:
        default:
            return "none";
        }
    }

    inline const char* nameOf(WeaponPartKind kind)
    {
        switch (kind) {
        case WeaponPartKind::Receiver:
            return "Receiver";
        case WeaponPartKind::Barrel:
            return "Barrel";
        case WeaponPartKind::Handguard:
            return "Handguard";
        case WeaponPartKind::Foregrip:
            return "Foregrip";
        case WeaponPartKind::Pump:
            return "Pump";
        case WeaponPartKind::Stock:
            return "Stock";
        case WeaponPartKind::Grip:
            return "Grip";
        case WeaponPartKind::Magazine:
            return "Magazine";
        case WeaponPartKind::Magwell:
            return "Magwell";
        case WeaponPartKind::Bolt:
            return "Bolt";
        case WeaponPartKind::Slide:
            return "Slide";
        case WeaponPartKind::ChargingHandle:
            return "ChargingHandle";
        case WeaponPartKind::BreakAction:
            return "BreakAction";
        case WeaponPartKind::Cylinder:
            return "Cylinder";
        case WeaponPartKind::Chamber:
            return "Chamber";
        case WeaponPartKind::Shell:
            return "Shell";
        case WeaponPartKind::Round:
            return "Round";
        case WeaponPartKind::LaserCell:
            return "LaserCell";
        case WeaponPartKind::Lever:
            return "Lever";
        case WeaponPartKind::Sight:
            return "Sight";
        case WeaponPartKind::Accessory:
            return "Accessory";
        case WeaponPartKind::CosmeticAmmo:
            return "CosmeticAmmo";
        case WeaponPartKind::Other:
        default:
            return "Other";
        }
    }

    inline const char* nameOf(WeaponInteractionKind kind)
    {
        switch (kind) {
        case WeaponInteractionKind::SupportGrip:
            return "SupportGrip";
        case WeaponInteractionKind::PassiveTouch:
            return "PassiveTouch";
        case WeaponInteractionKind::None:
        default:
            return "None";
        }
    }

    inline const char* nameOf(WeaponGripPoseId pose)
    {
        switch (pose) {
        case WeaponGripPoseId::BarrelWrap:
            return "BarrelWrap";
        case WeaponGripPoseId::HandguardClamp:
            return "HandguardClamp";
        case WeaponGripPoseId::VerticalForegrip:
            return "VerticalForegrip";
        case WeaponGripPoseId::AngledForegrip:
            return "AngledForegrip";
        case WeaponGripPoseId::PumpGrip:
            return "PumpGrip";
        case WeaponGripPoseId::MagwellHold:
            return "MagwellHold";
        case WeaponGripPoseId::ReceiverSupport:
            return "ReceiverSupport";
        case WeaponGripPoseId::None:
        default:
            return "None";
        }
    }

    inline const char* debugTextOrUnknown(const std::string& text)
    {
        return text.empty() ? "(unknown)" : text.c_str();
    }

    inline std::string formatFormId(std::uint32_t formId)
    {
        char buffer[9]{};
        std::snprintf(buffer, sizeof(buffer), "%08X", formId);
        return buffer;
    }

    inline WeaponNotificationKey makeWeaponNotificationKey(
        const WeaponInteractionContact& contact,
        const WeaponInteractionDecision& decision,
        WeaponContactSource source)
    {
        WeaponNotificationKey key{};
        key.valid = contact.valid && decision.kind != WeaponInteractionKind::None;
        key.bodyId = decision.bodyId;
        key.partKind = decision.partKind;
        key.interactionKind = decision.kind;
        key.gripPose = decision.gripPose;
        key.source = source;
        return key;
    }

    inline bool shouldNotifyWeaponContact(WeaponNotificationState& state, const WeaponNotificationKey& key)
    {
        if (!key.valid) {
            state.hasLastContact = false;
            state.lastContact = {};
            return false;
        }

        if (!state.hasLastContact || state.lastContact != key) {
            state.lastContact = key;
            state.hasLastContact = true;
            return true;
        }

        return false;
    }

    inline WeaponGripNotificationEvent observeWeaponSupportGrip(WeaponNotificationState& state, bool active)
    {
        if (active == state.supportGripActive) {
            return WeaponGripNotificationEvent::None;
        }

        state.supportGripActive = active;
        return active ? WeaponGripNotificationEvent::Started : WeaponGripNotificationEvent::Ended;
    }

    inline std::string formatWeaponTouchNotification(const WeaponNotificationKey& key)
    {
        return std::string("[ROCK] WPN TOUCH src=") + nameOf(key.source) + " part=" + nameOf(key.partKind) + " route=" + nameOf(key.interactionKind) +
               " pose=" + nameOf(key.gripPose) + " body=" + std::to_string(key.bodyId);
    }

    inline std::string formatWeaponGripNotification(WeaponGripNotificationEvent event, const WeaponNotificationKey& key, const WeaponInteractionDebugInfo& debugInfo)
    {
        if (event == WeaponGripNotificationEvent::Ended) {
            return "[ROCK] WPN GRIP END";
        }
        if (event == WeaponGripNotificationEvent::Started) {
            return std::string("[ROCK] WPN GRIP START weapon='") + debugTextOrUnknown(debugInfo.weaponName) + "' form=" + formatFormId(debugInfo.weaponFormId) +
                   " node='" + debugTextOrUnknown(debugInfo.weaponNodeName) + "' root='" + debugTextOrUnknown(debugInfo.sourceRootName) + "' nif='" +
                   debugTextOrUnknown(debugInfo.sourceName) + "' part=" + nameOf(key.partKind) + " route=" + nameOf(key.interactionKind) + " pose=" +
                   nameOf(key.gripPose) + " body=" + std::to_string(key.bodyId);
        }

        return {};
    }

    inline std::string formatWeaponGripNotification(WeaponGripNotificationEvent event, const WeaponNotificationKey& key)
    {
        return formatWeaponGripNotification(event, key, WeaponInteractionDebugInfo{});
    }
}
