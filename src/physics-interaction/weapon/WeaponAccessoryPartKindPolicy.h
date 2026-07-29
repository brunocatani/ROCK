#pragma once

#include "physics-interaction/weapon/WeaponSemantics.h"
#include "physics-interaction/weapon/WeaponTypes.h"

namespace rock::weapon_accessory_part_kind_policy
{
    /*
     * Record/emitter evidence refines an already-classified physical part.
     * A native-overlay OMOD is always a Scope. Otherwise laser and flashlight
     * capabilities describe the module, while reticle-only optics retain the
     * existing Sight classification.
     */
    struct Evidence
    {
        bool nativeScopeOverlay{ false };
        bool laserEmitter{ false };
        bool flashlightEmitter{ false };
    };

    [[nodiscard]] inline constexpr WeaponPartKind resolvePartKind(
        WeaponPartKind baseKind,
        const Evidence& evidence) noexcept
    {
        if (evidence.nativeScopeOverlay) {
            return WeaponPartKind::Scope;
        }
        if (evidence.laserEmitter && evidence.flashlightEmitter) {
            return WeaponPartKind::LaserFlashlightCombo;
        }
        if (evidence.laserEmitter) {
            return WeaponPartKind::LaserSight;
        }
        if (evidence.flashlightEmitter) {
            return WeaponPartKind::Flashlight;
        }
        return baseKind;
    }

    [[nodiscard]] inline WeaponPartClassification applyAttachmentEvidence(
        const WeaponPartClassification& base,
        const Evidence& evidence)
    {
        const auto resolvedKind = resolvePartKind(base.partKind, evidence);
        if (resolvedKind == base.partKind) {
            return base;
        }

        auto result = classifyWeaponPartKind(resolvedKind);
        result.classificationSource = WeaponPartClassificationSource::AttachmentEvidence;
        result.attachPointFormId = base.attachPointFormId;
        return result;
    }
}
