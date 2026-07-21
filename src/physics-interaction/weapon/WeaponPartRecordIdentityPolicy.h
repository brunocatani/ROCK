#pragma once

#include <cstdint>
#include <string_view>

#include "physics-interaction/weapon/WeaponSemantics.h"
#include "physics-interaction/weapon/WeaponTypes.h"

/*
 * Structure/record-authored part classification. NIF mesh names are author
 * discretion and misclassify modded parts (a "PMag" drum is not a "magazine"
 * token). Two structural signals in the assembled FO4VR weapon tree outrank
 * them, verified against in-game OMOD-DUMP evidence (AK-104, G18, MP7, M4A1,
 * Colt 605 - see docs/research/2026-07-03-omod-part-identity-plan.md):
 *
 *  - Slot anchors: connect-point nodes named "P-<slot>" host exactly the
 *    model subtree of the OMOD installed in that slot (P-Mag -> the magazine
 *    OMOD's mesh). The game's mod-attachment system depends on these names,
 *    so they are functional, not decorative.
 *  - Rig anchors: engine-animated weapon rig nodes (WeaponBolt,
 *    WeaponMagazine/WeaponMagazineChildN) that the game moves during
 *    fire/reload. Parts under them share that function.
 *
 * The nearest anchor above a mesh decides; slot anchors override name tokens
 * (except the receiver catch-all and action-named parts), rig anchors only
 * fill gaps where the name matched nothing gameplay-critical.
 */
namespace rock::weapon_part_record_identity_policy
{
    enum class StructureAnchor : std::uint8_t
    {
        None = 0,
        SlotMagazine,
        SlotBarrel,
        SlotMuzzle,
        SlotSight,
        SlotRearFurniture,
        SlotPistolGrip,
        SlotHandguard,
        SlotBipod,
        SlotReceiver,
        RigBolt,
        RigMagazineDisplay,
    };

    /*
     * Fallout4.esm attach-point keyword FormIDs for the vanilla slots,
     * observed directly in the equipped-instance OMOD records of multiple
     * weapons (vanilla and modded weapons share these for standard slots).
     */
    inline constexpr std::uint32_t kAttachPointMagazine = 0x0005D4D7;
    inline constexpr std::uint32_t kAttachPointBarrel = 0x0002249D;
    inline constexpr std::uint32_t kAttachPointMuzzle = 0x0002249C;
    inline constexpr std::uint32_t kAttachPointSight = 0x00022499;
    inline constexpr std::uint32_t kAttachPointGripStock = 0x0002249F;
    inline constexpr std::uint32_t kAttachPointReceiver = 0x00024004;

    [[nodiscard]] inline constexpr std::string_view canonicalConnectPointForAttachPoint(std::uint32_t attachPointFormId) noexcept
    {
        switch (attachPointFormId) {
        case kAttachPointMagazine:
            return "P-Mag";
        case kAttachPointBarrel:
            return "P-Barrel";
        case kAttachPointMuzzle:
            return "P-Muzzle";
        case kAttachPointSight:
            return "P-Scope";
        case kAttachPointGripStock:
            return "P-Grip";
        case kAttachPointReceiver:
            return "P-Receiver";
        default:
            return {};
        }
    }

    [[nodiscard]] inline constexpr std::uint32_t recoveryProviderAttachPointForAttachPoint(
        std::uint32_t attachPointFormId) noexcept
    {
        switch (attachPointFormId) {
        case kAttachPointMuzzle:
            // The barrel model owns P-Muzzle metadata.
            return kAttachPointBarrel;
        case kAttachPointMagazine:
        case kAttachPointBarrel:
        case kAttachPointSight:
        case kAttachPointGripStock:
            // Standard top-level slots are authored by the receiver model.
            return kAttachPointReceiver;
        default:
            return 0;
        }
    }

    [[nodiscard]] inline constexpr std::uint8_t recoveryDependencyRank(std::uint32_t attachPointFormId) noexcept
    {
        if (attachPointFormId == kAttachPointReceiver) {
            return 0;
        }
        if (attachPointFormId == kAttachPointMuzzle) {
            return 2;
        }
        return 1;
    }

    [[nodiscard]] inline constexpr StructureAnchor resolveStructureAnchor(std::string_view nodeName)
    {
        // Exact matches for the CK-standard connect points observed in
        // assembled trees. "P-Scope"/"P-Muzzle" follow the same standard.
        // Bipod slots are mod-added but retain a consistent functional Bipod
        // token in current runtime evidence; other unknown "P-*" names still
        // resolve None so the name classifier keeps authority there.
        if (nodeName == "P-Mag") {
            return StructureAnchor::SlotMagazine;
        }
        if (nodeName == "P-Barrel") {
            return StructureAnchor::SlotBarrel;
        }
        if (nodeName == "P-Muzzle" || nodeName == "P-Compensator" || nodeName == "P-Suppressor") {
            return StructureAnchor::SlotMuzzle;
        }
        if (nodeName == "P-Scope" || nodeName == "P-Sights") {
            return StructureAnchor::SlotSight;
        }
        if (nodeName == "P-Grip" || nodeName == "P-Stock") {
            return StructureAnchor::SlotRearFurniture;
        }
        if (nodeName == "P-PGrip") {
            return StructureAnchor::SlotPistolGrip;
        }
        if (nodeName == "P-Handguard" || nodeName == "P-HandguardU") {
            return StructureAnchor::SlotHandguard;
        }
        if (nodeName == "P-Bipod" ||
            (nodeName.starts_with("P-") &&
                (nodeName.find("Bipod") != std::string_view::npos ||
                    nodeName.find("bipod") != std::string_view::npos ||
                    nodeName.find("BIPOD") != std::string_view::npos))) {
            return StructureAnchor::SlotBipod;
        }
        if (nodeName == "P-Receiver") {
            return StructureAnchor::SlotReceiver;
        }
        if (nodeName == "WeaponBolt") {
            return StructureAnchor::RigBolt;
        }
        if (nodeName == "WeaponMagazine" || nodeName.starts_with("WeaponMagazineChild")) {
            return StructureAnchor::RigMagazineDisplay;
        }
        return StructureAnchor::None;
    }

    [[nodiscard]] inline constexpr std::uint32_t attachPointFormIdForAnchor(StructureAnchor anchor)
    {
        switch (anchor) {
        case StructureAnchor::SlotMagazine:
            return kAttachPointMagazine;
        case StructureAnchor::SlotBarrel:
            return kAttachPointBarrel;
        case StructureAnchor::SlotMuzzle:
            return kAttachPointMuzzle;
        case StructureAnchor::SlotSight:
            return kAttachPointSight;
        case StructureAnchor::SlotRearFurniture:
            return kAttachPointGripStock;
        case StructureAnchor::SlotReceiver:
            return kAttachPointReceiver;
        // Handguard/pistol-grip slots are mod-added attach points on the
        // weapons that carry them; no vanilla keyword exists for pairing.
        case StructureAnchor::SlotPistolGrip:
        case StructureAnchor::SlotHandguard:
        case StructureAnchor::SlotBipod:
        default:
            return 0;
        }
    }

    [[nodiscard]] inline constexpr StructureAnchor chooseStructureAnchor(
        StructureAnchor slotAnchor,
        StructureAnchor rigAnchor) noexcept
    {
        if (slotAnchor == StructureAnchor::None) {
            return rigAnchor;
        }
        // P-Receiver is the catch-all owner of the assembled weapon. A nearer
        // engine rig is more specific there; dedicated attachment slots such
        // as P-Mag remain authoritative over their internal animation rigs.
        if (slotAnchor == StructureAnchor::SlotReceiver && rigAnchor != StructureAnchor::None) {
            return rigAnchor;
        }
        return slotAnchor;
    }

    [[nodiscard]] inline WeaponPartKind partKindForAnchor(StructureAnchor anchor)
    {
        switch (anchor) {
        case StructureAnchor::SlotMagazine:
            return WeaponPartKind::Magazine;
        case StructureAnchor::SlotBarrel:
            return WeaponPartKind::Barrel;
        case StructureAnchor::SlotMuzzle:
            return WeaponPartKind::MuzzleDevice;
        case StructureAnchor::SlotSight:
            return WeaponPartKind::Sight;
        case StructureAnchor::SlotRearFurniture:
            return WeaponPartKind::Stock;
        case StructureAnchor::SlotPistolGrip:
            return WeaponPartKind::Grip;
        case StructureAnchor::SlotHandguard:
            return WeaponPartKind::Handguard;
        case StructureAnchor::SlotBipod:
            return WeaponPartKind::Bipod;
        case StructureAnchor::SlotReceiver:
            return WeaponPartKind::Receiver;
        case StructureAnchor::RigBolt:
            return WeaponPartKind::Bolt;
        case StructureAnchor::RigMagazineDisplay:
            return WeaponPartKind::CosmeticAmmo;
        case StructureAnchor::None:
        default:
            return WeaponPartKind::Other;
        }
    }

    /*
     * Slot anchors carry OMOD ownership and always override name tokens,
     * with two exceptions: parts whose name matched an action role keep it (a
     * pump inside the handguard slot must stay a pump; a slide under the bolt
     * rig must stay a slide), and the receiver slot is the catch-all ancestor
     * of nearly everything, so it only fills gaps. Rig anchors likewise only
     * fill gaps where the name classifier found nothing gameplay-critical.
     */
    [[nodiscard]] inline constexpr bool isAmmoPieceKind(WeaponPartKind kind) noexcept
    {
        return kind == WeaponPartKind::Shell ||
               kind == WeaponPartKind::Round ||
               kind == WeaponPartKind::LaserCell ||
               kind == WeaponPartKind::CosmeticAmmo;
    }

    [[nodiscard]] inline constexpr bool anchorOverridesName(
        StructureAnchor anchor,
        WeaponPartKind namePartKind,
        bool nameHasActionRole,
        bool nameGameplayCritical)
    {
        switch (anchor) {
        case StructureAnchor::SlotMagazine:
            // Magazine rigs frequently place visible cartridges and follower
            // meshes below the same P-Mag owner. Preserve explicit ammunition
            // identities while allowing an otherwise unnamed shell mesh to
            // inherit the physical magazine slot.
            return !nameHasActionRole && !isAmmoPieceKind(namePartKind);
        case StructureAnchor::SlotBarrel:
        case StructureAnchor::SlotMuzzle:
        case StructureAnchor::SlotSight:
        case StructureAnchor::SlotRearFurniture:
        case StructureAnchor::SlotPistolGrip:
        case StructureAnchor::SlotHandguard:
            return !nameHasActionRole;
        case StructureAnchor::SlotBipod:
            // A dedicated authored bipod connect point is stronger evidence
            // than incidental action words inside exported mesh names.
            return true;
        case StructureAnchor::SlotReceiver:
        case StructureAnchor::RigBolt:
        case StructureAnchor::RigMagazineDisplay:
            return !nameGameplayCritical;
        case StructureAnchor::None:
        default:
            return false;
        }
    }

    [[nodiscard]] inline WeaponPartClassification applyStructureAnchor(
        const WeaponPartClassification& nameResult,
        StructureAnchor anchor)
    {
        if (!anchorOverridesName(anchor, nameResult.partKind, nameResult.actionRole != WeaponActionRole::None, nameResult.gameplayCritical)) {
            return nameResult;
        }

        WeaponPartClassification result = classifyWeaponPartKind(partKindForAnchor(anchor));
        result.classificationSource =
            (anchor == StructureAnchor::RigBolt || anchor == StructureAnchor::RigMagazineDisplay) ?
                WeaponPartClassificationSource::RigAnchor :
                WeaponPartClassificationSource::SlotAnchor;
        result.attachPointFormId = attachPointFormIdForAnchor(anchor);
        return result;
    }
}
