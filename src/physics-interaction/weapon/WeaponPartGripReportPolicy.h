#pragma once

#include <cstdint>

#include "physics-interaction/weapon/WeaponPartRuntime.h"

/*
 * Pure classification for what a physical hand currently holds on the equipped
 * weapon. This is the single source of truth for two consumers:
 *  - the provider API per-hand grip report (PAPER-facing), and
 *  - part-carry authority accounting (an AttachOnly glue must never count as a
 *    carry anchor, become the carry pivot, or keep the weapon from dropping).
 */
namespace rock::weapon_part_grip_report_policy
{
    enum class HandGripKind : std::uint32_t
    {
        None = 0,
        FiringGrip = 1,
        SupportFullAuthority = 2,
        SupportVisualOnly = 3,
        PartCarry = 4,
        AttachOnly = 5,
    };

    /*
     * A part grip carries weapon transform authority only when it is not an
     * AttachOnly glue. Carry authority decides pivot selection, two-anchor
     * solve participation, drop-on-last-release, and stash carry-hand
     * eligibility.
     */
    [[nodiscard]] inline constexpr bool partGripCountsAsCarry(bool active, bool attachOnly) noexcept
    {
        return active && !attachOnly;
    }

    [[nodiscard]] inline constexpr bool providerGrabModeIsAttachOnly(bool providerAuthorityActive, std::uint32_t grabMode) noexcept
    {
        return providerAuthorityActive &&
               grabMode == static_cast<std::uint32_t>(weapon_part_runtime::GrabMode::AttachOnly);
    }

    [[nodiscard]] inline constexpr HandGripKind resolveHandGripKind(
        bool grippingActive,
        bool partCarryActive,
        bool primaryOnlyActive,
        bool isFiringHand,
        bool partGripActive,
        bool partGripAttachOnly,
        bool visualOnlyAuthority) noexcept
    {
        // The firing hand owns the firing grip in Gripping/PrimaryOnly; it can
        // only hold a part grip while detached (PartCarry).
        if (isFiringHand && (grippingActive || primaryOnlyActive)) {
            return HandGripKind::FiringGrip;
        }
        if (!partGripActive) {
            return HandGripKind::None;
        }
        if (partGripAttachOnly) {
            return HandGripKind::AttachOnly;
        }
        if (partCarryActive) {
            return HandGripKind::PartCarry;
        }
        if (grippingActive) {
            return visualOnlyAuthority ? HandGripKind::SupportVisualOnly : HandGripKind::SupportFullAuthority;
        }
        return HandGripKind::None;
    }
}
