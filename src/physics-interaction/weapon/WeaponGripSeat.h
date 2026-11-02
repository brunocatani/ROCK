#pragma once

#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/FrikWeaponOffsets.h"
#include "physics-interaction/weapon/HoldingGripMath.h"
#include "physics-interaction/weapon/HoldingGripRecords.h"

#include <cstdint>
#include <optional>

/*
 * One firing seat source for every equipped weapon, in either equip slot and
 * either hand. The native seat is the right hand-in-weapon relation harvested
 * from the weapon's native idle animation (AuthoredWeaponGripLibrary),
 * calibrated for pipe weapons and registered to this model, or the compiled
 * minigun seat. The left hand uses that native right seat mirrored across the
 * weapon's geometry plane plus the left firing grip offset, with the finger
 * locals mirrored alongside. A player-authored holding grip for a hand
 * replaces that hand's seat and fingers exactly.
 *
 * A player's FRIK weapon offset file outranks all of these: it replaces the
 * native right seat (so the left hand mirrors it too) and ROCK's own holding
 * grips. The harvested finger pose stays, as it does for the minigun seat.
 */
namespace rock::weapon_grip_seat
{
    struct Request
    {
        const RE::TESObjectWEAP* weapon{ nullptr };
        const RE::NiAVObject* weaponRoot{ nullptr };
        authored_weapon_grip_library::WeaponVariantIdentity variant{};
        std::uint64_t keywordFlags{ 0 };
        bool inPowerArmor{ false };
    };

    struct NativeSeat
    {
        // The library entry, pipe-calibrated and registered to this model's
        // frame, support relation included.
        authored_weapon_grip_library::LookupResult lookup{};
        // The native right seat: the entry's, or the compiled minigun seat.
        RE::NiTransform rightHandWeaponLocal{};
        // Raw animation space to this model's registration frame; live
        // captures apply it the same way.
        RE::NiPoint3 modelDisplacement{};
        bool compiledMinigun{ false };
        // The player's FRIK weapon offset for this weapon, if any. userSeat:
        // rightHandWeaponLocal is the seat it defines (it needs the native
        // seat when it only rotates the hand).
        std::optional<frik_weapon_offsets::Override> userOffset{};
        bool userSeat{ false };
        // The model's registration frame was readable. Nothing may seat a
        // model whose frame is unknown.
        bool modelRegistered{ false };

        // A seat not taken from the weapon's animation.
        [[nodiscard]] bool fixedSeat() const noexcept { return compiledMinigun || userSeat; }

        // The library holds the native idle animation's relation for it.
        [[nodiscard]] bool harvested() const noexcept
        {
            return lookup.found && authored_weapon_grip_library::isNativeIdleAuthority(lookup.source);
        }
    };

    [[nodiscard]] NativeSeat resolveNative(const Request& request);

    [[nodiscard]] bool leftFiringSeat(const RE::NiTransform& nativeRight, holding_grip_math::MirrorPlane mirrorPlane,
        RE::NiTransform& outLeft);

    [[nodiscard]] bool mirrorFingerPose(const authored_weapon_grip_library::FiringFingerPose& right,
        authored_weapon_grip_library::FiringFingerPose& outLeft);

    struct FiringSeat
    {
        RE::NiTransform handWeaponLocal{};
        authored_weapon_grip_library::FiringFingerPose fingers{};
        bool custom{ false };
    };

    // The firing seat for a hand. The custom holding grip wins; otherwise
    // the native seat requires the harvested finger pose.
    [[nodiscard]] bool firingSeat(const NativeSeat& native, const holding_grips::Record* custom, bool isLeft,
        holding_grip_math::MirrorPlane mirrorPlane, FiringSeat& out);
}
