#pragma once

#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/HoldingGripRecords.h"

#include <cstdint>

/*
 * One firing seat source for every equipped weapon, in either equip slot and
 * either hand. The native seat is the right hand-in-weapon relation harvested
 * from the weapon's native idle animation (AuthoredWeaponGripLibrary),
 * calibrated for pipe weapons and registered to this model, or the compiled
 * minigun seat. The left hand uses that native right seat mirrored across the
 * weapon's geometry plane plus the left firing grip offset, with the finger
 * locals mirrored alongside. A player-authored holding grip for a hand
 * replaces that hand's seat and fingers exactly.
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
        // The model's registration frame was readable. Nothing may seat a
        // model whose frame is unknown.
        bool modelRegistered{ false };

        // The library holds the native idle animation's relation for it.
        [[nodiscard]] bool harvested() const noexcept
        {
            return lookup.found && authored_weapon_grip_library::isNativeIdleAuthority(lookup.source);
        }
    };

    [[nodiscard]] NativeSeat resolveNative(const Request& request);

    [[nodiscard]] bool leftFiringSeat(const RE::NiTransform& nativeRight, float mirrorPlaneX, RE::NiTransform& outLeft);

    [[nodiscard]] bool mirrorFingerPose(
        const authored_weapon_grip_library::FiringFingerPose& right,
        authored_weapon_grip_library::FiringFingerPose& outLeft);

    struct FiringSeat
    {
        RE::NiTransform handWeaponLocal{};
        authored_weapon_grip_library::FiringFingerPose fingers{};
        bool custom{ false };
    };

    // The firing seat for a hand. The custom holding grip wins; otherwise
    // the native seat requires the harvested finger pose.
    [[nodiscard]] bool firingSeat(
        const NativeSeat& native,
        const holding_grips::Record* custom,
        bool isLeft,
        float mirrorPlaneX,
        FiringSeat& out);
}
