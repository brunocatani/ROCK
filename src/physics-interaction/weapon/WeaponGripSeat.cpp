#include "physics-interaction/weapon/WeaponGripSeat.h"

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/hand/HandFingerMirrorMath.h"
#include "physics-interaction/weapon/HoldingGripAdapter.h"
#include "physics-interaction/weapon/MinigunFiringGripPolicy.h"
#include "physics-interaction/weapon/VanillaWeaponGripFrame.h"
#include "physics-interaction/weapon/WeaponGripCalibration.h"
#include "RockConfig.h"

#include <cmath>
#include <span>

namespace rock::weapon_grip_seat
{
    namespace
    {
        [[nodiscard]] bool finiteTransform(const RE::NiTransform& transform)
        {
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return std::isfinite(transform.translate.x) && std::isfinite(transform.translate.y) &&
                   std::isfinite(transform.translate.z) && std::isfinite(transform.scale) &&
                   std::abs(transform.scale) > 0.000001f;
        }

        // hFRIK authored Weapon-in-Hand; the seat is the inverse
        // Hand-in-Weapon relation.
        [[nodiscard]] bool compiledMinigunSeat(bool inPowerArmor, RE::NiTransform& outHandInWeapon)
        {
            const auto& source = minigun_firing_grip_policy::weaponInFiringHand(inPowerArmor);
            RE::NiTransform weaponInHand{};
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    weaponInHand.rotate.entry[row][column] = source.rotation[static_cast<std::size_t>(row * 3 + column)];
                }
            }
            weaponInHand.translate = { source.translation[0], source.translation[1], source.translation[2] };
            weaponInHand.scale = source.scale;
            if (!finiteTransform(weaponInHand)) {
                outHandInWeapon = {};
                return false;
            }
            outHandInWeapon = transform_math::invertTransform(weaponInHand);
            return finiteTransform(outHandInWeapon);
        }
    }

    NativeSeat resolveNative(const Request& request)
    {
        NativeSeat seat{};
        if (!request.weapon ||
            !vanilla_weapon_grip_frame::resolveModelTranslation(request.weapon->formID, request.weaponRoot, seat.modelDisplacement)) {
            return seat;
        }
        const RE::NiPoint3& displacement = seat.modelDisplacement;
        seat.modelRegistered = true;
        seat.lookup = authored_weapon_grip_library::findResolvedVariant(request.weapon, request.variant, request.inPowerArmor);
        authored_weapon_grip_library::applyPipeDefaultOffset(seat.lookup);
        if (seat.lookup.found) {
            seat.lookup.rightHandWeaponLocal = vanilla_weapon_grip_frame::translateGrip(seat.lookup.rightHandWeaponLocal, displacement);
            if (seat.lookup.hasSupportRelation) {
                seat.lookup.supportHandWeaponLocal = vanilla_weapon_grip_frame::translateGrip(seat.lookup.supportHandWeaponLocal, displacement);
            }
        }
        RE::NiTransform minigun{};
        seat.compiledMinigun = minigun_firing_grip_policy::usesCompiledFiringSeat(request.keywordFlags) &&
                               compiledMinigunSeat(request.inPowerArmor, minigun);
        seat.rightHandWeaponLocal = seat.compiledMinigun ? minigun : seat.lookup.rightHandWeaponLocal;
        return seat;
    }

    bool leftFiringSeat(const RE::NiTransform& nativeRight, const float mirrorPlaneX, RE::NiTransform& outLeft)
    {
        const auto source = holding_grip_math::fromNative(nativeRight);
        if (!holding_grip_math::valid(source)) {
            return false;
        }
        outLeft = weapon_grip_calibration::shiftedHand(
            holding_grip_math::toNative(holding_grip_math::mirroredHandSeat(source, mirrorPlaneX)),
            g_rockConfig.rockLeftFiringGripOffsetGameUnits, true);
        return true;
    }

    bool mirrorFingerPose(
        const authored_weapon_grip_library::FiringFingerPose& right,
        authored_weapon_grip_library::FiringFingerPose& outLeft)
    {
        outLeft = {};
        if (!right.complete() ||
            !hand_finger_mirror_math::mirrorFingerLocalsAcrossHands<RE::NiTransform>(
                std::span<const RE::NiTransform>(right.localTransforms),
                std::span<RE::NiTransform>(outLeft.localTransforms))) {
            outLeft = {};
            return false;
        }
        outLeft.enabledMask = right.enabledMask;
        return true;
    }

    bool firingSeat(
        const NativeSeat& native,
        const holding_grips::Record* custom,
        const bool isLeft,
        const float mirrorPlaneX,
        FiringSeat& out)
    {
        out = {};
        if (custom) {
            out = { .handWeaponLocal = custom->wrist, .fingers = custom->fingers, .custom = true };
            return true;
        }
        if (!native.modelRegistered || !native.harvested() || !native.lookup.rightFiringFingerPose.complete()) {
            return false;
        }
        if (!isLeft) {
            out.handWeaponLocal = native.rightHandWeaponLocal;
            out.fingers = native.lookup.rightFiringFingerPose;
            return true;
        }
        return leftFiringSeat(native.rightHandWeaponLocal, mirrorPlaneX, out.handWeaponLocal) &&
               mirrorFingerPose(native.lookup.rightFiringFingerPose, out.fingers);
    }
}
