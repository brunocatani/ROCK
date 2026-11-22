// Gunstock mode: the two-handed solver aims the weapon along the line between
// the controllers and centres it laterally on that line, using virtual seats;
// the authored hand poses and the authored grip height are unchanged.
#pragma once

#include "physics-interaction/weapon/grip/WeaponSupport.h"
#include "physics-interaction/weapon/handling/WeaponAimBasis.h"

namespace rock::gunstock_mode
{
    // Only the solver gets these virtual seats. The authored primary/support
    // hand frames and finger poses remain unchanged. No weapon-specific trim.
    inline bool configureVirtualAim(WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3>& input,
        const RE::NiTransform& primaryController, const RE::NiTransform& supportController,
        const RE::NiPoint3& primaryRecoilDisplacement, const float acquisitionAlpha, const bool centerOnControllerLine)
    {
        if (!transform_math::isInvertibleTransform(primaryController) ||
            !transform_math::isInvertibleTransform(supportController) ||
            !weapon_support_acquisition_math::isFiniteVector(input.primaryGripLocal) ||
            !weapon_support_acquisition_math::isFiniteVector(input.primaryTargetWorld) ||
            !weapon_support_acquisition_math::isFiniteVector(primaryRecoilDisplacement) ||
            !std::isfinite(acquisitionAlpha))
            return false;

        RE::NiTransform seed{};
        if (!weapon_aim_basis::tryResolveWorld(primaryController, input.weaponWorldTransform.scale, seed))
            return false;
        // Recoil has already displaced the physical firing pivot. Apply the
        // same displacement to the rear aiming endpoint, keeping the front
        // fixed, as the normal two-hand recoil constraint does.
        const auto baseline = weaponSolverSub(weaponSolverSub(supportController.translate, primaryController.translate),
            primaryRecoilDisplacement);
        const float separation = weaponSolverLength(baseline);
        if (!std::isfinite(separation) || separation <= input.minimumSeparation)
            return false;

        auto candidate = input;
        candidate.weaponWorldTransform = seed;
        candidate.supportGripLocal = candidate.primaryGripLocal;
        candidate.supportGripLocal.y += separation / seed.scale;
        candidate.useSupportNormalTwist = false;
        candidate.supportNormalTwistFactor = 0;
        const auto forward = transform_math::rotateLocalVectorToWorld(seed.rotate, RE::NiPoint3{ 0, 1, 0 });
        const auto start = weaponSolverScale(forward, separation);
        const float alpha = std::clamp(acquisitionAlpha, 0.0f, 1.0f);
        const auto aimFromPalm = weaponSolverAdd(start, weaponSolverScale(weaponSolverSub(baseline, start), alpha));
        candidate.supportTargetWorld = weaponSolverAdd(candidate.primaryTargetWorld, aimFromPalm);
        if (!weapon_support_acquisition_math::isFiniteVector(candidate.supportGripLocal) ||
            !weapon_support_acquisition_math::isFiniteVector(candidate.supportTargetWorld))
            return false;
        if (!centerOnControllerLine) {
            input = candidate;
            return true;
        }

        // Authored firing seats differ in how far the palm sits beside the
        // model's centerline (weapon-local X = 0), so the same palm left some
        // weapons beside the controller line. A stock holds the barrel on that
        // line: slide the seat along weapon X only, keeping its authored
        // height and depth, until the line through the (recoil-displaced)
        // controller origins lies on the centerline. The orientation does not
        // depend on the seat's X, so the solve is unchanged by this shift.
        // Alpha eases it in with the aim, starting at the one-hand pose.
        // Orient around a zero pivot: palm-relative vectors keep full precision.
        auto orientation = candidate;
        orientation.weaponWorldTransform.translate = {};
        orientation.primaryTargetWorld = {};
        orientation.supportTargetWorld = aimFromPalm;
        const auto oriented = solveTwoHandedWeaponTransform(orientation);
        if (!oriented.solved)
            return false;
        const auto lateralAxis =
            transform_math::rotateLocalVectorToWorld(oriented.weaponWorldTransform.rotate, RE::NiPoint3{ 1, 0, 0 });
        // Subtract the two world positions before adding the small recoil
        // offset, so world-space and pivot-space callers round alike.
        const auto palmToLine = weaponSolverAdd(weaponSolverSub(primaryController.translate, candidate.primaryTargetWorld),
            primaryRecoilDisplacement);
        const float centeredX = -weaponSolverDot(palmToLine, lateralAxis) / seed.scale;
        if (!std::isfinite(centeredX))
            return false;
        candidate.primaryGripLocal.x += (centeredX - candidate.primaryGripLocal.x) * alpha;
        candidate.supportGripLocal.x = candidate.primaryGripLocal.x;
        input = candidate;
        return true;
    }

    inline WeaponTwoHandedSolverResult<RE::NiTransform> solveVirtualAim(
        const WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3>& input,
        const RE::NiTransform& primaryController, const RE::NiTransform& supportController,
        const RE::NiPoint3& primaryRecoilDisplacement, float acquisitionAlpha, const bool centerOnControllerLine)
    {
        // Captures at world positions near 60,000/73,000 lose small recoil
        // offsets when two world-space aiming endpoints are subtracted. Solve
        // orientation around a zero pivot, then restore translation once.
        // The controllers move into the same pivot frame: the centering
        // measures their line against the zeroed firing palm.
        auto local = input;
        local.primaryTargetWorld = {};
        auto primaryLocalController = primaryController;
        auto supportLocalController = supportController;
        primaryLocalController.translate = weaponSolverSub(primaryController.translate, input.primaryTargetWorld);
        supportLocalController.translate = weaponSolverSub(supportController.translate, input.primaryTargetWorld);
        if (!configureVirtualAim(local, primaryLocalController, supportLocalController, primaryRecoilDisplacement,
                acquisitionAlpha, centerOnControllerLine))
            return {};
        local.weaponWorldTransform.translate = {};
        auto result = solveTwoHandedWeaponTransformFrikPivot(local);
        if (result.solved)
            result.weaponWorldTransform.translate =
                weaponSolverAdd(result.weaponWorldTransform.translate, input.primaryTargetWorld);
        return result;
    }
}
