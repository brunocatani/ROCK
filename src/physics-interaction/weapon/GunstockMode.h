#pragma once

#include "physics-interaction/weapon/WeaponAimBasis.h"
#include "physics-interaction/weapon/WeaponSupport.h"

namespace rock::gunstock_mode
{
    // Only the solver gets these virtual seats. The authored primary/support
    // hand frames and finger poses remain unchanged. No weapon-specific trim.
    inline bool configureVirtualAim(
        WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3>& input,
        const RE::NiTransform& primaryController,
        const RE::NiTransform& supportController,
        const RE::NiPoint3& primaryRecoilDisplacement,
        const float acquisitionAlpha)
    {
        if (!weapon_support_acquisition_math::isUsableTransform(primaryController) ||
            !weapon_support_acquisition_math::isUsableTransform(supportController) ||
            !weapon_support_acquisition_math::isFiniteVector(input.primaryGripLocal) ||
            !weapon_support_acquisition_math::isFiniteVector(input.primaryTargetWorld) ||
            !weapon_support_acquisition_math::isFiniteVector(primaryRecoilDisplacement) ||
            !std::isfinite(acquisitionAlpha)) return false;

        RE::NiTransform seed{};
        if (!weapon_aim_basis::tryResolveWorld(primaryController, input.weaponWorldTransform.scale, seed)) return false;
        // Recoil has already displaced the physical firing pivot. Apply the
        // same displacement to the rear aiming endpoint, keeping the front
        // fixed, as the normal two-hand recoil constraint does.
        const auto baseline = weaponSolverSub(
            weaponSolverSub(supportController.translate, primaryController.translate), primaryRecoilDisplacement);
        const float separation = weaponSolverLength(baseline);
        if (!std::isfinite(separation) || separation <= input.minimumSeparation) return false;

        auto candidate = input;
        candidate.weaponWorldTransform = seed;
        candidate.supportGripLocal = candidate.primaryGripLocal;
        candidate.supportGripLocal.y += separation / seed.scale;
        candidate.useSupportNormalTwist = false;
        candidate.supportNormalTwistFactor = 0;
        const auto forward = transform_math::rotateLocalVectorToWorld(seed.rotate, RE::NiPoint3{0, 1, 0});
        const auto start = weaponSolverAdd(candidate.primaryTargetWorld, weaponSolverScale(forward, separation));
        const auto target = weaponSolverAdd(candidate.primaryTargetWorld, baseline);
        const float alpha = std::clamp(acquisitionAlpha, 0.0f, 1.0f);
        candidate.supportTargetWorld = weaponSolverAdd(start, weaponSolverScale(weaponSolverSub(target, start), alpha));
        if (!weapon_support_acquisition_math::isFiniteVector(candidate.supportGripLocal) ||
            !weapon_support_acquisition_math::isFiniteVector(candidate.supportTargetWorld)) return false;
        input = candidate;
        return true;
    }

    inline WeaponTwoHandedSolverResult<RE::NiTransform> solveVirtualAim(
        const WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3>& input,
        const RE::NiTransform& primaryController, const RE::NiTransform& supportController,
        const RE::NiPoint3& primaryRecoilDisplacement, float acquisitionAlpha)
    {
        // Captures at world positions near 60,000/73,000 lose small recoil
        // offsets when two world-space aiming endpoints are subtracted. Solve
        // orientation around a zero pivot, then restore translation once.
        auto local = input;
        local.primaryTargetWorld = {};
        if (!configureVirtualAim(local, primaryController, supportController, primaryRecoilDisplacement, acquisitionAlpha)) return {};
        local.weaponWorldTransform.translate = {};
        auto result = solveTwoHandedWeaponTransformFrikPivot(local);
        if (result.solved) result.weaponWorldTransform.translate =
            weaponSolverAdd(result.weaponWorldTransform.translate, input.primaryTargetWorld);
        return result;
    }
}
