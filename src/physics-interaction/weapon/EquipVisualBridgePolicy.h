#pragma once

#include "physics-interaction/weapon/WeaponSupport.h"
#include "physics-interaction/weapon/GunstockMode.h"

namespace rock::equip_visual_bridge_policy
{
    // The retained loose geometry and the equipped hands must use the same
    // current-frame weapon solve, accounting for their model registration.
    template <class Transform, class Point>
    [[nodiscard]] inline Transform registeredLooseWorld(const Transform& equippedWorld,
        const Point& sourceModelTranslation, const Point& equippedModelTranslation)
    {
        auto registration = transform_math::makeIdentityTransform<Transform>();
        registration.translate = weaponSolverSub(equippedModelTranslation, sourceModelTranslation);
        return transform_math::composeTransforms(equippedWorld, registration);
    }

    template <class Transform, class Point>
    [[nodiscard]] inline WeaponTwoHandedSolverResult<Transform> solvePairedBridge(
        const Transform& primaryWorld, const Point& primaryLocal,
        const Point& supportLocal, const Point& supportTargetWorld)
    {
        const auto primaryTarget = transform_math::localPointToWorld(primaryWorld, primaryLocal);
        const auto supportBase = transform_math::localPointToWorld(primaryWorld, supportLocal);
        const auto lockedSupport = makeLockedSupportGripTarget(primaryTarget, supportTargetWorld,
            supportBase, weaponSolverLength(weaponSolverSub(supportBase, primaryTarget)), 0.001f);
        return solveTwoHandedWeaponTransformFrikPivot(WeaponTwoHandedSolverInput<Transform, Point>{
            .weaponWorldTransform = primaryWorld,
            .primaryGripLocal = primaryLocal,
            .supportGripLocal = supportLocal,
            .primaryTargetWorld = primaryTarget,
            .supportTargetWorld = lockedSupport,
        });
    }

    inline WeaponTwoHandedSolverResult<RE::NiTransform> solvePairedGunstockBridge(
        const RE::NiTransform& capturedWorld, const RE::NiPoint3& primaryLocal,
        const RE::NiTransform& primaryController, const RE::NiTransform& supportController)
    {
        WeaponTwoHandedSolverInput<RE::NiTransform,RE::NiPoint3> aim{};
        aim.weaponWorldTransform = capturedWorld;
        aim.primaryGripLocal = primaryLocal;
        aim.primaryTargetWorld = transform_math::localPointToWorld(capturedWorld,primaryLocal);
        return gunstock_mode::solveVirtualAim(aim,primaryController,supportController,{},1.0f);
    }

    // The detached loose model is visual-only. It may cover the native
    // inventory-to-first-person gap, but it must never become a persistent
    // substitute for the equipped weapon's render and collision authority.
    constexpr float kMaximumPresentationLeaseSeconds = 1.0f;

    [[nodiscard]] inline constexpr float effectivePresentationLeaseSeconds(
        const float requestedSeconds) noexcept
    {
        if (!(requestedSeconds > 0.0f)) {
            return kMaximumPresentationLeaseSeconds;
        }
        return requestedSeconds < kMaximumPresentationLeaseSeconds ?
            requestedSeconds :
            kMaximumPresentationLeaseSeconds;
    }

    [[nodiscard]] inline constexpr bool presentationLeaseExpired(
        const float elapsedSeconds,
        const float requestedSeconds) noexcept
    {
        return elapsedSeconds >=
               effectivePresentationLeaseSeconds(requestedSeconds);
    }
}
