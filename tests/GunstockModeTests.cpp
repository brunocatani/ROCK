#include "physics-interaction/weapon/GunstockMode.h"
#include "GunstockCapturedFrames.h"
#include <iostream>
#include <limits>
#include <source_location>

using namespace rock;

namespace
{
    bool fail(std::source_location location = std::source_location::current()) { std::cerr << "Geometry failure at line " << location.line() << "\n"; return false; }
    bool nearPoint(const RE::NiPoint3& a, const RE::NiPoint3& b, float tolerance = 0.002f)
    { return weaponSolverLength(weaponSolverSub(a, b)) < tolerance; }
    bool sameRotation(const RE::NiTransform& a, const RE::NiTransform& b)
    {
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
            if (std::abs(a.rotate.entry[r][c] - b.rotate.entry[r][c]) > 0.00002f) return fail();
        return true;
    }
    bool check(const RE::NiTransform& rear, const RE::NiTransform& front,
        const RE::NiTransform& weapon, const RE::NiTransform& primary, const RE::NiTransform& support)
    {
        WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3> input{};
        input.weaponWorldTransform = weapon;
        input.primaryGripLocal = transform_math::worldPointToLocal(weapon, primary.translate);
        input.supportGripLocal = transform_math::worldPointToLocal(weapon, support.translate);
        input.primaryTargetWorld = primary.translate;
        input.useSupportNormalTwist = true;
        if (!gunstock_mode::configureVirtualAim(input, rear, front, {}, 1)) return fail();
        const auto solved = gunstock_mode::solveVirtualAim(input, rear, front, {}, 1);
        if (!solved.solved || !nearPoint(transform_math::localPointToWorld(solved.weaponWorldTransform, input.primaryGripLocal), primary.translate)) return fail();
        const auto forward = transform_math::rotateLocalVectorToWorld(solved.weaponWorldTransform.rotate, RE::NiPoint3{0,1,0});
        const auto baseline = weaponSolverNormalize(weaponSolverSub(front.translate, rear.translate));
        if (!nearPoint(forward, baseline, 0.00002f)) return fail();

        // Different authored support seats and support-controller wrist roll
        // must not alter the stock orientation at identical controller positions.
        for (const auto seat : {RE::NiPoint3{-12,21,17}, RE::NiPoint3{9,14,-15}}) {
            auto other = input;
            other.supportGripLocal = seat;
            auto turnedFront = front;
            turnedFront.rotate = weapon_aim_basis::weaponInController().rotate;
            if (!gunstock_mode::configureVirtualAim(other, rear, turnedFront, {}, 1)) return fail();
            const auto result = gunstock_mode::solveVirtualAim(other, rear, turnedFront, {}, 1);
            if (!result.solved || !sameRotation(solved.weaponWorldTransform, result.weaponWorldTransform)) return fail();
        }
        // Recoil remains a displacement of the rear endpoint against the
        // fixed front, rather than being erased by the collinear aiming rule.
        auto kicked = input;
        const RE::NiPoint3 kick{0.4f, -0.2f, 0.8f};
        kicked.primaryTargetWorld = weaponSolverAdd(primary.translate, kick);
        if (!gunstock_mode::configureVirtualAim(kicked, rear, front, kick, 1)) return fail();
        const auto kickResult = gunstock_mode::solveVirtualAim(kicked, rear, front, kick, 1);
        const auto kickDirection = weaponSolverNormalize(weaponSolverSub(weaponSolverSub(front.translate,rear.translate),kick));
        if (!kickResult.solved || !nearPoint(transform_math::rotateLocalVectorToWorld(kickResult.weaponWorldTransform.rotate, RE::NiPoint3{0,1,0}), kickDirection, 0.00002f)) return fail();
        if (!nearPoint(transform_math::localPointToWorld(kickResult.weaponWorldTransform, kicked.primaryGripLocal), kicked.primaryTargetWorld)) return fail();

        auto initial = input;
        RE::NiTransform seed{};
        if (!weapon_aim_basis::tryResolveWorld(rear, weapon.scale, seed) || !gunstock_mode::configureVirtualAim(initial, rear, front, {}, 0)) return fail();
        const auto start = gunstock_mode::solveVirtualAim(initial, rear, front, {}, 0);
        if (!start.solved || !sameRotation(start.weaponWorldTransform, seed)) return fail();

        const auto unchanged = input;
        if (gunstock_mode::configureVirtualAim(input, rear, rear, {}, 1)) return fail();
        if (!sameRotation(input.weaponWorldTransform, unchanged.weaponWorldTransform)) return fail();
        auto invalid = front;
        invalid.translate.x = (std::numeric_limits<float>::quiet_NaN)();
        if (gunstock_mode::configureVirtualAim(input, rear, invalid, {}, 1)) return fail();
        return true;
    }
}

int main()
{
    using namespace gunstock_fixture;
    bool ok = check(NoHeroics_right_controller, NoHeroics_left_controller, NoHeroics_weapon_after, NoHeroics_primary_seat_world, NoHeroics_support_seat_world);
    ok &= check(Mk17_right_controller, Mk17_left_controller, Mk17_weapon_after, Mk17_primary_seat_world, Mk17_support_seat_world);
    // Reverse physical firing roles; geometry and pose preservation must obey
    // the selected hand, not hardcoded left/right controller slots.
    ok &= check(Mk17_left_controller, Mk17_right_controller, Mk17_weapon_after, Mk17_support_seat_world, Mk17_primary_seat_world);
    auto world = weapon_aim_basis::weaponInController();
    world.translate = {1100, -850, 92};
    world.scale = 0.865347f;
    ok &= check(transform_math::composeTransforms(world, NoHeroics_right_controller),
        transform_math::composeTransforms(world, NoHeroics_left_controller),
        transform_math::composeTransforms(world, NoHeroics_weapon_after),
        transform_math::composeTransforms(world, NoHeroics_primary_seat_world),
        transform_math::composeTransforms(world, NoHeroics_support_seat_world));
    std::cout << (ok ? "Gunstock captured-pose geometry, pivot, role reversal, recoil and invalid-input checks passed.\n" : "Gunstock geometry check failed.\n");
    return ok ? 0 : 1;
}
