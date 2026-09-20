#include "physics-interaction/weapon/WeaponAimBasis.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "physics-interaction/hand/HandVisual.h"

#include <cstdio>
#include <limits>

namespace
{
    bool expect(const char* label, bool result)
    {
        if (!result) std::printf("FAILED: %s\n", label);
        return result;
    }

    bool near(float a, float b) { return std::fabs(a - b) < 0.00002f; }

    bool samePose(const RE::NiTransform& a, const RE::NiTransform& b)
    {
        if (rock::weaponSolverLength(rock::weaponSolverSub(a.translate, b.translate)) > 0.001f ||
            std::fabs(a.scale - b.scale) > 0.00002f) return false;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                if (std::fabs(a.rotate.entry[r][c] - b.rotate.entry[r][c]) > 0.00002f) return false;
        return true;
    }

    bool checkAuthoredTransitions(RE::NiTransform controller, float scale)
    {
        using namespace rock;
        namespace visual = hand_visual_lerp_math;
        bool ok = true;
        RE::NiTransform aim{};
        if (!weapon_aim_basis::tryResolveWorld(controller, scale, aim)) return false;
        const RE::NiPoint3 primary{ 0.271f, -1.042f, -1.747f };
        const RE::NiPoint3 support{ 0.680f, 20.454f, 0.910f };
        const RE::NiPoint3 palm = weaponSolverAdd(controller.translate, RE::NiPoint3{ 1.3f, -2.0f, 0.7f });
        const auto carry = authored_weapon_grip_capture_policy::resolveAuthoredPrimaryWeaponWorldPositionOnly(
            aim, primary, palm, [](const auto& t, const auto& p) { return transform_math::localPointToWorld(t, p); });

        // Simulate the same weapon under FRIK's intermediate arm/graph pose.
        // Its support seat points 55 degrees above the authored carry seat.
        auto graphPose = carry;
        const auto pitch = weaponSolverAxisAngleStored<RE::NiMatrix3, RE::NiPoint3>(
            transform_math::rotateLocalVectorToWorld(aim.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }),
            55.0f * 3.14159265358979323846f / 180.0f);
        graphPose.rotate = weaponSolverApplyWorldRotationToStoredBasis<RE::NiMatrix3, RE::NiPoint3>(pitch, aim.rotate);
        graphPose = authored_weapon_grip_capture_policy::resolveAuthoredPrimaryWeaponWorldPositionOnly(
            graphPose, primary, palm, [](const auto& t, const auto& p) { return transform_math::localPointToWorld(t, p); });
        WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3> input{};
        input.weaponWorldTransform = aim;
        input.primaryGripLocal = primary;
        input.supportGripLocal = support;
        input.primaryTargetWorld = palm;
        input.supportTargetWorld = transform_math::localPointToWorld(graphPose, support);
        const auto wrongStart = solveTwoHandedWeaponTransformFrikPivot(input);
        ok &= expect("mixed-phase acquisition reproduces the upward jump", wrongStart.solved &&
            visual::rotationDistanceDegrees(carry, wrongStart.weaponWorldTransform) > 50.0f);

        const auto startSeat = makePrimaryAnchoredSupportGripTarget(aim, primary, support, palm);
        input.supportTargetWorld = startSeat;
        const auto zero = solveTwoHandedWeaponTransformFrikPivot(input);
        ok &= expect("authored acquisition starts exactly at carry", zero.solved && samePose(zero.weaponWorldTransform, carry));

        // Step acquisition at different frame rates. The firing palm stays
        // fixed and correction progresses toward only the intended target.
        const auto fullTarget = makeLockedSupportGripTarget(palm,
            weaponSolverAdd(startSeat, RE::NiPoint3{ 3.0f, -1.0f, -6.0f }), startSeat,
            weaponSolverLength(weaponSolverSub(startSeat, palm)), 0.001f);
        RE::NiTransform held{};
        for (const int frameCount : { 6, 9, 14 }) {
            float previousAngle = -0.01f;
            for (int frame = 0; frame <= frameCount; ++frame) {
                const float alpha = static_cast<float>(frame) / frameCount;
                input.supportTargetWorld = weaponSolverAdd(startSeat,
                    weaponSolverScale(weaponSolverSub(fullTarget, startSeat), alpha));
                const auto solved = solveTwoHandedWeaponTransformFrikPivot(input);
                const float angle = visual::rotationDistanceDegrees(carry, solved.weaponWorldTransform);
                ok &= expect("acquisition keeps the firing pivot and advances continuously", solved.solved &&
                    solved.primaryError < 0.001f && angle + 0.01f >= previousAngle && angle < 45.0f);
                previousAngle = angle;
                held = solved.weaponWorldTransform;
            }
        }

        // The native parent moves during release. Recompute the endpoint
        // from controller aim, never from the node holding graph/return data.
        auto parent = controller;
        visual::VisualReturnTransition<RE::NiTransform> returning{};
        returning.begin(transform_math::composeTransforms(transform_math::invertTransform(parent), held));
        RE::NiTransform returned{};
        RE::NiTransform resumed{};
        bool complete = false;
        for (int frame = 0; frame < 32 && !complete; ++frame) {
            controller.translate.x += 0.05f;
            parent.translate.x += 0.05f;
            RE::NiTransform liveAim{};
            ok &= expect("release reads current controller aim", weapon_aim_basis::tryResolveWorld(controller, scale, liveAim));
            const auto movingPalm = weaponSolverAdd(palm, RE::NiPoint3{ 0.05f * (frame + 1), 0.0f, 0.0f });
            resumed = authored_weapon_grip_capture_policy::resolveAuthoredPrimaryWeaponWorldPositionOnly(
                liveAim, primary, movingPalm, [](const auto& t, const auto& p) { return transform_math::localPointToWorld(t, p); });
            const auto localTarget = transform_math::composeTransforms(transform_math::invertTransform(parent), resumed);
            const auto step = visual::advanceVisualReturn(returning, localTarget, 1.0f / 90.0f, visual::kEquippedWeaponReturnConfig);
            returned = transform_math::composeTransforms(parent, step.transform);
            complete = step.reachedTarget;

            // A support re-grab starts on the in-flight return, even though
            // its new controller-based solve is already aimed elsewhere.
            const auto residual = visual::captureHandoffResidualLocal(held, returned);
            ok &= expect("interrupted return continues without a re-grab jump",
                samePose(visual::applyHandoffResidual(held, residual, 0.0f), returned));
            ok &= expect("re-grab converges to the new solve",
                samePose(visual::applyHandoffResidual(held, residual, 1.0f), held));
        }
        ok &= expect("release completion equals resumed authored carry", complete && samePose(returned, resumed));
        return ok;
    }
}

int main()
{
    using namespace rock;
    bool ok = true;
    RE::NiTransform controller{};
    controller.scale = 1.0f;
    controller.translate = { -335.86664f, -341.83038f, 107.02197f };
    // Correlated pre-FRIK controller and native driver sample, 2026-09-17.
    const float controllerRows[3][3] = {
        { 0.9608731f, 0.2424956f, 0.1338615f },
        { -0.1981389f, 0.2640419f, 0.9439402f },
        { 0.1935563f, -0.9335299f, 0.3017586f },
    };
    const float driverRows[3][3] = {
        { 0.9608731f, 0.2424956f, 0.1338615f },
        { -0.2679592f, 0.9361829f, 0.2275077f },
        { -0.0701493f, -0.2544755f, 0.9645317f },
    };
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) controller.rotate.entry[r][c] = controllerRows[r][c];
    RE::NiTransform weapon{};
    ok &= expect("controller basis reproduces the healthy native aim", weapon_aim_basis::tryResolveWorld(controller, 1.0f, weapon));
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) ok &= expect("native rotation sample", near(weapon.rotate.entry[r][c], driverRows[r][c]));

    // Equipped and loose paths share the same aim; the authored palm seat
    // changes only placement, at either model scale and any world position.
    for (const float scale : { 1.0f, 0.865347f }) {
        RE::NiTransform aim{};
        ok &= expect("model scale accepted", weapon_aim_basis::tryResolveWorld(controller, scale, aim));
        const RE::NiPoint3 grip{ 0.815f, -0.734f, -2.679f };
        const RE::NiPoint3 palm{ -356.1f, -350.0f, 105.2f };
        const auto seated = authored_weapon_grip_capture_policy::resolveAuthoredPrimaryWeaponWorldPositionOnly(
            aim, grip, palm, [](const auto& t, const auto& p) { return transform_math::localPointToWorld(t, p); });
        const auto actualPalm = transform_math::localPointToWorld(seated, grip);
        ok &= expect("authored grip lands at the physical palm", near(actualPalm.x, palm.x) && near(actualPalm.y, palm.y) && near(actualPalm.z, palm.z));
        ok &= expect("model scale preserved", seated.scale == scale);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) ok &= expect("seating preserves controller aim", seated.rotate.entry[r][c] == weapon.rotate.entry[r][c]);
    }
    RE::NiTransform moved = controller;
    moved.translate = { 9500.0f, -2400.0f, 600.0f };
    RE::NiTransform movedAim{};
    ok &= expect("player translation accepted", weapon_aim_basis::tryResolveWorld(moved, 1.0f, movedAim));
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) ok &= expect("player translation cannot rotate aim", movedAim.rotate.entry[r][c] == weapon.rotate.entry[r][c]);

    // The authored two-hand solve leaves roll to its starting basis. Reusing
    // an animated Weapon root therefore lets a cycle roll the gun even when
    // both controller targets are stationary.
    WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3> input{};
    input.weaponWorldTransform = weapon;
    input.primaryGripLocal = { 0.815f, -0.734f, -2.679f };
    input.supportGripLocal = { 0.755f, 21.439f, -0.052f };
    input.primaryTargetWorld = transform_math::localPointToWorld(weapon, input.primaryGripLocal);
    input.supportTargetWorld = transform_math::localPointToWorld(weapon, input.supportGripLocal);
    const auto reference = solveTwoHandedWeaponTransformFrikPivot(input);
    ok &= expect("controller-based two-hand solve succeeds", reference.solved);
    RE::NiTransform animated = weapon;
    const auto axis = weaponSolverNormalize(weaponSolverSub(input.supportTargetWorld, input.primaryTargetWorld));
    const auto cycleRotation = weaponSolverAxisAngleStored<RE::NiMatrix3, RE::NiPoint3>(axis, 0.8f);
    animated.rotate = weaponSolverApplyWorldRotationToStoredBasis<RE::NiMatrix3, RE::NiPoint3>(cycleRotation, weapon.rotate);
    input.weaponWorldTransform = animated;
    const auto contaminated = solveTwoHandedWeaponTransformFrikPivot(input);
    ok &= expect("cycle roll survives the old animated-root solve", contaminated.solved &&
        weapon_support_acquisition_math::rotationAngleRadians(
            transform_math::composeTransforms(transform_math::invertTransform(reference.weaponWorldTransform), contaminated.weaponWorldTransform).rotate) > 0.7f);
    ok &= expect("authored solve obtains a fresh controller basis", weapon_aim_basis::tryResolveWorld(controller, animated.scale, input.weaponWorldTransform));
    const auto fixed = solveTwoHandedWeaponTransformFrikPivot(input);
    ok &= expect("controller-based solve retains both grip targets", fixed.solved && fixed.primaryError < 0.0001f && fixed.supportError < 0.0001f);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) ok &= expect("animated roll cannot enter controller-based solve", near(fixed.weaponWorldTransform.rotate.entry[r][c], reference.weaponWorldTransform.rotate.entry[r][c]));
    for (const float scale : { 1.0f, 0.865347f, 1.2f }) {
        ok &= checkAuthoredTransitions(controller, scale);
        auto mirroredController = controller;
        mirroredController.rotate = weaponSolverApplyWorldRotationToStoredBasis<RE::NiMatrix3, RE::NiPoint3>(
            weaponSolverAxisAngleStored<RE::NiMatrix3, RE::NiPoint3>(RE::NiPoint3{ 0.0f, 0.0f, 1.0f }, 1.2f), controller.rotate);
        ok &= checkAuthoredTransitions(mirroredController, scale);
    }
    ok &= expect("invalid scale rejected", !weapon_aim_basis::tryResolveWorld(controller, 0.0f, weapon));
    controller.rotate.entry[0][0] = std::numeric_limits<float>::quiet_NaN();
    ok &= expect("invalid controller rejected", !weapon_aim_basis::tryResolveWorld(controller, 1.0f, weapon));
    return ok ? 0 : 1;
}
