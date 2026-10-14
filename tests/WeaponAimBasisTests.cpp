#include "physics-interaction/weapon/WeaponAimBasis.h"
#include "physics-interaction/weapon/NativeSecondaryPose.h"
#include "physics-interaction/weapon/NativeSecondaryPoseTransport.h"
#include "physics-interaction/weapon/NativePresentedPoseHistory.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/AuthoredWeaponGripPose.h"
#include "physics-interaction/weapon/WeaponGripTransfer.h"
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

    bool checkMeleePitch()
    {
        using namespace rock;
        bool ok = true;
        const auto identity = transform_math::makeIdentityTransform<RE::NiTransform>();
        auto grip = identity;
        grip.rotate = weaponSolverAxisAngleStored<RE::NiMatrix3, RE::NiPoint3>(
            RE::NiPoint3{ 0, 0, 1 }, 0.4f);
        grip.translate = { 4.0f, -1.0f, 2.0f };
        const RE::NiPoint3 palmInHand{ 6.0f, -2.0f, -0.2f };
        const auto gripPoint = transform_math::localPointToWorld(grip, palmInHand);
        for (const float turn : { 0.0f, 1.3f }) {
            auto controller = identity;
            controller.rotate = weaponSolverAxisAngleStored<RE::NiMatrix3, RE::NiPoint3>(
                RE::NiPoint3{ 0, 0, 1 }, turn);
            controller.translate = { -9500.0f, 6200.0f, 600.0f };
            controller.scale = 1.2f;
            auto wrist = identity;
            wrist.rotate = weaponSolverAxisAngleStored<RE::NiMatrix3, RE::NiPoint3>(
                RE::NiPoint3{ 0, 1, 0 }, 0.6f);
            wrist.translate = { 1, -3, 5 };
            auto hand = transform_math::composeTransforms(controller, wrist);
            hand.scale = 1.0f;
            const auto palm = transform_math::localPointToWorld(hand, palmInHand);
            const auto upInWeapon = transform_math::rotateLocalVectorToWorld(grip.rotate,
                transform_math::rotateWorldVectorToLocal(wrist.rotate, RE::NiPoint3{ 0, 1, 0 }));
            for (const float scale : { 0.865347f, 1.0f, 1.2f }) {
                // A change back to zero must use the untouched physical hand,
                // never accumulate a correction on the last rendered pose.
                for (const float pitch : { 0.0f, 15.0f, -45.0f, -30.0f, 0.0f, 15.0f }) {
                    const float effectivePitch = 30.0f + pitch;
                    RE::NiTransform aim{}, base{};
                    ok &= expect("melee pitch resolves", weapon_aim_basis::tryResolveMeleeWorld(
                        hand, grip, controller, scale, pitch, aim));
                    ok &= expect("uncorrected melee resolves", weapon_aim_basis::tryResolveMeleeWorld(hand, grip, scale, base));
                    if (pitch == -30.0f) ok &= expect("minus thirty cancels the baseline and restores original aim", samePose(aim, base));
                    const auto seated = authored_weapon_grip_capture_policy::resolveAuthoredPrimaryWeaponWorldPositionOnly(
                        aim, gripPoint, palm, [](const auto& t, const auto& p) { return transform_math::localPointToWorld(t, p); });
                    const auto presentedHand = transform_math::composeTransforms(seated, grip);
                    ok &= expect("pitch pins both weapon grip and presented palm",
                        weaponSolverLength(weaponSolverSub(transform_math::localPointToWorld(seated, gripPoint), palm)) < 0.002f &&
                        weaponSolverLength(weaponSolverSub(transform_math::localPointToWorld(presentedHand, palmInHand), palm)) < 0.002f);
                    ok &= expect("hand and weapon rotate together by thirty degrees plus the adjustment",
                        std::fabs(hand_visual_lerp_math::rotationDistanceDegrees(hand, presentedHand) - std::fabs(effectivePitch)) < 0.05f &&
                        seated.scale == scale);
                    const auto direction = transform_math::rotateWorldVectorToLocal(controller.rotate,
                        transform_math::rotateLocalVectorToWorld(seated.rotate, upInWeapon));
                    const float radians = effectivePitch * 0.017453292519943295769f;
                    ok &= expect("forward tilt follows controller orientation",
                        near(direction.x, 0.0f) && near(direction.y, std::cos(radians)) && near(direction.z, -std::sin(radians)));

                    auto rotationOnlyController = controller;
                    rotationOnlyController.translate = {};
                    rotationOnlyController.scale = 1.0f;
                    const auto baseInWand = transform_math::composeTransforms(
                        transform_math::invertTransform(rotationOnlyController), base);
                    const auto pitchTrim = weapon_aim_basis::meleePitchInController(pitch);
                    const auto right = transform_math::composeTransforms(pitchTrim, baseInWand);
                    const auto left = transform_math::composeTransforms(pitchTrim,
                        left_firing_position_only_math::mirrorRightWeaponInWandOrientation(baseInWand));
                    const auto mirroredRight = left_firing_position_only_math::mirrorRightWeaponInWandOrientation(right);
                    for (int row = 0; row < 3; ++row)
                        for (int col = 0; col < 3; ++col)
                            ok &= expect("same pitch survives left hand mirroring exactly once",
                                near(left.rotate.entry[row][col], mirroredRight.rotate.entry[row][col]));
                }
            }
        }
        RE::NiTransform result{};
        ok &= expect("non-finite pitch fails closed", !weapon_aim_basis::tryResolveMeleeWorld(
            identity, grip, identity, 1.0f, std::numeric_limits<float>::quiet_NaN(), result));
        auto invalid = identity;
        invalid.rotate.entry[0][0] = std::numeric_limits<float>::infinity();
        ok &= expect("invalid pitch carrier fails closed", !weapon_aim_basis::tryResolveMeleeWorld(
            identity, grip, invalid, 1.0f, 15.0f, result));
        return ok;
    }

    bool checkMeleeAim()
    {
        using namespace rock;
        bool ok = true;
        // Two authored model conventions: a blade along +Y must stand up,
        // while a glove pointing along +X must face forward in hand space.
        for (const bool glove : { false, true }) {
            auto grip = transform_math::makeIdentityTransform<RE::NiTransform>();
            grip.rotate = weaponSolverAxisAngleStored<RE::NiMatrix3, RE::NiPoint3>(
                glove ? RE::NiPoint3{ 0, 0, 1 } : RE::NiPoint3{ 1, 0, 0 },
                -1.5707963267948966f);
            grip.translate = { 5.4f, -0.4f, -1.6f };
            const RE::NiPoint3 modelDirection = glove ? RE::NiPoint3{ 1, 0, 0 } : RE::NiPoint3{ 0, 1, 0 };
            const RE::NiPoint3 handDirection = glove ? RE::NiPoint3{ 0, 1, 0 } : RE::NiPoint3{ 0, 0, 1 };
            const RE::NiPoint3 palmInHand{ 0.8f, -0.7f, -2.6f };
            const auto gripPoint = transform_math::localPointToWorld(grip, palmInHand);
            for (const float scale : { 0.865347f, 1.0f, 1.2f }) {
                for (const float turn : { 0.0f, 1.3f }) {
                    auto hand = transform_math::makeIdentityTransform<RE::NiTransform>();
                    hand.rotate = weaponSolverAxisAngleStored<RE::NiMatrix3, RE::NiPoint3>(RE::NiPoint3{ 0, 0, 1 }, turn);
                    hand.translate = { 1858.5f, -341.8f, 103.5f };
                    RE::NiTransform aim{};
                    ok &= expect("authored melee aim resolves", weapon_aim_basis::tryResolveMeleeWorld(hand, grip, scale, aim));
                    const auto palm = transform_math::localPointToWorld(hand, palmInHand);
                    const auto seated = authored_weapon_grip_capture_policy::resolveAuthoredPrimaryWeaponWorldPositionOnly(
                        aim, gripPoint, palm, [](const auto& t, const auto& p) { return transform_math::localPointToWorld(t, p); });
                    const auto actualPalm = transform_math::localPointToWorld(seated, gripPoint);
                    ok &= expect("melee palm remains seated at every model scale",
                        weaponSolverLength(weaponSolverSub(actualPalm, palm)) < 0.001f && seated.scale == scale);
                    const auto direction = transform_math::rotateLocalVectorToWorld(seated.rotate, modelDirection);
                    const auto expectedDirection = transform_math::rotateLocalVectorToWorld(hand.rotate, handDirection);
                    ok &= expect("blade stays upright and glove stays forward as the hand turns",
                        weaponSolverLength(weaponSolverSub(direction, expectedDirection)) < 0.0001f);
                    const auto presentedHand = transform_math::composeTransforms(seated, grip);
                    ok &= expect("melee grip cannot bend the physical wrist",
                        hand_visual_lerp_math::rotationDistanceDegrees(hand, presentedHand) < 0.05f);

                    // Loose placement and the equipped return reconstruct the
                    // same weapon orientation from the same physical hand.
                    const auto placement = transform_math::composeTransforms(transform_math::invertTransform(seated), hand);
                    const auto loose = transform_math::composeTransforms(hand, transform_math::invertTransform(placement));
                    ok &= expect("melee pickup and equipped placement agree", samePose(loose, seated));
                }
            }
        }
        auto hand = transform_math::makeIdentityTransform<RE::NiTransform>();
        auto grip = hand;
        RE::NiTransform result{};
        ok &= expect("invalid melee scale fails closed", !weapon_aim_basis::tryResolveMeleeWorld(hand, grip, 0.0f, result));
        grip.scale = 0.0f;
        ok &= expect("singular authored grip fails closed", !weapon_aim_basis::tryResolveMeleeWorld(hand, grip, 1.0f, result));
        grip = hand;
        grip.rotate.entry[1][0] = std::numeric_limits<float>::quiet_NaN();
        ok &= expect("invalid authored rotation fails closed", !weapon_aim_basis::tryResolveMeleeWorld(hand, grip, 1.0f, result));
        grip = hand;
        hand.rotate.entry[0][1] = std::numeric_limits<float>::quiet_NaN();
        ok &= expect("invalid physical hand fails closed", !weapon_aim_basis::tryResolveMeleeWorld(hand, grip, 1.0f, result));
        return ok;
    }
}

int main()
{
    using namespace rock;
    bool ok = checkMeleeAim();
    ok &= checkMeleePitch();
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

    // Secondary native model placement must survive arbitrary player movement
    // and parent changes, without turning the gun into a reflected mesh.
    {
        auto wristInController = transform_math::makeIdentityTransform<RE::NiTransform>();
        wristInController.translate = { 2.0f, -3.0f, 1.0f };
        const auto physical = transform_math::composeTransforms(controller, wristInController);
        auto rightSeat = transform_math::makeIdentityTransform<RE::NiTransform>();
        rightSeat.rotate = weaponSolverAxisAngleStored<RE::NiMatrix3, RE::NiPoint3>({ 0, 0, 1 }, 0.37f);
        rightSeat.translate = { 0.815f, -0.734f, -2.679f };
        const auto leftSeat = holding_grip_math::toNative(holding_grip_math::mirroredHandSeat(
            holding_grip_math::fromNative(rightSeat)));
        const RE::NiPoint3 palm{ 6.0f, -2.0f, 0.2f };
        for (const float scale : { 0.865347f, 1.0f, 1.2f }) {
            RE::NiTransform weapon{}, hand{};
            ok &= expect("secondary pose resolves", native_secondary_pose::solve(controller, physical, leftSeat, palm, scale,
                weapon_aim_basis::weaponInController(), {}, weapon, hand));
            ok &= expect("secondary firing seat pins the physical palm at every model scale",
                weaponSolverLength(transform_math::localPointToWorld(hand, palm) - transform_math::localPointToWorld(physical, palm)) < 0.001f);
            ok &= expect("mirrored seat and rendered gun retain proper rotations",
                holding_grip_math::valid(holding_grip_math::fromNative(leftSeat)) &&
                holding_grip_math::valid(holding_grip_math::fromNative(weapon)));
            auto motion = transform_math::makeIdentityTransform<RE::NiTransform>();
            motion.rotate = weaponSolverAxisAngleStored<RE::NiMatrix3, RE::NiPoint3>({ 0, 0, 1 }, 1.3f);
            motion.translate = { 2100.0f, -3800.0f, 620.0f };
            RE::NiTransform movedWeapon{}, movedHand{};
            ok &= expect("moving controllers resolve independently", native_secondary_pose::solve(
                transform_math::composeTransforms(motion, controller), transform_math::composeTransforms(motion, physical),
                leftSeat, palm, scale, weapon_aim_basis::weaponInController(), {}, movedWeapon, movedHand));
            ok &= expect("secondary solve follows player motion without a frozen world pose",
                samePose(movedWeapon, transform_math::composeTransforms(motion, weapon)) &&
                samePose(movedHand, transform_math::composeTransforms(motion, hand)));
            // The game changes the parent after FRIK. Recompute local placement
            // under that parent; replaying the previous local is incorrect.
            const auto local = transform_math::composeTransforms(transform_math::invertTransform(motion), weapon);
            ok &= expect("late native parent movement cannot rotate the placed gun",
                samePose(transform_math::composeTransforms(motion, local), weapon));
        }
        auto invalidSeat = leftSeat;
        invalidSeat.scale = 0;
        RE::NiTransform rejectedWeapon{}, rejectedHand{};
        ok &= expect("invalid authored pose fails before a scene write", !native_secondary_pose::solve(
            controller, physical, invalidSeat, palm, 1, weapon_aim_basis::weaponInController(), {}, rejectedWeapon, rejectedHand));
    }

    // Slot 1 aims like slot 0 in the same hand: the right hand holds the
    // controller basis; the left mirrors it and applies the left aim trim.
    {
        const auto basis = weapon_aim_basis::weaponInController();
        ok &= expect("slot 1 right aim is the controller basis",
            samePose(native_secondary_pose::firingWeaponInWand(false, 12.0f, -7.0f), basis));
        const auto expectedLeft = transform_math::composeTransforms(
            left_firing_position_only_math::leftAimTrim<RE::NiTransform>(12.0f, -7.0f),
            left_firing_position_only_math::mirrorRightWeaponInWandOrientation(basis));
        ok &= expect("slot 1 left aim is slot 0's trimmed mirror",
            samePose(native_secondary_pose::firingWeaponInWand(true, 12.0f, -7.0f), expectedLeft));
        const RE::NiPoint3 palm{ 6.0f, -2.0f, 0.2f };
        const RE::NiPoint3 offset{ 0.5f, -1.0f, 0.25f };
        auto seat = transform_math::makeIdentityTransform<RE::NiTransform>();
        seat.translate = { 0.8f, -0.7f, -2.6f };
        const auto physical = transform_math::makeIdentityTransform<RE::NiTransform>();
        RE::NiTransform weapon{}, hand{};
        ok &= expect("left grip offset solve resolves", native_secondary_pose::solve(physical, physical, seat, palm, 1.0f,
            expectedLeft, offset, weapon, hand));
        ok &= expect("the left grip offset moves the seated palm to the shifted hand target",
            weaponSolverLength(transform_math::localPointToWorld(hand, palm) - transform_math::localPointToWorld(physical, palm + offset)) < 0.001f);
    }

    // Captured 2026-10-12 frame 3452: the primary native arm pass leaves the
    // secondary attachment near the origin; its own pass moves it to the
    // controller. A world write between those passes creates a huge local.
    {
        auto target = weapon_aim_basis::weaponInController();
        target.translate = { 0.9847f, -1.9616f, -2.0078f };
        auto temporaryParent = transform_math::makeIdentityTransform<RE::NiTransform>();
        temporaryParent.translate = { -48387.7695f, 47794.0703f, -40325.0781f };
        const float temporaryRows[3][3] = {
            {-0.649386f, 0.091125f, 0.754980f}, {0.676474f, 0.522750f, 0.518764f}, {-0.347393f, 0.847602f, -0.401110f}
        };
        auto placedParent = transform_math::makeIdentityTransform<RE::NiTransform>();
        placedParent.translate = {-0.5560f, -8.7682f, 3.5112f};
        const float placedRows[3][3] = {
            {-0.085775f, 0.977530f, -0.192554f}, {-0.064110f, -0.198281f, -0.978046f}, {-0.994249f, -0.071547f, 0.079677f}
        };
        for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col) {
            temporaryParent.rotate.entry[row][col] = temporaryRows[row][col];
            placedParent.rotate.entry[row][col] = placedRows[row][col];
        }
        const auto earlyLocal = weapon_visual_authority_math::worldTargetToParentLocal(temporaryParent, target);
        const auto corrupted = transform_math::composeTransforms(placedParent, earlyLocal);
        ok &= expect("captured unrelated arm repair reproduces the huge offset",
            weaponSolverLength(earlyLocal.translate) > 78000 && weaponSolverLength(corrupted.translate - target.translate) > 78000);
        const auto properLocal = weapon_visual_authority_math::worldTargetToParentLocal(placedParent, target);
        const auto restored = transform_math::composeTransforms(placedParent, properLocal);
        ok &= expect("same target after its own arm placement retains a grip-sized local",
            weaponSolverLength(properLocal.translate) < 15 && weaponSolverLength(restored.translate - target.translate) < 0.001f);

        auto evaluatedPart = transform_math::composeTransforms(placedParent, target);
        // Native evaluated worlds can differ from rebuilding the original local
        // (animation/OMOD output). Transport the evaluated frame intact.
        evaluatedPart.translate += RE::NiPoint3{ 1.0f, -2.0f, 3.0f };
        const auto relation = transform_math::composeTransforms(transform_math::invertTransform(placedParent), evaluatedPart);
        const auto delta = weapon_visual_authority_math::makePresentationWorldDelta(placedParent, target);
        const auto movedPart = weapon_visual_authority_math::applyPresentationWorldDelta(delta, evaluatedPart);
        const auto retainedRelation = transform_math::composeTransforms(transform_math::invertTransform(target), movedPart);
        ok &= expect("rigid presentation retains native evaluated descendant pose", samePose(relation, retainedRelation));

        // Rendered parents drift from orthonormal (0.00083 measured on the
        // second weapon's attachment parent). Transport and local resolution
        // must use the exact inverse: the transpose doubled that drift into
        // the moved model on every pass.
        auto skewedParent = placedParent;
        skewedParent.rotate.entry[0][1] += 0.0008f;
        skewedParent.rotate.entry[2][0] -= 0.0006f;
        const auto skewedLocal = weapon_visual_authority_math::worldTargetToParentLocal(skewedParent, target);
        const auto recomposed = transform_math::composeTransforms(skewedParent, skewedLocal);
        ok &= expect("skewed parent local recomposes the exact target",
            weaponSolverLength(recomposed.translate - target.translate) < 0.002f &&
                transform_math::storedRotationOrthonormalityError(recomposed.rotate) < 1.0e-5);
        auto modelLocal = transform_math::makeIdentityTransform<RE::NiTransform>();
        modelLocal.translate = RE::NiPoint3{ 2.0f, -1.0f, 0.5f };
        const auto skewedModel = transform_math::composeTransforms(skewedParent, modelLocal);
        const auto skewedDelta = weapon_visual_authority_math::makePresentationWorldDelta(skewedParent, target);
        const auto movedModel = weapon_visual_authority_math::applyPresentationWorldDelta(skewedDelta, skewedModel);
        const auto expectedModel = transform_math::composeTransforms(target, modelLocal);
        ok &= expect("rigid transport from a skewed root keeps the model rigid",
            weaponSolverLength(movedModel.translate - expectedModel.translate) < 0.002f &&
                transform_math::storedRotationOrthonormalityError(movedModel.rotate) < 1.0e-5);
    }

    // A generated proxy pocket is intentionally distinct from the authored
    // palm. Loose placement must match equipped placement while its body-local
    // constraint anchor still lands on that independent proxy pivot.
    for (bool left : { false, true }) {
        for (float modelScale : { 0.865347f, 1.0f, 1.2f }) {
            for (const RE::NiPoint3 origin : { RE::NiPoint3{10.0f, 20.0f, 30.0f},
                    RE::NiPoint3{-69457.0f, 79726.0f, 7475.0f} }) {
                auto hand = controller;
                hand.translate = origin;
                hand.scale = left ? 0.95f : 1.0f;
                const RE::NiPoint3 palmInHand{6.0f, -2.0f, left ? 0.2f : -0.2f};
                const auto palm = transform_math::localPointToWorld(hand, palmInHand);
                RE::NiTransform aim{};
                ok &= expect("authored placement aim available", weapon_aim_basis::tryResolveWorld(controller, modelScale, aim));
                const RE::NiPoint3 seat{0.815f, -0.734f, -2.679f};
                const auto equipped = authored_weapon_grip_capture_policy::resolveAuthoredPrimaryWeaponWorldPositionOnly(
                    aim, seat, palm, [](const auto& t, const auto& p) { return transform_math::localPointToWorld(t, p); });
                AuthoredWeaponGripPose pose{};
                pose.placementHandWeaponLocal = transform_math::composeTransforms(
                    transform_math::invertTransform(equipped), hand);
                const auto loose = pose.placementWorld(hand, modelScale, palmInHand);
                const float tolerance = 0.025f; // Float-coordinate rounding at the captured distant cell.
                ok &= expect("loose and equipped weapon origins agree across hand and model scales",
                    weaponSolverLength(loose.translate - equipped.translate) < tolerance && loose.scale == modelScale);
                for (int row = 0; row < 3; ++row)
                    for (int col = 0; col < 3; ++col)
                        ok &= expect("loose placement retains equipped orientation",
                            std::fabs(loose.rotate.entry[row][col] - equipped.rotate.entry[row][col]) < 0.0001f);
                const auto proxy = palm + transform_math::localVectorToWorld(hand, RE::NiPoint3{0.0f, left ? -0.86f : 0.86f, 0.0f});
                auto bodyInWeapon = transform_math::makeIdentityTransform<RE::NiTransform>();
                bodyInWeapon.translate = {1.0f, -3.0f, 2.0f};
                const auto bodyWorld = transform_math::composeTransforms(loose, bodyInWeapon);
                const auto pivotInBody = transform_math::worldPointToLocal(bodyWorld, proxy);
                ok &= expect("normalizing presentation preserves the generated proxy constraint pivot",
                    weaponSolverLength(transform_math::localPointToWorld(bodyWorld, pivotInBody) - proxy) < tolerance);
                const auto capturedHand = weapon_grip_transfer::handInWeapon(bodyInWeapon,
                    transform_math::makeIdentityTransform<RE::NiTransform>(),
                    transform_math::composeTransforms(transform_math::invertTransform(hand), bodyWorld));
                const auto capturedPalm = transform_math::localPointToWorld(capturedHand, palmInHand);
                const auto transferred = authored_weapon_grip_capture_policy::resolveAuthoredPrimaryWeaponWorldPositionOnly(
                    aim, capturedPalm, palm, [](const auto& t, const auto& p) { return transform_math::localPointToWorld(t, p); });
                ok &= expect("paired transfer carries the normalized palm rather than the physics pivot",
                    weaponSolverLength(transferred.translate - loose.translate) < tolerance);
                const auto wrongSeat = authored_weapon_grip_capture_policy::resolveAuthoredPrimaryWeaponWorldPositionOnly(
                    equipped, seat, proxy, [](const auto& t, const auto& p) { return transform_math::localPointToWorld(t, p); });
                ok &= expect("seating visual geometry at the physics pivot reproduces the measured mismatch",
                    weaponSolverLength(wrongSeat.translate - equipped.translate) > 0.8f);
            }
        }
    }
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
    // Captured failure: body around (29971,-66900,606), private parts near
    // the detached rig's origin. Its world must include the authored Weapon
    // offset before descendant worlds are evaluated.
    for (const float scale:{1.0f,0.865347f}) {
        auto target=weapon;
        target.translate={29971.7363f,-66900.9297f,606.3915f};
        target.scale=scale;
        auto weaponInRig=transform_math::makeIdentityTransform<RE::NiTransform>();
        weaponInRig.translate={1.6f,14.2f,-8.4f};
        weaponInRig.rotate=weaponSolverAxisAngleStored<RE::NiMatrix3,RE::NiPoint3>({1,0,0},0.65f);
        RE::NiTransform rigWorld{};
        ok &= expect("detached behavior root can be anchored",native_secondary_pose::resolveBehaviorRootWorld(weaponInRig,target,rigWorld));
        const auto placedWeapon=transform_math::composeTransforms(rigWorld,weaponInRig);
        ok &= expect("private Weapon lands on the held weapon",(placedWeapon.translate-target.translate).Length()<0.02f);
        for (const RE::NiPoint3 partOffset:{RE::NiPoint3{0,-2,-7},RE::NiPoint3{0,3,2},RE::NiPoint3{0,4.5f,2}}) {
            auto part=transform_math::makeIdentityTransform<RE::NiTransform>();
            part.translate=partOffset;
            const auto partInRig=transform_math::composeTransforms(weaponInRig,part);
            const auto presented=transform_math::composeTransforms(rigWorld,partInRig);
            const auto expected=transform_math::composeTransforms(target,part);
            ok &= expect("magazine and moving bolt retain their evaluated weapon-relative pose",(presented.translate-expected.translate).Length()<0.025f);
            ok &= expect("unplaced rig reproduces the missing-part distance",(partInRig.translate-expected.translate).Length()>10000);
        }
        auto collapsed=weaponInRig;
        collapsed.rotate=RE::NiMatrix3{};
        ok &= expect("collapsed private rig rejected before inversion",!native_secondary_pose::resolveBehaviorRootWorld(collapsed,target,rigWorld));
    }
    {
        namespace transport=native_secondary_pose_transport;
        auto initial=transform_math::makeIdentityTransform<RE::NiTransform>();
        initial.translate={29971.7363f,-66900.9297f,606.3915f};
        // Native matrix measured in frame1095, 16:26 session. Its tiny
        // non-orthogonality must not multiply into descendant scale each pass.
        const float rows[3][3]={{1.000163f,0.001669f,-0.002215f},{-0.001730f,0.550011f,-0.834557f},{0.000301f,0.835510f,0.551147f}};
        for (int r=0;r<3;++r) for (int c=0;c<3;++c) initial.rotate.entry[r][c]=rows[r][c];
        auto part=transform_math::makeIdentityTransform<RE::NiTransform>();
        part.translate={-0.8f,7.3f,-10.3f};
        auto previous=initial;
        auto output=transform_math::composeTransforms(previous,part);
        auto oldOutput=output;
        transport::EvaluatedPose evaluatedPose;
        bool reproduced=false;
        for (int frame=1;frame<=2000;++frame) {
            auto target=initial;
            target.translate.x+=std::sin(float(frame)*0.019f)*3.0f;
            const auto yaw=weaponSolverAxisAngleStored<RE::NiMatrix3,RE::NiPoint3>({0,0,1},std::sin(float(frame)*0.013f)*0.25f);
            target.rotate=transform_math::multiplyStoredRotations(initial.rotate,yaw);
            auto carry=transform_math::makeIdentityTransform<RE::NiTransform>();
            carry.translate={0.125f,-0.25f,0.0625f};
            previous=transform_math::composeTransforms(carry,previous);
            output=transform_math::composeTransforms(carry,output);
            oldOutput=transform_math::composeTransforms(carry,oldOutput);
            evaluatedPose.carryPublished(carry);
            RE::NiTransform next;
            ok &= expect("evaluated descendant transports",evaluatedPose.reframe(previous,target,output,next));
            const auto expected=transform_math::composeTransforms(target,part);
            ok &= expect("repeated transport retains part placement",(next.translate-expected.translate).Length()<0.15f);
            for (int r=0;r<3;++r) for (int c=0;c<3;++c)
                ok &= expect("repeated transport does not stretch rotation",std::abs(next.rotate.entry[r][c]-expected.rotate.entry[r][c])<0.0002f);
            if (!reproduced) {
                const auto badDelta=transform_math::composeTransforms(target,transform_math::invertTransform(previous));
                oldOutput=transform_math::composeTransforms(badDelta,oldOutput);
                reproduced=transport::inspect(oldOutput).orthogonalityError>0.1;
            }
            output=next; previous=target;
        }
        ok &= expect("old transpose transport reproduces accumulated distortion",reproduced);
        RE::NiTransform identical;
        ok &= expect("unchanged anchor is exactly idempotent",evaluatedPose.reframe(previous,previous,output,identical) && samePose(output,identical));
        auto nativeUpdate=output;
        nativeUpdate.translate.z+=0.75f;
        RE::NiTransform changedLocal;
        ok &= expect("native world-only animation can be captured",transport::relative(previous,nativeUpdate,changedLocal));
        ok &= expect("external animation replaces cached relation",evaluatedPose.reframe(previous,initial,nativeUpdate,identical) &&
            (identical.translate-transform_math::composeTransforms(initial,changedLocal).translate).Length()<0.02f);
        auto hiddenAnchor=initial; hiddenAnchor.scale=0;
        auto beforeHide=identical;
        ok &= expect("controlled anchor may hide descendants",evaluatedPose.reframe(initial,hiddenAnchor,beforeHide,identical) && identical.scale==0);
        auto hiddenWorld=identical;
        ok &= expect("retained relation survives native zero-scale hiding",evaluatedPose.reframe(hiddenAnchor,initial,hiddenWorld,identical) &&
            (identical.translate-beforeHide.translate).Length()<0.02f && identical.scale==beforeHide.scale);
        auto hidden=output; hidden.scale=0;
        ok &= expect("authored hidden descendant remains hidden",evaluatedPose.reframe(previous,initial,hidden,identical) && identical.scale==0);
        auto singular=previous; singular.rotate=RE::NiMatrix3{};
        ok &= expect("singular source is rejected before transport",!evaluatedPose.reframe(singular,initial,output,identical));
    }
    {
        native_secondary_pose_transport::PresentedHistory history;
        auto last=transform_math::makeIdentityTransform<RE::NiTransform>();
        last.translate={29985.1328f,-66897.5469f,599.2660f};
        ok &= expect("first presentation has no fabricated prior motion",samePose(history.publish(1899,last),last));
        auto corrected=last;
        corrected.translate.x+=0.25f;
        ok &= expect("same-frame correction cannot advance history",samePose(history.publish(1899,corrected),last));
        auto movedHistoryPose=corrected;
        movedHistoryPose.translate={29988.3242f,-66905.2031f,602.2858f};
        movedHistoryPose.rotate=weaponSolverAxisAngleStored<RE::NiMatrix3,RE::NiPoint3>({1,0,0},1.85f);
        ok &= expect("new frame retains the last corrected presentation",samePose(history.publish(1900,movedHistoryPose),corrected));
        // Captured native hand placement moves through another origin and a
        // different rotation. Six callbacks in one frame are still one frame.
        for (int phase=0;phase<6;++phase) {
            movedHistoryPose.translate.z+=0.12f;
            ok &= expect("native intermediate passes cannot become previous-frame pose",samePose(history.publish(1900,movedHistoryPose),corrected));
        }
        ok &= expect("real controller and part motion survive in history",samePose(history.publish(1901,corrected),movedHistoryPose));
        ok &= expect("menu gap seeds a new presentation epoch",samePose(history.publish(1920,last),last));
    }
    ok &= expect("invalid scale rejected", !weapon_aim_basis::tryResolveWorld(controller, 0.0f, weapon));
    controller.rotate.entry[0][0] = std::numeric_limits<float>::quiet_NaN();
    ok &= expect("invalid controller rejected", !weapon_aim_basis::tryResolveWorld(controller, 1.0f, weapon));
    return ok ? 0 : 1;
}
