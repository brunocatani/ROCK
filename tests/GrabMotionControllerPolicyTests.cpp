#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/grab/GrabInertiaPolicy.h"
#include "physics-interaction/grab/HeldMassMovement.h"
#include "physics-interaction/hand/HandVisual.h"

#include <cstdio>
#include <cstring>
#include <cmath>

namespace
{
    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        const float delta = actual > expected ? actual - expected : expected - actual;
        if (delta <= epsilon) {
            return true;
        }
        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }

    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }
        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectReason(const char* label, const char* actual, const char* expected)
    {
        if (actual && std::strcmp(actual, expected) == 0) {
            return true;
        }
        std::printf("%s expected %s got %s\n", label, expected, actual ? actual : "null");
        return false;
    }

    struct Vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct Mat3
    {
        float entry[3][3]{
            { 1.0f, 0.0f, 0.0f },
            { 0.0f, 1.0f, 0.0f },
            { 0.0f, 0.0f, 1.0f },
        };
    };

    struct Transform
    {
        Mat3 rotate{};
        Vec3 translate{};
        float scale = 1.0f;
    };

    Transform makeTransform(float x, float y, float z)
    {
        Transform transform{};
        transform.translate = Vec3{ x, y, z };
        return transform;
    }

    Transform makeRotatedTransform(float x, float y, float z, float angleDegrees)
    {
        Transform transform = makeTransform(x, y, z);
        const float radians = angleDegrees * 0.01745329251994329577f;
        const float cosine = std::cos(radians);
        const float sine = std::sin(radians);
        transform.rotate.entry[0][0] = cosine;
        transform.rotate.entry[0][1] = -sine;
        transform.rotate.entry[1][0] = sine;
        transform.rotate.entry[1][1] = cosine;
        return transform;
    }
}

int main()
{
    using namespace rock::grab_motion_controller;

    bool ok = true;

    MotorInput singleHand{};
    singleHand.baseMaxForce = 2000.0f;
    singleHand.mass = 2.0f;
    singleHand.forceToMassRatio = 500.0f;
    singleHand.fadeInEnabled = false;
    singleHand.authorityForceScale = 1.0f;

    const auto single = solveMotorTargets(singleHand, false);
    ok &= expectNear("single hand mass cap", single.linearMaxForce, 1000.0f, 0.001f);
    ok &= expectNear("single hand angular matches linear authority", single.angularMaxForce, 1000.0f, 0.001f);

    MotorInput shared = singleHand;
    shared.authorityForceScale = 0.5f;
    const auto twoHand = solveMotorTargets(shared, false);
    ok &= expectNear("two hands share mass-capped linear authority", twoHand.linearMaxForce, 500.0f, 0.001f);
    ok &= expectNear("two hands share angular authority", twoHand.angularMaxForce, 500.0f, 0.001f);

    MotorInput mediumMass = singleHand;
    mediumMass.mass = 10.0f;
    const auto mediumMassOutput = solveMotorTargets(mediumMass, false);
    ok &= expectNear("medium generic object keeps fixed HIGGS-style force", mediumMassOutput.linearMaxForce, 2000.0f, 0.001f);
    ok &= expectNear("medium generic object angular force follows full fixed force", mediumMassOutput.angularMaxForce, 2000.0f, 0.001f);

    MotorInput heavyMass = singleHand;
    heavyMass.mass = 50.0f;
    const auto heavyMassOutput = solveMotorTargets(heavyMass, false);
    ok &= expectNear("heavy generic object does not receive loose-weapon force", heavyMassOutput.linearMaxForce, 2000.0f, 0.001f);
    ok &= expectNear("heavy generic object angular force follows full generic force", heavyMassOutput.angularMaxForce, 2000.0f, 0.001f);

    MotorInput looseWeapon = singleHand;
    looseWeapon.baseMaxForce = 9000.0f;
    looseWeapon.angularToLinearForceRatio = 2.0f;
    looseWeapon.mass = 1000.0f;
    const auto looseWeaponOutput = solveMotorTargets(looseWeapon, false);
    ok &= expectNear("loose weapon base force is not double-boosted", looseWeaponOutput.linearMaxForce, 9000.0f, 0.001f);
    ok &= expectNear("loose weapon angular force can exceed linear pull authority", looseWeaponOutput.angularMaxForce, 18000.0f, 0.001f);

    ok &= expectNear("90hz physics force scale is neutral", computePhysicsRateForceScale(true, 1.0f / 90.0f, 90.0f, 0.5f, 0.75f, 1.35f), 1.0f, 0.001f);
    ok &= expectNear("60hz physics force scale strengthens held motors", computePhysicsRateForceScale(true, 1.0f / 60.0f, 90.0f, 0.5f, 0.75f, 1.35f), 1.224745f, 0.001f);
    ok &= expectNear("120hz physics force scale softens held motors", computePhysicsRateForceScale(true, 1.0f / 120.0f, 90.0f, 0.5f, 0.75f, 1.35f), 0.866025f, 0.001f);
    ok &= expectNear("high physics rate clamps to minimum force scale", computePhysicsRateForceScale(true, 1.0f / 240.0f, 90.0f, 0.5f, 0.75f, 1.35f), 0.75f, 0.001f);
    ok &= expectNear("invalid physics delta keeps neutral force scale", computePhysicsRateForceScale(true, 0.0f, 90.0f, 0.5f, 0.75f, 1.35f), 1.0f, 0.001f);

    /*
     * Effective substep rates produced by the Havok timing fix for the
     * supported game frame rates (min physics rate 70 Hz, max 3 substeps):
     * 45 FPS -> 2 x 90 Hz, 60 FPS -> 2 x 120 Hz, 72 FPS -> 1 x 72 Hz,
     * 90 FPS -> 1 x 90 Hz, 120 FPS -> 1 x 120 Hz. Force scaling must consume
     * the EFFECTIVE substep rate, so 45 FPS play lands on the neutral
     * calibration point and 72 FPS strengthens.
     */
    ok &= expectNear("45fps effective 90hz substeps stay neutral", computePhysicsRateForceScale(true, (1.0f / 45.0f) * 0.5f, 90.0f, 0.5f, 0.75f, 1.35f), 1.0f, 0.001f);
    ok &= expectNear("72hz physics force scale strengthens held motors", computePhysicsRateForceScale(true, 1.0f / 72.0f, 90.0f, 0.5f, 0.75f, 1.35f), 1.118034f, 0.001f);
    ok &= expectNear("60fps effective 120hz substeps soften held motors", computePhysicsRateForceScale(true, (1.0f / 60.0f) * 0.5f, 90.0f, 0.5f, 0.75f, 1.35f), 0.866025f, 0.001f);

    // Unknown physics rate is reported honestly as zero, never as a
    // pretended nominal measurement.
    ok &= expectNear("unknown physics delta reports zero hz", computePhysicsHz(0.0f), 0.0f, 0.0f);
    ok &= expectNear("measured physics delta reports its rate", computePhysicsHz(1.0f / 72.0f), 72.0f, 0.01f);

    // An unmeasured frame holds tau interpolation instead of stepping by a
    // fabricated nominal delta.
    ok &= expectNear("unmeasured delta holds tau advance", advanceToward(0.8f, 0.03f, 1.0f, 0.0f), 0.8f, 0.0f);
    ok &= expectNear("measured delta advances tau", advanceToward(0.8f, 0.03f, 1.0f, 1.0f), 0.03f, 0.001f);

    MotorInput scaledAt60Hz = singleHand;
    scaledAt60Hz.physicsRateForceScalingEnabled = true;
    scaledAt60Hz.physicsDeltaSeconds = 1.0f / 60.0f;
    scaledAt60Hz.mass = 100.0f;
    const auto scaledAt60HzOutput = solveMotorTargets(scaledAt60Hz, false);
    ok &= expectNear("60hz motor output records physics hz", scaledAt60HzOutput.physicsHz, 60.0f, 0.001f);
    ok &= expectNear("60hz force scale applies before mass cap", scaledAt60HzOutput.linearMaxForce, 2449.49f, 0.02f);
    ok &= expectNear("60hz angular force follows scaled linear force", scaledAt60HzOutput.angularMaxForce, 2449.49f, 0.02f);

    MotorInput massCappedScaled = scaledAt60Hz;
    massCappedScaled.mass = 2.0f;
    const auto massCappedScaledOutput = solveMotorTargets(massCappedScaled, false);
    ok &= expectNear("physics-rate scaling still obeys mass cap", massCappedScaledOutput.linearMaxForce, 1000.0f, 0.001f);

    MotorInput authorityScaled = scaledAt60Hz;
    authorityScaled.authorityForceScale = 0.5f;
    const auto authorityScaledOutput = solveMotorTargets(authorityScaled, false);
    ok &= expectNear("authority scale applies after physics-rate force scale", authorityScaledOutput.linearMaxForce, 1224.745f, 0.02f);

    MotorInput angularFixedTau = singleHand;
    angularFixedTau.mass = 100.0f;
    angularFixedTau.currentLinearTau = 0.8f;
    angularFixedTau.currentAngularTau = 0.8f;
    angularFixedTau.deltaTime = 1.0f;
    angularFixedTau.tauLerpSpeed = 1.0f;
    const auto angularFixedTauOutput = solveMotorTargets(angularFixedTau, false);
    ok &= expectNear("angular follow does not boost linear force", angularFixedTauOutput.linearMaxForce, 2000.0f, 0.001f);
    ok &= expectNear("linear follow keeps HIGGS-style tau fixed", angularFixedTauOutput.linearTau, 0.03f, 0.001f);
    ok &= expectNear("angular follow keeps HIGGS-style tau fixed", angularFixedTauOutput.angularTau, 0.03f, 0.001f);

    const auto contactSoftenedOutput = solveMotorTargets(angularFixedTau, true);
    ok &= expectNear("contact softening drives linear tau to collision tau", contactSoftenedOutput.linearTau, 0.01f, 0.001f);
    ok &= expectNear("contact softening drives angular tau to collision tau", contactSoftenedOutput.angularTau, 0.01f, 0.001f);
    ok &= expectNear("contact softening keeps the force budget", contactSoftenedOutput.linearMaxForce, 2000.0f, 0.001f);

    MotorInput tinyMassFloor = singleHand;
    tinyMassFloor.mass = 0.02f;
    tinyMassFloor.effectiveMotorMassFloorEnabled = true;
    tinyMassFloor.effectiveMotorMassFloor = 2.0f;
    const auto tinyMassFloorOutput = solveMotorTargets(tinyMassFloor, false);
    ok &= expectNear("tiny loose object uses motor-only effective mass floor", tinyMassFloorOutput.linearMaxForce, 1000.0f, 0.001f);
    ok &= expectNear("tiny loose object angular force follows floored linear force", tinyMassFloorOutput.angularMaxForce, 1000.0f, 0.001f);

    MotorInput tinyMassRaw = tinyMassFloor;
    tinyMassRaw.effectiveMotorMassFloorEnabled = false;
    const auto tinyMassRawOutput = solveMotorTargets(tinyMassRaw, false);
    ok &= expectNear("disabled effective mass floor preserves raw mass cap", tinyMassRawOutput.linearMaxForce, 10.0f, 0.001f);
    ok &= expectNear("disabled effective mass floor preserves raw angular cap", tinyMassRawOutput.angularMaxForce, 10.0f, 0.001f);

    ok &= expectNear("effective motor mass floors tiny finite mass", effectiveMotorMass(0.25f, true, 2.0f), 2.0f, 0.001f);
    ok &= expectNear("effective motor mass leaves heavier mass untouched", effectiveMotorMass(10.0f, true, 2.0f), 10.0f, 0.001f);
    ok &= expectNear("effective motor mass disabled keeps sanitized raw mass", effectiveMotorMass(0.25f, false, 2.0f), 0.25f, 0.001f);

    const float g3DecodedMass = rock::grab_mass_policy::massFromInverseMass(4.529953003e-05f);
    ok &= expectNear("tiny positive inverse mass remains readable", g3DecodedMass, 22075.284f, 0.02f);
    ok &= expectNear("loose weapon grab mass is capped while held",
        rock::grab_mass_policy::normalizedLooseWeaponGrabMass(g3DecodedMass, true),
        rock::grab_mass_policy::kLooseWeaponGrabMassCeiling,
        0.001f);
    ok &= expectNear("generic heavy object mass is not capped by weapon policy",
        rock::grab_mass_policy::normalizedLooseWeaponGrabMass(g3DecodedMass, false),
        g3DecodedMass,
        0.02f);
    ok &= expectTrue("large loose weapon mass requests normalization",
        rock::grab_mass_policy::shouldNormalizeLooseWeaponGrabMass(g3DecodedMass, true));
    ok &= expectTrue("large generic object mass does not request weapon normalization",
        !rock::grab_mass_policy::shouldNormalizeLooseWeaponGrabMass(g3DecodedMass, false));

    const float longObjectCap = computeAuthorityScaledAngularVelocityCap(18.0f, 0.50f);
    ok &= expectNear("release angular cap applies the long-object scale", longObjectCap, 9.0f, 0.001f);

    ok &= expectNear("visual hand lerp below minimum distance is immediate",
        rock::hand_visual_lerp_math::computeDistanceMappedDurationGameUnits(0.5f, 0.12f, 0.20f, 1.0f, 14.0f),
        0.0f,
        0.001f);
    ok &= expectNear("visual hand lerp maps midpoint distance to midpoint duration",
        rock::hand_visual_lerp_math::computeDistanceMappedDurationGameUnits(7.5f, 0.12f, 0.20f, 1.0f, 14.0f),
        0.16f,
        0.001f);
    ok &= expectNear("visual hand lerp clamps to max duration",
        rock::hand_visual_lerp_math::computeDistanceMappedDurationGameUnits(40.0f, 0.12f, 0.20f, 1.0f, 14.0f),
        0.20f,
        0.001f);

    const Transform visualStart = makeTransform(0.0f, 0.0f, 0.0f);
    const Transform visualTarget = makeTransform(10.0f, 0.0f, 0.0f);
    const auto immediateVisual = rock::hand_visual_lerp_math::blendTransformOverDuration(visualStart, visualTarget, 0.0f, 0.0f);
    ok &= expectTrue("zero-duration visual lerp reaches target", immediateVisual.reachedTarget);
    ok &= expectNear("zero-duration visual lerp publishes target x", immediateVisual.transform.translate.x, 10.0f, 0.001f);

    const auto halfwayVisual = rock::hand_visual_lerp_math::blendTransformOverDuration(visualStart, visualTarget, 0.10f, 0.20f);
    ok &= expectFalse("half-duration visual lerp is still moving", halfwayVisual.reachedTarget);
    ok &= expectNear("half-duration visual lerp reaches half distance", halfwayVisual.transform.translate.x, 5.0f, 0.001f);

    const auto completeVisual = rock::hand_visual_lerp_math::blendTransformOverDuration(visualStart, visualTarget, 0.20f, 0.20f);
    ok &= expectTrue("full-duration visual lerp reaches target", completeVisual.reachedTarget);
    ok &= expectNear("full-duration visual lerp publishes target x exactly", completeVisual.transform.translate.x, 10.0f, 0.001f);
    ok &= expectNear("timed visual lerp elapsed clamps to duration",
        rock::hand_visual_lerp_math::advanceTimedBlendElapsed(0.15f, 0.20f, 0.20f),
        0.20f,
        0.001f);

    const rock::hand_visual_lerp_math::VisualReturnConfig returnConfig{
        .minSeconds = 0.10f,
        .maxSeconds = 0.20f,
        .minDistanceGameUnits = 1.0f,
        .maxDistanceGameUnits = 11.0f,
        .minAngleDegrees = 5.0f,
        .maxAngleDegrees = 95.0f,
    };
    ok &= expectNear("translation-only return maps duration",
        rock::hand_visual_lerp_math::computeVisualReturnDuration(
            makeTransform(0.0f, 0.0f, 0.0f),
            makeTransform(6.0f, 0.0f, 0.0f),
            returnConfig),
        0.15f,
        0.001f);
    ok &= expectNear("rotation-only return receives non-zero duration",
        rock::hand_visual_lerp_math::computeVisualReturnDuration(
            makeRotatedTransform(0.0f, 0.0f, 0.0f, 0.0f),
            makeRotatedTransform(0.0f, 0.0f, 0.0f, 50.0f),
            returnConfig),
        0.15f,
        0.002f);
    ok &= expectNear("combined return chooses greater duration",
        rock::hand_visual_lerp_math::computeVisualReturnDuration(
            makeRotatedTransform(0.0f, 0.0f, 0.0f, 0.0f),
            makeRotatedTransform(3.0f, 0.0f, 0.0f, 95.0f),
            returnConfig),
        0.20f,
        0.002f);

    rock::hand_visual_lerp_math::VisualReturnTransition<Transform> movingReturn{};
    movingReturn.begin(makeTransform(0.0f, 0.0f, 0.0f));
    const auto movingReturnHalf = rock::hand_visual_lerp_math::advanceVisualReturn(
        movingReturn,
        makeTransform(11.0f, 0.0f, 0.0f),
        0.10f,
        returnConfig);
    ok &= expectFalse("visual return remains active at half duration", movingReturnHalf.reachedTarget);
    ok &= expectNear("visual return half target", movingReturnHalf.transform.translate.x, 5.5f, 0.001f);
    const auto movingReturnComplete = rock::hand_visual_lerp_math::advanceVisualReturn(
        movingReturn,
        makeTransform(13.0f, 0.0f, 0.0f),
        0.10f,
        returnConfig);
    ok &= expectTrue("moving-target return completes", movingReturnComplete.reachedTarget);
    ok &= expectNear("moving-target return publishes newest target exactly", movingReturnComplete.transform.translate.x, 13.0f, 0.001f);
    ok &= expectNear("return state retains exact last applied target", movingReturn.lastApplied.translate.x, 13.0f, 0.001f);

    rock::hand_visual_lerp_math::VisualReturnTransition<Transform> invalidDeltaReturn{};
    invalidDeltaReturn.begin(makeTransform(0.0f, 0.0f, 0.0f));
    const auto invalidDeltaResult = rock::hand_visual_lerp_math::advanceVisualReturn(
        invalidDeltaReturn,
        makeTransform(11.0f, 0.0f, 0.0f),
        -1.0f,
        returnConfig);
    ok &= expectNear("invalid negative delta does not advance return", invalidDeltaResult.transform.translate.x, 0.0f, 0.001f);

    rock::hand_visual_lerp_math::VisualReturnTransition<Transform> zeroDurationReturn{};
    zeroDurationReturn.begin(makeTransform(4.0f, 2.0f, 1.0f));
    const auto zeroDurationResult = rock::hand_visual_lerp_math::advanceVisualReturn(
        zeroDurationReturn,
        makeTransform(4.0f, 2.0f, 1.0f),
        0.0f,
        returnConfig);
    ok &= expectTrue("zero-duration return completes immediately", zeroDurationResult.reachedTarget);
    ok &= expectNear("zero-duration return publishes exact target", zeroDurationResult.transform.translate.x, 4.0f, 0.001f);

    const auto shortestArcHalf = rock::hand_visual_lerp_math::interpolateTransform(
        makeRotatedTransform(0.0f, 0.0f, 0.0f, 170.0f),
        makeRotatedTransform(0.0f, 0.0f, 0.0f, -170.0f),
        0.5f);
    const float shortestArcHalfAngle = rock::hand_visual_lerp_math::quaternionAngleRadians(
        rock::hand_visual_lerp_math::matrixToQuaternion(shortestArcHalf.rotate),
        rock::hand_visual_lerp_math::matrixToQuaternion(makeRotatedTransform(0.0f, 0.0f, 0.0f, 180.0f).rotate));
    ok &= expectNear("return rotation uses quaternion shortest arc", shortestArcHalfAngle, 0.0f, 0.002f);

    const auto ratioClamp = rock::grab_inertia_policy::normalizeInverseInertiaAxesForGrab(1.0f, 100.0f, 4.0f, 10.0f, 0.05f);
    ok &= expectTrue("inertia ratio clamp modifies axis", ratioClamp.modified);
    ok &= expectNear("inertia ratio clamp x", ratioClamp.normalized[0], 1.0f, 0.001f);
    ok &= expectNear("inertia ratio clamp y", ratioClamp.normalized[1], 10.0f, 0.001f);
    ok &= expectNear("inertia ratio clamp z", ratioClamp.normalized[2], 4.0f, 0.001f);
    ok &= expectNear("inertia ratio clamp output ratio", ratioClamp.normalizedRatio, 10.0f, 0.001f);

    const auto minInertiaClamp = rock::grab_inertia_policy::normalizeInverseInertiaAxesForGrab(50.0f, 80.0f, 60.0f, 10.0f, 0.02f);
    ok &= expectTrue("minimum inertia clamp modifies axes", minInertiaClamp.modified);
    ok &= expectNear("minimum inertia clamp x", minInertiaClamp.normalized[0], 50.0f, 0.001f);
    ok &= expectNear("minimum inertia clamp y", minInertiaClamp.normalized[1], 50.0f, 0.001f);
    ok &= expectNear("minimum inertia clamp z", minInertiaClamp.normalized[2], 50.0f, 0.001f);

    ok &= expectNear("long object disabled release angular scale",
        computeLongObjectAngularSpeedScale(false, 96.0f, 24.0f, 0.35f),
        1.0f,
        0.001f);
    ok &= expectNear("short object release angular scale",
        computeLongObjectAngularSpeedScale(true, 12.0f, 24.0f, 0.35f),
        1.0f,
        0.001f);
    ok &= expectNear("long object release angular scale",
        computeLongObjectAngularSpeedScale(true, 48.0f, 24.0f, 0.35f),
        0.5f,
        0.001f);
    ok &= expectNear("very long object release angular scale floor",
        computeLongObjectAngularSpeedScale(true, 200.0f, 24.0f, 0.35f),
        0.35f,
        0.001f);

    const rock::held_mass_movement::Config movementConfig{};
    ok &= expectNear("held mass movement scales by mass",
        rock::held_mass_movement::computeHeldMassReduction(10.0f, movementConfig),
        6.75f,
        0.001f);
    ok &= expectNear("held mass movement caps max reduction",
        rock::held_mass_movement::computeHeldMassReduction(200.0f, movementConfig),
        75.0f,
        0.001f);
    ok &= expectNear("held mass movement fades linearly toward zero",
        rock::held_mass_movement::computeFadeOutReduction(50.0f, 1.0f, 5.0f),
        40.0f,
        0.001f);
    ok &= expectNear("held mass movement fade reaches zero at configured duration",
        rock::held_mass_movement::computeFadeOutReduction(50.0f, 5.0f, 5.0f),
        0.0f,
        0.001f);

    return ok ? 0 : 1;
}
