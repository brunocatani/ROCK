#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/grab/GrabInertiaPolicy.h"

#include <cstdio>

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

    struct Vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };
}

int main()
{
    using namespace rock::grab_motion_controller;

    bool ok = true;

    MotorInput singleHand{};
    singleHand.enabled = false;
    singleHand.baseMaxForce = 2000.0f;
    singleHand.maxForceMultiplier = 1.0f;
    singleHand.mass = 2.0f;
    singleHand.forceToMassRatio = 500.0f;
    singleHand.angularToLinearForceRatio = 12.5f;
    singleHand.fadeInEnabled = false;
    singleHand.authorityForceScale = 1.0f;

    const auto single = solveMotorTargets(singleHand);
    ok &= expectNear("single hand mass cap", single.linearMaxForce, 1000.0f, 0.001f);
    ok &= expectNear("single hand angular ratio", single.angularMaxForce, 80.0f, 0.001f);

    MotorInput shared = singleHand;
    shared.authorityForceScale = 0.5f;
    const auto twoHand = solveMotorTargets(shared);
    ok &= expectNear("two hands share mass-capped linear authority", twoHand.linearMaxForce, 500.0f, 0.001f);
    ok &= expectNear("two hands share angular authority", twoHand.angularMaxForce, 40.0f, 0.001f);

    MotorInput mediumMass = singleHand;
    mediumMass.mass = 10.0f;
    mediumMass.massResponsiveMaxForce = 9000.0f;
    const auto mediumMassOutput = solveMotorTargets(mediumMass);
    ok &= expectNear("medium generic object rises to mass-scaled force", mediumMassOutput.linearMaxForce, 5000.0f, 0.001f);
    ok &= expectNear("medium generic object angular force follows linear force", mediumMassOutput.angularMaxForce, 400.0f, 0.001f);

    MotorInput heavyMass = singleHand;
    heavyMass.mass = 50.0f;
    heavyMass.massResponsiveMaxForce = 9000.0f;
    const auto heavyMassOutput = solveMotorTargets(heavyMass);
    ok &= expectNear("heavy generic object reaches mass-responsive ceiling", heavyMassOutput.linearMaxForce, 9000.0f, 0.001f);
    ok &= expectNear("heavy generic object angular force follows ceiling", heavyMassOutput.angularMaxForce, 720.0f, 0.001f);

    MotorInput looseWeapon = singleHand;
    looseWeapon.baseMaxForce = 9000.0f;
    looseWeapon.mass = 1000.0f;
    looseWeapon.massResponsiveMaxForce = 9000.0f;
    const auto looseWeaponOutput = solveMotorTargets(looseWeapon);
    ok &= expectNear("loose weapon base force is not double-boosted", looseWeaponOutput.linearMaxForce, 9000.0f, 0.001f);
    ok &= expectNear("loose weapon angular force follows existing weapon ceiling", looseWeaponOutput.angularMaxForce, 720.0f, 0.001f);

    MotorInput adaptive = singleHand;
    adaptive.enabled = true;
    adaptive.positionErrorGameUnits = 20.0f;
    adaptive.fullPositionErrorGameUnits = 20.0f;
    adaptive.maxForceMultiplier = 4.0f;
    adaptive.mass = 100.0f;
    adaptive.authorityForceScale = 0.5f;
    const auto adaptiveShared = solveMotorTargets(adaptive);
    ok &= expectNear("two hands share adaptive boosted authority", adaptiveShared.linearMaxForce, 4000.0f, 0.001f);

    MotorInput rotationOnly = singleHand;
    rotationOnly.enabled = true;
    rotationOnly.rotationErrorDegrees = 60.0f;
    rotationOnly.fullRotationErrorDegrees = 60.0f;
    rotationOnly.maxForceMultiplier = 4.0f;
    rotationOnly.mass = 100.0f;
    rotationOnly.deltaTime = 1.0f;
    rotationOnly.tauLerpSpeed = 1.0f;
    rotationOnly.maxAngularTau = 0.35f;
    const auto rotationOnlyOutput = solveMotorTargets(rotationOnly);
    ok &= expectNear("rotation error does not boost linear force", rotationOnlyOutput.linearMaxForce, 2000.0f, 0.001f);
    ok &= expectNear("rotation error drives angular tau only", rotationOnlyOutput.angularTau, 0.35f, 0.001f);

    MotorInput positionOnlyPivot = singleHand;
    positionOnlyPivot.pivotQualityAngularScalingEnabled = true;
    positionOnlyPivot.pivotAuthorityPositionOnly = true;
    positionOnlyPivot.pivotAuthorityNormalTrusted = false;
    positionOnlyPivot.contactPatchUsedAsPivot = true;
    positionOnlyPivot.contactPatchSampleCount = 1;
    positionOnlyPivot.longObjectLeverGameUnits = 8.0f;
    const auto positionOnlyOutput = solveMotorTargets(positionOnlyPivot);
    ok &= expectNear("position-only small weak pivot clamps angular authority", positionOnlyOutput.angularAuthorityScale, 0.30f, 0.001f);
    ok &= expectNear("position-only small weak pivot reduces angular force", positionOnlyOutput.angularMaxForce, 24.0f, 0.001f);
    ok &= expectNear("weak pivot raises angular damping multiplier", positionOnlyOutput.angularDampingMultiplier, 1.75f, 0.001f);
    ok &= expectNear("weak pivot twist scale propagates", positionOnlyOutput.weakPivotTwistScale, 0.35f, 0.001f);

    const auto trustedAuthority = computeAngularAuthorityScale(AngularAuthorityInput{
        .enabled = true,
        .positionOnlyPivot = false,
        .normalTrusted = true,
        .contactPatchUsedAsPivot = true,
        .contactPatchSampleCount = 4,
        .longObjectLeverGameUnits = 8.0f,
    });
    ok &= expectNear("trusted small sphere-like support softens orientation authority", trustedAuthority.authorityScale, 0.65f, 0.001f);
    ok &= expectTrue("trusted small support classifies as sphere-like", trustedAuthority.contactSupportShape == ContactSupportShape::SphereLike);
    ok &= expectNear("sphere-like support limits contact-normal spin", trustedAuthority.contactNormalScale, 0.30f, 0.001f);

    const auto lowSupportAuthority = computeAngularAuthorityScale(AngularAuthorityInput{
        .enabled = true,
        .positionOnlyPivot = false,
        .normalTrusted = true,
        .contactPatchUsedAsPivot = true,
        .contactPatchSampleCount = 1,
        .longObjectLeverGameUnits = 8.0f,
    });
    ok &= expectNear("small low-support contact softens angular authority", lowSupportAuthority.authorityScale, 0.4225f, 0.001f);
    ok &= expectNear("trusted low-support contact does not raise damping", lowSupportAuthority.dampingMultiplier, 1.0f, 0.001f);
    ok &= expectTrue("single-hit small support is sphere-like not surface-authoritative", lowSupportAuthority.contactSupportShape == ContactSupportShape::SphereLike);

    const auto rejectedPatchAuthority = computeAngularAuthorityScale(AngularAuthorityInput{
        .enabled = true,
        .positionOnlyPivot = false,
        .normalTrusted = true,
        .contactPatchUsedAsPivot = false,
        .contactPatchSampleCount = 1,
        .longObjectLeverGameUnits = 8.0f,
    });
    ok &= expectNear("trusted non-patch pivot ignores rejected patch support", rejectedPatchAuthority.authorityScale, 1.0f, 0.001f);

    const auto trustedPointAuthority = computeAngularAuthorityScale(AngularAuthorityInput{
        .enabled = true,
        .positionOnlyPivot = false,
        .normalTrusted = true,
        .contactPatchUsedAsPivot = true,
        .contactPatchSampleCount = 1,
        .longObjectLeverGameUnits = 20.0f,
    });
    ok &= expectTrue("trusted single-point support classifies as point", trustedPointAuthority.contactSupportShape == ContactSupportShape::Point);
    ok &= expectNear("trusted point limits twist around grab point", trustedPointAuthority.twistScale, 0.35f, 0.001f);

    const Vec3 twistLimited = scaleWeakPivotTwistAngularVelocity(Vec3{ 1.0f, 2.0f, 3.0f }, Vec3{ 0.0f, 0.0f, 2.0f }, true, 0.25f);
    ok &= expectNear("weak pivot twist preserves swing x", twistLimited.x, 1.0f, 0.001f);
    ok &= expectNear("weak pivot twist preserves swing y", twistLimited.y, 2.0f, 0.001f);
    ok &= expectNear("weak pivot twist scales twist z", twistLimited.z, 0.75f, 0.001f);

    const auto longHandleAuthority = computeAngularAuthorityScale(AngularAuthorityInput{
        .enabled = true,
        .positionOnlyPivot = false,
        .normalTrusted = true,
        .contactPatchUsedAsPivot = true,
        .contactPatchSampleCount = 2,
        .longObjectLeverGameUnits = 72.0f,
        .longObjectReferenceLeverGameUnits = 24.0f,
    });
    ok &= expectTrue("two-hit long object classifies as long handle", longHandleAuthority.contactSupportShape == ContactSupportShape::LongHandle);
    ok &= expectNear("long handle applies line-support authority", longHandleAuthority.authorityScale, 0.75f, 0.001f);
    ok &= expectNear("long handle limits twist axis", longHandleAuthority.twistScale, 0.55f, 0.001f);

    const Vec3 axisLimited = scaleAngularVelocityByHeldAuthorityAxes(
        Vec3{ 10.0f, 6.0f, 4.0f },
        Vec3{ 0.0f, 0.0f, 1.0f },
        Vec3{ 1.0f, 0.0f, 0.0f },
        AngularAuthorityOutput{
            .authorityScale = 1.0f,
            .swingScale = 1.0f,
            .twistScale = 0.50f,
            .contactNormalScale = 0.25f,
            .axisLimited = true,
        });
    ok &= expectNear("axis authority scales contact-normal spin x", axisLimited.x, 2.5f, 0.001f);
    ok &= expectNear("axis authority preserves tangent spin y", axisLimited.y, 6.0f, 0.001f);
    ok &= expectNear("axis authority scales pivot twist z", axisLimited.z, 2.0f, 0.001f);

    const auto weakHeldAuthority = evaluateHeldAuthority(HeldAuthorityInput{
        .angular = AngularAuthorityInput{
            .enabled = true,
            .positionOnlyPivot = true,
            .normalTrusted = false,
            .contactPatchUsedAsPivot = true,
            .contactPatchSampleCount = 1,
            .longObjectLeverGameUnits = 8.0f,
        },
        .heldBodyColliding = false,
        .positionErrorGameUnits = 20.0f,
        .rotationErrorDegrees = 60.0f,
        .fullPositionErrorGameUnits = 20.0f,
        .fullRotationErrorDegrees = 60.0f,
    });
    ok &= expectNear("held authority records position error factor", weakHeldAuthority.positionErrorFactor, 1.0f, 0.001f);
    ok &= expectNear("held authority records rotation error factor", weakHeldAuthority.rotationErrorFactor, 1.0f, 0.001f);
    ok &= expectNear("weak held authority gates direct angular assist", weakHeldAuthority.angularVelocityAssistScale, 0.30f, 0.001f);
    ok &= expectNear("weak held authority gates release angular velocity", weakHeldAuthority.releaseAngularVelocityScale, 0.30f, 0.001f);

    const auto contactHeldAuthority = evaluateHeldAuthority(HeldAuthorityInput{
        .angular = AngularAuthorityInput{
            .enabled = true,
            .positionOnlyPivot = false,
            .normalTrusted = true,
            .contactPatchUsedAsPivot = true,
            .contactPatchSampleCount = 4,
            .contactSupportShape = ContactSupportShape::Surface,
        },
        .heldBodyColliding = true,
    });
    ok &= expectTrue("contact held authority marks softening", contactHeldAuthority.softenForContact);
    ok &= expectNear("contact held authority caps angular assist scale", contactHeldAuthority.angularVelocityAssistScale, 0.75f, 0.001f);
    ok &= expectNear("contact held authority caps release angular scale", contactHeldAuthority.releaseAngularVelocityScale, 0.75f, 0.001f);

    const float authorityCap = computeAuthorityScaledAngularVelocityCap(18.0f, 0.30f, 0.50f);
    ok &= expectNear("authority angular cap composes support and long-object scale", authorityCap, 2.70f, 0.001f);

    const auto touchHeldSurfaceVisual = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = true,
        .acquisitionVisualEligible = false,
        .hasPivotTrackingError = true,
        .pivotAuthorityNormalTrusted = true,
        .angularAuthorityScale = 1.0f,
        .contactSupportShape = ContactSupportShape::Surface,
    });
    ok &= expectTrue("touch-held surface support may publish visual hand", touchHeldSurfaceVisual.apply);
    ok &= expectFalse("touch-held visual publish is not acquisition", touchHeldSurfaceVisual.acquisition);

    const auto weakPointTouchVisual = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = true,
        .acquisitionVisualEligible = false,
        .hasPivotTrackingError = true,
        .pivotAuthorityPositionOnly = true,
        .pivotAuthorityNormalTrusted = false,
        .angularAuthorityScale = 0.75f,
        .contactSupportShape = ContactSupportShape::Point,
    });
    ok &= expectFalse("touch-held weak point support does not publish visual hand", weakPointTouchVisual.apply);

    const auto weakNormalTouchVisual = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = true,
        .acquisitionVisualEligible = false,
        .hasPivotTrackingError = true,
        .pivotAuthorityPositionOnly = false,
        .pivotAuthorityNormalTrusted = false,
        .angularAuthorityScale = 0.75f,
        .contactSupportShape = ContactSupportShape::Unknown,
    });
    ok &= expectFalse("touch-held untrusted normal support does not publish visual hand", weakNormalTouchVisual.apply);

    const auto seatedPointTouchVisual = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = true,
        .acquisitionVisualEligible = false,
        .hasPivotTrackingError = true,
        .pivotAuthorityPositionOnly = true,
        .pivotAuthorityNormalTrusted = false,
        .hasSeatedPivotReacquire = true,
        .angularAuthorityScale = 0.35f,
        .contactSupportShape = ContactSupportShape::Point,
    });
    ok &= expectTrue("seated point support may publish visual hand", seatedPointTouchVisual.apply);

    const auto acquisitionSurfaceVisual = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = false,
        .acquisitionVisualEligible = true,
        .hasPivotTrackingError = true,
        .pivotAuthorityNormalTrusted = true,
        .angularAuthorityScale = 0.75f,
        .contactSupportShape = ContactSupportShape::Surface,
    });
    ok &= expectTrue("strong acquisition support may publish visual hand", acquisitionSurfaceVisual.apply);
    ok &= expectTrue("strong acquisition visual publish is marked acquisition", acquisitionSurfaceVisual.acquisition);

    const auto acquisitionLowAuthorityVisual = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = false,
        .acquisitionVisualEligible = true,
        .hasPivotTrackingError = true,
        .pivotAuthorityNormalTrusted = true,
        .angularAuthorityScale = 0.54f,
        .contactSupportShape = ContactSupportShape::Surface,
    });
    ok &= expectFalse("low angular authority blocks acquisition visual hand", acquisitionLowAuthorityVisual.apply);

    const auto acquisitionContactSoftenedVisual = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = false,
        .acquisitionVisualEligible = true,
        .hasPivotTrackingError = true,
        .motorContactSoftening = true,
        .pivotAuthorityNormalTrusted = true,
        .angularAuthorityScale = 0.75f,
        .contactSupportShape = ContactSupportShape::Surface,
    });
    ok &= expectFalse("push-into-contact blocks acquisition visual hand", acquisitionContactSoftenedVisual.apply);

    const auto acquisitionWeakPointVisual = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = false,
        .acquisitionVisualEligible = true,
        .hasPivotTrackingError = true,
        .pivotAuthorityPositionOnly = true,
        .pivotAuthorityNormalTrusted = false,
        .angularAuthorityScale = 0.75f,
        .contactSupportShape = ContactSupportShape::Point,
    });
    ok &= expectFalse("weak point support blocks acquisition visual hand", acquisitionWeakPointVisual.apply);

    const auto acquisitionWeakNormalVisual = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = false,
        .acquisitionVisualEligible = true,
        .hasPivotTrackingError = true,
        .pivotAuthorityPositionOnly = false,
        .pivotAuthorityNormalTrusted = false,
        .angularAuthorityScale = 0.75f,
        .contactSupportShape = ContactSupportShape::Unknown,
    });
    ok &= expectFalse("untrusted normal support blocks acquisition visual hand", acquisitionWeakNormalVisual.apply);

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

    ok &= expectNear("long object disabled angular scale",
        computeLongObjectAngularSpeedScale(false, 96.0f, 24.0f, 0.35f),
        1.0f,
        0.001f);
    ok &= expectNear("short object angular scale",
        computeLongObjectAngularSpeedScale(true, 12.0f, 24.0f, 0.35f),
        1.0f,
        0.001f);
    ok &= expectNear("long object angular scale",
        computeLongObjectAngularSpeedScale(true, 48.0f, 24.0f, 0.35f),
        0.5f,
        0.001f);
    ok &= expectNear("very long object angular scale floor",
        computeLongObjectAngularSpeedScale(true, 200.0f, 24.0f, 0.35f),
        0.35f,
        0.001f);

    return ok ? 0 : 1;
}
