#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/grab/GrabHeldObject.h"
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
    singleHand.mass = 2.0f;
    singleHand.maximumInertia = 0.5f;
    singleHand.fadeInEnabled = false;
    const auto single = solveMotorTargetsWithAuthority(singleHand, HeldAuthorityState{});
    ok &= expectTrue("complete physical input produces a drive", single.valid);
    ok &= expectNear("free grip acceleration", single.linearMaxForce / singleHand.mass, 1000.0f, 0.001f);
    ok &= expectNear("centered angular capacity follows inertia", single.angularMaxForce, 3000.0f, 0.001f);

    HeldAuthorityState contact{};
    contact.softenForContact = true;
    const auto touching = solveMotorTargetsWithAuthority(singleHand, contact);
    ok &= expectNear("light contact keeps existing mass cap", touching.linearMaxForce, 1000.0f, 0.001f);
    ok &= expectNear("light contact keeps existing angular cap", touching.angularMaxForce, 1000.0f, 0.001f);

    auto shared = singleHand;
    shared.authorityForceScale = 0.5f;
    const auto twoHand = solveMotorTargetsWithAuthority(shared, HeldAuthorityState{});
    ok &= expectNear("two shares retain total linear capacity", twoHand.linearMaxForce * 2, single.linearMaxForce, 0.001f);
    ok &= expectNear("two shares retain total angular capacity", twoHand.angularMaxForce * 2, single.angularMaxForce, 0.001f);

    for (float mass : {0.02f, 2.0f, 10.0f, 50.0f, 70.0171f}) {
        auto body = singleHand;
        body.mass = mass;
        body.effectiveMotorMassFloorEnabled = false;
        const auto free = solveMotorTargetsWithAuthority(body, HeldAuthorityState{});
        const auto blocked = solveMotorTargetsWithAuthority(body, contact);
        ok &= expectNear("free acceleration does not collapse for heavy bodies", free.linearMaxForce / mass, 1000.0f, 0.001f);
        ok &= expectTrue("contact does not inherit free grip force", blocked.linearMaxForce <= 2000.0f);
    }

    auto heavy = singleHand;
    heavy.mass = 70.0171f;
    heavy.maximumInertia = 3.0f;
    const auto centered = solveMotorTargetsWithAuthority(heavy, HeldAuthorityState{});
    heavy.gripRadiusHavok = 0.25f;
    const auto offset = solveMotorTargetsWithAuthority(heavy, HeldAuthorityState{});
    ok &= expectNear("grip lever does not increase linear authority", offset.linearMaxForce, centered.linearMaxForce, 0.001f);
    ok &= expectTrue("offset grip has capacity to oppose linear torque", offset.angularMaxForce > centered.angularMaxForce);
    ok &= expectNear("lever capacity covers simultaneous three-axis effort", offset.angularMaxForce - centered.angularMaxForce,
        0.25f * std::sqrt(3.0f) * centered.linearMaxForce, 0.01f);
    const auto heavyContact = solveMotorTargetsWithAuthority(heavy, contact);
    ok &= expectNear("heavy contact linear ceiling preserved", heavyContact.linearMaxForce, 2000.0f, 0.001f);
    ok &= expectNear("heavy contact angular ceiling preserved", heavyContact.angularMaxForce, 2000.0f, 0.001f);

    auto weapon = heavy;
    weapon.baseMaxForce = 9000.0f;
    weapon.angularForceMultiplier = 2.0f;
    const auto weaponContact = solveMotorTargetsWithAuthority(weapon, contact);
    const auto weaponFree = solveMotorTargetsWithAuthority(weapon, HeldAuthorityState{});
    ok &= expectNear("weapon contact linear ceiling preserved", weaponContact.linearMaxForce, 9000.0f, 0.001f);
    ok &= expectNear("weapon contact angular ceiling preserved", weaponContact.angularMaxForce, 18000.0f, 0.001f);
    ok &= expectNear("weapon classification does not duplicate free linear budget", weaponFree.linearMaxForce, offset.linearMaxForce, 0.001f);
    ok &= expectNear("weapon classification does not duplicate free angular budget", weaponFree.angularMaxForce, offset.angularMaxForce, 0.001f);

    for (float dt : {1.0f/45, 1.0f/72, 1.0f/90, 1.0f/144, 0.0075f, 0.014f}) {
        auto timed = heavy;
        timed.physicsDeltaSeconds = dt;
        timed.deltaTime = dt;
        const auto output = solveMotorTargetsWithAuthority(timed, HeldAuthorityState{});
        ok &= expectNear("force independent of native substep rate", output.linearMaxForce, offset.linearMaxForce, 0.001f);
        ok &= expectNear("torque independent of native substep rate", output.angularMaxForce, offset.angularMaxForce, 0.001f);
        ok &= expectNear("telemetry uses measured physics rate", output.physicsHz, 1.0f/dt, 0.001f);
    }
    ok &= expectNear("unmeasured rate stays unavailable", computePhysicsHz(0.0f), 0.0f, 0.001f);

    auto fading = heavy;
    fading.fadeInEnabled = true;
    fading.fadeElapsed = 0.05f;
    fading.fadeDuration = 0.1f;
    const auto halfFade = solveMotorTargetsWithAuthority(fading, HeldAuthorityState{});
    ok &= expectNear("startup fade scales free linear capacity", halfFade.linearMaxForce * 2, offset.linearMaxForce, 0.001f);
    ok &= expectNear("startup fade scales free angular capacity", halfFade.angularMaxForce * 2, offset.angularMaxForce, 0.001f);

    auto tiny = singleHand;
    tiny.mass = 0.02f;
    tiny.maximumInertia = 0.005f;
    const auto floored = solveMotorTargetsWithAuthority(tiny, HeldAuthorityState{});
    ok &= expectNear("one motor mass floor retained", floored.linearMaxForce, 2000.0f, 0.001f);
    tiny.effectiveMotorMassFloorEnabled = false;
    const auto unfloored = solveMotorTargetsWithAuthority(tiny, HeldAuthorityState{});
    ok &= expectNear("disabled floor uses actual tiny mass", unfloored.linearMaxForce, 20.0f, 0.001f);
    ok &= expectNear("inertia independent of motor mass floor", unfloored.angularMaxForce, floored.angularMaxForce, 0.001f);

    auto invalid = heavy;
    invalid.maximumInertia = 0;
    ok &= expectFalse("missing inertia rejects drive", solveMotorTargetsWithAuthority(invalid, {}).valid);
    invalid = heavy;
    invalid.mass = -1;
    ok &= expectFalse("invalid mass rejects drive", solveMotorTargetsWithAuthority(invalid, {}).valid);
    invalid = heavy;
    invalid.gripRadiusHavok = std::numeric_limits<float>::quiet_NaN();
    ok &= expectFalse("invalid lever rejects drive", solveMotorTargetsWithAuthority(invalid, {}).valid);
    invalid = heavy;
    invalid.freeLinearAcceleration = std::numeric_limits<float>::max();
    ok &= expectFalse("overflow rejects drive", solveMotorTargetsWithAuthority(invalid, {}).valid);

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

    const auto trustedAuthority = computeAngularAuthorityScale(AngularAuthorityInput{
        .enabled = true,
        .positionOnlyPivot = false,
        .normalTrusted = true,
        .contactPatchEvidence = true,
        .contactPatchSampleCount = 4,
        .longObjectLeverGameUnits = 8.0f,
    });
    ok &= expectNear("trusted small sphere-like support softens release orientation safety", trustedAuthority.authorityScale, 0.65f, 0.001f);
    ok &= expectTrue("trusted small support classifies as sphere-like", trustedAuthority.contactSupportShape == ContactSupportShape::SphereLike);
    ok &= expectNear("sphere-like support limits contact-normal spin", trustedAuthority.contactNormalScale, 0.30f, 0.001f);

    const auto lowSupportAuthority = computeAngularAuthorityScale(AngularAuthorityInput{
        .enabled = true,
        .positionOnlyPivot = false,
        .normalTrusted = true,
        .contactPatchEvidence = true,
        .contactPatchSampleCount = 1,
        .longObjectLeverGameUnits = 8.0f,
    });
    ok &= expectNear("small low-support contact softens release angular safety", lowSupportAuthority.authorityScale, 0.4225f, 0.001f);
    ok &= expectTrue("single-hit small support is sphere-like not surface-authoritative", lowSupportAuthority.contactSupportShape == ContactSupportShape::SphereLike);

    const auto rejectedPatchAuthority = computeAngularAuthorityScale(AngularAuthorityInput{
        .enabled = true,
        .positionOnlyPivot = false,
        .normalTrusted = true,
        .contactPatchEvidence = false,
        .contactPatchSampleCount = 1,
        .longObjectLeverGameUnits = 8.0f,
    });
    ok &= expectNear("trusted non-patch pivot ignores rejected patch support", rejectedPatchAuthority.authorityScale, 1.0f, 0.001f);

    const auto trustedPointAuthority = computeAngularAuthorityScale(AngularAuthorityInput{
        .enabled = true,
        .positionOnlyPivot = false,
        .normalTrusted = true,
        .contactPatchEvidence = true,
        .contactPatchSampleCount = 1,
        .longObjectLeverGameUnits = 20.0f,
    });
    ok &= expectTrue("trusted single-point support classifies as point", trustedPointAuthority.contactSupportShape == ContactSupportShape::Point);
    ok &= expectNear("trusted point limits twist around grab point", trustedPointAuthority.twistScale, 0.35f, 0.001f);

    const auto longHandleAuthority = computeAngularAuthorityScale(AngularAuthorityInput{
        .enabled = true,
        .positionOnlyPivot = false,
        .normalTrusted = true,
        .contactPatchEvidence = true,
        .contactPatchSampleCount = 2,
        .longObjectLeverGameUnits = 72.0f,
        .longObjectReferenceLeverGameUnits = 24.0f,
    });
    ok &= expectTrue("two-hit long object classifies as long handle", longHandleAuthority.contactSupportShape == ContactSupportShape::LongHandle);
    ok &= expectNear("long handle applies line-support authority", longHandleAuthority.authorityScale, 0.75f, 0.001f);
    ok &= expectNear("long handle does not fake twist damping", longHandleAuthority.twistScale, 1.0f, 0.001f);
    ok &= expectFalse("long handle alone is not axis-limited", longHandleAuthority.axisLimited);

    const auto unsupportedLongHandleAuthority = computeAngularAuthorityScale(AngularAuthorityInput{
        .enabled = true,
        .positionOnlyPivot = false,
        .normalTrusted = true,
        .contactPatchEvidence = false,
        .contactPatchSampleCount = 0,
        .longObjectLeverGameUnits = 72.0f,
        .longObjectReferenceLeverGameUnits = 24.0f,
    });
    ok &= expectTrue("unsupported long object still reports long handle shape", unsupportedLongHandleAuthority.contactSupportShape == ContactSupportShape::LongHandle);
    ok &= expectNear("long object length alone does not reduce authority", unsupportedLongHandleAuthority.authorityScale, 1.0f, 0.001f);

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
            .contactPatchEvidence = true,
            .contactPatchSampleCount = 1,
            .longObjectLeverGameUnits = 8.0f,
        },
        .heldBodyColliding = false,
    });
    ok &= expectNear("weak held authority gates release angular velocity", weakHeldAuthority.releaseAngularVelocityScale, 0.30f, 0.001f);
    ok &= expectNear("weak held authority keeps twist safety for release only",
        weakHeldAuthority.angular.weakPivotTwistScale,
        0.35f,
        0.001f);

    const auto contactHeldAuthority = evaluateHeldAuthority(HeldAuthorityInput{
        .angular = AngularAuthorityInput{
            .enabled = true,
            .positionOnlyPivot = false,
            .normalTrusted = true,
            .contactPatchEvidence = true,
            .contactPatchSampleCount = 4,
            .contactSupportShape = ContactSupportShape::Surface,
        },
        .heldBodyColliding = true,
    });
    ok &= expectTrue("contact held authority marks softening", contactHeldAuthority.softenForContact);
    ok &= expectNear("contact held authority caps release angular scale", contactHeldAuthority.releaseAngularVelocityScale, 0.75f, 0.001f);

    const float authorityCap = computeAuthorityScaledAngularVelocityCap(18.0f, 0.30f, 0.50f);
    ok &= expectNear("release angular cap composes support and long-object scale", authorityCap, 2.70f, 0.001f);

    const auto seatedImmediatePromotion = evaluateSeatedPalmPocketPromotion(SeatedPalmPocketPromotionInput{
        .weakMeshStart = true,
        .hasSeatedCandidate = true,
        .reachedTouchRange = true,
        .candidateNormalTrusted = true,
        .supportPatchValid = true,
        .supportPatchNormalTrusted = true,
        .currentContactPatchSampleCount = 1,
        .supportPatchSampleCount = 5,
        .candidateLocalDeltaGameUnits = 2.0f,
        .immediateMaxLocalDeltaGameUnits = 4.0f,
        .lerpMaxLocalDeltaGameUnits = 12.0f,
    });
    ok &= expectTrue("seated palm pocket promotion replaces small weak mesh delta", seatedImmediatePromotion.promotePivot);
    ok &= expectTrue("seated palm pocket promotion completes small delta", seatedImmediatePromotion.completeSeatedRelation);
    ok &= expectFalse("seated palm pocket promotion does not create support-only authority", seatedImmediatePromotion.enrichSupport);
    ok &= expectNear("seated palm pocket immediate blend", seatedImmediatePromotion.pivotBlend, 1.0f, 0.001f);

    const auto seatedLerpPromotion = evaluateSeatedPalmPocketPromotion(SeatedPalmPocketPromotionInput{
        .weakMeshStart = true,
        .hasSeatedCandidate = true,
        .reachedTouchRange = true,
        .supportPatchValid = true,
        .supportPatchNormalTrusted = true,
        .currentContactPatchSampleCount = 1,
        .supportPatchSampleCount = 4,
        .candidateLocalDeltaGameUnits = 8.0f,
        .immediateMaxLocalDeltaGameUnits = 4.0f,
        .lerpMaxLocalDeltaGameUnits = 12.0f,
    });
    ok &= expectFalse("seated palm pocket promotion rejects medium delta instead of blending", seatedLerpPromotion.promotePivot);
    ok &= expectFalse("seated palm pocket medium delta does not complete relation", seatedLerpPromotion.completeSeatedRelation);
    ok &= expectNear("seated palm pocket medium blend is removed", seatedLerpPromotion.pivotBlend, 0.0f, 0.001f);
    ok &= expectReason("seated palm pocket medium delta reason",
        seatedLerpPromotion.reason,
        "seatedPalmPocketPromotionCandidateTooFarKeepFrozen");

    const auto seatedContactSupportOnly = evaluateSeatedPalmPocketPromotion(SeatedPalmPocketPromotionInput{
        .weakMeshStart = true,
        .hasSeatedCandidate = true,
        .timedOutInsidePocket = true,
        .motorContactSoftening = true,
        .supportPatchValid = true,
        .currentContactPatchSampleCount = 1,
        .supportPatchSampleCount = 4,
        .candidateLocalDeltaGameUnits = 6.0f,
        .immediateMaxLocalDeltaGameUnits = 4.0f,
        .lerpMaxLocalDeltaGameUnits = 12.0f,
    });
    ok &= expectFalse("seated palm pocket contact softening blocks pivot move", seatedContactSupportOnly.promotePivot);
    ok &= expectFalse("seated palm pocket contact softening cannot enrich support authority", seatedContactSupportOnly.enrichSupport);
    ok &= expectReason("seated palm pocket contact softening reason",
        seatedContactSupportOnly.reason,
        "seatedPalmPocketPromotionContactSofteningKeepFrozen");

    const auto seatedTimeoutPromotion = evaluateSeatedPalmPocketPromotion(SeatedPalmPocketPromotionInput{
        .weakMeshStart = true,
        .hasSeatedCandidate = true,
        .timedOutInsidePocket = true,
        .motorContactSoftening = false,
        .candidateNormalTrusted = true,
        .supportPatchValid = true,
        .supportPatchNormalTrusted = true,
        .currentContactPatchSampleCount = 1,
        .supportPatchSampleCount = 5,
        .candidateLocalDeltaGameUnits = 2.0f,
        .immediateMaxLocalDeltaGameUnits = 4.0f,
        .lerpMaxLocalDeltaGameUnits = 12.0f,
    });
    ok &= expectTrue("seated timeout promotion accepts safe settled pivot", seatedTimeoutPromotion.promotePivot);
    ok &= expectTrue("seated timeout promotion completes safe settled relation", seatedTimeoutPromotion.completeSeatedRelation);
    ok &= expectReason("seated timeout promotion reason",
        seatedTimeoutPromotion.reason,
        "seatedPalmPocketPromotionImmediate");

    const auto seatedLargeDeltaSupportOnly = evaluateSeatedPalmPocketPromotion(SeatedPalmPocketPromotionInput{
        .weakMeshStart = true,
        .hasSeatedCandidate = true,
        .reachedTouchRange = true,
        .supportPatchValid = true,
        .supportPatchNormalTrusted = true,
        .currentContactPatchSampleCount = 1,
        .supportPatchSampleCount = 5,
        .candidateLocalDeltaGameUnits = 20.0f,
        .immediateMaxLocalDeltaGameUnits = 4.0f,
        .lerpMaxLocalDeltaGameUnits = 12.0f,
    });
    ok &= expectFalse("seated palm pocket large delta keeps current pivot", seatedLargeDeltaSupportOnly.promotePivot);
    ok &= expectFalse("seated palm pocket large delta does not publish support-only authority", seatedLargeDeltaSupportOnly.enrichSupport);
    ok &= expectReason("seated palm pocket large delta reason",
        seatedLargeDeltaSupportOnly.reason,
        "seatedPalmPocketPromotionCandidateTooFarKeepFrozen");

    const auto seatedProgrammaticArrival = evaluateSeatedPalmPocketPromotion(SeatedPalmPocketPromotionInput{
        .weakMeshStart = true,
        .hasSeatedCandidate = true,
        .reachedTouchRange = true,
        .candidateNormalTrusted = true,
        .supportPatchValid = true,
        .supportPatchNormalTrusted = true,
        .programmaticArrival = true,
        .supportPatchSampleCount = 5,
        .candidateLocalDeltaGameUnits = 20.0f,
        .immediateMaxLocalDeltaGameUnits = 4.0f,
        .lerpMaxLocalDeltaGameUnits = 12.0f,
    });
    ok &= expectTrue("programmatic arrival replaces stale far-ray seat", seatedProgrammaticArrival.promotePivot);
    ok &= expectTrue("programmatic arrival completes verified seated relation", seatedProgrammaticArrival.completeSeatedRelation);
    ok &= expectReason("programmatic arrival seated promotion reason",
        seatedProgrammaticArrival.reason,
        "seatedProgrammaticArrivalPromotion");

    const auto seatedProgrammaticArrivalNoSupport = evaluateSeatedPalmPocketPromotion(SeatedPalmPocketPromotionInput{
        .weakMeshStart = true,
        .hasSeatedCandidate = true,
        .reachedTouchRange = true,
        .candidateNormalTrusted = true,
        .supportPatchValid = false,
        .supportPatchNormalTrusted = false,
        .programmaticArrival = true,
        .supportPatchSampleCount = 0,
        .candidateLocalDeltaGameUnits = 20.0f,
        .immediateMaxLocalDeltaGameUnits = 4.0f,
        .lerpMaxLocalDeltaGameUnits = 12.0f,
    });
    ok &= expectTrue("programmatic arrival promotes a trusted mesh pivot without a support patch",
        seatedProgrammaticArrivalNoSupport.promotePivot);
    ok &= expectTrue("programmatic arrival without support completes the seated relation",
        seatedProgrammaticArrivalNoSupport.completeSeatedRelation);
    ok &= expectReason("programmatic arrival without support reason",
        seatedProgrammaticArrivalNoSupport.reason,
        "seatedProgrammaticArrivalPromotion");

    const auto seatedProgrammaticArrivalUntrusted = evaluateSeatedPalmPocketPromotion(SeatedPalmPocketPromotionInput{
        .weakMeshStart = true,
        .hasSeatedCandidate = true,
        .reachedTouchRange = true,
        .candidateNormalTrusted = false,
        .supportPatchValid = false,
        .programmaticArrival = true,
        .candidateLocalDeltaGameUnits = 20.0f,
        .immediateMaxLocalDeltaGameUnits = 4.0f,
        .lerpMaxLocalDeltaGameUnits = 12.0f,
    });
    ok &= expectFalse("programmatic arrival keeps the frozen seat without a trusted pivot or support",
        seatedProgrammaticArrivalUntrusted.promotePivot);
    ok &= expectReason("programmatic arrival untrusted reason",
        seatedProgrammaticArrivalUntrusted.reason,
        "seatedSupportGroupPromotionMissingSupport");

    const auto seatedNotWeak = evaluateSeatedPalmPocketPromotion(SeatedPalmPocketPromotionInput{
        .weakMeshStart = false,
        .hasSeatedCandidate = true,
        .reachedTouchRange = true,
        .candidateLocalDeltaGameUnits = 2.0f,
    });
    ok &= expectFalse("seated palm pocket promotion ignores non-weak authority", seatedNotWeak.promotePivot);
    ok &= expectReason("seated palm pocket non-weak reason", seatedNotWeak.reason, "seatedPalmPocketPromotionNotWeakMesh");

    // A transferred authored hold must keep its hand target during the new
    // body's seat acquisition, including contact, without enabling ordinary
    // unseated grabs or allowing missing physical evidence.
    VisualHandPublishInput transferredGrip{
        .hasTelemetryCapture = true,
        .hasPivotTrackingError = true,
        .motorContactSoftening = true,
        .requiresSettledVisualRelation = true,
        .transferredAuthoredGrip = true,
    };
    const auto transferredVisual = evaluateVisualHandPublishGate(transferredGrip);
    ok &= expectTrue("transferred grip retains visual authority before settling", transferredVisual.apply);
    ok &= expectFalse("transferred grip does not restart hand acquisition", transferredVisual.acquisition);
    transferredGrip.hasTelemetryCapture = false;
    ok &= expectFalse("transferred grip still requires a frozen relation", evaluateVisualHandPublishGate(transferredGrip).apply);
    transferredGrip.hasTelemetryCapture = true;
    transferredGrip.hasPivotTrackingError = false;
    ok &= expectFalse("transferred grip still requires live body tracking", evaluateVisualHandPublishGate(transferredGrip).apply);
    transferredGrip.hasPivotTrackingError = true;
    transferredGrip.transferredAuthoredGrip = false;
    ok &= expectFalse("ordinary unseated grab remains gated", evaluateVisualHandPublishGate(transferredGrip).apply);

    const auto touchHeldSurfaceVisual = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = true,
        .acquisitionVisualEligible = false,
        .hasPivotTrackingError = true,
        .pivotAuthorityNormalTrusted = true,
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
        .contactSupportShape = ContactSupportShape::Point,
    });
    ok &= expectTrue("touch-held weak point support may publish visual hand after seated relation", weakPointTouchVisual.apply);

    const auto weakNormalTouchVisual = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = true,
        .acquisitionVisualEligible = false,
        .hasPivotTrackingError = true,
        .pivotAuthorityPositionOnly = false,
        .pivotAuthorityNormalTrusted = false,
        .contactSupportShape = ContactSupportShape::Unknown,
    });
    ok &= expectTrue("touch-held untrusted normal support may publish visual hand after seated relation", weakNormalTouchVisual.apply);

    const auto seatedPointTouchVisual = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = true,
        .acquisitionVisualEligible = false,
        .hasPivotTrackingError = true,
        .pivotAuthorityPositionOnly = true,
        .pivotAuthorityNormalTrusted = false,
        .hasSeatedPivotReacquire = true,
        .contactSupportShape = ContactSupportShape::Point,
    });
    ok &= expectTrue("seated point support may publish visual hand", seatedPointTouchVisual.apply);

    const auto acquisitionSurfaceVisual = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = false,
        .acquisitionVisualEligible = true,
        .hasPivotTrackingError = true,
        .pivotAuthorityNormalTrusted = true,
        .contactSupportShape = ContactSupportShape::Surface,
    });
    ok &= expectTrue("strong acquisition support may publish visual hand", acquisitionSurfaceVisual.apply);
    ok &= expectTrue("strong acquisition visual publish is marked acquisition", acquisitionSurfaceVisual.acquisition);

    const auto acquisitionAwaitingSettledRelation = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = false,
        .acquisitionVisualEligible = true,
        .hasPivotTrackingError = true,
        .pivotAuthorityNormalTrusted = true,
        .requiresSettledVisualRelation = true,
        .contactSupportShape = ContactSupportShape::Surface,
    });
    ok &= expectFalse("acquisition visual hand waits for settled relation", acquisitionAwaitingSettledRelation.apply);
    ok &= expectReason("acquisition visual hand settled wait reason", acquisitionAwaitingSettledRelation.reason, "awaitingSettledVisualRelation");

    const auto touchHeldAwaitingSettledRelation = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = true,
        .acquisitionVisualEligible = false,
        .hasPivotTrackingError = true,
        .pivotAuthorityNormalTrusted = true,
        .requiresSettledVisualRelation = true,
        .contactSupportShape = ContactSupportShape::Surface,
    });
    ok &= expectFalse("touch-held visual hand waits for settled relation", touchHeldAwaitingSettledRelation.apply);

    const auto seatedSettledRelationVisual = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = true,
        .acquisitionVisualEligible = false,
        .hasPivotTrackingError = true,
        .pivotAuthorityNormalTrusted = true,
        .hasSeatedPivotReacquire = true,
        .requiresSettledVisualRelation = true,
        .contactSupportShape = ContactSupportShape::Surface,
    });
    ok &= expectTrue("seated settled relation may publish visual hand", seatedSettledRelationVisual.apply);

    const auto acquisitionContactSoftenedVisual = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = false,
        .acquisitionVisualEligible = true,
        .hasPivotTrackingError = true,
        .motorContactSoftening = true,
        .pivotAuthorityNormalTrusted = true,
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
        .contactSupportShape = ContactSupportShape::Point,
    });
    ok &= expectTrue("weak point support does not block acquisition visual hand", acquisitionWeakPointVisual.apply);

    const auto acquisitionWeakNormalVisual = evaluateVisualHandPublishGate(VisualHandPublishInput{
        .hasTelemetryCapture = true,
        .touchHeldPhase = false,
        .acquisitionVisualEligible = true,
        .hasPivotTrackingError = true,
        .pivotAuthorityPositionOnly = false,
        .pivotAuthorityNormalTrusted = false,
        .contactSupportShape = ContactSupportShape::Unknown,
    });
    ok &= expectTrue("untrusted normal support does not block acquisition visual hand", acquisitionWeakNormalVisual.apply);

    ok &= expectTrue("acquisition visual hand may smooth into object-relative pose",
        rock::hand_visual_lerp_math::shouldSmoothHeldObjectRelativeHand(true, false, true));
    ok &= expectFalse("touch-held visual hand tracks object-relative pose immediately",
        rock::hand_visual_lerp_math::shouldSmoothHeldObjectRelativeHand(true, true, false));
    ok &= expectFalse("disabled visual hand lerp tracks object-relative pose immediately",
        rock::hand_visual_lerp_math::shouldSmoothHeldObjectRelativeHand(false, false, true));

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

    {
        using namespace rock::grab_held_response;
        ReleaseVelocityInput<Vec3> input{
            .hasHandLocalVelocity = true,
            .hasObjectLocalVelocity = true,
            .handLocalVelocityHavok = { 0.0f, 4.0f, 0.0f },
            .objectLocalVelocityHavok = { 3.0f, 2.0f, -1.0f },
        };
        const auto forward = composeControllerReleaseVelocity(input);
        ok &= expectNear("forward swing has no sideways motor drift", forward.x, 0.0f, 0.001f);
        ok &= expectNear("forward throw uses movement speed once", forward.y, 6.0f, 0.001f);
        ok &= expectNear("forward swing has no vertical motor drift", forward.z, 0.0f, 0.001f);

        input.handLocalVelocityHavok = { -4.0f, 0.0f, 0.0f };
        const auto sideways = composeControllerReleaseVelocity(input);
        ok &= expectNear("sideways swing follows sideways movement", sideways.x, -6.0f, 0.001f);
        ok &= expectNear("sideways swing ignores forward object motion", sideways.y, 0.0f, 0.001f);

        input.handLocalVelocityHavok = {};
        const auto drop = composeControllerReleaseVelocity(input);
        ok &= expectNear("stationary hand drops without object drift", length(drop), 0.0f, 0.001f);
        const auto spin = composeControllerReleaseAngularVelocity(ReleaseAngularVelocityInput<Vec3>{
            .hasHandAngularVelocity = true,
            .handAngularVelocityRadiansPerSecond = { 0.0f, 0.0f, 5.0f },
        });
        ok &= expectNear("wrist rotation still supplies spin", spin.z, 5.0f, 0.001f);

        input.handLocalVelocityHavok = { 12.0f, 16.0f, 0.0f };
        const auto capped = composeControllerReleaseVelocity(input);
        ok &= expectNear("speed cap preserves movement direction x", capped.x, 7.2f, 0.001f);
        ok &= expectNear("speed cap preserves movement direction y", capped.y, 9.6f, 0.001f);

        input.hasHandLocalVelocity = false;
        const auto objectOnly = composeControllerReleaseVelocity(input);
        ok &= expectNear("unavailable hand retains object momentum x", objectOnly.x, 4.5f, 0.001f);
        ok &= expectNear("unavailable hand retains object momentum y", objectOnly.y, 3.0f, 0.001f);
        input.hasHandLocalVelocity = true;
        input.controllerDerivedEnabled = false;
        const auto controllerDisabled = composeControllerReleaseVelocity(input);
        ok &= expectNear("disabled controller mode retains object momentum", controllerDisabled.x, objectOnly.x, 0.001f);
    }

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
