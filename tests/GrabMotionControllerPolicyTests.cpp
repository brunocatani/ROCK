#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/grab/GrabInertiaPolicy.h"
#include "physics-interaction/grab/HeldMassMovement.h"
#include "physics-interaction/hand/HandVisual.h"

#include <cstdio>
#include <cstring>

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
}

int main()
{
    using namespace rock::grab_motion_controller;

    bool ok = true;

    MotorInput singleHand{};
    singleHand.baseMaxForce = 2000.0f;
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
    const auto mediumMassOutput = solveMotorTargets(mediumMass);
    ok &= expectNear("medium generic object keeps fixed HIGGS-style force", mediumMassOutput.linearMaxForce, 2000.0f, 0.001f);
    ok &= expectNear("medium generic object angular force follows fixed force", mediumMassOutput.angularMaxForce, 160.0f, 0.001f);

    MotorInput heavyMass = singleHand;
    heavyMass.mass = 50.0f;
    const auto heavyMassOutput = solveMotorTargets(heavyMass);
    ok &= expectNear("heavy generic object does not receive loose-weapon force", heavyMassOutput.linearMaxForce, 2000.0f, 0.001f);
    ok &= expectNear("heavy generic object angular force follows generic force", heavyMassOutput.angularMaxForce, 160.0f, 0.001f);

    MotorInput looseWeapon = singleHand;
    looseWeapon.baseMaxForce = 9000.0f;
    looseWeapon.mass = 1000.0f;
    const auto looseWeaponOutput = solveMotorTargets(looseWeapon);
    ok &= expectNear("loose weapon base force is not double-boosted", looseWeaponOutput.linearMaxForce, 9000.0f, 0.001f);
    ok &= expectNear("loose weapon angular force follows existing weapon ceiling", looseWeaponOutput.angularMaxForce, 720.0f, 0.001f);

    MotorInput angularFixedTau = singleHand;
    angularFixedTau.mass = 100.0f;
    angularFixedTau.currentLinearTau = 0.8f;
    angularFixedTau.currentAngularTau = 0.8f;
    angularFixedTau.deltaTime = 1.0f;
    angularFixedTau.tauLerpSpeed = 1.0f;
    const auto angularFixedTauOutput = solveMotorTargets(angularFixedTau);
    ok &= expectNear("angular follow does not boost linear force", angularFixedTauOutput.linearMaxForce, 2000.0f, 0.001f);
    ok &= expectNear("linear follow keeps HIGGS-style tau fixed", angularFixedTauOutput.linearTau, 0.03f, 0.001f);
    ok &= expectNear("angular follow keeps HIGGS-style tau fixed", angularFixedTauOutput.angularTau, 0.03f, 0.001f);
    ok &= expectNear("full support leaves angular force authority uncapped", angularFixedTauOutput.angularAuthorityScale, 1.0f, 0.001f);

    MotorInput positionOnlyPivot = singleHand;
    positionOnlyPivot.pivotQualityAngularScalingEnabled = true;
    positionOnlyPivot.pivotAuthorityPositionOnly = true;
    positionOnlyPivot.pivotAuthorityNormalTrusted = false;
    positionOnlyPivot.contactPatchUsedAsPivot = true;
    positionOnlyPivot.contactPatchSampleCount = 1;
    positionOnlyPivot.longObjectLeverGameUnits = 8.0f;
    const auto positionOnlyOutput = solveMotorTargets(positionOnlyPivot);
    ok &= expectNear("position-only small weak pivot clamps angular authority", positionOnlyOutput.angularAuthorityScale, 0.30f, 0.001f);
    ok &= expectNear("position-only small weak pivot does not reduce held angular force", positionOnlyOutput.angularMaxForce, 80.0f, 0.001f);
    ok &= expectNear("weak pivot twist scale propagates", positionOnlyOutput.weakPivotTwistScale, 0.35f, 0.001f);

    MotorInput weakAngularFixedTau = positionOnlyPivot;
    weakAngularFixedTau.deltaTime = 1.0f;
    weakAngularFixedTau.tauLerpSpeed = 1.0f;
    const auto weakAngularFixedTauOutput = solveMotorTargets(weakAngularFixedTau);
    ok &= expectNear("weak support still leaves angular tau fixed", weakAngularFixedTauOutput.angularTau, 0.03f, 0.001f);

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
    ok &= expectNear("long handle does not fake twist damping", longHandleAuthority.twistScale, 1.0f, 0.001f);
    ok &= expectFalse("long handle alone is not axis-limited", longHandleAuthority.axisLimited);

    const auto unsupportedLongHandleAuthority = computeAngularAuthorityScale(AngularAuthorityInput{
        .enabled = true,
        .positionOnlyPivot = false,
        .normalTrusted = true,
        .contactPatchUsedAsPivot = false,
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
            .contactPatchUsedAsPivot = true,
            .contactPatchSampleCount = 1,
            .longObjectLeverGameUnits = 8.0f,
        },
        .heldBodyColliding = false,
    });
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
    ok &= expectNear("contact held authority caps release angular scale", contactHeldAuthority.releaseAngularVelocityScale, 0.75f, 0.001f);

    const float authorityCap = computeAuthorityScaledAngularVelocityCap(18.0f, 0.30f, 0.50f);
    ok &= expectNear("release angular cap composes support and long-object scale", authorityCap, 2.70f, 0.001f);

    const auto supportRefreshUpgrade = evaluateHeldSupportRefresh(HeldSupportRefreshInput{
        .enabled = true,
        .hasLiveCandidate = true,
        .liveCandidateNormalTrusted = true,
        .currentPositionOnly = true,
        .currentNormalTrusted = false,
        .currentContactPatchUsedAsPivot = false,
        .currentContactPatchSampleCount = 0,
        .liveCandidateLocalDeltaGameUnits = 1.0f,
        .maxLiveCandidateLocalDeltaGameUnits = 4.0f,
        .liveCandidateLongObjectLeverGameUnits = 20.0f,
    });
    ok &= expectTrue("held support refresh upgrades trusted live normal", supportRefreshUpgrade.refresh);
    ok &= expectTrue("held support refresh never moves frozen pivot", supportRefreshUpgrade.keepFrozenPivot);
    ok &= expectTrue("held support refresh marks normal trusted", supportRefreshUpgrade.pivotAuthorityNormalTrusted);
    ok &= expectFalse("held support refresh clears position-only after normal upgrade", supportRefreshUpgrade.pivotAuthorityPositionOnly);
    ok &= expectTrue("held support refresh contributes conservative point support", supportRefreshUpgrade.contactPatchUsedAsPivot);
    ok &= expectTrue("held support refresh seeds one support sample", supportRefreshUpgrade.contactPatchSampleCount == 1);
    ok &= expectTrue("held support refresh applies trusted live normal", supportRefreshUpgrade.useLiveCandidateNormal);

    const auto staleEvidenceRefresh = evaluateHeldSupportRefresh(HeldSupportRefreshInput{
        .enabled = true,
        .hasLiveCandidate = true,
        .liveCandidateNormalTrusted = true,
        .currentPositionOnly = true,
        .currentNormalTrusted = false,
        .currentContactPatchUsedAsPivot = false,
        .currentContactPatchSampleCount = 5,
        .liveCandidateLocalDeltaGameUnits = 1.0f,
        .maxLiveCandidateLocalDeltaGameUnits = 4.0f,
        .liveCandidateLongObjectLeverGameUnits = 20.0f,
    });
    ok &= expectTrue("held support refresh upgrades stale evidence-only samples", staleEvidenceRefresh.refresh);
    ok &= expectTrue("held support refresh does not promote stale evidence-only sample count", staleEvidenceRefresh.contactPatchSampleCount == 1);
    ok &= expectTrue("held support refresh reseeds evidence-only support from the live candidate", staleEvidenceRefresh.useLiveCandidateContactSample);

    const auto supportRefreshFarCandidate = evaluateHeldSupportRefresh(HeldSupportRefreshInput{
        .enabled = true,
        .hasLiveCandidate = true,
        .liveCandidateNormalTrusted = true,
        .currentPositionOnly = true,
        .currentNormalTrusted = false,
        .liveCandidateLocalDeltaGameUnits = 9.0f,
        .maxLiveCandidateLocalDeltaGameUnits = 4.0f,
    });
    ok &= expectFalse("held support refresh rejects candidates that would move pivot", supportRefreshFarCandidate.refresh);

    const auto weakNormalDowngrade = evaluateHeldSupportRefresh(HeldSupportRefreshInput{
        .enabled = true,
        .hasLiveCandidate = true,
        .liveCandidateNormalTrusted = false,
        .currentPositionOnly = false,
        .currentNormalTrusted = true,
        .currentHasSeatedPivotReacquire = false,
        .currentContactPatchUsedAsPivot = true,
        .currentContactPatchSampleCount = 1,
        .currentMultiFingerContactGroupCount = 0,
        .liveCandidateLocalDeltaGameUnits = 1.0f,
        .maxLiveCandidateLocalDeltaGameUnits = 4.0f,
    });
    ok &= expectTrue("held support refresh can downgrade stale single-point normal", weakNormalDowngrade.refresh);
    ok &= expectFalse("held support refresh weak downgrade clears normal trust", weakNormalDowngrade.pivotAuthorityNormalTrusted);
    ok &= expectTrue("held support refresh weak downgrade becomes position-only", weakNormalDowngrade.pivotAuthorityPositionOnly);

    const auto seatedNormalPreserved = evaluateHeldSupportRefresh(HeldSupportRefreshInput{
        .enabled = true,
        .hasLiveCandidate = true,
        .liveCandidateNormalTrusted = false,
        .currentPositionOnly = false,
        .currentNormalTrusted = true,
        .currentHasSeatedPivotReacquire = true,
        .currentContactPatchUsedAsPivot = true,
        .currentContactPatchSampleCount = 1,
        .liveCandidateLocalDeltaGameUnits = 1.0f,
        .maxLiveCandidateLocalDeltaGameUnits = 4.0f,
    });
    ok &= expectFalse("held support refresh does not downgrade explicit seated normal from one weak sample", seatedNormalPreserved.refresh);

    const auto leverRefresh = evaluateHeldSupportRefresh(HeldSupportRefreshInput{
        .enabled = true,
        .hasLiveCandidate = true,
        .liveCandidateNormalTrusted = true,
        .currentPositionOnly = false,
        .currentNormalTrusted = true,
        .currentContactPatchUsedAsPivot = true,
        .currentContactPatchSampleCount = 2,
        .currentLongObjectLeverGameUnits = 24.0f,
        .liveCandidateLocalDeltaGameUnits = 1.0f,
        .maxLiveCandidateLocalDeltaGameUnits = 4.0f,
        .liveCandidateLongObjectLeverGameUnits = 72.0f,
    });
    ok &= expectTrue("held support refresh updates materially changed long lever", leverRefresh.refresh);
    ok &= expectNear("held support refresh publishes live long lever", leverRefresh.longObjectLeverGameUnits, 72.0f, 0.001f);

    const auto leverOnlyRefreshKeepsTrustedNormal = evaluateHeldSupportRefresh(HeldSupportRefreshInput{
        .enabled = true,
        .hasLiveCandidate = true,
        .liveCandidateNormalTrusted = false,
        .currentPositionOnly = false,
        .currentNormalTrusted = true,
        .currentContactPatchUsedAsPivot = true,
        .currentContactPatchSampleCount = 2,
        .currentLongObjectLeverGameUnits = 24.0f,
        .liveCandidateLocalDeltaGameUnits = 1.0f,
        .maxLiveCandidateLocalDeltaGameUnits = 4.0f,
        .liveCandidateLongObjectLeverGameUnits = 72.0f,
    });
    ok &= expectTrue("held support refresh allows untrusted lever-only evidence", leverOnlyRefreshKeepsTrustedNormal.refresh);
    ok &= expectTrue("held support refresh preserves existing trusted normal on lever-only update", leverOnlyRefreshKeepsTrustedNormal.pivotAuthorityNormalTrusted);
    ok &= expectFalse("held support refresh does not replace trusted normal with untrusted lever-only evidence", leverOnlyRefreshKeepsTrustedNormal.useLiveCandidateNormal);
    ok &= expectFalse("held support refresh does not rewrite support sample for untrusted lever-only evidence", leverOnlyRefreshKeepsTrustedNormal.useLiveCandidateContactSample);
    ok &= expectTrue("held support refresh preserves pivot-owned support sample count on lever-only update",
        leverOnlyRefreshKeepsTrustedNormal.contactPatchSampleCount == 2);

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
    ok &= expectTrue("seated palm pocket promotion enriches support", seatedImmediatePromotion.enrichSupport);
    ok &= expectNear("seated palm pocket immediate blend", seatedImmediatePromotion.pivotBlend, 1.0f, 0.001f);

    const auto seatedLerpPromotion = evaluateSeatedPalmPocketPromotion(SeatedPalmPocketPromotionInput{
        .weakMeshStart = true,
        .hasSeatedCandidate = true,
        .reachedTouchRange = true,
        .supportPatchValid = true,
        .currentContactPatchSampleCount = 1,
        .supportPatchSampleCount = 4,
        .candidateLocalDeltaGameUnits = 8.0f,
        .immediateMaxLocalDeltaGameUnits = 4.0f,
        .lerpMaxLocalDeltaGameUnits = 12.0f,
    });
    ok &= expectTrue("seated palm pocket promotion advances medium delta", seatedLerpPromotion.promotePivot);
    ok &= expectFalse("seated palm pocket medium delta does not complete relation", seatedLerpPromotion.completeSeatedRelation);
    ok &= expectNear("seated palm pocket medium blend", seatedLerpPromotion.pivotBlend, 0.5f, 0.001f);

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
    ok &= expectTrue("seated palm pocket contact softening can still enrich support", seatedContactSupportOnly.enrichSupport);
    ok &= expectReason("seated palm pocket contact softening reason",
        seatedContactSupportOnly.reason,
        "seatedPalmPocketPromotionContactSofteningSupportOnly");

    const auto seatedLargeDeltaSupportOnly = evaluateSeatedPalmPocketPromotion(SeatedPalmPocketPromotionInput{
        .weakMeshStart = true,
        .hasSeatedCandidate = true,
        .reachedTouchRange = true,
        .supportPatchValid = true,
        .currentContactPatchSampleCount = 1,
        .supportPatchSampleCount = 5,
        .candidateLocalDeltaGameUnits = 20.0f,
        .immediateMaxLocalDeltaGameUnits = 4.0f,
        .lerpMaxLocalDeltaGameUnits = 12.0f,
    });
    ok &= expectFalse("seated palm pocket large delta keeps current pivot", seatedLargeDeltaSupportOnly.promotePivot);
    ok &= expectTrue("seated palm pocket large delta preserves support evidence", seatedLargeDeltaSupportOnly.enrichSupport);

    const auto seatedNotWeak = evaluateSeatedPalmPocketPromotion(SeatedPalmPocketPromotionInput{
        .weakMeshStart = false,
        .hasSeatedCandidate = true,
        .reachedTouchRange = true,
        .candidateLocalDeltaGameUnits = 2.0f,
    });
    ok &= expectFalse("seated palm pocket promotion ignores non-weak authority", seatedNotWeak.promotePivot);
    ok &= expectReason("seated palm pocket non-weak reason", seatedNotWeak.reason, "seatedPalmPocketPromotionNotWeakMesh");

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
