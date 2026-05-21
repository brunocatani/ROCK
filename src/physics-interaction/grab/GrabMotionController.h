#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace rock::grab_motion_controller
{
    enum class ContactSupportShape : std::uint8_t
    {
        Unknown,
        Point,
        Line,
        Surface,
        Wrap,
        SphereLike,
        ThinFace,
        ThinEdge,
        LongHandle
    };

    inline const char* contactSupportShapeName(ContactSupportShape shape)
    {
        switch (shape) {
        case ContactSupportShape::Unknown:
            return "unknown";
        case ContactSupportShape::Point:
            return "point";
        case ContactSupportShape::Line:
            return "line";
        case ContactSupportShape::Surface:
            return "surface";
        case ContactSupportShape::Wrap:
            return "wrap";
        case ContactSupportShape::SphereLike:
            return "sphereLike";
        case ContactSupportShape::ThinFace:
            return "thinFace";
        case ContactSupportShape::ThinEdge:
            return "thinEdge";
        case ContactSupportShape::LongHandle:
            return "longHandle";
        }
        return "unknown";
    }

    /*
     * ROCK's grabbed-object feel comes from a motor controller, not from a
     * single static force value. The verified FO4VR hknp constraint remains the
     * owner, but motor targets are computed in one pure helper so stronger follow
     * behavior, collision softening, mass caps, and startup fade all use the same
     * rules before any low-level Havok fields are written.
     */
    struct MotorInput
    {
        bool enabled = true;
        bool heldBodyColliding = false;

        float positionErrorGameUnits = 0.0f;
        float rotationErrorDegrees = 0.0f;
        float fullPositionErrorGameUnits = 20.0f;
        float fullRotationErrorDegrees = 60.0f;

        float baseLinearTau = 0.03f;
        float baseAngularTau = 0.03f;
        float collisionTau = 0.01f;
        float maxTau = 0.8f;
        float maxAngularTau = 0.35f;
        float currentLinearTau = 0.03f;
        float currentAngularTau = 0.03f;
        float tauLerpSpeed = 0.5f;
        float deltaTime = 1.0f / 90.0f;

        float baseMaxForce = 2000.0f;
        float massResponsiveMaxForce = 2000.0f;
        float maxForceMultiplier = 4.0f;
        float authorityForceScale = 1.0f;
        float mass = 0.0f;
        float forceToMassRatio = 500.0f;
        float angularToLinearForceRatio = 12.5f;

        bool fadeInEnabled = true;
        float fadeElapsed = 1.0f;
        float fadeDuration = 0.1f;
        float fadeStartAngularRatio = 100.0f;

        bool pivotQualityAngularScalingEnabled = false;
        bool pivotAuthorityPositionOnly = false;
        bool pivotAuthorityNormalTrusted = true;
        bool contactPatchUsedAsPivot = false;
        std::uint32_t contactPatchSampleCount = 0;
        std::uint32_t multiFingerContactGroupCount = 0;
        float multiFingerContactSpreadGameUnits = 0.0f;
        float longObjectLeverGameUnits = 0.0f;
        float smallObjectReferenceLeverGameUnits = 12.0f;
        float positionOnlyAngularScale = 0.55f;
        float smallObjectAngularScale = 0.65f;
        float lowContactSupportAngularScale = 0.75f;
        float minAngularAuthorityScale = 0.30f;
        float weakNormalAngularDampingMultiplier = 1.75f;
        float weakPivotTwistScale = 0.35f;
        ContactSupportShape contactSupportShape = ContactSupportShape::Unknown;
        float longObjectReferenceLeverGameUnits = 24.0f;
    };

    struct MotorOutput
    {
        float errorFactor = 0.0f;
        float linearErrorFactor = 0.0f;
        float angularErrorFactor = 0.0f;
        float linearTau = 0.03f;
        float angularTau = 0.03f;
        float linearMaxForce = 0.0f;
        float angularMaxForce = 0.0f;
        float angularAuthorityScale = 1.0f;
        float angularTauAuthorityScale = 1.0f;
        float angularDampingMultiplier = 1.0f;
        float weakPivotTwistScale = 1.0f;
        float fadeFactor = 1.0f;
    };

    struct AngularAuthorityInput
    {
        bool enabled = true;
        bool positionOnlyPivot = false;
        bool normalTrusted = true;
        bool contactPatchUsedAsPivot = false;
        std::uint32_t contactPatchSampleCount = 0;
        std::uint32_t multiFingerContactGroupCount = 0;
        float multiFingerContactSpreadGameUnits = 0.0f;
        float longObjectLeverGameUnits = 0.0f;
        float smallObjectReferenceLeverGameUnits = 12.0f;
        float positionOnlyAngularScale = 0.55f;
        float smallObjectAngularScale = 0.65f;
        float lowContactSupportAngularScale = 0.75f;
        float minAngularAuthorityScale = 0.30f;
        float weakNormalAngularDampingMultiplier = 1.75f;
        float weakPivotTwistScale = 0.35f;
        ContactSupportShape contactSupportShape = ContactSupportShape::Unknown;
        float longObjectReferenceLeverGameUnits = 24.0f;
    };

    struct AngularAuthorityOutput
    {
        float authorityScale = 1.0f;
        float dampingMultiplier = 1.0f;
        float weakPivotTwistScale = 1.0f;
        float pivotQualityScale = 1.0f;
        float contactSupportScale = 1.0f;
        float smallObjectScale = 1.0f;
        float swingScale = 1.0f;
        float twistScale = 1.0f;
        float contactNormalScale = 1.0f;
        float angularTauScale = 1.0f;
        ContactSupportShape contactSupportShape = ContactSupportShape::Unknown;
        bool weakPivot = false;
        bool lowContactSupport = false;
        bool smallObject = false;
        bool axisLimited = false;
    };

    inline float finiteOr(float value, float fallback)
    {
        return std::isfinite(value) ? value : fallback;
    }

    inline float clamp01(float value)
    {
        if (!std::isfinite(value)) {
            return 0.0f;
        }
        return std::clamp(value, 0.0f, 1.0f);
    }

    inline float safePositive(float value, float fallback)
    {
        return (std::isfinite(value) && value > 0.0f) ? value : fallback;
    }

    inline float advanceToward(float current, float target, float speed, float deltaTime)
    {
        current = finiteOr(current, target);
        target = finiteOr(target, current);
        if (!std::isfinite(speed) || speed <= 0.0f) {
            return target;
        }

        const float dt = safePositive(deltaTime, 1.0f / 90.0f);
        const float step = speed * dt;
        const float delta = target - current;
        if (std::abs(delta) <= step) {
            return target;
        }
        return current + (delta > 0.0f ? step : -step);
    }

    inline float computeErrorFactor(float positionErrorGameUnits, float rotationErrorDegrees, float fullPositionErrorGameUnits, float fullRotationErrorDegrees)
    {
        const float positionFull = safePositive(fullPositionErrorGameUnits, 20.0f);
        const float rotationFull = safePositive(fullRotationErrorDegrees, 60.0f);
        const float positionFactor = clamp01(std::abs(finiteOr(positionErrorGameUnits, 0.0f)) / positionFull);
        const float rotationFactor = clamp01(std::abs(finiteOr(rotationErrorDegrees, 0.0f)) / rotationFull);
        return (std::max)(positionFactor, rotationFactor);
    }

    inline float computePositionErrorFactor(float positionErrorGameUnits, float fullPositionErrorGameUnits)
    {
        const float positionFull = safePositive(fullPositionErrorGameUnits, 20.0f);
        return clamp01(std::abs(finiteOr(positionErrorGameUnits, 0.0f)) / positionFull);
    }

    inline float computeRotationErrorFactor(float rotationErrorDegrees, float fullRotationErrorDegrees)
    {
        const float rotationFull = safePositive(fullRotationErrorDegrees, 60.0f);
        return clamp01(std::abs(finiteOr(rotationErrorDegrees, 0.0f)) / rotationFull);
    }

    inline float computeFadeFactor(float elapsed, float duration)
    {
        if (!std::isfinite(duration) || duration <= 0.001f) {
            return 1.0f;
        }
        return clamp01(finiteOr(elapsed, 0.0f) / duration);
    }

    inline float capForceByMass(float force, float mass, float forceToMassRatio)
    {
        if (!std::isfinite(force) || force <= 0.0f) {
            return 0.0f;
        }
        if (!std::isfinite(mass) || mass <= 0.0f || !std::isfinite(forceToMassRatio) || forceToMassRatio <= 0.0f) {
            return force;
        }
        return (std::min)(force, mass * forceToMassRatio);
    }

    inline float angularForceFromRatio(float linearForce, float ratio)
    {
        if (!std::isfinite(linearForce) || linearForce <= 0.0f) {
            return 0.0f;
        }
        if (!std::isfinite(ratio) || ratio <= 0.001f) {
            return linearForce;
        }
        return linearForce / ratio;
    }

    inline float computeLongObjectAngularSpeedScale(bool enabled, float leverGameUnits, float referenceLeverGameUnits, float minScale)
    {
        /*
         * Long-object handling should reduce angular authority, not move the
         * grip. A long rifle held near one end can keep the same contact pivot
         * and hand/object relation while its far end receives less sweep speed.
         * The scale is therefore a cap multiplier only.
         */
        if (!enabled) {
            return 1.0f;
        }

        const float lever = finiteOr(leverGameUnits, 0.0f);
        const float reference = safePositive(referenceLeverGameUnits, 24.0f);
        const float floor = std::clamp(safePositive(minScale, 0.35f), 0.05f, 1.0f);
        if (lever <= reference) {
            return 1.0f;
        }
        return std::clamp(reference / lever, floor, 1.0f);
    }

    inline ContactSupportShape classifyContactSupportShape(ContactSupportShape explicitShape,
        bool normalTrusted,
        bool contactPatchUsedAsPivot,
        std::uint32_t contactPatchSampleCount,
        std::uint32_t multiFingerContactGroupCount,
        float multiFingerContactSpreadGameUnits,
        float longObjectLeverGameUnits,
        float smallObjectReferenceLeverGameUnits,
        float longObjectReferenceLeverGameUnits)
    {
        if (explicitShape != ContactSupportShape::Unknown) {
            return explicitShape;
        }

        const float lever = finiteOr(longObjectLeverGameUnits, 0.0f);
        const float smallReference = safePositive(smallObjectReferenceLeverGameUnits, 12.0f);
        const float longReference = safePositive(longObjectReferenceLeverGameUnits, 24.0f);
        const float spread = finiteOr(multiFingerContactSpreadGameUnits, 0.0f);
        const bool smallObject = lever > 0.0f && lever <= smallReference;
        const bool longObject = lever >= longReference * 1.35f;
        const bool patchPivot = contactPatchUsedAsPivot && contactPatchSampleCount > 0;
        const bool multiFingerWrap = multiFingerContactGroupCount >= 3 && spread >= 4.0f;

        if (longObject && (!patchPivot || contactPatchSampleCount <= 2 || multiFingerContactGroupCount < 2)) {
            return ContactSupportShape::LongHandle;
        }
        if (smallObject && multiFingerWrap) {
            return ContactSupportShape::SphereLike;
        }
        if (multiFingerWrap) {
            return ContactSupportShape::Wrap;
        }
        if (!patchPivot) {
            return multiFingerContactGroupCount >= 2 ? ContactSupportShape::Wrap : ContactSupportShape::Unknown;
        }
        if (contactPatchSampleCount <= 1) {
            return smallObject ? ContactSupportShape::SphereLike : ContactSupportShape::Point;
        }
        if (contactPatchSampleCount == 2) {
            return longObject ? ContactSupportShape::LongHandle : (smallObject ? ContactSupportShape::ThinEdge : ContactSupportShape::Line);
        }
        if (smallObject) {
            return ContactSupportShape::SphereLike;
        }
        if (!normalTrusted) {
            return ContactSupportShape::ThinFace;
        }
        return ContactSupportShape::Surface;
    }

    inline AngularAuthorityOutput computeAngularAuthorityScale(const AngularAuthorityInput& input)
    {
        AngularAuthorityOutput output{};
        if (!input.enabled) {
            return output;
        }

        const float floor = std::clamp(safePositive(input.minAngularAuthorityScale, 0.30f), 0.05f, 1.0f);
        const float positionOnlyScale = std::clamp(safePositive(input.positionOnlyAngularScale, 0.55f), floor, 1.0f);
        const float lowContactScale = std::clamp(safePositive(input.lowContactSupportAngularScale, 0.75f), floor, 1.0f);
        const float smallObjectScale = std::clamp(safePositive(input.smallObjectAngularScale, 0.65f), floor, 1.0f);
        const float configuredWeakPivotTwistScale =
            std::clamp(std::isfinite(input.weakPivotTwistScale) ? input.weakPivotTwistScale : 0.35f, 0.0f, 1.0f);

        output.weakPivot = input.positionOnlyPivot || !input.normalTrusted;
        if (output.weakPivot) {
            output.pivotQualityScale = positionOnlyScale;
            output.dampingMultiplier = (std::max)(1.0f, safePositive(input.weakNormalAngularDampingMultiplier, 1.75f));
            output.weakPivotTwistScale = configuredWeakPivotTwistScale;
        }

        const bool patchSupportsPivot = input.contactPatchUsedAsPivot && input.contactPatchSampleCount > 0;
        const bool hasContactSupport = patchSupportsPivot || input.multiFingerContactGroupCount > 0;
        const bool strongContactSupport =
            input.multiFingerContactGroupCount >= 2 ||
            (input.contactPatchUsedAsPivot && input.contactPatchSampleCount >= 3) ||
            (std::isfinite(input.multiFingerContactSpreadGameUnits) && input.multiFingerContactSpreadGameUnits >= 4.0f);
        output.lowContactSupport = hasContactSupport && !strongContactSupport;
        if (output.lowContactSupport) {
            output.contactSupportScale = lowContactScale;
        }

        const float lever = finiteOr(input.longObjectLeverGameUnits, 0.0f);
        const float smallObjectReference = safePositive(input.smallObjectReferenceLeverGameUnits, 12.0f);
        output.smallObject = lever > 0.0f && lever <= smallObjectReference;
        if (output.smallObject && (output.weakPivot || output.lowContactSupport)) {
            output.smallObjectScale = smallObjectScale;
        }

        output.contactSupportShape = classifyContactSupportShape(
            input.contactSupportShape,
            input.normalTrusted,
            input.contactPatchUsedAsPivot,
            input.contactPatchSampleCount,
            input.multiFingerContactGroupCount,
            input.multiFingerContactSpreadGameUnits,
            input.longObjectLeverGameUnits,
            input.smallObjectReferenceLeverGameUnits,
            input.longObjectReferenceLeverGameUnits);

        switch (output.contactSupportShape) {
        case ContactSupportShape::Point:
            output.lowContactSupport = true;
            output.contactSupportScale = (std::min)(output.contactSupportScale, lowContactScale);
            output.twistScale = (std::min)(output.twistScale, configuredWeakPivotTwistScale);
            output.contactNormalScale = (std::min)(output.contactNormalScale, positionOnlyScale);
            output.axisLimited = true;
            break;
        case ContactSupportShape::Line:
            output.contactNormalScale = (std::min)(output.contactNormalScale, lowContactScale);
            output.twistScale = (std::min)(output.twistScale, (std::max)(floor, 0.65f));
            output.axisLimited = true;
            break;
        case ContactSupportShape::ThinEdge:
            output.contactSupportScale = (std::min)(output.contactSupportScale, lowContactScale);
            output.twistScale = (std::min)(output.twistScale, (std::max)(floor, 0.50f));
            output.contactNormalScale = (std::min)(output.contactNormalScale, (std::max)(floor, 0.55f));
            output.axisLimited = true;
            break;
        case ContactSupportShape::ThinFace:
            output.contactNormalScale = (std::min)(output.contactNormalScale, (std::max)(floor, 0.60f));
            output.axisLimited = true;
            break;
        case ContactSupportShape::SphereLike:
            output.contactSupportScale = (std::min)(output.contactSupportScale, output.smallObject ? smallObjectScale : lowContactScale);
            output.twistScale = (std::min)(output.twistScale, (std::max)(floor, 0.45f));
            output.contactNormalScale = (std::min)(output.contactNormalScale, floor);
            output.axisLimited = true;
            break;
        case ContactSupportShape::LongHandle:
            output.contactSupportScale = (std::min)(output.contactSupportScale, lowContactScale);
            output.twistScale = (std::min)(output.twistScale, (std::max)(floor, 0.55f));
            output.swingScale = (std::min)(output.swingScale, (std::max)(floor, 0.85f));
            output.axisLimited = true;
            break;
        case ContactSupportShape::Wrap:
        case ContactSupportShape::Surface:
        case ContactSupportShape::Unknown:
            break;
        }

        output.authorityScale = std::clamp(
            output.pivotQualityScale * output.contactSupportScale * output.smallObjectScale,
            floor,
            1.0f);
        output.angularTauScale = output.authorityScale;
        if (output.weakPivot || output.axisLimited || output.weakPivotTwistScale < 0.999f) {
            output.angularTauScale = (std::min)(output.angularTauScale, output.swingScale);
            output.angularTauScale = (std::min)(output.angularTauScale, output.twistScale);
            output.angularTauScale = (std::min)(output.angularTauScale, output.contactNormalScale);
            output.angularTauScale = (std::min)(output.angularTauScale, output.weakPivotTwistScale);
        }
        output.angularTauScale = std::clamp(output.angularTauScale, floor, 1.0f);
        return output;
    }

    struct HeldAuthorityInput
    {
        AngularAuthorityInput angular{};
        bool heldBodyColliding = false;
        float positionErrorGameUnits = 0.0f;
        float rotationErrorDegrees = 0.0f;
        float fullPositionErrorGameUnits = 20.0f;
        float fullRotationErrorDegrees = 60.0f;
    };

    struct HeldAuthorityState
    {
        float positionErrorFactor = 0.0f;
        float rotationErrorFactor = 0.0f;
        bool softenForContact = false;
        AngularAuthorityOutput angular{};
        float angularVelocityAssistScale = 1.0f;
        float releaseAngularVelocityScale = 1.0f;
        const char* reason = "full-authority";
    };

    struct HeldSupportRefreshInput
    {
        bool enabled = true;
        bool hasLiveCandidate = false;
        bool liveCandidateNormalTrusted = false;
        bool currentPositionOnly = false;
        bool currentNormalTrusted = false;
        bool currentHasSeatedPivotReacquire = false;
        bool currentContactPatchUsedAsPivot = false;
        std::uint32_t currentContactPatchSampleCount = 0;
        std::uint32_t currentMultiFingerContactGroupCount = 0;
        float currentLongObjectLeverGameUnits = 0.0f;
        float liveCandidateLocalDeltaGameUnits = 0.0f;
        float maxLiveCandidateLocalDeltaGameUnits = 4.0f;
        float liveCandidateLongObjectLeverGameUnits = 0.0f;
    };

    struct HeldSupportRefreshDecision
    {
        bool refresh = false;
        bool keepFrozenPivot = true;
        bool useLiveCandidateNormal = false;
        bool useLiveCandidateContactSample = false;
        bool pivotAuthorityPositionOnly = false;
        bool pivotAuthorityNormalTrusted = false;
        bool contactPatchUsedAsPivot = false;
        std::uint32_t contactPatchSampleCount = 0;
        float longObjectLeverGameUnits = 0.0f;
        const char* reason = "notEvaluated";
    };

    inline HeldSupportRefreshDecision evaluateHeldSupportRefresh(const HeldSupportRefreshInput& input)
    {
        HeldSupportRefreshDecision decision{};
        decision.pivotAuthorityPositionOnly = input.currentPositionOnly;
        decision.pivotAuthorityNormalTrusted = input.currentNormalTrusted;
        decision.contactPatchUsedAsPivot = input.currentContactPatchUsedAsPivot;
        decision.contactPatchSampleCount = input.currentContactPatchSampleCount;
        decision.longObjectLeverGameUnits = finiteOr(input.currentLongObjectLeverGameUnits, 0.0f);

        if (!input.enabled) {
            decision.reason = "heldSupportRefreshDisabled";
            return decision;
        }
        if (!input.hasLiveCandidate) {
            decision.reason = "heldSupportRefreshMissingLiveCandidate";
            return decision;
        }

        const float liveDelta = finiteOr(input.liveCandidateLocalDeltaGameUnits, std::numeric_limits<float>::infinity());
        const float maxLiveDelta = safePositive(input.maxLiveCandidateLocalDeltaGameUnits, 4.0f);
        if (!std::isfinite(liveDelta) || liveDelta > maxLiveDelta) {
            decision.reason = "heldSupportRefreshCandidateMovedPivot";
            return decision;
        }

        const bool currentHasPatchSupport = input.currentContactPatchUsedAsPivot && input.currentContactPatchSampleCount > 0;
        const bool currentLowSupport = input.currentContactPatchSampleCount <= 1 && input.currentMultiFingerContactGroupCount < 2;
        const bool candidateHasLever =
            std::isfinite(input.liveCandidateLongObjectLeverGameUnits) && input.liveCandidateLongObjectLeverGameUnits > 0.0f;
        const float currentLever = finiteOr(input.currentLongObjectLeverGameUnits, 0.0f);
        const float liveLever = candidateHasLever ? input.liveCandidateLongObjectLeverGameUnits : currentLever;
        const float leverDelta = std::fabs(liveLever - currentLever);
        const bool leverChanged = candidateHasLever && leverDelta > (std::max)(1.0f, currentLever * 0.10f);
        const bool normalUpgrade = input.liveCandidateNormalTrusted && (!input.currentNormalTrusted || input.currentPositionOnly);
        const bool weakNormalDowngrade =
            !input.liveCandidateNormalTrusted &&
            input.currentNormalTrusted &&
            currentLowSupport &&
            !input.currentHasSeatedPivotReacquire;
        const bool addSupport = !currentHasPatchSupport;

        if (!normalUpgrade && !weakNormalDowngrade && !addSupport && !leverChanged) {
            decision.reason = "heldSupportRefreshNoBetterEvidence";
            return decision;
        }

        decision.refresh = true;
        decision.contactPatchUsedAsPivot = true;
        decision.contactPatchSampleCount = (std::max<std::uint32_t>)(input.currentContactPatchSampleCount, 1u);
        decision.longObjectLeverGameUnits = liveLever;
        if (input.liveCandidateNormalTrusted) {
            decision.pivotAuthorityPositionOnly = false;
            decision.pivotAuthorityNormalTrusted = true;
            decision.useLiveCandidateNormal = true;
            decision.useLiveCandidateContactSample = true;
            decision.reason = normalUpgrade ? "heldSupportRefreshNormalUpgrade" : "heldSupportRefreshLiveSupport";
        } else if (weakNormalDowngrade || !input.currentNormalTrusted || input.currentPositionOnly) {
            decision.pivotAuthorityPositionOnly = true;
            decision.pivotAuthorityNormalTrusted = false;
            decision.useLiveCandidateNormal = true;
            decision.useLiveCandidateContactSample = true;
            decision.reason = weakNormalDowngrade ? "heldSupportRefreshWeakNormalDowngrade" : "heldSupportRefreshPositionOnly";
        } else {
            decision.pivotAuthorityPositionOnly = input.currentPositionOnly;
            decision.pivotAuthorityNormalTrusted = input.currentNormalTrusted;
            decision.useLiveCandidateNormal = false;
            decision.useLiveCandidateContactSample = addSupport;
            decision.reason = leverChanged ? "heldSupportRefreshLeverUpdate" : "heldSupportRefreshSupportOnly";
        }
        return decision;
    }

    struct VisualHandPublishInput
    {
        bool hasTelemetryCapture = false;
        bool touchHeldPhase = false;
        bool acquisitionVisualEligible = false;
        bool hasPivotTrackingError = false;
        bool motorContactSoftening = false;
        bool pivotAuthorityPositionOnly = false;
        bool pivotAuthorityNormalTrusted = false;
        bool hasSeatedPivotReacquire = false;
        std::uint32_t multiFingerContactGroupCount = 0;
        std::uint32_t contactPatchSampleCount = 0;
        float angularAuthorityScale = 1.0f;
        ContactSupportShape contactSupportShape = ContactSupportShape::Unknown;
    };

    struct VisualHandPublishDecision
    {
        bool apply = false;
        bool acquisition = false;
        const char* reason = "notEvaluated";
    };

    inline HeldAuthorityState evaluateHeldAuthority(const HeldAuthorityInput& input)
    {
        HeldAuthorityState state{};
        state.positionErrorFactor = computePositionErrorFactor(input.positionErrorGameUnits, input.fullPositionErrorGameUnits);
        state.rotationErrorFactor = computeRotationErrorFactor(input.rotationErrorDegrees, input.fullRotationErrorDegrees);
        state.softenForContact = input.heldBodyColliding;
        state.angular = computeAngularAuthorityScale(input.angular);
        state.angularVelocityAssistScale = state.angular.authorityScale;
        state.releaseAngularVelocityScale = state.angular.authorityScale;
        if (state.softenForContact) {
            state.angularVelocityAssistScale = (std::min)(state.angularVelocityAssistScale, 0.75f);
            state.releaseAngularVelocityScale = (std::min)(state.releaseAngularVelocityScale, 0.75f);
            state.reason = "contact-softened-authority";
        } else if (state.angular.weakPivot) {
            state.reason = "weak-pivot-authority";
        } else if (state.angular.lowContactSupport) {
            state.reason = "low-contact-authority";
        } else if (state.angular.smallObject) {
            state.reason = "small-object-authority";
        }
        return state;
    }

    inline float computeAuthorityScaledAngularVelocityCap(float configuredMaxSpeedRadiansPerSecond, float authorityScale, float longObjectAngularScale)
    {
        const float configuredMax = std::clamp(
            std::isfinite(configuredMaxSpeedRadiansPerSecond) ? configuredMaxSpeedRadiansPerSecond : 18.0f,
            0.25f,
            64.0f);
        const float authority = std::clamp(std::isfinite(authorityScale) && authorityScale > 0.0f ? authorityScale : 1.0f, 0.05f, 1.0f);
        const float longObject = std::clamp(std::isfinite(longObjectAngularScale) && longObjectAngularScale > 0.0f ? longObjectAngularScale : 1.0f, 0.05f, 1.0f);
        return (std::max)(0.25f, configuredMax * authority * longObject);
    }

    inline VisualHandPublishDecision evaluateVisualHandPublishGate(const VisualHandPublishInput& input)
    {
        VisualHandPublishDecision decision{};
        if (!input.hasTelemetryCapture) {
            decision.reason = "missingTelemetryCapture";
            return decision;
        }
        if (!input.hasPivotTrackingError) {
            decision.reason = "missingPivotTracking";
            return decision;
        }
        if (!input.touchHeldPhase && !input.acquisitionVisualEligible) {
            decision.reason = "phaseNotVisualEligible";
            return decision;
        }

        const float authorityScale = std::clamp(std::isfinite(input.angularAuthorityScale) ? input.angularAuthorityScale : 0.0f, 0.0f, 1.0f);
        const bool strongSupport =
            input.pivotAuthorityNormalTrusted ||
            input.hasSeatedPivotReacquire ||
            input.multiFingerContactGroupCount >= 2 ||
            input.contactSupportShape == ContactSupportShape::Surface ||
            input.contactSupportShape == ContactSupportShape::Wrap;
        const bool weakPointLikeSupport =
            input.contactSupportShape == ContactSupportShape::Point ||
            input.contactSupportShape == ContactSupportShape::SphereLike ||
            input.contactSupportShape == ContactSupportShape::ThinEdge;
        const bool weakNormalSupport = !input.pivotAuthorityNormalTrusted;
        const bool weakLeverSupport =
            input.contactSupportShape == ContactSupportShape::LongHandle &&
            input.multiFingerContactGroupCount < 2 &&
            !input.hasSeatedPivotReacquire;

        if (input.acquisitionVisualEligible && !input.touchHeldPhase) {
            decision.acquisition = true;
            if (input.motorContactSoftening) {
                decision.reason = "acquisitionPushingIntoContact";
                return decision;
            }
            if (authorityScale < 0.55f) {
                decision.reason = "acquisitionLowAngularAuthority";
                return decision;
            }
            if ((weakNormalSupport || weakPointLikeSupport || weakLeverSupport || input.pivotAuthorityPositionOnly) && !strongSupport) {
                decision.reason = "acquisitionAwaitingAuthorityEvidence";
                return decision;
            }
            decision.apply = true;
            decision.reason = "acquisitionAuthorityAccepted";
            return decision;
        }

        if ((weakNormalSupport || input.pivotAuthorityPositionOnly || weakPointLikeSupport || weakLeverSupport) && !strongSupport) {
            decision.reason = "touchHeldWeakVisualAuthority";
            return decision;
        }

        decision.apply = true;
        decision.reason = "touchHeldAuthorityAccepted";
        return decision;
    }

    template <class Vector>
    inline float vectorDot(const Vector& lhs, const Vector& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    template <class Vector>
    inline float vectorLengthSquared(const Vector& value)
    {
        return vectorDot(value, value);
    }

    template <class Vector>
    inline Vector scaleWeakPivotTwistAngularVelocity(const Vector& angularVelocity, const Vector& pivotToCenterOfMass, bool enabled, float twistScale)
    {
        if (!enabled) {
            return angularVelocity;
        }

        const float axisLengthSquared = vectorLengthSquared(pivotToCenterOfMass);
        if (!std::isfinite(axisLengthSquared) || axisLengthSquared <= 1.0e-6f) {
            return angularVelocity;
        }

        const float scale = std::clamp(std::isfinite(twistScale) ? twistScale : 1.0f, 0.0f, 1.0f);
        if (scale >= 0.999f) {
            return angularVelocity;
        }

        const float invAxisLength = 1.0f / std::sqrt(axisLengthSquared);
        const Vector axis{
            pivotToCenterOfMass.x * invAxisLength,
            pivotToCenterOfMass.y * invAxisLength,
            pivotToCenterOfMass.z * invAxisLength,
        };
        const float twistMagnitude = vectorDot(angularVelocity, axis);
        const Vector twist{
            axis.x * twistMagnitude,
            axis.y * twistMagnitude,
            axis.z * twistMagnitude,
        };
        const Vector swing{
            angularVelocity.x - twist.x,
            angularVelocity.y - twist.y,
            angularVelocity.z - twist.z,
        };
        return Vector{
            swing.x + twist.x * scale,
            swing.y + twist.y * scale,
            swing.z + twist.z * scale,
        };
    }

    template <class Vector>
    inline Vector scaleAngularVelocityComponentAroundAxis(const Vector& angularVelocity, const Vector& axisRaw, float scale)
    {
        const float axisLengthSquared = vectorLengthSquared(axisRaw);
        if (!std::isfinite(axisLengthSquared) || axisLengthSquared <= 1.0e-6f) {
            return angularVelocity;
        }

        const float sanitizedScale = std::clamp(std::isfinite(scale) ? scale : 1.0f, 0.0f, 1.0f);
        if (sanitizedScale >= 0.999f) {
            return angularVelocity;
        }

        const float invAxisLength = 1.0f / std::sqrt(axisLengthSquared);
        const Vector axis{
            axisRaw.x * invAxisLength,
            axisRaw.y * invAxisLength,
            axisRaw.z * invAxisLength,
        };
        const float componentMagnitude = vectorDot(angularVelocity, axis);
        const Vector component{
            axis.x * componentMagnitude,
            axis.y * componentMagnitude,
            axis.z * componentMagnitude,
        };
        const Vector remainder{
            angularVelocity.x - component.x,
            angularVelocity.y - component.y,
            angularVelocity.z - component.z,
        };
        return Vector{
            remainder.x + component.x * sanitizedScale,
            remainder.y + component.y * sanitizedScale,
            remainder.z + component.z * sanitizedScale,
        };
    }

    template <class Vector>
    inline Vector scaleAngularVelocityByHeldAuthorityAxes(
        const Vector& angularVelocity,
        const Vector& pivotToCenterOfMass,
        const Vector& contactNormal,
        const AngularAuthorityOutput& authority)
    {
        Vector result = angularVelocity;
        if (authority.swingScale < 0.999f) {
            result = Vector{ result.x * authority.swingScale, result.y * authority.swingScale, result.z * authority.swingScale };
        }
        result = scaleAngularVelocityComponentAroundAxis(result, contactNormal, authority.contactNormalScale);
        result = scaleAngularVelocityComponentAroundAxis(result, pivotToCenterOfMass, authority.twistScale);
        return result;
    }

    inline HeldAuthorityInput makeHeldAuthorityInput(const MotorInput& input)
    {
        return HeldAuthorityInput{
            .angular = AngularAuthorityInput{
                .enabled = input.pivotQualityAngularScalingEnabled,
                .positionOnlyPivot = input.pivotAuthorityPositionOnly,
                .normalTrusted = input.pivotAuthorityNormalTrusted,
                .contactPatchUsedAsPivot = input.contactPatchUsedAsPivot,
                .contactPatchSampleCount = input.contactPatchSampleCount,
                .multiFingerContactGroupCount = input.multiFingerContactGroupCount,
                .multiFingerContactSpreadGameUnits = input.multiFingerContactSpreadGameUnits,
                .longObjectLeverGameUnits = input.longObjectLeverGameUnits,
                .smallObjectReferenceLeverGameUnits = input.smallObjectReferenceLeverGameUnits,
                .positionOnlyAngularScale = input.positionOnlyAngularScale,
                .smallObjectAngularScale = input.smallObjectAngularScale,
                .lowContactSupportAngularScale = input.lowContactSupportAngularScale,
                .minAngularAuthorityScale = input.minAngularAuthorityScale,
                .weakNormalAngularDampingMultiplier = input.weakNormalAngularDampingMultiplier,
                .weakPivotTwistScale = input.weakPivotTwistScale,
                .contactSupportShape = input.contactSupportShape,
                .longObjectReferenceLeverGameUnits = input.longObjectReferenceLeverGameUnits,
            },
            .heldBodyColliding = input.heldBodyColliding,
            .positionErrorGameUnits = input.positionErrorGameUnits,
            .rotationErrorDegrees = input.rotationErrorDegrees,
            .fullPositionErrorGameUnits = input.fullPositionErrorGameUnits,
            .fullRotationErrorDegrees = input.fullRotationErrorDegrees,
        };
    }

    inline MotorOutput solveMotorTargetsWithAuthority(const MotorInput& input, const HeldAuthorityState& heldAuthority)
    {
        MotorOutput out{};

        const float baseLinearTau = safePositive(input.baseLinearTau, 0.03f);
        const float baseAngularTau = safePositive(input.baseAngularTau, baseLinearTau);
        const float maxTau = (std::max)(baseLinearTau, safePositive(input.maxTau, baseLinearTau));
        const float maxAngularTau = (std::max)(baseAngularTau, safePositive(input.maxAngularTau, baseAngularTau));
        const float collisionTau = safePositive(input.collisionTau, baseLinearTau);
        if (input.enabled) {
            out.linearErrorFactor = computePositionErrorFactor(input.positionErrorGameUnits, input.fullPositionErrorGameUnits);
            out.angularErrorFactor = computeRotationErrorFactor(input.rotationErrorDegrees, input.fullRotationErrorDegrees);
            out.errorFactor = (std::max)(out.linearErrorFactor, out.angularErrorFactor);
        }

        const auto& angularAuthority = heldAuthority.angular;
        out.angularAuthorityScale = angularAuthority.authorityScale;
        out.angularTauAuthorityScale = angularAuthority.angularTauScale;
        out.angularDampingMultiplier = angularAuthority.dampingMultiplier;
        out.weakPivotTwistScale = angularAuthority.weakPivotTwistScale;

        const float linearTauTarget = heldAuthority.softenForContact ? collisionTau : baseLinearTau + (maxTau - baseLinearTau) * out.linearErrorFactor;
        const float angularTauTarget = heldAuthority.softenForContact ?
            collisionTau :
            baseAngularTau + (maxAngularTau - baseAngularTau) * out.angularErrorFactor * out.angularTauAuthorityScale;
        out.linearTau = advanceToward(input.currentLinearTau, linearTauTarget, input.tauLerpSpeed, input.deltaTime);
        out.angularTau = advanceToward(input.currentAngularTau, angularTauTarget, input.tauLerpSpeed, input.deltaTime);

        const float maxMultiplier = (std::max)(1.0f, safePositive(input.maxForceMultiplier, 1.0f));
        const float baseForce = (std::max)(0.0f, finiteOr(input.baseMaxForce, 0.0f));
        float targetForce = baseForce * (1.0f + (maxMultiplier - 1.0f) * out.linearErrorFactor);
        const float massResponsiveCeiling = (std::max)(baseForce, finiteOr(input.massResponsiveMaxForce, baseForce));
        if (massResponsiveCeiling > baseForce && std::isfinite(input.mass) && input.mass > 0.0f && std::isfinite(input.forceToMassRatio) &&
            input.forceToMassRatio > 0.0f) {
            targetForce = (std::max)(targetForce, (std::min)(massResponsiveCeiling, input.mass * input.forceToMassRatio));
        }
        const float authorityForceScale = std::clamp(safePositive(input.authorityForceScale, 1.0f), 0.05f, 1.0f);
        out.fadeFactor = input.fadeInEnabled ? computeFadeFactor(input.fadeElapsed, input.fadeDuration) : 1.0f;
        out.linearMaxForce = capForceByMass(targetForce * out.fadeFactor, input.mass, input.forceToMassRatio) * authorityForceScale;

        const float angularFadeRatio =
            safePositive(input.fadeStartAngularRatio, input.angularToLinearForceRatio) +
            (safePositive(input.angularToLinearForceRatio, 12.5f) - safePositive(input.fadeStartAngularRatio, input.angularToLinearForceRatio)) * out.fadeFactor;
        out.angularMaxForce = angularForceFromRatio(out.linearMaxForce, angularFadeRatio) * out.angularAuthorityScale;
        return out;
    }

    inline MotorOutput solveMotorTargets(const MotorInput& input)
    {
        return solveMotorTargetsWithAuthority(input, evaluateHeldAuthority(makeHeldAuthorityInput(input)));
    }
}
