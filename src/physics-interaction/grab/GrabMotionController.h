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
        bool heldBodyColliding = false;

        float baseLinearTau = 0.03f;
        float baseAngularTau = 0.03f;
        float collisionTau = 0.01f;
        float currentLinearTau = 0.03f;
        float currentAngularTau = 0.03f;
        float tauLerpSpeed = 0.5f;
        float deltaTime = 1.0f / 90.0f;
        bool physicsRateForceScalingEnabled = false;
        float physicsDeltaSeconds = 1.0f / 90.0f;
        float physicsRateReferenceHz = 90.0f;
        float physicsRateForceScaleExponent = 0.5f;
        float physicsRateMinForceScale = 0.75f;
        float physicsRateMaxForceScale = 1.35f;

        float baseMaxForce = 2000.0f;
        float authorityForceScale = 1.0f;
        float angularForceMultiplier = 1.0f;
        float mass = 0.0f;
        float forceToMassRatio = 500.0f;
        bool effectiveMotorMassFloorEnabled = true;
        float effectiveMotorMassFloor = 2.0f;

        bool fadeInEnabled = true;
        float fadeElapsed = 1.0f;
        float fadeDuration = 0.1f;
    };

    struct MotorOutput
    {
        float linearTau = 0.03f;
        float angularTau = 0.03f;
        float linearMaxForce = 0.0f;
        float angularMaxForce = 0.0f;
        float fadeFactor = 1.0f;
        float physicsHz = 90.0f;
        float physicsRateForceScale = 1.0f;
    };

    struct AngularAuthorityInput
    {
        bool enabled = true;
        bool positionOnlyPivot = false;
        bool normalTrusted = true;
        bool contactPatchEvidence = false;
        std::uint32_t contactPatchSampleCount = 0;
        std::uint32_t multiFingerContactGroupCount = 0;
        float multiFingerContactSpreadGameUnits = 0.0f;
        float longObjectLeverGameUnits = 0.0f;
        float smallObjectReferenceLeverGameUnits = 12.0f;
        float positionOnlyAngularScale = 0.55f;
        float smallObjectAngularScale = 0.65f;
        float lowContactSupportAngularScale = 0.75f;
        float minAngularAuthorityScale = 0.30f;
        float weakPivotTwistScale = 0.35f;
        ContactSupportShape contactSupportShape = ContactSupportShape::Unknown;
        float longObjectReferenceLeverGameUnits = 24.0f;
    };

    struct AngularAuthorityOutput
    {
        float authorityScale = 1.0f;
        float weakPivotTwistScale = 1.0f;
        float pivotQualityScale = 1.0f;
        float contactSupportScale = 1.0f;
        float smallObjectScale = 1.0f;
        float swingScale = 1.0f;
        float twistScale = 1.0f;
        float contactNormalScale = 1.0f;
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

    inline float effectiveMotorMass(float mass, bool floorEnabled, float massFloor)
    {
        const float sanitizedMass = (std::isfinite(mass) && mass > 0.0f) ? mass : 0.0f;
        if (!floorEnabled) {
            return sanitizedMass;
        }

        const float sanitizedFloor = (std::isfinite(massFloor) && massFloor > 0.0f) ? massFloor : 0.0f;
        return (std::max)(sanitizedMass, sanitizedFloor);
    }

    inline float computePhysicsHz(float physicsDeltaSeconds, float fallbackHz = 90.0f)
    {
        const float sanitizedDelta = safePositive(physicsDeltaSeconds, 0.0f);
        if (sanitizedDelta <= 0.0f) {
            return safePositive(fallbackHz, 90.0f);
        }

        const float hz = 1.0f / sanitizedDelta;
        return std::isfinite(hz) && hz > 0.0f ? hz : safePositive(fallbackHz, 90.0f);
    }

    inline float computePhysicsRateForceScale(
        bool enabled,
        float physicsDeltaSeconds,
        float referenceHz,
        float exponent,
        float minScale,
        float maxScale)
    {
        if (!enabled) {
            return 1.0f;
        }

        const float sanitizedReferenceHz = safePositive(referenceHz, 90.0f);
        const float physicsHz = computePhysicsHz(physicsDeltaSeconds, sanitizedReferenceHz);
        const float sanitizedExponent = (std::isfinite(exponent) && exponent >= 0.0f) ? exponent : 0.5f;
        const float lowerScale = safePositive((std::min)(minScale, maxScale), 1.0f);
        const float upperScale = (std::max)(lowerScale, safePositive((std::max)(minScale, maxScale), 1.0f));
        if (physicsHz <= 0.0f || sanitizedReferenceHz <= 0.0f) {
            return 1.0f;
        }

        const float scale = std::pow(sanitizedReferenceHz / physicsHz, sanitizedExponent);
        if (!std::isfinite(scale)) {
            return 1.0f;
        }
        return std::clamp(scale, lowerScale, upperScale);
    }

    inline float computeLongObjectAngularSpeedScale(bool enabled, float leverGameUnits, float referenceLeverGameUnits, float minScale)
    {
        /*
         * Long-object release handling should reduce angular throw velocity,
         * not move the grip. Held rotation is owned by the constraint motor;
         * this scale only limits release angular velocity after the grab ends.
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
        bool contactPatchEvidence,
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
        const bool patchEvidence = contactPatchEvidence && contactPatchSampleCount > 0;
        const bool multiFingerWrap = multiFingerContactGroupCount >= 3 && spread >= 4.0f;

        if (longObject && (!patchEvidence || contactPatchSampleCount <= 2 || multiFingerContactGroupCount < 2)) {
            return ContactSupportShape::LongHandle;
        }
        if (smallObject && multiFingerWrap) {
            return ContactSupportShape::SphereLike;
        }
        if (multiFingerWrap) {
            return ContactSupportShape::Wrap;
        }
        if (!patchEvidence) {
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
            output.weakPivotTwistScale = configuredWeakPivotTwistScale;
        }

        const bool patchSupportsReleaseSafety = input.contactPatchEvidence && input.contactPatchSampleCount > 0;
        const bool hasContactSupport = patchSupportsReleaseSafety || input.multiFingerContactGroupCount > 0;
        const bool strongContactSupport =
            input.multiFingerContactGroupCount >= 2 ||
            (input.contactPatchEvidence && input.contactPatchSampleCount >= 3) ||
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
            input.contactPatchEvidence,
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
            // Length is telemetry here; actual low-support contact is handled before shape-specific rules.
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
        return output;
    }

    struct HeldAuthorityInput
    {
        AngularAuthorityInput angular{};
        bool heldBodyColliding = false;
    };

    struct HeldAuthorityState
    {
        bool softenForContact = false;
        AngularAuthorityOutput angular{};
        float releaseAngularVelocityScale = 1.0f;
        const char* reason = "full-authority";
    };

    struct SeatedPalmPocketPromotionInput
    {
        bool enabled = true;
        bool weakMeshStart = false;
        bool hasSeatedCandidate = false;
        bool reachedTouchRange = false;
        bool timedOutInsidePocket = false;
        bool motorContactSoftening = false;
        bool candidateNormalTrusted = false;
        bool supportPatchValid = false;
        bool supportPatchNormalTrusted = false;
        std::uint32_t currentContactPatchSampleCount = 0;
        std::uint32_t supportPatchSampleCount = 0;
        std::uint32_t currentMultiFingerContactGroupCount = 0;
        std::uint32_t liveMultiFingerContactGroupCount = 0;
        float candidateLocalDeltaGameUnits = std::numeric_limits<float>::infinity();
        float immediateMaxLocalDeltaGameUnits = 4.0f;
        float lerpMaxLocalDeltaGameUnits = 12.0f;
    };

    struct SeatedPalmPocketPromotionDecision
    {
        bool promotePivot = false;
        bool completeSeatedRelation = false;
        bool enrichSupport = false;
        float pivotBlend = 0.0f;
        const char* reason = "notEvaluated";
    };

    inline SeatedPalmPocketPromotionDecision evaluateSeatedPalmPocketPromotion(const SeatedPalmPocketPromotionInput& input)
    {
        SeatedPalmPocketPromotionDecision decision{};
        if (!input.enabled) {
            decision.reason = "seatedPalmPocketPromotionDisabled";
            return decision;
        }
        if (!input.weakMeshStart) {
            decision.reason = "seatedPalmPocketPromotionNotWeakMesh";
            return decision;
        }
        if (!input.hasSeatedCandidate) {
            decision.reason = "seatedPalmPocketPromotionMissingCandidate";
            return decision;
        }
        if (!input.reachedTouchRange && !input.timedOutInsidePocket) {
            decision.reason = "seatedPalmPocketPromotionAwaitingPocket";
            return decision;
        }

        if (input.motorContactSoftening && !input.reachedTouchRange) {
            decision.reason = "seatedPalmPocketPromotionContactSofteningKeepFrozen";
            return decision;
        }

        if (!input.supportPatchValid ||
            !input.supportPatchNormalTrusted ||
            input.supportPatchSampleCount < 4) {
            decision.reason = "seatedSupportGroupPromotionMissingSupport";
            return decision;
        }

        const float localDelta = finiteOr(input.candidateLocalDeltaGameUnits, std::numeric_limits<float>::infinity());
        const float immediateMax = safePositive(input.immediateMaxLocalDeltaGameUnits, 4.0f);
        if (std::isfinite(localDelta) && localDelta <= immediateMax) {
            decision.promotePivot = true;
            decision.completeSeatedRelation = true;
            decision.enrichSupport = false;
            decision.pivotBlend = 1.0f;
            decision.reason = "seatedPalmPocketPromotionImmediate";
            return decision;
        }

        decision.reason = "seatedPalmPocketPromotionCandidateTooFarKeepFrozen";
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
        bool requiresSettledVisualRelation = false;
        std::uint32_t multiFingerContactGroupCount = 0;
        std::uint32_t contactPatchSampleCount = 0;
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
        state.softenForContact = input.heldBodyColliding;
        state.angular = computeAngularAuthorityScale(input.angular);
        state.releaseAngularVelocityScale = state.angular.authorityScale;
        if (state.softenForContact) {
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
        if (input.requiresSettledVisualRelation && !input.hasSeatedPivotReacquire) {
            decision.reason = "awaitingSettledVisualRelation";
            return decision;
        }

        if (input.acquisitionVisualEligible && !input.touchHeldPhase) {
            decision.acquisition = true;
            if (input.motorContactSoftening) {
                decision.reason = "acquisitionPushingIntoContact";
                return decision;
            }
            decision.apply = true;
            decision.reason = "acquisitionAuthorityAccepted";
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

    inline MotorOutput solveMotorTargetsWithAuthority(const MotorInput& input, const HeldAuthorityState& heldAuthority)
    {
        MotorOutput out{};

        const float baseLinearTau = safePositive(input.baseLinearTau, 0.03f);
        const float baseAngularTau = safePositive(input.baseAngularTau, baseLinearTau);
        const float collisionTau = safePositive(input.collisionTau, baseLinearTau);

        /*
         * ROCK dynamic grabs keep normal held motors on fixed base tau and one
         * shared force budget. Patch/contact/lever quality remains available to release safety,
         * not live motor authority. Tiny or one-point patches must not make the
         * held object too weak to follow the hand.
         */
        const float linearTauTarget = heldAuthority.softenForContact ? collisionTau : baseLinearTau;
        const float angularTauTarget = heldAuthority.softenForContact ? collisionTau : baseAngularTau;
        out.linearTau = advanceToward(input.currentLinearTau, linearTauTarget, input.tauLerpSpeed, input.deltaTime);
        out.angularTau = advanceToward(input.currentAngularTau, angularTauTarget, input.tauLerpSpeed, input.deltaTime);

        const float baseForce = (std::max)(0.0f, finiteOr(input.baseMaxForce, 0.0f));
        const float authorityForceScale = std::clamp(safePositive(input.authorityForceScale, 1.0f), 0.05f, 1.0f);
        out.fadeFactor = input.fadeInEnabled ? computeFadeFactor(input.fadeElapsed, input.fadeDuration) : 1.0f;
        out.physicsHz = computePhysicsHz(input.physicsDeltaSeconds, input.physicsRateReferenceHz);
        out.physicsRateForceScale = computePhysicsRateForceScale(
            input.physicsRateForceScalingEnabled,
            input.physicsDeltaSeconds,
            input.physicsRateReferenceHz,
            input.physicsRateForceScaleExponent,
            input.physicsRateMinForceScale,
            input.physicsRateMaxForceScale);
        const float motorMass = effectiveMotorMass(
            input.mass,
            input.effectiveMotorMassFloorEnabled,
            input.effectiveMotorMassFloor);
        const float scaledBaseForce = baseForce * out.physicsRateForceScale;
        out.linearMaxForce = capForceByMass(scaledBaseForce * out.fadeFactor, motorMass, input.forceToMassRatio) * authorityForceScale;
        const float angularForceMultiplier =
            std::clamp(safePositive(input.angularForceMultiplier, 1.0f), 0.05f, 8.0f);
        out.angularMaxForce = out.linearMaxForce * angularForceMultiplier;
        return out;
    }

    inline MotorOutput solveMotorTargets(const MotorInput& input)
    {
        return solveMotorTargetsWithAuthority(input, HeldAuthorityState{
            .softenForContact = input.heldBodyColliding,
            .reason = input.heldBodyColliding ? "contact-softened-authority" : "full-authority",
        });
    }
}
