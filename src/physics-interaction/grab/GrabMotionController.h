#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::grab_motion_controller
{
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
    };

    struct AngularAuthorityOutput
    {
        float authorityScale = 1.0f;
        float dampingMultiplier = 1.0f;
        float weakPivotTwistScale = 1.0f;
        float pivotQualityScale = 1.0f;
        float contactSupportScale = 1.0f;
        float smallObjectScale = 1.0f;
        bool weakPivot = false;
        bool lowContactSupport = false;
        bool smallObject = false;
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

        output.weakPivot = input.positionOnlyPivot || !input.normalTrusted;
        if (output.weakPivot) {
            output.pivotQualityScale = positionOnlyScale;
            output.dampingMultiplier = (std::max)(1.0f, safePositive(input.weakNormalAngularDampingMultiplier, 1.75f));
            output.weakPivotTwistScale = std::clamp(std::isfinite(input.weakPivotTwistScale) ? input.weakPivotTwistScale : 0.35f, 0.0f, 1.0f);
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

        output.authorityScale = std::clamp(
            output.pivotQualityScale * output.contactSupportScale * output.smallObjectScale,
            floor,
            1.0f);
        return output;
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

    inline MotorOutput solveMotorTargets(const MotorInput& input)
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

        const float linearTauTarget = input.heldBodyColliding ? collisionTau : baseLinearTau + (maxTau - baseLinearTau) * out.linearErrorFactor;
        const float angularTauTarget = input.heldBodyColliding ? collisionTau : baseAngularTau + (maxAngularTau - baseAngularTau) * out.angularErrorFactor;
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
        const auto angularAuthority = computeAngularAuthorityScale(AngularAuthorityInput{
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
        });
        out.angularAuthorityScale = angularAuthority.authorityScale;
        out.angularDampingMultiplier = angularAuthority.dampingMultiplier;
        out.weakPivotTwistScale = angularAuthority.weakPivotTwistScale;
        out.angularMaxForce = angularForceFromRatio(out.linearMaxForce, angularFadeRatio) * out.angularAuthorityScale;
        return out;
    }
}
