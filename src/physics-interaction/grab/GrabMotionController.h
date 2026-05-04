#pragma once

#include <algorithm>
#include <cmath>

namespace rock::grab_motion_controller
{
    /*
     * HIGGS' grabbed-object feel comes from a motor controller, not from a
     * single static force value. ROCK keeps the verified FO4VR hknp constraint
     * ownership, but computes the motor targets in one pure helper so stronger
     * follow behavior, collision softening, mass caps, and startup fade all use
     * the same rules before any low-level Havok fields are written.
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
        float currentLinearTau = 0.03f;
        float currentAngularTau = 0.03f;
        float tauLerpSpeed = 0.5f;
        float deltaTime = 1.0f / 90.0f;

        float baseMaxForce = 2000.0f;
        float maxForceMultiplier = 4.0f;
        float mass = 0.0f;
        float forceToMassRatio = 500.0f;
        float angularToLinearForceRatio = 12.5f;

        bool fadeInEnabled = true;
        float fadeElapsed = 1.0f;
        float fadeDuration = 0.1f;
        float fadeStartAngularRatio = 100.0f;
    };

    struct MotorOutput
    {
        float errorFactor = 0.0f;
        float linearTau = 0.03f;
        float angularTau = 0.03f;
        float linearMaxForce = 0.0f;
        float angularMaxForce = 0.0f;
        float fadeFactor = 1.0f;
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

    inline MotorOutput solveMotorTargets(const MotorInput& input)
    {
        MotorOutput out{};

        const float baseLinearTau = safePositive(input.baseLinearTau, 0.03f);
        const float baseAngularTau = safePositive(input.baseAngularTau, baseLinearTau);
        const float maxTau = (std::max)(baseLinearTau, safePositive(input.maxTau, baseLinearTau));
        const float collisionTau = safePositive(input.collisionTau, baseLinearTau);
        out.errorFactor = input.enabled ?
            computeErrorFactor(input.positionErrorGameUnits, input.rotationErrorDegrees, input.fullPositionErrorGameUnits, input.fullRotationErrorDegrees) :
            0.0f;

        const float linearTauTarget = input.heldBodyColliding ? collisionTau : baseLinearTau + (maxTau - baseLinearTau) * out.errorFactor;
        const float angularTauTarget = input.heldBodyColliding ? collisionTau : baseAngularTau + (maxTau - baseAngularTau) * out.errorFactor;
        out.linearTau = advanceToward(input.currentLinearTau, linearTauTarget, input.tauLerpSpeed, input.deltaTime);
        out.angularTau = advanceToward(input.currentAngularTau, angularTauTarget, input.tauLerpSpeed, input.deltaTime);

        const float maxMultiplier = (std::max)(1.0f, safePositive(input.maxForceMultiplier, 1.0f));
        const float baseForce = (std::max)(0.0f, finiteOr(input.baseMaxForce, 0.0f));
        const float targetForce = baseForce * (1.0f + (maxMultiplier - 1.0f) * out.errorFactor);
        out.fadeFactor = input.fadeInEnabled ? computeFadeFactor(input.fadeElapsed, input.fadeDuration) : 1.0f;
        out.linearMaxForce = capForceByMass(targetForce * out.fadeFactor, input.mass, input.forceToMassRatio);

        const float angularFadeRatio =
            safePositive(input.fadeStartAngularRatio, input.angularToLinearForceRatio) +
            (safePositive(input.angularToLinearForceRatio, 12.5f) - safePositive(input.fadeStartAngularRatio, input.angularToLinearForceRatio)) * out.fadeFactor;
        out.angularMaxForce = angularForceFromRatio(out.linearMaxForce, angularFadeRatio);
        return out;
    }
}
