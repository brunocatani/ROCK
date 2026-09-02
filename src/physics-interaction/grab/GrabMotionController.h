#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace rock::grab_motion_controller
{
    /*
     * ROCK's grabbed-object feel comes from a motor controller, not from a
     * single static force value. The verified FO4VR hknp constraint remains the
     * owner, but motor targets are computed in one pure helper so stronger follow
     * behavior, collision softening, mass caps, and startup fade all use the same
     * rules before any low-level Havok fields are written.
     */
    /*
     * Angular motor budget as a ratio of the linear budget. Generic held
     * objects use this fixed ratio; loose weapons scale it through
     * fGrabLooseWeaponSharedConstraintAngularForceMultiplier instead.
     */
    inline constexpr float kGrabAngularToLinearForceRatio = 1.0f;

    struct MotorInput
    {
        float baseLinearTau = 0.03f;
        float baseAngularTau = 0.03f;
        float collisionTau = 0.01f;
        float currentLinearTau = 0.03f;
        float currentAngularTau = 0.03f;
        float tauLerpSpeed = 0.5f;
        // Measured game-frame delta; zero (no tau advance) until measured.
        float deltaTime = 0.0f;
        bool physicsRateForceScalingEnabled = false;
        // Measured physics substep delta; zero means unknown rate and yields
        // a neutral force scale.
        float physicsDeltaSeconds = 0.0f;
        // Named calibration reference: the physics rate the grab force tuning
        // was authored at. A rate baseline, not a clock fallback.
        float physicsRateReferenceHz = 90.0f;
        float physicsRateForceScaleExponent = 0.5f;
        float physicsRateMinForceScale = 0.75f;
        float physicsRateMaxForceScale = 1.35f;

        float baseMaxForce = 2000.0f;
        float authorityForceScale = 1.0f;
        float angularToLinearForceRatio = kGrabAngularToLinearForceRatio;
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
        // Zero until a measured physics delta produced a rate.
        float physicsHz = 0.0f;
        float physicsRateForceScale = 1.0f;
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

        // An unmeasured frame advances no time: hold instead of stepping by a
        // fabricated nominal rate.
        const float dt = safePositive(deltaTime, 0.0f);
        if (dt <= 0.0f) {
            return current;
        }
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

    // Returns the measured physics rate, or zero when the delta is
    // unmeasured. Zero means "unknown" honestly; force scaling treats it as
    // the neutral calibration point.
    inline float computePhysicsHz(float physicsDeltaSeconds)
    {
        const float sanitizedDelta = safePositive(physicsDeltaSeconds, 0.0f);
        if (sanitizedDelta <= 0.0f) {
            return 0.0f;
        }

        const float hz = 1.0f / sanitizedDelta;
        return std::isfinite(hz) && hz > 0.0f ? hz : 0.0f;
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
        const float physicsHz = computePhysicsHz(physicsDeltaSeconds);
        const float sanitizedExponent = (std::isfinite(exponent) && exponent >= 0.0f) ? exponent : 0.5f;
        const float lowerScale = safePositive((std::min)(minScale, maxScale), 1.0f);
        const float upperScale = (std::max)(lowerScale, safePositive((std::max)(minScale, maxScale), 1.0f));
        if (physicsHz <= 0.0f || sanitizedReferenceHz <= 0.0f) {
            // Unknown physics rate: neutral scale (the calibration point),
            // never a pretended 90 Hz measurement.
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

    inline float computeAuthorityScaledAngularVelocityCap(float configuredMaxSpeedRadiansPerSecond, float longObjectAngularScale)
    {
        const float configuredMax = std::clamp(
            std::isfinite(configuredMaxSpeedRadiansPerSecond) ? configuredMaxSpeedRadiansPerSecond : 18.0f,
            0.25f,
            64.0f);
        const float longObject = std::clamp(std::isfinite(longObjectAngularScale) && longObjectAngularScale > 0.0f ? longObjectAngularScale : 1.0f, 0.05f, 1.0f);
        return (std::max)(0.25f, configuredMax * longObject);
    }

    inline MotorOutput solveMotorTargets(const MotorInput& input, bool softenForContact)
    {
        MotorOutput out{};

        const float baseLinearTau = safePositive(input.baseLinearTau, 0.03f);
        const float baseAngularTau = safePositive(input.baseAngularTau, baseLinearTau);
        const float collisionTau = safePositive(input.collisionTau, baseLinearTau);

        /*
         * ROCK dynamic grabs keep normal held motors on fixed base tau and one
         * shared force budget. Contact against static or keyframed geometry is
         * the only condition that softens the drive.
         */
        const float linearTauTarget = softenForContact ? collisionTau : baseLinearTau;
        const float angularTauTarget = softenForContact ? collisionTau : baseAngularTau;
        out.linearTau = advanceToward(input.currentLinearTau, linearTauTarget, input.tauLerpSpeed, input.deltaTime);
        out.angularTau = advanceToward(input.currentAngularTau, angularTauTarget, input.tauLerpSpeed, input.deltaTime);

        const float baseForce = (std::max)(0.0f, finiteOr(input.baseMaxForce, 0.0f));
        const float authorityForceScale = std::clamp(safePositive(input.authorityForceScale, 1.0f), 0.05f, 1.0f);
        out.fadeFactor = input.fadeInEnabled ? computeFadeFactor(input.fadeElapsed, input.fadeDuration) : 1.0f;
        out.physicsHz = computePhysicsHz(input.physicsDeltaSeconds);
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
        const float angularToLinearForceRatio =
            std::clamp(safePositive(input.angularToLinearForceRatio, kGrabAngularToLinearForceRatio), 0.05f, 8.0f);
        out.angularMaxForce = out.linearMaxForce * angularToLinearForceRatio;
        return out;
    }
}
