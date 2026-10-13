#pragma once

#include "physics-interaction/VectorMath.h"

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
    struct MotorInput
    {
        bool heldBodyColliding = false;

        float baseLinearTau = 0.1f;
        float baseAngularTau = 0.03f;
        float collisionTau = 0.01f;
        float currentLinearTau = 0.1f;
        float currentAngularTau = 0.03f;
        float tauLerpSpeed = 0.5f;
        // Measured game-frame delta; zero (no tau advance) until measured.
        float deltaTime = 0.0f;
        // Measured physics substep delta; zero means unknown rate and yields
        // no measured rate.
        float physicsDeltaSeconds = 0.0f;

        // Contact effort caps; free motion uses mass and inertia below.
        float baseMaxForce = 2000.0f;
        float authorityForceScale = 1.0f;
        float angularForceMultiplier = 1.0f;
        float mass = 0.0f;
        float maximumInertia = 0.0f;
        float gripRadiusHavok = 0.0f;
        float freeLinearAcceleration = 1000.0f;
        float freeAngularAcceleration = 6000.0f;
        float forceToMassRatio = 500.0f;
        bool effectiveMotorMassFloorEnabled = true;
        float effectiveMotorMassFloor = 2.0f;

        bool fadeInEnabled = true;
        float fadeElapsed = 1.0f;
        float fadeDuration = 0.1f;
    };

    struct MotorOutput
    {
        bool valid = false;
        float linearTau = 0.1f;
        float angularTau = 0.03f;
        float linearMaxForce = 0.0f;
        float angularMaxForce = 0.0f;
        float fadeFactor = 1.0f;
        // Zero until a measured physics delta produced a rate.
        float physicsHz = 0.0f;
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

    /*
     * Held motors read one fact: whether the held object is pressed into
     * something. Release keeps a lower spin cap for that case.
     */
    struct HeldAuthorityState
    {
        bool softenForContact = false;
        float releaseAngularVelocityScale = 1.0f;
        const char* reason = "full-authority";
    };

    struct VisualHandPublishInput
    {
        bool hasTelemetryCapture = false;
        bool touchHeldPhase = false;
        bool acquisitionVisualEligible = false;
        bool hasPivotTrackingError = false;
        bool motorContactSoftening = false;
        bool hasSettledVisualRelation = false;
        bool requiresSettledVisualRelation = false;
        bool transferredAuthoredGrip = false;
    };

    struct VisualHandPublishDecision
    {
        bool apply = false;
        bool acquisition = false;
        const char* reason = "notEvaluated";
    };

    inline HeldAuthorityState evaluateHeldAuthority(bool heldBodyColliding)
    {
        HeldAuthorityState state{};
        state.softenForContact = heldBodyColliding;
        if (state.softenForContact) {
            state.releaseAngularVelocityScale = 0.75f;
            state.reason = "contact-softened-authority";
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
        // An equipped-to-loose transfer continues a held grip. Its authored
        // hand relation is already valid while the new physics seat converges;
        // new-grab contact/settling gates must not release that rendered hand.
        if (input.transferredAuthoredGrip) {
            decision.apply = true;
            decision.reason = "transferredAuthoredGripAccepted";
            return decision;
        }
        if (!input.touchHeldPhase && !input.acquisitionVisualEligible) {
            decision.reason = "phaseNotVisualEligible";
            return decision;
        }
        if (input.requiresSettledVisualRelation && !input.hasSettledVisualRelation) {
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

    inline MotorOutput solveMotorTargetsWithAuthority(const MotorInput& input, const HeldAuthorityState& heldAuthority)
    {
        MotorOutput out{};

        if (!std::isfinite(input.mass) || input.mass <= 0.0f ||
            !std::isfinite(input.maximumInertia) || input.maximumInertia <= 0.0f ||
            !std::isfinite(input.gripRadiusHavok) || input.gripRadiusHavok < 0.0f ||
            !std::isfinite(input.freeLinearAcceleration) || input.freeLinearAcceleration <= 0.0f ||
            !std::isfinite(input.freeAngularAcceleration) || input.freeAngularAcceleration <= 0.0f) return out;
        const float baseLinearTau = safePositive(input.baseLinearTau, 0.1f);
        const float baseAngularTau = safePositive(input.baseAngularTau, baseLinearTau);
        const float collisionTau = safePositive(input.collisionTau, baseLinearTau);

        // Native tau multiplies recovery demand: higher tau is stronger.
        // Contact retains its existing softened response and finite force cap.
        const float linearTauTarget = heldAuthority.softenForContact ? collisionTau : baseLinearTau;
        const float angularTauTarget = heldAuthority.softenForContact ? collisionTau : baseAngularTau;
        out.linearTau = advanceToward(input.currentLinearTau, linearTauTarget, input.tauLerpSpeed, input.deltaTime);
        out.angularTau = advanceToward(input.currentAngularTau, angularTauTarget, input.tauLerpSpeed, input.deltaTime);

        const float baseForce = (std::max)(0.0f, finiteOr(input.baseMaxForce, 0.0f));
        const float authorityForceScale = std::clamp(safePositive(input.authorityForceScale, 1.0f), 0.05f, 1.0f);
        out.fadeFactor = input.fadeInEnabled ? computeFadeFactor(input.fadeElapsed, input.fadeDuration) : 1.0f;
        out.physicsHz = computePhysicsHz(input.physicsDeltaSeconds);
        const float motorMass = effectiveMotorMass(
            input.mass,
            input.effectiveMotorMassFloorEnabled,
            input.effectiveMotorMassFloor);
        const float angularForceMultiplier =
            std::clamp(safePositive(input.angularForceMultiplier, 1.0f), 0.05f, 8.0f);
        if (heldAuthority.softenForContact) {
            out.linearMaxForce = capForceByMass(baseForce * out.fadeFactor, motorMass, input.forceToMassRatio) * authorityForceScale;
            out.angularMaxForce = out.linearMaxForce * angularForceMultiplier;
        } else {
            const float freeLinear = motorMass * input.freeLinearAcceleration;
            // A shared angular motor needs a bound valid for every orientation.
            // max principal inertia bounds n^T I n. The lever term allows the
            // angular row to oppose r x F from all three linear rows. These are
            // CAPACITIES only: the native coupled solver still computes effort.
            // No parallel-axis torque or feed-forward impulse is applied twice.
            const float freeAngular = input.maximumInertia * input.freeAngularAcceleration +
                input.gripRadiusHavok * 1.732050808f * freeLinear;
            out.linearMaxForce = freeLinear * out.fadeFactor * authorityForceScale;
            out.angularMaxForce = freeAngular * out.fadeFactor * authorityForceScale;
        }
        if (!std::isfinite(out.linearMaxForce) || !std::isfinite(out.angularMaxForce)) return {};
        out.valid = true;
        return out;
    }

}
