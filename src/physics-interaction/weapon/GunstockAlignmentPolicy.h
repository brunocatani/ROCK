#pragma once

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/WeaponSupport.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::gunstock_alignment_policy
{
    inline constexpr std::uint32_t kRequiredStableSamples = 6;
    inline constexpr float kStableSampleCosine = 0.9999904807f;  // 0.25 degrees.
    inline constexpr float kMinimumDirectionLength = 0.000001f;
    inline constexpr float kDegreesToRadians =
        0.01745329251994329577f;

    struct FineTuneDegrees
    {
        float pitchDegrees{ 0.0f };
        float yawDegrees{ 0.0f };
        float rollDegrees{ 0.0f };
    };

    enum class ModeToggleEdge : std::uint8_t
    {
        None,
        Enabled,
        Disabled,
    };

    struct ModeToggleState
    {
        bool initialized{ false };
        bool enabled{ false };
    };

    [[nodiscard]] inline ModeToggleEdge observeModeToggle(
        ModeToggleState& state,
        const bool enabled)
    {
        if (!state.initialized) {
            state.initialized = true;
            state.enabled = enabled;
            return ModeToggleEdge::None;
        }
        if (state.enabled == enabled) {
            return ModeToggleEdge::None;
        }

        state.enabled = enabled;
        return enabled ? ModeToggleEdge::Enabled : ModeToggleEdge::Disabled;
    }

    /*
     * Both gunstock stages consume this one value-only witness. A native gun
     * type observation together with a valid native fire node establishes
     * firearm eligibility for its exact weapon root and collision generation.
     * Eligibility remains latched across transient firing-animation node loss,
     * but never crosses an identity boundary or a frame where behavior and
     * diagnostics are both disabled.
     */
    struct WeaponEligibilityState
    {
        std::uintptr_t weaponNodeIdentity{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        bool eligible{ false };
    };

    inline void observeWeaponEligibility(
        WeaponEligibilityState& state,
        const std::uintptr_t weaponNodeIdentity,
        const std::uint64_t weaponGenerationKey,
        const bool featureOrDebugActive,
        const bool gunTypeWitnessObserved,
        const bool validFireNodeObserved)
    {
        if (!featureOrDebugActive ||
            weaponNodeIdentity == 0 ||
            weaponGenerationKey == 0) {
            state = {};
            return;
        }

        if (state.weaponNodeIdentity != weaponNodeIdentity ||
            state.weaponGenerationKey != weaponGenerationKey) {
            state = {
                .weaponNodeIdentity = weaponNodeIdentity,
                .weaponGenerationKey = weaponGenerationKey,
                .eligible = false,
            };
        }
        if (gunTypeWitnessObserved && validFireNodeObserved) {
            state.eligible = true;
        }
    }

    [[nodiscard]] inline bool isWeaponEligible(
        const WeaponEligibilityState& state,
        const std::uintptr_t weaponNodeIdentity,
        const std::uint64_t weaponGenerationKey)
    {
        return state.eligible &&
               weaponNodeIdentity != 0 &&
               state.weaponNodeIdentity == weaponNodeIdentity &&
               weaponGenerationKey != 0 &&
               state.weaponGenerationKey == weaponGenerationKey;
    }

    template <class Vector>
    struct DirectionLatch
    {
        Vector candidateSum{};
        Vector neutralHandLocal{};
        std::uint32_t candidateSamples{ 0 };
        bool latched{ false };
    };

    template <class Vector>
    [[nodiscard]] inline bool finiteVector(const Vector& value)
    {
        return std::isfinite(value.x) &&
               std::isfinite(value.y) &&
               std::isfinite(value.z);
    }

    template <class Matrix>
    [[nodiscard]] inline bool finiteRotation(const Matrix& value)
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(value.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    template <class Transform>
    [[nodiscard]] inline bool usableTransform(const Transform& value)
    {
        return finiteRotation(value.rotate) &&
               finiteVector(value.translate) &&
               std::isfinite(value.scale) &&
               std::abs(value.scale) > 0.0001f;
    }

    [[nodiscard]] inline bool finiteFineTune(
        const FineTuneDegrees& value)
    {
        return std::isfinite(value.pitchDegrees) &&
               std::isfinite(value.yawDegrees) &&
               std::isfinite(value.rollDegrees);
    }

    [[nodiscard]] inline bool hasFineTune(
        const FineTuneDegrees& value)
    {
        return finiteFineTune(value) &&
               (value.pitchDegrees != 0.0f ||
                   value.yawDegrees != 0.0f ||
                   value.rollDegrees != 0.0f);
    }

    template <class Vector>
    [[nodiscard]] inline bool tryNormalizeDirection(
        const Vector& value,
        Vector& outDirection)
    {
        outDirection = {};
        if (!finiteVector(value)) {
            return false;
        }

        const float length = weaponSolverLength(value);
        if (!std::isfinite(length) || length <= kMinimumDirectionLength) {
            return false;
        }

        outDirection = weaponSolverScale(value, 1.0f / length);
        return finiteVector(outDirection);
    }

    template <class Vector>
    inline void resetCandidate(DirectionLatch<Vector>& state)
    {
        state.candidateSum = {};
        state.candidateSamples = 0;
    }

    template <class Vector>
    [[nodiscard]] inline bool observeStableDirection(
        DirectionLatch<Vector>& state,
        const Vector& sampleRaw)
    {
        if (state.latched) {
            return true;
        }

        Vector sample{};
        if (!tryNormalizeDirection(sampleRaw, sample)) {
            resetCandidate(state);
            return false;
        }

        if (state.candidateSamples == 0) {
            state.candidateSum = sample;
            state.candidateSamples = 1;
            return false;
        }

        Vector candidateDirection{};
        if (!tryNormalizeDirection(state.candidateSum, candidateDirection) ||
            weaponSolverDot(candidateDirection, sample) < kStableSampleCosine) {
            state.candidateSum = sample;
            state.candidateSamples = 1;
            return false;
        }

        state.candidateSum = weaponSolverAdd(state.candidateSum, sample);
        ++state.candidateSamples;
        if (state.candidateSamples < kRequiredStableSamples) {
            return false;
        }

        if (!tryNormalizeDirection(
                state.candidateSum,
                state.neutralHandLocal)) {
            resetCandidate(state);
            return false;
        }

        state.latched = true;
        return true;
    }

    /*
     * EquippedWeaponData::fireNode is ROCK's authoritative projectile frame.
     * Its stored local +Y axis is the same direction published by the provider
     * muzzle snapshot. Capture it in the reconstructed physical firing-wrist
     * frame only while the weapon is neutral. That frame contains hFRIK's
     * damping but none of ROCK's prior presentation output, so the fixed
     * additive correction follows the damped hand without feedback.
     * The latched value must not follow live recoil.
     */
    template <class Transform, class Vector>
    [[nodiscard]] inline bool tryCaptureHandLocalBore(
        const Transform& firingHandWorld,
        const Transform& projectileWorld,
        Vector& outHandLocalBore)
    {
        outHandLocalBore = {};
        if (!usableTransform(firingHandWorld) ||
            !usableTransform(projectileWorld)) {
            return false;
        }

        const Vector localForward{ 0.0f, 1.0f, 0.0f };
        const Vector boreWorld =
            transform_math::localVectorToWorld(projectileWorld, localForward);
        const Vector boreHandLocal =
            transform_math::worldVectorToLocal(
                firingHandWorld,
                boreWorld);
        return tryNormalizeDirection(
            boreHandLocal,
            outHandLocalBore);
    }

    template <class Transform, class Matrix, class Vector>
    [[nodiscard]] inline bool tryBuildWorldCorrection(
        const Transform& firingHandWorld,
        const Vector& neutralHandLocal,
        const Vector& targetForwardWorld,
        Matrix& outCorrection,
        float* outDirectionDot = nullptr)
    {
        outCorrection = transform_math::makeIdentityRotation<Matrix>();
        if (!usableTransform(firingHandWorld)) {
            return false;
        }

        Vector neutralLocal{};
        if (!tryNormalizeDirection(
                neutralHandLocal,
                neutralLocal)) {
            return false;
        }

        Vector expectedNeutralWorld{};
        Vector targetWorld{};
        if (!tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    firingHandWorld,
                    neutralLocal),
                expectedNeutralWorld) ||
            !tryNormalizeDirection(
                targetForwardWorld,
                targetWorld)) {
            return false;
        }

        const float directionDot = (std::max)(
            -1.0f,
            (std::min)(
                1.0f,
                weaponSolverDot(
                    expectedNeutralWorld,
                    targetWorld)));
        if (outDirectionDot) {
            *outDirectionDot = directionDot;
        }
        outCorrection = weaponSolverRotationBetweenStored<Matrix, Vector>(
            expectedNeutralWorld,
            targetWorld);
        return finiteRotation(outCorrection);
    }

    /*
     * Fine tuning is expressed in the untrimmed damped firing-wrist frame so
     * the same INI values follow either physical firing hand. The extrinsic
     * order is yaw about wrist +Z, pitch about wrist +Y, then roll about wrist
     * +X. The caller composes this after automatic bore-to-+X alignment and
     * applies the result around the existing damped-driver pivot; weapon and
     * posed hands therefore retain their exact relative grip transforms.
     */
    template <class Transform, class Matrix, class Vector>
    [[nodiscard]] inline bool tryBuildWorldFineTuneRotation(
        const Transform& firingHandWorld,
        const FineTuneDegrees& fineTune,
        Matrix& outFineTuneWorld)
    {
        outFineTuneWorld =
            transform_math::makeIdentityRotation<Matrix>();
        if (!usableTransform(firingHandWorld) ||
            !finiteFineTune(fineTune)) {
            return false;
        }

        Vector rollAxisWorld{};
        Vector pitchAxisWorld{};
        Vector yawAxisWorld{};
        if (!tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    firingHandWorld,
                    Vector{ 1.0f, 0.0f, 0.0f }),
                rollAxisWorld) ||
            !tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    firingHandWorld,
                    Vector{ 0.0f, 1.0f, 0.0f }),
                pitchAxisWorld) ||
            !tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    firingHandWorld,
                    Vector{ 0.0f, 0.0f, 1.0f }),
                yawAxisWorld)) {
            return false;
        }

        const auto appendWorldRotation =
            [&outFineTuneWorld](
                const Vector& axisWorld,
                const float degrees) {
                if (degrees == 0.0f) {
                    return true;
                }
                const Matrix step =
                    weaponSolverAxisAngleStored<Matrix, Vector>(
                        axisWorld,
                        degrees * kDegreesToRadians);
                outFineTuneWorld =
                    weaponSolverApplyWorldRotationToStoredBasis<
                        Matrix,
                        Vector>(step, outFineTuneWorld);
                return finiteRotation(outFineTuneWorld);
            };

        return appendWorldRotation(
                   yawAxisWorld,
                   fineTune.yawDegrees) &&
               appendWorldRotation(
                   pitchAxisWorld,
                   fineTune.pitchDegrees) &&
               appendWorldRotation(
                   rollAxisWorld,
                   fineTune.rollDegrees);
    }

    template <class Transform, class Matrix, class Vector>
    [[nodiscard]] inline bool tryBuildFineTunedWorldCorrection(
        const Transform& firingHandWorld,
        const Vector& neutralHandLocal,
        const Vector& automaticTargetForwardWorld,
        const FineTuneDegrees& fineTune,
        Matrix& outCorrection,
        Vector* outFineTunedTargetForwardWorld = nullptr,
        float* outAutomaticDirectionDot = nullptr)
    {
        outCorrection =
            transform_math::makeIdentityRotation<Matrix>();
        if (outFineTunedTargetForwardWorld) {
            *outFineTunedTargetForwardWorld = {};
        }

        Matrix automaticCorrection{};
        Matrix fineTuneWorld{};
        Vector automaticTarget{};
        if (!tryNormalizeDirection(
                automaticTargetForwardWorld,
                automaticTarget) ||
            !tryBuildWorldCorrection<Transform, Matrix, Vector>(
                firingHandWorld,
                neutralHandLocal,
                automaticTarget,
                automaticCorrection,
                outAutomaticDirectionDot) ||
            !tryBuildWorldFineTuneRotation<Transform, Matrix, Vector>(
                firingHandWorld,
                fineTune,
                fineTuneWorld)) {
            return false;
        }

        outCorrection =
            weaponSolverApplyWorldRotationToStoredBasis<Matrix, Vector>(
                fineTuneWorld,
                automaticCorrection);
        if (!finiteRotation(outCorrection)) {
            return false;
        }

        if (outFineTunedTargetForwardWorld) {
            const Vector fineTunedTarget =
                weaponSolverApplyStoredWorldRotationToVector<
                    Matrix,
                    Vector>(fineTuneWorld, automaticTarget);
            if (!tryNormalizeDirection(
                    fineTunedTarget,
                    *outFineTunedTargetForwardWorld)) {
                return false;
            }
        }
        return true;
    }

    template <class Transform, class Matrix, class Vector>
    [[nodiscard]] inline Transform rotateRigidlyAroundPivot(
        const Transform& value,
        const Matrix& worldCorrection,
        const Vector& pivotWorld)
    {
        Transform result = value;
        result.rotate =
            weaponSolverApplyWorldRotationToStoredBasis<Matrix, Vector>(
                worldCorrection,
                value.rotate);
        result.translate = weaponSolverAdd(
            pivotWorld,
            weaponSolverApplyStoredWorldRotationToVector<Matrix, Vector>(
                worldCorrection,
                weaponSolverSub(value.translate, pivotWorld)));
        return result;
    }

    template <class Transform>
    [[nodiscard]] inline Transform deriveAppliedWorldDelta(
        const Transform& requestedWorld,
        const Transform& appliedWorld)
    {
        return transform_math::composeTransforms(
            appliedWorld,
            transform_math::invertTransform(requestedWorld));
    }

    template <class Transform>
    [[nodiscard]] inline Transform precompensateWorldTarget(
        const Transform& appliedWorldDelta,
        const Transform& desiredAppliedWorld)
    {
        return transform_math::composeTransforms(
            transform_math::invertTransform(appliedWorldDelta),
            desiredAppliedWorld);
    }
}
