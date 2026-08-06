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

    template <class Vector>
    struct DirectionLatch
    {
        Vector candidateSum{};
        Vector neutralControllerLocal{};
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
                state.neutralControllerLocal)) {
            resetCandidate(state);
            return false;
        }

        state.latched = true;
        return true;
    }

    /*
     * EquippedWeaponData::fireNode is ROCK's authoritative projectile frame.
     * Its stored local +Y axis is the same direction published by the provider
     * muzzle snapshot. Capture it in controller space only while the weapon is
     * neutral; the latched value must not follow live recoil.
     */
    template <class Transform, class Vector>
    [[nodiscard]] inline bool tryCaptureControllerLocalBore(
        const Transform& controllerWorld,
        const Transform& projectileWorld,
        Vector& outControllerLocalBore)
    {
        outControllerLocalBore = {};
        if (!usableTransform(controllerWorld) ||
            !usableTransform(projectileWorld)) {
            return false;
        }

        const Vector localForward{ 0.0f, 1.0f, 0.0f };
        const Vector boreWorld =
            transform_math::localVectorToWorld(projectileWorld, localForward);
        const Vector boreControllerLocal =
            transform_math::worldVectorToLocal(
                controllerWorld,
                boreWorld);
        return tryNormalizeDirection(
            boreControllerLocal,
            outControllerLocalBore);
    }

    template <class Transform, class Matrix, class Vector>
    [[nodiscard]] inline bool tryBuildWorldCorrection(
        const Transform& controllerWorld,
        const Vector& neutralControllerLocal,
        const Vector& targetForwardWorld,
        Matrix& outCorrection,
        float* outDirectionDot = nullptr)
    {
        outCorrection = transform_math::makeIdentityRotation<Matrix>();
        if (!usableTransform(controllerWorld)) {
            return false;
        }

        Vector neutralLocal{};
        if (!tryNormalizeDirection(
                neutralControllerLocal,
                neutralLocal)) {
            return false;
        }

        Vector expectedNeutralWorld{};
        Vector targetWorld{};
        if (!tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    controllerWorld,
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
