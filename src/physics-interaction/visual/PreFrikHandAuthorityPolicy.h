#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "physics-interaction/TransformMath.h"

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

namespace rock::prefrik_hand_authority_policy
{
    inline constexpr std::uint32_t kRequiredStableCalibrationSamples = 6;
    inline constexpr float kCalibrationTranslationToleranceGameUnits = 1.0f;
    inline constexpr float kCalibrationRotationToleranceDegrees = 3.0f;

    [[nodiscard]] inline bool isImmediateSuccessor(
        std::uint64_t sourceSequence,
        std::uint64_t currentSequence) noexcept
    {
        return currentSequence != 0 && sourceSequence == currentSequence - 1;
    }

    [[nodiscard]] inline bool isFinitePoint(const RE::NiPoint3& point) noexcept
    {
        return std::isfinite(point.x) &&
               std::isfinite(point.y) &&
               std::isfinite(point.z);
    }

    [[nodiscard]] inline bool isUsableTransform(
        const RE::NiTransform& transform) noexcept
    {
        if (!isFinitePoint(transform.translate) ||
            !std::isfinite(transform.scale) ||
            transform.scale <= 0.0001f) {
            return false;
        }
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(transform.rotate.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    [[nodiscard]] inline float translationDeltaGameUnits(
        const RE::NiTransform& lhs,
        const RE::NiTransform& rhs) noexcept
    {
        const float x = lhs.translate.x - rhs.translate.x;
        const float y = lhs.translate.y - rhs.translate.y;
        const float z = lhs.translate.z - rhs.translate.z;
        return std::sqrt(x * x + y * y + z * z);
    }

    [[nodiscard]] inline float rotationDeltaDegrees(
        const RE::NiTransform& lhs,
        const RE::NiTransform& rhs) noexcept
    {
        float matchingAxisDotSum = 0.0f;
        for (int axis = 0; axis < 3; ++axis) {
            for (int component = 0; component < 3; ++component) {
                matchingAxisDotSum +=
                    lhs.rotate.entry[axis][component] *
                    rhs.rotate.entry[axis][component];
            }
        }
        const float cosine = std::clamp(
            (matchingAxisDotSum - 1.0f) * 0.5f,
            -1.0f,
            1.0f);
        return std::acos(cosine) * 57.29577951308232f;
    }

    [[nodiscard]] inline bool calibrationRelationsCoherent(
        const RE::NiTransform& candidate,
        const RE::NiTransform& sample,
        float translationToleranceGameUnits =
            kCalibrationTranslationToleranceGameUnits,
        float rotationToleranceDegrees =
            kCalibrationRotationToleranceDegrees) noexcept
    {
        if (!isUsableTransform(candidate) ||
            !isUsableTransform(sample) ||
            !std::isfinite(translationToleranceGameUnits) ||
            !std::isfinite(rotationToleranceDegrees) ||
            translationToleranceGameUnits < 0.0f ||
            rotationToleranceDegrees < 0.0f) {
            return false;
        }
        return translationDeltaGameUnits(candidate, sample) <=
                   translationToleranceGameUnits &&
               rotationDeltaDegrees(candidate, sample) <=
                   rotationToleranceDegrees;
    }

    [[nodiscard]] inline RE::NiTransform captureDriverToTargetLocal(
        const RE::NiTransform& driverWorld,
        const RE::NiTransform& targetWorld)
    {
        return transform_math::composeTransforms(
            transform_math::invertTransform(driverWorld),
            targetWorld);
    }

    [[nodiscard]] inline RE::NiTransform reconstructTargetWorld(
        const RE::NiTransform& driverWorld,
        const RE::NiTransform& driverToTargetLocal)
    {
        return transform_math::composeTransforms(
            driverWorld,
            driverToTargetLocal);
    }

    struct ContactTransportResult
    {
        RE::NiTransform targetWorld{};
        float removedInwardMotionGameUnits = 0.0f;
        bool valid = false;
    };

    /*
     * A post-solve DHC deviation points from the requested raw hand toward the
     * solver-safe live body. Advancing the old absolute target by all current
     * controller/player motion would move it through the blocking surface.
     * Preserve the previous blocked point along that outward correction normal,
     * while allowing tangential motion and motion away from the surface. The
     * next physics solve remains the authority for a new correction magnitude.
     */
    [[nodiscard]] inline ContactTransportResult transportContactTarget(
        const RE::NiTransform& sourceRawHandWorld,
        const RE::NiTransform& currentRawHandWorld,
        const RE::NiPoint3& appliedDeviationWorldGame,
        float maximumRawMotionGameUnits) noexcept
    {
        ContactTransportResult result{};
        if (!isUsableTransform(sourceRawHandWorld) ||
            !isUsableTransform(currentRawHandWorld) ||
            !isFinitePoint(appliedDeviationWorldGame) ||
            !std::isfinite(maximumRawMotionGameUnits) ||
            maximumRawMotionGameUnits <= 0.0f) {
            return result;
        }

        const RE::NiPoint3 rawMotion{
            currentRawHandWorld.translate.x - sourceRawHandWorld.translate.x,
            currentRawHandWorld.translate.y - sourceRawHandWorld.translate.y,
            currentRawHandWorld.translate.z - sourceRawHandWorld.translate.z,
        };
        const float rawMotionLengthSquared =
            rawMotion.x * rawMotion.x +
            rawMotion.y * rawMotion.y +
            rawMotion.z * rawMotion.z;
        if (!std::isfinite(rawMotionLengthSquared) ||
            rawMotionLengthSquared >
                maximumRawMotionGameUnits * maximumRawMotionGameUnits) {
            return result;
        }

        const float deviationLengthSquared =
            appliedDeviationWorldGame.x * appliedDeviationWorldGame.x +
            appliedDeviationWorldGame.y * appliedDeviationWorldGame.y +
            appliedDeviationWorldGame.z * appliedDeviationWorldGame.z;
        if (!std::isfinite(deviationLengthSquared) ||
            deviationLengthSquared <= 0.000001f) {
            return result;
        }

        const float inverseDeviationLength =
            1.0f / std::sqrt(deviationLengthSquared);
        const RE::NiPoint3 outwardNormal{
            appliedDeviationWorldGame.x * inverseDeviationLength,
            appliedDeviationWorldGame.y * inverseDeviationLength,
            appliedDeviationWorldGame.z * inverseDeviationLength,
        };
        const float motionAlongOutwardNormal =
            rawMotion.x * outwardNormal.x +
            rawMotion.y * outwardNormal.y +
            rawMotion.z * outwardNormal.z;
        const float inwardMotion = std::min(motionAlongOutwardNormal, 0.0f);
        const RE::NiPoint3 contactSafeMotion{
            rawMotion.x - inwardMotion * outwardNormal.x,
            rawMotion.y - inwardMotion * outwardNormal.y,
            rawMotion.z - inwardMotion * outwardNormal.z,
        };

        result.targetWorld = currentRawHandWorld;
        result.targetWorld.translate = RE::NiPoint3{
            sourceRawHandWorld.translate.x + appliedDeviationWorldGame.x +
                contactSafeMotion.x,
            sourceRawHandWorld.translate.y + appliedDeviationWorldGame.y +
                contactSafeMotion.y,
            sourceRawHandWorld.translate.z + appliedDeviationWorldGame.z +
                contactSafeMotion.z,
        };
        result.removedInwardMotionGameUnits = -inwardMotion;
        result.valid = isUsableTransform(result.targetWorld);
        return result;
    }

    /*
     * Preserve the compound body's complete rigid correction across the
     * scheduler boundary. Translation keeps the existing contact-safe inward
     * motion rejection; orientation is reconstructed from the solved-hand
     * relation captured against the source raw hand.
     */
    [[nodiscard]] inline ContactTransportResult transportRigidContactTarget(
        const RE::NiTransform& sourceRawHandWorld,
        const RE::NiTransform& sourceSolvedHandWorld,
        const RE::NiTransform& currentRawHandWorld,
        float maximumRawMotionGameUnits) noexcept
    {
        if (!isUsableTransform(sourceSolvedHandWorld)) {
            return {};
        }
        const RE::NiPoint3 appliedDeviation{
            sourceSolvedHandWorld.translate.x -
                sourceRawHandWorld.translate.x,
            sourceSolvedHandWorld.translate.y -
                sourceRawHandWorld.translate.y,
            sourceSolvedHandWorld.translate.z -
                sourceRawHandWorld.translate.z,
        };
        const float deviationLengthSquared =
            appliedDeviation.x * appliedDeviation.x +
            appliedDeviation.y * appliedDeviation.y +
            appliedDeviation.z * appliedDeviation.z;
        ContactTransportResult result{};
        if (std::isfinite(deviationLengthSquared) &&
            deviationLengthSquared <= 0.000001f) {
            const float rawMotion = translationDeltaGameUnits(
                sourceRawHandWorld,
                currentRawHandWorld);
            if (!std::isfinite(rawMotion) ||
                rawMotion > maximumRawMotionGameUnits) {
                return {};
            }
            result.targetWorld = currentRawHandWorld;
            result.valid = true;
        } else {
            result = transportContactTarget(
                sourceRawHandWorld,
                currentRawHandWorld,
                appliedDeviation,
                maximumRawMotionGameUnits);
        }
        if (!result.valid) {
            return result;
        }

        const RE::NiTransform rawToSolved = captureDriverToTargetLocal(
            sourceRawHandWorld,
            sourceSolvedHandWorld);
        RE::NiTransform rigidTarget = reconstructTargetWorld(
            currentRawHandWorld,
            rawToSolved);
        rigidTarget.translate = result.targetWorld.translate;
        rigidTarget.scale = currentRawHandWorld.scale;
        if (!isUsableTransform(rigidTarget)) {
            return {};
        }
        result.targetWorld = rigidTarget;
        return result;
    }
}
