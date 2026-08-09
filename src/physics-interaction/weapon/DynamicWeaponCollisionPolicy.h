#pragma once

#include "physics-interaction/TransformMath.h"

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace rock::dynamic_weapon_collision_policy
{
    inline constexpr float kMinimumBoxHalfExtentGameUnits = 0.25f;

    struct BoxGeometry
    {
        RE::NiPoint3 centerWeaponLocal{};
        RE::NiPoint3 halfExtentsWeaponLocal{};
        bool valid{ false };
    };

    struct AttachedHandSelection
    {
        bool left{ false };
        bool right{ false };
    };

    inline AttachedHandSelection selectAttachedHands(
        const bool partCarry,
        const bool firingGripOccupied,
        const bool firingHandIsLeft,
        const bool leftPartGripActive,
        const bool rightPartGripActive)
    {
        AttachedHandSelection result{
            .left = leftPartGripActive,
            .right = rightPartGripActive,
        };
        if (!partCarry) {
            const bool firingHandIsActuallyLeft =
                firingGripOccupied && firingHandIsLeft;
            if (firingHandIsActuallyLeft) {
                result.left = true;
            } else {
                // Passive/native equipped carry is always captured from the
                // physical right firing hand in ROCK's current topology.
                result.right = true;
            }
        }
        return result;
    }

    inline bool isFinitePoint(const RE::NiPoint3& point)
    {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    }

    inline bool isFiniteTransform(const RE::NiTransform& transform)
    {
        if (!isFinitePoint(transform.translate) || !std::isfinite(transform.scale) || std::abs(transform.scale) <= 0.0001f) {
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

    inline BoxGeometry makeBoxGeometry(const RE::NiPoint3& boundsMin, const RE::NiPoint3& boundsMax)
    {
        BoxGeometry result{};
        if (!isFinitePoint(boundsMin) || !isFinitePoint(boundsMax) ||
            boundsMax.x < boundsMin.x || boundsMax.y < boundsMin.y || boundsMax.z < boundsMin.z) {
            return result;
        }

        result.centerWeaponLocal = RE::NiPoint3{
            (boundsMin.x + boundsMax.x) * 0.5f,
            (boundsMin.y + boundsMax.y) * 0.5f,
            (boundsMin.z + boundsMax.z) * 0.5f,
        };
        result.halfExtentsWeaponLocal = RE::NiPoint3{
            (std::max)((boundsMax.x - boundsMin.x) * 0.5f, kMinimumBoxHalfExtentGameUnits),
            (std::max)((boundsMax.y - boundsMin.y) * 0.5f, kMinimumBoxHalfExtentGameUnits),
            (std::max)((boundsMax.z - boundsMin.z) * 0.5f, kMinimumBoxHalfExtentGameUnits),
        };
        result.valid = isFinitePoint(result.centerWeaponLocal) && isFinitePoint(result.halfExtentsWeaponLocal);
        return result;
    }

    inline std::array<RE::NiPoint3, 8> makeBoxCornerPointsHavok(
        const BoxGeometry& geometry,
        float weaponScale,
        float paddingGameUnits,
        float gameToHavokScale)
    {
        std::array<RE::NiPoint3, 8> result{};
        const float scale = std::abs(weaponScale);
        const float padding = (std::max)(0.0f, paddingGameUnits);
        const RE::NiPoint3 half{
            (geometry.halfExtentsWeaponLocal.x * scale + padding) * gameToHavokScale,
            (geometry.halfExtentsWeaponLocal.y * scale + padding) * gameToHavokScale,
            (geometry.halfExtentsWeaponLocal.z * scale + padding) * gameToHavokScale,
        };

        std::size_t index = 0;
        for (int x = -1; x <= 1; x += 2) {
            for (int y = -1; y <= 1; y += 2) {
                for (int z = -1; z <= 1; z += 2) {
                    result[index++] = RE::NiPoint3{
                        half.x * static_cast<float>(x),
                        half.y * static_cast<float>(y),
                        half.z * static_cast<float>(z),
                    };
                }
            }
        }
        return result;
    }

    inline RE::NiTransform makeProxyBodyTarget(
        const RE::NiTransform& weaponRootWorld,
        const RE::NiPoint3& centerWeaponLocal)
    {
        RE::NiTransform result = weaponRootWorld;
        result.rotate = transform_math::transposeRotation(weaponRootWorld.rotate);
        result.translate = transform_math::localPointToWorld(weaponRootWorld, centerWeaponLocal);
        return result;
    }

    inline RE::NiTransform reconstructWeaponRoot(
        const RE::NiTransform& proxyBodyWorld,
        const RE::NiPoint3& centerWeaponLocal,
        float weaponScale)
    {
        RE::NiTransform result = proxyBodyWorld;
        result.rotate = transform_math::transposeRotation(proxyBodyWorld.rotate);
        result.scale = weaponScale;
        const RE::NiPoint3 scaledCenter{
            centerWeaponLocal.x * weaponScale,
            centerWeaponLocal.y * weaponScale,
            centerWeaponLocal.z * weaponScale,
        };
        const RE::NiPoint3 centerOffsetWorld = transform_math::rotateLocalVectorToWorld(result.rotate, scaledCenter);
        result.translate = RE::NiPoint3{
            proxyBodyWorld.translate.x - centerOffsetWorld.x,
            proxyBodyWorld.translate.y - centerOffsetWorld.y,
            proxyBodyWorld.translate.z - centerOffsetWorld.z,
        };
        return result;
    }

    inline RE::NiTransform resolveCurrentIntentFromSample(
        const RE::NiTransform& sampledRequestedProxyBodyWorld,
        const RE::NiTransform& sampledLiveProxyBodyWorld,
        const RE::NiPoint3& centerWeaponLocal,
        float sampledWeaponScale,
        const RE::NiTransform& currentRequestedWeaponWorld)
    {
        const RE::NiTransform sampledRequestedWeaponWorld = reconstructWeaponRoot(
            sampledRequestedProxyBodyWorld,
            centerWeaponLocal,
            sampledWeaponScale);
        const RE::NiTransform sampledLiveWeaponWorld = reconstructWeaponRoot(
            sampledLiveProxyBodyWorld,
            centerWeaponLocal,
            sampledWeaponScale);
        const RE::NiTransform currentRelativeToSample = transform_math::composeTransforms(
            transform_math::invertTransform(sampledRequestedWeaponWorld),
            currentRequestedWeaponWorld);
        return transform_math::composeTransforms(sampledLiveWeaponWorld, currentRelativeToSample);
    }

    inline RE::NiTransform reframeAttachedHand(
        const RE::NiTransform& requestedWeaponWorld,
        const RE::NiTransform& resolvedWeaponWorld,
        const RE::NiTransform& requestedHandWorld)
    {
        const RE::NiTransform handWeaponLocal = transform_math::composeTransforms(
            transform_math::invertTransform(requestedWeaponWorld),
            requestedHandWorld);
        return transform_math::composeTransforms(
            resolvedWeaponWorld,
            handWeaponLocal);
    }

    inline float translationDeltaGameUnits(const RE::NiTransform& lhs, const RE::NiTransform& rhs)
    {
        const float x = lhs.translate.x - rhs.translate.x;
        const float y = lhs.translate.y - rhs.translate.y;
        const float z = lhs.translate.z - rhs.translate.z;
        return std::sqrt(x * x + y * y + z * z);
    }

    inline float rotationDeltaDegrees(const RE::NiTransform& lhs, const RE::NiTransform& rhs)
    {
        float matchingAxisDotSum = 0.0f;
        for (int axis = 0; axis < 3; ++axis) {
            for (int component = 0; component < 3; ++component) {
                matchingAxisDotSum += lhs.rotate.entry[axis][component] * rhs.rotate.entry[axis][component];
            }
        }
        const float cosine = std::clamp((matchingAxisDotSum - 1.0f) * 0.5f, -1.0f, 1.0f);
        return std::acos(cosine) * 57.29577951308232f;
    }

    inline RE::NiTransform blendTransforms(
        const RE::NiTransform& from,
        const RE::NiTransform& to,
        float translationAlpha,
        float rotationAlpha)
    {
        translationAlpha = std::clamp(translationAlpha, 0.0f, 1.0f);
        rotationAlpha = std::clamp(rotationAlpha, 0.0f, 1.0f);

        RE::NiTransform result = from;
        result.translate = RE::NiPoint3{
            from.translate.x + (to.translate.x - from.translate.x) * translationAlpha,
            from.translate.y + (to.translate.y - from.translate.y) * translationAlpha,
            from.translate.z + (to.translate.z - from.translate.z) * translationAlpha,
        };
        result.scale = from.scale + (to.scale - from.scale) * translationAlpha;

        float fromQuaternion[4]{};
        float toQuaternion[4]{};
        transform_math::niRowsToHavokQuaternion(from.rotate, fromQuaternion);
        transform_math::niRowsToHavokQuaternion(to.rotate, toQuaternion);
        const float quaternionDot =
            fromQuaternion[0] * toQuaternion[0] +
            fromQuaternion[1] * toQuaternion[1] +
            fromQuaternion[2] * toQuaternion[2] +
            fromQuaternion[3] * toQuaternion[3];
        if (quaternionDot < 0.0f) {
            for (int component = 0; component < 4; ++component) {
                toQuaternion[component] = -toQuaternion[component];
            }
        }

        float blendedQuaternion[4]{};
        for (int component = 0; component < 4; ++component) {
            blendedQuaternion[component] =
                fromQuaternion[component] +
                (toQuaternion[component] - fromQuaternion[component]) * rotationAlpha;
        }
        result.rotate = transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(blendedQuaternion);
        return result;
    }

    inline RE::NiTransform makeBoundedContactAnchor(
        const RE::NiTransform& liveProxyBodyWorld,
        const RE::NiTransform& requestedProxyBodyWorld,
        float maxTranslationBiasGameUnits,
        float maxRotationBiasDegrees)
    {
        const float translation = translationDeltaGameUnits(liveProxyBodyWorld, requestedProxyBodyWorld);
        const float rotation = rotationDeltaDegrees(liveProxyBodyWorld, requestedProxyBodyWorld);
        float alpha = 1.0f;
        if (std::isfinite(translation) && translation > 0.0001f &&
            std::isfinite(maxTranslationBiasGameUnits) && maxTranslationBiasGameUnits >= 0.0f) {
            alpha = (std::min)(alpha, maxTranslationBiasGameUnits / translation);
        }
        if (std::isfinite(rotation) && rotation > 0.0001f &&
            std::isfinite(maxRotationBiasDegrees) && maxRotationBiasDegrees >= 0.0f) {
            alpha = (std::min)(alpha, maxRotationBiasDegrees / rotation);
        }
        return blendTransforms(liveProxyBodyWorld, requestedProxyBodyWorld, alpha, alpha);
    }

    inline RE::NiTransform advanceSurfaceCoupledTarget(
        const RE::NiTransform& previousRawProxyBodyWorld,
        const RE::NiTransform& currentRawProxyBodyWorld,
        const RE::NiTransform& previousCoupledProxyBodyWorld)
    {
        const RE::NiTransform currentRelativeToPreviousIntent = transform_math::composeTransforms(
            transform_math::invertTransform(previousRawProxyBodyWorld),
            currentRawProxyBodyWorld);
        return transform_math::composeTransforms(
            previousCoupledProxyBodyWorld,
            currentRelativeToPreviousIntent);
    }

    struct SurfaceRecoveryResult
    {
        RE::NiTransform target{};
        float translationAlpha{ 0.0f };
        float rotationAlpha{ 0.0f };
        bool translationOpposedRawMotion{ false };
    };

    inline SurfaceRecoveryResult recoverSurfaceCoupledTarget(
        const RE::NiTransform& followedCoupledProxyBodyWorld,
        const RE::NiTransform& previousRawProxyBodyWorld,
        const RE::NiTransform& currentRawProxyBodyWorld,
        float deltaSeconds,
        float recoveryHalfLifeSeconds,
        float maxOpposingRawMotionFraction)
    {
        SurfaceRecoveryResult result{};
        result.target = followedCoupledProxyBodyWorld;
        if (!isFiniteTransform(followedCoupledProxyBodyWorld) ||
            !isFiniteTransform(previousRawProxyBodyWorld) ||
            !isFiniteTransform(currentRawProxyBodyWorld) ||
            !std::isfinite(deltaSeconds) || deltaSeconds <= 0.0f ||
            !std::isfinite(recoveryHalfLifeSeconds) || recoveryHalfLifeSeconds <= 0.0f) {
            return result;
        }

        const float clampedDeltaSeconds = std::clamp(deltaSeconds, 0.0f, 0.1f);
        const float recoveryAlpha = std::clamp(
            1.0f - std::exp2(-clampedDeltaSeconds / recoveryHalfLifeSeconds),
            0.0f,
            1.0f);
        result.translationAlpha = recoveryAlpha;
        result.rotationAlpha = recoveryAlpha;

        const RE::NiPoint3 rawStep{
            currentRawProxyBodyWorld.translate.x - previousRawProxyBodyWorld.translate.x,
            currentRawProxyBodyWorld.translate.y - previousRawProxyBodyWorld.translate.y,
            currentRawProxyBodyWorld.translate.z - previousRawProxyBodyWorld.translate.z,
        };
        const RE::NiPoint3 recoveryDirection{
            currentRawProxyBodyWorld.translate.x - followedCoupledProxyBodyWorld.translate.x,
            currentRawProxyBodyWorld.translate.y - followedCoupledProxyBodyWorld.translate.y,
            currentRawProxyBodyWorld.translate.z - followedCoupledProxyBodyWorld.translate.z,
        };
        const float rawStepLength = std::sqrt(
            rawStep.x * rawStep.x + rawStep.y * rawStep.y + rawStep.z * rawStep.z);
        const float translationError = std::sqrt(
            recoveryDirection.x * recoveryDirection.x +
            recoveryDirection.y * recoveryDirection.y +
            recoveryDirection.z * recoveryDirection.z);
        const float recoveryDotRawStep =
            recoveryDirection.x * rawStep.x +
            recoveryDirection.y * rawStep.y +
            recoveryDirection.z * rawStep.z;
        const float motionFraction = std::clamp(maxOpposingRawMotionFraction, 0.0f, 1.0f);
        if (recoveryDotRawStep < 0.0f && rawStepLength > 0.0001f && translationError > 0.0001f) {
            result.translationOpposedRawMotion = true;
            result.translationAlpha = (std::min)(
                result.translationAlpha,
                (rawStepLength * motionFraction) / translationError);
        }

        const float rawRotationStep = rotationDeltaDegrees(
            previousRawProxyBodyWorld,
            currentRawProxyBodyWorld);
        const float rotationError = rotationDeltaDegrees(
            followedCoupledProxyBodyWorld,
            currentRawProxyBodyWorld);
        if (std::isfinite(rawRotationStep) && rawRotationStep > 0.0001f &&
            std::isfinite(rotationError) && rotationError > 0.0001f) {
            // Rotation direction is intentionally not inferred from Euler axes.
            // Bound recovery during meaningful controller rotation, then use
            // normal exponential convergence as soon as that motion subsides.
            result.rotationAlpha = (std::min)(
                result.rotationAlpha,
                (rawRotationStep * motionFraction) / rotationError);
        }

        result.target = blendTransforms(
            followedCoupledProxyBodyWorld,
            currentRawProxyBodyWorld,
            result.translationAlpha,
            result.rotationAlpha);
        return result;
    }
}
