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
}
