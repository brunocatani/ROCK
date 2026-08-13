#pragma once

#include "physics-interaction/TransformMath.h"

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::dynamic_weapon_collision_policy
{
    inline constexpr float kMinimumBoundingBoxHalfExtentGameUnits = 0.25f;
    inline constexpr float kFallbackWeaponMass = 2.0f;
    inline constexpr float kMaximumWeaponMass = 50.0f;
    inline constexpr std::int32_t kMaximumProcessedManifoldContactPoints = 4;
    inline constexpr float kProcessedManifoldContactRetentionSeconds = 0.35f;
    inline constexpr float kFallbackContactRetentionDeltaSeconds = 1.0f / 90.0f;

    inline constexpr bool hasSolvedProcessedManifoldContact(
        const std::int32_t pointCount)
    {
        return pointCount > 0 &&
               pointCount <= kMaximumProcessedManifoldContactPoints;
    }

    inline float advanceProcessedManifoldContactRetention(
        const float currentSeconds,
        const bool positivePointWitness,
        const bool teleported,
        const float deltaSeconds)
    {
        if (teleported) {
            return 0.0f;
        }
        if (positivePointWitness) {
            return kProcessedManifoldContactRetentionSeconds;
        }

        const float retainedSeconds = std::isfinite(currentSeconds) ?
            (std::max)(0.0f, currentSeconds) :
            0.0f;
        const float elapsedSeconds =
            std::isfinite(deltaSeconds) && deltaSeconds > 0.000001f ?
            std::clamp(deltaSeconds, 0.0f, 0.1f) :
            kFallbackContactRetentionDeltaSeconds;
        return (std::max)(0.0f, retainedSeconds - elapsedSeconds);
    }

    inline float sanitizeWeaponMass(float weaponWeightGame)
    {
        if (!std::isfinite(weaponWeightGame) || weaponWeightGame <= 0.0f) {
            return kFallbackWeaponMass;
        }
        return std::clamp(weaponWeightGame, 0.1f, kMaximumWeaponMass);
    }

    struct BoundingBoxGeometry
    {
        RE::NiPoint3 centerWeaponLocal{};
        RE::NiPoint3 halfExtentsWeaponLocal{};
        bool valid{ false };
    };

    struct BoundingBoxMassProperties
    {
        RE::NiPoint3 halfExtentsHavok{};
        RE::NiPoint3 inverseInertia{};
        float inverseMass{ 0.0f };
        bool valid{ false };
    };

    struct CompoundChildFrame
    {
        RE::NiPoint3 translationHavok{};
        float pointScaleHavok{ 0.0f };
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

    inline BoundingBoxGeometry makeBoundingBoxGeometry(const RE::NiPoint3& boundsMin, const RE::NiPoint3& boundsMax)
    {
        BoundingBoxGeometry result{};
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
            (std::max)((boundsMax.x - boundsMin.x) * 0.5f, kMinimumBoundingBoxHalfExtentGameUnits),
            (std::max)((boundsMax.y - boundsMin.y) * 0.5f, kMinimumBoundingBoxHalfExtentGameUnits),
            (std::max)((boundsMax.z - boundsMin.z) * 0.5f, kMinimumBoundingBoxHalfExtentGameUnits),
        };
        result.valid = isFinitePoint(result.centerWeaponLocal) && isFinitePoint(result.halfExtentsWeaponLocal);
        return result;
    }

    inline RE::NiPoint3 makeBoundingBoxHalfExtentsHavok(
        const BoundingBoxGeometry& geometry,
        float weaponScale,
        float paddingGameUnits,
        float gameToHavokScale)
    {
        const float scale = std::abs(weaponScale);
        const float padding = (std::max)(0.0f, paddingGameUnits);
        return RE::NiPoint3{
            (geometry.halfExtentsWeaponLocal.x * scale + padding) * gameToHavokScale,
            (geometry.halfExtentsWeaponLocal.y * scale + padding) * gameToHavokScale,
            (geometry.halfExtentsWeaponLocal.z * scale + padding) * gameToHavokScale,
        };
    }

    inline BoundingBoxMassProperties makeBoundingBoxMassProperties(
        const BoundingBoxGeometry& geometry,
        float weaponScale,
        float paddingGameUnits,
        float gameToHavokScale,
        float mass)
    {
        BoundingBoxMassProperties result{};
        if (!geometry.valid || !std::isfinite(weaponScale) || std::abs(weaponScale) <= 0.0001f ||
            !std::isfinite(paddingGameUnits) || !std::isfinite(gameToHavokScale) || gameToHavokScale <= 0.0f ||
            !std::isfinite(mass) || mass <= 0.0f) {
            return result;
        }

        result.halfExtentsHavok = makeBoundingBoxHalfExtentsHavok(
            geometry,
            weaponScale,
            paddingGameUnits,
            gameToHavokScale);
        if (!isFinitePoint(result.halfExtentsHavok) ||
            result.halfExtentsHavok.x <= 0.0f ||
            result.halfExtentsHavok.y <= 0.0f ||
            result.halfExtentsHavok.z <= 0.0f) {
            return result;
        }

        // A solid box with half-extents h has principal moments
        // I_x = m/3 * (h_y^2 + h_z^2), and cyclic permutations.
        // hknp stores the inverse principal moments in the motion's local axes.
        const float massOverThree = mass / 3.0f;
        const float inertiaX = massOverThree *
                               (result.halfExtentsHavok.y * result.halfExtentsHavok.y +
                                   result.halfExtentsHavok.z * result.halfExtentsHavok.z);
        const float inertiaY = massOverThree *
                               (result.halfExtentsHavok.x * result.halfExtentsHavok.x +
                                   result.halfExtentsHavok.z * result.halfExtentsHavok.z);
        const float inertiaZ = massOverThree *
                               (result.halfExtentsHavok.x * result.halfExtentsHavok.x +
                                   result.halfExtentsHavok.y * result.halfExtentsHavok.y);
        if (!std::isfinite(inertiaX) || !std::isfinite(inertiaY) || !std::isfinite(inertiaZ) ||
            inertiaX <= 0.0f || inertiaY <= 0.0f || inertiaZ <= 0.0f) {
            return result;
        }

        result.inverseInertia = RE::NiPoint3{
            1.0f / inertiaX,
            1.0f / inertiaY,
            1.0f / inertiaZ,
        };
        result.inverseMass = 1.0f / mass;
        result.valid = isFinitePoint(result.inverseInertia) &&
                       result.inverseInertia.x > 0.0f &&
                       result.inverseInertia.y > 0.0f &&
                       result.inverseInertia.z > 0.0f &&
                       std::isfinite(result.inverseMass) &&
                       result.inverseMass > 0.0f;
        return result;
    }

    inline CompoundChildFrame makeCompoundChildFrame(
        const RE::NiPoint3& childCenterWeaponLocal,
        const RE::NiPoint3& aggregateCenterWeaponLocal,
        float weaponScale,
        float gameToHavokScale)
    {
        CompoundChildFrame result{};
        if (!isFinitePoint(childCenterWeaponLocal) || !isFinitePoint(aggregateCenterWeaponLocal) ||
            !std::isfinite(weaponScale) || std::abs(weaponScale) <= 0.0001f ||
            !std::isfinite(gameToHavokScale) || gameToHavokScale <= 0.0f) {
            return result;
        }

        result.pointScaleHavok = std::abs(weaponScale) * gameToHavokScale;
        result.translationHavok = RE::NiPoint3{
            (childCenterWeaponLocal.x - aggregateCenterWeaponLocal.x) * result.pointScaleHavok,
            (childCenterWeaponLocal.y - aggregateCenterWeaponLocal.y) * result.pointScaleHavok,
            (childCenterWeaponLocal.z - aggregateCenterWeaponLocal.z) * result.pointScaleHavok,
        };
        result.valid = isFinitePoint(result.translationHavok) &&
                       std::isfinite(result.pointScaleHavok) &&
                       result.pointScaleHavok > 0.0f;
        return result;
    }

    inline RE::NiPoint3 makeCompoundChildPointHavok(
        const RE::NiPoint3& pointWeaponLocal,
        const RE::NiPoint3& childCenterWeaponLocal,
        float pointScaleHavok)
    {
        return RE::NiPoint3{
            (pointWeaponLocal.x - childCenterWeaponLocal.x) * pointScaleHavok,
            (pointWeaponLocal.y - childCenterWeaponLocal.y) * pointScaleHavok,
            (pointWeaponLocal.z - childCenterWeaponLocal.z) * pointScaleHavok,
        };
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

    inline RE::NiTransform makeGripAuthorityTarget(const RE::NiTransform& weaponRootWorld)
    {
        RE::NiTransform result = weaponRootWorld;
        result.rotate = transform_math::transposeRotation(weaponRootWorld.rotate);
        result.scale = 1.0f;
        return result;
    }

    inline RE::NiTransform makeContactBodyTargetFromGripAuthority(
        const RE::NiTransform& gripAuthorityWorld,
        const RE::NiPoint3& centerWeaponLocal,
        float weaponScale)
    {
        RE::NiTransform weaponRootWorld = gripAuthorityWorld;
        weaponRootWorld.rotate = transform_math::transposeRotation(gripAuthorityWorld.rotate);
        weaponRootWorld.scale = weaponScale;
        RE::NiTransform result = makeProxyBodyTarget(weaponRootWorld, centerWeaponLocal);
        result.scale = 1.0f;
        return result;
    }

    inline RE::NiTransform makeContactBodyInGripAuthoritySpace(
        const RE::NiPoint3& centerWeaponLocal,
        float weaponScale)
    {
        RE::NiTransform result = transform_math::makeIdentityTransform<RE::NiTransform>();
        const float scale = std::abs(weaponScale);
        result.translate = RE::NiPoint3{
            centerWeaponLocal.x * scale,
            centerWeaponLocal.y * scale,
            centerWeaponLocal.z * scale,
        };
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
