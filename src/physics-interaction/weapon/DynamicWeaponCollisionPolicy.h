#pragma once

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/VectorMath.h"

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace rock::dynamic_weapon_collision_policy
{
    inline constexpr bool kDynamicCompoundEnabled = true;
    inline constexpr float kInertiaEnvelopePaddingGameUnits = 0.5f;
    inline constexpr float kInverseInertiaMultiplier = 1.2f;
    inline constexpr float kMaximumLinearVelocityHavok = 15.0f;
    inline constexpr float kMaximumAngularVelocityRadiansPerSecond = 35.0f;
    inline constexpr float kDivergenceTeleportDistanceGameUnits = 80.0f;
    inline constexpr float kDivergenceTeleportDwellSeconds = 0.3f;
    inline constexpr float kMinimumBoundingBoxHalfExtentGameUnits = 0.25f;
    inline constexpr float kFallbackWeaponMass = 2.0f;
    inline constexpr float kMaximumWeaponMass = 50.0f;

    enum class VisualIntentSource
    {
        None,
        NativePhysicalHand,
        AuthoredPrimary,
        ManagedGrip,
    };

    inline const char* visualIntentSourceName(VisualIntentSource source) noexcept
    {
        switch (source) {
        case VisualIntentSource::NativePhysicalHand: return "native-physical-hand";
        case VisualIntentSource::AuthoredPrimary: return "authored-primary";
        case VisualIntentSource::ManagedGrip: return "managed-grip";
        default: return "none";
        }
    }

    struct DivergenceDwellResult
    {
        float elapsedSeconds{ 0.0f };
        bool recoverNow{ false };
    };

    [[nodiscard]] inline DivergenceDwellResult advanceDivergenceDwell(
        float previousElapsedSeconds,
        float requestedGapGameUnits,
        float measuredDeltaSeconds)
    {
        if (!std::isfinite(requestedGapGameUnits) ||
            requestedGapGameUnits <= kDivergenceTeleportDistanceGameUnits) {
            return {};
        }

        const float previous =
            std::isfinite(previousElapsedSeconds) && previousElapsedSeconds > 0.0f ?
            previousElapsedSeconds :
            0.0f;
        const float measuredDelta =
            std::isfinite(measuredDeltaSeconds) && measuredDeltaSeconds > 0.0f ?
            std::clamp(measuredDeltaSeconds, 0.0f, 0.1f) :
            0.0f;
        const float elapsed = previous + measuredDelta;
        return {
            .elapsedSeconds = elapsed,
            .recoverNow = elapsed >= kDivergenceTeleportDwellSeconds,
        };
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
        return vector_math::hasFiniteComponents(point);
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

    inline RE::NiTransform makeProxyBodyTarget(
        const RE::NiTransform& weaponRootWorld,
        const RE::NiPoint3& centerWeaponLocal)
    {
        RE::NiTransform result = weaponRootWorld;
        result.rotate = transform_math::transposeRotation(weaponRootWorld.rotate);
        result.translate = transform_math::localPointToWorld(weaponRootWorld, centerWeaponLocal);
        return result;
    }

    inline RE::NiTransform makeGripAuthorityTarget(const RE::NiTransform& weaponRootWorld,
        const RE::NiPoint3& pivotWeaponLocal = {})
    {
        RE::NiTransform result = weaponRootWorld;
        result.rotate = transform_math::transposeRotation(weaponRootWorld.rotate);
        result.translate = transform_math::localPointToWorld(weaponRootWorld, pivotWeaponLocal);
        result.scale = 1.0f;
        return result;
    }

    inline RE::NiTransform makeContactBodyTargetFromGripAuthority(
        const RE::NiTransform& gripAuthorityWorld,
        const RE::NiPoint3& centerWeaponLocal,
        float weaponScale,
        const RE::NiPoint3& pivotWeaponLocal = {})
    {
        RE::NiTransform weaponRootWorld = gripAuthorityWorld;
        weaponRootWorld.rotate = transform_math::transposeRotation(gripAuthorityWorld.rotate);
        weaponRootWorld.scale = weaponScale;
        const auto pivotOffset = transform_math::localVectorToWorld(weaponRootWorld, pivotWeaponLocal);
        weaponRootWorld.translate.x -= pivotOffset.x;
        weaponRootWorld.translate.y -= pivotOffset.y;
        weaponRootWorld.translate.z -= pivotOffset.z;
        RE::NiTransform result = makeProxyBodyTarget(weaponRootWorld, centerWeaponLocal);
        result.scale = 1.0f;
        return result;
    }

    inline RE::NiTransform makeContactBodyInGripAuthoritySpace(
        const RE::NiPoint3& centerWeaponLocal,
        float weaponScale,
        const RE::NiPoint3& pivotWeaponLocal = {})
    {
        RE::NiTransform result = transform_math::makeIdentityTransform<RE::NiTransform>();
        const float scale = std::abs(weaponScale);
        result.translate = RE::NiPoint3{
            (centerWeaponLocal.x - pivotWeaponLocal.x) * scale,
            (centerWeaponLocal.y - pivotWeaponLocal.y) * scale,
            (centerWeaponLocal.z - pivotWeaponLocal.z) * scale,
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

    // Preserve this frame's native local animation, but never take the world
    // pose of the rendered hand as the physical input. Intermediate animation
    // nodes are supported; missing or cyclic ancestry fails closed.
    template <class Node>
    inline bool reconstructNativeIntent(const Node* weapon, const Node* hand,
        const RE::NiTransform& physicalHandWorld, RE::NiTransform& result)
    {
        result = {};
        if (!weapon || !hand || weapon == hand || !isFiniteTransform(physicalHandWorld)) {
            return false;
        }
        auto local = transform_math::makeIdentityTransform<RE::NiTransform>();
        const Node* node = weapon;
        for (std::size_t depth = 0; node && node != hand && depth < 64; ++depth) {
            if (!isFiniteTransform(node->local)) {
                return false;
            }
            local = transform_math::composeTransforms(node->local, local);
            node = node->parent;
        }
        if (node != hand) {
            return false;
        }
        result = transform_math::composeTransforms(physicalHandWorld, local);
        return isFiniteTransform(result);
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
        // Carry the sampled physical deviation on the current target clock.
        // Applying the new movement through the deflected sample instead would
        // rotate locomotion translation by the weapon's collision deflection.
        const RE::NiTransform sampledResidual = transform_math::composeTransforms(
            transform_math::invertTransform(sampledRequestedWeaponWorld),
            sampledLiveWeaponWorld);
        return transform_math::composeTransforms(currentRequestedWeaponWorld, sampledResidual);
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

    struct GripRecoveryDecision
    {
        float distanceGameUnits{ 0.0f };
        bool resetNow{ false };
    };

    [[nodiscard]] inline GripRecoveryDecision evaluateGripRecovery(
        const RE::NiTransform& liveContactBodyWorld,
        const RE::NiTransform& requestedGripAuthorityWorld,
        const RE::NiPoint3& centerWeaponLocal,
        float weaponScale,
        float recoveryDistanceGameUnits,
        const RE::NiPoint3& pivotWeaponLocal = {})
    {
        if (!isFiniteTransform(liveContactBodyWorld) ||
            !isFiniteTransform(requestedGripAuthorityWorld) ||
            !isFinitePoint(centerWeaponLocal) ||
            !std::isfinite(weaponScale) ||
            std::abs(weaponScale) <= 0.0001f ||
            !std::isfinite(recoveryDistanceGameUnits) ||
            recoveryDistanceGameUnits <= 0.0f) {
            return {};
        }

        // Reconstruct the current constraint pivot (weapon root for free carry,
        // captured contact for surface support), so rotation around the body
        // center cannot hide a large separation at that pivot.
        const RE::NiTransform liveWeaponRootWorld = reconstructWeaponRoot(
            liveContactBodyWorld,
            centerWeaponLocal,
            weaponScale);
        auto livePivotWorld = liveWeaponRootWorld;
        livePivotWorld.translate = transform_math::localPointToWorld(liveWeaponRootWorld, pivotWeaponLocal);
        const float distanceGameUnits = translationDeltaGameUnits(
            livePivotWorld,
            requestedGripAuthorityWorld);
        if (!std::isfinite(distanceGameUnits)) {
            return {};
        }

        return {
            .distanceGameUnits = distanceGameUnits,
            .resetNow = distanceGameUnits > recoveryDistanceGameUnits,
        };
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

    struct VisualCorrectionDecision
    {
        bool apply = false;
        float translationGameUnits = 0.0f;
        float rotationDegrees = 0.0f;
    };

    inline VisualCorrectionDecision evaluateVisualCorrection(
        const RE::NiTransform& requested, const RE::NiTransform& solved)
    {
        if (!isFiniteTransform(requested) || !isFiniteTransform(solved)) {
            return {};
        }
        VisualCorrectionDecision result{};
        result.translationGameUnits = translationDeltaGameUnits(requested, solved);
        result.rotationDegrees = rotationDeltaDegrees(requested, solved);
        // Valid physics owns presentation continuously, including zero error.
        result.apply = std::isfinite(result.translationGameUnits) && std::isfinite(result.rotationDegrees);
        return result;
    }
}
