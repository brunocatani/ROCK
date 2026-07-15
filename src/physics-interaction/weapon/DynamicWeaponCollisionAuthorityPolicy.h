#pragma once

#include "physics-interaction/TransformMath.h"

#include <cmath>
#include <cstdint>

namespace rock::dynamic_weapon_collision_authority_policy
{
    struct HeldHandCouplingInput
    {
        bool weaponVisualAvailable = false;
        bool dynamicHandProxyEnabled = false;
        bool rightHandDisabled = false;
        bool leftHandDisabled = false;
        bool rightHandWeaponAuthorityActive = false;
        bool rightPartGripActive = false;
        bool leftFiringGripActive = false;
        bool leftSupportOrPartGripActive = false;
    };

    struct HeldHandCoupling
    {
        bool right = false;
        bool left = false;
    };

    /*
     * Role-driven and deliberately hand-symmetric. A native right firing grip,
     * a manual left firing grip, either support grip, and either part-carry
     * grip all move that physical hand's proxies into the weapon motion. A
     * missing weapon visual cannot publish fresh raw intent, so it never keeps
     * stale hand members coupled to a retained collision generation.
     */
    [[nodiscard]] inline HeldHandCoupling resolveHeldHandCoupling(const HeldHandCouplingInput& input)
    {
        if (!input.weaponVisualAvailable || !input.dynamicHandProxyEnabled) {
            return {};
        }
        return HeldHandCoupling{
            .right = !input.rightHandDisabled &&
                     (input.rightHandWeaponAuthorityActive || input.rightPartGripActive),
            .left = !input.leftHandDisabled &&
                    (input.leftFiringGripActive || input.leftSupportOrPartGripActive),
        };
    }

    inline constexpr std::uint64_t kContactSignalGraceSolves = 3;

    [[nodiscard]] inline bool hasRecentWorldContactSignal(
        std::uint64_t currentPostSolveSequence,
        std::uint64_t lastContactPostSolveSequence,
        std::uint64_t graceSolves = kContactSignalGraceSolves)
    {
        return lastContactPostSolveSequence != 0 &&
            currentPostSolveSequence >= lastContactPostSolveSequence &&
            currentPostSolveSequence - lastContactPostSolveSequence <= graceSolves;
    }

    struct ResidualSafetyLimits
    {
        float maxFreeTranslationGameUnits = 4.0f;
        float maxFreeRotationDegrees = 8.0f;
        float maxContactTranslationGameUnits = 96.0f;
        float maxContactRotationDegrees = 75.0f;
    };

    template <class Transform>
    [[nodiscard]] bool isFiniteTransform(const Transform& transform)
    {
        if (!std::isfinite(transform.translate.x) || !std::isfinite(transform.translate.y) ||
            !std::isfinite(transform.translate.z) || !std::isfinite(transform.scale) ||
            std::abs(transform.scale) <= 0.0001f) {
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

    [[nodiscard]] inline bool residualIsPlausible(
        bool hasRecentWorldContact,
        float translationResidualGameUnits,
        float rotationResidualDegrees,
        const ResidualSafetyLimits& limits = {})
    {
        if (!std::isfinite(translationResidualGameUnits) || !std::isfinite(rotationResidualDegrees)) {
            return false;
        }
        return hasRecentWorldContact ?
            translationResidualGameUnits <= limits.maxContactTranslationGameUnits &&
                rotationResidualDegrees <= limits.maxContactRotationDegrees :
            translationResidualGameUnits <= limits.maxFreeTranslationGameUnits &&
                rotationResidualDegrees <= limits.maxFreeRotationDegrees;
    }

    template <class Transform>
    [[nodiscard]] Transform reconstructWeaponRootFromAnchor(
        const Transform& anchorWorld,
        const Transform& anchorWeaponLocal)
    {
        return transform_math::composeTransforms(anchorWorld, transform_math::invertTransform(anchorWeaponLocal));
    }

    template <class Transform>
    [[nodiscard]] Transform applySampledAnchorCorrectionToCurrentIntent(
        const Transform& currentRequestedWeaponWorld,
        const Transform& sampledCommandedAnchorWorld,
        const Transform& sampledLiveAnchorWorld)
    {
        const Transform worldCorrection = transform_math::composeTransforms(
            sampledLiveAnchorWorld,
            transform_math::invertTransform(sampledCommandedAnchorWorld));
        return transform_math::composeTransforms(worldCorrection, currentRequestedWeaponWorld);
    }
}
