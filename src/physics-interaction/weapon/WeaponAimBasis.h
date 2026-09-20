#pragma once

#include "physics-interaction/TransformMath.h"
#include "RE/NetImmerse/NiTransform.h"

#include <cmath>

namespace rock::weapon_aim_basis
{
    inline constexpr float kMeleeBasePitchDegrees = 30.0f;

    // Controller-local pitch, shared by both physical hands. Zero adjustment
    // keeps the calibrated 30-degree forward tilt (+Y toward -Z). Every caller
    // seats the authored grip on the unchanged physical palm afterwards.
    [[nodiscard]] inline RE::NiTransform meleePitchInController(const float adjustmentDegrees) noexcept
    {
        auto result = transform_math::makeIdentityTransform<RE::NiTransform>();
        const float radians = (kMeleeBasePitchDegrees + adjustmentDegrees) * 0.017453292519943295769f;
        const float cosine = std::cos(radians);
        const float sine = std::sin(radians);
        result.rotate.entry[1][1] = result.rotate.entry[2][2] = cosine;
        result.rotate.entry[1][2] = -sine;
        result.rotate.entry[2][1] = sine;
        return result;
    }

    // ROCK's firearm controller-to-weapon basis. The native pre-FRIK driver samples
    // from 2026-09-17 agree on a 59-degree pitch across weapons and world
    // orientations. Grip translation and fingers come from the weapon's
    // authored animation, never from a provider offset or presented node.
    [[nodiscard]] inline RE::NiTransform weaponInController() noexcept
    {
        RE::NiTransform result{};
        result.rotate.entry[0][0] = 1.0f;
        result.rotate.entry[0][1] = result.rotate.entry[0][2] = 0.0f;
        result.rotate.entry[1][0] = result.rotate.entry[2][0] = 0.0f;
        result.rotate.entry[1][1] = result.rotate.entry[2][2] = 0.5150380749f;
        result.rotate.entry[1][2] = -0.8571673007f;
        result.rotate.entry[2][1] = 0.8571673007f;
        result.translate = {};
        result.scale = 1.0f;
        return result;
    }

    [[nodiscard]] inline bool tryResolveWorld(
        const RE::NiTransform& controllerWorld,
        const float weaponScale,
        RE::NiTransform& result) noexcept
    {
        result = {};
        if (!std::isfinite(weaponScale) || weaponScale <= 0.0f ||
            !std::isfinite(controllerWorld.scale) || controllerWorld.scale <= 0.0f ||
            !std::isfinite(controllerWorld.translate.x) ||
            !std::isfinite(controllerWorld.translate.y) ||
            !std::isfinite(controllerWorld.translate.z)) {
            return false;
        }
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(controllerWorld.rotate.entry[row][column])) {
                    return false;
                }
            }
        }
        result = transform_math::composeTransforms(controllerWorld, weaponInController());
        result.scale = weaponScale;
        return true;
    }

    // Melee models have no shared barrel axis. Preserve the authored wrist
    // relation by rotating the weapon onto the physical hand, rather than
    // bending the presented wrist onto the firearm aim. Palm seating remains
    // the caller's responsibility, including non-unit presentation scales.
    [[nodiscard]] inline bool tryResolveMeleeWorld(
        const RE::NiTransform& physicalHandWorld,
        const RE::NiTransform& authoredHandWeaponLocal,
        const float weaponScale,
        RE::NiTransform& result) noexcept
    {
        result = {};
        const auto usable = [](const RE::NiTransform& transform) {
            if (!std::isfinite(transform.scale) || transform.scale <= 0.000001f ||
                !std::isfinite(transform.translate.x) ||
                !std::isfinite(transform.translate.y) ||
                !std::isfinite(transform.translate.z)) return false;
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) return false;
                }
            }
            return true;
        };
        if (!std::isfinite(weaponScale) || weaponScale <= 0.0f ||
            !usable(physicalHandWorld) || !usable(authoredHandWeaponLocal)) return false;

        result = transform_math::composeTransforms(
            physicalHandWorld, transform_math::invertTransform(authoredHandWeaponLocal));
        result.translate = physicalHandWorld.translate;
        result.scale = weaponScale;
        return usable(result);
    }

    [[nodiscard]] inline bool tryResolveMeleeWorld(
        const RE::NiTransform& physicalHandWorld,
        const RE::NiTransform& authoredHandWeaponLocal,
        const RE::NiTransform& controllerWorld,
        const float weaponScale,
        const float pitchAdjustmentDegrees,
        RE::NiTransform& result) noexcept
    {
        if (!std::isfinite(pitchAdjustmentDegrees) ||
            !tryResolveMeleeWorld(physicalHandWorld, authoredHandWeaponLocal, weaponScale, result)) {
            result = {};
            return false;
        }
        if (pitchAdjustmentDegrees == -kMeleeBasePitchDegrees) return true;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(controllerWorld.rotate.entry[row][column])) {
                    result = {};
                    return false;
                }
            }
        }
        // Conjugate only rotation. Controller/wrist origins and scales must
        // not introduce a translation or become a different rotation pivot.
        auto controller = transform_math::makeIdentityTransform<RE::NiTransform>();
        controller.rotate = controllerWorld.rotate;
        auto aim = transform_math::makeIdentityTransform<RE::NiTransform>();
        aim.rotate = result.rotate;
        const auto inController = transform_math::composeTransforms(
            transform_math::invertTransform(controller), aim);
        result.rotate = transform_math::composeTransforms(controller,
            transform_math::composeTransforms(meleePitchInController(pitchAdjustmentDegrees), inController)).rotate;
        return true;
    }
}
