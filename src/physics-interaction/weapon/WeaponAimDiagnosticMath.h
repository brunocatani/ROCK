#pragma once

#include "physics-interaction/TransformMath.h"
#include "RE/NetImmerse/NiTransform.h"

#include <algorithm>
#include <cmath>

namespace rock::weapon_aim_diagnostic
{
    struct Sample
    {
        RE::NiPoint3 handForward{}, barrelForward{}, barrelInController{};
        RE::NiPoint3 barrelInHand{};
        RE::NiMatrix3 handInControllerRotation{};
        float yawDegrees{ 0.0f }, pitchDegrees{ 0.0f }, divergenceDegrees{ 0.0f };
        bool valid{ false };
    };

    inline bool finiteFrame(const RE::NiTransform& frame) noexcept
    {
        if (!std::isfinite(frame.scale) || frame.scale <= 0.0001f ||
            !std::isfinite(frame.translate.x) || !std::isfinite(frame.translate.y) || !std::isfinite(frame.translate.z)) return false;
        for (const auto& row : frame.rotate.entry)
            if (!std::isfinite(row.x) || !std::isfinite(row.y) || !std::isfinite(row.z)) return false;
        return true;
    }

    inline bool normalize(RE::NiPoint3& direction) noexcept
    {
        const float length = std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
        if (!std::isfinite(length) || length <= 0.0001f) return false;
        direction *= 1.0f / length;
        return true;
    }

    // The visible reference is the isolated physical hand's finger-forward +X,
    // never the wrist already posed to the weapon. Numeric yaw/pitch remain in
    // the game wand basis used by mirroring (+X lateral, +Y reference, +Z up).
    // No assumption is made that a device's physical forward equals wand +Y.
    inline Sample measure(const RE::NiTransform& controller, const RE::NiTransform& barrel,
        const RE::NiTransform& physicalHand) noexcept
    {
        Sample result{};
        if (!finiteFrame(controller) || !finiteFrame(barrel) || !finiteFrame(physicalHand)) return result;
        const RE::NiPoint3 forward{ 0.0f, 1.0f, 0.0f };
        result.handForward = transform_math::localVectorToWorld(physicalHand, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
        result.barrelForward = transform_math::localVectorToWorld(barrel, forward);
        if (!normalize(result.handForward) || !normalize(result.barrelForward)) return result;
        result.barrelInController = transform_math::localVectorToWorld(
            transform_math::invertTransform(controller), result.barrelForward);
        if (!normalize(result.barrelInController)) return result;
        // The total hand/barrel angle alone cannot distinguish palm-depth
        // tilt from lateral tilt. Preserve the signed direction and the basis
        // needed to map an anatomical correction back into controller space.
        result.barrelInHand = transform_math::localVectorToWorld(
            transform_math::invertTransform(physicalHand), result.barrelForward);
        if (!normalize(result.barrelInHand)) return result;
        result.handInControllerRotation = transform_math::composeTransforms(
            transform_math::invertTransform(controller), physicalHand).rotate;
        constexpr float degrees = 57.2957795131f;
        const auto& direction = result.barrelInController;
        result.yawDegrees = std::atan2(direction.x, direction.y) * degrees;
        result.pitchDegrees = std::atan2(direction.z, std::hypot(direction.x, direction.y)) * degrees;
        const float handDotBarrel = result.handForward.x * result.barrelForward.x +
            result.handForward.y * result.barrelForward.y + result.handForward.z * result.barrelForward.z;
        result.divergenceDegrees = std::acos(std::clamp(handDotBarrel, -1.0f, 1.0f)) * degrees;
        result.valid = true;
        return result;
    }
}
