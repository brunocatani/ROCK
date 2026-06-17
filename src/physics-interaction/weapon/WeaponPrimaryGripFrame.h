#pragma once

#include "physics-interaction/TransformMath.h"

#include "RE/NetImmerse/NiTransform.h"

#include <cmath>
#include <cstring>

namespace rock::weapon_primary_grip_frame_policy
{
    inline constexpr float kDefaultReattachEnterRadiusGameUnits = 4.0f;
    inline constexpr float kDefaultReattachExitRadiusGameUnits = 6.0f;

    enum class PrimaryGripFrameSource
    {
        None = 0,
        FrikWeaponOffset = 1,
        WeaponNodeLocalBaseline = 2,
    };

    struct PrimaryGripFrameResult
    {
        bool valid{ false };
        RE::NiTransform weaponLocalFrame{};
        PrimaryGripFrameSource source{ PrimaryGripFrameSource::None };
        const char* reason{ "missing" };
    };

    [[nodiscard]] inline const char* primaryGripFrameSourceName(PrimaryGripFrameSource source) noexcept
    {
        switch (source) {
        case PrimaryGripFrameSource::FrikWeaponOffset:
            return "frik-offset";
        case PrimaryGripFrameSource::WeaponNodeLocalBaseline:
            return "weapon-node-local-baseline";
        case PrimaryGripFrameSource::None:
        default:
            return "none";
        }
    }

    [[nodiscard]] inline bool isFiniteRotation(const RE::NiMatrix3& rotation) noexcept
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(rotation.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    [[nodiscard]] inline bool isFiniteTransform(const RE::NiTransform& transform) noexcept
    {
        return isFiniteRotation(transform.rotate) &&
               std::isfinite(transform.translate.x) &&
               std::isfinite(transform.translate.y) &&
               std::isfinite(transform.translate.z) &&
               std::isfinite(transform.scale) &&
               std::abs(transform.scale) > 0.0001f;
    }

    [[nodiscard]] inline bool isExplicitFrikWeaponOffsetReason(const char* reason) noexcept
    {
        return reason &&
               (std::strcmp(reason, "offset") == 0 ||
                   std::strcmp(reason, "powerArmorOffset") == 0);
    }

    [[nodiscard]] inline PrimaryGripFrameResult primaryGripFrameFromWeaponOffset(
        const RE::NiTransform& weaponOffsetLocal,
        PrimaryGripFrameSource source,
        const char* reason) noexcept
    {
        if (!isFiniteTransform(weaponOffsetLocal)) {
            return PrimaryGripFrameResult{ .valid = false, .source = PrimaryGripFrameSource::None, .reason = "invalid-offset" };
        }

        const RE::NiTransform gripFrameWeaponLocal = transform_math::invertTransform(weaponOffsetLocal);
        if (!isFiniteTransform(gripFrameWeaponLocal)) {
            return PrimaryGripFrameResult{ .valid = false, .source = PrimaryGripFrameSource::None, .reason = "invalid-inverse" };
        }

        return PrimaryGripFrameResult{
            .valid = true,
            .weaponLocalFrame = gripFrameWeaponLocal,
            .source = source,
            .reason = reason ? reason : primaryGripFrameSourceName(source),
        };
    }

    enum class DetachedPrimaryGripRoute
    {
        FreeHand = 0,
        ReattachCandidate = 1,
        ReattachNow = 2,
    };

    struct DetachedPrimaryReattachInput
    {
        bool primaryDetached{ false };
        bool weaponGenerationCurrent{ false };
        bool firingGripResolved{ false };
        bool primaryHandHasNormalGrabOwner{ false };
        bool supportHandStillOwnsWeapon{ false };
        bool primaryGripHeld{ false };
        bool primaryGripPressed{ false };
        float distanceToFiringGripGameUnits{ 0.0f };
        float reattachEnterRadiusGameUnits{ kDefaultReattachEnterRadiusGameUnits };
        float reattachExitRadiusGameUnits{ kDefaultReattachExitRadiusGameUnits };
    };

    [[nodiscard]] inline float sanitizeReattachRadius(float value, float fallback) noexcept
    {
        if (!std::isfinite(value) || value <= 0.0f) {
            return fallback;
        }
        return value;
    }

    [[nodiscard]] inline DetachedPrimaryGripRoute resolveDetachedPrimaryGripRoute(const DetachedPrimaryReattachInput& input) noexcept
    {
        if (!input.primaryDetached ||
            !input.weaponGenerationCurrent ||
            !input.firingGripResolved ||
            !input.supportHandStillOwnsWeapon ||
            input.primaryHandHasNormalGrabOwner ||
            !std::isfinite(input.distanceToFiringGripGameUnits)) {
            return DetachedPrimaryGripRoute::FreeHand;
        }

        const float enterRadius = sanitizeReattachRadius(input.reattachEnterRadiusGameUnits, kDefaultReattachEnterRadiusGameUnits);
        const float sanitizedExitRadius = sanitizeReattachRadius(input.reattachExitRadiusGameUnits, kDefaultReattachExitRadiusGameUnits);
        const float exitRadius = sanitizedExitRadius >= enterRadius ? sanitizedExitRadius : enterRadius;
        if (input.distanceToFiringGripGameUnits > exitRadius) {
            return DetachedPrimaryGripRoute::FreeHand;
        }

        if (input.distanceToFiringGripGameUnits <= enterRadius && (input.primaryGripHeld || input.primaryGripPressed)) {
            return DetachedPrimaryGripRoute::ReattachNow;
        }

        return DetachedPrimaryGripRoute::ReattachCandidate;
    }
}
