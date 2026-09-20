#pragma once

#include "physics-interaction/weapon/LooseWeaponAuthoredGrabPolicy.h"
#include "physics-interaction/TransformMath.h"
#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

namespace rock::weapon_grip_transfer
{
    [[nodiscard]] inline bool validFrame(const RE::NiTransform& value) noexcept
    {
        if (!std::isfinite(value.scale) || value.scale <= 0.0001f ||
            !std::isfinite(value.translate.x) || !std::isfinite(value.translate.y) || !std::isfinite(value.translate.z)) return false;
        for (const auto& row : value.rotate.entry)
            if (!std::isfinite(row.x) || !std::isfinite(row.y) || !std::isfinite(row.z)) return false;
        const auto& r = value.rotate.entry;
        const float determinant = r[0].x * (r[1].y * r[2].z - r[1].z * r[2].y) -
            r[0].y * (r[1].x * r[2].z - r[1].z * r[2].x) +
            r[0].z * (r[1].x * r[2].y - r[1].y * r[2].x);
        return std::isfinite(determinant) && std::abs(determinant) > 0.000001f;
    }

    // Value-only hand capture. Inventory pickup destroys the loose bodies;
    // only these weapon-relative relations cross into equipped ownership.
    struct HandGrip
    {
        RE::NiTransform handWeaponLocal{};
        RE::NiPoint3 gripWeaponLocal{};
        std::array<float, 15> fingerValues{};
        std::array<RE::NiTransform, 15> fingerLocals{};
        std::uint16_t fingerMask{ 0 };
        loose_weapon_authored_grab_policy::Role authoredRole{ loose_weapon_authored_grab_policy::Role::None };
        bool hasFingerPose{ false };

        [[nodiscard]] bool valid() const noexcept
        {
            if (!hasFingerPose || !validFrame(handWeaponLocal) || (fingerMask & ~0x7FFFu) ||
                !std::isfinite(gripWeaponLocal.x) || !std::isfinite(gripWeaponLocal.y) || !std::isfinite(gripWeaponLocal.z)) return false;
            for (std::size_t i = 0; i < fingerValues.size(); ++i) {
                if (!std::isfinite(fingerValues[i]) || ((fingerMask & (1u << i)) && !validFrame(fingerLocals[i]))) return false;
            }
            return true;
        }
    };

    [[nodiscard]] inline RE::NiTransform handInWeapon(const RE::NiTransform& bodyInWeapon,
        const RE::NiTransform& bodyInObject, const RE::NiTransform& objectInHand)
    {
        return transform_math::composeTransforms(
            transform_math::composeTransforms(bodyInWeapon, transform_math::invertTransform(bodyInObject)),
            transform_math::invertTransform(objectInHand));
    }

    struct Pair
    {
        HandGrip primary{};
        HandGrip support{};
        RE::NiPoint3 sourceModelTranslation{};
        std::uint32_t weaponFormID{ 0 };
        bool firingHandIsLeft{ false };
        loose_weapon_authored_grab_policy::Arrangement arrangement{ loose_weapon_authored_grab_policy::Arrangement::Pending };

        [[nodiscard]] bool valid() const noexcept
        {
            return weaponFormID != 0 && primary.valid() && support.valid() &&
                std::isfinite(sourceModelTranslation.x) && std::isfinite(sourceModelTranslation.y) && std::isfinite(sourceModelTranslation.z);
        }
    };

    // A lone support grip equips without acquiring the vacant firing station.
    struct Support
    {
        HandGrip grip{};
        // Capture placement separately from the authored visual wrist. The
        // controller may hold the loose weapon at any rotation on trigger-equip.
        std::optional<RE::NiTransform> weaponInDriver{};
        RE::NiPoint3 sourceModelTranslation{};
        std::uint32_t weaponFormID{ 0 };
        bool isLeft{ false };

        [[nodiscard]] bool valid() const noexcept
        {
            return weaponFormID != 0 && grip.valid() && weaponInDriver && validFrame(*weaponInDriver) && grip.fingerMask == 0x7FFFu &&
                grip.authoredRole == loose_weapon_authored_grab_policy::Role::Support &&
                std::isfinite(sourceModelTranslation.x) && std::isfinite(sourceModelTranslation.y) &&
                std::isfinite(sourceModelTranslation.z);
        }

        [[nodiscard]] RE::NiTransform registeredWeaponInDriver(const RE::NiPoint3& targetModelTranslation) const
        {
            auto registration = transform_math::makeIdentityTransform<RE::NiTransform>();
            registration.translate = sourceModelTranslation - targetModelTranslation;
            return transform_math::composeTransforms(*weaponInDriver, registration);
        }
    };

    // Match the loose shared-pivot owner: a firing station wins; otherwise the
    // first grab remains primary. Triggering with the support hand cannot swap it.
    [[nodiscard]] constexpr bool requesterIsPrimary(bool requesterFiring, bool peerFiring,
        std::uint64_t requesterGrab, std::uint64_t peerGrab) noexcept
    {
        return requesterFiring != peerFiring ? requesterFiring : requesterGrab < peerGrab;
    }
}
