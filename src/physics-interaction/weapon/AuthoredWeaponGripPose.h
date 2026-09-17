#pragma once

#include "physics-interaction/weapon/LooseWeaponAuthoredGrabPolicy.h"
#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <cmath>
#include <cstdint>

namespace rock
{
    // Value-only pose carried across the equipped reference's inventory transfer.
    // The request/handle owns lifetime; no scene or provider pointer is retained.
    struct AuthoredWeaponGripPose
    {
        loose_weapon_authored_grab_policy::Role role{ loose_weapon_authored_grab_policy::Role::None };
        RE::NiTransform handWeaponLocal{};
        std::array<RE::NiTransform, 15> fingerLocals{};
        std::uint32_t weaponFormId{ 0 };
        std::uint16_t fingerMask{ 0 };
        bool isLeft{ false };

        [[nodiscard]] bool valid() const noexcept
        {
            const auto finite = [](const RE::NiTransform& transform) {
                if (!std::isfinite(transform.scale) || transform.scale <= 0.0001f ||
                    !std::isfinite(transform.translate.x) || !std::isfinite(transform.translate.y) ||
                    !std::isfinite(transform.translate.z)) return false;
                for (int row = 0; row < 3; ++row) {
                    for (int column = 0; column < 3; ++column) {
                        if (!std::isfinite(transform.rotate.entry[row][column])) return false;
                    }
                }
                const auto& r = transform.rotate.entry;
                const float determinant = r[0].x * (r[1].y * r[2].z - r[1].z * r[2].y) -
                    r[0].y * (r[1].x * r[2].z - r[1].z * r[2].x) +
                    r[0].z * (r[1].x * r[2].y - r[1].y * r[2].x);
                return std::isfinite(determinant) && std::abs(determinant) > 0.000001f;
            };
            if (role == loose_weapon_authored_grab_policy::Role::None || weaponFormId == 0 ||
                fingerMask != 0x7FFFu || !finite(handWeaponLocal)) return false;
            for (const auto& local : fingerLocals) if (!finite(local)) return false;
            return true;
        }
    };
}
