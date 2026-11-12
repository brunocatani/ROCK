// Resolves the scope sight anchor, accepting generated scope geometry only
// while it belongs to the exact equipped weapon.
#pragma once

#include "physics-interaction/VectorMath.h"

#include <cmath>
#include <cstdint>

namespace rock::native_scope_sight_anchor_policy
{
    enum class AnchorSource : std::uint8_t
    {
        None,
        GeneratedSight,
        FiringGrip,
    };

    struct PublicationIdentity
    {
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t equippedWeaponOwnershipKey{ 0 };
        std::uint32_t weaponFormID{ 0 };
    };

    /*
     * Generated bodies can remain published while a replacement weapon is
     * staged. The reusable first-person Weapon node is therefore not an
     * ownership witness. Scope geometry is usable only when its body-set
     * publication still belongs to the exact equipped instance and form.
     */
    [[nodiscard]] inline constexpr bool matchesCurrentEquippedWeapon(const PublicationIdentity& published,
        const PublicationIdentity& current) noexcept
    {
        return published.weaponGenerationKey != 0 && published.equippedWeaponOwnershipKey != 0 &&
            published.weaponFormID != 0 && published.weaponGenerationKey == current.weaponGenerationKey &&
            published.equippedWeaponOwnershipKey == current.equippedWeaponOwnershipKey &&
            published.weaponFormID == current.weaponFormID;
    }

    template <class Point>
    struct ResolvedAnchor
    {
        Point weaponLocal{};
        AnchorSource source{ AnchorSource::None };
        bool valid{ false };
    };

    template <class Point>
    [[nodiscard]] inline bool isFinitePoint(const Point& point) noexcept
    {
        return vector_math::hasFiniteComponents(point);
    }

    /*
     * Generated Scope or Sight geometry places the anchor when the weapon has
     * it. Without it (unscoped weapons, and scope parts whose geometry is not
     * published) the anchor sits at the exact equipped firing-grip seat,
     * moved by a weapon-local offset that leaves the hand/weapon relation alone.
     *
     * Generated geometry remains authoritative whenever it is valid.
     */
    template <class Point>
    [[nodiscard]] inline ResolvedAnchor<Point> resolve(const bool generatedSightValid,
        const Point& generatedSightWeaponLocal, const bool firingGripValid, const Point& firingGripWeaponLocal,
        const Point& firingGripOffsetWeaponLocal) noexcept
    {
        if (generatedSightValid && isFinitePoint(generatedSightWeaponLocal)) {
            return ResolvedAnchor<Point>{
                .weaponLocal = generatedSightWeaponLocal,
                .source = AnchorSource::GeneratedSight,
                .valid = true,
            };
        }

        if (!firingGripValid || !isFinitePoint(firingGripWeaponLocal) || !isFinitePoint(firingGripOffsetWeaponLocal)) {
            return {};
        }

        const Point firingGripAnchorWeaponLocal{
            firingGripWeaponLocal.x + firingGripOffsetWeaponLocal.x,
            firingGripWeaponLocal.y + firingGripOffsetWeaponLocal.y,
            firingGripWeaponLocal.z + firingGripOffsetWeaponLocal.z,
        };
        if (!isFinitePoint(firingGripAnchorWeaponLocal)) {
            return {};
        }

        return ResolvedAnchor<Point>{
            .weaponLocal = firingGripAnchorWeaponLocal,
            .source = AnchorSource::FiringGrip,
            .valid = true,
        };
    }
}
