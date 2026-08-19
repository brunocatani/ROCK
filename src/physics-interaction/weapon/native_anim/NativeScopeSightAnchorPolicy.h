#pragma once

#include <cmath>
#include <cstdint>

namespace rock::native_scope_sight_anchor_policy
{
    enum class AnchorSource : std::uint8_t
    {
        None,
        GeneratedSight,
        FiringGripFallback,
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
    [[nodiscard]] inline constexpr bool matchesCurrentEquippedWeapon(
        const PublicationIdentity& published,
        const PublicationIdentity& current) noexcept
    {
        return published.weaponGenerationKey != 0 &&
               published.equippedWeaponOwnershipKey != 0 &&
               published.weaponFormID != 0 &&
               published.weaponGenerationKey == current.weaponGenerationKey &&
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
        return std::isfinite(point.x) &&
               std::isfinite(point.y) &&
               std::isfinite(point.z);
    }

    /*
     * Some FO4 weapon attachments do not expose trustworthy generated optic
     * geometry. Keep the exact equipped firing-grip seat as a permanent
     * compatibility origin for those assets, with a weapon-local offset that
     * can be tuned without changing the stable hand/weapon relation.
     *
     * Generated geometry remains authoritative unless the user explicitly
     * forces the fallback for a collider that is present but semantically
     * wrong. A forced fallback with no current firing grip fails closed.
     */
    template <class Point>
    [[nodiscard]] inline ResolvedAnchor<Point> resolve(
        const bool forceFiringGripFallback,
        const bool generatedSightValid,
        const Point& generatedSightWeaponLocal,
        const bool firingGripValid,
        const Point& firingGripWeaponLocal,
        const Point& firingGripOffsetWeaponLocal) noexcept
    {
        if (!forceFiringGripFallback &&
            generatedSightValid &&
            isFinitePoint(generatedSightWeaponLocal)) {
            return ResolvedAnchor<Point>{
                .weaponLocal = generatedSightWeaponLocal,
                .source = AnchorSource::GeneratedSight,
                .valid = true,
            };
        }

        if (!firingGripValid ||
            !isFinitePoint(firingGripWeaponLocal) ||
            !isFinitePoint(firingGripOffsetWeaponLocal)) {
            return {};
        }

        const Point fallbackWeaponLocal{
            firingGripWeaponLocal.x + firingGripOffsetWeaponLocal.x,
            firingGripWeaponLocal.y + firingGripOffsetWeaponLocal.y,
            firingGripWeaponLocal.z + firingGripOffsetWeaponLocal.z,
        };
        if (!isFinitePoint(fallbackWeaponLocal)) {
            return {};
        }

        return ResolvedAnchor<Point>{
            .weaponLocal = fallbackWeaponLocal,
            .source = AnchorSource::FiringGripFallback,
            .valid = true,
        };
    }
}
