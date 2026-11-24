// Turns FRIK's weapon offsets for a weapon into a firing seat and scale.
#pragma once

#include "physics-interaction/weapon/authored/HoldingGripMath.h"

#include <cmath>
#include <optional>
#include <string>
#include <string_view>

/*
 * FRIK's weapon offsets, read through FRIK's API (getWeaponOffset): the
 * offsets FRIK ships with and the ones a player saved in FRIK's weapon
 * adjustment, a saved one winning. For a weapon that has one they replace
 * ROCK's animation-authored firing seat and scale. FRIK resolves the power
 * armor and left-handed variants; ROCK forces bLeftHandedMode:VR off.
 *
 * FRIK's two modes that move the firing hand on the weapon are read:
 * - Weapon is the Weapon node's local transform under RArm_Hand (the Weapon
 *   bone is a direct child of the hand), so the seat is its inverse and its
 *   scale is the weapon's scale relative to that hand.
 * - PrimaryHand is a rotation FRIK applies as hand.local = R * local in
 *   stored order after placing the weapon; with world = local * parent that is
 *   a rotation in the hand's own frame about the wrist, the weapon unmoved.
 * The OffHand, Throwable and BackOfHandUI modes configure FRIK's own two-handed
 * grip and UI, which ROCK does not use.
 *
 * Matrices are holding_grip_math::Pose (column-vector, r[row * 3 + column]),
 * holding_grip_math::fromNative of FRIK's NiTransform.
 */
namespace rock::frik_weapon_offset_policy
{
    using holding_grip_math::Pose;


    /*
     * The weapons FRIK names pistol or rifle by their stock; only these need
     * the model's grip stock name.
     */
    [[nodiscard]] inline bool namedByStock(const std::string_view name)
    {
        return name == "Plasma" || name == "Pipe" || name == "Pipe Bolt-Action" || name == "Laser" ||
            name == "Institute";
    }

    /*
     * FRIK's weapon key: the base form's display name, plus " Rifle" for the
     * weapons that are pistol or rifle by stock (F4VR-CommonFramework
     * EquippedWeaponHandler::getExtendedWeaponName). gripStockName is the name
     * of the first child of the model's P-Grip node.
     */
    [[nodiscard]] inline std::string weaponKey(const std::string_view name, const std::string_view gripStockName)
    {
        std::string key{ name };
        bool rifle = false;
        if (name == "Plasma") {
            rifle = gripStockName.starts_with("RiotGrip") || gripStockName.starts_with("Sniper") ||
                gripStockName.find("Rifle") != std::string_view::npos;
        } else if (name == "Pipe" || name == "Pipe Bolt-Action") {
            rifle = gripStockName.starts_with("HandmadePaddedStock") || gripStockName.starts_with("SpringStock") ||
                gripStockName.starts_with("PipeStock");
        } else if (name == "Laser" || name == "Institute") {
            rifle = gripStockName.find("Rifle") != std::string_view::npos;
        }
        if (rifle) {
            key += " Rifle";
        }
        return key;
    }

    // Rows to a proper rotation. FRIK files are often hand-edited to three
    // decimals; a mirrored or degenerate matrix is not a hand placement.
    [[nodiscard]] inline std::optional<Pose> orthonormalized(Pose pose)
    {
        auto column = [&](int c) { return holding_grip_math::V{ pose.r[c], pose.r[3 + c], pose.r[6 + c] }; };
        auto dot = [](const holding_grip_math::V& a, const holding_grip_math::V& b) {
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        };
        auto normalized = [&](holding_grip_math::V v) -> std::optional<holding_grip_math::V> {
            const float length = std::sqrt(dot(v, v));
            if (!std::isfinite(length) || length < 0.5f) {
                return std::nullopt;
            }
            for (auto& x : v) {
                x /= length;
            }
            return v;
        };
        const auto x = normalized(column(0));
        if (!x) {
            return std::nullopt;
        }
        auto yRaw = column(1);
        const float xy = dot(*x, yRaw);
        for (int i = 0; i < 3; ++i) {
            yRaw[i] -= xy * (*x)[i];
        }
        const auto y = normalized(yRaw);
        if (!y) {
            return std::nullopt;
        }
        const holding_grip_math::V z{ (*x)[1] * (*y)[2] - (*x)[2] * (*y)[1], (*x)[2] * (*y)[0] - (*x)[0] * (*y)[2],
            (*x)[0] * (*y)[1] - (*x)[1] * (*y)[0] };
        if (dot(z, column(2)) <= 0.0f) {
            return std::nullopt;
        }
        for (int row = 0; row < 3; ++row) {
            pose.r[row * 3 + 0] = (*x)[row];
            pose.r[row * 3 + 1] = (*y)[row];
            pose.r[row * 3 + 2] = z[row];
        }
        return holding_grip_math::valid(pose) ? std::optional<Pose>{ pose } : std::nullopt;
    }

    /*
     * One FRIK offset as a hand placement, or none. Offsets are often hand-edited
     * to three decimals, so the rotation is re-orthonormalized; a mirrored or
     * degenerate rotation, or a scale or translation no hand placement has, is
     * refused.
     */
    [[nodiscard]] inline std::optional<Pose> validated(const Pose& pose)
    {
        if (!std::isfinite(pose.s) || pose.s < 0.05f || pose.s > 20.0f) {
            return std::nullopt;
        }
        for (const float t : pose.t) {
            if (!std::isfinite(t) || std::abs(t) > 1000.0f) {
                return std::nullopt;
            }
        }
        return orthonormalized(pose);
    }

    struct Override
    {
        // FRIK's Weapon node local under RArm_Hand.
        Pose weaponInHand{};
        // FRIK's primary-hand rotation; only its rotation is used.
        Pose primaryHandRotation{};
        bool hasWeaponOffset{ false };
        bool hasPrimaryHandRotation{ false };

        // The Weapon node's scale relative to its hand.
        [[nodiscard]] float weaponScale() const noexcept { return hasWeaponOffset ? weaponInHand.s : 1.0f; }

        // A seat needs the native one only to apply a lone hand rotation to.
        [[nodiscard]] bool replacesSeat() const noexcept { return hasWeaponOffset; }
    };

    /*
     * The override of a weapon's two offsets, or none when it has neither.
     */
    [[nodiscard]] inline std::optional<Override> resolve(const std::optional<Pose>& weaponInHand,
        const std::optional<Pose>& primaryHandRotation)
    {
        if (!weaponInHand && !primaryHandRotation) {
            return std::nullopt;
        }
        Override result{};
        if (weaponInHand) {
            result.weaponInHand = *weaponInHand;
            result.hasWeaponOffset = true;
        }
        if (primaryHandRotation) {
            result.primaryHandRotation = *primaryHandRotation;
            result.hasPrimaryHandRotation = true;
        }
        return result;
    }

    /*
     * RArm_Hand in Weapon for this override. Without a Weapon offset FRIK keeps
     * the animation's Weapon local, which is ROCK's native seat.
     */
    [[nodiscard]] inline Pose seat(const Override& override, const Pose& nativeHandInWeapon)
    {
        Pose result = override.hasWeaponOffset ? holding_grip_math::inverse(override.weaponInHand) : nativeHandInWeapon;
        if (override.hasPrimaryHandRotation) {
            Pose rotation{};
            rotation.r = override.primaryHandRotation.r;
            result = holding_grip_math::compose(result, rotation);
        }
        return result;
    }
}
