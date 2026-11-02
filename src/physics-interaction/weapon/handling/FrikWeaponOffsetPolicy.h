#pragma once

#include "physics-interaction/weapon/authored/HoldingGripMath.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>

/*
 * Player-made FRIK weapon offsets (FRIK_Config\Weapons_Offsets\*.json).
 *
 * FRIK ships compiled default offsets for many weapons and loads them on
 * every start; ROCK never reads those. Only the files a player saved from
 * FRIK's weapon reposition mode (or installed into that folder) are honoured,
 * and for a weapon that has one they replace ROCK's animation-authored firing
 * seat and scale. ROCK forces bLeftHandedMode:VR off, so the "-leftHanded"
 * variants never apply.
 *
 * FRIK's two modes that move the firing hand on the weapon are read:
 * - "<name>" is the Weapon node's local transform under RArm_Hand (the Weapon
 *   bone is a direct child of the hand), so the seat is its inverse and its
 *   scale is the weapon's scale relative to that hand.
 * - "<name>-primHand" is a rotation FRIK applies as hand.local = R * local in
 *   stored order after placing the weapon; with world = local * parent that is
 *   a rotation in the hand's own frame about the wrist, the weapon unmoved.
 * The "-offHand", "-throwable" and "-backOfHand" modes configure FRIK's own
 * two-handed grip and UI, which ROCK does not use. Each mode is looked up as
 * FRIK does: the "-PowerArmor" key first in power armor, then the plain key.
 *
 * Matrices are holding_grip_math::Pose (column-vector, r[row * 3 + column]);
 * FRIK's JSON stores the engine's NiMatrix3 rows with a fourth padding column,
 * which is that matrix transposed.
 */
namespace rock::frik_weapon_offset_policy
{
    using holding_grip_math::Pose;

    inline constexpr std::string_view kPrimaryHandSuffix = "-primHand";
    inline constexpr std::string_view kPowerArmorSuffix = "-PowerArmor";

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
     * One FRIK offset entry. Throws on a malformed entry, so a broken file is
     * skipped whole, as FRIK itself rejects it.
     */
    [[nodiscard]] inline Pose readEntry(const nlohmann::json& value)
    {
        const auto& rotation = value.at("rotation");
        if (!rotation.is_array() || rotation.size() != 12) {
            throw std::runtime_error("rotation needs 12 values");
        }
        Pose pose;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                pose.r[column * 3 + row] = rotation.at(static_cast<std::size_t>(row * 4 + column)).get<float>();
            }
        }
        pose.t = { value.at("x").get<float>(), value.at("y").get<float>(), value.at("z").get<float>() };
        pose.s = value.at("scale").get<float>();
        if (!std::isfinite(pose.s) || pose.s < 0.05f || pose.s > 20.0f) {
            throw std::runtime_error("scale out of range");
        }
        for (const float t : pose.t) {
            if (!std::isfinite(t) || std::abs(t) > 1000.0f) {
                throw std::runtime_error("translation out of range");
            }
        }
        const auto proper = orthonormalized(pose);
        if (!proper) {
            throw std::runtime_error("rotation is not a proper rotation");
        }
        return *proper;
    }

    using EntryMap = std::unordered_map<std::string, Pose>;

    // Every entry of one file, keyed by its JSON key (FRIK's authority; the
    // file name is only a sanitized copy of it).
    [[nodiscard]] inline EntryMap readFile(const nlohmann::json& json)
    {
        if (!json.is_object()) {
            throw std::runtime_error("expected an object of named offsets");
        }
        EntryMap entries;
        for (const auto& [key, value] : json.items()) {
            entries.insert_or_assign(key, readEntry(value));
        }
        return entries;
    }

    [[nodiscard]] inline const Pose* lookup(const EntryMap& entries, const std::string& key, const bool inPowerArmor)
    {
        if (inPowerArmor) {
            if (const auto it = entries.find(key + std::string{ kPowerArmorSuffix }); it != entries.end()) {
                return &it->second;
            }
        }
        const auto it = entries.find(key);
        return it != entries.end() ? &it->second : nullptr;
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

    [[nodiscard]] inline std::optional<Override> resolve(const EntryMap& entries, const std::string& key,
        const bool inPowerArmor)
    {
        Override result{};
        if (const auto* weapon = lookup(entries, key, inPowerArmor)) {
            result.weaponInHand = *weapon;
            result.hasWeaponOffset = true;
        }
        if (const auto* hand = lookup(entries, key + std::string{ kPrimaryHandSuffix }, inPowerArmor)) {
            result.primaryHandRotation = *hand;
            result.hasPrimaryHandRotation = true;
        }
        if (!result.hasWeaponOffset && !result.hasPrimaryHandRotation) {
            return std::nullopt;
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
