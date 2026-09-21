#pragma once

#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"

#include <string_view>

namespace rock::blade_penetration
{
    inline constexpr float kHandClearanceMeters = 0.03f;
    inline constexpr float kTipContactRadiusGame = 1.25f;
    inline constexpr float kEntryPressureGame = 0.5f;
    inline constexpr float kWithdrawalClearanceGame = 2.0f;

    enum class Family { None, Switchblade, Knife, Machete, ChineseSword, RevolutionarySword, Shishkebab };

    inline Family family(std::uint32_t formId)
    {
        // Fallout4.esm WEAP identities. OneHandSword also includes blunt weapons.
        switch (formId) {
        case 0x000FDC81u: return Family::Switchblade;
        case 0x000913CAu:
        case 0x00062AA3u: return Family::Knife;
        case 0x00033FE0u: return Family::Machete;
        case 0x00147BE4u: return Family::ChineseSword;
        case 0x00143AB5u: return Family::RevolutionarySword;
        case 0x000FA2FBu: return Family::Shishkebab;
        default: return Family::None;
        }
    }

    struct SourceProfile
    {
        Family family;
        std::string_view name;
        RE::NiPoint3 minimum;
        RE::NiPoint3 maximum;
    };

    // nif_mcp measurements of the physical shapes in Fallout4 - Meshes.ba2,
    // Weapons/{Switchblade,Knife,Machete,ChineseOfficerSword,Swords,Shishkebab}.
    // These shapes point along source +Y. Live source transforms retain part
    // offsets and scale (notably the revolutionary sword's 0.799009 scale).
    inline const SourceProfile kSources[] = {
        { Family::Switchblade, "Blade:1", { -0.881348f, -0.479248f, -0.030731f }, { 0.852539f, 11.382813f, 0.113831f } },
        { Family::Switchblade, "SerratedBlade:0", { -0.874512f, -0.367188f, -0.050018f }, { 0.852539f, 11.296875f, 0.135864f } },
        { Family::Knife, "Blade:0", { -1.330078f, -0.484131f, -0.169189f }, { 1.192383f, 14.289063f, 0.281738f } },
        { Family::Knife, "Serration:0", { -1.316406f, -0.490479f, -0.225464f }, { 1.206055f, 14.281250f, 0.225464f } },
        { Family::Knife, "Serration006:0", { -1.316406f, -0.490479f, -0.225464f }, { 1.206055f, 14.281250f, 0.225464f } },
        { Family::Machete, "Blade:1", { -3.617188f, -2.595703f, -0.347900f }, { 2.322266f, 37.125000f, 0.347168f } },
        { Family::Machete, "SerratedBlade:0", { -3.605469f, -2.740234f, -0.348877f }, { 2.351563f, 37.093750f, 0.348389f } },
        { Family::Machete, "SacrificalBlade:0", { -3.751953f, -2.843750f, -0.411621f }, { 4.804688f, 39.531250f, 0.411621f } },
        { Family::ChineseSword, "Blade:0", { -1.362305f, -2.324219f, -0.320557f }, { 1.363281f, 54.593750f, 0.402100f } },
        { Family::ChineseSword, "SerratedBlade:0", { -1.558594f, -2.324219f, -0.320557f }, { 1.559570f, 54.593750f, 0.402344f } },
        { Family::ChineseSword, "SerratedBlade001:2", { -1.452148f, -2.324219f, -0.320557f }, { 1.274414f, 54.593750f, 0.402100f } },
        { Family::ChineseSword, "SerratedBladeShock:0", { -1.558594f, -2.324219f, -0.320557f }, { 1.559570f, 54.593750f, 0.402344f } },
        { Family::RevolutionarySword, "Blade:0", { -1.570313f, -0.590820f, 0.000000f }, { 1.570313f, 58.156250f, 0.928711f } },
        { Family::RevolutionarySword, "SerratedBlade:0", { -1.570313f, -0.590820f, 0.000000f }, { 1.570313f, 58.156250f, 0.928711f } },
        { Family::RevolutionarySword, "Blade003:0", { -5.859375f, -7.214844f, -0.933105f }, { 4.593750f, 52.656250f, 1.674805f } },
        { Family::Shishkebab, "Shishkebab_02:0", { -4.730469f, -12.812500f, -3.660156f }, { 7.269531f, 55.562500f, 3.580078f } },
    };

    inline const SourceProfile* sourceProfile(Family weaponFamily, std::string_view name)
    {
        for (const auto& source : kSources) {
            if (source.family == weaponFamily && source.name == name) return &source;
        }
        return nullptr;
    }

    inline bool matchesSourceBounds(const SourceProfile& source, const RE::NiPoint3& minimum,
        const RE::NiPoint3& maximum)
    {
        // First/third person tessellation differs by at most 0.053 gu here.
        const auto close = [](const RE::NiPoint3& a, const RE::NiPoint3& b) {
            return dynamic_weapon_collision_policy::isFinitePoint(a) &&
                std::abs(a.x - b.x) <= 0.06f && std::abs(a.y - b.y) <= 0.06f && std::abs(a.z - b.z) <= 0.06f;
        };
        return close(minimum, source.minimum) && close(maximum, source.maximum);
    }

    struct Blade
    {
        RE::NiPoint3 tipLocal{};
        RE::NiPoint3 axisLocal{};
        bool valid{ false };
    };

    inline RE::NiTransform weaponFromBody(const RE::NiTransform& bodyWorld,
        const RE::NiPoint3& centerWeaponLocal, float weaponScale)
    {
        // This input is the BODY array, not the MOTION quaternion adapter used
        // by DynamicWeaponCollision's general sample. BODY axes are already
        // stored in the same row convention as localPointToWorld.
        RE::NiTransform result = bodyWorld;
        result.scale = weaponScale;
        const auto centerOffset = transform_math::localVectorToWorld(result, centerWeaponLocal);
        result.translate.x -= centerOffset.x;
        result.translate.y -= centerOffset.y;
        result.translate.z -= centerOffset.z;
        return result;
    }

    inline float dot(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline RE::NiPoint3 difference(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        return { a.x - b.x, a.y - b.y, a.z - b.z };
    }

    inline bool normalized(const RE::NiPoint3& value, RE::NiPoint3& result)
    {
        const float lengthSquared = dot(value, value);
        if (!std::isfinite(lengthSquared) || lengthSquared < 0.000001f) return false;
        const float inverseLength = 1.0f / std::sqrt(lengthSquared);
        result = { value.x * inverseLength, value.y * inverseLength, value.z * inverseLength };
        return true;
    }

    inline bool axisWorld(const Blade& blade, const RE::NiTransform& weapon, RE::NiPoint3& result)
    {
        return blade.valid && dynamic_weapon_collision_policy::isFiniteTransform(weapon) &&
            normalized(transform_math::localVectorToWorld(weapon, blade.axisLocal), result);
    }

    inline float maximumDepth(const Blade& blade, const RE::NiPoint3& gripLocal,
        float weaponScale, float gameUnitsPerMeter)
    {
        RE::NiPoint3 axis{};
        if (!blade.valid || !dynamic_weapon_collision_policy::isFinitePoint(blade.tipLocal) ||
            !dynamic_weapon_collision_policy::isFinitePoint(gripLocal) ||
            !std::isfinite(weaponScale) || weaponScale <= 0.0f ||
            !std::isfinite(gameUnitsPerMeter) || gameUnitsPerMeter <= 0.0f ||
            !normalized(blade.axisLocal, axis)) return 0.0f;
        const float depth = dot(difference(blade.tipLocal, gripLocal), axis) * weaponScale -
            kHandClearanceMeters * gameUnitsPerMeter;
        return std::isfinite(depth) ? (std::max)(0.0f, depth) : 0.0f;
    }

    struct Entry
    {
        float tipDistance{ 0.0f };
        float pressure{ 0.0f };
        float lateralError{ 0.0f };
        bool accepted{ false };
    };

    inline Entry evaluateEntry(const Blade& blade, const RE::NiTransform& physicalWeapon,
        const RE::NiTransform& requestedWeapon, const RE::NiPoint3& contact)
    {
        Entry result{};
        RE::NiPoint3 axis{}, requestedAxis{};
        if (!axisWorld(blade, physicalWeapon, axis) || !axisWorld(blade, requestedWeapon, requestedAxis) ||
            !dynamic_weapon_collision_policy::isFinitePoint(contact)) return result;
        const auto tip = transform_math::localPointToWorld(physicalWeapon, blade.tipLocal);
        const auto requestedTip = transform_math::localPointToWorld(requestedWeapon, blade.tipLocal);
        const auto contactError = difference(tip, contact);
        const auto error = difference(requestedTip, tip);
        result.tipDistance = std::sqrt(dot(contactError, contactError));
        result.pressure = dot(error, axis);
        result.lateralError = std::sqrt((std::max)(0.0f, dot(error, error) - result.pressure * result.pressure));
        result.accepted = std::isfinite(result.tipDistance) && std::isfinite(result.pressure) &&
            std::isfinite(result.lateralError) && result.tipDistance <= kTipContactRadiusGame &&
            result.pressure >= kEntryPressureGame && result.lateralError <= kTipContactRadiusGame &&
            dot(axis, requestedAxis) >= 0.95f;
        return result;
    }

    struct Guide
    {
        RE::NiTransform weaponWorld{};
        float requestedDepth{ 0.0f };
        bool valid{ false };
    };

    // The captured orientation and entry point belong to the struck BODY,
    // so turning the controller cannot redirect the blade inside the actor.
    inline Guide guide(const Blade& blade, const RE::NiTransform& entryWeaponInTarget,
        const RE::NiTransform& targetWorld, const RE::NiTransform& requestedWeapon, float maximumDepthGame)
    {
        Guide result{};
        if (!std::isfinite(maximumDepthGame) || maximumDepthGame <= 0.0f ||
            !dynamic_weapon_collision_policy::isFiniteTransform(entryWeaponInTarget) ||
            !dynamic_weapon_collision_policy::isFiniteTransform(targetWorld) ||
            !dynamic_weapon_collision_policy::isFiniteTransform(requestedWeapon)) return result;
        result.weaponWorld = transform_math::composeTransforms(targetWorld, entryWeaponInTarget);
        RE::NiPoint3 axis{};
        if (!axisWorld(blade, result.weaponWorld, axis)) return result;
        const auto entryTip = transform_math::localPointToWorld(result.weaponWorld, blade.tipLocal);
        const auto requestedTip = transform_math::localPointToWorld(requestedWeapon, blade.tipLocal);
        result.requestedDepth = dot(difference(requestedTip, entryTip), axis);
        if (!std::isfinite(result.requestedDepth)) return result;
        const float depth = std::clamp(result.requestedDepth, -kWithdrawalClearanceGame - 1.0f, maximumDepthGame);
        result.weaponWorld.translate.x += axis.x * depth;
        result.weaponWorld.translate.y += axis.y * depth;
        result.weaponWorld.translate.z += axis.z * depth;
        result.valid = dynamic_weapon_collision_policy::isFiniteTransform(result.weaponWorld);
        return result;
    }

    inline bool withdrawn(float requestedDepth, float actualDepth)
    {
        return std::isfinite(requestedDepth) && std::isfinite(actualDepth) &&
            requestedDepth <= -kWithdrawalClearanceGame && actualDepth <= -kWithdrawalClearanceGame;
    }
}
