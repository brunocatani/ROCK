#pragma once

#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"

namespace rock::blade_penetration
{
    inline constexpr std::uint32_t kSwitchbladeFormId = 0x000FDC81u;
    inline constexpr float kMaximumDepthGame = 5.0f;
    inline constexpr float kTipContactRadiusGame = 1.25f;
    inline constexpr float kEntryPressureGame = 0.5f;
    inline constexpr float kWithdrawalClearanceGame = 2.0f;

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
        const RE::NiTransform& targetWorld, const RE::NiTransform& requestedWeapon)
    {
        Guide result{};
        if (!dynamic_weapon_collision_policy::isFiniteTransform(entryWeaponInTarget) ||
            !dynamic_weapon_collision_policy::isFiniteTransform(targetWorld) ||
            !dynamic_weapon_collision_policy::isFiniteTransform(requestedWeapon)) return result;
        result.weaponWorld = transform_math::composeTransforms(targetWorld, entryWeaponInTarget);
        RE::NiPoint3 axis{};
        if (!axisWorld(blade, result.weaponWorld, axis)) return result;
        const auto entryTip = transform_math::localPointToWorld(result.weaponWorld, blade.tipLocal);
        const auto requestedTip = transform_math::localPointToWorld(requestedWeapon, blade.tipLocal);
        result.requestedDepth = dot(difference(requestedTip, entryTip), axis);
        if (!std::isfinite(result.requestedDepth)) return result;
        const float depth = std::clamp(result.requestedDepth, -kWithdrawalClearanceGame - 1.0f, kMaximumDepthGame);
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
