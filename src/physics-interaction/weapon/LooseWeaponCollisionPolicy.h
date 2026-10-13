#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"

namespace rock::loose_weapon_collision_policy
{
    inline constexpr std::size_t kMaxWeapons = 16;
    inline constexpr std::size_t kMaxNativeBodies = 64;
    // Parked owners leave this bounded pool immediately; no cell/global scans.
    inline constexpr double kPreparationSliceMilliseconds = 0.5;
    template<class WeaponType>
    inline constexpr bool eligibleType(WeaponType type) noexcept
    {
        return type == WeaponType::kGun || weapon_type_policy::isEquippedMelee(type);
    }
    inline constexpr float kSettleSeconds = 2.0f;
    inline constexpr float kRestTranslationMeters = 0.01f;
    inline constexpr float kRestRotationRadians = 0.034906585f; // Two degrees.

    struct Slot
    {
        std::uint32_t reference{};
    };

    // A dropped, still-active weapon keeps its slot until safe retirement.
    // Hand changes never change ownership; two hands on one ref use one slot.
    inline std::array<std::uint32_t,kMaxWeapons> admissions(
        const std::array<Slot,kMaxWeapons>& slots, std::array<std::uint32_t,2> held)
    {
        std::array<std::uint32_t,kMaxWeapons> result{};
        for (const auto reference:held) {
            if (!reference) continue;
            bool owned=false;
            for (std::size_t i=0;i<slots.size();++i)
                owned=owned || slots[i].reference==reference || result[i]==reference;
            if (owned) continue;
            auto selected=slots.size();
            for (std::size_t i=0;i<slots.size();++i)
                if (!result[i] && !slots[i].reference) { selected=i; break; }
            if (selected!=slots.size()) result[selected]=reference;
        }
        return result;
    }

    struct PoseDelta
    {
        float translationMeters{}, rotationRadians{};
        bool valid{};

        bool withinRestTolerance() const noexcept
        {
            return valid && translationMeters <= kRestTranslationMeters && rotationRadians <= kRestRotationRadians;
        }
    };

    template<class Transform>
    PoseDelta poseDelta(const Transform& a, const Transform& b, float gameToMeters)
    {
        if (!std::isfinite(gameToMeters) || gameToMeters <= 0) return {};
        const float x=(a.translate.x-b.translate.x)*gameToMeters;
        const float y=(a.translate.y-b.translate.y)*gameToMeters;
        const float z=(a.translate.z-b.translate.z)*gameToMeters;
        const float translation=std::sqrt(x*x+y*y+z*z);
        if (!std::isfinite(translation)) return {};
        float trace=0;
        for (unsigned r=0;r<3;++r) for (unsigned c=0;c<3;++c) {
            if (!std::isfinite(a.rotate.entry[r][c]) || !std::isfinite(b.rotate.entry[r][c])) return {};
            trace+=a.rotate.entry[r][c]*b.rotate.entry[r][c];
        }
        // trace(A * transpose(B)) gives the relative rigid-body rotation,
        // independent of the BODY/scene matrix storage convention.
        if (!std::isfinite(trace)) return {};
        return {translation,std::acos(std::clamp((trace-1.0f)*0.5f,-1.0f,1.0f)),true};
    }

    template<class Transform, class Point>
    Transform partInShapeBody(const Transform& rootInBody, Transform sourceInRoot, const Point& sourceCenter)
    {
        sourceInRoot.translate = transform_math::localPointToWorld(sourceInRoot, sourceCenter);
        return transform_math::composeTransforms(rootInBody, sourceInRoot);
    }

    struct Parking
    {
        float supportedRestSeconds{};
        bool update(bool held, bool supported, bool stationary, float seconds) noexcept
        {
            if (held || !supported || !stationary || !std::isfinite(seconds) || seconds <= 0) {
                supportedRestSeconds = 0;
                return false;
            }
            supportedRestSeconds = (std::min)(kSettleSeconds,supportedRestSeconds+seconds);
            return supportedRestSeconds >= kSettleSeconds;
        }
    };

    // One-body counterpart to the loose weapon's per-body rest anchors.
    // Measure the entire dwell against its initial pose, never adjacent frames.
    template<class Transform>
    class AnchoredRest
    {
    public:
        bool update(bool held, bool supported, const Transform* pose, float gameToMeters, float seconds)
        {
            if (!pose) { reset(); return false; }
            if (_parking.supportedRestSeconds == 0) _anchor = *pose;
            return _parking.update(held, supported,
                poseDelta(*pose, _anchor, gameToMeters).withinRestTolerance(), seconds);
        }
        bool ready() const { return _parking.supportedRestSeconds >= kSettleSeconds; }
        float seconds() const { return _parking.supportedRestSeconds; }
        void reset() { _parking = {}; _anchor = {}; }

    private:
        Parking _parking;
        Transform _anchor{};
    };
}
