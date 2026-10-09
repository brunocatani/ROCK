#pragma once

#include <RE/NetImmerse/NiPoint.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>

namespace rock::weapon_grip_selection
{
    enum class Kind : std::uint8_t { None, Part, Firing, Support, Surface };

    struct Candidate
    {
        Kind kind{ Kind::None };
        RE::NiPoint3 world{};
        float distance{ std::numeric_limits<float>::infinity() };
        float radius{};
        std::uint64_t owner{};
        std::uint64_t source{};
        std::uint32_t point{};
        bool eligible{};
        std::uint32_t group{};
        float poseCredit{};

        float score() const noexcept { return distance - poseCredit; }

        bool sameTarget(const Candidate& other) const noexcept
        {
            return kind == other.kind && owner == other.owner &&
                source == other.source && point == other.point && group == other.group;
        }
    };

    struct SupportPosePreference
    {
        float distance{}, palmDot{}, wristDot{}, strength{}, credit{};
        bool valid{};
    };

    // Game-unit distances. Full preference is confined to the seated palm;
    // smooth falloff ends well inside the general acquisition sphere. Two
    // independent directions distinguish wrapping the grip from rolling the
    // hand across a slide, without demanding exact authored wrist rotation.
    inline constexpr float kSupportPoseOuterRadius = 5.0f;
    inline SupportPosePreference supportPosePreference(const RE::NiPoint3& palm,
        const RE::NiPoint3& seat, const RE::NiPoint3& palmNormal,
        const RE::NiPoint3& targetNormal, const RE::NiPoint3& wristAxis,
        const RE::NiPoint3& targetWristAxis) noexcept
    {
        SupportPosePreference result{};
        const auto dot = [](const auto& a, const auto& b) { return a.x*b.x+a.y*b.y+a.z*b.z; };
        const auto directionDot = [&](const auto& a, const auto& b, float& value) {
            const float aa=dot(a,a), bb=dot(b,b), ab=dot(a,b);
            if (!std::isfinite(aa) || !std::isfinite(bb) || !std::isfinite(ab) || aa<0.000001f || bb<0.000001f) return false;
            value=std::clamp(ab/std::sqrt(aa*bb),-1.0f,1.0f);
            return std::isfinite(value);
        };
        const RE::NiPoint3 delta{palm.x-seat.x,palm.y-seat.y,palm.z-seat.z};
        const float squared=dot(delta,delta);
        if (!std::isfinite(squared) || !directionDot(palmNormal,targetNormal,result.palmDot) ||
            !directionDot(wristAxis,targetWristAxis,result.wristDot)) return result;
        result.distance=std::sqrt(squared);
        const auto smooth = [](float value) { value=std::clamp(value,0.0f,1.0f); return value*value*(3.0f-2.0f*value); };
        const float position=smooth((kSupportPoseOuterRadius-result.distance)/(kSupportPoseOuterRadius-1.5f));
        // Full directional credit within 25 degrees, none beyond 70 degrees.
        const float orientation=smooth(((std::min)(result.palmDot,result.wristDot)-0.34202014f)/(0.90630779f-0.34202014f));
        result.strength=position*orientation;
        result.credit=6.0f*result.strength;
        result.valid=true;
        return result;
    }

    struct State
    {
        std::uint64_t generation{};
        std::uint64_t ownership{};
        Candidate previous{};
    };

    // Distances are physical game units, never divided by activation radius.
    // A local pistol support-pose credit may adjust score, never eligibility.
    // Keep a still-eligible hover through sub-unit tracking noise. No grace
    // period admits a missing target, and identity changes drop the preference.
    inline Candidate select(State& state, std::span<const Candidate> candidates,
        std::uint64_t generation, std::uint64_t ownership, bool resetPreference) noexcept
    {
        if (!generation || !ownership) { state = {}; return {}; }
        if (resetPreference || state.generation != generation || state.ownership != ownership)
            state = {generation, ownership, {}};
        Candidate best{}, previous{};
        for (const auto& candidate : candidates) {
            if (!candidate.eligible || candidate.kind == Kind::None ||
                !std::isfinite(candidate.distance) || candidate.distance < 0.0f ||
                !std::isfinite(candidate.poseCredit) || candidate.poseCredit < 0.0f) continue;
            if (candidate.sameTarget(state.previous)) previous = candidate;
            if (candidate.score() < best.score() ||
                (candidate.score() == best.score() && candidate.kind < best.kind)) best = candidate;
        }
        if (previous.eligible && previous.score() <= best.score() + 0.35f) best = previous;
        state.previous = best;
        return best;
    }

    struct DebugSnapshot
    {
        std::array<Candidate, 3> candidates{};
        Candidate selected{};
        RE::NiPoint3 palm{};
        RE::NiPoint3 supportSeat{}, palmNormal{}, supportNormal{};
        RE::NiPoint3 wristAxis{}, supportWristAxis{};
        SupportPosePreference supportPose{};
        bool pistolSupportContext{}, supportPreferenceApplied{};
        bool valid{};
    };

    inline const char* name(Kind kind) noexcept
    {
        switch (kind) {
        case Kind::Firing: return "FIRING";
        case Kind::Support: return "SUPPORT";
        case Kind::Part: return "API PART";
        case Kind::Surface: return "SURFACE";
        default: return "NONE";
        }
    }
}
