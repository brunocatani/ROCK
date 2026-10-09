#pragma once

#include <RE/NetImmerse/NiPoint.h>
#include <array>
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

        bool sameTarget(const Candidate& other) const noexcept
        {
            return kind == other.kind && owner == other.owner &&
                source == other.source && point == other.point && group == other.group;
        }
    };

    struct State
    {
        std::uint64_t generation{};
        std::uint64_t ownership{};
        Candidate previous{};
    };

    // Distances are physical game units, never divided by activation radius.
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
                !std::isfinite(candidate.distance) || candidate.distance < 0.0f) continue;
            if (candidate.sameTarget(state.previous)) previous = candidate;
            if (candidate.distance < best.distance ||
                (candidate.distance == best.distance && candidate.kind < best.kind)) best = candidate;
        }
        if (previous.eligible && previous.distance <= best.distance + 0.35f) best = previous;
        state.previous = best;
        return best;
    }

    struct DebugSnapshot
    {
        std::array<Candidate, 3> candidates{};
        Candidate selected{};
        RE::NiPoint3 palm{};
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
