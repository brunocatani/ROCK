#pragma once

#include <cstdint>

namespace rock::weapon_grip_authority_policy
{
    enum class Source : std::uint8_t
    {
        None,
        FrikCustomFile,
        AuthoredAnimation,
        FrikEmbeddedResource,
        FrikLiveNodeFallback,
    };

    struct Availability
    {
        bool frikCustomFile{ false };
        bool authoredAnimation{ false };
        bool frikEmbeddedResource{ false };
        bool allowFrikLiveNodeFallback{ false };
    };

    /*
     * A user-authored hFRIK JSON is explicit correction authority and must
     * always beat ROCK's native-animation calibration. ROCK's learned pose is
     * otherwise authoritative; hFRIK's embedded table is retained only for a
     * weapon that has not produced a usable native capture in this session.
     * The live-node fallback is legacy behavior and is deliberately available
     * only when authored capture is ineligible for the canonical hand.
     */
    [[nodiscard]] constexpr Source select(const Availability& availability) noexcept
    {
        if (availability.frikCustomFile) {
            return Source::FrikCustomFile;
        }
        if (availability.authoredAnimation) {
            return Source::AuthoredAnimation;
        }
        if (availability.frikEmbeddedResource) {
            return Source::FrikEmbeddedResource;
        }
        if (availability.allowFrikLiveNodeFallback) {
            return Source::FrikLiveNodeFallback;
        }
        return Source::None;
    }

    [[nodiscard]] constexpr const char* sourceName(const Source source) noexcept
    {
        switch (source) {
        case Source::FrikCustomFile:
            return "frikCustomFile";
        case Source::AuthoredAnimation:
            return "authoredAnimationLibrary";
        case Source::FrikEmbeddedResource:
            return "frikEmbeddedColdFallback";
        case Source::FrikLiveNodeFallback:
            return "frikLiveNodeLegacyFallback";
        case Source::None:
        default:
            return "gripAuthorityUnavailable";
        }
    }
}
