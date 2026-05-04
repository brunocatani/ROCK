#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace frik::rock::physics_scale
{
    /*
     * ROCK must not guess the game/Havok conversion from round numbers or from
     * FRIK's player VR scale. FO4VR already owns the hknp unit scale and HIGGS
     * reads Skyrim's equivalent runtime globals instead of hardcoding 70. Keeping
     * that distinction centralized prevents distance-from-origin drift while still
     * letting diagnostics report FRIK's live scale separately.
     */
    constexpr float kFallbackHavokToGame = 70.0f;
    constexpr float kFallbackGameToHavok = 1.0f / kFallbackHavokToGame;

    struct Snapshot
    {
        float gameToHavok = kFallbackGameToHavok;
        float havokToGame = kFallbackHavokToGame;
        float vrScale = 0.0f;
        float vrGlobalScale = 0.0f;
        float raycastScale = 0.0f;
        std::uint32_t revision = 0;
    };

    inline bool isUsableScale(float value)
    {
        return std::isfinite(value) && value > 0.000001f && value < 10000.0f;
    }

    inline Snapshot makeSnapshot(float gameToHavok, float havokToGame, float vrScale, std::uint32_t revision = 0, float vrGlobalScale = 0.0f, float raycastScale = 0.0f)
    {
        Snapshot snapshot{};
        snapshot.vrScale = std::isfinite(vrScale) ? vrScale : 0.0f;
        snapshot.vrGlobalScale = std::isfinite(vrGlobalScale) ? vrGlobalScale : 0.0f;
        snapshot.raycastScale = std::isfinite(raycastScale) ? raycastScale : 0.0f;
        snapshot.revision = revision;

        if (isUsableScale(gameToHavok)) {
            snapshot.gameToHavok = gameToHavok;
        } else if (isUsableScale(havokToGame)) {
            snapshot.gameToHavok = 1.0f / havokToGame;
        }

        if (isUsableScale(havokToGame)) {
            snapshot.havokToGame = havokToGame;
        } else if (isUsableScale(gameToHavok)) {
            snapshot.havokToGame = 1.0f / gameToHavok;
        }

        return snapshot;
    }

    inline bool sameConversionScale(const Snapshot& a, const Snapshot& b)
    {
        return std::fabs(a.gameToHavok - b.gameToHavok) <= 0.0000001f && std::fabs(a.havokToGame - b.havokToGame) <= 0.0001f;
    }

    inline bool shouldInvalidateCachedScaleData(const Snapshot& previous, const Snapshot& current)
    {
        return previous.revision != current.revision || !sameConversionScale(previous, current);
    }

    inline float reciprocalDriftGameUnits(const Snapshot& scale)
    {
        if (!isUsableScale(scale.gameToHavok) || !isUsableScale(scale.havokToGame)) {
            return (std::numeric_limits<float>::max)();
        }

        return std::fabs((1.0f / scale.gameToHavok) - scale.havokToGame);
    }

    inline bool hasReciprocalMismatch(const Snapshot& scale, float warningThresholdGameUnits)
    {
        const float threshold = warningThresholdGameUnits > 0.0f ? warningThresholdGameUnits : 0.0f;
        return reciprocalDriftGameUnits(scale) > threshold;
    }

    template <class Vec3>
    inline Vec3 gameToHavokPoint(const Vec3& point, const Snapshot& scale)
    {
        Vec3 result{};
        result.x = point.x * scale.gameToHavok;
        result.y = point.y * scale.gameToHavok;
        result.z = point.z * scale.gameToHavok;
        return result;
    }

    template <class Vec3>
    inline Vec3 havokToGamePoint(const Vec3& point, const Snapshot& scale)
    {
        Vec3 result{};
        result.x = point.x * scale.havokToGame;
        result.y = point.y * scale.havokToGame;
        result.z = point.z * scale.havokToGame;
        return result;
    }

    template <class Vec3>
    inline float distanceGame(const Vec3& a, const Vec3& b)
    {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        const float dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    Snapshot current();
    float gameToHavok();
    float havokToGame();
    std::uint32_t revision();
    bool refreshAndLogIfChanged();
}
