#pragma once

#include "physics-interaction/body/BodyZone.h"

#include "RE/NetImmerse/NiPoint.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::shoulder_stash
{
    /*
     * Shoulder stash now uses the HMD-relative back volume as the primary
     * spatial authority because a literal shoulder-collider touch makes storage
     * require the player to reach their real back. Generated body-zone collider
     * and contact evidence remains as an explicit backup for tracking loss,
     * transitional compatibility, and future holster systems that need physical
     * body-zone precision.
     */
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;

    enum class EvidenceSource : std::uint8_t
    {
        None = 0,
        BodyZoneCollider,
        BodyZoneContact,
        BodyZoneSustainedContact,
        BodyZoneColliderAndContact,
        HmdBackVolume,
    };

    struct CapsuleZone
    {
        body_zone::BodyZoneKind zone = body_zone::BodyZoneKind::Unknown;
        body_zone::BodyZoneSide side = body_zone::BodyZoneSide::Center;
        std::uint32_t bodyId = kInvalidBodyId;
        RE::NiPoint3 startGame{};
        RE::NiPoint3 endGame{};
        float radiusGameUnits = 0.0f;
    };

    struct Probe
    {
        RE::NiPoint3 pointGame{};
        RE::NiPoint3 velocityGamePerSecond{};
        bool hasVelocity = false;
    };

    struct RuntimeState
    {
        bool candidate = false;
        bool confirmed = false;
        body_zone::BodyZoneKind zone = body_zone::BodyZoneKind::Unknown;
        EvidenceSource source = EvidenceSource::None;
        std::uint32_t shoulderBodyId = kInvalidBodyId;
        body_zone::BodyZoneKind sustainedZone = body_zone::BodyZoneKind::Unknown;
        std::uint32_t sustainedShoulderBodyId = kInvalidBodyId;
        std::uint32_t sustainedHeldBodyId = kInvalidBodyId;
        RE::NiPoint3 sustainedHeldBodyLocalPointGame{};
        RE::NiPoint3 sustainedPointGame{};
        std::uint32_t sustainedMissFrames = 0;
        bool hasSustainedContactAnchor = false;
        bool hasSustainedPointGame = false;
        float dwellSeconds = 0.0f;
        float nextCandidatePulseTimeSeconds = 0.0f;
        RE::NiPoint3 lastProbePointGame{};
        bool hasLastProbePoint = false;
    };

    struct Decision
    {
        bool candidate = false;
        bool confirmedForCommit = false;
        bool enteredCandidate = false;
        bool changedCandidate = false;
        body_zone::BodyZoneKind zone = body_zone::BodyZoneKind::Unknown;
        EvidenceSource source = EvidenceSource::None;
        std::uint32_t shoulderBodyId = kInvalidBodyId;
        RE::NiPoint3 nearestPointGame{};
        float distanceGameUnits = 0.0f;
        float confidence = 0.0f;
        float speedGameUnitsPerSecond = 0.0f;
    };

    [[nodiscard]] inline const char* evidenceSourceName(EvidenceSource source) noexcept
    {
        switch (source) {
        case EvidenceSource::BodyZoneCollider:
            return "body-zone-collider";
        case EvidenceSource::BodyZoneContact:
            return "body-zone-contact";
        case EvidenceSource::BodyZoneSustainedContact:
            return "body-zone-sustained-contact";
        case EvidenceSource::BodyZoneColliderAndContact:
            return "body-zone-collider-and-contact";
        case EvidenceSource::HmdBackVolume:
            return "hmd-back-volume";
        default:
            return "none";
        }
    }

    [[nodiscard]] inline bool isBodyZoneEvidenceSource(EvidenceSource source) noexcept
    {
        return source == EvidenceSource::BodyZoneCollider ||
               source == EvidenceSource::BodyZoneContact ||
               source == EvidenceSource::BodyZoneSustainedContact ||
               source == EvidenceSource::BodyZoneColliderAndContact;
    }

    [[nodiscard]] inline bool sustainedContactMissWithinTolerance(std::uint32_t missFrames, int maxMissFrames) noexcept
    {
        return maxMissFrames >= 0 && missFrames <= static_cast<std::uint32_t>(maxMissFrames);
    }

    inline void clearSustainedContact(RuntimeState& state) noexcept
    {
        state.sustainedZone = body_zone::BodyZoneKind::Unknown;
        state.sustainedShoulderBodyId = kInvalidBodyId;
        state.sustainedHeldBodyId = kInvalidBodyId;
        state.sustainedHeldBodyLocalPointGame = {};
        state.sustainedPointGame = {};
        state.sustainedMissFrames = 0;
        state.hasSustainedContactAnchor = false;
        state.hasSustainedPointGame = false;
    }

    [[nodiscard]] inline bool isHmdBackVolumeEvidenceSource(EvidenceSource source) noexcept
    {
        return source == EvidenceSource::HmdBackVolume;
    }

    [[nodiscard]] inline bool shoulderStashDwellIdentityMatches(
        body_zone::BodyZoneKind previousZone,
        EvidenceSource previousSource,
        std::uint32_t previousShoulderBodyId,
        body_zone::BodyZoneKind nextZone,
        EvidenceSource nextSource,
        std::uint32_t nextShoulderBodyId) noexcept
    {
        if (previousZone != nextZone) {
            return false;
        }

        if (previousSource == nextSource && previousShoulderBodyId == nextShoulderBodyId) {
            return true;
        }

        const bool previousHmd = isHmdBackVolumeEvidenceSource(previousSource);
        const bool nextHmd = isHmdBackVolumeEvidenceSource(nextSource);
        if (previousHmd != nextHmd) {
            const auto bodySource = previousHmd ? nextSource : previousSource;
            return isBodyZoneEvidenceSource(bodySource);
        }

        return isBodyZoneEvidenceSource(previousSource) &&
               isBodyZoneEvidenceSource(nextSource) &&
               previousShoulderBodyId == nextShoulderBodyId;
    }

    [[nodiscard]] inline bool finitePoint(const RE::NiPoint3& value) noexcept
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    [[nodiscard]] inline RE::NiPoint3 add(const RE::NiPoint3& a, const RE::NiPoint3& b) noexcept
    {
        return RE::NiPoint3{ a.x + b.x, a.y + b.y, a.z + b.z };
    }

    [[nodiscard]] inline RE::NiPoint3 sub(const RE::NiPoint3& a, const RE::NiPoint3& b) noexcept
    {
        return RE::NiPoint3{ a.x - b.x, a.y - b.y, a.z - b.z };
    }

    [[nodiscard]] inline RE::NiPoint3 mul(const RE::NiPoint3& value, float scale) noexcept
    {
        return RE::NiPoint3{ value.x * scale, value.y * scale, value.z * scale };
    }

    [[nodiscard]] inline float dot(const RE::NiPoint3& a, const RE::NiPoint3& b) noexcept
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    [[nodiscard]] inline RE::NiPoint3 cross(const RE::NiPoint3& a, const RE::NiPoint3& b) noexcept
    {
        return RE::NiPoint3{
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x,
        };
    }

    [[nodiscard]] inline float lengthSquared(const RE::NiPoint3& value) noexcept
    {
        return dot(value, value);
    }

    [[nodiscard]] inline float length(const RE::NiPoint3& value) noexcept
    {
        const float sq = lengthSquared(value);
        return sq > 0.0f && std::isfinite(sq) ? std::sqrt(sq) : 0.0f;
    }

    [[nodiscard]] inline RE::NiPoint3 normalizeOr(const RE::NiPoint3& value, const RE::NiPoint3& fallback) noexcept
    {
        const float len = length(value);
        if (len <= 0.00001f) {
            return fallback;
        }
        return mul(value, 1.0f / len);
    }

    [[nodiscard]] inline float pointSegmentDistance(
        const RE::NiPoint3& point,
        const RE::NiPoint3& start,
        const RE::NiPoint3& end,
        RE::NiPoint3* outNearest = nullptr) noexcept
    {
        const auto segment = sub(end, start);
        const float denom = lengthSquared(segment);
        float t = 0.0f;
        if (denom > 0.000001f && std::isfinite(denom)) {
            t = std::clamp(dot(sub(point, start), segment) / denom, 0.0f, 1.0f);
        }

        const auto nearest = add(start, mul(segment, t));
        if (outNearest) {
            *outNearest = nearest;
        }
        return length(sub(point, nearest));
    }

    [[nodiscard]] inline float probeSpeed(const Probe& probe) noexcept
    {
        return probe.hasVelocity ? length(probe.velocityGamePerSecond) : 0.0f;
    }

    [[nodiscard]] inline bool isShoulderZone(body_zone::BodyZoneKind zone) noexcept
    {
        return zone == body_zone::BodyZoneKind::LeftShoulder || zone == body_zone::BodyZoneKind::RightShoulder;
    }

    [[nodiscard]] inline bool isSameSideShoulder(bool isLeftHand, body_zone::BodyZoneKind zone) noexcept
    {
        return (isLeftHand && zone == body_zone::BodyZoneKind::LeftShoulder) ||
               (!isLeftHand && zone == body_zone::BodyZoneKind::RightShoulder);
    }
}
