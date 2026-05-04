#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <format>
#include <limits>

#include "physics-interaction/PhysicsBodyFrame.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/NetImmerse/NiPoint.h"

namespace RE
{
    class NiAVObject;
    class TESObjectREFR;
    class bhkWorld;
    class hknpWorld;
}

namespace frik::rock::debug
{
    struct BodyOverlayFrame;
}

namespace frik::rock::origin_diagnostics
{
    /*
     * FO4VR creates exterior Havok worlds with a per-world origin/broadphase frame.
     * ROCK samples both the hknpBody transform and hknpMotion transform, but treats
     * the hknpBody transform as the collider/shape origin. hknpMotion.position is
     * center-of-mass telemetry and must not drive collider placement diagnostics.
     */
    enum class BodyPositionSource : std::uint8_t
    {
        BodyTransform = 0,
        MotionTransform
    };

    enum class VisualSourceKind : std::uint8_t
    {
        None = 0,
        BodyOwnerNode,
        HitNode,
        VisualNode,
        ReferenceRoot
    };

    enum class OriginCandidateKind : std::uint8_t
    {
        Raw = 0,
        BhkStoredOriginAdd,
        BhkStoredOriginSubtract,
        HknpBroadphaseMinShiftAdd,
        HknpBroadphaseMinShiftSubtract,
        HknpBroadphaseMaxShiftAdd,
        HknpBroadphaseMaxShiftSubtract,
        HknpBroadphaseCenterShiftAdd,
        HknpBroadphaseCenterShiftSubtract
    };

    inline constexpr std::uint32_t kOriginCandidateCapacity = 9;
    inline constexpr std::uint32_t kFreeMotionIndex = 0x7FFF'FFFF;
    inline constexpr std::uint32_t kMaxDiagnosticMotionIndex = 4096;

    struct OriginCandidate
    {
        OriginCandidateKind kind{ OriginCandidateKind::Raw };
        RE::NiPoint3 positionGame{};
        float distanceToVisual{ (std::numeric_limits<float>::max)() };
    };

    struct OriginCandidateResult
    {
        std::array<OriginCandidate, kOriginCandidateCapacity> candidates{};
        std::uint32_t count{ 0 };
        OriginCandidate best{};
        float warningThresholdGameUnits{ 0.0f };
        bool exceedsWarningThreshold{ false };
    };

    struct WorldOriginSnapshot
    {
        RE::NiPoint3 bhkStoredOriginGame{};
        RE::NiPoint3 hknpBroadphaseMinShiftGame{};
        RE::NiPoint3 hknpBroadphaseMaxShiftGame{};
        RE::NiPoint3 hknpBroadphaseCenterShiftGame{};
        RE::NiPoint3 hknpBroadphaseBoundsMinGame{};
        RE::NiPoint3 hknpBroadphaseBoundsMaxGame{};
        bool hasBhkStoredOrigin{ false };
        bool hasHknpBroadphaseQueryShift{ false };
        bool hasHknpBroadphaseBounds{ false };
    };

    struct TargetOriginSample
    {
        bool valid{ false };
        RE::hknpBodyId bodyId{ 0x7FFF'FFFF };
        RE::TESObjectREFR* refr{ nullptr };
        RE::NiAVObject* bodyOwnerNode{ nullptr };
        RE::NiAVObject* visualSourceNode{ nullptr };
        VisualSourceKind visualSourceKind{ VisualSourceKind::None };
        BodyPositionSource bodyPositionSource{ BodyPositionSource::BodyTransform };
        RE::NiPoint3 observedBodyPositionGame{};
        RE::NiPoint3 bodyTransformPositionGame{};
        RE::NiPoint3 motionPositionGame{};
        bool hasMotionPosition{ false };
        RE::NiPoint3 visualPositionGame{};
        float bodyTransformDistanceToVisual{ (std::numeric_limits<float>::max)() };
        float motionDistanceToVisual{ (std::numeric_limits<float>::max)() };
        std::uint32_t bodyFlags{ 0 };
        std::uint32_t filterInfo{ 0 };
        std::uint32_t motionIndex{ kFreeMotionIndex };
        std::uint16_t motionPropertiesId{ 0 };
        WorldOriginSnapshot origins{};
        OriginCandidateResult candidates{};
    };

    inline RE::NiPoint3 addPoint(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return RE::NiPoint3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }

    inline RE::NiPoint3 subtractPoint(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return RE::NiPoint3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    inline RE::NiPoint3 scalePoint(const RE::NiPoint3& value, float scale)
    {
        return RE::NiPoint3(value.x * scale, value.y * scale, value.z * scale);
    }

    inline float distanceSquared(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        const auto delta = subtractPoint(lhs, rhs);
        return delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
    }

    inline float distance(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return std::sqrt(distanceSquared(lhs, rhs));
    }

    inline BodyPositionSource chooseBodyPositionSource(std::uint32_t motionIndex)
    {
        (void)motionIndex;
        return BodyPositionSource::BodyTransform;
    }

    inline VisualSourceKind chooseVisualSourceKind(bool hasBodyOwnerNode, bool hasHitNode, bool hasVisualNode, bool hasReferenceRoot)
    {
        if (hasBodyOwnerNode) {
            return VisualSourceKind::BodyOwnerNode;
        }
        if (hasHitNode) {
            return VisualSourceKind::HitNode;
        }
        if (hasVisualNode) {
            return VisualSourceKind::VisualNode;
        }
        if (hasReferenceRoot) {
            return VisualSourceKind::ReferenceRoot;
        }
        return VisualSourceKind::None;
    }

    inline const char* bodyPositionSourceName(BodyPositionSource source)
    {
        switch (source) {
        case BodyPositionSource::BodyTransform:
            return "bodyTransform";
        case BodyPositionSource::MotionTransform:
            return "motionTransform";
        default:
            return "unknown";
        }
    }

    inline const char* visualSourceKindName(VisualSourceKind kind)
    {
        switch (kind) {
        case VisualSourceKind::BodyOwnerNode:
            return "bodyOwnerNode";
        case VisualSourceKind::HitNode:
            return "hitNode";
        case VisualSourceKind::VisualNode:
            return "visualNode";
        case VisualSourceKind::ReferenceRoot:
            return "referenceRoot";
        case VisualSourceKind::None:
        default:
            return "none";
        }
    }

    inline const char* candidateKindName(OriginCandidateKind kind)
    {
        switch (kind) {
        case OriginCandidateKind::Raw:
            return "raw";
        case OriginCandidateKind::BhkStoredOriginAdd:
            return "bhk+";
        case OriginCandidateKind::BhkStoredOriginSubtract:
            return "bhk-";
        case OriginCandidateKind::HknpBroadphaseMinShiftAdd:
            return "hknpMinShift+";
        case OriginCandidateKind::HknpBroadphaseMinShiftSubtract:
            return "hknpMinShift-";
        case OriginCandidateKind::HknpBroadphaseMaxShiftAdd:
            return "hknpMaxShift+";
        case OriginCandidateKind::HknpBroadphaseMaxShiftSubtract:
            return "hknpMaxShift-";
        case OriginCandidateKind::HknpBroadphaseCenterShiftAdd:
            return "hknpCenterShift+";
        case OriginCandidateKind::HknpBroadphaseCenterShiftSubtract:
            return "hknpCenterShift-";
        default:
            return "unknown";
        }
    }

    inline OriginCandidate makeCandidate(OriginCandidateKind kind, const RE::NiPoint3& positionGame, const RE::NiPoint3& visualGame)
    {
        return OriginCandidate{
            .kind = kind,
            .positionGame = positionGame,
            .distanceToVisual = distance(positionGame, visualGame),
        };
    }

    inline OriginCandidateResult evaluateOriginCandidates(
        const RE::NiPoint3& observedBodyGame,
        const RE::NiPoint3& visualGame,
        const RE::NiPoint3& bhkStoredOriginGame,
        const RE::NiPoint3& hknpBroadphaseMinShiftGame,
        const RE::NiPoint3& hknpBroadphaseMaxShiftGame,
        float warningThresholdGameUnits)
    {
        OriginCandidateResult result{};
        result.warningThresholdGameUnits = warningThresholdGameUnits;

        const auto hknpBroadphaseCenterShiftGame = scalePoint(addPoint(hknpBroadphaseMinShiftGame, hknpBroadphaseMaxShiftGame), 0.5f);

        result.candidates[0] = makeCandidate(OriginCandidateKind::Raw, observedBodyGame, visualGame);
        result.candidates[1] = makeCandidate(OriginCandidateKind::BhkStoredOriginAdd, addPoint(observedBodyGame, bhkStoredOriginGame), visualGame);
        result.candidates[2] = makeCandidate(OriginCandidateKind::BhkStoredOriginSubtract, subtractPoint(observedBodyGame, bhkStoredOriginGame), visualGame);
        result.candidates[3] = makeCandidate(OriginCandidateKind::HknpBroadphaseMinShiftAdd, addPoint(observedBodyGame, hknpBroadphaseMinShiftGame), visualGame);
        result.candidates[4] =
            makeCandidate(OriginCandidateKind::HknpBroadphaseMinShiftSubtract, subtractPoint(observedBodyGame, hknpBroadphaseMinShiftGame), visualGame);
        result.candidates[5] = makeCandidate(OriginCandidateKind::HknpBroadphaseMaxShiftAdd, addPoint(observedBodyGame, hknpBroadphaseMaxShiftGame), visualGame);
        result.candidates[6] =
            makeCandidate(OriginCandidateKind::HknpBroadphaseMaxShiftSubtract, subtractPoint(observedBodyGame, hknpBroadphaseMaxShiftGame), visualGame);
        result.candidates[7] =
            makeCandidate(OriginCandidateKind::HknpBroadphaseCenterShiftAdd, addPoint(observedBodyGame, hknpBroadphaseCenterShiftGame), visualGame);
        result.candidates[8] =
            makeCandidate(OriginCandidateKind::HknpBroadphaseCenterShiftSubtract, subtractPoint(observedBodyGame, hknpBroadphaseCenterShiftGame), visualGame);
        result.count = kOriginCandidateCapacity;

        result.best = result.candidates[0];
        for (std::uint32_t i = 1; i < result.count; ++i) {
            if (result.candidates[i].distanceToVisual + 0.0001f < result.best.distanceToVisual) {
                result.best = result.candidates[i];
            }
        }

        result.exceedsWarningThreshold = warningThresholdGameUnits > 0.0f && result.best.distanceToVisual > warningThresholdGameUnits;
        return result;
    }

    bool sampleTarget(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        RE::hknpBodyId bodyId,
        RE::TESObjectREFR* refr,
        RE::NiAVObject* hitNode,
        RE::NiAVObject* visualNode,
        float warningThresholdGameUnits,
        TargetOriginSample& outSample);

    void logSampleIfNeeded(const char* handLabel, bool held, const TargetOriginSample& sample, std::uint32_t intervalFrames);

    void publishMarkers(debug::BodyOverlayFrame& frame, const TargetOriginSample& sample);
}
