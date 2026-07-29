#include "physics-interaction/debug/PhysicsWorldOriginDiagnostics.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

#include "physics-interaction/debug/DebugBodyOverlay.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"

#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiAVObject.h"

namespace rock::origin_diagnostics
{
    namespace
    {
        constexpr std::uintptr_t kBhkStoredOriginOffset = 0x50;
        constexpr std::uintptr_t kHknpBroadphaseMinShiftOffset = 0x180;
        constexpr std::uintptr_t kHknpBroadphaseMaxShiftOffset = 0x190;
        constexpr std::uintptr_t kHknpBroadphaseBoundsMinOffset = 0x1C0;
        constexpr std::uintptr_t kHknpBroadphaseBoundsMaxOffset = 0x1D0;

        struct BodySnapshot
        {
            const RE::hknpBody* body{ nullptr };
            RE::NiPoint3 bodyTransformPositionGame{};
            RE::NiPoint3 motionPositionGame{};
            RE::NiPoint3 observedPositionGame{};
            BodyPositionSource positionSource{ BodyPositionSource::BodyTransform };
            bool hasMotionPosition{ false };
            std::uint32_t flags{ 0 };
            std::uint32_t filterInfo{ 0 };
            std::uint32_t motionIndex{ kFreeMotionIndex };
            std::uint16_t motionPropertiesId{ 0 };
        };

        struct VisualSourceChoice
        {
            RE::NiAVObject* node{ nullptr };
            VisualSourceKind kind{ VisualSourceKind::None };
        };

        bool isFinitePoint(const RE::NiPoint3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        RE::NiPoint3 readHavokPointAsGameUnits(const void* base, std::uintptr_t offset)
        {
            const auto* floats = reinterpret_cast<const float*>(reinterpret_cast<std::uintptr_t>(base) + offset);
            return RE::NiPoint3(floats[0] * havokToGameScale(), floats[1] * havokToGameScale(), floats[2] * havokToGameScale());
        }

        RE::NiPoint3 readBodyTransformPositionGameUnits(const RE::hknpBody* body)
        {
            const auto* floats = reinterpret_cast<const float*>(body);
            return RE::NiPoint3(floats[12] * havokToGameScale(), floats[13] * havokToGameScale(), floats[14] * havokToGameScale());
        }

        RE::NiPoint3 readMotionPositionGameUnits(const RE::hknpMotion* motion)
        {
            return RE::NiPoint3(motion->position.x * havokToGameScale(), motion->position.y * havokToGameScale(), motion->position.z * havokToGameScale());
        }

        bool readBodySnapshot(RE::hknpWorld* world, RE::hknpBodyId bodyId, BodySnapshot& out)
        {
            out = BodySnapshot{};
            auto* body = havok_runtime::getBody(world, bodyId);
            if (!body || !body->shape) {
                return false;
            }

            out.body = body;
            out.bodyTransformPositionGame = readBodyTransformPositionGameUnits(body);
            out.positionSource = chooseBodyPositionSource(body->motionIndex);
            out.observedPositionGame = out.bodyTransformPositionGame;
            out.flags = body->flags;
            out.filterInfo = body->collisionFilterInfo;
            out.motionIndex = body->motionIndex;
            out.motionPropertiesId = body->motionPropertiesId;

            if (!isFinitePoint(out.bodyTransformPositionGame)) {
                return false;
            }

            if (body_frame::hasUsableMotionIndex(body->motionIndex)) {
                auto* motion = havok_runtime::getMotion(world, body->motionIndex);
                if (!motion) {
                    return false;
                }

                out.motionPositionGame = readMotionPositionGameUnits(motion);
                if (!isFinitePoint(out.motionPositionGame)) {
                    return false;
                }

                out.hasMotionPosition = true;
            }

            return true;
        }

        VisualSourceChoice chooseVisualSource(
            RE::TESObjectREFR* refr,
            RE::NiAVObject* bodyOwnerNode,
            RE::NiAVObject* hitNode,
            RE::NiAVObject* visualNode)
        {
            auto* referenceRoot = refr ? refr->Get3D() : nullptr;
            const VisualSourceChoice choices[]{
                { bodyOwnerNode, VisualSourceKind::BodyOwnerNode },
                { hitNode, VisualSourceKind::HitNode },
                { visualNode, VisualSourceKind::VisualNode },
                { referenceRoot, VisualSourceKind::ReferenceRoot },
            };

            for (const auto& choice : choices) {
                if (choice.node && isFinitePoint(choice.node->world.translate)) {
                    return choice;
                }
            }

            return {};
        }

        void appendMarker(debug::BodyOverlayFrame& frame, debug::MarkerOverlayRole role, const RE::NiPoint3& position, const RE::NiPoint3& lineEnd, float size, bool drawPoint,
            bool drawLine)
        {
            if (frame.markerCount >= frame.markerEntries.size()) {
                return;
            }

            auto& entry = frame.markerEntries[frame.markerCount++];
            entry.role = role;
            entry.position = position;
            entry.lineEnd = lineEnd;
            entry.size = size;
            entry.drawPoint = drawPoint;
            entry.drawLine = drawLine;
        }

        float candidateDistance(const OriginCandidateResult& result, OriginCandidateKind kind)
        {
            for (std::uint32_t i = 0; i < result.count; ++i) {
                if (result.candidates[i].kind == kind) {
                    return result.candidates[i].distanceToVisual;
                }
            }

            return (std::numeric_limits<float>::max)();
        }
    }

    bool sampleTarget(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        RE::hknpBodyId bodyId,
        RE::TESObjectREFR* refr,
        RE::NiAVObject* hitNode,
        RE::NiAVObject* visualNode,
        float warningThresholdGameUnits,
        TargetOriginSample& outSample)
    {
        outSample = TargetOriginSample{};
        outSample.bodyId = bodyId;
        outSample.refr = refr;

        if (!bhkWorld || !hknpWorld) {
            return false;
        }

        BodySnapshot bodySnapshot{};
        if (!readBodySnapshot(hknpWorld, bodyId, bodySnapshot)) {
            return false;
        }

        auto* bodyOwnerNode = getOwnerNodeFromBody(bodySnapshot.body);
        const auto visualSource = chooseVisualSource(refr, bodyOwnerNode, hitNode, visualNode);
        if (!visualSource.node) {
            return false;
        }

        const auto visualPositionGame = visualSource.node->world.translate;
        if (!isFinitePoint(bodySnapshot.observedPositionGame) || !isFinitePoint(visualPositionGame)) {
            return false;
        }

        WorldOriginSnapshot origins{};
        origins.bhkStoredOriginGame = readHavokPointAsGameUnits(bhkWorld, kBhkStoredOriginOffset);
        origins.hasBhkStoredOrigin = isFinitePoint(origins.bhkStoredOriginGame);
        origins.hknpBroadphaseMinShiftGame = readHavokPointAsGameUnits(hknpWorld, kHknpBroadphaseMinShiftOffset);
        origins.hknpBroadphaseMaxShiftGame = readHavokPointAsGameUnits(hknpWorld, kHknpBroadphaseMaxShiftOffset);
        origins.hknpBroadphaseCenterShiftGame = scalePoint(addPoint(origins.hknpBroadphaseMinShiftGame, origins.hknpBroadphaseMaxShiftGame), 0.5f);
        origins.hknpBroadphaseBoundsMinGame = readHavokPointAsGameUnits(hknpWorld, kHknpBroadphaseBoundsMinOffset);
        origins.hknpBroadphaseBoundsMaxGame = readHavokPointAsGameUnits(hknpWorld, kHknpBroadphaseBoundsMaxOffset);
        origins.hasHknpBroadphaseQueryShift =
            isFinitePoint(origins.hknpBroadphaseMinShiftGame) && isFinitePoint(origins.hknpBroadphaseMaxShiftGame) && isFinitePoint(origins.hknpBroadphaseCenterShiftGame);
        origins.hasHknpBroadphaseBounds = isFinitePoint(origins.hknpBroadphaseBoundsMinGame) && isFinitePoint(origins.hknpBroadphaseBoundsMaxGame);

        outSample.valid = true;
        outSample.bodyOwnerNode = bodyOwnerNode;
        outSample.visualSourceNode = visualSource.node;
        outSample.visualSourceKind = visualSource.kind;
        outSample.bodyPositionSource = bodySnapshot.positionSource;
        outSample.observedBodyPositionGame = bodySnapshot.observedPositionGame;
        outSample.bodyTransformPositionGame = bodySnapshot.bodyTransformPositionGame;
        outSample.motionPositionGame = bodySnapshot.motionPositionGame;
        outSample.hasMotionPosition = bodySnapshot.hasMotionPosition;
        outSample.visualPositionGame = visualPositionGame;
        outSample.bodyTransformDistanceToVisual = distance(bodySnapshot.bodyTransformPositionGame, visualPositionGame);
        outSample.motionDistanceToVisual = bodySnapshot.hasMotionPosition ? distance(bodySnapshot.motionPositionGame, visualPositionGame) : -1.0f;
        outSample.bodyFlags = bodySnapshot.flags;
        outSample.filterInfo = bodySnapshot.filterInfo;
        outSample.motionIndex = bodySnapshot.motionIndex;
        outSample.motionPropertiesId = bodySnapshot.motionPropertiesId;
        outSample.origins = origins;
        outSample.candidates = evaluateOriginCandidates(bodySnapshot.observedPositionGame,
            visualPositionGame,
            origins.hasBhkStoredOrigin ? origins.bhkStoredOriginGame : RE::NiPoint3(),
            origins.hasHknpBroadphaseQueryShift ? origins.hknpBroadphaseMinShiftGame : RE::NiPoint3(),
            origins.hasHknpBroadphaseQueryShift ? origins.hknpBroadphaseMaxShiftGame : RE::NiPoint3(),
            (std::max)(warningThresholdGameUnits, 0.0f));
        return true;
    }

    void logSampleIfNeeded(const char* handLabel, bool held, const TargetOriginSample& sample, std::uint32_t intervalFrames)
    {
        static std::uint32_t s_logCounter = 0;

        if (!sample.valid) {
            return;
        }

        const std::uint32_t sanitizedInterval = (std::max)(intervalFrames, 1u);
        if (++s_logCounter < sanitizedInterval) {
            return;
        }
        s_logCounter = 0;

        auto emitWarn = [](auto&&... args) {
            ROCK_LOG_WARN(Hand,
                "World object origin diagnostic: hand={} state={} formID={:08X} bodyId={} visualSource={} bodySource={} motionIndex={} flags=0x{:08X} filter=0x{:08X} motionProps={} visual=({:.2f},{:.2f},{:.2f}) observed=({:.2f},{:.2f},{:.2f}) bodyTransform=({:.2f},{:.2f},{:.2f}) motionValid={} motion=({:.2f},{:.2f},{:.2f}) best={} bestPos=({:.2f},{:.2f},{:.2f}) distBest={:.2f} distObserved={:.2f} distBodyTransform={:.2f} distMotion={:.2f} distBhk+={:.2f} distBhk-={:.2f} distHknpMin+={:.2f} distHknpMin-={:.2f} distHknpMax+={:.2f} distHknpMax-={:.2f} distHknpCenter+={:.2f} distHknpCenter-={:.2f} bhkOrigin=({:.2f},{:.2f},{:.2f}) hknpMinShift=({:.2f},{:.2f},{:.2f}) hknpMaxShift=({:.2f},{:.2f},{:.2f}) hknpCenterShift=({:.2f},{:.2f},{:.2f}) hknpBoundsMin=({:.2f},{:.2f},{:.2f}) hknpBoundsMax=({:.2f},{:.2f},{:.2f})",
                std::forward<decltype(args)>(args)...);
        };
        auto emitDebug = [](auto&&... args) {
            ROCK_LOG_DEBUG(Hand,
                "World object origin diagnostic: hand={} state={} formID={:08X} bodyId={} visualSource={} bodySource={} motionIndex={} flags=0x{:08X} filter=0x{:08X} motionProps={} visual=({:.2f},{:.2f},{:.2f}) observed=({:.2f},{:.2f},{:.2f}) bodyTransform=({:.2f},{:.2f},{:.2f}) motionValid={} motion=({:.2f},{:.2f},{:.2f}) best={} bestPos=({:.2f},{:.2f},{:.2f}) distBest={:.2f} distObserved={:.2f} distBodyTransform={:.2f} distMotion={:.2f} distBhk+={:.2f} distBhk-={:.2f} distHknpMin+={:.2f} distHknpMin-={:.2f} distHknpMax+={:.2f} distHknpMax-={:.2f} distHknpCenter+={:.2f} distHknpCenter-={:.2f} bhkOrigin=({:.2f},{:.2f},{:.2f}) hknpMinShift=({:.2f},{:.2f},{:.2f}) hknpMaxShift=({:.2f},{:.2f},{:.2f}) hknpCenterShift=({:.2f},{:.2f},{:.2f}) hknpBoundsMin=({:.2f},{:.2f},{:.2f}) hknpBoundsMax=({:.2f},{:.2f},{:.2f})",
                std::forward<decltype(args)>(args)...);
        };
        auto emitPayload = [&](auto&& emit) {
            emit(handLabel ? handLabel : "unknown",
                held ? "held" : "selected",
                sample.refr ? sample.refr->GetFormID() : 0,
                sample.bodyId.value,
                visualSourceKindName(sample.visualSourceKind),
                bodyPositionSourceName(sample.bodyPositionSource),
                sample.motionIndex,
                sample.bodyFlags,
                sample.filterInfo,
                sample.motionPropertiesId,
                sample.visualPositionGame.x,
                sample.visualPositionGame.y,
                sample.visualPositionGame.z,
                sample.observedBodyPositionGame.x,
                sample.observedBodyPositionGame.y,
                sample.observedBodyPositionGame.z,
                sample.bodyTransformPositionGame.x,
                sample.bodyTransformPositionGame.y,
                sample.bodyTransformPositionGame.z,
                sample.hasMotionPosition ? 1 : 0,
                sample.motionPositionGame.x,
                sample.motionPositionGame.y,
                sample.motionPositionGame.z,
                candidateKindName(sample.candidates.best.kind),
                sample.candidates.best.positionGame.x,
                sample.candidates.best.positionGame.y,
                sample.candidates.best.positionGame.z,
                sample.candidates.best.distanceToVisual,
                candidateDistance(sample.candidates, OriginCandidateKind::Raw),
                sample.bodyTransformDistanceToVisual,
                sample.motionDistanceToVisual,
                candidateDistance(sample.candidates, OriginCandidateKind::BhkStoredOriginAdd),
                candidateDistance(sample.candidates, OriginCandidateKind::BhkStoredOriginSubtract),
                candidateDistance(sample.candidates, OriginCandidateKind::HknpBroadphaseMinShiftAdd),
                candidateDistance(sample.candidates, OriginCandidateKind::HknpBroadphaseMinShiftSubtract),
                candidateDistance(sample.candidates, OriginCandidateKind::HknpBroadphaseMaxShiftAdd),
                candidateDistance(sample.candidates, OriginCandidateKind::HknpBroadphaseMaxShiftSubtract),
                candidateDistance(sample.candidates, OriginCandidateKind::HknpBroadphaseCenterShiftAdd),
                candidateDistance(sample.candidates, OriginCandidateKind::HknpBroadphaseCenterShiftSubtract),
                sample.origins.bhkStoredOriginGame.x,
                sample.origins.bhkStoredOriginGame.y,
                sample.origins.bhkStoredOriginGame.z,
                sample.origins.hknpBroadphaseMinShiftGame.x,
                sample.origins.hknpBroadphaseMinShiftGame.y,
                sample.origins.hknpBroadphaseMinShiftGame.z,
                sample.origins.hknpBroadphaseMaxShiftGame.x,
                sample.origins.hknpBroadphaseMaxShiftGame.y,
                sample.origins.hknpBroadphaseMaxShiftGame.z,
                sample.origins.hknpBroadphaseCenterShiftGame.x,
                sample.origins.hknpBroadphaseCenterShiftGame.y,
                sample.origins.hknpBroadphaseCenterShiftGame.z,
                sample.origins.hknpBroadphaseBoundsMinGame.x,
                sample.origins.hknpBroadphaseBoundsMinGame.y,
                sample.origins.hknpBroadphaseBoundsMinGame.z,
                sample.origins.hknpBroadphaseBoundsMaxGame.x,
                sample.origins.hknpBroadphaseBoundsMaxGame.y,
                sample.origins.hknpBroadphaseBoundsMaxGame.z);
        };

        if (sample.candidates.exceedsWarningThreshold) {
            emitPayload(emitWarn);
        } else {
            emitPayload(emitDebug);
        }
    }

    void publishMarkers(debug::BodyOverlayFrame& frame, const TargetOriginSample& sample)
    {
        if (!sample.valid) {
            return;
        }

        frame.drawMarkers = true;
        appendMarker(frame, debug::MarkerOverlayRole::TargetVisualOrigin, sample.visualPositionGame, sample.visualPositionGame, 3.0f, true, false);
        appendMarker(frame, debug::MarkerOverlayRole::TargetRawBodyOrigin, sample.observedBodyPositionGame, sample.observedBodyPositionGame, 2.5f, true, false);
        appendMarker(frame, debug::MarkerOverlayRole::TargetBodyTransformOrigin, sample.bodyTransformPositionGame, sample.bodyTransformPositionGame, 1.8f, true, false);
        if (sample.hasMotionPosition) {
            appendMarker(frame, debug::MarkerOverlayRole::TargetMotionOrigin, sample.motionPositionGame, sample.motionPositionGame, 1.8f, true, false);
        }
        appendMarker(frame, debug::MarkerOverlayRole::TargetBestOriginCandidate, sample.candidates.best.positionGame, sample.candidates.best.positionGame, 2.25f, true, false);
        appendMarker(frame, debug::MarkerOverlayRole::TargetOriginErrorLine, sample.visualPositionGame, sample.observedBodyPositionGame, 0.0f, false, true);
    }
}
