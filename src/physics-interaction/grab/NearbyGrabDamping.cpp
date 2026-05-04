#include "physics-interaction/grab/NearbyGrabDamping.h"

#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/object/ObjectDetection.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsShapeCast.h"

#include "RE/Havok/hknpAllHitsCollector.h"
#include "RE/Havok/hknpWorld.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace rock::nearby_grab_damping
{
    namespace
    {
        bool containsBodyId(const std::vector<std::uint32_t>& bodyIds, std::uint32_t bodyId)
        {
            return std::find(bodyIds.begin(), bodyIds.end(), bodyId) != bodyIds.end();
        }

        void appendUniqueMotionRecords(const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
            PureDampingCandidateSet& candidates,
            std::unordered_set<std::uint32_t>& seenMotionIds)
        {
            for (const auto* record : bodySet.uniqueAcceptedMotionRecords()) {
                if (!record || record->bodyId == INVALID_BODY_ID || record->motionId == 0) {
                    continue;
                }

                if (!seenMotionIds.insert(record->motionId).second) {
                    candidates.add(PureDampingCandidate{ .bodyId = record->bodyId, .motionId = record->motionId, .accepted = true });
                    continue;
                }

                candidates.add(PureDampingCandidate{ .bodyId = record->bodyId, .motionId = record->motionId, .accepted = true });
            }
        }
    }

    NearbyGrabDampingState beginNearbyGrabDamping(RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        RE::TESObjectREFR* heldRef,
        const std::vector<std::uint32_t>& heldBodyIds,
        RE::NiPoint3 centerGame,
        float radiusGame,
        float durationSeconds,
        float linearDamping,
        float angularDamping,
        const object_physics_body_set::BodySetScanOptions& baseOptions)
    {
        NearbyGrabDampingState state;
        const float radius = sanitizeRadius(radiusGame);
        const float duration = sanitizeDuration(durationSeconds);
        state.linearDamping = sanitizeDamping(linearDamping);
        state.angularDamping = sanitizeDamping(angularDamping);
        state.remainingSeconds = duration;

        if (!shouldBeginRuntimeNearbyDamping(true, runtimeDampingWritesVerified(), radius, duration)) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                1000,
                "Nearby grab damping disabled: runtime motion damping write path is not verified radius={:.1f} duration={:.2f}s",
                radius,
                duration);
            return state;
        }

        if (!bhkWorld || !hknpWorld || radius <= 0.0f || duration <= 0.0f) {
            return state;
        }

        RE::hknpAllHitsCollector collector;
        physics_shape_cast::SphereCastDiagnostics diagnostics;
        if (!physics_shape_cast::castSelectionSphere(
                hknpWorld,
                physics_shape_cast::SphereCastInput{ .startGame = centerGame,
                    .directionGame = RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
                    .distanceGame = 1.0f,
                    .radiusGame = radius,
                    .collisionFilterInfo = physics_shape_cast::kSelectionQueryCollisionFilterInfo },
                collector,
                &diagnostics)) {
            ROCK_LOG_DEBUG(Hand, "Nearby grab damping skipped: sphere query failed radius={:.1f}", radius);
            return state;
        }

        PureDampingCandidateSet candidates;
        std::unordered_set<RE::TESObjectREFR*> scannedRefs;
        std::unordered_set<std::uint32_t> seenMotionIds;
        std::uint32_t rejectedHeldBodies = 0;
        std::uint32_t rejectedRefs = 0;

        auto* hits = collector.hits._data;
        for (int i = 0; i < collector.hits._size; ++i) {
            const auto bodyId = hits[i].hitBodyInfo.m_bodyId;
            if (bodyId.value == INVALID_BODY_ID || containsBodyId(heldBodyIds, bodyId.value)) {
                ++rejectedHeldBodies;
                continue;
            }

            auto* ref = resolveBodyToRef(bhkWorld, hknpWorld, bodyId);
            if (!ref || ref == heldRef) {
                ++rejectedRefs;
                continue;
            }
            if (!scannedRefs.insert(ref).second) {
                continue;
            }

            auto options = baseOptions;
            options.mode = physics_body_classifier::InteractionMode::PassivePush;
            options.heldBySameHand = &heldBodyIds;
            auto bodySet = object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, hknpWorld, ref, options);
            appendUniqueMotionRecords(bodySet, candidates, seenMotionIds);
        }

        for (const auto bodyId : candidates.uniqueAcceptedMotionBodyIds()) {
            auto* body = havok_runtime::getBody(hknpWorld, RE::hknpBodyId{ bodyId });
            if (!body || bodyId == INVALID_BODY_ID) {
                continue;
            }
            state.motions.push_back(SavedNearbyMotionDamping{
                .representativeBodyId = bodyId,
                .motionId = body->motionIndex,
                .originalLinearDamping = 0.0f,
                .originalAngularDamping = 0.0f,
                .active = true,
            });
        }

        state.active = !state.motions.empty();
        ROCK_LOG_DEBUG(Hand,
            "Nearby grab damping begin: hits={} refs={} motions={} dupMotionSkips={} rejectHeld={} rejectRef={} radius={:.1f} duration={:.2f}s linearDamp={:.2f} angularDamp={:.2f}",
            diagnostics.hitCount,
            scannedRefs.size(),
            state.motions.size(),
            candidates.duplicateMotionSkips(),
            rejectedHeldBodies,
            rejectedRefs,
            radius,
            duration,
            state.linearDamping,
            state.angularDamping);

        return state;
    }

    void tickNearbyGrabDamping(RE::hknpWorld* world, NearbyGrabDampingState& state, float deltaTime)
    {
        if (!world || !state.active) {
            return;
        }

        if (advanceTimer(state, deltaTime)) {
            restoreNearbyGrabDamping(world, state);
        }
    }

    void restoreNearbyGrabDamping(RE::hknpWorld* world, NearbyGrabDampingState& state)
    {
        if (state.motions.empty() && !state.active) {
            return;
        }

        std::uint32_t restored = 0;
        if (world) {
            for (auto& motionState : state.motions) {
                if (motionState.active) {
                    motionState.active = false;
                    ++restored;
                }
            }
        }

        ROCK_LOG_DEBUG(Hand, "Nearby grab damping restored: motions={}", restored);
        state.clear();
    }
}
