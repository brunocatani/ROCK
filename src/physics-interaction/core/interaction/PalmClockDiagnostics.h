// The PALM_CLOCK diagnostic row: how far the raw hand, the queued palm target
// and the live palm body are from each other at one authority boundary.
// Logs only, and only with verbose logging on.
#pragma once

#include "RockConfig.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/native/havok/HavokPhysicsTiming.h"
#include "physics-interaction/native/havok/PhysicsUtils.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>

namespace rock
{
    namespace physics_interaction_internal
    {
        struct TransformDelta
        {
            float position = 0.0f;
            float rotationDegrees = 0.0f;
        };

        inline TransformDelta measureTransformDelta(const RE::NiTransform& a, const RE::NiTransform& b)
        {
            const float dx = a.translate.x - b.translate.x;
            const float dy = a.translate.y - b.translate.y;
            const float dz = a.translate.z - b.translate.z;

            const auto qa = niRotToHkQuat(a.rotate);
            const auto qb = niRotToHkQuat(b.rotate);
            const float dot = std::clamp(std::fabs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w), 0.0f, 1.0f);
            const float angleRadians = 2.0f * std::acos(dot);

            return TransformDelta{ .position = std::sqrt(dx * dx + dy * dy + dz * dz),
                .rotationDegrees = angleRadians * (180.0f / std::numbers::pi_v<float>)};
        }

        inline const char* physicsStepPhaseName(havok_physics_timing::PhysicsStepPhase phase)
        {
            switch (phase) {
            case havok_physics_timing::PhysicsStepPhase::WholePreStep:
                return "whole-pre";
            case havok_physics_timing::PhysicsStepPhase::SubstepPreCollide:
                return "substep-pre-collide";
            case havok_physics_timing::PhysicsStepPhase::BetweenCollideAndSolve:
                return "between-collide-solve";
            case havok_physics_timing::PhysicsStepPhase::SubstepPostSolve:
                return "substep-post-solve";
            }
            return "unknown";
        }

        /*
         * Palm clock diagnostics compare the game-frame queued skeleton target to
         * the live palm body at each authority boundary. This keeps rate-mismatch
         * evidence in one log row without changing grab authority.
         */
        inline void logPalmClockSampleForHand(const char* stage, const Hand& hand, RE::hknpWorld* world,
            const RE::NiTransform* rawHandWorld, std::uint64_t gameFrameIndex, float gameDeltaSeconds,
            const havok_physics_timing::PhysicsTimingSample* timing)
        {
            if (!g_rockConfig.rockDebugVerboseLogging) {
                return;
            }

            RE::NiTransform palmTargetWorld{};
            const bool targetOk = hand.tryGetPalmAnchorTarget(palmTargetWorld);
            Hand::LivePalmAnchorReference livePalm{};
            const bool liveOk = hand.tryResolveLivePalmAnchorReference(world, livePalm);
            if (!targetOk && !liveOk && !rawHandWorld) {
                return;
            }

            const bool rawOk = rawHandWorld != nullptr;
            const TransformDelta rawToTarget = (rawOk && targetOk)
                ? measureTransformDelta(*rawHandWorld, palmTargetWorld)
                : TransformDelta{ -1.0f, -1.0f };
            const TransformDelta rawToLive = (rawOk && liveOk) ? measureTransformDelta(*rawHandWorld, livePalm.world)
                                                               : TransformDelta{ -1.0f, -1.0f };
            const TransformDelta targetToLive = (targetOk && liveOk)
                ? measureTransformDelta(palmTargetWorld, livePalm.world)
                : TransformDelta{ -1.0f, -1.0f };

            const float rawDt = timing ? timing->rawDeltaSeconds : -1.0f;
            const float subDt = timing ? timing->substepDeltaSeconds : -1.0f;
            float driveDt = -1.0f;
            if (timing) {
                (void)havok_physics_timing::tryGetDriveDeltaSeconds(*timing, driveDt);
            }
            const float progress = timing ? timing->substepProgress : -1.0f;
            const std::uint32_t substepIndex = timing ? timing->substepIndex + 1 : 0;
            const std::uint32_t substepCount = timing ? timing->substepCount : 0;
            const char* physicsPhase = timing ? physicsStepPhaseName(timing->phase) : "game-frame";

            auto emit = [&]() {
                const char* rawState = rawOk ? "ok" : "none";
                const char* targetState = targetOk ? "ok" : "none";
                const char* liveState = liveOk ? "ok" : "none";
                const std::uint32_t palmMotion = liveOk ? livePalm.motionIndex : body_frame::kFreeMotionIndex;
                const RE::NiPoint3 rawPosition = rawOk ? rawHandWorld->translate : RE::NiPoint3{};
                const RE::NiPoint3 targetPosition = targetOk ? palmTargetWorld.translate : RE::NiPoint3{};
                const RE::NiPoint3 livePosition = liveOk ? livePalm.world.translate : RE::NiPoint3{};

                ROCK_LOG_SAMPLE_DEBUG(Hand, g_rockConfig.rockLogSampleMilliseconds,
                    "PALM_CLOCK stage={} hand={} frame={} holding={} raw={} target={} live={} body={} proxyBody={} "
                    "gameDt={:.6f} physicsPhase={} rawDt={:.6f} subDt={:.6f} driveDt={:.6f} substep={}/{} "
                    "progress={:.3f} rawToTarget={:.3f}gu/{:.3f}deg rawToLive={:.3f}gu/{:.3f}deg "
                    "targetToLive={:.3f}gu/{:.3f}deg rawPos=({:.2f},{:.2f},{:.2f}) targetPos=({:.2f},{:.2f},{:.2f}) "
                    "livePos=({:.2f},{:.2f},{:.2f}) liveMotion={}",
                    stage ? stage : "unknown", hand.handName(), gameFrameIndex, hand.isHoldingAtomic() ? "yes" : "no",
                    rawState, targetState, liveState, hand.getCollisionBodyId().value,
                    hand.getGrabAuthorityProxyBodyId().value, gameDeltaSeconds, physicsPhase, rawDt, subDt, driveDt,
                    substepIndex, substepCount, progress, rawToTarget.position, rawToTarget.rotationDegrees,
                    rawToLive.position, rawToLive.rotationDegrees, targetToLive.position, targetToLive.rotationDegrees,
                    rawPosition.x, rawPosition.y, rawPosition.z, targetPosition.x, targetPosition.y, targetPosition.z,
                    livePosition.x, livePosition.y, livePosition.z, palmMotion);
            };

            emit();
        }
    }
}
