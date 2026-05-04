#include "physics-interaction/native/GeneratedKeyframedBodyDrive.h"

#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "RockConfig.h"

#include <cmath>

namespace frik::rock
{
    namespace
    {
        RE::hkTransformf makeHavokTransform(const RE::NiTransform& target)
        {
            RE::hkTransformf transform{};
            transform.rotation = niRotToHkTransformRotation(target.rotate);
            transform.translation = RE::NiPoint4(
                target.translate.x * gameToHavokScale(),
                target.translate.y * gameToHavokScale(),
                target.translate.z * gameToHavokScale(),
                0.0f);
            return transform;
        }

        RE::NiPoint3 havokTranslationToGamePoint(const RE::hkTransformf& target)
        {
            return RE::NiPoint3{ target.translation.x, target.translation.y, target.translation.z };
        }

        bool targetMovedFarEnoughForTeleport(const RE::NiPoint3& previous, const RE::NiPoint3& current, float thresholdGameUnits)
        {
            if (thresholdGameUnits <= 0.0f) {
                return false;
            }

            const float dx = current.x - previous.x;
            const float dy = current.y - previous.y;
            const float dz = current.z - previous.z;
            return (dx * dx + dy * dy + dz * dz) > (thresholdGameUnits * thresholdGameUnits);
        }

        void captureTargetAndBodyTelemetry(
            RE::hknpWorld* world,
            BethesdaPhysicsBody& body,
            const RE::NiTransform& target,
            const RE::hkTransformf& targetHavok,
            GeneratedKeyframedBodyDriveResult& result)
        {
            result.targetGamePosition = target.translate;
            result.targetHavokPosition = havokTranslationToGamePoint(targetHavok);

            body_frame::BodyFrameSource frameSource = body_frame::BodyFrameSource::Fallback;
            std::uint32_t motionIndex = body_frame::kFreeMotionIndex;
            RE::NiTransform liveTransform{};
            if (tryResolveLiveBodyWorldTransform(world, body.getBodyId(), liveTransform, &frameSource, &motionIndex)) {
                result.hasLiveBodyTransform = true;
                result.liveBodyGamePosition = liveTransform.translate;
                result.liveBodyFrameSource = frameSource;
                result.motionIndex = motionIndex;
                result.bodyDeltaGameUnits = body_frame::distance(liveTransform.translate, target.translate);
            }
        }

        void logGeneratedBodyDriveTelemetry(
            const GeneratedKeyframedBodyDriveResult& result,
            const havok_physics_timing::PhysicsTimingSample& timing,
            const char* ownerName,
            std::uint32_t bodyIndex,
            RE::hknpBodyId bodyId)
        {
            if (!g_rockConfig.rockDebugVerboseLogging || !result.driven) {
                return;
            }

            ROCK_LOG_SAMPLE_DEBUG(Physics,
                g_rockConfig.rockLogSampleMilliseconds,
                "Generated keyframed body drive owner={} bodyIndex={} bodyId={} targetGame=({:.2f},{:.2f},{:.2f}) targetHavok=({:.4f},{:.4f},{:.4f}) bodyGame=({:.2f},{:.2f},{:.2f}) bodyDeltaGame={:.2f} bodySource={} motion={} rawDt={:.6f} subDt={:.6f} simulatedDt={:.6f} driveDt={:.6f} substeps={} scaleG2H={:.8f} teleported={}",
                ownerName ? ownerName : "unknown",
                bodyIndex,
                bodyId.value,
                result.targetGamePosition.x,
                result.targetGamePosition.y,
                result.targetGamePosition.z,
                result.targetHavokPosition.x,
                result.targetHavokPosition.y,
                result.targetHavokPosition.z,
                result.liveBodyGamePosition.x,
                result.liveBodyGamePosition.y,
                result.liveBodyGamePosition.z,
                result.hasLiveBodyTransform ? result.bodyDeltaGameUnits : -1.0f,
                result.hasLiveBodyTransform ? body_frame::bodyFrameSourceName(result.liveBodyFrameSource) : "unreadable",
                result.motionIndex,
                timing.rawDeltaSeconds,
                timing.substepDeltaSeconds,
                timing.simulatedDeltaSeconds,
                result.driveDeltaSeconds,
                timing.substepCount,
                gameToHavokScale(),
                result.teleported ? "yes" : "no");
        }
    }

    void clearGeneratedKeyframedBodyDriveState(GeneratedKeyframedBodyDriveState& state)
    {
        state = {};
    }

    void queueGeneratedKeyframedBodyTarget(
        GeneratedKeyframedBodyDriveState& state,
        const RE::NiTransform& target,
        float teleportDistanceGameUnits)
    {
        state.pendingTarget = target;
        state.pendingTeleport = state.hasPreviousTarget &&
                                targetMovedFarEnoughForTeleport(state.previousTargetPosition, target.translate, teleportDistanceGameUnits);
        state.hasPendingTarget = true;
        ++state.queuedSequence;
    }

    bool placeGeneratedKeyframedBodyImmediately(BethesdaPhysicsBody& body, const RE::NiTransform& target)
    {
        if (!body.isValid()) {
            return false;
        }

        const bool moved = body.setTransform(makeHavokTransform(target));
        alignas(16) float zeroLinear[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
        alignas(16) float zeroAngular[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
        const bool zeroed = body.setVelocity(zeroLinear, zeroAngular);
        return moved && zeroed;
    }

    GeneratedKeyframedBodyDriveResult driveGeneratedKeyframedBody(
        RE::hknpWorld* world,
        BethesdaPhysicsBody& body,
        GeneratedKeyframedBodyDriveState& state,
        const havok_physics_timing::PhysicsTimingSample& timing,
        const char* ownerName,
        std::uint32_t bodyIndex)
    {
        GeneratedKeyframedBodyDriveResult result{};
        result.driveDeltaSeconds = havok_physics_timing::driveDeltaSeconds(timing);

        if (!state.hasPendingTarget) {
            result.skippedStale = true;
            return result;
        }

        result.attempted = true;
        if (!world || !body.isValid() || !havok_runtime::getBody(world, body.getBodyId())) {
            result.missingBody = true;
            ROCK_LOG_SAMPLE_WARN(Physics,
                1000,
                "Generated keyframed body drive missing owner={} bodyIndex={} bodyId={} world={:p}",
                ownerName ? ownerName : "unknown",
                bodyIndex,
                body.getBodyId().value,
                static_cast<void*>(world));
            return result;
        }

        const RE::hkTransformf targetHavok = makeHavokTransform(state.pendingTarget);
        captureTargetAndBodyTelemetry(world, body, state.pendingTarget, targetHavok, result);

        if (state.pendingTeleport) {
            result.teleported = placeGeneratedKeyframedBodyImmediately(body, state.pendingTarget);
            result.driven = result.teleported;
            result.placementFailed = !result.teleported;
            if (!result.teleported) {
                ROCK_LOG_SAMPLE_WARN(Physics,
                    1000,
                    "Generated keyframed body teleport placement failed owner={} bodyIndex={} bodyId={} targetGame=({:.2f},{:.2f},{:.2f}) targetHavok=({:.4f},{:.4f},{:.4f})",
                    ownerName ? ownerName : "unknown",
                    bodyIndex,
                    body.getBodyId().value,
                    result.targetGamePosition.x,
                    result.targetGamePosition.y,
                    result.targetGamePosition.z,
                    result.targetHavokPosition.x,
                    result.targetHavokPosition.y,
                    result.targetHavokPosition.z);
            }
        } else {
            const float driveDelta = result.driveDeltaSeconds;
            result.driven = body.driveToKeyFrame(targetHavok, driveDelta);
            result.nativeDriveFailed = !result.driven;
            if (!result.driven) {
                ROCK_LOG_SAMPLE_WARN(Physics,
                    1000,
                    "Generated keyframed body drive failed owner={} bodyIndex={} bodyId={} physicsDt={:.6f} targetGame=({:.2f},{:.2f},{:.2f}) targetHavok=({:.4f},{:.4f},{:.4f}) bodyDeltaGame={:.2f}",
                    ownerName ? ownerName : "unknown",
                    bodyIndex,
                    body.getBodyId().value,
                    driveDelta,
                    result.targetGamePosition.x,
                    result.targetGamePosition.y,
                    result.targetGamePosition.z,
                    result.targetHavokPosition.x,
                    result.targetHavokPosition.y,
                    result.targetHavokPosition.z,
                    result.hasLiveBodyTransform ? result.bodyDeltaGameUnits : -1.0f);
            }
        }

        logGeneratedBodyDriveTelemetry(result, timing, ownerName, bodyIndex, body.getBodyId());

        if (result.driven) {
            state.previousTargetPosition = state.pendingTarget.translate;
            state.hasPreviousTarget = true;
            state.consumedSequence = state.queuedSequence;
            state.hasPendingTarget = false;
            state.pendingTeleport = false;
        }

        return result;
    }
}
