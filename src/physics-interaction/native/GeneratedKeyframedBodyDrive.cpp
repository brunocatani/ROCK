#include "physics-interaction/native/GeneratedKeyframedBodyDrive.h"

#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "RockConfig.h"

#include <algorithm>
#include <cmath>

namespace rock
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

        RE::NiPoint3 pointDelta(const RE::NiPoint3& from, const RE::NiPoint3& to)
        {
            return RE::NiPoint3{ to.x - from.x, to.y - from.y, to.z - from.z };
        }

        float pointLength(const RE::NiPoint3& value)
        {
            return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
        }

        float radiansToDegrees(float radians)
        {
            return radians * 57.29577951308232f;
        }

        RE::NiPoint3 localAxisWorld(const RE::NiMatrix3& matrix, const RE::NiPoint3& axis)
        {
            return transform_math::rotateLocalVectorToWorld(matrix, axis);
        }

        float directionLength(const RE::NiPoint3& value)
        {
            return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
        }

        RE::NiPoint3 normalizedOrZero(const RE::NiPoint3& value)
        {
            const float len = directionLength(value);
            if (len <= 0.000001f || !std::isfinite(len)) {
                return {};
            }
            const float inv = 1.0f / len;
            return RE::NiPoint3{ value.x * inv, value.y * inv, value.z * inv };
        }

        float directionDot(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            const RE::NiPoint3 a = normalizedOrZero(lhs);
            const RE::NiPoint3 b = normalizedOrZero(rhs);
            return std::clamp(a.x * b.x + a.y * b.y + a.z * b.z, -1.0f, 1.0f);
        }

        float directionDeltaDegrees(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return radiansToDegrees(std::acos(directionDot(lhs, rhs)));
        }

        void fillRequiredVelocityTelemetry(
            const RE::NiTransform& liveTransform,
            const RE::NiTransform& target,
            float driveDeltaSeconds,
            GeneratedKeyframedBodyDriveResult& result)
        {
            const float driveDelta = havok_physics_timing::isUsableDelta(driveDeltaSeconds) ? driveDeltaSeconds : havok_physics_timing::kFallbackPhysicsDeltaSeconds;
            const float linearGame = pointLength(pointDelta(liveTransform.translate, target.translate));
            result.requiredLinearVelocityHavok = linearGame * gameToHavokScale() / driveDelta;
            const float angle = generated_keyframed_body_drive_math::rotationAngleRadians(liveTransform.rotate, target.rotate);
            result.requiredAngularVelocityRadians = std::isfinite(angle) ? angle / driveDelta : 0.0f;
        }

        void fillRotationReadbackTelemetry(
            const RE::NiTransform& liveTransform,
            const RE::NiTransform& target,
            GeneratedKeyframedBodyDriveResult& result)
        {
            /*
             * Generated-body convention tests need rotation evidence, not just
             * position deltas. A capsule/box can be centered correctly while
             * its basis is transposed or 90/180 degrees wrong, so record both
             * total rotation error and local X/Y/Z axis deltas using the same
             * Ni local-vector convention as the grab-frame overlay.
             */
            result.targetAxisXWorld = localAxisWorld(target.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
            result.targetAxisYWorld = localAxisWorld(target.rotate, RE::NiPoint3{ 0.0f, 1.0f, 0.0f });
            result.targetAxisZWorld = localAxisWorld(target.rotate, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
            result.liveBodyAxisXWorld = localAxisWorld(liveTransform.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
            result.liveBodyAxisYWorld = localAxisWorld(liveTransform.rotate, RE::NiPoint3{ 0.0f, 1.0f, 0.0f });
            result.liveBodyAxisZWorld = localAxisWorld(liveTransform.rotate, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });

            const float rotationRadians = generated_keyframed_body_drive_math::rotationAngleRadians(liveTransform.rotate, target.rotate);
            result.targetToBodyRotationDegrees = std::isfinite(rotationRadians) ? radiansToDegrees(rotationRadians) : 0.0f;
            result.targetToBodyAxisXDegrees = directionDeltaDegrees(result.targetAxisXWorld, result.liveBodyAxisXWorld);
            result.targetToBodyAxisYDegrees = directionDeltaDegrees(result.targetAxisYWorld, result.liveBodyAxisYWorld);
            result.targetToBodyAxisZDegrees = directionDeltaDegrees(result.targetAxisZWorld, result.liveBodyAxisZWorld);
        }

        void fillTargetTelemetry(
            const RE::NiTransform& target,
            const RE::hkTransformf& targetHavok,
            GeneratedKeyframedBodyDriveResult& result)
        {
            result.targetGamePosition = target.translate;
            result.targetHavokPosition = havokTranslationToGamePoint(targetHavok);
        }

        void fillLiveBodyTelemetry(
            const RE::NiTransform& liveTransform,
            body_frame::BodyFrameSource frameSource,
            std::uint32_t motionIndex,
            const RE::NiTransform& target,
            GeneratedKeyframedBodyDriveResult& result)
        {
            result.hasLiveBodyTransform = true;
            result.liveBodyGamePosition = liveTransform.translate;
            result.liveBodyFrameSource = frameSource;
            result.motionIndex = motionIndex;
            result.bodyDeltaGameUnits = body_frame::distance(liveTransform.translate, target.translate);
            fillRequiredVelocityTelemetry(liveTransform, target, result.driveDeltaSeconds, result);
            fillRotationReadbackTelemetry(liveTransform, target, result);
        }

        void captureTargetAndBodyTelemetry(
            RE::hknpWorld* world,
            BethesdaPhysicsBody& body,
            const RE::NiTransform& target,
            const RE::hkTransformf& targetHavok,
            GeneratedKeyframedBodyDriveResult& result,
            RE::NiTransform* outLiveTransform = nullptr)
        {
            fillTargetTelemetry(target, targetHavok, result);
            body_frame::BodyFrameSource frameSource = body_frame::BodyFrameSource::Fallback;
            std::uint32_t motionIndex = body_frame::kFreeMotionIndex;
            RE::NiTransform liveTransform{};
            if (tryResolveLiveBodyWorldTransform(world, body.getBodyId(), liveTransform, &frameSource, &motionIndex)) {
                if (outLiveTransform) {
                    *outLiveTransform = liveTransform;
                }
                fillLiveBodyTelemetry(liveTransform, frameSource, motionIndex, target, result);
            }
        }

        const char* physicsStepPhaseName(havok_physics_timing::PhysicsStepPhase phase)
        {
            switch (phase) {
            case havok_physics_timing::PhysicsStepPhase::WholePreStep:
                return "whole-pre";
            case havok_physics_timing::PhysicsStepPhase::SubstepPreCollide:
                return "substep-pre-collide";
            default:
                return "unknown";
            }
        }

        void logGeneratedBodyDriveTelemetry(
            const GeneratedKeyframedBodyDriveResult& result,
            const havok_physics_timing::PhysicsTimingSample& timing,
            const char* ownerName,
            std::uint32_t bodyIndex,
            RE::hknpBodyId bodyId)
        {
            if (g_rockConfig.rockDebugGrabFrameLogging && result.driven && result.hasLiveBodyTransform) {
                ROCK_LOG_SAMPLE_INFO(Physics,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Generated body frame compare owner={} bodyIndex={} bodyId={} phase={} substep={}/{} bodyDeltaGame={:.2f} bodyRotErr={:.2f} axisDeg=({:.1f},{:.1f},{:.1f}) targetX=({:.2f},{:.2f},{:.2f}) targetY=({:.2f},{:.2f},{:.2f}) targetZ=({:.2f},{:.2f},{:.2f}) bodyX=({:.2f},{:.2f},{:.2f}) bodyY=({:.2f},{:.2f},{:.2f}) bodyZ=({:.2f},{:.2f},{:.2f}) bodySource={} motion={} teleported={} hardSync={} linCap={} angCap={} capAlpha={:.3f}",
                    ownerName ? ownerName : "unknown",
                    bodyIndex,
                    bodyId.value,
                    physicsStepPhaseName(timing.phase),
                    timing.substepIndex + 1,
                    timing.substepCount,
                    result.bodyDeltaGameUnits,
                    result.targetToBodyRotationDegrees,
                    result.targetToBodyAxisXDegrees,
                    result.targetToBodyAxisYDegrees,
                    result.targetToBodyAxisZDegrees,
                    result.targetAxisXWorld.x,
                    result.targetAxisXWorld.y,
                    result.targetAxisXWorld.z,
                    result.targetAxisYWorld.x,
                    result.targetAxisYWorld.y,
                    result.targetAxisYWorld.z,
                    result.targetAxisZWorld.x,
                    result.targetAxisZWorld.y,
                    result.targetAxisZWorld.z,
                    result.liveBodyAxisXWorld.x,
                    result.liveBodyAxisXWorld.y,
                    result.liveBodyAxisXWorld.z,
                    result.liveBodyAxisYWorld.x,
                    result.liveBodyAxisYWorld.y,
                    result.liveBodyAxisYWorld.z,
                    result.liveBodyAxisZWorld.x,
                    result.liveBodyAxisZWorld.y,
                    result.liveBodyAxisZWorld.z,
                    body_frame::bodyFrameSourceName(result.liveBodyFrameSource),
                    result.motionIndex,
                    result.teleported ? "yes" : "no",
                    result.hardSynced ? "yes" : "no",
                    result.linearLimitExceeded ? "yes" : "no",
                    result.angularLimitExceeded ? "yes" : "no",
                    result.targetLimitAlpha);
            }

            if (!g_rockConfig.rockDebugVerboseLogging || !result.driven) {
                return;
            }

            ROCK_LOG_SAMPLE_DEBUG(Physics,
                g_rockConfig.rockLogSampleMilliseconds,
                "Generated keyframed body drive owner={} bodyIndex={} bodyId={} phase={} substep={}/{} progress={:.3f} targetGame=({:.2f},{:.2f},{:.2f}) targetHavok=({:.4f},{:.4f},{:.4f}) bodyGame=({:.2f},{:.2f},{:.2f}) bodyDeltaGame={:.2f} bodyRotErr={:.2f} axisDeg=({:.1f},{:.1f},{:.1f}) bodySource={} motion={} rawDt={:.6f} subDt={:.6f} simulatedDt={:.6f} driveDt={:.6f} sourceDt={:.6f} sourceAge={:.6f} predictLead={:.6f} stale={} noSourceSteps={} reqLinHavok={:.2f} reqAng={:.2f} uncappedReqLinHavok={:.2f} uncappedReqAng={:.2f} linCap={} angCap={} capAlpha={:.3f} substeps={} scaleG2H={:.8f} teleported={} hardSync={}",
                ownerName ? ownerName : "unknown",
                bodyIndex,
                bodyId.value,
                physicsStepPhaseName(timing.phase),
                timing.substepIndex + 1,
                timing.substepCount,
                timing.substepProgress,
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
                result.hasLiveBodyTransform ? result.targetToBodyRotationDegrees : -1.0f,
                result.hasLiveBodyTransform ? result.targetToBodyAxisXDegrees : -1.0f,
                result.hasLiveBodyTransform ? result.targetToBodyAxisYDegrees : -1.0f,
                result.hasLiveBodyTransform ? result.targetToBodyAxisZDegrees : -1.0f,
                result.hasLiveBodyTransform ? body_frame::bodyFrameSourceName(result.liveBodyFrameSource) : "unreadable",
                result.motionIndex,
                timing.rawDeltaSeconds,
                timing.substepDeltaSeconds,
                timing.simulatedDeltaSeconds,
                result.driveDeltaSeconds,
                result.sourceDeltaSeconds,
                result.sourceAgeSeconds,
                result.predictionLeadSeconds,
                result.sourceStale ? "yes" : "no",
                result.stepsWithoutSource,
                result.requiredLinearVelocityHavok,
                result.requiredAngularVelocityRadians,
                result.uncappedRequiredLinearVelocityHavok,
                result.uncappedRequiredAngularVelocityRadians,
                result.linearLimitExceeded ? "yes" : "no",
                result.angularLimitExceeded ? "yes" : "no",
                result.targetLimitAlpha,
                timing.substepCount,
                gameToHavokScale(),
                result.teleported ? "yes" : "no",
                result.hardSynced ? "yes" : "no");
        }

        RE::NiTransform selectGeneratedDriveTarget(const GeneratedKeyframedBodyDriveState& state, const havok_physics_timing::PhysicsTimingSample& timing)
        {
            if (generated_keyframed_body_drive_math::shouldUseSubstepInterpolatedTarget(
                    timing,
                    state.hasPendingTarget,
                    state.hasPreviousTarget,
                    false)) {
                return generated_keyframed_body_drive_math::interpolateTargetTransform(state.previousTarget, state.pendingTarget, timing.substepProgress);
            }

            return state.hasPendingTarget ? state.pendingTarget : state.previousTarget;
        }

        RE::NiTransform selectGeneratedImmediatePlacementTarget(const GeneratedKeyframedBodyDriveState& state)
        {
            return state.hasPendingTarget ? state.pendingTarget : state.previousTarget;
        }

    }

    namespace
    {
        void resetGeneratedKeyframedBodyDriveStateUnlocked(GeneratedKeyframedBodyDriveState& state)
        {
            state.pendingTarget = {};
            state.previousTarget = {};
            state.sampledLinearVelocityHavok = {};
            state.sourceDeltaSeconds = havok_physics_timing::kFallbackPhysicsDeltaSeconds;
            state.secondsSinceSourceSample = generated_keyframed_body_drive_math::kMaxStaleSeconds;
            state.teleportDistanceGameUnits = 1000.0f;
            state.stepsWithoutSource = 0;
            state.queuedSequence = 0;
            state.consumedSequence = 0;
            state.hasPendingTarget = false;
            state.hasPreviousTarget = false;
            state.pendingTeleport = false;
            state.hasSampledLinearVelocityHavok = false;
        }
    }

    void clearGeneratedKeyframedBodyDriveState(GeneratedKeyframedBodyDriveState& state)
    {
        std::scoped_lock lock(state.mutex);
        resetGeneratedKeyframedBodyDriveStateUnlocked(state);
    }

    void initializeGeneratedKeyframedBodyDriveState(GeneratedKeyframedBodyDriveState& state, const RE::NiTransform& target)
    {
        std::scoped_lock lock(state.mutex);
        resetGeneratedKeyframedBodyDriveStateUnlocked(state);
        state.pendingTarget = target;
        state.previousTarget = target;
        state.hasPendingTarget = true;
        state.hasPreviousTarget = true;
        state.sourceDeltaSeconds = havok_physics_timing::kFallbackPhysicsDeltaSeconds;
        state.secondsSinceSourceSample = 0.0f;
        state.queuedSequence = 1;
        state.consumedSequence = 1;
    }

    GeneratedKeyframedBodyDriveQueueResult queueGeneratedKeyframedBodyTarget(
        GeneratedKeyframedBodyDriveState& state,
        const RE::NiTransform& target,
        float sourceDeltaSeconds,
        float teleportDistanceGameUnits)
    {
        std::scoped_lock lock(state.mutex);
        GeneratedKeyframedBodyDriveQueueResult result{};
        const float sanitizedSourceDelta = generated_keyframed_body_drive_math::sanitizeSourceDeltaSeconds(sourceDeltaSeconds);
        const bool hadReferenceTarget = state.hasPendingTarget || state.hasPreviousTarget;
        const RE::NiTransform referenceTarget = state.hasPendingTarget ? state.pendingTarget : state.previousTarget;
        state.hasSampledLinearVelocityHavok = false;
        state.sampledLinearVelocityHavok = {};
        if (hadReferenceTarget) {
            state.hasSampledLinearVelocityHavok = generated_keyframed_body_drive_math::tryComputeSampledLinearVelocityHavok(
                referenceTarget.translate,
                target.translate,
                sanitizedSourceDelta,
                gameToHavokScale(),
                state.sampledLinearVelocityHavok);
        }
        result.sampledVelocityValid = state.hasSampledLinearVelocityHavok &&
                                      std::isfinite(state.sampledLinearVelocityHavok.x) &&
                                      std::isfinite(state.sampledLinearVelocityHavok.y) &&
                                      std::isfinite(state.sampledLinearVelocityHavok.z);
        if (result.sampledVelocityValid) {
            result.sampledLinearVelocityHavok = state.sampledLinearVelocityHavok;
        }

        if (!state.hasPreviousTarget) {
            state.previousTarget = target;
            state.hasPreviousTarget = true;
        }

        state.pendingTarget = target;
        state.sourceDeltaSeconds = sanitizedSourceDelta;
        state.secondsSinceSourceSample = 0.0f;
        state.teleportDistanceGameUnits = teleportDistanceGameUnits;
        state.pendingTeleport = state.hasPreviousTarget && targetMovedFarEnoughForTeleport(state.previousTarget.translate, target.translate, teleportDistanceGameUnits);
        state.hasPendingTarget = true;
        ++state.queuedSequence;
        result.queued = true;
        result.queuedSequence = state.queuedSequence;
        return result;
    }

    GeneratedKeyframedBodyDriveSampledVelocity snapshotGeneratedKeyframedBodyDriveSampledVelocity(const GeneratedKeyframedBodyDriveState& state)
    {
        std::scoped_lock lock(state.mutex);
        GeneratedKeyframedBodyDriveSampledVelocity snapshot{};
        snapshot.velocityHavok = state.sampledLinearVelocityHavok;
        snapshot.valid = state.hasSampledLinearVelocityHavok &&
                         std::isfinite(snapshot.velocityHavok.x) &&
                         std::isfinite(snapshot.velocityHavok.y) &&
                         std::isfinite(snapshot.velocityHavok.z);
        return snapshot;
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
        std::uint32_t bodyIndex,
        float maxLinearVelocityHavok,
        float maxAngularVelocityRadians)
    {
        GeneratedKeyframedBodyDriveResult result{};
        result.driveDeltaSeconds = havok_physics_timing::driveDeltaSeconds(timing);

        std::scoped_lock lock(state.mutex);
        if (!hasGeneratedKeyframedBodyDriveTargetUnlocked(state)) {
            result.skippedStale = true;
            return result;
        }

        refreshGeneratedKeyframedBodySourceClockForDriveUnlocked(state, result.driveDeltaSeconds);
        result.sourceDeltaSeconds = state.sourceDeltaSeconds;
        result.sourceAgeSeconds = state.secondsSinceSourceSample;
        result.sourceStale = generated_keyframed_body_drive_math::sourceIsStale(state.secondsSinceSourceSample);
        result.stepsWithoutSource = state.stepsWithoutSource;

        if (result.sourceStale) {
            result.skippedStale = true;
            return result;
        }

        result.attempted = true;
        auto* liveBody = world ? havok_runtime::getBody(world, body.getBodyId()) : nullptr;
        if (!world || !body.isValid() || !liveBody) {
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

        // driveToKeyFrame enters native bhk code through the wrapper collision
        // object. The hknp body slot must still point back to that object so a
        // recycled body id cannot drive a stale wrapper.
        auto* expectedCollisionObject = body.getCollisionObject();
        auto* liveCollisionObject = havok_runtime::getCollisionObjectFromBody(liveBody);
        if (!expectedCollisionObject || liveCollisionObject != expectedCollisionObject) {
            result.bodyCollisionObjectMismatch = true;
            ROCK_LOG_SAMPLE_WARN(Physics,
                1000,
                "Generated keyframed body drive ownership mismatch owner={} bodyIndex={} bodyId={} expectedCollObj={:p} liveCollObj={:p}",
                ownerName ? ownerName : "unknown",
                bodyIndex,
                body.getBodyId().value,
                static_cast<void*>(expectedCollisionObject),
                static_cast<void*>(liveCollisionObject));
            return result;
        }

        result.predictionLeadSeconds = 0.0f;
        result.predicted = false;
        const bool hardSyncForVelocity = false;
        const bool immediatePlacement = state.pendingTeleport || hardSyncForVelocity;
        const RE::NiTransform requestedTarget = immediatePlacement ? selectGeneratedImmediatePlacementTarget(state) : selectGeneratedDriveTarget(state, timing);
        RE::NiTransform target = requestedTarget;
        RE::hkTransformf targetHavok = makeHavokTransform(target);
        RE::NiTransform liveTransform{};
        captureTargetAndBodyTelemetry(world, body, target, targetHavok, result, &liveTransform);
        result.uncappedRequiredLinearVelocityHavok = result.requiredLinearVelocityHavok;
        result.uncappedRequiredAngularVelocityRadians = result.requiredAngularVelocityRadians;

        result.linearLimitExceeded = false;
        result.angularLimitExceeded = false;
        result.targetLimitAlpha = 1.0f;

        if (!immediatePlacement && result.hasLiveBodyTransform) {
            const auto limitedTarget = generated_keyframed_body_drive_math::limitGeneratedDriveTarget(
                liveTransform,
                requestedTarget,
                result.driveDeltaSeconds,
                gameToHavokScale(),
                maxLinearVelocityHavok,
                maxAngularVelocityRadians);
            result.linearLimitExceeded = limitedTarget.limit.linearLimitExceeded;
            result.angularLimitExceeded = limitedTarget.limit.angularLimitExceeded;
            result.targetLimitAlpha = limitedTarget.limit.alpha;

            if (result.linearLimitExceeded || result.angularLimitExceeded) {
                target = limitedTarget.target;
                targetHavok = makeHavokTransform(target);
                fillTargetTelemetry(target, targetHavok, result);
                fillLiveBodyTelemetry(liveTransform, result.liveBodyFrameSource, result.motionIndex, target, result);
            }
        }

        if (immediatePlacement) {
            result.hardSynced = hardSyncForVelocity;
            result.teleported = placeGeneratedKeyframedBodyImmediately(body, target);
            result.driven = result.teleported;
            result.placementFailed = !result.teleported;
            if (!result.teleported) {
                ROCK_LOG_SAMPLE_WARN(Physics,
                    1000,
                    "Generated keyframed body immediate placement failed owner={} bodyIndex={} bodyId={} teleport={} hardSync={} targetGame=({:.2f},{:.2f},{:.2f}) targetHavok=({:.4f},{:.4f},{:.4f})",
                    ownerName ? ownerName : "unknown",
                    bodyIndex,
                    body.getBodyId().value,
                    state.pendingTeleport ? "yes" : "no",
                    hardSyncForVelocity ? "yes" : "no",
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
                    "Generated keyframed body drive failed owner={} bodyIndex={} bodyId={} physicsDt={:.6f} sourceAge={:.6f} predictLead={:.6f} targetGame=({:.2f},{:.2f},{:.2f}) targetHavok=({:.4f},{:.4f},{:.4f}) bodyDeltaGame={:.2f}",
                    ownerName ? ownerName : "unknown",
                    bodyIndex,
                    body.getBodyId().value,
                    driveDelta,
                    result.sourceAgeSeconds,
                    result.predictionLeadSeconds,
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
            markGeneratedKeyframedBodyDrivePlacedUnlocked(
                state,
                result.driveDeltaSeconds,
                generated_keyframed_body_drive_math::shouldFinalizePlacedTargetForNextSource(timing, immediatePlacement));
        }

        return result;
    }
}
