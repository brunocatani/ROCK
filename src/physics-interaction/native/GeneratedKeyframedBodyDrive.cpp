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

        RE::NiPoint3 lerpPoint(const RE::NiPoint3& from, const RE::NiPoint3& to, float alpha)
        {
            return RE::NiPoint3{
                from.x + (to.x - from.x) * alpha,
                from.y + (to.y - from.y) * alpha,
                from.z + (to.z - from.z) * alpha,
            };
        }

        float pointLength(const RE::NiPoint3& value)
        {
            return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
        }

        struct Quaternion
        {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            float w = 1.0f;
        };

        Quaternion normalize(Quaternion q)
        {
            const float length = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (length <= 0.000001f || !std::isfinite(length)) {
                return {};
            }

            const float invLength = 1.0f / length;
            q.x *= invLength;
            q.y *= invLength;
            q.z *= invLength;
            q.w *= invLength;
            return q;
        }

        float dot(const Quaternion& lhs, const Quaternion& rhs)
        {
            return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
        }

        Quaternion matrixToQuaternion(const RE::NiMatrix3& matrix)
        {
            float values[4]{};
            transform_math::niRowsToHavokQuaternion(matrix, values);
            return normalize(Quaternion{ values[0], values[1], values[2], values[3] });
        }

        RE::NiMatrix3 quaternionToMatrix(const Quaternion& q)
        {
            const Quaternion n = normalize(q);
            const float values[4] = { n.x, n.y, n.z, n.w };
            return transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(values);
        }

        Quaternion slerp(Quaternion from, Quaternion to, float alpha)
        {
            alpha = std::clamp(alpha, 0.0f, 1.0f);
            from = normalize(from);
            to = normalize(to);

            float cosine = dot(from, to);
            if (cosine < 0.0f) {
                to.x = -to.x;
                to.y = -to.y;
                to.z = -to.z;
                to.w = -to.w;
                cosine = -cosine;
            }

            if (cosine > 0.9995f) {
                return normalize(Quaternion{
                    from.x + (to.x - from.x) * alpha,
                    from.y + (to.y - from.y) * alpha,
                    from.z + (to.z - from.z) * alpha,
                    from.w + (to.w - from.w) * alpha,
                });
            }

            cosine = std::clamp(cosine, -1.0f, 1.0f);
            const float theta = std::acos(cosine);
            const float sinTheta = std::sin(theta);
            if (std::abs(sinTheta) <= 0.000001f) {
                return from;
            }

            const float fromScale = std::sin((1.0f - alpha) * theta) / sinTheta;
            const float toScale = std::sin(alpha * theta) / sinTheta;
            return normalize(Quaternion{
                from.x * fromScale + to.x * toScale,
                from.y * fromScale + to.y * toScale,
                from.z * fromScale + to.z * toScale,
                from.w * fromScale + to.w * toScale,
            });
        }

        RE::NiTransform interpolateTargetTransform(const RE::NiTransform& from, const RE::NiTransform& to, float alpha)
        {
            RE::NiTransform result = to;
            alpha = std::clamp(std::isfinite(alpha) ? alpha : 1.0f, 0.0f, 1.0f);
            result.translate = lerpPoint(from.translate, to.translate, alpha);
            result.rotate = quaternionToMatrix(slerp(matrixToQuaternion(from.rotate), matrixToQuaternion(to.rotate), alpha));
            result.scale = from.scale + (to.scale - from.scale) * alpha;
            return result;
        }

        float matrixDot(const RE::NiMatrix3& a, const RE::NiMatrix3& b)
        {
            return a.entry[0][0] * b.entry[0][0] + a.entry[0][1] * b.entry[0][1] + a.entry[0][2] * b.entry[0][2] +
                   a.entry[1][0] * b.entry[1][0] + a.entry[1][1] * b.entry[1][1] + a.entry[1][2] * b.entry[1][2] +
                   a.entry[2][0] * b.entry[2][0] + a.entry[2][1] * b.entry[2][1] + a.entry[2][2] * b.entry[2][2];
        }

        float rotationAngleRadians(const RE::NiMatrix3& previous, const RE::NiMatrix3& current)
        {
            const float trace = matrixDot(previous, current);
            const float cosine = std::clamp((trace - 1.0f) * 0.5f, -1.0f, 1.0f);
            return std::acos(cosine);
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
            const float angle = rotationAngleRadians(liveTransform.rotate, target.rotate);
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

            const float rotationRadians = rotationAngleRadians(liveTransform.rotate, target.rotate);
            result.targetToBodyRotationDegrees = std::isfinite(rotationRadians) ? radiansToDegrees(rotationRadians) : 0.0f;
            result.targetToBodyAxisXDegrees = directionDeltaDegrees(result.targetAxisXWorld, result.liveBodyAxisXWorld);
            result.targetToBodyAxisYDegrees = directionDeltaDegrees(result.targetAxisYWorld, result.liveBodyAxisYWorld);
            result.targetToBodyAxisZDegrees = directionDeltaDegrees(result.targetAxisZWorld, result.liveBodyAxisZWorld);
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
                fillRequiredVelocityTelemetry(liveTransform, target, result.driveDeltaSeconds, result);
                fillRotationReadbackTelemetry(liveTransform, target, result);
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
                    "Generated body frame compare owner={} bodyIndex={} bodyId={} phase={} substep={}/{} bodyDeltaGame={:.2f} bodyRotErr={:.2f} axisDeg=({:.1f},{:.1f},{:.1f}) targetX=({:.2f},{:.2f},{:.2f}) targetY=({:.2f},{:.2f},{:.2f}) targetZ=({:.2f},{:.2f},{:.2f}) bodyX=({:.2f},{:.2f},{:.2f}) bodyY=({:.2f},{:.2f},{:.2f}) bodyZ=({:.2f},{:.2f},{:.2f}) bodySource={} motion={} teleported={} hardSync={}",
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
                    result.hardSynced ? "yes" : "no");
            }

            if (!g_rockConfig.rockDebugVerboseLogging || !result.driven) {
                return;
            }

            ROCK_LOG_SAMPLE_DEBUG(Physics,
                g_rockConfig.rockLogSampleMilliseconds,
                "Generated keyframed body drive owner={} bodyIndex={} bodyId={} phase={} substep={}/{} progress={:.3f} targetGame=({:.2f},{:.2f},{:.2f}) targetHavok=({:.4f},{:.4f},{:.4f}) bodyGame=({:.2f},{:.2f},{:.2f}) bodyDeltaGame={:.2f} bodyRotErr={:.2f} axisDeg=({:.1f},{:.1f},{:.1f}) bodySource={} motion={} rawDt={:.6f} subDt={:.6f} simulatedDt={:.6f} driveDt={:.6f} sourceDt={:.6f} sourceAge={:.6f} predictLead={:.6f} stale={} noSourceSteps={} reqLinHavok={:.2f} reqAng={:.2f} substeps={} scaleG2H={:.8f} teleported={} hardSync={}",
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
                return interpolateTargetTransform(state.previousTarget, state.pendingTarget, timing.substepProgress);
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

    void queueGeneratedKeyframedBodyTarget(
        GeneratedKeyframedBodyDriveState& state,
        const RE::NiTransform& target,
        float sourceDeltaSeconds,
        float teleportDistanceGameUnits)
    {
        std::scoped_lock lock(state.mutex);
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
        (void)maxLinearVelocityHavok;
        (void)maxAngularVelocityRadians;
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

        result.predictionLeadSeconds = 0.0f;
        result.predicted = false;
        const bool hardSyncForVelocity = false;
        const bool immediatePlacement = state.pendingTeleport || hardSyncForVelocity;
        const RE::NiTransform target = immediatePlacement ? selectGeneratedImmediatePlacementTarget(state) : selectGeneratedDriveTarget(state, timing);
        const RE::hkTransformf targetHavok = makeHavokTransform(target);
        captureTargetAndBodyTelemetry(world, body, target, targetHavok, result);

        result.linearLimitExceeded = false;
        result.angularLimitExceeded = false;

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
