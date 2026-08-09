#include "physics-interaction/weapon/DynamicWeaponCollision.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/native/HavokConvexShapeBuilder.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/PhysicsCallbackQuiescenceGate.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/WeaponCollision.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace rock
{
    namespace
    {
        constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;
        constexpr std::uint32_t kDynamicWeaponCollisionGroup = 0x000Du;
        constexpr std::uint32_t kRaiseManifoldProcessedEvents = 0x40u;
        constexpr std::uint32_t kRebuildBodyCollisionState = 0u;
        constexpr std::uint32_t kContactGraceSolves = 3;
        constexpr float kMaxVisualCorrectionRotationDegrees = 85.0f;
        constexpr float kSurfaceContactTranslationBiasGameUnits = 0.25f;
        constexpr float kSurfaceContactRotationBiasDegrees = 0.5f;
        constexpr float kSurfaceRecoveryHalfLifeSeconds = 0.2f;
        constexpr float kSurfaceRecoveryMaxOpposingRawMotionFraction = 0.25f;

        std::uint32_t dynamicWeaponProxyFilterInfo()
        {
            return (kDynamicWeaponCollisionGroup << 16) |
                   (collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY & collision_layer_policy::FO4_LAYER_FILTER_MASK);
        }

        float pointDistance(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            const float x = lhs.x - rhs.x;
            const float y = lhs.y - rhs.y;
            const float z = lhs.z - rhs.z;
            return std::sqrt(x * x + y * y + z * z);
        }

    }

    void DynamicWeaponCollisionRuntime::beginFrame(
        const std::uint64_t frameIndex,
        RE::hknpWorld* world,
        void* bhkWorld,
        RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const bool enabled,
        const bool suppressDefaultNativeIntent)
    {
        _frameIndex = frameIndex;
        _frameWorld = world;
        _frameBhkWorld = bhkWorld;
        _frameWeaponNode = weaponNode;
        _frameGenerationKey = weaponGenerationKey;
        _frameRequestedWeaponWorld = {};
        _frameResetRawMotionBaseline = false;
        _frameAcceptingIntent = enabled && world && bhkWorld && weaponNode && weaponGenerationKey != 0;
        _frameHasIntent =
            _frameAcceptingIntent &&
            !suppressDefaultNativeIntent &&
            dynamic_weapon_collision_policy::isFiniteTransform(weaponNode->world);
        if (_frameHasIntent) {
            // Native/F4SE visual authority is the collision-free intent when
            // ROCK does not publish a later manual-grip or gunstock pose.
            _frameRequestedWeaponWorld = weaponNode->world;
        }
        _enabledAtomic.store(_frameAcceptingIntent, std::memory_order_release);
    }

    void DynamicWeaponCollisionRuntime::observeWeaponVisualIntent(
        void* context,
        RE::NiNode* weaponNode,
        const RE::NiTransform& requestedWeaponWorld,
        const std::uint64_t weaponGenerationKey,
        const bool resetMotionBaseline)
    {
        auto* runtime = static_cast<DynamicWeaponCollisionRuntime*>(context);
        if (runtime) {
            runtime->captureVisualIntent(
                weaponNode,
                requestedWeaponWorld,
                weaponGenerationKey,
                resetMotionBaseline);
        }
    }

    void DynamicWeaponCollisionRuntime::captureVisualIntent(
        RE::NiNode* weaponNode,
        const RE::NiTransform& requestedWeaponWorld,
        const std::uint64_t weaponGenerationKey,
        const bool resetMotionBaseline)
    {
        if (!_frameAcceptingIntent || weaponNode != _frameWeaponNode || weaponGenerationKey != _frameGenerationKey ||
            !dynamic_weapon_collision_policy::isFiniteTransform(requestedWeaponWorld)) {
            return;
        }
        _frameRequestedWeaponWorld = requestedWeaponWorld;
        _frameHasIntent = true;
        _frameResetRawMotionBaseline =
            _frameResetRawMotionBaseline || resetMotionBaseline;
    }

    DynamicWeaponCollisionRuntime::FrameResult DynamicWeaponCollisionRuntime::finishFrame(
        const PhysicsFrameContext& frame,
        const bool physicsWritesAllowed,
        RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const WeaponCollision& weaponCollision)
    {
        FrameResult result{};
        result.requestedWeaponWorld = _frameRequestedWeaponWorld;
        result.resolvedWeaponWorld = _frameRequestedWeaponWorld;

        const bool frameMatches =
            _frameAcceptingIntent &&
            _frameHasIntent &&
            physicsWritesAllowed &&
            frame.worldReady &&
            frame.hknpWorld == _frameWorld &&
            frame.bhkWorld == _frameBhkWorld &&
            weaponNode == _frameWeaponNode &&
            weaponGenerationKey == _frameGenerationKey &&
            !frame.menuBlocked;
        if (!frameMatches || !ensureProxyBody(frame, weaponCollision, _frameRequestedWeaponWorld)) {
            if (_created) {
                retireAll(frame.bhkWorld);
            }
            _debugSnapshot = {};
            return result;
        }

        result.proxyActive = true;
        const RE::NiTransform rawRequestedBodyTarget = dynamic_weapon_collision_policy::makeProxyBodyTarget(
            _frameRequestedWeaponWorld,
            _createdCenterWeaponLocal);
        if (_frameResetRawMotionBaseline) {
            /*
             * A visual-authority handoff can replace the collision-free pose
             * source without representing physical controller motion. Rebase
             * the raw delta origin so the clutch follows the return overlay on
             * subsequent frames instead of interpreting the source switch as
             * a one-frame wall drive.
             */
            _previousRawProxyBodyTarget = rawRequestedBodyTarget;
            _previousRawProxyBodyTargetValid = true;
            ROCK_LOG_DEBUG(
                Weapon,
                "Dynamic weapon raw intent motion baseline reset: body={} generation={:016X}",
                _body.getBodyId().value,
                _createdGenerationKey);
        }

        PhysicsSnapshot snapshot{};
        const bool snapshotReadable = readPhysicsSnapshot(snapshot);
        const bool snapshotIdentityCurrent =
            snapshotReadable &&
            snapshot.valid &&
            snapshot.world == reinterpret_cast<std::uintptr_t>(frame.hknpWorld) &&
            snapshot.bodyId == _body.getBodyId().value &&
            snapshot.generationKey == _createdGenerationKey;

        if (!snapshotIdentityCurrent || snapshot.teleported) {
            _surfaceClutchActive = false;
            _surfaceClutchTargetValid = false;
        }

        RE::NiTransform requestedBodyTarget = rawRequestedBodyTarget;
        if (snapshotIdentityCurrent && snapshot.contactActive) {
            const RE::NiTransform contactAnchor = dynamic_weapon_collision_policy::makeBoundedContactAnchor(
                snapshot.liveProxyBodyWorld,
                snapshot.requestedProxyBodyWorld,
                kSurfaceContactTranslationBiasGameUnits,
                kSurfaceContactRotationBiasDegrees);
            requestedBodyTarget = _previousRawProxyBodyTargetValid ?
                dynamic_weapon_collision_policy::advanceSurfaceCoupledTarget(
                    _previousRawProxyBodyTarget,
                    rawRequestedBodyTarget,
                    contactAnchor) :
                contactAnchor;
            _surfaceClutchActive = dynamic_weapon_collision_policy::isFiniteTransform(requestedBodyTarget);
            _surfaceClutchTargetValid = _surfaceClutchActive;
            if (_surfaceClutchTargetValid) {
                _surfaceClutchProxyBodyTarget = requestedBodyTarget;
            } else {
                requestedBodyTarget = rawRequestedBodyTarget;
            }
        } else if (snapshotIdentityCurrent &&
                   _surfaceClutchActive &&
                   _surfaceClutchTargetValid &&
                   _previousRawProxyBodyTargetValid) {
            /*
             * Preserve the collision-local offset while following every raw
             * controller delta, then remove that offset continuously. Recovery
             * never waits for controller stillness. When it points against the
             * user's current translation, it can consume at most one quarter
             * of that motion, so withdrawal remains immediate and monotonic.
             */
            const RE::NiTransform followedTarget = dynamic_weapon_collision_policy::advanceSurfaceCoupledTarget(
                _previousRawProxyBodyTarget,
                rawRequestedBodyTarget,
                _surfaceClutchProxyBodyTarget);
            const auto recovery = dynamic_weapon_collision_policy::recoverSurfaceCoupledTarget(
                followedTarget,
                _previousRawProxyBodyTarget,
                rawRequestedBodyTarget,
                frame.deltaSeconds,
                kSurfaceRecoveryHalfLifeSeconds,
                kSurfaceRecoveryMaxOpposingRawMotionFraction);
            requestedBodyTarget = recovery.target;
            if (!dynamic_weapon_collision_policy::isFiniteTransform(requestedBodyTarget)) {
                _surfaceClutchActive = false;
                _surfaceClutchTargetValid = false;
                requestedBodyTarget = rawRequestedBodyTarget;
            } else {
                _surfaceClutchProxyBodyTarget = requestedBodyTarget;
                const bool targetConverged =
                    dynamic_weapon_collision_policy::translationDeltaGameUnits(
                        requestedBodyTarget,
                        rawRequestedBodyTarget) <= g_rockConfig.rockWeaponCollisionDynamicRenderMinTranslationGameUnits &&
                    dynamic_weapon_collision_policy::rotationDeltaDegrees(
                        requestedBodyTarget,
                        rawRequestedBodyTarget) <= g_rockConfig.rockWeaponCollisionDynamicRenderMinRotationDegrees;
                const bool liveConverged =
                    dynamic_weapon_collision_policy::translationDeltaGameUnits(
                        snapshot.liveProxyBodyWorld,
                        rawRequestedBodyTarget) <= g_rockConfig.rockWeaponCollisionDynamicRenderMinTranslationGameUnits &&
                    dynamic_weapon_collision_policy::rotationDeltaDegrees(
                        snapshot.liveProxyBodyWorld,
                        rawRequestedBodyTarget) <= g_rockConfig.rockWeaponCollisionDynamicRenderMinRotationDegrees;
                if (targetConverged && liveConverged) {
                    _surfaceClutchActive = false;
                    _surfaceClutchTargetValid = false;
                    requestedBodyTarget = rawRequestedBodyTarget;
                }
            }
        }

        const auto queueResult = queueGeneratedKeyframedBodyTarget(
            _driveState,
            requestedBodyTarget,
            frame.deltaSeconds,
            g_rockConfig.rockWeaponCollisionDynamicDivergenceTeleportGameUnits);
        _previousRawProxyBodyTarget = rawRequestedBodyTarget;
        _previousRawProxyBodyTargetValid = true;
        if (!queueResult.queued) {
            _rebuildRequestedAtomic.store(true, std::memory_order_release);
        }

        const bool snapshotCurrent =
            snapshotIdentityCurrent &&
            (snapshot.contactActive || _surfaceClutchActive) &&
            !snapshot.teleported;

        _debugSnapshot = {};
        _debugSnapshot.valid = true;
        _debugSnapshot.physicsSnapshotReadable = snapshotReadable;
        _debugSnapshot.physicsSnapshotValid = snapshotReadable && snapshot.valid;
        _debugSnapshot.physicsSnapshotIdentityCurrent = snapshotIdentityCurrent;
        _debugSnapshot.physicsSnapshotContactActive = snapshotReadable && snapshot.contactActive;
        _debugSnapshot.physicsSnapshotTeleported = snapshotReadable && snapshot.teleported;
        _debugSnapshot.surfaceClutchActive = _surfaceClutchActive;
        _debugSnapshot.bodyId = _body.getBodyId().value;
        _debugSnapshot.generationKey = _createdGenerationKey;
        _debugSnapshot.proxyPairCallbackSequence = _proxyPairCallbackSequenceAtomic.load(std::memory_order_acquire);
        _debugSnapshot.worldSurfaceCallbackSequence = _worldSurfaceCallbackSequenceAtomic.load(std::memory_order_acquire);
        _debugSnapshot.rawPointCallbackSequence = _rawPointCallbackSequenceAtomic.load(std::memory_order_acquire);
        _debugSnapshot.processedManifoldCallbackSequence = _processedManifoldCallbackSequenceAtomic.load(std::memory_order_acquire);
        _debugSnapshot.admittedContactSequence = _contactSequenceAtomic.load(std::memory_order_acquire);
        _debugSnapshot.centerWeaponLocal = _createdCenterWeaponLocal;
        _debugSnapshot.halfExtentsWeaponLocal = _createdHalfExtentsWeaponLocal;
        _debugSnapshot.requestedWeaponWorld = _frameRequestedWeaponWorld;

        const auto logPipelineStage = [&](const char* stage) {
            if (!g_rockConfig.rockDebugDrawDynamicWeaponColliders) {
                return;
            }
            ROCK_LOG_SAMPLE_INFO(
                Weapon,
                500,
                "DWC pipeline: stage={} body={} callbacks(pair/world/raw/manifold/admit)={}/{}/{}/{}/{} snapshot(read/valid/identity/contact/teleport)={}/{}/{}/{}/{} clutch={} correction=({:.2f}gu,{:.2f}deg) visual={}",
                stage,
                _debugSnapshot.bodyId,
                _debugSnapshot.proxyPairCallbackSequence,
                _debugSnapshot.worldSurfaceCallbackSequence,
                _debugSnapshot.rawPointCallbackSequence,
                _debugSnapshot.processedManifoldCallbackSequence,
                _debugSnapshot.admittedContactSequence,
                _debugSnapshot.physicsSnapshotReadable,
                _debugSnapshot.physicsSnapshotValid,
                _debugSnapshot.physicsSnapshotIdentityCurrent,
                _debugSnapshot.physicsSnapshotContactActive,
                _debugSnapshot.physicsSnapshotTeleported,
                _debugSnapshot.surfaceClutchActive,
                result.translationCorrectionGameUnits,
                result.rotationCorrectionDegrees,
                result.applyVisualCorrection);
        };

        if (!snapshotCurrent) {
            logPipelineStage("snapshot-gate");
            return result;
        }

        const RE::NiTransform sampledRequestedWeaponWorld = dynamic_weapon_collision_policy::reconstructWeaponRoot(
            snapshot.requestedProxyBodyWorld,
            _createdCenterWeaponLocal,
            snapshot.weaponScale);
        const RE::NiTransform sampledLiveWeaponWorld = dynamic_weapon_collision_policy::reconstructWeaponRoot(
            snapshot.liveProxyBodyWorld,
            _createdCenterWeaponLocal,
            snapshot.weaponScale);
        const RE::NiTransform resolvedWeaponWorld = _surfaceClutchActive ?
            sampledLiveWeaponWorld :
            dynamic_weapon_collision_policy::resolveCurrentIntentFromSample(
                snapshot.requestedProxyBodyWorld,
                snapshot.liveProxyBodyWorld,
                _createdCenterWeaponLocal,
                snapshot.weaponScale,
                _frameRequestedWeaponWorld);
        if (!dynamic_weapon_collision_policy::isFiniteTransform(sampledRequestedWeaponWorld) ||
            !dynamic_weapon_collision_policy::isFiniteTransform(sampledLiveWeaponWorld) ||
            !dynamic_weapon_collision_policy::isFiniteTransform(resolvedWeaponWorld)) {
            logPipelineStage("transform-gate");
            return result;
        }

        result.translationCorrectionGameUnits = dynamic_weapon_collision_policy::translationDeltaGameUnits(
            resolvedWeaponWorld,
            _frameRequestedWeaponWorld);
        result.rotationCorrectionDegrees = dynamic_weapon_collision_policy::rotationDeltaDegrees(
            resolvedWeaponWorld,
            _frameRequestedWeaponWorld);
        _debugSnapshot.contactActive = snapshot.contactActive;
        _debugSnapshot.otherBodyId = snapshot.otherBodyId;
        _debugSnapshot.otherLayer = snapshot.otherLayer;
        _debugSnapshot.contactGraceSolves = snapshot.contactGraceSolves;
        _debugSnapshot.solveSequence = snapshot.solveSequence;
        _debugSnapshot.liveWeaponWorld = sampledLiveWeaponWorld;
        _debugSnapshot.resolvedWeaponWorld = resolvedWeaponWorld;
        _debugSnapshot.translationCorrectionGameUnits = result.translationCorrectionGameUnits;
        _debugSnapshot.rotationCorrectionDegrees = result.rotationCorrectionDegrees;

        const float safetyTranslationCorrectionGameUnits = _surfaceClutchActive ?
            dynamic_weapon_collision_policy::translationDeltaGameUnits(
                snapshot.liveProxyBodyWorld,
                requestedBodyTarget) :
            result.translationCorrectionGameUnits;
        const float safetyRotationCorrectionDegrees = _surfaceClutchActive ?
            dynamic_weapon_collision_policy::rotationDeltaDegrees(
                snapshot.liveProxyBodyWorld,
                requestedBodyTarget) :
            result.rotationCorrectionDegrees;
        const bool correctionWithinSafetyEnvelope =
            std::isfinite(result.translationCorrectionGameUnits) &&
            std::isfinite(result.rotationCorrectionDegrees) &&
            std::isfinite(safetyTranslationCorrectionGameUnits) &&
            std::isfinite(safetyRotationCorrectionDegrees) &&
            safetyTranslationCorrectionGameUnits <= g_rockConfig.rockWeaponCollisionDynamicMaxVisualCorrectionGameUnits &&
            safetyRotationCorrectionDegrees <= kMaxVisualCorrectionRotationDegrees;
        if (!correctionWithinSafetyEnvelope) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "Dynamic weapon visual correction rejected: body={} requested=({:.2f}gu,{:.2f}deg) solver=({:.2f}gu,{:.2f}deg) clutch={}",
                snapshot.bodyId,
                result.translationCorrectionGameUnits,
                result.rotationCorrectionDegrees,
                safetyTranslationCorrectionGameUnits,
                safetyRotationCorrectionDegrees,
                _surfaceClutchActive);
            logPipelineStage("safety-gate");
            return result;
        }

        const bool correctionVisible =
            result.translationCorrectionGameUnits >= g_rockConfig.rockWeaponCollisionDynamicRenderMinTranslationGameUnits ||
            result.rotationCorrectionDegrees >= g_rockConfig.rockWeaponCollisionDynamicRenderMinRotationDegrees;
        if (correctionVisible) {
            result.applyVisualCorrection = true;
            result.resolvedWeaponWorld = resolvedWeaponWorld;
            _debugSnapshot.visualCorrectionActive = true;
        }
        logPipelineStage(correctionVisible ? "publish-requested" : "visibility-gate");
        return result;
    }

    bool DynamicWeaponCollisionRuntime::ensureProxyBody(
        const PhysicsFrameContext& frame,
        const WeaponCollision& weaponCollision,
        const RE::NiTransform& requestedWeaponWorld)
    {
        WeaponCollision::ApproximateBoundsSnapshot bounds{};
        if (!weaponCollision.getApproximateBoundsSnapshot(bounds) || !bounds.valid ||
            bounds.generationKey != _frameGenerationKey ||
            !dynamic_weapon_collision_policy::isFiniteTransform(requestedWeaponWorld)) {
            return false;
        }
        const auto geometry = dynamic_weapon_collision_policy::makeBoxGeometry(bounds.minWeaponLocal, bounds.maxWeaponLocal);
        if (!geometry.valid) {
            return false;
        }

        const float scale = std::abs(requestedWeaponWorld.scale);
        const float padding = g_rockConfig.rockWeaponCollisionDynamicBoxPaddingGameUnits;
        const bool bodyMatches =
            _created &&
            _body.isValid() &&
            _createdWorld == frame.hknpWorld &&
            _createdBhkWorld == frame.bhkWorld &&
            _createdGenerationKey == bounds.generationKey &&
            std::abs(_createdWeaponScale - scale) <= 0.0001f &&
            std::abs(_createdPaddingGameUnits - padding) <= 0.0001f &&
            !_rebuildRequestedAtomic.load(std::memory_order_acquire);
        if (bodyMatches) {
            return true;
        }

        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        if (_created) {
            retireProxyLocked(frame.bhkWorld);
        }

        const auto corners = dynamic_weapon_collision_policy::makeBoxCornerPointsHavok(
            geometry,
            scale,
            padding,
            physics_scale::gameToHavok());
        std::vector<RE::NiPoint3> pointCloud(corners.begin(), corners.end());
        auto* shape = havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(
            pointCloud,
            (std::max)(0.0f, g_rockConfig.rockWeaponCollisionConvexRadius));
        if (!shape) {
            return false;
        }

        const RE::NiTransform initialTarget = dynamic_weapon_collision_policy::makeProxyBodyTarget(
            requestedWeaponWorld,
            geometry.centerWeaponLocal);
        if (!_body.create(
                frame.hknpWorld,
                frame.bhkWorld,
                shape,
                dynamicWeaponProxyFilterInfo(),
                havok_material_registry::registerGeneratedBodyMaterial(frame.hknpWorld),
                BethesdaMotionType::Dynamic,
                "ROCK_DynamicWeaponBox")) {
            havok_ref_count::release(shape);
            return false;
        }

        const auto proxyBodyId = _body.getBodyId();
        _bodyIdAtomic.store(proxyBodyId.value, std::memory_order_release);
        const bool processedManifoldFlagEnabled = havok_runtime::enableBodyFlags(
            frame.hknpWorld,
            proxyBodyId.value,
            kRaiseManifoldProcessedEvents,
            kRebuildBodyCollisionState);
        const auto flaggedBody = havok_runtime::snapshotBody(frame.hknpWorld, proxyBodyId);
        const bool processedManifoldFlagPublished =
            processedManifoldFlagEnabled &&
            flaggedBody.valid &&
            flaggedBody.body &&
            (flaggedBody.body->flags & kRaiseManifoldProcessedEvents) == kRaiseManifoldProcessedEvents;
        if (!processedManifoldFlagPublished) {
            ROCK_LOG_ERROR(
                Weapon,
                "Dynamic weapon box {} failed processed-manifold event opt-in: enabled={} readable={} flags=0x{:08X}",
                proxyBodyId.value,
                processedManifoldFlagEnabled,
                flaggedBody.valid && flaggedBody.body,
                flaggedBody.body ? flaggedBody.body->flags : 0u);
            _bodyIdAtomic.store(kInvalidBodyId, std::memory_order_release);
            _body.retireDeferred(frame.bhkWorld);
            havok_ref_count::release(shape);
            return false;
        }

        _shape = shape;
        _createdWorld = frame.hknpWorld;
        _createdBhkWorld = frame.bhkWorld;
        _createdGenerationKey = bounds.generationKey;
        _createdCenterWeaponLocal = geometry.centerWeaponLocal;
        _createdHalfExtentsWeaponLocal = geometry.halfExtentsWeaponLocal;
        _createdWeaponScale = scale;
        _createdPaddingGameUnits = padding;
        _created = true;
        _rebuildRequestedAtomic.store(false, std::memory_order_release);
        initializeGeneratedKeyframedBodyDriveState(_driveState, initialTarget);
        _previousRawProxyBodyTarget = initialTarget;
        _surfaceClutchProxyBodyTarget = initialTarget;
        _previousRawProxyBodyTargetValid = true;
        _surfaceClutchTargetValid = false;
        _surfaceClutchActive = false;
        if (!placeGeneratedKeyframedBodyImmediately(_body, initialTarget)) {
            retireProxyLocked(frame.bhkWorld);
            return false;
        }

        ROCK_LOG_INFO(
            Weapon,
            "Dynamic weapon box created: body={} generation={:016X} center=({:.2f},{:.2f},{:.2f}) half=({:.2f},{:.2f},{:.2f}) scale={:.3f} padding={:.2f} layer={}",
            _body.getBodyId().value,
            _createdGenerationKey,
            _createdCenterWeaponLocal.x,
            _createdCenterWeaponLocal.y,
            _createdCenterWeaponLocal.z,
            _createdHalfExtentsWeaponLocal.x,
            _createdHalfExtentsWeaponLocal.y,
            _createdHalfExtentsWeaponLocal.z,
            _createdWeaponScale,
            _createdPaddingGameUnits,
            collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY);
        return true;
    }

    void DynamicWeaponCollisionRuntime::flushPendingPhysicsDrive(
        RE::hknpWorld* world,
        const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!_enabledAtomic.load(std::memory_order_acquire) || !world || !_created || _createdWorld != world || !_body.isValid()) {
            return;
        }

        const float divergenceThreshold = g_rockConfig.rockWeaponCollisionDynamicDivergenceTeleportGameUnits;
        GeneratedBodyDriveMode mode{
            .dynamicVelocity = true,
            .divergenceTeleportGameUnits =
                _divergenceDwellSeconds >= g_rockConfig.rockWeaponCollisionDynamicDivergenceTeleportDwellSeconds ?
                    divergenceThreshold :
                    0.0f,
        };
        if (_contactGraceSolves > 0 && _physicsRequestedTargetValid && _physicsLiveTargetValid &&
            g_rockConfig.rockWeaponCollisionDynamicContactPressMaxVelocityHavok > 0.0f) {
            const RE::NiPoint3 press{
                _physicsRequestedTarget.translate.x - _physicsLiveTarget.translate.x,
                _physicsRequestedTarget.translate.y - _physicsLiveTarget.translate.y,
                _physicsRequestedTarget.translate.z - _physicsLiveTarget.translate.z,
            };
            const float length = std::sqrt(press.x * press.x + press.y * press.y + press.z * press.z);
            if (std::isfinite(length) && length > 0.25f) {
                mode.hasContactPressDirection = true;
                mode.contactPressDirection[0] = press.x / length;
                mode.contactPressDirection[1] = press.y / length;
                mode.contactPressDirection[2] = press.z / length;
                mode.contactPressMaxVelocityHavok = g_rockConfig.rockWeaponCollisionDynamicContactPressMaxVelocityHavok;
            }
        }

        const auto driveResult = driveGeneratedKeyframedBody(
            world,
            _body,
            _driveState,
            timing,
            "DynamicWeaponBox",
            0,
            g_rockConfig.rockWeaponCollisionDynamicMaxLinearVelocityHavok,
            g_rockConfig.rockWeaponCollisionDynamicMaxAngularVelocityRadians,
            mode);
        if (driveResult.shouldRequestRebuild()) {
            _rebuildRequestedAtomic.store(true, std::memory_order_release);
            _droveThisSubstep = false;
            clearPublishedPhysicsSnapshot();
            return;
        }

        _droveThisSubstep =
            driveResult.driven &&
            driveResult.hasRequestedTargetGameTransform;
        _physicsRequestedTargetValid = driveResult.hasRequestedTargetGameTransform;
        _physicsRequestedTarget = driveResult.requestedTargetGameTransform;
        _physicsDriveTeleported = driveResult.teleported;
        if (driveResult.hasLiveBodyTransform) {
            _physicsLiveTarget = driveResult.liveBodyGameTransform;
            _physicsLiveTargetValid = true;
        }

        const float requestedGap = driveResult.hasLiveBodyTransform && driveResult.hasRequestedTargetGameTransform ?
            pointDistance(driveResult.liveBodyGameTransform.translate, driveResult.requestedTargetGameTransform.translate) :
            0.0f;
        if (driveResult.teleported) {
            _divergenceDwellSeconds = 0.0f;
            _contactGraceSolves = 0;
        } else if (std::isfinite(requestedGap) && divergenceThreshold > 0.0f && requestedGap > divergenceThreshold) {
            _divergenceDwellSeconds += std::clamp(driveResult.driveDeltaSeconds, 0.0f, 0.1f);
        } else {
            _divergenceDwellSeconds = 0.0f;
        }
    }

    void DynamicWeaponCollisionRuntime::samplePostSolve(RE::hknpWorld* world, const std::uint64_t solveSequence)
    {
        if (!_enabledAtomic.load(std::memory_order_acquire) || !world || !_created || _createdWorld != world ||
            !_body.isValid() || !_droveThisSubstep || !_physicsRequestedTargetValid) {
            return;
        }
        _droveThisSubstep = false;

        RE::NiTransform liveBodyWorld{};
        if (!havok_runtime::tryResolveLiveBodyWorldTransform(world, _body.getBodyId(), liveBodyWorld) ||
            !dynamic_weapon_collision_policy::isFiniteTransform(liveBodyWorld)) {
            _contactGraceSolves = 0;
            clearPublishedPhysicsSnapshot();
            return;
        }
        _physicsLiveTarget = liveBodyWorld;
        _physicsLiveTargetValid = true;

        const auto contactSequence = _contactSequenceAtomic.load(std::memory_order_acquire);
        bool newMatchingContact = false;
        std::uint32_t otherBodyId = kInvalidBodyId;
        std::uint32_t otherLayer = 0;
        if (contactSequence != _consumedContactSequence) {
            _consumedContactSequence = contactSequence;
            const auto contactWorld = _contactWorldAtomic.load(std::memory_order_relaxed);
            const auto contactProxy = _contactProxyBodyIdAtomic.load(std::memory_order_relaxed);
            otherBodyId = _contactOtherBodyIdAtomic.load(std::memory_order_relaxed);
            otherLayer = _contactOtherLayerAtomic.load(std::memory_order_relaxed);
            newMatchingContact =
                contactWorld == reinterpret_cast<std::uintptr_t>(world) &&
                contactProxy == _body.getBodyId().value &&
                collision_layer_policy::isWorldSurfaceLayer(otherLayer);
        }
        if (_physicsDriveTeleported) {
            _contactGraceSolves = 0;
        } else if (newMatchingContact) {
            _contactGraceSolves = kContactGraceSolves;
        } else if (_contactGraceSolves > 0) {
            --_contactGraceSolves;
        }

        PhysicsSnapshot snapshot{};
        snapshot.valid = true;
        snapshot.contactActive = _contactGraceSolves > 0;
        snapshot.teleported = _physicsDriveTeleported;
        snapshot.world = reinterpret_cast<std::uintptr_t>(world);
        snapshot.bodyId = _body.getBodyId().value;
        snapshot.otherBodyId = newMatchingContact ? otherBodyId : _snapshotOtherBodyIdAtomic.load(std::memory_order_relaxed);
        snapshot.otherLayer = newMatchingContact ? otherLayer : _snapshotOtherLayerAtomic.load(std::memory_order_relaxed);
        snapshot.contactGraceSolves = _contactGraceSolves;
        snapshot.generationKey = _createdGenerationKey;
        snapshot.solveSequence = solveSequence;
        snapshot.weaponScale = _createdWeaponScale;
        snapshot.requestedProxyBodyWorld = _physicsRequestedTarget;
        snapshot.liveProxyBodyWorld = liveBodyWorld;
        publishPhysicsSnapshot(snapshot);
        _physicsDriveTeleported = false;
    }

    bool DynamicWeaponCollisionRuntime::isProxyBodyIdAtomic(const std::uint32_t bodyId) const
    {
        return bodyId != kInvalidBodyId && _bodyIdAtomic.load(std::memory_order_acquire) == bodyId;
    }

    void DynamicWeaponCollisionRuntime::recordWorldSurfaceContactCallback(
        RE::hknpWorld* world,
        const std::uint32_t proxyBodyId,
        const std::uint32_t otherBodyId,
        const bool otherLayerRead,
        const std::uint32_t otherLayer,
        const bool rawContactPointValid)
    {
        if (!world || !isProxyBodyIdAtomic(proxyBodyId)) {
            return;
        }
        _proxyPairCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (!otherLayerRead || !collision_layer_policy::isWorldSurfaceLayer(otherLayer)) {
            return;
        }
        _worldSurfaceCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (!rawContactPointValid) {
            return;
        }
        _rawPointCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        _contactWorldAtomic.store(reinterpret_cast<std::uintptr_t>(world), std::memory_order_relaxed);
        _contactProxyBodyIdAtomic.store(proxyBodyId, std::memory_order_relaxed);
        _contactOtherBodyIdAtomic.store(otherBodyId, std::memory_order_relaxed);
        _contactOtherLayerAtomic.store(otherLayer, std::memory_order_relaxed);
        _contactSequenceAtomic.fetch_add(1, std::memory_order_release);
    }

    void DynamicWeaponCollisionRuntime::recordWorldSurfaceManifoldProcessedCallback(
        RE::hknpWorld* world,
        const std::uint32_t proxyBodyId,
        const std::uint32_t otherBodyId,
        const bool otherLayerRead,
        const std::uint32_t otherLayer)
    {
        if (!world || !isProxyBodyIdAtomic(proxyBodyId)) {
            return;
        }
        _proxyPairCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (!otherLayerRead || !collision_layer_policy::isWorldSurfaceLayer(otherLayer)) {
            return;
        }
        _worldSurfaceCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        _processedManifoldCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        _contactWorldAtomic.store(reinterpret_cast<std::uintptr_t>(world), std::memory_order_relaxed);
        _contactProxyBodyIdAtomic.store(proxyBodyId, std::memory_order_relaxed);
        _contactOtherBodyIdAtomic.store(otherBodyId, std::memory_order_relaxed);
        _contactOtherLayerAtomic.store(otherLayer, std::memory_order_relaxed);
        _contactSequenceAtomic.fetch_add(1, std::memory_order_release);
    }

    void DynamicWeaponCollisionRuntime::retireProxyLocked(void* bhkWorld)
    {
        const auto bodyId = _bodyIdAtomic.exchange(kInvalidBodyId, std::memory_order_acq_rel);
        const bool liveOwnerMatches =
            _created &&
            _body.isValid() &&
            bhkWorld &&
            bhkWorld == _createdBhkWorld;
        if (liveOwnerMatches) {
            _body.retireDeferred(bhkWorld);
        } else {
            if (_created && _body.isValid()) {
                ROCK_LOG_WARN(
                    Weapon,
                    "Dynamic weapon box owner changed before retirement; abandoning native body without mutating an unverified world: body={}",
                    bodyId);
            }
            _body.reset();
        }
        clearLocalProxyStateLocked();
        if (bodyId != kInvalidBodyId && liveOwnerMatches) {
            ROCK_LOG_INFO(Weapon, "Dynamic weapon box retired: body={}", bodyId);
        }
    }

    void DynamicWeaponCollisionRuntime::clearLocalProxyStateLocked()
    {
        if (_shape) {
            havok_ref_count::release(_shape);
        }
        _shape = nullptr;
        _createdWorld = nullptr;
        _createdBhkWorld = nullptr;
        _createdGenerationKey = 0;
        _createdCenterWeaponLocal = {};
        _createdHalfExtentsWeaponLocal = {};
        _createdWeaponScale = 1.0f;
        _createdPaddingGameUnits = 0.0f;
        _created = false;
        _droveThisSubstep = false;
        _physicsRequestedTargetValid = false;
        _physicsLiveTargetValid = false;
        _physicsDriveTeleported = false;
        _physicsRequestedTarget = {};
        _physicsLiveTarget = {};
        _divergenceDwellSeconds = 0.0f;
        _contactGraceSolves = 0;
        _consumedContactSequence = 0;
        _previousRawProxyBodyTarget = {};
        _surfaceClutchProxyBodyTarget = {};
        _previousRawProxyBodyTargetValid = false;
        _surfaceClutchTargetValid = false;
        _surfaceClutchActive = false;
        _proxyPairCallbackSequenceAtomic.store(0, std::memory_order_release);
        _worldSurfaceCallbackSequenceAtomic.store(0, std::memory_order_release);
        _rawPointCallbackSequenceAtomic.store(0, std::memory_order_release);
        _processedManifoldCallbackSequenceAtomic.store(0, std::memory_order_release);
        _contactSequenceAtomic.store(0, std::memory_order_release);
        _contactWorldAtomic.store(0, std::memory_order_release);
        _contactProxyBodyIdAtomic.store(kInvalidBodyId, std::memory_order_release);
        _contactOtherBodyIdAtomic.store(kInvalidBodyId, std::memory_order_release);
        _contactOtherLayerAtomic.store(0, std::memory_order_release);
        _rebuildRequestedAtomic.store(false, std::memory_order_release);
        clearGeneratedKeyframedBodyDriveState(_driveState);
        clearPublishedPhysicsSnapshot();
    }

    void DynamicWeaponCollisionRuntime::retireAll(void* bhkWorld)
    {
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        retireProxyLocked(bhkWorld);
        _enabledAtomic.store(false, std::memory_order_release);
        _frameAcceptingIntent = false;
        _frameHasIntent = false;
        _debugSnapshot = {};
    }

    void DynamicWeaponCollisionRuntime::abandonHavokStateAfterWorldLoss()
    {
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        const auto bodyId = _bodyIdAtomic.exchange(kInvalidBodyId, std::memory_order_acq_rel);
        _body.reset();
        clearLocalProxyStateLocked();
        _enabledAtomic.store(false, std::memory_order_release);
        _frameAcceptingIntent = false;
        _frameHasIntent = false;
        _frameIndex = 0;
        _frameGenerationKey = 0;
        _frameWorld = nullptr;
        _frameBhkWorld = nullptr;
        _frameWeaponNode = nullptr;
        _frameRequestedWeaponWorld = {};
        _frameResetRawMotionBaseline = false;
        _debugSnapshot = {};
        if (bodyId != kInvalidBodyId) {
            ROCK_LOG_WARN(
                Weapon,
                "Dynamic weapon box abandoned after Havok world loss: body={}",
                bodyId);
        }
    }

    RE::hknpBodyId DynamicWeaponCollisionRuntime::proxyBodyIdForDebug() const
    {
        return RE::hknpBodyId{ _bodyIdAtomic.load(std::memory_order_acquire) };
    }

    bool DynamicWeaponCollisionRuntime::getDebugSnapshot(DebugSnapshot& outSnapshot) const
    {
        outSnapshot = _debugSnapshot;
        return outSnapshot.valid;
    }

    void DynamicWeaponCollisionRuntime::storeAtomicTransform(AtomicTransform& target, const RE::NiTransform& value)
    {
        std::size_t index = 0;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                target.rotation[index++].store(value.rotate.entry[row][column], std::memory_order_relaxed);
            }
        }
        target.translation[0].store(value.translate.x, std::memory_order_relaxed);
        target.translation[1].store(value.translate.y, std::memory_order_relaxed);
        target.translation[2].store(value.translate.z, std::memory_order_relaxed);
        target.scale.store(value.scale, std::memory_order_relaxed);
    }

    RE::NiTransform DynamicWeaponCollisionRuntime::loadAtomicTransform(const AtomicTransform& source)
    {
        RE::NiTransform result{};
        std::size_t index = 0;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                result.rotate.entry[row][column] = source.rotation[index++].load(std::memory_order_relaxed);
            }
        }
        result.translate = RE::NiPoint3{
            source.translation[0].load(std::memory_order_relaxed),
            source.translation[1].load(std::memory_order_relaxed),
            source.translation[2].load(std::memory_order_relaxed),
        };
        result.scale = source.scale.load(std::memory_order_relaxed);
        return result;
    }

    void DynamicWeaponCollisionRuntime::publishPhysicsSnapshot(const PhysicsSnapshot& snapshot)
    {
        _snapshotVersionAtomic.fetch_add(1, std::memory_order_acq_rel);
        _snapshotValidAtomic.store(snapshot.valid, std::memory_order_relaxed);
        _snapshotContactActiveAtomic.store(snapshot.contactActive, std::memory_order_relaxed);
        _snapshotTeleportedAtomic.store(snapshot.teleported, std::memory_order_relaxed);
        _snapshotWorldAtomic.store(snapshot.world, std::memory_order_relaxed);
        _snapshotBodyIdAtomic.store(snapshot.bodyId, std::memory_order_relaxed);
        _snapshotOtherBodyIdAtomic.store(snapshot.otherBodyId, std::memory_order_relaxed);
        _snapshotOtherLayerAtomic.store(snapshot.otherLayer, std::memory_order_relaxed);
        _snapshotContactGraceAtomic.store(snapshot.contactGraceSolves, std::memory_order_relaxed);
        _snapshotGenerationKeyAtomic.store(snapshot.generationKey, std::memory_order_relaxed);
        _snapshotSolveSequenceAtomic.store(snapshot.solveSequence, std::memory_order_relaxed);
        _snapshotWeaponScaleAtomic.store(snapshot.weaponScale, std::memory_order_relaxed);
        storeAtomicTransform(_snapshotRequestedProxyBodyWorld, snapshot.requestedProxyBodyWorld);
        storeAtomicTransform(_snapshotLiveProxyBodyWorld, snapshot.liveProxyBodyWorld);
        _snapshotVersionAtomic.fetch_add(1, std::memory_order_release);
    }

    void DynamicWeaponCollisionRuntime::clearPublishedPhysicsSnapshot()
    {
        PhysicsSnapshot snapshot{};
        publishPhysicsSnapshot(snapshot);
    }

    bool DynamicWeaponCollisionRuntime::readPhysicsSnapshot(PhysicsSnapshot& outSnapshot) const
    {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const auto before = _snapshotVersionAtomic.load(std::memory_order_acquire);
            if ((before & 1u) != 0) {
                continue;
            }
            PhysicsSnapshot candidate{};
            candidate.valid = _snapshotValidAtomic.load(std::memory_order_relaxed);
            candidate.contactActive = _snapshotContactActiveAtomic.load(std::memory_order_relaxed);
            candidate.teleported = _snapshotTeleportedAtomic.load(std::memory_order_relaxed);
            candidate.world = _snapshotWorldAtomic.load(std::memory_order_relaxed);
            candidate.bodyId = _snapshotBodyIdAtomic.load(std::memory_order_relaxed);
            candidate.otherBodyId = _snapshotOtherBodyIdAtomic.load(std::memory_order_relaxed);
            candidate.otherLayer = _snapshotOtherLayerAtomic.load(std::memory_order_relaxed);
            candidate.contactGraceSolves = _snapshotContactGraceAtomic.load(std::memory_order_relaxed);
            candidate.generationKey = _snapshotGenerationKeyAtomic.load(std::memory_order_relaxed);
            candidate.solveSequence = _snapshotSolveSequenceAtomic.load(std::memory_order_relaxed);
            candidate.weaponScale = _snapshotWeaponScaleAtomic.load(std::memory_order_relaxed);
            candidate.requestedProxyBodyWorld = loadAtomicTransform(_snapshotRequestedProxyBodyWorld);
            candidate.liveProxyBodyWorld = loadAtomicTransform(_snapshotLiveProxyBodyWorld);
            const auto after = _snapshotVersionAtomic.load(std::memory_order_acquire);
            if (before == after && (after & 1u) == 0) {
                outSnapshot = candidate;
                return candidate.valid;
            }
        }
        outSnapshot = {};
        return false;
    }
}
