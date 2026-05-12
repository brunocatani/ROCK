#include "physics-interaction/native/GrabAuthorityPhase0Probe.h"

#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/native/HavokConvexShapeBuilder.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokTlsDiagnostics.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"

#include "RE/Havok/hknpShape.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace rock::grab_authority_phase0
{
    namespace
    {
        constexpr float kProbeConvexRadiusHavok = 0.001f;
        constexpr float kProbeHullHalfExtentHavok = 0.0125f;
        constexpr float kDefaultAmplitudeGameUnits = 8.0f;
        constexpr float kTwoPi = 6.28318530717958647692f;
        constexpr std::uint32_t kGeneratedCollisionGroup = 0x000B;
        constexpr std::uint32_t kInvalidSemanticBodyId = 0xFFFF'FFFFu;

        std::vector<RE::NiPoint3> makeProbeHullPoints()
        {
            const float s = kProbeHullHalfExtentHavok;
            return {
                RE::NiPoint3{ s, s, s },
                RE::NiPoint3{ -s, -s, s },
                RE::NiPoint3{ -s, s, -s },
                RE::NiPoint3{ s, -s, -s },
            };
        }

        RE::hknpShape* buildProbeShape()
        {
            return havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(makeProbeHullPoints(), kProbeConvexRadiusHavok);
        }

        RE::NiMatrix3 makeZRotation(float radians)
        {
            auto rotation = transform_math::makeIdentityRotation<RE::NiMatrix3>();
            const float c = std::cos(radians);
            const float s = std::sin(radians);
            rotation.entry[0][0] = c;
            rotation.entry[0][1] = -s;
            rotation.entry[0][2] = 0.0f;
            rotation.entry[1][0] = s;
            rotation.entry[1][1] = c;
            rotation.entry[1][2] = 0.0f;
            rotation.entry[2][0] = 0.0f;
            rotation.entry[2][1] = 0.0f;
            rotation.entry[2][2] = 1.0f;
            return rotation;
        }

        RE::NiTransform makeProbeTarget(std::uint64_t sequence, float amplitudeGameUnits)
        {
            const float amplitude = std::clamp(std::isfinite(amplitudeGameUnits) ? amplitudeGameUnits : kDefaultAmplitudeGameUnits, 0.25f, 64.0f);
            const float phase = static_cast<float>(sequence % 240) * (kTwoPi / 240.0f);

            auto target = transform_math::makeIdentityTransform<RE::NiTransform>();
            target.rotate = makeZRotation(phase * 0.5f);
            target.translate.x = std::sin(phase) * amplitude;
            target.translate.y = std::cos(phase * 0.7f) * amplitude * 0.5f;
            target.translate.z = std::sin(phase * 0.37f) * amplitude * 0.25f;
            return target;
        }

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

        float distanceGameUnits(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const float dx = a.x - b.x;
            const float dy = a.y - b.y;
            const float dz = a.z - b.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        float rotationDeltaDegrees(const RE::NiMatrix3& a, const RE::NiMatrix3& b)
        {
            const RE::NiMatrix3 delta = a.Transpose() * b;
            const float trace = delta.entry[0][0] + delta.entry[1][1] + delta.entry[2][2];
            const float cosine = std::clamp((trace - 1.0f) * 0.5f, -1.0f, 1.0f);
            return std::acos(cosine) * 57.29577951308232f;
        }

        void computeLinearVelocityHavok(const RE::NiTransform& previous, const RE::NiTransform& current, float deltaSeconds, float outVelocity[4])
        {
            outVelocity[0] = 0.0f;
            outVelocity[1] = 0.0f;
            outVelocity[2] = 0.0f;
            outVelocity[3] = 0.0f;
            if (!havok_physics_timing::isUsableDelta(deltaSeconds)) {
                return;
            }

            const float scale = gameToHavokScale() / deltaSeconds;
            outVelocity[0] = (current.translate.x - previous.translate.x) * scale;
            outVelocity[1] = (current.translate.y - previous.translate.y) * scale;
            outVelocity[2] = (current.translate.z - previous.translate.z) * scale;
        }

        std::uint32_t layerForPolicy(int policy)
        {
            switch (static_cast<ProxyFilterPolicy>(policy)) {
            case ProxyFilterPolicy::RockBodyLayerPlusNoCollideBit:
                return collision_layer_policy::ROCK_LAYER_BODY;
            case ProxyFilterPolicy::RockHandLayerPlusNoCollideBit:
                return collision_layer_policy::ROCK_LAYER_HAND;
            case ProxyFilterPolicy::NonCollidableLayerPlusNoCollideBit:
            default:
                return collision_layer_policy::FO4_LAYER_NONCOLLIDABLE;
            }
        }

        std::uint32_t buildFilterInfo(int policy)
        {
            return (kGeneratedCollisionGroup << 16) |
                   (layerForPolicy(policy) & collision_layer_policy::FO4_LAYER_FILTER_MASK) |
                   collision_suppression_registry::kSuppressionNoCollideBit;
        }

        const char* policyName(int policy)
        {
            switch (static_cast<ProxyFilterPolicy>(policy)) {
            case ProxyFilterPolicy::RockBodyLayerPlusNoCollideBit:
                return "rockBody+bit14";
            case ProxyFilterPolicy::RockHandLayerPlusNoCollideBit:
                return "rockHand+bit14";
            case ProxyFilterPolicy::NonCollidableLayerPlusNoCollideBit:
            default:
                return "nonCollidable+bit14";
            }
        }

        bool filterHasNoContactPolicy(std::uint32_t filterInfo, int policy)
        {
            const std::uint32_t layer = filterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
            return layer == layerForPolicy(policy) && (filterInfo & collision_suppression_registry::kSuppressionNoCollideBit) != 0;
        }

        bool shouldLog(int& counter, int intervalFrames)
        {
            const int interval = std::max(1, intervalFrames);
            ++counter;
            if (counter >= interval) {
                counter = 0;
                return true;
            }
            return false;
        }
    }

    void Probe::updateGameFrame(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const Config& config)
    {
        if (!config.enabled) {
            if (hasBodies() || _solverConstraint.isValid()) {
                shutdown(bhkWorld);
            }
            _createRetryFrames = 0;
            return;
        }

        const int sanitizedPolicy = std::clamp(config.proxyFilterPolicy, 0, 2);
        const bool configChanged = hasBodies() &&
                                   (_bhkWorld != bhkWorld ||
                                       _hknpWorld != hknpWorld ||
                                       _activeFilterPolicy != sanitizedPolicy ||
                                       _activeSolverProbeEnabled != config.solverProbeEnabled);
        if (configChanged) {
            destroyBodies(_bhkWorld ? _bhkWorld : bhkWorld);
            _createRetryFrames = 0;
        }

        if (!bhkWorld || !hknpWorld) {
            return;
        }

        if (!hasBodies()) {
            if (_createRetryFrames > 0) {
                --_createRetryFrames;
                return;
            }

            if (createBodies(bhkWorld, hknpWorld, config)) {
                _createRetryFrames = 0;
            } else {
                _createRetryFrames = 120;
            }
        }
    }

    void Probe::driveBetweenCollideAndSolve(RE::hknpWorld* hknpWorld, const havok_physics_timing::PhysicsTimingSample& timing, const Config& config)
    {
        if (!config.enabled || !_proxyBody.isValid() || hknpWorld != _hknpWorld) {
            return;
        }

        const float dt = havok_physics_timing::driveDeltaSeconds(timing);
        const RE::NiTransform target = makeProbeTarget(_driveSequence++, config.motionAmplitudeGameUnits);
        const RE::NiTransform previous = _hasPreviousProxyTarget ? _previousProxyTarget : target;
        const RE::hkTransformf targetHavok = makeHavokTransform(target);

        float linearVelocityHavok[4]{};
        float angularVelocityHavok[4]{};
        computeLinearVelocityHavok(previous, target, dt, linearVelocityHavok);

        const bool setTransformOk = _proxyBody.setTransform(targetHavok);
        const bool setVelocityOk = _proxyBody.setVelocity(linearVelocityHavok, angularVelocityHavok);

        RE::NiTransform readback{};
        body_frame::BodyFrameSource readbackSource = body_frame::BodyFrameSource::Fallback;
        std::uint32_t motionIndex = body_frame::kFreeMotionIndex;
        const bool readbackOk =
            havok_runtime::tryResolveLiveBodyWorldTransform(hknpWorld, _proxyBody.getBodyId(), readback, &readbackSource, &motionIndex);

        std::uint32_t filterInfo = 0;
        const bool filterReadOk = havok_runtime::tryReadFilterInfo(hknpWorld, _proxyBody.getBodyId(), filterInfo);
        const auto tlsState = havok_tls_diagnostics::readCurrentCommandQueueState();
        const bool semanticLeak = bodyIdMatchesSemanticContact(_proxyBody.getBodyId().value) || bodyIdMatchesSemanticContact(_receiverBody.getBodyId().value);

        _lastProxyTarget = target;
        _previousProxyTarget = target;
        _hasPreviousProxyTarget = true;

        if (shouldLog(_betweenLogCounter, config.logIntervalFrames) || !setTransformOk || !setVelocityOk || !readbackOk || semanticLeak) {
            ROCK_LOG_INFO(GrabPhase0,
                "between proxy={} receiver={} constraint={} phase=between substep={}/{} dt={:.6f} tlsReadable={} tlsCmd={} tlsPhys={} tlsThread={} setTransform={} setVelocity={} readback={} readbackSource={} motion={} posErr={:.4f} rotErr={:.3f} filter=0x{:08X} noContact={} semanticLeak={} policy={}",
                _proxyBody.getBodyId().value,
                _receiverBody.isValid() ? _receiverBody.getBodyId().value : kInvalidBodyId,
                _solverConstraint.isValid() ? _solverConstraint.constraintId : kInvalidBodyId,
                timing.substepIndex,
                timing.substepCount,
                dt,
                tlsState.readable ? "yes" : "no",
                tlsState.wrapperQueueModeActive() ? "queue" : "direct",
                tlsState.physicsContext,
                tlsState.threadCommandIndex,
                setTransformOk ? "ok" : "fail",
                setVelocityOk ? "ok" : "fail",
                readbackOk ? "ok" : "fail",
                static_cast<int>(readbackSource),
                motionIndex,
                readbackOk ? distanceGameUnits(readback.translate, target.translate) : -1.0f,
                readbackOk ? rotationDeltaDegrees(readback.rotate, target.rotate) : -1.0f,
                filterReadOk ? filterInfo : 0u,
                (filterReadOk && filterHasNoContactPolicy(filterInfo, _activeFilterPolicy)) ? "yes" : "no",
                semanticLeak ? "yes" : "no",
                policyName(_activeFilterPolicy));
        }
    }

    void Probe::observeAfterAny(RE::hknpWorld* hknpWorld, const havok_physics_timing::PhysicsTimingSample& timing, const Config& config)
    {
        if (!config.enabled || !_receiverBody.isValid() || hknpWorld != _hknpWorld) {
            return;
        }

        RE::NiTransform proxyReadback{};
        RE::NiTransform receiverReadback{};
        body_frame::BodyFrameSource proxySource = body_frame::BodyFrameSource::Fallback;
        body_frame::BodyFrameSource receiverSource = body_frame::BodyFrameSource::Fallback;
        std::uint32_t proxyMotionIndex = body_frame::kFreeMotionIndex;
        std::uint32_t receiverMotionIndex = body_frame::kFreeMotionIndex;
        const bool proxyOk =
            havok_runtime::tryResolveLiveBodyWorldTransform(hknpWorld, _proxyBody.getBodyId(), proxyReadback, &proxySource, &proxyMotionIndex);
        const bool receiverOk =
            havok_runtime::tryResolveLiveBodyWorldTransform(hknpWorld, _receiverBody.getBodyId(), receiverReadback, &receiverSource, &receiverMotionIndex);

        std::uint32_t proxyFilter = 0;
        std::uint32_t receiverFilter = 0;
        const bool proxyFilterOk = havok_runtime::tryReadFilterInfo(hknpWorld, _proxyBody.getBodyId(), proxyFilter);
        const bool receiverFilterOk = havok_runtime::tryReadFilterInfo(hknpWorld, _receiverBody.getBodyId(), receiverFilter);
        const bool semanticLeak = bodyIdMatchesSemanticContact(_proxyBody.getBodyId().value) || bodyIdMatchesSemanticContact(_receiverBody.getBodyId().value);

        if (shouldLog(_afterSolveLogCounter, config.logIntervalFrames) || !proxyOk || !receiverOk || semanticLeak) {
            ROCK_LOG_INFO(GrabPhase0,
                "afterSolve proxy={} receiver={} constraint={} substep={}/{} proxyRead={} receiverRead={} proxySrc={} receiverSrc={} proxyMotion={} receiverMotion={} targetProxyErr={:.4f} receiverLagFromProxy={:.4f} proxyFilter=0x{:08X} receiverFilter=0x{:08X} proxyNoContact={} receiverNoContact={} semanticLeak={}",
                _proxyBody.getBodyId().value,
                _receiverBody.getBodyId().value,
                _solverConstraint.isValid() ? _solverConstraint.constraintId : kInvalidBodyId,
                timing.substepIndex,
                timing.substepCount,
                proxyOk ? "ok" : "fail",
                receiverOk ? "ok" : "fail",
                static_cast<int>(proxySource),
                static_cast<int>(receiverSource),
                proxyMotionIndex,
                receiverMotionIndex,
                proxyOk ? distanceGameUnits(proxyReadback.translate, _lastProxyTarget.translate) : -1.0f,
                (proxyOk && receiverOk) ? distanceGameUnits(receiverReadback.translate, proxyReadback.translate) : -1.0f,
                proxyFilterOk ? proxyFilter : 0u,
                receiverFilterOk ? receiverFilter : 0u,
                (proxyFilterOk && filterHasNoContactPolicy(proxyFilter, _activeFilterPolicy)) ? "yes" : "no",
                (receiverFilterOk && filterHasNoContactPolicy(receiverFilter, _activeFilterPolicy)) ? "yes" : "no",
                semanticLeak ? "yes" : "no");
        }
    }

    void Probe::noteSemanticContactBodyIds(std::uint32_t rightHandBodyId, std::uint32_t leftHandBodyId, std::uint32_t weaponBodyId)
    {
        _lastSemanticRightHandBodyId = rightHandBodyId;
        _lastSemanticLeftHandBodyId = leftHandBodyId;
        _lastSemanticWeaponBodyId = weaponBodyId;
    }

    void Probe::shutdown(RE::bhkWorld* fallbackBhkWorld)
    {
        destroyBodies(_bhkWorld ? _bhkWorld : fallbackBhkWorld);
        _createRetryFrames = 0;
    }

    void Probe::abandon()
    {
        _solverConstraint.clear();
        _proxyBody.reset();
        _receiverBody.reset();
        _bhkWorld = nullptr;
        _hknpWorld = nullptr;
        _hasPreviousProxyTarget = false;
        _driveSequence = 0;
        _betweenLogCounter = 0;
        _afterSolveLogCounter = 0;
        _createRetryFrames = 0;
    }

    bool Probe::createBodies(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const Config& config)
    {
        const int sanitizedPolicy = std::clamp(config.proxyFilterPolicy, 0, 2);
        const std::uint32_t filterInfo = buildFilterInfo(sanitizedPolicy);
        const auto material = havok_material_registry::registerGeneratedBodyMaterial(hknpWorld);
        const RE::NiTransform initialTarget = makeProbeTarget(0, config.motionAmplitudeGameUnits);
        const auto initialHavok = makeHavokTransform(initialTarget);

        auto* proxyShape = buildProbeShape();
        if (!proxyShape) {
            ROCK_LOG_ERROR(GrabPhase0, "Proxy shape creation failed");
            return false;
        }

        if (!_proxyBody.create(hknpWorld, bhkWorld, proxyShape, filterInfo, material, BethesdaMotionType::Keyframed, "ROCK_GrabAuthorityPhase0Proxy")) {
            havok_ref_count::release(proxyShape);
            ROCK_LOG_ERROR(GrabPhase0, "Proxy body creation failed policy={} filter=0x{:08X}", policyName(sanitizedPolicy), filterInfo);
            return false;
        }
        havok_ref_count::release(proxyShape);
        _proxyBody.setTransform(initialHavok);
        float zeroVelocity[4]{};
        _proxyBody.setVelocity(zeroVelocity, zeroVelocity);

        if (config.solverProbeEnabled) {
            auto* receiverShape = buildProbeShape();
            if (!receiverShape) {
                destroyBodies(bhkWorld);
                ROCK_LOG_ERROR(GrabPhase0, "Receiver shape creation failed");
                return false;
            }

            if (!_receiverBody.create(hknpWorld, bhkWorld, receiverShape, filterInfo, material, BethesdaMotionType::Dynamic, "ROCK_GrabAuthorityPhase0Receiver")) {
                havok_ref_count::release(receiverShape);
                destroyBodies(bhkWorld);
                ROCK_LOG_ERROR(GrabPhase0, "Receiver body creation failed policy={} filter=0x{:08X}", policyName(sanitizedPolicy), filterInfo);
                return false;
            }
            havok_ref_count::release(receiverShape);
            _receiverBody.setTransform(initialHavok);
            _receiverBody.setMass(1.0f);

            float pivotB[4]{ 0.0f, 0.0f, 0.0f, 0.0f };
            auto desiredBodyInProxy = transform_math::makeIdentityTransform<RE::NiTransform>();
            GrabConstraintMotorTuning tuning{};
            tuning.linearTau = 0.03f;
            tuning.linearDamping = 0.85f;
            tuning.linearProportionalRecovery = 2.0f;
            tuning.linearConstantRecovery = 1.0f;
            tuning.linearMaxForce = 600.0f;
            tuning.angularTau = 0.04f;
            tuning.angularDamping = 0.85f;
            tuning.angularProportionalRecovery = 2.0f;
            tuning.angularConstantRecovery = 1.0f;
            tuning.angularMaxForce = 60.0f;

            _solverConstraint = createGrabConstraint(
                hknpWorld,
                _proxyBody.getBodyId(),
                _receiverBody.getBodyId(),
                initialTarget,
                initialTarget.translate,
                pivotB,
                desiredBodyInProxy,
                tuning);

            if (!_solverConstraint.isValid()) {
                destroyBodies(bhkWorld);
                ROCK_LOG_ERROR(GrabPhase0, "Solver probe constraint creation failed");
                return false;
            }
        }

        _bhkWorld = bhkWorld;
        _hknpWorld = hknpWorld;
        _activeFilterPolicy = sanitizedPolicy;
        _activeSolverProbeEnabled = config.solverProbeEnabled;
        _previousProxyTarget = initialTarget;
        _lastProxyTarget = initialTarget;
        _hasPreviousProxyTarget = true;
        _driveSequence = 1;
        _betweenLogCounter = 0;
        _afterSolveLogCounter = 0;

        std::uint32_t proxyFilter = 0;
        std::uint32_t receiverFilter = 0;
        const bool proxyFilterOk = havok_runtime::tryReadFilterInfo(hknpWorld, _proxyBody.getBodyId(), proxyFilter);
        const bool receiverFilterOk =
            _receiverBody.isValid() && havok_runtime::tryReadFilterInfo(hknpWorld, _receiverBody.getBodyId(), receiverFilter);

        ROCK_LOG_INFO(GrabPhase0,
            "created proxy={} receiver={} constraint={} policy={} filter=0x{:08X} proxyFilter=0x{:08X} receiverFilter=0x{:08X} solverProbe={} noContactProxy={} noContactReceiver={}",
            _proxyBody.getBodyId().value,
            _receiverBody.isValid() ? _receiverBody.getBodyId().value : kInvalidBodyId,
            _solverConstraint.isValid() ? _solverConstraint.constraintId : kInvalidBodyId,
            policyName(_activeFilterPolicy),
            filterInfo,
            proxyFilterOk ? proxyFilter : 0u,
            receiverFilterOk ? receiverFilter : 0u,
            _activeSolverProbeEnabled ? "yes" : "no",
            (proxyFilterOk && filterHasNoContactPolicy(proxyFilter, _activeFilterPolicy)) ? "yes" : "no",
            (!_receiverBody.isValid() || (receiverFilterOk && filterHasNoContactPolicy(receiverFilter, _activeFilterPolicy))) ? "yes" : "no");

        return true;
    }

    void Probe::destroyBodies(RE::bhkWorld* fallbackBhkWorld)
    {
        auto* world = _hknpWorld;
        auto* bhkWorld = _bhkWorld ? _bhkWorld : fallbackBhkWorld;
        const std::uint32_t proxyId = _proxyBody.isValid() ? _proxyBody.getBodyId().value : kInvalidBodyId;
        const std::uint32_t receiverId = _receiverBody.isValid() ? _receiverBody.getBodyId().value : kInvalidBodyId;
        const std::uint32_t constraintId = _solverConstraint.isValid() ? _solverConstraint.constraintId : kInvalidBodyId;

        if (_solverConstraint.isValid()) {
            destroyGrabConstraint(world, _solverConstraint);
        }
        if (_receiverBody.isValid()) {
            _receiverBody.destroy(bhkWorld);
        } else {
            _receiverBody.reset();
        }
        if (_proxyBody.isValid()) {
            _proxyBody.destroy(bhkWorld);
        } else {
            _proxyBody.reset();
        }

        _bhkWorld = nullptr;
        _hknpWorld = nullptr;
        _hasPreviousProxyTarget = false;
        _driveSequence = 0;
        _betweenLogCounter = 0;
        _afterSolveLogCounter = 0;
        _createRetryFrames = 0;

        if (proxyId != kInvalidBodyId || receiverId != kInvalidBodyId || constraintId != kInvalidBodyId) {
            ROCK_LOG_INFO(GrabPhase0, "destroyed proxy={} receiver={} constraint={}", proxyId, receiverId, constraintId);
        }
    }

    bool Probe::bodyIdMatchesSemanticContact(std::uint32_t bodyId) const noexcept
    {
        if (bodyId == kInvalidBodyId || bodyId == kInvalidSemanticBodyId) {
            return false;
        }
        return bodyId == _lastSemanticRightHandBodyId || bodyId == _lastSemanticLeftHandBodyId || bodyId == _lastSemanticWeaponBodyId;
    }
}
