// The held object each frame: controller and object motion samples for the
// release throw, the held body set and scope, provider held-object targets,
// coordination of an object held by both hands, the drive and visual
// presentation (updateHeldObject), acquisition and the hand's visual return
// after a grab.
#include "physics-interaction/hand/HandGrabInternal.h"

namespace rock
{

    namespace
    {
        RE::NiPoint3 scalePoint(const RE::NiPoint3& value, float scalar)
        {
            return RE::NiPoint3{ value.x * scalar, value.y * scalar, value.z * scalar };
        }

        template <std::size_t N>
        float recordDeviationAverage(std::array<float, N>& history, std::size_t& count, std::size_t& next, float sample)
        {
            if constexpr (N == 0) {
                return std::isfinite(sample) ? sample : 0.0f;
            } else {
                const float sanitizedSample = std::isfinite(sample) && sample > 0.0f ? sample : 0.0f;
                history[next] = sanitizedSample;
                next = (next + 1) % N;
                if (count < N) {
                    ++count;
                }

                float total = 0.0f;
                for (std::size_t i = 0; i < count; ++i) {
                    total += history[i];
                }
                return count > 0 ? total / static_cast<float>(count) : sanitizedSample;
            }
        }

        constexpr int GRAB_EXTERNAL_HAND_PRIORITY = 90;

        constexpr const char* GRAB_RETURN_HAND_TAG = "ROCK_GrabReturn";

        constexpr int GRAB_RETURN_HAND_PRIORITY = 85;

        bool applyGrabExternalHandWorldTransform(bool isLeft, const RE::NiTransform& adjustedHandTransform)
        {
            return frik_visual_authority::publishHandWorld(GRAB_EXTERNAL_HAND_TAG, handFromBool(isLeft),
                adjustedHandTransform, GRAB_EXTERNAL_HAND_PRIORITY);
        }

        bool applyGrabReturnHandWorldTransform(bool isLeft, const RE::NiTransform& handTransform)
        {
            return frik_visual_authority::publishHandWorld(GRAB_RETURN_HAND_TAG, handFromBool(isLeft), handTransform,
                GRAB_RETURN_HAND_PRIORITY);
        }

        void clearGrabReturnHandWorldTransform(bool isLeft)
        {
            (void)frik_visual_authority::clearHandWorld(GRAB_RETURN_HAND_TAG, handFromBool(isLeft));
        }
    }

    void Hand::beginGrabVisualReturn()
    {
        const bool weaponReturn = isHoldingLooseWeapon();
        if (!(weaponReturn ? hand_visual_lerp_math::kEquippedWeaponReturnEnabled
                           : g_rockConfig.rockGrabHandReturnEnabled) ||
            !_hasLastPublishedGrabVisualHandTransform ||
            !isUsableGrabVisualTransform(_lastPublishedGrabVisualHandTransform) ||
            !frik_visual_authority::isAvailable()) {
            clearGrabVisualReturn("release-not-eligible", false);
            return;
        }

        // A submitted IK target is not necessarily the rendered wrist (reach,
        // final arm solve and locomotion can move it). Release from what was
        // actually shown, before discarding the held-object authority.
        auto start = _lastPublishedGrabVisualHandTransform;
        RE::NiTransform presented{};
        const bool hasPresentedStart = weaponReturn &&
            frik_hand_world_authority::tryGetPresentedHandWorld(_isLeft, presented) &&
            isUsableGrabVisualTransform(presented);
        if (hasPresentedStart)
            start = presented;
        _grabVisualReturnConfig = weaponReturn ? hand_visual_lerp_math::kEquippedWeaponReturnConfig
                                               : hand_visual_lerp_math::VisualReturnConfig{
                                                     .minSeconds = g_rockConfig.rockGrabHandReturnTimeMin,
                                                     .maxSeconds = g_rockConfig.rockGrabHandReturnTimeMax,
                                                     .minDistanceGameUnits = g_rockConfig.rockGrabHandReturnMinDistance,
                                                     .maxDistanceGameUnits = g_rockConfig.rockGrabHandReturnMaxDistance,
                                                     .minAngleDegrees = g_rockConfig.rockGrabHandReturnMinAngleDegrees,
                                                     .maxAngleDegrees = g_rockConfig.rockGrabHandReturnMaxAngleDegrees,
                                                 };
        _grabVisualReturn.begin(start);
        if (!applyGrabReturnHandWorldTransform(_isLeft, _grabVisualReturn.start)) {
            _grabVisualReturn.clear();
            clearGrabReturnHandWorldTransform(_isLeft);
            ROCK_LOG_WARN(Hand, "{} hand visual return start failed; restoring tracked authority immediately",
                handName());
            return;
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand visual return started from=({:.2f},{:.2f},{:.2f}) weapon={} source={} "
            "publishedGap={:.3f}gu/{:.2f}deg",
            handName(), _grabVisualReturn.start.translate.x, _grabVisualReturn.start.translate.y,
            _grabVisualReturn.start.translate.z, weaponReturn, hasPresentedStart ? "rendered" : "published",
            hand_visual_lerp_math::distanceGameUnits(start.translate, _lastPublishedGrabVisualHandTransform.translate),
            hand_visual_lerp_math::rotationDistanceDegrees(start, _lastPublishedGrabVisualHandTransform));
    }

    void Hand::updateGrabVisualReturn(const RE::NiTransform& trackedHandWorld, float deltaTime)
    {
        if (!_grabVisualReturn.active) {
            return;
        }
        if (!runtime_state::isLocalSkeletonReady() || !frik_visual_authority::isAvailable() ||
            !isUsableGrabVisualTransform(trackedHandWorld)) {
            clearGrabVisualReturn("tracked-hand-unavailable", true);
            return;
        }

        const auto result = hand_visual_lerp_math::driveVisualReturn(
            _grabVisualReturn, trackedHandWorld, deltaTime, _grabVisualReturnConfig,
            [](const RE::NiTransform& transform) { return isUsableGrabVisualTransform(transform); },
            [this](const RE::NiTransform& transform) { return applyGrabReturnHandWorldTransform(_isLeft, transform); });
        if (result.timingInitializedThisFrame) {
            ROCK_LOG_DEBUG(Hand, "{} hand visual return timing distance={:.2f}gu angle={:.1f}deg duration={:.3f}s",
                handName(), result.initialDistanceGameUnits, result.initialAngleDegrees, result.durationSeconds);
        }
        if (result.status == hand_visual_lerp_math::VisualReturnDriveStatus::InvalidTransform ||
            result.status == hand_visual_lerp_math::VisualReturnDriveStatus::PublishFailed) {
            clearGrabVisualReturn("publish-failed", true);
            return;
        }

        if (result.status != hand_visual_lerp_math::VisualReturnDriveStatus::Completed) {
            return;
        }

        const float completedDuration = result.durationSeconds;
        clearGrabReturnHandWorldTransform(_isLeft);
        _grabVisualReturn.clear();
        ROCK_LOG_DEBUG(Hand, "{} hand visual return completed duration={:.3f}s", handName(), completedDuration);
    }

    void Hand::clearGrabVisualReturn(const char* reason, bool logCancellation)
    {
        const bool wasActive = _grabVisualReturn.active;
        clearGrabReturnHandWorldTransform(_isLeft);
        _grabVisualReturn.clear();
        if (wasActive && logCancellation) {
            ROCK_LOG_DEBUG(Hand, "{} hand visual return cancelled reason={}", handName(), reason ? reason : "unknown");
        }
    }

    void Hand::cancelGrabVisualReturn(const char* reason) { clearGrabVisualReturn(reason, true); }

    Hand::HeldHandMotionSample Hand::recordHeldControllerMotionSample(const RE::NiTransform& handWorldTransform,
        const game_frame_timing_policy::GameFrameTiming& timing)
    {
        HeldHandMotionSample handMotion{};
        const auto admission = release_velocity::admitControllerSample(_releaseControllerFrame, timing);
        if (admission.duplicate)
            return handMotion;
        _releaseControllerFrame = timing.sequence;
        if (admission.rebase) {
            if (_hasPreviousHeldRawHandWorld && timing.discontinuity && timing.valid) {
                ROCK_LOG_SAMPLE_WARN(Hand, 1000, "{} throw history rebased after timing discontinuity rawDt={:.4f}",
                    handName(), timing.rawDeltaSeconds);
            }
            _controllerReleaseHistory.clear();
            _objectReleaseHistory.clear();
            _releaseObjectNotBefore = release_velocity::sampleTimeSeconds();
            _hasPreviousHeldRawHandWorld = false;
        }
        const float deltaTime = admission.deltaSeconds;
        const bool usableDeltaTime = std::isfinite(deltaTime) && deltaTime > 0.000001f;
        const RE::NiPoint3 currentHandPositionHavok = gamePointToHavokPoint(handWorldTransform.translate);
        _lastHeldHandPositionHavok = currentHandPositionHavok;
        _hasLastHeldHandPositionHavok = true;

        // The hand's world motion is the arm's motion in player space plus the
        // player's locomotion. A throw scales only the arm; the released object
        // carries locomotion at exactly the player's speed. The previous hand
        // pose is re-based into this frame's room, so the difference is the
        // arm alone; the room's turning (snap or smooth) is neither and is not
        // passed to the object.
        const auto& playerSpace = runtime_state::currentFrame().playerSpace;
        const bool samePlayerSpace = playerSpace.valid && _previousHeldRoomSource == playerSpace.source;
        if (_hasPreviousHeldRawHandWorld && samePlayerSpace && usableDeltaTime) {
            const RE::NiTransform previousHandInCurrentRoom = multiplyTransforms(playerSpace.world,
                multiplyTransforms(invertTransform(_previousHeldRoomWorld), _previousHeldRawHandWorld));
            handMotion.localLinearVelocityHavok = scalePoint(
                currentHandPositionHavok - gamePointToHavokPoint(previousHandInCurrentRoom.translate), 1.0f / deltaTime);
            handMotion.hasLocalLinearVelocity = true;

            handMotion.angularVelocityRadiansPerSecond = angularVelocityFromRotationDelta(
                previousHandInCurrentRoom.rotate, handWorldTransform.rotate, deltaTime);
            handMotion.hasAngularVelocity = lengthSquared(handMotion.angularVelocityRadiansPerSecond) > 0.000001f;

            const RE::NiPoint3 locomotionVelocityHavok = scalePoint(
                gamePointToHavokPoint(playerSpace.world.translate - _previousHeldRoomWorld.translate),
                1.0f / deltaTime);
            _controllerReleaseHistory.append(timing.sequence, timing.elapsedGameSeconds,
                handMotion.localLinearVelocityHavok, handMotion.angularVelocityRadiansPerSecond,
                locomotionVelocityHavok);
        }

        // Without player space there is no frame to measure the arm in; the
        // next sample with one starts a new pair.
        _previousHeldRawHandWorld = handWorldTransform;
        _previousHeldRoomWorld = playerSpace.world;
        _previousHeldRoomSource = playerSpace.valid ? playerSpace.source : nullptr;
        _hasPreviousHeldRawHandWorld = playerSpace.valid;
        return handMotion;
    }

    void Hand::recordHeldObjectVelocitySample(RE::hknpWorld* world)
    {
        const auto& timing = runtime_state::currentFrame().timing;
        if (!timing.valid || timing.discontinuity || timing.menuPaused)
            return;
        ReleaseObjectSample sample{};
        {
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            sample = _releaseObjectSample;
        }
        if (sample.world != world || sample.bodyId != _savedObjectState.bodyId.value ||
            sample.grabTrace != _grabFrame.traceId || !_objectReleaseHistory.needsSource(sample.solve) ||
            !release_velocity::usablePhysicsSample(sample.solve, sample.capturedAt, _releaseObjectNotBefore,
                release_velocity::sampleTimeSeconds()))
            return;
        _objectReleaseHistory.append(sample.solve, sample.capturedAt, sample.velocity);
    }

    void Hand::captureHeldReleaseMotion(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform,
        const game_frame_timing_policy::GameFrameTiming& timing)
    {
        if (!isHolding() || !world) {
            return;
        }

        recordHeldControllerMotionSample(handWorldTransform, timing);
        recordHeldObjectVelocitySample(world);
    }

    void Hand::captureCarriedWeaponMotion(const RE::NiTransform& handWorldTransform,
        const game_frame_timing_policy::GameFrameTiming& timing)
    {
        // A held object's update owns the history while the hand holds one.
        if (!isHolding())
            recordHeldControllerMotionSample(handWorldTransform, timing);
    }

    void Hand::applyReleaseVelocitySnapshot(RE::hknpWorld* world,
        const GrabReleaseOutcome::VelocitySnapshot& snapshot) const
    {
        if (!world || !snapshot.available) {
            return;
        }

        std::vector<std::uint32_t> bodyIds;
        bodyIds.reserve(snapshot.bodyCount);
        const auto count =
            (std::min<std::uint32_t>)(snapshot.bodyCount, static_cast<std::uint32_t>(snapshot.bodyIds.size()));
        for (std::uint32_t i = 0; i < count; ++i) {
            bodyIds.push_back(snapshot.bodyIds[i]);
        }

        const auto releaseActivation = activateHeldObjectBodySet(world, snapshot.primaryBodyId.value, bodyIds);
        if (releaseActivation.failedActivationCount > 0) {
            ROCK_LOG_WARN(Hand,
                "{} hand pending-transfer release activation incomplete: primaryBody={} bodies={} activated={} "
                "failed={}",
                handName(), snapshot.primaryBodyId.value, releaseActivation.bodyCount, releaseActivation.activatedCount,
                releaseActivation.failedActivationCount);
        }

        setHeldVelocity(world, snapshot.primaryBodyId, bodyIds, snapshot.linearVelocityHavok,
            snapshot.angularVelocityRadiansPerSecond, snapshot.overrideAngularVelocity);
    }

    void Hand::publishHeldBodyScope(RE::hknpWorld* world)
    {
        _heldBodyIdsSequence.fetch_add(1, std::memory_order_acq_rel);
        {
            int count = (std::min)(static_cast<int>(_heldBodyIds.size()), MAX_HELD_BODIES);
            for (int i = 0; i < count; i++) {
                _heldBodyIdsSnapshot[i].store(_heldBodyIds[i], std::memory_order_release);
            }
            _heldBodyIdsCount.store(count, std::memory_order_release);
        }

        _heldBodyIdsSequence.fetch_add(1, std::memory_order_release);

        // A living actor's limb lags the hand by design and his other bodies
        // follow his animation. Presenting them would draw every body of his
        // ragdoll shifted by the hand's lead over the held limb: his hands and
        // feet shook with a shaken head. His nodes stay with his own bodies.
        if (_savedObjectState.liveLimb) {
            held_scene_presentation::clearHeldBodies(_isLeft);
            return;
        }

        held_scene_presentation::Registration sceneRegistration{};
        sceneRegistration.traceId = _grabFrame.traceId;
        for (const std::uint32_t heldBodyId : _heldBodyIds) {
            if (sceneRegistration.count >= held_scene_presentation::kMaxRegisteredBodies) {
                sceneRegistration.complete = false;
                break;
            }

            auto* collisionObject = havok_runtime::getCollisionObjectFromBody(world, RE::hknpBodyId{ heldBodyId });
            if (!collisionObject) {
                sceneRegistration.complete = false;
                continue;
            }

            sceneRegistration.bodies[sceneRegistration.count++] = held_scene_presentation::RegisteredBody{
                .collisionObject = collisionObject,
                .world = world,
                .bodyId = heldBodyId,
            };
        }
        held_scene_presentation::publishHeldBodies(_isLeft, sceneRegistration);
        if (sceneRegistration.count == 0) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB could not publish held-body scene presentation identity: trace={} heldBodies={}",
                handName(), _grabFrame.traceId, _heldBodyIds.size());
        }
    }

    bool Hand::refreshRagdollBodyScope(RE::hknpWorld* world, const GrabReleaseContext& releaseContext)
    {
        if (!grab_target::requiresNativeBodyOwnership(_savedObjectState.targetKind))
            return true;
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::RagdollBodyRefresh);
        const auto component = ragdoll::readComponent(world, _savedObjectState.bodyId.value);
        const auto* liveOwner =
            component.valid ? havok_runtime::getCollisionObjectFromBody(world, _savedObjectState.bodyId) : nullptr;
        bool sameNode = false;
        for (std::size_t i = 0; i < component.count; ++i) {
            if (component.bodies[i].body.id == _savedObjectState.bodyId.value)
                sameNode = component.bodies[i].node == _grabFrame.heldNode;
        }
        segment_visibility::VisibleRanges visible{};
        std::uint32_t triangles = 0;
        const bool surfaceVisible = _ragdollGrabSurface && (_ragdollGrabSurface->flags.flags & 1) == 0 &&
            native_memory::tryReadField(_ragdollGrabSurface.get(), VROffset::numTriangles, triangles) &&
            readVisibleTriangleRanges(_ragdollGrabSurface.get(), triangles, visible) &&
            visible.contains(_ragdollGrabTriangle);
        if (!component.valid || !sameNode || !surfaceVisible || liveOwner != _ragdollGrabOwner.get() ||
            component.count > MAX_HELD_BODIES) {
            ROCK_LOG_WARN(Hand, "{} ragdoll hold ended: body={} stage={} ownerMatch={} surfaceVisible={}", handName(),
                _savedObjectState.bodyId.value, component.reason, liveOwner == _ragdollGrabOwner.get(), surfaceVisible);
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }
        bool changed = component.count != _heldBodyIds.size();
        for (const auto id : _heldBodyIds)
            changed = changed || !component.contains(id);
        _heldDriveDecision.includeConnectedMass = !component.fixedAttached && !_savedObjectState.liveLimb;
        if (!changed)
            return true;

        // Native topology changes are rare. Allocation and lease reconciliation
        // occur only on this transition; unchanged frames read bounded arrays.
        std::vector<std::uint32_t> removed, added;
        for (const auto id : _heldBodyIds)
            if (!component.contains(id))
                removed.push_back(id);
        for (std::size_t i = 0; i < component.count; ++i) {
            const auto id = component.bodies[i].body.id;
            if (!held_object_body_set_policy::containsBody(_heldBodyIds, id))
                added.push_back(id);
        }
        const auto released = releaseHeldObjectBodyFlagLeases(world, INVALID_BODY_ID, removed,
            _savedObjectState.targetKind, heldBodyFlagLeaseOwner(this), true);
        const auto acquired = acquireHeldObjectBodyFlagLeases(world, INVALID_BODY_ID, added,
            _savedObjectState.targetKind, heldBodyFlagLeaseOwner(this));
        // Include newly acquired leases even on failure so normal release owns all cleanup.
        std::erase_if(_heldBodyIds, [&](auto id) { return !component.contains(id); });
        _heldBodyIds.insert(_heldBodyIds.end(), added.begin(), added.end());
        publishHeldBodyScope(world);
        clearHeldBodyContactSnapshot();
        ROCK_LOG_INFO(Hand, "{} ragdoll component changed: primary={} bodies={} removed={} added={} leaseFailures={}",
            handName(), _savedObjectState.bodyId.value, _heldBodyIds.size(), removed.size(), added.size(),
            released.failedLeaseCount + acquired.failedLeaseCount);
        if (acquired.failedLeaseCount) {
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }
        return true;
    }

    std::size_t Hand::moveBodySetRigidly(RE::hknpWorld* world, std::uint32_t primaryBodyId,
        const std::vector<std::uint32_t>& bodyIds, const RE::NiTransform& worldDelta)
    {
        // One rigid move for the whole set, written once per motion (bodies
        // sharing a motion move with it), with velocity cleared. Every body is
        // resolved and every target validated before the first write.
        if (!world || !isFiniteNiTransform(worldDelta))
            return 0;
        std::array<std::uint32_t, MAX_HELD_BODIES> motions{};
        std::array<std::uint32_t, MAX_HELD_BODIES> bodies{};
        std::array<RE::NiTransform, MAX_HELD_BODIES> targets{};
        std::size_t count = 0;
        const auto add = [&](std::uint32_t bodyId) {
            const auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ bodyId });
            RE::NiTransform current{};
            if (!body || count == bodies.size() ||
                !havok_runtime::tryGetBodyArrayWorldTransform(world, RE::hknpBodyId{ bodyId }, current))
                return false;
            for (std::size_t i = 0; i < count; ++i)
                if (motions[i] == body->motionIndex)
                    return true;
            motions[count] = body->motionIndex;
            bodies[count] = bodyId;
            targets[count] = transform_math::composeTransforms(worldDelta, current);
            return isFiniteNiTransform(targets[count++]);
        };
        if (!add(primaryBodyId))
            return 0;
        for (const auto bodyId : bodyIds) {
            if (bodyId != primaryBodyId && !add(bodyId))
                return 0;
        }

        const RE::hkVector4f zero{};
        for (std::size_t i = 0; i < count; ++i) {
            if (!havok_runtime::setBodyTransformDeferred(world, bodies[i], targets[i], 1) ||
                !havok_runtime::setBodyVelocityDeferred(world, bodies[i], zero, zero))
                return 0;
            (void)havok_runtime::activateBody(world, bodies[i]);
        }
        return count;
    }

    bool Hand::placeHeldBodiesOnHandTarget(RE::hknpWorld* world, const RE::NiTransform& primaryBodyWorld,
        const RE::NiTransform& targetBodyWorld, float gripErrorGameUnits)
    {
        // A mechanically anchored set cannot reach the palm, and an incomplete
        // scan does not know every body that would have to move with it.
        // A living actor's body follows its animation; it is never moved onto the hand.
        if (!world || _savedObjectState.liveLimb ||
            _heldDriveDecision.mode == held_object_drive_policy::HeldBodySetDriveMode::FixedAttached ||
            _heldDriveDecision.mode == held_object_drive_policy::HeldBodySetDriveMode::IncompleteNativeScan) {
            return false;
        }
        const auto delta =
            transform_math::composeTransforms(targetBodyWorld, transform_math::invertTransform(primaryBodyWorld));
        const auto count = moveBodySetRigidly(world, _savedObjectState.bodyId.value, _heldBodyIds, delta);
        if (count == 0)
            return false;
        ROCK_LOG_WARN(Hand,
            "{} hand GRAB placed stuck object on the hand target: trace={} ref={:08X} gripError={:.2f}gu "
            "elapsed={:.3f}s motions={} heldBodies={}",
            handName(), _grabFrame.traceId, _savedObjectState.refr ? _savedObjectState.refr->GetFormID() : 0,
            gripErrorGameUnits, _grabStartTime, count, _heldBodyIds.size());
        return true;
    }

    bool Hand::validateHeldObjectUpdate(RE::hknpWorld* world, const GrabReleaseContext& releaseContext)
    {
        if (!refreshRagdollBodyScope(world, releaseContext))
            return false;
        if (_grabAuthorityProxyReleasePending.load(std::memory_order_acquire) || !_activeConstraint.isValid() ||
            !_grabAuthorityProxy.isValid()) {
            ROCK_LOG_WARN(Hand, "{} hand release: proxy constraint authority marked grab invalid", handName());
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }

        if (!_savedObjectState.refr || _savedObjectState.refr->IsDeleted() || _savedObjectState.refr->IsDisabled()) {
            ROCK_LOG_WARN(Hand, "{} hand held reference invalid: grab={} ref={:08X} missing={} deleted={} disabled={}",
                handName(), _grabFrame.traceId, _savedObjectState.refr ? _savedObjectState.refr->GetFormID() : 0,
                !_savedObjectState.refr, _savedObjectState.refr && _savedObjectState.refr->IsDeleted(),
                _savedObjectState.refr && _savedObjectState.refr->IsDisabled());
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }

        return true;
    }

    std::uint64_t Hand::runtimeFrameForObjectTarget() const { return runtime_state::currentFrame().frameIndex; }

    bool Hand::readHeldObjectRoot(RE::hknpWorld* world, RE::NiTransform& root) const
    {
        RE::NiTransform body;
        if (!isHolding() || !_savedObjectState.isValid() ||
            !tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, body))
            return false;
        root = multiplyTransforms(body, invertTransform(_grabFrame.rootBodyLocal));
        return isFiniteNiTransform(root);
    }

    api::Status Hand::setHeldObjectTarget(api::OwnerToken owner, const api::grab::HeldObjectTarget& request,
        std::span<const std::uint32_t> weaponBodies)
    {
        if (!isHolding() || !getHeldRef() || getHeldRef()->GetFormID() != request.referenceFormId)
            return api::Status::HandUnavailable;
        if (_savedObjectState.targetKind != grab_target::Kind::LooseObject || _heldObjectIsLooseWeapon ||
            _grabFrame.looseObjectSharedPeerTrace || !_activeConstraint.isValid())
            return api::Status::TargetUnavailable;
        auto* root = getHeldRef()->Get3D();
        if (!root || !std::isfinite(request.rootWorld.scale) ||
            std::abs(request.rootWorld.scale - root->world.scale) > .001f)
            return api::Status::InvalidArgument;
        for (const auto value : request.rootWorld.translate)
            if (!std::isfinite(value))
                return api::Status::InvalidArgument;
        for (int row = 0; row < 3; ++row)
            for (int other = 0; other < 3; ++other) {
                float dot = 0;
                for (int c = 0; c < 3; ++c)
                    dot += request.rootWorld.rotate[row * 3 + c] * request.rootWorld.rotate[other * 3 + c];
                if (!std::isfinite(dot) || std::abs(dot - (row == other ? 1.f : 0.f)) > .001f)
                    return api::Status::InvalidArgument;
            }
        const auto* r = request.rootWorld.rotate;
        const float determinant = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6]) +
            r[2] * (r[3] * r[7] - r[4] * r[6]);
        if (determinant < .999f)
            return api::Status::InvalidArgument;
        if (_heldBodyIds.size() * weaponBodies.size() > _consumerPairs.size())
            return api::Status::CapacityFull;
        const auto status =
            _consumerObjectTarget.set(owner, heldGrabIdentity(), runtimeFrameForObjectTarget(), request);
        if (status != api::Status::Ok)
            return status;
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        _consumerPairCount = 0;
        _consumerPairsExpire = _consumerObjectTarget.expires;
        for (const auto held : _heldBodyIds)
            for (const auto weapon : weaponBodies)
                _consumerPairs[_consumerPairCount++] = { held, weapon, 0 };
        return api::Status::Ok;
    }

    api::Status Hand::clearHeldObjectTarget(api::OwnerToken owner)
    {
        const auto result = _consumerObjectTarget.clear(owner, heldGrabIdentity(), runtimeFrameForObjectTarget());
        if (result == api::Status::Ok) {
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            _consumerPairCount = 0;
        }
        return result;
    }

    void Hand::flushConsumerObjectPairs(RE::hknpWorld* world)
    {
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        if (world != _grabAuthorityProxyHknpWorld || provider::runtime::currentGameFrameIndex() > _consumerPairsExpire)
            _consumerPairCount = 0;
        const auto result = _consumerPairLeases.reconcile(world, _consumerPairs.data(), _consumerPairCount);
        if (_consumerPairCount && (!result.filterAvailable || result.activePairCount != _consumerPairCount)) {
            _grabAuthorityProxyReleasePending.store(true, std::memory_order_release);
            ROCK_LOG_SAMPLE_WARN(Hand, 1000, "Held object connection pairs unavailable; releasing guided object");
        }
    }

    bool Hand::coordinateLooseObjectProxy(RE::hknpWorld* world, Hand* peer, const RE::NiTransform* peerHandWorld,
        RE::NiTransform& proxyWorld)
    {
        const bool sameObject = isHolding() && peer && peerHandWorld && peer->isHolding() && getHeldRef() &&
            getHeldRef() == peer->getHeldRef();
        const bool sameBody = sameObject && _savedObjectState.bodyId.value == peer->_savedObjectState.bodyId.value;
        // Detached pieces can retain the same actor ref without sharing a
        // rigid root. Only two grips on the same ragdoll body may share a pivot.
        const bool sameAssembly = sameObject &&
            (sameBody ||
                (!grab_target::requiresNativeBodyOwnership(_savedObjectState.targetKind) &&
                    !grab_target::requiresNativeBodyOwnership(peer->_savedObjectState.targetKind)));
        const bool shared = sameAssembly &&
            held_object_drive_policy::canShareTwoHandPivot(_heldDriveDecision.mode, peer->_heldDriveDecision.mode,
                sameBody, isHoldingLooseWeapon() && peer->isHoldingLooseWeapon());
        const auto finishOldPair = [](Hand& hand, std::uint64_t peerTrace) {
            auto& frame = hand._grabFrame;
            if (frame.looseObjectSharedPeerTrace && frame.looseObjectSharedPeerTrace != peerTrace &&
                frame.looseObjectSharedWithoutPivot) {
                // The object was never turned by the pair, so there is no
                // shared pose to keep: the remaining hand holds it the way it
                // grabbed it.
                frame.looseObjectSharedWithoutPivot = false;
                frame.looseObjectSharedPeerTrace = 0;
                ROCK_LOG_INFO(Hand, "{} hand close two-hand hold ended; holding as grabbed", hand.handName());
            }
            if (frame.looseObjectSharedPeerTrace && frame.looseObjectSharedPeerTrace != peerTrace) {
                // A remaining firing grip returns to its solo aim. A support
                // carry or prop retains its acquired orientation.
                const bool preserveSharedPose = !hand.isHoldingFiringGrip();
                frame.looseObjectSoloProxyCorrection = preserveSharedPose
                    ? frame.looseObjectSharedProxyCorrection
                    : transform_math::makeIdentityTransform<RE::NiTransform>();
                frame.hasLooseObjectSoloProxyCorrection = preserveSharedPose;
                frame.looseObjectSharedPeerTrace = 0;
                ROCK_LOG_INFO(Hand, "{} hand loose two-hand pivot ended; {}", hand.handName(),
                    preserveSharedPose ? "preserving remaining grip" : "restoring solo weapon grip");
            }
        };
        const auto correctedProxy = [](const Hand& hand, RE::NiTransform proxy) {
            if (hand._grabFrame.hasLooseObjectSoloProxyCorrection) {
                proxy.rotate = transform_math::transposeRotation(proxy.rotate);
                proxy = transform_math::composeTransforms(proxy, hand._grabFrame.looseObjectSoloProxyCorrection);
                proxy.rotate = transform_math::transposeRotation(proxy.rotate);
            }
            return proxy;
        };
        const auto readPresentedBody = [world](const Hand& hand, RE::NiTransform& body) {
            if (hand._grabFrame.heldNode && isFiniteNiTransform(hand._grabFrame.heldNode->world)) {
                body = transform_math::composeTransforms(hand._grabFrame.heldNode->world,
                    hand._grabFrame.authority.bodyLocal);
                return isFiniteNiTransform(body);
            }
            return hand.tryGetGrabDriveObjectWorldTransform(world, hand._savedObjectState.bodyId, body) &&
                isFiniteNiTransform(body);
        };
        const bool leavingPair = _grabFrame.looseObjectSharedPeerTrace != 0 && !shared;
        finishOldPair(*this, shared ? peer->_grabFrame.traceId : 0);
        if (!shared) {
            _looseSharedAcquisition = {};
            _looseSharedAcquisitionPeer = 0;
            _looseSharedAcquisitionFrame = 0;
            _looseSharedGunstockAim = false;
            _looseSharedHandBlendAlpha = -1.0f;
            proxyWorld = correctedProxy(*this, proxyWorld);
            if (leavingPair && isHoldingLooseWeapon()) {
                RE::NiTransform presentedBody{};
                if (!readPresentedBody(*this, presentedBody))
                    return false;
                if (!isHoldingFiringGrip()) {
                    // When the firing hand leaves, keep the support carry as
                    // displayed. Reattaching a firing grip will acquire it anew.
                    const auto carriedProxy = grab_frame_math::generatedProxyFromObjectWorld(presentedBody,
                        _grabFrame.proxyAuthorityBodyHandSpace);
                    auto physical = proxyWorld;
                    physical.rotate = transform_math::transposeRotation(physical.rotate);
                    if (_grabFrame.hasLooseObjectSoloProxyCorrection)
                        physical = transform_math::composeTransforms(physical,
                            transform_math::invertTransform(_grabFrame.looseObjectSoloProxyCorrection));
                    auto carried = carriedProxy;
                    carried.rotate = transform_math::transposeRotation(carried.rotate);
                    _grabFrame.looseObjectSoloProxyCorrection =
                        transform_math::composeTransforms(transform_math::invertTransform(physical), carried);
                    _grabFrame.hasLooseObjectSoloProxyCorrection = true;
                    proxyWorld = carriedProxy;
                }
                _grabOffsetAcquisition = grab_offset_acquisition::beginWeapon(proxyWorld, presentedBody,
                    _grabFrame.proxyAuthorityBodyHandSpace, _grabFrame.authority.pivotBConstraintLocalGame,
                    hand_visual_lerp_math::kEquippedWeaponReturnConfig);
                _looseWeaponReturnStartFrame = runtime_state::currentFrame().timing.sequence;
            }
            return isFiniteNiTransform(proxyWorld);
        }
        finishOldPair(*peer, _grabFrame.traceId);
        RE::NiTransform peerProxy{};
        const char* peerSource = "unavailable";
        if (!peer->resolveGrabAuthorityProxyFrame(world, peerProxy, peerSource,
                GrabAuthorityProxyFramePolicy::PreferQueuedPalmTarget) ||
            !frik_hand_world_authority::filterHeldProxyWorld(peer->_isLeft, peerProxy)) {
            // A peer palm frame missing for a frame is not release intent: this
            // hand keeps its own target and the pair resumes when it returns.
            ROCK_LOG_SAMPLE_WARN(Hand, 1000, "Loose two-hand pivot missing peer palm source={}; holding own target",
                peerSource);
            proxyWorld = correctedProxy(*this, proxyWorld);
            return isFiniteNiTransform(proxyWorld);
        }
        // A firing seat owns the pivot. For two arbitrary mesh seats, keep
        // the first grab as the pivot until that hand releases.
        const auto firingSeat = [](const Hand& hand) {
            return hand._grabFrame.syntheticLooseWeaponPrimaryAttach && !hand._grabFrame.authoredLooseWeaponSupportGrip;
        };
        const bool thisPrimary =
            firingSeat(*this) != firingSeat(*peer) ? firingSeat(*this) : _grabFrame.traceId < peer->_grabFrame.traceId;
        Hand& primary = thisPrimary ? *this : *peer;
        Hand& support = thisPrimary ? *peer : *this;
        const bool sharedFiringStation =
            loose_weapon_authored_grab_policy::sharedFiringZone(primary._grabFrame.authoredWeaponArrangement) ||
            loose_weapon_authored_grab_policy::sharedFiringZone(support._grabFrame.authoredWeaponArrangement);
        const auto* weapon = looseWeaponFormFromRef(getHeldRef());
        const bool gunstockAim = g_rockConfig.rockGunstockMode && primary.isHoldingFiringGrip() &&
            support.isHoldingLooseWeapon() && !sharedFiringStation && weapon &&
            weapon->weaponData.type == RE::WEAPON_TYPE::kGun;
        const auto primaryProxy = correctedProxy(primary, thisPrimary ? proxyWorld : peerProxy);
        const auto supportProxy = correctedProxy(support, thisPrimary ? peerProxy : proxyWorld);
        // One driven body is its own common frame, including two seats on a
        // jointed object's same part. Different rigid bodies use the assembly
        // root; independent articulated parts keep their existing joint motion.
        // Gunstock forward is weapon-root +Y, never the selected collision
        // body's axes. Keep body-to-root registration even for a shared body.
        const auto primaryBodyLocal = sameBody && !gunstockAim
            ? transform_math::makeIdentityTransform<RE::NiTransform>()
            : primary._grabFrame.rootBodyLocal;
        const auto supportBodyLocal = sameBody && !gunstockAim
            ? transform_math::makeIdentityTransform<RE::NiTransform>()
            : support._grabFrame.rootBodyLocal;
        const auto primaryBody = grab_frame_math::objectFromGeneratedProxyLocalSpace(primaryProxy,
            primary._grabFrame.proxyAuthorityBodyHandSpace);
        const auto root =
            transform_math::composeTransforms(primaryBody, transform_math::invertTransform(primaryBodyLocal));
        const auto primaryLocal =
            transform_math::localPointToWorld(primaryBodyLocal, primary.activeProxyConstraintPivotBLocalGame());
        const auto supportLocal =
            transform_math::localPointToWorld(supportBodyLocal, support.activeProxyConstraintPivotBLocalGame());
        const auto primaryTarget =
            generatedProxyLocalPointToWorld(primaryProxy, primary._grabFrame.authority.pivotAHandBodyLocalGame);
        const auto supportTarget =
            generatedProxyLocalPointToWorld(supportProxy, support._grabFrame.authority.pivotAHandBodyLocalGame);
        const auto supportBase = transform_math::localPointToWorld(root, supportLocal);
        const float separation =
            pointDistanceGameUnits(transform_math::localPointToWorld(root, primaryLocal), supportBase);
        /*
         * The pivot aims the object along the line between the two hands. With
         * the grip points closer together than the hands can be (two pinches
         * on one cartridge are about a unit apart, the hands' own tremor is
         * larger), that line swings with every tremor: the hands fought over
         * the object, and the hand left holding it kept whatever angle the
         * fight ended on. Such an object is not aimed: it stays as the first
         * hand holds it, the second hand holds on to it, and when either lets
         * go the other holds it the way it grabbed it. The grip points are
         * fixed on the object, so a pair never changes kind while it lasts.
         */
        constexpr float kMinimumPivotSeparationGameUnits = 6.0f;
        const bool withoutPivot = !isHoldingLooseWeapon() && !peer->isHoldingLooseWeapon() &&
            separation < kMinimumPivotSeparationGameUnits;
        const auto lockedSupport =
            makeLockedSupportGripTarget(primaryTarget, supportTarget, supportBase, separation, 0.001f);
        const WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3> solverInput{
            .weaponWorldTransform = root,
            .primaryGripLocal = primaryLocal,
            .supportGripLocal = supportLocal,
            .primaryTargetWorld = primaryTarget,
            .supportTargetWorld = lockedSupport,
        };
        WeaponTwoHandedSolverResult<RE::NiTransform> solved{};
        if (gunstockAim) {
            RE::NiTransform primaryWand{}, supportWand{};
            if (frik_hand_world_authority::tryGetHandlingWandWorld(primary._isLeft, primaryWand) &&
                frik_hand_world_authority::tryGetHandlingWandWorld(support._isLeft, supportWand)) {
                // Acquisition is already eased below. Solve the full stock
                // endpoint here, exactly as equipped dynamic acquisition does.
                solved = gunstock_mode::solveVirtualAim(solverInput, primaryWand, supportWand, {}, 1.0f);
            }
            if (!solved.solved) {
                // Tracking loss is not release intent. Both constraints hold
                // the current displayed assembly until a valid baseline returns.
                RE::NiTransform presentedBody{};
                if (!readPresentedBody(*this, presentedBody))
                    return false;
                primary._looseSharedAcquisitionPeer = 0;
                proxyWorld = grab_frame_math::generatedProxyFromObjectWorld(presentedBody,
                    _grabFrame.proxyAuthorityBodyHandSpace);
                ROCK_LOG_SAMPLE_WARN(Hand, 1000, "{} loose gunstock waiting for tracked controller baseline",
                    handName());
                return isFiniteNiTransform(proxyWorld);
            }
        } else if (!sharedFiringStation && !withoutPivot) {
            solved = solveTwoHandedWeaponTransformFrikPivot(solverInput);
        }
        // Coincident seats cannot define an aim axis. Both constraints then
        // share the primary carry target until separation defines that axis.
        if (!sharedFiringStation && !withoutPivot && !solved.solved) {
            ROCK_LOG_SAMPLE_WARN(Hand, 1000,
                "Loose two-hand pivot degenerate axis: separation={:.4f}; sharing primary target", separation);
        }
        auto sharedRoot = solved.solved ? solved.weaponWorldTransform : root;
        if (isHoldingLooseWeapon() && peer->isHoldingLooseWeapon()) {
            const auto targetBody = transform_math::composeTransforms(sharedRoot, primaryBodyLocal);
            const auto targetInProxy = grab_frame_math::objectInGeneratedProxyLocalSpace(primaryProxy, targetBody);
            const auto pivot = primary.activeProxyConstraintPivotBLocalGame();
            auto& acquisition = primary._looseSharedAcquisition;
            const auto sequence = runtime_state::currentFrame().timing.sequence;
            if (primary._looseSharedAcquisitionPeer != support._grabFrame.traceId ||
                primary._looseSharedGunstockAim != gunstockAim) {
                RE::NiTransform liveBody{};
                if (!readPresentedBody(primary, liveBody))
                    return false;
                acquisition = grab_offset_acquisition::begin(primaryProxy, liveBody, targetInProxy, pivot);
                if (acquisition.active) {
                    const auto liveRoot =
                        transform_math::composeTransforms(liveBody, transform_math::invertTransform(primaryBodyLocal));
                    const float distance =
                        (std::max)(pointDistanceGameUnits(transform_math::localPointToWorld(liveRoot, primaryLocal),
                                       primaryTarget),
                            pointDistanceGameUnits(transform_math::localPointToWorld(liveRoot, supportLocal),
                                transform_math::localPointToWorld(sharedRoot, supportLocal)));
                    const float linearDuration = hand_visual_lerp_math::computeDistanceMappedDurationGameUnits(distance,
                        g_rockConfig.rockWeaponSupportGripHandLerpTimeMin,
                        g_rockConfig.rockWeaponSupportGripHandLerpTimeMax,
                        g_rockConfig.rockWeaponSupportGripHandLerpMinDistance,
                        g_rockConfig.rockWeaponSupportGripHandLerpMaxDistance);
                    const float angularDuration = hand_visual_lerp_math::computeAngleMappedDurationDegrees(
                        hand_visual_lerp_math::rotationDistanceDegrees(liveRoot, sharedRoot),
                        g_rockConfig.rockWeaponSupportGripHandLerpTimeMin,
                        g_rockConfig.rockWeaponSupportGripHandLerpTimeMax,
                        hand_visual_lerp_math::kEquippedWeaponReturnConfig.minAngleDegrees,
                        hand_visual_lerp_math::kEquippedWeaponReturnConfig.maxAngleDegrees);
                    acquisition.durationSeconds = g_rockConfig.rockWeaponSupportGripHandLerpEnabled
                        ? (std::max)(linearDuration, angularDuration)
                        : 0.0f;
                }
                primary._looseSharedAcquisitionPeer = support._grabFrame.traceId;
                primary._looseSharedAcquisitionFrame = sequence;
                primary._looseSharedGunstockAim = gunstockAim;
                // A live mode change/tracking recovery reuses these grips.
                // Rebase the wrist too; replaying its original free-hand start
                // would produce a hand snap even though the weapon is eased.
                Hand& joining = primary._grabFrame.traceId > support._grabFrame.traceId ? primary : support;
                if (joining._hasGrabVisualHandTransform) {
                    auto wrist = joining._grabVisualHandTransform;
                    RE::NiTransform presented{};
                    if (frik_hand_world_authority::tryGetPresentedHandWorld(joining._isLeft, presented) &&
                        isUsableGrabVisualTransform(presented))
                        wrist = presented;
                    joining._grabVisualHandLerpStartTransform = wrist;
                    joining._grabVisualHandLerpElapsedSeconds = 0.0f;
                    joining._grabVisualHandLerpDurationSeconds = acquisition.durationSeconds;
                }
                ROCK_LOG_INFO(Hand, "Loose weapon handling ref={:08X} firingHand={} gunstock={} blend={:.3f}s",
                    getHeldRef()->formID, primary.handName(), gunstockAim, acquisition.durationSeconds);
            }
            const auto blendedProxy =
                grab_offset_acquisition::advanceShared(acquisition, primary._looseSharedAcquisitionFrame, sequence,
                    primaryProxy, targetInProxy, pivot, runtime_state::currentFrame().deltaSeconds);
            primary._looseSharedHandBlendAlpha = -1.0f;
            support._looseSharedHandBlendAlpha = -1.0f;
            const float alpha =
                hand_visual_lerp_math::timedBlendAlpha(acquisition.elapsedSeconds, acquisition.durationSeconds);
            Hand& joining = primary._grabFrame.traceId > support._grabFrame.traceId ? primary : support;
            joining._looseSharedHandBlendAlpha = !sharedFiringStation && acquisition.durationSeconds > 0.0f
                ? alpha * alpha * (3.0f - 2.0f * alpha)
                : -1.0f;
            sharedRoot = transform_math::composeTransforms(
                grab_frame_math::objectFromGeneratedProxyLocalSpace(blendedProxy, targetInProxy),
                transform_math::invertTransform(primaryBodyLocal));
        }
        const auto sharedBody =
            transform_math::composeTransforms(sharedRoot, thisPrimary ? primaryBodyLocal : supportBodyLocal);
        const auto targetProxy =
            grab_frame_math::generatedProxyFromObjectWorld(sharedBody, _grabFrame.proxyAuthorityBodyHandSpace);
        if (!isFiniteNiTransform(targetProxy) || targetProxy.scale <= 0.0001f) {
            return false;
        }
        auto physicalNi = proxyWorld;
        physicalNi.rotate = transform_math::transposeRotation(physicalNi.rotate);
        auto targetNi = targetProxy;
        targetNi.rotate = transform_math::transposeRotation(targetNi.rotate);
        _grabFrame.looseObjectSharedProxyCorrection =
            transform_math::composeTransforms(transform_math::invertTransform(physicalNi), targetNi);
        if (!_grabFrame.looseObjectSharedPeerTrace) {
            ROCK_LOG_INFO(Hand, "{} hand loose two-hand {} started ref={:08X} primary={} separation={:.3f}",
                handName(), withoutPivot ? "close hold (no pivot, follows the primary hand)" : "pivot",
                getHeldRef()->GetFormID(), primary.handName(), separation);
        }
        _grabFrame.looseObjectSharedWithoutPivot = withoutPivot;
        _grabFrame.looseObjectSharedPeerTrace = peer->_grabFrame.traceId;
        _grabFrame.looseObjectSharedPrimaryIsLeft = primary._isLeft;
        proxyWorld = targetProxy;
        return true;
    }

    bool Hand::updateHeldDrive(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, float deltaTime,
        float forceFadeInTime, float tauMin, const GrabReleaseContext& releaseContext, HeldDriveUpdate& update,
        Hand* peerHand, const RE::NiTransform* peerHandWorld)
    {
        update.proxyAuthorityWorld = handWorldTransform;
        update.hasProxyAuthorityFrame = resolveGrabAuthorityProxyFrame(world, update.proxyAuthorityWorld,
            update.proxyAuthoritySource, GrabAuthorityProxyFramePolicy::PreferQueuedPalmTarget);
        if (!update.hasProxyAuthorityFrame ||
            !frik_hand_world_authority::filterHeldProxyWorld(_isLeft, update.proxyAuthorityWorld)) {
            ROCK_LOG_WARN(Hand, "{} hand release: palm anchor proxy frame unavailable while held source={}", handName(),
                update.proxyAuthoritySource);
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }

        if (!coordinateLooseObjectProxy(world, peerHand, peerHandWorld, update.proxyAuthorityWorld)) {
            ROCK_LOG_WARN(Hand, "{} hand release: shared object pivot target unavailable", handName());
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }
        if (_grabFrame.looseObjectSharedPeerTrace) {
            _looseWeaponReturnStartFrame = 0;
            update.proxyAuthoritySource = "sharedLooseObjectPalmPivot";
            if (_grabOffsetAcquisition.active) {
                ROCK_LOG_INFO(Hand, "{} OFFSET_ACQUIRE ended trace={} reason=shared-grip", handName(),
                    _grabFrame.traceId);
                _grabOffsetAcquisition = {};
            }
        }
        const bool consumerTarget = _consumerObjectTarget.current(heldGrabIdentity(), runtimeFrameForObjectTarget()) &&
            !_grabFrame.looseObjectSharedPeerTrace &&
            provider::runtime::authorize(_consumerObjectTarget.owner, api::grab::kInterfaceId, 2, true) ==
                api::Status::Ok;
        if (!consumerTarget && _consumerObjectTarget.owner) {
            _consumerObjectTarget = {};
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            _consumerPairCount = 0;
        }
        if (consumerTarget) {
            // Change only the queued proxy target. Frozen body/constraint
            // relations, finite-force motors and collision stay ROCK-owned.
            const auto& source = _consumerObjectTarget.target;
            RE::NiTransform target;
            for (int row = 0; row < 3; ++row)
                for (int col = 0; col < 3; ++col)
                    target.rotate.entry[row][col] = source.rotate[row * 3 + col];
            target.translate = { source.translate[0], source.translate[1], source.translate[2] };
            target.scale = source.scale;
            const auto bodyWorld = multiplyTransforms(target, _grabFrame.rootBodyLocal);
            update.proxyAuthorityWorld =
                grab_frame_math::generatedProxyFromObjectWorld(bodyWorld, _grabFrame.proxyAuthorityBodyHandSpace);
            update.proxyAuthoritySource = "consumerHeldObjectTarget";
            _grabOffsetAcquisition = {};
        }
        const bool advancingOffset = _grabOffsetAcquisition.active;
        if (advancingOffset) {
            const bool synchronizeAcquiringWeaponHand =
                isHoldingLooseWeapon() && !_grabFrame.looseObjectSharedPeerTrace && _looseWeaponReturnStartFrame == 0;
            // Both endpoints ride the current physical hand frame. The fixed
            // body-local grip follows a straight segment while rotation uses
            // the same fraction; no offset is applied around the body origin.
            const float acquisitionDelta = _looseWeaponReturnStartFrame != runtime_state::currentFrame().timing.sequence
                ? held_object_physics_math::measuredDeltaOrZero(deltaTime)
                : 0.0f;
            update.proxyAuthorityWorld = grab_offset_acquisition::advance(_grabOffsetAcquisition,
                update.proxyAuthorityWorld, _grabFrame.proxyAuthorityBodyHandSpace,
                _grabFrame.authority.pivotBConstraintLocalGame, acquisitionDelta);
            if (synchronizeAcquiringWeaponHand) {
                const float alpha = hand_visual_lerp_math::timedBlendAlpha(_grabOffsetAcquisition.elapsedSeconds,
                    _grabOffsetAcquisition.durationSeconds);
                _looseSharedHandBlendAlpha = alpha * alpha * (3.0f - 2.0f * alpha);
            }
            if (!_grabOffsetAcquisition.active)
                _looseWeaponReturnStartFrame = 0;
            update.proxyAuthoritySource = "offsetAcquisitionGripPath";
            if (!isFiniteNiTransform(update.proxyAuthorityWorld)) {
                ROCK_LOG_WARN(Hand, "{} hand release: invalid offset acquisition target trace={}", handName(),
                    _grabFrame.traceId);
                releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
                return false;
            }
        }
        update.desiredObjectWorld = grab_frame_math::objectFromGeneratedProxyLocalSpace(update.proxyAuthorityWorld,
            _grabFrame.proxyAuthorityHandSpace);
        update.desiredBodyWorld = grab_frame_math::objectFromGeneratedProxyLocalSpace(update.proxyAuthorityWorld,
            _grabFrame.proxyAuthorityBodyHandSpace);
        update.activePivotBBodyLocalGame = activeProxyConstraintPivotBLocalGame();
        update.desiredTargetPointWorld =
            transform_math::localPointToWorld(update.desiredBodyWorld, update.activePivotBBodyLocalGame);

        if (tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, update.solvedBodyWorld)) {
            update.liveGripWorldForAuthority =
                transform_math::localPointToWorld(update.solvedBodyWorld, update.activePivotBBodyLocalGame);
            update.pivotTrackingErrorGameUnits =
                pointDistanceGameUnits(update.liveGripWorldForAuthority, update.desiredTargetPointWorld);
            update.hasPivotTrackingError = true;
            update.grabRotationErrorDegrees = _grabFrame.heldNode
                ? rotationDeltaDegrees(_grabFrame.heldNode->world.rotate, update.desiredObjectWorld.rotate)
                : rotationDeltaDegrees(update.solvedBodyWorld.rotate, update.desiredBodyWorld.rotate);
        }
        if (!update.hasPivotTrackingError) {
            ROCK_LOG_WARN(Hand,
                "{} hand release: held object drive body readback failed before queuing grab authority bodyId={} "
                "phase={}",
                handName(), _savedObjectState.bodyId.value, grab_three_phase::phaseName(_grabAcquisitionPhase));
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }
        if (advancingOffset) {
            _grabOffsetMaximumGripError = (std::max)(_grabOffsetMaximumGripError, update.pivotTrackingErrorGameUnits);
            _grabOffsetMaximumRotationError =
                (std::max)(_grabOffsetMaximumRotationError, update.grabRotationErrorDegrees);
            if (!_grabOffsetAcquisition.active) {
                ROCK_LOG_INFO(Hand,
                    "{} OFFSET_ACQUIRE target-complete trace={} gripError={:.3f}gu rotationError={:.3f}deg "
                    "maxGripError={:.3f}gu maxRotationError={:.3f}deg",
                    handName(), _grabFrame.traceId, update.pivotTrackingErrorGameUnits, update.grabRotationErrorDegrees,
                    _grabOffsetMaximumGripError, _grabOffsetMaximumRotationError);
            }
        }
        // A lagging held object stays held; only a broken (non-finite) pivot
        // relation releases it.
        if (!std::isfinite(update.pivotTrackingErrorGameUnits)) {
            ROCK_LOG_WARN(Hand, "{} hand release: held object pivot error is non-finite", handName());
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Delayed, releaseContext);
            return false;
        }

        // A free loose reference owns the whole model, including branches
        // outside its selected collision owner (e.g. a bottle's opaque mesh).
        // Actor/gore and fixed mechanical scopes retain their per-body path.
        const bool presentReferenceRoot = _heldObjectIsLooseWeapon ||
            (_savedObjectState.targetKind == grab_target::Kind::LooseObject &&
                _heldDriveDecision.mode != held_object_drive_policy::HeldBodySetDriveMode::FixedAttached &&
                _heldDriveDecision.mode != held_object_drive_policy::HeldBodySetDriveMode::IncompleteNativeScan);
        // A live limb is never presented (publishHeldBodyScope).
        const auto presentation = _savedObjectState.liveLimb
            ? held_scene_presentation::TargetTransportPublication{}
            : held_scene_presentation::publishTargetTransport(_isLeft, world,
                _savedObjectState.bodyId.value, _grabFrame.traceId, update.desiredBodyWorld, update.solvedBodyWorld,
                presentReferenceRoot && _savedObjectState.refr ? _savedObjectState.refr->Get3D() : nullptr,
                _grabFrame.rootBodyLocal, deltaTime);
        update.hasPresentedBodyWorld = presentation.applied;
        update.presentedBodyWorld = presentation.presentedBodyWorld;

        update.heldBodyColliding = isHeldBodyColliding();
        const auto heldContactSnapshot = readHeldBodyContactSnapshot();
        update.heldMotorContactSoftening = update.heldBodyColliding;
        update.heldMotorContactReason = update.heldBodyColliding ? "legacy-recent-contact" : "no-recent-contact";
        if (heldContactSnapshot.recent) {
            const RE::NiPoint3 correctionGame = update.desiredTargetPointWorld - update.liveGripWorldForAuthority;
            const RE::NiPoint3 correctionHavok = gamePointToHavokPoint(correctionGame);
            RE::NiTransform heldContactBodyWorld{};
            RE::NiTransform otherContactBodyWorld{};
            const bool hasHeldContactBody = heldContactSnapshot.heldBodyId != INVALID_BODY_ID &&
                tryResolveLiveBodyWorldTransform(world, RE::hknpBodyId{ heldContactSnapshot.heldBodyId },
                    heldContactBodyWorld);
            const bool hasOtherContactBody = heldContactSnapshot.otherBodyId != INVALID_BODY_ID &&
                tryResolveLiveBodyWorldTransform(world, RE::hknpBodyId{ heldContactSnapshot.otherBodyId },
                    otherContactBodyWorld);
            const RE::NiPoint3 heldToOtherHavok = (hasHeldContactBody && hasOtherContactBody)
                ? gamePointToHavokPoint(otherContactBodyWorld.translate - heldContactBodyWorld.translate)
                : RE::NiPoint3{};
            const auto contactSoftening = held_object_contact_policy::evaluateHeldContactMotorSoftening(
                held_object_contact_policy::HeldContactMotorSofteningInput<RE::NiPoint3>{
                    .recentContact = true,
                    .hasCorrectionVector = update.hasPivotTrackingError,
                    .hasHeldToOtherVector = hasHeldContactBody && hasOtherContactBody,
                    .hasContactNormal = heldContactSnapshot.hasNormal,
                    .otherMotion = classifyHeldContactOtherMotion(world, heldContactSnapshot.otherBodyId),
                    .correctionTowardTarget = correctionHavok,
                    .heldToOther = heldToOtherHavok,
                    .contactNormal = heldContactSnapshot.contactNormalHavok,
                });
            update.heldMotorContactSoftening = contactSoftening.soften;
            update.heldMotorContactReason = contactSoftening.reason;
        }

        update.authorityForceScale = held_object_drive_policy::sanitizeMotorAuthorityScale(
            sharedGrabAuthorityForceScale(releaseContext.peerHandStillHolding));
        // Every measured frame queues its target, however long it was: a slow
        // frame is real hand motion. The source clock rebases across hitch
        // intervals and jumps; skipping the frame left the object behind.
        if (held_object_physics_math::measuredDeltaOrZero(deltaTime) > 0.0f) {
            queueProxyGrabAuthorityTarget(update.proxyAuthorityWorld, handWorldTransform, update.proxyAuthoritySource,
                deltaTime, forceFadeInTime, tauMin, update.pivotTrackingErrorGameUnits, update.grabRotationErrorDegrees,
                update.authorityForceScale, update.heldMotorContactSoftening);
        }

        update.averageGrabDeviationGameUnits = recordDeviationAverage(_grabDeviationHistory, _grabDeviationHistoryCount,
            _grabDeviationHistoryNext, update.pivotTrackingErrorGameUnits);

        return true;
    }

    bool Hand::updateHeldVisualPresentation(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform,
        float deltaTime, const HeldDriveUpdate& driveUpdate, const GrabReleaseContext& releaseContext,
        bool& outConvergingAcquisitionPhase)
    {
        const float pivotTrackingErrorGameUnits = driveUpdate.pivotTrackingErrorGameUnits;
        const bool hasPivotTrackingError = driveUpdate.hasPivotTrackingError;
        const bool heldMotorContactSoftening = driveUpdate.heldMotorContactSoftening;
        tickHeldBodyContact();
        outConvergingAcquisitionPhase = _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::NearConverging ||
            _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::GravityPulling;
        const float acquisitionVisualEnvelope = grab_three_phase::computeAcquisitionVisualEnvelopeGameUnits(
            g_rockConfig.rockGrabTouchAcquireDistanceGameUnits, g_rockConfig.rockGrabNearConvergeDistanceGameUnits,
            g_rockConfig.rockGrabAcquisitionVisualStartDistanceGameUnits);
        const float acquisitionVisualAttachEnvelope = (std::min)(acquisitionVisualEnvelope,
            (std::max)(g_rockConfig.rockGrabTouchAcquireDistanceGameUnits, g_rockConfig.rockGrabPocketRadiusGameUnits));
        const bool acquisitionVisualEligible = outConvergingAcquisitionPhase && hasPivotTrackingError &&
            _grabObjectGripAtGrab.valid && pivotTrackingErrorGameUnits <= acquisitionVisualAttachEnvelope;
        const bool preserveTransferredHand = _grabFrame.equippedWeaponTransfer && _grabFrame.authoredWeaponPose.valid();
        const auto visualPublishDecision =
            grab_motion_controller::evaluateVisualHandPublishGate(grab_motion_controller::VisualHandPublishInput{
                .hasTelemetryCapture = _grabFrame.hasTelemetryCapture,
                .touchHeldPhase = _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld,
                .acquisitionVisualEligible = acquisitionVisualEligible,
                .hasPivotTrackingError = hasPivotTrackingError,
                .motorContactSoftening = heldMotorContactSoftening,
                .hasSettledVisualRelation = _grabFrame.seat.hasSettledVisualHandRelation,
                .requiresSettledVisualRelation = _grabFrame.seat.requiresSettledVisualHandRelation,
                .transferredAuthoredGrip = preserveTransferredHand,
            });
        // A transferred grip already owns the hand. Its small constraint-seat
        // correction must carry that hand with the presented weapon, rather than
        // returning the arm to the controller until acquisition finishes.
        const bool weaponVisual = isHoldingLooseWeapon();
        if (_grabFrame.hasTelemetryCapture &&
            (weaponVisual ||
                ((!_grabOffsetAcquisition.active || preserveTransferredHand) && visualPublishDecision.apply))) {
            RE::NiTransform heldVisualNodeWorld{};
            bool hasHeldVisualNodeWorld = false;
            bool heldVisualNodeFromPresentedPose = false;
            if (driveUpdate.hasPresentedBodyWorld) {
                /*
                 * Pose the hand on the pose the scene writer presents THIS
                 * frame. heldNode->world still holds the previous frame's
                 * presented pose here (the writer runs after physics), so
                 * reading it would put the rendered hand one locomotion step
                 * behind the held mesh and the skeleton -- the per-hand arm
                 * stutter measured during stick locomotion.
                 */
                heldVisualNodeWorld =
                    deriveNodeWorldFromBodyWorld(driveUpdate.presentedBodyWorld, _grabFrame.authority.bodyLocal);
                hasHeldVisualNodeWorld = true;
                heldVisualNodeFromPresentedPose = true;
            } else if (_grabFrame.heldNode) {
                heldVisualNodeWorld = _grabFrame.heldNode->world;
                hasHeldVisualNodeWorld = true;
            } else {
                RE::NiTransform grabBodyWorld{};
                if (tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, grabBodyWorld)) {
                    heldVisualNodeWorld = deriveNodeWorldFromBodyWorld(grabBodyWorld, _grabFrame.authority.bodyLocal);
                    hasHeldVisualNodeWorld = true;
                }
            }

            if (hasHeldVisualNodeWorld) {
                /*
                 * ROCK visual hand update:
                 *     adjustedHand = heldObjectWorld * inverse(frozenObjectHandSpace)
                 *
                 * This is intentionally visual-only. The active dynamic grab
                 * drive remains the only object motor authority, so the rendered
                 * hand can settle to the object without feeding wrist/object
                 * rotation back into the grab relation.
                 */
                RE::NiTransform targetVisualHandWorld = hand_visual_lerp_math::buildHeldObjectRelativeHandWorld(
                    heldVisualNodeWorld, _grabFrame.rawHandSpace);
                targetVisualHandWorld.scale = handWorldTransform.scale;

                RE::NiTransform nextVisualHandWorld = targetVisualHandWorld;
                const bool smoothVisualHand = !preserveTransferredHand &&
                    hand_visual_lerp_math::shouldSmoothHeldObjectRelativeHand(isHoldingLooseWeapon()
                            ? g_rockConfig.rockWeaponSupportGripHandLerpEnabled
                            : g_rockConfig.rockGrabHandLerpEnabled,
                        _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld,
                        visualPublishDecision.acquisition, weaponVisual);
                if (!_hasGrabVisualHandTransform) {
                    RE::NiTransform acquisitionStart =
                        _grabVisualReturn.active && isUsableGrabVisualTransform(_grabVisualReturn.lastApplied)
                        ? _grabVisualReturn.lastApplied
                        : handWorldTransform;
                    if (weaponVisual && !_grabVisualReturn.active) {
                        RE::NiTransform presented{};
                        if (frik_hand_world_authority::tryGetPresentedHandWorld(_isLeft, presented) &&
                            isUsableGrabVisualTransform(presented))
                            acquisitionStart = presented;
                    }
                    _grabVisualHandTransform = acquisitionStart;
                    _grabVisualHandLerpStartTransform = acquisitionStart;
                    _grabVisualHandLerpElapsedSeconds = 0.0f;
                    _grabVisualHandLerpDurationSeconds = smoothVisualHand
                        ? (isHoldingLooseWeapon()
                                  ? hand_visual_lerp_math::computeVisualReturnDuration(acquisitionStart,
                                        targetVisualHandWorld, looseWeaponAcquireConfig())
                                  : hand_visual_lerp_math::computeDistanceMappedDurationGameUnits(
                                        hand_visual_lerp_math::distanceGameUnits(acquisitionStart.translate,
                                            targetVisualHandWorld.translate),
                                        g_rockConfig.rockGrabHandLerpTimeMin, g_rockConfig.rockGrabHandLerpTimeMax,
                                        g_rockConfig.rockGrabHandLerpMinDistance,
                                        g_rockConfig.rockGrabHandLerpMaxDistance))
                        : 0.0f;
                    _hasGrabVisualHandTransform = true;
                }

                float visualHandLerpAlpha = 1.0f;
                if (smoothVisualHand) {
                    _grabVisualHandLerpElapsedSeconds = hand_visual_lerp_math::advanceTimedBlendElapsed(
                        _grabVisualHandLerpElapsedSeconds, deltaTime, _grabVisualHandLerpDurationSeconds);
                    const auto advancedVisual = hand_visual_lerp_math::blendTransformOverDuration(
                        _grabVisualHandLerpStartTransform, targetVisualHandWorld, _grabVisualHandLerpElapsedSeconds,
                        _grabVisualHandLerpDurationSeconds);
                    nextVisualHandWorld = advancedVisual.transform;
                    visualHandLerpAlpha = hand_visual_lerp_math::timedBlendAlpha(_grabVisualHandLerpElapsedSeconds,
                        _grabVisualHandLerpDurationSeconds);
                    if (weaponVisual && _looseSharedHandBlendAlpha >= 0.0f) {
                        // Initial wrist acquisition and paired joining use the
                        // weapon's exact eased progress, including its last frame.
                        // Visual-only support retains its independent wrist blend.
                        visualHandLerpAlpha = _looseSharedHandBlendAlpha;
                        nextVisualHandWorld = hand_visual_lerp_math::interpolateTransform(
                            _grabVisualHandLerpStartTransform, targetVisualHandWorld, visualHandLerpAlpha);
                        if (visualHandLerpAlpha >= 1.0f)
                            _grabVisualHandLerpElapsedSeconds = _grabVisualHandLerpDurationSeconds;
                        if (visualHandLerpAlpha < 1.0f) {
                            ROCK_LOG_SAMPLE_DEBUG(Hand, 100,
                                "{} loose grip slurp trace={} paired={} sharedAlpha={:.3f} wristDistance={:.3f}gu "
                                "wristAngle={:.2f}deg",
                                handName(), _grabFrame.traceId, _grabFrame.looseObjectSharedPeerTrace != 0,
                                visualHandLerpAlpha,
                                pointDistanceGameUnits(nextVisualHandWorld.translate, targetVisualHandWorld.translate),
                                hand_visual_lerp_math::rotationDistanceDegrees(nextVisualHandWorld,
                                    targetVisualHandWorld));
                        }
                    }
                } else {
                    _grabVisualHandLerpStartTransform = targetVisualHandWorld;
                    _grabVisualHandLerpElapsedSeconds = 0.0f;
                    _grabVisualHandLerpDurationSeconds = 0.0f;
                }

                const float visualHandDeviationGameUnits =
                    pointDistanceGameUnits(nextVisualHandWorld.translate, handWorldTransform.translate);
                if (!std::isfinite(visualHandDeviationGameUnits)) {
                    ROCK_LOG_WARN(Hand, "{} hand release: visual held-object hand target is non-finite", handName());
                    clearGrabExternalHandWorldTransform(_isLeft);
                    releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Delayed, releaseContext);
                    return false;
                }
                _grabVisualHandTransform = nextVisualHandWorld;
                if (applyGrabExternalHandWorldTransform(_isLeft, _grabVisualHandTransform)) {
                    _lastPublishedGrabVisualHandTransform = _grabVisualHandTransform;
                    _hasLastPublishedGrabVisualHandTransform = true;
                    clearGrabVisualReturn("active-grab-authority-acquired", false);
                    if (!_grabOffsetAcquisition.active &&
                        _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld)
                        _grabFrame.equippedWeaponTransfer = false;
                }

                ROCK_LOG_SAMPLE_DEBUG(Hand, g_rockConfig.rockLogSampleMilliseconds,
                    "{} GRAB VISUAL HAND: relation=heldRelative phase={} visualOnly=yes authority={} follow={} "
                    "lerpAlpha={:.2f} lerpDuration={:.3f}s target=({:.1f},{:.1f},{:.1f}) "
                    "applied=({:.1f},{:.1f},{:.1f}) live=({:.1f},{:.1f},{:.1f}) deviation={:.2f}gu "
                    "normalAuthority=false",
                    handName(), grab_three_phase::phaseName(_grabAcquisitionPhase), visualPublishDecision.reason,
                    smoothVisualHand ? "smoothedAcquisition" : "immediateHeldObject", visualHandLerpAlpha,
                    _grabVisualHandLerpDurationSeconds, targetVisualHandWorld.translate.x,
                    targetVisualHandWorld.translate.y, targetVisualHandWorld.translate.z,
                    _grabVisualHandTransform.translate.x, _grabVisualHandTransform.translate.y,
                    _grabVisualHandTransform.translate.z, handWorldTransform.translate.x,
                    handWorldTransform.translate.y, handWorldTransform.translate.z, visualHandDeviationGameUnits);
            } else {
                if (_hasGrabVisualHandTransform) {
                    clearGrabExternalHandWorldTransform(_isLeft);
                    _hasGrabVisualHandTransform = false;
                    _lastPublishedGrabVisualHandTransform = {};
                    _hasLastPublishedGrabVisualHandTransform = false;
                }
                _grabVisualHandLerpStartTransform = {};
                _grabVisualHandLerpElapsedSeconds = 0.0f;
                _grabVisualHandLerpDurationSeconds = 0.0f;
            }
        } else {
            if (_hasGrabVisualHandTransform) {
                clearGrabExternalHandWorldTransform(_isLeft);
                _hasGrabVisualHandTransform = false;
                _lastPublishedGrabVisualHandTransform = {};
                _hasLastPublishedGrabVisualHandTransform = false;
            }
            _grabVisualHandLerpStartTransform = {};
            _grabVisualHandLerpElapsedSeconds = 0.0f;
            _grabVisualHandLerpDurationSeconds = 0.0f;
        }

        return true;
    }

    void Hand::updateHeldAcquisition(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, float deltaTime,
        const HeldDriveUpdate& driveUpdate, bool convergingAcquisitionPhase)
    {
        const auto& desiredObjectWorld = driveUpdate.desiredObjectWorld;
        const auto& proxyAuthorityWorld = driveUpdate.proxyAuthorityWorld;
        const auto& desiredTargetPointWorld = driveUpdate.desiredTargetPointWorld;
        const auto& activePivotBBodyLocalGame = driveUpdate.activePivotBBodyLocalGame;
        const bool heldBodyColliding = driveUpdate.heldBodyColliding;

        /*
         * Swept arc only. FRIK publishes joint values before the root-flattened
         * scene reflects them. Surface-local corrections built in that same
         * update would use the previous finger skeleton and can lock a closed or
         * displaced pose. Finish the local-transform layer once, on the next
         * TouchHeld update, after the solved joint pose is the live skeleton source.
         */
        if (_grabFingerSweptArc && _grabFingerLocalTransformFinalizePending && _grabFingerPosePublished &&
            _grabFingerPose.solved && _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld) {
            const auto finalFingerPose =
                grab_finger_pose_runtime::resolveSurfaceAimObjectLocal(_grabFingerPose, desiredObjectWorld);
            applySweptArcGrabHandPose(_isLeft, finalFingerPose, _grabFingerJointPose, _hasGrabFingerJointPose,
                _grabFingerLocalTransforms, _grabFingerLocalTransformMask, _hasGrabFingerLocalTransforms, deltaTime,
                /*publishLocalTransforms=*/true,
                /*snapJointPoseToTarget=*/false);
            _grabFingerLocalTransformFinalizePending = false;
            ROCK_LOG_DEBUG(Hand,
                "{} THREE-PHASE GRAB FINAL POSE: local transforms finalized after settled joint publication",
                handName());
            _grabPoseQualityUpdatesRemaining = g_rockConfig.rockDebugTestbenchTelemetry ? 60 : -1;
        }
        if (_grabPoseQualityUpdatesRemaining > 0 && --_grabPoseQualityUpdatesRemaining == 0) {
            logGrabPoseQuality();
            _grabPoseQualityUpdatesRemaining = -1;
            _grabPoseReplay.reset();
        }

        if (convergingAcquisitionPhase && !_grabOffsetAcquisition.active && _grabObjectGripAtGrab.valid) {
            const auto previousAcquisitionPhase = _grabAcquisitionPhase;
            RE::NiTransform grabBodyWorld{};
            const bool hasGrabBody =
                tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, grabBodyWorld);
            const RE::NiPoint3 liveGripWorld = hasGrabBody
                ? transform_math::localPointToWorld(grabBodyWorld, activePivotBBodyLocalGame)
                : RE::NiPoint3{};
            const RE::NiPoint3 targetGripWorld = desiredTargetPointWorld;
            const RE::NiPoint3 gripError = targetGripWorld - liveGripWorld;
            const float gripErrorGameUnits = hasGrabBody
                ? std::sqrt(gripError.x * gripError.x + gripError.y * gripError.y + gripError.z * gripError.z)
                : std::numeric_limits<float>::max();
            const float touchDistance = (std::max)(0.1f, g_rockConfig.rockGrabTouchAcquireDistanceGameUnits);
            const auto convergenceDecision =
                grab_three_phase::evaluateConvergencePromotion(grab_three_phase::ConvergencePromotionInput{
                    .hasGrabBody = hasGrabBody,
                    .heldBodyColliding = heldBodyColliding,
                    .gripErrorGameUnits = gripErrorGameUnits,
                    .previousGripErrorGameUnits = _grabConvergePreviousGripErrorGameUnits,
                    .deltaSeconds = deltaTime,
                    .elapsedSeconds = _grabStartTime,
                    .maxTimeSeconds = g_rockConfig.rockGrabConvergeMaxTimeSeconds,
                    .touchDistanceGameUnits = touchDistance,
                    .pocketRadiusGameUnits = g_rockConfig.rockGrabPocketRadiusGameUnits,
                    .stableInsidePocketSeconds = _grabConvergeStableInsidePocketSeconds,
                    .requiredStableInsidePocketSeconds = g_rockConfig.rockGrabConvergeStableSeconds,
                    .maxSeparatingSpeedGameUnitsPerSecond =
                        g_rockConfig.rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond,
                });
            _grabConvergeStableInsidePocketSeconds = convergenceDecision.nextStableInsidePocketSeconds;
            _grabConvergePreviousGripErrorGameUnits = gripErrorGameUnits;
            const bool reachedTouchRange = convergenceDecision.reachedTouchRange;
            const bool convergenceTimedOutInsidePocket = convergenceDecision.timedOutInsidePocket;
            /*
             * The object always ends in the palm. One that has not arrived by
             * the convergence timeout (wedged, or snagged in transit) is moved
             * onto its hand target; the touch test promotes it on a later frame,
             * never while it is away from the hand. If something knocks it out
             * again, the move repeats once per timeout interval.
             */
            const float convergeTimeout = g_rockConfig.rockGrabConvergeMaxTimeSeconds;
            if (hasGrabBody && !reachedTouchRange && !convergenceTimedOutInsidePocket && convergeTimeout > 0.0f &&
                _grabStartTime - _grabTransitPlacementElapsedSeconds >= convergeTimeout) {
                _grabTransitPlacementElapsedSeconds = _grabStartTime;
                if (placeHeldBodiesOnHandTarget(world, grabBodyWorld, driveUpdate.desiredBodyWorld,
                        gripErrorGameUnits)) {
                    _grabConvergeStableInsidePocketSeconds = 0.0f;
                    _grabConvergePreviousGripErrorGameUnits = std::numeric_limits<float>::max();
                }
            }
            if (convergenceTimedOutInsidePocket) {
                ROCK_LOG_SAMPLE_DEBUG(Hand, g_rockConfig.rockLogSampleMilliseconds,
                    "{} THREE-PHASE GRAB CONVERGE PROMOTION READY: phase={} gripErr={:.2f}gu touch={:.2f}gu "
                    "pocket={:.2f}gu elapsed={:.3f}s colliding={} stableDwell={:.3f}s sepSpeed={:.2f}gu/s",
                    handName(), grab_three_phase::phaseName(previousAcquisitionPhase), gripErrorGameUnits,
                    touchDistance, g_rockConfig.rockGrabPocketRadiusGameUnits, _grabStartTime,
                    heldBodyColliding ? "yes" : "no", _grabConvergeStableInsidePocketSeconds,
                    convergenceDecision.separatingSpeedGameUnitsPerSecond);
            }

            if (hasGrabBody && !reachedTouchRange && !convergenceTimedOutInsidePocket &&
                !_grabFrame.syntheticLooseWeaponPrimaryAttach && g_rockConfig.rockGrabMeshFingerPoseEnabled &&
                _hasGrabFingerPose && _grabFingerPosePublished) {
                const float nearDistance =
                    (std::max)(touchDistance, g_rockConfig.rockGrabNearConvergeDistanceGameUnits);
                const float progressDenominator = (std::max)(0.001f, nearDistance - touchDistance);
                const float acquisitionProgress =
                    1.0f - std::clamp((gripErrorGameUnits - touchDistance) / progressDenominator, 0.0f, 1.0f);
                if (_grabFingerSweptArc) {
                    const auto resolvedTargetPose =
                        grab_finger_pose_runtime::resolveSurfaceAimObjectLocal(_grabFingerPose, desiredObjectWorld);
                    const auto acquisitionFingerPose =
                        buildAcquisitionFingerPose(resolvedTargetPose, acquisitionProgress);
                    applySweptArcGrabHandPose(_isLeft, acquisitionFingerPose, _grabFingerJointPose,
                        _hasGrabFingerJointPose, _grabFingerLocalTransforms, _grabFingerLocalTransformMask,
                        _hasGrabFingerLocalTransforms, deltaTime, false);
                } else {
                    const auto acquisitionFingerPose = buildAcquisitionFingerPose(_grabFingerPose, acquisitionProgress);
                    applyRockGrabHandPose(_isLeft, acquisitionFingerPose, _grabFingerJointPose, _hasGrabFingerJointPose,
                        _grabFingerLocalTransforms, _grabFingerLocalTransformMask, _hasGrabFingerLocalTransforms,
                        deltaTime);
                }
            }

            if (reachedTouchRange || convergenceTimedOutInsidePocket) {
                const char* promotionReason = _grabFrame.syntheticLooseWeaponPrimaryAttach
                    ? "looseWeaponPrimaryAttachSettled"
                    : (reachedTouchRange ? "threePhaseTouchReachedFrozenRelation" : "threePhaseTimeoutInsidePocket");
                /*
                 * The seat was chosen once at capture against the pose the
                 * object rests in, so arrival never re-picks it: it only lets
                 * the visual hand lock onto that frozen relation.
                 */
                if (_grabFrame.syntheticLooseWeaponPrimaryAttach || _grabFrame.seat.requiresSettledVisualHandRelation) {
                    _grabFrame.seat.hasSettledVisualHandRelation = true;
                }
                _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::TouchHeld;
                _grabFrame.fadeInGrabConstraint = false;
                _grabFrame.motorFadeReason = promotionReason;
                ROCK_LOG_DEBUG(Hand,
                    "{} THREE-PHASE GRAB TRANSITION: {} -> TouchHeld relation=frozenRockPointToPalm reason={} "
                    "gripErr={:.2f}gu elapsed={:.3f}s colliding={} posePublished={}",
                    handName(), grab_three_phase::phaseName(previousAcquisitionPhase), promotionReason,
                    gripErrorGameUnits, _grabStartTime, heldBodyColliding ? "yes" : "no",
                    _grabFingerPosePublished ? "yes" : "no");

                if (!_grabFrame.syntheticLooseWeaponPrimaryAttach && g_rockConfig.rockGrabMeshFingerPoseEnabled &&
                    _hasGrabFingerPose) {
                    if (!_grabFingerPosePublished) {
                        const RE::NiTransform currentNodeWorld =
                            deriveNodeWorldFromBodyWorld(grabBodyWorld, _grabFrame.authority.bodyLocal);
                        RE::NiTransform finalPoseHandWorld = hand_visual_lerp_math::buildHeldObjectRelativeHandWorld(
                            currentNodeWorld, _grabFrame.rawHandSpace);
                        finalPoseHandWorld.scale = handWorldTransform.scale;
                        if (_grabFingerSweptArc && !_grabFrame.fingerPoseAimValid &&
                            _grabFrame.pivotAuthority.normalTrusted) {
                            _grabFrame.fingerPoseAimValid = true;
                            _grabFrame.fingerPoseAimReason = "touchHeldNormalTrusted";
                        }
                        RE::NiPoint3 fingerPosePivotWorld = _grabFrame.hasTelemetryCapture
                            ? _grabFrame.authority.grabPivotWorldAtGrab
                            : computeGrabPivotAWorld(world, handWorldTransform);
                        RE::NiPoint3 livePivotAWorld{};
                        if (tryComputeGrabProxyLocalPalmPocketPivotAWorld(world, livePivotAWorld)) {
                            fingerPosePivotWorld = livePivotAWorld;
                        }
                        solveAndPublishGrabFingerPose(GrabFingerSolveFrame{
                            .objectWorld = currentNodeWorld,
                            .liveHandWorld = handWorldTransform,
                            .solveHandWorld = finalPoseHandWorld,
                            .pivotWorld = fingerPosePivotWorld,
                            .touchHeld = true,
                            .stage = "touchHeld",
                        });
                    } else {
                        if (_grabFingerSweptArc) {
                            const RE::NiTransform finalPoseObjectWorld =
                                grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyAuthorityWorld,
                                    _grabFrame.proxyAuthorityHandSpace);
                            const auto finalFingerPose = grab_finger_pose_runtime::resolveSurfaceAimObjectLocal(
                                _grabFingerPose, finalPoseObjectWorld);
                            applySweptArcGrabHandPose(_isLeft, finalFingerPose, _grabFingerJointPose,
                                _hasGrabFingerJointPose, _grabFingerLocalTransforms, _grabFingerLocalTransformMask,
                                _hasGrabFingerLocalTransforms, 0.0f, /*publishLocalTransforms=*/false, true);
                            _grabFingerLocalTransformFinalizePending = _grabFingerPose.solved;
                        } else {
                            // Solved at commit: TouchHeld snaps the acquisition blend to the full pose.
                            applyRockGrabHandPose(_isLeft, _grabFingerPose, _grabFingerJointPose,
                                _hasGrabFingerJointPose, _grabFingerLocalTransforms, _grabFingerLocalTransformMask,
                                _hasGrabFingerLocalTransforms, 0.0f, true);
                            if (_grabFingerPose.solved) {
                                _grabPoseQualityUpdatesRemaining = g_rockConfig.rockDebugTestbenchTelemetry ? 60 : -1;
                            }
                        }
                    }
                }
            }
        }
    }

    void Hand::finalizeHeldObjectUpdate(RE::hknpWorld* world, float forceFadeInTime, const HeldDriveUpdate& driveUpdate)
    {
        const auto& desiredBodyWorld = driveUpdate.desiredBodyWorld;
        const auto& desiredTargetPointWorld = driveUpdate.desiredTargetPointWorld;
        const auto& activePivotBBodyLocalGame = driveUpdate.activePivotBBodyLocalGame;
        const char* proxyAuthoritySource = driveUpdate.proxyAuthoritySource;
        const char* heldMotorContactReason = driveUpdate.heldMotorContactReason;
        const float grabRotationErrorDegrees = driveUpdate.grabRotationErrorDegrees;
        const float authorityForceScale = driveUpdate.authorityForceScale;
        const float averageGrabDeviationGameUnits = driveUpdate.averageGrabDeviationGameUnits;
        const bool hasProxyAuthorityFrame = driveUpdate.hasProxyAuthorityFrame;
        const bool heldBodyColliding = driveUpdate.heldBodyColliding;
        const bool heldMotorContactSoftening = driveUpdate.heldMotorContactSoftening;

        const float fadeDuration = std::max(forceFadeInTime, 0.0001f);
        const float grabFadeFactor =
            _grabFrame.fadeInGrabConstraint ? std::clamp(_grabStartTime / fadeDuration, 0.0f, 1.0f) : 1.0f;

        if (_state == HandState::HeldInit) {
            if (grabFadeFactor >= 0.999f) {
                applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::HeldFadeComplete });
                ROCK_LOG_DEBUG(Hand, "{} hand: HeldInit -> HeldBody ({} dynamic grab fade complete, {:.2f}s)",
                    handName(), kHeldObjectDriveName, _grabStartTime);
            }
        }

        recordHeldObjectVelocitySample(world);

        _heldLogCounter++;
        if (_heldLogCounter >= 45) {
            _heldLogCounter = 0;

            std::uint64_t driveQueuedTargets = 0;
            std::uint64_t driveFlushedTargets = 0;
            std::uint64_t driveFailedFlushes = 0;
            float driveLastFlushDeltaSeconds = 0.0f;
            {
                std::scoped_lock lock(_grabAuthorityProxyMutex);
                driveQueuedTargets = _grabAuthorityProxyQueuedSequence;
                driveFlushedTargets = _grabAuthorityProxyFlushSequence;
                driveFailedFlushes = _grabAuthorityProxyFailedFlushes;
                driveLastFlushDeltaSeconds = _grabAuthorityProxyLastFlushDeltaSeconds;
            }
            RE::NiTransform grabDriveObjectWorld{};
            const bool hasGrabDriveObject =
                tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, grabDriveObjectWorld);

            const bool hasGrabPivotFrame = hasGrabDriveObject;
            const RE::NiPoint3 pivotAWorld = desiredTargetPointWorld;
            const RE::NiPoint3 pivotBWorld = hasGrabPivotFrame
                ? transform_math::localPointToWorld(grabDriveObjectWorld, activePivotBBodyLocalGame)
                : RE::NiPoint3{};
            const RE::NiPoint3 pivotError = pivotAWorld - pivotBWorld;
            const float pivotErrGame = hasGrabPivotFrame
                ? std::sqrt(pivotError.x * pivotError.x + pivotError.y * pivotError.y + pivotError.z * pivotError.z)
                : 0.0f;

            const RE::NiPoint3 bodyDelta =
                hasGrabPivotFrame ? (grabDriveObjectWorld.translate - desiredBodyWorld.translate) : RE::NiPoint3{};
            const float bodyDistGame = hasGrabPivotFrame
                ? std::sqrt(bodyDelta.x * bodyDelta.x + bodyDelta.y * bodyDelta.y + bodyDelta.z * bodyDelta.z)
                : 0.0f;

            float objVelMag = 0.0f;
            {
                auto* objMotion = havok_runtime::getBodyMotion(world, _savedObjectState.bodyId);
                if (objMotion) {
                    const auto& velocity = objMotion->linearVelocity;
                    objVelMag = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
                }
            }

            const std::uint32_t heldFormId = _savedObjectState.refr ? _savedObjectState.refr->GetFormID() : 0;
            const float lastGrabPhysicsHz = _lastGrabPhysicsHz.load(std::memory_order_relaxed);

            ROCK_LOG_DEBUG(Hand,
                "{} HELD dynamic: drive={} bodyDriveMode={} linearScope={} angularScope={} massScope={} looseWeapon={} "
                "formID={:08X} constraint={} queued={} flushed={} failedFlushes={} lastDt={:.6f} proxyFrame={}/{} "
                "phase={} posePublished={} fade={:.2f}/{} reason={} colliding={} motorContact={} contactReason={} "
                "forceBudget={:.2f} physHz={:.1f} longLever={:.1f}gu pivotTrack={:.1f}gu avgTrack={:.1f}gu "
                "rotErr={:.1f}deg bDist={:.1f}gu objVel={:.3f} "
                "paW=({:.1f},{:.1f},{:.1f}) pbW=({:.1f},{:.1f},{:.1f}) "
                "targetBody=({:.1f},{:.1f},{:.1f}) objW=({:.1f},{:.1f},{:.1f})",
                handName(), kHeldObjectDriveName, held_object_drive_policy::modeName(_heldDriveDecision.mode),
                _heldDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
                _heldDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly",
                _heldDriveDecision.includeConnectedMass ? "bodySet" : "primaryOnly",
                _heldObjectIsLooseWeapon ? "yes" : "no", heldFormId,
                _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu, driveQueuedTargets,
                driveFlushedTargets, driveFailedFlushes, driveLastFlushDeltaSeconds, proxyAuthoritySource,
                hasProxyAuthorityFrame ? "ok" : "fallback", grab_three_phase::phaseName(_grabAcquisitionPhase),
                _grabFingerPosePublished ? "yes" : "no", grabFadeFactor, _grabFrame.fadeInGrabConstraint ? "on" : "off",
                _grabFrame.motorFadeReason, heldBodyColliding ? "yes" : "no",
                heldMotorContactSoftening ? "soften" : "preserve", heldMotorContactReason, authorityForceScale,
                lastGrabPhysicsHz, _grabFrame.pivotAuthority.longLeverGameUnits, pivotErrGame,
                averageGrabDeviationGameUnits, grabRotationErrorDegrees, bodyDistGame, objVelMag, pivotAWorld.x,
                pivotAWorld.y, pivotAWorld.z, pivotBWorld.x, pivotBWorld.y, pivotBWorld.z, desiredBodyWorld.translate.x,
                desiredBodyWorld.translate.y, desiredBodyWorld.translate.z, grabDriveObjectWorld.translate.x,
                grabDriveObjectWorld.translate.y, grabDriveObjectWorld.translate.z);
        }

        {
            const auto wakeBodies = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(
                _savedObjectState.bodyId.value, _heldBodyIds);
            for (const auto bodyId : wakeBodies) {
                physics_recursive_wrappers::activateBody(world, bodyId);
            }
        }
    }

    void Hand::updateHeldObject(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, float deltaTime,
        float forceFadeInTime, float tauMin, const GrabReleaseContext& releaseContext, Hand* peerHand,
        const RE::NiTransform* peerHandWorld)
    {
        if (!isHolding() || !world)
            return;
        // A transfer may refresh the assembly after its owning hand already
        // ran earlier in the bilateral input pass. Never tick timers or consume
        // presentation history twice; a new grab resets this frame identity.
        const auto frame = runtime_state::currentFrame().frameIndex;
        if (frame != 0 && _heldObjectUpdateFrame == frame)
            return;
        _heldObjectUpdateFrame = frame;
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabHeldObjectUpdate);
        if (!validateHeldObjectUpdate(world, releaseContext)) {
            return;
        }

        suppressHandCollisionForGrab(world);

        _grabStartTime += held_object_physics_math::finitePositiveOrZero(deltaTime);

        const HeldHandMotionSample handMotion =
            recordHeldControllerMotionSample(handWorldTransform, runtime_state::currentFrame().timing);
        (void)handMotion;

        /*
         * ROCK freezes the visible object/node relation in generated/proxy
         * authority space, then composes it with the rigid-body local transform
         * for the driven body target. BODY remains object-space authority and
         * MOTION remains COM/weight/diagnostic data only.
         */
        HeldDriveUpdate driveUpdate{};
        if (!updateHeldDrive(world, handWorldTransform, deltaTime, forceFadeInTime, tauMin, releaseContext, driveUpdate,
                peerHand, peerHandWorld)) {
            return;
        }
        carryCountBasedAmmoAnchor(world);
        bool convergingAcquisitionPhase = false;
        if (!updateHeldVisualPresentation(world, handWorldTransform, deltaTime, driveUpdate, releaseContext,
                convergingAcquisitionPhase)) {
            return;
        }
        updateHeldAcquisition(world, handWorldTransform, deltaTime, driveUpdate, convergingAcquisitionPhase);
        finalizeHeldObjectUpdate(world, forceFadeInTime, driveUpdate);
    }
}
