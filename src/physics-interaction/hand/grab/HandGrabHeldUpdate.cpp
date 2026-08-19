#include "physics-interaction/hand/grab/HandGrabInternal.h"

namespace rock
{
    using namespace hand_grab_detail;

    RE::NiTransform Hand::heldNodeWorldFromBodyWorld(const RE::NiTransform& bodyWorld) const
    {
        return deriveNodeWorldFromBodyWorld(bodyWorld, _grabFrame.bodyLocal);
    }




    Hand::HeldHandMotionSample Hand::recordHeldControllerMotionSample(
        const RE::NiTransform& handWorldTransform,
        float deltaTime)
    {
        HeldHandMotionSample handMotion{};
        const bool usableDeltaTime = std::isfinite(deltaTime) && deltaTime > 0.000001f;
        const RE::NiPoint3 currentHandPositionHavok = gamePointToHavokPoint(handWorldTransform.translate);
        _lastHeldHandPositionHavok = currentHandPositionHavok;
        _hasLastHeldHandPositionHavok = true;

        if (_hasPreviousHeldRawHandWorld && usableDeltaTime) {
            handMotion.localLinearVelocityHavok = scalePoint(currentHandPositionHavok - _previousHeldHandPositionHavok, 1.0f / deltaTime);
            handMotion.hasLocalLinearVelocity = true;

            handMotion.angularVelocityRadiansPerSecond =
                angularVelocityFromRotationDelta(_previousHeldRawHandWorld.rotate, handWorldTransform.rotate, deltaTime);
            handMotion.hasAngularVelocity = lengthSquared(handMotion.angularVelocityRadiansPerSecond) > 0.000001f;

            _heldLocalHandVelocityHistory[_heldHandVelocityHistoryNext] = handMotion.localLinearVelocityHavok;
            _heldHandAngularVelocityHistory[_heldHandVelocityHistoryNext] = handMotion.angularVelocityRadiansPerSecond;
            _heldHandVelocityHistoryNext = (_heldHandVelocityHistoryNext + 1) % _heldLocalHandVelocityHistory.size();
            if (_heldHandVelocityHistoryCount < _heldLocalHandVelocityHistory.size()) {
                ++_heldHandVelocityHistoryCount;
            }
        }

        _previousHeldRawHandWorld = handWorldTransform;
        _previousHeldHandPositionHavok = currentHandPositionHavok;
        _hasPreviousHeldRawHandWorld = true;
        return handMotion;
    }

    void Hand::recordHeldObjectVelocitySample(RE::hknpWorld* world)
    {
        const auto objectMotion = sampleHeldObjectMotion(
            world,
            _savedObjectState.bodyId,
            _heldBodyIds,
            _heldDriveDecision.includeConnectedLinearVelocity);
        if (objectMotion.hasPrimaryVelocity) {
            _heldLocalLinearVelocityHistory[_heldLocalLinearVelocityHistoryNext] = objectMotion.primaryLocalLinearVelocity;
            _heldLocalLinearVelocityHistoryNext = (_heldLocalLinearVelocityHistoryNext + 1) % _heldLocalLinearVelocityHistory.size();
            if (_heldLocalLinearVelocityHistoryCount < _heldLocalLinearVelocityHistory.size()) {
                ++_heldLocalLinearVelocityHistoryCount;
            }
            _lastHeldObjectLocalLinearVelocityHavok = objectMotion.primaryLocalLinearVelocity;
            _hasLastHeldObjectLocalLinearVelocityHavok = true;
        }
    }


    void Hand::updateHeldObject(RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        float deltaTime,
        float forceFadeInTime,
        float tauMin,
        const BodyBoneColliderSet* bodyBoneColliders,
        const std::uint64_t sourceSchedulerSequence,
        const GrabReleaseContext& releaseContext)
    {
        if (!isHolding() || !world)
            return;
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabHeldObjectUpdate);
        if (_grabAuthorityProxyReleasePending.load(std::memory_order_acquire) || !_activeConstraint.isValid() || !_grabAuthorityProxy.isValid()) {
            ROCK_LOG_WARN(Hand, "{} hand release: proxy constraint authority marked grab invalid", handName());
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return;
        }

        if (!_savedObjectState.refr || _savedObjectState.refr->IsDeleted() || _savedObjectState.refr->IsDisabled()) {
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return;
        }

        const bool finalSeatMode =
            _grabFrame.seatMode == GrabSeatMode::PinchPocket ||
            _grabFrame.seatMode == GrabSeatMode::SupportGroup;
        const bool finalPivotSource = pivotAuthoritySourceIsFinalPinchOrSupport(_grabFrame.pivotAuthoritySource);
        if (!finalSeatMode || !finalPivotSource) {
            ROCK_LOG_WARN(Hand,
                "{} hand release: held grab has non-final authority seat={} source={} phase={}",
                handName(),
                grabSeatModeName(_grabFrame.seatMode),
                _grabFrame.pivotAuthoritySource ? _grabFrame.pivotAuthoritySource : "none",
                grab_three_phase::phaseName(_grabAcquisitionPhase));
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return;
        }

        nearby_grab_damping::tickNearbyGrabDamping(world, _nearbyGrabDamping, deltaTime);

        suppressHandCollisionForGrab(world, bodyBoneColliders);
        if (_heldObjectIsLooseWeapon) {
            suppressBodyCollisionForHeldLooseWeapon(world, bodyBoneColliders);
        }

        /*
         * Scene-writer probe registration, once per grab trace: collect the
         * held ref's collision objects so the engine scene-writer hook
         * (SceneWriterProbe.h) can gate on exact pointers. Registered here,
         * not at commit, so every grab path (touch, force, pull-catch, loose
         * weapon) funnels through one site. Bounded walk; game thread only.
         */
        if (scene_writer_probe::isInstalled() && _sceneWriterProbeRegisteredTraceId != _grabFrame.traceId) {
            scene_writer_probe::HeldTargetRegistration probeRegistration{};
            auto appendProbeCollisionObject = [&probeRegistration](const RE::NiCollisionObject* collisionObject) {
                if (!collisionObject ||
                    probeRegistration.collisionObjectCount >= scene_writer_probe::kMaxTrackedCollisionObjects) {
                    return;
                }
                for (std::uint32_t i = 0; i < probeRegistration.collisionObjectCount; ++i) {
                    if (probeRegistration.collisionObjects[i] == collisionObject) {
                        return;
                    }
                }
                probeRegistration.collisionObjects[probeRegistration.collisionObjectCount++] = collisionObject;
            };
            if (_grabFrame.heldNode) {
                appendProbeCollisionObject(_grabFrame.heldNode->collisionObject.get());
            }
            struct ProbeWalkEntry
            {
                RE::NiAVObject* node = nullptr;
                int depth = 0;
            };
            std::array<ProbeWalkEntry, 24> pendingProbeNodes{};
            std::size_t pendingProbeNodeCount = 0;
            if (auto* heldRoot3D = _savedObjectState.refr ? _savedObjectState.refr->Get3D() : nullptr) {
                pendingProbeNodes[pendingProbeNodeCount++] = { heldRoot3D, 0 };
            }
            while (pendingProbeNodeCount > 0 &&
                   probeRegistration.collisionObjectCount < scene_writer_probe::kMaxTrackedCollisionObjects) {
                const ProbeWalkEntry entry = pendingProbeNodes[--pendingProbeNodeCount];
                if (!entry.node || entry.depth > 8) {
                    continue;
                }
                appendProbeCollisionObject(entry.node->collisionObject.get());
                if (auto* entryNode = entry.node->IsNode()) {
                    auto& entryChildren = entryNode->GetRuntimeData().children;
                    for (auto i = decltype(entryChildren.size()){ 0 };
                         i < entryChildren.size() && pendingProbeNodeCount < pendingProbeNodes.size();
                         ++i) {
                        pendingProbeNodes[pendingProbeNodeCount++] = { entryChildren[i].get(), entry.depth + 1 };
                    }
                }
            }
            probeRegistration.world = world;
            if (const auto* playerNodes = f4vr::getPlayerNodes(); playerNodes && playerNodes->roomnode) {
                probeRegistration.roomNode = playerNodes->roomnode;
            }
            probeRegistration.bodyId = _savedObjectState.bodyId.value;
            probeRegistration.havokToGame = physics_scale::havokToGame();
            probeRegistration.traceId = _grabFrame.traceId;
            scene_writer_probe::registerHeldTarget(_isLeft, probeRegistration);
            _sceneWriterProbeRegisteredTraceId = _grabFrame.traceId;
        }

        _grabStartTime += held_object_physics_math::finitePositiveOrZero(deltaTime);

        const HeldHandMotionSample handMotion = recordHeldControllerMotionSample(handWorldTransform, deltaTime);
        (void)handMotion;

        /*
         * ROCK freezes the visible object/node relation in generated/proxy
         * authority space, then composes it with the rigid-body local transform
         * for the driven body target. BODY remains object-space authority and
         * MOTION remains COM/weight/diagnostic data only.
         */
        RE::NiTransform proxyAuthorityWorld = handWorldTransform;
        const char* proxyAuthoritySource = "notProxy";
        const bool hasProxyAuthorityFrame = resolveGrabAuthorityProxyFrame(
            world,
            handWorldTransform,
            nullptr,
            proxyAuthorityWorld,
            proxyAuthoritySource,
            GrabAuthorityProxyFramePolicy::PreferQueuedPalmTarget);
        if (!hasProxyAuthorityFrame) {
            ROCK_LOG_WARN(Hand,
                "{} hand release: palm anchor proxy frame unavailable while held source={}",
                handName(),
                proxyAuthoritySource);
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return;
        }
        RE::NiTransform desiredObjectWorld =
            grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyAuthorityWorld, _grabFrame.proxyAuthorityHandSpace);
        if (_hasGrabFingerSweepDebug) {
            _grabFingerSweepDebugObjectWorld = desiredObjectWorld;
        }
        RE::NiTransform desiredBodyWorld =
            grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyAuthorityWorld, _grabFrame.proxyAuthorityBodyHandSpace);
        const RE::NiPoint3 activePivotBBodyLocalGame = activeProxyConstraintPivotBLocalGame();
        const RE::NiPoint3 desiredTargetPointWorld = transform_math::localPointToWorld(desiredBodyWorld, activePivotBBodyLocalGame);
        float pivotTrackingErrorGameUnits = 0.0f;
        float grabRotationErrorDegrees = 0.0f;
        bool hasPivotTrackingError = false;
        RE::NiPoint3 liveGripWorldForAuthority{};
        {
            RE::NiTransform grabBodyWorld{};
            if (tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, grabBodyWorld)) {
                const RE::NiPoint3 liveGripWorld = transform_math::localPointToWorld(grabBodyWorld, activePivotBBodyLocalGame);
                liveGripWorldForAuthority = liveGripWorld;
                pivotTrackingErrorGameUnits = pointDistanceGameUnits(liveGripWorld, desiredTargetPointWorld);
                hasPivotTrackingError = true;
                if (_grabFrame.heldNode) {
                    grabRotationErrorDegrees = rotationDeltaDegrees(_grabFrame.heldNode->world.rotate, desiredObjectWorld.rotate);
                } else {
                    grabRotationErrorDegrees = rotationDeltaDegrees(grabBodyWorld.rotate, desiredBodyWorld.rotate);
                }
            }
        }
        if (!hasPivotTrackingError) {
            ROCK_LOG_WARN(Hand,
                "{} hand release: held object drive body readback failed before queuing grab authority bodyId={} phase={}",
                handName(),
                _savedObjectState.bodyId.value,
                grab_three_phase::phaseName(_grabAcquisitionPhase));
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return;
        }
        if (held_object_physics_math::instantDeviationExceeded(pivotTrackingErrorGameUnits, g_rockConfig.rockGrabMaxDeviation)) {
            ROCK_LOG_WARN(Hand,
                "{} hand release: held object instant pivot deviation exceeded ({:.1f}gu > {:.1f}gu)",
                handName(),
                pivotTrackingErrorGameUnits,
                held_object_physics_math::instantDeviationReleaseThreshold(g_rockConfig.rockGrabMaxDeviation));
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Delayed, releaseContext);
            return;
        }

        const bool heldBodyColliding = isHeldBodyColliding();
        const auto heldContactSnapshot = readHeldBodyContactSnapshot();
        bool heldMotorContactSoftening = heldBodyColliding;
        const char* heldMotorContactReason = heldBodyColliding ? "legacy-recent-contact" : "no-recent-contact";
        if (heldContactSnapshot.recent) {
            const RE::NiPoint3 correctionGame = desiredTargetPointWorld - liveGripWorldForAuthority;
            const RE::NiPoint3 correctionHavok = gamePointToHavokPoint(correctionGame);
            RE::NiTransform heldContactBodyWorld{};
            RE::NiTransform otherContactBodyWorld{};
            const bool hasHeldContactBody =
                heldContactSnapshot.heldBodyId != INVALID_BODY_ID &&
                tryResolveLiveBodyWorldTransform(world, RE::hknpBodyId{ heldContactSnapshot.heldBodyId }, heldContactBodyWorld);
            const bool hasOtherContactBody =
                heldContactSnapshot.otherBodyId != INVALID_BODY_ID &&
                tryResolveLiveBodyWorldTransform(world, RE::hknpBodyId{ heldContactSnapshot.otherBodyId }, otherContactBodyWorld);
            const RE::NiPoint3 heldToOtherHavok =
                (hasHeldContactBody && hasOtherContactBody) ?
                    gamePointToHavokPoint(otherContactBodyWorld.translate - heldContactBodyWorld.translate) :
                    RE::NiPoint3{};
            const auto contactSoftening =
                held_object_contact_policy::evaluateHeldContactMotorSoftening(
                    held_object_contact_policy::HeldContactMotorSofteningInput<RE::NiPoint3>{
                        .recentContact = true,
                        .hasCorrectionVector = hasPivotTrackingError,
                        .hasHeldToOtherVector = hasHeldContactBody && hasOtherContactBody,
                        .hasContactNormal = heldContactSnapshot.hasNormal,
                        .otherMotion = classifyHeldContactOtherMotion(world, heldContactSnapshot.otherBodyId),
                        .correctionTowardTarget = correctionHavok,
                        .heldToOther = heldToOtherHavok,
                        .contactNormal = heldContactSnapshot.contactNormalHavok,
                    });
            heldMotorContactSoftening = contactSoftening.soften;
            heldMotorContactReason = contactSoftening.reason;
        }
        const float authorityForceScale =
            held_object_drive_policy::sanitizeMotorAuthorityScale(sharedGrabAuthorityForceScale(releaseContext.peerHandStillHolding));
        if (held_object_physics_math::shouldQueueGrabAuthorityTargetForDelta(deltaTime)) {
            queueProxyGrabAuthorityTarget(
                proxyAuthorityWorld,
                handWorldTransform,
                proxyAuthoritySource,
                deltaTime,
                forceFadeInTime,
                tauMin,
                pivotTrackingErrorGameUnits,
                grabRotationErrorDegrees,
                authorityForceScale,
                heldMotorContactSoftening);
        } else {
            ROCK_LOG_SAMPLE_WARN(Hand,
                500,
                "{} hand skipped grab authority target after stutter delta dt={:.6f}s threshold={:.3f}s; holding last proxy target",
                handName(),
                std::isfinite(deltaTime) ? deltaTime : -1.0f,
                held_object_physics_math::kMaxGrabAuthorityTargetDeltaSeconds);
        }

        const float averageGrabDeviationGameUnits = recordDeviationAverage(
            _grabDeviationHistory,
            _grabDeviationHistoryCount,
            _grabDeviationHistoryNext,
            pivotTrackingErrorGameUnits);
        _grabDeviationExceededSeconds = held_object_physics_math::advanceDeviationSeconds(
            _grabDeviationExceededSeconds, averageGrabDeviationGameUnits, g_rockConfig.rockGrabMaxDeviation, deltaTime);
        if (held_object_physics_math::deviationExceeded(_grabDeviationExceededSeconds, g_rockConfig.rockGrabMaxDeviationTime)) {
            ROCK_LOG_WARN(Hand,
                "{} hand release: held object exceeded max deviation average ({:.1f}gu > {:.1f}gu for {:.2f}s)",
                handName(),
                averageGrabDeviationGameUnits,
                g_rockConfig.rockGrabMaxDeviation,
                _grabDeviationExceededSeconds);
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Delayed, releaseContext);
            return;
        }

        tickHeldBodyContact();
        const bool convergingAcquisitionPhase =
            _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::NearConverging ||
            _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::GravityPulling ||
            _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::SeatedPivotReacquire;
        const float acquisitionVisualEnvelope =
            grab_three_phase::computeAcquisitionVisualEnvelopeGameUnits(
                g_rockConfig.rockGrabTouchAcquireDistanceGameUnits,
                g_rockConfig.rockGrabNearConvergeDistanceGameUnits,
                g_rockConfig.rockGrabAcquisitionVisualStartDistanceGameUnits);
        const float acquisitionVisualAttachEnvelope =
            (std::min)(
                acquisitionVisualEnvelope,
                (std::max)(g_rockConfig.rockGrabTouchAcquireDistanceGameUnits, g_rockConfig.rockGrabPocketRadiusGameUnits));
        const bool acquisitionVisualEligible =
            convergingAcquisitionPhase &&
            hasPivotTrackingError &&
            _grabObjectGripAtGrab.valid &&
            pivotTrackingErrorGameUnits <= acquisitionVisualAttachEnvelope;
        const auto heldAuthority = evaluateRuntimeHeldAuthority(
            _grabFrame,
            heldMotorContactSoftening);
        const auto& heldAngularAuthority = heldAuthority.angular;
        const auto visualPublishDecision = grab_motion_controller::evaluateVisualHandPublishGate(
            grab_motion_controller::VisualHandPublishInput{
                .hasTelemetryCapture = _grabFrame.hasTelemetryCapture,
                .touchHeldPhase = _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld,
                .acquisitionVisualEligible = acquisitionVisualEligible,
                .hasPivotTrackingError = hasPivotTrackingError,
                .motorContactSoftening = heldMotorContactSoftening,
                .pivotAuthorityPositionOnly = _grabFrame.pivotAuthorityPositionOnly,
                .pivotAuthorityNormalTrusted = _grabFrame.pivotAuthorityNormalTrusted,
                .hasSeatedPivotReacquire = _grabFrame.hasSeatedPivotReacquire || _grabFrame.hasSettledVisualHandRelation,
                .requiresSettledVisualRelation = _grabFrame.requiresSettledVisualHandRelation,
                .multiFingerContactGroupCount = _grabFrame.multiFingerContactGroupCount,
                .contactPatchSampleCount = _grabFrame.contactPatchSampleCount,
                .contactSupportShape = heldAngularAuthority.contactSupportShape,
            });
        if (_grabFrame.hasTelemetryCapture &&
            visualPublishDecision.apply) {
            /*
             * Physics-clock candidate. Always derived from the live BODY, not
             * heldNode->world: once ROCK owns the node below, the node carries
             * the anchored pose and is no longer physics truth.
             */
            RE::NiTransform bodyDerivedNodeWorld{};
            bool hasBodyDerivedNodeWorld = false;
            {
                RE::NiTransform grabBodyWorld{};
                // While the render-pose masquerade holds the body slot, the
                // slot carries the anchor; physics truth is the saved solver
                // pose the masquerade preserved.
                if (held_body_render_pose::tryGetSolverPoseOverride(
                        _savedObjectState.bodyId.value,
                        physics_scale::havokToGame(),
                        grabBodyWorld)) {
                    bodyDerivedNodeWorld = heldNodeWorldFromBodyWorld(grabBodyWorld);
                    hasBodyDerivedNodeWorld = true;
                } else if (tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, grabBodyWorld)) {
                    bodyDerivedNodeWorld = heldNodeWorldFromBodyWorld(grabBodyWorld);
                    hasBodyDerivedNodeWorld = true;
                } else if (_grabFrame.heldNode) {
                    bodyDerivedNodeWorld = _grabFrame.heldNode->world;
                    hasBodyDerivedNodeWorld = true;
                }
            }

            // ANCHOR_CLOCK probe: the held node's pose at producer entry,
            // before ROCK writes it this frame. Compared against last frame's
            // anchor write (did the engine stomp it?) and the live body pose
            // (what did it get stomped TO?).
            const bool anchorProbeHeldNodeOk = _grabFrame.heldNode != nullptr;
            const RE::NiTransform anchorProbeHeldEntryWorld =
                anchorProbeHeldNodeOk ? _grabFrame.heldNode->world : RE::NiTransform{};

            /*
             * Render-clock candidate: the object rendered rigidly from the raw
             * interaction hand through the frozen grab relation. The raw hand
             * shares the skeleton's clock, so this candidate has zero relative
             * motion against the skeleton by construction. The physics body
             * runs one clock/basis behind the render skeleton during stick
             * locomotion (2026-08-16 captures), and publishing body-derived
             * hand transforms into FRIK injected that disagreement into the
             * holding arm's bones/IK/mesh as a per-frame buzz -- the defect
             * this anchor removes. Contact hands authority back to the body
             * candidate through a rate-limited blend so walls still visibly
             * stop the object.
             */
            RE::NiTransform handDerivedNodeWorld{};
            bool hasHandDerivedNodeWorld = false;
            if (g_rockConfig.rockGrabHeldRenderClockAnchor &&
                hasBodyDerivedNodeWorld &&
                isUsableGrabVisualTransform(handWorldTransform) &&
                isUsableGrabVisualTransform(_grabFrame.rawHandSpace)) {
                handDerivedNodeWorld = hand_visual_lerp_math::buildHandRelativeHeldObjectWorld(
                    handWorldTransform,
                    _grabFrame.rawHandSpace);
                handDerivedNodeWorld.scale = bodyDerivedNodeWorld.scale;
                hasHandDerivedNodeWorld = isUsableGrabVisualTransform(handDerivedNodeWorld);
            }

            RE::NiTransform heldVisualNodeWorld = bodyDerivedNodeWorld;
            bool hasHeldVisualNodeWorld = hasBodyDerivedNodeWorld;
            bool renderClockAnchorEngaged = false;
            if (hasHandDerivedNodeWorld) {
                const float bodyToHandAnchorDeviationGameUnits =
                    pointDistanceGameUnits(bodyDerivedNodeWorld.translate, handDerivedNodeWorld.translate);
                // Acquisition stays body-anchored: the hand travels to the
                // object, the object must not travel to the hand. The anchor
                // takes over only after the grab settles into TouchHeld.
                const bool acquisitionSettled =
                    _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld;
                const float anchorBlendTarget = acquisitionSettled ?
                    hand_visual_lerp_math::heldAnchorBodyBlendTarget(
                        heldBodyColliding,
                        bodyToHandAnchorDeviationGameUnits) :
                    1.0f;
                _grabHeldAnchorBodyBlend = hand_visual_lerp_math::advanceHeldAnchorBodyBlend(
                    _grabHeldAnchorBodyBlend,
                    anchorBlendTarget,
                    deltaTime);
                heldVisualNodeWorld = hand_visual_lerp_math::interpolateTransform(
                    handDerivedNodeWorld,
                    bodyDerivedNodeWorld,
                    _grabHeldAnchorBodyBlend);
                renderClockAnchorEngaged = isUsableGrabVisualTransform(heldVisualNodeWorld);
                if (!renderClockAnchorEngaged) {
                    heldVisualNodeWorld = bodyDerivedNodeWorld;
                }
            } else {
                // Fail closed onto the physics-body anchor (pre-anchor behavior).
                _grabHeldAnchorBodyBlend = 1.0f;
            }

            rock::debug::RockGrabClockStageProducerV1 grabClockProducerSample{};
            {
                const auto anchorProbeRoom = sampleAnchorClockRoom();
                const float anchorProbeEntryVsLastWrite =
                    (_hasGrabProbeLastAnchorWrite && anchorProbeHeldNodeOk) ?
                    pointDistanceGameUnits(anchorProbeHeldEntryWorld.translate, _grabProbeLastAnchorWrite.translate) :
                    -1.0f;
                const float anchorProbeEntryVsBody =
                    (hasBodyDerivedNodeWorld && anchorProbeHeldNodeOk) ?
                    pointDistanceGameUnits(anchorProbeHeldEntryWorld.translate, bodyDerivedNodeWorld.translate) :
                    -1.0f;

                grabClockProducerSample.schedulerSequence = sourceSchedulerSequence;
                grabClockProducerSample.deltaSeconds = deltaTime;
                assignGrabClockFeedVec(grabClockProducerSample.roomPos, anchorProbeRoom.position);
                grabClockProducerSample.roomYawDegrees = anchorProbeRoom.yawDegrees;
                grabClockProducerSample.roomValid = anchorProbeRoom.valid ? 1u : 0u;
                assignGrabClockFeedVec(grabClockProducerSample.rawHandPos, handWorldTransform.translate);
                assignGrabClockFeedVec(grabClockProducerSample.heldEntryPos, anchorProbeHeldEntryWorld.translate);
                grabClockProducerSample.heldEntryValid = anchorProbeHeldNodeOk ? 1u : 0u;
                grabClockProducerSample.entryVsLastWriteGu = anchorProbeEntryVsLastWrite;
                grabClockProducerSample.entryVsBodyGu = anchorProbeEntryVsBody;
                assignGrabClockFeedVec(grabClockProducerSample.bodyPos, bodyDerivedNodeWorld.translate);
                grabClockProducerSample.bodyValid = hasBodyDerivedNodeWorld ? 1u : 0u;
                assignGrabClockFeedVec(grabClockProducerSample.anchorPos, heldVisualNodeWorld.translate);
                grabClockProducerSample.bodyBlend = _grabHeldAnchorBodyBlend;
                grabClockProducerSample.anchorEngaged = renderClockAnchorEngaged ? 1u : 0u;
                ROCK_LOG_INFO(Hand,
                    "{} ANCHOR_CLOCK stage=producer seq={} room=({:.2f},{:.2f},{:.2f}) roomYaw={:.3f} rawHand=({:.2f},{:.2f},{:.2f}) heldEntry=({:.2f},{:.2f},{:.2f}) entryVsLastWrite={:.3f}gu entryVsBody={:.3f}gu body=({:.2f},{:.2f},{:.2f}) anchor=({:.2f},{:.2f},{:.2f}) blend={:.2f} engaged={}",
                    handName(),
                    sourceSchedulerSequence,
                    anchorProbeRoom.position.x,
                    anchorProbeRoom.position.y,
                    anchorProbeRoom.position.z,
                    anchorProbeRoom.yawDegrees,
                    handWorldTransform.translate.x,
                    handWorldTransform.translate.y,
                    handWorldTransform.translate.z,
                    anchorProbeHeldEntryWorld.translate.x,
                    anchorProbeHeldEntryWorld.translate.y,
                    anchorProbeHeldEntryWorld.translate.z,
                    anchorProbeEntryVsLastWrite,
                    anchorProbeEntryVsBody,
                    bodyDerivedNodeWorld.translate.x,
                    bodyDerivedNodeWorld.translate.y,
                    bodyDerivedNodeWorld.translate.z,
                    heldVisualNodeWorld.translate.x,
                    heldVisualNodeWorld.translate.y,
                    heldVisualNodeWorld.translate.z,
                    _grabHeldAnchorBodyBlend,
                    renderClockAnchorEngaged ? "yes" : "no");
                _grabProbeProducerHandPos = handWorldTransform.translate;
                _grabProbeProducerRoomPos = anchorProbeRoom.position;
                _hasGrabProbeProducerSample = anchorProbeRoom.valid;
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
                RE::NiTransform targetVisualHandWorld =
                    hand_visual_lerp_math::buildHeldObjectRelativeHandWorld(heldVisualNodeWorld, _grabFrame.rawHandSpace);
                targetVisualHandWorld.scale = handWorldTransform.scale;

                RE::NiTransform nextVisualHandWorld = targetVisualHandWorld;
                const bool smoothVisualHand = hand_visual_lerp_math::shouldSmoothHeldObjectRelativeHand(
                    g_rockConfig.rockGrabHandLerpEnabled,
                    _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld,
                    visualPublishDecision.acquisition);
                if (!_hasGrabVisualHandTransform) {
                    const RE::NiTransform acquisitionStart =
                        _grabVisualReturn.active && isUsableGrabVisualTransform(_grabVisualReturn.lastApplied) ?
                        _grabVisualReturn.lastApplied :
                        handWorldTransform;
                    _grabHeldAnchorBodyBlend = 1.0f;
                    _hasGrabProbeLastAnchorWrite = false;
                    _hasGrabProbeProducerSample = false;
                    _grabVisualHandTransform = acquisitionStart;
                    _grabVisualHandLerpStartTransform = acquisitionStart;
                    _grabVisualHandLerpElapsedSeconds = 0.0f;
                    _grabVisualHandLerpDurationSeconds = smoothVisualHand ?
                        hand_visual_lerp_math::computeDistanceMappedDurationGameUnits(
                            hand_visual_lerp_math::distanceGameUnits(acquisitionStart.translate, targetVisualHandWorld.translate),
                            g_rockConfig.rockGrabHandLerpTimeMin,
                            g_rockConfig.rockGrabHandLerpTimeMax,
                            g_rockConfig.rockGrabHandLerpMinDistance,
                            g_rockConfig.rockGrabHandLerpMaxDistance) :
                        0.0f;
                    _hasGrabVisualHandTransform = true;
                }

                float visualHandLerpAlpha = 1.0f;
                if (smoothVisualHand) {
                    _grabVisualHandLerpElapsedSeconds = hand_visual_lerp_math::advanceTimedBlendElapsed(
                        _grabVisualHandLerpElapsedSeconds,
                        deltaTime,
                        _grabVisualHandLerpDurationSeconds);
                    const auto advancedVisual = hand_visual_lerp_math::blendTransformOverDuration(
                        _grabVisualHandLerpStartTransform,
                        targetVisualHandWorld,
                        _grabVisualHandLerpElapsedSeconds,
                        _grabVisualHandLerpDurationSeconds);
                    nextVisualHandWorld = advancedVisual.transform;
                    visualHandLerpAlpha = hand_visual_lerp_math::timedBlendAlpha(
                        _grabVisualHandLerpElapsedSeconds,
                        _grabVisualHandLerpDurationSeconds);
                } else {
                    _grabVisualHandLerpStartTransform = targetVisualHandWorld;
                    _grabVisualHandLerpElapsedSeconds = 0.0f;
                    _grabVisualHandLerpDurationSeconds = 0.0f;
                }

                const float visualHandDeviationGameUnits =
                    pointDistanceGameUnits(nextVisualHandWorld.translate, handWorldTransform.translate);
                if (held_object_physics_math::instantDeviationExceeded(visualHandDeviationGameUnits, g_rockConfig.rockGrabMaxDeviation)) {
                    ROCK_LOG_WARN(Hand,
                        "{} hand release: visual held-object hand target instant deviation exceeded ({:.1f}gu > {:.1f}gu)",
                        handName(),
                        visualHandDeviationGameUnits,
                        held_object_physics_math::instantDeviationReleaseThreshold(g_rockConfig.rockGrabMaxDeviation));
                    clearGrabExternalHandWorldTransform(_isLeft);
                    releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Delayed, releaseContext);
                    return;
                }
                const float averageVisualHandDeviationGameUnits = recordDeviationAverage(
                    _grabVisualDeviationHistory,
                    _grabVisualDeviationHistoryCount,
                    _grabVisualDeviationHistoryNext,
                    visualHandDeviationGameUnits);
                _grabVisualDeviationExceededSeconds = held_object_physics_math::advanceDeviationSeconds(
                    _grabVisualDeviationExceededSeconds,
                    averageVisualHandDeviationGameUnits,
                    g_rockConfig.rockGrabMaxDeviation,
                    deltaTime);
                if (held_object_physics_math::deviationExceeded(_grabVisualDeviationExceededSeconds, g_rockConfig.rockGrabMaxDeviationTime)) {
                    ROCK_LOG_WARN(Hand,
                        "{} hand release: visual held-object hand target exceeded max deviation average ({:.1f}gu > {:.1f}gu for {:.2f}s)",
                        handName(),
                        averageVisualHandDeviationGameUnits,
                        g_rockConfig.rockGrabMaxDeviation,
                        _grabVisualDeviationExceededSeconds);
                    clearGrabExternalHandWorldTransform(_isLeft);
                    releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Delayed, releaseContext);
                    return;
                }

                _grabVisualHandTransform = nextVisualHandWorld;
                if (applyGrabExternalHandWorldTransform(_isLeft, _grabVisualHandTransform)) {
                    _lastPublishedGrabVisualHandTransform = _grabVisualHandTransform;
                    _hasLastPublishedGrabVisualHandTransform = true;
                    /*
                     * Render-clock node ownership: with the acquisition lerp
                     * complete, the published hand equals
                     * buildHeldObjectRelativeHandWorld(anchor, relation), so
                     * writing the anchor keeps the rendered hand+object pair
                     * exactly rigid. During acquisition (alpha < 1) the node
                     * stays on the engine's body sync so the hand travels to
                     * the object, never the reverse.
                     */
                    bool renderClockNodeWritten = false;
                    if (renderClockAnchorEngaged &&
                        visualHandLerpAlpha >= 1.0f &&
                        _grabFrame.heldNode) {
                        // Render-consumption probe: +Z here, -Z at pre-FRIK.
                        // The rendered object's visible offset identifies
                        // which node write the renderer consumes (see
                        // rockGrabRenderClockProbeOffsetGameUnits).
                        RE::NiTransform producerNodeWritePose = heldVisualNodeWorld;
                        const float renderProbeOffset =
                            g_rockConfig.rockGrabRenderClockProbeOffsetGameUnits;
                        if (renderProbeOffset != 0.0f) {
                            producerNodeWritePose.translate.z += renderProbeOffset;
                            ROCK_LOG_SAMPLE_WARN(Hand,
                                1000,
                                "{} RENDER_PROBE active: producer node write +{:.1f}gu Z, preFrik write -{:.1f}gu Z (diagnostic, object only)",
                                handName(),
                                renderProbeOffset,
                                renderProbeOffset);
                        }
                        applyHeldVisualNodeWorldTransform(_grabFrame.heldNode, producerNodeWritePose);
                        _grabProbeLastAnchorWrite = producerNodeWritePose;
                        _hasGrabProbeLastAnchorWrite = true;
                        grabClockProducerSample.nodeWriteApplied = 1;
                        renderClockNodeWritten = true;
                    } else {
                        _hasGrabProbeLastAnchorWrite = false;
                    }
                    // Held-body render-pose masquerade target: while ROCK
                    // owns the rendered node, the final-substep post-solve
                    // callback writes this anchor into the BODY (the pose
                    // the renderer actually consumes) and restores the
                    // solver pose before the next collide.
                    if (renderClockNodeWritten &&
                        g_rockConfig.rockGrabHeldRenderBodyPose &&
                        _savedObjectState.bodyId.value != INVALID_BODY_ID) {
                        held_body_render_pose::publishTarget(
                            _isLeft,
                            _savedObjectState.bodyId.value,
                            heldVisualNodeWorld,
                            physics_scale::gameToHavok());
                    } else {
                        held_body_render_pose::invalidateTarget(_isLeft);
                    }
                    /*
                     * Scene-writer render-pose sync anchor (producer clock).
                     * The writer input is the BODY pose, so the published
                     * anchor is the held visual pose carried through the
                     * rigid node->body relation of this frame's authority.
                     * The pre-FRIK refresh republishes the same anchor from
                     * the fresh-clock reconstructed node.
                     */
                    _preFrikGrabVisualAuthority.hasHeldNodeToBodyAnchorLocal = false;
                    if (renderClockNodeWritten && g_rockConfig.rockGrabHeldScenePoseSync) {
                        const RE::NiTransform heldNodeToBodyAnchorLocal =
                            prefrik_hand_authority_policy::captureDriverToTargetLocal(
                                desiredObjectWorld,
                                desiredBodyWorld);
                        const RE::NiTransform bodyAnchorWorld =
                            prefrik_hand_authority_policy::reconstructTargetWorld(
                                heldVisualNodeWorld,
                                heldNodeToBodyAnchorLocal);
                        if (prefrik_hand_authority_policy::isUsableTransform(bodyAnchorWorld)) {
                            const auto anchorSourceRoom = sampleAnchorClockRoom();
                            scene_writer_probe::AnchorRootSample anchorRootSample{};
                            anchorRootSample.roomPositionGame[0] = anchorSourceRoom.position.x;
                            anchorRootSample.roomPositionGame[1] = anchorSourceRoom.position.y;
                            anchorRootSample.roomPositionGame[2] = anchorSourceRoom.position.z;
                            anchorRootSample.roomYawRadians = anchorSourceRoom.yawDegrees * 0.01745329252f;
                            anchorRootSample.roomValid = anchorSourceRoom.valid;
                            scene_writer_probe::publishHeldAnchor(
                                _isLeft,
                                bodyAnchorWorld,
                                scene_writer_probe::AnchorStage::Producer,
                                anchorRootSample);
                            _preFrikGrabVisualAuthority.heldNodeToBodyAnchorLocal = heldNodeToBodyAnchorLocal;
                            _preFrikGrabVisualAuthority.hasHeldNodeToBodyAnchorLocal = true;
                        } else {
                            scene_writer_probe::invalidateHeldAnchor(_isLeft);
                        }
                    } else {
                        scene_writer_probe::invalidateHeldAnchor(_isLeft);
                    }
                    rock::debug::publishGrabClockProducerStage(_isLeft, grabClockProducerSample);
                    if (_grabFrame.heldNode &&
                        sourceSchedulerSequence != 0 &&
                        prefrik_hand_authority_policy::isUsableTransform(
                            heldVisualNodeWorld)) {
                        _preFrikGrabVisualAuthority.heldNode.reset(
                            _grabFrame.heldNode);
                        _preFrikGrabVisualAuthority.heldNodeToHandLocal =
                            prefrik_hand_authority_policy::captureDriverToTargetLocal(
                                heldVisualNodeWorld,
                                _grabVisualHandTransform);
                        _preFrikGrabVisualAuthority.sourceRawHandWorld =
                            handWorldTransform;
                        _preFrikGrabVisualAuthority.rawHandToHeldLocal =
                            prefrik_hand_authority_policy::captureDriverToTargetLocal(
                                handWorldTransform,
                                heldVisualNodeWorld);
                        _preFrikGrabVisualAuthority.rawHandToHandLocal =
                            prefrik_hand_authority_policy::captureDriverToTargetLocal(
                                handWorldTransform,
                                _grabVisualHandTransform);
                        _preFrikGrabVisualAuthority.renderClockNodeOwned =
                            renderClockNodeWritten &&
                            prefrik_hand_authority_policy::isUsableTransform(
                                handWorldTransform) &&
                            prefrik_hand_authority_policy::isUsableTransform(
                                _preFrikGrabVisualAuthority.rawHandToHeldLocal) &&
                            prefrik_hand_authority_policy::isUsableTransform(
                                _preFrikGrabVisualAuthority.rawHandToHandLocal);
                        _preFrikGrabVisualAuthority.sourceSchedulerSequence =
                            sourceSchedulerSequence;
                        _preFrikGrabVisualAuthority.heldBodyId =
                            _savedObjectState.bodyId.value;
                        _preFrikGrabVisualAuthority.constraintId =
                            _activeConstraint.constraintId;
                        _preFrikGrabVisualAuthority.valid =
                            prefrik_hand_authority_policy::isUsableTransform(
                                _preFrikGrabVisualAuthority.heldNodeToHandLocal);
                    } else {
                        _preFrikGrabVisualAuthority.clear();
                    }
                    clearGrabVisualReturn("active-grab-authority-acquired", false);
                }

                ROCK_LOG_SAMPLE_DEBUG(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} GRAB VISUAL HAND: relation=heldRelative phase={} visualOnly=yes authority={} shape={} follow={} anchor={}/bodyBlend={:.2f} scale={:.2f} lerpAlpha={:.2f} lerpDuration={:.3f}s target=({:.1f},{:.1f},{:.1f}) applied=({:.1f},{:.1f},{:.1f}) live=({:.1f},{:.1f},{:.1f}) deviation={:.2f}gu avgDeviation={:.2f}gu normalAuthority=false",
                    handName(),
                    grab_three_phase::phaseName(_grabAcquisitionPhase),
                    visualPublishDecision.reason,
                    grab_motion_controller::contactSupportShapeName(heldAngularAuthority.contactSupportShape),
                    smoothVisualHand ? "smoothedAcquisition" : "immediateHeldObject",
                    renderClockAnchorEngaged ? "renderClock" : "body",
                    _grabHeldAnchorBodyBlend,
                    heldAngularAuthority.authorityScale,
                    visualHandLerpAlpha,
                    _grabVisualHandLerpDurationSeconds,
                    targetVisualHandWorld.translate.x,
                    targetVisualHandWorld.translate.y,
                    targetVisualHandWorld.translate.z,
                    _grabVisualHandTransform.translate.x,
                    _grabVisualHandTransform.translate.y,
                    _grabVisualHandTransform.translate.z,
                    handWorldTransform.translate.x,
                    handWorldTransform.translate.y,
                    handWorldTransform.translate.z,
                    visualHandDeviationGameUnits,
                    averageVisualHandDeviationGameUnits);
            } else {
                _grabVisualDeviationExceededSeconds = 0.0f;
                _grabVisualDeviationHistory = {};
                _grabVisualDeviationHistoryCount = 0;
                _grabVisualDeviationHistoryNext = 0;
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
            if (_grabFrame.hasTelemetryCapture && g_rockConfig.rockDebugGrabFrameLogging &&
                (_grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld || acquisitionVisualEligible)) {
                ROCK_LOG_SAMPLE_DEBUG(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} GRAB VISUAL HAND HELD: phase={} authority={} shape={} scale={:.2f} positionOnly={} normalTrusted={} seated={} contactSoftening={}",
                    handName(),
                    grab_three_phase::phaseName(_grabAcquisitionPhase),
                    visualPublishDecision.reason,
                    grab_motion_controller::contactSupportShapeName(heldAngularAuthority.contactSupportShape),
                    heldAngularAuthority.authorityScale,
                    _grabFrame.pivotAuthorityPositionOnly ? "yes" : "no",
                    _grabFrame.pivotAuthorityNormalTrusted ? "yes" : "no",
                    _grabFrame.hasSeatedPivotReacquire ? "yes" : "no",
                    heldMotorContactSoftening ? "yes" : "no");
            }
            _grabVisualDeviationExceededSeconds = 0.0f;
            _grabVisualDeviationHistory = {};
            _grabVisualDeviationHistoryCount = 0;
            _grabVisualDeviationHistoryNext = 0;
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

        if (convergingAcquisitionPhase && _grabObjectGripAtGrab.valid) {
            const auto previousAcquisitionPhase = _grabAcquisitionPhase;
            RE::NiTransform grabBodyWorld{};
            const bool hasGrabBody = tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, grabBodyWorld);
            const RE::NiPoint3 liveGripWorld =
                hasGrabBody ? transform_math::localPointToWorld(grabBodyWorld, activePivotBBodyLocalGame) : RE::NiPoint3{};
            const RE::NiPoint3 targetGripWorld = desiredTargetPointWorld;
            const RE::NiPoint3 gripError = targetGripWorld - liveGripWorld;
            const float gripErrorGameUnits = hasGrabBody ? std::sqrt(gripError.x * gripError.x + gripError.y * gripError.y + gripError.z * gripError.z) :
                                                           std::numeric_limits<float>::max();
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
                    .stableInsidePocketFrames = _grabConvergeStableInsidePocketFrames,
                    .requiredStableInsidePocketFrames = g_rockConfig.rockGrabConvergeStableFrames,
                    .maxSeparatingSpeedGameUnitsPerSecond = g_rockConfig.rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond,
                });
            _grabConvergeStableInsidePocketFrames = convergenceDecision.nextStableInsidePocketFrames;
            _grabConvergePreviousGripErrorGameUnits = gripErrorGameUnits;
            const bool reachedTouchRange = convergenceDecision.reachedTouchRange;
            const bool convergenceTimedOutInsidePocket = convergenceDecision.timedOutInsidePocket;
            if (convergenceTimedOutInsidePocket) {
                ROCK_LOG_SAMPLE_DEBUG(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} THREE-PHASE GRAB CONVERGE PROMOTION READY: phase={} gripErr={:.2f}gu touch={:.2f}gu pocket={:.2f}gu elapsed={:.3f}s colliding={} stableFrames={} sepSpeed={:.2f}gu/s",
                    handName(),
                    grab_three_phase::phaseName(previousAcquisitionPhase),
                    gripErrorGameUnits,
                    touchDistance,
                    g_rockConfig.rockGrabPocketRadiusGameUnits,
                    _grabStartTime,
                    heldBodyColliding ? "yes" : "no",
                    _grabConvergeStableInsidePocketFrames,
                    convergenceDecision.separatingSpeedGameUnitsPerSecond);
            }

            if (hasGrabBody && !reachedTouchRange && !convergenceTimedOutInsidePocket && !_grabFrame.syntheticLooseWeaponPrimaryAttach &&
                g_rockConfig.rockGrabMeshFingerPoseEnabled && _hasGrabFingerPose && _grabFingerPosePublished) {
                const float nearDistance = (std::max)(touchDistance, g_rockConfig.rockGrabNearConvergeDistanceGameUnits);
                const float progressDenominator = (std::max)(0.001f, nearDistance - touchDistance);
                const float acquisitionProgress = 1.0f - std::clamp((gripErrorGameUnits - touchDistance) / progressDenominator, 0.0f, 1.0f);
                const auto resolvedTargetPose = grab_finger_pose_runtime::resolveSurfaceAimObjectLocal(_grabFingerPose, desiredObjectWorld);
                const auto acquisitionFingerPose = buildAcquisitionFingerPose(resolvedTargetPose, acquisitionProgress);
                applyRockGrabHandPose(_isLeft, acquisitionFingerPose, _grabFingerJointPose, _hasGrabFingerJointPose, _grabFingerLocalTransforms, _grabFingerLocalTransformMask,
                    _hasGrabFingerLocalTransforms, deltaTime, false);
            }

            const bool promotionRequested = reachedTouchRange || convergenceTimedOutInsidePocket;
            const bool pivotNeedsSeatedReacquire =
                promotionRequested &&
                _grabFrame.seatMode != GrabSeatMode::PinchPocket &&
                !_grabFrame.syntheticLooseWeaponPrimaryAttach &&
                (_grabFrame.requiresSettledVisualHandRelation ||
                    pivotAuthoritySourceShouldReacquireAtSeat(_grabFrame.pivotAuthoritySource, _grabFrame.pivotAuthorityPositionOnly));
            bool timeoutReacquiredSeatedPivot = false;
            bool seatedRetargetRejectedKeepFrozen = false;
            const char* timeoutReacquireReason = pivotNeedsSeatedReacquire ? (hasGrabBody ? "notAttempted" : "missingGrabBody") : "notNeeded";
            if (pivotNeedsSeatedReacquire) {
                _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::SeatedPivotReacquire;
                _grabFrame.lastSeatedPivotReacquirePhase = grab_three_phase::phaseName(previousAcquisitionPhase);
            }
            if (pivotNeedsSeatedReacquire && hasGrabBody) {
                RE::NiPoint3 livePivotAWorld{};
                if (!tryComputeGrabProxyLocalPalmPocketPivotAWorld(world, livePivotAWorld)) {
                    timeoutReacquireReason = "missingProxyLocalPalmPocketPivot";
                    seatedRetargetRejectedKeepFrozen = true;
                } else {
                    const RE::NiTransform currentNodeWorld = heldNodeWorldFromBodyWorld(grabBodyWorld);
                    const RE::NiTransform authorityFrame =
                        makeGeneratedProxyAuthorityRelationFrame(proxyAuthorityWorld);
                    const float seatedEnvelope =
                        (std::max)(touchDistance, g_rockConfig.rockGrabPocketRadiusGameUnits) +
                        (std::max)(1.0f, finitePositiveOr(g_rockConfig.rockGrabContactPatchProbeSpacingGameUnits, 3.0f));
                    const auto seatedPocket = grab_three_phase::buildGrabPocketFrameWithPalmCenter(
                        authorityFrame,
                        _isLeft,
                        livePivotAWorld,
                        g_rockConfig.rockGrabPocketDepthGameUnits,
                        g_rockConfig.rockGrabPocketRadiusGameUnits);
                    const RE::NiPoint3 palmNormalWorld =
                        seatedPocket.valid ? seatedPocket.palmNormalWorld : computePalmNormalFromHandBasis(authorityFrame, _isLeft);
                    const RE::NiPoint3 palmTangentWorld =
                        seatedPocket.valid ? seatedPocket.fingerForwardWorld : transformHandspaceDirection(authorityFrame, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, _isLeft);
                    const RE::NiPoint3 palmBitangentWorld =
                        seatedPocket.valid ? seatedPocket.crossPalmWorld : transformHandspaceDirection(authorityFrame, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }, _isLeft);
                    const auto seatedPivot = findSeatedGrabPivotNearPalmPocket(
                        _grabFrame.localMeshTriangles,
                        currentNodeWorld,
                        grabBodyWorld,
                        livePivotAWorld,
                        palmNormalWorld,
                        seatedEnvelope,
                        g_rockConfig.rockGrabContactPatchMaxNormalAngleDegrees);
                    timeoutReacquireReason = seatedPivot.reason;
                    if (seatedPivot.valid) {
                        const RE::NiPoint3 previousGripPointLocal = _grabFrame.gripPointLocal;
                        const RE::NiPoint3 previousGripLocalDelta = seatedPivot.pointNodeLocal - previousGripPointLocal;
                        const float nodeScale =
                            std::isfinite(currentNodeWorld.scale) && currentNodeWorld.scale > 0.0f ? currentNodeWorld.scale : 1.0f;
                        const float reacquireLocalDeltaGameUnits = vectorMagnitude(previousGripLocalDelta) * nodeScale;
                        const float immediateLocalDelta =
                            (std::max)(2.0f,
                                finitePositiveOr(g_rockConfig.rockGrabContactPatchProbeSpacingGameUnits, 3.0f) +
                                    finitePositiveOr(g_rockConfig.rockGrabContactPatchProbeRadiusGameUnits, 2.0f));
                        const float lerpLocalDelta =
                            (std::max)(immediateLocalDelta * 3.0f,
                                (std::max)(g_rockConfig.rockGrabPocketRadiusGameUnits, g_rockConfig.rockGrabNearConvergeDistanceGameUnits * 0.50f));
                        const bool seatedSupportMayBecomeAuthority =
                            !(heldMotorContactSoftening && !reachedTouchRange) &&
                            std::isfinite(reacquireLocalDeltaGameUnits) &&
                            reacquireLocalDeltaGameUnits <= lerpLocalDelta;
                        SeatedPalmPocketSupportPatch seatedSupportPatch{};
                        if (seatedSupportMayBecomeAuthority) {
                            seatedSupportPatch = buildSeatedPalmPocketSupportPatch(
                                _grabFrame.localMeshTriangles,
                                _savedObjectState.bodyId.value,
                                currentNodeWorld,
                                livePivotAWorld,
                                seatedPivot.pointWorld,
                                palmNormalWorld,
                                palmTangentWorld,
                                palmBitangentWorld,
                                seatedPivot.longLeverGameUnits);
                        } else {
                            seatedSupportPatch.reason = "seatedSupportSkippedUntilPivotCanSettle";
                            seatedSupportPatch.patch.fallbackReason = seatedSupportPatch.reason;
                        }
                        const auto promotionDecision = grab_motion_controller::evaluateSeatedPalmPocketPromotion(
                            grab_motion_controller::SeatedPalmPocketPromotionInput{
                                .enabled = true,
                                .weakMeshStart = _grabFrame.requiresSettledVisualHandRelation ||
                                                 pivotAuthoritySourceShouldReacquireAtSeat(_grabFrame.pivotAuthoritySource, _grabFrame.pivotAuthorityPositionOnly),
                                .hasSeatedCandidate = seatedPivot.valid,
                                .reachedTouchRange = reachedTouchRange,
                                .timedOutInsidePocket = convergenceTimedOutInsidePocket,
                                .motorContactSoftening = heldMotorContactSoftening,
                                .candidateNormalTrusted = seatedPivot.normalTrusted,
                                .supportPatchValid = seatedSupportPatch.valid,
                                .supportPatchNormalTrusted = seatedSupportPatch.normalTrusted,
                                .currentContactPatchSampleCount = _grabFrame.contactPatchSampleCount,
                                .supportPatchSampleCount = seatedSupportPatch.sampleCount,
                                .currentMultiFingerContactGroupCount = _grabFrame.multiFingerContactGroupCount,
                                .liveMultiFingerContactGroupCount = _grabFrame.multiFingerContactGroupCount,
                                .candidateLocalDeltaGameUnits = reacquireLocalDeltaGameUnits,
                                .immediateMaxLocalDeltaGameUnits = immediateLocalDelta,
                                .lerpMaxLocalDeltaGameUnits = lerpLocalDelta,
                            });
                        timeoutReacquireReason = promotionDecision.reason;

                        if (promotionDecision.promotePivot) {
                            const float pivotBlend = 1.0f;
                            const RE::NiPoint3 promotedPointNodeLocal = seatedPivot.pointNodeLocal;
                            const RE::NiPoint3 promotedPointWorld = transform_math::localPointToWorld(currentNodeWorld, promotedPointNodeLocal);
                            const bool promotedNormalTrusted =
                                seatedSupportPatch.valid ? seatedSupportPatch.normalTrusted : seatedPivot.normalTrusted;
                            const RE::NiPoint3 promotedNormalWorld =
                                promotedNormalTrusted && seatedSupportPatch.valid ? seatedSupportPatch.patch.normal : seatedPivot.normalWorld;
                            const RE::NiPoint3 promotedNormalNodeLocal =
                                promotedNormalTrusted && seatedSupportPatch.valid ?
                                    transform_math::worldVectorToLocal(currentNodeWorld, promotedNormalWorld) :
                                    seatedPivot.normalNodeLocal;
                            /*
                             * Seat depth stop, reacquire flavor: the promoted point is
                             * the mesh point nearest the palm pocket, but seating it on
                             * the palm center still ignores mesh extending past it
                             * toward the palm. Same pivot-A push as the capture path so
                             * a reacquire can never undo the capture-time correction.
                             */
                            const auto seatDepthStop = computeGrabSeatDepthStop(
                                _grabFrame.localMeshTriangles,
                                currentNodeWorld,
                                promotedPointWorld,
                                palmNormalWorld,
                                g_rockConfig.rockGrabSeatDepthFootprintRadiusGameUnits,
                                g_rockConfig.rockGrabSeatDepthMaxGameUnits);
                            RE::NiPoint3 seatPivotAWorld = livePivotAWorld;
                            if (seatDepthStop.valid && seatDepthStop.depthGameUnits > 0.01f) {
                                seatPivotAWorld = livePivotAWorld +
                                                  palmNormalWorld * (seatDepthStop.depthGameUnits +
                                                                        (std::max)(0.0f, g_rockConfig.rockGrabSeatDepthSkinGameUnits));
                            }
                            const float promotedPocketDistanceGameUnits = pointDistanceGameUnits(promotedPointWorld, seatPivotAWorld);
                            const RE::NiTransform desiredBodyWorldAtSeat =
                                grab_frame_math::shiftObjectToAlignGripWithPocket(grabBodyWorld, seatPivotAWorld, promotedPointWorld);
                            const RE::NiTransform desiredObjectWorldAtSeat =
                                heldNodeWorldFromBodyWorld(desiredBodyWorldAtSeat);
                            const RE::NiTransform proxyAuthorityFrameWorld =
                                makeGeneratedProxyAuthorityRelationFrame(proxyAuthorityWorld);
                            const auto frozenSeatAuthorityFrame = grab_authority_frame_math::freezeGrabAuthorityFrame<RE::NiTransform>(
                                grab_authority_frame_math::GrabAuthorityFrameFreezeInput<RE::NiTransform>{
                                    .rawHandWorld = handWorldTransform,
                                    .proxyWorld = proxyAuthorityWorld,
                                    .proxyAuthorityFrameWorld = proxyAuthorityFrameWorld,
                                    .objectWorld = currentNodeWorld,
                                    .bodyWorld = grabBodyWorld,
                                    .constraintBodyWorld = grabBodyWorld,
                                    .rootBodyLocal = _grabFrame.rootBodyLocal,
                                    .ownerBodyLocal = _grabFrame.ownerBodyLocal,
                                    .desiredObjectWorld = desiredObjectWorldAtSeat,
                                    .desiredBodyWorld = desiredBodyWorldAtSeat,
                                    .pivotAWorld = seatPivotAWorld,
                                    .gripPointWorld = promotedPointWorld,
                                    .visualNormalWorld = promotedNormalWorld,
                                    .source = grab_authority_frame_math::GrabAuthorityPivotSource::GripSupportModel,
                                    .hasDesiredObjectWorld = true,
                                    .hasDesiredBodyWorld = true,
                                    .visualNormalValid = promotedNormalTrusted,
                                });
                            if (!frozenSeatAuthorityFrame.valid) {
                                seatedRetargetRejectedKeepFrozen = true;
                                timeoutReacquireReason = "seatedPalmPocketFreezeFailedKeepFrozen";
                            } else {
                                applyFrozenGrabAuthorityFrameToGrabFrame(_grabFrame, frozenSeatAuthorityFrame);
                                _grabFrame.gripEvidenceLocal = promotedPointNodeLocal;
                                _grabFrame.gripNormalLocal = promotedNormalNodeLocal;
                                _grabFrame.gripSourceNode = nullptr;
                                _grabFrame.gripSourceNodeWorldAtGrab = currentNodeWorld;
                                _grabFrame.gripPointSourceNodeLocal = {};
                                _grabFrame.gripNormalSourceNodeLocal = {};
                                _grabFrame.hasGripSourceNodePoint = false;
                                _grabFrame.hasGripSourceNodeNormal = false;
                                _grabFrame.pocketToGripDistanceGameUnits = promotedPocketDistanceGameUnits;
                                _grabFrame.selectionToGripEvidenceDistanceGameUnits = promotedPocketDistanceGameUnits;
                                _grabFrame.palmSeatPointWorldAtGrab = promotedPointWorld;
                                _grabFrame.hasPalmSeatPoint = true;
                                _grabFrame.activeGrabPointUsesMultiFingerEvidence = false;
                                _grabFrame.activeGrabPointMode = "seatedSupportGroupPromotion";
                                _grabFrame.palmSeatPointMode = _grabFrame.activeGrabPointMode;
                                _grabFrame.pivotAuthoritySource = grabPivotAuthoritySourceName(GrabPivotAuthoritySource::GripSupportModel);
                                _grabFrame.pivotAuthorityPositionOnly = false;
                                _grabFrame.pivotAuthorityNormalTrusted = promotedNormalTrusted;
                                _grabFrame.pivotAuthorityPositionConfidence = 0.92f;
                                _grabFrame.requiresSettledVisualHandRelation = false;
                                _grabFrame.seatMode = GrabSeatMode::SupportGroup;
                                _grabFrame.hasGripSupportModel = true;
                                _grabFrame.gripSupportAuthoredPivot = true;
                                _grabFrame.gripSupportKind = grab_support_model_math::GripSupportKind::PalmWrap;
                                _grabFrame.gripSupportReason = promotionDecision.reason;
                                _grabFrame.gripSupportConfidence = 0.92f;
                                _grabFrame.gripSupportSpanGameUnits = seatedSupportPatch.clusterMaxLateralGameUnits;
                                _grabFrame.gripSupportPivotShiftGameUnits = reacquireLocalDeltaGameUnits;
                                _grabFrame.fingerPoseAimValid = promotedNormalTrusted;
                                _grabFrame.fingerPoseAimReason = promotedNormalTrusted ?
                                    promotionDecision.reason :
                                    "seatedPalmPocketPositionOnly";
                                _grabFrame.objectNodeWorldAtGrab = currentNodeWorld;
                                _grabFrame.longObjectLeverGameUnits = seatedPivot.longLeverGameUnits;
                                _grabFrame.hasSeatedPivotReacquire = true;
                                _grabFrame.lastSeatedPivotReacquireLocalDeltaGameUnits = reacquireLocalDeltaGameUnits;
                                _grabFrame.lastSeatedPivotReacquireReason = promotionDecision.reason ? promotionDecision.reason : "none";
                                ++_grabFrame.seatedPivotReacquireCount;
                                const auto seatedPoseTargets = buildRuntimeFingerPoseTargets(promotedPointWorld, promotedNormalWorld);
                                storeFingerPoseTargetsInGrabFrame(_grabFrame, seatedPoseTargets, currentNodeWorld);

                                /*
                                 * The finger endpoint is object-local and was
                                 * solved against the frozen target relation at
                                 * commit. A seated authority promotion may move
                                 * the constraint pivot, but it must not discard
                                 * and visibly re-fire that endpoint at touch.
                                 */

                                _grabObjectGripAtGrab.objectBodyWorldAtCapture = grabBodyWorld;
                                _grabObjectGripAtGrab.contactSeedWorld = promotedPointWorld;
                                _grabObjectGripAtGrab.contactSeedBodyLocal = frozenSeatAuthorityFrame.pivotBBodyLocalGame;
                                _grabObjectGripAtGrab.gripCenterWorld = promotedPointWorld;
                                _grabObjectGripAtGrab.gripCenterBodyLocal = frozenSeatAuthorityFrame.pivotBBodyLocalGame;
                                _grabObjectGripAtGrab.source = _grabFrame.activeGrabPointMode;
                                _grabObjectGripAtGrab.fallbackReason = promotionDecision.reason;
                                _grabObjectGripAtGrab.confidence = _grabFrame.pivotAuthorityPositionConfidence;
                                _grabObjectGripAtGrab.valid = true;

                                {
                                    // Guard the live proxy pivots during seated promotion.
                                    std::scoped_lock lock(_grabAuthorityProxyMutex);
                                    if (_grabAuthorityProxyFrameValid) {
                                        _grabAuthorityPivotAProxyLocalGame = frozenSeatAuthorityFrame.pivotAHandBodyLocalGame;
                                        _grabAuthorityPivotBConstraintLocalGame = frozenSeatAuthorityFrame.pivotBConstraintLocalGame;
                                    }
                                }

                                timeoutReacquiredSeatedPivot = true;
                                clearGrabExternalHandWorldTransform(_isLeft);
                                _grabVisualHandTransform = handWorldTransform;
                                _hasGrabVisualHandTransform = false;
                                _lastPublishedGrabVisualHandTransform = {};
                                _hasLastPublishedGrabVisualHandTransform = false;
                                _grabVisualHandLerpStartTransform = handWorldTransform;
                                _grabVisualHandLerpElapsedSeconds = 0.0f;
                                _grabVisualHandLerpDurationSeconds = 0.0f;
                                _grabVisualDeviationExceededSeconds = 0.0f;
                                _grabVisualDeviationHistory = {};
                                _grabVisualDeviationHistoryCount = 0;
                                _grabVisualDeviationHistoryNext = 0;
                                ROCK_LOG_DEBUG(Hand,
                                    "{} THREE-PHASE GRAB SEATED SUPPORT-GROUP PROMOTION: phase={} source={} reason={} complete={} blend={:.2f} point=({:.1f},{:.1f},{:.1f}) "
                                    "pivotB=({:.2f},{:.2f},{:.2f}) pocketDistance={:.2f}gu meshDistance={:.2f}gu localDelta={:.2f}gu normalTrusted={} samples={} patchReason={} fingerGroups={} longLever={:.1f}gu count={}",
                                    handName(),
                                    grab_three_phase::phaseName(previousAcquisitionPhase),
                                    _grabFrame.pivotAuthoritySource,
                                    promotionDecision.reason,
                                    promotionDecision.completeSeatedRelation ? "yes" : "no",
                                    pivotBlend,
                                    promotedPointWorld.x,
                                    promotedPointWorld.y,
                                    promotedPointWorld.z,
                                    frozenSeatAuthorityFrame.pivotBConstraintLocalGame.x,
                                    frozenSeatAuthorityFrame.pivotBConstraintLocalGame.y,
                                    frozenSeatAuthorityFrame.pivotBConstraintLocalGame.z,
                                    promotedPocketDistanceGameUnits,
                                    seatedPivot.meshDistanceGameUnits,
                                    reacquireLocalDeltaGameUnits,
                                    promotedNormalTrusted ? "yes" : "no",
                                    _grabFrame.contactPatchSampleCount,
                                    seatedSupportPatch.reason,
                                    _grabFrame.multiFingerContactGroupCount,
                                    seatedPivot.longLeverGameUnits,
                                    _grabFrame.seatedPivotReacquireCount);
                            }
                        } else {
                            seatedRetargetRejectedKeepFrozen = true;
                            _grabFrame.lastSeatedPivotReacquireReason = promotionDecision.reason ? promotionDecision.reason : "none";
                        }
                    } else {
                        seatedRetargetRejectedKeepFrozen = true;
                    }
                }
                if (!timeoutReacquiredSeatedPivot) {
                    _grabFrame.lastSeatedPivotReacquireReason = timeoutReacquireReason ? timeoutReacquireReason : "none";
                }
            }
            if (pivotNeedsSeatedReacquire && !hasGrabBody) {
                _grabFrame.lastSeatedPivotReacquireReason = timeoutReacquireReason;
            }

            const bool seatedReacquireSatisfied = !pivotNeedsSeatedReacquire || timeoutReacquiredSeatedPivot || seatedRetargetRejectedKeepFrozen;
            const bool reachedTouchMayPromote = reachedTouchRange && seatedReacquireSatisfied;
            const bool timeoutMayPromote = convergenceTimedOutInsidePocket && seatedReacquireSatisfied;

            if (promotionRequested && !seatedReacquireSatisfied) {
                ROCK_LOG_SAMPLE_DEBUG(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} THREE-PHASE GRAB CONVERGE PROMOTION HELD: phase={} reason=awaitingSeatedPivot reacquire={} pivotAuthoritySource={} positionOnlyPatch={} settledVisualRequired={} gripErr={:.2f}gu elapsed={:.3f}s colliding={}",
                    handName(),
                    grab_three_phase::phaseName(previousAcquisitionPhase),
                    timeoutReacquireReason,
                    _grabFrame.pivotAuthoritySource,
                    _grabFrame.pivotAuthorityPositionOnly ? "yes" : "no",
                    _grabFrame.requiresSettledVisualHandRelation ? "yes" : "no",
                    gripErrorGameUnits,
                    _grabStartTime,
                    heldBodyColliding ? "yes" : "no");
            }

            if (reachedTouchMayPromote || timeoutMayPromote) {
                const char* promotionReason =
                    timeoutReacquiredSeatedPivot ? "threePhaseReacquiredSeatedPivot" :
                    (_grabFrame.syntheticLooseWeaponPrimaryAttach ? "looseWeaponPrimaryAttachSettled" :
                                                                  (reachedTouchMayPromote ? "threePhaseTouchReachedFrozenRelation" : "threePhaseTimeoutInsidePocket"));
                if (_grabFrame.syntheticLooseWeaponPrimaryAttach) {
                    _grabFrame.hasSettledVisualHandRelation = true;
                }
                /*
                 * Fallback visual settle: a rejected seated reacquire
                 * (seatedRetargetRejectedKeepFrozen) still promotes to
                 * TouchHeld, and this converging block never runs again once
                 * held - without settling here the visual publish gate would
                 * report awaitingSettledVisualRelation for the entire hold,
                 * leaving the rendered hand frozen at the controller while
                 * the motors keep driving the object (hand posed but never
                 * locked; pushing the object slid it out of the hand). The
                 * motors hold the frozen commanded relation (rawHandSpace),
                 * so locking the visual hand onto that same relation is
                 * strictly better than never locking. WARN-logged because it
                 * also records that the seated reacquire failed and why.
                 */
                if (_grabFrame.requiresSettledVisualHandRelation &&
                    !_grabFrame.hasSeatedPivotReacquire &&
                    !_grabFrame.hasSettledVisualHandRelation) {
                    _grabFrame.hasSettledVisualHandRelation = true;
                    ROCK_LOG_WARN(Hand,
                        "{} THREE-PHASE GRAB VISUAL SETTLE FALLBACK: TouchHeld promotion with unsatisfied settle requirement -> settling visual relation to frozen commanded seat (reacquire={} pivotAuthoritySource={} phase={})",
                        handName(),
                        timeoutReacquireReason,
                        _grabFrame.pivotAuthoritySource,
                        grab_three_phase::phaseName(previousAcquisitionPhase));
                }
                _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::TouchHeld;
                _grabFrame.fadeInGrabConstraint = false;
                _grabFrame.motorFadeReason = promotionReason;
                ROCK_LOG_DEBUG(Hand,
                    "{} THREE-PHASE GRAB TRANSITION: {} -> TouchHeld relation=frozenRockPointToPalm reason={} gripErr={:.2f}gu elapsed={:.3f}s colliding={} posePublished={}",
                    handName(),
                    grab_three_phase::phaseName(previousAcquisitionPhase),
                    promotionReason,
                    gripErrorGameUnits,
                    _grabStartTime,
                    heldBodyColliding ? "yes" : "no",
                    _grabFingerPosePublished ? "yes" : "no");

                if (!_grabFrame.syntheticLooseWeaponPrimaryAttach && g_rockConfig.rockGrabMeshFingerPoseEnabled && _hasGrabFingerPose) {
                    if (!_grabFingerPosePublished) {
                        const RE::NiTransform currentNodeWorld = heldNodeWorldFromBodyWorld(grabBodyWorld);
                        if (!_grabFrame.fingerPoseAimValid && _grabFrame.pivotAuthorityNormalTrusted) {
                            _grabFrame.fingerPoseAimValid = true;
                            _grabFrame.fingerPoseAimReason = "touchHeldNormalTrusted";
                        }
                        RE::NiPoint3 fingerPosePivotWorld = _grabFrame.hasTelemetryCapture ? _grabFrame.grabPivotWorldAtGrab : computeGrabPivotAWorld(world, handWorldTransform);
                        RE::NiPoint3 livePivotAWorld{};
                        if (tryComputeGrabProxyLocalPalmPocketPivotAWorld(world, livePivotAWorld)) {
                            fingerPosePivotWorld = livePivotAWorld;
                        }
                        const auto touchHeldWorldTriangles = rebuildFingerPoseWorldTrianglesFromGrabFrame(_grabFrame, currentNodeWorld);
                        root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshot{};
                        const auto* liveFingerSnapshotPtr =
                            root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(_isLeft, liveFingerSnapshot) ? &liveFingerSnapshot : nullptr;
                        const auto touchHeldFingerPoseTargets = rebuildFingerPoseTargetsFromGrabFrame(_grabFrame, currentNodeWorld);
                        const bool pinchFingerPose = _grabFrame.seatMode == GrabSeatMode::PinchPocket;
                        _grabFingerPose = grab_finger_pose_runtime::solveGrabFingerPoseFromTriangles(touchHeldWorldTriangles, handWorldTransform, _isLeft, fingerPosePivotWorld,
                            touchHeldFingerPoseTargets, g_rockConfig.rockGrabFingerMinValue, g_rockConfig.rockGrabMaxTriangleDistance, !pinchFingerPose, liveFingerSnapshotPtr,
                            g_rockConfig.rockGrabFingerRejectBacksideHits, g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits, _grabFrame.fingerPoseAimValid,
                            g_rockConfig.rockGrabFingerSweepContactRadiusGameUnits, -1.0f, g_rockConfig.rockGrabThumbSweepMaxOpenValue,
                            g_rockConfig.rockGrabFingerSweepMaxOpenValue);
                        if (pinchFingerPose) {
                            applyPinchFingerPosePolicy(_grabFingerPose, _grabFrame, g_rockConfig.rockGrabFingerMinValue);
                        }
                        grab_finger_pose_runtime::useThumbIndexCurveOnlyPose(_grabFingerPose);
                        std::array<grab_finger_pose_runtime::FingerPadSurfaceEvidence, 5> padCaptureEvidence{};
                        (void)grab_finger_pose_runtime::refineGrabFingerPoseWithPadProbes(_grabFingerPose, touchHeldWorldTriangles, touchHeldFingerPoseTargets, liveFingerSnapshot,
                            currentNodeWorld, g_rockConfig.rockGrabMeshFingerPoseEnabled, true, padCaptureEvidence, true);
                        grab_finger_pose_runtime::captureSurfaceAimObjectLocal(_grabFingerPose, currentNodeWorld);
                        _grabFingerProbeStart = _grabFingerPose.probeStart;
                        _grabFingerProbeEnd = _grabFingerPose.probeEnd;
                        _hasGrabFingerProbeDebug = _grabFingerPose.candidateTriangleCount > 0;
                        auto publishFingerPose = grab_finger_pose_runtime::resolveSurfaceAimObjectLocal(_grabFingerPose, currentNodeWorld);
                        if (g_rockConfig.rockDebugShowGrabFingerProbes) {
                            std::array<grab_finger_pose_runtime::FingerPadSurfaceEvidence, 5> padEvidence{};
                            (void)grab_finger_pose_runtime::refineGrabFingerPoseWithPadProbes(publishFingerPose, touchHeldWorldTriangles, touchHeldFingerPoseTargets,
                                liveFingerSnapshot, currentNodeWorld, g_rockConfig.rockGrabMeshFingerPoseEnabled, true, padEvidence, false);
                            const auto padDebug = makeFingerPadPublishDebug(publishFingerPose, padEvidence);
                            _grabFingerPadProbeStart = padDebug.padProbeStart;
                            _grabFingerPadProbeEnd = padDebug.padProbeEnd;
                            _grabFingerPadProbeHit = padDebug.padProbeHit;
                            _grabFingerPadProbeHitValid = padDebug.padProbeHitValid;
                            _hasGrabFingerPadProbeDebug = padDebug.hasPadProbeDebug;
                            _grabFingerSurfaceTarget = padDebug.surfaceTarget;
                            _grabFingerSurfaceTargetValid = padDebug.surfaceTargetValid;
                            _hasGrabFingerSurfaceTargetDebug = padDebug.hasSurfaceTargetDebug;
                        } else {
                            _hasGrabFingerPadProbeDebug = false;
                            _hasGrabFingerSurfaceTargetDebug = false;
                        }
                        applyRockGrabHandPose(_isLeft, publishFingerPose, _grabFingerJointPose, _hasGrabFingerJointPose, _grabFingerLocalTransforms, _grabFingerLocalTransformMask,
                            _hasGrabFingerLocalTransforms, 0.0f, true, true);
                        _grabFingerPosePublished = true;
                    } else {
                        const RE::NiTransform finalPoseObjectWorld = grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyAuthorityWorld, _grabFrame.proxyAuthorityHandSpace);
                        const auto finalFingerPose = grab_finger_pose_runtime::resolveSurfaceAimObjectLocal(_grabFingerPose, finalPoseObjectWorld);
                        applyRockGrabHandPose(_isLeft, finalFingerPose, _grabFingerJointPose, _hasGrabFingerJointPose, _grabFingerLocalTransforms, _grabFingerLocalTransformMask,
                            _hasGrabFingerLocalTransforms, 0.0f, true, true);
                    }
                }
            }
        }

        const float fadeDuration = std::max(forceFadeInTime, 0.0001f);
        const float grabFadeFactor = _grabFrame.fadeInGrabConstraint ? std::clamp(_grabStartTime / fadeDuration, 0.0f, 1.0f) : 1.0f;

        if (_state == HandState::HeldInit) {
            if (grabFadeFactor >= 0.999f) {
                applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::HeldFadeComplete });
                ROCK_LOG_DEBUG(Hand, "{} hand: HeldInit -> HeldBody ({} dynamic grab fade complete, {:.2f}s)", handName(), kHeldObjectDriveName, _grabStartTime);
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
                // Guard the proxy sequence counters during the log snapshot.
                std::scoped_lock lock(_grabAuthorityProxyMutex);
                driveQueuedTargets = _grabAuthorityProxyQueuedSequence;
                driveFlushedTargets = _grabAuthorityProxyFlushSequence;
                driveFailedFlushes = _grabAuthorityProxyFailedFlushes;
                driveLastFlushDeltaSeconds = _grabAuthorityProxyLastFlushDeltaSeconds;
            }
            RE::NiTransform liveHandBodyWorld{};
            RE::NiTransform grabObjectBodyWorld{};
            RE::NiTransform grabDriveObjectWorld{};
            RE::NiTransform motionObjectBodyWorld{};
            const bool hasLiveHandBody =
                _handBody.getBodyId().value != INVALID_BODY_ID && tryResolveLiveBodyWorldTransform(world, _handBody.getBodyId(), liveHandBodyWorld);
            const bool hasGrabObjectBody = tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, grabObjectBodyWorld);
            const bool hasGrabDriveObject = tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, grabDriveObjectWorld);
            const bool hasMotionObjectBody = tryResolveLiveBodyWorldTransform(world, _savedObjectState.bodyId, motionObjectBodyWorld);

            if (g_rockConfig.rockDebugGrabFrameLogging) {
                const RE::NiTransform desiredNodeWorldRaw = multiplyTransforms(handWorldTransform, _grabFrame.rawHandSpace);
                const RE::NiTransform desiredBodyWorldRaw = multiplyTransforms(desiredNodeWorldRaw, _grabFrame.bodyLocal);
                const RE::NiTransform grabAuthorityBodyWorld = hasGrabObjectBody ? grabObjectBodyWorld : getGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId);
                auto* ownerCell = _savedObjectState.refr ? _savedObjectState.refr->GetParentCell() : nullptr;
                auto* heldBhkWorld = ownerCell ? ownerCell->GetbhkWorld() : nullptr;
                auto* bodyCollisionObject = heldBhkWorld ? RE::bhkNPCollisionObject::Getbhk(heldBhkWorld, _savedObjectState.bodyId) : nullptr;
                auto* ownerNode = bodyCollisionObject ? bodyCollisionObject->sceneObject : nullptr;
                auto* hitNode = (_currentSelection.refr == _savedObjectState.refr) ? _currentSelection.hitNode : nullptr;
                auto* rootNode = _savedObjectState.refr ? _savedObjectState.refr->Get3D() : nullptr;

                struct NodeFrameMetrics
                {
                    RE::NiAVObject* node = nullptr;
                    RE::NiTransform world = {};
                    RE::NiTransform expectedWorld = {};
                    RE::NiPoint3 finger = {};
                    RE::NiPoint3 expectedFinger = {};
                    float rotErrDeg = -1.0f;
                    float posErrGameUnits = -1.0f;
                    bool hasCollisionObject = false;
                    bool ownsBodyCollisionObject = false;
                };

                const auto captureNodeMetrics = [&](RE::NiAVObject* node, const RE::NiTransform& bodyLocalTransform) {
                    NodeFrameMetrics metrics{};
                    metrics.node = node;
                    metrics.world = node ? node->world : grabAuthorityBodyWorld;
                    metrics.expectedWorld = node ? deriveNodeWorldFromBodyWorld(grabAuthorityBodyWorld, bodyLocalTransform) : grabAuthorityBodyWorld;
                    metrics.finger = getMatrixColumn(metrics.world.rotate, 0);
                    metrics.expectedFinger = getMatrixColumn(metrics.expectedWorld.rotate, 0);
                    if (node) {
                        metrics.rotErrDeg = rotationDeltaDegrees(metrics.world.rotate, metrics.expectedWorld.rotate);
                        metrics.posErrGameUnits = translationDeltaGameUnits(metrics.world, metrics.expectedWorld);
                        auto* nodeCollisionObject = node->collisionObject.get();
                        metrics.hasCollisionObject = (nodeCollisionObject != nullptr);
                        metrics.ownsBodyCollisionObject = (nodeCollisionObject != nullptr && nodeCollisionObject == bodyCollisionObject);
                    }
                    return metrics;
                };

                const NodeFrameMetrics ownerMetrics = captureNodeMetrics(ownerNode, _grabFrame.ownerBodyLocal);
                const NodeFrameMetrics hitMetrics = captureNodeMetrics(hitNode, _grabFrame.bodyLocal);
                const NodeFrameMetrics heldMetrics = captureNodeMetrics(_grabFrame.heldNode, _grabFrame.bodyLocal);
                const NodeFrameMetrics rootMetrics = captureNodeMetrics(rootNode, _grabFrame.rootBodyLocal);

                const RE::NiPoint3 rawFinger = getMatrixColumn(handWorldTransform.rotate, 2);
                const RE::NiPoint3 desiredRawFinger = getMatrixColumn(desiredBodyWorldRaw.rotate, 0);
                const RE::NiPoint3 bodyFinger = getMatrixColumn(grabAuthorityBodyWorld.rotate, 0);
                const float motionDiagVsGrabRot =
                    (hasMotionObjectBody && hasGrabObjectBody) ? rotationDeltaDegrees(motionObjectBodyWorld.rotate, grabObjectBodyWorld.rotate) : -1.0f;
                const float motionDiagVsGrabPos =
                    (hasMotionObjectBody && hasGrabObjectBody) ? translationDeltaGameUnits(motionObjectBodyWorld, grabObjectBodyWorld) : -1.0f;
                const float rawRowMax = max3(axisDeltaDegrees(getMatrixRow(grabAuthorityBodyWorld.rotate, 0), getMatrixRow(desiredBodyWorldRaw.rotate, 0)),
                    axisDeltaDegrees(getMatrixRow(grabAuthorityBodyWorld.rotate, 1), getMatrixRow(desiredBodyWorldRaw.rotate, 1)),
                    axisDeltaDegrees(getMatrixRow(grabAuthorityBodyWorld.rotate, 2), getMatrixRow(desiredBodyWorldRaw.rotate, 2)));
                const float rawColMax = max3(axisDeltaDegrees(getMatrixColumn(grabAuthorityBodyWorld.rotate, 0), getMatrixColumn(desiredBodyWorldRaw.rotate, 0)),
                    axisDeltaDegrees(getMatrixColumn(grabAuthorityBodyWorld.rotate, 1), getMatrixColumn(desiredBodyWorldRaw.rotate, 1)),
                    axisDeltaDegrees(getMatrixColumn(grabAuthorityBodyWorld.rotate, 2), getMatrixColumn(desiredBodyWorldRaw.rotate, 2)));

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB FRAME HOLD: hasLiveHandBody={} bodyVsRaw={:.2f}deg "
                    "ownerErr={:.2f}deg/{:.2f}gu hitErr={:.2f}deg/{:.2f}gu "
                    "heldErr={:.2f}deg/{:.2f}gu rootErr={:.2f}deg/{:.2f}gu "
                    "rawFinger=({:.3f},{:.3f},{:.3f}) bodyFinger=({:.3f},{:.3f},{:.3f}) desiredRawFinger=({:.3f},{:.3f},{:.3f})",
                    handName(), hasLiveHandBody ? "yes" : "no", rotationDeltaDegrees(grabAuthorityBodyWorld.rotate, desiredBodyWorldRaw.rotate),
                    ownerMetrics.rotErrDeg, ownerMetrics.posErrGameUnits, hitMetrics.rotErrDeg, hitMetrics.posErrGameUnits, heldMetrics.rotErrDeg, heldMetrics.posErrGameUnits,
                    rootMetrics.rotErrDeg, rootMetrics.posErrGameUnits, rawFinger.x, rawFinger.y, rawFinger.z,
                    bodyFinger.x, bodyFinger.y, bodyFinger.z, desiredRawFinger.x, desiredRawFinger.y, desiredRawFinger.z);

                ROCK_LOG_TRACE(Hand,
                    "{} GRAB FRAME NODES: bodyColl={:p} "
                    "owner='{}'({:p}) hasCol={} ownsBodyCol={} "
                    "hit='{}'({:p}) hasCol={} ownsBodyCol={} "
                    "held='{}'({:p}) hasCol={} ownsBodyCol={} "
                    "root='{}'({:p}) hasCol={} ownsBodyCol={} "
                    "sameOwnerHeld={} sameHitHeld={} sameRootHeld={}",
                    handName(), static_cast<const void*>(bodyCollisionObject), nodeDebugName(ownerMetrics.node), static_cast<const void*>(ownerMetrics.node),
                    ownerMetrics.hasCollisionObject ? "yes" : "no", ownerMetrics.ownsBodyCollisionObject ? "yes" : "no", nodeDebugName(hitMetrics.node),
                    static_cast<const void*>(hitMetrics.node), hitMetrics.hasCollisionObject ? "yes" : "no", hitMetrics.ownsBodyCollisionObject ? "yes" : "no",
                    nodeDebugName(heldMetrics.node), static_cast<const void*>(heldMetrics.node), heldMetrics.hasCollisionObject ? "yes" : "no",
                    heldMetrics.ownsBodyCollisionObject ? "yes" : "no", nodeDebugName(rootMetrics.node), static_cast<const void*>(rootMetrics.node),
                    rootMetrics.hasCollisionObject ? "yes" : "no", rootMetrics.ownsBodyCollisionObject ? "yes" : "no", ownerMetrics.node == heldMetrics.node ? "yes" : "no",
                    hitMetrics.node == heldMetrics.node ? "yes" : "no", rootMetrics.node == heldMetrics.node ? "yes" : "no");

                ROCK_LOG_TRACE(Hand,
                    "{} GRAB FRAME VISUALS: desiredRawFinger=({:.3f},{:.3f},{:.3f}) bodyFinger=({:.3f},{:.3f},{:.3f}) "
                    "ownerFinger=({:.3f},{:.3f},{:.3f}) ownerExpectedFinger=({:.3f},{:.3f},{:.3f}) "
                    "hitFinger=({:.3f},{:.3f},{:.3f}) hitExpectedFinger=({:.3f},{:.3f},{:.3f}) "
                    "heldFinger=({:.3f},{:.3f},{:.3f}) heldExpectedFinger=({:.3f},{:.3f},{:.3f}) "
                    "rootFinger=({:.3f},{:.3f},{:.3f}) rootExpectedFinger=({:.3f},{:.3f},{:.3f})",
                    handName(), desiredRawFinger.x, desiredRawFinger.y, desiredRawFinger.z, bodyFinger.x, bodyFinger.y, bodyFinger.z, ownerMetrics.finger.x, ownerMetrics.finger.y,
                    ownerMetrics.finger.z, ownerMetrics.expectedFinger.x,
                    ownerMetrics.expectedFinger.y, ownerMetrics.expectedFinger.z, hitMetrics.finger.x, hitMetrics.finger.y, hitMetrics.finger.z, hitMetrics.expectedFinger.x,
                    hitMetrics.expectedFinger.y, hitMetrics.expectedFinger.z, heldMetrics.finger.x, heldMetrics.finger.y, heldMetrics.finger.z, heldMetrics.expectedFinger.x,
                    heldMetrics.expectedFinger.y, heldMetrics.expectedFinger.z, rootMetrics.finger.x, rootMetrics.finger.y, rootMetrics.finger.z, rootMetrics.expectedFinger.x,
                    rootMetrics.expectedFinger.y, rootMetrics.expectedFinger.z);

                const auto massSummary = readHeldBodyMassSummary(
                    world,
                    _savedObjectState.bodyId,
                    _heldBodyIds,
                    _heldDriveDecision.includeConnectedMass);
                ROCK_LOG_TRACE(Hand,
                    "{} GRAB ANGULAR PROBE: rawAxisErr(rowMax={:.2f} colMax={:.2f}) driveErr=({:.2f}gu,{:.2f}deg) mass={:.2f} primaryMass={:.2f} massBodies={} motions={} "
                    "motionDiagVsGrab={:.2f}deg/{:.2f}gu fade={:.2f} fadeEnabled={} fadeReason={} drive={} constraint={} queued={} flushed={} failedFlushes={} lastDt={:.6f}",
                    handName(),
                    rawRowMax,
                    rawColMax,
                    pivotTrackingErrorGameUnits,
                    grabRotationErrorDegrees,
                    massSummary.motorMass(),
                    massSummary.primaryMass,
                    massSummary.sampledBodies,
                    massSummary.uniqueMotions,
                    motionDiagVsGrabRot,
                    motionDiagVsGrabPos,
                    grabFadeFactor,
                    _grabFrame.fadeInGrabConstraint ? "yes" : "no",
                    _grabFrame.motorFadeReason,
                    kHeldObjectDriveName,
                    _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
                    driveQueuedTargets,
                    driveFlushedTargets,
                    driveFailedFlushes,
                    driveLastFlushDeltaSeconds);
            }

            const bool hasGrabPivotFrame = hasGrabDriveObject;
            const RE::NiPoint3 pivotAWorld = desiredTargetPointWorld;
            const RE::NiPoint3 pivotBWorld = hasGrabPivotFrame ? transform_math::localPointToWorld(grabDriveObjectWorld, activePivotBBodyLocalGame) : RE::NiPoint3{};
            const RE::NiPoint3 pivotError = pivotAWorld - pivotBWorld;
            const float pivotErrGame = hasGrabPivotFrame ? std::sqrt(pivotError.x * pivotError.x + pivotError.y * pivotError.y + pivotError.z * pivotError.z) : 0.0f;

            const RE::NiPoint3 bodyDelta = hasGrabPivotFrame ? (grabDriveObjectWorld.translate - desiredBodyWorld.translate) : RE::NiPoint3{};
            const float bodyDistGame = hasGrabPivotFrame ? std::sqrt(bodyDelta.x * bodyDelta.x + bodyDelta.y * bodyDelta.y + bodyDelta.z * bodyDelta.z) : 0.0f;

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
            const float lastGrabPhysicsRateForceScale = _lastGrabPhysicsRateForceScale.load(std::memory_order_relaxed);

            ROCK_LOG_DEBUG(Hand,
                "{} HELD dynamic: drive={} bodyDriveMode={} linearScope={} angularScope={} massScope={} looseWeapon={} formID={:08X} constraint={} queued={} flushed={} failedFlushes={} lastDt={:.6f} proxyFrame={}/{} "
                "phase={} posePublished={} fade={:.2f}/{} reason={} colliding={} motorContact={} contactReason={} forceBudget={:.2f} physHz={:.1f} forceScale={:.3f} longLever={:.1f}gu pivotTrack={:.1f}gu avgTrack={:.1f}gu rotErr={:.1f}deg bDist={:.1f}gu objVel={:.3f} "
                "paW=({:.1f},{:.1f},{:.1f}) pbW=({:.1f},{:.1f},{:.1f}) "
                "targetBody=({:.1f},{:.1f},{:.1f}) objW=({:.1f},{:.1f},{:.1f})",
                handName(),
                kHeldObjectDriveName,
                held_object_drive_policy::modeName(_heldDriveDecision.mode),
                _heldDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
                _heldDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly",
                _heldDriveDecision.includeConnectedMass ? "bodySet" : "primaryOnly",
                _heldObjectIsLooseWeapon ? "yes" : "no",
                heldFormId,
                _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
                driveQueuedTargets,
                driveFlushedTargets,
                driveFailedFlushes,
                driveLastFlushDeltaSeconds,
                proxyAuthoritySource,
                hasProxyAuthorityFrame ? "ok" : "fallback",
                grab_three_phase::phaseName(_grabAcquisitionPhase),
                _grabFingerPosePublished ? "yes" : "no",
                grabFadeFactor,
                _grabFrame.fadeInGrabConstraint ? "on" : "off",
                _grabFrame.motorFadeReason,
                heldBodyColliding ? "yes" : "no",
                heldMotorContactSoftening ? "soften" : "preserve",
                heldMotorContactReason,
                authorityForceScale,
                lastGrabPhysicsHz,
                lastGrabPhysicsRateForceScale,
                _grabFrame.longObjectLeverGameUnits,
                pivotErrGame,
                averageGrabDeviationGameUnits,
                grabRotationErrorDegrees,
                bodyDistGame,
                objVelMag,
                pivotAWorld.x, pivotAWorld.y, pivotAWorld.z, pivotBWorld.x, pivotBWorld.y, pivotBWorld.z,
                desiredBodyWorld.translate.x, desiredBodyWorld.translate.y, desiredBodyWorld.translate.z,
                grabDriveObjectWorld.translate.x, grabDriveObjectWorld.translate.y, grabDriveObjectWorld.translate.z);

            _notifCounter++;
            if (hasGrabPivotFrame && g_rockConfig.rockDebugShowGrabNotifications && _notifCounter >= 6) {
                _notifCounter = 0;
                const RE::NiPoint3 pivotAToHand = pivotAWorld - desiredBodyWorld.translate;
                const RE::NiPoint3 pivotBToObject = pivotBWorld - grabDriveObjectWorld.translate;
                const float paToHand = std::sqrt(pivotAToHand.x * pivotAToHand.x + pivotAToHand.y * pivotAToHand.y + pivotAToHand.z * pivotAToHand.z);
                const float pbToObj = std::sqrt(pivotBToObject.x * pivotBToObject.x + pivotBToObject.y * pivotBToObject.y + pivotBToObject.z * pivotBToObject.z);
                f4vr::showNotification(
                    std::format("[ROCK] track={:.1f}gu vel={:.2f} flush={} paOff={:.1f} pbOff={:.1f}", pivotErrGame, objVelMag, driveFlushedTargets, paToHand, pbToObj));
            }
        }

        {
            const auto wakeBodies = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(_savedObjectState.bodyId.value, _heldBodyIds);
            for (const auto bodyId : wakeBodies) {
                physics_recursive_wrappers::activateBody(world, bodyId);
            }
        }
    }

}
