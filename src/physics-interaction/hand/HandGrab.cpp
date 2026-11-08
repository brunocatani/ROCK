// Hand::grabSelectedObject: the grab of a selected object, end to end. It
// validates the selection, prepares the object's bodies and the grab authority
// proxy, captures the grab surface and seat (HandGrabCapture.cpp,
// HandGrabSeat.cpp) and commits the constraint and finger pose
// (HandGrabCommit.cpp). Any failure rolls the preparation back.
#include "physics-interaction/hand/HandGrabInternal.h"

namespace rock
{

    namespace
    {
        template <class Rollback>
        class GrabPreparationTransaction
        {
        public:
            explicit GrabPreparationTransaction(Rollback rollback) : _rollback(std::move(rollback)) {}

            GrabPreparationTransaction(const GrabPreparationTransaction&) = delete;
            GrabPreparationTransaction& operator=(const GrabPreparationTransaction&) = delete;
            GrabPreparationTransaction(GrabPreparationTransaction&&) = delete;
            GrabPreparationTransaction& operator=(GrabPreparationTransaction&&) = delete;

            ~GrabPreparationTransaction() noexcept { rollback(); }

            void rollback() noexcept
            {
                if (_active) {
                    _active = false;
                    _rollback();
                }
            }

            void commit() noexcept { _active = false; }

        private:
            Rollback _rollback;
            bool _active = true;
        };

        std::uint64_t nextGrabTraceId() noexcept
        {
            static std::atomic<std::uint64_t> nextTraceId{ 1 };
            return nextTraceId.fetch_add(1, std::memory_order_relaxed);
        }

        grab_pinch_pocket_policy::Config currentPinchPocketConfig()
        {
            return grab_pinch_pocket_policy::sanitizeConfig(grab_pinch_pocket_policy::Config{
                .enabled = g_rockConfig.rockGrabPinchPocketEnabled,
                .maxPocketDistanceGameUnits = g_rockConfig.rockGrabPinchMaxPocketDistanceGameUnits,
                .minFingerGapGameUnits = g_rockConfig.rockGrabPinchMinFingerGapGameUnits,
                .maxFingerGapGameUnits = g_rockConfig.rockGrabPinchMaxFingerGapGameUnits,
                .thumbIndexMaxOpenValue = g_rockConfig.rockGrabPinchThumbIndexMaxOpenValue,
                .otherFingerCurlValue = g_rockConfig.rockGrabPinchOtherFingerCurlValue,
                .detectionDirectionHandspace = g_rockConfig.rockGrabPinchDetectionDirectionHandspace,
                .detectionAxisBlend = g_rockConfig.rockGrabPinchDetectionAxisBlend,
            });
        }

        bool solveRuntimePinchClosure(RuntimePinchPocketCandidate& candidate,
            const std::vector<GrabLocalTriangle>& localTriangles, const RE::NiTransform& objectWorld,
            const RE::NiTransform& handWorld, bool isLeft, const Hand& hand);

        // The object's span along the pinch axis through the pinch surface: the
        // mesh between the two pads. False when no span can be measured.
        bool measurePinchSpanGameUnits(const std::vector<GrabLocalTriangle>& localTriangles,
            const RE::NiTransform& objectWorld, const RE::NiPoint3& surfacePointWorld, const RE::NiPoint3& axisWorld,
            float maxDepthGameUnits, float& outSpanGameUnits);

        RuntimePinchPocketCandidate buildRuntimePinchPocketCandidate(const SelectedObject& selection,
            const object_physics_body_set::ObjectPhysicsBodySet& bodySet, std::uint32_t resolvedBodyId,
            const RE::NiTransform& objectWorldTransform, const std::vector<GrabSurfaceTriangleData>& surfaceTriangles,
            const GrabSurfaceQueryIndex& queryIndex, const std::vector<GrabLocalTriangle>& localMeshTriangles,
            const RE::NiTransform& handWorldTransform, bool isLeft, const Hand& hand, bool closeGrab,
            bool handPocketOnlyGrab, bool looseWeaponGrab)
        {
            performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabPinchPocket);
            RuntimePinchPocketCandidate candidate{};
            const auto config = currentPinchPocketConfig();
            candidate.meshExtents =
                grab_pinch_pocket_policy::computeMeshExtents(localMeshTriangles, objectWorldTransform.scale);

            grab_pinch_pocket_policy::FingerFrame pinchFrame{};
            const bool hasFingerSnapshot = hand.tryGetPinchFingerFrame(pinchFrame);

            bool hasPinchSurface = false;
            bool ownerMatchesResolvedBody = false;
            if (hasFingerSnapshot && !surfaceTriangles.empty()) {
                candidate.thumbPadWorld = pinchFrame.thumbTip;
                candidate.indexPadWorld = pinchFrame.indexTip;
                candidate.thumbIndexGapGameUnits = pinchFrame.gapGameUnits;
                candidate.pinchAxisWorld = pinchFrame.axis;
                candidate.pinchPocketWorld = pinchFrame.center;
                const RE::NiPoint3 configuredDetectionWorld =
                    transformHandspaceDirection(handWorldTransform, config.detectionDirectionHandspace, isLeft);
                candidate.pinchDetectionDirectionWorld = grab_pinch_pocket_policy::detectionDirection(pinchFrame,
                    configuredDetectionWorld, config.detectionAxisBlend);

                GrabSurfaceHit surfaceHit{};
                hasPinchSurface = findClosestGrabSurfaceHitToPointPositionOnly(surfaceTriangles,
                    candidate.pinchPocketWorld, candidate.pinchDetectionDirectionWorld,
                    config.maxPocketDistanceGameUnits, surfaceHit, &queryIndex);
                if (hasPinchSurface) {
                    candidate.surfaceHit = surfaceHit;
                    candidate.pocketToSurfaceDistanceGameUnits =
                        grab_pinch_pocket_policy::distance(candidate.pinchPocketWorld, surfaceHit.position);
                    if (surfaceHit.sourceNode) {
                        const auto* ownerRecord = bodySet.findAcceptedRecordByOwnerNode(surfaceHit.sourceNode);
                        ownerMatchesResolvedBody = (ownerRecord && ownerRecord->bodyId == resolvedBodyId) ||
                            acceptsSelectedOwnerlessVisualMesh(selection, bodySet, resolvedBodyId,
                                surfaceHit.sourceNode, ownerRecord);
                    }
                    // Is the object between the live pads: its span along the live pinch axis.
                    float spanGameUnits = 0.0f;
                    if (candidate.meshExtents.valid &&
                        measurePinchSpanGameUnits(localMeshTriangles, objectWorldTransform, surfaceHit.position,
                            pinchFrame.axis, candidate.meshExtents.maxExtentGameUnits * 1.73205081f, spanGameUnits)) {
                        candidate.pinchSpanGameUnits = spanGameUnits;
                    }
                }
            }

            // Count-based loose ammo is one rigid object: its root body is carried
            // on the visible body (restoreCountBasedAmmoAssembly), so it does not
            // make the round a multi-body object for the pinch.
            const std::uint32_t carriedAnchorBodyId = hand.countBasedAmmoAnchorBodyId();
            const std::size_t movingBodyCount = bodySet.acceptedCount() -
                ((carriedAnchorBodyId != object_physics_body_set::INVALID_BODY_ID &&
                     bodySet.containsAcceptedBody(carriedAnchorBodyId))
                        ? 1u
                        : 0u);
            candidate.decision = grab_pinch_pocket_policy::evaluateObject(grab_pinch_pocket_policy::ObjectDecisionInput{
                .config = config,
                .mesh = candidate.meshExtents,
                .closeGrab = closeGrab,
                .handPocketOnlyGrab = handPocketOnlyGrab,
                .looseWeaponGrab = looseWeaponGrab,
                .ownerMatchesResolvedBody = ownerMatchesResolvedBody,
                .hasFingerSnapshot = hasFingerSnapshot,
                .hasPinchSurface = hasPinchSurface,
                .multipleAcceptedBodies = movingBodyCount > 1,
                .thumbIndexGapGameUnits = candidate.thumbIndexGapGameUnits,
                .pocketToSurfaceDistanceGameUnits = candidate.pocketToSurfaceDistanceGameUnits,
                .pinchSpanGameUnits = candidate.pinchSpanGameUnits,
            });
            candidate.valid = candidate.decision.accept;
            if (candidate.valid &&
                !solveRuntimePinchClosure(candidate, localMeshTriangles, objectWorldTransform, handWorldTransform,
                    isLeft, hand)) {
                candidate.valid = false;
                candidate.decision.accept = false;
                candidate.decision.retryable = true;
                candidate.decision.reason = "pinchPoseUnavailable";
            }
            return candidate;
        }

        constexpr float kGrabFrameMismatchRawProxyRotationWarnDegrees = 20.0f;

        /*
         * Pinch thickness uses the existing circular pad footprint on both
         * sides of the pinch axis. Palm seating uses the created palm hull
         * footprint above; finger reach must not widen that support area.
         */
        GrabSeatDepthStopResult computeGrabSeatDepthStop(const std::vector<GrabLocalTriangle>& localTriangles,
            const RE::NiTransform& objectNodeWorld, const RE::NiPoint3& gripPointWorld,
            const RE::NiPoint3& palmNormalWorld, float footprintRadiusGameUnits, float maxDepthGameUnits)
        {
            GrabSeatDepthStopResult result{};
            if (!std::isfinite(maxDepthGameUnits) || maxDepthGameUnits <= 0.0f) {
                result.reason = "seatDepthDisabled";
                return result;
            }
            if (localTriangles.empty()) {
                result.reason = "noLocalTriangles";
                return result;
            }
            if (!grab_three_phase::isFinite(objectNodeWorld) || !grab_three_phase::isFinite(gripPointWorld) ||
                !grab_three_phase::isFinite(palmNormalWorld)) {
                result.reason = "nonFiniteSeatFrame";
                return result;
            }
            const RE::NiPoint3 inwardLocal = normalizeOrZero(transform_math::worldVectorToLocal(objectNodeWorld,
                RE::NiPoint3{ -palmNormalWorld.x, -palmNormalWorld.y, -palmNormalWorld.z }));
            if (lengthSquared(inwardLocal) <= 0.000001f) {
                result.reason = "degeneratePalmNormal";
                return result;
            }

            const float objectScale = finitePositiveOr(objectNodeWorld.scale, 1.0f);
            const float footprintRadiusLocal =
                (std::max)(0.1f, finitePositiveOr(footprintRadiusGameUnits, 10.0f)) / objectScale;
            const float footprintRadiusLocalSquared = footprintRadiusLocal * footprintRadiusLocal;
            const float maxDepthLocal = maxDepthGameUnits / objectScale;
            const RE::NiPoint3 gripLocal = transform_math::worldPointToLocal(objectNodeWorld, gripPointWorld);
            if (!grab_three_phase::isFinite(gripLocal)) {
                result.reason = "nonFiniteGripLocal";
                return result;
            }

            float bestDepthLocal = 0.0f;
            std::uint32_t footprintSampleCount = 0;
            auto considerLocalPoint = [&](const RE::NiPoint3& pointLocal) {
                const RE::NiPoint3 delta = pointLocal - gripLocal;
                const float depthLocal = dotProduct(delta, inwardLocal);
                if (!std::isfinite(depthLocal) || depthLocal <= 0.0f) {
                    return;
                }
                const RE::NiPoint3 lateral = delta - inwardLocal * depthLocal;
                const float lateralSquared = lengthSquared(lateral);
                if (!std::isfinite(lateralSquared) || lateralSquared > footprintRadiusLocalSquared) {
                    return;
                }
                ++footprintSampleCount;
                bestDepthLocal = (std::max)(bestDepthLocal, (std::min)(depthLocal, maxDepthLocal));
            };
            const std::array<float, 3> axisProbeDepthsLocal{ 0.0f, maxDepthLocal * 0.5f, maxDepthLocal };
            for (const auto& localTriangle : localTriangles) {
                considerLocalPoint(localTriangle.v0);
                considerLocalPoint(localTriangle.v1);
                considerLocalPoint(localTriangle.v2);
                const TriangleData triangle{ localTriangle.v0, localTriangle.v1, localTriangle.v2 };
                for (const float axisDepthLocal : axisProbeDepthsLocal) {
                    const RE::NiPoint3 axisPointLocal = gripLocal + inwardLocal * axisDepthLocal;
                    float distanceSquared = 0.0f;
                    considerLocalPoint(closestPointOnTriangleToPoint(axisPointLocal, triangle, distanceSquared));
                }
            }

            result.depthGameUnits = bestDepthLocal * objectScale;
            result.footprintSampleCount = footprintSampleCount;
            result.reason = footprintSampleCount > 0 ? "meshSupportDepth" : "noMeshInsideFootprint";
            result.valid = true;
            return result;
        }

        bool measurePinchSpanGameUnits(const std::vector<GrabLocalTriangle>& localTriangles,
            const RE::NiTransform& objectWorld, const RE::NiPoint3& surfacePointWorld, const RE::NiPoint3& axisWorld,
            float maxDepthGameUnits, float& outSpanGameUnits)
        {
            // The pad footprint on both sides of the axis, as the pinch closure measures thickness.
            constexpr float footprintRadius = 2.5f;
            const auto towardIndex = computeGrabSeatDepthStop(localTriangles, objectWorld, surfacePointWorld,
                axisWorld * -1.0f, footprintRadius, maxDepthGameUnits);
            const auto towardThumb = computeGrabSeatDepthStop(localTriangles, objectWorld, surfacePointWorld,
                axisWorld, footprintRadius, maxDepthGameUnits);
            if (!towardIndex.valid || !towardThumb.valid) {
                return false;
            }
            outSpanGameUnits = towardIndex.depthGameUnits + towardThumb.depthGameUnits;
            return std::isfinite(outSpanGameUnits);
        }

        bool solveRuntimePinchClosure(RuntimePinchPocketCandidate& candidate,
            const std::vector<GrabLocalTriangle>& localTriangles, const RE::NiTransform& objectWorld,
            const RE::NiTransform& handWorld, bool isLeft, const Hand& hand)
        {
            namespace pinch = grab_pinch_pocket_policy;
            const auto config = currentPinchPocketConfig();
            constexpr auto tipSegment = static_cast<std::size_t>(hand_collider_semantics::HandFingerSegment::Tip);
            const auto& colliders = hand.dynamicTwinTargets();
            const float maxDepth = candidate.meshExtents.maxExtentGameUnits * 1.73205081f;
            const char* failureStage = "mesh-span";
            auto measure = [&](pinch::ClosureSample sample) {
                if (!sample.fingers.valid)
                    return pinch::ClosureSample{};
                if (!measurePinchSpanGameUnits(localTriangles, objectWorld, candidate.surfaceHit.position,
                        sample.fingers.axis, maxDepth, sample.thicknessGameUnits)) {
                    failureStage = "mesh-span";
                    return pinch::ClosureSample{};
                }
                return sample;
            };
            auto evaluate = [&](float opening) {
                pinch::ClosureSample sample{};
                auto poseConfig = config;
                poseConfig.thumbIndexMaxOpenValue = opening;
                sample.pose = pinch::buildStablePinchFingerPose(poseConfig, g_rockConfig.rockGrabFingerMinValue);
                const auto handPose =
                    frik_visual_authority::makeHandPoseDataFromJointValues(sample.pose.jointValues.data());
                frik_visual_authority::FingerLocalTransformOverride locals{};
                if (!frik_visual_authority::getHandPoseLocalTransformsForPose(handFromBool(isLeft), handPose,
                        &locals) ||
                    (locals.enabledMask & 0x003Fu) != 0x003Fu) {
                    failureStage = "provider-pose";
                    return sample;
                }
                std::array<RE::NiPoint3, 2> tips{};
                for (std::size_t finger = 0; finger < tips.size(); ++finger) {
                    RE::NiTransform joint = handWorld;
                    hand_bone_collider_geometry_math::BoneColliderFrameInput<RE::NiTransform, RE::NiPoint3> tipInput{};
                    tipInput.extrapolateFromPrevious = true;
                    tipInput.extrapolateAlongStartBoneAxis = true;
                    for (std::size_t segment = 0; segment < 3; ++segment) {
                        const auto& local = locals.localTransforms[finger * 3 + segment];
                        if (!grab_authority_frame_math::isFiniteTransform(local)) {
                            failureStage = "joint-transform";
                            return pinch::ClosureSample{};
                        }
                        joint = transform_math::composeTransforms(joint, local);
                        if (segment == 1)
                            tipInput.previous = joint;
                        if (segment == 2)
                            tipInput.start = joint;
                    }
                    const auto tipFrame = hand_bone_collider_geometry_math::buildSegmentColliderFrame(tipInput);
                    const auto& collider = colliders.fingers[finger][tipSegment];
                    if (!tipFrame.valid || !collider.valid ||
                        !pinch::colliderTipEndpoint(tipFrame.transform.translate, tipFrame.xAxis, collider.length,
                            collider.convexRadius, tips[finger])) {
                        failureStage = "tip-collider";
                        return pinch::ClosureSample{};
                    }
                }
                sample.fingers = pinch::makeFingerFrame(tips[0], tips[1]);
                return measure(sample);
            };
            if (g_rockConfig.rockGrabMeshFingerPoseEnabled) {
                // Thumb and index open as far as the object is thick, up to the
                // fully open pose; a cap here left a 0.7 gu gap at most, so the
                // pads closed through anything thicker than paper.
                candidate.closure = pinch::solveClosure(g_rockConfig.rockGrabFingerMinValue, 1.0f, evaluate);
            } else {
                // With automatic finger posing disabled, only the live tips
                // can define the seat; no uncommanded closing pose is predicted.
                pinch::ClosureSample live{};
                live.fingers = pinch::makeFingerFrame(candidate.thumbPadWorld, candidate.indexPadWorld);
                candidate.closure.sample = measure(live);
                candidate.closure.valid = candidate.closure.sample.fingers.valid;
                candidate.closure.bracketed = true;
            }
            if (!candidate.closure.valid) {
                ROCK_LOG_SAMPLE_WARN(Hand, g_rockConfig.rockLogSampleMilliseconds,
                    "{} pinch closure unavailable: stage={}", isLeft ? "Left" : "Right", failureStage);
                return false;
            }
            const auto& solved = candidate.closure.sample;
            candidate.thumbPadWorld = solved.fingers.thumbTip;
            candidate.indexPadWorld = solved.fingers.indexTip;
            candidate.pinchPocketWorld = solved.fingers.center;
            candidate.pinchAxisWorld = solved.fingers.axis;
            candidate.thumbIndexGapGameUnits = solved.fingers.gapGameUnits;
            candidate.pinchDetectionDirectionWorld = pinch::detectionDirection(solved.fingers,
                transformHandspaceDirection(handWorld, config.detectionDirectionHandspace, isLeft),
                config.detectionAxisBlend);
            if (!candidate.closure.bracketed) {
                ROCK_LOG_SAMPLE_WARN(Hand, g_rockConfig.rockLogSampleMilliseconds,
                    "{} pinch closure reached curl limit: opening={:.3f} range={:.3f}/1.000 gap={:.3f} "
                    "thickness={:.3f} error={:.3f}",
                    isLeft ? "Left" : "Right", solved.opening, g_rockConfig.rockGrabFingerMinValue,
                    solved.fingers.gapGameUnits, solved.thicknessGameUnits, candidate.closure.gapErrorGameUnits);
            }
            return true;
        }
    }

    GrabAttemptResult Hand::grabSelectedObject(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform,
        float tau, float damping, float maxForce, float proportionalRecovery, float constantRecovery,
        const GrabSharedObjectContext& sharedContext, const AuthoredWeaponGripPose* transferPose)
    {
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::GrabAcquisition);
        ValidatedGrabSelection validatedSelection{};
        if (!validateSelectedGrab(world, sharedContext, validatedSelection)) {
            return GrabAttemptResult::Rejected;
        }

        const auto& sel = _currentSelection;
        const auto selectedRef = validatedSelection.retainedRef;
        const bool joiningPeerHeldObject = validatedSelection.joiningPeerHeldObject;
        if (joiningPeerHeldObject)
            performance_profiler::addCounter(performance_profiler::Counter::GrabAcquisitionPeerHeld);
        if (sel.equippedWeaponTransfer)
            performance_profiler::addCounter(performance_profiler::Counter::GrabAcquisitionEquippedTransfer);
        const bool grabbedFromPullCatch = validatedSelection.grabbedFromPullCatch;
        const bool looseWeaponGrab = validatedSelection.looseWeaponGrab;
        const bool handPocketOnlyGrab = validatedSelection.handPocketOnlyGrab;
        saved_grab_offset::HandOffset savedGrabOffset{};
        bool hasSavedGrabOffset = false;

        auto objectBodyId = validatedSelection.bodyId;
        auto* rootNode = validatedSelection.rootNode;
        auto* bhkWorld = validatedSelection.bhkWorld;
        std::string objName = std::move(validatedSelection.objectName);

        const char* motionTypeStr = "UNKNOWN";
        std::uint16_t selectedOriginalMotionPropsId = 1;
        {
            auto* body = havok_runtime::getBody(world, objectBodyId);
            auto* objMotion = havok_runtime::getBodyMotion(world, objectBodyId);
            if (body && objMotion) {
                std::uint32_t bodyFlags = body->flags;
                std::uint8_t bodyMotionPropsId = static_cast<std::uint8_t>(body->motionPropertiesId);
                std::uint16_t motionPropsId = selectedOriginalMotionPropsId;
                if (havok_runtime::tryReadMotionPropertiesId(objMotion, motionPropsId)) {
                    selectedOriginalMotionPropsId = motionPropsId;
                }

                havok_runtime::MotionVelocityCaps velocityCaps{};
                const bool hasVelocityCaps = havok_runtime::tryReadMotionVelocityCaps(objMotion, velocityCaps);
                const float maxLinVel = hasVelocityCaps ? velocityCaps.maxLinearVelocity : 0.0f;
                const float maxAngVel = hasVelocityCaps ? velocityCaps.maxAngularVelocity : 0.0f;

                bool isDynamicInit = (bodyFlags & 0x2) != 0;
                bool isKeyframed = (bodyFlags & 0x4) != 0;

                switch (motionPropsId & 0xFF) {
                case 0:
                    motionTypeStr = "STATIC";
                    break;
                case 1:
                    motionTypeStr = "DYNAMIC";
                    break;
                case 2:
                    motionTypeStr = "KEYFRAMED";
                    break;
                default:
                    motionTypeStr = "OTHER";
                    break;
                }

                ROCK_LOG_DEBUG(Hand,
                    "{} hand GRAB MOTION: body={} motionPropsId={} ({}) "
                    "bodyFlags=0x{:08X} dynInit={} keyfr={} bodyPropsId={} "
                    "maxLinVel={:.1f} maxAngVel={:.1f}",
                    handName(), objectBodyId.value, motionPropsId, motionTypeStr, bodyFlags, isDynamicInit, isKeyframed,
                    bodyMotionPropsId, maxLinVel, maxAngVel);

                {
                    static bool libraryDumped = false;
                    if (!libraryDumped) {
                        libraryDumped = true;
                        havok_runtime::MotionPropertiesLibrarySnapshot motionProperties{};
                        if (havok_runtime::snapshotMotionPropertiesLibrary(world, motionProperties)) {
                            ROCK_LOG_TRACE(Hand, "Motion properties library: {} entries, stride=0x40",
                                motionProperties.count);
                            for (std::uint32_t i = 0; i < motionProperties.copiedCount; i++) {
                                const auto& record = motionProperties.records[i];
                                const auto* f = record.values;
                                const auto* u = record.words;
                                ROCK_LOG_TRACE(Hand,
                                    "  [{}] +00: {:.4f} {:.4f} {:.4f} {:.4f} | "
                                    "+10: {:.4f} {:.4f} {:.4f} {:.4f} | "
                                    "+20: {:.4f} {:.4f} {:.4f} {:.4f} | "
                                    "+30: {:.4f} {:.4f} {:.4f} {:.4f}",
                                    i, f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11], f[12],
                                    f[13], f[14], f[15]);
                                ROCK_LOG_TRACE(Hand,
                                    "  [{}] hex: {:08X} {:08X} {:08X} {:08X} | "
                                    "{:08X} {:08X} {:08X} {:08X}",
                                    i, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
                            }
                        }
                    }
                }
            }
        }

        const std::uint64_t grabTraceId = nextGrabTraceId();

        GrabBodyPreparation bodyPreparation{};
        prepareSelectedGrabBodies(world, sharedContext, validatedSelection, bodyPreparation);
        // Only the unconstrained root body moves, so this is also safe when the
        // peer hand already holds the object; both hands then carry it.
        (void)restoreCountBasedAmmoAssembly(world, count_based_ammo::resolve(sel.refr),
            bodyPreparation.preparedBodySet, "grab");
        auto& activeLifecycle = bodyPreparation.activeLifecycle;
        const bool consumedPullPrepLifecycle = bodyPreparation.consumedPullPrepLifecycle;
        const auto& beforePrepBodySet = bodyPreparation.beforePrepBodySet;
        const bool beforePrepScanCacheHit = bodyPreparation.beforePrepScanCacheHit;
        const bool motionConverted = bodyPreparation.motionConverted;
        const bool collisionEnabled = bodyPreparation.collisionEnabled;
        auto& preparedBodySet = bodyPreparation.preparedBodySet;
        const bool preparedScanCacheHit = bodyPreparation.preparedScanCacheHit;
        const bool preparedBodySetPostPrepComplete = bodyPreparation.preparedBodySetPostPrepComplete;
        const auto& scanOptions = bodyPreparation.scanOptions;

        auto restoreFailedGrabPrep = [&]() {
            if (!joiningPeerHeldObject) {
                if (consumedPullPrepLifecycle) {
                    const auto releaseRestorePolicy =
                        active_grab_body_lifecycle::releaseRestorePolicyForTargetKind(sel.targetKind);
                    const auto releasePlan = activeLifecycle.restorePlanForRelease(releaseRestorePolicy, sel.targetKind,
                        active_grab_body_lifecycle::BodyReleaseIntent::PhysicalDrop);
                    restoreActiveGrabLifecycle(world, activeLifecycle, releasePlan, objectBodyId.value, handName(),
                        "failed-pull-catch-setup-physical-drop");
                    if (activeLifecycle.hasIncompleteNativeScan()) {
                        if (active_grab_body_lifecycle::shouldSkipIncompleteScanRootRestore(releasePlan,
                                selectedOriginalMotionPropsId)) {
                            ROCK_LOG_DEBUG(Hand,
                                "{} hand failed pull-catch setup: skipped recursive root restore for converted "
                                "loose-object physical drop root='{}' motionProps={} preservedMotion={}",
                                handName(), nodeDebugName(rootNode), selectedOriginalMotionPropsId,
                                releasePlan.preservedConvertedMotionCount);
                        } else {
                            restoreIncompleteActivePrepRoot(rootNode, selectedOriginalMotionPropsId, handName(),
                                "failed-pull-catch-setup-incomplete-scan");
                        }
                    }
                    if (world && objectBodyId.value != INVALID_BODY_ID) {
                        const auto releaseActivation =
                            activateHeldObjectBodySet(world, objectBodyId.value, _pulledBodyIds);
                        if (releaseActivation.failedActivationCount > 0) {
                            ROCK_LOG_WARN(Hand,
                                "{} hand failed pull-catch setup activation incomplete: primaryBody={} bodies={} "
                                "activated={} failed={}",
                                handName(), objectBodyId.value, releaseActivation.bodyCount,
                                releaseActivation.activatedCount, releaseActivation.failedActivationCount);
                        }
                    }
                } else {
                    restoreActiveGrabLifecycle(world, activeLifecycle, activeLifecycle.restorePlanForFailure(),
                        objectBodyId.value, handName(), "failed-setup");
                    if (activeLifecycle.hasIncompleteNativeScan()) {
                        restoreIncompleteActivePrepRoot(rootNode, selectedOriginalMotionPropsId, handName(),
                            "failed-setup-incomplete-scan");
                    }
                }
            }
            /*
             * Some setup failures occur after body resolution has staged a saved
             * object state but before ROCK owns a live grab constraint. Clear it
             * here so a rejected strict-authority grab cannot look held or block
             * later reset cleanup.
             */
            _savedObjectState.clear();
        };
        GrabPreparationTransaction grabPreparationTransaction{ restoreFailedGrabPrep };

        GrabProxyPreparation proxyPreparation{};
        if (!prepareGrabProxyAuthority(world, handWorldTransform, proxyPreparation)) {
            ROCK_LOG_ERROR(Hand,
                "{} hand GRAB FAILED: live palm anchor frame unavailable before grab evidence capture bodyId={} "
                "handBody={} source={} formID={:08X}",
                handName(), objectBodyId.value, _handBody.isValid() ? _handBody.getBodyId().value : INVALID_BODY_ID,
                proxyPreparation.proxyFrameSourceAtGrab, sel.refr ? sel.refr->GetFormID() : 0);
            _grabFrame.clear();
            _heldBodyIds.clear();
            _heldBodyIdsCount.store(0, std::memory_order_release);
            grabPreparationTransaction.rollback();
            clearGrabExternalHandWorldTransform(_isLeft);
            return GrabAttemptResult::Rejected;
        }
        const auto& grabAuthorityPivotAWorld = proxyPreparation.grabAuthorityPivotAWorld;
        const auto& palmPocketPivotAWorld = proxyPreparation.palmPocketPivotAWorld;
        const auto& grabPalmBasisDelta = proxyPreparation.palmBasisDelta;
        const char* proxyFrameSourceAtGrab = proxyPreparation.proxyFrameSourceAtGrab;
        const float palmPocketToProxyDeltaGameUnits = proxyPreparation.palmPocketToProxyDeltaGameUnits;
        /*
         * Live proxy motion and close-grab pocket acquisition both resolve the
         * configured seat offset in generated/proxy local space. The generated
         * collision body path and hidden proxy seat now agree at startup.
         */
        if (grabPalmBasisDelta.rotationDegrees > kGrabFrameMismatchRawProxyRotationWarnDegrees) {
            ROCK_LOG_SAMPLE_WARN(Hand, g_rockConfig.rockLogSampleMilliseconds,
                "{} PROXY GRAB PALM BASIS MISMATCH: ref='{}' formID={:08X} source={} rawToProxy={:.1f}deg "
                "axisDeg=({:.1f},{:.1f},{:.1f}) determinant=({:.3f},{:.3f}) proxyPivot=({:.1f},{:.1f},{:.1f}) "
                "palmPocketPivot=({:.1f},{:.1f},{:.1f}) pocketProxyDelta={:.2f}gu phase={}",
                handName(), objName, sel.refr ? sel.refr->GetFormID() : 0, proxyFrameSourceAtGrab,
                grabPalmBasisDelta.rotationDegrees, grabPalmBasisDelta.xAxisDegrees, grabPalmBasisDelta.yAxisDegrees,
                grabPalmBasisDelta.zAxisDegrees, grabPalmBasisDelta.rawDeterminant, grabPalmBasisDelta.proxyDeterminant,
                grabAuthorityPivotAWorld.x, grabAuthorityPivotAWorld.y, grabAuthorityPivotAWorld.z,
                palmPocketPivotAWorld.x, palmPocketPivotAWorld.y, palmPocketPivotAWorld.z,
                palmPocketToProxyDeltaGameUnits, grab_three_phase::phaseName(_grabAcquisitionPhase));
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand object-tree prep: ref='{}' formID={:08X} beforeBodies={} afterBodies={} accepted={} rejected={} "
            "seedBody={} seeded={} scanSource={}/{} preparedComplete={} cachedBodyIds={} cacheHits={} scanFailures={} "
            "invalidSystems={} benignSkips={} foreignSkips={} unresolvedAccepted={} unresolvedSkips={} "
            "latePrepared={} incompleteScan={} collisionObjects={} visitedNodes={} setMotion={} enableCollision={} "
            "sharedPeer={} "
            "proxyPivot=({:.1f},{:.1f},{:.1f}) palmPocketPivot=({:.1f},{:.1f},{:.1f}) pocketProxyDelta={:.2f} "
            "palmBasis={:.1f}deg axisDeg=({:.1f},{:.1f},{:.1f}) determinant=({:.3f},{:.3f})",
            handName(), objName, sel.refr->GetFormID(), beforePrepBodySet.records.size(),
            preparedBodySet.records.size(), preparedBodySet.acceptedCount(), preparedBodySet.rejectedCount(),
            scanOptions.seedBodyId, preparedBodySet.diagnostics.seedBodiesAdded,
            beforePrepScanCacheHit ? "cache" : "direct", preparedScanCacheHit ? "cache" : "direct",
            preparedBodySetPostPrepComplete ? "yes" : "no", preparedBodySet.diagnostics.cachedBodyIds,
            preparedBodySet.diagnostics.cachedScanHits, preparedBodySet.diagnostics.scanFailures,
            preparedBodySet.diagnostics.invalidPhysicsSystems, preparedBodySet.diagnostics.benignScanSkips,
            preparedBodySet.diagnostics.foreignRefBodySkips, preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
            preparedBodySet.diagnostics.unresolvedRefBodySkips, activeLifecycle.latePreparedBodyCount(),
            activeLifecycle.hasIncompleteNativeScan() ? "yes" : "no", preparedBodySet.diagnostics.collisionObjects,
            preparedBodySet.diagnostics.visitedNodes, motionConverted ? "ok" : "failed",
            collisionEnabled ? "ok" : "failed", joiningPeerHeldObject ? "yes" : "no", grabAuthorityPivotAWorld.x,
            grabAuthorityPivotAWorld.y, grabAuthorityPivotAWorld.z, palmPocketPivotAWorld.x, palmPocketPivotAWorld.y,
            palmPocketPivotAWorld.z, palmPocketToProxyDeltaGameUnits, grabPalmBasisDelta.rotationDegrees,
            grabPalmBasisDelta.xAxisDegrees, grabPalmBasisDelta.yAxisDegrees, grabPalmBasisDelta.zAxisDegrees,
            grabPalmBasisDelta.rawDeterminant, grabPalmBasisDelta.proxyDeterminant);

        GrabMeshCaptureSetup meshCaptureSetup{};
        if (!prepareGrabMeshCapture(handWorldTransform, validatedSelection, objName, meshCaptureSetup)) {
            grabPreparationTransaction.rollback();
            clearGrabExternalHandWorldTransform(_isLeft);
            return GrabAttemptResult::Rejected;
        }
        auto* collidableNode = meshCaptureSetup.collidableNode;
        auto* meshSourceNode = meshCaptureSetup.meshSourceNode;
        RE::NiTransform objectWorldTransform = meshCaptureSetup.objectWorldTransform;

        namespace authoredPolicy = loose_weapon_authored_grab_policy;
        const auto acquisitionSource =
            authoredPolicy::acquisitionSource(grabbedFromPullCatch, sel.forcedArrival, transferPose != nullptr);
        const bool authoredWeapon = looseWeaponGrab && !isThrowableLooseWeapon(selectedLooseWeaponForm(sel));
        loose_weapon_grip_zone::NearGrab preparedAuthoredGrip{};
        const auto prepareAuthoredGrip = [&](bool arrival) {
            performance_profiler::ScopedTimer preparationTimer(performance_profiler::Scope::GrabAuthoredPreparation);
            preparedAuthoredGrip = {};
            (void)loose_weapon_grip_zone::tryResolveNearGrab(_isLeft, sel.refr, preparedAuthoredGrip,
                joiningPeerHeldObject);
            const auto role = authoredPolicy::surfaceIndependentAcquisitionRole(arrival, transferPose != nullptr,
                transferPose ? transferPose->role : authoredPolicy::Role::None, preparedAuthoredGrip.arrangement,
                joiningPeerHeldObject, preparedAuthoredGrip.role,
                authoredPolicy::freeWeaponRole(acquisitionSource, preparedAuthoredGrip.nearestRole));
            ROCK_LOG_SAMPLE_INFO(Hand, 1000,
                "{} loose weapon arrival role ref={:08X} arrival={} peerHolding={} transfer={} arrangement={} "
                "zoneRole={} nearestRole={} selectedRole={}",
                handName(), sel.refr ? sel.refr->GetFormID() : 0, arrival, joiningPeerHeldObject,
                transferPose != nullptr, static_cast<unsigned>(preparedAuthoredGrip.arrangement),
                static_cast<unsigned>(preparedAuthoredGrip.role),
                static_cast<unsigned>(preparedAuthoredGrip.nearestRole), static_cast<unsigned>(role));
            if (role == authoredPolicy::Role::None)
                return false;
            preparedAuthoredGrip.role = role;
            if (transferPose)
                preparedAuthoredGrip.pose = *transferPose;
            else
                (void)loose_weapon_grip_zone::tryResolveAuthoredGrabPose(_isLeft, sel.refr, role,
                    preparedAuthoredGrip.pose);
            const auto* weapon = selectedLooseWeaponForm(sel);
            return preparedAuthoredGrip.handWorldValid && preparedAuthoredGrip.pose.valid() && weapon &&
                preparedAuthoredGrip.pose.weaponFormId == weapon->formID &&
                preparedAuthoredGrip.pose.isLeft == _isLeft && preparedAuthoredGrip.pose.role == role;
        };
        const bool authoredArrivalRequested = authoredWeapon && !handPocketOnlyGrab &&
            authoredPolicy::useAuthoredArrival(acquisitionSource, true, false, sel.pinchCloseSelectionFallback);
        const bool arrivalPoseAvailable = authoredArrivalRequested && prepareAuthoredGrip(true);
        bool authoredSeatPrepared = authoredPolicy::useAuthoredArrival(acquisitionSource, arrivalPoseAvailable,
            !authoredWeapon || handPocketOnlyGrab, sel.pinchCloseSelectionFallback);
        if (authoredArrivalRequested && !authoredSeatPrepared) {
            performance_profiler::addCounter(performance_profiler::Counter::GrabAuthoredArrivalDynamicFallback);
        }
        const auto* selectedBodyRecord = preparedBodySet.findRecord(objectBodyId.value);
        const auto& bodyScan = preparedBodySet.diagnostics;
        const bool authoredBodyOwnerVerified = authoredSeatPrepared && preparedBodySet.rootRef == sel.refr &&
            preparedBodySet.rootNode == rootNode && sel.refr->Get3D() == rootNode &&
            preparedBodySet.records.size() == 1 && selectedBodyRecord && selectedBodyRecord->accepted &&
            selectedBodyRecord->refResolutionKnown && selectedBodyRecord->resolvedRef == sel.refr &&
            nodeIsOrDescendsFrom(rootNode, selectedBodyRecord->owningNode) &&
            !activeLifecycle.hasIncompleteNativeScan() && !bodyScan.scanFailures && !bodyScan.invalidPhysicsSystems &&
            !bodyScan.depthLimitSkips && !bodyScan.foreignRefBodySkips && !bodyScan.unresolvedRefBodySkips;
        if (authoredBodyOwnerVerified) {
            performance_profiler::addCounter(performance_profiler::Counter::GrabAuthoredPositionsOnly);
        }
        const auto seedAuthoredPoint = [&](GrabSurfaceEvidence& surface) {
            const auto seatLocal =
                computeGrabPalmPointWorldFromHandBasis(preparedAuthoredGrip.pose.handWeaponLocal, _isLeft);
            surface.gripPoint = transform_math::localPointToWorld(rootNode->world, seatLocal);
            surface.pointMode = "authoredWeaponSeat";
            surface.pointAuthoritySource = GrabPivotAuthoritySource::LooseWeaponPrimaryAttach;
            surface.fallbackReason = "authoredArrival";
            // No mesh hit is fabricated: the authored pose is the position authority.
        };

        // Hand-owned buffers, reset by extractGrabMeshEvidence below.
        auto& meshExtraction = _grabMeshExtraction;
        auto& grabMeshTriangles = meshExtraction.meshTriangles;
        auto& grabSurfaceTriangles = meshExtraction.surfaceTriangles;
        auto& grabLocalMeshTriangles = meshExtraction.objectLocalTriangles;
        std::vector<GrabLocalTriangle> grabFingerPoseLocalMeshTriangles;
        const bool meshContactOnly = g_rockConfig.rockGrabMeshContactOnly;
        extractGrabMeshEvidence(world, objectBodyId, rootNode, collidableNode, meshSourceNode, handPocketOnlyGrab,
            !authoredBodyOwnerVerified, meshExtraction);
        meshSourceNode = meshExtraction.meshSourceNode;
        if (grab_target::requiresNativeBodyOwnership(sel.targetKind)) {
            const auto selectedComponent = ragdoll::readComponent(world, sel.bodyId.value);
            if (!selectedComponent.valid) {
                ROCK_LOG_WARN(Hand, "{} ragdoll surface rejected: selected body={} topology={}", handName(),
                    sel.bodyId.value, selectedComponent.reason);
                grabPreparationTransaction.rollback();
                clearGrabExternalHandWorldTransform(_isLeft);
                return GrabAttemptResult::Rejected;
            }
            std::unordered_map<RE::NiAVObject*, RE::NiAVObject*> ownerCache;
            auto resolveOwner = [&](RE::NiAVObject* node) {
                if (auto it = ownerCache.find(node); it != ownerCache.end())
                    return it->second;
                const auto* record = preparedBodySet.findAcceptedRagdollOwner(node);
                auto* owner = record && selectedComponent.contains(record->bodyId) ? record->owningNode : nullptr;
                ownerCache.emplace(node, owner);
                return owner;
            };
            auto mapSurfaceOwners = [&] {
                // Triangles share their shape's influence table; map each
                // table's bones to ragdoll owners once.
                std::vector<const GrabSurfaceSkin*> mappedSkins;
                for (auto& triangle : grabSurfaceTriangles) {
                    if (triangle.skin &&
                        std::find(mappedSkins.begin(), mappedSkins.end(), triangle.skin.get()) == mappedSkins.end()) {
                        for (auto& vertex : triangle.skin->vertices)
                            for (auto& influence : vertex) {
                                influence.bone = resolveOwner(influence.bone);
                            }
                        mappedSkins.push_back(triangle.skin.get());
                    }
                    if (triangle.sourceKind == GrabSurfaceSourceKind::Skinned) {
                        bool completeOwners = true;
                        for (std::size_t corner = 0; triangle.skin && corner < 3; ++corner) {
                            for (const auto& influence : triangle.vertexInfluences(corner)) {
                                if (influence.weight > 0.0f && !influence.bone)
                                    completeOwners = false;
                            }
                        }
                        triangle.hasSkinInfluences = completeOwners && hasAnySkinInfluence(triangle);
                        if (!triangle.hasSkinInfluences)
                            triangle.sourceNode = nullptr;
                    } else
                        triangle.sourceNode = resolveOwner(triangle.sourceNode);
                }
                std::erase_if(grabSurfaceTriangles,
                    [](const auto& triangle) { return !triangle.hasSkinInfluences && !triangle.sourceNode; });
            };
            mapSurfaceOwners();
            if (grabSurfaceTriangles.empty() && sel.targetKind == grab_target::Kind::DetachedBody) {
                // The actor's 3D is not necessarily the detached part's render root.
                // Choose the nearest bounded ancestor with surfaces owned by this
                // exact native component. Never accept the actor's unrelated mesh.
                auto* candidate = sel.hitNode;
                for (int level = 0; candidate && level < 8 && grabSurfaceTriangles.empty();
                    ++level, candidate = candidate->parent) {
                    if (candidate == meshSourceNode)
                        continue;
                    grabMeshTriangles.clear();
                    const auto extraction = extractBoundedSurfaceTriangles(candidate, grabMeshTriangles,
                        grabSurfaceTriangles, (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth), 64,
                        kMaxMeshExtractionTriangles, true);
                    if (extraction.nodeBudgetExceeded || extraction.shapeBudgetExceeded ||
                        extraction.triangleBudgetExceeded) {
                        grabSurfaceTriangles.clear();
                        ROCK_LOG_WARN(Hand, "{} detached part render search exceeded budget at '{}'", handName(),
                            nodeDebugName(candidate));
                        break;
                    }
                    mapSurfaceOwners();
                    if (!grabSurfaceTriangles.empty()) {
                        meshSourceNode = meshExtraction.meshSourceNode = meshCaptureSetup.meshSourceNode = candidate;
                        ROCK_LOG_INFO(Hand, "{} detached part render root: '{}' body={} triangles={}", handName(),
                            nodeDebugName(candidate), sel.bodyId.value, grabSurfaceTriangles.size());
                    }
                }
            }
            grabMeshTriangles.clear();
            for (const auto& triangle : grabSurfaceTriangles)
                grabMeshTriangles.push_back(triangle.triangle);
        }
        GrabSurfaceEvidence surfaceEvidence{};
        if (authoredBodyOwnerVerified) {
            seedAuthoredPoint(surfaceEvidence);
        } else {
            meshExtraction.queryIndex.build(grabSurfaceTriangles);
            resolveGrabSurfaceEvidence(validatedSelection, proxyPreparation, meshCaptureSetup, meshExtraction,
                meshContactOnly, surfaceEvidence);
            if (authoredSeatPrepared && !surfaceEvidence.meshGrabFound)
                seedAuthoredPoint(surfaceEvidence);
        }
        auto& meshGrabFound = surfaceEvidence.meshGrabFound;
        auto& grabSurfaceHit = surfaceEvidence.surfaceHit;
        auto*& surfaceOwnerNode = surfaceEvidence.surfaceOwnerNode;
        const bool hasMeshSurfaceContact =
            meshGrabFound && grabSurfaceHit.valid && grabSurfaceHit.sourceKind != GrabSurfaceSourceKind::CollisionQuery;
        GrabBodyResolution bodyResolution{};
        resolveGrabBodyAndContactPolicy(validatedSelection, beforePrepBodySet, preparedBodySet, activeLifecycle,
            proxyPreparation, surfaceEvidence, bodyResolution);
        const auto& primaryChoice = bodyResolution.primaryChoice;
        bool& surfaceOwnerMatchesResolvedBody = bodyResolution.surfaceOwnerMatchesResolvedBody;
        const auto& mechanicalScope = bodyResolution.mechanicalScope;
        const bool relaxedArticulatedAuthority = bodyResolution.relaxedArticulatedAuthority;

        if (primaryChoice.bodyId == INVALID_BODY_ID) {
            const auto* rejectedBody = diagnosticRejectedBodyRecord(preparedBodySet, sel.bodyId.value);
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: no accepted dynamic body after recursive prep for '{}' formID={:08X} "
                "visitedNodes={} collisionObjects={} seeded={} scanFailures={} invalidSystems={} benignSkips={} "
                "foreignSkips={} unresolvedAccepted={} unresolvedSkips={} rejectReason={} rejectBody={} rejectLayer={} "
                "rejectMotion={} rejectFlags=0x{:08X} rejectMotionProps={} latePrepared={} incompleteScan={}",
                handName(), objName, sel.refr->GetFormID(), preparedBodySet.diagnostics.visitedNodes,
                preparedBodySet.diagnostics.collisionObjects, preparedBodySet.diagnostics.seedBodiesAdded,
                preparedBodySet.diagnostics.scanFailures, preparedBodySet.diagnostics.invalidPhysicsSystems,
                preparedBodySet.diagnostics.benignScanSkips, preparedBodySet.diagnostics.foreignRefBodySkips,
                preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
                preparedBodySet.diagnostics.unresolvedRefBodySkips,
                rejectedBody ? physics_body_classifier::rejectReasonName(rejectedBody->rejectReason) : "none",
                rejectedBody ? rejectedBody->bodyId : INVALID_BODY_ID, rejectedBody ? rejectedBody->collisionLayer : 0,
                rejectedBody ? bodyMotionTypeName(rejectedBody->motionType) : "none",
                rejectedBody ? rejectedBody->bodyFlags : 0, rejectedBody ? rejectedBody->motionPropertiesId : 0,
                activeLifecycle.latePreparedBodyCount(), activeLifecycle.hasIncompleteNativeScan() ? "yes" : "no");
            grabPreparationTransaction.rollback();
            clearGrabExternalHandWorldTransform(_isLeft);
            return GrabAttemptResult::Rejected;
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand MECHANICAL SCOPE: targetKind={} kind={} reason={} primaryBody={} bodies={} accepted={} motions={} "
            "fixedRejects={} incomplete={} driveMode={} linearScope={} angularScope={} massScope={}",
            handName(), grab_target::name(sel.targetKind),
            mechanical_connected_body_set::scopeKindName(mechanicalScope.kind), mechanicalScope.reason,
            mechanicalScope.primaryBodyId, mechanicalScope.committedBodyIds.size(), mechanicalScope.acceptedBodyCount,
            mechanicalScope.uniqueMotionCount, mechanicalScope.rejectedFixedOrNonDynamicCount,
            mechanicalScope.incompleteDiscovery ? "yes" : "no",
            held_object_drive_policy::modeName(mechanicalScope.driveDecision.mode),
            mechanicalScope.driveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
            mechanicalScope.driveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly",
            mechanicalScope.driveDecision.includeConnectedMass ? "bodySet" : "primaryOnly");

        if (hasMeshSurfaceContact && !surfaceOwnerMatchesResolvedBody && !relaxedArticulatedAuthority) {
            // A mesh point on a sibling branch still lies on this object; hold it
            // there instead of refusing the grab.
            ROCK_LOG_SAMPLE_DEBUG(Hand, g_rockConfig.rockLogSampleMilliseconds,
                "{} hand GRAB mesh owner differs from resolved body for '{}' formID={:08X}; selectedBody={} "
                "resolvedBody={} sourceNode='{}'",
                handName(), objName, sel.refr->GetFormID(), sel.bodyId.value, primaryChoice.bodyId,
                nodeDebugName(surfaceOwnerNode));
        }

        const bool visualMeshPivotAvailable = hasMeshSurfaceContact;
        const RE::NiPoint3 canonicalPivotNormalWorld = grabSurfaceHit.valid ? grabSurfaceHit.normal : RE::NiPoint3{};

        if (grab_target::requiresNativeBodyOwnership(sel.targetKind)) {
            auto* owner = preparedBodySet.findRecord(primaryChoice.bodyId)->owningNode;
            std::erase_if(grabSurfaceTriangles, [owner](const auto& triangle) {
                if (!triangle.hasSkinInfluences)
                    return triangle.sourceNode != owner;
                for (std::size_t corner = 0; corner < 3; ++corner) {
                    for (const auto& influence : triangle.vertexInfluences(corner)) {
                        if (influence.bone == owner && influence.weight > 0.0f)
                            return false;
                    }
                }
                return true;
            });
            grabMeshTriangles.clear();
            for (const auto& triangle : grabSurfaceTriangles)
                grabMeshTriangles.push_back(triangle.triangle);
            meshExtraction.queryIndex.build(grabSurfaceTriangles);
        }
        ResolvedGrabBodyCapture resolvedBodyCapture{};
        if (!captureResolvedGrabBody(world, meshCaptureSetup, bodyResolution, beforePrepBodySet, grabMeshTriangles,
                selectedRef, selectedOriginalMotionPropsId, objName, resolvedBodyCapture, grabLocalMeshTriangles)) {
            grabPreparationTransaction.rollback();
            clearGrabExternalHandWorldTransform(_isLeft);
            return GrabAttemptResult::Rejected;
        }
        objectBodyId = resolvedBodyCapture.bodyId;
        collidableNode = resolvedBodyCapture.collidableNode;
        objectWorldTransform = resolvedBodyCapture.objectWorldTransform;
        if (grab_target::requiresNativeBodyOwnership(sel.targetKind)) {
            // Skin weights may resolve to a different bone than the cast hit.
            // Later frame/constraint capture reads this setup, so it must use
            // the resolved owner's frame just like the local mesh cache above.
            meshCaptureSetup.collidableNode = collidableNode;
            meshCaptureSetup.objectWorldTransform = objectWorldTransform;
            ROCK_LOG_SAMPLE_DEBUG(Hand, 2000,
                "{} ragdoll resolved frame: selectedBody={} resolvedBody={} selectedNode='{}' captureNode='{}' "
                "surfaceNode='{}'",
                handName(), sel.bodyId.value, objectBodyId.value, nodeDebugName(sel.hitNode),
                nodeDebugName(collidableNode), nodeDebugName(surfaceEvidence.surfaceOwnerNode));
        }

        if (authoredWeapon) {
            const char* preparationReason = authoredSeatPrepared
                ? "authored-arrival-ready"
                : (authoredArrivalRequested ? "authored-unavailable-dynamic" : "surface-evidence-required");
            if (!authoredSeatPrepared && !authoredArrivalRequested && !handPocketOnlyGrab &&
                !sel.pinchCloseSelectionFallback && sel.refr->Get3D() == rootNode &&
                preparedBodySet.acceptedCount() == 1 && !activeLifecycle.hasIncompleteNativeScan() &&
                surfaceOwnerMatchesResolvedBody && visualMeshPivotAvailable) {
                authoredSeatPrepared = prepareAuthoredGrip(false);
                if (authoredSeatPrepared)
                    preparationReason = "authored-close-seat-ready";
            }
            performance_profiler::addCounter(authoredSeatPrepared
                    ? performance_profiler::Counter::GrabAuthoredSurfaceWorkSkipped
                    : performance_profiler::Counter::GrabAuthoredSurfaceWorkRequired);
            if (performance_profiler::enabled()) {
                ROCK_LOG_SAMPLE_INFO(Hand, 1000,
                    "{} authored pickup preparation: ref={:08X} body={} source={} role={} skipSurfaceWork={} "
                    "positionsOnly={} reason={} triangles={} surfaceRecords={}",
                    handName(), sel.refr ? sel.refr->GetFormID() : 0, objectBodyId.value,
                    static_cast<unsigned>(acquisitionSource), static_cast<unsigned>(preparedAuthoredGrip.role),
                    authoredSeatPrepared, authoredBodyOwnerVerified, preparationReason, grabMeshTriangles.size(),
                    grabSurfaceTriangles.size());
            }
        }

        GrabPivotEvidence pivotEvidence{};
        resolveGrabPivotEvidence(proxyPreparation, resolvedBodyCapture, surfaceEvidence, pivotEvidence);

        const RuntimePinchPocketCandidate pinchPocketCandidate = authoredSeatPrepared
            ? RuntimePinchPocketCandidate{}
            : buildRuntimePinchPocketCandidate(sel, preparedBodySet, objectBodyId.value, objectWorldTransform,
                  grabSurfaceTriangles, meshExtraction.queryIndex, grabLocalMeshTriangles, handWorldTransform, _isLeft,
                  *this, !sel.isFarSelection && !grabbedFromPullCatch, handPocketOnlyGrab, looseWeaponGrab);
        if (pinchPocketCandidate.valid) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PINCH POCKET candidate accepted: reason={} pocket=({:.1f},{:.1f},{:.1f}) "
                "point=({:.1f},{:.1f},{:.1f}) dir=({:.2f},{:.2f},{:.2f}) gap={:.2f}gu span={:.2f}gu dist={:.2f}gu "
                "extents=({:.2f},{:.2f},{:.2f})",
                handName(), pinchPocketCandidate.decision.reason, pinchPocketCandidate.pinchPocketWorld.x,
                pinchPocketCandidate.pinchPocketWorld.y, pinchPocketCandidate.pinchPocketWorld.z,
                pinchPocketCandidate.surfaceHit.position.x, pinchPocketCandidate.surfaceHit.position.y,
                pinchPocketCandidate.surfaceHit.position.z, pinchPocketCandidate.pinchDetectionDirectionWorld.x,
                pinchPocketCandidate.pinchDetectionDirectionWorld.y,
                pinchPocketCandidate.pinchDetectionDirectionWorld.z, pinchPocketCandidate.thumbIndexGapGameUnits,
                pinchPocketCandidate.pinchSpanGameUnits, pinchPocketCandidate.pocketToSurfaceDistanceGameUnits,
                pinchPocketCandidate.meshExtents.minExtentGameUnits,
                pinchPocketCandidate.meshExtents.middleExtentGameUnits,
                pinchPocketCandidate.meshExtents.maxExtentGameUnits);
        } else if (!authoredSeatPrepared && !sel.isFarSelection && !grabbedFromPullCatch) {
            // Why this close grab seats in the palm instead; the span against the
            // gap is the between-the-fingers test (grab_pinch_pocket_policy).
            ROCK_LOG_DEBUG(Hand, "{} hand PINCH POCKET candidate rejected: reason={} gap={:.2f}gu span={:.2f}gu dist={:.2f}gu",
                handName(), pinchPocketCandidate.decision.reason, pinchPocketCandidate.thumbIndexGapGameUnits,
                pinchPocketCandidate.pinchSpanGameUnits, pinchPocketCandidate.pocketToSurfaceDistanceGameUnits);
        }

        const GrabCommitPreparationInput commitPreparationInput{
            .handWorldTransform = &handWorldTransform,
            .collidableNode = collidableNode,
            .objectBodyId = objectBodyId,
            .objectName = &objName,
            .mechanicalScope = &mechanicalScope,
            .sharedContext = &sharedContext,
            .preparedBodySet = &preparedBodySet,
            .joiningPeerHeldObject = joiningPeerHeldObject,
        };
        beginResolvedGrabCommit(commitPreparationInput);

        {
            GrabBodyFrameCaptureInput bodyFrameInput{
                .handWorldTransform = &handWorldTransform,
                .proxy = &proxyPreparation,
                .meshCapture = &meshCaptureSetup,
                .mesh = &meshExtraction,
                .surface = &surfaceEvidence,
                .pivotEvidence = &pivotEvidence,
                .bodyResolution = &bodyResolution,
                .localMeshTriangles = &grabLocalMeshTriangles,
                .rootNode = rootNode,
                .objectBodyId = objectBodyId,
                .traceId = grabTraceId,
                .objectName = &objName,
                .authoredSeatPrepared = authoredSeatPrepared,
            };
            GrabBodyFrameCapture bodyFrameCapture{};
            if (!captureGrabBodyFrame(world, bodyFrameInput, bodyFrameCapture)) {
                grabPreparationTransaction.rollback();
                if (bodyFrameCapture.clearExternalOnFailure) {
                    clearGrabExternalHandWorldTransform(_isLeft);
                }
                return GrabAttemptResult::Rejected;
            }
            grabFingerPoseLocalMeshTriangles = std::move(bodyFrameCapture.fingerPoseLocalMeshTriangles);

            const GrabRollbackAction seatRollback{
                .context = &grabPreparationTransaction,
                .invoke =
                    [](void* context) noexcept {
                        static_cast<decltype(grabPreparationTransaction)*>(context)->rollback();
                    },
            };
            GrabSeatCaptureInput seatCaptureInput{
                .handWorldTransform = &handWorldTransform,
                .proxy = &proxyPreparation,
                .meshCapture = &meshCaptureSetup,
                .mesh = &meshExtraction,
                .surface = &surfaceEvidence,
                .pivotEvidence = &pivotEvidence,
                .bodyFrame = &bodyFrameCapture,
                .pinchPocketCandidate = &pinchPocketCandidate,
                .savedGrabOffset = &savedGrabOffset,
                .hasSavedGrabOffset = &hasSavedGrabOffset,
                .rootNode = rootNode,
                .collidableNode = collidableNode,
                .objectBodyId = objectBodyId,
                .grabbedFromPullCatch = grabbedFromPullCatch,
                .looseWeaponGrab = looseWeaponGrab,
                .joiningPeerHeldObject = joiningPeerHeldObject,
                .transferPose = transferPose,
                .preparedAuthoredGrip = authoredSeatPrepared ? &preparedAuthoredGrip : nullptr,
                .canonicalPivotNormalWorld = canonicalPivotNormalWorld,
                .rollback = seatRollback,
            };
            GrabSeatCaptureResult seatCapture{};
            if (!resolveGrabSeatCapture(seatCaptureInput, seatCapture)) {
                return GrabAttemptResult::Rejected;
            }

            GrabFrozenCommitInput frozenCommitInput{
                .handWorldTransform = &handWorldTransform,
                .proxy = &proxyPreparation,
                .meshCapture = &meshCaptureSetup,
                .bodyFrame = &bodyFrameCapture,
                .seatCapture = &seatCapture,
                .surface = &surfaceEvidence,
                .objectBodyId = objectBodyId,
                .traceId = grabTraceId,
                .rollback = seatRollback,
            };
            if (!commitFrozenGrabAuthority(frozenCommitInput)) {
                return GrabAttemptResult::Rejected;
            }

            const GrabPostFreezeInput postFreezeInput{
                .handWorldTransform = &handWorldTransform,
                .meshCapture = &meshCaptureSetup,
                .mesh = &meshExtraction,
                .bodyFrame = &bodyFrameCapture,
                .surface = &surfaceEvidence,
                .objectBodyId = objectBodyId,
                .sharedContext = &sharedContext,
                .joiningPeerHeldObject = joiningPeerHeldObject,
                .looseWeaponGrab = looseWeaponGrab,
            };
            initializePostFreezeGrab(world, postFreezeInput);
        }

        const GrabRollbackAction constraintRollback{
            .context = &grabPreparationTransaction,
            .invoke =
                [](void* context) noexcept { static_cast<decltype(grabPreparationTransaction)*>(context)->rollback(); },
        };
        const GrabConstraintCommitInput constraintCommitInput{
            .handWorldTransform = &handWorldTransform,
            .proxy = &proxyPreparation,
            .surface = &surfaceEvidence,
            .bhkWorld = bhkWorld,
            .objectBodyId = objectBodyId,
            .activeLifecycle = &activeLifecycle,
            .savedGrabOffset = &savedGrabOffset,
            .hasSavedGrabOffset = hasSavedGrabOffset,
            .joiningPeerHeldObject = joiningPeerHeldObject,
            .looseWeaponGrab = looseWeaponGrab,
            .tau = tau,
            .damping = damping,
            .maxForce = maxForce,
            .proportionalRecovery = proportionalRecovery,
            .constantRecovery = constantRecovery,
            .rollback = constraintRollback,
        };
        if (!commitGrabConstraintAndPose(world, constraintCommitInput)) {
            return GrabAttemptResult::Rejected;
        }

        grabPreparationTransaction.commit();
        performance_profiler::addCounter(performance_profiler::Counter::GrabAcquisitionSucceeded);
        applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::GrabCommitSucceeded });
        clearPullRuntimeState();
        clearPullCatchIntent(grabbedFromPullCatch ? "pullCatchGrabbed" : "grabbed");

        ROCK_LOG_INFO(Hand, "{} hand grab success -> HeldInit: bodyId={}", handName(), objectBodyId.value);
        _grabFrame.equippedWeaponTransfer = sel.equippedWeaponTransfer;
        if (sel.equippedWeaponTransfer && rootNode) {
            // Seed presentation from the constraint's initial physical target.
            // The same-frame held update can then carry the measured body pose
            // onto the queued hand target instead of discarding its first sample.
            const auto& initialTarget = _grabOffsetAcquisition.active ? _grabFrame.authority.bodyWorldAtGrab
                                                                      : _grabFrame.authority.desiredBodyWorldAtGrab;
            (void)held_scene_presentation::publishTargetTransport(_isLeft, world, objectBodyId.value,
                _grabFrame.traceId, initialTarget, _grabFrame.authority.bodyWorldAtGrab, rootNode,
                _grabFrame.rootBodyLocal, runtime_state::currentFrame().timing.deltaSeconds);
        }
        return GrabAttemptResult::Grabbed;
    }
}
