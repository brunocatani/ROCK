#include "physics-interaction/hand/Hand.h"

/*
 * The SEAT SOLVE: the middle of grab acquisition, where a hand that has already
 * chosen an object, a body, a pivot and its contact evidence decides the exact
 * pose the object will be held in, and freezes it into _grabFrame.
 *
 * Order of work in captureCanonicalGrabFrame:
 *   1. capture frames   - body, motion and owner frames at grab time
 *   2. classify         - build the pocket and grip area, accept or reject
 *   3. seat             - grip normal, support model, swing/roll, depth stop
 *   4. freeze           - lock the authority frame and populate _grabFrame
 *   5. reset            - clear the held motion history the hold starts from
 * The gated trace and debug dumps are extracted below the function so the flow
 * above stays readable; they are pure reads and change nothing.
 *
 * _grabFrame is written ONCE here, at the point each value is final. Do not add
 * an early write and a later correction: the second write silently wins and the
 * capture log then describes a frame the hold never used.
 *
 * Every failure exit returns through abortGrab, which routes to
 * abortGrabAcquisition in HandGrabAcquire.cpp - the complete teardown.
 */

#include "physics-interaction/hand/grab/HandGrabAcquisitionContext.h"

namespace rock
{
    using namespace hand_grab_detail;

    bool Hand::captureCanonicalGrabFrame(hand_grab_detail::GrabAcquisitionContext& context)
    {
        auto* world = context.world;
        const auto& sel = context.selection;
        const auto& sharedContext = *context.sharedContext;
        const bool joiningPeerHeldObject = context.joiningPeerHeldObject;
        const bool looseWeaponGrab = context.looseWeaponGrab;
        const bool grabbedFromPullCatch = context.grabbedFromPullCatch;
        auto& objectBodyId = context.objectBodyId;
        auto* rootNode = context.rootNode;
        const auto& objName = context.objectName;
        const auto motionTypeStr = context.motionType;
        const auto grabTraceId = context.grabTraceId;
        const auto& preparedBodySet = context.preparedBodySet;
        const auto& handWorldTransform = context.handWorldTransform;
        const auto& handBodyWorldAtGrab = context.handBodyWorldAtGrab;
        const auto& proxyFrameWorldAtGrab = context.proxyFrameWorldAtGrab;
        const auto proxyFrameSourceAtGrab = context.proxyFrameSourceAtGrab;
        const bool hasPalmProxyFrameAtGrab = context.hasPalmProxyFrameAtGrab;
        const auto& proxyAuthorityFrameWorldAtGrab = context.proxyAuthorityFrameWorldAtGrab;
        const auto& palmPocketPivotAWorld = context.palmPocketPivotAWorld;
        auto*& collidableNode = context.collidableNode;
        auto* meshSourceNode = context.meshSourceNode;
        auto& objectWorldTransform = context.objectWorldTransform;
        auto& grabGripPoint = context.grabGripPoint;
        auto& selectionToMeshDistanceGameUnits = context.selectionToMeshDistanceGameUnits;
        const auto& grabMeshTriangles = context.grabMeshTriangles;
        auto& grabFingerPoseMeshTriangles = context.grabFingerPoseMeshTriangles;
        const auto& grabSurfaceTriangles = context.grabSurfaceTriangles;
        auto& grabLocalMeshTriangles = context.grabLocalMeshTriangles;
        auto& grabFingerPoseLocalMeshTriangles = context.grabFingerPoseLocalMeshTriangles;
        auto& grabSurfaceHit = context.grabSurfaceHit;
        auto& contactPatchRuntime = context.contactPatchRuntime;
        auto& multiFingerGripRuntime = context.multiFingerGripRuntime;
        const auto& fingerEvidencePointWorld = context.fingerEvidencePointWorld;
        auto& contactPatchEvidenceAvailable = context.contactPatchEvidenceAvailable;
        auto& multiFingerGripUsed = context.multiFingerGripUsed;
        const auto fingerEvidencePointValid = context.fingerEvidencePointValid;
        auto& pivotAuthoritySource = context.pivotAuthoritySource;
        auto& pivotAuthorityNormalTrusted = context.pivotAuthorityNormalTrusted;
        auto& pivotAuthorityPositionOnly = context.pivotAuthorityPositionOnly;
        auto& pivotAuthorityPositionConfidence = context.pivotAuthorityPositionConfidence;
        auto& pivotAuthorityPocketDistanceGameUnits = context.pivotAuthorityPocketDistanceGameUnits;
        auto& pivotAuthoritySelectionDeltaGameUnits = context.pivotAuthoritySelectionDeltaGameUnits;
        auto& pivotAuthorityLongLeverGameUnits = context.pivotAuthorityLongLeverGameUnits;
        const auto& mechanicalScope = context.mechanicalScope;
        const auto& primaryChoice = context.primaryChoice;
        const auto authoredGrabNode = context.authoredGrabNode;
        const auto& canonicalPivotNormalWorld = context.canonicalPivotNormalWorld;
        const auto& semanticContacts = context.semanticContacts;
        const auto& palmPocketSurfaceHit = context.palmPocketSurfaceHit;
        const bool palmPocketMeshAvailable = context.palmPocketMeshAvailable;
        const auto& pinchPocketCandidate = context.pinchPocketCandidate;
        auto& grabPointMode = context.grabPointMode;
        auto& grabFallbackReason = context.grabFallbackReason;
        auto& resolvedGrabOffsetSource = context.resolvedGrabOffsetSource;

        auto abortGrab = [&]() {
            return abortGrabAcquisition(context);
        };
        const auto logGrabCaptureRefresh = [&](const GrabCaptureTransformRefreshResult& refresh) {
            if (!grabTimelineTraceEnabled()) {
                return;
            }
            for (std::uint32_t i = 0; i < refresh.count; ++i) {
                const auto& sample = refresh.samples[i];
                ROCK_LOG_INFO(Hand,
                    "{} GRAB_TRACE stage=capture_refresh trace={} hand={} role={} node='{}'({:p}) validBefore={} validAfter={} posDelta={:.4f}gu rotDelta={:.3f}deg",
                    handName(),
                    grabTraceId,
                    _isLeft ? "left" : "right",
                    sample.role,
                    nodeDebugName(sample.node),
                    static_cast<const void*>(sample.node),
                    sample.validBefore ? "yes" : "no",
                    sample.validAfter ? "yes" : "no",
                    sample.positionDeltaGameUnits,
                    sample.rotationDeltaDegrees);
            }
        };

        logRuntimeScaleIfChanged(_isLeft, handName(), handWorldTransform, collidableNode);

        ROCK_LOG_INFO(Hand, "{} hand GRAB: '{}' formID={:08X} bodyId={}", handName(), objName, sel.refr->GetFormID(), objectBodyId.value);

        if (g_rockConfig.rockDebugShowGrabNotifications) {
            auto msg = std::format("[ROCK] {} GRAB: {} ({})", _isLeft ? "L" : "R", objName, motionTypeStr);
            f4vr::showNotification(msg);
        }

        bool adoptedPeerHeldBodySet = false;
        _heldBodyIds = buildCommittedHeldBodyIds(objectBodyId.value, mechanicalScope.committedBodyIds, sharedContext, adoptedPeerHeldBodySet);
        if (_heldBodyIds.empty()) {
            _heldBodyIds.push_back(objectBodyId.value);
        }
        _heldDriveDecision = mechanicalScope.driveDecision;
        if (adoptedPeerHeldBodySet) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand SHARED held body set adopted: primaryBody={} peerBodies={} committedBodies={} scanAccepted={} mechanicalKind={} driveMode={} driveReason={}",
                handName(),
                objectBodyId.value,
                sharedContext.peerHeldBodyIds ? sharedContext.peerHeldBodyIds->size() : 0,
                _heldBodyIds.size(),
                preparedBodySet.acceptedCount(),
                mechanical_connected_body_set::scopeKindName(mechanicalScope.kind),
                held_object_drive_policy::modeName(_heldDriveDecision.mode),
                _heldDriveDecision.reason);
        }

        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (player && !joiningPeerHeldObject) {
                nativeVRGrabDrop(player, 0);
                nativeVRGrabDrop(player, 1);
            }
        }

        _grabStartTime = 0.0f;
        _grabConvergeStableInsidePocketFrames = 0;
        _grabConvergePreviousGripErrorGameUnits = std::numeric_limits<float>::max();

        {
            // ---- Stage 1: capture frames -------------------------------------
            // Read every frame the seat solve compares against, once, before
            // anything moves. An unreadable body frame aborts here rather than
            // seating against a frame the constraint cannot reproduce.
            const RE::NiPoint3 palmPos = computeGrabLegacyPalmPivotAWorldFromHandBasis(handWorldTransform, _isLeft);
            RE::NiTransform grabBodyWorldAtGrab{};
            if (!tryGetGrabAuthorityBodyWorldTransform(world, objectBodyId, grabBodyWorldAtGrab)) {
                ROCK_LOG_ERROR(Hand,
                    "{} hand GRAB FAILED: native grab BODY frame unreadable bodyId={} formID={:08X}",
                    handName(),
                    objectBodyId.value,
                    sel.refr ? sel.refr->GetFormID() : 0);
                return abortGrab();
            }
            RE::NiTransform motionBodyWorldAtGrab{};
            body_frame::BodyFrameSource motionBodySourceAtGrab = body_frame::BodyFrameSource::Fallback;
            const bool hasMotionBodyWorldAtGrab =
                tryResolveLiveBodyWorldTransform(world, objectBodyId, motionBodyWorldAtGrab, &motionBodySourceAtGrab);
            /*
             * Custom constraint body-B data is authored in the hknp BODY frame.
             * HIGGS does the same kind of thing on Skyrim's hkp side: it freezes
             * pivot B from the rigid body transform and treats COM/motion as mass
             * data, not as a local-frame owner. A runtime MOTION-frame experiment
             * made the object settle at the wrong rotation even when the proxy
             * target was stable, so the live motion transform remains diagnostic
             * evidence only.
             */
            const bool constraintUsesMotionBodyAtGrab = false;
            const RE::NiTransform constraintBodyWorldAtGrab = grabBodyWorldAtGrab;
            /*
             * The hidden proxy is body A for dynamic grab. The close-grab
             * pocket pivot is captured from the same generated/proxy-local
             * seat frame. The proxy body keeps its seat offset, and transform A
             * carries the selected pivot as an explicit local point on body A.
             */
            RE::NiPoint3 grabPivotAWorld = palmPocketPivotAWorld;
            auto* ownerCellAtGrab = sel.refr ? sel.refr->GetParentCell() : nullptr;
            auto* bhkWorldAtGrab = ownerCellAtGrab ? ownerCellAtGrab->GetbhkWorld() : nullptr;
            auto* bodyCollisionObjectAtGrab = bhkWorldAtGrab ? RE::bhkNPCollisionObject::Getbhk(bhkWorldAtGrab, objectBodyId) : nullptr;
            auto* ownerNodeAtGrab = bodyCollisionObjectAtGrab ? bodyCollisionObjectAtGrab->sceneObject : nullptr;
            if (ownerNodeAtGrab && ownerNodeAtGrab != collidableNode) {
                GrabCaptureTransformRefreshResult ownerAtGrabRefresh{};
                refreshGrabCaptureNodeTransform(ownerAtGrabRefresh, "ownerAtGrab", ownerNodeAtGrab);
                logGrabCaptureRefresh(ownerAtGrabRefresh);
                if (!ownerAtGrabRefresh.ok) {
                    ROCK_LOG_WARN(Hand,
                        "{} hand GRAB failed: owner-at-grab transform refresh produced a non-finite node transform for '{}' formID={:08X}; owner='{}'",
                        handName(),
                        objName,
                        sel.refr ? sel.refr->GetFormID() : 0,
                        nodeDebugName(ownerNodeAtGrab));
                    return abortGrab();
                }
            }
            const RE::NiTransform objectToBodyAtGrab = computeRuntimeBodyLocalTransform(objectWorldTransform, grabBodyWorldAtGrab);
            /*
             * ROCK dynamic grab has one production authority convention:
             * the generated/proxy palm frame seats the selected BODY-local grip
             * point and owns the object angular relation through a row-view of
             * its generated local axes. The custom constraint stores body-B local
             * data in the rigid BODY frame. MOTION and COM are mass/diagnostic
             * data only.
             */
            const RE::NiTransform ownerBodyLocalAtGrab =
                ownerNodeAtGrab ? computeRuntimeBodyLocalTransform(ownerNodeAtGrab->world, grabBodyWorldAtGrab) : makeIdentityTransform();
            const RE::NiTransform rootBodyLocalAtGrab =
                rootNode ? computeRuntimeBodyLocalTransform(rootNode->world, grabBodyWorldAtGrab) : makeIdentityTransform();
            RE::NiPoint3 selectedGripPointLocal = transform_math::worldPointToLocal(objectWorldTransform, grabGripPoint);
            RE::NiPoint3 selectedPivotBBodyLocalGame = transform_math::worldPointToLocal(grabBodyWorldAtGrab, grabGripPoint);
            if (!grabMeshTriangles.empty()) {
                grabFingerPoseMeshTriangles = selectNearestGrabFingerPoseTriangles(
                    grabMeshTriangles,
                    grabGripPoint,
                    kMaxGrabRuntimeFingerPoseTriangles);
                grabFingerPoseLocalMeshTriangles = cacheTrianglesInLocalSpace(grabFingerPoseMeshTriangles, objectWorldTransform);
                if (grabFingerPoseMeshTriangles.size() != grabMeshTriangles.size()) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand MESH FINGER POSE TRIANGLES: sourceTris={} localTris={} center=({:.1f},{:.1f},{:.1f})",
                        handName(),
                        grabMeshTriangles.size(),
                        grabFingerPoseMeshTriangles.size(),
                        grabGripPoint.x,
                        grabGripPoint.y,
                        grabGripPoint.z);
                }
            }
            RE::NiTransform desiredObjectWorld = objectWorldTransform;
            RE::NiTransform desiredBodyWorld = grabBodyWorldAtGrab;
            bool looseWeaponPrimaryAttachApplied = false;
            bool looseWeaponPrimaryAttachSourceVisible = false;
            const char* looseWeaponPrimaryAttachReason = "notEvaluated";
            // ---- Stage 2: classify and seat ----------------------------------
            // Build the pocket and the grip area, decide accept or reject, then
            // solve the seat pose inside the accepted branch. Rejection is a
            // normal outcome here, not an error.
            const auto oppositionContacts = hand_semantic_contact_state::selectThumbOppositionContacts(semanticContacts);
            auto resolvedAuthorityPivotSourceForFreeze = grab_authority_frame_math::GrabAuthorityPivotSource::None;
            const char* resolvedAuthorityPivotReasonForFreeze = "notResolved";
            /*
             * Runtime grab authority is intentionally single-source. Mesh hits,
             * authored nodes, semantic contacts, contact patches, and multi-finger
             * contacts are evidence until the pivot resolver chooses one BODY-local
             * point to freeze. Final dynamic authority is strict: only a pinch
             * pocket or an authorable support group may create the frozen frame.
             */
            {
                auto pocket = grab_three_phase::buildGrabPocketFrameWithPalmCenter(
                    proxyAuthorityFrameWorldAtGrab,
                    _isLeft,
                    grabPivotAWorld,
                    g_rockConfig.rockGrabPocketDepthGameUnits,
                    g_rockConfig.rockGrabPocketRadiusGameUnits);
                auto gripArea = grab_three_phase::buildObjectGripArea(grab_three_phase::GripAreaInput{
                    .objectBodyWorld = grabBodyWorldAtGrab,
                    .contactSeedWorld = grabGripPoint,
                    .centerOfMassWorld = grabBodyWorldAtGrab.translate,
                    .interiorDirectionWorld = RE::NiPoint3{},
                    .preferredInsetGameUnits = 0.0f,
                    .maxInsetGameUnits = 0.0f,
                    .centerOfMassValid = false,
                    .interiorDirectionValid = false,
                    .source = grabPointMode,
                });
                if (pinchPocketCandidate.valid) {
                    gripArea = grab_three_phase::buildObjectGripArea(grab_three_phase::GripAreaInput{
                        .objectBodyWorld = grabBodyWorldAtGrab,
                        .contactSeedWorld = pinchPocketCandidate.surfaceHit.position,
                        .centerOfMassWorld = grabBodyWorldAtGrab.translate,
                        .interiorDirectionWorld = RE::NiPoint3{},
                        .preferredInsetGameUnits = 0.0f,
                        .maxInsetGameUnits = 0.0f,
                        .centerOfMassValid = false,
                        .interiorDirectionValid = false,
                        .source = "pinchPocket",
                    });
                }
                const RE::NiPoint3 phaseGripSeed = gripArea.valid ? gripArea.contactSeedWorld : grabGripPoint;
                const RE::NiPoint3 palmSeatAnchorWorld = pocket.valid ? pocket.palmCenterWorld : grabPivotAWorld;
                const RE::NiPoint3 gripToPocket = pocket.valid ? (phaseGripSeed - palmSeatAnchorWorld) : RE::NiPoint3{};
                const float gripToPocketDistance =
                    pocket.valid ? std::sqrt(gripToPocket.x * gripToPocket.x + gripToPocket.y * gripToPocket.y + gripToPocket.z * gripToPocket.z) :
                                   std::numeric_limits<float>::max();
                const float stableTouchEnvelope = (std::max)(g_rockConfig.rockGrabTouchAcquireDistanceGameUnits, pocket.pocketRadiusGameUnits);
                const bool hasStablePocketTouchContact =
                    oppositionContacts.valid && gripToPocketDistance <= stableTouchEnvelope;
                const bool meshSurfaceAuthorityEvidence =
                    grabSurfaceHit.valid && grabSurfaceHit.sourceKind != GrabSurfaceSourceKind::CollisionQuery;
                const bool pullArrivalTouchHeldAuthorityEvidence =
                    hasStablePocketTouchContact ||
                    meshSurfaceAuthorityEvidence ||
                    contactPatchEvidenceAvailable ||
                    multiFingerGripUsed ||
                    authoredGrabNode != nullptr;
                const auto phaseDecision = grab_three_phase::classifyAcquisitionPhase(grab_three_phase::PhaseClassificationInput{
                    .pocket = pocket,
                    .gripSeedWorld = phaseGripSeed,
                    .hasFreshTouchContact = hasStablePocketTouchContact,
                    .isFarSelection = sel.isFarSelection,
                    .programmaticArrival = sel.forcedArrival,
                    .requireEvidenceForTouchHeld = grabbedFromPullCatch,
                    .hasTouchHeldAuthorityEvidence = pullArrivalTouchHeldAuthorityEvidence,
                    .touchAcquireDistanceGameUnits = g_rockConfig.rockGrabTouchAcquireDistanceGameUnits,
                    .touchContactMaxDistanceGameUnits = stableTouchEnvelope,
                    .nearConvergeDistanceGameUnits = g_rockConfig.rockGrabNearConvergeDistanceGameUnits,
                    .behindPalmToleranceGameUnits = g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits,
                });

                const bool usingPinchPocket = pinchPocketCandidate.valid && gripArea.valid;
                const bool captureAccepted = usingPinchPocket || (pocket.valid && gripArea.valid && phaseDecision.accepted);
                if (captureAccepted) {
                    _grabObjectGripAtGrab = gripArea;
                    _grabAcquisitionPhase = usingPinchPocket ? grab_three_phase::AcquisitionPhase::TouchHeld : phaseDecision.phase;
                    if (usingPinchPocket) {
                        _grabObjectGripAtGrab.source = "pinchPocket";
                        _grabObjectGripAtGrab.fallbackReason = pinchPocketCandidate.decision.reason;
                        _grabObjectGripAtGrab.confidence = 0.95f;
                    }
                    const char* relationMode = usingPinchPocket ? "pinchPocket" : "rockPointToPalm";
                    const char* captureReason = usingPinchPocket ? pinchPocketCandidate.decision.reason : phaseDecision.reason;

                    if (usingPinchPocket) {
                        grabPivotAWorld = pinchPocketCandidate.pinchPocketWorld;
                    } else {
                        grabPivotAWorld = pocket.palmCenterWorld;
                    }
                    grabGripPoint = gripArea.contactSeedWorld;
                    grabPointMode = relationMode;
                    grabFallbackReason = captureReason;
                    if (usingPinchPocket) {
                        grabSurfaceHit = pinchPocketCandidate.surfaceHit;
                        grabSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                        grabSurfaceHit.selectionToMeshDistanceGameUnits =
                            sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
                        grabSurfaceHit.pivotToSurfaceDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                        grabSurfaceHit.resolvedOwnerMatchesBody = true;
                        selectionToMeshDistanceGameUnits = grabSurfaceHit.selectionToMeshDistanceGameUnits;
                        pivotAuthoritySource = grabPivotAuthoritySourceName(GrabPivotAuthoritySource::PinchPocketMeshPoint);
                        pivotAuthorityPositionOnly = false;
                        pivotAuthorityNormalTrusted = true;
                        pivotAuthorityPositionConfidence = 0.95f;
                        pivotAuthorityPocketDistanceGameUnits = grabSurfaceHit.pivotToSurfaceDistanceGameUnits;
                        pivotAuthoritySelectionDeltaGameUnits = grabSurfaceHit.selectionToMeshDistanceGameUnits;
                    }

                    auto firstValidNormal = [](std::initializer_list<RE::NiPoint3> candidates) {
                        for (const auto& candidate : candidates) {
                            const RE::NiPoint3 normal = normalizeOrZero(candidate);
                            if (lengthSquared(normal) > 0.000001f) {
                                return normal;
                            }
                        }
                        return RE::NiPoint3{};
                    };

                    /*
                     * Pivot B may be chosen from hand/proxy proximity, but normal
                     * authority must come from the object surface that produced the
                     * pivot. The palm/proxy normal is only a final fallback and
                     * tangent/sign guide; promoting it to face authority makes the
                     * frozen support frame follow the hand instead of the mesh.
                     */
                    RE::NiPoint3 gripNormalWorld = firstValidNormal({
                        grabSurfaceHit.valid ? grabSurfaceHit.normal : RE::NiPoint3{},
                        contactPatchRuntime.meshSnapped ? contactPatchRuntime.meshSnapHit.normal : RE::NiPoint3{},
                        contactPatchRuntime.normalTrusted ? contactPatchRuntime.patch.normal : RE::NiPoint3{},
                        palmPocketMeshAvailable ? palmPocketSurfaceHit.normal : RE::NiPoint3{},
                        canonicalPivotNormalWorld,
                        usingPinchPocket ? pinchPocketCandidate.pinchAxisWorld : RE::NiPoint3{},
                        pocket.palmNormalWorld,
                    });

                    RE::NiPoint3 gripEvidencePointWorld = grabGripPoint;
                    RE::NiPoint3 gripEvidenceNormalWorld = gripNormalWorld;
                    RuntimeGripSupportModel gripSupportRuntime{};
                    const float supportObjectScale =
                        std::isfinite(objectWorldTransform.scale) && objectWorldTransform.scale > 0.0f ? objectWorldTransform.scale : 1.0f;
                    const float supportLeverGameUnits =
                        !grabLocalMeshTriangles.empty() && grab_three_phase::isFinite(objectWorldTransform) ?
                            computeLocalMeshMaxDistanceFromPoint(
                                grabLocalMeshTriangles,
                                transform_math::worldPointToLocal(objectWorldTransform, grabGripPoint)) *
                                supportObjectScale :
                            0.0f;
                    if (!grabSurfaceTriangles.empty() && !grabLocalMeshTriangles.empty()) {
                        gripSupportRuntime = buildRuntimeGripSupportModel(sel,
                            preparedBodySet,
                            objectBodyId.value,
                            objectWorldTransform,
                            grabSurfaceTriangles,
                            grabLocalMeshTriangles,
                            contactPatchRuntime,
                            pinchPocketCandidate,
                            grabGripPoint,
                            gripNormalWorld,
                            grabPivotAWorld,
                            pocket.valid ? pocket.palmNormalWorld : gripNormalWorld,
                            pocket.valid ? pocket.fingerForwardWorld : RE::NiPoint3{},
                            pocket.valid ? pocket.crossPalmWorld : RE::NiPoint3{},
                            supportLeverGameUnits);
                    }
                    /*
                     * Support/pinch convergence is a promotion step, not a
                     * rejection gate. If rich support probing only produced weak
                     * same-surface or single-point evidence, promote the already
                     * selected object-side point into SupportGroup authority
                     * instead of aborting a valid grab.
                     */
                    if (!usingPinchPocket && !gripSupportRuntime.model.canAuthorPivot) {
                        const bool hadWeakSupportModel = gripSupportRuntime.model.valid;
                        forceRuntimeGripSupportAuthority(gripSupportRuntime,
                            grabGripPoint,
                            gripNormalWorld,
                            pocket.valid ? pocket.palmNormalWorld : gripNormalWorld,
                            pocket.valid ? pocket.crossPalmWorld : RE::NiPoint3{},
                            hadWeakSupportModel ? 0.55f : 0.35f,
                            hadWeakSupportModel ? "forcedSupportGroupFromWeakEvidence" : "forcedSupportGroupFromGrabPoint");
                    }
                    if (gripSupportRuntime.model.canAuthorPivot) {
                        grabGripPoint = gripSupportRuntime.model.pivotPoint;
                        gripArea.contactSeedWorld = grabGripPoint;
                        grabPointMode = gripSupportActivePointMode(gripSupportRuntime.model.kind);
                        relationMode = grabPointMode;
                        grabFallbackReason = gripSupportRuntime.model.reason;
                        _grabObjectGripAtGrab.contactSeedWorld = grabGripPoint;
                        _grabObjectGripAtGrab.source = grabPointMode;
                        _grabObjectGripAtGrab.fallbackReason = grabFallbackReason;
                        _grabObjectGripAtGrab.confidence = (std::max)(_grabObjectGripAtGrab.confidence, gripSupportRuntime.model.confidence);
                        pivotAuthoritySource = grabPivotAuthoritySourceName(GrabPivotAuthoritySource::GripSupportModel);
                        pivotAuthorityPositionOnly = false;
                        pivotAuthorityNormalTrusted = lengthSquared(gripSupportRuntime.model.supportNormal) > 0.000001f;
                        pivotAuthorityPositionConfidence = gripSupportRuntime.model.confidence;
                        pivotAuthorityPocketDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                        pivotAuthoritySelectionDeltaGameUnits =
                            sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
                        pivotAuthorityLongLeverGameUnits = supportLeverGameUnits;
                        if (pivotAuthorityNormalTrusted) {
                            gripNormalWorld = gripSupportRuntime.model.supportNormal;
                        }
                    }

                    {
                        using AuthorityCandidate = grab_authority_frame_math::GrabAuthorityPivotCandidate<RE::NiPoint3>;
                        std::array<AuthorityCandidate, 2> authorityCandidates{};
                        authorityCandidates[0] = AuthorityCandidate{
                            .valid = pinchPocketCandidate.valid,
                            .source = grab_authority_frame_math::GrabAuthorityPivotSource::PinchPocket,
                            .pointWorld = pinchPocketCandidate.valid ? pinchPocketCandidate.surfaceHit.position : RE::NiPoint3{},
                            .normalWorld = pinchPocketCandidate.valid ? pinchPocketCandidate.surfaceHit.normal : RE::NiPoint3{},
                            .normalValid = pinchPocketCandidate.valid && lengthSquared(pinchPocketCandidate.surfaceHit.normal) > 0.000001f,
                            .bodyId = objectBodyId.value,
                            .sourceNode = pinchPocketCandidate.valid ? pinchPocketCandidate.surfaceHit.sourceNode : nullptr,
                            .reason = pinchPocketCandidate.valid ? pinchPocketCandidate.decision.reason : "pinchPocketUnavailable",
                        };
                        authorityCandidates[1] = AuthorityCandidate{
                            .valid = gripSupportRuntime.model.canAuthorPivot,
                            .source = grab_authority_frame_math::GrabAuthorityPivotSource::GripSupportModel,
                            .pointWorld = gripSupportRuntime.model.canAuthorPivot ? gripSupportRuntime.model.pivotPoint : RE::NiPoint3{},
                            .normalWorld = gripSupportRuntime.model.canAuthorPivot ? gripSupportRuntime.model.supportNormal : RE::NiPoint3{},
                            .normalValid = gripSupportRuntime.model.canAuthorPivot && lengthSquared(gripSupportRuntime.model.supportNormal) > 0.000001f,
                            .bodyId = objectBodyId.value,
                            .sourceNode = grabSurfaceHit.sourceNode ? grabSurfaceHit.sourceNode : collidableNode,
                            .reason = gripSupportRuntime.model.canAuthorPivot ? gripSupportRuntime.model.reason : "gripSupportUnavailable",
                        };

                        const auto resolvedAuthorityPivot = grab_authority_frame_math::resolveGrabAuthorityPivot<RE::NiPoint3>(
                            authorityCandidates);
                        resolvedAuthorityPivotSourceForFreeze = resolvedAuthorityPivot.source;
                        resolvedAuthorityPivotReasonForFreeze = resolvedAuthorityPivot.reason ? resolvedAuthorityPivot.reason : "none";
                        if (!resolvedAuthorityPivot.valid) {
                            ROCK_LOG_WARN(Hand,
                                "{} hand GRAB failed: no final pinch/support authority bodyId={} currentMode={} reason={} supportKind={} supportReason={} pinchReason={}",
                                handName(),
                                objectBodyId.value,
                                grabPointMode,
                                resolvedAuthorityPivot.reason ? resolvedAuthorityPivot.reason : "none",
                                grab_support_model_math::gripSupportKindName(gripSupportRuntime.model.kind),
                                gripSupportRuntime.model.reason ? gripSupportRuntime.model.reason : "none",
                                pinchPocketCandidate.decision.reason ? pinchPocketCandidate.decision.reason : "none");
                            return abortGrab();
                        }
                        if (g_rockConfig.rockDebugGrabFrameLogging) {
                            for (const auto& candidate : authorityCandidates) {
                                const float pivotDistance = candidate.valid ? pointDistanceGameUnits(grabPivotAWorld, candidate.pointWorld) : -1.0f;
                                const float pocketDistance =
                                    (candidate.valid && pocket.valid) ? pointDistanceGameUnits(candidate.pointWorld, pocket.palmCenterWorld) : -1.0f;
                                const float selectionDistance =
                                    (candidate.valid && sel.hasHitPoint) ? pointDistanceGameUnits(candidate.pointWorld, sel.hitPointWorld) : -1.0f;
                                const float leverDistance =
                                    candidate.valid ? pointDistanceGameUnits(candidate.pointWorld, grabBodyWorldAtGrab.translate) : -1.0f;
                                const bool selectedCandidate = candidate.valid && candidate.source == resolvedAuthorityPivot.source;
                                ROCK_LOG_INFO(Hand,
                                    "{} GRAB FREEZE CANDIDATE: formID={:08X} body={} source={} valid={} selected={} reason={} node='{}' "
                                    "point=({:.2f},{:.2f},{:.2f}) normal=({:.3f},{:.3f},{:.3f}) pivotA={:.2f}gu pocket={:.2f}gu selection={:.2f}gu lever={:.2f}gu mode={}",
                                    handName(),
                                    sel.refr ? sel.refr->GetFormID() : 0,
                                    objectBodyId.value,
                                    grab_authority_frame_math::grabAuthorityPivotSourceName(candidate.source),
                                    candidate.valid ? "yes" : "no",
                                    selectedCandidate ? "yes" : "no",
                                    candidate.reason ? candidate.reason : "none",
                                    nodeDebugName(static_cast<const RE::NiAVObject*>(candidate.sourceNode)),
                                    candidate.valid ? candidate.pointWorld.x : 0.0f,
                                    candidate.valid ? candidate.pointWorld.y : 0.0f,
                                    candidate.valid ? candidate.pointWorld.z : 0.0f,
                                    candidate.normalValid ? candidate.normalWorld.x : 0.0f,
                                    candidate.normalValid ? candidate.normalWorld.y : 0.0f,
                                    candidate.normalValid ? candidate.normalWorld.z : 0.0f,
                                    pivotDistance,
                                    pocketDistance,
                                    selectionDistance,
                                    leverDistance,
                                    grabPointMode);
                            }
                        }

                        switch (resolvedAuthorityPivot.source) {
                        case grab_authority_frame_math::GrabAuthorityPivotSource::PinchPocket:
                            grabGripPoint = pinchPocketCandidate.surfaceHit.position;
                            grabSurfaceHit = pinchPocketCandidate.surfaceHit;
                            grabPointMode = "pinchPocket";
                            grabFallbackReason = pinchPocketCandidate.decision.reason;
                            pivotAuthoritySource = grabPivotAuthoritySourceName(GrabPivotAuthoritySource::PinchPocketMeshPoint);
                            pivotAuthorityNormalTrusted = resolvedAuthorityPivot.normalValid;
                            pivotAuthorityPositionConfidence = 0.95f;
                            break;
                        case grab_authority_frame_math::GrabAuthorityPivotSource::GripSupportModel:
                            grabGripPoint = gripSupportRuntime.model.pivotPoint;
                            grabPointMode = gripSupportActivePointMode(gripSupportRuntime.model.kind);
                            grabFallbackReason = gripSupportRuntime.model.reason;
                            pivotAuthoritySource = grabPivotAuthoritySourceName(GrabPivotAuthoritySource::GripSupportModel);
                            pivotAuthorityNormalTrusted = resolvedAuthorityPivot.normalValid;
                            pivotAuthorityPositionConfidence = gripSupportRuntime.model.confidence;
                            break;
                        case grab_authority_frame_math::GrabAuthorityPivotSource::None:
                        case grab_authority_frame_math::GrabAuthorityPivotSource::PalmPocketMesh:
                        case grab_authority_frame_math::GrabAuthorityPivotSource::SelectionMeshSnap:
                        case grab_authority_frame_math::GrabAuthorityPivotSource::CollisionFallback:
                        default:
                            break;
                        }
                        pivotAuthorityPositionOnly = false;
                        pivotAuthorityPocketDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                        pivotAuthoritySelectionDeltaGameUnits =
                            sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
                        if (resolvedAuthorityPivot.normalValid) {
                            gripNormalWorld = resolvedAuthorityPivot.normalWorld;
                        }
                        gripEvidencePointWorld = grabGripPoint;
                        gripEvidenceNormalWorld = gripNormalWorld;
                        _grabObjectGripAtGrab.contactSeedWorld = grabGripPoint;
                        _grabObjectGripAtGrab.gripCenterWorld = grabGripPoint;
                        _grabObjectGripAtGrab.source = grabPointMode;
                        _grabObjectGripAtGrab.fallbackReason = grabFallbackReason;
                    }

                    const RE::NiPoint3 finalGripToPocketVector =
                        pocket.valid ? (grabGripPoint - pocket.palmCenterWorld) : RE::NiPoint3{};
                    const float finalGripToPocketDistance =
                        pocket.valid ?
                            std::sqrt(finalGripToPocketVector.x * finalGripToPocketVector.x +
                                      finalGripToPocketVector.y * finalGripToPocketVector.y +
                                      finalGripToPocketVector.z * finalGripToPocketVector.z) :
                            std::numeric_limits<float>::max();
                    const float finalSignedPalmDistance =
                        pocket.valid ? grab_three_phase::dot(finalGripToPocketVector, pocket.palmNormalWorld) : 0.0f;
                    const auto pullCatchSeatSafety =
                        grab_three_phase::evaluatePullCatchSeatSafety(grab_three_phase::PullCatchSeatSafetyInput{
                            .grabbedFromPullCatch = grabbedFromPullCatch,
                            .usingPinchPocket = usingPinchPocket,
                            .capturePhase = _grabAcquisitionPhase,
                            .pocketValid = pocket.valid,
                            .stablePocketTouchContact = hasStablePocketTouchContact,
                            .pivotAuthorityNormalTrusted = pivotAuthorityNormalTrusted,
                            .pivotAuthorityPositionOnly = pivotAuthorityPositionOnly,
                            .gripToPocketDistanceGameUnits = finalGripToPocketDistance,
                            .signedPalmDistanceGameUnits = finalSignedPalmDistance,
                            .behindPalmToleranceGameUnits = g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits,
                            .touchAcquireDistanceGameUnits = g_rockConfig.rockGrabTouchAcquireDistanceGameUnits,
                            .pocketRadiusGameUnits = pocket.valid ? pocket.pocketRadiusGameUnits : g_rockConfig.rockGrabPocketRadiusGameUnits,
                            .palmNormalWorld = pocket.valid ? pocket.palmNormalWorld : RE::NiPoint3{},
                            .gripNormalWorld = gripNormalWorld,
                        });
                    if (grabbedFromPullCatch &&
                        !usingPinchPocket &&
                        _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld &&
                        !pullCatchSeatSafety.allowImmediateTouchHeld) {
                        _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::NearConverging;
                        captureReason = pullCatchSeatSafety.reason;
                        grabFallbackReason = captureReason;
                        _grabObjectGripAtGrab.fallbackReason = captureReason;
                    }

                    desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(
                        grabBodyWorldAtGrab,
                        grabPivotAWorld,
                        grabGripPoint);
                    desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                    const bool programmaticArrival = grabbedFromPullCatch || sel.forcedArrival;
                    /*
                     * Guns and melee never seat from saved offsets: the FRIK
                     * weapon-offset attach below is the only weapon authority
                     * (saved_grab_offset::participatesInSavedGrabOffsets).
                     * Leaving the source empty here also keeps a saved finger
                     * pose from overriding the FRIK weapon hand pose.
                     */
                    if (programmaticArrival &&
                        saved_grab_offset::participatesInSavedGrabOffsets(
                            looseWeaponGrab,
                            isThrowableLooseWeapon(selectedLooseWeaponForm(sel)))) {
                        resolvedGrabOffsetSource = resolveGrabOffsetSource(_isLeft, sel.refr);
                    }
                    RE::NiTransform grabProxyWorldForOffset{};
                    const bool grabProxyWorldValidForOffset =
                        programmaticArrival && tryComputeGrabProxyLocalPalmPocketFrameWorld(world, grabProxyWorldForOffset);
                    const auto grabOffsetAttachSource = resolveGrabOffsetAttachSource(
                        grabProxyWorldForOffset, grabProxyWorldValidForOffset, resolvedGrabOffsetSource);
                    const auto looseWeaponPrimaryAttachFrame = resolveLooseWeaponPrimaryAttachFrame(
                        looseWeaponGrab,
                        grabbedFromPullCatch,
                        _isLeft,
                        sel,
                        rootNode,
                        rootBodyLocalAtGrab,
                        objectToBodyAtGrab,
                        grabBodyWorldAtGrab,
                        grabPivotAWorld,
                        handWorldTransform,
                        grabOffsetAttachSource);
                    looseWeaponPrimaryAttachReason = looseWeaponPrimaryAttachFrame.reason;
                    if (looseWeaponPrimaryAttachFrame.valid) {
                        desiredObjectWorld = looseWeaponPrimaryAttachFrame.desiredObjectWorld;
                        desiredBodyWorld = looseWeaponPrimaryAttachFrame.desiredBodyWorld;
                        grabGripPoint = looseWeaponPrimaryAttachFrame.gripPointWorld;
                        gripArea.contactSeedWorld = grabGripPoint;
                        gripEvidencePointWorld = grabGripPoint;
                        gripNormalWorld = firstValidNormal({
                            pocket.valid ? pocket.palmNormalWorld : RE::NiPoint3{},
                            gripNormalWorld,
                        });
                        gripEvidenceNormalWorld = gripNormalWorld;
                        _grabObjectGripAtGrab.contactSeedWorld = grabGripPoint;
                        _grabObjectGripAtGrab.gripCenterWorld = grabGripPoint;
                        grabPointMode = "looseWeaponPrimaryAttach";
                        relationMode = grabPointMode;
                        grabFallbackReason = looseWeaponPrimaryAttachFrame.reason;
                        _grabObjectGripAtGrab.source = grabPointMode;
                        _grabObjectGripAtGrab.fallbackReason = grabFallbackReason;
                        if (_grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld) {
                            _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::NearConverging;
                            captureReason = "looseWeaponPrimaryAttachSettle";
                        }
                        grabSurfaceHit = GrabSurfaceHit{};
                        contactPatchRuntime = RuntimeGrabContactPatch{};
                        contactPatchEvidenceAvailable = false;
                        pivotAuthoritySource = grabPivotAuthoritySourceName(GrabPivotAuthoritySource::LooseWeaponPrimaryAttach);
                        pivotAuthorityPositionOnly = false;
                        pivotAuthorityNormalTrusted = true;
                        pivotAuthorityPositionConfidence = 1.0f;
                        pivotAuthorityPocketDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                        pivotAuthoritySelectionDeltaGameUnits = sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
                        resolvedAuthorityPivotSourceForFreeze = grab_authority_frame_math::GrabAuthorityPivotSource::LooseWeaponPrimaryAttach;
                        resolvedAuthorityPivotReasonForFreeze = looseWeaponPrimaryAttachFrame.reason;
                        looseWeaponPrimaryAttachApplied = true;
                        looseWeaponPrimaryAttachSourceVisible = looseWeaponPrimaryAttachFrame.sourceVisible;
                    }
                    /*
                     * Seat-time self-alignment. Forced arrivals (grenade menu,
                     * provider API) commit without a pull flight, so the
                     * long-axis presentation servo never gets to run: rotate
                     * the SEAT pose instead - the minimal rotation taking the
                     * mesh principal axis onto the pocket's cross-palm
                     * (thumb->pinky) line, about the grip point so the pivot
                     * pair is untouched. Pull-catch arrivals already aligned
                     * their long axis in flight and keep it.
                     *
                     * Face-to-palm alignment is a separate correction: a
                     * minimal-arc swing (flight servo or seat rotation) adds
                     * zero twist, so an object that arrived tilted keeps its
                     * arrival roll and its flat face can cut diagonally into
                     * the palm - the depth stop below only translates and
                     * cannot fix a tilted face. Which rotation achieves it
                     * depends on the shape class, read from the same PCA
                     * spectrum:
                     *
                     *   ROD/PLANK (elongationRatio gate passes): the long axis
                     *   is already committed to the cross-palm line, so the
                     *   only free DOF is a TWIST about it - roll until the
                     *   thinnest-extent direction faces the palm. Round cross
                     *   sections skip; any roll is equivalent for them.
                     *
                     *   PLATE (elongation gate fails but the minor axis is
                     *   well separated - books, trays, boards): there is no
                     *   meaningful long axis to twist about, but the face
                     *   normal is exact. Swing that normal onto the palm
                     *   normal directly. Nearest-hemisphere keeps the swing
                     *   under 90 degrees and never flips the object over. The
                     *   free in-plane spin is deliberately left alone: it
                     *   cannot affect clipping, and for near-square plates the
                     *   in-plane axes are numerically interchangeable.
                     *
                     *   Otherwise (compact/isotropic): no alignment.
                     *
                     * Rotations survive the freeze (pivot alignment only
                     * rewrites translation).
                     */
                    RE::NiTransform seatBodyWorld = grabBodyWorldAtGrab;
                    RE::NiTransform seatObjectWorld = objectWorldTransform;
                    float seatAlignmentAngleDegrees = 0.0f;
                    const char* seatAlignmentReason = "inactive";
                    float seatRollAngleDegrees = 0.0f;
                    const char* seatRollReason = "inactive";
                    bool seatPoseChanged = false;
                    const bool seatSwingAlignmentWanted =
                        sel.forcedArrival && g_rockConfig.rockForceGrabSeatAlignmentEnabled;
                    const bool seatRollAlignmentWanted =
                        programmaticArrival && g_rockConfig.rockGrabSeatRollAlignmentEnabled;
                    /*
                     * The shape class also selects the penetration backstop
                     * footprint below, and that backstop runs on EVERY arrival
                     * (close grabs never rotate the object), so the PCA is
                     * resolved outside the alignment gates. One pass at
                     * capture, never per frame.
                     */
                    const bool seatShapeEvaluable =
                        !looseWeaponPrimaryAttachApplied &&
                        !usingPinchPocket &&
                        pocket.valid &&
                        !grabMeshTriangles.empty();
                    const auto seatLongAxis =
                        seatShapeEvaluable ? computeGrabMeshLongAxis(grabMeshTriangles) : GrabMeshLongAxisResult{};
                    const bool seatRodShape =
                        seatLongAxis.valid &&
                        seatLongAxis.elongationRatio >= g_rockConfig.rockPullPresentationMinElongationRatio;
                    const bool seatFlatFace =
                        seatLongAxis.valid &&
                        seatLongAxis.secondElongationRatio >= g_rockConfig.rockGrabSeatRollMinSecondElongationRatio;
                    const bool seatPlateShape = seatLongAxis.valid && !seatRodShape && seatFlatFace;
                    if ((seatSwingAlignmentWanted || seatRollAlignmentWanted) && seatShapeEvaluable) {
                        if (seatSwingAlignmentWanted) {
                            seatAlignmentReason = seatLongAxis.reason;
                        }
                        if (seatRollAlignmentWanted) {
                            seatRollReason = seatLongAxis.reason;
                        }
                        RE::NiPoint3 currentAxisWorld = normalizeOrZero(seatLongAxis.axisWorld);
                        RE::NiPoint3 currentSecondAxisWorld = normalizeOrZero(seatLongAxis.secondAxisWorld);
                        if (seatRodShape) {
                            if (seatSwingAlignmentWanted) {
                                // Same hand-signed thumb-clearance tilt as the flight servo.
                                const float gripAxisTiltRadians = gripAxisTiltRadiansForHand(_isLeft);
                                RE::NiPoint3 targetAxisWorld = normalizeOrZero(
                                    pocket.crossPalmWorld * std::cos(gripAxisTiltRadians) +
                                    pocket.fingerForwardWorld * std::sin(gripAxisTiltRadians));
                                if (lengthSquared(currentAxisWorld) > 0.000001f && lengthSquared(targetAxisWorld) > 0.000001f) {
                                    if (dotProduct(currentAxisWorld, targetAxisWorld) < 0.0f) {
                                        targetAxisWorld = RE::NiPoint3{ -targetAxisWorld.x, -targetAxisWorld.y, -targetAxisWorld.z };
                                    }
                                    const RE::NiPoint3 rotationAxisRaw = crossProduct(currentAxisWorld, targetAxisWorld);
                                    const float sinAngle = std::sqrt((std::max)(0.0f, lengthSquared(rotationAxisRaw)));
                                    const float cosAngle = std::clamp(dotProduct(currentAxisWorld, targetAxisWorld), -1.0f, 1.0f);
                                    const float angleRadians = std::atan2(sinAngle, cosAngle);
                                    if (sinAngle > 0.000001f && angleRadians > 0.01f) {
                                        const float invSin = 1.0f / sinAngle;
                                        const RE::NiPoint3 rotationAxis{
                                            rotationAxisRaw.x * invSin,
                                            rotationAxisRaw.y * invSin,
                                            rotationAxisRaw.z * invSin,
                                        };
                                        seatBodyWorld = rotateTransformWorldAboutPoint(
                                            grabBodyWorldAtGrab, rotationAxis, angleRadians, grabGripPoint);
                                        seatObjectWorld = rotateTransformWorldAboutPoint(
                                            objectWorldTransform, rotationAxis, angleRadians, grabGripPoint);
                                        // Carry the mesh axes into the swung pose so the roll below runs in seat space.
                                        currentAxisWorld = grab_finger_pose_math::rotateAroundUnitAxis(
                                            currentAxisWorld, rotationAxis, angleRadians);
                                        currentSecondAxisWorld = grab_finger_pose_math::rotateAroundUnitAxis(
                                            currentSecondAxisWorld, rotationAxis, angleRadians);
                                        seatAlignmentAngleDegrees = angleRadians * 57.29577951308232f;
                                        seatAlignmentReason = "longAxisSeatAligned";
                                        seatPoseChanged = true;
                                    } else {
                                        seatAlignmentReason = "alreadyAligned";
                                    }
                                } else {
                                    seatAlignmentReason = "degenerateAxes";
                                }
                            }
                            if (seatRollAlignmentWanted) {
                                if (seatFlatFace &&
                                    lengthSquared(currentAxisWorld) > 0.000001f &&
                                    lengthSquared(currentSecondAxisWorld) > 0.000001f) {
                                    // Thinnest-extent direction = the flat face's normal.
                                    const RE::NiPoint3 faceNormalWorld = crossProduct(currentAxisWorld, currentSecondAxisWorld);
                                    const RE::NiPoint3 rollTargetRaw =
                                        pocket.palmNormalWorld - currentAxisWorld * dotProduct(pocket.palmNormalWorld, currentAxisWorld);
                                    const RE::NiPoint3 rollCurrentRaw =
                                        faceNormalWorld - currentAxisWorld * dotProduct(faceNormalWorld, currentAxisWorld);
                                    RE::NiPoint3 rollTargetWorld = normalizeOrZero(rollTargetRaw);
                                    const RE::NiPoint3 rollCurrentWorld = normalizeOrZero(rollCurrentRaw);
                                    if (lengthSquared(rollTargetWorld) > 0.000001f && lengthSquared(rollCurrentWorld) > 0.000001f) {
                                        if (dotProduct(rollCurrentWorld, rollTargetWorld) < 0.0f) {
                                            // A face has two sides; twist toward the nearer one.
                                            rollTargetWorld = RE::NiPoint3{ -rollTargetWorld.x, -rollTargetWorld.y, -rollTargetWorld.z };
                                        }
                                        const float rollSin = dotProduct(crossProduct(rollCurrentWorld, rollTargetWorld), currentAxisWorld);
                                        const float rollCos = std::clamp(dotProduct(rollCurrentWorld, rollTargetWorld), -1.0f, 1.0f);
                                        const float rollAngleRadians = std::atan2(rollSin, rollCos);
                                        if (std::fabs(rollAngleRadians) > 0.01f) {
                                            seatBodyWorld = rotateTransformWorldAboutPoint(
                                                seatBodyWorld, currentAxisWorld, rollAngleRadians, grabGripPoint);
                                            seatObjectWorld = rotateTransformWorldAboutPoint(
                                                seatObjectWorld, currentAxisWorld, rollAngleRadians, grabGripPoint);
                                            seatRollAngleDegrees = rollAngleRadians * 57.29577951308232f;
                                            seatRollReason = "faceRollSeatAligned";
                                            seatPoseChanged = true;
                                        } else {
                                            seatRollReason = "alreadyRollAligned";
                                        }
                                    } else {
                                        seatRollReason = "degenerateRollAxes";
                                    }
                                } else {
                                    seatRollReason = "belowSecondElongationGate";
                                }
                            }
                        } else if (seatLongAxis.valid) {
                            if (seatSwingAlignmentWanted) {
                                // Cross-palm swing is a rod correction only.
                                seatAlignmentReason = "belowElongationGate";
                            }
                            if (seatRollAlignmentWanted) {
                                /*
                                 * Plates deliberately get NO face alignment.
                                 * The intuition that a flat object should lie
                                 * face-down on the palm is simply wrong for
                                 * this hand: across 31 user-verified plate
                                 * holds (2026-07-25 ground-truth captures) the
                                 * mesh face normal sits 63 degrees off the palm
                                 * normal, IQR 57..72 - plates are held by an
                                 * EDGE or CORNER, and that is the intended
                                 * grip, not an artifact. A face swing fought
                                 * that preference on every plate, so the branch
                                 * was removed rather than retuned. The plate
                                 * CLASS is still resolved above because the
                                 * penetration backstop uses it.
                                 */
                                seatRollReason = seatPlateShape ? "plateEdgeHoldNoFaceAlign" : "belowSecondElongationGate";
                            }
                        }
                    }

                    /*
                     * Seat depth stop: the freeze below re-aligns the selected grip
                     * point exactly onto pivot A, which pulls any mesh behind the
                     * grip point through the palm. Push pivot A out along the palm
                     * normal by the mesh support depth so the object's surface rests
                     * ON the palm instead of its interior. Measured against the SEAT
                     * orientation (post-alignment), not the capture pose. This
                     * supersedes the old fixed-distance pulled-grab adjust, whose
                     * desired-world translation was erased by that same pivot
                     * re-alignment (alignLocalPointInTransformToLocalTarget) on
                     * every successful grab. Weapon attach frames and pinch pockets
                     * keep their own seat authority and are excluded.
                     */
                    GrabSeatDepthStopResult seatDepthStop{};
                    float seatDepthOffsetGameUnits = 0.0f;
                    float seatPenetrationBackstopGameUnits = 0.0f;
                    const char* seatPenetrationBackstopReason = "inactive";
                    if (!looseWeaponPrimaryAttachApplied && !usingPinchPocket && pocket.valid) {
                        seatDepthStop = computeGrabSeatDepthStop(
                            grabLocalMeshTriangles,
                            seatObjectWorld,
                            grabGripPoint,
                            pocket.palmNormalWorld,
                            g_rockConfig.rockGrabSeatDepthFootprintRadiusGameUnits,
                            g_rockConfig.rockGrabSeatDepthMaxGameUnits);
                        if (seatDepthStop.valid && seatDepthStop.depthGameUnits > 0.01f) {
                            seatDepthOffsetGameUnits =
                                seatDepthStop.depthGameUnits + (std::max)(0.0f, g_rockConfig.rockGrabSeatDepthSkinGameUnits);
                            grabPivotAWorld = grabPivotAWorld + pocket.palmNormalWorld * seatDepthOffsetGameUnits;
                            seatPoseChanged = true;
                        }
                        if (seatPoseChanged) {
                            desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(
                                seatBodyWorld,
                                grabPivotAWorld,
                                grabGripPoint);
                            desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                        }
                        /*
                         * Fail-closed palm-plane backstop: the depth stop above
                         * measures from the GRIP POINT and rejects mesh at
                         * negative grip-relative depth, so a grip seed at or
                         * behind the surface (inset grips, concave shells) or
                         * an empty footprint leaves the correction at zero and
                         * commits the hand inside the object. Re-measure on the
                         * FINAL seat pose from the palm plane itself: any mesh
                         * still reaching past the palm plane within the same
                         * tuned footprint is penetration by definition. One
                         * rigid push restores the surface-on-palm invariant
                         * exactly (a translation reduces every support depth by
                         * the push, so no iteration is needed). Engagement is
                         * always WARN-logged: it means the primary depth stop
                         * failed, and the root cause still needs the capture
                         * line's seatDepthReason evidence.
                         *
                         * Footprint width is shape-selected. The primary stop's
                         * radius is tuned small for seating precision, which
                         * bounds how much penetration a TILTED face can even
                         * express: inside radius r a face tilted by theta shows
                         * at most r*sin(theta), so a large plate cutting deep
                         * through the fingers registers a fraction of a unit at
                         * the palm axis and commits. Plates - where the near
                         * surface is flat and one rigid push-out is therefore
                         * exactly right everywhere - get a hand-sized footprint
                         * instead. Every other shape keeps the tuned radius:
                         * widening it for irregular geometry would let lobes
                         * that merely pass BESIDE the palm start pushing seats
                         * out (mug bodies, shell rims) and reintroduce the
                         * floaty seats that radius was tuned to remove.
                         */
                        const float backstopFootprintRadiusGameUnits =
                            seatPlateShape ? g_rockConfig.rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits
                                           : g_rockConfig.rockGrabSeatDepthFootprintRadiusGameUnits;
                        const auto palmPlaneOvershoot = computeGrabSeatDepthStop(
                            grabLocalMeshTriangles,
                            desiredObjectWorld,
                            pocket.palmCenterWorld,
                            pocket.palmNormalWorld,
                            backstopFootprintRadiusGameUnits,
                            g_rockConfig.rockGrabSeatDepthMaxGameUnits);
                        seatPenetrationBackstopReason = palmPlaneOvershoot.reason;
                        constexpr float kSeatPenetrationBackstopThresholdGameUnits = 1.0f;
                        if (palmPlaneOvershoot.valid &&
                            palmPlaneOvershoot.depthGameUnits > kSeatPenetrationBackstopThresholdGameUnits) {
                            seatPenetrationBackstopGameUnits =
                                palmPlaneOvershoot.depthGameUnits + (std::max)(0.0f, g_rockConfig.rockGrabSeatDepthSkinGameUnits);
                            grabPivotAWorld = grabPivotAWorld + pocket.palmNormalWorld * seatPenetrationBackstopGameUnits;
                            desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(
                                seatBodyWorld,
                                grabPivotAWorld,
                                grabGripPoint);
                            desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                            seatPoseChanged = true;
                            seatPenetrationBackstopReason = "palmPlanePenetrationPushedOut";
                            ROCK_LOG_WARN(Hand,
                                "{} SEAT DEPTH BACKSTOP: final seat still reached {:.2f}gu past the palm plane -> pushed out {:.2f}gu "
                                "(shape={} footprint={:.1f} primary seatDepthReason={} samples={} depth={:.2f} offset={:.2f} overshootSamples={})",
                                handName(),
                                palmPlaneOvershoot.depthGameUnits,
                                seatPenetrationBackstopGameUnits,
                                seatPlateShape ? "plate" : (seatRodShape ? "rod" : "compact"),
                                backstopFootprintRadiusGameUnits,
                                seatDepthStop.reason,
                                seatDepthStop.footprintSampleCount,
                                seatDepthStop.depthGameUnits,
                                seatDepthOffsetGameUnits,
                                palmPlaneOvershoot.footprintSampleCount);
                        }
                    }

                    /*
                     * Pinch seat centering: the freeze puts the pinch SURFACE hit
                     * on the pocket point, which parks the object's near face at
                     * the pocket and shifts its body toward one finger pad by its
                     * full local thickness. Measure the mesh extents both ways
                     * along the pinch axis from the grip point (small footprint -
                     * only the material actually between the pads matters) and
                     * offset pivot A so the object's MID-THICKNESS sits exactly at
                     * the pocket middle. Same pivot-A mechanism as the depth stop;
                     * the correction is zero for a surface hit already centered.
                     */
                    float pinchCenterOffsetGameUnits = 0.0f;
                    if (usingPinchPocket && !looseWeaponPrimaryAttachApplied) {
                        const RE::NiPoint3 pinchAxisWorld = normalizeOrZero(pinchPocketCandidate.pinchAxisWorld);
                        if (lengthSquared(pinchAxisWorld) > 0.000001f) {
                            // Finger-pad scale; pinch objects are small by classification.
                            constexpr float kPinchCenterFootprintRadiusGameUnits = 2.5f;
                            constexpr float kPinchCenterMaxExtentGameUnits = 8.0f;
                            const auto extentTowardIndex = computeGrabSeatDepthStop(
                                grabLocalMeshTriangles,
                                objectWorldTransform,
                                grabGripPoint,
                                RE::NiPoint3{ -pinchAxisWorld.x, -pinchAxisWorld.y, -pinchAxisWorld.z },
                                kPinchCenterFootprintRadiusGameUnits,
                                kPinchCenterMaxExtentGameUnits);
                            const auto extentTowardThumb = computeGrabSeatDepthStop(
                                grabLocalMeshTriangles,
                                objectWorldTransform,
                                grabGripPoint,
                                pinchAxisWorld,
                                kPinchCenterFootprintRadiusGameUnits,
                                kPinchCenterMaxExtentGameUnits);
                            if (extentTowardIndex.valid && extentTowardThumb.valid) {
                                pinchCenterOffsetGameUnits =
                                    (extentTowardIndex.depthGameUnits - extentTowardThumb.depthGameUnits) * 0.5f;
                                if (std::fabs(pinchCenterOffsetGameUnits) > 0.05f) {
                                    grabPivotAWorld = grabPivotAWorld - pinchAxisWorld * pinchCenterOffsetGameUnits;
                                    desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(
                                        grabBodyWorldAtGrab,
                                        grabPivotAWorld,
                                        grabGripPoint);
                                    desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                                }
                            }
                        }
                    }

                    selectedGripPointLocal = transform_math::worldPointToLocal(objectWorldTransform, grabGripPoint);
                    selectedPivotBBodyLocalGame = transform_math::worldPointToLocal(grabBodyWorldAtGrab, grabGripPoint);
                    const bool effectivePinchPocket = usingPinchPocket && !looseWeaponPrimaryAttachApplied;
                    RE::NiPoint3 supportFrameNormalWorld = normalizeOrZero(gripNormalWorld);
                    if (lengthSquared(supportFrameNormalWorld) <= 0.000001f) {
                        supportFrameNormalWorld = firstValidNormal({
                            gripEvidenceNormalWorld,
                            gripSupportRuntime.model.supportNormal,
                            pocket.palmNormalWorld,
                        });
                    }
                    const RE::NiPoint3 supportFrameAxisWorld = resolveSupportFrameAxisWorld(supportFrameNormalWorld,
                        gripSupportRuntime.model.supportAxis,
                        usingPinchPocket ? pinchPocketCandidate.pinchAxisWorld : RE::NiPoint3{},
                        pocket.valid ? pocket.crossPalmWorld : RE::NiPoint3{},
                        pocket.valid ? pocket.fingerForwardWorld : RE::NiPoint3{});
                    const RE::NiPoint3 supportFrameBinormalWorld = normalizeOrZero(crossProduct(supportFrameNormalWorld, supportFrameAxisWorld));
                    /*
                     * Seat diagnostics: the classification and every
                     * correction outcome, kept past the log line so a saved
                     * ground-truth capture can record the seat ROCK produced
                     * for this grab next to the pose the user corrected it to.
                     */
                    _grabFrame.heldNode = collidableNode;
                    _grabFrame.localMeshTriangles = grabLocalMeshTriangles;
                    _grabFrame.fingerPoseLocalMeshTriangles = grabFingerPoseLocalMeshTriangles;
                    _grabFrame.hasMeshPoseData =
                        !grabLocalMeshTriangles.empty() || !grabFingerPoseLocalMeshTriangles.empty();
                    _grabFrame.bodyResolutionReason = primaryBodyChoiceReasonName(primaryChoice.reason);

                    _grabFrame.seatDiagnostics = GrabSeatDiagnostics{
                        .acquisitionMode = sel.forcedArrival ? "forceGrab" : (grabbedFromPullCatch ? "pullCatch" : "closeGrab"),
                        .shapeClass = seatPlateShape ? "plate" : (seatRodShape ? "rod" : (seatLongAxis.valid ? "compact" : "none")),
                        .elongationRatio = seatLongAxis.elongationRatio,
                        .secondElongationRatio = seatLongAxis.secondElongationRatio,
                        .alignmentAngleDegrees = seatAlignmentAngleDegrees,
                        .alignmentReason = seatAlignmentReason,
                        .rollAngleDegrees = seatRollAngleDegrees,
                        .rollReason = seatRollReason,
                        .depthGameUnits = seatDepthStop.depthGameUnits,
                        .depthOffsetGameUnits = seatDepthOffsetGameUnits,
                        .depthReason = seatDepthStop.reason,
                        .penetrationBackstopGameUnits = seatPenetrationBackstopGameUnits,
                        .penetrationBackstopReason = seatPenetrationBackstopReason,
                    };
                    _grabFrame.gripEvidenceLocal = selectedGripPointLocal;
                    _grabFrame.gripNormalLocal = transform_math::worldVectorToLocal(objectWorldTransform, gripNormalWorld);
                    _grabFrame.gripEvidenceTriangleIndex =
                        !looseWeaponPrimaryAttachApplied && grabSurfaceHit.valid && grabSurfaceHit.hasTriangle ?
                        static_cast<std::uint32_t>(grabSurfaceHit.triangleIndex) :
                        0xFFFF'FFFF;
                    _grabFrame.gripEvidenceShapeKey =
                        !looseWeaponPrimaryAttachApplied && grabSurfaceHit.valid ? grabSurfaceHit.shapeKey : 0xFFFF'FFFF;
                    _grabFrame.gripEvidenceShapeCollisionFilterInfo =
                        !looseWeaponPrimaryAttachApplied && grabSurfaceHit.valid ? grabSurfaceHit.shapeCollisionFilterInfo : 0;
                    _grabFrame.gripEvidenceHitFraction =
                        !looseWeaponPrimaryAttachApplied && grabSurfaceHit.valid ? grabSurfaceHit.hitFraction : 1.0f;
                    _grabFrame.hasGripEvidenceShapeKey =
                        !looseWeaponPrimaryAttachApplied && grabSurfaceHit.valid && grabSurfaceHit.hasShapeKey;
                    if (looseWeaponPrimaryAttachApplied) {
                        _grabFrame.gripSourceNode = nullptr;
                        _grabFrame.gripPointSourceNodeLocal = {};
                        _grabFrame.gripNormalSourceNodeLocal = {};
                        _grabFrame.hasGripSourceNodePoint = false;
                        _grabFrame.hasGripSourceNodeNormal = false;
                    } else {
                        storeGripSourceEvidence(_grabFrame,
                            grabSurfaceHit.sourceNode ? grabSurfaceHit.sourceNode : collidableNode,
                            objectWorldTransform,
                            gripEvidencePointWorld,
                            gripEvidenceNormalWorld,
                            lengthSquared(gripEvidenceNormalWorld) > 0.000001f);
                    }

                    _grabFrame.pocketToGripDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                    _grabFrame.selectionToGripEvidenceDistanceGameUnits =
                        sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, gripArea.contactSeedWorld) : std::numeric_limits<float>::max();
                    if (grabSurfaceHit.valid) {
                        grabSurfaceHit.pivotToSurfaceDistanceGameUnits = _grabFrame.pocketToGripDistanceGameUnits;
                    }
                    _grabFrame.grabPivotWorldAtGrab = grabPivotAWorld;
                    _grabFrame.gripPointWorldAtGrab = grabGripPoint;
                    _grabFrame.activeGrabPointMode = grabPointMode;
                    _grabFrame.seatMode =
                        !looseWeaponPrimaryAttachApplied && usingPinchPocket ? GrabSeatMode::PinchPocket : GrabSeatMode::SupportGroup;
                    _grabFrame.hasPinchPocket = effectivePinchPocket;
                    _grabFrame.pinchPocketWorldAtGrab = effectivePinchPocket ? pinchPocketCandidate.pinchPocketWorld : RE::NiPoint3{};
                    _grabFrame.pinchAxisWorldAtGrab = effectivePinchPocket ? pinchPocketCandidate.pinchAxisWorld : RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
                    _grabFrame.palmSeatPointWorldAtGrab = effectivePinchPocket ? pinchPocketCandidate.pinchPocketWorld : pocket.palmCenterWorld;
                    _grabFrame.fingerEvidencePointWorldAtGrab =
                        looseWeaponPrimaryAttachApplied || effectivePinchPocket ? grabGripPoint : fingerEvidencePointWorld;
                    _grabFrame.hasPalmSeatPoint = true;
                    _grabFrame.hasFingerEvidencePoint =
                        looseWeaponPrimaryAttachApplied || effectivePinchPocket || fingerEvidencePointValid;
                    _grabFrame.palmSeatPointMode = effectivePinchPocket ? "pinchPocket" : "threePhasePocket";
                    _grabFrame.fingerEvidencePointMode = looseWeaponPrimaryAttachApplied ?
                        "looseWeaponPrimaryAttach" :
                        (effectivePinchPocket ? "pinchThumbIndex" : "evidenceOnly");
                    _grabFrame.activeGrabPointUsesMultiFingerEvidence = false;
                    _grabFrame.pivotAuthoritySource = pivotAuthoritySource;
                    _grabFrame.pivotAuthorityPositionOnly = pivotAuthorityPositionOnly;
                    _grabFrame.pivotAuthorityNormalTrusted = pivotAuthorityNormalTrusted;
                    _grabFrame.pivotAuthorityPositionConfidence = pivotAuthorityPositionConfidence;
                    _grabFrame.syntheticLooseWeaponPrimaryAttach = looseWeaponPrimaryAttachApplied;
                    _grabFrame.hasSettledVisualHandRelation = false;
                    _grabFrame.hasGripSupportModel = looseWeaponPrimaryAttachApplied ? false : gripSupportRuntime.valid;
                    _grabFrame.supportFrameNormalBodyLocal = transform_math::worldVectorToLocal(grabBodyWorldAtGrab, supportFrameNormalWorld);
                    _grabFrame.supportFrameAxisBodyLocal = transform_math::worldVectorToLocal(grabBodyWorldAtGrab, supportFrameAxisWorld);
                    _grabFrame.supportFrameBinormalBodyLocal = transform_math::worldVectorToLocal(grabBodyWorldAtGrab, supportFrameBinormalWorld);
                    _grabFrame.hasSupportFrameNormal = lengthSquared(supportFrameNormalWorld) > 0.000001f;
                    _grabFrame.hasSupportFrameAxis = lengthSquared(supportFrameAxisWorld) > 0.000001f;
                    _grabFrame.hasSupportFrameBinormal = lengthSquared(supportFrameBinormalWorld) > 0.000001f;
                    _grabFrame.gripSupportAuthoredPivot = looseWeaponPrimaryAttachApplied ? false : gripSupportRuntime.model.canAuthorPivot;
                    _grabFrame.gripSupportKind = looseWeaponPrimaryAttachApplied ? grab_support_model_math::GripSupportKind::None : gripSupportRuntime.model.kind;
                    _grabFrame.gripSupportReason = looseWeaponPrimaryAttachApplied ? looseWeaponPrimaryAttachReason : gripSupportRuntime.model.reason;
                    _grabFrame.gripSupportConfidence = looseWeaponPrimaryAttachApplied ? 0.0f : gripSupportRuntime.model.confidence;
                    _grabFrame.gripSupportSpanGameUnits = looseWeaponPrimaryAttachApplied ? 0.0f : gripSupportRuntime.model.supportSpanGameUnits;
                    _grabFrame.gripSupportPivotShiftGameUnits = looseWeaponPrimaryAttachApplied ? 0.0f : gripSupportRuntime.model.pivotShiftGameUnits;
                    _grabFrame.requiresSettledVisualHandRelation = looseWeaponPrimaryAttachApplied ?
                        true :
                        (effectivePinchPocket ?
                                false :
                                (pullCatchSeatSafety.requireSettledVisualRelation ||
                                    pivotAuthorityRequiresSettledVisualRelation(
                                        pivotAuthoritySource,
                                        pivotAuthorityPositionOnly,
                                        _grabAcquisitionPhase)));
                    _grabFrame.fingerPoseAimValid = true;
                    _grabFrame.fingerPoseAimReason = effectivePinchPocket ? "pinchPocketThumbIndexTargets" : "rockPointToPalmEvidence";

                    // Final evidence values are known only after the seat solve.
                    const bool keepContactPatch =
                        !looseWeaponPrimaryAttachApplied && !effectivePinchPocket && contactPatchEvidenceAvailable;
                    decltype(_grabFrame.contactPatchSamples) finalContactPatchSamples{};
                    std::uint32_t finalContactPatchSampleCount = 0;
                    if (keepContactPatch) {
                        finalContactPatchSampleCount = (std::min)(
                            contactPatchRuntime.sampleCount,
                            static_cast<std::uint32_t>(finalContactPatchSamples.size()));
                        for (std::uint32_t i = 0; i < finalContactPatchSampleCount; ++i) {
                            auto sample = contactPatchRuntime.samples[i];
                            sample.point = transform_math::worldPointToLocal(grabBodyWorldAtGrab, sample.point);
                            sample.normal = transform_math::worldVectorToLocal(grabBodyWorldAtGrab, sample.normal);
                            finalContactPatchSamples[i] = sample;
                        }
                    }
                    _grabFrame.contactPatchSamples = finalContactPatchSamples;
                    _grabFrame.contactPatchSampleCount = finalContactPatchSampleCount;
                    _grabFrame.hasContactPatch = keepContactPatch;
                    _grabFrame.hasContactPatchEvidence = keepContactPatch;
                    _grabFrame.contactPatchMeshSnapDeltaGameUnits =
                        !looseWeaponPrimaryAttachApplied && !effectivePinchPocket ?
                        contactPatchRuntime.patch.meshSnapDeltaGameUnits :
                        0.0f;
                    const bool keepMultiFingerPatch =
                        !looseWeaponPrimaryAttachApplied && (effectivePinchPocket || multiFingerGripUsed);
                    _grabFrame.hasMultiFingerContactPatch = keepMultiFingerPatch;
                    _grabFrame.multiFingerContactGroupCount = effectivePinchPocket ?
                        2u :
                        (looseWeaponPrimaryAttachApplied ? 0u : multiFingerGripRuntime.gripSet.groupCount);
                    _grabFrame.multiFingerContactReason = looseWeaponPrimaryAttachApplied ?
                        "looseWeaponPrimaryAttach" :
                        (effectivePinchPocket ?
                                "pinchPocketThumbIndex" :
                                (multiFingerGripRuntime.reason ? multiFingerGripRuntime.reason : "none"));
                    _grabFrame.multiFingerContactSpreadGameUnits = effectivePinchPocket ?
                        pinchPocketCandidate.thumbIndexGapGameUnits :
                        (looseWeaponPrimaryAttachApplied ? 0.0f : multiFingerGripRuntime.gripSet.spreadGameUnits);
                    _grabFrame.multiFingerGripCenterWorldAtGrab = effectivePinchPocket ?
                        grabGripPoint :
                        (looseWeaponPrimaryAttachApplied ? RE::NiPoint3{} : multiFingerGripRuntime.gripSet.contactCenterWorld);
                    _grabFrame.multiFingerHandCenterWorldAtGrab = effectivePinchPocket ?
                        pinchPocketCandidate.pinchPocketWorld :
                        (looseWeaponPrimaryAttachApplied ? RE::NiPoint3{} : multiFingerGripRuntime.gripSet.handCenterWorld);
                    _grabFrame.multiFingerAverageNormalWorldAtGrab = effectivePinchPocket ?
                        gripNormalWorld :
                        (looseWeaponPrimaryAttachApplied ? RE::NiPoint3{} : multiFingerGripRuntime.gripSet.averageNormalWorld);
                    /*
                     * desiredObjectWorld is already the final frozen seat even
                     * while the live body is still converging. Surface aim is
                     * therefore valid at commit; waiting for live touch was the
                     * source of the visible post-grab correction.
                     */
                    const bool fullHeldAuthorityAtCapture =
                        _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld;
                    const auto captureFingerTargets = effectivePinchPocket ?
                        buildRuntimePinchFingerPoseTargets(pinchPocketCandidate) :
                        buildRuntimeFingerPoseTargets(gripEvidencePointWorld, gripEvidenceNormalWorld);
                    storeFingerPoseTargetsInGrabFrame(_grabFrame, captureFingerTargets, objectWorldTransform);

                    ROCK_LOG_DEBUG(Hand,
                        "{} THREE-PHASE GRAB CAPTURE: relation={} seat={} rotation={} phase={} reason={} touchContact={} stableTouch={} pocket=({:.1f},{:.1f},{:.1f}) "
                        "palm=({:.1f},{:.1f},{:.1f}) normal=({:.3f},{:.3f},{:.3f}) seed=({:.1f},{:.1f},{:.1f}) "
                        "grip=({:.1f},{:.1f},{:.1f}) gripLocal=({:.2f},{:.2f},{:.2f}) pivotB=({:.2f},{:.2f},{:.2f}) dist={:.1f} signedPalm={:.1f} "
                        "fullHeldAuthority={} pivotAuthoritySource={} positionOnlyPatch={} normalTrusted={} support={} supportPivot={} supportConfidence={:.2f} supportSpan={:.2f} supportShift={:.2f} supportReason={} supportSamples={} supportMeshHits={} supportRejectOwner={} supportRejectDistance={} settledVisualRequired={} pullSeatSafety={} pullSeatDot={:.3f} pullSeatSigned={:.1f} pullSeatDist={:.1f} seatDepth={:.2f} seatDepthOffset={:.2f} seatDepthSamples={} seatDepthReason={} seatAlignDeg={:.1f} seatAlignReason={} seatRollDeg={:.1f} seatRollReason={} seatShape={} seatRatio12={:.2f} seatRatio23={:.2f} seatBackstop={:.2f} seatBackstopReason={} pinchCenter={:.2f} inset={:.2f} insetSource={} looseWeaponPrimaryAttach={} attachReason={} attachVisible={}",
                        handName(),
                        relationMode,
                        grabSeatModeName(_grabFrame.seatMode),
                        looseWeaponPrimaryAttachApplied ? "primaryWeaponAttach" : "preserve",
                        grab_three_phase::phaseName(_grabAcquisitionPhase),
                        captureReason,
                        semanticContacts.count > 0 ? "yes" : "no",
                        hasStablePocketTouchContact ? "yes" : "no",
                        pocket.pocketCenterWorld.x,
                        pocket.pocketCenterWorld.y,
                        pocket.pocketCenterWorld.z,
                        pocket.palmCenterWorld.x,
                        pocket.palmCenterWorld.y,
                        pocket.palmCenterWorld.z,
                        gripNormalWorld.x,
                        gripNormalWorld.y,
                        gripNormalWorld.z,
                        gripArea.contactSeedWorld.x,
                        gripArea.contactSeedWorld.y,
                        gripArea.contactSeedWorld.z,
                        grabGripPoint.x,
                        grabGripPoint.y,
                        grabGripPoint.z,
                        selectedGripPointLocal.x,
                        selectedGripPointLocal.y,
                        selectedGripPointLocal.z,
                        selectedPivotBBodyLocalGame.x,
                        selectedPivotBBodyLocalGame.y,
                        selectedPivotBBodyLocalGame.z,
                        usingPinchPocket ? pinchPocketCandidate.pocketToSurfaceDistanceGameUnits : finalGripToPocketDistance,
                        usingPinchPocket ? 0.0f : finalSignedPalmDistance,
                        fullHeldAuthorityAtCapture ? "yes" : "no",
                        _grabFrame.pivotAuthoritySource,
                        _grabFrame.pivotAuthorityPositionOnly ? "yes" : "no",
                        _grabFrame.pivotAuthorityNormalTrusted ? "yes" : "no",
                        grab_support_model_math::gripSupportKindName(_grabFrame.gripSupportKind),
                        _grabFrame.gripSupportAuthoredPivot ? "yes" : "no",
                        _grabFrame.gripSupportConfidence,
                        _grabFrame.gripSupportSpanGameUnits,
                        _grabFrame.gripSupportPivotShiftGameUnits,
                        _grabFrame.gripSupportReason,
                        gripSupportRuntime.sampleCount,
                        gripSupportRuntime.meshProbeHitCount,
                        gripSupportRuntime.rejectedOwnerCount,
                        gripSupportRuntime.rejectedDistanceCount,
                        _grabFrame.requiresSettledVisualHandRelation ? "yes" : "no",
                        pullCatchSeatSafety.reason,
                        pullCatchSeatSafety.normalDotPalm,
                        finalSignedPalmDistance,
                        finalGripToPocketDistance,
                        seatDepthStop.depthGameUnits,
                        seatDepthOffsetGameUnits,
                        seatDepthStop.footprintSampleCount,
                        seatDepthStop.reason,
                        seatAlignmentAngleDegrees,
                        seatAlignmentReason,
                        seatRollAngleDegrees,
                        seatRollReason,
                        seatPlateShape ? "plate" : (seatRodShape ? "rod" : (seatLongAxis.valid ? "compact" : "none")),
                        seatLongAxis.elongationRatio,
                        seatLongAxis.secondElongationRatio,
                        seatPenetrationBackstopGameUnits,
                        seatPenetrationBackstopReason,
                        pinchCenterOffsetGameUnits,
                        gripArea.seedInsetGameUnits,
                        gripArea.fallbackReason,
                        looseWeaponPrimaryAttachApplied ? "yes" : "no",
                        looseWeaponPrimaryAttachReason,
                        looseWeaponPrimaryAttachSourceVisible ? "yes" : "no");
                } else {
                    ROCK_LOG_WARN(Hand,
                        "{} THREE-PHASE GRAB ABORT: pocketValid={} gripValid={} accepted={} reason={} dist={:.2f}gu stableTouch={}",
                        handName(),
                        pocket.valid ? "yes" : "no",
                        gripArea.valid ? "yes" : "no",
                        phaseDecision.accepted ? "yes" : "no",
                        phaseDecision.reason,
                        gripToPocketDistance,
                        hasStablePocketTouchContact ? "yes" : "no");
                    return abortGrab();
                }
            }

            // ---- Stage 3: freeze ---------------------------------------------
            // The seat is decided. Lock the authority frame and populate
            // _grabFrame from it. Every _grabFrame field is written once, here,
            // with its final value.
            logGrabNodeInfo(handName(),
                _isLeft,
                collidableNode,
                authoredGrabNode,
                desiredObjectWorld,
                handWorldTransform,
                grabPivotAWorld,
                grabPointMode);

            const RE::NiPoint3 frozenVisualNormalWorld = gripEvidenceNormalWorld(_grabFrame, objectWorldTransform);
            const auto frozenAuthorityFrame = grab_authority_frame_math::freezeGrabAuthorityFrame<RE::NiTransform>(
                grab_authority_frame_math::GrabAuthorityFrameFreezeInput<RE::NiTransform>{
                    .rawHandWorld = handWorldTransform,
                    .proxyWorld = proxyFrameWorldAtGrab,
                    .proxyAuthorityFrameWorld = proxyAuthorityFrameWorldAtGrab,
                    .objectWorld = objectWorldTransform,
                    .bodyWorld = grabBodyWorldAtGrab,
                    .constraintBodyWorld = constraintBodyWorldAtGrab,
                    .rootBodyLocal = rootBodyLocalAtGrab,
                    .ownerBodyLocal = ownerBodyLocalAtGrab,
                    .desiredObjectWorld = desiredObjectWorld,
                    .desiredBodyWorld = desiredBodyWorld,
                    .pivotAWorld = grabPivotAWorld,
                    .gripPointWorld = grabGripPoint,
                    .visualNormalWorld = frozenVisualNormalWorld,
                    .source = resolvedAuthorityPivotSourceForFreeze,
                    .hasDesiredObjectWorld = true,
                    .hasDesiredBodyWorld = true,
                    .visualNormalValid = lengthSquared(frozenVisualNormalWorld) > 0.000001f,
                });
            if (!frozenAuthorityFrame.valid) {
                ROCK_LOG_ERROR(Hand,
                    "{} GRAB FAILED: unable to freeze coherent authority frame point=({:.2f},{:.2f},{:.2f}) pivotA=({:.2f},{:.2f},{:.2f}) mode={} bodyId={}",
                    handName(),
                    grabGripPoint.x,
                    grabGripPoint.y,
                    grabGripPoint.z,
                    grabPivotAWorld.x,
                    grabPivotAWorld.y,
                    grabPivotAWorld.z,
                    grabPointMode,
                    objectBodyId.value);
                return abortGrab();
            }
            applyFrozenGrabAuthorityFrameToGrabFrame(_grabFrame, frozenAuthorityFrame);
            desiredObjectWorld = frozenAuthorityFrame.desiredObjectWorld;
            desiredBodyWorld = frozenAuthorityFrame.desiredBodyWorld;
            const float objectScaleForLever =
                std::isfinite(objectWorldTransform.scale) && objectWorldTransform.scale > 0.0f ? objectWorldTransform.scale : 1.0f;
            _grabFrame.longObjectLeverGameUnits =
                computeLocalMeshMaxDistanceFromPoint(_grabFrame.localMeshTriangles, _grabFrame.gripPointLocal) * objectScaleForLever;
            _grabFrame.liveHandWorldAtGrab = handWorldTransform;
            _grabFrame.handBodyWorldAtGrab = proxyFrameWorldAtGrab;
            _grabFrame.objectNodeWorldAtGrab = objectWorldTransform;
            _grabFrame.hasTelemetryCapture = true;
            _grabFrame.handScaleAtGrab = handWorldTransform.scale;
            _grabFrame.traceId = grabTraceId;
            _grabFrame.traceTargetWriteSequence = 0;
            _grabFrame.freezeCaptureTelemetry(objectBodyId.value);
            if (grabTimelineTraceEnabled()) {
                std::array<float, 12> traceTransformBRotation{};
                std::array<float, 4> traceTransformBTranslation{};
                std::array<float, 12> traceTargetBRca{};
                grab_constraint_math::writeGrabConstraintCreationAtoms(
                    traceTransformBRotation.data(),
                    traceTransformBTranslation.data(),
                    traceTargetBRca.data(),
                    frozenAuthorityFrame.proxyAuthorityBodyHandSpace,
                    frozenAuthorityFrame.pivotAHandBodyLocalGame,
                    gameToHavokScale());
                const RE::NiTransform traceDesiredBodyToHandSpace =
                    invertTransform(frozenAuthorityFrame.proxyAuthorityBodyHandSpace);
                const RE::NiMatrix3 traceTargetRows = matrixFromHkRows(traceTargetBRca.data());
                const RE::NiMatrix3 traceTransformBColumns = matrixFromHkColumns(traceTransformBRotation.data());
                const RE::NiPoint3 traceRelationPivotB =
                    grab_constraint_math::computeDynamicTransformBTranslationGame(
                        frozenAuthorityFrame.proxyAuthorityBodyHandSpace,
                        frozenAuthorityFrame.pivotAHandBodyLocalGame);
                const RE::NiPoint3 traceTransformBTranslationGame{
                    traceTransformBTranslation[0] * havokToGameScale(),
                    traceTransformBTranslation[1] * havokToGameScale(),
                    traceTransformBTranslation[2] * havokToGameScale(),
                };
                const float traceSelectedPivotRelationDeltaGameUnits =
                    pointDistanceGameUnits(frozenAuthorityFrame.pivotBConstraintLocalGame, traceRelationPivotB);
                const float tracePivotBRelationDeltaGameUnits =
                    pointDistanceGameUnits(traceTransformBTranslationGame, traceRelationPivotB);
                const float traceTargetToHiggsRelationDegrees =
                    rotationDeltaDegrees(traceTargetRows, traceDesiredBodyToHandSpace.rotate);
                const float traceTransformBFrozenDeltaDegrees =
                    rotationDeltaDegrees(traceTransformBColumns, traceDesiredBodyToHandSpace.rotate);
                const float traceRawToProxyRotDegrees =
                    rotationDeltaDegrees(handWorldTransform.rotate, proxyFrameWorldAtGrab.rotate);
                const float traceRawToProxyMaxAxisDegrees =
                    maxColumnAxisDeltaDegrees(handWorldTransform.rotate, proxyFrameWorldAtGrab.rotate);
                const float traceObjectToDesiredMaxAxisDegrees =
                    maxColumnAxisDeltaDegrees(objectWorldTransform.rotate, frozenAuthorityFrame.desiredObjectWorld.rotate);
                const float traceDesiredBodyToGrabBodyMaxAxisDegrees =
                    maxColumnAxisDeltaDegrees(frozenAuthorityFrame.desiredBodyWorld.rotate, grabBodyWorldAtGrab.rotate);
                const RE::NiPoint3 tracePivotBeforeFreeze =
                    transform_math::localPointToWorld(grabBodyWorldAtGrab, frozenAuthorityFrame.pivotBConstraintLocalGame);
                const RE::NiPoint3 tracePivotAfterFreeze =
                    transform_math::localPointToWorld(frozenAuthorityFrame.desiredBodyWorld, frozenAuthorityFrame.pivotBConstraintLocalGame);
                const RE::NiPoint3 traceFreezeShift =
                    frozenAuthorityFrame.desiredBodyWorld.translate - grabBodyWorldAtGrab.translate;
                const RE::NiPoint3 traceExpectedShift =
                    frozenAuthorityFrame.pivotAWorld - frozenAuthorityFrame.gripPointWorldAtGrab;
                const float traceFreezeShiftLength = vectorMagnitude(traceFreezeShift);
                const float traceExpectedShiftLength = vectorMagnitude(traceExpectedShift);
                const float traceFreezeShiftDot =
                    traceFreezeShiftLength > 0.0001f && traceExpectedShiftLength > 0.0001f ?
                        std::clamp(dotProduct(traceFreezeShift, traceExpectedShift) / (traceFreezeShiftLength * traceExpectedShiftLength), -1.0f, 1.0f) :
                        0.0f;
                const float tracePivotGapBeforeFreeze =
                    pointDistanceGameUnits(tracePivotBeforeFreeze, frozenAuthorityFrame.pivotAWorld);
                const float tracePivotGapAfterFreeze =
                    pointDistanceGameUnits(tracePivotAfterFreeze, frozenAuthorityFrame.pivotAWorld);
                const float traceBodyShiftDegrees =
                    rotationDeltaDegrees(frozenAuthorityFrame.desiredBodyWorld.rotate, grabBodyWorldAtGrab.rotate);
                const auto traceRawBasis = grab_transform_telemetry::makeOrientationBasis(handWorldTransform);
                const auto traceProxyBasis = grab_transform_telemetry::makeOrientationBasis(proxyFrameWorldAtGrab);
                const auto traceBodyBasis = grab_transform_telemetry::makeOrientationBasis(grabBodyWorldAtGrab);
                const auto traceDesiredBodyBasis = grab_transform_telemetry::makeOrientationBasis(frozenAuthorityFrame.desiredBodyWorld);

                ROCK_LOG_INFO(Hand,
                    "{} GRAB_TRACE stage=capture trace={} hand={} formID={:08X} name='{}' body={} mode={} pivotAuthority={} frozenSource={} resolverSource={} resolverReason={} seat={} phase={} pinch={} gripSupport={} supportPivot={} fullHeld={} settledReq={} shapeKey=0x{:08X} triangle=0x{:08X} pivotA=({:.2f},{:.2f},{:.2f}) grip=({:.2f},{:.2f},{:.2f}) pivotBBody=({:.2f},{:.2f},{:.2f}) pivotBSelected=({:.2f},{:.2f},{:.2f}) relationPivotB=({:.2f},{:.2f},{:.2f}) selectedPivotRelationDelta={:.3f}gu pivotBRelationDelta={:.3f}gu pocket={:.2f}gu selection={:.2f}gu longLever={:.2f}gu positionOnly={} normalTrusted={} confidence={:.2f}",
                    handName(),
                    _grabFrame.traceId,
                    _isLeft ? "left" : "right",
                    sel.refr ? sel.refr->GetFormID() : 0,
                    objName,
                    objectBodyId.value,
                    _grabFrame.activeGrabPointMode ? _grabFrame.activeGrabPointMode : "none",
                    _grabFrame.pivotAuthoritySource ? _grabFrame.pivotAuthoritySource : "none",
                    grab_authority_frame_math::grabAuthorityPivotSourceName(frozenAuthorityFrame.source),
                    grab_authority_frame_math::grabAuthorityPivotSourceName(resolvedAuthorityPivotSourceForFreeze),
                    resolvedAuthorityPivotReasonForFreeze ? resolvedAuthorityPivotReasonForFreeze : "none",
                    grabSeatModeName(_grabFrame.seatMode),
                    grab_three_phase::phaseName(_grabAcquisitionPhase),
                    _grabFrame.hasPinchPocket ? "yes" : "no",
                    grab_support_model_math::gripSupportKindName(_grabFrame.gripSupportKind),
                    _grabFrame.gripSupportAuthoredPivot ? "yes" : "no",
                    _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld ? "yes" : "no",
                    _grabFrame.requiresSettledVisualHandRelation ? "yes" : "no",
                    _grabFrame.gripEvidenceShapeKey,
                    _grabFrame.gripEvidenceTriangleIndex,
                    frozenAuthorityFrame.pivotAWorld.x,
                    frozenAuthorityFrame.pivotAWorld.y,
                    frozenAuthorityFrame.pivotAWorld.z,
                    frozenAuthorityFrame.gripPointWorldAtGrab.x,
                    frozenAuthorityFrame.gripPointWorldAtGrab.y,
                    frozenAuthorityFrame.gripPointWorldAtGrab.z,
                    frozenAuthorityFrame.pivotBBodyLocalGame.x,
                    frozenAuthorityFrame.pivotBBodyLocalGame.y,
                    frozenAuthorityFrame.pivotBBodyLocalGame.z,
                    frozenAuthorityFrame.pivotBConstraintLocalGame.x,
                    frozenAuthorityFrame.pivotBConstraintLocalGame.y,
                    frozenAuthorityFrame.pivotBConstraintLocalGame.z,
                    traceRelationPivotB.x,
                    traceRelationPivotB.y,
                    traceRelationPivotB.z,
                    traceSelectedPivotRelationDeltaGameUnits,
                    tracePivotBRelationDeltaGameUnits,
                    _grabFrame.pocketToGripDistanceGameUnits,
                    _grabFrame.selectionToGripEvidenceDistanceGameUnits,
                    _grabFrame.longObjectLeverGameUnits,
                    _grabFrame.pivotAuthorityPositionOnly ? "yes" : "no",
                    _grabFrame.pivotAuthorityNormalTrusted ? "yes" : "no",
                    _grabFrame.pivotAuthorityPositionConfidence);

                ROCK_LOG_INFO(Hand,
                    "{} GRAB_TRACE stage=capture_frames trace={} rawPos=({:.2f},{:.2f},{:.2f}) proxyPos=({:.2f},{:.2f},{:.2f}) objectPos=({:.2f},{:.2f},{:.2f}) bodyPos=({:.2f},{:.2f},{:.2f}) desiredBodyPos=({:.2f},{:.2f},{:.2f}) rawProxyRot={:.2f}deg rawProxyAxisMax={:.2f}deg objectDesiredAxisMax={:.2f}deg desiredBodyGrabBodyAxisMax={:.2f}deg targetToHiggsRelation={:.2f}deg transformBFrozenDelta={:.2f}deg",
                    handName(),
                    _grabFrame.traceId,
                    handWorldTransform.translate.x,
                    handWorldTransform.translate.y,
                    handWorldTransform.translate.z,
                    proxyFrameWorldAtGrab.translate.x,
                    proxyFrameWorldAtGrab.translate.y,
                    proxyFrameWorldAtGrab.translate.z,
                    objectWorldTransform.translate.x,
                    objectWorldTransform.translate.y,
                    objectWorldTransform.translate.z,
                    grabBodyWorldAtGrab.translate.x,
                    grabBodyWorldAtGrab.translate.y,
                    grabBodyWorldAtGrab.translate.z,
                    frozenAuthorityFrame.desiredBodyWorld.translate.x,
                    frozenAuthorityFrame.desiredBodyWorld.translate.y,
                    frozenAuthorityFrame.desiredBodyWorld.translate.z,
                    traceRawToProxyRotDegrees,
                    traceRawToProxyMaxAxisDegrees,
                    traceObjectToDesiredMaxAxisDegrees,
                    traceDesiredBodyToGrabBodyMaxAxisDegrees,
                    traceTargetToHiggsRelationDegrees,
                    traceTransformBFrozenDeltaDegrees);

                ROCK_LOG_INFO(Hand,
                    "{} GRAB_TRACE stage=capture_freeze_geometry trace={} body={} mode={} pivotAuthority={} "
                    "bodyShift=({:.2f},{:.2f},{:.2f}) shiftLen={:.2f}gu expectedShift=({:.2f},{:.2f},{:.2f}) expectedLen={:.2f}gu shiftDot={:.3f} "
                    "bodyRotShift={:.2f}deg pivotGapBefore={:.3f}gu pivotGapAfter={:.3f}gu pivotBefore=({:.2f},{:.2f},{:.2f}) pivotAfter=({:.2f},{:.2f},{:.2f}) pivotA=({:.2f},{:.2f},{:.2f}) grip=({:.2f},{:.2f},{:.2f})",
                    handName(),
                    _grabFrame.traceId,
                    objectBodyId.value,
                    _grabFrame.activeGrabPointMode ? _grabFrame.activeGrabPointMode : "none",
                    _grabFrame.pivotAuthoritySource ? _grabFrame.pivotAuthoritySource : "none",
                    traceFreezeShift.x,
                    traceFreezeShift.y,
                    traceFreezeShift.z,
                    traceFreezeShiftLength,
                    traceExpectedShift.x,
                    traceExpectedShift.y,
                    traceExpectedShift.z,
                    traceExpectedShiftLength,
                    traceFreezeShiftDot,
                    traceBodyShiftDegrees,
                    tracePivotGapBeforeFreeze,
                    tracePivotGapAfterFreeze,
                    tracePivotBeforeFreeze.x,
                    tracePivotBeforeFreeze.y,
                    tracePivotBeforeFreeze.z,
                    tracePivotAfterFreeze.x,
                    tracePivotAfterFreeze.y,
                    tracePivotAfterFreeze.z,
                    frozenAuthorityFrame.pivotAWorld.x,
                    frozenAuthorityFrame.pivotAWorld.y,
                    frozenAuthorityFrame.pivotAWorld.z,
                    frozenAuthorityFrame.gripPointWorldAtGrab.x,
                    frozenAuthorityFrame.gripPointWorldAtGrab.y,
                    frozenAuthorityFrame.gripPointWorldAtGrab.z);

                ROCK_LOG_INFO(Hand,
                    "{} GRAB_TRACE stage=capture_basis trace={} {} {} {} {}",
                    handName(),
                    _grabFrame.traceId,
                    grab_transform_telemetry::formatBasis("raw", traceRawBasis),
                    grab_transform_telemetry::formatBasis("proxy", traceProxyBasis),
                    grab_transform_telemetry::formatBasis("body", traceBodyBasis),
                    grab_transform_telemetry::formatBasis("desiredBody", traceDesiredBodyBasis));
                ROCK_LOG_INFO(Hand,
                    "{} GRAB_TRACE stage=capture_axismap trace={} {} {} {} {}",
                    handName(),
                    _grabFrame.traceId,
                    grab_transform_telemetry::formatBasisCrossMap("bodyToProxy", traceBodyBasis, traceProxyBasis),
                    grab_transform_telemetry::formatBasisCrossMap("desiredBodyToProxy", traceDesiredBodyBasis, traceProxyBasis),
                    grab_transform_telemetry::formatBasisCrossMap("proxyToBody", traceProxyBasis, traceBodyBasis),
                    grab_transform_telemetry::formatBasisCrossMap("bodyToDesiredBody", traceBodyBasis, traceDesiredBodyBasis));
            }
            if (g_rockConfig.rockDebugGrabFrameLogging) {
                std::array<float, 12> freezeTransformBRotation{};
                std::array<float, 4> freezeTransformBTranslation{};
                std::array<float, 12> freezeTargetBRca{};
                grab_constraint_math::writeGrabConstraintCreationAtoms(
                    freezeTransformBRotation.data(),
                    freezeTransformBTranslation.data(),
                    freezeTargetBRca.data(),
                    frozenAuthorityFrame.proxyAuthorityBodyHandSpace,
                    frozenAuthorityFrame.pivotAHandBodyLocalGame,
                    gameToHavokScale());
                const RE::NiTransform frozenDesiredBodyToHandSpace =
                    invertTransform(frozenAuthorityFrame.proxyAuthorityBodyHandSpace);
                const RE::NiMatrix3 freezeTargetRows = matrixFromHkRows(freezeTargetBRca.data());
                const RE::NiMatrix3 freezeTransformBColumns = matrixFromHkColumns(freezeTransformBRotation.data());
                const RE::NiPoint3 predictedTransformBLocal =
                    grab_constraint_math::computeDynamicTransformBTranslationGame(
                        frozenAuthorityFrame.proxyAuthorityBodyHandSpace,
                        frozenAuthorityFrame.pivotAHandBodyLocalGame);
                const RE::NiPoint3 freezeTransformBTranslationGame{
                    freezeTransformBTranslation[0] * havokToGameScale(),
                    freezeTransformBTranslation[1] * havokToGameScale(),
                    freezeTransformBTranslation[2] * havokToGameScale(),
                };
                const float freezeSelectedPivotRelationDelta =
                    pointDistanceGameUnits(frozenAuthorityFrame.pivotBConstraintLocalGame, predictedTransformBLocal);
                const float freezePivotBRelationDelta =
                    pointDistanceGameUnits(freezeTransformBTranslationGame, predictedTransformBLocal);
                const float freezeTargetToHiggsRelationDegrees =
                    rotationDeltaDegrees(freezeTargetRows, frozenDesiredBodyToHandSpace.rotate);
                const float freezeTransformBFrozenDeltaDegrees =
                    rotationDeltaDegrees(freezeTransformBColumns, frozenDesiredBodyToHandSpace.rotate);
                const float rawToProxyRotDegrees =
                    rotationDeltaDegrees(handWorldTransform.rotate, proxyFrameWorldAtGrab.rotate);
                const float rawToProxyMaxAxisDegrees =
                    maxColumnAxisDeltaDegrees(handWorldTransform.rotate, proxyFrameWorldAtGrab.rotate);
                const float objectToDesiredMaxAxisDegrees =
                    maxColumnAxisDeltaDegrees(objectWorldTransform.rotate, frozenAuthorityFrame.desiredObjectWorld.rotate);
                const float desiredBodyToGrabBodyMaxAxisDegrees =
                    maxColumnAxisDeltaDegrees(frozenAuthorityFrame.desiredBodyWorld.rotate, grabBodyWorldAtGrab.rotate);
                const float pivotBLeverGameUnits =
                    pointDistanceGameUnits(frozenAuthorityFrame.gripPointWorldAtGrab, grabBodyWorldAtGrab.translate);
                const bool fullHeldAuthorityAtFreeze =
                    _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld;

                ROCK_LOG_INFO(Hand,
                    "{} GRAB FREEZE AUTHORITY: formID={:08X} name='{}' body={} mode={} pivotAuthority={} frozenSource={} resolverSource={} "
                    "resolverReason={} seat={} phase={} pinch={} gripSupport={} supportPivot={} supportReason={} palmPocketMesh={} fullHeld={} settledReq={} "
                    "pivotA=({:.2f},{:.2f},{:.2f}) grip=({:.2f},{:.2f},{:.2f}) pivotBBody=({:.2f},{:.2f},{:.2f}) "
                    "pivotBSelected=({:.2f},{:.2f},{:.2f}) relationPivotB=({:.2f},{:.2f},{:.2f}) "
                    "lever={:.2f}gu pocket={:.2f}gu selection={:.2f}gu rawProxy={:.2f}deg rawProxyAxisMax={:.2f}deg "
                    "objectDesiredAxisMax={:.2f}deg desiredBodyGrabBodyAxisMax={:.2f}deg targetToHiggsRelation={:.2f}deg transformBFrozenDelta={:.2f}deg selectedPivotRelationDelta={:.3f}gu pivotBRelationDelta={:.3f}gu",
                    handName(),
                    sel.refr ? sel.refr->GetFormID() : 0,
                    objName,
                    objectBodyId.value,
                    _grabFrame.activeGrabPointMode ? _grabFrame.activeGrabPointMode : "none",
                    _grabFrame.pivotAuthoritySource ? _grabFrame.pivotAuthoritySource : "none",
                    grab_authority_frame_math::grabAuthorityPivotSourceName(frozenAuthorityFrame.source),
                    grab_authority_frame_math::grabAuthorityPivotSourceName(resolvedAuthorityPivotSourceForFreeze),
                    resolvedAuthorityPivotReasonForFreeze ? resolvedAuthorityPivotReasonForFreeze : "none",
                    grabSeatModeName(_grabFrame.seatMode),
                    grab_three_phase::phaseName(_grabAcquisitionPhase),
                    _grabFrame.hasPinchPocket ? "yes" : "no",
                    grab_support_model_math::gripSupportKindName(_grabFrame.gripSupportKind),
                    _grabFrame.gripSupportAuthoredPivot ? "yes" : "no",
                    _grabFrame.gripSupportReason ? _grabFrame.gripSupportReason : "none",
                    palmPocketMeshAvailable ? "yes" : "no",
                    fullHeldAuthorityAtFreeze ? "yes" : "no",
                    _grabFrame.requiresSettledVisualHandRelation ? "yes" : "no",
                    frozenAuthorityFrame.pivotAWorld.x,
                    frozenAuthorityFrame.pivotAWorld.y,
                    frozenAuthorityFrame.pivotAWorld.z,
                    frozenAuthorityFrame.gripPointWorldAtGrab.x,
                    frozenAuthorityFrame.gripPointWorldAtGrab.y,
                    frozenAuthorityFrame.gripPointWorldAtGrab.z,
                    frozenAuthorityFrame.pivotBBodyLocalGame.x,
                    frozenAuthorityFrame.pivotBBodyLocalGame.y,
                    frozenAuthorityFrame.pivotBBodyLocalGame.z,
                    frozenAuthorityFrame.pivotBConstraintLocalGame.x,
                    frozenAuthorityFrame.pivotBConstraintLocalGame.y,
                    frozenAuthorityFrame.pivotBConstraintLocalGame.z,
                    predictedTransformBLocal.x,
                    predictedTransformBLocal.y,
                    predictedTransformBLocal.z,
                    pivotBLeverGameUnits,
                    _grabFrame.pocketToGripDistanceGameUnits,
                    _grabFrame.selectionToGripEvidenceDistanceGameUnits,
                    rawToProxyRotDegrees,
                    rawToProxyMaxAxisDegrees,
                    objectToDesiredMaxAxisDegrees,
                    desiredBodyToGrabBodyMaxAxisDegrees,
                    freezeTargetToHiggsRelationDegrees,
                    freezeTransformBFrozenDeltaDegrees,
                    freezeSelectedPivotRelationDelta,
                    freezePivotBRelationDelta);
            }
            // ---- Stage 4: reset the hold's starting state ---------------------
            // The hold begins with no motion history and no published visual
            // transform. Stale history here reads as hand motion on frame one
            // and throws the object on acquisition.
            clearGrabExternalHandWorldTransform(_isLeft);
            _grabVisualHandTransform = handWorldTransform;
            _hasGrabVisualHandTransform = false;
            _lastPublishedGrabVisualHandTransform = {};
            _hasLastPublishedGrabVisualHandTransform = false;
            _grabVisualHandLerpStartTransform = handWorldTransform;
            _grabVisualHandLerpElapsedSeconds = 0.0f;
            _grabVisualHandLerpDurationSeconds = 0.0f;
            _grabVisualDeviationExceededSeconds = 0.0f;
            _grabDeviationExceededSeconds = 0.0f;
            const RE::NiPoint3 initialGrabDelta = grabPivotAWorld - grabGripPoint;
            const float initialGrabDistance =
                std::sqrt(initialGrabDelta.x * initialGrabDelta.x + initialGrabDelta.y * initialGrabDelta.y + initialGrabDelta.z * initialGrabDelta.z);
            const bool needsLargeInitialSync = initialGrabDistance >= g_rockConfig.rockGrabHandLerpMinDistance;
            _grabFrame.fadeInGrabConstraint = needsLargeInitialSync;
            _grabFrame.motorFadeReason = needsLargeInitialSync ? "largeInitialSync" : "none";
            _heldLocalLinearVelocityHistory = {};
            _heldLocalLinearVelocityHistoryCount = 0;
            _heldLocalLinearVelocityHistoryNext = 0;
            _heldLocalHandVelocityHistory = {};
            _heldHandAngularVelocityHistory = {};
            _heldHandVelocityHistoryCount = 0;
            _heldHandVelocityHistoryNext = 0;
            _lastHeldObjectLocalLinearVelocityHavok = {};
            _hasLastHeldObjectLocalLinearVelocityHavok = false;
            _previousHeldRawHandWorld = {};
            _previousHeldHandPositionHavok = {};
            _lastHeldHandPositionHavok = {};
            _hasPreviousHeldRawHandWorld = false;
            _hasLastHeldHandPositionHavok = false;
            _grabAuthorityProxyReleasePending.store(false, std::memory_order_release);

            if (g_rockConfig.rockDebugGrabFrameLogging) {
                const RE::NiPoint3 legacyPalmPivotAHandspace = computeGrabLegacyPalmPivotAHandspacePosition(_isLeft);
                auto* vrScaleSetting = f4vr::getIniSetting("fVrScale:VR");
                const float vrScale = vrScaleSetting ? vrScaleSetting->GetFloat() : -1.0f;

                const RE::NiPoint3 rawLateral = getMatrixColumn(handWorldTransform.rotate, 0);
                const RE::NiPoint3 rawFinger = getMatrixColumn(handWorldTransform.rotate, 2);
                const RE::NiPoint3 rawBack = getMatrixColumn(handWorldTransform.rotate, 1);
                const RE::NiPoint3 proxyFinger = getMatrixColumn(proxyFrameWorldAtGrab.rotate, 2);
                const RE::NiPoint3 proxyBack = getMatrixColumn(proxyFrameWorldAtGrab.rotate, 1);
                const RE::NiPoint3 proxyLateral = getMatrixColumn(proxyFrameWorldAtGrab.rotate, 0);
                const RE::NiPoint3 grabSpaceRawFinger = getMatrixColumn(_grabFrame.rawHandSpace.rotate, 0);
                const RE::NiPoint3 grabSpaceProxyFinger = getMatrixColumn(_grabFrame.proxyAuthorityHandSpace.rotate, 0);
                const RE::NiPoint3 grabPosDelta = _grabFrame.proxyAuthorityHandSpace.translate - _grabFrame.rawHandSpace.translate;
                const float rawVsProxyRot = rotationDeltaDegrees(_grabFrame.rawHandSpace.rotate, _grabFrame.proxyAuthorityHandSpace.rotate);
                const float rawVsProxyPos = std::sqrt(grabPosDelta.x * grabPosDelta.x + grabPosDelta.y * grabPosDelta.y + grabPosDelta.z * grabPosDelta.z);
                const float motionVsGrabRot = hasMotionBodyWorldAtGrab ? rotationDeltaDegrees(motionBodyWorldAtGrab.rotate, grabBodyWorldAtGrab.rotate) : -1.0f;
                const float motionVsGrabPos = hasMotionBodyWorldAtGrab ? translationDeltaGameUnits(motionBodyWorldAtGrab, grabBodyWorldAtGrab) : -1.0f;

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB FRAME SUMMARY: vrScale={:.3f} handScale={:.3f} bodyScale={:.3f} objectScale={:.3f} "
                    "legacyPivotHS=({:.2f},{:.2f},{:.2f}) pocketW=({:.1f},{:.1f},{:.1f}) gripW=({:.1f},{:.1f},{:.1f}) "
                    "pivotBBodyLocal=({:.2f},{:.2f},{:.2f}) pivotBConstraintLocal=({:.2f},{:.2f},{:.2f}) "
                    "rawProxyDelta={:.2f}deg/{:.2f}gu motionDiagVsGrab={:.2f}deg/{:.2f}gu proxyBodyFrame={} rotRef={} proxyFrame={} rootPalm={} pivotALocal=({:.2f},{:.2f},{:.2f}) meshMode={} meshTris={} "
                    "pocketGrip={:.1f}gu selectionGripEvidence={:.1f}gu "
                    "shapeKey=0x{:08X} shapeFilter=0x{:08X} hitFraction={:.4f} contactPatchEvidence={} contactPatchPivot={} patchHits={} snapDelta={:.1f}gu "
                    "multiFinger={} mfGroups={} mfSpread={:.2f}gu mfReason={} "
                    "activePoint={} activeUsesFingerEvidence={} pivotAuthoritySource={} positionOnlyPatch={} normalTrusted={} positionConfidence={:.2f} palmSeatPoint={} fingerEvidencePoint={} "
                    "poseTargets={} motorFade={} motorFadeReason={} bodyReason={} "
                    "fingerPoseAim={} fingerPoseAimReason={}",
                    handName(), vrScale, handWorldTransform.scale, handBodyWorldAtGrab.scale, collidableNode ? collidableNode->world.scale : -1.0f,
                    legacyPalmPivotAHandspace.x, legacyPalmPivotAHandspace.y, legacyPalmPivotAHandspace.z, grabPivotAWorld.x, grabPivotAWorld.y, grabPivotAWorld.z, grabGripPoint.x,
                    grabGripPoint.y, grabGripPoint.z, _grabFrame.pivotBBodyLocalGame.x, _grabFrame.pivotBBodyLocalGame.y, _grabFrame.pivotBBodyLocalGame.z,
                    _grabFrame.pivotBConstraintLocalGame.x, _grabFrame.pivotBConstraintLocalGame.y, _grabFrame.pivotBConstraintLocalGame.z,
                    rawVsProxyRot, rawVsProxyPos, motionVsGrabRot, motionVsGrabPos,
                    constraintUsesMotionBodyAtGrab ? "MOTION" : "BODY",
                    kGrabObjectRotationReferenceName,
                    proxyFrameSourceAtGrab,
                    hasPalmProxyFrameAtGrab ? "yes" : "no", _grabFrame.pivotAHandBodyLocalGame.x,
                    _grabFrame.pivotAHandBodyLocalGame.y, _grabFrame.pivotAHandBodyLocalGame.z, grabPointMode, grabMeshTriangles.size(),
                    _grabFrame.pocketToGripDistanceGameUnits, _grabFrame.selectionToGripEvidenceDistanceGameUnits,
                    _grabFrame.gripEvidenceShapeKey, _grabFrame.gripEvidenceShapeCollisionFilterInfo, _grabFrame.gripEvidenceHitFraction,
                    _grabFrame.hasContactPatchEvidence ? "yes" : "no", "no",
                    _grabFrame.contactPatchSampleCount, _grabFrame.contactPatchMeshSnapDeltaGameUnits,
                    _grabFrame.hasMultiFingerContactPatch ? "yes" : "no", _grabFrame.multiFingerContactGroupCount,
                    _grabFrame.multiFingerContactSpreadGameUnits, _grabFrame.multiFingerContactReason,
                    _grabFrame.activeGrabPointMode, _grabFrame.activeGrabPointUsesMultiFingerEvidence ? "yes" : "no",
                    _grabFrame.pivotAuthoritySource,
                    _grabFrame.pivotAuthorityPositionOnly ? "yes" : "no",
                    _grabFrame.pivotAuthorityNormalTrusted ? "yes" : "no",
                    _grabFrame.pivotAuthorityPositionConfidence,
                    _grabFrame.palmSeatPointMode, _grabFrame.fingerEvidencePointMode,
                    _grabFrame.fingerPoseTargetCount, _grabFrame.fadeInGrabConstraint ? "yes" : "no",
                    _grabFrame.motorFadeReason, _grabFrame.bodyResolutionReason,
                    _grabFrame.fingerPoseAimValid ? "yes" : "no", _grabFrame.fingerPoseAimReason);

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB FRAME SNAPSHOT: rawVsProxy rotDelta={:.2f}deg posDelta=({:.2f},{:.2f},{:.2f}) "
                    "rawFinger=({:.3f},{:.3f},{:.3f}) rawBack=({:.3f},{:.3f},{:.3f}) rawLat=({:.3f},{:.3f},{:.3f}) "
                    "proxyFinger=({:.3f},{:.3f},{:.3f}) proxyBack=({:.3f},{:.3f},{:.3f}) proxyLat=({:.3f},{:.3f},{:.3f})",
                    handName(), rawVsProxyRot, grabPosDelta.x, grabPosDelta.y, grabPosDelta.z, rawFinger.x,
                    rawFinger.y, rawFinger.z, rawBack.x, rawBack.y, rawBack.z, rawLateral.x, rawLateral.y, rawLateral.z, proxyFinger.x, proxyFinger.y, proxyFinger.z,
                    proxyBack.x, proxyBack.y, proxyBack.z, proxyLateral.x, proxyLateral.y, proxyLateral.z);

                const auto rawHandBasis = grab_transform_telemetry::makeOrientationBasis(handWorldTransform);
                const auto proxyPalmBasis = grab_transform_telemetry::makeOrientationBasis(proxyFrameWorldAtGrab);
                const auto objectAtGrabBasis = grab_transform_telemetry::makeOrientationBasis(objectWorldTransform);
                const auto desiredObjectBasis = grab_transform_telemetry::makeOrientationBasis(desiredObjectWorld);
                const auto desiredBodyBasis = grab_transform_telemetry::makeOrientationBasis(desiredBodyWorld);
                const auto grabBodyBasis = grab_transform_telemetry::makeOrientationBasis(grabBodyWorldAtGrab);
                const auto motionBodyBasis = grab_transform_telemetry::makeOrientationBasis(motionBodyWorldAtGrab);
                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB BASIS CAPTURE side={} phase=capture convention=niLocalVectorToWorld proxySource={} motionBody={} motionSrc={} proxyBodyFrame={} rotRef={} {} {} {} {} {} {} {}",
                    handName(),
                    _isLeft ? "left" : "right",
                    proxyFrameSourceAtGrab,
                    hasMotionBodyWorldAtGrab ? "yes" : "no",
                    body_frame::bodyFrameSourceCode(motionBodySourceAtGrab),
                    constraintUsesMotionBodyAtGrab ? "MOTION" : "BODY",
                    kGrabObjectRotationReferenceName,
                    grab_transform_telemetry::formatBasis("rawHand", rawHandBasis),
                    grab_transform_telemetry::formatBasis("proxyPalm", proxyPalmBasis),
                    grab_transform_telemetry::formatBasis("objectAtGrab", objectAtGrabBasis),
                    grab_transform_telemetry::formatBasis("desiredObject", desiredObjectBasis),
                    grab_transform_telemetry::formatBasis("desiredBody", desiredBodyBasis),
                    grab_transform_telemetry::formatBasis("grabBody", grabBodyBasis),
                    grab_transform_telemetry::formatBasis("motionBody", motionBodyBasis));

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB BASIS CAPTURE DELTA side={} phase=capture {} {} {} {} {}",
                    handName(),
                    _isLeft ? "left" : "right",
                    grab_transform_telemetry::formatBasisDelta("rawHandToProxyPalm", rawHandBasis, proxyPalmBasis),
                    grab_transform_telemetry::formatBasisDelta("objectAtGrabToDesiredObject", objectAtGrabBasis, desiredObjectBasis),
                    grab_transform_telemetry::formatBasisDelta("desiredObjectToDesiredBody", desiredObjectBasis, desiredBodyBasis),
                    grab_transform_telemetry::formatBasisDelta("grabBodyToDesiredBody", grabBodyBasis, desiredBodyBasis),
                    grab_transform_telemetry::formatBasisDelta("motionBodyToGrabBody", motionBodyBasis, grabBodyBasis));

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB BASIS CAPTURE AXISMAP side={} phase=capture {} {} {} {} {}",
                    handName(),
                    _isLeft ? "left" : "right",
                    grab_transform_telemetry::formatBasisCrossMap("bodyToProxy", grabBodyBasis, proxyPalmBasis),
                    grab_transform_telemetry::formatBasisCrossMap("desiredBodyToProxy", desiredBodyBasis, proxyPalmBasis),
                    grab_transform_telemetry::formatBasisCrossMap("proxyToBody", proxyPalmBasis, grabBodyBasis),
                    grab_transform_telemetry::formatBasisCrossMap("rawHandToProxy", rawHandBasis, proxyPalmBasis),
                    grab_transform_telemetry::formatBasisCrossMap("bodyToDesiredBody", grabBodyBasis, desiredBodyBasis));

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB FRAME TARGETS: grabHSRaw.pos=({:.2f},{:.2f},{:.2f}) grabHSProxy.pos=({:.2f},{:.2f},{:.2f}) "
                    "grabHSRawFinger=({:.3f},{:.3f},{:.3f}) grabHSProxyFinger=({:.3f},{:.3f},{:.3f}) "
                    "bodyLocal.pos=({:.2f},{:.2f},{:.2f}) bodyLocalFinger=({:.3f},{:.3f},{:.3f})",
                    handName(), _grabFrame.rawHandSpace.translate.x, _grabFrame.rawHandSpace.translate.y, _grabFrame.rawHandSpace.translate.z, _grabFrame.proxyAuthorityHandSpace.translate.x,
                    _grabFrame.proxyAuthorityHandSpace.translate.y, _grabFrame.proxyAuthorityHandSpace.translate.z, grabSpaceRawFinger.x, grabSpaceRawFinger.y, grabSpaceRawFinger.z,
                    grabSpaceProxyFinger.x, grabSpaceProxyFinger.y, grabSpaceProxyFinger.z, _grabFrame.bodyLocal.translate.x, _grabFrame.bodyLocal.translate.y,
                    _grabFrame.bodyLocal.translate.z, _grabFrame.bodyLocal.rotate.entry[0][0], _grabFrame.bodyLocal.rotate.entry[1][0],
                    _grabFrame.bodyLocal.rotate.entry[2][0]);

                const RE::NiPoint3 rootBodyLocalFinger = getMatrixColumn(_grabFrame.rootBodyLocal.rotate, 0);
                const RE::NiPoint3 ownerBodyLocalFinger = getMatrixColumn(_grabFrame.ownerBodyLocal.rotate, 0);
                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB NODE FRAMES: owner='{}'({:p}) hasCol={} ownsBodyCol={} "
                    "root='{}'({:p}) held='{}'({:p}) mesh='{}'({:p}) sameOwnerHeld={} sameRootHeld={} "
                    "ownerBodyLocal.pos=({:.2f},{:.2f},{:.2f}) ownerBodyLocalFinger=({:.3f},{:.3f},{:.3f}) "
                    "rootBodyLocal.pos=({:.2f},{:.2f},{:.2f}) rootBodyLocalFinger=({:.3f},{:.3f},{:.3f})",
                    handName(), nodeDebugName(ownerNodeAtGrab), static_cast<const void*>(ownerNodeAtGrab),
                    (ownerNodeAtGrab && ownerNodeAtGrab->collisionObject.get()) ? "yes" : "no",
                    (ownerNodeAtGrab && ownerNodeAtGrab->collisionObject.get() == bodyCollisionObjectAtGrab) ? "yes" : "no", nodeDebugName(rootNode),
                    static_cast<const void*>(rootNode), nodeDebugName(collidableNode), static_cast<const void*>(collidableNode), nodeDebugName(meshSourceNode),
                    static_cast<const void*>(meshSourceNode), ownerNodeAtGrab == collidableNode ? "yes" : "no", rootNode == collidableNode ? "yes" : "no",
                    _grabFrame.ownerBodyLocal.translate.x, _grabFrame.ownerBodyLocal.translate.y, _grabFrame.ownerBodyLocal.translate.z, ownerBodyLocalFinger.x,
                    ownerBodyLocalFinger.y, ownerBodyLocalFinger.z, _grabFrame.rootBodyLocal.translate.x, _grabFrame.rootBodyLocal.translate.y,
                    _grabFrame.rootBodyLocal.translate.z, rootBodyLocalFinger.x, rootBodyLocalFinger.y, rootBodyLocalFinger.z);
            }

            ROCK_LOG_DEBUG(Hand,
                "{} GRAB HAND SPACE: pos=({:.1f},{:.1f},{:.1f}) "
                "palmPos=({:.1f},{:.1f},{:.1f}) pivotA=({:.1f},{:.1f},{:.1f}) grabPt=({:.1f},{:.1f},{:.1f})",
                handName(), _grabFrame.rawHandSpace.translate.x, _grabFrame.rawHandSpace.translate.y, _grabFrame.rawHandSpace.translate.z, palmPos.x, palmPos.y, palmPos.z, grabPivotAWorld.x,
                grabPivotAWorld.y, grabPivotAWorld.z, grabGripPoint.x, grabGripPoint.y, grabGripPoint.z);
            ROCK_LOG_DEBUG(Hand, "{} BODY LOCAL: pos=({:.2f},{:.2f},{:.2f}) scale={:.3f}", handName(), _grabFrame.bodyLocal.translate.x, _grabFrame.bodyLocal.translate.y,
                _grabFrame.bodyLocal.translate.z, _grabFrame.bodyLocal.scale);
        }

        {
            RE::NiTransform handBodyDiag{};
            RE::NiTransform objectBodyDiag{};
            const bool hasLiveDiag = tryResolveLiveBodyWorldTransform(world, _handBody.getBodyId(), handBodyDiag) &&
                                     tryResolveLiveBodyWorldTransform(world, objectBodyId, objectBodyDiag);
            if (hasLiveDiag) {
                ROCK_LOG_TRACE(Hand,
                    "{} DIAG: handBodyLive pos=({:.1f},{:.1f},{:.1f}) objBodyLive pos=({:.1f},{:.1f},{:.1f})",
                    handName(),
                    handBodyDiag.translate.x,
                    handBodyDiag.translate.y,
                    handBodyDiag.translate.z,
                    objectBodyDiag.translate.x,
                    objectBodyDiag.translate.y,
                    objectBodyDiag.translate.z);
            }
            ROCK_LOG_TRACE(Hand, "{} DIAG: handNi pos=({:.1f},{:.1f},{:.1f}) objNi pos=({:.1f},{:.1f},{:.1f})", handName(), handWorldTransform.translate.x,
                handWorldTransform.translate.y, handWorldTransform.translate.z, objectWorldTransform.translate.x, objectWorldTransform.translate.y,
                objectWorldTransform.translate.z);

            float comX, comY, comZ;
            if (getBodyCOMWorld(world, objectBodyId, comX, comY, comZ) && hasLiveDiag) {
                ROCK_LOG_TRACE(Hand,
                    "{} B8 COM LIVE: comHk=({:.3f},{:.3f},{:.3f}) objBodyLive=({:.1f},{:.1f},{:.1f})",
                    handName(),
                    comX,
                    comY,
                    comZ,
                    objectBodyDiag.translate.x,
                    objectBodyDiag.translate.y,
                    objectBodyDiag.translate.z);
            }
        }


        return true;
    }
}
