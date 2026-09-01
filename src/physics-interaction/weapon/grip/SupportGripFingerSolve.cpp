#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// Support grip capture transaction including the per-finger surface solve (capturePartGrip).

namespace rock
{
    authored_support_grab_policy::Selection TwoHandedGrip::capturePartGrip(
        bool isLeft,
        RE::NiNode* weaponNode,
        const WeaponInteractionDecision& decision,
        const WeaponCollision& weaponCollision,
        const WeaponProviderPartAuthority& providerPartAuthority,
        const bool firingGripProximityAuthorityEnabled,
        const bool ambidextrousHandoffCaptureContext,
        RE::NiTransform* const outCapturedHandWorld)
    {
        if (outCapturedHandWorld) {
            *outCapturedHandWorld = {};
        }
        if (!weaponNode) {
            return authored_support_grab_policy::Selection::Failure;
        }
        if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(decision.weaponGenerationKey, _activeWeaponGenerationKey)) {
            ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: part grip capture skipped because contact generation is stale hand={}", isLeft ? "left" : "right");
            return authored_support_grab_policy::Selection::Failure;
        }

        RE::NiTransform handTransform{};
        if (!tryGetSolverHandTransform(isLeft, handTransform)) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: part grip capture skipped because authoritative hand transforms are unavailable hand={}", isLeft ? "left" : "right");
            return authored_support_grab_policy::Selection::Failure;
        }
        if (outCapturedHandWorld) {
            *outCapturedHandWorld = handTransform;
        }

        WeaponPartGrip& grip = partGrip(isLeft);
        grip = {};
        RE::NiAVObject* supportAttachmentRoot = decision.sourceRoot ? decision.sourceRoot : static_cast<RE::NiAVObject*>(weaponNode);
        grip.gripPose = decision.gripPose != WeaponGripPoseId::None ? decision.gripPose : WeaponGripPoseId::BarrelWrap;
        grip.partKind = decision.partKind;
        grip.attachmentRoot = supportAttachmentRoot;
        grip.providerPartAuthority = providerPartAuthority.active ? providerPartAuthority : WeaponProviderPartAuthority{};
        grip.attachOnly = weapon_part_grip_report_policy::providerGrabModeIsAttachOnly(
            grip.providerPartAuthority.active,
            grip.providerPartAuthority.grabMode);
        grip.contactBodyId = decision.bodyId;
        grip.reloadRole = decision.reloadRole;
        grip.socketRole = decision.socketRole;
        grip.actionRole = decision.actionRole;
        grip.weaponGenerationKey = decision.weaponGenerationKey;
        grip.gripSequence = ++_gripCaptureSequence;
        {
            // The routing decision carries no support role or authored source
            // name; both come from the evidence descriptor keyed by the
            // contact body, matching the provider target-query construction.
            WeaponCollisionProfileEvidenceDescriptor descriptor{};
            RE::NiAVObject* descriptorSourceNode = nullptr;
            if (weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(decision.bodyId, descriptor, descriptorSourceNode) &&
                descriptor.weaponGenerationKey == decision.weaponGenerationKey) {
                grip.supportRole = descriptor.semantic.supportGripRole;
                grip.omodFormId = descriptor.omodFormId;
                grip.attachPointFormId = descriptor.semantic.attachPointFormId;
                grip.classificationSource = descriptor.semantic.classificationSource;
                const std::size_t copyLength = (std::min)(descriptor.sourceName.size(), grip.sourceName.size() - 1);
                std::memcpy(grip.sourceName.data(), descriptor.sourceName.data(), copyLength);
                grip.sourceName[copyLength] = '\0';
            } else if (grip.providerPartAuthority.active) {
                grip.supportRole = static_cast<WeaponSupportGripRole>(grip.providerPartAuthority.supportRole);
                grip.sourceName = grip.providerPartAuthority.sourceName;
                grip.sourceName[grip.sourceName.size() - 1] = '\0';
            }
        }

        performance_profiler::ScopedTimer fingerPoseCaptureTimer(performance_profiler::Scope::EquippedWeaponFingerPoseCapture);

        const RE::NiPoint3 palmPos = computeGrabLegacyPalmPivotAWorldFromHandBasis(handTransform, isLeft);
        const RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(handTransform, isLeft);
        const RE::NiPoint3 firingGripWorld =
            weaponLocalToWorld(_primaryGripLocal, weaponNode);
        const RE::NiPoint3 palmToFiringGrip =
            sub(palmPos, firingGripWorld);
        const float supportPalmToFiringGripDistance =
            std::sqrt(dot(palmToFiringGrip, palmToFiringGrip));

        /*
         * Acquisition-only grip priority. Physical contact and the proximity
         * probe are equivalent entry sources. Exact provider authority wins
         * first. A live palm at the firing grip may then select the bounded
         * dynamic ambidextrous handoff station when the authored seat is not
         * itself at that grip. Otherwise an eligible authored relation wins,
         * but only inside the enforced family cone and radial cap. Generated
         * collision witnesses remain optional visualization and never qualify
         * or reject the captured authored relation. A matched provider target
         * remains the first
         * authority. Otherwise the authored-only policy permits the dynamic
         * mesh solver only when the mode is disabled or this exact weapon and
         * support topology has qualified as lacking a usable authored pose. A
         * missed cone/radius rejects instead of silently becoming dynamic. The
         * final authored seat also selects visual-only versus full weapon
         * authority. Provider AttachOnly remains PAPER/consumer glue. Once
         * selected, the exact hand/weapon relation and 15 finger locals are
         * latched; later candidate changes cannot move it.
         */
        const bool authoredWeaponIdentityMatches =
            _authoredSupportGripCandidate.weaponNode == weaponNode;
        const bool authoredGenerationMatches =
            _authoredSupportGripCandidate.weaponGenerationKey ==
            decision.weaponGenerationKey;
        RE::NiTransform authoredSupportHandWeaponLocal{};
        std::array<RE::NiTransform, 15> authoredSupportFingerLocalTransforms{};
        std::uint16_t authoredSupportFingerLocalTransformMask = 0;
        const bool authoredSupportCandidateForHandValid =
            tryResolveAuthoredSupportGripCandidateForHand(
                isLeft,
                weaponNode,
                decision.weaponGenerationKey,
                authoredSupportHandWeaponLocal,
                authoredSupportFingerLocalTransforms,
                authoredSupportFingerLocalTransformMask);
        AuthoredSupportPalmSeatProximity authoredSupportProximity{};
        RE::NiTransform authoredSupportHandWorld{};
        RE::NiPoint3 authoredSupportPalmWeaponLocal{};
        RE::NiPoint3 authoredSupportPalmNormalWorld{};
        float authoredSupportTouchProbeDistance =
            (std::numeric_limits<float>::infinity)();
        float authoredSupportPalmToFiringGripDistance =
            (std::numeric_limits<float>::infinity)();
        bool authoredSupportFrameValid = false;
        if (authoredSupportCandidateForHandValid &&
            resolveAuthoredSupportPalmSeatProximity(
                weaponNode->world,
                handTransform,
                authoredSupportHandWeaponLocal,
                isLeft,
                authoredSupportProximity)) {
            authoredSupportHandWorld = authoredSupportProximity.authoredHandWorld;
            authoredSupportPalmWeaponLocal =
                authoredSupportProximity.authoredPalmSeatWeaponLocal;
            authoredSupportPalmNormalWorld =
                computePalmNormalFromHandBasis(
                    authoredSupportHandWorld,
                    isLeft);
            authoredSupportTouchProbeDistance =
                authoredSupportProximity.weaponRelativeDistanceGameUnits;
            authoredSupportFrameValid =
                std::isfinite(authoredSupportTouchProbeDistance) &&
                std::isfinite(authoredSupportPalmNormalWorld.x) &&
                std::isfinite(authoredSupportPalmNormalWorld.y) &&
                std::isfinite(authoredSupportPalmNormalWorld.z);
        }

        refreshAuthoredSupportGripActivationState(
            weaponNode,
            decision.weaponGenerationKey,
            weaponCollision);
        const auto& authoredActivation =
            _authoredSupportGripDebugSnapshot;
        const bool authoredActivationStateMatches =
            authoredActivation.valid &&
            authoredActivation.supportHandIsLeft == isLeft &&
            authoredActivation.weaponGenerationKey ==
                decision.weaponGenerationKey &&
            authoredActivation.captureSequence ==
                _authoredSupportGripCandidate.captureSequence;
        const bool authoredActivationZoneValid =
            authoredActivationStateMatches &&
            authoredActivation.activationSpatialPass;
        const bool authoredPoseCollisionDiagnosticPass =
            authoredActivationStateMatches &&
            authoredActivation.poseEvidenceEvaluated &&
            authoredActivation.poseEvidencePass;
        const bool authoredSeatWeaponSurfaceValid =
            authoredActivationStateMatches &&
            (authoredActivation.poseSurfaceWitnessMask & 0x01u) != 0;
        const float authoredSupportSurfaceDistance =
            authoredSeatWeaponSurfaceValid ?
            authoredActivation.poseSurfaceDistanceGameUnits[0] :
            (std::numeric_limits<float>::infinity)();

        bool authoredSupportAuthorityGateValid =
            !firingGripProximityAuthorityEnabled;
        auto authoredSupportAuthorityMode =
            weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
        if (authoredSupportFrameValid &&
            firingGripProximityAuthorityEnabled &&
            std::isfinite(weaponNode->world.scale)) {
            const RE::NiPoint3 authoredSeatToFiringGripLocal =
                sub(authoredSupportPalmWeaponLocal, _primaryGripLocal);
            const float authoredSeatToFiringGripLocalDistance = std::sqrt(
                dot(authoredSeatToFiringGripLocal,
                    authoredSeatToFiringGripLocal));
            authoredSupportPalmToFiringGripDistance =
                authoredSeatToFiringGripLocalDistance *
                std::abs(weaponNode->world.scale);
            if (std::isfinite(authoredSupportPalmToFiringGripDistance)) {
                authoredSupportAuthorityMode =
                    weapon_support_authority_policy::resolveFiringGripProximityAuthorityMode(
                        authoredSupportPalmToFiringGripDistance,
                        _handlingSettings.firingGripProximitySupportRadiusGameUnits);
                authoredSupportAuthorityGateValid = true;
            }
        }

        const bool authoredInteractionAcquisitionValid =
            decision.acquisitionSource ==
                WeaponInteractionAcquisitionSource::AuthoredSeat ||
            decision.acquisitionSource ==
                WeaponInteractionAcquisitionSource::PhysicalContact ||
            decision.acquisitionSource ==
                WeaponInteractionAcquisitionSource::ProximityProbe;

        constexpr std::uint16_t kCompleteAuthoredFingerMask = 0x7FFFu;
        const bool useAuthoredSupportGrip =
            authored_weapon_grip_capture_policy::shouldUseAuthoredSupportGrip(
                authored_weapon_grip_capture_policy::AuthoredSupportGripCandidateInput{
                    .interactionAcquisitionValid =
                        authoredInteractionAcquisitionValid,
                    .activationZoneValid = authoredActivationZoneValid,
                    .providerAuthorityActive = providerPartAuthority.active,
                    .attachOnly = grip.attachOnly,
                    .captureValid =
                        authoredSupportCandidateForHandValid &&
                        authoredSupportFrameValid &&
                        authoredSupportAuthorityGateValid,
                    .weaponIdentityMatches = authoredWeaponIdentityMatches,
                    .generationMatches = authoredGenerationMatches,
                    .completeFingerPose =
                        authoredSupportFingerLocalTransformMask ==
                        kCompleteAuthoredFingerMask,
                });
        const bool useDynamicHandoffGrip =
            weapon_support_authority_policy::
                shouldCaptureDynamicHandoffGrip(
                    weapon_support_authority_policy::
                        DynamicHandoffGripCaptureInput{
                            .normalSupportAcquisition =
                                ambidextrousHandoffCaptureContext,
                            .ambidextrousHandoffEnabled =
                                _handlingSettings.
                                    ambidextrousHandoffEnabled,
                            .firingGripProximityAuthorityEnabled =
                                firingGripProximityAuthorityEnabled,
                            .providerPartAuthorityActive =
                                providerPartAuthority.active,
                            .authoredCaptureEligible =
                                useAuthoredSupportGrip,
                            .supportPalmToFiringGripDistance =
                                supportPalmToFiringGripDistance,
                            .authoredSeatToFiringGripDistance =
                                authoredSupportPalmToFiringGripDistance,
                            .firingGripPromotionRadius =
                                _handlingSettings.
                                    firingGripPromotionRadiusGameUnits,
                        });
        if (_handlingSettings.authoredOnlySupportGrabsEnabled &&
            !providerPartAuthority.active) {
            observeAuthoredSupportCapability(
                weaponNode,
                decision.weaponGenerationKey);
        }
        const auto selectionDecision =
            authored_support_grab_policy::select(
                authored_support_grab_policy::SelectionInput{
                    .modeEnabled =
                        _handlingSettings.authoredOnlySupportGrabsEnabled,
                    .providerPartAuthorityActive =
                        providerPartAuthority.active,
                    .dynamicHandoffCaptureEligible =
                        useDynamicHandoffGrip,
                    .authoredCaptureEligible = useAuthoredSupportGrip,
                    .capability =
                        _authoredSupportCapability.capability,
                });
        recordSupportGrabSelection(
            selectionDecision,
            isLeft,
            decision.weaponGenerationKey);

        if (selectionDecision.selection ==
            authored_support_grab_policy::Selection::Authored) {
            if (firingGripProximityAuthorityEnabled) {
                _authorityMode = authoredSupportAuthorityMode;
            }
            grip.authoredSupportGrip = true;
            /*
             * Palm-normal twist belongs to the retired full-rigid authored
             * support behavior, not to either ambidextrous topology. The
             * mirrored right-support pose uses the same authored canonical and
             * must inherit the exact right-fire/left-support contract: retain
             * two-point axis aiming and primary-anchored translation, but never
             * corkscrew the weapon around that axis. Do not gate this by the
             * current firing hand again.
             */
            grip.disableAuthoredSupportNormalTwist =
                _rightFiringHandCanonicalSource ==
                    RightFiringCanonicalSource::AuthoredAnimation;
            grip.authoredSupportCaptureSequence =
                _authoredSupportGripCandidate.captureSequence;
            grip.attachmentRoot = weaponNode;
            grip.gripLocal = authoredSupportPalmWeaponLocal;
            grip.grabNormalWorld = authoredSupportPalmNormalWorld;
            grip.normalLocal = transform_math::worldVectorToLocal(
                weaponNode->world,
                authoredSupportPalmNormalWorld);
            grip.handWeaponLocal =
                authoredSupportHandWeaponLocal;
            grip.hasHandWeaponLocal = true;
            grip.hasSourceFrames = false;
            grip.hasAttachmentWeaponLocal = false;

            // hFRIK requires the role-tagged numeric pose to exist before the
            // exact per-joint local override can win at the same priority.
            setSupportGripPose(isLeft, nullptr, nullptr);
            grip.fingerLocalTransforms =
                authoredSupportFingerLocalTransforms;
            grip.fingerLocalTransformMask =
                authoredSupportFingerLocalTransformMask;
            grip.hasFingerLocalTransforms = true;
            grip.visualLerp = {};
            grip.active = true;

            performance_profiler::observeValue(
                performance_profiler::ValueMetric::EquippedWeaponFingerPoseSourceTriangles,
                0);
            performance_profiler::observeValue(
                performance_profiler::ValueMetric::EquippedWeaponFingerPoseSelectedTriangles,
                0);
            if (isLeft) {
                _hapticEvents.leftPartGripCaptured = true;
            } else {
                _hapticEvents.rightPartGripCaptured = true;
            }

            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: authored support grip captured hand={} weapon='{}' gripLocal=({:.3f},{:.3f},{:.3f}) touchToSeat={:.3f} radialCap={:.3f} surfaceDistance={:.3f} poseWitnesses={}/6 poseMask={:02X} topology={} sideDot={:.3f} downDot={:.3f} arcDot={:.3f} region={} authoredSeatToFiringGrip={:.3f} seatLocal=({:.3f},{:.3f},{:.3f}) touchLocal=({:.3f},{:.3f},{:.3f}) frameError={:.4f} capture={} generation={:016X} acquisition={} authority={} priority=provider>handoff>authored>dynamic",
                isLeft ? "left" : "right",
                weaponNode->name.c_str(),
                grip.gripLocal.x,
                grip.gripLocal.y,
                grip.gripLocal.z,
                authoredSupportTouchProbeDistance,
                authoredActivation.radialCapGameUnits,
                authoredSupportSurfaceDistance,
                static_cast<unsigned>(
                    authoredActivation.poseSurfaceWitnessCount),
                static_cast<unsigned>(
                    authoredActivation.poseSurfaceWitnessMask),
                authored_weapon_grip_activation_policy::handTopologyName(
                    authoredActivation.handTopology),
                authoredActivation.supportSideDot,
                authoredActivation.downDot,
                authoredActivation.sweptArcDot,
                authored_weapon_grip_activation_policy::activationRegionName(
                    authoredActivation.selectedRegion),
                authoredSupportPalmToFiringGripDistance,
                authoredSupportPalmWeaponLocal.x,
                authoredSupportPalmWeaponLocal.y,
                authoredSupportPalmWeaponLocal.z,
                authoredSupportProximity.liveTouchProbeWeaponLocal.x,
                authoredSupportProximity.liveTouchProbeWeaponLocal.y,
                authoredSupportProximity.liveTouchProbeWeaponLocal.z,
                authoredSupportProximity.frameAgreementErrorGameUnits,
                grip.authoredSupportCaptureSequence,
                _activeWeaponGenerationKey,
                weaponInteractionAcquisitionSourceName(
                    decision.acquisitionSource),
                _authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport ?
                    "visual-only" :
                    "full");
            return selectionDecision.selection;
        }

        if (authoredSupportCandidateForHandValid) {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: authored support grip not selected hand={} selection={} source={} family={} topology={} axes={} activation={} poseDiagnostic={} palmDiagnostic={} witnesses={}/6 mask={:02X} distance={:.3f} cap={:.3f} sideDot={:.3f} downDot={:.3f} arcDot={:.3f} region={} class={} radial={} direction={} topologyGate={} provider={} attachOnly={} capture={} identity={} generation={} fingers={}",
                isLeft ? "left" : "right",
                authored_support_grab_policy::selectionName(
                    selectionDecision.selection),
                weaponInteractionAcquisitionSourceName(
                    decision.acquisitionSource),
                authored_weapon_grip_activation_policy::weaponFamilyName(
                    authoredActivation.weaponFamily),
                authored_weapon_grip_activation_policy::handTopologyName(
                    authoredActivation.handTopology),
                authoredActivation.canonicalAxesValid ? "pass" : "fail",
                authoredActivationZoneValid ? "pass" : "fail",
                authoredPoseCollisionDiagnosticPass ? "pass" :
                    (authoredActivation.poseEvidenceEvaluated ? "fail" : "not-run"),
                authoredSeatWeaponSurfaceValid ? "pass" :
                    (authoredActivation.poseEvidenceEvaluated ? "fail" : "not-run"),
                static_cast<unsigned>(
                    authoredActivation.poseSurfaceWitnessCount),
                static_cast<unsigned>(
                    authoredActivation.poseSurfaceWitnessMask),
                authoredActivation.weaponRelativeDistanceGameUnits,
                authoredActivation.radialCapGameUnits,
                authoredActivation.supportSideDot,
                authoredActivation.downDot,
                authoredActivation.sweptArcDot,
                authored_weapon_grip_activation_policy::activationRegionName(
                    authoredActivation.selectedRegion),
                authoredActivation.classifierSupported ? "pass" : "fail",
                authoredActivation.radialPass ? "pass" : "fail",
                authoredActivation.directionPass ? "pass" : "fail",
                authoredActivation.topologyPass ? "pass" : "fail",
                providerPartAuthority.active ? "yes" : "no",
                grip.attachOnly ? "yes" : "no",
                authoredSupportFrameValid &&
                        authoredSupportAuthorityGateValid ?
                    "pass" : "fail",
                authoredWeaponIdentityMatches ? "pass" : "fail",
                authoredGenerationMatches ? "pass" : "fail",
                authoredSupportFingerLocalTransformMask ==
                        kCompleteAuthoredFingerMask ?
                    "pass" : "fail");
        }

        if (selectionDecision.selection ==
            authored_support_grab_policy::Selection::Reject) {
            // The transaction-local grip was cleared before selection, so an
            // intentional policy rejection leaves no partial hand authority.
            grip = {};
            return selectionDecision.selection;
        }

        auto& fingerScratch = _fingerPoseSolveScratch->hands[isLeft ? 0u : 1u];
        for (auto& ranking : fingerScratch.rankings) {
            ranking.clear();
        }
        fingerScratch.localTriangles.clear();
        fingerScratch.worldTriangles.clear();
        fingerScratch.spatialIndex.clear();

        WeaponCollision::SupportGripEvidenceView evidenceView{};
        const bool cachedTrianglesFound = weaponCollision.tryGetSupportGripEvidenceView(decision.bodyId, weaponNode, evidenceView) &&
            evidenceView.weaponGenerationKey == decision.weaponGenerationKey &&
            evidenceView.weaponGenerationKey == _activeWeaponGenerationKey;
        const std::size_t contactedSourceTriangleCount =
            cachedTrianglesFound ?
            evidenceView.localTriangles.size() :
            0u;

        GrabPoint grabPoint{};
        bool meshFound = false;
        if (!useDynamicHandoffGrip && cachedTrianglesFound) {
            const TransformedSupportGripTriangleView worldEvidence{
                .localTriangles = evidenceView.localTriangles,
                .localToWorld = evidenceView.localToWorld,
            };
            meshFound = findClosestGrabPoint(
                worldEvidence,
                palmPos,
                palmDir,
                g_rockConfig.rockGrabLateralWeight,
                g_rockConfig.rockGrabDirectionalWeight,
                grabPoint,
                g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits);
        }

        if (useDynamicHandoffGrip) {
            // This is a real second station at the firing grip, not an
            // authored support seat or whichever receiver triangle happened
            // to satisfy the contact probe. Locking the exact station makes
            // the later promotion test deterministic while the dynamic finger
            // solver still wraps the live weapon geometry below.
            supportAttachmentRoot = weaponNode;
            grip.attachmentRoot = weaponNode;
            grip.gripLocal = _primaryGripLocal;
            grip.grabNormalWorld = palmDir;
        } else if (meshFound) {
            grip.gripLocal = worldToWeaponLocal(grabPoint.position, weaponNode);
            grip.grabNormalWorld = grabPoint.normal;
        } else {
            grip.gripLocal = worldToWeaponLocal(palmPos, weaponNode);
            grip.grabNormalWorld = palmDir;
        }
        const RE::NiPoint3 gripWorldPoint = useDynamicHandoffGrip ?
            firingGripWorld :
            (meshFound ? grabPoint.position : palmPos);
        constexpr float kDegreesToRadians =
            0.01745329251994329577f;
        const float surfaceSeatMaxRadians =
            g_rockConfig.rockWeaponSupportSurfaceSeatEnabled && meshFound ?
            g_rockConfig.rockWeaponSupportSurfaceSeatMaxDegrees *
                kDegreesToRadians :
            0.0f;
        const auto surfaceSeat =
            weapon_support_acquisition_math::
                alignHandFrameToGripSurface<
                    RE::NiTransform,
                    RE::NiPoint3>(
                    handTransform,
                    palmPos,
                    palmDir,
                    gripWorldPoint,
                    grip.grabNormalWorld,
                    surfaceSeatMaxRadians);
        const RE::NiTransform adjustedHandTransform =
            surfaceSeat.valid ?
            surfaceSeat.handWorld :
            weapon_two_handed_grip_math::alignHandFrameToGripPoint(
                handTransform,
                palmPos,
                gripWorldPoint);
        grip.surfaceSeatRotationRadians =
            surfaceSeat.valid ?
            surfaceSeat.appliedRotationRadians :
            0.0f;
        const RE::NiPoint3 seatedPalmNormal =
            computePalmNormalFromHandBasis(
                adjustedHandTransform,
                isLeft);
        grip.handWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), adjustedHandTransform);
        grip.hasHandWeaponLocal = true;
        grip.normalLocal = transform_math::worldVectorToLocal(
            weaponNode->world,
            seatedPalmNormal);
        if (supportAttachmentRoot) {
            grip.gripSourceLocal = transform_math::worldPointToLocal(supportAttachmentRoot->world, gripWorldPoint);
            grip.normalSourceLocal = transform_math::worldVectorToLocal(
                supportAttachmentRoot->world,
                seatedPalmNormal);
            grip.handSourceLocal = transform_math::composeTransforms(transform_math::invertTransform(supportAttachmentRoot->world), adjustedHandTransform);
            grip.attachmentWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), supportAttachmentRoot->world);
            grip.hasSourceFrames = true;
            grip.hasAttachmentWeaponLocal = true;
        }

        /*
         * Capture the root-flattened fingers once for this transaction. The
         * compact sweep snapshot and any exact-local thumb/surface correction
         * must describe the same pre-authority hand, not two scene reads split
         * by grip publication.
         */
        DirectSkeletonBoneSnapshot capturedFingerBoneSnapshot{};
        const bool capturedFingerBoneSnapshotValid =
            rootFlattenedTwoHandedReader().capture(
                skeleton_bone_debug_math::DebugSkeletonBoneMode::
                    HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::
                    GameRootFlattenedBoneTree,
                capturedFingerBoneSnapshot);
        root_flattened_finger_skeleton_runtime::Snapshot
            capturedFingerSnapshot{};
        const bool capturedFingerSnapshotValid =
            capturedFingerBoneSnapshotValid &&
            root_flattened_finger_skeleton_runtime::
                buildFingerSkeletonSnapshot(
                    capturedFingerBoneSnapshot,
                    isLeft,
                    capturedFingerSnapshot);
        SupportGripFingerReferenceSet fingerReferenceSet{};
        fingerReferenceSet.seatPointWorld = gripWorldPoint;
        fingerReferenceSet.seatPointValid =
            grab_finger_pose_runtime::isFinitePoint(gripWorldPoint);
        if (capturedFingerSnapshotValid) {
            const RE::NiTransform rawToSeatedWorld =
                transform_math::composeTransforms(
                    adjustedHandTransform,
                    transform_math::invertTransform(handTransform));
            const auto liveLandmarks =
                root_flattened_finger_skeleton_runtime::
                    buildLandmarkSet(capturedFingerSnapshot);
            std::array<RE::NiPoint3,
                kSupportGripFingerLaneCount>
                commandedOpenDirectionsWorld{};
            const bool commandedDirectionsValid =
                grab_finger_pose_runtime::
                    resolveCommandedOpenDirectionsWorld(
                        isLeft,
                        adjustedHandTransform,
                        commandedOpenDirectionsWorld);
            const RE::NiPoint3 seatedSweepNormal =
                liveLandmarks.valid ?
                transform_math::localVectorToWorld(
                    rawToSeatedWorld,
                    liveLandmarks.palmNormalWorld) :
                RE::NiPoint3{};
            const auto appendLanePoint = [&fingerReferenceSet](
                                             const std::size_t lane,
                                             const RE::NiPoint3& pointWorld) {
                if (lane >= kSupportGripFingerLaneCount ||
                    !grab_finger_pose_runtime::isFinitePoint(
                        pointWorld)) {
                    return;
                }
                auto& count =
                    fingerReferenceSet.lanePointCounts[lane];
                if (count >=
                    kSupportGripFingerLaneReferenceCapacity) {
                    return;
                }
                fingerReferenceSet.lanePointsWorld[lane][count++] =
                    pointWorld;
            };

            for (std::size_t finger = 0;
                 finger < capturedFingerSnapshot.fingers.size();
                 ++finger) {
                const auto& chain =
                    capturedFingerSnapshot.fingers[finger];
                if (!chain.valid) {
                    continue;
                }
                for (const auto& pointWorld : chain.points) {
                    appendLanePoint(
                        finger,
                        transform_math::localPointToWorld(
                            rawToSeatedWorld,
                            pointWorld));
                }

                if (!liveLandmarks.valid ||
                    !commandedDirectionsValid ||
                    finger >= liveLandmarks.fingers.size() ||
                    !liveLandmarks.fingers[finger].valid ||
                    !std::isfinite(
                        liveLandmarks.fingers[finger].length) ||
                    liveLandmarks.fingers[finger].length <=
                        0.0001f) {
                    continue;
                }
                const RE::NiPoint3 seatedBase =
                    transform_math::localPointToWorld(
                        rawToSeatedWorld,
                        liveLandmarks.fingers[finger].base);
                const auto sweepCurve =
                    grab_finger_pose_math::
                        makeBakedCalibratedFingerCurve<
                            RE::NiPoint3>(
                            finger,
                            isLeft,
                            capturedFingerSnapshot.inPowerArmor,
                            seatedBase,
                            seatedSweepNormal,
                            commandedOpenDirectionsWorld[finger],
                            liveLandmarks.fingers[finger].length);
                const auto* tipProbe =
                    sweepCurve.probeCount > 0 ?
                    &sweepCurve.probes[0] :
                    nullptr;
                if (!tipProbe || tipProbe->sampleCount == 0 ||
                    tipProbe->sampleCount >
                        tipProbe->samples.size()) {
                    continue;
                }
                constexpr std::size_t kSweepSamples = 7;
                const RE::NiPoint3 curveNormal =
                    grab_finger_pose_runtime::normalizedOrFallback(
                        sweepCurve.normal,
                        seatedSweepNormal);
                const RE::NiPoint3 curveZero =
                    grab_finger_pose_runtime::normalizedOrFallback(
                        sweepCurve.zeroAngleVector,
                        commandedOpenDirectionsWorld[finger]);
                for (std::size_t sample = 0;
                     sample < kSweepSamples;
                     ++sample) {
                    const std::size_t row =
                        sample * (tipProbe->sampleCount - 1) /
                        (kSweepSamples - 1);
                    const auto& baked = tipProbe->samples[row];
                    const RE::NiPoint3 arm =
                        grab_finger_pose_math::rotateAroundUnitAxis(
                            curveZero,
                            curveNormal,
                            baked.angleRadians);
                    appendLanePoint(
                        finger,
                        grab_finger_pose_math::add(
                            sweepCurve.center,
                            grab_finger_pose_math::scale(
                                arm,
                                baked.reachLength)));
                }
            }
        }

        std::array<WeaponCollision::SupportGripEvidenceView,
            MAX_WEAPON_COLLISION_BODIES>
            compositeEvidenceViews{};
        std::size_t compositeEvidenceViewCount = 0;
        std::size_t sourceTriangleCount = 0;
        if (g_rockConfig.rockGrabMeshFingerPoseEnabled) {
            const std::size_t discoveredViewCount =
                weaponCollision.findSupportGripEvidenceViews(
                    weaponNode,
                    compositeEvidenceViews);
            for (std::size_t index = 0;
                 index < discoveredViewCount;
                 ++index) {
                const auto& candidateView =
                    compositeEvidenceViews[index];
                if (candidateView.weaponGenerationKey !=
                        decision.weaponGenerationKey ||
                    candidateView.weaponGenerationKey !=
                        _activeWeaponGenerationKey ||
                    candidateView.localTriangles.empty()) {
                    continue;
                }
                if (compositeEvidenceViewCount != index) {
                    compositeEvidenceViews[
                        compositeEvidenceViewCount] = candidateView;
                }
                sourceTriangleCount +=
                    candidateView.localTriangles.size();
                ++compositeEvidenceViewCount;
            }
            if (compositeEvidenceViewCount == 0 &&
                cachedTrianglesFound) {
                compositeEvidenceViews[0] = evidenceView;
                compositeEvidenceViewCount = 1;
                sourceTriangleCount =
                    evidenceView.localTriangles.size();
            }
            selectNearestSupportGripFingerTriangles(
                std::span<const WeaponCollision::SupportGripEvidenceView>(
                    compositeEvidenceViews.data(),
                    compositeEvidenceViewCount),
                weaponNode->world,
                fingerReferenceSet,
                grab_finger_pose_runtime::
                    kMaxFingerPoseCandidateTriangles,
                fingerScratch.rankings,
                fingerScratch.localTriangles);
        }
        performance_profiler::observeValue(
            performance_profiler::ValueMetric::
                EquippedWeaponFingerPoseSourceTriangles,
            static_cast<std::uint64_t>(sourceTriangleCount));
        performance_profiler::observeValue(
            performance_profiler::ValueMetric::
                EquippedWeaponFingerPoseSelectedTriangles,
            static_cast<std::uint64_t>(
                fingerScratch.localTriangles.size()));

        grab_finger_pose_runtime::SolvedGrabFingerPose meshFingerPose{};
        const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPosePtr = nullptr;
        std::array<float, 5> capturedFingerSplayRadians{};
        const std::array<float, 5>* capturedFingerSplayRadiansPtr = nullptr;
        bool spatialIndexBuilt = false;
        bool commandedOpenDirectionsValid = false;
        if (g_rockConfig.rockGrabMeshFingerPoseEnabled && !fingerScratch.localTriangles.empty()) {
            const RE::NiTransform seatedToRawWorld =
                transform_math::composeTransforms(
                    handTransform,
                    transform_math::invertTransform(
                        adjustedHandTransform));
            const RE::NiTransform frozenMeshWorld = weapon_two_handed_grip_math::virtualizeMeshForSeatedHand(
                weaponNode->world,
                handTransform,
                adjustedHandTransform);
            const RE::NiPoint3 frozenGripPoint = weapon_two_handed_grip_math::virtualizeWorldPointForSeatedHand(
                gripWorldPoint,
                handTransform,
                adjustedHandTransform);
            const RE::NiPoint3 frozenGripNormal =
                transform_math::localVectorToWorld(
                    seatedToRawWorld,
                    grip.grabNormalWorld);
            auto fingerPoseTargets = grab_finger_pose_runtime::makeSharedGripPoseTarget(frozenGripPoint, frozenGripNormal);
            fingerPoseTargets.useSeatPointForMissingTargets = false;
            fingerPoseTargets.useWholeMeshForMissingTargets = true;
            /*
             * Equipped support keeps the indexed frozen base, but owns its
             * presentation policy. Loose-grab thumb/index clearing and generic
             * pad-probe refinement erased useful weapon-surface opposition.
             */
            auto frozenSolve =
                grab_finger_pose_runtime::solveFrozenMeshFingerPoseBase(
                fingerScratch.localTriangles,
                frozenMeshWorld,
                handTransform,
                isLeft,
                frozenGripPoint,
                fingerPoseTargets,
                fingerScratch.spatialIndex,
                fingerScratch.worldTriangles,
                grab_finger_pose_runtime::FrozenMeshFingerPoseSolveOptions{
                    .minValue = g_rockConfig.rockGrabFingerMinValue,
                    .maxTriangleDistanceSquared = g_rockConfig.rockGrabMaxTriangleDistance,
                    .rejectBacksideHits = g_rockConfig.rockGrabFingerRejectBacksideHits,
                    .surfacePlaneToleranceGameUnits = g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits,
                    .allowSurfaceAimTargets = true,
                    .sweepContactRadiusGameUnits = g_rockConfig.rockGrabFingerSweepContactRadiusGameUnits,
                    .thumbSweepMaxOpenValue = g_rockConfig.rockGrabThumbSweepMaxOpenValue,
                    .fingerSweepMaxOpenValue = g_rockConfig.rockGrabFingerSweepMaxOpenValue,
                    .meshFingerPoseEnabled = g_rockConfig.rockGrabMeshFingerPoseEnabled,
                    .captureSweepDebug = false,
                },
                capturedFingerSnapshotValid ?
                    &capturedFingerSnapshot :
                    nullptr);
            grab_finger_pose_runtime::captureSurfaceAimObjectLocal(
                frozenSolve.pose,
                frozenMeshWorld);
            spatialIndexBuilt = frozenSolve.spatialIndexBuilt;
            commandedOpenDirectionsValid = frozenSolve.commandedOpenDirectionsValid;
            meshFingerPose = frozenSolve.pose;
            performance_profiler::observeValue(
                performance_profiler::ValueMetric::EquippedWeaponFingerPoseSpatialNodeVisits,
                meshFingerPose.spatialNodeVisitCount);
            performance_profiler::observeValue(
                performance_profiler::ValueMetric::EquippedWeaponFingerPoseTriangleTests,
                meshFingerPose.spatialTriangleTestCount);
            if (meshFingerPose.solved) {
                const bool completeDirectFingerEvidence =
                    grab_finger_pose_runtime::
                        hasCompleteFingerContactEvidence(
                            meshFingerPose);
                const auto oppositionConfig =
                    currentWeaponOppositionPocketConfig();
                const auto oppositionPocket =
                    !completeDirectFingerEvidence &&
                            oppositionConfig.enabled &&
                            frozenSolve.liveFingerSnapshotValid ?
                        grab_finger_pose_runtime::
                            findLocalOppositionPocketEvidence(
                                fingerScratch.worldTriangles,
                                frozenSolve.liveFingerSnapshot,
                                frozenGripPoint,
                                meshFingerPose.contactValidMask,
                                oppositionConfig.
                                    minFingerGapGameUnits,
                                (std::max)(
                                    oppositionConfig.
                                        maxFingerGapGameUnits,
                                    WEAPON_OPPOSITION_MAX_FINGER_GAP_GAME_UNITS),
                                oppositionConfig.
                                    maxPocketDistanceGameUnits,
                                (std::min)(
                                    WEAPON_OPPOSITION_SEGMENT_PROBE_RADIUS_GAME_UNITS,
                                    (std::max)(
                                        0.0f,
                                        g_rockConfig.
                                            rockGrabFingerSweepContactRadiusGameUnits))) :
                        grab_finger_pose_runtime::
                            OppositionPocketEvidence{};
                if (oppositionPocket.valid) {
                    applyStableWeaponOppositionPose(
                        meshFingerPose,
                        oppositionConfig,
                        oppositionPocket.opposedFingerIndex);
                    ROCK_LOG_INFO(
                        Weapon,
                        "TwoHandedGrip: local opposition pocket accepted hand={} kind={} directMask=0x{:02X} endpointMask=0x{:02X} directEndpoints=0x{:02X} gap={:.3f} gripSurfaceDistance={:.3f}",
                        isLeft ? "left" : "right",
                        grab_finger_pose_runtime::
                            oppositionPocketKindName(
                                oppositionPocket.kind),
                        static_cast<unsigned>(
                            meshFingerPose.contactValidMask),
                        static_cast<unsigned>(
                            oppositionPocket.endpointMask),
                        static_cast<unsigned>(
                            oppositionPocket.directEndpointMask),
                        oppositionPocket.fingerGapGameUnits,
                        oppositionPocket.
                            gripToSurfaceDistanceGameUnits);
                }
                const bool completeFingerEvidence =
                    completeDirectFingerEvidence ||
                    oppositionPocket.valid;
                if (completeFingerEvidence) {
                    meshFingerPosePtr = &meshFingerPose;
                } else {
                    ROCK_LOG_INFO(
                        Weapon,
                        "TwoHandedGrip: mesh finger pose failed closed hand={} contactMask=0x{:02X} requiredMask=0x{:02X} hits={} sources={} sourceTriangles={} candidateTriangles={}",
                        isLeft ? "left" : "right",
                        static_cast<unsigned>(
                            meshFingerPose.contactValidMask),
                        static_cast<unsigned>(
                            grab_finger_pose_runtime::
                                kCompleteFingerContactMask),
                        meshFingerPose.hitCount,
                        compositeEvidenceViewCount,
                        sourceTriangleCount,
                        meshFingerPose.candidateTriangleCount);
                }
                if (completeDirectFingerEvidence &&
                    frozenSolve.liveFingerSnapshotValid &&
                    grab_finger_pose_runtime::buildSurfaceContactSplayValues(
                        meshFingerPose,
                        frozenSolve.liveFingerSnapshot,
                        capturedFingerSplayRadians)) {
                    capturedFingerSplayRadiansPtr = &capturedFingerSplayRadians;
                }
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: mesh finger pose hand={} values=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) hits={} contactMask=0x{:02X} sources={} sourceTris={} candidateTris={} spatial={} nodes={} tests={} commandedAnchors={} altThumb={} thumbLane={}",
                    isLeft ? "left" : "right",
                    meshFingerPose.values[0],
                    meshFingerPose.values[1],
                    meshFingerPose.values[2],
                    meshFingerPose.values[3],
                    meshFingerPose.values[4],
                    meshFingerPose.hitCount,
                    static_cast<unsigned>(
                        meshFingerPose.contactValidMask),
                    compositeEvidenceViewCount,
                    sourceTriangleCount,
                    meshFingerPose.candidateTriangleCount,
                    spatialIndexBuilt ? "yes" : "no",
                    meshFingerPose.spatialNodeVisitCount,
                    meshFingerPose.spatialTriangleTestCount,
                    commandedOpenDirectionsValid ? "yes" : "no",
                    meshFingerPose.usedAlternateThumbCurve ? "yes" : "no",
                    grab_finger_pose_math::thumbLaneName(meshFingerPose.selectedThumbLane));
                if (meshFingerPose.hasThumbCurveDiagnostics) {
                    ROCK_LOG_DEBUG(Weapon,
                        "TwoHandedGrip: thumb curve primary(hit={} value={:.2f} behind={}) opposition(hit={} value={:.2f} behind={}) sidePad(hit={} value={:.2f} behind={}) selected={}",
                        meshFingerPose.thumbPrimaryCurve.hit ? "yes" : "no",
                        meshFingerPose.thumbPrimaryCurve.value,
                        meshFingerPose.thumbPrimaryCurve.openedByBehindContact ? "yes" : "no",
                        meshFingerPose.thumbAlternateCurve.hit ? "yes" : "no",
                        meshFingerPose.thumbAlternateCurve.value,
                        meshFingerPose.thumbAlternateCurve.openedByBehindContact ? "yes" : "no",
                        meshFingerPose.thumbSidePadCurve.hit ? "yes" : "no",
                        meshFingerPose.thumbSidePadCurve.value,
                        meshFingerPose.thumbSidePadCurve.openedByBehindContact ? "yes" : "no",
                        grab_finger_pose_math::thumbLaneName(meshFingerPose.selectedThumbLane));
                }
            }
        }

        setSupportGripPose(
            isLeft,
            meshFingerPosePtr,
            capturedFingerSplayRadiansPtr,
            SupportGripPoseFallback::FullyClosed);
        if (meshFingerPosePtr && grip.hasFingerPose) {
            std::array<RE::NiTransform, 15> localTransforms{};
            std::uint16_t localTransformMask = 0;
            const auto handPose = grip.hasFingerSplay ?
                frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose, grip.fingerSplayRadians) :
                frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose);
            if (buildFullHandLocalTransformsForMeshPose(
                    isLeft,
                    *meshFingerPosePtr,
                    handPose,
                    capturedFingerBoneSnapshotValid ?
                        &capturedFingerBoneSnapshot :
                        nullptr,
                    localTransforms,
                    localTransformMask)) {
                grip.fingerLocalTransforms = localTransforms;
                grip.fingerLocalTransformMask = localTransformMask;
                grip.hasFingerLocalTransforms = true;
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: full-hand local transform override prepared hand={} mask=0x{:04X}",
                    isLeft ? "left" : "right",
                    grip.fingerLocalTransformMask);
            }
        }

        grip.visualLerp = {};
        grip.active = true;
        if (isLeft) {
            _hapticEvents.leftPartGripCaptured = true;
        } else {
            _hapticEvents.rightPartGripCaptured = true;
        }

        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: part grip captured hand={} weapon='{}' gripLocal=({:.3f},{:.3f},{:.3f}) meshGrab={} sourceTriangles={} sources={} contactedTriangles={} fingerTriangles={} cachedTriangles={} sourceNodeCurrent={} surfaceSeat={:.2f}deg authoredSupport=NO acquisition={} authority={} provider={} attachOnly={} authoredCandidate={} authoredFrame={} authoredIdentity={} authoredGeneration={} authoredSurface={} surfaceDistance={:.3f} surfaceRadius={:.3f} authoredFingerMask=0x{:04X} touchToAuthoredSeat={:.3f} authoredSeatLocal=({:.3f},{:.3f},{:.3f}) touchProbeLocal=({:.3f},{:.3f},{:.3f}) frameError={:.4f} partKind={} pose={} generation={:016X}",
            isLeft ? "left" : "right",
            weaponNode->name.c_str(),
            grip.gripLocal.x,
            grip.gripLocal.y,
            grip.gripLocal.z,
            useDynamicHandoffGrip ?
                "HANDOFF" :
                (meshFound ? "YES" : "FALLBACK"),
            sourceTriangleCount,
            compositeEvidenceViewCount,
            contactedSourceTriangleCount,
            fingerScratch.localTriangles.size(),
            cachedTrianglesFound ? "yes" : "no",
            cachedTrianglesFound && evidenceView.sourceNodeCurrent ? "yes" : "no",
            grip.surfaceSeatRotationRadians *
                RADIANS_TO_DEGREES,
            weaponInteractionAcquisitionSourceName(
                decision.acquisitionSource),
            _authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport ?
                "visual-only" :
                "full",
            providerPartAuthority.active ? "yes" : "no",
            grip.attachOnly ? "yes" : "no",
            authoredSupportCandidateForHandValid ? "yes" : "no",
            authoredSupportFrameValid ? "yes" : "no",
            authoredWeaponIdentityMatches ? "yes" : "no",
            authoredGenerationMatches ? "yes" : "no",
            authoredSeatWeaponSurfaceValid ? "yes" : "no",
            authoredSupportSurfaceDistance,
            g_rockConfig.rockWeaponInteractionTouchRadius,
            authoredSupportFingerLocalTransformMask,
            authoredSupportTouchProbeDistance,
            authoredSupportPalmWeaponLocal.x,
            authoredSupportPalmWeaponLocal.y,
            authoredSupportPalmWeaponLocal.z,
            authoredSupportProximity.liveTouchProbeWeaponLocal.x,
            authoredSupportProximity.liveTouchProbeWeaponLocal.y,
            authoredSupportProximity.liveTouchProbeWeaponLocal.z,
            authoredSupportProximity.frameAgreementErrorGameUnits,
            static_cast<int>(grip.partKind),
            static_cast<int>(grip.gripPose),
            _activeWeaponGenerationKey);
        return selectionDecision.selection;
    }
}
