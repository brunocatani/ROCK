#include "physics-interaction/weapon/TwoHandedGripInternal.h"
#include "physics-interaction/hand/HandFingerMirrorMath.h"

// Support-hand grip: authored capability qualification and selection, dynamic
// acquisition, the capture transaction (capturePartGrip, whose finger solve
// lives in SupportGripFingerSolve.cpp), grip formation (transitionToGripping),
// release, and rebind across collision generations.
//
// updateFullWeaponAuthorityGrip also lives here deliberately: the full
// two-hand solve only runs while a support grip holds the weapon, so its
// state and lifetime are owned by this module.

namespace rock
{
    bool TwoHandedGrip::tryResolveAuthoredSupportActivationAxes(
        RE::NiNode* weaponNode,
        const RE::NiTransform& weaponWorld,
        const std::uint64_t currentWeaponGenerationKey,
        const authored_weapon_grip_activation_policy::HandTopology
            handTopology,
        RE::NiPoint3& outSupportSideAxisWorld,
        RE::NiPoint3& outDownAxisWorld,
        RE::NiPoint3& outReferenceAxisWorld) const
    {
        outSupportSideAxisWorld = {};
        outDownAxisWorld = {};
        outReferenceAxisWorld = {};

        using authored_weapon_grip_activation_policy::HandTopology;
        if (handTopology == HandTopology::Invalid ||
            !weaponNode || currentWeaponGenerationKey == 0 ||
            !isInvertibleTransform(weaponWorld) ||
            !_firing.hasRightCanonicalHandWeaponLocal ||
            _firing.rightCanonicalSource !=
                RightFiringCanonicalSource::AuthoredAnimation ||
            _firing.rightCanonicalWeaponNode != weaponNode ||
            _firing.rightCanonicalGenerationKey !=
                currentWeaponGenerationKey ||
            !isFiniteTransform(_firing.rightCanonicalHandWeaponLocal)) {
            return false;
        }

        const auto normalizeVector = [](const RE::NiPoint3& input,
                                         RE::NiPoint3& output) {
            output = {};
            const float lengthSquared =
                input.x * input.x +
                input.y * input.y +
                input.z * input.z;
            if (!std::isfinite(lengthSquared) ||
                lengthSquared <= 0.000001f) {
                return false;
            }
            const float inverseLength = 1.0f / std::sqrt(lengthSquared);
            output = RE::NiPoint3{
                input.x * inverseLength,
                input.y * inverseLength,
                input.z * inverseLength,
            };
            return std::isfinite(output.x) &&
                   std::isfinite(output.y) &&
                   std::isfinite(output.z);
        };

        /*
         * The native authored frame defines the RIGHT-fire/LEFT-support
         * activation axes. The LEFT-fire/RIGHT-support topology must not reuse
         * that left-facing cone. Reflect both axes through weapon-local X, the
         * same bilateral plane used by the mirrored right support seat. This
         * keeps LEFT and RIGHT topology independent while preserving DOWN and
         * the full swept cone as an exact geometric mirror.
         */
        const RE::NiTransform rightFiringHandWorld =
            transform_math::composeTransforms(
                weaponWorld,
                _firing.rightCanonicalHandWeaponLocal);
        if (!isFiniteTransform(rightFiringHandWorld)) {
            return false;
        }

        RE::NiPoint3 nativeLeftAxisWorld{};
        if (!normalizeVector(
                computePalmNormalFromHandBasis(
                    rightFiringHandWorld,
                    false),
                nativeLeftAxisWorld)) {
            return false;
        }
        const RE::NiPoint3 nativeThumbUpWorld =
            transformHandspaceDirection(
                rightFiringHandWorld,
                RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
                false);
        const float thumbSideProjection =
            nativeThumbUpWorld.x * nativeLeftAxisWorld.x +
            nativeThumbUpWorld.y * nativeLeftAxisWorld.y +
            nativeThumbUpWorld.z * nativeLeftAxisWorld.z;
        const RE::NiPoint3 orthogonalUpWorld{
            nativeThumbUpWorld.x -
                nativeLeftAxisWorld.x * thumbSideProjection,
            nativeThumbUpWorld.y -
                nativeLeftAxisWorld.y * thumbSideProjection,
            nativeThumbUpWorld.z -
                nativeLeftAxisWorld.z * thumbSideProjection,
        };
        RE::NiPoint3 normalizedUpWorld{};
        if (!normalizeVector(orthogonalUpWorld, normalizedUpWorld)) {
            return false;
        }
        const RE::NiPoint3 nativeDownAxisWorld{
            -normalizedUpWorld.x,
            -normalizedUpWorld.y,
            -normalizedUpWorld.z,
        };

        const auto orientAxisForTopology = [&](
                                               const RE::NiPoint3&
                                                   rightTopologyAxisWorld,
                                               RE::NiPoint3& outAxisWorld) {
            const RE::NiPoint3 rightTopologyAxisWeaponLocal =
                transform_math::worldVectorToLocal(
                    weaponWorld,
                    rightTopologyAxisWorld);
            const auto orientedAxisWeaponLocal =
                authored_weapon_grip_activation_policy::
                    orientRightFiringAxisForTopology(
                        authored_weapon_grip_activation_policy::Vec3{
                            rightTopologyAxisWeaponLocal.x,
                            rightTopologyAxisWeaponLocal.y,
                            rightTopologyAxisWeaponLocal.z,
                        },
                        handTopology);
            return normalizeVector(
                transform_math::localVectorToWorld(
                    weaponWorld,
                    RE::NiPoint3{
                        orientedAxisWeaponLocal.x,
                        orientedAxisWeaponLocal.y,
                        orientedAxisWeaponLocal.z,
                    }),
                outAxisWorld);
        };
        if (!orientAxisForTopology(
                nativeLeftAxisWorld,
                outSupportSideAxisWorld) ||
            !orientAxisForTopology(
                nativeDownAxisWorld,
                outDownAxisWorld)) {
            return false;
        }

        const RE::NiPoint3 referenceAxis{
            outSupportSideAxisWorld.y * outDownAxisWorld.z -
                outSupportSideAxisWorld.z * outDownAxisWorld.y,
            outSupportSideAxisWorld.z * outDownAxisWorld.x -
                outSupportSideAxisWorld.x * outDownAxisWorld.z,
            outSupportSideAxisWorld.x * outDownAxisWorld.y -
                outSupportSideAxisWorld.y * outDownAxisWorld.x,
        };
        return normalizeVector(referenceAxis, outReferenceAxisWorld);
    }

    void TwoHandedGrip::refreshAuthoredSupportGripActivationState(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const WeaponCollision& weaponCollision)
    {
        _support.authoredDebugSnapshot = {};
        const bool collectPoseEvidence =
            g_rockConfig.rockDebugDrawAuthoredGripActivationZones;

        const auto& candidate = _support.authoredCandidate;
        if (!weaponNode ||
            !candidate.valid ||
            candidate.weaponNode != weaponNode ||
            candidate.weaponGenerationKey == 0 ||
            candidate.weaponGenerationKey != currentWeaponGenerationKey ||
            candidate.captureSequence == 0 ||
            !isFiniteTransform(weaponNode->world)) {
            return;
        }

        const bool supportHandIsLeft = isSupportHandLeft();
        const auto handTopology =
            authored_weapon_grip_activation_policy::resolveHandTopology(
                isFiringHandLeft(),
                supportHandIsLeft);
        if (_support.lastStableDirectionGenerationKey !=
                candidate.weaponGenerationKey ||
            _support.lastStableDirectionCaptureSequence !=
                candidate.captureSequence ||
            _support.lastStableDirectionHandTopology !=
                handTopology) {
            _support.lastStableApproachDirectionWorld = {};
            _support.lastStableDirectionGenerationKey =
                candidate.weaponGenerationKey;
            _support.lastStableDirectionCaptureSequence =
                candidate.captureSequence;
            _support.lastStableDirectionHandTopology = handTopology;
            _support.lastStableApproachDirectionValid = false;
        }

        RE::NiTransform authoredSupportHandWeaponLocal{};
        std::array<RE::NiTransform, 15> authoredSupportFingerLocalTransforms{};
        std::uint16_t authoredSupportFingerLocalTransformMask = 0;
        if (!tryResolveAuthoredSupportGripCandidateForHand(
                supportHandIsLeft,
                weaponNode,
                candidate.weaponGenerationKey,
                authoredSupportHandWeaponLocal,
                authoredSupportFingerLocalTransforms,
                authoredSupportFingerLocalTransformMask)) {
            return;
        }

        RE::NiTransform liveSupportHandWorld{};
        if (!tryGetSolverHandTransform(supportHandIsLeft, liveSupportHandWorld)) {
            return;
        }

        /*
         * While both hands hold a left-fired weapon under full authority,
         * weaponNode->world is the wand-aimed basis pre-write that the
         * two-hand solve overrides later this frame. The seat, its cone, and
         * the touch gate belong on the weapon the player sees, so use the
         * rendered record there; every other state renders the node pose.
         */
        const bool nodeHoldsBasisPreWrite =
            _session.state == TwoHandedState::Gripping &&
            usesLeftFiringCarry() &&
            ownsWeaponTransform() &&
            _visuals.hasLastRenderedWeaponWorld &&
            isFiniteTransform(_visuals.lastRenderedWeaponWorld);
        const RE::NiTransform activationWeaponWorld =
            nodeHoldsBasisPreWrite ?
            _visuals.lastRenderedWeaponWorld :
            weaponNode->world;

        AuthoredSupportPalmSeatProximity proximity{};
        if (!resolveAuthoredSupportPalmSeatProximity(
                activationWeaponWorld,
                liveSupportHandWorld,
                authoredSupportHandWeaponLocal,
                supportHandIsLeft,
                proximity)) {
            return;
        }

        auto& snapshot = _support.authoredDebugSnapshot;
        snapshot.weaponWorld = activationWeaponWorld;
        snapshot.authoredPalmSeatWeaponLocal =
            proximity.authoredPalmSeatWeaponLocal;
        snapshot.authoredPalmSeatWorld = proximity.authoredPalmSeatWorld;
        snapshot.liveTouchProbeWeaponLocal =
            proximity.liveTouchProbeWeaponLocal;
        snapshot.liveTouchProbeWorld = proximity.liveTouchProbeWorld;
        snapshot.weaponRelativeDistanceGameUnits =
            proximity.weaponRelativeDistanceGameUnits;
        snapshot.worldReadbackDistanceGameUnits =
            proximity.worldReadbackDistanceGameUnits;
        snapshot.frameAgreementErrorGameUnits =
            proximity.frameAgreementErrorGameUnits;
        snapshot.touchRadiusGameUnits =
            g_rockConfig.rockWeaponInteractionTouchRadius;
        snapshot.radialCapGameUnits =
            g_rockConfig.rockWeaponInteractionProbeRadius;
        snapshot.weaponGenerationKey = candidate.weaponGenerationKey;
        snapshot.captureSequence = candidate.captureSequence;
        snapshot.supportHandIsLeft = supportHandIsLeft;
        snapshot.handTopology = handTopology;
        snapshot.insideTouchRadius =
            proximity.weaponRelativeDistanceGameUnits <=
            snapshot.touchRadiusGameUnits;

        const auto identity = weaponCollision.getEquippedWeaponClassification();
        snapshot.weaponFormID = identity.formID;
        snapshot.effectiveEquipSlotFormID =
            identity.effectiveEquipSlotFormID;
        snapshot.baseEquipSlotFormID = identity.baseEquipSlotFormID;
        snapshot.effectiveEquipSlotUsesInstanceData =
            identity.effectiveEquipSlotUsesInstanceData;
        const bool meleeOrUnarmed =
            identity.sizeClass == WeaponSizeClass::Melee;
        snapshot.weaponFamily =
            authored_weapon_grip_activation_policy::resolveWeaponFamily(
                authored_weapon_grip_activation_policy::WeaponFamilyInput{
                    .effectiveEquipSlotFormID =
                        identity.effectiveEquipSlotFormID,
                    .equippedWeaponPresent = identity.hasEquippedWeapon,
                    .meleeOrUnarmed = meleeOrUnarmed,
                });

        snapshot.canonicalAxesValid =
            tryResolveAuthoredSupportActivationAxes(
                weaponNode,
                activationWeaponWorld,
                currentWeaponGenerationKey,
                handTopology,
                snapshot.supportSideAxisWorld,
                snapshot.downAxisWorld,
                snapshot.referenceAxisWorld);

        using ActivationVec3 =
            authored_weapon_grip_activation_policy::Vec3;
        const auto toActivationVector = [](const RE::NiPoint3& value) {
            return ActivationVec3{ value.x, value.y, value.z };
        };
        const auto gate =
            authored_weapon_grip_activation_policy::evaluateDirectionGate(
                authored_weapon_grip_activation_policy::DirectionGateInput{
                    .weaponFamily = snapshot.weaponFamily,
                    .handTopology = handTopology,
                    .authoredSeatWorld = toActivationVector(
                        snapshot.authoredPalmSeatWorld),
                    .liveProbeWorld = toActivationVector(
                        snapshot.liveTouchProbeWorld),
                    .supportSideAxisWorld = toActivationVector(
                        snapshot.supportSideAxisWorld),
                    .downAxisWorld = toActivationVector(
                        snapshot.downAxisWorld),
                    .lastStableDirectionWorld = toActivationVector(
                        _support.lastStableApproachDirectionWorld),
                    .radialCapGameUnits = snapshot.radialCapGameUnits,
                    .lastStableDirectionValid =
                        _support.lastStableApproachDirectionValid,
                });
        snapshot.approachDirectionWorld = RE::NiPoint3{
            gate.approachDirectionWorld.x,
            gate.approachDirectionWorld.y,
            gate.approachDirectionWorld.z,
        };
        snapshot.supportSideDot = gate.supportSideDot;
        snapshot.downDot = gate.downDot;
        snapshot.sweptArcDot = gate.sweptArcDot;
        snapshot.selectedRegion = gate.selectedRegion;
        snapshot.classifierSupported = gate.familySupported;
        snapshot.directionUsedLastStableSample =
            gate.usedLastStableDirection;
        snapshot.radialPass = gate.radialPass;
        snapshot.directionPass = gate.directionPass;
        snapshot.topologyPass = gate.topologyPass;
        snapshot.activationSpatialPass = gate.spatialPass;
        if (gate.directionValid &&
            gate.radialDistanceGameUnits >=
                authored_weapon_grip_activation_policy::
                    kMinimumDirectionDistanceGameUnits) {
            _support.lastStableApproachDirectionWorld =
                snapshot.approachDirectionWorld;
            _support.lastStableApproachDirectionValid = true;
        }

        if (collectPoseEvidence) {
            snapshot.poseEvidenceEvaluated = true;
            std::array<RE::NiPoint3,
                AuthoredSupportGripDebugSnapshot::kPoseLandmarkCount>
                surfaceQueryLandmarksWorld{};
            snapshot.poseLandmarksWorld[0] = snapshot.authoredPalmSeatWorld;
            surfaceQueryLandmarksWorld[0] =
                transform_math::localPointToWorld(
                    weaponNode->world,
                    snapshot.authoredPalmSeatWeaponLocal);
            constexpr std::array<std::size_t, 5> kDistalFingerLocalIndices{
                2, 5, 8, 11, 14
            };
            for (std::size_t fingerIndex = 0;
                 fingerIndex < kDistalFingerLocalIndices.size();
                 ++fingerIndex) {
                const std::size_t distalIndex =
                    kDistalFingerLocalIndices[fingerIndex];
                const std::size_t chainStart = distalIndex - 2;
                RE::NiTransform fingerWeaponLocal =
                    transform_math::composeTransforms(
                        authoredSupportHandWeaponLocal,
                        authoredSupportFingerLocalTransforms[chainStart]);
                fingerWeaponLocal = transform_math::composeTransforms(
                    fingerWeaponLocal,
                    authoredSupportFingerLocalTransforms[chainStart + 1]);
                fingerWeaponLocal = transform_math::composeTransforms(
                    fingerWeaponLocal,
                    authoredSupportFingerLocalTransforms[distalIndex]);
                snapshot.poseLandmarksWorld[fingerIndex + 1] =
                    transform_math::localPointToWorld(
                        activationWeaponWorld,
                        fingerWeaponLocal.translate);
                surfaceQueryLandmarksWorld[fingerIndex + 1] =
                    transform_math::localPointToWorld(
                        weaponNode->world,
                        fingerWeaponLocal.translate);
            }

            std::array<WeaponCollision::WeaponSurfaceProximityWitness,
                AuthoredSupportGripDebugSnapshot::kPoseLandmarkCount>
                poseWitnesses{};
            (void)weaponCollision.findCurrentWeaponSurfaceNearPoints(
                weaponNode,
                surfaceQueryLandmarksWorld,
                snapshot.touchRadiusGameUnits,
                poseWitnesses);
            for (std::size_t landmarkIndex = 0;
                 landmarkIndex < poseWitnesses.size();
                 ++landmarkIndex) {
                const auto& witness = poseWitnesses[landmarkIndex];
                if (!witness.valid ||
                    witness.weaponGenerationKey != currentWeaponGenerationKey) {
                    continue;
                }
                snapshot.poseSurfaceWitnessMask |=
                    static_cast<std::uint8_t>(1u << landmarkIndex);
                ++snapshot.poseSurfaceWitnessCount;
                snapshot.poseSurfaceWitnessWorld[landmarkIndex] =
                    transform_math::localPointToWorld(
                        activationWeaponWorld,
                        transform_math::worldPointToLocal(
                            weaponNode->world,
                            witness.closestPointWorld));
                snapshot.poseSurfaceDistanceGameUnits[landmarkIndex] =
                    witness.distanceGameUnits;
            }
            snapshot.poseEvidencePass =
                (snapshot.poseSurfaceWitnessMask & 0x01u) != 0 &&
                snapshot.poseSurfaceWitnessCount >= 3;
        }
        const auto& activeSupportGrip = partGrip(supportHandIsLeft);
        snapshot.currentSupportGripActive = activeSupportGrip.active;
        snapshot.currentAuthoredSupportGripActive =
            activeSupportGrip.active && activeSupportGrip.authoredSupportGrip;
        snapshot.valid = true;
    }

    void TwoHandedGrip::resetAuthoredSupportCapability(const char* reason)
    {
        if (_support.authoredCapability.initialized) {
            ROCK_LOG_DEBUG(
                Weapon,
                "TwoHandedGrip: authored support capability reset reason={} capability={} detail={} generation={:016X}",
                reason ? reason : "unknown",
                authored_support_grab_policy::capabilityName(
                    _support.authoredCapability.capability),
                authored_support_grab_policy::capabilityReasonName(
                    _support.authoredCapability.reason),
                _support.authoredCapability.weaponGenerationKey);
        }
        _support.authoredCapability = {};
        _support.lastSelection = {};
        _support.lastSelectionGenerationKey = 0;
        _support.lastSelectionHandIsLeft = true;
        _support.lastSelectionValid = false;
    }

    void TwoHandedGrip::synchronizeAuthoredSupportCapabilityIdentity(
        RE::NiNode* weaponNode,
        const std::uint64_t weaponOwnershipKey,
        const std::uint64_t weaponGenerationKey,
        const authored_weapon_grip_activation_policy::HandTopology handTopology)
    {
        using authored_weapon_grip_activation_policy::HandTopology;
        const bool identityValid = weaponNode &&
            weaponOwnershipKey != 0 &&
            weaponGenerationKey != 0 &&
            handTopology != HandTopology::Invalid;
        if (!identityValid) {
            resetAuthoredSupportCapability("identity-unavailable");
            _support.authoredCapability.reason =
                authored_support_grab_policy::CapabilityReason::AwaitingIdentity;
            return;
        }

        const auto& state = _support.authoredCapability;
        if (state.initialized &&
            state.weaponNodeIdentity == weaponNode &&
            state.weaponOwnershipKey == weaponOwnershipKey &&
            state.weaponGenerationKey == weaponGenerationKey &&
            state.handTopology == handTopology) {
            return;
        }

        resetAuthoredSupportCapability("weapon-or-topology-boundary");
        _support.authoredCapability = AuthoredSupportCapabilityState{
            .weaponNodeIdentity = weaponNode,
            .weaponOwnershipKey = weaponOwnershipKey,
            .weaponGenerationKey = weaponGenerationKey,
            .handTopology = handTopology,
            .capability = authored_support_grab_policy::Capability::Pending,
            .reason = authored_support_grab_policy::
                CapabilityReason::AwaitingQualification,
            .initialized = true,
        };
        ROCK_LOG_DEBUG(
            Weapon,
            "TwoHandedGrip: authored support capability qualification started ownership={:016X} generation={:016X} topology={}",
            weaponOwnershipKey,
            weaponGenerationKey,
            authored_weapon_grip_activation_policy::handTopologyName(
                handTopology));
    }

    void TwoHandedGrip::advanceAuthoredSupportCapabilityQualification(
        const bool ready,
        const float deltaSeconds)
    {
        auto& state = _support.authoredCapability;
        if (!state.initialized) {
            return;
        }
        if (!ready) {
            // Absence becomes fallback authority only after continuously ready
            // elapsed time. Reload/equip gaps therefore cannot spend the
            // qualification budget while authored capture is unavailable.
            if (state.capability ==
                authored_support_grab_policy::Capability::Pending) {
                state.readySeconds = 0.0f;
            }
            state.usableCandidateMissingSeconds = 0.0f;
            return;
        }
        if (state.capability ==
            authored_support_grab_policy::Capability::Usable) {
            const bool supportHandIsLeft = state.handTopology ==
                authored_weapon_grip_activation_policy::HandTopology::
                    RightFiringLeftSupport;
            RE::NiTransform supportHandWeaponLocal{};
            std::array<RE::NiTransform, 15> supportFingerLocals{};
            std::uint16_t supportFingerMask = 0;
            const bool candidateStillResolves =
                tryResolveAuthoredSupportGripCandidateForHand(
                    supportHandIsLeft,
                    state.weaponNodeIdentity,
                    state.weaponGenerationKey,
                    supportHandWeaponLocal,
                    supportFingerLocals,
                    supportFingerMask) &&
                supportFingerMask == 0x7FFFu;
            if (candidateStillResolves) {
                state.usableCandidateMissingSeconds = 0.0f;
                return;
            }

            state.usableCandidateMissingSeconds =
                authored_support_grab_policy::
                    advanceContinuousEvidenceSeconds(
                        state.usableCandidateMissingSeconds,
                        deltaSeconds,
                        authored_support_grab_policy::
                            kUsableCandidateLossSeconds,
                        true);
            if (state.usableCandidateMissingSeconds >=
                authored_support_grab_policy::kUsableCandidateLossSeconds) {
                state.readySeconds = 0.0f;
                state.usableCandidateMissingSeconds = 0.0f;
                setAuthoredSupportCapability(
                    authored_support_grab_policy::Capability::Pending,
                    authored_support_grab_policy::
                        CapabilityReason::AwaitingCandidate);
            }
            return;
        }

        if (state.capability ==
            authored_support_grab_policy::Capability::Pending) {
            state.readySeconds = authored_support_grab_policy::
                advanceContinuousEvidenceSeconds(
                    state.readySeconds,
                    deltaSeconds,
                    authored_support_grab_policy::kQualificationSeconds,
                    true);
        }
    }

    void TwoHandedGrip::setAuthoredSupportCapability(
        const authored_support_grab_policy::Capability capability,
        const authored_support_grab_policy::CapabilityReason reason)
    {
        auto& state = _support.authoredCapability;
        const bool capabilityChanged = state.capability != capability;
        const bool reasonChanged = state.reason != reason;
        if (!capabilityChanged && !reasonChanged) {
            return;
        }

        state.capability = capability;
        state.reason = reason;
        if (capabilityChanged &&
            capability != authored_support_grab_policy::Capability::Pending) {
            ROCK_LOG_INFO(
                Weapon,
                "TwoHandedGrip: authored support capability={} reason={} ownership={:016X} generation={:016X} topology={} ready={:.3f}s",
                authored_support_grab_policy::capabilityName(capability),
                authored_support_grab_policy::capabilityReasonName(reason),
                state.weaponOwnershipKey,
                state.weaponGenerationKey,
                authored_weapon_grip_activation_policy::handTopologyName(
                    state.handTopology),
                state.readySeconds);
        } else {
            ROCK_LOG_DEBUG(
                Weapon,
                "TwoHandedGrip: authored support capability={} reason={} generation={:016X}",
                authored_support_grab_policy::capabilityName(capability),
                authored_support_grab_policy::capabilityReasonName(reason),
                state.weaponGenerationKey);
        }
    }

    void TwoHandedGrip::observeAuthoredSupportCapability(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey)
    {
        using authored_weapon_grip_activation_policy::WeaponFamily;
        using authored_support_grab_policy::Capability;

        auto& state = _support.authoredCapability;
        const bool identityCurrent = state.initialized &&
            state.weaponNodeIdentity == weaponNode &&
            state.weaponGenerationKey == currentWeaponGenerationKey;
        const auto& candidate = _support.authoredCandidate;
        const bool candidatePublished = identityCurrent &&
            candidate.valid &&
            candidate.weaponNode == weaponNode &&
            candidate.weaponGenerationKey == currentWeaponGenerationKey &&
            candidate.captureSequence != 0;

        const bool supportHandIsLeft = isSupportHandLeft();
        RE::NiTransform supportHandWeaponLocal{};
        std::array<RE::NiTransform, 15> supportFingerLocals{};
        std::uint16_t supportFingerMask = 0;
        const bool candidateResolved = candidatePublished &&
            tryResolveAuthoredSupportGripCandidateForHand(
                supportHandIsLeft,
                weaponNode,
                currentWeaponGenerationKey,
                supportHandWeaponLocal,
                supportFingerLocals,
                supportFingerMask);

        const auto& snapshot = _support.authoredDebugSnapshot;
        const bool snapshotCurrent = snapshot.valid &&
            snapshot.supportHandIsLeft == supportHandIsLeft &&
            snapshot.weaponGenerationKey == currentWeaponGenerationKey &&
            snapshot.captureSequence == candidate.captureSequence;
        const bool familyKnown = snapshotCurrent &&
            snapshot.weaponFamily != WeaponFamily::Unknown;
        const auto observation =
            authored_support_grab_policy::observeCapability(
                authored_support_grab_policy::CapabilityObservationInput{
                    .modeEnabled =
                        _handlingSettings.authoredOnlySupportGrabsEnabled,
                    .identityCurrent = identityCurrent,
                    .qualificationExpired =
                        state.readySeconds >=
                        authored_support_grab_policy::kQualificationSeconds,
                    .candidatePublished = candidatePublished,
                    .candidateResolvedForSupportHand = candidateResolved,
                    .weaponFamilyKnown = familyKnown,
                    .weaponFamilySupported = snapshotCurrent &&
                        snapshot.classifierSupported,
                    .canonicalAxesValid = snapshotCurrent &&
                        snapshot.canonicalAxesValid,
                    .completeFingerPose = candidateResolved &&
                        supportFingerMask == 0x7FFFu,
                });

        // A proven usable pose stays authoritative across the short frame-local
        // capture gaps bridged by advanceAuthoredSupportCapabilityQualification.
        // Positive captured data always recovers immediately, including after
        // an unavailable verdict.
        if (state.capability == Capability::Usable &&
            observation.capability != Capability::Usable &&
            !candidateResolved) {
            // The timed candidate-loss path owns the transition back to
            // Pending. A single missing frame never revokes authored authority.
        } else {
            setAuthoredSupportCapability(
                observation.capability,
                observation.reason);
        }

        if (_support.authoredDebugSnapshot.valid) {
            _support.authoredDebugSnapshot.authoredCapability =
                state.capability;
            _support.authoredDebugSnapshot.authoredCapabilityReason =
                state.reason;
            _support.authoredDebugSnapshot.authoredCapabilityReadySeconds =
                state.readySeconds;
            if (_support.lastSelectionValid) {
                _support.authoredDebugSnapshot.lastSelection =
                    _support.lastSelection.selection;
                _support.authoredDebugSnapshot.lastSelectionReason =
                    _support.lastSelection.reason;
            }
        }
    }

    void TwoHandedGrip::recordSupportGrabSelection(
        const authored_support_grab_policy::SelectionDecision& decision,
        const bool isLeft,
        const std::uint64_t weaponGenerationKey)
    {
        const bool changed = !_support.lastSelectionValid ||
            _support.lastSelection.selection != decision.selection ||
            _support.lastSelection.reason != decision.reason ||
            _support.lastSelectionGenerationKey != weaponGenerationKey ||
            _support.lastSelectionHandIsLeft != isLeft;
        _support.lastSelection = decision;
        _support.lastSelectionGenerationKey = weaponGenerationKey;
        _support.lastSelectionHandIsLeft = isLeft;
        _support.lastSelectionValid = true;

        if (_support.authoredDebugSnapshot.valid) {
            _support.authoredDebugSnapshot.lastSelection =
                decision.selection;
            _support.authoredDebugSnapshot.lastSelectionReason =
                decision.reason;
        }
        if (!changed) {
            return;
        }

        const auto& capability = _support.authoredCapability;
        if (authored_support_grab_policy::captured(decision.selection)) {
            ROCK_LOG_INFO(
                Weapon,
                "TwoHandedGrip: support grab selection={} reason={} hand={} capability={} capabilityReason={} generation={:016X}",
                authored_support_grab_policy::selectionName(
                    decision.selection),
                authored_support_grab_policy::selectionReasonName(
                    decision.reason),
                isLeft ? "left" : "right",
                authored_support_grab_policy::capabilityName(
                    capability.capability),
                authored_support_grab_policy::capabilityReasonName(
                    capability.reason),
                weaponGenerationKey);
        } else {
            ROCK_LOG_DEBUG(
                Weapon,
                "TwoHandedGrip: support grab selection={} reason={} hand={} capability={} capabilityReason={} generation={:016X}",
                authored_support_grab_policy::selectionName(
                    decision.selection),
                authored_support_grab_policy::selectionReasonName(
                    decision.reason),
                isLeft ? "left" : "right",
                authored_support_grab_policy::capabilityName(
                    capability.capability),
                authored_support_grab_policy::capabilityReasonName(
                    capability.reason),
                weaponGenerationKey);
        }
    }

    void TwoHandedGrip::beginDynamicSupportAcquisition(
        const bool supportHandIsLeft,
        const WeaponPartGrip& supportGrip,
        const RE::NiTransform& primaryStartWorld,
        const RE::NiTransform& supportStartWorld)
    {
        if (_support.dynamicAcquisition.active) {
            clearDynamicSupportAcquisition(
                "replaced-by-new-dynamic-support-grip",
                true);
        }

        _support.dynamicAcquisition = {};
        _support.dynamicAcquisition.active = true;
        _support.dynamicAcquisition.supportHandIsLeft = supportHandIsLeft;
        _support.dynamicAcquisition.weaponGenerationKey =
            supportGrip.weaponGenerationKey;
        _support.dynamicAcquisition.gripSequence = supportGrip.gripSequence;
        _support.dynamicAcquisition.primaryStartWorld = primaryStartWorld;
        _support.dynamicAcquisition.supportStartWorld = supportStartWorld;
    }

    void TwoHandedGrip::clearDynamicSupportAcquisition(
        const char* reason,
        const bool logCancellation)
    {
        if (!_support.dynamicAcquisition.active) {
            _support.dynamicAcquisition = {};
            return;
        }

        if (logCancellation) {
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: dynamic support acquisition event=cancel reason={} hand={} grip={} generation={:016X} elapsed={:.3f}s duration={:.3f}s rawAlpha={:.3f} easedAlpha={:.3f} fullCorrection={:.2f}deg appliedCorrection={:.2f}deg primaryPivotError={:.4f} supportTargetError={:.4f}",
                reason ? reason : "unknown",
                _support.dynamicAcquisition.supportHandIsLeft ?
                    "left" :
                    "right",
                _support.dynamicAcquisition.gripSequence,
                _support.dynamicAcquisition.weaponGenerationKey,
                _support.dynamicAcquisition.elapsedSeconds,
                _support.dynamicAcquisition.durationSeconds,
                _support.dynamicAcquisition.rawAlpha,
                _support.dynamicAcquisition.easedAlpha,
                _support.dynamicAcquisition.fullCorrectionRadians *
                    RADIANS_TO_DEGREES,
                _support.dynamicAcquisition.lastAppliedRotationRadians *
                    RADIANS_TO_DEGREES,
                _support.dynamicAcquisition.lastPrimaryPivotError,
                _support.dynamicAcquisition.lastSupportTargetError);
        }
        _support.dynamicAcquisition = {};
    }

    bool TwoHandedGrip::dynamicSupportAcquisitionMatches(
        const bool supportHandIsLeft,
        const WeaponPartGrip& supportGrip) const
    {
        return _support.dynamicAcquisition.active &&
               _support.dynamicAcquisition.supportHandIsLeft ==
                   supportHandIsLeft &&
               _support.dynamicAcquisition.weaponGenerationKey != 0 &&
               _support.dynamicAcquisition.weaponGenerationKey ==
                   _session.weaponGenerationKey &&
               _support.dynamicAcquisition.weaponGenerationKey ==
                   supportGrip.weaponGenerationKey &&
               _support.dynamicAcquisition.gripSequence != 0 &&
               _support.dynamicAcquisition.gripSequence ==
                   supportGrip.gripSequence;
    }

    RE::NiTransform
    TwoHandedGrip::resolveDynamicSupportAcquisitionHandTarget(
        const RE::NiTransform& targetWorld,
        const bool primaryHand,
        LockedHandVisualLerpState& visualState)
    {
        const RE::NiTransform& startWorld =
            primaryHand ?
                _support.dynamicAcquisition.primaryStartWorld :
                _support.dynamicAcquisition.supportStartWorld;
        visualState.initialized = true;
        visualState.startWorld = startWorld;
        visualState.elapsedSeconds =
            _support.dynamicAcquisition.elapsedSeconds;
        visualState.durationSeconds =
            _support.dynamicAcquisition.durationSeconds;
        visualState.lastAlpha =
            _support.dynamicAcquisition.easedAlpha;
        return hand_visual_lerp_math::interpolateTransform(
            startWorld,
            targetWorld,
            _support.dynamicAcquisition.easedAlpha);
    }

    void TwoHandedGrip::lockPartGripToWeaponRoot(bool isLeft)
    {
        /*
         * Part-carry feeds its own solved weapon transform back as the next
         * frame's base, so grips must resolve exclusively through the captured
         * weapon-root frames while it is active. Following live part-node
         * chains lets any per-frame part animation integrate into a steady
         * carry drift and pulls the locked hand visuals apart (verified by
         * telemetry: rigid-weapon grip separation grew frame over frame).
         */
        WeaponPartGrip& grip = partGrip(isLeft);
        grip.hasSourceFrames = false;
        grip.hasAttachmentWeaponLocal = false;
    }

    void TwoHandedGrip::releasePartGrip(bool isLeft, const char* reason, const bool smoothHandReturn)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        if (!grip.active) {
            return;
        }
        if (dynamicSupportAcquisitionMatches(isLeft, grip)) {
            clearDynamicSupportAcquisition(reason, true);
        }
        if (smoothHandReturn) {
            beginHandVisualReturn(isLeft, reason);
        }
        clearSupportGripPose(isLeft);
        grip = {};
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: part grip released hand={} reason={}", isLeft ? "left" : "right", reason ? reason : "unknown");
    }

    void TwoHandedGrip::transitionToGripping(
        RE::NiNode* weaponNode,
        const WeaponInteractionDecision& decision,
        const WeaponCollision& weaponCollision,
        weapon_support_authority_policy::WeaponSupportAuthorityMode supportAuthorityMode,
        bool firingGripProximityAuthorityEnabled,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const WeaponProviderPartAuthority& providerPartAuthority)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::TwoHandedGripStart);

        if (!weaponNode || currentEquippedWeaponOwnershipKey == 0) {
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: support grip acquisition rejected before commit because weapon identity is unavailable");
            return;
        }

        const bool supportHandIsLeft = isSupportHandLeft();
        const bool primaryHandIsLeft = isFiringHandLeft();
        RE::NiTransform primaryTransform{};
        if (!tryGetSolverHandTransform(
                primaryHandIsLeft,
                primaryTransform)) {
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: support grip acquisition rejected before commit because the primary hand frame is unavailable");
            return;
        }
        if (!primaryHandIsLeft) {
            (void)captureRightNativeWeaponAimFrame(
                weaponNode,
                decision.weaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                weaponCollision.getCurrentEquippedWeaponInstanceContentKey());
        }

        /*
         * Position-only authored mode must also reuse the canonical: the
         * solver hand deliberately reports the physical (natural) wrist while
         * ROCK presents the authored seat, so recapturing the relation from
         * it would rebase the whole two-hand hold - presented seat and
         * PAPER's manual-cycle baseline - onto the un-authored wrist frame.
         */
        const bool reuseRightFiringCanonicalGrip =
            scope_safe_hand_frame_math::
                shouldReuseRightFiringCanonicalGrip(
                    _scope.menuOpenThisFrame,
                    isFiringHandLeft(),
                    hasRightFiringHandCanonicalFrame(
                        weaponNode,
                        decision.weaponGenerationKey,
                        currentEquippedWeaponOwnershipKey),
                    _firing.rightCanonicalGenerationKey,
                    decision.weaponGenerationKey) ||
            (usesNativeRightCarry() &&
                (_firing.rightCanonicalSource == RightFiringCanonicalSource::AuthoredAnimation ||
                    decision.acquisitionSource == WeaponInteractionAcquisitionSource::FiringGripZone) &&
                hasRightFiringHandCanonicalFrame(
                    weaponNode,
                    decision.weaponGenerationKey,
                    currentEquippedWeaponOwnershipKey));
        if (_scope.menuOpenThisFrame && usesNativeRightCarry() &&
            !reuseRightFiringCanonicalGrip) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "TwoHandedGrip: scoped support grip acquisition deferred before commit because the matching pre-scope firing grip is unavailable generation={:016X} canonicalGeneration={:016X}",
                decision.weaponGenerationKey,
                _firing.rightCanonicalGenerationKey);
            return;
        }

        RE::NiTransform nativeWeaponLocalBaseline = weaponNode->local;
        if (_visuals.returningWeapon.localTransition.active && _visuals.returningWeapon.weaponNode == weaponNode) {
            nativeWeaponLocalBaseline = _visuals.returningWeapon.nativeBaselineLocal;
        }

        /*
         * A LEFT firing hand entering a two-handed grip KEEPS its captured
         * authored hand seat and grip point; native weapon aim remains in its
         * separate generation-bound frame. Recapturing from the live hand both
         * replaced that authored hold with the momentary squeeze orientation
         * (round-2 arm break) and rebased the promotion grip point onto
         * whatever pose the node carried at grab time (round-4 role theft).
         * The right hand recaptures as before - its frames deliberately ride
         * FRIK's authored carry and feed the canonical snapshot.
         */
        // A direct handoff must capture against the same firing seat used by
        // its admission cylinder, without rebasing it onto the live wrist.
        const bool keepFiringHold = _firing.hasPrimaryHandWeaponLocal &&
            (usesLeftFiringCarry() ||
                (decision.acquisitionSource == WeaponInteractionAcquisitionSource::FiringGripZone &&
                    _session.weaponNode == weaponNode &&
                    _session.weaponGenerationKey == decision.weaponGenerationKey &&
                    _session.equippedWeaponOwnershipKey == currentEquippedWeaponOwnershipKey));

        const auto previousAuthorityMode = _session.authorityMode;
        RE::NiNode* const previousActiveWeaponNode = _session.weaponNode;
        const std::uint64_t previousWeaponGenerationKey =
            _session.weaponGenerationKey;
        const std::uint64_t previousWeaponOwnershipKey =
            _session.equippedWeaponOwnershipKey;
        const RE::NiTransform previousWeaponLocalBaseline =
            _weaponNodeLocalBaseline;
        const bool previousHasWeaponLocalBaseline =
            _hasWeaponNodeLocalBaseline;
        const RE::NiPoint3 previousPrimaryGripLocal = _firing.primaryGripLocal;
        const RE::NiTransform previousPrimaryHandWeaponLocal =
            _firing.primaryHandWeaponLocal;
        const bool previousHasFiringHandWeaponLocal =
            _firing.hasPrimaryHandWeaponLocal;
        const float previousPrimaryGripConfidence =
            _firing.primaryGripConfidence;
        const std::uint64_t previousFiringGripSequence =
            _session.firingGripSequence;
        const auto previousPartGrips = _support.partGrips;
        const TwoHandedGripHapticEvents previousHapticEvents =
            _hapticEvents;
        const AuthoredSupportGripDebugSnapshot
            previousAuthoredSupportGripDebugSnapshot =
                _support.authoredDebugSnapshot;
        const float previousLockedGripSeparationWorld =
            _support.lockedGripSeparationWorld;
        const auto rollbackPreparedAcquisition = [&]() {
            _session.authorityMode = previousAuthorityMode;
            _session.weaponNode = previousActiveWeaponNode;
            _session.weaponGenerationKey = previousWeaponGenerationKey;
            _session.equippedWeaponOwnershipKey =
                previousWeaponOwnershipKey;
            _weaponNodeLocalBaseline = previousWeaponLocalBaseline;
            _hasWeaponNodeLocalBaseline =
                previousHasWeaponLocalBaseline;
            _firing.primaryGripLocal = previousPrimaryGripLocal;
            _firing.primaryHandWeaponLocal = previousPrimaryHandWeaponLocal;
            _firing.hasPrimaryHandWeaponLocal =
                previousHasFiringHandWeaponLocal;
            _firing.primaryGripConfidence = previousPrimaryGripConfidence;
            _session.firingGripSequence = previousFiringGripSequence;
            _support.partGrips = previousPartGrips;
            _hapticEvents = previousHapticEvents;
            _support.authoredDebugSnapshot =
                previousAuthoredSupportGripDebugSnapshot;
            _support.lockedGripSeparationWorld =
                previousLockedGripSeparationWorld;
        };

        _session.authorityMode = supportAuthorityMode;
        _session.weaponNode = weaponNode;
        _session.weaponGenerationKey = decision.weaponGenerationKey;
        _session.equippedWeaponOwnershipKey = currentEquippedWeaponOwnershipKey;
        _weaponNodeLocalBaseline = nativeWeaponLocalBaseline;
        _hasWeaponNodeLocalBaseline = true;
        if (!keepFiringHold) {
            _firing.primaryGripConfidence = 0.0f;
            _firing.hasPrimaryHandWeaponLocal = false;
        }
        if (reuseRightFiringCanonicalGrip && !keepFiringHold) {
            _firing.primaryHandWeaponLocal = _firing.rightCanonicalHandWeaponLocal;
            _firing.primaryGripLocal = _firing.rightCanonicalGripWeaponLocal;
        }

        const RE::NiPoint3 primaryPalmPos =
            reuseRightFiringCanonicalGrip ? weaponLocalToWorld(_firing.primaryGripLocal, weaponNode) : computeGrabLegacyPalmPivotAWorldFromHandBasis(primaryTransform, primaryHandIsLeft);
        if (!keepFiringHold) {
            if (!reuseRightFiringCanonicalGrip) {
                _firing.primaryGripLocal = worldToWeaponLocal(primaryPalmPos, weaponNode);
                _firing.primaryHandWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), primaryTransform);
            }
            _firing.primaryGripConfidence = 1.0f;
            _firing.hasPrimaryHandWeaponLocal = true;
        }

        /*
         * At capture the firing grip point is the primary palm, so the support
         * palm distance selects visual-only attachment near the firing grip or
         * full two-handed manipulation farther out. This applies uniformly to
         * equipped weapons and is bypassed by explicit provider grab modes.
         * If the distance cannot be measured, retain full authority rather
         * than assuming the hand is inside the proximity radius.
         */
        if (firingGripProximityAuthorityEnabled) {
            RE::NiTransform supportTransform{};
            if (tryGetSolverHandTransform(supportHandIsLeft, supportTransform)) {
                const RE::NiPoint3 supportPalmPos = computeGrabLegacyPalmPivotAWorldFromHandBasis(supportTransform, supportHandIsLeft);
                const RE::NiPoint3 supportToGrip = sub(primaryPalmPos, supportPalmPos);
                const float supportPalmToGripDistance = std::sqrt(dot(supportToGrip, supportToGrip));
                if (std::isfinite(supportPalmToGripDistance)) {
                    _session.authorityMode = weapon_support_authority_policy::resolveFiringGripProximityAuthorityMode(
                        supportPalmToGripDistance,
                        _handlingSettings.firingGripProximitySupportRadiusGameUnits);
                    ROCK_LOG_INFO(Weapon,
                        "TwoHandedGrip: firing-grip proximity support distance={:.2f} radius={:.2f} mode={}",
                        supportPalmToGripDistance,
                        _handlingSettings.firingGripProximitySupportRadiusGameUnits,
                        _session.authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport ?
                            "visual-only" :
                            "full-authority");
                }
            }
        }

        RE::NiTransform supportCaptureHandWorld{};
        const auto partGripCapture = capturePartGrip(
                supportHandIsLeft,
                weaponNode,
                decision,
                weaponCollision,
                providerPartAuthority,
                firingGripProximityAuthorityEnabled,
                true,
                &supportCaptureHandWorld);
        if (!authored_support_grab_policy::captured(partGripCapture)) {
            rollbackPreparedAcquisition();
            if (partGripCapture ==
                authored_support_grab_policy::Selection::Reject) {
                ROCK_LOG_SAMPLE_DEBUG(
                    Weapon,
                    1000,
                    "TwoHandedGrip: support grip acquisition rejected by authored-only policy");
            } else {
                ROCK_LOG_WARN(
                    Weapon,
                    "TwoHandedGrip: support grip acquisition rolled back because part-grip capture failed");
            }
            return;
        }

        const RE::NiPoint3 supportGripWorldPoint = resolvePartGripWorld(partGrip(supportHandIsLeft), weaponNode);
        const RE::NiPoint3 primaryToSupportWorld = sub(supportGripWorldPoint, primaryPalmPos);
        _support.lockedGripSeparationWorld = std::sqrt(dot(primaryToSupportWorld, primaryToSupportWorld));

        WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        const bool useDynamicSupportAcquisition =
            weapon_support_authority_policy::
                shouldUseDynamicSupportAcquisition(
                    _session.authorityMode,
                    supportGrip.authoredSupportGrip,
                    supportGrip.providerPartAuthority.active,
                    supportGrip.attachOnly);
        if (useDynamicSupportAcquisition &&
            !initializeDynamicSupportBaseline(
                weaponNode,
                supportHandIsLeft,
                "support-attach")) {
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: support grip start failed closed because the zero-delta dynamic baseline could not be captured hand={} grip={} generation={:016X}",
                supportHandIsLeft ? "left" : "right",
                supportGrip.gripSequence,
                supportGrip.weaponGenerationKey);
            rollbackPreparedAcquisition();
            return;
        }
        const bool dynamicBaselineActive =
            isDynamicSupportBaselineActive(
                supportHandIsLeft,
                supportGrip);
        RE::NiTransform dynamicPrimaryStartWorld{};
        RE::NiTransform dynamicSupportStartWorld{};
        if (useDynamicSupportAcquisition) {
            const auto& primaryReturn =
                _visuals.returningHands[
                    primaryHandIsLeft ? 0u : 1u]
                    .transition;
            const auto& supportReturn =
                _visuals.returningHands[
                    supportHandIsLeft ? 0u : 1u]
                    .transition;
            dynamicPrimaryStartWorld =
                primaryReturn.active &&
                    isUsableHandAuthorityTransform(
                        primaryReturn.lastApplied) ?
                primaryReturn.lastApplied :
                primaryTransform;
            dynamicSupportStartWorld =
                supportReturn.active &&
                    isUsableHandAuthorityTransform(
                        supportReturn.lastApplied) ?
                supportReturn.lastApplied :
                supportCaptureHandWorld;
        }

        if (_visuals.returningWeapon.localTransition.active &&
            _visuals.returningWeapon.weaponNode == weaponNode) {
            clearWeaponVisualReturn(
                "new-two-hand-acquisition",
                true,
                true);
        }
        clearDynamicSupportAcquisition(
            "new-two-hand-acquisition",
            true);
        resetLockedHandVisualLerp();
        clearPrimaryGripFingerPose(primaryHandIsLeft);
        clearPrimaryGripWorldAuthority(primaryHandIsLeft);
        /*
         * capturePartGrip has already committed the new support finger data
         * into this hand's WeaponPartGrip. Do not run the support-pose release
         * helper here: it erases the captured scalar and exact local
         * transforms before the alpha-zero publication below. The existing
         * role tag is replaced by publishGripHandPoses in the same update, and
         * applyPartGripLockedVisual replaces its world authority.
         */
        if (!keepFiringHold) {
            _session.firingGripSequence = ++_session.gripCaptureSequence;
            // A right-hand capture here rides hFRIK's authored carry. Commit
            // the canonical snapshot only after every support baseline is
            // valid, so a failed acquisition cannot replace the prior carry.
            rememberRightFiringHandCanonicalFrame(
                weaponCollision.getCurrentEquippedWeaponInstanceContentKey());
        }

        clearLeftFiringSupportReleaseReturn("support-grip-started");
        _session.state = TwoHandedState::Gripping;
        _partCarry.detachAuthority =
            immersive_weapon_policy::DetachAuthority::None;
        _support.rotationBlend = 0.0f;
        _gripLogCounter = 0;
        _support.gripAgeSeconds = 0.0f;
        _support.freshGripDeferLogged = false;

        const char* supportBaselineName = "inactive";
        if (dynamicBaselineActive) {
            supportBaselineName = "dynamic";
        } else if (supportGrip.authoredSupportGrip &&
                   _session.authorityMode == weapon_support_authority_policy::
                                         WeaponSupportAuthorityMode::
                                             FullTwoHandedSolver) {
            supportBaselineName =
                supportGrip.disableAuthoredSupportNormalTwist ?
                "authored-position-only" :
                "authored-ramped";
        }
        const bool visualOnlyRecoilAssist =
            hasVisualOnlySupportRecoilAssist();
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: grip active weapon='{}', "
            "primaryLocal=({:.3f},{:.3f},{:.3f}), supportLocal=({:.3f},{:.3f},{:.3f}), "
            "gripSeparation={:.3f}, primaryGripSource={}, primaryGripConfidence={:.2f}, partKind={}, pose={}, authorityMode={}, recoilAssist={}, supportBaseline={}, generation={:016X}",
            weaponNode->name.c_str(), _firing.primaryGripLocal.x, _firing.primaryGripLocal.y, _firing.primaryGripLocal.z, supportGrip.gripLocal.x, supportGrip.gripLocal.y, supportGrip.gripLocal.z,
            _support.lockedGripSeparationWorld, reuseRightFiringCanonicalGrip ? "pre-scope-canonical" : (_scope.menuOpenThisFrame ? "frik-driver-reconstructed" : "root-flattened"),
            _firing.primaryGripConfidence, static_cast<int>(supportGrip.partKind), static_cast<int>(supportGrip.gripPose), static_cast<int>(_session.authorityMode), visualOnlyRecoilAssist ? "enabled" : "disabled", supportBaselineName, _session.weaponGenerationKey);

        if (useDynamicSupportAcquisition) {
            beginDynamicSupportAcquisition(
                supportHandIsLeft,
                supportGrip,
                dynamicPrimaryStartWorld,
                dynamicSupportStartWorld);
            /*
             * Capture and first publication are one transaction. dt=0 keeps
             * alpha exactly zero while publishing the frozen finger pose, the
             * pivot-preserving one-hand weapon frame, and both live hand roots
             * before this update returns to PhysicsInteraction.
             */
            updateFullWeaponAuthorityGrip(weaponNode, 0.0f);
        } else if (_session.authorityMode == weapon_support_authority_policy::
                                         WeaponSupportAuthorityMode::
                                             FullTwoHandedSolver &&
                   supportGrip.authoredSupportGrip &&
                   !supportGrip.providerPartAuthority.active &&
                   !supportGrip.attachOnly) {
            /*
             * Authored support keeps its established independent hand-seat
             * interpolation. Publish one exact alpha-zero weapon frame now so
             * its position and axis-aim corrections begin from the firing-hand
             * carry instead of appearing one frame later as an authority
             * refresh.
             */
            updateFullWeaponAuthorityGrip(weaponNode, 0.0f);
        }
    }

    bool TwoHandedGrip::providerPartAuthorityStillCurrent(WeaponPartGrip& grip, std::uint64_t currentWeaponGenerationKey)
    {
        if (!grip.providerPartAuthority.active) {
            return true;
        }
        if (currentWeaponGenerationKey == 0 || currentWeaponGenerationKey != grip.providerPartAuthority.weaponGenerationKey) {
            return false;
        }

        ::rock::provider::RockProviderWeaponPartTargetQueryV1 query{};
        query.weaponGenerationKey = grip.providerPartAuthority.weaponGenerationKey;
        query.bodyId = grip.providerPartAuthority.bodyId;
        query.partKind = grip.providerPartAuthority.partKind;
        query.reloadRole = grip.providerPartAuthority.reloadRole;
        query.supportRole = grip.providerPartAuthority.supportRole;
        query.socketRole = grip.providerPartAuthority.socketRole;
        query.actionRole = grip.providerPartAuthority.actionRole;
        query.sourceRoot = grip.providerPartAuthority.sourceRoot;
        std::memcpy(query.sourceName, grip.providerPartAuthority.sourceName.data(), grip.providerPartAuthority.sourceName.size());
        query.sourceName[sizeof(query.sourceName) - 1] = '\0';

        ::rock::provider::RockProviderWeaponPartTargetResolutionV1 resolution{};
        if (!::rock::provider::resolveWeaponPartTargetV1(query, resolution)) {
            return false;
        }
        return resolution.matched != 0 &&
               resolution.ownerToken == grip.providerPartAuthority.ownerToken &&
               resolution.groupId == grip.providerPartAuthority.groupId &&
               static_cast<std::uint32_t>(resolution.grabMode) == grip.providerPartAuthority.grabMode;
    }

    bool TwoHandedGrip::providerPartTargetNewlyMatchesGrip(const WeaponPartGrip& grip, std::uint64_t currentWeaponGenerationKey) const
    {
        /*
         * Upgrade twin of providerPartAuthorityStillCurrent: a support grip
         * captured WITHOUT provider authority whose own part NOW resolves to
         * a matched provider target — a consumer armed its whitelist while
         * the hand was already holding the part (PAPER_Toolkit: pulling the
         * trigger mid-hold switches an authority grab to attach-only). The
         * caller releases the grip; the still-held grab recaptures within a
         * couple of frames under the new resolution, through the same
         * re-resolve path the downgrade direction uses when a target
         * disappears mid-grip. The query is built from the grip's own
         * captured contact identity, not the live contact, so a flickering
         * contact cannot convert against the wrong part.
         */
        if (!grip.active || grip.providerPartAuthority.active) {
            return false;
        }
        if (currentWeaponGenerationKey == 0 || currentWeaponGenerationKey != grip.weaponGenerationKey) {
            return false;
        }

        ::rock::provider::RockProviderWeaponPartTargetQueryV1 query{};
        query.weaponGenerationKey = grip.weaponGenerationKey;
        query.bodyId = grip.contactBodyId;
        query.partKind = static_cast<std::uint32_t>(grip.partKind);
        query.reloadRole = static_cast<std::uint32_t>(grip.reloadRole);
        query.supportRole = static_cast<std::uint32_t>(grip.supportRole);
        query.socketRole = static_cast<std::uint32_t>(grip.socketRole);
        query.actionRole = static_cast<std::uint32_t>(grip.actionRole);
        query.sourceRoot = reinterpret_cast<std::uintptr_t>(grip.attachmentRoot);
        std::memcpy(query.sourceName, grip.sourceName.data(), grip.sourceName.size());
        query.sourceName[sizeof(query.sourceName) - 1] = '\0';

        ::rock::provider::RockProviderWeaponPartTargetResolutionV1 resolution{};
        return ::rock::provider::resolveWeaponPartTargetV1(query, resolution) && resolution.matched != 0;
    }

    bool TwoHandedGrip::tryRebindPartGripToCurrentGeneration(
        WeaponPartGrip& grip,
        std::uint64_t currentWeaponGenerationKey,
        const WeaponCollision& weaponCollision)
    {
        if (!grip.active) {
            return true;
        }

        if (grip.authoredSupportGrip) {
            // Authored grips are weapon-root-local pose data. A generated-body
            // rebuild changes only the collision generation; it cannot revoke
            // or relocate the authored seat that is already being held.
            grip.weaponGenerationKey = currentWeaponGenerationKey;
            grip.contactBodyId = 0x7FFF'FFFFu;
            grip.attachmentRoot = _session.weaponNode;
            grip.supportInputBaseline = {};
            return true;
        }

        WeaponCollisionProfileEvidenceDescriptor bestDescriptor{};
        RE::NiAVObject* bestSourceNode = nullptr;
        int bestScore = 0;
        float bestDistanceSquared = (std::numeric_limits<float>::max)();
        bool bestAmbiguous = false;
        const std::string_view capturedSourceName{ grip.sourceName.data() };
        const auto distanceSquaredToBounds = [&grip](const WeaponEvidenceBounds3& bounds) {
            if (!bounds.valid) {
                return (std::numeric_limits<float>::max)();
            }
            const auto axisDistance = [](float value, float minimum, float maximum) {
                if (value < minimum) {
                    return minimum - value;
                }
                if (value > maximum) {
                    return value - maximum;
                }
                return 0.0f;
            };
            const float dx = axisDistance(grip.gripLocal.x, bounds.min.x, bounds.max.x);
            const float dy = axisDistance(grip.gripLocal.y, bounds.min.y, bounds.max.y);
            const float dz = axisDistance(grip.gripLocal.z, bounds.min.z, bounds.max.z);
            return dx * dx + dy * dy + dz * dz;
        };
        const auto bodyCount = weaponCollision.getWeaponBodyCount();
        for (std::uint32_t i = 0; i < bodyCount; ++i) {
            const auto bodyId = weaponCollision.getWeaponBodyIdAtomic(i);
            WeaponCollisionProfileEvidenceDescriptor descriptor{};
            RE::NiAVObject* sourceNode = nullptr;
            if (!weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(bodyId, descriptor, sourceNode) ||
                !descriptor.valid || descriptor.weaponGenerationKey != currentWeaponGenerationKey) {
                continue;
            }

            const bool sourcePointerMatches = sourceNode && sourceNode == grip.attachmentRoot;
            const bool sourceNameMatches = !capturedSourceName.empty() && descriptor.sourceName == capturedSourceName;
            if ((!sourcePointerMatches && !sourceNameMatches) || descriptor.semantic.partKind != grip.partKind) {
                continue;
            }
            if (grip.omodFormId != 0 && descriptor.omodFormId != grip.omodFormId) {
                continue;
            }
            if (grip.attachPointFormId != 0 && descriptor.semantic.attachPointFormId != grip.attachPointFormId) {
                continue;
            }

            const int score = sourcePointerMatches ? 2 : 1;
            const float distanceSquared = distanceSquaredToBounds(descriptor.localBoundsGame);
            constexpr float kDistanceTieEpsilon = 0.0001f;
            if (score > bestScore ||
                (score == bestScore && distanceSquared + kDistanceTieEpsilon < bestDistanceSquared)) {
                bestScore = score;
                bestDistanceSquared = distanceSquared;
                bestAmbiguous = false;
                bestDescriptor = descriptor;
                bestSourceNode = sourceNode;
            } else if (score == bestScore &&
                       (distanceSquared == bestDistanceSquared ||
                           (std::isfinite(distanceSquared) && std::isfinite(bestDistanceSquared) &&
                               std::fabs(distanceSquared - bestDistanceSquared) <= kDistanceTieEpsilon))) {
                bestAmbiguous = true;
            }
        }

        if (bestScore == 0 || bestAmbiguous) {
            ROCK_LOG_WARN(Weapon,
                "TwoHandedGrip: part grip rebind failed closed hand={} generation={:016X} source='{}' part={} omod={:08X} attachPoint={:08X} reason={}",
                (&grip == &_support.partGrips[0]) ? "left" : "right",
                currentWeaponGenerationKey,
                capturedSourceName,
                static_cast<std::uint32_t>(grip.partKind),
                grip.omodFormId,
                grip.attachPointFormId,
                bestScore == 0 ? "missing" : "ambiguous");
            return false;
        }

        grip.weaponGenerationKey = currentWeaponGenerationKey;
        // The support input-to-target relation belongs to the old collision
        // generation. The game-thread reconciliation step recaptures it from
        // current transforms only after the new generation is eligible.
        grip.supportInputBaseline = {};
        grip.contactBodyId = bestDescriptor.bodyId;
        grip.attachmentRoot = grip.authoredSupportGrip ?
            _session.weaponNode :
            (bestSourceNode ? bestSourceNode : grip.attachmentRoot);
        grip.partKind = bestDescriptor.semantic.partKind;
        grip.reloadRole = bestDescriptor.semantic.reloadRole;
        grip.supportRole = bestDescriptor.semantic.supportGripRole;
        grip.socketRole = bestDescriptor.semantic.socketRole;
        grip.actionRole = bestDescriptor.semantic.actionRole;
        grip.omodFormId = bestDescriptor.omodFormId;
        grip.attachPointFormId = bestDescriptor.semantic.attachPointFormId;
        grip.classificationSource = bestDescriptor.semantic.classificationSource;
        const auto copyLength = (std::min)(bestDescriptor.sourceName.size(), grip.sourceName.size() - 1);
        std::memcpy(grip.sourceName.data(), bestDescriptor.sourceName.data(), copyLength);
        grip.sourceName[copyLength] = '\0';

        if (grip.providerPartAuthority.active) {
            grip.providerPartAuthority.weaponGenerationKey = currentWeaponGenerationKey;
            grip.providerPartAuthority.bodyId = bestDescriptor.bodyId;
            grip.providerPartAuthority.sourceRoot = reinterpret_cast<std::uintptr_t>(bestSourceNode);
            grip.providerPartAuthority.partKind = static_cast<std::uint32_t>(grip.partKind);
            grip.providerPartAuthority.reloadRole = static_cast<std::uint32_t>(grip.reloadRole);
            grip.providerPartAuthority.supportRole = static_cast<std::uint32_t>(grip.supportRole);
            grip.providerPartAuthority.socketRole = static_cast<std::uint32_t>(grip.socketRole);
            grip.providerPartAuthority.actionRole = static_cast<std::uint32_t>(grip.actionRole);
            std::memcpy(
                grip.providerPartAuthority.sourceName.data(),
                grip.sourceName.data(),
                grip.providerPartAuthority.sourceName.size());
        }
        return true;
    }

    bool TwoHandedGrip::reconcileCollisionGeneration(
        RE::NiNode* currentWeaponNode,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const WeaponCollision& weaponCollision)
    {
        if (!equipped_weapon_manual_ownership_policy::canPreserveManualOwnership(
                _session.equippedWeaponOwnershipKey,
                currentEquippedWeaponOwnershipKey,
                currentWeaponGenerationKey,
                _session.state != TwoHandedState::PrimaryOnly)) {
            return false;
        }
        if (currentWeaponGenerationKey == 0) {
            // PrimaryOnly rides the native firing-hand attach and can retain
            // ownership while the complete collider set is still building.
            _session.weaponNode = currentWeaponNode;
            _session.weaponGenerationKey = 0;
            _weaponNodeLocalBaseline = currentWeaponNode->local;
            _hasWeaponNodeLocalBaseline = true;
            return rebindLeftCarryFramesToWeapon(
                currentWeaponNode,
                0,
                currentEquippedWeaponOwnershipKey,
                false);
        }

        const bool generationChanged = _session.weaponGenerationKey != currentWeaponGenerationKey;
        const bool weaponRootChanged = _session.weaponNode != currentWeaponNode;
        const bool preserveFiringCanonical =
            generationChanged && !weaponRootChanged &&
            hasRightFiringHandCanonicalFrame(
                currentWeaponNode,
                _session.weaponGenerationKey,
                currentEquippedWeaponOwnershipKey) &&
            _firing.rightCanonicalInstanceContentKey != 0 &&
            _firing.rightCanonicalInstanceContentKey ==
                weaponCollision.getCurrentEquippedWeaponInstanceContentKey();
        if (generationChanged || weaponRootChanged) {
            const auto previousGeneration = _session.weaponGenerationKey;
            _session.weaponNode = currentWeaponNode;
            _session.weaponGenerationKey = currentWeaponGenerationKey;
            if (weaponRootChanged) {
                _weaponNodeLocalBaseline = currentWeaponNode->local;
                _hasWeaponNodeLocalBaseline = true;
            }
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: preserving manual ownership across collision rebuild oldGeneration={:016X} newGeneration={:016X} ownership={:016X} rootChanged={}",
                previousGeneration,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                weaponRootChanged ? "yes" : "no");
        }

        if ((generationChanged || weaponRootChanged) &&
            !rebindLeftCarryFramesToWeapon(
                currentWeaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                true)) {
            return false;
        }

        if (generationChanged || weaponRootChanged) {
            for (auto& grip : _support.partGrips) {
                if (grip.active && !tryRebindPartGripToCurrentGeneration(grip, currentWeaponGenerationKey, weaponCollision)) {
                    return false;
                }
            }
        }
        if (preserveFiringCanonical) {
            // The seat and exact finger poses are weapon-local, independent
            // of generated bodies. Rebind their generation after the carry
            // survives the rebuild; recapturing a detached hand would replace
            // the firing seat with the free hand's current pose.
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: firing canonical rebound across collision rebuild oldGeneration={:016X} newGeneration={:016X} ownership={:016X} content={:016X}",
                _firing.rightCanonicalGenerationKey,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                _firing.rightCanonicalInstanceContentKey);
            _firing.rightCanonicalGenerationKey = currentWeaponGenerationKey;
        }
        return true;
    }

    void TwoHandedGrip::updateFullWeaponAuthorityGrip(RE::NiNode* weaponNode, float dt)
    {
        const bool supportHandIsLeft = isSupportHandLeft();
        const bool primaryHandIsLeft = isFiringHandLeft();
        WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        const bool dynamicBaselineActive =
            isDynamicSupportBaselineActive(
                supportHandIsLeft,
                supportGrip);
        const bool supportInputBaselineActive = dynamicBaselineActive;
        if (supportGrip.supportInputBaseline.active &&
            !supportInputBaselineActive) {
            supportGrip.supportInputBaseline = {};
        }

        bool dynamicAcquisition =
            dynamicSupportAcquisitionMatches(
                supportHandIsLeft,
                supportGrip);
        if (_support.dynamicAcquisition.active &&
            !dynamicAcquisition) {
            clearDynamicSupportAcquisition(
                "grip-or-generation-witness-changed",
                true);
        }
        if (!dynamicAcquisition) {
            _support.rotationBlend = (std::min)(
                1.0f,
                _support.rotationBlend +
                    (std::isfinite(dt) && dt > 0.0f ? dt : 0.0f) *
                        ROTATION_BLEND_SPEED);
        }

        RE::NiTransform primaryTransform{};
        RE::NiTransform supportTransform{};
        RE::NiTransform primaryDriverWorld{};
        const bool primaryTransformAvailable =
            primaryHandIsLeft ?
            tryResolvePhysicalHandFrame(
                true,
                primaryTransform,
                primaryDriverWorld) :
            tryGetSolverHandTransform(false, primaryTransform);
        if (!primaryTransformAvailable ||
            !tryGetSolverHandTransform(
                supportHandIsLeft,
                supportTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing support grip because authoritative hand transforms are unavailable");
            logGripFailureIncident("authoritative-hand-frame-unavailable");
            transitionToInactive(false);
            return;
        }

        /*
         * LEFT_CARRY_CLOCK probe (debug grab-frame logging, left-firing carry
         * with a support hand only): the rendered left arm in this topology is
         * the only pose hFRIK solves from ROCK's driver-based physical frame
         * rather than the root-flattened hand, so the row keeps the tracked
         * left hand and elbow before any publication this frame and compares
         * them with the presented target and the final root pose below.
         */
        const bool leftCarryProbeEnabled =
            g_rockConfig.rockDebugGrabFrameLogging &&
            primaryHandIsLeft &&
            usesLeftFiringCarry();
        const LeftCarryProbeSample leftBonesBefore =
            leftCarryProbeEnabled ? sampleLeftCarryProbe() : LeftCarryProbeSample{};

        RE::NiTransform calibratedPrimaryTransform = primaryTransform;
        RE::NiTransform calibratedSupportTransform = supportTransform;
        bool inputBaselineResolved = true;
        if (dynamicBaselineActive) {
            const auto& primaryDriver =
                _scope.currentHandDriverFrames[
                    primaryHandIsLeft ? 0u : 1u];
            const auto& supportDriver =
                _scope.currentHandDriverFrames[
                    supportHandIsLeft ? 0u : 1u];
            inputBaselineResolved =
                primaryDriver.valid &&
                supportDriver.valid &&
                weapon_support_acquisition_math::
                    tryResolveDynamicSupportDriverTargets(
                        primaryDriver.world,
                        supportGrip.supportInputBaseline.
                            primaryInputToGripTargetLocal,
                        supportDriver.world,
                        supportGrip.supportInputBaseline.
                            inputToGripTargetLocal,
                        calibratedPrimaryTransform,
                        calibratedSupportTransform);
            /*
             * Dynamic support grabs have TWO intended behaviors, selected by
             * the grabbed part kind. This split is deliberate product
             * behavior, not a workaround - keep both paths when refactoring.
             *
             * 1) Delta-preserving parts (Magazine, Bolt, ChargingHandle,
             *    Slide):
             *    the captured tandem relation is kept as-is for the whole
             *    hold. The resolved targets start with zero solver error, so
             *    only post-capture hand deltas move the part/weapon and the
             *    physical controller-to-seat gap from the accept moment is
             *    intentionally preserved. These are manipulation parts: the
             *    player grabs them at a distance and pulls/pushes relative
             *    to where the grab began, and snapping the seat onto the
             *    controller would yank the manipulation stroke.
             *
             * 2) Every other part (foregrip, handguard, barrel, ...): the
             *    hold should converge onto the real hand like authored
             *    grabs, closing the accept-moment gap (alignmentBlend
             *    below). Without this, a detached grab stayed visibly
             *    detached for the whole hold.
             */
            const bool preserveCaptureDelta =
                supportGrip.partKind == WeaponPartKind::Magazine ||
                supportGrip.partKind == WeaponPartKind::Bolt ||
                supportGrip.partKind == WeaponPartKind::ChargingHandle ||
                supportGrip.partKind == WeaponPartKind::Slide;
            if (inputBaselineResolved && !preserveCaptureDelta) {
                /*
                 * Behavior 2: retarget the support input onto the true
                 * physical support hand (raw damped driver x natural-bone
                 * relation; never the rendered hand, which is ROCK's own
                 * output during the hold) so the seat converges onto the
                 * real controller the way authored grips do. The primary
                 * relation stays captured: that hand is already seated and
                 * anchors weapon translation. The blend starts at 0 on the
                 * attach frame (first-publication invariant: attaching must
                 * not move the weapon) and is advanced only after being
                 * consumed. If the physical frame is unavailable the blend
                 * holds and the captured tandem target remains the input
                 * (behavior 1 as the fail-closed state).
                 */
                auto& baseline = supportGrip.supportInputBaseline;
                RE::NiTransform physicalSupportHandWorld{};
                RE::NiTransform physicalSupportDriverWorld{};
                if (tryResolvePhysicalHandFrame(
                        supportHandIsLeft,
                        physicalSupportHandWorld,
                        physicalSupportDriverWorld)) {
                    ROCK_LOG_SAMPLE_INFO(Weapon, 250,
                        "TwoHandedGrip: dynamic support alignment blend={:.3f} capturedGap={:.3f}gu grip={} generation={:016X}",
                        baseline.alignmentBlend,
                        weaponSolverLength(sub(
                            calibratedSupportTransform.translate,
                            physicalSupportHandWorld.translate)),
                        supportGrip.gripSequence,
                        supportGrip.weaponGenerationKey);
                    if (baseline.alignmentBlend > 0.0f) {
                        calibratedSupportTransform =
                            scope_safe_hand_frame_math::
                                interpolateRebaseTransform(
                                    calibratedSupportTransform,
                                    physicalSupportHandWorld,
                                    baseline.alignmentBlend);
                    }
                    baseline.alignmentBlend = (std::min)(
                        1.0f,
                        baseline.alignmentBlend +
                            (std::isfinite(dt) && dt > 0.0f ? dt : 0.0f) *
                                ROTATION_BLEND_SPEED);
                } else if (baseline.alignmentBlend > 0.0f) {
                    ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                        "TwoHandedGrip: dynamic support alignment holding at blend={:.3f} because the physical support hand frame is unavailable grip={} generation={:016X}",
                        baseline.alignmentBlend,
                        supportGrip.gripSequence,
                        supportGrip.weaponGenerationKey);
                }
            }
        }
        if (supportInputBaselineActive &&
            !inputBaselineResolved) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon,
                "TwoHandedGrip: clearing support grip because its captured input baseline became invalid mode=dynamic hand={} grip={} generation={:016X} primaryDriver={} supportDriver={}",
                supportHandIsLeft ? "left" : "right",
                supportGrip.gripSequence,
                supportGrip.weaponGenerationKey,
                _scope.currentHandDriverFrames[
                    primaryHandIsLeft ? 0u : 1u]
                        .valid ?
                    "valid" :
                    "missing",
                _scope.currentHandDriverFrames[
                    supportHandIsLeft ? 0u : 1u]
                        .valid ?
                    "valid" :
                    "missing");
            logGripFailureIncident("dynamic-input-baseline-invalid");
            transitionToInactive(false);
            return;
        }

        RE::NiTransform recoilDelta{};
        const bool recoilConsumed = consumeOwnedWeaponRecoil(recoilDelta);
        if (recoilConsumed) {
            // Both solver inputs are isolated physical intent, including the
            // dynamic driver baseline above. Add recoil once after baseline
            // selection, then constrain it against the fixed support target.
            // FRIK does not add recoil to either already-solved hand seat.
            const auto recoiledPrimary = transform_math::composeTransforms(
                recoilDelta, calibratedPrimaryTransform);
            if (isUsableHandAuthorityTransform(recoiledPrimary)) {
                calibratedPrimaryTransform = recoiledPrimary;
            } else {
                ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                    "Weapon recoil: invalid two-hand target; holding recoil neutral");
            }
        }

        const RE::NiPoint3 primaryController =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                calibratedPrimaryTransform,
                primaryHandIsLeft);
        const RE::NiPoint3 supportController =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                calibratedSupportTransform,
                supportHandIsLeft);

        const RE::NiPoint3 currentSupportWorld = resolvePartGripWorld(supportGrip, weaponNode);
        const RE::NiPoint3 currentPrimaryGripWorld = transform_math::localPointToWorld(weaponNode->world, _firing.primaryGripLocal);
        const float currentGripSeparationWorld = std::sqrt(dot(sub(currentSupportWorld, currentPrimaryGripWorld), sub(currentSupportWorld, currentPrimaryGripWorld)));
        const float lockedGripSeparationWorld = supportGrip.hasSourceFrames ? currentGripSeparationWorld : _support.lockedGripSeparationWorld;
        const RE::NiPoint3 supportGripLocal = resolvePartGripWeaponLocal(supportGrip, weaponNode);
        const RE::NiPoint3 lockedSupportControllerTarget = makeLockedSupportGripTarget(
            primaryController,
            supportController,
            currentSupportWorld,
            lockedGripSeparationWorld,
            0.001f);

        WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3> solverInput{};
        solverInput.weaponWorldTransform = weaponNode->world;
        solverInput.primaryGripLocal = _firing.primaryGripLocal;
        solverInput.supportGripLocal = supportGripLocal;
        solverInput.primaryTargetWorld = primaryController;
        solverInput.supportTargetWorld =
            dynamicAcquisition ?
                lockedSupportControllerTarget :
                lerpPoint(
                    currentSupportWorld,
                    lockedSupportControllerTarget,
                    _support.rotationBlend);
        solverInput.supportNormalLocal = resolvePartGripNormalWeaponLocal(supportGrip, weaponNode);
        solverInput.supportNormalTargetWorld = computePalmNormalFromHandBasis(
            calibratedSupportTransform,
            supportHandIsLeft);
        solverInput.useSupportNormalTwist = true;
        /*
         * Authored support acquisition already eases the support-point axis
         * through _support.rotationBlend. Applying its palm-normal twist at full
         * strength on the first solver frame bypassed that easing and snapped
         * weapons whose native firing hold has a non-neutral orientation.
         * Dynamic acquisition still solves the complete composite correction
         * here and applies its own shortest-arc partial rotation below.
         */
        solverInput.supportNormalTwistFactor = SUPPORT_NORMAL_TWIST_FACTOR *
            (supportGrip.authoredSupportGrip && !dynamicAcquisition ?
                    _support.rotationBlend :
                    1.0f);

        if (supportGrip.disableAuthoredSupportNormalTwist) {
            /*
             * Authored support in either hand topology removes the palm-normal
             * twist about the primary-support grip axis - the roll that
             * corkscrews the weapon around its own barrel on offhand attach -
             * not the axis-aiming rotation. Keep the proven full solve (axis
             * aim toward the offhand plus primary-anchored translation) and
             * drop only the roll term.
             *
             * Do NOT reintroduce "freeze all rotation and let the support
             * seat own translation". Tried twice (locked-ray anchor, then
             * real offhand-controller anchor): the native weapon rotation
             * swings with wrist/arm IK as the firing hand translates, and
             * the weapon-origin-to-support-seat lever converts that swing
             * into inverted, amplified weapon motion around the static
             * offhand seat. Trace evidence 2026-08-27, Docs/ROCK/lessons.
             */
            solverInput.useSupportNormalTwist = false;
            solverInput.supportNormalTwistFactor = 0.0f;
        }

        WeaponTwoHandedSolverResult<RE::NiTransform> solved{};
        RE::NiTransform appliedWeaponWorld{};
        solved = solveTwoHandedWeaponTransformFrikPivot(solverInput);
        if (!solved.solved) {
            if (dynamicAcquisition || supportInputBaselineActive) {
                clearDynamicSupportAcquisition(
                    "full-target-solve-degenerate",
                    true);
                logGripFailureIncident("two-hand-target-solve-degenerate");
                transitionToInactive(false);
            }
            return;
        }
        appliedWeaponWorld = solved.weaponWorldTransform;
        if (supportGrip.disableAuthoredSupportNormalTwist) {
            ROCK_LOG_SAMPLE_INFO(Weapon, 250,
                "Authored support position-only authority trace: primaryTarget=({:.3f},{:.3f},{:.3f}) supportTarget=({:.3f},{:.3f},{:.3f}) weaponT=({:.3f},{:.3f},{:.3f})->({:.3f},{:.3f},{:.3f}) axisRotDeg={:.2f} twist=disabled blend={:.3f}",
                primaryController.x,
                primaryController.y,
                primaryController.z,
                solverInput.supportTargetWorld.x,
                solverInput.supportTargetWorld.y,
                solverInput.supportTargetWorld.z,
                solverInput.weaponWorldTransform.translate.x,
                solverInput.weaponWorldTransform.translate.y,
                solverInput.weaponWorldTransform.translate.z,
                appliedWeaponWorld.translate.x,
                appliedWeaponWorld.translate.y,
                appliedWeaponWorld.translate.z,
                weapon_support_acquisition_math::rotationAngleRadians(
                    solved.rotationDelta) * RADIANS_TO_DEGREES,
                _support.rotationBlend);
        }
        if (dynamicAcquisition) {
            auto& acquisition = _support.dynamicAcquisition;
            if (!acquisition.durationInitialized) {
                auto axisOnlyInput = solverInput;
                axisOnlyInput.useSupportNormalTwist = false;
                axisOnlyInput.supportNormalTwistFactor = 0.0f;
                const auto axisOnlySolved =
                    solveTwoHandedWeaponTransformFrikPivot(
                        axisOnlyInput);

                acquisition.fullCorrectionRadians =
                    weapon_support_acquisition_math::
                        rotationAngleRadians(solved.rotationDelta);
                acquisition.axisCorrectionRadians =
                    axisOnlySolved.solved ?
                    weapon_support_acquisition_math::
                        rotationAngleRadians(
                            axisOnlySolved.rotationDelta) :
                    0.0f;
                acquisition.twistContributionRadians =
                    axisOnlySolved.solved ?
                    weapon_support_acquisition_math::
                        rotationDistanceRadians(
                            axisOnlySolved.rotationDelta,
                            solved.rotationDelta) :
                    0.0f;
                if (!std::isfinite(
                        acquisition.fullCorrectionRadians) ||
                    !std::isfinite(
                        acquisition.axisCorrectionRadians) ||
                    !std::isfinite(
                        acquisition.twistContributionRadians)) {
                    clearDynamicSupportAcquisition(
                        "non-finite-correction-angle",
                        true);
                    transitionToInactive(false);
                    return;
                }

                const RE::NiTransform fullPrimaryHandWorld =
                    weapon_visual_authority_math::
                        weaponLocalFrameToWorld(
                            solved.weaponWorldTransform,
                            _firing.primaryHandWeaponLocal);
                const RE::NiTransform fullSupportHandWorld =
                    weapon_visual_authority_math::
                        weaponLocalFrameToWorld(
                            solved.weaponWorldTransform,
                            supportGrip.handWeaponLocal);
                const float primarySeatDistance =
                    hand_visual_lerp_math::
                        distanceGameUnits(
                            acquisition.primaryStartWorld.translate,
                            fullPrimaryHandWorld.translate);
                const float supportSeatDistance =
                    hand_visual_lerp_math::
                        distanceGameUnits(
                            acquisition.supportStartWorld.translate,
                            fullSupportHandWorld.translate);
                acquisition.initialSeatDistanceGameUnits =
                    (std::max)(
                        primarySeatDistance,
                        supportSeatDistance);
                acquisition.durationSeconds = 0.0f;
                if (g_rockConfig
                        .rockWeaponSupportGripHandLerpEnabled) {
                    acquisition.durationSeconds =
                        hand_visual_lerp_math::
                            computeDistanceMappedDurationGameUnits(
                                acquisition
                                    .initialSeatDistanceGameUnits,
                                g_rockConfig
                                    .rockWeaponSupportGripHandLerpTimeMin,
                                g_rockConfig
                                    .rockWeaponSupportGripHandLerpTimeMax,
                                g_rockConfig
                                    .rockWeaponSupportGripHandLerpMinDistance,
                                g_rockConfig
                                    .rockWeaponSupportGripHandLerpMaxDistance);
                    if (acquisition.durationSeconds <= 0.0f &&
                        acquisition.fullCorrectionRadians >=
                            DYNAMIC_SUPPORT_MINIMUM_SMOOTHED_ROTATION_RADIANS) {
                        acquisition.durationSeconds =
                            1.0f / ROTATION_BLEND_SPEED;
                    }
                }
                acquisition.durationInitialized = true;
            }

            acquisition.elapsedSeconds =
                hand_visual_lerp_math::
                    advanceTimedBlendElapsed(
                        acquisition.elapsedSeconds,
                        dt,
                        acquisition.durationSeconds);
            acquisition.rawAlpha =
                hand_visual_lerp_math::timedBlendAlpha(
                    acquisition.elapsedSeconds,
                    acquisition.durationSeconds);
            acquisition.easedAlpha =
                weapon_support_acquisition_math::
                    timedSmoothStepAlpha(
                        acquisition.elapsedSeconds,
                        acquisition.durationSeconds);
            _support.rotationBlend = acquisition.easedAlpha;

            const auto acquisitionSolve =
                weapon_support_acquisition_math::
                    applyRotationAroundPrimaryPivot<
                        RE::NiTransform,
                        RE::NiPoint3>(
                        solverInput.weaponWorldTransform,
                        solved.rotationDelta,
                        _firing.primaryGripLocal,
                        primaryController,
                        acquisition.easedAlpha);
            if (!acquisitionSolve.valid) {
                clearDynamicSupportAcquisition(
                    "pivot-preserving-slerp-invalid",
                    true);
                transitionToInactive(false);
                return;
            }
            appliedWeaponWorld =
                acquisitionSolve.weaponWorldTransform;
            acquisition.lastAppliedRotationRadians =
                acquisitionSolve.appliedRotationRadians;
            acquisition.lastPrimaryPivotError =
                acquisitionSolve.primaryError;
            const RE::NiPoint3 appliedSupportWorld =
                transform_math::localPointToWorld(
                    appliedWeaponWorld,
                    supportGripLocal);
            acquisition.lastSupportTargetError =
                weaponSolverLength(
                weaponSolverSub(
                    appliedSupportWorld,
                    lockedSupportControllerTarget));
        }

        const bool supportBaselineAttachPublication =
            supportInputBaselineActive &&
            supportGrip.supportInputBaseline.firstPublicationPending;
        if (supportBaselineAttachPublication) {
            /*
             * Acquisition may move the visual support hand, but the first
             * support publication cannot alter the primary-owned weapon
             * transform. Starting next frame, the calibrated support input
             * contributes only its post-capture tandem delta.
             */
            appliedWeaponWorld =
                supportGrip.supportInputBaseline.weaponWorldAtCapture;
        }

        // Firing-grip reattach: ease the rendered part-carry pose into this
        // two-hand solve; both locked hands ride the eased weapon.
        appliedWeaponWorld = resolveWeaponPoseHandoffBlend(appliedWeaponWorld, dt);

        if (!applyWeaponVisualAuthority(weaponNode, appliedWeaponWorld)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing support grip because ROCK visual weapon authority failed");
            logGripFailureIncident("weapon-visual-authority-publication-failed");
            transitionToInactive(false);
            return;
        }

        static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
        static_assert(weapon_visual_authority_math::weaponVisualPrecedesLockedHandAuthority());
        publishGripHandPoses(supportHandIsLeft);

        const bool applyPrimaryHandAuthority = weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(_session.authorityMode);
        if (!applyLockedHandVisualAuthority(weaponNode, applyPrimaryHandAuthority, true, dt, &primaryTransform, &supportTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing support grip because ROCK locked hand authority failed");
            logGripFailureIncident("locked-hand-authority-publication-failed");
            transitionToInactive(false);
            return;
        }
        if (usesLeftFiringCarry() && applyPrimaryHandAuthority &&
            !applyWeaponVisualAuthority(
                weaponNode,
                appliedWeaponWorld,
                _session.weaponGenerationKey,
                false)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: clearing support grip because final left position-only weapon publication failed");
            logGripFailureIncident(
                "final-left-weapon-publication-failed");
            transitionToInactive(false);
            return;
        }

        if (recoilConsumed) {
            traceRecoilPresentation("two-hand-solver");
        }

        const auto transformTranslationDistance = [](
                                                      const RE::NiTransform& left,
                                                      const RE::NiTransform& right) {
            return weaponSolverLength(
                weaponSolverSub(left.translate, right.translate));
        };
        const auto transformRotationDistanceDegrees = [](
                                                            const RE::NiTransform& left,
                                                            const RE::NiTransform& right) {
            return weapon_support_acquisition_math::
                       rotationDistanceRadians(
                           left.rotate,
                           right.rotate) *
                   RADIANS_TO_DEGREES;
        };

        if (supportBaselineAttachPublication) {
            const float attachRotationDegrees =
                transformRotationDistanceDegrees(
                    supportGrip.supportInputBaseline.weaponWorldAtCapture,
                    weaponNode->world);
            const float attachTranslationGameUnits =
                transformTranslationDistance(
                    supportGrip.supportInputBaseline.weaponWorldAtCapture,
                    weaponNode->world);
            constexpr float kAttachRotationWarningDegrees = 0.05f;
            constexpr float kAttachTranslationWarningGameUnits = 0.01f;
            const bool attachInvariantHeld =
                std::isfinite(attachRotationDegrees) &&
                std::isfinite(attachTranslationGameUnits) &&
                attachRotationDegrees <=
                    kAttachRotationWarningDegrees &&
                attachTranslationGameUnits <=
                    kAttachTranslationWarningGameUnits;
            if (attachInvariantHeld) {
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: support baseline published mode=dynamic hand={} grip={} generation={:016X} attachWeaponDelta=({:.5f}gu,{:.5f}deg) surfaceSeat={:.2f}deg tandemDeltaAuthority=armed",
                    supportHandIsLeft ? "left" : "right",
                    supportGrip.gripSequence,
                    supportGrip.weaponGenerationKey,
                    attachTranslationGameUnits,
                    attachRotationDegrees,
                    supportGrip.surfaceSeatRotationRadians *
                        RADIANS_TO_DEGREES);
            } else {
                ROCK_LOG_WARN(Weapon,
                    "TwoHandedGrip: support attach invariant exceeded mode=dynamic hand={} grip={} generation={:016X} attachWeaponDelta=({:.5f}gu,{:.5f}deg)",
                    supportHandIsLeft ? "left" : "right",
                    supportGrip.gripSequence,
                    supportGrip.weaponGenerationKey,
                    attachTranslationGameUnits,
                    attachRotationDegrees);
            }
            supportGrip.supportInputBaseline.firstPublicationPending = false;
        }

        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;

        RE::NiPoint3 primaryGripFinal = transform_math::localPointToWorld(_lastSolvedWeaponTransform, _firing.primaryGripLocal);
        RE::NiPoint3 offhandGripFinal = resolvePartGripWorld(supportGrip, weaponNode);

        if (dynamicAcquisition) {
            auto& acquisition = _support.dynamicAcquisition;
            if (!acquisition.firstPublicationRecorded) {
                acquisition.firstPublicationRecorded = true;
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: dynamic support acquisition event=start hand={} authored=no provider=no attachOnly=no grip={} generation={:016X} captureToFirstPublicationFrames=0 seatDistance={:.3f} duration={:.3f}s fullCorrection={:.2f}deg axisCorrection={:.2f}deg twistContribution={:.2f}deg firstRawAlpha={:.3f} firstEasedAlpha={:.3f} firstAppliedCorrection={:.2f}deg primaryPivotError={:.4f} supportTargetError={:.4f}",
                    supportHandIsLeft ? "left" : "right",
                    acquisition.gripSequence,
                    acquisition.weaponGenerationKey,
                    acquisition.initialSeatDistanceGameUnits,
                    acquisition.durationSeconds,
                    acquisition.fullCorrectionRadians *
                        RADIANS_TO_DEGREES,
                    acquisition.axisCorrectionRadians *
                        RADIANS_TO_DEGREES,
                    acquisition.twistContributionRadians *
                        RADIANS_TO_DEGREES,
                    acquisition.rawAlpha,
                    acquisition.easedAlpha,
                    acquisition.lastAppliedRotationRadians *
                        RADIANS_TO_DEGREES,
                    acquisition.lastPrimaryPivotError,
                    acquisition.lastSupportTargetError);
            }

            if (acquisition.easedAlpha >= 1.0f) {
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: dynamic support acquisition event=complete hand={} grip={} generation={:016X} elapsed={:.3f}s duration={:.3f}s appliedCorrection={:.2f}deg primaryPivotError={:.4f} supportTargetError={:.4f}",
                    supportHandIsLeft ? "left" : "right",
                    acquisition.gripSequence,
                    acquisition.weaponGenerationKey,
                    acquisition.elapsedSeconds,
                    acquisition.durationSeconds,
                    acquisition.lastAppliedRotationRadians *
                        RADIANS_TO_DEGREES,
                    acquisition.lastPrimaryPivotError,
                    acquisition.lastSupportTargetError);
                _support.rotationBlend = 1.0f;
                clearDynamicSupportAcquisition(
                    "completed",
                    false);
                dynamicAcquisition = false;
            }
        }

        if (++_gripLogCounter >= 90) {
            _gripLogCounter = 0;
            float separation = std::sqrt(dot(sub(primaryGripFinal, offhandGripFinal), sub(primaryGripFinal, offhandGripFinal)));
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: blend={:.2f}, dynamicAcquisition={}, separation={:.1f}gu, "
                "primaryGrip=({:.1f},{:.1f},{:.1f}), offhandGrip=({:.1f},{:.1f},{:.1f}), handLerp=({:.2f}/{:.3f}s,{:.2f}/{:.3f}s)",
                _support.rotationBlend,
                dynamicAcquisition ? "active" : "inactive",
                separation,
                primaryGripFinal.x,
                primaryGripFinal.y,
                primaryGripFinal.z,
                offhandGripFinal.x,
                offhandGripFinal.y,
                offhandGripFinal.z,
                _visuals.primaryHandLerp.lastAlpha,
                _visuals.primaryHandLerp.durationSeconds,
                supportGrip.visualLerp.lastAlpha,
                supportGrip.visualLerp.durationSeconds);
        }

        if (leftCarryProbeEnabled) {
            const LeftCarryProbeSample leftBonesAfter = sampleLeftCarryProbe();
            RE::NiTransform rightRootAfter{};
            const bool rightRootAfterValid =
                tryGetPresentedRootFlattenedHandBoneTransform(false, rightRootAfter);
            const RE::NiTransform presentedLeftTarget =
                transform_math::composeTransforms(
                    weaponNode->world,
                    _firing.primaryHandWeaponLocal);
            const auto& frame = runtime_state::currentFrame();
            ROCK_LOG_DEBUG(Weapon,
                "LEFT_CARRY_CLOCK: frame={} dt={:.6f} bones={}{}/{}{} driver=({:.2f},{:.2f},{:.2f}) physL=({:.2f},{:.2f},{:.2f}) rootBefore=({:.2f},{:.2f},{:.2f}) rootAfter=({:.2f},{:.2f},{:.2f}) nodeBefore=({:.3f},{:.3f},{:.3f}) nodeAfter=({:.3f},{:.3f},{:.3f}) arrayVsNodeBefore={:.3f} arrayVsNodeAfter={:.3f} physVsRootBefore={:.3f}gu/{:.2f}deg rootStep={:.3f}gu/{:.2f}deg target=({:.2f},{:.2f},{:.2f}) physVsTarget={:.3f}gu/{:.2f}deg targetVsRootAfter={:.3f}gu/{:.2f}deg targetVsNodeAfter={:.3f}gu/{:.2f}deg elbowBefore=({:.2f},{:.2f},{:.2f}) elbowAfter=({:.2f},{:.2f},{:.2f}) elbowNodeAfter=({:.3f},{:.3f},{:.3f}) elbowStep={:.3f}gu rightRoot=({:.2f},{:.2f},{:.2f}) rightRootAfter=({:.2f},{:.2f},{:.2f}) supportTarget=({:.2f},{:.2f},{:.2f}) weaponPre=({:.2f},{:.2f},{:.2f}) weaponPost=({:.2f},{:.2f},{:.2f}) weaponStep={:.3f}gu/{:.2f}deg axisRot={:.2f}deg pulsePrev={}/{} lerp={:.2f}/{:.2f} blend={:.3f}",
                frame.frameIndex,
                dt,
                leftBonesBefore.arrayValid ? "A" : "-",
                leftBonesBefore.nodeValid ? "N" : "-",
                leftBonesAfter.arrayValid ? "A" : "-",
                leftBonesAfter.nodeValid ? "N" : "-",
                primaryDriverWorld.translate.x,
                primaryDriverWorld.translate.y,
                primaryDriverWorld.translate.z,
                primaryTransform.translate.x,
                primaryTransform.translate.y,
                primaryTransform.translate.z,
                leftBonesBefore.handArray.translate.x,
                leftBonesBefore.handArray.translate.y,
                leftBonesBefore.handArray.translate.z,
                leftBonesAfter.handArray.translate.x,
                leftBonesAfter.handArray.translate.y,
                leftBonesAfter.handArray.translate.z,
                leftBonesBefore.handNode.translate.x,
                leftBonesBefore.handNode.translate.y,
                leftBonesBefore.handNode.translate.z,
                leftBonesAfter.handNode.translate.x,
                leftBonesAfter.handNode.translate.y,
                leftBonesAfter.handNode.translate.z,
                transformTranslationDistance(leftBonesBefore.handArray, leftBonesBefore.handNode),
                transformTranslationDistance(leftBonesAfter.handArray, leftBonesAfter.handNode),
                transformTranslationDistance(primaryTransform, leftBonesBefore.handArray),
                transformRotationDistanceDegrees(primaryTransform, leftBonesBefore.handArray),
                transformTranslationDistance(leftBonesAfter.handArray, leftBonesBefore.handArray),
                transformRotationDistanceDegrees(leftBonesAfter.handArray, leftBonesBefore.handArray),
                presentedLeftTarget.translate.x,
                presentedLeftTarget.translate.y,
                presentedLeftTarget.translate.z,
                transformTranslationDistance(primaryTransform, presentedLeftTarget),
                transformRotationDistanceDegrees(primaryTransform, presentedLeftTarget),
                transformTranslationDistance(presentedLeftTarget, leftBonesAfter.handArray),
                transformRotationDistanceDegrees(presentedLeftTarget, leftBonesAfter.handArray),
                transformTranslationDistance(presentedLeftTarget, leftBonesAfter.handNode),
                transformRotationDistanceDegrees(presentedLeftTarget, leftBonesAfter.handNode),
                leftBonesBefore.forearmArray.translate.x,
                leftBonesBefore.forearmArray.translate.y,
                leftBonesBefore.forearmArray.translate.z,
                leftBonesAfter.forearmArray.translate.x,
                leftBonesAfter.forearmArray.translate.y,
                leftBonesAfter.forearmArray.translate.z,
                leftBonesAfter.forearmNode.translate.x,
                leftBonesAfter.forearmNode.translate.y,
                leftBonesAfter.forearmNode.translate.z,
                transformTranslationDistance(leftBonesAfter.forearmArray, leftBonesBefore.forearmArray),
                supportTransform.translate.x,
                supportTransform.translate.y,
                supportTransform.translate.z,
                rightRootAfterValid ? rightRootAfter.translate.x : 0.0f,
                rightRootAfterValid ? rightRootAfter.translate.y : 0.0f,
                rightRootAfterValid ? rightRootAfter.translate.z : 0.0f,
                solverInput.supportTargetWorld.x,
                solverInput.supportTargetWorld.y,
                solverInput.supportTargetWorld.z,
                solverInput.weaponWorldTransform.translate.x,
                solverInput.weaponWorldTransform.translate.y,
                solverInput.weaponWorldTransform.translate.z,
                weaponNode->world.translate.x,
                weaponNode->world.translate.y,
                weaponNode->world.translate.z,
                transformTranslationDistance(weaponNode->world, solverInput.weaponWorldTransform),
                transformRotationDistanceDegrees(weaponNode->world, solverInput.weaponWorldTransform),
                weapon_support_acquisition_math::rotationAngleRadians(solved.rotationDelta) * RADIANS_TO_DEGREES,
                _visuals.weaponCollisionHandPresentationFromPreviousFrame[0] ? "L" : "-",
                _visuals.weaponCollisionHandPresentationFromPreviousFrame[1] ? "R" : "-",
                _visuals.primaryHandLerp.lastAlpha,
                supportGrip.visualLerp.lastAlpha,
                _support.rotationBlend);
        }
    }

    void TwoHandedGrip::updateVisualOnlySupportGrip(RE::NiNode* weaponNode, float dt)
    {
        const bool supportHandIsLeft = isSupportHandLeft();

        if (usesLeftFiringCarry()) {
            // Visual-only support never steers aim, but with a LEFT firing
            // hand the weapon itself must still be ROCK-carried (FRIK's glue
            // is blocked); the shooting-cup right hand stays visual-only.
            if (!solveLeftFiringWeaponCarry(weaponNode, dt)) {
                return;
            }
        }

        static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
        publishGripHandPoses(supportHandIsLeft);

        RE::NiTransform supportTransform{};
        const RE::NiTransform* liveSupportTransform = tryGetSolverHandTransform(supportHandIsLeft, supportTransform) ? &supportTransform : nullptr;
        if (!applyLockedHandVisualAuthority(weaponNode, false, true, dt, nullptr, liveSupportTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing visual-only support grip because ROCK support hand authority failed");
            logGripFailureIncident(
                "visual-only-support-authority-publication-failed");
            transitionToInactive(false);
            return;
        }

        _lastSolvedWeaponTransform = weaponNode ? weaponNode->world : RE::NiTransform{};
        _hasSolvedWeaponTransform = usesLeftFiringCarry() && _hasSolvedWeaponTransform;

        if (weaponNode && ++_gripLogCounter >= 90) {
            _gripLogCounter = 0;
            const WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
            const RE::NiPoint3 offhandGripFinal = resolvePartGripWorld(supportGrip, weaponNode);
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: visual-only support follows weapon='{}', offhandGrip=({:.1f},{:.1f},{:.1f}), handLerp={:.2f}/{:.3f}s",
                weaponNode->name.c_str(),
                offhandGripFinal.x,
                offhandGripFinal.y,
                offhandGripFinal.z,
                supportGrip.visualLerp.lastAlpha,
                supportGrip.visualLerp.durationSeconds);
        }
    }

    void TwoHandedGrip::setSupportGripPose(
        bool isLeft,
        const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPose,
        const std::array<float, 5>* capturedSplayRadians,
        const SupportGripPoseFallback fallback)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        if (meshFingerPose && meshFingerPose->solved) {
            grip.fingerPose = meshFingerPose->hasJointValues ? meshFingerPose->jointValues : grab_finger_pose_math::expandFingerCurlsToJointValues(meshFingerPose->values);
            grip.fingerSplayRadians = capturedSplayRadians ? *capturedSplayRadians : std::array<float, 5>{};
            grip.hasFingerSplay = capturedSplayRadians != nullptr;
            grip.hasFingerPose = true;
            return;
        }

        float fallbackValue = 0.0f;
        if (fallback == SupportGripPoseFallback::SelectedClose) {
            const float fallbackMin =
                std::clamp(std::isfinite(g_rockConfig.rockGrabFingerMinValue) ? g_rockConfig.rockGrabFingerMinValue : 0.2f, 0.0f, 1.0f);
            const float configuredFallback =
                std::isfinite(g_rockConfig.rockSelectedCloseFingerAnimValue) ? g_rockConfig.rockSelectedCloseFingerAnimValue : 0.9f;
            fallbackValue = std::clamp(
                configuredFallback,
                fallbackMin,
                1.0f);
        }
        const std::array<float, 5> fallbackCurls{
            fallbackValue,
            fallbackValue,
            fallbackValue,
            fallbackValue,
            fallbackValue,
        };
        grip.fingerPose = grab_finger_pose_math::expandFingerCurlsToJointValues(fallbackCurls);
        grip.fingerSplayRadians = {};
        grip.hasFingerPose = true;
        grip.hasFingerSplay = false;
        grip.fingerLocalTransforms = {};
        grip.fingerLocalTransformMask = 0;
        grip.hasFingerLocalTransforms = false;
        if (fallback == SupportGripPoseFallback::FullyClosed) {
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: using whole-hand-closed finger fallback hand={} value={:.2f}",
                isLeft ? "left" : "right",
                fallbackValue);
        } else {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: using selected-close finger fallback hand={} value={:.2f}",
                isLeft ? "left" : "right",
                fallbackValue);
        }
    }

    void TwoHandedGrip::clearSupportGripPose(bool isLeft)
    {
        _visuals.hasLastPublishedHandWorld[isLeft ? 0u : 1u] = false;
        WeaponPartGrip& grip = partGrip(isLeft);
        grip.fingerPose = {};
        grip.fingerSplayRadians = {};
        grip.hasFingerPose = false;
        grip.hasFingerSplay = false;
        grip.fingerLocalTransforms = {};
        grip.fingerLocalTransformMask = 0;
        grip.hasFingerLocalTransforms = false;

        (void)frik_visual_authority::clearHandPose(SUPPORT_GRIP_TAG, handFromBool(isLeft));
        if (_scope.menuOpenThisFrame || _scope.menuClosedThisFrame) {
            deferScopeHandAuthorityClear(scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip, isLeft);
        } else {
            (void)clearHandAuthorityRoleNow(scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip, isLeft);
        }
    }

    void TwoHandedGrip::clearAuthoredSupportGripCandidate()
    {
        _support.authoredCandidate = {};
    }

    authored_support_grab_policy::LeftFiringTakeoverReadiness
    TwoHandedGrip::getLeftFiringTakeoverReadiness(
        RE::NiNode* weaponNode,
        const std::uint64_t authoredGenerationKey,
        const std::uint64_t weaponOwnershipKey,
        const bool authoredOnlyModeEnabled) const noexcept
    {
        RE::NiTransform mirroredRightSupportHandWeaponLocal{};
        std::array<RE::NiTransform, 15> mirroredRightFingerLocals{};
        std::uint16_t mirroredRightFingerMask = 0;
        const bool mirroredCandidateAvailable =
            authoredGenerationKey != 0 &&
            tryResolveAuthoredSupportGripCandidateForHand(
                false,
                weaponNode,
                authoredGenerationKey,
                mirroredRightSupportHandWeaponLocal,
                mirroredRightFingerLocals,
                mirroredRightFingerMask) &&
            mirroredRightFingerMask == 0x7FFFu;

        const auto& capability = _support.authoredCapability;
        const bool capabilityIdentityCurrent =
            capability.initialized &&
            capability.weaponNodeIdentity == weaponNode &&
            weaponOwnershipKey != 0 &&
            capability.weaponOwnershipKey == weaponOwnershipKey &&
            authoredGenerationKey != 0 &&
            capability.weaponGenerationKey == authoredGenerationKey &&
            capability.handTopology ==
                authored_weapon_grip_activation_policy::HandTopology::
                    RightFiringLeftSupport;
        return authored_support_grab_policy::
            resolveLeftFiringTakeoverReadiness(
                authored_support_grab_policy::
                    LeftFiringTakeoverReadinessInput{
                        .targetFiringHandIsLeft = true,
                        .authoredOnlyModeEnabled =
                            authoredOnlyModeEnabled,
                        .authoredGenerationKey =
                            authoredGenerationKey,
                        .capabilityIdentityCurrent =
                            capabilityIdentityCurrent,
                        .capability = capability.capability,
                        .mirroredCandidateAvailable =
                            mirroredCandidateAvailable,
                    });
    }

    bool TwoHandedGrip::setAuthoredSupportGripCandidate(
        RE::NiNode* weaponNode,
        const RE::NiTransform& handWeaponLocal,
        const std::array<RE::NiTransform, 15>& fingerLocalTransforms,
        const std::uint16_t fingerLocalTransformMask,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t captureSequence)
    {
        clearAuthoredSupportGripCandidate();
        constexpr std::uint16_t kCompleteFingerLocalTransformMask = 0x7FFFu;
        if (!weaponNode ||
            weaponGenerationKey == 0 ||
            captureSequence == 0 ||
            fingerLocalTransformMask != kCompleteFingerLocalTransformMask ||
            !isFiniteTransform(handWeaponLocal) ||
            std::abs(handWeaponLocal.scale) <= 0.0001f) {
            return false;
        }
        for (const auto& fingerLocal : fingerLocalTransforms) {
            if (!isFiniteTransform(fingerLocal) ||
                std::abs(fingerLocal.scale) <= 0.0001f) {
                return false;
            }
        }

        AuthoredSupportGripCandidate candidate{
            .weaponNode = weaponNode,
            .leftHandWeaponLocal = handWeaponLocal,
            .leftFingerLocalTransforms = fingerLocalTransforms,
            .leftFingerLocalTransformMask = fingerLocalTransformMask,
            .weaponGenerationKey = weaponGenerationKey,
            .captureSequence = captureSequence,
            .valid = true,
        };

        _support.authoredCandidate = candidate;
        refreshAuthoredSupportRightMirror();
        return true;
    }

    void TwoHandedGrip::refreshAuthoredSupportRightMirror()
    {
        auto& candidate = _support.authoredCandidate;
        if (!candidate.valid || candidate.rightMirrorValid) {
            return;
        }

        constexpr std::uint16_t kCompleteFingerLocalTransformMask = 0x7FFFu;
        RE::NiTransform mirroredRightHandWeaponLocal{};
        const bool rightHandTransformMirrored =
            tryBuildMirroredRightSupportHandWeaponLocal(
                candidate.leftHandWeaponLocal,
                mirroredRightHandWeaponLocal);

        // Exact skeleton mirror of the authored left finger locals; the
        // bone order and therefore the mask are identical on both hands.
        std::array<RE::NiTransform, 15> mirroredRightFingerLocals{};
        const bool rightFingerPoseMirrored =
            candidate.leftFingerLocalTransformMask ==
                kCompleteFingerLocalTransformMask &&
            hand_finger_mirror_math::mirrorFingerLocalsAcrossHands<RE::NiTransform>(
                std::span<const RE::NiTransform>(candidate.leftFingerLocalTransforms),
                std::span<RE::NiTransform>(mirroredRightFingerLocals));
        bool rightFingerPoseFinite = rightFingerPoseMirrored;
        if (rightFingerPoseFinite) {
            for (const auto& fingerLocal : mirroredRightFingerLocals) {
                if (!isFiniteTransform(fingerLocal) ||
                    std::abs(fingerLocal.scale) <= 0.0001f) {
                    rightFingerPoseFinite = false;
                    break;
                }
            }
        }

        if (rightHandTransformMirrored && rightFingerPoseFinite) {
            candidate.rightHandWeaponLocal = mirroredRightHandWeaponLocal;
            candidate.rightFingerLocalTransforms = mirroredRightFingerLocals;
            candidate.rightFingerLocalTransformMask =
                candidate.leftFingerLocalTransformMask;
            candidate.rightMirrorValid = true;
            return;
        }

        if (usesLeftFiringCarry()) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000,
                "TwoHandedGrip: authored right-support mirror unavailable transform={} fingers={} naturalFrames=({}, {})",
                rightHandTransformMirrored ? "ready" : "missing",
                rightFingerPoseFinite ? "ready" : "missing",
                _firing.hasLeftNaturalBoneInWand ? "left" : "no-left",
                _firing.hasRightNaturalBoneInWand ? "right" : "no-right");
        }
    }

    bool TwoHandedGrip::tryResolveAuthoredSupportGripCandidateForHand(
        const bool isLeft,
        RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        RE::NiTransform& outHandWeaponLocal,
        std::array<RE::NiTransform, 15>& outFingerLocalTransforms,
        std::uint16_t& outFingerLocalTransformMask) const
    {
        outHandWeaponLocal = {};
        outFingerLocalTransforms = {};
        outFingerLocalTransformMask = 0;

        constexpr std::uint16_t kCompleteFingerLocalTransformMask = 0x7FFFu;
        const auto& candidate = _support.authoredCandidate;
        if (!candidate.valid ||
            !weaponNode ||
            candidate.weaponNode != weaponNode ||
            weaponGenerationKey == 0 ||
            candidate.weaponGenerationKey != weaponGenerationKey) {
            return false;
        }

        if (isLeft) {
            if (candidate.leftFingerLocalTransformMask !=
                kCompleteFingerLocalTransformMask) {
                return false;
            }
            outHandWeaponLocal = candidate.leftHandWeaponLocal;
            outFingerLocalTransforms = candidate.leftFingerLocalTransforms;
            outFingerLocalTransformMask = candidate.leftFingerLocalTransformMask;
        } else {
            if (!candidate.rightMirrorValid ||
                candidate.rightFingerLocalTransformMask !=
                    kCompleteFingerLocalTransformMask) {
                return false;
            }
            outHandWeaponLocal = candidate.rightHandWeaponLocal;
            outFingerLocalTransforms = candidate.rightFingerLocalTransforms;
            outFingerLocalTransformMask = candidate.rightFingerLocalTransformMask;
        }

        return isFiniteTransform(outHandWeaponLocal);
    }

    bool TwoHandedGrip::isDynamicSupportBaselineActive(
        const bool supportHandIsLeft,
        const WeaponPartGrip& supportGrip) const
    {
        return isSupportInputBaselineActive(
            supportHandIsLeft,
            supportGrip,
            SupportInputBaselineKind::Dynamic);
    }

    bool TwoHandedGrip::isPartCarryInputBaselineActive(
        const bool pivotHandIsLeft,
        const WeaponPartGrip& pivotGrip) const
    {
        return _session.state == TwoHandedState::PartCarry &&
               _partCarry.pivotIsLeft == pivotHandIsLeft &&
               isSupportInputBaselineActive(
                   pivotHandIsLeft,
                   pivotGrip,
                   SupportInputBaselineKind::PartCarry);
    }

    bool TwoHandedGrip::isSupportInputBaselineActive(
        const bool supportHandIsLeft,
        const WeaponPartGrip& supportGrip,
        const SupportInputBaselineKind kind) const
    {
        const auto& baseline = supportGrip.supportInputBaseline;
        return supportGrip.active &&
               baseline.active &&
               baseline.kind == kind &&
               (kind != SupportInputBaselineKind::Dynamic ||
                   baseline.pairedDynamicDrivers) &&
               baseline.supportHandIsLeft == supportHandIsLeft &&
               baseline.weaponGenerationKey != 0 &&
               baseline.weaponGenerationKey ==
                   supportGrip.weaponGenerationKey &&
               baseline.equippedWeaponOwnershipKey != 0 &&
               baseline.equippedWeaponOwnershipKey ==
                   _session.equippedWeaponOwnershipKey &&
               baseline.gripSequence != 0 &&
               baseline.gripSequence == supportGrip.gripSequence;
    }

    bool TwoHandedGrip::initializeDynamicSupportBaseline(
        RE::NiNode* weaponNode,
        const bool supportHandIsLeft,
        const char* reason)
    {
        WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        if (_session.authorityMode != weapon_support_authority_policy::
                                  WeaponSupportAuthorityMode::
                                      FullTwoHandedSolver ||
            supportGrip.authoredSupportGrip ||
            supportGrip.providerPartAuthority.active ||
            supportGrip.attachOnly ||
            !weaponNode ||
            !supportGrip.active ||
            !supportGrip.hasHandWeaponLocal ||
            !_firing.hasPrimaryHandWeaponLocal ||
            supportGrip.weaponGenerationKey == 0 ||
            supportGrip.gripSequence == 0) {
            return false;
        }
        if (isDynamicSupportBaselineActive(
                supportHandIsLeft,
                supportGrip)) {
            return true;
        }

        const bool primaryHandIsLeft = !supportHandIsLeft;
        const auto& primaryDriver =
            _scope.currentHandDriverFrames[
                primaryHandIsLeft ? 0u : 1u];
        const auto& supportDriver =
            _scope.currentHandDriverFrames[
                supportHandIsLeft ? 0u : 1u];
        const RE::NiTransform primaryGripTargetWorld =
            weapon_visual_authority_math::weaponLocalFrameToWorld(
                weaponNode->world,
                _firing.primaryHandWeaponLocal);
        const RE::NiTransform supportGripTargetWorld =
            resolvePartGripHandWorld(supportGrip, weaponNode);
        RE::NiTransform primaryDriverToTargetLocal{};
        RE::NiTransform supportDriverToTargetLocal{};
        if (!primaryDriver.valid ||
            !supportDriver.valid ||
            !weapon_support_acquisition_math::
                tryCaptureDynamicSupportDriverBaseline(
                    primaryDriver.world,
                    primaryGripTargetWorld,
                    supportDriver.world,
                    supportGripTargetWorld,
                    primaryDriverToTargetLocal,
                    supportDriverToTargetLocal)) {
            supportGrip.supportInputBaseline = {};
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: dynamic driver baseline capture failed hand={} primaryDriver={} supportDriver={} grip={} generation={:016X} reason={}",
                supportHandIsLeft ? "left" : "right",
                primaryDriver.valid ? "valid" : "missing",
                supportDriver.valid ? "valid" : "missing",
                supportGrip.gripSequence,
                supportGrip.weaponGenerationKey,
                reason ? reason : "unknown");
            return false;
        }

        supportGrip.supportInputBaseline = {
            .inputToGripTargetLocal = supportDriverToTargetLocal,
            .primaryInputToGripTargetLocal =
                primaryDriverToTargetLocal,
            .weaponWorldAtCapture = weaponNode->world,
            .weaponGenerationKey = supportGrip.weaponGenerationKey,
            .equippedWeaponOwnershipKey =
                _session.equippedWeaponOwnershipKey,
            .gripSequence = supportGrip.gripSequence,
            .supportHandIsLeft = supportHandIsLeft,
            .kind = SupportInputBaselineKind::Dynamic,
            .active = true,
            .pairedDynamicDrivers = true,
            .firstPublicationPending = true,
        };
        ROCK_LOG_INFO(
            Weapon,
            "TwoHandedGrip: paired dynamic driver baseline captured hand={} grip={} generation={:016X} reason={}; rendered hands excluded from solver input",
            supportHandIsLeft ? "left" : "right",
            supportGrip.gripSequence,
            supportGrip.weaponGenerationKey,
            reason ? reason : "unknown");
        return true;
    }

    void TwoHandedGrip::clearSupportInputBaselines()
    {
        for (auto& grip : _support.partGrips) {
            grip.supportInputBaseline = {};
        }
    }

    bool TwoHandedGrip::tryResolvePhysicalHandFrame(
        const bool isLeft,
        RE::NiTransform& outHandWorld,
        RE::NiTransform& outDriverWorld) const
    {
        outHandWorld = {};
        outDriverWorld = {};
        const RE::NiTransform& boneInDriver = isLeft ?
            _firing.leftNaturalBoneInDampedDriver :
            _firing.rightNaturalBoneInDampedDriver;
        const bool relationValid = isLeft ?
            _firing.hasLeftNaturalBoneInDampedDriver :
            _firing.hasRightNaturalBoneInDampedDriver;
        if (!relationValid ||
            !frik_hand_world_authority::tryGetInputDriverWorld(isLeft, outDriverWorld) ||
            !isFiniteTransform(boneInDriver)) {
            return false;
        }

        outHandWorld = transform_math::composeTransforms(
            outDriverWorld,
            boneInDriver);
        return isUsableHandAuthorityTransform(outHandWorld) &&
               isFiniteTransform(outDriverWorld);
    }

    bool TwoHandedGrip::tryGetPhysicalHandWorld(
        const bool isLeft,
        RE::NiTransform& outHandWorld) const
    {
        RE::NiTransform driverWorld{};
        if (!tryResolvePhysicalHandFrame(isLeft, outHandWorld, driverWorld)) {
            outHandWorld = {};
            return false;
        }
        return true;
    }

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
        if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(decision.weaponGenerationKey, _session.weaponGenerationKey)) {
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
        grip.acquisitionSource = decision.acquisitionSource;
        grip.gripSequence = ++_session.gripCaptureSequence;
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
            weaponLocalToWorld(_firing.primaryGripLocal, weaponNode);
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
            _support.authoredCandidate.weaponNode == weaponNode;
        const bool authoredGenerationMatches =
            _support.authoredCandidate.weaponGenerationKey ==
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
            _support.authoredDebugSnapshot;
        const bool authoredActivationStateMatches =
            authoredActivation.valid &&
            authoredActivation.supportHandIsLeft == isLeft &&
            authoredActivation.weaponGenerationKey ==
                decision.weaponGenerationKey &&
            authoredActivation.captureSequence ==
                _support.authoredCandidate.captureSequence;
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
                sub(authoredSupportPalmWeaponLocal, _firing.primaryGripLocal);
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
                WeaponInteractionAcquisitionSource::FiringGripZone ||
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
        firing_grip_reattach_zone_policy::ZoneInput handoffInput{};
        bool supportPalmInsideHandoffZone = false;
        bool authoredSeatInsideHandoffZone = false;
        const bool firingGripZoneAcquisition = decision.acquisitionSource ==
            WeaponInteractionAcquisitionSource::FiringGripZone;
        if (ambidextrousHandoffCaptureContext &&
            _handlingSettings.ambidextrousHandoffEnabled &&
            tryBuildFiringGripZoneInput(weaponNode,
                decision.weaponGenerationKey, _session.equippedWeaponOwnershipKey,
                weapon_support_authority_policy::firingGripCaptureReach(
                    firingGripZoneAcquisition,
                    _handlingSettings.firingGripReattachRadiusGameUnits,
                    _handlingSettings.firingGripPromotionRadiusGameUnits), handoffInput)) {
            handoffInput.palmWorld = { palmPos.x, palmPos.y, palmPos.z };
            supportPalmInsideHandoffZone =
                firing_grip_reattach_zone_policy::evaluateZone(handoffInput).inside;
            if (authoredSupportFrameValid) {
                const auto seatWorld = transform_math::localPointToWorld(
                    weaponNode->world, authoredSupportPalmWeaponLocal);
                handoffInput.palmWorld = { seatWorld.x, seatWorld.y, seatWorld.z };
                authoredSeatInsideHandoffZone =
                    firing_grip_reattach_zone_policy::evaluateZone(handoffInput).inside;
            }
        }
        if (firingGripZoneAcquisition && !supportPalmInsideHandoffZone) {
            ROCK_LOG_SAMPLE_DEBUG(Weapon, 1000,
                "TwoHandedGrip: firing-grip handoff capture rejected hand={} reason=zone-no-longer-current",
                isLeft ? "left" : "right");
            grip = {};
            return authored_support_grab_policy::Selection::Reject;
        }
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
                            .supportPalmInsideHandoffZone =
                                supportPalmInsideHandoffZone,
                            .authoredSeatInsideHandoffZone =
                                authoredSeatInsideHandoffZone,
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
                        _support.authoredCapability.capability,
                });
        recordSupportGrabSelection(
            selectionDecision,
            isLeft,
            decision.weaponGenerationKey);

        if (selectionDecision.selection ==
            authored_support_grab_policy::Selection::Authored) {
            if (firingGripProximityAuthorityEnabled) {
                _session.authorityMode = authoredSupportAuthorityMode;
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
                _firing.rightCanonicalSource ==
                    RightFiringCanonicalSource::AuthoredAnimation;
            grip.authoredSupportCaptureSequence =
                _support.authoredCandidate.captureSequence;
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
                _session.weaponGenerationKey,
                weaponInteractionAcquisitionSourceName(
                    decision.acquisitionSource),
                _session.authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport ?
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

        WeaponCollision::SupportGripEvidenceView evidenceView{};
        const bool cachedTrianglesFound = weaponCollision.tryGetSupportGripEvidenceView(decision.bodyId, weaponNode, evidenceView) &&
            evidenceView.weaponGenerationKey == decision.weaponGenerationKey &&
            evidenceView.weaponGenerationKey == _session.weaponGenerationKey;
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
            grip.gripLocal = _firing.primaryGripLocal;
            grip.grabNormalWorld = palmDir;
            if (firingGripZoneAcquisition) {
                // The cylinder admits a distant palm, but the captured station
                // is the firing grip itself. Coincident seats use visual-only
                // support, never a full two-hand solver selected by approach distance.
                _session.authorityMode = weapon_support_authority_policy::
                    WeaponSupportAuthorityMode::VisualOnlySupport;
            }
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

        solveSupportGripFingerPose(
            isLeft,
            weaponNode,
            decision,
            weaponCollision,
            handTransform,
            adjustedHandTransform,
            gripWorldPoint,
            cachedTrianglesFound,
            evidenceView,
            grip);

        grip.visualLerp = {};
        grip.active = true;
        if (isLeft) {
            _hapticEvents.leftPartGripCaptured = true;
        } else {
            _hapticEvents.rightPartGripCaptured = true;
        }

        // The authored-rejection WARN above already reports the full authored
        // gate diagnostics when selection falls through; the capture line
        // records only what identifies the resulting grip.
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: part grip captured hand={} weapon='{}' meshGrab={} fingerTriangles={} surfaceSeat={:.2f}deg acquisition={} authority={} provider={} attachOnly={} generation={:016X}",
            isLeft ? "left" : "right",
            weaponNode->name.c_str(),
            useDynamicHandoffGrip ?
                "HANDOFF" :
                (meshFound ? "YES" : "FALLBACK"),
            fingerScratch.localTriangles.size(),
            grip.surfaceSeatRotationRadians *
                RADIANS_TO_DEGREES,
            weaponInteractionAcquisitionSourceName(
                decision.acquisitionSource),
            _session.authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport ?
                "visual-only" :
                "full",
            providerPartAuthority.active ? "yes" : "no",
            grip.attachOnly ? "yes" : "no",
            _session.weaponGenerationKey);
        return selectionDecision.selection;
    }
}
