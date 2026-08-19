#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/TwoHandedGripInternal.h"

#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/hand/skeleton/HandSkeleton.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RockConfig.h"
#include "RockUtils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string_view>

namespace rock
{
    using two_handed_grip_detail::AuthoredSupportPalmSeatProximity;
    using two_handed_grip_detail::AUTHORED_PRIMARY_POSE_BLOCK_TAG;
    using two_handed_grip_detail::evaluateAuthoredSupportGripDirectionGate;
    using two_handed_grip_detail::GRIP_HAND_POSE_PRIORITY;
    using two_handed_grip_detail::isFiniteTransform;
    using two_handed_grip_detail::PRIMARY_GRIP_TAG;
    using two_handed_grip_detail::resolveAuthoredSupportPalmSeatProximity;
    using two_handed_grip_detail::resolveAuthoredSupportPalmSeatProximityFromPoints;

    // ---- Authored support activation and debug state ----

    bool TwoHandedGrip::getAuthoredSupportGripDebugSnapshot(
        AuthoredSupportGripDebugSnapshot& outSnapshot) const
    {
        outSnapshot = _authoredSupportGripDebugSnapshot;
        return outSnapshot.valid;
    }

    void TwoHandedGrip::refreshAuthoredSupportGripActivationState(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const WeaponCollision& weaponCollision,
        const bool requirePoseEvidence)
    {
        _authoredSupportGripDebugSnapshot = {};
        const bool collectPoseEvidence =
            requirePoseEvidence ||
            g_rockConfig.rockDebugDrawAuthoredGripActivationZones ||
            g_rockConfig.rockDebugShowHandAxes ||
            g_rockConfig.rockDebugShowGrabPivots;

        const auto& candidate = _authoredSupportGripCandidate;
        if (!weaponNode ||
            !candidate.valid ||
            candidate.weaponNode != weaponNode ||
            candidate.weaponGenerationKey == 0 ||
            candidate.weaponGenerationKey != currentWeaponGenerationKey ||
            candidate.captureSequence == 0 ||
            !isFiniteTransform(weaponNode->world)) {
            return;
        }

        if (_authoredSupportLastStableDirectionGenerationKey !=
                candidate.weaponGenerationKey ||
            _authoredSupportLastStableDirectionCaptureSequence !=
                candidate.captureSequence) {
            _authoredSupportLastStableApproachDirectionWorld = {};
            _authoredSupportLastStableDirectionGenerationKey =
                candidate.weaponGenerationKey;
            _authoredSupportLastStableDirectionCaptureSequence =
                candidate.captureSequence;
            _authoredSupportLastStableApproachDirectionValid = false;
        }

        const bool supportHandIsLeft = !_firingHandIsLeft;
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

        RE::NiTransform activationWeaponWorld = weaponNode->world;
        RE::NiTransform alignmentHandWorld{};
        RE::NiMatrix3 gunstockCorrection{};
        RE::NiPoint3 gunstockPivotWorld{};
        if (tryResolveGunstockPrimaryGroupCorrection(
                weaponNode,
                currentWeaponGenerationKey,
                alignmentHandWorld,
                gunstockCorrection,
                gunstockPivotWorld)) {
            const RE::NiTransform correctedWeaponWorld =
                gunstock_alignment_policy::rotateRigidlyAroundPivot<
                    RE::NiTransform,
                    RE::NiMatrix3,
                    RE::NiPoint3>(
                    activationWeaponWorld,
                    gunstockCorrection,
                    gunstockPivotWorld);
            if (isFiniteTransform(correctedWeaponWorld)) {
                activationWeaponWorld = correctedWeaponWorld;
            }
        }

        AuthoredSupportPalmSeatProximity proximity{};
        if (!resolveAuthoredSupportPalmSeatProximity(
                activationWeaponWorld,
                liveSupportHandWorld,
                authoredSupportHandWeaponLocal,
                supportHandIsLeft,
                proximity)) {
            return;
        }

        auto& snapshot = _authoredSupportGripDebugSnapshot;
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
            g_rockConfig.rockWeaponAuthoredGripActivationRadius;
        snapshot.weaponGenerationKey = candidate.weaponGenerationKey;
        snapshot.captureSequence = candidate.captureSequence;
        snapshot.supportHandIsLeft = supportHandIsLeft;
        snapshot.mirroredForRightSupport = !supportHandIsLeft;
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
        const bool heavyGun = hasWeaponKeywordFlag(
            identity.keywordFlags,
            WeaponKeywordFlag::HeavyGun);
        snapshot.weaponFamily =
            authored_weapon_grip_activation_policy::resolveWeaponFamily(
                authored_weapon_grip_activation_policy::WeaponFamilyInput{
                    .effectiveEquipSlotFormID =
                        identity.effectiveEquipSlotFormID,
                    .equippedWeaponPresent = identity.hasEquippedWeapon,
                    .meleeOrUnarmed = meleeOrUnarmed,
                    .heavyGun = heavyGun,
                });

        const bool canonicalCurrent =
            !_firingHandIsLeft &&
            supportHandIsLeft &&
            _hasRightFiringHandCanonicalWeaponLocal &&
            _rightFiringHandCanonicalSource ==
                RightFiringCanonicalSource::AuthoredAnimation &&
            _rightFiringHandCanonicalWeaponNode == weaponNode &&
            _rightFiringHandCanonicalGenerationKey ==
                currentWeaponGenerationKey &&
            isFiniteTransform(_rightFiringHandCanonicalWeaponLocal);
        if (canonicalCurrent) {
            const RE::NiTransform firingHandWorld =
                transform_math::composeTransforms(
                    activationWeaponWorld,
                    _rightFiringHandCanonicalWeaponLocal);
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
            RE::NiPoint3 leftAxis{};
            const bool leftAxisValid = normalizeVector(
                computePalmNormalFromHandBasis(firingHandWorld, false),
                leftAxis);
            const RE::NiPoint3 thumbUp = transformHandspaceDirection(
                firingHandWorld,
                RE::NiPoint3{ 0.0f, 0.0f, 1.0f },
                false);
            const float thumbLeftProjection =
                thumbUp.x * leftAxis.x +
                thumbUp.y * leftAxis.y +
                thumbUp.z * leftAxis.z;
            RE::NiPoint3 orthogonalUp{
                thumbUp.x - leftAxis.x * thumbLeftProjection,
                thumbUp.y - leftAxis.y * thumbLeftProjection,
                thumbUp.z - leftAxis.z * thumbLeftProjection,
            };
            RE::NiPoint3 normalizedUp{};
            const bool upAxisValid =
                leftAxisValid && normalizeVector(orthogonalUp, normalizedUp);
            if (leftAxisValid && upAxisValid) {
                snapshot.leftAxisWorld = leftAxis;
                snapshot.downAxisWorld = RE::NiPoint3{
                    -normalizedUp.x,
                    -normalizedUp.y,
                    -normalizedUp.z,
                };
                const RE::NiPoint3 referenceAxis{
                    leftAxis.y * snapshot.downAxisWorld.z -
                        leftAxis.z * snapshot.downAxisWorld.y,
                    leftAxis.z * snapshot.downAxisWorld.x -
                        leftAxis.x * snapshot.downAxisWorld.z,
                    leftAxis.x * snapshot.downAxisWorld.y -
                        leftAxis.y * snapshot.downAxisWorld.x,
                };
                snapshot.canonicalAxesValid = normalizeVector(
                    referenceAxis,
                    snapshot.referenceAxisWorld);
            }
        }

        const auto gate = evaluateAuthoredSupportGripDirectionGate(
            snapshot,
            _authoredSupportLastStableApproachDirectionWorld,
            _authoredSupportLastStableApproachDirectionValid,
            snapshot.canonicalAxesValid &&
                !_firingHandIsLeft && supportHandIsLeft);
        if (gate.directionValid &&
            gate.radialDistanceGameUnits >=
                authored_weapon_grip_activation_policy::
                    kMinimumDirectionDistanceGameUnits) {
            _authoredSupportLastStableApproachDirectionWorld =
                snapshot.approachDirectionWorld;
            _authoredSupportLastStableApproachDirectionValid = true;
        }

        if (collectPoseEvidence) {
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


    // ---- Support candidates and primary firing canonicals ----

    void TwoHandedGrip::clearAuthoredSupportGripCandidate()
    {
        _authoredSupportGripCandidate = {};
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
        if (!weaponNode ||
            weaponGenerationKey == 0 ||
            captureSequence == 0 ||
            fingerLocalTransformMask !=
                authored_weapon_grip_library::kCompleteFiringFingerMask ||
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

        _authoredSupportGripCandidate = candidate;
        refreshAuthoredSupportRightMirror();
        return true;
    }

    void TwoHandedGrip::refreshAuthoredSupportRightMirror()
    {
        auto& candidate = _authoredSupportGripCandidate;
        if (!candidate.valid || candidate.rightMirrorValid) {
            return;
        }

        RE::NiTransform mirroredRightHandWeaponLocal{};
        frik_visual_authority::FingerLocalTransformOverride leftFingerLocals{};
        leftFingerLocals.enabledMask = candidate.leftFingerLocalTransformMask;
        for (std::size_t index = 0;
             index < candidate.leftFingerLocalTransforms.size();
             ++index) {
            leftFingerLocals.localTransforms[index] =
                candidate.leftFingerLocalTransforms[index];
        }

        frik_visual_authority::FingerLocalTransformOverride mirroredRightFingerLocals{};
        const bool rightHandTransformMirrored =
            tryBuildMirroredRightSupportHandWeaponLocal(
                candidate.leftHandWeaponLocal,
                mirroredRightHandWeaponLocal);
        const bool rightFingerPoseMirrored =
            frik_visual_authority::mirrorFingerLocalTransforms(
                frik_visual_authority::Hand::Left,
                leftFingerLocals,
                mirroredRightFingerLocals) &&
            mirroredRightFingerLocals.enabledMask ==
                authored_weapon_grip_library::kCompleteFiringFingerMask;
        bool rightFingerPoseFinite = rightFingerPoseMirrored;
        if (rightFingerPoseFinite) {
            for (const auto& fingerLocal : mirroredRightFingerLocals.localTransforms) {
                if (!isFiniteTransform(fingerLocal) ||
                    std::abs(fingerLocal.scale) <= 0.0001f) {
                    rightFingerPoseFinite = false;
                    break;
                }
            }
        }

        if (rightHandTransformMirrored && rightFingerPoseFinite) {
            candidate.rightHandWeaponLocal = mirroredRightHandWeaponLocal;
            for (std::size_t index = 0;
                 index < candidate.rightFingerLocalTransforms.size();
                 ++index) {
                candidate.rightFingerLocalTransforms[index] =
                    mirroredRightFingerLocals.localTransforms[index];
            }
            candidate.rightFingerLocalTransformMask =
                mirroredRightFingerLocals.enabledMask;
            candidate.rightMirrorValid = true;
            return;
        }

        if (_firingHandIsLeft) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000,
                "TwoHandedGrip: authored right-support mirror unavailable transform={} fingers={} naturalFrames=({}, {})",
                rightHandTransformMirrored ? "ready" : "missing",
                rightFingerPoseFinite ? "ready" : "missing",
                _hasLeftNaturalBoneInWand ? "left" : "no-left",
                _hasRightNaturalBoneInWand ? "right" : "no-right");
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

        const auto& candidate = _authoredSupportGripCandidate;
        if (!candidate.valid ||
            !weaponNode ||
            candidate.weaponNode != weaponNode ||
            weaponGenerationKey == 0 ||
            candidate.weaponGenerationKey != weaponGenerationKey) {
            return false;
        }

        if (isLeft) {
            if (candidate.leftFingerLocalTransformMask !=
                authored_weapon_grip_library::
                    kCompleteFiringFingerMask) {
                return false;
            }
            outHandWeaponLocal = candidate.leftHandWeaponLocal;
            outFingerLocalTransforms = candidate.leftFingerLocalTransforms;
            outFingerLocalTransformMask = candidate.leftFingerLocalTransformMask;
        } else {
            if (!candidate.rightMirrorValid ||
                candidate.rightFingerLocalTransformMask !=
                    authored_weapon_grip_library::
                        kCompleteFiringFingerMask) {
                return false;
            }
            outHandWeaponLocal = candidate.rightHandWeaponLocal;
            outFingerLocalTransforms = candidate.rightFingerLocalTransforms;
            outFingerLocalTransformMask = candidate.rightFingerLocalTransformMask;
        }

        return isFiniteTransform(outHandWeaponLocal);
    }

    bool TwoHandedGrip::setAuthoredPrimaryFiringGripCanonical(
        RE::NiNode* weaponNode,
        const RE::NiTransform& rightHandWeaponLocal,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t weaponOwnershipKey,
        const std::uint64_t captureSequence, const authored_weapon_grip_library::FiringFingerPose* rightFingerPose,
        const authored_weapon_grip_library::FiringFingerPose* leftFingerPose)
    {
        const auto validFingerPose = [](const authored_weapon_grip_library::FiringFingerPose* pose) {
            if (!pose) {
                return true;
            }
            if (!pose->complete()) {
                return false;
            }
            return std::ranges::all_of(pose->localTransforms, [](const RE::NiTransform& transform) { return isFiniteTransform(transform) && std::abs(transform.scale) > 0.0001f; });
        };
        if (!weaponNode ||
            weaponGenerationKey == 0 ||
            weaponOwnershipKey == 0 ||
            captureSequence == 0 ||
            !isFiniteTransform(rightHandWeaponLocal) ||
            std::abs(rightHandWeaponLocal.scale) <= 0.0001f || !validFingerPose(rightFingerPose) || !validFingerPose(leftFingerPose) || (leftFingerPose && !rightFingerPose)) {
            return false;
        }

        // HandFrame helpers are frame-agnostic: feeding Hand-in-Weapon yields
        // the configured right palm seat directly in Weapon coordinates. The
        // mirror needs this authored seat, not _primaryGripLocal (which can be
        // a live squeeze or an older native-offset capture).
        const RE::NiPoint3 authoredGripWeaponLocal =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                rightHandWeaponLocal,
                false);
        if (!std::isfinite(authoredGripWeaponLocal.x) ||
            !std::isfinite(authoredGripWeaponLocal.y) ||
            !std::isfinite(authoredGripWeaponLocal.z)) {
            return false;
        }

        const std::uint16_t incomingRightFingerMask = rightFingerPose ? rightFingerPose->enabledMask : 0;
        const std::uint16_t incomingLeftFingerMask = leftFingerPose ? leftFingerPose->enabledMask : 0;
        const bool fingerPoseBoundary =
            _rightFiringFingerLocalTransformMask != incomingRightFingerMask ||
            _leftFiringFingerLocalTransformMask != incomingLeftFingerMask;
        const bool sourceBoundary =
            _rightFiringHandCanonicalSource !=
                RightFiringCanonicalSource::AuthoredAnimation ||
            _rightFiringHandCanonicalWeaponNode != weaponNode ||
            _rightFiringHandCanonicalGenerationKey != weaponGenerationKey ||
            _rightFiringHandCanonicalOwnershipKey != weaponOwnershipKey ||
            fingerPoseBoundary;

        _rightFiringHandCanonicalWeaponLocal = rightHandWeaponLocal;
        _rightFiringGripCanonicalWeaponLocal = authoredGripWeaponLocal;
        _rightFiringHandCanonicalWeaponNode = weaponNode;
        _rightFiringHandCanonicalGenerationKey = weaponGenerationKey;
        _rightFiringHandCanonicalOwnershipKey = weaponOwnershipKey;
        _rightFiringHandCanonicalCaptureSequence = captureSequence;
        _rightFiringHandCanonicalSource =
            RightFiringCanonicalSource::AuthoredAnimation;
        _hasRightFiringHandCanonicalWeaponLocal = true;
        _rightFiringFingerLocalTransforms = rightFingerPose ? rightFingerPose->localTransforms : std::array<RE::NiTransform, 15>{};
        _rightFiringFingerLocalTransformMask = rightFingerPose ? rightFingerPose->enabledMask : 0;
        _leftFiringFingerLocalTransforms = leftFingerPose ? leftFingerPose->localTransforms : std::array<RE::NiTransform, 15>{};
        _leftFiringFingerLocalTransformMask = leftFingerPose ? leftFingerPose->enabledMask : 0;

        if (sourceBoundary) {
            ROCK_LOG_INFO(Animation,
                "TwoHandedGrip: authored firing canonical active generation={:016X} ownership={:016X} capture={} handWeaponT=({:.3f},{:.3f},{:.3f}) "
                "gripWeapon=({:.3f},{:.3f},{:.3f}) rightFingerMask=0x{:04X} leftFingerMask=0x{:04X} leftSource=wand-and-anatomy-mirror",
                weaponGenerationKey,
                weaponOwnershipKey,
                captureSequence,
                rightHandWeaponLocal.translate.x,
                rightHandWeaponLocal.translate.y,
                rightHandWeaponLocal.translate.z,
                authoredGripWeaponLocal.x,
                authoredGripWeaponLocal.y,
                authoredGripWeaponLocal.z, _rightFiringFingerLocalTransformMask, _leftFiringFingerLocalTransformMask);
        }
        return true;
    }

    bool TwoHandedGrip::publishAuthoredPrimaryFiringGripFingerPose(const bool isLeft)
    {
        const bool targetHandHoldingObject =
            isLeft ? _leftHandHoldingObjectForPose : _rightHandHoldingObjectForPose;
        if (_authoredPrimaryFingerPoseSuppressed ||
            !authored_weapon_grip_capture_policy::shouldPublishAuthoredFiringFingerPose(
                targetHandHoldingObject) ||
            _rightFiringHandCanonicalSource != RightFiringCanonicalSource::AuthoredAnimation) {
            return false;
        }

        const auto& transforms = isLeft ? _leftFiringFingerLocalTransforms : _rightFiringFingerLocalTransforms;
        const std::uint16_t mask = isLeft ? _leftFiringFingerLocalTransformMask : _rightFiringFingerLocalTransformMask;
        if (mask != authored_weapon_grip_library::kCompleteFiringFingerMask) {
            return false;
        }

        if (_authoredPrimaryFingerPosePublished && _publishedFiringFingerPoseIsLeft != isLeft) {
            clearAuthoredPrimaryFiringGripFingerPose();
        }

        if (!_authoredPrimaryFingerPoseBlockEngaged) {
            if (!frik_visual_authority::blockPrimaryHandWeaponPose(AUTHORED_PRIMARY_POSE_BLOCK_TAG, true)) {
                return false;
            }
            _authoredPrimaryFingerPoseBlockEngaged = true;
        }

        const auto hand = handFromBool(isLeft);
        _publishedFiringFingerPoseIsLeft = isLeft;
        if (!frik_visual_authority::setHandPoseCustomWithPriority(PRIMARY_GRIP_TAG, hand, frik_visual_authority::HandPoseData{}, GRIP_HAND_POSE_PRIORITY)) {
            clearAuthoredPrimaryFiringGripFingerPose();
            return false;
        }

        frik_visual_authority::FingerLocalTransformOverride overrideData{};
        overrideData.enabledMask = mask;
        for (std::size_t index = 0; index < transforms.size(); ++index) {
            overrideData.localTransforms[index] = transforms[index];
        }
        if (!frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(PRIMARY_GRIP_TAG, hand, &overrideData, GRIP_HAND_POSE_PRIORITY)) {
            clearAuthoredPrimaryFiringGripFingerPose();
            return false;
        }

        _publishedFiringFingerPoseIsLeft = isLeft;
        _authoredPrimaryFingerPosePublished = true;
        return true;
    }

    void TwoHandedGrip::clearAuthoredPrimaryFiringGripFingerPose()
    {
        if (_authoredPrimaryFingerPosePublished || _authoredPrimaryFingerPoseBlockEngaged) {
            (void)frik_visual_authority::clearHandPose(PRIMARY_GRIP_TAG, handFromBool(_publishedFiringFingerPoseIsLeft));
        }
        if (_authoredPrimaryFingerPoseBlockEngaged) {
            (void)frik_visual_authority::blockPrimaryHandWeaponPose(AUTHORED_PRIMARY_POSE_BLOCK_TAG, false);
        }
        _publishedFiringFingerPoseIsLeft = false;
        _authoredPrimaryFingerPosePublished = false;
        _authoredPrimaryFingerPoseBlockEngaged = false;
    }

    void TwoHandedGrip::setAuthoredPrimaryFiringGripFingerPoseSuppressed(const bool suppressed)
    {
        _authoredPrimaryFingerPoseSuppressed = suppressed;
        if (suppressed) {
            clearAuthoredPrimaryFiringGripFingerPose();
        }
    }

    void TwoHandedGrip::setGrabbedObjectHandPoseOwnership(
        const bool leftHandHoldingObject,
        const bool rightHandHoldingObject)
    {
        _leftHandHoldingObjectForPose = leftHandHoldingObject;
        _rightHandHoldingObjectForPose = rightHandHoldingObject;

        if (!_authoredPrimaryFingerPosePublished) {
            return;
        }

        const bool publishedHandHoldingObject =
            _publishedFiringFingerPoseIsLeft ?
                _leftHandHoldingObjectForPose :
                _rightHandHoldingObjectForPose;
        if (publishedHandHoldingObject) {
            clearAuthoredPrimaryFiringGripFingerPose();
        }
    }

    void TwoHandedGrip::clearAuthoredPrimaryFiringGripCanonical(
        const char* reason)
    {
        if (_rightFiringHandCanonicalSource !=
            RightFiringCanonicalSource::AuthoredAnimation) {
            return;
        }

        ROCK_LOG_DEBUG(Animation,
            "TwoHandedGrip: clearing authored firing canonical reason={} generation={:016X} ownership={:016X} capture={}",
            reason ? reason : "unknown",
            _rightFiringHandCanonicalGenerationKey,
            _rightFiringHandCanonicalOwnershipKey,
            _rightFiringHandCanonicalCaptureSequence);
        clearAuthoredPrimaryFiringGripFingerPose();
        clearRightFiringHandCanonicalFrame();
    }

    bool TwoHandedGrip::applyAuthoredPrimaryGripWeaponAlignment(
        RE::NiNode* weaponNode,
        const RE::NiTransform& solvedWeaponWorld,
        const std::uint64_t currentWeaponGenerationKey)
    {
        if (blocksAuthoredPrimaryGripWeaponAlignment() || isWeaponVisualReturnActive()) {
            return false;
        }
        return applyWeaponVisualAuthority(
            weaponNode,
            solvedWeaponWorld,
            currentWeaponGenerationKey);
    }

    void TwoHandedGrip::reframeAuthoredSupportGripDebugSnapshot(
        const RE::NiTransform& finalWeaponWorld)
    {
        auto& snapshot = _authoredSupportGripDebugSnapshot;
        if (!snapshot.valid ||
            !isFiniteTransform(snapshot.weaponWorld) ||
            !isFiniteTransform(finalWeaponWorld)) {
            return;
        }

        const RE::NiTransform previousWeaponWorld = snapshot.weaponWorld;
        const auto reframePoint = [&](const RE::NiPoint3& pointWorld) {
            return transform_math::localPointToWorld(
                finalWeaponWorld,
                transform_math::worldPointToLocal(
                    previousWeaponWorld,
                    pointWorld));
        };
        const auto reframeDirection = [&](const RE::NiPoint3& directionWorld) {
            RE::NiPoint3 reframed{};
            (void)gunstock_alignment_policy::tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    finalWeaponWorld,
                    transform_math::worldVectorToLocal(
                        previousWeaponWorld,
                        directionWorld)),
                reframed);
            return reframed;
        };

        snapshot.authoredPalmSeatWorld =
            transform_math::localPointToWorld(
                finalWeaponWorld,
                snapshot.authoredPalmSeatWeaponLocal);
        snapshot.leftAxisWorld = reframeDirection(snapshot.leftAxisWorld);
        snapshot.downAxisWorld = reframeDirection(snapshot.downAxisWorld);
        snapshot.referenceAxisWorld =
            reframeDirection(snapshot.referenceAxisWorld);
        for (auto& landmark : snapshot.poseLandmarksWorld) {
            landmark = reframePoint(landmark);
        }
        for (std::size_t index = 0;
             index < snapshot.poseSurfaceWitnessWorld.size();
             ++index) {
            if ((snapshot.poseSurfaceWitnessMask &
                    static_cast<std::uint8_t>(1u << index)) != 0) {
                snapshot.poseSurfaceWitnessWorld[index] =
                    reframePoint(snapshot.poseSurfaceWitnessWorld[index]);
            }
        }

        AuthoredSupportPalmSeatProximity proximity{};
        if (!resolveAuthoredSupportPalmSeatProximityFromPoints(
                finalWeaponWorld,
                snapshot.liveTouchProbeWorld,
                snapshot.authoredPalmSeatWeaponLocal,
                proximity)) {
            return;
        }
        snapshot.authoredPalmSeatWorld = proximity.authoredPalmSeatWorld;
        snapshot.liveTouchProbeWeaponLocal =
            proximity.liveTouchProbeWeaponLocal;
        const RE::NiPoint3 approach{
            snapshot.liveTouchProbeWorld.x -
                snapshot.authoredPalmSeatWorld.x,
            snapshot.liveTouchProbeWorld.y -
                snapshot.authoredPalmSeatWorld.y,
            snapshot.liveTouchProbeWorld.z -
                snapshot.authoredPalmSeatWorld.z,
        };
        snapshot.approachDirectionWorld = {};
        const bool approachValid =
            gunstock_alignment_policy::tryNormalizeDirection(
                approach,
                snapshot.approachDirectionWorld);
        snapshot.weaponRelativeDistanceGameUnits =
            proximity.weaponRelativeDistanceGameUnits;
        snapshot.worldReadbackDistanceGameUnits =
            proximity.worldReadbackDistanceGameUnits;
        snapshot.frameAgreementErrorGameUnits =
            proximity.frameAgreementErrorGameUnits;
        snapshot.insideTouchRadius =
            snapshot.weaponRelativeDistanceGameUnits <=
            snapshot.touchRadiusGameUnits;

        (void)evaluateAuthoredSupportGripDirectionGate(
            snapshot,
            snapshot.approachDirectionWorld,
            approachValid,
            snapshot.canonicalAxesValid &&
                !_firingHandIsLeft && snapshot.supportHandIsLeft);
        snapshot.weaponWorld = finalWeaponWorld;
    }


    // ---- Selected authored-pose snapshot ----

    bool TwoHandedGrip::getSelectedAuthoredGripPoseSnapshot(
        SelectedAuthoredGripPoseSnapshot& outSnapshot) const
    {
        outSnapshot = {};
        const auto generationKey = _activeWeaponGenerationKey != 0 ?
            _activeWeaponGenerationKey :
            _rightFiringHandCanonicalGenerationKey;
        if (generationKey == 0) {
            return false;
        }

        const bool canonicalCurrent =
            _hasRightFiringHandCanonicalWeaponLocal &&
            _rightFiringHandCanonicalGenerationKey == generationKey &&
            (!_activeWeaponNode ||
                _rightFiringHandCanonicalWeaponNode == _activeWeaponNode);
        const bool supportCurrent =
            _authoredSupportGripCandidate.valid &&
            _authoredSupportGripCandidate.weaponGenerationKey == generationKey &&
            (!_activeWeaponNode ||
                _authoredSupportGripCandidate.weaponNode == _activeWeaponNode);
        if (!canonicalCurrent && !supportCurrent) {
            return false;
        }

        outSnapshot.weaponGenerationKey = generationKey;
        if (canonicalCurrent) {
            outSnapshot.rightHandWeaponLocal =
                _rightFiringHandCanonicalWeaponLocal;
            outSnapshot.rightHandValid = true;
            outSnapshot.rightFingerLocalTransforms =
                _rightFiringFingerLocalTransforms;
            outSnapshot.rightFingerLocalTransformMask =
                _rightFiringFingerLocalTransformMask;
            outSnapshot.captureSequence =
                _rightFiringHandCanonicalCaptureSequence;
            outSnapshot.source =
                _rightFiringHandCanonicalSource ==
                        RightFiringCanonicalSource::AuthoredAnimation ?
                    SelectedAuthoredGripPoseSnapshot::Source::NativeIdlePreharvest :
                    SelectedAuthoredGripPoseSnapshot::Source::RuntimeCanonical;
        }

        if (supportCurrent) {
            outSnapshot.leftHandWeaponLocal =
                _authoredSupportGripCandidate.leftHandWeaponLocal;
            outSnapshot.leftHandValid = true;
            outSnapshot.leftFingerLocalTransforms =
                _authoredSupportGripCandidate.leftFingerLocalTransforms;
            outSnapshot.leftFingerLocalTransformMask =
                _authoredSupportGripCandidate.leftFingerLocalTransformMask;
            outSnapshot.captureSequence = (std::max)(
                outSnapshot.captureSequence,
                _authoredSupportGripCandidate.captureSequence);
            if (!canonicalCurrent) {
                outSnapshot.rightHandWeaponLocal =
                    _authoredSupportGripCandidate.rightHandWeaponLocal;
                outSnapshot.rightHandValid =
                    _authoredSupportGripCandidate.rightMirrorValid;
                outSnapshot.rightFingerLocalTransforms =
                    _authoredSupportGripCandidate.rightFingerLocalTransforms;
                outSnapshot.rightFingerLocalTransformMask =
                    _authoredSupportGripCandidate.rightFingerLocalTransformMask;
            }
            outSnapshot.source =
                SelectedAuthoredGripPoseSnapshot::Source::NativeIdlePreharvest;
        } else if (canonicalCurrent) {
            RE::NiTransform leftHandWeaponLocal{};
            if (tryComputeMirroredLeftFiringHandWeaponLocal(
                    leftHandWeaponLocal,
                    nullptr,
                    false)) {
                outSnapshot.leftHandWeaponLocal = leftHandWeaponLocal;
                outSnapshot.leftHandValid = true;
                outSnapshot.leftFingerLocalTransforms =
                    _leftFiringFingerLocalTransforms;
                outSnapshot.leftFingerLocalTransformMask =
                    _leftFiringFingerLocalTransformMask;
            }
        }

        outSnapshot.variantKey = outSnapshot.captureSequence != 0 ?
            outSnapshot.captureSequence :
            generationKey;
        outSnapshot.valid =
            outSnapshot.rightHandValid || outSnapshot.leftHandValid;
        return outSnapshot.valid;
    }

}
