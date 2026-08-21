#include "physics-interaction/weapon/two_handed/TwoHandedGrip.h"

/*
 * GUNSTOCK mode: shouldering a two-handed weapon so the sight line follows the
 * head instead of the hands.
 *
 * Flow order in this file is eligibility, mode reconcile, baselines, correction,
 * publication, then the post-animation finalize.
 *
 * ORDERING COUPLINGS, documented at both ends:
 *   - refreshScopeSafeHandFrames in TwoHandedGripNativeScope.cpp calls
 *     tryResolveGunstockPhysicalFiringFrame here and reads
 *     _gunstockWeaponEligibility. Scope work therefore runs AFTER gunstock
 *     eligibility is observed for the frame.
 *   - refreshAuthoredSupportGripActivationState in
 *     TwoHandedGripAuthoredGrip.cpp calls
 *     tryResolveGunstockPrimaryGroupCorrection here.
 *   - applyGunstockAlignment and
 *     finalizeGunstockPresentationAfterNativeWeaponAnimation both write
 *     _authoredSupportGripDebugSnapshot, which the authored TU also writes. The
 *     gunstock write runs later and reframes it deliberately.
 *   - hasVisualAuthorityForHand in TwoHandedGripHandAuthority.cpp reads
 *     _gunstockHandAuthorityActive, which only this file sets.
 *
 * applyGunstockAlignment publishes hand and weapon rigidly, and rolls the whole
 * publication back on failure. Keep the rollback next to the publication it
 * unwinds. A rollback that drifts away from its publication stops matching it.
 */
#include "physics-interaction/weapon/two_handed/TwoHandedGripInternal.h"

#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/collision/WeaponCollision.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

namespace rock
{
    using two_handed_grip_detail::arePointsNearlyEqual;
    using two_handed_grip_detail::areTransformsNearlyEqual;
    using two_handed_grip_detail::AuthoredSupportPalmSeatProximity;
    using two_handed_grip_detail::configuredGunstockFineTune;
    using two_handed_grip_detail::GRIP_HAND_POSE_PRIORITY;
    using two_handed_grip_detail::GUNSTOCK_ALIGNMENT_TAG;
    using two_handed_grip_detail::isFiniteTransform;
    using two_handed_grip_detail::isUsableHandAuthorityTransform;
    using two_handed_grip_detail::moveWeaponPresentationRigidly;
    using two_handed_grip_detail::PRIMARY_GRIP_TAG;
    using two_handed_grip_detail::SUPPORT_GRIP_TAG;
    using two_handed_grip_detail::tryGetRootFlattenedHandBoneTransform;

    void TwoHandedGrip::observeGunstockWeaponEligibility(
        RE::NiNode* weaponNode,
        RE::NiAVObject* observedFireNode,
        const bool observedGunType,
        const std::uint64_t currentWeaponGenerationKey)
    {
        const bool featureOrDebugActive =
            g_rockConfig.rockGunstockModeEnabled ||
            g_rockConfig.rockDebugDrawGunstockAlignment;
        const bool validFireNodeObserved =
            observedFireNode &&
            isFiniteTransform(observedFireNode->world);
        gunstock_alignment_policy::observeWeaponEligibility(
            _gunstockWeaponEligibility,
            reinterpret_cast<std::uintptr_t>(weaponNode),
            currentWeaponGenerationKey,
            featureOrDebugActive,
            observedGunType,
            validFireNodeObserved);
    }


    bool TwoHandedGrip::isGunstockWeaponEligible(
        const RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey) const
    {
        return gunstock_alignment_policy::isWeaponEligible(
            _gunstockWeaponEligibility,
            reinterpret_cast<std::uintptr_t>(weaponNode),
            currentWeaponGenerationKey);
    }


    bool TwoHandedGrip::isGunstockWeaponGenerationEligible(
        const std::uint64_t currentWeaponGenerationKey) const
    {
        return _gunstockWeaponEligibility.eligible &&
               currentWeaponGenerationKey != 0 &&
               _gunstockWeaponEligibility.weaponGenerationKey ==
                   currentWeaponGenerationKey;
    }


    bool TwoHandedGrip::isGunstockSupportBaselineActive(
        const bool supportHandIsLeft,
        const WeaponPartGrip& supportGrip) const
    {
        return g_rockConfig.rockGunstockModeEnabled &&
               isGunstockWeaponGenerationEligible(
                   supportGrip.weaponGenerationKey) &&
               isSupportInputBaselineActive(
                   supportHandIsLeft,
                   supportGrip,
                   SupportInputBaselineKind::Gunstock);
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
               baseline.gripSequence != 0 &&
               baseline.gripSequence == supportGrip.gripSequence;
    }


    bool TwoHandedGrip::initializeSupportInputBaseline(
        RE::NiNode* weaponNode,
        const bool supportHandIsLeft,
        const RE::NiTransform& supportInputWorld,
        const SupportInputBaselineKind kind,
        const char* reason)
    {
        WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        if (!weaponNode ||
            !supportGrip.active ||
            supportGrip.weaponGenerationKey == 0 ||
            supportGrip.gripSequence == 0 ||
            kind == SupportInputBaselineKind::None) {
            return false;
        }
        if (isSupportInputBaselineActive(
                supportHandIsLeft,
                supportGrip,
                kind)) {
            return true;
        }

        supportGrip.supportInputBaseline = {};
        const RE::NiTransform supportGripTargetWorld =
            resolvePartGripHandWorld(supportGrip, weaponNode);
        RE::NiTransform inputToGripTargetLocal{};
        if (!weapon_support_acquisition_math::
                tryCaptureSupportInputBaseline(
                    supportInputWorld,
                    supportGripTargetWorld,
                    inputToGripTargetLocal)) {
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: support input baseline capture failed mode={} hand={} authored={} provider={} grip={} generation={:016X} reason={}",
                kind == SupportInputBaselineKind::Gunstock ?
                    "gunstock" :
                    "dynamic",
                supportHandIsLeft ? "left" : "right",
                supportGrip.authoredSupportGrip ? "yes" : "no",
                supportGrip.providerPartAuthority.active ? "yes" : "no",
                supportGrip.gripSequence,
                supportGrip.weaponGenerationKey,
                reason ? reason : "unknown");
            return false;
        }

        supportGrip.supportInputBaseline = {
            .inputToGripTargetLocal = inputToGripTargetLocal,
            .weaponWorldAtCapture = weaponNode->world,
            .weaponGenerationKey = supportGrip.weaponGenerationKey,
            .gripSequence = supportGrip.gripSequence,
            .supportHandIsLeft = supportHandIsLeft,
            .kind = kind,
            .active = true,
            .firstPublicationPending = true,
        };
        ROCK_LOG_INFO(
            Weapon,
            "TwoHandedGrip: support input baseline captured mode={} hand={} authored={} provider={} grip={} generation={:016X} reason={}; attach weapon transform retained",
            kind == SupportInputBaselineKind::Gunstock ?
                "gunstock" :
                "dynamic",
            supportHandIsLeft ? "left" : "right",
            supportGrip.authoredSupportGrip ? "yes" : "no",
            supportGrip.providerPartAuthority.active ? "yes" : "no",
            supportGrip.gripSequence,
            supportGrip.weaponGenerationKey,
            reason ? reason : "unknown");
        return true;
    }


    bool TwoHandedGrip::initializeGunstockSupportRole(
        RE::NiNode* weaponNode,
        const bool supportHandIsLeft,
        const RE::NiTransform& supportInputWorld,
        const char* reason)
    {
        WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        if (!g_rockConfig.rockGunstockModeEnabled ||
            _authorityMode != weapon_support_authority_policy::
                                  WeaponSupportAuthorityMode::
                                      FullTwoHandedSolver ||
            !isGunstockWeaponGenerationEligible(
                supportGrip.weaponGenerationKey)) {
            if (supportGrip.supportInputBaseline.kind ==
                SupportInputBaselineKind::Gunstock) {
                supportGrip.supportInputBaseline = {};
            }
            return true;
        }
        if (isGunstockSupportBaselineActive(
                supportHandIsLeft,
                supportGrip)) {
            return true;
        }
        return initializeSupportInputBaseline(
            weaponNode,
            supportHandIsLeft,
            supportInputWorld,
            SupportInputBaselineKind::Gunstock,
            reason);
    }


    void TwoHandedGrip::clearSupportInputBaselines()
    {
        for (auto& grip : _partGrips) {
            grip.supportInputBaseline = {};
        }
    }


    void TwoHandedGrip::clearGunstockSupportBaselines()
    {
        for (auto& grip : _partGrips) {
            if (grip.supportInputBaseline.kind ==
                SupportInputBaselineKind::Gunstock) {
                grip.supportInputBaseline = {};
            }
        }
    }


    bool TwoHandedGrip::reconcileGunstockModeState(
        RE::NiNode* weaponNode)
    {
        const bool enabled = g_rockConfig.rockGunstockModeEnabled;
        const auto edge = gunstock_alignment_policy::observeModeToggle(
            _gunstockModeToggle,
            enabled);

        if (edge == gunstock_alignment_policy::ModeToggleEdge::Disabled) {
            const bool supportHandIsLeft = !_firingHandIsLeft;
            WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
            RE::NiTransform primaryStartWorld{};
            RE::NiTransform supportStartWorld{};
            (void)tryGetRootFlattenedHandBoneTransform(
                _firingHandIsLeft,
                primaryStartWorld);
            (void)tryGetRootFlattenedHandBoneTransform(
                supportHandIsLeft,
                supportStartWorld);
            const std::size_t primaryIndex =
                _firingHandIsLeft ? 0u : 1u;
            const std::size_t supportIndex =
                supportHandIsLeft ? 0u : 1u;
            if (!isUsableHandAuthorityTransform(primaryStartWorld) &&
                _hasLastPublishedHandWorld[primaryIndex]) {
                primaryStartWorld =
                    _lastPublishedHandWorld[primaryIndex];
            }
            if (!isUsableHandAuthorityTransform(supportStartWorld) &&
                _hasLastPublishedHandWorld[supportIndex]) {
                supportStartWorld =
                    _lastPublishedHandWorld[supportIndex];
            }

            resetGunstockAlignment("config-disabled-edge");
            clearGunstockSupportBaselines();
            if (_state == TwoHandedState::Gripping &&
                _authorityMode == weapon_support_authority_policy::
                                      WeaponSupportAuthorityMode::
                                          FullTwoHandedSolver &&
                supportGrip.active &&
                isUsableHandAuthorityTransform(primaryStartWorld) &&
                isUsableHandAuthorityTransform(supportStartWorld)) {
                if (!initializeDynamicSupportBaseline(
                        weaponNode,
                        supportHandIsLeft,
                        "gunstock-mode-disabled")) {
                    ROCK_LOG_WARN(
                        Weapon,
                        "TwoHandedGrip: gunstock disable transition failed closed because the ordinary support baseline could not be captured");
                    transitionToInactive(false);
                    return false;
                }
                beginDynamicSupportAcquisition(
                    supportHandIsLeft,
                    supportGrip,
                    primaryStartWorld,
                    supportStartWorld);
                _rotationBlend = 0.0f;
            }
            ROCK_LOG_INFO(
                Weapon,
                "TwoHandedGrip: gunstock mode disabled; final alignment and support baselines cleared with ordinary support acquisition rebased from the rendered hands");
            return true;
        }

        if (edge == gunstock_alignment_policy::ModeToggleEdge::Enabled) {
            resetGunstockAlignment("config-enabled-edge");
            clearDynamicSupportAcquisition(
                "gunstock-mode-enabled",
                true);
            clearGunstockSupportBaselines();
            ROCK_LOG_INFO(
                Weapon,
                "TwoHandedGrip: gunstock mode enabled; awaiting one generation-bound firearm/support transaction");
        }

        if (!enabled) {
            clearGunstockSupportBaselines();
            return true;
        }

        if (_state != TwoHandedState::Gripping ||
            _authorityMode != weapon_support_authority_policy::
                                  WeaponSupportAuthorityMode::
                                      FullTwoHandedSolver) {
            return true;
        }

        const bool supportHandIsLeft = !_firingHandIsLeft;
        WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        if (!supportGrip.active ||
            !isGunstockWeaponGenerationEligible(
                supportGrip.weaponGenerationKey)) {
            if (supportGrip.supportInputBaseline.kind ==
                SupportInputBaselineKind::Gunstock) {
                supportGrip.supportInputBaseline = {};
            }
            return true;
        }
        if (isGunstockSupportBaselineActive(
                supportHandIsLeft,
                supportGrip)) {
            return true;
        }

        RE::NiTransform supportInputWorld{};
        if (!tryGetSolverHandTransform(
                supportHandIsLeft,
                supportInputWorld)) {
            return false;
        }
        return initializeGunstockSupportRole(
            _activeWeaponNode ? _activeWeaponNode : weaponNode,
            supportHandIsLeft,
            supportInputWorld,
            edge == gunstock_alignment_policy::ModeToggleEdge::Enabled ?
                "mode-enabled-live-grip" :
                "eligibility-or-generation-rebase");
    }


    void TwoHandedGrip::clearGunstockDedicatedHandAuthority()
    {
        for (std::size_t index = 0; index <
             _gunstockDedicatedHandAuthorityActive.size(); ++index) {
            if (!_gunstockDedicatedHandAuthorityActive[index]) {
                continue;
            }
            const bool isLeft = index == 0u;
            (void)frik_visual_authority::clearExternalHandWorldTransform(
                GUNSTOCK_ALIGNMENT_TAG,
                handFromBool(isLeft));
            clearPreFrikRetainedHandAuthority(
                RetainedHandAuthorityKind::GunstockAlignment,
                isLeft);
            _gunstockDedicatedHandAuthorityActive[index] = false;
        }
        _gunstockHandAuthorityActive = {};
    }


    void TwoHandedGrip::resetGunstockAlignment(const char* reason)
    {
        const bool hadState =
            _gunstockAlignment.weaponNodeIdentity != nullptr ||
            _gunstockAlignment.directionLatch.candidateSamples != 0 ||
            _gunstockAlignment.directionLatch.latched ||
            _gunstockDedicatedHandAuthorityActive[0] ||
            _gunstockDedicatedHandAuthorityActive[1];
        clearGunstockDedicatedHandAuthority();
        _gunstockAlignment = {};
        _gunstockFramePresentation = {};
        _gunstockWaitingLogged = false;
        _gunstockYieldLogged = false;
        _gunstockLatchedContinuityLogged = false;
        _gunstockProjectileWitnessMissingLogged = false;
        if (hadState) {
            ROCK_LOG_DEBUG(
                Weapon,
                "TwoHandedGrip: gunstock alignment cleared reason={}",
                reason ? reason : "unknown");
        }
    }


    GunstockAlignmentDebugYieldReason
        TwoHandedGrip::currentGunstockAlignmentYieldReason(
            const bool authorityBlocked) const
    {
        const bool firingHandHoldingObject = _firingHandIsLeft ?
            _leftHandHoldingObjectForPose :
            _rightHandHoldingObjectForPose;

        if (authorityBlocked) {
            return GunstockAlignmentDebugYieldReason::AuthorityBlocked;
        }
        if (_state == TwoHandedState::PartCarry) {
            return GunstockAlignmentDebugYieldReason::PartCarry;
        }
        if (firingHandHoldingObject) {
            return GunstockAlignmentDebugYieldReason::
                FiringHandHoldingObject;
        }
        return GunstockAlignmentDebugYieldReason::None;
    }


    bool TwoHandedGrip::tryResolveGunstockPhysicalFiringFrame(
        RE::NiTransform& outHandWorld,
        RE::NiTransform& outDriverWorld) const
    {
        outHandWorld = {};
        outDriverWorld = {};

        auto* playerNodes = f4vr::getPlayerNodes();
        RE::NiNode* dampedDriver = playerNodes ?
            (_firingHandIsLeft ?
                    playerNodes->SecondaryMeleeWeaponOffsetNode2 :
                    playerNodes->primaryWeaponOffsetNOde) :
            nullptr;
        const RE::NiTransform& boneInDriver = _firingHandIsLeft ?
            _leftNaturalBoneInDampedDriver :
            _rightNaturalBoneInDampedDriver;
        const bool relationValid = _firingHandIsLeft ?
            _hasLeftNaturalBoneInDampedDriver :
            _hasRightNaturalBoneInDampedDriver;
        if (!dampedDriver ||
            !relationValid ||
            !isFiniteTransform(dampedDriver->world) ||
            !isFiniteTransform(boneInDriver)) {
            return false;
        }

        outDriverWorld = dampedDriver->world;
        outHandWorld = transform_math::composeTransforms(
            outDriverWorld,
            boneInDriver);
        return isUsableHandAuthorityTransform(outHandWorld) &&
               isFiniteTransform(outDriverWorld);
    }


    bool TwoHandedGrip::tryGetGunstockTrackedFiringHandWorld(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        RE::NiTransform& outHandWorld) const
    {
        outHandWorld = {};
        if (!g_rockConfig.rockGunstockModeEnabled ||
            !isGunstockWeaponEligible(
                weaponNode,
                currentWeaponGenerationKey) ||
            !runtime_state::isLocalSkeletonReady()) {
            return false;
        }

        RE::NiTransform driverWorld{};
        return tryResolveGunstockPhysicalFiringFrame(
            outHandWorld,
            driverWorld);
    }


    bool TwoHandedGrip::tryResolveGunstockPrimaryGroupCorrection(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        RE::NiTransform& outAlignmentHandWorld,
        RE::NiMatrix3& outCorrectionWorld,
        RE::NiPoint3& outPivotWorld,
        float* outDirectionDot,
        bool* outFineTuneActive) const
    {
        outAlignmentHandWorld = {};
        outCorrectionWorld = {};
        outPivotWorld = {};
        if (outFineTuneActive) {
            *outFineTuneActive = false;
        }
        if (!g_rockConfig.rockGunstockModeEnabled ||
            !weaponNode ||
            currentWeaponGenerationKey == 0 ||
            !isGunstockWeaponEligible(
                weaponNode,
                currentWeaponGenerationKey) ||
            _gunstockAlignment.weaponNodeIdentity != weaponNode ||
            _gunstockAlignment.weaponGenerationKey !=
                currentWeaponGenerationKey ||
            _gunstockAlignment.firingHandIsLeft != _firingHandIsLeft ||
            !_gunstockAlignment.directionLatch.latched ||
            currentGunstockAlignmentYieldReason(
                _gunstockAlignmentBlockedThisFrame) !=
                GunstockAlignmentDebugYieldReason::None) {
            return false;
        }

        RE::NiTransform driverWorld{};
        if (!tryResolveGunstockPhysicalFiringFrame(
                outAlignmentHandWorld,
                driverWorld)) {
            return false;
        }

        const RE::NiPoint3 localWristForward{ 1.0f, 0.0f, 0.0f };
        RE::NiPoint3 wristForwardWorld{};
        const auto fineTune = configuredGunstockFineTune();
        if (!gunstock_alignment_policy::tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    outAlignmentHandWorld,
                    localWristForward),
                wristForwardWorld) ||
            !gunstock_alignment_policy::tryBuildFineTunedWorldCorrection<
                RE::NiTransform,
                RE::NiMatrix3,
                RE::NiPoint3>(
                outAlignmentHandWorld,
                _gunstockAlignment.directionLatch.neutralHandLocal,
                wristForwardWorld,
                fineTune,
                outCorrectionWorld,
                nullptr,
                outDirectionDot)) {
            return false;
        }
        if (outFineTuneActive) {
            *outFineTuneActive =
                gunstock_alignment_policy::hasFineTune(fineTune);
        }

        outPivotWorld = driverWorld.translate;
        return std::isfinite(outPivotWorld.x) &&
               std::isfinite(outPivotWorld.y) &&
               std::isfinite(outPivotWorld.z);
    }

    bool TwoHandedGrip::populateGunstockAlignmentDebugPrediction(
        const RE::NiPoint3& neutralHandLocal)
    {
        auto& snapshot = _gunstockAlignmentDebugSnapshot;
        snapshot.correctionValid = false;
        snapshot.correctionAxisValid = false;
        snapshot.correctionUsedAntiparallelFallback = false;
        snapshot.predictionValid = false;
        snapshot.correctionWorld = {};
        snapshot.predictedWeaponWorld = {};
        snapshot.predictedFireNodeWorld = {};
        snapshot.wristForwardWorld = {};
        snapshot.fineTunedTargetForwardWorld = {};
        snapshot.weaponRootForwardWorld = {};
        snapshot.unalignedLiveFireWorld = {};
        snapshot.neutralFireWorldBefore = {};
        snapshot.predictedNeutralFireWorld = {};
        snapshot.correctionAxisWorld = {};
        snapshot.unalignedAngleDegrees = 0.0f;
        snapshot.correctionAngleRadians = 0.0f;
        snapshot.correctionAngleDegrees = 0.0f;
        const auto fineTune = configuredGunstockFineTune();
        snapshot.fineTunePitchDegrees = fineTune.pitchDegrees;
        snapshot.fineTuneYawDegrees = fineTune.yawDegrees;
        snapshot.fineTuneRollDegrees = fineTune.rollDegrees;
        snapshot.fineTuneTargetOffsetDegrees = 0.0f;
        snapshot.fineTuneActive =
            gunstock_alignment_policy::hasFineTune(fineTune);
        snapshot.neutralResidualDegrees = 0.0f;

        if (!snapshot.firingHandValid ||
            !snapshot.weaponBeforeValid ||
            !snapshot.fireNodeBeforeValid) {
            return false;
        }

        const RE::NiPoint3 localForward{ 0.0f, 1.0f, 0.0f };
        const RE::NiPoint3 localWristForward{ 1.0f, 0.0f, 0.0f };
        RE::NiPoint3 wristForward{};
        RE::NiPoint3 weaponRootForward{};
        RE::NiPoint3 unalignedLiveFire{};
        RE::NiPoint3 neutralLocal{};
        RE::NiPoint3 neutralFireWorld{};
        if (!gunstock_alignment_policy::tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    snapshot.firingHandWorld,
                    localWristForward),
                wristForward) ||
            !gunstock_alignment_policy::tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    snapshot.weaponWorldBefore,
                    localForward),
                weaponRootForward) ||
            !gunstock_alignment_policy::tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    snapshot.fireNodeWorldBefore,
                    localForward),
                unalignedLiveFire) ||
            !gunstock_alignment_policy::tryNormalizeDirection(
                neutralHandLocal,
                neutralLocal) ||
            !gunstock_alignment_policy::tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    snapshot.firingHandWorld,
                    neutralLocal),
                neutralFireWorld)) {
            return false;
        }

        float directionDot = 1.0f;
        RE::NiMatrix3 correction{};
        RE::NiPoint3 fineTunedTarget{};
        if (!gunstock_alignment_policy::tryBuildFineTunedWorldCorrection<
                RE::NiTransform,
                RE::NiMatrix3,
                RE::NiPoint3>(
                snapshot.firingHandWorld,
                neutralLocal,
                wristForward,
                fineTune,
                correction,
                &fineTunedTarget,
                &directionDot)) {
            return false;
        }

        const RE::NiTransform predictedWeapon =
            gunstock_alignment_policy::rotateRigidlyAroundPivot<
                RE::NiTransform,
                RE::NiMatrix3,
                RE::NiPoint3>(
                snapshot.weaponWorldBefore,
                correction,
                snapshot.pivotWorld);
        const RE::NiTransform predictedFire =
            gunstock_alignment_policy::rotateRigidlyAroundPivot<
                RE::NiTransform,
                RE::NiMatrix3,
                RE::NiPoint3>(
                snapshot.fireNodeWorldBefore,
                correction,
                snapshot.pivotWorld);
        RE::NiPoint3 predictedNeutral{};
        if (!isFiniteTransform(predictedWeapon) ||
            !isFiniteTransform(predictedFire) ||
            !gunstock_alignment_policy::tryNormalizeDirection(
                weaponSolverApplyStoredWorldRotationToVector<
                    RE::NiMatrix3,
                    RE::NiPoint3>(
                    correction,
                    neutralFireWorld),
                predictedNeutral)) {
            return false;
        }

        const float clampedDot = (std::max)(
            -1.0f,
            (std::min)(1.0f, directionDot));
        const float correctionAngle = std::acos(clampedDot);
        RE::NiPoint3 correctionAxis =
            weaponSolverCross(neutralFireWorld, wristForward);
        if (weaponSolverLength(correctionAxis) >
            gunstock_alignment_policy::kMinimumDirectionLength) {
            correctionAxis = weaponSolverNormalize(correctionAxis);
            snapshot.correctionAxisValid = true;
        } else if (clampedDot < -0.9999f) {
            correctionAxis = weaponSolverOrthogonalAxis(neutralFireWorld);
            snapshot.correctionAxisValid = true;
            snapshot.correctionUsedAntiparallelFallback = true;
        }

        const float neutralResidualDot = (std::max)(
            -1.0f,
            (std::min)(
                1.0f,
                weaponSolverDot(
                    predictedNeutral,
                    fineTunedTarget)));
        const float fineTuneTargetOffsetDot = (std::max)(
            -1.0f,
            (std::min)(
                1.0f,
                weaponSolverDot(
                    fineTunedTarget,
                    wristForward)));
        snapshot.correctionWorld = correction;
        snapshot.predictedWeaponWorld = predictedWeapon;
        snapshot.predictedFireNodeWorld = predictedFire;
        snapshot.wristForwardWorld = wristForward;
        snapshot.fineTunedTargetForwardWorld = fineTunedTarget;
        snapshot.weaponRootForwardWorld = weaponRootForward;
        snapshot.unalignedLiveFireWorld = unalignedLiveFire;
        snapshot.neutralFireWorldBefore = neutralFireWorld;
        snapshot.predictedNeutralFireWorld = predictedNeutral;
        snapshot.correctionAxisWorld = correctionAxis;
        snapshot.unalignedAngleDegrees =
            correctionAngle * two_handed_grip_detail::kRadiansToDegrees;
        snapshot.correctionAngleRadians = correctionAngle;
        snapshot.correctionAngleDegrees =
            correctionAngle * two_handed_grip_detail::kRadiansToDegrees;
        snapshot.fineTuneTargetOffsetDegrees =
            std::acos(fineTuneTargetOffsetDot) *
                two_handed_grip_detail::kRadiansToDegrees;
        snapshot.neutralResidualDegrees =
            std::acos(neutralResidualDot) *
                two_handed_grip_detail::kRadiansToDegrees;
        snapshot.correctionValid = true;
        snapshot.predictionValid = true;
        return true;
    }


    void TwoHandedGrip::prepareGunstockAlignmentDebugSnapshot(
        RE::NiNode* weaponNode,
        RE::NiAVObject* projectileNode,
        const std::uint32_t currentWeaponFormID,
        const std::uint64_t currentWeaponGenerationKey,
        const bool calibrationSampleBlocked,
        const bool authorityBlocked)
    {
        if (!g_rockConfig.rockDebugDrawGunstockAlignment) {
            _gunstockAlignmentDebugSnapshot = {};
            return;
        }

        ++_gunstockAlignmentDebugSequence;
        if (_gunstockAlignmentDebugSequence == 0) {
            ++_gunstockAlignmentDebugSequence;
        }

        auto& snapshot = _gunstockAlignmentDebugSnapshot;
        snapshot = {};
        snapshot.publicationSequence = _gunstockAlignmentDebugSequence;
        snapshot.weaponGenerationKey = currentWeaponGenerationKey;
        snapshot.canonicalCaptureSequence =
            _rightFiringHandCanonicalCaptureSequence;
        snapshot.weaponNodeIdentity =
            reinterpret_cast<std::uintptr_t>(weaponNode);
        snapshot.fireNodeIdentity =
            reinterpret_cast<std::uintptr_t>(projectileNode);
        snapshot.weaponFormID = currentWeaponFormID;
        snapshot.behaviorEnabled =
            g_rockConfig.rockGunstockModeEnabled;
        snapshot.weaponEligible = isGunstockWeaponEligible(
            weaponNode,
            currentWeaponGenerationKey);
        snapshot.firingHandIsLeft = _firingHandIsLeft;
        snapshot.gripState = _state;

        if (!snapshot.weaponEligible) {
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::WeaponNotEligible;
            return;
        }
        if (!weaponNode ||
            !projectileNode ||
            currentWeaponGenerationKey == 0 ||
            !isFiniteTransform(weaponNode->world) ||
            !isFiniteTransform(projectileNode->world)) {
            snapshot.yieldReason = GunstockAlignmentDebugYieldReason::
                WeaponOrFireNodeUnavailable;
            return;
        }
        if (!runtime_state::isLocalSkeletonReady()) {
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::SkeletonUnavailable;
            return;
        }
        if (!f4vr::isNodeVisible(weaponNode)) {
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::WeaponHidden;
            return;
        }

        snapshot.weaponWorldBefore = weaponNode->world;
        snapshot.fireNodeWorldBefore = projectileNode->world;
        snapshot.weaponBeforeValid = true;
        snapshot.fireNodeBeforeValid = true;

        if (frik_visual_authority::isAvailable()) {
            RE::NiTransform firingHandWorld{};
            RE::NiTransform firingDriverWorld{};
            if (tryResolveGunstockPhysicalFiringFrame(
                    firingHandWorld,
                    firingDriverWorld)) {
                snapshot.dampedDriverWorld = firingDriverWorld;
                snapshot.firingHandWorld = firingHandWorld;
                snapshot.dampedDriverValid = true;
                snapshot.firingHandValid = true;
                snapshot.pivotWorld = firingDriverWorld.translate;

                if (_firingHandIsLeft) {
                    snapshot.leftHandWorld = firingHandWorld;
                    snapshot.leftHandValid = true;
                }
            }

            if (!_firingHandIsLeft) {
                RE::NiTransform leftHandWorld{};
                if (tryGetRootFlattenedHandBoneTransform(
                        true,
                        leftHandWorld)) {
                    snapshot.leftHandWorld = leftHandWorld;
                    snapshot.leftHandValid = true;
                }
            }
        }

        if (!snapshot.dampedDriverValid ||
            !snapshot.firingHandValid) {
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::
                    DampedDriverUnavailable;
            return;
        }

        RE::NiPoint3 previewNeutralHandLocal{};
        const bool runtimeIdentityMatches =
            snapshot.behaviorEnabled &&
            _gunstockAlignment.weaponNodeIdentity == weaponNode &&
            _gunstockAlignment.weaponGenerationKey ==
                currentWeaponGenerationKey &&
            _gunstockAlignment.firingHandIsLeft == _firingHandIsLeft;
        if (runtimeIdentityMatches &&
            _gunstockAlignment.directionLatch.latched) {
            previewNeutralHandLocal =
                _gunstockAlignment.directionLatch.neutralHandLocal;
        } else if (!gunstock_alignment_policy::
                       tryCaptureHandLocalBore(
                           snapshot.firingHandWorld,
                           snapshot.fireNodeWorldBefore,
                           previewNeutralHandLocal)) {
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::FrikUnavailable;
            return;
        }
        (void)populateGunstockAlignmentDebugPrediction(
            previewNeutralHandLocal);

        if (!snapshot.behaviorEnabled) {
            snapshot.state =
                GunstockAlignmentDebugState::DisabledPreview;
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::AlignmentDisabled;
            return;
        }
        if (!frik_visual_authority::isAvailable()) {
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::FrikUnavailable;
            return;
        }

        const bool neutralSampleBlocked =
            calibrationSampleBlocked ||
            _scopeMenuOpenThisFrame ||
            _scopeMenuClosedThisFrame;
        snapshot.yieldReason =
            currentGunstockAlignmentYieldReason(authorityBlocked);
        if (snapshot.yieldReason !=
            GunstockAlignmentDebugYieldReason::None) {
            snapshot.state = GunstockAlignmentDebugState::Yielded;
            return;
        }
        if (!runtimeIdentityMatches ||
            !_gunstockAlignment.directionLatch.latched) {
            snapshot.state = GunstockAlignmentDebugState::Waiting;
            snapshot.yieldReason = neutralSampleBlocked ?
                GunstockAlignmentDebugYieldReason::NeutralSampleBlocked :
                GunstockAlignmentDebugYieldReason::NeutralSampleCollecting;
            return;
        }
        snapshot.state = GunstockAlignmentDebugState::Latched;
    }


    void TwoHandedGrip::finalizeGunstockAlignmentDebugSnapshot(
        RE::NiNode* weaponNode,
        RE::NiAVObject* projectileNode,
        const std::uint64_t currentWeaponGenerationKey)
    {
        if (!g_rockConfig.rockDebugDrawGunstockAlignment) {
            _gunstockAlignmentDebugSnapshot = {};
            return;
        }

        auto& snapshot = _gunstockAlignmentDebugSnapshot;
        if (snapshot.publicationSequence == 0 ||
            !snapshot.dampedDriverValid ||
            !snapshot.weaponBeforeValid ||
            !snapshot.fireNodeBeforeValid ||
            !weaponNode ||
            !projectileNode ||
            snapshot.weaponNodeIdentity !=
                reinterpret_cast<std::uintptr_t>(weaponNode) ||
            snapshot.fireNodeIdentity !=
                reinterpret_cast<std::uintptr_t>(projectileNode) ||
            snapshot.weaponGenerationKey != currentWeaponGenerationKey ||
            !runtime_state::isLocalSkeletonReady() ||
            !f4vr::isNodeVisible(weaponNode) ||
            !isFiniteTransform(weaponNode->world) ||
            !isFiniteTransform(projectileNode->world)) {
            snapshot = {};
            return;
        }

        const bool runtimeIdentityMatches =
            _gunstockAlignment.weaponNodeIdentity == weaponNode &&
            _gunstockAlignment.weaponGenerationKey ==
                currentWeaponGenerationKey &&
            _gunstockAlignment.firingHandIsLeft == _firingHandIsLeft;
        if (snapshot.behaviorEnabled && runtimeIdentityMatches) {
            snapshot.candidateSamples =
                _gunstockAlignment.directionLatch.candidateSamples;
            RE::NiPoint3 runtimeNeutralHandLocal{};
            if (_gunstockAlignment.directionLatch.latched) {
                runtimeNeutralHandLocal =
                    _gunstockAlignment.directionLatch.
                        neutralHandLocal;
                (void)populateGunstockAlignmentDebugPrediction(
                    runtimeNeutralHandLocal);
            } else if (_gunstockAlignment.directionLatch.candidateSamples !=
                           0 &&
                       gunstock_alignment_policy::tryNormalizeDirection(
                           _gunstockAlignment.directionLatch.candidateSum,
                           runtimeNeutralHandLocal)) {
                (void)populateGunstockAlignmentDebugPrediction(
                    runtimeNeutralHandLocal);
            }
        }

        snapshot.finalWeaponWorld = weaponNode->world;
        snapshot.finalFireNodeWorld = projectileNode->world;
        snapshot.finalWeaponValid = true;
        snapshot.finalFireNodeValid = true;
        if (!snapshot.renderedFiringHandValid) {
            RE::NiTransform renderedFiringHandWorld{};
            if (tryGetRootFlattenedHandBoneTransform(
                    _firingHandIsLeft,
                    renderedFiringHandWorld)) {
                snapshot.renderedFiringHandWorld =
                    renderedFiringHandWorld;
                snapshot.renderedFiringHandValid = true;
            }
        }
        snapshot.alignmentAppliedThisFrame =
            _gunstockAlignmentAppliedThisFrame;

        const RE::NiPoint3 localForward{ 0.0f, 1.0f, 0.0f };
        RE::NiPoint3 finalLiveFire{};
        if (!gunstock_alignment_policy::tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    snapshot.finalFireNodeWorld,
                    localForward),
                finalLiveFire)) {
            snapshot = {};
            return;
        }
        snapshot.finalLiveFireWorld = finalLiveFire;
        const float liveDot = (std::max)(
            -1.0f,
            (std::min)(
                1.0f,
                weaponSolverDot(
                    snapshot.finalLiveFireWorld,
                    snapshot.predictionValid ?
                        snapshot.fineTunedTargetForwardWorld :
                        snapshot.wristForwardWorld)));
        snapshot.liveDeviationDegrees =
            std::acos(liveDot) *
                two_handed_grip_detail::kRadiansToDegrees;
        if (snapshot.predictionValid) {
            snapshot.finalWeaponPredictionErrorGameUnits =
                hand_visual_lerp_math::distanceGameUnits(
                    snapshot.finalWeaponWorld.translate,
                    snapshot.predictedWeaponWorld.translate);
        }

        if (!snapshot.behaviorEnabled) {
            snapshot.state =
                GunstockAlignmentDebugState::DisabledPreview;
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::AlignmentDisabled;
        } else if (snapshot.yieldReason ==
                       GunstockAlignmentDebugYieldReason::
                           FrikUnavailable ||
                   snapshot.yieldReason ==
                       GunstockAlignmentDebugYieldReason::
                           SkeletonUnavailable) {
            snapshot.state = GunstockAlignmentDebugState::Invalid;
        } else if (snapshot.yieldReason !=
                       GunstockAlignmentDebugYieldReason::None &&
                   snapshot.yieldReason !=
                       GunstockAlignmentDebugYieldReason::
                           NeutralSampleBlocked &&
                   snapshot.yieldReason !=
                       GunstockAlignmentDebugYieldReason::
                           NeutralSampleCollecting) {
            snapshot.state = GunstockAlignmentDebugState::Yielded;
        } else if (!runtimeIdentityMatches ||
                   !_gunstockAlignment.directionLatch.latched) {
            snapshot.state = GunstockAlignmentDebugState::Waiting;
            if (snapshot.yieldReason !=
                GunstockAlignmentDebugYieldReason::NeutralSampleBlocked) {
                snapshot.yieldReason =
                    GunstockAlignmentDebugYieldReason::
                        NeutralSampleCollecting;
            }
        } else if (_gunstockAlignmentAppliedThisFrame) {
            snapshot.state = GunstockAlignmentDebugState::Active;
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::None;
            if (snapshot.recoilWitness ==
                GunstockAlignmentDebugRecoilWitness::None) {
                snapshot.recoilWitness =
                    GunstockAlignmentDebugRecoilWitness::Regular;
            }
        } else {
            snapshot.state = GunstockAlignmentDebugState::Latched;
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::
                    RuntimeAuthorityUnavailable;
        }
        snapshot.published = true;
    }


    bool TwoHandedGrip::applyGunstockAlignment(
        RE::NiNode* weaponNode,
        RE::NiAVObject* projectileNode,
        const std::uint64_t currentWeaponGenerationKey,
        const bool calibrationSampleBlocked,
        const bool authorityBlocked)
    {
        _gunstockHandAuthorityActive = {};
        _gunstockAlignmentAppliedThisFrame = false;
        _gunstockAlignmentBlockedThisFrame = authorityBlocked;

        if (!g_rockConfig.rockGunstockModeEnabled) {
            resetGunstockAlignment("disabled");
            return false;
        }

        if (!isGunstockWeaponEligible(
                weaponNode,
                currentWeaponGenerationKey)) {
            resetGunstockAlignment("weapon-not-eligible");
            return false;
        }

        if (!weaponNode ||
            currentWeaponGenerationKey == 0 ||
            !runtime_state::isLocalSkeletonReady() ||
            !frik_visual_authority::isAvailable() ||
            !f4vr::isNodeVisible(weaponNode) ||
            !isFiniteTransform(weaponNode->world)) {
            resetGunstockAlignment("weapon-or-skeleton-unavailable");
            return false;
        }

        const bool identityMatches =
            _gunstockAlignment.weaponNodeIdentity == weaponNode &&
            _gunstockAlignment.weaponGenerationKey ==
                currentWeaponGenerationKey &&
            _gunstockAlignment.firingHandIsLeft == _firingHandIsLeft;
        if (!identityMatches) {
            resetGunstockAlignment("weapon-hand-identity-changed");
            _gunstockAlignment.weaponNodeIdentity = weaponNode;
            _gunstockAlignment.weaponGenerationKey =
                currentWeaponGenerationKey;
            _gunstockAlignment.firingHandIsLeft = _firingHandIsLeft;
        }

        const bool projectileWitnessUsable =
            projectileNode && isFiniteTransform(projectileNode->world);
        const bool neutralSampleBlocked =
            calibrationSampleBlocked ||
            _scopeMenuOpenThisFrame ||
            _scopeMenuClosedThisFrame;
        if (!neutralSampleBlocked) {
            _gunstockLatchedContinuityLogged = false;
        }

        const bool supportHandIsLeft = !_firingHandIsLeft;
        const bool supportHandParticipates =
            _state == TwoHandedState::Gripping &&
            partGrip(supportHandIsLeft).active;
        const auto yieldReason =
            currentGunstockAlignmentYieldReason(authorityBlocked);
        if (yieldReason != GunstockAlignmentDebugYieldReason::None) {
            clearGunstockDedicatedHandAuthority();
            if (!_gunstockYieldLogged) {
                _gunstockYieldLogged = true;
                ROCK_LOG_DEBUG(
                    Weapon,
                    "TwoHandedGrip: gunstock alignment yielded state={} scope={} authorityBlocked={} weaponReturn={} firingHand={}",
                    static_cast<int>(_state),
                    _scopeMenuOpenThisFrame ? "open" :
                        (_scopeMenuClosedThisFrame ? "closing" : "closed"),
                    authorityBlocked ? "yes" : "no",
                    isWeaponVisualReturnActive() ? "yes" : "no",
                    _firingHandIsLeft ? "left" : "right");
            }
            return false;
        }
        _gunstockYieldLogged = false;

        RE::NiTransform alignmentHandWorld{};
        RE::NiTransform firingDriverWorld{};
        if (!tryResolveGunstockPhysicalFiringFrame(
                alignmentHandWorld,
                firingDriverWorld)) {
            clearGunstockDedicatedHandAuthority();
            return false;
        }

        auto& directionLatch = _gunstockAlignment.directionLatch;
        bool latchedThisFrame = false;
        if (!directionLatch.latched) {
            if (!projectileWitnessUsable) {
                gunstock_alignment_policy::resetCandidate(directionLatch);
                if (!_gunstockProjectileWitnessMissingLogged) {
                    _gunstockProjectileWitnessMissingLogged = true;
                    ROCK_LOG_DEBUG(
                        Weapon,
                        "TwoHandedGrip: gunstock projectile witness unavailable before neutral latch; waiting without changing weapon identity");
                }
                return false;
            }
            _gunstockProjectileWitnessMissingLogged = false;
            if (neutralSampleBlocked) {
                gunstock_alignment_policy::resetCandidate(directionLatch);
                if (!_gunstockWaitingLogged) {
                    _gunstockWaitingLogged = true;
                    ROCK_LOG_DEBUG(
                        Weapon,
                        "TwoHandedGrip: gunstock alignment waiting for neutral firing-hand samples");
                }
                return false;
            }

            RE::NiPoint3 handLocalBore{};
            if (!gunstock_alignment_policy::tryCaptureHandLocalBore(
                    alignmentHandWorld,
                    projectileNode->world,
                    handLocalBore)) {
                gunstock_alignment_policy::resetCandidate(directionLatch);
                return false;
            }

            const bool latched =
                gunstock_alignment_policy::observeStableDirection(
                    directionLatch,
                    handLocalBore);
            if (!latched) {
                if (!_gunstockWaitingLogged) {
                    _gunstockWaitingLogged = true;
                    ROCK_LOG_DEBUG(
                        Weapon,
                        "TwoHandedGrip: gunstock alignment collecting neutral firing-hand samples required={}",
                        gunstock_alignment_policy::kRequiredStableSamples);
                }
                return false;
            }

            _gunstockWaitingLogged = false;
            latchedThisFrame = true;
        } else if (!projectileWitnessUsable) {
            if (!_gunstockProjectileWitnessMissingLogged) {
                _gunstockProjectileWitnessMissingLogged = true;
                ROCK_LOG_DEBUG(
                    Weapon,
                    "TwoHandedGrip: gunstock projectile witness unavailable during firing animation; retaining and publishing the latched correction");
            }
        } else {
            _gunstockProjectileWitnessMissingLogged = false;
        }

        if (neutralSampleBlocked &&
            !_gunstockLatchedContinuityLogged) {
            _gunstockLatchedContinuityLogged = true;
            ROCK_LOG_DEBUG(
                Weapon,
                "TwoHandedGrip: gunstock retaining latched presentation while neutral sampling is blocked state={} scope={} firingHand={}",
                static_cast<int>(_state),
                _scopeMenuOpenThisFrame ? "open" :
                    (_scopeMenuClosedThisFrame ? "closing" : "closed"),
                _firingHandIsLeft ? "left" : "right");
        }

        RE::NiMatrix3 correction{};
        RE::NiPoint3 pivotWorld{};
        float directionDot = 1.0f;
        bool fineTuneActive = false;
        if (!tryResolveGunstockPrimaryGroupCorrection(
                weaponNode,
                currentWeaponGenerationKey,
                alignmentHandWorld,
                correction,
                pivotWorld,
                &directionDot,
                &fineTuneActive)) {
            resetGunstockAlignment("correction-invalid");
            return false;
        }

        if (latchedThisFrame) {
            ROCK_LOG_INFO(
                Weapon,
                "TwoHandedGrip: gunstock neutral bore latched firingHand={} generation={:016X} samples={} handLocal=({:.4f},{:.4f},{:.4f}) target=wrist+X correctionDegrees={:.2f}",
                _firingHandIsLeft ? "left" : "right",
                currentWeaponGenerationKey,
                directionLatch.candidateSamples,
                directionLatch.neutralHandLocal.x,
                directionLatch.neutralHandLocal.y,
                directionLatch.neutralHandLocal.z,
                std::acos((std::max)(
                    -1.0f,
                    (std::min)(1.0f, directionDot))) *
                    two_handed_grip_detail::kRadiansToDegrees);
        }

        const auto rolePublishedThisFrame = [this](
                                                      const bool isLeft,
                                                      const scope_safe_hand_frame_math::HandAuthorityRole role) {
            return scope_safe_hand_frame_math::hasRole(
                _scopeHandAuthorityPublishedThisFrame[isLeft ? 0u : 1u],
                role);
        };

        const std::size_t firingIndex = _firingHandIsLeft ? 0u : 1u;
        const bool firingGripPublishedThisFrame =
            rolePublishedThisFrame(
                _firingHandIsLeft,
                scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip);
        RE::NiTransform firingGroupHandWorld{};
        const char* firingGroupSource = "physical-wrist";
        if (firingGripPublishedThisFrame) {
            if (!_hasLastPublishedHandWorld[firingIndex] ||
                !isUsableHandAuthorityTransform(
                    _lastPublishedHandWorld[firingIndex])) {
                clearGunstockDedicatedHandAuthority();
                return false;
            }
            firingGroupHandWorld =
                _lastPublishedHandWorld[firingIndex];
            firingGroupSource = "same-frame-primary-grip";
        } else {
            const auto& firingReturn =
                _returningHandVisuals[firingIndex].transition;
            const bool authoredRightCanonicalCurrent =
                !_firingHandIsLeft &&
                _hasRightFiringHandCanonicalWeaponLocal &&
                _rightFiringHandCanonicalWeaponNode == weaponNode &&
                _rightFiringHandCanonicalGenerationKey ==
                    currentWeaponGenerationKey &&
                isFiniteTransform(
                    _rightFiringHandCanonicalWeaponLocal);
            const bool manualFiringFrameCurrent =
                _hasFiringHandWeaponLocal &&
                _activeWeaponNode == weaponNode &&
                _activeWeaponGenerationKey ==
                    currentWeaponGenerationKey &&
                isFiniteTransform(_primaryHandWeaponLocal);
            if (firingReturn.active &&
                isUsableHandAuthorityTransform(
                    firingReturn.lastApplied)) {
                firingGroupHandWorld = firingReturn.lastApplied;
                firingGroupSource = "hand-return";
            } else if (authoredRightCanonicalCurrent) {
                firingGroupHandWorld =
                    transform_math::composeTransforms(
                        weaponNode->world,
                        _rightFiringHandCanonicalWeaponLocal);
                firingGroupSource = "authored-primary-canonical";
            } else if (manualFiringFrameCurrent) {
                firingGroupHandWorld =
                    transform_math::composeTransforms(
                        weaponNode->world,
                        _primaryHandWeaponLocal);
                firingGroupSource = "manual-primary-frame";
            } else {
                firingGroupHandWorld = alignmentHandWorld;
            }
        }
        if (!isUsableHandAuthorityTransform(firingGroupHandWorld)) {
            clearGunstockDedicatedHandAuthority();
            return false;
        }

        const RE::NiTransform weaponWorldBeforeCorrection =
            weaponNode->world;
        const auto captureRenderedFiringRelation =
            [this,
                weaponNode,
                &weaponWorldBeforeCorrection,
                &firingGroupHandWorld,
                firingGroupSource,
                projectileWitnessUsable]() {
                RE::NiTransform renderedFiringHandWorld{};
                const bool renderedFiringHandValid =
                    tryGetRootFlattenedHandBoneTransform(
                        _firingHandIsLeft,
                        renderedFiringHandWorld);

                float renderedFiringRelationPositionError = -1.0f;
                float renderedFiringRelationRotationError = -1.0f;
                bool renderedFiringRelationValid = false;
                if (renderedFiringHandValid) {
                    const RE::NiTransform firingInWeaponBefore =
                        transform_math::composeTransforms(
                            transform_math::invertTransform(
                                weaponWorldBeforeCorrection),
                            firingGroupHandWorld);
                    const RE::NiTransform renderedFiringInWeaponAfter =
                        transform_math::composeTransforms(
                            transform_math::invertTransform(
                                weaponNode->world),
                            renderedFiringHandWorld);
                    if (isFiniteTransform(firingInWeaponBefore) &&
                        isFiniteTransform(renderedFiringInWeaponAfter)) {
                        renderedFiringRelationPositionError =
                            hand_visual_lerp_math::distanceGameUnits(
                                renderedFiringInWeaponAfter.translate,
                                firingInWeaponBefore.translate);
                        renderedFiringRelationRotationError =
                            hand_visual_lerp_math::rotationDistanceDegrees(
                                firingInWeaponBefore,
                                renderedFiringInWeaponAfter);
                        renderedFiringRelationValid =
                            std::isfinite(
                                renderedFiringRelationPositionError) &&
                            std::isfinite(
                                renderedFiringRelationRotationError);
                    }
                }

                if (g_rockConfig.rockDebugDrawGunstockAlignment) {
                    auto& snapshot = _gunstockAlignmentDebugSnapshot;
                    snapshot.renderedFiringHandWorld =
                        renderedFiringHandValid ?
                        renderedFiringHandWorld :
                        RE::NiTransform{};
                    snapshot.renderedFiringHandValid =
                        renderedFiringHandValid;
                    snapshot.renderedFiringRelationValid =
                        renderedFiringRelationValid;
                    if (renderedFiringRelationValid) {
                        snapshot.
                            renderedFiringRelationPositionErrorGameUnits =
                            renderedFiringRelationPositionError;
                        snapshot.
                            renderedFiringRelationRotationErrorDegrees =
                            renderedFiringRelationRotationError;
                    }
                }
                ROCK_LOG_SAMPLE_DEBUG(
                    Weapon,
                    2000,
                    "TwoHandedGrip: gunstock group source={} pivot=damped-driver projectileWitness={} weaponReturn={} handReturn={} renderedRelationValid={} renderedFiringRelationError=({:.4f}gu,{:.3f}deg)",
                    firingGroupSource,
                    projectileWitnessUsable ? "live" : "latched",
                    isWeaponVisualReturnActive() ? "active" : "inactive",
                    isHandVisualReturnActive(_firingHandIsLeft) ?
                        "active" : "inactive",
                    renderedFiringRelationValid ? "yes" : "no",
                    renderedFiringRelationPositionError,
                    renderedFiringRelationRotationError);
            };

        if (directionDot > 0.999999f && !fineTuneActive) {
            if (!applyWeaponVisualAuthority(
                    weaponNode,
                    weaponNode->world,
                    currentWeaponGenerationKey)) {
                clearGunstockDedicatedHandAuthority();
                return false;
            }
            _gunstockFramePresentation = GunstockFramePresentationState{
                .weaponNodeIdentity = weaponNode,
                .weaponGenerationKey = currentWeaponGenerationKey,
                .runtimeFrameIndex =
                    runtime_state::currentFrame().frameIndex,
                .correctionWorld = correction,
                .pivotWorld = pivotWorld,
                .sourceWeaponWorld = weaponWorldBeforeCorrection,
                .correctedWeaponWorld = weaponNode->world,
                .firingHandIsLeft = _firingHandIsLeft,
                .valid = true,
            };
            clearGunstockDedicatedHandAuthority();
            (void)publishAuthoredPrimaryFiringGripFingerPose(
                _firingHandIsLeft);
            reframeAuthoredSupportGripDebugSnapshot(weaponNode->world);
            captureRenderedFiringRelation();
            _gunstockAlignmentAppliedThisFrame = true;
            return true;
        }

        RE::NiTransform supportHandWorld{};
        if (supportHandParticipates) {
            const std::size_t supportIndex =
                supportHandIsLeft ? 0u : 1u;
            const bool supportGripPublishedThisFrame =
                rolePublishedThisFrame(
                    supportHandIsLeft,
                    scope_safe_hand_frame_math::HandAuthorityRole::
                        SupportGrip);

            // Keep the support hand and weapon in one source frame. When the
            // grip solve published a target this frame, hFRIK's rendered hand
            // is post-IK output and may contain a reach residual. Feeding that
            // output back into the rigid correction makes the residual look
            // like the hand is sliding over the grabbed weapon part.
            if (supportGripPublishedThisFrame) {
                if (!_hasLastPublishedHandWorld[supportIndex] ||
                    !isUsableHandAuthorityTransform(
                        _lastPublishedHandWorld[supportIndex])) {
                    clearGunstockDedicatedHandAuthority();
                    return false;
                }
                supportHandWorld =
                    _lastPublishedHandWorld[supportIndex];
            } else {
                (void)tryGetRootFlattenedHandBoneTransform(
                    supportHandIsLeft,
                    supportHandWorld);
            }
            if (!isUsableHandAuthorityTransform(supportHandWorld)) {
                clearGunstockDedicatedHandAuthority();
                return false;
            }
        }

        const RE::NiTransform correctedFiringHandWorld =
            gunstock_alignment_policy::rotateRigidlyAroundPivot<
                RE::NiTransform,
                    RE::NiMatrix3,
                    RE::NiPoint3>(
                firingGroupHandWorld,
                correction,
                pivotWorld);
        const RE::NiTransform correctedWeaponWorld =
            gunstock_alignment_policy::rotateRigidlyAroundPivot<
                RE::NiTransform,
                RE::NiMatrix3,
                RE::NiPoint3>(
                weaponWorldBeforeCorrection,
                correction,
                pivotWorld);
        const RE::NiTransform correctedSupportHandWorld =
            supportHandParticipates ?
            gunstock_alignment_policy::rotateRigidlyAroundPivot<
                RE::NiTransform,
                RE::NiMatrix3,
                RE::NiPoint3>(
                supportHandWorld,
                correction,
                pivotWorld) :
            RE::NiTransform{};
        if (!isUsableHandAuthorityTransform(correctedFiringHandWorld) ||
            !isFiniteTransform(correctedWeaponWorld) ||
            (supportHandParticipates &&
                !isUsableHandAuthorityTransform(
                    correctedSupportHandWorld))) {
            resetGunstockAlignment("corrected-group-invalid");
            return false;
        }

        struct AppliedHandCorrection
        {
            const char* tag{ nullptr };
            RE::NiTransform originalWorld{};
            RE::NiTransform requestedWorld{};
            RE::NiTransform recoilWorldDelta{};
            bool isLeft{ false };
            bool reusedGripRole{ false };
            bool recoilPrecompensated{ false };
            bool applied{ false };
        };

        const auto applyHandCorrection = [this,
                                             &rolePublishedThisFrame](
                                             const bool isLeft,
                                             const bool firingRole,
                                             const RE::NiTransform& originalWorld,
                                             const RE::NiTransform& desiredWorld,
                                             AppliedHandCorrection& out) {
            out = {};
            out.isLeft = isLeft;
            out.originalWorld = originalWorld;
            const std::size_t index = isLeft ? 0u : 1u;
            const auto role = firingRole ?
                scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip :
                scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip;
            out.reusedGripRole = rolePublishedThisFrame(isLeft, role);
            out.tag = out.reusedGripRole ?
                (firingRole ? PRIMARY_GRIP_TAG : SUPPORT_GRIP_TAG) :
                GUNSTOCK_ALIGNMENT_TAG;
            const auto hand = handFromBool(isLeft);

            if (out.reusedGripRole &&
                _gunstockDedicatedHandAuthorityActive[index]) {
                (void)frik_visual_authority::clearExternalHandWorldTransform(
                    GUNSTOCK_ALIGNMENT_TAG,
                    hand);
                _gunstockDedicatedHandAuthorityActive[index] = false;
            }

            RE::NiTransform requestedWorld = desiredWorld;
            bool recoilProbeApplied = false;
            const bool leftManualFiringRecoil =
                firingRole &&
                isLeft &&
                _firingHandIsLeft &&
                isManualOwnershipActive() &&
                _weaponNodeOwnershipBlockEngaged;
            if (leftManualFiringRecoil) {
                RE::NiTransform recoilAppliedWorld{};
                RE::NiTransform recoilRequestedWorld{};
                if (out.reusedGripRole) {
                    if (!_hasLastPublishedHandWorld[index] ||
                        !isUsableHandAuthorityTransform(
                            _lastPublishedHandWorld[index])) {
                        return false;
                    }
                    recoilRequestedWorld =
                        _lastPublishedHandWorld[index];
                    // hFRIK applies controlled recoil synchronously inside
                    // the same primary-role publication. Read the presented
                    // full-body hand bone now; the requested target is not
                    // evidence of the transform that recoil actually produced.
                    if (!tryGetRootFlattenedHandBoneTransform(
                            isLeft,
                            recoilAppliedWorld)) {
                        return false;
                    }
                } else {
                    recoilRequestedWorld = originalWorld;
                    if (!frik_visual_authority::applyExternalHandWorldTransform(
                            GUNSTOCK_ALIGNMENT_TAG,
                            hand,
                            recoilRequestedWorld,
                            GRIP_HAND_POSE_PRIORITY)) {
                        return false;
                    }
                    _gunstockDedicatedHandAuthorityActive[index] = true;
                    recoilProbeApplied = true;
                    if (!tryGetRootFlattenedHandBoneTransform(
                            isLeft,
                            recoilAppliedWorld)) {
                        (void)frik_visual_authority::
                            clearExternalHandWorldTransform(
                                GUNSTOCK_ALIGNMENT_TAG,
                                hand);
                        _gunstockDedicatedHandAuthorityActive[index] = false;
                        return false;
                    }
                }

                out.recoilWorldDelta =
                    gunstock_alignment_policy::deriveAppliedWorldDelta(
                        recoilRequestedWorld,
                        recoilAppliedWorld);
                if (!isUsableHandAuthorityTransform(
                        out.recoilWorldDelta)) {
                    if (!out.reusedGripRole) {
                        (void)frik_visual_authority::
                            clearExternalHandWorldTransform(
                                GUNSTOCK_ALIGNMENT_TAG,
                                hand);
                        _gunstockDedicatedHandAuthorityActive[index] = false;
                    }
                    return false;
                }
                requestedWorld =
                    gunstock_alignment_policy::precompensateWorldTarget(
                        out.recoilWorldDelta,
                        desiredWorld);
                out.recoilPrecompensated = true;
            }

            if (!isUsableHandAuthorityTransform(requestedWorld) ||
                !frik_visual_authority::applyExternalHandWorldTransform(
                    out.tag,
                    hand,
                    requestedWorld,
                    GRIP_HAND_POSE_PRIORITY)) {
                if (!out.reusedGripRole) {
                    if (recoilProbeApplied &&
                        isUsableHandAuthorityTransform(
                            out.recoilWorldDelta)) {
                        const RE::NiTransform restoreTarget =
                            gunstock_alignment_policy::
                                precompensateWorldTarget(
                                    out.recoilWorldDelta,
                                    originalWorld);
                        if (isUsableHandAuthorityTransform(restoreTarget)) {
                            (void)frik_visual_authority::
                                applyExternalHandWorldTransform(
                                    GUNSTOCK_ALIGNMENT_TAG,
                                    hand,
                                    restoreTarget,
                                    GRIP_HAND_POSE_PRIORITY);
                        }
                    }
                    (void)frik_visual_authority::
                        clearExternalHandWorldTransform(
                            GUNSTOCK_ALIGNMENT_TAG,
                            hand);
                    _gunstockDedicatedHandAuthorityActive[index] = false;
                }
                return false;
            }
            out.requestedWorld = requestedWorld;

            if (!out.reusedGripRole) {
                _gunstockDedicatedHandAuthorityActive[index] = true;
            }

            recordPreFrikRetainedHandAuthority(
                out.reusedGripRole ?
                    (firingRole ?
                        RetainedHandAuthorityKind::PrimaryGrip :
                        RetainedHandAuthorityKind::SupportGrip) :
                    RetainedHandAuthorityKind::GunstockAlignment,
                isLeft,
                requestedWorld);

            if (out.reusedGripRole) {
                // Keep the role's logical target coherent with the weapon.
                // The bridge accepts this request now, and FRIK consumes it on
                // the next skeleton update. Its recoil and hand-pose stages
                // can then move the presented wrist away from the requested
                // root. Publication success, rather than exact root-tree
                // equality, is the authority contract here.
                recordPublishedHandWorld(isLeft, desiredWorld);
            }
            _gunstockHandAuthorityActive[index] = true;
            out.applied = true;
            return true;
        };

        const auto restoreHandCorrection = [this](
                                                AppliedHandCorrection& applied) {
            if (!applied.applied || !applied.tag) {
                return;
            }
            RE::NiTransform restoreTarget = applied.originalWorld;
            if (applied.recoilPrecompensated) {
                restoreTarget =
                    gunstock_alignment_policy::precompensateWorldTarget(
                        applied.recoilWorldDelta,
                        applied.originalWorld);
            }
            const bool restored =
                frik_visual_authority::applyExternalHandWorldTransform(
                applied.tag,
                handFromBool(applied.isLeft),
                restoreTarget,
                GRIP_HAND_POSE_PRIORITY);
            if (restored && applied.reusedGripRole) {
                recordPreFrikRetainedHandAuthority(
                    applied.isLeft == _firingHandIsLeft ?
                        RetainedHandAuthorityKind::PrimaryGrip :
                        RetainedHandAuthorityKind::SupportGrip,
                    applied.isLeft,
                    restoreTarget);
            }
            const std::size_t index = applied.isLeft ? 0u : 1u;
            if (applied.reusedGripRole) {
                recordPublishedHandWorld(
                    applied.isLeft,
                    applied.originalWorld);
            } else {
                (void)frik_visual_authority::clearExternalHandWorldTransform(
                    GUNSTOCK_ALIGNMENT_TAG,
                    handFromBool(applied.isLeft));
                _gunstockDedicatedHandAuthorityActive[index] = false;
            }
            _gunstockHandAuthorityActive[index] = false;
            applied.applied = false;
        };

        AppliedHandCorrection firingCorrection{};
        if (!applyHandCorrection(
                _firingHandIsLeft,
                true,
                firingGroupHandWorld,
                correctedFiringHandWorld,
                firingCorrection)) {
            clearGunstockDedicatedHandAuthority();
            return false;
        }

        AppliedHandCorrection supportCorrection{};
        if (supportHandParticipates &&
            !applyHandCorrection(
                supportHandIsLeft,
                false,
                supportHandWorld,
                correctedSupportHandWorld,
                supportCorrection)) {
            restoreHandCorrection(firingCorrection);
            clearGunstockDedicatedHandAuthority();
            return false;
        }

        if (!applyWeaponVisualAuthority(
                weaponNode,
                correctedWeaponWorld,
                currentWeaponGenerationKey)) {
            restoreHandCorrection(supportCorrection);
            restoreHandCorrection(firingCorrection);
            clearGunstockDedicatedHandAuthority();
            return false;
        }
        _gunstockFramePresentation = GunstockFramePresentationState{
            .weaponNodeIdentity = weaponNode,
            .weaponGenerationKey = currentWeaponGenerationKey,
            .runtimeFrameIndex =
                runtime_state::currentFrame().frameIndex,
            .correctionWorld = correction,
            .pivotWorld = pivotWorld,
            .sourceWeaponWorld = weaponWorldBeforeCorrection,
            .correctedWeaponWorld = weaponNode->world,
            .firingHandIsLeft = _firingHandIsLeft,
            .valid = true,
        };

        if (g_rockConfig.rockDebugDrawGunstockAlignment) {
            _gunstockAlignmentDebugSnapshot.recoilWitness =
                firingCorrection.recoilPrecompensated ?
                GunstockAlignmentDebugRecoilWitness::Controlled :
                GunstockAlignmentDebugRecoilWitness::Regular;
        }

        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;
        (void)publishAuthoredPrimaryFiringGripFingerPose(
            _firingHandIsLeft);
        reframeAuthoredSupportGripDebugSnapshot(weaponNode->world);

        captureRenderedFiringRelation();
        _gunstockAlignmentAppliedThisFrame = true;
        return true;
    }


    bool TwoHandedGrip::finalizeGunstockPresentationAfterNativeWeaponAnimation(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey)
    {
        auto& frameState = _gunstockFramePresentation;
        const bool nativeWeaponAnimationActive =
            (provider::currentNativeAnimationAuthorityFlagsV1() &
                authored_weapon_grip_capture_policy::kWeapon) != 0;
        if (!g_rockConfig.rockGunstockModeEnabled ||
            !nativeWeaponAnimationActive ||
            !runtime_state::isLocalSkeletonReady() ||
            !frik_visual_authority::isAvailable() ||
            !weaponNode ||
            currentWeaponGenerationKey == 0 ||
            !f4vr::isNodeVisible(weaponNode) ||
            !isFiniteTransform(weaponNode->world) ||
            !frameState.valid ||
            frameState.finalizedAfterNativeAnimation ||
            frameState.runtimeFrameIndex !=
                runtime_state::currentFrame().frameIndex ||
            frameState.weaponNodeIdentity != weaponNode ||
            frameState.weaponGenerationKey !=
                currentWeaponGenerationKey ||
            frameState.firingHandIsLeft != _firingHandIsLeft ||
            !_gunstockAlignment.directionLatch.latched) {
            return false;
        }

        /*
         * Some animation providers preserve ROCK's final weapon themselves.
         * Treat that as an already-complete transaction: applying the cached
         * correction a second time would rotate both weapon and hands twice.
         * Republish the retained scope frame because the animation may still
         * have restored its camera hierarchy.
         */
        if (areTransformsNearlyEqual(
                weaponNode->world,
                frameState.correctedWeaponWorld)) {
            if (!applyWeaponVisualAuthority(
                    weaponNode,
                    frameState.correctedWeaponWorld,
                    currentWeaponGenerationKey)) {
                return false;
            }
            frameState.finalizedAfterNativeAnimation = true;
            return true;
        }

        /*
         * PAPER has already authored the complete arm/hand/finger pose into
         * both its source tree and hFRIK's visible root flattened tree.
         * Calling FRIK's external-hand API here would restore default arm
         * locals, rerun IK, and republish the ordinary hand-pose stack over
         * that animation. Move only each visible Hand root in parent space:
         * the wrist absorbs the gunstock correction, while every authored
         * finger local and the rest of the arm clip stay intact. The matching
         * flattened `world` entries are rotated as a frame-local presentation
         * overlay; their authored graph locals remain untouched for the next
         * native/hFRIK frame.
         */
        using BoneTree = f4vr::BSFlattenedBoneTree;
        constexpr int kMaxPostAnimationFlattenedTransforms = 768;
        const auto validBoneTree = [](const BoneTree* tree) {
            return tree && tree->transforms && tree->numTransforms > 0 &&
                tree->numTransforms <=
                    kMaxPostAnimationFlattenedTransforms;
        };
        const auto transformNameEquals = [](const BoneTree::BoneTransforms& transform,
                                             const char* name) {
            const char* transformName = transform.name.c_str();
            return transformName && name &&
                _stricmp(transformName, name) == 0;
        };
        const auto findTransformIndex = [&transformNameEquals](
                                            const BoneTree& tree,
                                            const char* name) {
            for (int index = 0; index < tree.numTransforms; ++index) {
                const auto& transform = tree.transforms[index];
                if (transform.refNode &&
                    transformNameEquals(transform, name)) {
                    return index;
                }
            }
            return -1;
        };
        const auto belongsToRigidGroup = [&transformNameEquals](
                                              const BoneTree& tree,
                                              int index,
                                              const int leftHandIndex,
                                              const int rightHandIndex,
                                              const RE::NiNode* currentWeapon) {
            for (int depth = 0;
                 index >= 0 && index < tree.numTransforms &&
                 depth < tree.numTransforms;
                 ++depth) {
                const auto& transform = tree.transforms[index];
                if (index == leftHandIndex || index == rightHandIndex ||
                    transform.refNode == currentWeapon ||
                    transformNameEquals(transform, "Weapon") ||
                    transformNameEquals(transform, "WeaponLeft")) {
                    return true;
                }
                const int parentIndex = transform.parPos;
                if (parentIndex == index) {
                    break;
                }
                index = parentIndex;
            }
            return false;
        };
        const auto validateFlattenedOverlay = [&belongsToRigidGroup](
                                                   const BoneTree& tree,
                                                   const int leftHandIndex,
                                                   const int rightHandIndex,
                                                   const RE::NiNode* currentWeapon) {
            bool found = false;
            for (int index = 0; index < tree.numTransforms; ++index) {
                if (!belongsToRigidGroup(
                        tree,
                        index,
                        leftHandIndex,
                        rightHandIndex,
                        currentWeapon)) {
                    continue;
                }
                found = true;
                if (!isFiniteTransform(tree.transforms[index].world)) {
                    return false;
                }
            }
            return found;
        };
        const auto applyFlattenedOverlay = [&belongsToRigidGroup,
                                               &frameState](
                                               BoneTree& tree,
                                               const int leftHandIndex,
                                               const int rightHandIndex,
                                               const RE::NiNode* currentWeapon) {
            for (int index = 0; index < tree.numTransforms; ++index) {
                auto& transform = tree.transforms[index];
                if (!belongsToRigidGroup(
                        tree,
                        index,
                        leftHandIndex,
                        rightHandIndex,
                        currentWeapon)) {
                    continue;
                }
                transform.world =
                    gunstock_alignment_policy::rotateRigidlyAroundPivot<
                        RE::NiTransform,
                        RE::NiMatrix3,
                        RE::NiPoint3>(
                        transform.world,
                        frameState.correctionWorld,
                        frameState.pivotWorld);
            }
        };

        BoneTree* visibleBoneTree = f4vr::getFlattenedBoneTree();
        if (!validBoneTree(visibleBoneTree)) {
            return false;
        }
        const int leftHandTransformIndex =
            findTransformIndex(*visibleBoneTree, "LArm_Hand");
        const int rightHandTransformIndex =
            findTransformIndex(*visibleBoneTree, "RArm_Hand");
        if (leftHandTransformIndex < 0 || rightHandTransformIndex < 0 ||
            !validateFlattenedOverlay(
                *visibleBoneTree,
                leftHandTransformIndex,
                rightHandTransformIndex,
                weaponNode)) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                2000,
                "TwoHandedGrip: gunstock post-animation transaction skipped because the visible flattened hand tree was unavailable");
            return false;
        }

        BoneTree* firstPersonBoneTree = f4vr::getFirstPersonBoneTree();
        const bool firstPersonWeaponOverlayReady =
            firstPersonBoneTree &&
            firstPersonBoneTree != visibleBoneTree &&
            validBoneTree(firstPersonBoneTree) &&
            validateFlattenedOverlay(
                *firstPersonBoneTree,
                -1,
                -1,
                weaponNode);

        struct PostAnimationHandCorrection
        {
            RE::NiNode* node{ nullptr };
            RE::NiTransform originalLocal{};
            RE::NiTransform originalWorld{};
            RE::NiTransform correctedWorld{};
        };

        const auto prepareHand = [&frameState, visibleBoneTree](
                                     const int transformIndex,
                                     PostAnimationHandCorrection& out) {
            out = {};
            if (transformIndex < 0 ||
                transformIndex >= visibleBoneTree->numTransforms) {
                return false;
            }
            out.node =
                visibleBoneTree->transforms[transformIndex].refNode;
            if (!out.node || !out.node->parent ||
                !isFiniteTransform(out.node->local) ||
                !isFiniteTransform(out.node->world) ||
                !isFiniteTransform(out.node->parent->world)) {
                return false;
            }
            out.originalLocal = out.node->local;
            out.originalWorld = out.node->world;
            if (!isUsableHandAuthorityTransform(out.originalWorld)) {
                return false;
            }
            out.correctedWorld =
                gunstock_alignment_policy::rotateRigidlyAroundPivot<
                    RE::NiTransform,
                    RE::NiMatrix3,
                    RE::NiPoint3>(
                    out.originalWorld,
                    frameState.correctionWorld,
                    frameState.pivotWorld);
            if (!isUsableHandAuthorityTransform(out.correctedWorld)) {
                return false;
            }
            return true;
        };

        PostAnimationHandCorrection leftHand{};
        PostAnimationHandCorrection rightHand{};
        if (!prepareHand(leftHandTransformIndex, leftHand) ||
            !prepareHand(rightHandTransformIndex, rightHand) ||
            leftHand.node == rightHand.node) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                2000,
                "TwoHandedGrip: gunstock post-animation transaction skipped because a visible authored reload hand root was unavailable");
            return false;
        }

        const RE::NiTransform postAnimationWeaponWorld =
            weaponNode->world;
        const RE::NiTransform correctedPostAnimationWeaponWorld =
            gunstock_alignment_policy::rotateRigidlyAroundPivot<
                RE::NiTransform,
                RE::NiMatrix3,
                RE::NiPoint3>(
                postAnimationWeaponWorld,
                frameState.correctionWorld,
                frameState.pivotWorld);
        if (!isFiniteTransform(correctedPostAnimationWeaponWorld)) {
            return false;
        }

        const char* ignoredWeaponNodeName =
            weaponNode->name.c_str();
        const auto applyHand = [ignoredWeaponNodeName](
                                   PostAnimationHandCorrection& handState) {
            const RE::NiTransform correctedLocal =
                weapon_visual_authority_math::worldTargetToParentLocal(
                    handState.node->parent->world,
                    handState.correctedWorld);
            if (!isFiniteTransform(correctedLocal)) {
                return false;
            }
            handState.node->local = correctedLocal;
            f4vr::updateTransformsDown(
                handState.node,
                true,
                ignoredWeaponNodeName);
            return areTransformsNearlyEqual(
                handState.node->world,
                handState.correctedWorld,
                0.01f);
        };

        const auto rollbackHand = [ignoredWeaponNodeName](
                                      const PostAnimationHandCorrection& handState) {
            if (!handState.node) {
                return;
            }
            handState.node->local = handState.originalLocal;
            f4vr::updateTransformsDown(
                handState.node,
                true,
                ignoredWeaponNodeName);
        };

        if (!applyHand(leftHand) || !applyHand(rightHand)) {
            rollbackHand(rightHand);
            rollbackHand(leftHand);
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                2000,
                "TwoHandedGrip: gunstock post-animation direct hand reframe failed; weapon remained under native animation authority");
            return false;
        }

        if (!applyWeaponVisualAuthority(
                weaponNode,
                correctedPostAnimationWeaponWorld,
                currentWeaponGenerationKey)) {
            rollbackHand(rightHand);
            rollbackHand(leftHand);
            return false;
        }

        // Scene-node propagation does not update BSFlattenedBoneTree's
        // presentation cache. Rotate the complete visible hand/weapon group
        // there as well, and mirror the Weapon branch in PAPER's source tree
        // when that separate tree is available. Locals deliberately remain
        // the authored animation data.
        applyFlattenedOverlay(
            *visibleBoneTree,
            leftHandTransformIndex,
            rightHandTransformIndex,
            weaponNode);
        if (firstPersonWeaponOverlayReady) {
            applyFlattenedOverlay(
                *firstPersonBoneTree,
                -1,
                -1,
                weaponNode);
        }

        const float sourcePositionError =
            hand_visual_lerp_math::distanceGameUnits(
                postAnimationWeaponWorld.translate,
                frameState.sourceWeaponWorld.translate);
        const float sourceRotationError =
            hand_visual_lerp_math::rotationDistanceDegrees(
                frameState.sourceWeaponWorld,
                postAnimationWeaponWorld);
        const float finalPositionError =
            hand_visual_lerp_math::distanceGameUnits(
                weaponNode->world.translate,
                frameState.correctedWeaponWorld.translate);
        const float finalRotationError =
            hand_visual_lerp_math::rotationDistanceDegrees(
                frameState.correctedWeaponWorld,
                weaponNode->world);
        ROCK_LOG_SAMPLE_DEBUG(
            Weapon,
            2000,
            "TwoHandedGrip: gunstock reapplied after native Weapon animation sourceError=({:.4f}gu,{:.3f}deg) finalError=({:.4f}gu,{:.3f}deg)",
            sourcePositionError,
            sourceRotationError,
            finalPositionError,
            finalRotationError);

        frameState.correctedWeaponWorld = weaponNode->world;
        frameState.finalizedAfterNativeAnimation = true;
        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;
        reframeAuthoredSupportGripDebugSnapshot(weaponNode->world);
        _gunstockAlignmentAppliedThisFrame = true;
        return true;
    }

}
