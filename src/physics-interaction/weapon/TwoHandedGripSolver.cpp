#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/TwoHandedGripInternal.h"

#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/hand/skeleton/HandFrame.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RockConfig.h"
#include "RockUtils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>

namespace rock
{
    using two_handed_grip_detail::DYNAMIC_SUPPORT_MINIMUM_SMOOTHED_ROTATION_RADIANS;
    using two_handed_grip_detail::isFiniteTransform;
    using two_handed_grip_detail::lerpPoint;
    using two_handed_grip_detail::orthonormalizeStoredRotation;
    using two_handed_grip_detail::SUPPORT_NORMAL_TWIST_FACTOR;

    namespace
    {

        [[nodiscard]] WeaponTwoHandedSolverInput<
            RE::NiTransform,
            RE::NiPoint3>
            buildTwoHandedSolverInput(
                const RE::NiTransform& weaponWorld,
                const RE::NiPoint3& primaryGripLocal,
                const RE::NiPoint3& supportGripLocal,
                const RE::NiPoint3& primaryTargetWorld,
                const RE::NiPoint3& supportTargetWorld,
                const RE::NiPoint3& supportNormalLocal,
                const RE::NiPoint3& supportNormalTargetWorld)
        {
            WeaponTwoHandedSolverInput<
                RE::NiTransform,
                RE::NiPoint3>
                input{};
            input.weaponWorldTransform = weaponWorld;
            input.primaryGripLocal = primaryGripLocal;
            input.supportGripLocal = supportGripLocal;
            input.primaryTargetWorld = primaryTargetWorld;
            input.supportTargetWorld = supportTargetWorld;
            input.supportNormalLocal = supportNormalLocal;
            input.supportNormalTargetWorld = supportNormalTargetWorld;
            input.useSupportNormalTwist = true;
            input.supportNormalTwistFactor = SUPPORT_NORMAL_TWIST_FACTOR;
            return input;
        }

    }

    // ---- Dynamic support acquisition ----

    void TwoHandedGrip::beginDynamicSupportAcquisition(
        const bool supportHandIsLeft,
        const WeaponPartGrip& supportGrip,
        const RE::NiTransform& primaryStartWorld,
        const RE::NiTransform& supportStartWorld)
    {
        if (_dynamicSupportAcquisition.active) {
            clearDynamicSupportAcquisition(
                "replaced-by-new-dynamic-support-grip",
                true);
        }

        _dynamicSupportAcquisition = {};
        _dynamicSupportAcquisition.active = true;
        _dynamicSupportAcquisition.supportHandIsLeft = supportHandIsLeft;
        _dynamicSupportAcquisition.weaponGenerationKey =
            supportGrip.weaponGenerationKey;
        _dynamicSupportAcquisition.gripSequence = supportGrip.gripSequence;
        _dynamicSupportAcquisition.primaryStartWorld = primaryStartWorld;
        _dynamicSupportAcquisition.supportStartWorld = supportStartWorld;
    }

    void TwoHandedGrip::clearDynamicSupportAcquisition(
        const char* reason,
        const bool logCancellation)
    {
        if (!_dynamicSupportAcquisition.active) {
            _dynamicSupportAcquisition = {};
            return;
        }

        if (logCancellation) {
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: dynamic support acquisition event=cancel reason={} hand={} grip={} generation={:016X} elapsed={:.3f}s duration={:.3f}s rawAlpha={:.3f} easedAlpha={:.3f} fullCorrection={:.2f}deg appliedCorrection={:.2f}deg primaryPivotError={:.4f} supportTargetError={:.4f}",
                reason ? reason : "unknown",
                _dynamicSupportAcquisition.supportHandIsLeft ?
                    "left" :
                    "right",
                _dynamicSupportAcquisition.gripSequence,
                _dynamicSupportAcquisition.weaponGenerationKey,
                _dynamicSupportAcquisition.elapsedSeconds,
                _dynamicSupportAcquisition.durationSeconds,
                _dynamicSupportAcquisition.rawAlpha,
                _dynamicSupportAcquisition.easedAlpha,
                _dynamicSupportAcquisition.fullCorrectionRadians *
                    two_handed_grip_detail::kRadiansToDegrees,
                _dynamicSupportAcquisition.lastAppliedRotationRadians *
                    two_handed_grip_detail::kRadiansToDegrees,
                _dynamicSupportAcquisition.lastPrimaryPivotError,
                _dynamicSupportAcquisition.lastSupportTargetError);
        }
        _dynamicSupportAcquisition = {};
    }

    bool TwoHandedGrip::dynamicSupportAcquisitionMatches(
        const bool supportHandIsLeft,
        const WeaponPartGrip& supportGrip) const
    {
        return _dynamicSupportAcquisition.active &&
               _dynamicSupportAcquisition.supportHandIsLeft ==
                   supportHandIsLeft &&
               _dynamicSupportAcquisition.weaponGenerationKey != 0 &&
               _dynamicSupportAcquisition.weaponGenerationKey ==
                   _activeWeaponGenerationKey &&
               _dynamicSupportAcquisition.weaponGenerationKey ==
                   supportGrip.weaponGenerationKey &&
               _dynamicSupportAcquisition.gripSequence != 0 &&
               _dynamicSupportAcquisition.gripSequence ==
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
                _dynamicSupportAcquisition.primaryStartWorld :
                _dynamicSupportAcquisition.supportStartWorld;
        visualState.active = true;
        visualState.startWorld = startWorld;
        visualState.elapsedSeconds =
            _dynamicSupportAcquisition.elapsedSeconds;
        visualState.durationSeconds =
            _dynamicSupportAcquisition.durationSeconds;
        visualState.lastAlpha =
            _dynamicSupportAcquisition.easedAlpha;
        return hand_visual_lerp_math::interpolateTransform(
            startWorld,
            targetWorld,
            _dynamicSupportAcquisition.easedAlpha);
    }


    // ---- Full two-hand weapon solve ----

    void TwoHandedGrip::updateFullWeaponAuthorityGrip(RE::NiNode* weaponNode, float dt)
    {
        const bool supportHandIsLeft = !_firingHandIsLeft;
        const bool primaryHandIsLeft = _firingHandIsLeft;
        WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        const bool gunstockBaselineActive =
            isGunstockSupportBaselineActive(
                supportHandIsLeft,
                supportGrip);
        const bool dynamicBaselineActive =
            isDynamicSupportBaselineActive(
                supportHandIsLeft,
                supportGrip);
        const bool supportInputBaselineActive =
            gunstockBaselineActive || dynamicBaselineActive;
        if (supportGrip.supportInputBaseline.active &&
            !supportInputBaselineActive) {
            supportGrip.supportInputBaseline = {};
        }

        bool dynamicAcquisition =
            dynamicSupportAcquisitionMatches(
                supportHandIsLeft,
                supportGrip);
        if (_dynamicSupportAcquisition.active &&
            !dynamicAcquisition) {
            clearDynamicSupportAcquisition(
                "grip-or-generation-witness-changed",
                true);
        }
        if (!dynamicAcquisition) {
            _rotationBlend = (std::min)(
                1.0f,
                _rotationBlend +
                    (std::isfinite(dt) && dt > 0.0f ? dt : 0.0f) *
                        ROTATION_BLEND_SPEED);
        }

        RE::NiTransform primaryTransform{};
        RE::NiTransform supportTransform{};
        if (!tryGetSolverHandTransform(primaryHandIsLeft, primaryTransform) || !tryGetSolverHandTransform(supportHandIsLeft, supportTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing support grip because authoritative hand transforms are unavailable");
            transitionToInactive(false);
            return;
        }

        RE::NiTransform calibratedPrimaryTransform = primaryTransform;
        RE::NiTransform calibratedSupportTransform = supportTransform;
        bool inputBaselineResolved = true;
        if (dynamicBaselineActive) {
            const auto& primaryDriver =
                _currentHandDriverFrames[
                    primaryHandIsLeft ? 0u : 1u];
            const auto& supportDriver =
                _currentHandDriverFrames[
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
        } else if (gunstockBaselineActive) {
            inputBaselineResolved =
                weapon_support_acquisition_math::
                    tryResolveSupportInputTarget(
                        supportTransform,
                        supportGrip.supportInputBaseline.
                            inputToGripTargetLocal,
                        calibratedSupportTransform);
        }
        if (supportInputBaselineActive &&
            !inputBaselineResolved) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon,
                "TwoHandedGrip: clearing support grip because its captured input baseline became invalid mode={} hand={} grip={} generation={:016X} primaryDriver={} supportDriver={}",
                gunstockBaselineActive ? "gunstock" : "dynamic",
                supportHandIsLeft ? "left" : "right",
                supportGrip.gripSequence,
                supportGrip.weaponGenerationKey,
                _currentHandDriverFrames[
                    primaryHandIsLeft ? 0u : 1u]
                        .valid ?
                    "valid" :
                    "missing",
                _currentHandDriverFrames[
                    supportHandIsLeft ? 0u : 1u]
                        .valid ?
                    "valid" :
                    "missing");
            transitionToInactive(false);
            return;
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
        const RE::NiPoint3 currentPrimaryGripWorld = transform_math::localPointToWorld(weaponNode->world, _primaryGripLocal);
        const float currentGripSeparationWorld = std::sqrt(dot(sub(currentSupportWorld, currentPrimaryGripWorld), sub(currentSupportWorld, currentPrimaryGripWorld)));
        const float lockedGripSeparationWorld = supportGrip.hasSourceFrames ? currentGripSeparationWorld : _lockedGripSeparationWorld;
        const RE::NiPoint3 supportGripLocal = resolvePartGripWeaponLocal(supportGrip, weaponNode);
        const RE::NiPoint3 lockedSupportControllerTarget = makeLockedSupportGripTarget(
            primaryController,
            supportController,
            currentSupportWorld,
            lockedGripSeparationWorld,
            0.001f);

        const RE::NiPoint3 supportTargetWorld =
            dynamicAcquisition ?
                lockedSupportControllerTarget :
                lerpPoint(
                    currentSupportWorld,
                    lockedSupportControllerTarget,
                    _rotationBlend);
        const auto solverInput = buildTwoHandedSolverInput(
            weaponNode->world,
            _primaryGripLocal,
            supportGripLocal,
            primaryController,
            supportTargetWorld,
            resolvePartGripNormalWeaponLocal(supportGrip, weaponNode),
            computePalmNormalFromHandBasis(
                calibratedSupportTransform,
                supportHandIsLeft));

        GunstockSupportBaselineDebugSnapshot* gunstockDebug = nullptr;
        if (g_rockConfig.rockDebugDrawGunstockAlignment) {
            ++_gunstockSupportBaselineDebugSequence;
            if (_gunstockSupportBaselineDebugSequence == 0) {
                ++_gunstockSupportBaselineDebugSequence;
            }
            auto& snapshot = _gunstockSupportBaselineDebugSnapshot;
            snapshot = {};
            snapshot.publicationSequence =
                _gunstockSupportBaselineDebugSequence;
            snapshot.weaponGenerationKey = supportGrip.weaponGenerationKey;
            snapshot.gripSequence = supportGrip.gripSequence;
            snapshot.weaponNodeIdentity =
                reinterpret_cast<std::uintptr_t>(weaponNode);
            snapshot.gripState = _state;
            snapshot.supportInputWorld = supportTransform;
            snapshot.calibratedSupportWorld = calibratedSupportTransform;
            snapshot.supportGripHandWorld =
                resolvePartGripHandWorld(supportGrip, weaponNode);
            snapshot.weaponWorldBefore = weaponNode->world;
            snapshot.primaryTargetWorld = primaryController;
            snapshot.supportGripWorld = currentSupportWorld;
            snapshot.supportTargetWorld =
                solverInput.supportTargetWorld;
            snapshot.behaviorEnabled =
                g_rockConfig.rockGunstockModeEnabled;
            snapshot.supportHandIsLeft = supportHandIsLeft;
            snapshot.baselineActive = gunstockBaselineActive;
            snapshot.attachPublication =
                gunstockBaselineActive &&
                supportGrip.supportInputBaseline.firstPublicationPending;
            snapshot.supportInputValid =
                isFiniteTransform(snapshot.supportInputWorld);
            snapshot.calibratedSupportValid =
                isFiniteTransform(snapshot.calibratedSupportWorld);
            snapshot.supportGripHandValid =
                isFiniteTransform(snapshot.supportGripHandWorld);
            snapshot.weaponBeforeValid =
                isFiniteTransform(snapshot.weaponWorldBefore);
            gunstockDebug = &snapshot;
        }

        const auto solved = solveTwoHandedWeaponTransformFrikPivot(solverInput);
        if (!solved.solved) {
            if (dynamicAcquisition || supportInputBaselineActive) {
                clearDynamicSupportAcquisition(
                    "full-target-solve-degenerate",
                    true);
                transitionToInactive(false);
            }
            return;
        }

        RE::NiTransform appliedWeaponWorld =
            solved.weaponWorldTransform;
        if (dynamicAcquisition) {
            auto& acquisition = _dynamicSupportAcquisition;
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
                            _primaryHandWeaponLocal);
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
            _rotationBlend = acquisition.easedAlpha;

            const auto acquisitionSolve =
                weapon_support_acquisition_math::
                    applyRotationAroundPrimaryPivot<
                        RE::NiTransform,
                        RE::NiPoint3>(
                        solverInput.weaponWorldTransform,
                        solved.rotationDelta,
                        _primaryGripLocal,
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

        if (!applyWeaponVisualAuthority(weaponNode, appliedWeaponWorld)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing support grip because ROCK visual weapon authority failed");
            transitionToInactive(false);
            return;
        }

        static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
        static_assert(weapon_visual_authority_math::weaponVisualPrecedesLockedHandAuthority());
        publishGripHandPoses(supportHandIsLeft);

        const bool applyPrimaryHandAuthority = weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(_authorityMode);
        if (!applyLockedHandVisualAuthority(weaponNode, applyPrimaryHandAuthority, true, dt, &primaryTransform, &supportTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing support grip because ROCK locked hand authority failed");
            transitionToInactive(false);
            return;
        }

        if (gunstockDebug) {
            gunstockDebug->weaponWorldAfter = weaponNode->world;
            gunstockDebug->weaponAfterValid =
                isFiniteTransform(gunstockDebug->weaponWorldAfter);
            gunstockDebug->appliedRotationDegrees =
                hand_visual_lerp_math::rotationDistanceDegrees(
                    gunstockDebug->weaponWorldBefore,
                    gunstockDebug->weaponWorldAfter);
            gunstockDebug->appliedTranslationGameUnits =
                hand_visual_lerp_math::distanceGameUnits(
                    gunstockDebug->weaponWorldBefore.translate,
                    gunstockDebug->weaponWorldAfter.translate);
            if (gunstockBaselineActive &&
                supportBaselineAttachPublication) {
                gunstockDebug->attachRotationDegrees =
                    hand_visual_lerp_math::rotationDistanceDegrees(
                        supportGrip.supportInputBaseline.
                            weaponWorldAtCapture,
                        gunstockDebug->weaponWorldAfter);
                gunstockDebug->attachTranslationGameUnits =
                    hand_visual_lerp_math::distanceGameUnits(
                        supportGrip.supportInputBaseline.
                            weaponWorldAtCapture.translate,
                        gunstockDebug->weaponWorldAfter.translate);
            }
            gunstockDebug->published = true;
        }

        if (supportBaselineAttachPublication) {
            const float attachRotationDegrees =
                hand_visual_lerp_math::rotationDistanceDegrees(
                    supportGrip.supportInputBaseline.weaponWorldAtCapture,
                    weaponNode->world);
            const float attachTranslationGameUnits =
                hand_visual_lerp_math::distanceGameUnits(
                    supportGrip.supportInputBaseline.
                        weaponWorldAtCapture.translate,
                    weaponNode->world.translate);
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
                    "TwoHandedGrip: support baseline published mode={} hand={} grip={} generation={:016X} attachWeaponDelta=({:.5f}gu,{:.5f}deg) surfaceSeat={:.2f}deg tandemDeltaAuthority=armed",
                    gunstockBaselineActive ? "gunstock" : "dynamic",
                    supportHandIsLeft ? "left" : "right",
                    supportGrip.gripSequence,
                    supportGrip.weaponGenerationKey,
                    attachTranslationGameUnits,
                    attachRotationDegrees,
                    supportGrip.surfaceSeatRotationRadians *
                        two_handed_grip_detail::kRadiansToDegrees);
            } else {
                ROCK_LOG_WARN(Weapon,
                    "TwoHandedGrip: support attach invariant exceeded mode={} hand={} grip={} generation={:016X} attachWeaponDelta=({:.5f}gu,{:.5f}deg)",
                    gunstockBaselineActive ? "gunstock" : "dynamic",
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

        RE::NiPoint3 primaryGripFinal = transform_math::localPointToWorld(_lastSolvedWeaponTransform, _primaryGripLocal);
        RE::NiPoint3 offhandGripFinal = resolvePartGripWorld(supportGrip, weaponNode);

        if (dynamicAcquisition) {
            auto& acquisition = _dynamicSupportAcquisition;
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
                        two_handed_grip_detail::kRadiansToDegrees,
                    acquisition.axisCorrectionRadians *
                        two_handed_grip_detail::kRadiansToDegrees,
                    acquisition.twistContributionRadians *
                        two_handed_grip_detail::kRadiansToDegrees,
                    acquisition.rawAlpha,
                    acquisition.easedAlpha,
                    acquisition.lastAppliedRotationRadians *
                        two_handed_grip_detail::kRadiansToDegrees,
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
                        two_handed_grip_detail::kRadiansToDegrees,
                    acquisition.lastPrimaryPivotError,
                    acquisition.lastSupportTargetError);
                _rotationBlend = 1.0f;
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
                _rotationBlend,
                dynamicAcquisition ? "active" : "inactive",
                separation,
                primaryGripFinal.x,
                primaryGripFinal.y,
                primaryGripFinal.z,
                offhandGripFinal.x,
                offhandGripFinal.y,
                offhandGripFinal.z,
                _primaryHandVisualLerp.lastAlpha,
                _primaryHandVisualLerp.durationSeconds,
                supportGrip.visualLerp.lastAlpha,
                supportGrip.visualLerp.durationSeconds);
        }
    }


    // ---- Part-carry update and solve ----

    bool TwoHandedGrip::tryRecaptureProviderPartGrip(
        const bool isLeft,
        const bool otherGripCarries,
        RE::NiNode* weaponNode,
        const WeaponInteractionContact& contact,
        const WeaponInteractionRuntimeState& runtimeState,
        const WeaponCollision& weaponCollision,
        const char* handRole)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        if (!grip.active ||
            grip.providerPartAuthority.active ||
            !runtimeState.providerPartAuthority.active ||
            runtimeState.providerPartAuthority.bodyId !=
                grip.contactBodyId ||
            !otherGripCarries) {
            return false;
        }

        const WeaponInteractionDecision decision =
            routeWeaponInteraction(contact, runtimeState);
        if (decision.kind != WeaponInteractionKind::SupportGrip) {
            return false;
        }

        ROCK_LOG_INFO(
            Weapon,
            "TwoHandedGrip: recapturing {} part grip under newly matched provider weapon-part target",
            handRole ? handRole : "held-hand");
        releasePartGrip(isLeft, "provider-part-target-newly-matched");
        (void)capturePartGrip(
            isLeft,
            weaponNode,
            decision,
            weaponCollision,
            runtimeState.providerPartAuthority,
            false);
        return true;
    }

    void TwoHandedGrip::updatePartCarryGrip(
        RE::NiNode* weaponNode,
        float dt,
        const EquippedWeaponGripFrameInput& frameInput,
        const WeaponInteractionContact& leftWeaponContact,
        const WeaponInteractionContact& rightWeaponContact,
        const WeaponCollision& weaponCollision,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const WeaponInteractionRuntimeState& leftRuntimeState,
        const WeaponInteractionRuntimeState& rightRuntimeState)
    {
        const bool supportHandIsLeft = !_firingHandIsLeft;
        const bool firingHandIsLeft = _firingHandIsLeft;
        const WeaponInteractionContact& firingHandContact = firingHandIsLeft ? leftWeaponContact : rightWeaponContact;
        const WeaponInteractionContact& supportHandContact = supportHandIsLeft ? leftWeaponContact : rightWeaponContact;
        const WeaponInteractionRuntimeState& firingRuntimeState = firingHandIsLeft ? leftRuntimeState : rightRuntimeState;
        const WeaponInteractionRuntimeState& supportRuntimeState = supportHandIsLeft ? leftRuntimeState : rightRuntimeState;
        const bool supportGripHeld = supportHandIsLeft ? frameInput.leftGripHeld : frameInput.rightGripHeld;
        const bool supportHandHoldingObject = supportHandIsLeft ? frameInput.leftHandHoldingObject : frameInput.rightHandHoldingObject;
        const bool firingHandHoldingObject = firingHandIsLeft ? frameInput.leftHandHoldingObject : frameInput.rightHandHoldingObject;
        const bool authoredProviderAuthorityActive =
            leftRuntimeState.providerPartAuthority.active ||
            rightRuntimeState.providerPartAuthority.active;
        const bool authoredAttachOnlyAuthorityActive =
            weapon_part_grip_report_policy::providerGrabModeIsAttachOnly(
                leftRuntimeState.providerPartAuthority.active,
                leftRuntimeState.providerPartAuthority.grabMode) ||
            weapon_part_grip_report_policy::providerGrabModeIsAttachOnly(
                rightRuntimeState.providerPartAuthority.active,
                rightRuntimeState.providerPartAuthority.grabMode) ||
            (partGrip(true).active && partGrip(true).attachOnly) ||
            (partGrip(false).active && partGrip(false).attachOnly);

        /*
         * Firing-grip reattach is the squeeze gesture: a held grab with a
         * free palm inside the reattach radius re-takes the grip. Nothing
         * attaches to an open hand, and the gesture cannot re-capture a fresh
         * detach because the detach requires the grab to be open. A hand
         * already part-gripping is never converted; open it first, then
         * squeeze the grip. With ambidextrous takeover available, EITHER free
         * hand can squeeze the firing grip - whichever hand takes it becomes
         * the firing hand (the grip point itself stays weapon-relative). The
         * current firing hand is tested first so same-frame ties keep today's
         * behavior.
         */
        struct FiringGripReattachCandidate
        {
            bool isLeft;
            bool eligible;
            bool gripHeld;
            const WeaponInteractionContact* contact;
        };
        const FiringGripReattachCandidate reattachCandidates[2] = {
            { firingHandIsLeft,
                firingHandIsLeft ? frameInput.leftReattachEligible : frameInput.rightReattachEligible,
                frameInput.primaryGripInput.held,
                &firingHandContact },
            { supportHandIsLeft,
                supportHandIsLeft ? frameInput.leftReattachEligible : frameInput.rightReattachEligible,
                supportGripHeld,
                &supportHandContact },
        };
        for (const FiringGripReattachCandidate& candidate : reattachCandidates) {
            if (candidate.isLeft != firingHandIsLeft &&
                (!_handlingSettings.ambidextrousHandoffEnabled ||
                    !canBeginPrimaryOnlyGripForHand(candidate.isLeft))) {
                continue;
            }
            if (!candidate.eligible || partGrip(candidate.isLeft).active) {
                continue;
            }
            float palmToGripDistance = 0.0f;
            if (!tryComputePalmToGripDistanceForHand(weaponNode, candidate.isLeft, palmToGripDistance)) {
                continue;
            }
            const bool reattachRequested = weapon_two_handed_grip_math::shouldReattachFiringGripOnGrab(
                candidate.gripHeld,
                palmToGripDistance,
                _handlingSettings.firingGripReattachRadiusGameUnits) ||
                (candidate.gripHeld &&
                    candidate.contact->acquisitionSource ==
                        WeaponInteractionAcquisitionSource::ProximityProbe &&
                    !authoredProviderAuthorityActive &&
                    !authoredAttachOnlyAuthorityActive);
            if (!_firingGripReattachHoverInsideRadius &&
                weapon_two_handed_grip_math::isFiringGripReattachHoverCandidate(
                    candidate.gripHeld,
                    palmToGripDistance,
                    _handlingSettings.firingGripReattachRadiusGameUnits)) {
                _firingGripReattachHoverInsideRadius = true;
                _firingGripReattachHoverHandIsLeft = candidate.isLeft;
            }
            if (reattachRequested &&
                tryReattachFiringGrip(
                    candidate.isLeft,
                    weaponNode,
                    *candidate.contact,
                    authoredProviderAuthorityActive,
                    authoredAttachOnlyAuthorityActive)) {
                const bool newSupportHandIsLeft = !_firingHandIsLeft;
                if (partGrip(newSupportHandIsLeft).active) {
                    // Re-lock the two-hand separation against the (possibly
                    // swapped) support grip and re-blend the support target in.
                    const RE::NiPoint3 supportGripWorld = resolvePartGripWorld(partGrip(newSupportHandIsLeft), weaponNode);
                    const RE::NiPoint3 firingGripWorld = weaponLocalToWorld(_primaryGripLocal, weaponNode);
                    const RE::NiPoint3 separationDelta = sub(supportGripWorld, firingGripWorld);
                    const float separation = std::sqrt(dot(separationDelta, separationDelta));
                    if (std::isfinite(separation)) {
                        _lockedGripSeparationWorld = separation;
                    }
                    if (candidate.isLeft != firingHandIsLeft) {
                        _rotationBlend = 0.0f;
                    }
                    _state = TwoHandedState::Gripping;
                    // Fresh two-hand configuration: the just-taken firing grip
                    // gets the same release-defer window as a fresh support grab.
                    _supportGripAgeFrames = 0;
                    _freshSupportGripDeferLogged = false;
                    RE::NiTransform supportInputWorld{};
                    if (!tryGetSolverHandTransform(
                            newSupportHandIsLeft,
                            supportInputWorld) ||
                        !initializeGunstockSupportRole(
                            weaponNode,
                            newSupportHandIsLeft,
                            supportInputWorld,
                            "part-carry-firing-grip-reattach")) {
                        ROCK_LOG_WARN(
                            Weapon,
                            "TwoHandedGrip: part-carry reattach failed closed because the new support role could not be initialized");
                        transitionToInactive(false);
                        return;
                    }
                    WeaponPartGrip& reattachedSupportGrip =
                        partGrip(newSupportHandIsLeft);
                    const bool reattachedGunstockBaselineActive =
                        isGunstockSupportBaselineActive(
                            newSupportHandIsLeft,
                            reattachedSupportGrip);
                    if (!reattachedGunstockBaselineActive &&
                        weapon_support_authority_policy::
                            shouldUseDynamicSupportAcquisition(
                                _authorityMode,
                                reattachedSupportGrip.authoredSupportGrip,
                                reattachedSupportGrip.providerPartAuthority.active,
                                reattachedSupportGrip.attachOnly) &&
                        !initializeDynamicSupportBaseline(
                            weaponNode,
                            newSupportHandIsLeft,
                            "part-carry-firing-grip-reattach")) {
                        ROCK_LOG_WARN(
                            Weapon,
                            "TwoHandedGrip: part-carry reattach failed closed because the zero-delta dynamic baseline could not be captured");
                        transitionToInactive(false);
                        return;
                    }
                    updateFullWeaponAuthorityGrip(weaponNode, dt);
                } else {
                    if (!_firingHandIsLeft && ownsWeaponTransform()) {
                        beginWeaponVisualReturn("part-carry-reattached-primary-only");
                    }
                    transitionToPrimaryOnly(
                        weaponNode,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        "part-carry-reattached-firing-grip");
                }
                return;
            }
        }

        bool lastReleaseWasSupportHand = true;

        WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        if (supportGrip.active) {
            if (!providerPartAuthorityStillCurrent(supportGrip, currentWeaponGenerationKey) || !supportRuntimeState.supportGripAllowed) {
                /*
                 * Provider revocation and offhand reservation are policy
                 * changes, not a player release: return the weapon to
                 * FRIK-native carry instead of dropping it.
                 */
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing part-carry authority because provider revoked the support part grip");
                transitionToInactive(false);
                return;
            }
            if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(supportGripHeld, supportHandHoldingObject)) {
                releasePartGrip(supportHandIsLeft, "support-grip-released", true);
                lastReleaseWasSupportHand = true;
            }
        }

        WeaponPartGrip& freeHandGrip = partGrip(firingHandIsLeft);
        if (freeHandGrip.active) {
            if (!providerPartAuthorityStillCurrent(freeHandGrip, currentWeaponGenerationKey)) {
                releasePartGrip(firingHandIsLeft, "provider-part-authority-lost");
                lastReleaseWasSupportHand = false;
            } else if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(frameInput.primaryGripInput.held, firingHandHoldingObject)) {
                releasePartGrip(firingHandIsLeft, "free-hand-grip-released", true);
                lastReleaseWasSupportHand = false;
            }
        }

        /*
         * Mid-hold conversion, part-carry flavor: a part grip captured
         * WITHOUT provider authority whose own part now resolves to a
         * matched provider target (consumer armed its whitelist while the
         * hand was already holding — per-hand trigger arming) recaptures
         * immediately under the new resolution. Gated on the OTHER grip
         * holding carry authority: converting the last carry grip to
         * attach-only glue would drop the weapon through the fail-closed
         * all-grips check below. The free hand recaptures in the same
         * update rather than release-to-recapture because its capture path
         * is press-edged; the support hand gets the same treatment for
         * symmetry (no one-frame glue gap).
         */
        (void)tryRecaptureProviderPartGrip(
            firingHandIsLeft,
            weapon_part_grip_report_policy::partGripCountsAsCarry(
                supportGrip.active,
                supportGrip.attachOnly),
            weaponNode,
            firingHandContact,
            firingRuntimeState,
            weaponCollision,
            "free-hand");
        (void)tryRecaptureProviderPartGrip(
            supportHandIsLeft,
            weapon_part_grip_report_policy::partGripCountsAsCarry(
                freeHandGrip.active,
                freeHandGrip.attachOnly),
            weaponNode,
            supportHandContact,
            supportRuntimeState,
            weaponCollision,
            "support");

        if (!freeHandGrip.active && frameInput.primaryGripInput.pressed) {
            const WeaponInteractionDecision freeHandDecision = routeWeaponInteraction(firingHandContact, firingRuntimeState);
            if (weapon_two_handed_grip_math::canStartFreeHandPartGrip(
                    freeHandDecision.kind == WeaponInteractionKind::SupportGrip,
                    frameInput.primaryGripInput.pressed,
                    firingHandHoldingObject,
                    freeHandGrip.active)) {
                if (capturePartGrip(firingHandIsLeft, weaponNode, freeHandDecision, weaponCollision, firingRuntimeState.providerPartAuthority, false)) {
                    /*
                     * AttachOnly keeps its authored source frames so the glued
                     * hand follows provider-driven part motion; it never joins
                     * the carry solve, so it cannot feed part animation back
                     * into the carry (the drift lockPartGripToWeaponRoot
                     * prevents). Separation/blend only matter for a two-anchor
                     * carry, which needs both grips to hold carry authority.
                     */
                    if (freeHandGrip.attachOnly) {
                        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: free-hand part grip captured as provider attach-only glue");
                    } else {
                        lockPartGripToWeaponRoot(firingHandIsLeft);
                        if (weapon_part_grip_report_policy::partGripCountsAsCarry(supportGrip.active, supportGrip.attachOnly)) {
                            _partCarryGripSeparationWorld = partCarryGripSeparation(weaponNode);
                            _rotationBlend = 0.0f;
                        }
                    }
                }
            }
        }

        if (!supportGrip.active) {
            const WeaponInteractionDecision supportDecision = routeWeaponInteraction(supportHandContact, supportRuntimeState);
            if (supportDecision.kind == WeaponInteractionKind::SupportGrip &&
                weapon_two_handed_grip_math::canStartSupportGrip(true, supportGripHeld, supportHandHoldingObject)) {
                if (capturePartGrip(supportHandIsLeft, weaponNode, supportDecision, weaponCollision, supportRuntimeState.providerPartAuthority, false)) {
                    // Symmetric to the free-hand capture above: attach-only
                    // glue keeps source frames and stays out of the carry.
                    if (supportGrip.attachOnly) {
                        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: support-hand part grip captured as provider attach-only glue");
                    } else {
                        lockPartGripToWeaponRoot(supportHandIsLeft);
                        if (weapon_part_grip_report_policy::partGripCountsAsCarry(freeHandGrip.active, freeHandGrip.attachOnly)) {
                            _partCarryGripSeparationWorld = partCarryGripSeparation(weaponNode);
                            _rotationBlend = 0.0f;
                        }
                    }
                }
            }
        }

        /*
         * Only carry-authority grips can hold the weapon. When the last carry
         * grip releases, a remaining AttachOnly glue cannot inherit pivot
         * authority (never upgrade), so it releases with the carry and the
         * normal manual-drop request proceeds (fail closed).
         */
        if (!weapon_part_grip_report_policy::partGripCountsAsCarry(supportGrip.active, supportGrip.attachOnly) &&
            !weapon_part_grip_report_policy::partGripCountsAsCarry(freeHandGrip.active, freeHandGrip.attachOnly)) {
            releasePartGrip(supportHandIsLeft, "carry-authority-lost", true);
            releasePartGrip(firingHandIsLeft, "carry-authority-lost", true);
            requestEquippedWeaponDrop(
                "part-carry-all-grips-released",
                lastReleaseWasSupportHand ?
                    (supportHandIsLeft ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right) :
                    (firingHandIsLeft ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right));
            return;
        }

        // The pivot must always be a carry-authority grip; after the check
        // above, the other hand is guaranteed to hold one.
        if (!weapon_part_grip_report_policy::partGripCountsAsCarry(
                partGrip(_partCarryPivotIsLeft).active,
                partGrip(_partCarryPivotIsLeft).attachOnly)) {
            _partCarryPivotIsLeft = !_partCarryPivotIsLeft;
        }

        (void)solvePartCarryWeaponAuthority(weaponNode, dt);
    }

    float TwoHandedGrip::partCarryGripSeparation(RE::NiNode* weaponNode) const
    {
        const RE::NiPoint3 leftGripWorld = resolvePartGripWorld(partGrip(true), weaponNode);
        const RE::NiPoint3 rightGripWorld = resolvePartGripWorld(partGrip(false), weaponNode);
        const RE::NiPoint3 delta = sub(leftGripWorld, rightGripWorld);
        const float separation = std::sqrt(dot(delta, delta));
        return std::isfinite(separation) ? separation : 0.0f;
    }

    bool TwoHandedGrip::solvePartCarryWeaponAuthority(RE::NiNode* weaponNode, float dt)
    {
        const bool pivotIsLeft = _partCarryPivotIsLeft;
        const WeaponPartGrip& pivotGrip = partGrip(pivotIsLeft);
        const WeaponPartGrip& aimGrip = partGrip(!pivotIsLeft);
        // An AttachOnly glue never aims the weapon; the carry solves
        // single-anchor around the pivot and the glue publishes afterwards.
        const bool aimGripCarries = weapon_part_grip_report_policy::partGripCountsAsCarry(aimGrip.active, aimGrip.attachOnly);
        if (!pivotGrip.active || !pivotGrip.hasHandWeaponLocal) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because captured hand frames are unavailable");
            transitionToInactive(false);
            return false;
        }

        RE::NiTransform pivotHandTransform{};
        if (!tryGetSolverHandTransform(pivotIsLeft, pivotHandTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because authoritative hand transforms are unavailable");
            transitionToInactive(false);
            return false;
        }

        if (aimGripCarries) {
            RE::NiTransform aimHandTransform{};
            if (!tryGetSolverHandTransform(!pivotIsLeft, aimHandTransform)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because aim hand transform is unavailable");
                transitionToInactive(false);
                return false;
            }

            _rotationBlend = (std::min)(1.0f, _rotationBlend + dt * ROTATION_BLEND_SPEED);

            /*
             * The two-anchor solve must be closed over the captured
             * weapon-root-local grip points. Part-carry feeds its own solved
             * transform back as the next frame's base, so re-resolving grips
             * through live part-node chains lets any per-frame part animation
             * integrate into a steady carry drift (verified by telemetry:
             * rigid-weapon grip separation grew frame over frame). The
             * trade-off is that a two-anchor carry does not follow externally
             * driven part motion; provider part revocation releases the grip
             * in that case.
             */
            const RE::NiPoint3 pivotPalm = computeGrabLegacyPalmPivotAWorldFromHandBasis(pivotHandTransform, pivotIsLeft);
            const RE::NiPoint3 aimPalm = computeGrabLegacyPalmPivotAWorldFromHandBasis(aimHandTransform, !pivotIsLeft);
            const RE::NiPoint3 currentAimGripWorld = weaponLocalToWorld(aimGrip.gripLocal, weaponNode);
            const RE::NiPoint3 currentPivotGripWorld = weaponLocalToWorld(pivotGrip.gripLocal, weaponNode);
            const RE::NiPoint3 currentSeparationDelta = sub(currentAimGripWorld, currentPivotGripWorld);
            const float currentSeparation = std::sqrt(dot(currentSeparationDelta, currentSeparationDelta));
            const float lockedSeparation = _partCarryGripSeparationWorld > 0.0f ? _partCarryGripSeparationWorld : currentSeparation;
            const RE::NiPoint3 lockedAimTarget = makeLockedSupportGripTarget(
                pivotPalm,
                aimPalm,
                currentAimGripWorld,
                lockedSeparation,
                0.001f);
            const RE::NiPoint3 blendedAimTarget = lerpPoint(currentAimGripWorld, lockedAimTarget, _rotationBlend);

            const auto solverInput = buildTwoHandedSolverInput(
                weaponNode->world,
                pivotGrip.gripLocal,
                aimGrip.gripLocal,
                pivotPalm,
                blendedAimTarget,
                aimGrip.normalLocal,
                computePalmNormalFromHandBasis(
                    aimHandTransform,
                    !pivotIsLeft));

            const auto solved = solveTwoHandedWeaponTransformFrikPivot(solverInput);
            if (!solved.solved) {
                return true;
            }

            // Break the rotation feedback loop's orthonormality decay before
            // the solved transform becomes next frame's base.
            RE::NiTransform stabilizedWeaponWorld = solved.weaponWorldTransform;
            stabilizedWeaponWorld.rotate = orthonormalizeStoredRotation(stabilizedWeaponWorld.rotate);

            if (!applyWeaponVisualAuthority(weaponNode, stabilizedWeaponWorld)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because ROCK visual weapon authority failed");
                transitionToInactive(false);
                return false;
            }

            static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
            static_assert(weapon_visual_authority_math::weaponVisualPrecedesLockedHandAuthority());
            publishGripHandPoses(pivotIsLeft);
            publishGripHandPoses(!pivotIsLeft);

            if (!applyPartGripLockedVisual(pivotIsLeft, weaponNode, dt, &pivotHandTransform) ||
                !applyPartGripLockedVisual(!pivotIsLeft, weaponNode, dt, &aimHandTransform)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because ROCK part grip hand authority failed");
                transitionToInactive(false);
                return false;
            }
        } else {
            RE::NiTransform solvedWeaponWorld{};
            if (pivotGrip.hasSourceFrames && pivotGrip.hasAttachmentWeaponLocal && resolveCurrentSupportAttachmentRoot(pivotGrip, weaponNode)) {
                const RE::NiTransform solvedSourceWorld =
                    transform_math::composeTransforms(pivotHandTransform, transform_math::invertTransform(pivotGrip.handSourceLocal));
                solvedWeaponWorld = transform_math::composeTransforms(solvedSourceWorld, transform_math::invertTransform(pivotGrip.attachmentWeaponLocal));
            } else {
                solvedWeaponWorld = transform_math::composeTransforms(pivotHandTransform, transform_math::invertTransform(pivotGrip.handWeaponLocal));
            }
            if (!isFiniteTransform(solvedWeaponWorld)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because single-anchor weapon solve produced invalid transform");
                transitionToInactive(false);
                return false;
            }

            if (!applyWeaponVisualAuthority(weaponNode, solvedWeaponWorld)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because ROCK visual weapon authority failed");
                transitionToInactive(false);
                return false;
            }

            static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
            publishGripHandPoses(pivotIsLeft);
            if (!applyPartGripLockedVisual(pivotIsLeft, weaponNode, dt, &pivotHandTransform)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because ROCK part grip hand authority failed");
                transitionToInactive(false);
                return false;
            }
        }

        /*
         * AttachOnly glue publishes after the weapon solve so it composes from
         * this frame's part transforms (including provider part drives applied
         * earlier in the frame). A glue visual failure only loses the attach;
         * the carry pivot must survive it.
         */
        if (aimGrip.active && aimGrip.attachOnly) {
            RE::NiTransform attachHandTransform{};
            const RE::NiTransform* liveAttachHandWorld =
                tryGetSolverHandTransform(!pivotIsLeft, attachHandTransform) ? &attachHandTransform : nullptr;
            publishGripHandPoses(!pivotIsLeft);
            if (!applyPartGripLockedVisual(!pivotIsLeft, weaponNode, dt, liveAttachHandWorld)) {
                releasePartGrip(!pivotIsLeft, "attach-only-visual-authority-failed");
            }
        }

        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;

        if (++_gripLogCounter >= 90) {
            _gripLogCounter = 0;
            const RE::NiPoint3 pivotGripFinal = resolvePartGripWorld(pivotGrip, weaponNode);
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: part-carry authority pivot={} anchors={} pivotGrip=({:.1f},{:.1f},{:.1f}) handLerp=({:.2f}/{:.3f}s,{:.2f}/{:.3f}s)",
                pivotIsLeft ? "left" : "right",
                aimGrip.active ? 2 : 1,
                pivotGripFinal.x,
                pivotGripFinal.y,
                pivotGripFinal.z,
                pivotGrip.visualLerp.lastAlpha,
                pivotGrip.visualLerp.durationSeconds,
                aimGrip.visualLerp.lastAlpha,
                aimGrip.visualLerp.durationSeconds);
        }
        return true;
    }

    // ---- Visual-only support solve ----

    void TwoHandedGrip::updateVisualOnlySupportGrip(RE::NiNode* weaponNode, float dt)
    {
        const bool supportHandIsLeft = !_firingHandIsLeft;

        if (_firingHandIsLeft) {
            // Visual-only support never steers aim, but with a LEFT firing
            // hand the weapon itself must still be ROCK-carried (FRIK's glue
            // is blocked); the shooting-cup right hand stays visual-only.
            if (!solveLeftFiringWeaponCarry(weaponNode)) {
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
            transitionToInactive(false);
            return;
        }

        _lastSolvedWeaponTransform = weaponNode ? weaponNode->world : RE::NiTransform{};
        _hasSolvedWeaponTransform = _firingHandIsLeft && _hasSolvedWeaponTransform;

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


    // ---- Dynamic support input baseline ----

    bool TwoHandedGrip::isDynamicSupportBaselineActive(
        const bool supportHandIsLeft,
        const WeaponPartGrip& supportGrip) const
    {
        return isSupportInputBaselineActive(
            supportHandIsLeft,
            supportGrip,
            SupportInputBaselineKind::Dynamic);
    }

    bool TwoHandedGrip::initializeDynamicSupportBaseline(
        RE::NiNode* weaponNode,
        const bool supportHandIsLeft,
        const char* reason)
    {
        WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        if (_authorityMode != weapon_support_authority_policy::
                                  WeaponSupportAuthorityMode::
                                      FullTwoHandedSolver ||
            supportGrip.authoredSupportGrip ||
            supportGrip.providerPartAuthority.active ||
            supportGrip.attachOnly ||
            !weaponNode ||
            !supportGrip.active ||
            !supportGrip.hasHandWeaponLocal ||
            !_hasFiringHandWeaponLocal ||
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
            _currentHandDriverFrames[
                primaryHandIsLeft ? 0u : 1u];
        const auto& supportDriver =
            _currentHandDriverFrames[
                supportHandIsLeft ? 0u : 1u];
        const RE::NiTransform primaryGripTargetWorld =
            weapon_visual_authority_math::weaponLocalFrameToWorld(
                weaponNode->world,
                _primaryHandWeaponLocal);
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

}
