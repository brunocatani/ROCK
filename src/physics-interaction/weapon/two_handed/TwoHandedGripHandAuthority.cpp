#include "physics-interaction/weapon/two_handed/TwoHandedGrip.h"

/*
 * HAND and WEAPON visual authority: the single publication chokepoint between
 * ROCK's solved transforms and what FRIK renders.
 *
 * applyWeaponVisualAuthority is that chokepoint. Recoil references, hand and
 * weapon visual returns, collision hand authority, native reload suspension,
 * pre-FRIK reconstruction and the locked firing-hand visual all funnel through
 * this file, so that exactly one system owns each FRIK tag at a time.
 *
 * ORDERING COUPLING: hasVisualAuthorityForHand reads
 * _gunstockHandAuthorityActive, which TwoHandedGripGunstock.cpp owns. Gunstock
 * alignment must therefore run before authority is queried for the frame. The
 * gunstock TU carries the matching note.
 *
 * The native scope camera capture and apply helpers this file calls live in
 * TwoHandedGripInternal.h, not in the scope TU, so that this file can call them
 * without a cross-TU cycle.
 */
#include "physics-interaction/weapon/two_handed/TwoHandedGripInternal.h"

#include "physics-interaction/hand/skeleton/HandFrame.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/visual/PreFrikHandAuthorityPolicy.h"
#include "physics-interaction/weapon/collision/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/native_anim/NativeScopeSightAnchorPolicy.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RockConfig.h"
#include "RockUtils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string_view>

namespace rock
{
    using two_handed_grip_detail::applyNativeScopeCameraWorldTarget;
    using two_handed_grip_detail::captureNativeScopeCameraFollow;
    using two_handed_grip_detail::GRIP_HAND_POSE_PRIORITY;
    using two_handed_grip_detail::GUNSTOCK_ALIGNMENT_TAG;
    using two_handed_grip_detail::isFiniteTransform;
    using two_handed_grip_detail::isUsableHandAuthorityTransform;
    using two_handed_grip_detail::moveWeaponPresentationRigidly;
    using two_handed_grip_detail::makeNativeScopeCameraDebugSnapshot;
    using two_handed_grip_detail::NativeScopeCameraFollowCapture;
    using two_handed_grip_detail::NativeScopeCameraFollowResult;
    using two_handed_grip_detail::PRIMARY_DETACH_TAG;
    using two_handed_grip_detail::PRIMARY_GRIP_TAG;
    using two_handed_grip_detail::RETURN_HAND_TAG;
    using two_handed_grip_detail::RETURN_HAND_VISUAL_PRIORITY;
    using two_handed_grip_detail::SUPPORT_GRIP_TAG;
    using two_handed_grip_detail::tryGetRootFlattenedHandBoneTransform;
    using two_handed_grip_detail::WEAPON_COLLISION_HAND_PRIORITY;
    using two_handed_grip_detail::WEAPON_COLLISION_HAND_TAG;

    // ---- Recoil references and visual returns ----

    void TwoHandedGrip::resetLockedHandVisualLerp()
    {
        _primaryHandVisualLerp = {};
        partGrip(true).visualLerp = {};
        partGrip(false).visualLerp = {};
    }

    bool TwoHandedGrip::isHandVisualReturnActive(const bool isLeft) const
    {
        return _returningHandVisuals[isLeft ? 0u : 1u].transition.active;
    }

    bool TwoHandedGrip::hasVisualAuthorityForHand(const bool isLeft) const
    {
        if (isNativeReloadSupportHand(isLeft)) {
            return false;
        }
        if (_gunstockHandAuthorityActive[isLeft ? 0u : 1u] ||
            _gunstockDedicatedHandAuthorityActive[isLeft ? 0u : 1u] ||
            isHandVisualReturnActive(isLeft) ||
            partGrip(isLeft).active) {
            return true;
        }
        return isLeft == _firingHandIsLeft &&
            _state == TwoHandedState::Gripping &&
            weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(_authorityMode);
    }

    void TwoHandedGrip::recordPublishedHandWorld(const bool isLeft, const RE::NiTransform& appliedWorld)
    {
        if (!isUsableHandAuthorityTransform(appliedWorld)) {
            return;
        }
        const std::size_t index = isLeft ? 0u : 1u;
        _lastPublishedHandWorld[index] = appliedWorld;
        _hasLastPublishedHandWorld[index] = true;
        _weaponCollisionBaselineHandWorld[index] = appliedWorld;
        _weaponCollisionBaselineHandWorldValid[index] = true;
        rememberFiringRecoilReference(isLeft, appliedWorld);
    }

    bool TwoHandedGrip::tryResolveControlledFiringRecoilSource(
        const bool isLeft,
        RE::NiNode*& outWeaponNode,
        std::uint64_t& outWeaponGenerationKey) const
    {
        outWeaponNode = nullptr;
        outWeaponGenerationKey = 0;
        if (isLeft != _firingHandIsLeft ||
            _nativeReloadHandAuthorityActive) {
            return false;
        }

        if (isLeft) {
            if (!_activeWeaponNode ||
                _activeWeaponGenerationKey == 0 ||
                !_weaponNodeOwnershipBlockEngaged ||
                !isManualOwnershipActive()) {
                return false;
            }
            outWeaponNode = _activeWeaponNode;
            outWeaponGenerationKey = _activeWeaponGenerationKey;
            return true;
        }

        if (_activeWeaponNode &&
            _activeWeaponGenerationKey != 0 &&
            _state == TwoHandedState::Gripping &&
            weapon_support_authority_policy::
                supportGripAppliesPrimaryHandAuthority(_authorityMode)) {
            outWeaponNode = _activeWeaponNode;
            outWeaponGenerationKey = _activeWeaponGenerationKey;
            return true;
        }

        /*
         * Normal physical-right carry is owned by the authored primary grip
         * runtime rather than the manual two-hand state machine. It still
         * needs the same controlled recoil contract as physical-left carry:
         * hFRIK moves the hand, then ROCK applies that exact world delta to
         * the identity-bound weapon before collision and muzzle publication.
         */
        if (_rightFiringHandCanonicalSource !=
                RightFiringCanonicalSource::AuthoredAnimation ||
            !_hasRightFiringHandCanonicalWeaponLocal ||
            !_rightFiringHandCanonicalWeaponNode ||
            _rightFiringHandCanonicalGenerationKey == 0 ||
            _rightFiringHandCanonicalOwnershipKey == 0) {
            return false;
        }

        outWeaponNode = _rightFiringHandCanonicalWeaponNode;
        outWeaponGenerationKey =
            _rightFiringHandCanonicalGenerationKey;
        return true;
    }

    void TwoHandedGrip::rememberFiringRecoilReference(
        const bool isLeft,
        const RE::NiTransform& handWorld)
    {
        RE::NiNode* recoilWeaponNode = nullptr;
        std::uint64_t recoilWeaponGenerationKey = 0;
        if (!tryResolveControlledFiringRecoilSource(
                isLeft,
                recoilWeaponNode,
                recoilWeaponGenerationKey) ||
            !isUsableHandAuthorityTransform(handWorld)) {
            return;
        }

        const std::size_t index = isLeft ? 0u : 1u;
        if (_firingRecoilReferenceCapturedBeforeFrik[index] &&
            _currentSourceSchedulerSequence != 0 &&
            _firingRecoilReferenceSchedulerSequence[index] ==
                _currentSourceSchedulerSequence) {
            return;
        }
        _firingRecoilReferenceHandWorld[index] = handWorld;
        _firingRecoilReferenceGenerationKey[index] =
            recoilWeaponGenerationKey;
        _firingRecoilReferenceSchedulerSequence[index] =
            _currentSourceSchedulerSequence;
        _firingRecoilReferenceCapturedBeforeFrik[index] = false;
        _hasFiringRecoilReference[index] = true;
    }

    void TwoHandedGrip::clearFiringRecoilPresentationState()
    {
        _firingRecoilReferenceHandWorld = {};
        _firingRecoilReferenceGenerationKey = {};
        _firingRecoilReferenceSchedulerSequence = {};
        _firingRecoilReferenceCapturedBeforeFrik = {};
        _firingRecoilAcceptedGenerationKey = 0;
        _firingRecoilAcceptedSequence = 0;
        _firingRecoilConsumedSequence = 0;
        _hasFiringRecoilReference = {};
        _firingRecoilAcceptedHandIsLeft = false;
    }

    void TwoHandedGrip::clearFiringRecoilPresentationState(const bool isLeft)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        _firingRecoilReferenceHandWorld[index] = {};
        _firingRecoilReferenceGenerationKey[index] = 0;
        _firingRecoilReferenceSchedulerSequence[index] = 0;
        _firingRecoilReferenceCapturedBeforeFrik[index] = false;
        _hasFiringRecoilReference[index] = false;
        if (_firingRecoilAcceptedHandIsLeft == isLeft) {
            _firingRecoilConsumedSequence =
                _firingRecoilAcceptedSequence;
            _firingRecoilAcceptedGenerationKey = 0;
        }
    }

    void TwoHandedGrip::beginHandVisualReturn(const bool isLeft, const char* reason)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        auto& state = _returningHandVisuals[index].transition;
        if (!g_rockConfig.rockWeaponVisualReturnEnabled ||
            !_hasLastPublishedHandWorld[index] ||
            !isUsableHandAuthorityTransform(_lastPublishedHandWorld[index]) ||
            !frik_visual_authority::isAvailable()) {
            clearHandVisualReturn(isLeft, "not-eligible", false);
            return;
        }

        state.begin(_lastPublishedHandWorld[index]);
        if (!frik_visual_authority::applyExternalHandWorldTransform(
                RETURN_HAND_TAG,
                handFromBool(isLeft),
                state.start,
                RETURN_HAND_VISUAL_PRIORITY)) {
            state.clear();
            clearPreFrikRetainedHandAuthority(
                RetainedHandAuthorityKind::Return,
                isLeft);
            (void)frik_visual_authority::clearExternalHandWorldTransform(RETURN_HAND_TAG, handFromBool(isLeft));
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: hand return start failed hand={}", isLeft ? "left" : "right");
            return;
        }

        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: hand return started hand={} reason={} from=({:.2f},{:.2f},{:.2f})",
            isLeft ? "left" : "right",
            reason ? reason : "unknown",
            state.start.translate.x,
            state.start.translate.y,
            state.start.translate.z);
    }

    void TwoHandedGrip::updateHandVisualReturns(const float dt)
    {
        if (_scopeMenuOpenThisFrame) {
            return;
        }

        for (const bool isLeft : { true, false }) {
            const std::size_t index = isLeft ? 0u : 1u;
            auto& state = _returningHandVisuals[index].transition;
            if (!state.active) {
                continue;
            }

            RE::NiTransform targetWorld{};
            if (!frik_visual_authority::isAvailable() ||
                !tryGetSolverHandTransform(isLeft, targetWorld) ||
                !isUsableHandAuthorityTransform(targetWorld)) {
                clearHandVisualReturn(isLeft, "tracked-hand-unavailable", true);
                continue;
            }

            const bool timingPending = !state.durationInitialized;
            const float initialDistance = timingPending ?
                hand_visual_lerp_math::distanceGameUnits(state.start.translate, targetWorld.translate) :
                0.0f;
            const float initialAngleDegrees = timingPending ?
                hand_visual_lerp_math::rotationDistanceDegrees(state.start, targetWorld) :
                0.0f;
            const auto result = hand_visual_lerp_math::advanceVisualReturn(
                state,
                targetWorld,
                dt,
                hand_visual_lerp_math::VisualReturnConfig{
                    .minSeconds = g_rockConfig.rockWeaponVisualReturnTimeMin,
                    .maxSeconds = g_rockConfig.rockWeaponVisualReturnTimeMax,
                    .minDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMinDistance,
                    .maxDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMaxDistance,
                    .minAngleDegrees = g_rockConfig.rockWeaponVisualReturnMinAngleDegrees,
                    .maxAngleDegrees = g_rockConfig.rockWeaponVisualReturnMaxAngleDegrees,
                });
            if (timingPending) {
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: hand return timing hand={} distance={:.2f}gu angle={:.1f}deg duration={:.3f}s",
                    isLeft ? "left" : "right",
                    initialDistance,
                    initialAngleDegrees,
                    state.durationSeconds);
            }
            if (!isUsableHandAuthorityTransform(result.transform) ||
                !frik_visual_authority::applyExternalHandWorldTransform(
                    RETURN_HAND_TAG,
                    handFromBool(isLeft),
                    result.transform,
                    RETURN_HAND_VISUAL_PRIORITY)) {
                clearHandVisualReturn(isLeft, "publish-failed", true);
                continue;
            }
            recordPreFrikRetainedHandAuthority(
                RetainedHandAuthorityKind::Return,
                isLeft,
                result.transform);

            if (result.reachedTarget) {
                const float completedDuration = state.durationSeconds;
                (void)frik_visual_authority::clearExternalHandWorldTransform(RETURN_HAND_TAG, handFromBool(isLeft));
                state.clear();
                clearPreFrikRetainedHandAuthority(
                    RetainedHandAuthorityKind::Return,
                    isLeft);
                _hasLastPublishedHandWorld[index] = false;
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: hand return completed hand={} duration={:.3f}s",
                    isLeft ? "left" : "right",
                    completedDuration);
            }
        }
    }

    void TwoHandedGrip::clearHandVisualReturn(const bool isLeft, const char* reason, const bool logCancellation)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        auto& state = _returningHandVisuals[index].transition;
        const bool wasActive = state.active;
        (void)frik_visual_authority::clearExternalHandWorldTransform(RETURN_HAND_TAG, handFromBool(isLeft));
        state.clear();
        clearPreFrikRetainedHandAuthority(
            RetainedHandAuthorityKind::Return,
            isLeft);
        _hasLastPublishedHandWorld[index] = false;
        if (wasActive && logCancellation) {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: hand return cancelled hand={} reason={}",
                isLeft ? "left" : "right",
                reason ? reason : "unknown");
        }
    }

    void TwoHandedGrip::cancelHandVisualReturn(const bool isLeft, const char* reason)
    {
        clearHandVisualReturn(isLeft, reason, true);
    }

    void TwoHandedGrip::beginWeaponVisualReturn(const char* reason)
    {
        if (!g_rockConfig.rockWeaponVisualReturnEnabled ||
            _returningWeaponVisual.localTransition.active ||
            !_activeWeaponNode ||
            !_hasWeaponNodeLocalBaseline ||
            _activeWeaponGenerationKey == 0 ||
            _activeEquippedWeaponOwnershipKey == 0) {
            return;
        }

        RE::NiTransform startWorld = _hasLastRenderedWeaponWorld ? _lastRenderedWeaponWorld : _activeWeaponNode->world;
        if (!isFiniteTransform(startWorld) || !isFiniteTransform(_weaponNodeLocalBaseline)) {
            return;
        }

        RE::NiNode* nativeParent = _activeWeaponNode->parent;
        if (_weaponNodeReparentedToLeftHand) {
            nativeParent = resolveFirstPersonHandNode(false);
            if (!nativeParent) {
                return;
            }
        }
        if (!nativeParent) {
            return;
        }

        const RE::NiTransform startLocal = weapon_visual_authority_math::worldTargetToParentLocal(nativeParent->world, startWorld);
        if (!isFiniteTransform(startLocal)) {
            return;
        }

        /*
         * blockPrimaryWeaponNodeOwnership is hFRIK's external LEFT-carry
         * topology switch, not a transform-write-only blocker. Retaining it
         * here makes hFRIK reparent the weapon back under LArm_Hand on the next
         * frame, which invalidates this right-parent-local return and snaps the
         * weapon immediately. Release left-carry topology before beginning the
         * overlay. ROCK runs after hFRIK and republishes the interpolated node
         * every frame, so hFRIK's earlier native write cannot reach rendering;
         * at the exact endpoint both writers already agree on the baseline.
         */
        releaseFiringHandWeaponNodeOwnership(_activeWeaponNode);
        if (_activeWeaponNode->parent != nativeParent) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: weapon return skipped because native right-hand parenting could not be restored");
            return;
        }

        ReturningWeaponVisualState returnState{};
        returnState.weaponNode = _activeWeaponNode;
        returnState.nativeParent = nativeParent;
        returnState.weaponGenerationKey = _activeWeaponGenerationKey;
        returnState.equippedWeaponOwnershipKey = _activeEquippedWeaponOwnershipKey;
        returnState.nativeBaselineLocal = _weaponNodeLocalBaseline;
        returnState.retainPrimaryPoseBlocker = _firingHandIsLeft;
        returnState.localTransition.begin(startLocal);
        returnState.localTransition.durationSeconds = hand_visual_lerp_math::computeVisualReturnDuration(
            startLocal,
            returnState.nativeBaselineLocal,
            hand_visual_lerp_math::VisualReturnConfig{
                .minSeconds = g_rockConfig.rockWeaponVisualReturnTimeMin,
                .maxSeconds = g_rockConfig.rockWeaponVisualReturnTimeMax,
                .minDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMinDistance,
                .maxDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMaxDistance,
                .minAngleDegrees = g_rockConfig.rockWeaponVisualReturnMinAngleDegrees,
                .maxAngleDegrees = g_rockConfig.rockWeaponVisualReturnMaxAngleDegrees,
            });
        returnState.localTransition.durationInitialized = true;
        if (!moveWeaponPresentationRigidly(_activeWeaponNode, startWorld)) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                2000,
                "TwoHandedGrip: weapon return rejected an invalid presentation subtree");
            return;
        }
        _returningWeaponVisual = returnState;
        _lastRenderedWeaponWorld = _activeWeaponNode->world;
        _hasLastRenderedWeaponWorld = true;
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: weapon return started reason={} distance={:.2f}gu angle={:.1f}deg duration={:.3f}s",
            reason ? reason : "unknown",
            hand_visual_lerp_math::distanceGameUnits(startLocal.translate, returnState.nativeBaselineLocal.translate),
            hand_visual_lerp_math::rotationDistanceDegrees(startLocal, returnState.nativeBaselineLocal),
            _returningWeaponVisual.localTransition.durationSeconds);
    }

    void TwoHandedGrip::updateWeaponVisualReturn(
        RE::NiNode* currentWeaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey,
        const float dt)
    {
        auto& state = _returningWeaponVisual;
        if (!state.localTransition.active) {
            return;
        }
        if (!runtime_state::isLocalSkeletonReady() ||
            !currentWeaponNode ||
            currentWeaponNode != state.weaponNode ||
            currentWeaponGenerationKey != state.weaponGenerationKey ||
            currentEquippedWeaponOwnershipKey != state.equippedWeaponOwnershipKey ||
            !state.nativeParent ||
            !isFiniteTransform(state.nativeParent->world) ||
            currentWeaponNode->parent != state.nativeParent) {
            clearAllVisualReturns("weapon-identity-or-parent-changed", true, true);
            return;
        }

        const auto result = hand_visual_lerp_math::advanceVisualReturn(
            state.localTransition,
            state.nativeBaselineLocal,
            dt,
            hand_visual_lerp_math::VisualReturnConfig{
                .minSeconds = g_rockConfig.rockWeaponVisualReturnTimeMin,
                .maxSeconds = g_rockConfig.rockWeaponVisualReturnTimeMax,
                .minDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMinDistance,
                .maxDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMaxDistance,
                .minAngleDegrees = g_rockConfig.rockWeaponVisualReturnMinAngleDegrees,
                .maxAngleDegrees = g_rockConfig.rockWeaponVisualReturnMaxAngleDegrees,
            });
        if (!isFiniteTransform(result.transform)) {
            clearWeaponVisualReturn("non-finite-return-transform", true, true);
            return;
        }

        const RE::NiTransform returnedWeaponWorld =
            transform_math::composeTransforms(state.nativeParent->world, result.transform);
        if (!applyWeaponVisualAuthority(currentWeaponNode, returnedWeaponWorld, state.weaponGenerationKey)) {
            clearWeaponVisualReturn("weapon-return-publish-failed", true, true);
            return;
        }
        _lastSolvedWeaponTransform = currentWeaponNode->world;
        _hasSolvedWeaponTransform = true;
        if (result.reachedTarget) {
            const float completedDuration = state.localTransition.durationSeconds;
            clearWeaponVisualReturn("completed", false, true);
            ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: weapon return completed duration={:.3f}s", completedDuration);
        }
    }

    void TwoHandedGrip::clearWeaponVisualReturn(const char* reason, const bool logCancellation, const bool restoreBlockers)
    {
        const bool wasActive = _returningWeaponVisual.localTransition.active;
        const bool retainedPrimaryPoseBlocker = _returningWeaponVisual.retainPrimaryPoseBlocker;
        RE::NiNode* returnNode = _returningWeaponVisual.weaponNode;
        _returningWeaponVisual = {};
        if (restoreBlockers) {
            releaseFiringHandWeaponNodeOwnership(returnNode);
            if (retainedPrimaryPoseBlocker) {
                restoreFrikPrimaryWeaponPose();
            }
        }
        if (wasActive && logCancellation) {
            ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: weapon return cancelled reason={}", reason ? reason : "unknown");
        }
    }

    void TwoHandedGrip::clearAllVisualReturns(const char* reason, const bool logCancellation, const bool restoreBlockers)
    {
        clearHandVisualReturn(true, reason, logCancellation);
        clearHandVisualReturn(false, reason, logCancellation);
        clearWeaponVisualReturn(reason, logCancellation, restoreBlockers);
    }

    RE::NiTransform TwoHandedGrip::resolveLockedHandVisualTarget(
        const RE::NiTransform& targetWorld,
        const RE::NiTransform* liveHandWorld,
        float dt,
        LockedHandVisualLerpState& state)
    {
        /*
         * Authored, provider-owned, and visual-only support paths retain their
         * independent external-hand transition. Normal dynamic full-authority
         * acquisition is intercepted by resolveDynamicSupportAcquisitionHandTarget
         * so both hands share the pivot-preserving weapon correction alpha.
         */
        if (!g_rockConfig.rockWeaponSupportGripHandLerpEnabled) {
            state = {};
            return targetWorld;
        }

        if (!state.active) {
            const RE::NiTransform startWorld = (liveHandWorld && isFiniteTransform(*liveHandWorld)) ? *liveHandWorld : targetWorld;
            const float initialDistance =
                hand_visual_lerp_math::distanceGameUnits(startWorld.translate, targetWorld.translate);
            const float durationSeconds =
                hand_visual_lerp_math::computeDistanceMappedDurationGameUnits(
                    initialDistance,
                    g_rockConfig.rockWeaponSupportGripHandLerpTimeMin,
                    g_rockConfig.rockWeaponSupportGripHandLerpTimeMax,
                    g_rockConfig.rockWeaponSupportGripHandLerpMinDistance,
                    g_rockConfig.rockWeaponSupportGripHandLerpMaxDistance);
            if (durationSeconds <= 0.0f) {
                state = {};
                state.lastAlpha = 1.0f;
                return targetWorld;
            }

            state.active = true;
            state.startWorld = startWorld;
            state.elapsedSeconds = 0.0f;
            state.durationSeconds = durationSeconds;
            state.lastAlpha = 0.0f;
        }

        state.elapsedSeconds =
            hand_visual_lerp_math::advanceTimedBlendElapsed(state.elapsedSeconds, dt, state.durationSeconds);
        const auto blended =
            hand_visual_lerp_math::blendTransformOverDuration(state.startWorld, targetWorld, state.elapsedSeconds, state.durationSeconds);
        state.lastAlpha = hand_visual_lerp_math::timedBlendAlpha(state.elapsedSeconds, state.durationSeconds);
        return blended.transform;
    }


    // ---- Weapon and hand authority publication ----

    bool TwoHandedGrip::applyWeaponVisualAuthority(
        RE::NiNode* weaponNode,
        const RE::NiTransform& solvedWeaponWorld,
        const std::uint64_t authorityGenerationKey,
        const bool notifyVisualIntentObserver)
    {
        if (!weaponNode || !isFiniteTransform(weaponNode->world) ||
            !isFiniteTransform(solvedWeaponWorld)) {
            return false;
        }

        const RE::NiTransform scaleStableSolvedWeaponWorld =
            weapon_visual_authority_math::preserveLiveWeaponWorldScale(
                weaponNode->world,
                solvedWeaponWorld);

        const std::uint64_t effectiveGenerationKey = authorityGenerationKey != 0 ? authorityGenerationKey : _activeWeaponGenerationKey;
        if (notifyVisualIntentObserver && _weaponVisualIntentObserver) {
            _weaponVisualIntentObserver(
                _weaponVisualIntentObserverContext,
                weaponNode,
                scaleStableSolvedWeaponWorld,
                effectiveGenerationKey);
        }
        const bool scopeAnchorMatchesAuthority =
            _nativeScopeAnchorValid && _nativeScopeAnchorWeaponNode == weaponNode && _nativeScopeAnchorGenerationKey == effectiveGenerationKey;

        // Capture hFRIK's engine-specific camera axis only before changing the
        // weapon. Once captured, every hand mode resolves the same immutable
        // generation-bound weapon-local scope frame.
        const NativeScopeCameraFollowCapture scopeCameraFollow = captureNativeScopeCameraFollow(weaponNode);
        if (scopeAnchorMatchesAuthority && scopeCameraFollow.valid) {
            (void)captureNativeScopeRigidFrame(weaponNode, effectiveGenerationKey, scopeCameraFollow.camera, scopeCameraFollow.cameraWorldBefore);
            (void)captureNativeScopeOverlayCalibration(scopeCameraFollow.cameraWorldBefore, effectiveGenerationKey);
        }

        if (!moveWeaponPresentationRigidly(
                weaponNode,
                scaleStableSolvedWeaponWorld)) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                2000,
                "TwoHandedGrip: rejected weapon visual authority because the root or bounded presentation subtree was invalid");
            return false;
        }

        const bool rigidFrameMatchesAuthority = _nativeScopeRigidFrame.valid && _nativeScopeRigidFrame.weaponGenerationKey == effectiveGenerationKey &&
            _nativeScopeRigidFrame.weaponNodeIdentity == weaponNode && _nativeScopeRigidFrame.scopeCameraIdentity == scopeCameraFollow.camera;
        const bool scopeTargetReady =
            rigidFrameMatchesAuthority &&
            isFiniteTransform(
                _nativeScopeRigidFrame.cameraWeaponLocal) &&
            std::abs(_nativeScopeRigidFrame.cameraWeaponLocal.scale) >
                0.0001f;
        const NativeScopeCameraFollowResult scopeCameraResult =
            scopeTargetReady ?
                applyNativeScopeCameraWorldTarget(
                    scopeCameraFollow,
                    native_scope_camera_follow_math::
                        resolveRigidAnchorFrameWorld(
                            weaponNode->world,
                            _nativeScopeRigidFrame.cameraWeaponLocal)) :
                NativeScopeCameraFollowResult{};
        if (scopeCameraResult.targetValid && scopeCameraResult.writeApplied) {
            (void)applyNativeScopeOverlayTarget(scopeCameraResult.targetCameraWorld, effectiveGenerationKey);
        }
        if (g_rockConfig.rockDebugDrawNativeScopeActivation &&
            scopeTargetReady &&
            scopeCameraResult.immediateReadbackValid) {
            const RE::NiTransform immediateCameraWeaponLocal =
                transform_math::composeTransforms(
                    transform_math::invertTransform(weaponNode->world),
                    scopeCameraResult.immediateCameraWorldAfter);
            const float rigidPositionError =
                hand_visual_lerp_math::distanceGameUnits(
                    immediateCameraWeaponLocal.translate,
                    _nativeScopeRigidFrame.cameraWeaponLocal.translate);
            const float rigidRotationError =
                hand_visual_lerp_math::rotationDistanceDegrees(
                    _nativeScopeRigidFrame.cameraWeaponLocal,
                    immediateCameraWeaponLocal);
            ROCK_LOG_SAMPLE_DEBUG(
                Weapon,
                2000,
                "TwoHandedGrip: native scope retained weapon-local frame relationError=({:.4f}gu,{:.3f}deg) generation={:016X}",
                rigidPositionError,
                rigidRotationError,
                effectiveGenerationKey);
        }
        _lastRenderedWeaponWorld = weaponNode->world;
        _hasLastRenderedWeaponWorld = isFiniteTransform(_lastRenderedWeaponWorld);
        if (g_rockConfig.rockDebugDrawNativeScopeActivation) {
            _nativeScopeCameraDebugSnapshot = makeNativeScopeCameraDebugSnapshot(_nativeScopeCameraDebugSnapshot, effectiveGenerationKey,
                NativeScopeCameraWriteSource::WeaponVisualAuthority, scopeCameraFollow, scopeCameraResult,
                scopeTargetReady ? _nativeScopeAnchorSource : native_scope_sight_anchor_policy::AnchorSource::None);
        }
        return true;
    }

    bool TwoHandedGrip::rebaseWeaponLocalForDeferredParentHandTarget(
        RE::NiNode* weaponNode,
        const bool isLeft,
        const RE::NiTransform& deferredHandWorld)
    {
        if (!weaponNode || !isFiniteTransform(weaponNode->world) ||
            !isUsableHandAuthorityTransform(deferredHandWorld)) {
            return false;
        }

        auto* const handNode = resolveFirstPersonHandNode(isLeft);
        if (!handNode || weaponNode->parent != handNode) {
            return true;
        }

        const RE::NiTransform deferredParentLocal =
            weapon_visual_authority_math::worldTargetToParentLocal(
                deferredHandWorld,
                weaponNode->world);
        if (!isFiniteTransform(deferredParentLocal)) {
            return false;
        }

        /*
         * FRIK V2 consumes the hand target on its next skeleton frame. The
         * equipped weapon is normally a child of RArm_Hand, so a local derived
         * from the hand's current world makes that later parent update carry
         * the weapon a second time. Store the local against the deferred parent
         * frame while retaining the already-solved weapon world for this frame.
         */
        weaponNode->local = deferredParentLocal;
        return true;
    }

    bool TwoHandedGrip::clearWeaponCollisionHandAuthority(const bool isLeft)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        _preFrikWeaponHandAuthority[index] = {};
        if (!_weaponCollisionHandAuthorityLive[index]) {
            _weaponCollisionHandAuthorityGenerationKey[index] = 0;
            return true;
        }
        if (!frik_visual_authority::isAvailable() ||
            !frik_visual_authority::clearExternalHandWorldTransform(
                WEAPON_COLLISION_HAND_TAG,
                handFromBool(isLeft))) {
            return false;
        }
        _weaponCollisionHandAuthorityLive[index] = false;
        _weaponCollisionHandAuthorityGenerationKey[index] = 0;
        return true;
    }

    void TwoHandedGrip::suspendNativeReloadSupportHandAuthority(
        const bool isLeft)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        (void)frik_visual_authority::clearHandPose(
            SUPPORT_GRIP_TAG,
            handFromBool(isLeft));
        (void)frik_visual_authority::clearHandPose(
            PRIMARY_GRIP_TAG,
            handFromBool(isLeft));
        clearPreFrikRetainedHandAuthority(
            RetainedHandAuthorityKind::SupportGrip,
            isLeft);
        (void)clearHandAuthorityRoleNow(
            scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip,
            isLeft);
        (void)clearHandAuthorityRoleNow(
            scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip,
            isLeft);
        (void)clearHandAuthorityRoleNow(
            scope_safe_hand_frame_math::HandAuthorityRole::PrimaryDetach,
            isLeft);
        (void)clearWeaponCollisionHandAuthority(isLeft);
        clearHandVisualReturn(isLeft, "native-reload-authority", false);
        _weaponCollisionBaselineHandWorldValid[index] = false;
        _weaponCollisionHandPresentationFromPreviousFrame[index] = false;
        _hasLastPublishedHandWorld[index] = false;

        // Gunstock publishes both participating hands as one rigid group.
        // Yield the complete group so it cannot immediately reacquire the
        // support hand after the role-specific clears above.
        clearGunstockDedicatedHandAuthority();
    }

    void TwoHandedGrip::setNativeReloadHandAuthorityActive(
        const bool active)
    {
        const bool supportHandIsLeft = !_firingHandIsLeft;
        const bool supportRoleChanged =
            _nativeReloadHandAuthorityActive &&
            _nativeReloadSupportHandIsLeft != supportHandIsLeft;

        if (!active) {
            if (_nativeReloadHandAuthorityActive) {
                ROCK_LOG_INFO(
                    Weapon,
                    "TwoHandedGrip: native reload released support-hand FRIK authority hand={}",
                    _nativeReloadSupportHandIsLeft ? "left" : "right");
            }
            _nativeReloadHandAuthorityActive = false;
            _nativeReloadSupportHandIsLeft = supportHandIsLeft;
            return;
        }

        if (_nativeReloadHandAuthorityActive && !supportRoleChanged) {
            return;
        }

        if (supportRoleChanged) {
            suspendNativeReloadSupportHandAuthority(
                _nativeReloadSupportHandIsLeft);
        }
        clearFiringRecoilPresentationState();
        _nativeReloadHandAuthorityActive = true;
        _nativeReloadSupportHandIsLeft = supportHandIsLeft;
        suspendNativeReloadSupportHandAuthority(supportHandIsLeft);
        ROCK_LOG_INFO(
            Weapon,
            "TwoHandedGrip: native reload suspended support-hand FRIK authority hand={} logicalGripRetained={}",
            supportHandIsLeft ? "left" : "right",
            partGrip(supportHandIsLeft).active ? "yes" : "no");
    }

    void TwoHandedGrip::refreshWeaponCollisionHandAuthorityBeforeFrik(
        const EquippedWeaponScopeHandDriverFrame& leftHandDriver,
        const EquippedWeaponScopeHandDriverFrame& rightHandDriver,
        const std::uint64_t currentWeaponGenerationKey,
        const bool firingHandIsLeft,
        const std::uint64_t currentSchedulerSequence)
    {
        const std::array<EquippedWeaponScopeHandDriverFrame, 2> drivers{
            leftHandDriver,
            rightHandDriver,
        };
        for (std::size_t index = 0;
             index < _preFrikWeaponHandAuthority.size();
             ++index) {
            const bool isLeft = index == 0u;
            if (isNativeReloadSupportHand(isLeft)) {
                (void)clearWeaponCollisionHandAuthority(isLeft);
                continue;
            }
            const auto& source = _preFrikWeaponHandAuthority[index];
            const auto& driver = drivers[index];
            const bool sourceCurrent =
                source.valid &&
                _weaponCollisionHandAuthorityLive[index] &&
                driver.valid &&
                currentWeaponGenerationKey != 0 &&
                source.weaponGenerationKey == currentWeaponGenerationKey &&
                source.firingHandIsLeft == firingHandIsLeft &&
                prefrik_hand_authority_policy::isImmediateSuccessor(
                    source.sourceSchedulerSequence,
                    currentSchedulerSequence) &&
                prefrik_hand_authority_policy::isUsableTransform(
                    driver.world) &&
                prefrik_hand_authority_policy::isUsableTransform(
                    source.driverToHandLocal);
            if (!sourceCurrent) {
                if (_weaponCollisionHandAuthorityLive[index] || source.valid) {
                    (void)clearWeaponCollisionHandAuthority(isLeft);
                }
                continue;
            }

            const RE::NiTransform targetWorld =
                prefrik_hand_authority_policy::reconstructTargetWorld(
                    driver.world,
                    source.driverToHandLocal);
            if (!prefrik_hand_authority_policy::isUsableTransform(
                    targetWorld) ||
                !frik_visual_authority::applyExternalHandWorldTransform(
                    WEAPON_COLLISION_HAND_TAG,
                    handFromBool(isLeft),
                    targetWorld,
                    WEAPON_COLLISION_HAND_PRIORITY)) {
                (void)clearWeaponCollisionHandAuthority(isLeft);
            }
        }
    }

    void TwoHandedGrip::recordPreFrikRetainedHandAuthority(
        const RetainedHandAuthorityKind kind,
        const bool isLeft,
        const RE::NiTransform& targetWorld)
    {
        const std::size_t handIndex = isLeft ? 0u : 1u;
        const std::size_t kindIndex = static_cast<std::size_t>(kind);
        auto& source =
            _preFrikRetainedHandAuthorities[handIndex][kindIndex];
        source = {};

        const auto& driver = _currentHandDriverFrames[handIndex];
        const bool generationRequired =
            kind != RetainedHandAuthorityKind::Return;
        if (!driver.valid ||
            _currentSourceSchedulerSequence == 0 ||
            (generationRequired && _activeWeaponGenerationKey == 0) ||
            !prefrik_hand_authority_policy::isUsableTransform(driver.world) ||
            !prefrik_hand_authority_policy::isUsableTransform(targetWorld)) {
            return;
        }

        source.driverToHandLocal =
            prefrik_hand_authority_policy::captureDriverToTargetLocal(
                driver.world,
                targetWorld);
        source.weaponGenerationKey = generationRequired ?
            _activeWeaponGenerationKey :
            0;
        source.sourceSchedulerSequence =
            _currentSourceSchedulerSequence;
        source.firingHandIsLeft = _firingHandIsLeft;
        source.valid =
            prefrik_hand_authority_policy::isUsableTransform(
                source.driverToHandLocal);
    }

    void TwoHandedGrip::clearPreFrikRetainedHandAuthority(
        const RetainedHandAuthorityKind kind,
        const bool isLeft)
    {
        _preFrikRetainedHandAuthorities[isLeft ? 0u : 1u]
            [static_cast<std::size_t>(kind)] = {};
    }

    void TwoHandedGrip::refreshRetainedHandVisualAuthoritiesBeforeFrik(
        const EquippedWeaponScopeHandDriverFrame& leftHandDriver,
        const EquippedWeaponScopeHandDriverFrame& rightHandDriver,
        const std::uint64_t currentWeaponGenerationKey,
        const bool firingHandIsLeft,
        const std::uint64_t currentSchedulerSequence)
    {
        const std::array<EquippedWeaponScopeHandDriverFrame, 2> drivers{
            leftHandDriver,
            rightHandDriver,
        };
        for (std::size_t handIndex = 0; handIndex < 2; ++handIndex) {
            const bool isLeft = handIndex == 0u;
            if (isNativeReloadSupportHand(isLeft)) {
                continue;
            }
            for (std::size_t kindIndex = 0;
                 kindIndex < kRetainedHandAuthorityKindCount;
                 ++kindIndex) {
                const auto kind =
                    static_cast<RetainedHandAuthorityKind>(kindIndex);
                const char* tag = nullptr;
                int priority = GRIP_HAND_POSE_PRIORITY;
                switch (kind) {
                case RetainedHandAuthorityKind::PrimaryGrip:
                    tag = PRIMARY_GRIP_TAG;
                    break;
                case RetainedHandAuthorityKind::SupportGrip:
                    tag = SUPPORT_GRIP_TAG;
                    break;
                case RetainedHandAuthorityKind::GunstockAlignment:
                    tag = GUNSTOCK_ALIGNMENT_TAG;
                    break;
                case RetainedHandAuthorityKind::Return:
                    tag = RETURN_HAND_TAG;
                    priority = RETURN_HAND_VISUAL_PRIORITY;
                    break;
                case RetainedHandAuthorityKind::Count:
                    continue;
                }

                auto& source =
                    _preFrikRetainedHandAuthorities[handIndex][kindIndex];
                const auto hand = handFromBool(isLeft);
                if (!frik_visual_authority::
                        hasPublishedExternalHandWorldTransform(tag, hand)) {
                    source = {};
                    continue;
                }

                const bool generationRequired =
                    kind != RetainedHandAuthorityKind::Return;
                const auto& driver = drivers[handIndex];
                const bool sourceCurrent =
                    source.valid &&
                    driver.valid &&
                    prefrik_hand_authority_policy::isImmediateSuccessor(
                        source.sourceSchedulerSequence,
                        currentSchedulerSequence) &&
                    (!generationRequired ||
                        (currentWeaponGenerationKey != 0 &&
                            source.weaponGenerationKey ==
                                currentWeaponGenerationKey &&
                            source.firingHandIsLeft == firingHandIsLeft)) &&
                    prefrik_hand_authority_policy::isUsableTransform(
                        driver.world) &&
                    prefrik_hand_authority_policy::isUsableTransform(
                        source.driverToHandLocal);
                const RE::NiTransform refreshedHandWorld = sourceCurrent ?
                    prefrik_hand_authority_policy::reconstructTargetWorld(
                        driver.world,
                        source.driverToHandLocal) :
                    RE::NiTransform{};
                if (!sourceCurrent ||
                    !prefrik_hand_authority_policy::isUsableTransform(
                        refreshedHandWorld) ||
                    !frik_visual_authority::applyExternalHandWorldTransform(
                        tag,
                        hand,
                        refreshedHandWorld,
                        priority)) {
                    (void)frik_visual_authority::
                        clearExternalHandWorldTransform(tag, hand);
                    source = {};
                }
            }
        }
    }

    void TwoHandedGrip::captureFiringRecoilReferenceBeforeFrik(
        const EquippedWeaponScopeHandDriverFrame& firingHandDriver,
        const std::uint64_t currentWeaponGenerationKey,
        const bool firingHandIsLeft,
        const std::uint64_t currentSchedulerSequence)
    {
        const std::size_t handIndex = firingHandIsLeft ? 0u : 1u;
        RE::NiNode* recoilWeaponNode = nullptr;
        std::uint64_t recoilWeaponGenerationKey = 0;
        if (currentSchedulerSequence == 0 ||
            firingHandIsLeft != _firingHandIsLeft ||
            currentWeaponGenerationKey == 0 ||
            !tryResolveControlledFiringRecoilSource(
                firingHandIsLeft,
                recoilWeaponNode,
                recoilWeaponGenerationKey) ||
            currentWeaponGenerationKey != recoilWeaponGenerationKey) {
            _firingRecoilReferenceSchedulerSequence[handIndex] = 0;
            _firingRecoilReferenceCapturedBeforeFrik[handIndex] = false;
            _hasFiringRecoilReference[handIndex] = false;
            return;
        }

        const auto hand = handFromBool(firingHandIsLeft);
        RE::NiTransform preRecoilHandWorld{};
        const bool hasPublishedTarget =
            frik_visual_authority::
                hasPublishedExternalHandWorldTransform(hand);
        const bool capturedPublishedTarget =
            hasPublishedTarget &&
            frik_visual_authority::
                tryGetPublishedExternalHandWorldTarget(
                    hand,
                    preRecoilHandWorld);
        const bool capturedControllerTarget =
            !hasPublishedTarget &&
            firingHandDriver.valid &&
            isUsableHandAuthorityTransform(firingHandDriver.world);
        if (capturedControllerTarget) {
            preRecoilHandWorld = firingHandDriver.world;
        }
        if ((!capturedPublishedTarget && !capturedControllerTarget) ||
            !isUsableHandAuthorityTransform(preRecoilHandWorld)) {
            _firingRecoilReferenceSchedulerSequence[handIndex] =
                currentSchedulerSequence;
            _firingRecoilReferenceCapturedBeforeFrik[handIndex] = true;
            _hasFiringRecoilReference[handIndex] = false;
            return;
        }

        _firingRecoilReferenceHandWorld[handIndex] = preRecoilHandWorld;
        _firingRecoilReferenceGenerationKey[handIndex] =
            recoilWeaponGenerationKey;
        _firingRecoilReferenceSchedulerSequence[handIndex] =
            currentSchedulerSequence;
        _firingRecoilReferenceCapturedBeforeFrik[handIndex] = true;
        _hasFiringRecoilReference[handIndex] = true;
    }

    void TwoHandedGrip::beginWeaponCollisionPresentationFrame(
        const std::uint64_t currentWeaponGenerationKey)
    {
        _weaponCollisionHandPresentationFromPreviousFrame =
            _weaponCollisionHandAuthorityLive;
        _weaponCollisionBaselineHandWorldValid = {};

        /*
         * FRIK V2 owns tagged transforms as persistent claims and consumes the
         * winning claim in its next regular skeleton update. Clearing and then
         * recreating this tag every ROCK frame opened a scheduler-dependent
         * window in which FRIK could select the lower-priority grip claim. Keep
         * the same owner registered and update its value after post-solve.
         * A weapon-generation edge is different: FRIK already consumed the old
         * claim for this frame, so preserve the previous-presentation witness
         * but retire the claim before the replacement weapon can publish.
         */
        bool generationClaimsCleared = true;
        for (std::size_t index = 0; index < _weaponCollisionHandAuthorityLive.size(); ++index) {
            if (!_weaponCollisionHandAuthorityLive[index] ||
                (currentWeaponGenerationKey != 0 &&
                    _weaponCollisionHandAuthorityGenerationKey[index] == currentWeaponGenerationKey)) {
                continue;
            }
            generationClaimsCleared =
                clearWeaponCollisionHandAuthority(index == 0u) &&
                generationClaimsCleared;
        }
        if (!generationClaimsCleared) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "TwoHandedGrip: stale dynamic weapon collision hand authority clear failed generation={:016X} live(L/R)={}/{}",
                currentWeaponGenerationKey,
                _weaponCollisionHandAuthorityLive[0],
                _weaponCollisionHandAuthorityLive[1]);
        }
    }

    void TwoHandedGrip::finishWeaponCollisionPresentationFrame(
        const bool presentationActive)
    {
        if (presentationActive) {
            return;
        }

        const bool leftCleared = clearWeaponCollisionHandAuthority(true);
        const bool rightCleared = clearWeaponCollisionHandAuthority(false);
        if (!leftCleared || !rightCleared) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "TwoHandedGrip: inactive dynamic weapon contact presentation hand authority clear failed left={} right={} live(L/R)={}/{}",
                leftCleared ? "ok" : "failed",
                rightCleared ? "ok" : "failed",
                _weaponCollisionHandAuthorityLive[0],
                _weaponCollisionHandAuthorityLive[1]);
        }
    }

    bool TwoHandedGrip::applyFiringWeaponRecoilPresentation(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey)
    {
        const std::uint64_t acceptedSequence =
            _firingRecoilAcceptedSequence;
        if (acceptedSequence == 0 ||
            acceptedSequence == _firingRecoilConsumedSequence) {
            if (_currentSourceSchedulerSequence != 0 &&
                _firingRecoilCallbackSchedulerSequence ==
                    _currentSourceSchedulerSequence &&
                (_firingRecoilCallbackDecision ==
                        FiringRecoilCallbackDecision::SourceUnavailable ||
                    _firingRecoilCallbackDecision ==
                        FiringRecoilCallbackDecision::ReferenceUnavailable)) {
                ROCK_LOG_SAMPLE_DEBUG(
                    Weapon,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "TwoHandedGrip: firing recoil rejected before ticket decision={} hand={} currentGeneration={:016X} activeGeneration={:016X} canonicalGeneration={:016X} reference(valid/generation/sequence)={}/{:016X}/{} scheduler={}",
                    static_cast<unsigned>(_firingRecoilCallbackDecision),
                    _firingHandIsLeft ? "left" : "right",
                    currentWeaponGenerationKey,
                    _activeWeaponGenerationKey,
                    _rightFiringHandCanonicalGenerationKey,
                    _hasFiringRecoilReference[_firingHandIsLeft ? 0u : 1u],
                    _firingRecoilReferenceGenerationKey[
                        _firingHandIsLeft ? 0u : 1u],
                    _firingRecoilReferenceSchedulerSequence[
                        _firingHandIsLeft ? 0u : 1u],
                    _currentSourceSchedulerSequence);
            }
            return false;
        }

        // Consume first so a failed/stale sample can never kick a later weapon.
        _firingRecoilConsumedSequence = acceptedSequence;
        const bool acceptedHandIsLeft =
            _firingRecoilAcceptedHandIsLeft;
        const std::size_t acceptedHandIndex =
            acceptedHandIsLeft ? 0u : 1u;
        RE::NiNode* recoilWeaponNode = nullptr;
        std::uint64_t recoilWeaponGenerationKey = 0;
        const auto rejectTicket = [&](const char* const reason) {
            ROCK_LOG_SAMPLE_DEBUG(
                Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "TwoHandedGrip: firing recoil ticket rejected reason={} hand={} acceptedGeneration={:016X} currentGeneration={:016X} sourceGeneration={:016X} nodeMatch={} reference(valid/generation/preFrik/sequence)={}/{:016X}/{}/{} scheduler={}",
                reason,
                acceptedHandIsLeft ? "left" : "right",
                _firingRecoilAcceptedGenerationKey,
                currentWeaponGenerationKey,
                recoilWeaponGenerationKey,
                weaponNode && weaponNode == recoilWeaponNode,
                _hasFiringRecoilReference[acceptedHandIndex],
                _firingRecoilReferenceGenerationKey[acceptedHandIndex],
                _firingRecoilReferenceCapturedBeforeFrik[acceptedHandIndex],
                _firingRecoilReferenceSchedulerSequence[acceptedHandIndex],
                _currentSourceSchedulerSequence);
            return false;
        };
        if (!weaponNode) {
            return rejectTicket("weapon-missing");
        }
        if (currentWeaponGenerationKey == 0) {
            return rejectTicket("generation-missing");
        }
        if (!tryResolveControlledFiringRecoilSource(
                acceptedHandIsLeft,
                recoilWeaponNode,
                recoilWeaponGenerationKey)) {
            return rejectTicket("source-unavailable");
        }
        if (weaponNode != recoilWeaponNode) {
            return rejectTicket("weapon-node-mismatch");
        }
        if (currentWeaponGenerationKey != recoilWeaponGenerationKey ||
            currentWeaponGenerationKey !=
                _firingRecoilAcceptedGenerationKey) {
            return rejectTicket("generation-mismatch");
        }
        if (acceptedHandIsLeft != _firingHandIsLeft) {
            return rejectTicket("firing-hand-changed");
        }
        if (!_hasFiringRecoilReference[acceptedHandIndex] ||
            _firingRecoilReferenceGenerationKey[acceptedHandIndex] !=
                currentWeaponGenerationKey ||
            !_firingRecoilReferenceCapturedBeforeFrik[acceptedHandIndex]) {
            return rejectTicket("reference-invalid");
        }
        if (_currentSourceSchedulerSequence == 0 ||
            _firingRecoilReferenceSchedulerSequence[acceptedHandIndex] !=
                _currentSourceSchedulerSequence) {
            return rejectTicket("scheduler-mismatch");
        }
        if (!isFiniteTransform(weaponNode->world)) {
            return rejectTicket("weapon-transform-invalid");
        }

        RE::NiTransform presentedFiringHandWorld{};
        if (!tryGetRootFlattenedHandBoneTransform(
                acceptedHandIsLeft,
                presentedFiringHandWorld)) {
            return rejectTicket("presented-hand-unavailable");
        }

        const RE::NiTransform recoilWorldDelta =
            gunstock_alignment_policy::deriveAppliedWorldDelta(
                _firingRecoilReferenceHandWorld[acceptedHandIndex],
                presentedFiringHandWorld);
        if (!isUsableHandAuthorityTransform(recoilWorldDelta)) {
            return rejectTicket("recoil-delta-invalid");
        }

        const RE::NiTransform recoiledWeaponWorld =
            transform_math::composeTransforms(
                recoilWorldDelta,
                weaponNode->world);
        if (!isFiniteTransform(recoiledWeaponWorld)) {
            return rejectTicket("recoiled-weapon-invalid");
        }

        // This is a terminal presentation overlay, not new collision intent.
        // The collision bodies and muzzle sample the resulting weapon below.
        const bool applied = applyWeaponVisualAuthority(
            weaponNode,
            recoiledWeaponWorld,
            currentWeaponGenerationKey,
            false);
        ROCK_LOG_SAMPLE_DEBUG(
            Weapon,
            g_rockConfig.rockLogSampleMilliseconds,
            "TwoHandedGrip: firing recoil weapon publication {} hand={} generation={:016X} deltaT=({:.4f},{:.4f},{:.4f}) scheduler={}",
            applied ? "applied" : "failed",
            acceptedHandIsLeft ? "left" : "right",
            currentWeaponGenerationKey,
            recoilWorldDelta.translate.x,
            recoilWorldDelta.translate.y,
            recoilWorldDelta.translate.z,
            _currentSourceSchedulerSequence);
        return applied;
    }

    bool TwoHandedGrip::applyWeaponCollisionResolvedAuthority(
        RE::NiNode* weaponNode,
        const RE::NiTransform& requestedWeaponWorld,
        const RE::NiTransform& resolvedWeaponWorld,
        const std::uint64_t authorityGenerationKey)
    {
        if (!weaponNode ||
            !isFiniteTransform(weaponNode->world) ||
            !isFiniteTransform(requestedWeaponWorld) ||
            !isFiniteTransform(resolvedWeaponWorld)) {
            return false;
        }


        const RE::NiTransform scaleStableRequestedWeaponWorld =
            weapon_visual_authority_math::preserveLiveWeaponWorldScale(
                weaponNode->world,
                requestedWeaponWorld);
        const RE::NiTransform scaleStableResolvedWeaponWorld =
            weapon_visual_authority_math::preserveLiveWeaponWorldScale(
                weaponNode->world,
                resolvedWeaponWorld);

        const auto attachedHands = weaponCollisionAttachedHands();

        struct CollisionHandPulse
        {
            RE::NiTransform targetWorld{};
            bool isLeft{ false };
            bool requested{ false };
            bool targetValid{ false };
            bool applied{ false };
            bool retained{ false };
        };
        std::array<CollisionHandPulse, 2> pulses{
            CollisionHandPulse{
                .isLeft = true,
                .requested = attachedHands.left,
            },
            CollisionHandPulse{
                .isLeft = false,
                .requested = attachedHands.right,
            },
        };

        bool detachedHandsCleared = true;
        for (const auto& pulse : pulses) {
            if (!pulse.requested) {
                detachedHandsCleared =
                    clearWeaponCollisionHandAuthority(pulse.isLeft) &&
                    detachedHandsCleared;
            }
        }

        const bool anyHandRequested = attachedHands.left || attachedHands.right;
        bool handTargetsReady =
            !anyHandRequested || frik_visual_authority::isAvailable();
        for (auto& pulse : pulses) {
            if (!pulse.requested) {
                continue;
            }
            /*
             * A locked firing/support pose was already published earlier in
             * this ROCK frame. It is the exact collision-free hand baseline;
             * replacing it with controller input at this higher priority makes
             * the hand slide off the part and feeds equip-time rotation back
             * through FRIK. Native passive carry has no ROCK hand publication,
             * so it deliberately falls back to the scope-safe physical input.
             * The weapon basis remains the explicit pre-physics intent because
             * weaponNode->world can contain the previous deferred claim.
             */
            const std::size_t handIndex = pulse.isLeft ? 0u : 1u;
            RE::NiTransform collisionFreeHandWorld{};
            bool collisionFreeHandValid = false;
            if (_weaponCollisionBaselineHandWorldValid[handIndex]) {
                collisionFreeHandWorld =
                    _weaponCollisionBaselineHandWorld[handIndex];
                collisionFreeHandValid =
                    isUsableHandAuthorityTransform(collisionFreeHandWorld);
            } else {
                collisionFreeHandValid =
                    tryGetSolverHandTransform(
                        pulse.isLeft,
                        collisionFreeHandWorld) &&
                    isUsableHandAuthorityTransform(collisionFreeHandWorld);
            }
            pulse.targetWorld =
                dynamic_weapon_collision_policy::reframeAttachedHand(
                    scaleStableRequestedWeaponWorld,
                    scaleStableResolvedWeaponWorld,
                    collisionFreeHandWorld);
            pulse.targetValid =
                collisionFreeHandValid &&
                isUsableHandAuthorityTransform(pulse.targetWorld);
            handTargetsReady = handTargetsReady && pulse.targetValid;
        }

        bool handPulsesSucceeded = handTargetsReady && detachedHandsCleared;
        if (handTargetsReady) {
            for (auto& pulse : pulses) {
                if (!pulse.requested) {
                    continue;
                }
                const auto hand = handFromBool(pulse.isLeft);
                pulse.applied =
                    frik_visual_authority::applyExternalHandWorldTransform(
                        WEAPON_COLLISION_HAND_TAG,
                        hand,
                        pulse.targetWorld,
                        WEAPON_COLLISION_HAND_PRIORITY);
                /*
                 * Update the same high-priority owner in place. Clearing it at
                 * either frame boundary would let FRIK's regular solve select
                 * the live priority-100 firing/support owner for one scheduling
                 * interval. Each physical hand remains reconstructed from its
                 * own unaffected driver while this claim is live.
                 */
                pulse.retained = pulse.applied;
                if (pulse.retained) {
                    const std::size_t handIndex = pulse.isLeft ? 0u : 1u;
                    _weaponCollisionHandAuthorityLive[handIndex] = true;
                    _weaponCollisionHandAuthorityGenerationKey[handIndex] =
                        authorityGenerationKey;
                    auto& preFrikSource =
                        _preFrikWeaponHandAuthority[handIndex];
                    preFrikSource = {};
                    const auto& driver =
                        _currentHandDriverFrames[handIndex];
                    if (driver.valid &&
                        _currentSourceSchedulerSequence != 0 &&
                        authorityGenerationKey != 0 &&
                        prefrik_hand_authority_policy::isUsableTransform(
                            driver.world)) {
                        preFrikSource.driverToHandLocal =
                            prefrik_hand_authority_policy::
                                captureDriverToTargetLocal(
                                    driver.world,
                                    pulse.targetWorld);
                        preFrikSource.weaponGenerationKey =
                            authorityGenerationKey;
                        preFrikSource.sourceSchedulerSequence =
                            _currentSourceSchedulerSequence;
                        preFrikSource.firingHandIsLeft = _firingHandIsLeft;
                        preFrikSource.valid =
                            prefrik_hand_authority_policy::isUsableTransform(
                                preFrikSource.driverToHandLocal);
                    }
                }
                handPulsesSucceeded =
                    handPulsesSucceeded && pulse.applied && pulse.retained;
            }
        }

        /*
         * A firing-hand pulse can propagate through the weapon's native parent
         * chain. Publish the solver-authoritative weapon last so the final
         * rendered weapon pose is exact while both hands keep the rigid
         * pre-collision weapon-local relationship captured above.
         */
        const bool weaponPublished = applyWeaponVisualAuthority(
            weaponNode,
            scaleStableResolvedWeaponWorld,
            authorityGenerationKey,
            false);
        bool deferredParentRebased = weaponPublished;
        if (weaponPublished) {
            for (const auto& pulse : pulses) {
                if (!pulse.requested || !pulse.applied) {
                    continue;
                }
                const bool pulseParentRebased =
                    rebaseWeaponLocalForDeferredParentHandTarget(
                        weaponNode,
                        pulse.isLeft,
                        pulse.targetWorld);
                if (!pulseParentRebased) {
                    (void)clearWeaponCollisionHandAuthority(pulse.isLeft);
                }
                deferredParentRebased =
                    deferredParentRebased && pulseParentRebased;
            }
        }
        handPulsesSucceeded = handPulsesSucceeded && deferredParentRebased;
        if (!weaponPublished || !handPulsesSucceeded) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "TwoHandedGrip: dynamic weapon collision group publication incomplete weapon={} hands={} left(req/target/apply/live)={}/{}/{}/{} right(req/target/apply/live)={}/{}/{}/{} state={} firingHand={}",
                weaponPublished ? "ok" : "failed",
                handPulsesSucceeded ? "ok" : "failed",
                pulses[0].requested,
                pulses[0].targetValid,
                pulses[0].applied,
                pulses[0].retained,
                pulses[1].requested,
                pulses[1].targetValid,
                pulses[1].applied,
                pulses[1].retained,
                static_cast<int>(_state),
                _firingHandIsLeft ? "left" : "right");
        }
        return weaponPublished && handPulsesSucceeded;
    }

    bool TwoHandedGrip::applyFiringHandLockedVisual(RE::NiNode* weaponNode, float dt, const RE::NiTransform* liveHandWorld)
    {
        if (!weaponNode || !_hasFiringHandWeaponLocal) {
            return false;
        }
        if (!scope_safe_hand_frame_math::shouldPublishLockedHandVisualAuthority(_scopeMenuOpenThisFrame)) {
            return true;
        }
        if (!frik_visual_authority::isAvailable()) {
            return false;
        }

        const RE::NiTransform firingHandWorld =
            weapon_visual_authority_math::weaponLocalFrameToWorld(weaponNode->world, _primaryHandWeaponLocal);
        const auto& returningHand = _returningHandVisuals[_firingHandIsLeft ? 0u : 1u].transition;
        const RE::NiTransform* acquisitionStart =
            returningHand.active && isUsableHandAuthorityTransform(returningHand.lastApplied) ?
            &returningHand.lastApplied :
            liveHandWorld;
        const bool synchronizedDynamicAcquisition =
            dynamicSupportAcquisitionMatches(
                !_firingHandIsLeft,
                partGrip(!_firingHandIsLeft));
        const RE::NiTransform appliedFiringHandWorld =
            synchronizedDynamicAcquisition ?
            resolveDynamicSupportAcquisitionHandTarget(
                firingHandWorld,
                true,
                _primaryHandVisualLerp) :
            resolveLockedHandVisualTarget(
                firingHandWorld,
                acquisitionStart,
                dt,
                _primaryHandVisualLerp);
        (void)publishAuthoredPrimaryFiringGripFingerPose(_firingHandIsLeft);
        const bool applied = frik_visual_authority::applyExternalHandWorldTransform(
            PRIMARY_GRIP_TAG, handFromBool(_firingHandIsLeft), appliedFiringHandWorld, GRIP_HAND_POSE_PRIORITY);
        if (applied &&
            !rebaseWeaponLocalForDeferredParentHandTarget(
                weaponNode,
                _firingHandIsLeft,
                appliedFiringHandWorld)) {
            (void)frik_visual_authority::clearExternalHandWorldTransform(
                PRIMARY_GRIP_TAG,
                handFromBool(_firingHandIsLeft));
            return false;
        }
        if (applied) {
            recordPreFrikRetainedHandAuthority(
                RetainedHandAuthorityKind::PrimaryGrip,
                _firingHandIsLeft,
                appliedFiringHandWorld);
            recordScopeHandAuthorityPublication(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip, _firingHandIsLeft);
            clearHandVisualReturn(_firingHandIsLeft, "firing-grip-authority-acquired", false);
            recordPublishedHandWorld(_firingHandIsLeft, appliedFiringHandWorld);
        }
        return applied;
    }

    bool TwoHandedGrip::applyPartGripLockedVisual(bool isLeft, RE::NiNode* weaponNode, float dt, const RE::NiTransform* liveHandWorld)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        if (!weaponNode || !grip.active || !grip.hasHandWeaponLocal) {
            return false;
        }
        if (isNativeReloadSupportHand(isLeft)) {
            return true;
        }
        if (!scope_safe_hand_frame_math::shouldPublishLockedHandVisualAuthority(_scopeMenuOpenThisFrame)) {
            return true;
        }
        if (!frik_visual_authority::isAvailable()) {
            return false;
        }

        const RE::NiTransform partGripHandWorld = resolvePartGripHandWorld(grip, weaponNode);
        const auto& returningHand = _returningHandVisuals[isLeft ? 0u : 1u].transition;
        const RE::NiTransform* acquisitionStart =
            returningHand.active && isUsableHandAuthorityTransform(returningHand.lastApplied) ?
            &returningHand.lastApplied :
            liveHandWorld;
        const bool synchronizedDynamicAcquisition =
            dynamicSupportAcquisitionMatches(isLeft, grip);
        const RE::NiTransform appliedHandWorld =
            synchronizedDynamicAcquisition ?
            resolveDynamicSupportAcquisitionHandTarget(
                partGripHandWorld,
                false,
                grip.visualLerp) :
            resolveLockedHandVisualTarget(
                partGripHandWorld,
                acquisitionStart,
                dt,
                grip.visualLerp);
        const bool applied = frik_visual_authority::applyExternalHandWorldTransform(
            SUPPORT_GRIP_TAG, handFromBool(isLeft), appliedHandWorld, GRIP_HAND_POSE_PRIORITY);
        if (applied &&
            !rebaseWeaponLocalForDeferredParentHandTarget(
                weaponNode,
                isLeft,
                appliedHandWorld)) {
            (void)frik_visual_authority::clearExternalHandWorldTransform(
                SUPPORT_GRIP_TAG,
                handFromBool(isLeft));
            return false;
        }
        if (applied) {
            recordPreFrikRetainedHandAuthority(
                RetainedHandAuthorityKind::SupportGrip,
                isLeft,
                appliedHandWorld);
            recordScopeHandAuthorityPublication(scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip, isLeft);
            clearHandVisualReturn(isLeft, "part-grip-authority-acquired", false);
            recordPublishedHandWorld(isLeft, appliedHandWorld);
        }
        return applied;
    }

    bool TwoHandedGrip::applyLockedHandVisualAuthority(
        RE::NiNode* weaponNode,
        bool applyPrimaryHand,
        bool applySupportHand,
        float dt,
        const RE::NiTransform* livePrimaryHandWorld,
        const RE::NiTransform* liveSupportHandWorld)
    {
        if (!weaponNode) {
            return false;
        }

        if (!scope_safe_hand_frame_math::shouldPublishLockedHandVisualAuthority(_scopeMenuOpenThisFrame)) {
            return true;
        }

        if (!frik_visual_authority::isAvailable()) {
            return false;
        }

        if (!applyPrimaryHand && !applySupportHand) {
            return true;
        }

        const bool supportHandIsLeft = !_firingHandIsLeft;
        bool primaryApplied = true;
        bool supportApplied = true;
        if (applyPrimaryHand) {
            primaryApplied = applyFiringHandLockedVisual(weaponNode, dt, livePrimaryHandWorld);
        }
        if (applySupportHand) {
            supportApplied = applyPartGripLockedVisual(supportHandIsLeft, weaponNode, dt, liveSupportHandWorld);
        }
        if (primaryApplied && supportApplied) {
            return true;
        }

        ROCK_LOG_WARN(Weapon,
            "TwoHandedGrip: locked hand authority publication failed primary={} support={} "
            "firingHand={} supportHand={} state={} scopeMenu={} primaryFrame={} supportGrip={} supportFrame={}",
            primaryApplied ? "ok" : "failed",
            supportApplied ? "ok" : "failed",
            _firingHandIsLeft ? "left" : "right",
            supportHandIsLeft ? "left" : "right",
            static_cast<int>(_state),
            _scopeMenuOpenThisFrame ? "open" : "closed",
            _hasFiringHandWeaponLocal ? "ready" : "missing",
            partGrip(supportHandIsLeft).active ? "active" : "inactive",
            partGrip(supportHandIsLeft).hasHandWeaponLocal ? "ready" : "missing");

        if (applyPrimaryHand && primaryApplied) {
            (void)clearHandAuthorityRoleNow(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip, _firingHandIsLeft);
        }
        if (applySupportHand && supportApplied) {
            (void)clearHandAuthorityRoleNow(scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip, supportHandIsLeft);
        }
        return false;
    }

    void TwoHandedGrip::publishGripHandPoses(bool isLeft)
    {
        if (isNativeReloadSupportHand(isLeft) ||
            !frik_visual_authority::isAvailable()) {
            return;
        }

        const WeaponPartGrip& grip = partGrip(isLeft);
        if (weapon_visual_authority_math::shouldPublishTwoHandedGripPose(weapon_visual_authority_math::LockedHandRole::Support) && grip.hasFingerPose) {
            const auto handPose = grip.hasFingerSplay ?
                frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose, grip.fingerSplayRadians) :
                frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose);
            (void)frik_visual_authority::setHandPoseCustomWithPriority(
                SUPPORT_GRIP_TAG,
                handFromBool(isLeft),
                handPose,
                GRIP_HAND_POSE_PRIORITY);
        }

        if (grip.hasFingerLocalTransforms) {
            frik_visual_authority::FingerLocalTransformOverride overrideData{};
            overrideData.enabledMask = grip.fingerLocalTransformMask;
            for (std::size_t i = 0; i < grip.fingerLocalTransforms.size(); ++i) {
                overrideData.localTransforms[i] = grip.fingerLocalTransforms[i];
            }
            (void)frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(SUPPORT_GRIP_TAG, handFromBool(isLeft), &overrideData, GRIP_HAND_POSE_PRIORITY);
        }
    }

    void TwoHandedGrip::clearPrimaryGripPose(bool isLeft)
    {
        _hasLastPublishedHandWorld[isLeft ? 0u : 1u] = false;
        if (_authoredPrimaryFingerPosePublished && _publishedFiringFingerPoseIsLeft == isLeft) {
            clearAuthoredPrimaryFiringGripFingerPose();
        } else {
            (void)frik_visual_authority::clearHandPose(PRIMARY_GRIP_TAG, handFromBool(isLeft));
        }
        deferOrClearHandAuthorityRole(
            scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip,
            isLeft);
    }

    void TwoHandedGrip::clearPrimaryDetachVisualAuthority(bool isLeft)
    {
        (void)frik_visual_authority::clearHandPose(PRIMARY_DETACH_TAG, handFromBool(isLeft));
        deferOrClearHandAuthorityRole(
            scope_safe_hand_frame_math::HandAuthorityRole::PrimaryDetach,
            isLeft);
    }

    void TwoHandedGrip::killFrikOffhandGrip()
    {
        if (frik_visual_authority::blockOffHandWeaponGripping("ROCK_TwoHanded", true)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK offhand grip suppressed");
        }
    }

    void TwoHandedGrip::restoreFrikOffhandGrip()
    {
        if (frik_visual_authority::blockOffHandWeaponGripping("ROCK_TwoHanded", false)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK offhand grip restored");
        }
    }

    bool TwoHandedGrip::blockFrikPrimaryWeaponPose()
    {
        if (frik_visual_authority::blockPrimaryHandWeaponPose("ROCK_PrimaryDetach", true)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK primary weapon pose suppressed");
            return true;
        }
        return false;
    }

    void TwoHandedGrip::restoreFrikPrimaryWeaponPose()
    {
        if (frik_visual_authority::blockPrimaryHandWeaponPose("ROCK_PrimaryDetach", false)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK primary weapon pose restored");
        }
    }

}
