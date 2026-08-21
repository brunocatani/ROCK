#include "physics-interaction/weapon/two_handed/TwoHandedGrip.h"

/*
 * NATIVE SCOPE support: keeping weapon and hands coherent while the engine's own
 * scope camera is active.
 *
 * Covers the rigid scope frame, overlay calibration, sight-anchor resolution,
 * scope-safe hand frames, the collision-isolated right-hand intent, and the
 * deferred hand-authority role machinery.
 *
 * ORDERING COUPLING: refreshScopeSafeHandFrames calls
 * tryResolveGunstockPhysicalFiringFrame and reads _gunstockWeaponEligibility,
 * both owned by TwoHandedGripGunstock.cpp. Scope frames are only valid after
 * gunstock eligibility has been observed this frame. The gunstock TU carries the
 * matching note.
 *
 * Role clears are DEFERRED, not immediate, while a scope-menu frame is in
 * flight. Clearing a FRIK role mid-menu-frame drops the weapon presentation for
 * one frame. deferOrClearHandAuthorityRole is that decision, in one place.
 */
#include "physics-interaction/weapon/two_handed/TwoHandedGripInternal.h"

#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/native_anim/NativeScopeSightAnchorPolicy.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/collision/WeaponCollision.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock
{
    using two_handed_grip_detail::applyNativeScopeCameraWorldTarget;
    using two_handed_grip_detail::arePointsNearlyEqual;
    using two_handed_grip_detail::areTransformsNearlyEqual;
    using two_handed_grip_detail::captureNativeScopeCameraFollow;
    using two_handed_grip_detail::captureScopeHandAuthorityCleanupVisuals;
    using two_handed_grip_detail::GRIP_HAND_POSE_PRIORITY;
    using two_handed_grip_detail::isFiniteTransform;
    using two_handed_grip_detail::isInvertibleTransform;
    using two_handed_grip_detail::isUsableHandAuthorityTransform;
    using two_handed_grip_detail::makeNativeScopeCameraDebugSnapshot;
    using two_handed_grip_detail::NativeScopeCameraFollowCapture;
    using two_handed_grip_detail::NativeScopeCameraFollowResult;
    using two_handed_grip_detail::PRIMARY_GRIP_TAG;
    using two_handed_grip_detail::PRIMARY_DETACH_TAG;
    using two_handed_grip_detail::restoreScopeHandAuthorityCleanupVisuals;
    using two_handed_grip_detail::SCOPE_DRIVER_MISS_GRACE_FRAMES;
    using two_handed_grip_detail::SCOPE_ROOT_REBASE_DURATION_SECONDS;
    using two_handed_grip_detail::SCOPE_TRANSITION_TRACE_FRAMES;
    using two_handed_grip_detail::ScopeHandAuthorityCleanupVisualSnapshot;
    using two_handed_grip_detail::SUPPORT_GRIP_TAG;
    using two_handed_grip_detail::tryGetRootFlattenedHandBoneTransform;
    using two_handed_grip_detail::tryResolveWeaponRootLocal;
    using two_handed_grip_detail::WEAPON_NODE_OWNERSHIP_TAG;

    namespace
    {
        bool tryGetComposedNodeWorld(const RE::NiAVObject* node, RE::NiTransform& outWorld)
        {
            if (!node) {
                return false;
            }
            outWorld = node->parent ?
                transform_math::composeTransforms(node->parent->world, node->local) :
                node->world;
            return isFiniteTransform(outWorld);
        }

    }

    void TwoHandedGrip::clearNativeScopeRigidFrame() { _nativeScopeRigidFrame = {}; }


    void TwoHandedGrip::clearNativeScopeAnchorState()
    {
        _nativeScopeAnchorWeaponNode = nullptr;
        _nativeScopeAnchorGenerationKey = 0;
        _nativeScopeAnchorOwnershipKey = 0;
        _nativeScopeAnchorWeaponFormID = 0;
        _nativeScopeAnchorWeaponLocal = {};
        _nativeScopeAnchorSource =
            native_scope_sight_anchor_policy::AnchorSource::None;
        _nativeScopeAnchorValid = false;
        _nativeScopeFallbackRotationDegrees = {};
    }


    bool TwoHandedGrip::rebuildNativeScopeRigidFrameTarget()
    {
        if (!_nativeScopeRigidFrame.valid ||
            !_nativeScopeAnchorValid ||
            _nativeScopeRigidFrame.weaponNodeIdentity !=
                _nativeScopeAnchorWeaponNode ||
            _nativeScopeRigidFrame.weaponGenerationKey !=
                _nativeScopeAnchorGenerationKey) {
            return false;
        }

        RE::NiTransform targetCameraWeaponLocal =
            _nativeScopeRigidFrame.nativeCameraWeaponLocal;
        targetCameraWeaponLocal.translate = _nativeScopeAnchorWeaponLocal;
        if (_nativeScopeAnchorSource ==
            native_scope_sight_anchor_policy::AnchorSource::
                FiringGripFallback) {
            targetCameraWeaponLocal =
                native_scope_camera_follow_math::
                    applyWeaponLocalRotationOffset(
                        targetCameraWeaponLocal,
                        _nativeScopeFallbackRotationDegrees.x,
                        _nativeScopeFallbackRotationDegrees.y,
                        _nativeScopeFallbackRotationDegrees.z);
        }
        if (!isFiniteTransform(targetCameraWeaponLocal) ||
            std::abs(targetCameraWeaponLocal.scale) <= 0.0001f) {
            return false;
        }

        _nativeScopeRigidFrame.cameraWeaponLocal =
            targetCameraWeaponLocal;
        return true;
    }


    NativeScopeCameraTargetPreviewSnapshot
        TwoHandedGrip::getNativeScopeCameraTargetPreviewSnapshot() const
    {
        const bool valid =
            _nativeScopeRigidFrame.valid &&
            _nativeScopeAnchorValid &&
            _nativeScopeRigidFrame.weaponNodeIdentity ==
                _nativeScopeAnchorWeaponNode &&
            _nativeScopeRigidFrame.weaponGenerationKey ==
                _nativeScopeAnchorGenerationKey &&
            isFiniteTransform(
                _nativeScopeRigidFrame.cameraWeaponLocal) &&
            std::abs(_nativeScopeRigidFrame.cameraWeaponLocal.scale) >
                0.0001f;
        return NativeScopeCameraTargetPreviewSnapshot{
            .weaponGenerationKey =
                _nativeScopeRigidFrame.weaponGenerationKey,
            .equippedWeaponOwnershipKey =
                _nativeScopeAnchorOwnershipKey,
            .weaponFormID = _nativeScopeAnchorWeaponFormID,
            .anchorSource = _nativeScopeAnchorSource,
            .cameraWeaponLocal =
                _nativeScopeRigidFrame.cameraWeaponLocal,
            .valid = valid,
        };
    }


    bool TwoHandedGrip::captureNativeScopeRigidFrame(RE::NiNode* weaponNode, const std::uint64_t currentWeaponGenerationKey, RE::NiNode* scopeCamera,
        const RE::NiTransform& nativeCameraWorld)
    {
        if (_nativeScopeRigidFrame.valid && _nativeScopeRigidFrame.weaponGenerationKey == currentWeaponGenerationKey && _nativeScopeRigidFrame.weaponNodeIdentity == weaponNode &&
            _nativeScopeRigidFrame.scopeCameraIdentity == scopeCamera) {
            return true;
        }

        clearNativeScopeRigidFrame();
        if (!weaponNode || currentWeaponGenerationKey == 0 || !_nativeScopeAnchorValid || _nativeScopeAnchorWeaponNode != weaponNode ||
            _nativeScopeAnchorGenerationKey != currentWeaponGenerationKey || !scopeCamera || !isFiniteTransform(weaponNode->world) || !isFiniteTransform(nativeCameraWorld)) {
            return false;
        }

        const RE::NiTransform nativeCameraWeaponLocal =
            native_scope_camera_follow_math::captureRigidAnchorFrameWeaponLocal(weaponNode->world, nativeCameraWorld, _nativeScopeAnchorWeaponLocal);
        if (!isFiniteTransform(nativeCameraWeaponLocal) ||
            std::abs(nativeCameraWeaponLocal.scale) <= 0.0001f) {
            return false;
        }

        _nativeScopeRigidFrame = NativeScopeRigidFrameState{
            .weaponGenerationKey = currentWeaponGenerationKey,
            .weaponNodeIdentity = weaponNode,
            .scopeCameraIdentity = scopeCamera,
            .nativeCameraWeaponLocal = nativeCameraWeaponLocal,
            .cameraWeaponLocal = nativeCameraWeaponLocal,
            .valid = true,
        };
        if (!rebuildNativeScopeRigidFrameTarget()) {
            clearNativeScopeRigidFrame();
            return false;
        }
        ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: native scope rigid frame captured generation={:016X} cameraLocal=({:.2f},{:.2f},{:.2f}) scale={:.3f}", currentWeaponGenerationKey,
            _nativeScopeRigidFrame.cameraWeaponLocal.translate.x, _nativeScopeRigidFrame.cameraWeaponLocal.translate.y,
            _nativeScopeRigidFrame.cameraWeaponLocal.translate.z, _nativeScopeRigidFrame.cameraWeaponLocal.scale);
        return true;
    }


    void TwoHandedGrip::synchronizeNativeScopePresentationAfterFrikUpdate(RE::NiNode* weaponNode, const std::uint64_t currentWeaponGenerationKey)
    {
        if (!weaponNode || currentWeaponGenerationKey == 0 || !_nativeScopeAnchorValid || _nativeScopeAnchorWeaponNode != weaponNode ||
            _nativeScopeAnchorGenerationKey != currentWeaponGenerationKey) {
            clearNativeScopeOverlayAuthority(true);
            clearNativeScopeRigidFrame();
            return;
        }

        if ((_nativeScopeOverlayCalibration.valid && _nativeScopeOverlayCalibration.weaponGenerationKey != currentWeaponGenerationKey) ||
            (_nativeScopeRigidFrame.valid && _nativeScopeRigidFrame.weaponGenerationKey != currentWeaponGenerationKey)) {
            clearNativeScopeOverlayAuthority(true);
            clearNativeScopeRigidFrame();
        }

        /*
         * PlayerCharacter's native scope gate ran earlier in the frame. hFRIK
         * has now authored its engine-specific camera axis calibration; capture
         * that calibration once, then publish the complete rigid weapon-local
         * scope frame before FO4VR's later mono render. ROCK's two-hand solve
         * republishes this same frame from its final weapon transform below.
         */
        const NativeScopeCameraFollowCapture capture = captureNativeScopeCameraFollow(weaponNode);
        if (!capture.valid || !captureNativeScopeRigidFrame(weaponNode, currentWeaponGenerationKey, capture.camera, capture.cameraWorldBefore)) {
            return;
        }

        const bool overlayCalibrationReady = captureNativeScopeOverlayCalibration(capture.cameraWorldBefore, currentWeaponGenerationKey);
        const RE::NiTransform targetCameraWorld = native_scope_camera_follow_math::resolveRigidAnchorFrameWorld(weaponNode->world, _nativeScopeRigidFrame.cameraWeaponLocal);
        const NativeScopeCameraFollowResult result = applyNativeScopeCameraWorldTarget(capture, targetCameraWorld);
        if (overlayCalibrationReady && result.targetValid && result.writeApplied) {
            (void)applyNativeScopeOverlayTarget(result.targetCameraWorld, currentWeaponGenerationKey);
        }
        if (g_rockConfig.rockDebugDrawNativeScopeActivation) {
            _nativeScopeCameraDebugSnapshot = makeNativeScopeCameraDebugSnapshot(_nativeScopeCameraDebugSnapshot, currentWeaponGenerationKey,
                NativeScopeCameraWriteSource::PostFrikPresentationSync, capture, result, _nativeScopeAnchorSource);
        }
    }


    void TwoHandedGrip::clearNativeScopeOverlayAuthority(const bool restoreNativeLocal)
    {
        if (restoreNativeLocal && _nativeScopeOverlayCalibration.valid && _nativeScopeOverlayCalibration.hasAppliedLocal && runtime_state::isLocalSkeletonReady() &&
            RE::PlayerCharacter::GetSingleton()) {
            const auto* playerNodes = f4vr::getPlayerNodes();
            auto* scopeParent = playerNodes ? playerNodes->ScopeParentNode : nullptr;
            if (scopeParent == _nativeScopeOverlayCalibration.scopeParentIdentity &&
                areTransformsNearlyEqual(scopeParent->local, _nativeScopeOverlayCalibration.lastAppliedScopeParentLocal)) {
                scopeParent->local = _nativeScopeOverlayCalibration.nativeScopeParentLocal;
                if (scopeParent->parent) {
                    f4vr::updateTransformsDown(scopeParent, true);
                } else {
                    scopeParent->world = scopeParent->local;
                    f4vr::updateTransformsDown(scopeParent, false);
                }
            }
        }

        _nativeScopeOverlayCalibration = {};
    }


    bool TwoHandedGrip::captureNativeScopeOverlayCalibration(
        const RE::NiTransform& nativeCameraWorld,
        const std::uint64_t currentWeaponGenerationKey)
    {
        if (currentWeaponGenerationKey == 0 || !isFiniteTransform(nativeCameraWorld) ||
            std::abs(nativeCameraWorld.scale) <= 0.0001f || !RE::PlayerCharacter::GetSingleton()) {
            return false;
        }

        const auto* playerNodes = f4vr::getPlayerNodes();
        auto* scopeParent = playerNodes ? playerNodes->ScopeParentNode : nullptr;
        if (!scopeParent || !scopeParent->parent || !isFiniteTransform(scopeParent->local) ||
            std::abs(scopeParent->parent->world.scale) <= 0.0001f) {
            return false;
        }

        auto* scopeModelRoot = f4vr::find1StChildNode(scopeParent, "world_scope.nif");
        if (!scopeModelRoot || scopeModelRoot->parent != scopeParent ||
            !isFiniteTransform(scopeModelRoot->local) || std::abs(scopeModelRoot->local.scale) <= 0.0001f) {
            return false;
        }

        if (_nativeScopeOverlayCalibration.valid) {
            const bool sameOwner =
                _nativeScopeOverlayCalibration.weaponGenerationKey == currentWeaponGenerationKey &&
                _nativeScopeOverlayCalibration.scopeParentIdentity == scopeParent &&
                _nativeScopeOverlayCalibration.scopeModelRootIdentity == scopeModelRoot &&
                areTransformsNearlyEqual(scopeModelRoot->local, _nativeScopeOverlayCalibration.scopeModelRootLocal);
            const bool engineStillHasRockLocal =
                !_nativeScopeOverlayCalibration.hasAppliedLocal ||
                areTransformsNearlyEqual(scopeParent->local, _nativeScopeOverlayCalibration.lastAppliedScopeParentLocal);
            if (sameOwner && engineStillHasRockLocal) {
                return true;
            }

            /*
             * A changed local transform is an engine re-authoring event (for
             * example an equip/OMOD change), not ours to restore. A changed
             * generation/node with our last local still present is restored
             * before the new native baseline is captured.
             */
            clearNativeScopeOverlayAuthority(!sameOwner && engineStillHasRockLocal);
        }

        RE::NiTransform nativeScopeModelRootWorld{};
        if (!tryGetComposedNodeWorld(scopeModelRoot, nativeScopeModelRootWorld)) {
            return false;
        }
        const RE::NiTransform modelRootCalibrationInCameraLocal =
            native_scope_overlay_follow_math::captureModelRootCalibrationInCameraLocal(
                nativeCameraWorld,
                nativeScopeModelRootWorld);
        if (!isFiniteTransform(modelRootCalibrationInCameraLocal) ||
            std::abs(modelRootCalibrationInCameraLocal.scale) <= 0.0001f) {
            return false;
        }

        _nativeScopeOverlayCalibration = NativeScopeOverlayCalibrationState{
            .weaponGenerationKey = currentWeaponGenerationKey,
            .scopeParentIdentity = scopeParent,
            .scopeModelRootIdentity = scopeModelRoot,
            .scopeModelRootLocal = scopeModelRoot->local,
            .scopeModelRootCalibrationInCameraLocal = modelRootCalibrationInCameraLocal,
            .nativeScopeParentLocal = scopeParent->local,
            .lastAppliedScopeParentLocal = {},
            .valid = true,
            .hasAppliedLocal = false,
        };
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: native scope overlay calibrated generation={:016X} modelRootLocal=({:.2f},{:.2f},{:.2f}) cameraCalibrationScale={:.3f} nativeParentLocal=({:.2f},{:.2f},{:.2f})",
            currentWeaponGenerationKey,
            scopeModelRoot->local.translate.x,
            scopeModelRoot->local.translate.y,
            scopeModelRoot->local.translate.z,
            modelRootCalibrationInCameraLocal.scale,
            scopeParent->local.translate.x,
            scopeParent->local.translate.y,
            scopeParent->local.translate.z);
        return true;
    }


    bool TwoHandedGrip::applyNativeScopeOverlayTarget(
        const RE::NiTransform& correctedCameraWorld,
        const std::uint64_t currentWeaponGenerationKey)
    {
        if (!_nativeScopeOverlayCalibration.valid ||
            _nativeScopeOverlayCalibration.weaponGenerationKey != currentWeaponGenerationKey ||
            !isFiniteTransform(correctedCameraWorld) || !RE::PlayerCharacter::GetSingleton()) {
            return false;
        }

        const auto* playerNodes = f4vr::getPlayerNodes();
        auto* scopeParent = playerNodes ? playerNodes->ScopeParentNode : nullptr;
        if (!scopeParent || scopeParent != _nativeScopeOverlayCalibration.scopeParentIdentity ||
            !scopeParent->parent || std::abs(scopeParent->parent->world.scale) <= 0.0001f) {
            return false;
        }

        auto* scopeModelRoot = f4vr::find1StChildNode(scopeParent, "world_scope.nif");
        if (!scopeModelRoot || scopeModelRoot != _nativeScopeOverlayCalibration.scopeModelRootIdentity ||
            scopeModelRoot->parent != scopeParent ||
            !areTransformsNearlyEqual(scopeModelRoot->local, _nativeScopeOverlayCalibration.scopeModelRootLocal)) {
            return false;
        }

        if (_nativeScopeOverlayCalibration.hasAppliedLocal && !areTransformsNearlyEqual(scopeParent->local, _nativeScopeOverlayCalibration.lastAppliedScopeParentLocal)) {
            // FO4VR reclaimed the node after our calibration. The next
            // post-hFRIK presentation sync captures its new native baseline.
            return false;
        }

        const RE::NiTransform modelRootFineTuneLocal =
            native_scope_overlay_follow_math::makeModelRootFineTuneLocal<RE::NiTransform>(
                g_rockConfig.rockNativeScopeOverlayOffsetXGameUnits,
                g_rockConfig.rockNativeScopeOverlayOffsetYGameUnits,
                g_rockConfig.rockNativeScopeOverlayOffsetZGameUnits,
                g_rockConfig.rockNativeScopeOverlayPitchDegrees,
                g_rockConfig.rockNativeScopeOverlayYawDegrees,
                g_rockConfig.rockNativeScopeOverlayRollDegrees);
        const RE::NiTransform targetScopeModelRootWorld =
            native_scope_overlay_follow_math::resolveScopeModelRootWorld(
                correctedCameraWorld,
                _nativeScopeOverlayCalibration.scopeModelRootCalibrationInCameraLocal,
                modelRootFineTuneLocal);
        if (!isFiniteTransform(targetScopeModelRootWorld)) {
            return false;
        }

        const RE::NiTransform targetScopeParentWorld =
            native_scope_overlay_follow_math::resolveScopeParentWorldForModelRoot(
                targetScopeModelRootWorld,
                _nativeScopeOverlayCalibration.scopeModelRootLocal);
        if (!isFiniteTransform(targetScopeParentWorld)) {
            return false;
        }

        const RE::NiTransform targetScopeParentLocal =
            weapon_visual_authority_math::worldTargetToParentLocal(
                scopeParent->parent->world,
                targetScopeParentWorld);
        if (!isFiniteTransform(targetScopeParentLocal)) {
            return false;
        }

        scopeParent->local = targetScopeParentLocal;
        f4vr::updateTransformsDown(scopeParent, true);
        _nativeScopeOverlayCalibration.lastAppliedScopeParentLocal = targetScopeParentLocal;
        _nativeScopeOverlayCalibration.hasAppliedLocal = true;

        RE::NiTransform immediateScopeModelRootWorld{};
        return tryGetComposedNodeWorld(scopeModelRoot, immediateScopeModelRootWorld) &&
               areTransformsNearlyEqual(immediateScopeModelRootWorld, targetScopeModelRootWorld, 0.01f);
    }


    void TwoHandedGrip::refreshNativeScopeAnchor(
        RE::NiNode* weaponNode,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        std::uint32_t currentEquippedWeaponFormID,
        const WeaponCollision& weaponCollision)
    {
        const bool forceFiringGripFallback =
            g_rockConfig.rockNativeScopeForceFiringGripFallback;
        const RE::NiPoint3 fallbackOffsetWeaponLocal{
            g_rockConfig.rockNativeScopeFiringGripFallbackOffsetXGameUnits,
            g_rockConfig.rockNativeScopeFiringGripFallbackOffsetYGameUnits,
            g_rockConfig.rockNativeScopeFiringGripFallbackOffsetZGameUnits,
        };
        const RE::NiPoint3 fallbackRotationDegrees{
            g_rockConfig.rockNativeScopeFiringGripFallbackPitchDegrees,
            g_rockConfig.rockNativeScopeFiringGripFallbackYawDegrees,
            g_rockConfig.rockNativeScopeFiringGripFallbackRollDegrees,
        };

        RE::NiPoint3 firingGripWeaponLocal{};
        bool firingGripFromCanonical = false;
        bool firingGripValid = hasRightFiringHandCanonicalFrame(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey);
        if (firingGripValid) {
            firingGripWeaponLocal = _rightFiringGripCanonicalWeaponLocal;
            firingGripFromCanonical = true;
        } else if (isManualOwnershipActive() &&
                   _activeWeaponNode == weaponNode &&
                   _activeWeaponGenerationKey == currentWeaponGenerationKey &&
                   _activeEquippedWeaponOwnershipKey ==
                       currentEquippedWeaponOwnershipKey &&
                   _primaryGripConfidence > 0.0f &&
                   native_scope_sight_anchor_policy::isFinitePoint(
                       _primaryGripLocal)) {
            firingGripWeaponLocal = _primaryGripLocal;
            firingGripValid = true;
        }

        const bool sameIdentity =
            _nativeScopeAnchorWeaponNode == weaponNode &&
            _nativeScopeAnchorGenerationKey == currentWeaponGenerationKey &&
            _nativeScopeAnchorOwnershipKey ==
                currentEquippedWeaponOwnershipKey &&
            _nativeScopeAnchorWeaponFormID == currentEquippedWeaponFormID;
        if (sameIdentity &&
            _nativeScopeAnchorSource ==
                native_scope_sight_anchor_policy::AnchorSource::GeneratedSight &&
            !forceFiringGripFallback) {
            return;
        }

        const bool identityChanged = !sameIdentity;
        if (identityChanged) {
            clearNativeScopeOverlayAuthority(true);
            clearNativeScopeRigidFrame();
            _nativeScopeAnchorWeaponNode = weaponNode;
            _nativeScopeAnchorGenerationKey = currentWeaponGenerationKey;
            _nativeScopeAnchorOwnershipKey =
                currentEquippedWeaponOwnershipKey;
            _nativeScopeAnchorWeaponFormID = currentEquippedWeaponFormID;
            _nativeScopeAnchorWeaponLocal = {};
            _nativeScopeAnchorSource =
                native_scope_sight_anchor_policy::AnchorSource::None;
            _nativeScopeAnchorValid = false;
            _nativeScopeFallbackRotationDegrees = fallbackRotationDegrees;
        }
        if (!weaponNode || currentWeaponGenerationKey == 0 ||
            currentEquippedWeaponOwnershipKey == 0 ||
            currentEquippedWeaponFormID == 0) {
            return;
        }

        WeaponCollision::NativeScopeSightAnchorSnapshot snapshot{};
        if (!forceFiringGripFallback) {
            snapshot = weaponCollision.getNativeScopeSightAnchorSnapshot();
            const native_scope_sight_anchor_policy::PublicationIdentity
                publishedIdentity{
                    .weaponGenerationKey = snapshot.weaponGenerationKey,
                    .equippedWeaponOwnershipKey =
                        snapshot.equippedWeaponOwnershipKey,
                    .weaponFormID = snapshot.weaponFormID,
                };
            const native_scope_sight_anchor_policy::PublicationIdentity
                currentIdentity{
                    .weaponGenerationKey = currentWeaponGenerationKey,
                    .equippedWeaponOwnershipKey =
                        currentEquippedWeaponOwnershipKey,
                    .weaponFormID = currentEquippedWeaponFormID,
                };
            if (snapshot.weaponGenerationKey !=
                    currentWeaponGenerationKey ||
                !native_scope_sight_anchor_policy::
                    matchesCurrentEquippedWeapon(
                        publishedIdentity,
                        currentIdentity)) {
                // A racing publication is never allowed to select either its
                // geometry or a sticky fallback. Clear the identity so the
                // next frame retries against one coherent body-set snapshot.
                clearNativeScopeAnchorState();
                clearNativeScopeOverlayAuthority(true);
                clearNativeScopeRigidFrame();
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: rejected stale native scope sight anchor generation={:016X}->{:016X} ownership={:016X}->{:016X} form={:08X}->{:08X}",
                    snapshot.weaponGenerationKey,
                    currentWeaponGenerationKey,
                    snapshot.equippedWeaponOwnershipKey,
                    currentEquippedWeaponOwnershipKey,
                    snapshot.weaponFormID,
                    currentEquippedWeaponFormID);
                return;
            }
        }

        const auto resolved = native_scope_sight_anchor_policy::resolve(
            forceFiringGripFallback,
            snapshot.valid,
            snapshot.anchorWeaponLocal,
            firingGripValid,
            firingGripWeaponLocal,
            fallbackOffsetWeaponLocal);
        if (!resolved.valid) {
            if (_nativeScopeAnchorValid) {
                clearNativeScopeOverlayAuthority(true);
                clearNativeScopeRigidFrame();
            }
            _nativeScopeAnchorWeaponLocal = {};
            _nativeScopeAnchorSource =
                native_scope_sight_anchor_policy::AnchorSource::None;
            _nativeScopeAnchorValid = false;
            _nativeScopeFallbackRotationDegrees =
                fallbackRotationDegrees;
            ROCK_LOG_SAMPLE_DEBUG(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "TwoHandedGrip: native scope anchor unavailable generation={:016X} generatedSight={} firingGrip={} forced={}; leaving native camera untouched",
                currentWeaponGenerationKey,
                snapshot.valid,
                firingGripValid,
                forceFiringGripFallback);
            return;
        }

        const bool sameResolvedAnchor =
            !identityChanged &&
            _nativeScopeAnchorValid &&
            _nativeScopeAnchorSource == resolved.source &&
            arePointsNearlyEqual(
                _nativeScopeAnchorWeaponLocal,
                resolved.weaponLocal);
        const bool fallbackRotationChanged =
            resolved.source ==
                native_scope_sight_anchor_policy::AnchorSource::
                    FiringGripFallback &&
            !arePointsNearlyEqual(
                _nativeScopeFallbackRotationDegrees,
                fallbackRotationDegrees);
        if (sameResolvedAnchor && !fallbackRotationChanged) {
            return;
        }

        if (sameResolvedAnchor) {
            _nativeScopeFallbackRotationDegrees =
                fallbackRotationDegrees;
            if (_nativeScopeRigidFrame.valid &&
                !rebuildNativeScopeRigidFrameTarget()) {
                clearNativeScopeOverlayAuthority(true);
                clearNativeScopeRigidFrame();
                ROCK_LOG_WARN(Weapon,
                    "TwoHandedGrip: rejected invalid native scope fallback rotation pitch={:.2f} yaw={:.2f} roll={:.2f}",
                    fallbackRotationDegrees.x,
                    fallbackRotationDegrees.y,
                    fallbackRotationDegrees.z);
                return;
            }
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: native scope firing-grip fallback rotation refreshed generation={:016X} pitch={:.2f} yaw={:.2f} roll={:.2f}",
                currentWeaponGenerationKey,
                fallbackRotationDegrees.x,
                fallbackRotationDegrees.y,
                fallbackRotationDegrees.z);
            return;
        }

        _nativeScopeAnchorWeaponLocal = resolved.weaponLocal;
        _nativeScopeAnchorSource = resolved.source;
        _nativeScopeAnchorValid = true;
        _nativeScopeFallbackRotationDegrees =
            fallbackRotationDegrees;
        if (_nativeScopeRigidFrame.valid &&
            !rebuildNativeScopeRigidFrameTarget()) {
            clearNativeScopeOverlayAuthority(true);
            clearNativeScopeRigidFrame();
            ROCK_LOG_WARN(Weapon,
                "TwoHandedGrip: native scope target rebuild failed generation={:016X} source={}",
                currentWeaponGenerationKey,
                resolved.source ==
                        native_scope_sight_anchor_policy::AnchorSource::
                            FiringGripFallback ?
                    "firing-grip-fallback" :
                    "generated-sight");
        }
        if (resolved.source ==
            native_scope_sight_anchor_policy::AnchorSource::GeneratedSight) {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: native scope anchor generation={:016X} source=generated-sight bodies={} local=({:.2f},{:.2f},{:.2f}) boundsMin=({:.2f},{:.2f},{:.2f}) boundsMax=({:.2f},{:.2f},{:.2f}) policy=rear-center",
                currentWeaponGenerationKey,
                snapshot.sightBodyCount,
                snapshot.anchorWeaponLocal.x,
                snapshot.anchorWeaponLocal.y,
                snapshot.anchorWeaponLocal.z,
                snapshot.sightBoundsMinWeaponLocal.x,
                snapshot.sightBoundsMinWeaponLocal.y,
                snapshot.sightBoundsMinWeaponLocal.z,
                snapshot.sightBoundsMaxWeaponLocal.x,
                snapshot.sightBoundsMaxWeaponLocal.y,
                snapshot.sightBoundsMaxWeaponLocal.z);
        } else {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: native scope anchor generation={:016X} source=firing-grip-fallback origin={} gripLocal=({:.2f},{:.2f},{:.2f}) offset=({:.2f},{:.2f},{:.2f}) rotation=({:.2f},{:.2f},{:.2f}) anchorLocal=({:.2f},{:.2f},{:.2f}) forced={}",
                currentWeaponGenerationKey,
                firingGripFromCanonical ? "canonical" : "active-grip",
                firingGripWeaponLocal.x,
                firingGripWeaponLocal.y,
                firingGripWeaponLocal.z,
                fallbackOffsetWeaponLocal.x,
                fallbackOffsetWeaponLocal.y,
                fallbackOffsetWeaponLocal.z,
                fallbackRotationDegrees.x,
                fallbackRotationDegrees.y,
                fallbackRotationDegrees.z,
                _nativeScopeAnchorWeaponLocal.x,
                _nativeScopeAnchorWeaponLocal.y,
                _nativeScopeAnchorWeaponLocal.z,
                forceFiringGripFallback);
        }
    }


    void TwoHandedGrip::refreshScopeSafeHandFrames(RE::NiNode* weaponNode, const EquippedWeaponGripFrameInput& frameInput, float dt)
    {
        // A final trace is valid only for the same update that produced its
        // pre-solve sample. Early-return frames deliberately remain pre-only.
        _nativeScopeTransitionFinalTracePending = false;
        const bool activationStateChanged =
            frameInput.manualScopeActivationRequested !=
                _manualScopeActivationRequested ||
            frameInput.nativeScopeRequestStateValid != _nativeScopeRequestStateValid ||
            (frameInput.nativeScopeRequestStateValid &&
                frameInput.nativeScopeRequestActive != _nativeScopeRequestActive);
        _manualScopeActivationRequested =
            frameInput.manualScopeActivationRequested;
        _nativeScopeRequestStateValid = frameInput.nativeScopeRequestStateValid;
        _nativeScopeRequestActive = frameInput.nativeScopeRequestStateValid &&
                                    frameInput.nativeScopeRequestActive;
        _nativeScopeActivationDebugSnapshot = NativeScopeActivationDebugSnapshot{
            .publicationSequence =
                _nativeScopeActivationDebugSnapshot.publicationSequence + 1,
            .weaponGenerationKey = _nativeScopeAnchorGenerationKey,
            .anchorSource = _nativeScopeAnchorSource,
            .manualInputRequested = _manualScopeActivationRequested,
            .rendererStateValid = _nativeScopeRequestStateValid,
            .rendererActive = _nativeScopeRequestActive,
        };
        if (activationStateChanged) {
            ++_nativeScopeTransitionTraceSequence;
            _nativeScopeTransitionTraceFramesRemaining =
                SCOPE_TRANSITION_TRACE_FRAMES;
        }

        const bool scopeStateChanged = _scopeMenuOpenThisFrame != frameInput.scopeMenuOpen;
        _scopeMenuOpenThisFrame = frameInput.scopeMenuOpen;
        _scopeMenuClosedThisFrame = scopeStateChanged && !_scopeMenuOpenThisFrame;
        const bool driverFrameAuthorityWasActive = _scopeDriverFrameAuthorityActive;
        _scopeDriverFrameAuthorityActive = scope_safe_hand_frame_math::retainDriverFrameAuthority(
            _scopeMenuOpenThisFrame,
            _manualScopeActivationRequested,
            isManualOwnershipActive(),
            driverFrameAuthorityWasActive);
        const bool driverFrameAuthorityStoppedThisFrame =
            driverFrameAuthorityWasActive && !_scopeDriverFrameAuthorityActive;

        if (scopeStateChanged) {
            // Never resume a pre-menu visual interpolation after hFRIK restores
            // its visible body. The weapon solver itself remains continuous.
            resetLockedHandVisualLerp();
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: native scope hand-frame menu={} solver={} leftCache={} rightCache={}",
                _scopeMenuOpenThisFrame ? "open" : "closed",
                _scopeDriverFrameAuthorityActive ?
                    (_scopeMenuOpenThisFrame ? "frik-driver" : "frik-driver-latched") :
                    "root-flattened",
                _scopeSafeHandFrames[0].hasDriverToHandLocal ? "ready" : "missing",
                _scopeSafeHandFrames[1].hasDriverToHandLocal ? "ready" : "missing");
        }

        const float frameDeltaSeconds = std::isfinite(dt) && dt > 0.0f ? (std::min)(dt, 0.1f) : (1.0f / 90.0f);
        const auto refreshHand = [this, weaponNode, driverFrameAuthorityStoppedThisFrame, frameDeltaSeconds](
                                     bool isLeft,
                                     const EquippedWeaponScopeHandDriverFrame& driverFrame,
                                     const EquippedWeaponScopeHandDriverFrame& physicalHandFrame) {
            const std::size_t handIndex = isLeft ? 0u : 1u;
            ScopeSafeHandFrameState& state = _scopeSafeHandFrames[handIndex];
            state.currentHandWorldValid = false;

            RE::NiTransform rootHandWorld{};
            const bool persistentRockHandWorldPublished =
                frik_visual_authority::
                    hasPublishedExternalHandWorldTransform(
                        handFromBool(isLeft)) ||
                collision_isolated_hand_frame_runtime::
                    hadPersistentWorldAuthorityAtFrameInput(isLeft);
            const bool physicalHandValid =
                physicalHandFrame.valid &&
                isUsableHandAuthorityTransform(physicalHandFrame.world);

            // Outside scope/gunstock presentation, the physical controller
            // reconstruction is already the complete collision-isolated hand
            // input. Never reconstruct it through hFRIK's weapon-offset node:
            // firearm animation changes on that node are presentation state,
            // not controller motion, and otherwise feed a native rifle angle
            // back into ROCK's authored weapon solve.
            if (!_scopeDriverFrameAuthorityActive &&
                persistentRockHandWorldPublished &&
                physicalHandValid) {
                state.rootRebaseActive = false;
                state.consecutiveDriverMissFrames = 0;
                state.currentHandWorld = physicalHandFrame.world;
                state.currentHandWorldValid = true;
                state.lastHandWorld = physicalHandFrame.world;
                state.hasLastHandWorld = true;
                return;
            }
            const bool rootHandValid =
                !_scopeDriverFrameAuthorityActive &&
                !persistentRockHandWorldPublished &&
                tryGetRootFlattenedHandBoneTransform(
                    isLeft,
                    rootHandWorld);
            const bool driverValid = driverFrame.valid &&
                                     isUsableHandAuthorityTransform(driverFrame.world);
            RE::NiTransform reconstructedHandWorld{};
            bool reconstructedHandValid = false;
            if (driverValid && state.hasDriverToHandLocal) {
                reconstructedHandWorld = scope_safe_hand_frame_math::resolveHandWorld(
                    driverFrame.world,
                    state.driverToHandLocal);
                reconstructedHandValid = isUsableHandAuthorityTransform(reconstructedHandWorld);
            }
            const auto resolutionMode = scope_safe_hand_frame_math::resolveCollisionIsolatedMode(
                _weaponCollisionHandPresentationFromPreviousFrame[handIndex] ||
                    persistentRockHandWorldPublished,
                _scopeDriverFrameAuthorityActive,
                rootHandValid,
                reconstructedHandValid,
                state.hasLastHandWorld,
                state.consecutiveDriverMissFrames,
                SCOPE_DRIVER_MISS_GRACE_FRAMES);

            if (resolutionMode == scope_safe_hand_frame_math::ResolutionMode::RootFlattened) {
                const bool recentScopedHandAvailable = state.hasLastHandWorld &&
                                                       state.consecutiveDriverMissFrames < SCOPE_DRIVER_MISS_GRACE_FRAMES;
                if (scope_safe_hand_frame_math::shouldStartRootRebase(
                        _manualScopeActivationRequested,
                        driverFrameAuthorityStoppedThisFrame,
                        reconstructedHandValid,
                        recentScopedHandAvailable)) {
                    // The previous ROCK output is the continuity authority.
                    // hFRIK may resume non-scope damping from a stale internal
                    // sample on this exact edge even though its driver is finite.
                    const RE::NiTransform& continuityHandWorld = recentScopedHandAvailable ?
                                                                      state.lastHandWorld :
                                                                      reconstructedHandWorld;
                    const RE::NiTransform rootRebaseLocalStart = transform_math::composeTransforms(
                        transform_math::invertTransform(rootHandWorld),
                        continuityHandWorld);
                    if (isUsableHandAuthorityTransform(rootRebaseLocalStart)) {
                        state.rootRebaseLocalStart = rootRebaseLocalStart;
                        state.rootRebaseElapsedSeconds = 0.0f;
                        state.rootRebaseActive = true;
                    }
                }
                state.consecutiveDriverMissFrames = 0;

                RE::NiTransform resolvedHandWorld = rootHandWorld;
                if (state.rootRebaseActive) {
                    const RE::NiTransform identity = transform_math::makeIdentityTransform<RE::NiTransform>();
                    const float rebaseAlpha = scope_safe_hand_frame_math::rebaseAlpha(
                        state.rootRebaseElapsedSeconds,
                        SCOPE_ROOT_REBASE_DURATION_SECONDS);
                    const RE::NiTransform rebase = scope_safe_hand_frame_math::interpolateRebaseTransform(
                        state.rootRebaseLocalStart,
                        identity,
                        rebaseAlpha);
                    const RE::NiTransform rebasedHandWorld = transform_math::composeTransforms(rootHandWorld, rebase);
                    if (isUsableHandAuthorityTransform(rebasedHandWorld)) {
                        resolvedHandWorld = rebasedHandWorld;
                    } else {
                        state.rootRebaseActive = false;
                    }
                    if (rebaseAlpha >= 1.0f) {
                        state.rootRebaseActive = false;
                    } else {
                        state.rootRebaseElapsedSeconds = (std::min)(
                            SCOPE_ROOT_REBASE_DURATION_SECONDS,
                            state.rootRebaseElapsedSeconds + frameDeltaSeconds);
                    }
                }

                state.currentHandWorld = resolvedHandWorld;
                state.currentHandWorldValid = true;
                state.lastHandWorld = resolvedHandWorld;
                state.hasLastHandWorld = true;

                const bool preserveGunstockPhysicalRelation =
                    g_rockConfig.rockGunstockModeEnabled &&
                    _gunstockWeaponEligibility.eligible &&
                    _gunstockWeaponEligibility.weaponNodeIdentity ==
                        reinterpret_cast<std::uintptr_t>(weaponNode) &&
                    isLeft == _firingHandIsLeft &&
                    (isLeft ?
                            _hasLeftNaturalBoneInDampedDriver :
                            _hasRightNaturalBoneInDampedDriver);
                if (driverValid && !preserveGunstockPhysicalRelation) {
                    const RE::NiTransform driverToHandLocal =
                        scope_safe_hand_frame_math::captureDriverToHandLocal(driverFrame.world, resolvedHandWorld);
                    if (isUsableHandAuthorityTransform(driverToHandLocal)) {
                        state.driverToHandLocal = driverToHandLocal;
                        state.hasDriverToHandLocal = true;
                    }
                }
                return;
            }

            state.rootRebaseActive = false;
            if (resolutionMode == scope_safe_hand_frame_math::ResolutionMode::DriverReconstructed) {
                state.consecutiveDriverMissFrames = 0;
                state.currentHandWorld = reconstructedHandWorld;
                state.currentHandWorldValid = true;
                state.lastHandWorld = reconstructedHandWorld;
                state.hasLastHandWorld = true;
                return;
            }

            // A transient hFRIK arm-driver miss must not become an ownership
            // release/reacquire loop. Hold only a few frames: a real driver
            // loss must still fail closed instead of pinning the weapon in the
            // world indefinitely.
            if (resolutionMode == scope_safe_hand_frame_math::ResolutionMode::LastKnown) {
                ++state.consecutiveDriverMissFrames;
                state.currentHandWorld = state.lastHandWorld;
                state.currentHandWorldValid = true;
            } else if (_scopeDriverFrameAuthorityActive) {
                state.consecutiveDriverMissFrames = SCOPE_DRIVER_MISS_GRACE_FRAMES;
            }
        };

        refreshHand(
            true,
            frameInput.leftScopeHandDriverFrame,
            frameInput.leftHandDriverFrame);
        refreshHand(
            false,
            frameInput.rightScopeHandDriverFrame,
            frameInput.rightHandDriverFrame);

        if (g_rockConfig.rockGunstockModeEnabled &&
            _gunstockWeaponEligibility.eligible &&
            _gunstockWeaponEligibility.weaponNodeIdentity ==
                reinterpret_cast<std::uintptr_t>(weaponNode)) {
            RE::NiTransform physicalFiringHandWorld{};
            RE::NiTransform firingDriverWorld{};
            if (tryResolveGunstockPhysicalFiringFrame(
                    physicalFiringHandWorld,
                    firingDriverWorld)) {
                auto& firingState =
                    _scopeSafeHandFrames[_firingHandIsLeft ? 0u : 1u];
                firingState.currentHandWorld = physicalFiringHandWorld;
                firingState.currentHandWorldValid = true;
                firingState.lastHandWorld = physicalFiringHandWorld;
                firingState.hasLastHandWorld = true;
            }
        }

        if (_nativeScopeTransitionTraceFramesRemaining > 0) {
            struct HandTrace
            {
                RE::NiTransform rootWorld{};
                RE::NiTransform driverWorld{};
                RE::NiTransform reconstructedWorld{};
                RE::NiTransform solverWorld{};
                bool rootValid{ false };
                bool driverValid{ false };
                bool reconstructedValid{ false };
                bool solverValid{ false };
                float rootToReconstructedDistance{ -1.0f };
                float rootToSolverDistance{ -1.0f };
            };

            const auto distanceBetween = [](const RE::NiPoint3& left, const RE::NiPoint3& right) {
                const RE::NiPoint3 delta = left - right;
                const float distance = delta.Length();
                return std::isfinite(distance) ? distance : -1.0f;
            };
            const auto captureHandTrace = [this, &distanceBetween](bool isLeft, const EquippedWeaponScopeHandDriverFrame& driverFrame) {
                HandTrace trace{};
                const ScopeSafeHandFrameState& state =
                    _scopeSafeHandFrames[isLeft ? 0u : 1u];
                trace.rootValid =
                    tryGetRootFlattenedHandBoneTransform(isLeft, trace.rootWorld);
                trace.driverValid = driverFrame.valid &&
                                    isUsableHandAuthorityTransform(driverFrame.world);
                if (trace.driverValid) {
                    trace.driverWorld = driverFrame.world;
                }
                if (trace.driverValid && state.hasDriverToHandLocal) {
                    trace.reconstructedWorld =
                        scope_safe_hand_frame_math::resolveHandWorld(
                            driverFrame.world,
                            state.driverToHandLocal);
                    trace.reconstructedValid =
                        isUsableHandAuthorityTransform(trace.reconstructedWorld);
                }
                trace.solverValid = state.currentHandWorldValid;
                if (trace.solverValid) {
                    trace.solverWorld = state.currentHandWorld;
                }
                if (trace.rootValid && trace.reconstructedValid) {
                    trace.rootToReconstructedDistance = distanceBetween(
                        trace.rootWorld.translate,
                        trace.reconstructedWorld.translate);
                }
                if (trace.rootValid && trace.solverValid) {
                    trace.rootToSolverDistance = distanceBetween(
                        trace.rootWorld.translate,
                        trace.solverWorld.translate);
                }
                return trace;
            };

            const HandTrace leftTrace =
                captureHandTrace(true, frameInput.leftScopeHandDriverFrame);
            const HandTrace rightTrace =
                captureHandTrace(false, frameInput.rightScopeHandDriverFrame);
            RE::NiPoint3 weaponWorldPosition{};
            const bool weaponWorldValid = weaponNode &&
                                          isFiniteTransform(weaponNode->world);
            if (weaponWorldValid) {
                weaponWorldPosition = weaponNode->world.translate;
            }
            RE::NiPoint3 playerWorldOffset{};
            bool playerWorldOffsetValid = false;
            if (const auto* playerNodes = f4vr::getPlayerNodes();
                playerNodes && playerNodes->playerworldnode &&
                isFiniteTransform(playerNodes->playerworldnode->local)) {
                playerWorldOffset =
                    playerNodes->playerworldnode->local.translate;
                playerWorldOffsetValid = true;
            }

            const std::uint32_t sampleIndex =
                SCOPE_TRANSITION_TRACE_FRAMES -
                _nativeScopeTransitionTraceFramesRemaining;
            ROCK_LOG_INFO(Weapon,
                "SCOPE-TRANSITION seq={} sample={}/{} buttonRequested={} rendererValid={} rendererActive={} menuOpen={} manual={} state={} driverAuthority={} weaponValid={} weapon=({:.2f},{:.2f},{:.2f}) playerOffsetValid={} playerOffset=({:.2f},{:.2f},{:.2f}) left[root={} driver={} reconstructed={} solver={} rootT=({:.2f},{:.2f},{:.2f}) driverT=({:.2f},{:.2f},{:.2f}) reconstructedT=({:.2f},{:.2f},{:.2f}) solverT=({:.2f},{:.2f},{:.2f}) rootToReconstructed={:.2f} rootToSolver={:.2f}] right[root={} driver={} reconstructed={} solver={} rootT=({:.2f},{:.2f},{:.2f}) driverT=({:.2f},{:.2f},{:.2f}) reconstructedT=({:.2f},{:.2f},{:.2f}) solverT=({:.2f},{:.2f},{:.2f}) rootToReconstructed={:.2f} rootToSolver={:.2f}]",
                _nativeScopeTransitionTraceSequence,
                sampleIndex,
                SCOPE_TRANSITION_TRACE_FRAMES,
                _manualScopeActivationRequested ? "yes" : "no",
                frameInput.nativeScopeRequestStateValid ? "yes" : "no",
                _nativeScopeRequestActive ? "yes" : "no",
                _scopeMenuOpenThisFrame ? "yes" : "no",
                isManualOwnershipActive() ? "yes" : "no",
                static_cast<std::uint32_t>(_state),
                _scopeDriverFrameAuthorityActive ? "yes" : "no",
                weaponWorldValid ? "yes" : "no",
                weaponWorldPosition.x,
                weaponWorldPosition.y,
                weaponWorldPosition.z,
                playerWorldOffsetValid ? "yes" : "no",
                playerWorldOffset.x,
                playerWorldOffset.y,
                playerWorldOffset.z,
                leftTrace.rootValid ? "yes" : "no",
                leftTrace.driverValid ? "yes" : "no",
                leftTrace.reconstructedValid ? "yes" : "no",
                leftTrace.solverValid ? "yes" : "no",
                leftTrace.rootWorld.translate.x,
                leftTrace.rootWorld.translate.y,
                leftTrace.rootWorld.translate.z,
                leftTrace.driverWorld.translate.x,
                leftTrace.driverWorld.translate.y,
                leftTrace.driverWorld.translate.z,
                leftTrace.reconstructedWorld.translate.x,
                leftTrace.reconstructedWorld.translate.y,
                leftTrace.reconstructedWorld.translate.z,
                leftTrace.solverWorld.translate.x,
                leftTrace.solverWorld.translate.y,
                leftTrace.solverWorld.translate.z,
                leftTrace.rootToReconstructedDistance,
                leftTrace.rootToSolverDistance,
                rightTrace.rootValid ? "yes" : "no",
                rightTrace.driverValid ? "yes" : "no",
                rightTrace.reconstructedValid ? "yes" : "no",
                rightTrace.solverValid ? "yes" : "no",
                rightTrace.rootWorld.translate.x,
                rightTrace.rootWorld.translate.y,
                rightTrace.rootWorld.translate.z,
                rightTrace.driverWorld.translate.x,
                rightTrace.driverWorld.translate.y,
                rightTrace.driverWorld.translate.z,
                rightTrace.reconstructedWorld.translate.x,
                rightTrace.reconstructedWorld.translate.y,
                rightTrace.reconstructedWorld.translate.z,
                rightTrace.solverWorld.translate.x,
                rightTrace.solverWorld.translate.y,
                rightTrace.solverWorld.translate.z,
                rightTrace.rootToReconstructedDistance,
                rightTrace.rootToSolverDistance);
            _nativeScopeTransitionFinalTraceSequence =
                _nativeScopeTransitionTraceSequence;
            _nativeScopeTransitionFinalTraceSample = sampleIndex;
            _nativeScopeTransitionFinalTracePending = true;
            --_nativeScopeTransitionTraceFramesRemaining;
        }
    }


    bool TwoHandedGrip::tryGetPostFrikNativeRightWeaponLocal(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        RE::NiTransform& outWeaponLocal) const
    {
        const auto& captured = _postFrikNativeRightWeaponLocal;
        if (!captured.valid ||
            !weaponNode ||
            captured.weaponNode != weaponNode ||
            currentWeaponGenerationKey == 0 ||
            captured.weaponGenerationKey != currentWeaponGenerationKey ||
            _currentSourceSchedulerSequence == 0 ||
            captured.schedulerSequence != _currentSourceSchedulerSequence ||
            !isFiniteTransform(captured.local)) {
            return false;
        }

        outWeaponLocal = captured.local;
        return true;
    }


    void TwoHandedGrip::publishCollisionIsolatedRightNativeWeaponIntent(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey)
    {
        if (!_weaponCollisionHandPresentationFromPreviousFrame[1] ||
            !_weaponVisualIntentObserver ||
            !weaponNode ||
            currentWeaponGenerationKey == 0 ||
            _firingHandIsLeft ||
            ownsWeaponTransform() ||
            !isFiniteTransform(weaponNode->local)) {
            return;
        }

        RE::NiNode* rightHand = resolveFirstPersonHandNode(false);
        if (!rightHand || weaponNode->parent != rightHand) {
            return;
        }

        RE::NiTransform physicalRightHandWorld{};
        if (!tryGetSolverHandTransform(false, physicalRightHandWorld) ||
            !isUsableHandAuthorityTransform(physicalRightHandWorld)) {
            return;
        }

        RE::NiTransform nativeWeaponLocal = weaponNode->local;
        (void)tryGetPostFrikNativeRightWeaponLocal(
            weaponNode,
            currentWeaponGenerationKey,
            nativeWeaponLocal);

        const RE::NiTransform requestedWeaponWorld =
            transform_math::composeTransforms(
                physicalRightHandWorld,
                nativeWeaponLocal);
        const RE::NiTransform scaleStableRequestedWeaponWorld =
            weapon_visual_authority_math::preserveLiveWeaponWorldScale(
                weaponNode->world,
                requestedWeaponWorld);
        if (!isFiniteTransform(scaleStableRequestedWeaponWorld)) {
            return;
        }

        /*
         * FRIK has already authored this frame's weapon-local animation, but
         * its parent hand still contains the previous collision presentation.
         * The post-FRIK isolation restore rewrites Weapon.local to keep the
         * weapon world independent of that deferred hand move, so use the
         * separately captured native local here. Preserve the native animation
         * while replacing only the parent basis with the collision-isolated
         * physical hand. Later ROCK-owned grip, return, and gunstock
         * publications naturally supersede this default intent through the
         * same observer.
         */
        _weaponVisualIntentObserver(
            _weaponVisualIntentObserverContext,
            weaponNode,
            scaleStableRequestedWeaponWorld,
            currentWeaponGenerationKey);
    }


    void TwoHandedGrip::traceNativeScopeTransitionFinalState(RE::NiNode* weaponNode)
    {
        if (!_nativeScopeTransitionFinalTracePending) {
            return;
        }
        _nativeScopeTransitionFinalTracePending = false;

        struct HmdRelativeTrace
        {
            RE::NiPoint3 position{};
            bool valid{ false };
        };

        const auto* playerNodes = f4vr::getPlayerNodes();
        RE::NiTransform hmdWorld{};
        const bool hmdWorldValid =
            playerNodes && playerNodes->HmdNode &&
            isFiniteTransform(playerNodes->HmdNode->world);
        if (hmdWorldValid) {
            hmdWorld = playerNodes->HmdNode->world;
        }

        const auto captureWorld = [&hmdWorld, hmdWorldValid](
                                      const RE::NiTransform& world,
                                      const bool worldValid) {
            HmdRelativeTrace trace{};
            if (!hmdWorldValid || !worldValid) {
                return trace;
            }
            trace.position = transform_math::worldPointToLocal(
                hmdWorld,
                world.translate);
            trace.valid = std::isfinite(trace.position.x) &&
                          std::isfinite(trace.position.y) &&
                          std::isfinite(trace.position.z);
            return trace;
        };
        const auto captureNode = [&captureWorld](const RE::NiAVObject* node) {
            return captureWorld(
                node ? node->world : RE::NiTransform{},
                node && isFiniteTransform(node->world));
        };

        RE::NiTransform scopeCameraWorld{};
        bool scopeCameraWorldValid = false;
        if (playerNodes && playerNodes->primaryWeaponScopeCamera) {
            const auto* scopeCamera = playerNodes->primaryWeaponScopeCamera;
            scopeCameraWorld = scopeCamera->world;
            if (scopeCamera->parent &&
                isFiniteTransform(scopeCamera->parent->world) &&
                isFiniteTransform(scopeCamera->local)) {
                scopeCameraWorld = transform_math::composeTransforms(
                    scopeCamera->parent->world,
                    scopeCamera->local);
            }
            scopeCameraWorldValid = isFiniteTransform(scopeCameraWorld);
        }

        RE::NiTransform leftRootWorld{};
        RE::NiTransform rightRootWorld{};
        const bool leftRootWorldValid =
            tryGetRootFlattenedHandBoneTransform(true, leftRootWorld);
        const bool rightRootWorldValid =
            tryGetRootFlattenedHandBoneTransform(false, rightRootWorld);
        const auto* playerCamera = f4vr::getPlayerCamera();
        const bool cameraValuesValid =
            playerCamera &&
            std::isfinite(playerCamera->zoomInput) &&
            std::isfinite(playerCamera->worldFOV) &&
            std::isfinite(playerCamera->firstPersonFOV) &&
            std::isfinite(playerCamera->fovAdjustCurrent) &&
            std::isfinite(playerCamera->fovAdjustTarget) &&
            std::isfinite(playerCamera->fovAdjustPerSec) &&
            std::isfinite(playerCamera->fovAnimatorAdjust);

        const HmdRelativeTrace weaponTrace = captureWorld(
            weaponNode ? weaponNode->world : RE::NiTransform{},
            weaponNode && isFiniteTransform(weaponNode->world));
        const HmdRelativeTrace leftRootTrace = captureWorld(
            leftRootWorld,
            leftRootWorldValid);
        const HmdRelativeTrace rightRootTrace = captureWorld(
            rightRootWorld,
            rightRootWorldValid);
        const HmdRelativeTrace scopeCameraTrace = captureWorld(
            scopeCameraWorld,
            scopeCameraWorldValid);
        const HmdRelativeTrace scopeParentTrace = captureNode(
            playerNodes ? playerNodes->ScopeParentNode : nullptr);
        const HmdRelativeTrace cameraRootTrace = captureNode(
            playerCamera ? playerCamera->cameraRoot.get() : nullptr);
        const HmdRelativeTrace roomTrace = captureNode(
            playerNodes ? playerNodes->roomnode : nullptr);
        const HmdRelativeTrace uprightHmdTrace = captureNode(
            playerNodes ? playerNodes->UprightHmdNode : nullptr);
        const HmdRelativeTrace skeletonRootTrace =
            captureNode(f4vr::getRootNode());

        ROCK_LOG_INFO(Weapon,
            "SCOPE-VIEW-FINAL seq={} sample={}/{} buttonRequested={} rendererActive={} menuOpen={} driverAuthority={} state={} hmdValid={} hmdWorld=({:.2f},{:.2f},{:.2f}) weaponHmdValid={} weaponHmd=({:.2f},{:.2f},{:.2f}) leftRootHmdValid={} leftRootHmd=({:.2f},{:.2f},{:.2f}) rightRootHmdValid={} rightRootHmd=({:.2f},{:.2f},{:.2f}) scopeCameraHmdValid={} scopeCameraHmd=({:.2f},{:.2f},{:.2f}) scopeParentHmdValid={} scopeParentHmd=({:.2f},{:.2f},{:.2f}) cameraRootHmdValid={} cameraRootHmd=({:.2f},{:.2f},{:.2f}) roomHmdValid={} roomHmd=({:.2f},{:.2f},{:.2f}) uprightHmdValid={} uprightHmd=({:.2f},{:.2f},{:.2f}) skeletonRootHmdValid={} skeletonRootHmd=({:.2f},{:.2f},{:.2f}) cameraValuesValid={} zoomInput={:.4f} worldFov={:.4f} firstPersonFov={:.4f} fovAdjust=({:.4f},{:.4f},{:.4f},{:.4f})",
            _nativeScopeTransitionFinalTraceSequence,
            _nativeScopeTransitionFinalTraceSample,
            SCOPE_TRANSITION_TRACE_FRAMES,
            _manualScopeActivationRequested ? "yes" : "no",
            _nativeScopeRequestActive ? "yes" : "no",
            _scopeMenuOpenThisFrame ? "yes" : "no",
            _scopeDriverFrameAuthorityActive ? "yes" : "no",
            static_cast<std::uint32_t>(_state),
            hmdWorldValid ? "yes" : "no",
            hmdWorld.translate.x,
            hmdWorld.translate.y,
            hmdWorld.translate.z,
            weaponTrace.valid ? "yes" : "no",
            weaponTrace.position.x,
            weaponTrace.position.y,
            weaponTrace.position.z,
            leftRootTrace.valid ? "yes" : "no",
            leftRootTrace.position.x,
            leftRootTrace.position.y,
            leftRootTrace.position.z,
            rightRootTrace.valid ? "yes" : "no",
            rightRootTrace.position.x,
            rightRootTrace.position.y,
            rightRootTrace.position.z,
            scopeCameraTrace.valid ? "yes" : "no",
            scopeCameraTrace.position.x,
            scopeCameraTrace.position.y,
            scopeCameraTrace.position.z,
            scopeParentTrace.valid ? "yes" : "no",
            scopeParentTrace.position.x,
            scopeParentTrace.position.y,
            scopeParentTrace.position.z,
            cameraRootTrace.valid ? "yes" : "no",
            cameraRootTrace.position.x,
            cameraRootTrace.position.y,
            cameraRootTrace.position.z,
            roomTrace.valid ? "yes" : "no",
            roomTrace.position.x,
            roomTrace.position.y,
            roomTrace.position.z,
            uprightHmdTrace.valid ? "yes" : "no",
            uprightHmdTrace.position.x,
            uprightHmdTrace.position.y,
            uprightHmdTrace.position.z,
            skeletonRootTrace.valid ? "yes" : "no",
            skeletonRootTrace.position.x,
            skeletonRootTrace.position.y,
            skeletonRootTrace.position.z,
            cameraValuesValid ? "yes" : "no",
            cameraValuesValid ? playerCamera->zoomInput : 0.0f,
            cameraValuesValid ? playerCamera->worldFOV : 0.0f,
            cameraValuesValid ? playerCamera->firstPersonFOV : 0.0f,
            cameraValuesValid ? playerCamera->fovAdjustCurrent : 0.0f,
            cameraValuesValid ? playerCamera->fovAdjustTarget : 0.0f,
            cameraValuesValid ? playerCamera->fovAdjustPerSec : 0.0f,
            cameraValuesValid ? playerCamera->fovAnimatorAdjust : 0.0f);
    }


    void TwoHandedGrip::deferOrClearHandAuthorityRole(
        const scope_safe_hand_frame_math::HandAuthorityRole role,
        const bool isLeft)
    {
        // A scope edge keeps one authority owner until its replacement lands.
        if (_scopeMenuOpenThisFrame || _scopeMenuClosedThisFrame) {
            deferScopeHandAuthorityClear(role, isLeft);
            return;
        }
        (void)clearHandAuthorityRoleNow(role, isLeft);
    }


    void TwoHandedGrip::deferScopeHandAuthorityClear(
        const scope_safe_hand_frame_math::HandAuthorityRole role,
        const bool isLeft)
    {
        _scopeDeferredHandAuthorityClears[isLeft ? 0u : 1u] |= scope_safe_hand_frame_math::roleMask(role);
    }


    void TwoHandedGrip::recordScopeHandAuthorityPublication(
        const scope_safe_hand_frame_math::HandAuthorityRole role,
        const bool isLeft)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        const auto roleBit = scope_safe_hand_frame_math::roleMask(role);
        _scopeHandAuthorityPublishedThisFrame[index] |= roleBit;
        // A successfully republished role is live again; a clear requested for
        // the same role while ScopeMenu was open is obsolete.
        _scopeDeferredHandAuthorityClears[index] &= static_cast<scope_safe_hand_frame_math::HandAuthorityRoleMask>(~roleBit);
    }


    bool TwoHandedGrip::clearHandAuthorityRoleNow(
        const scope_safe_hand_frame_math::HandAuthorityRole role,
        const bool isLeft)
    {
        const char* tag = nullptr;
        switch (role) {
        case scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip:
            tag = PRIMARY_GRIP_TAG;
            break;
        case scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip:
            tag = SUPPORT_GRIP_TAG;
            break;
        case scope_safe_hand_frame_math::HandAuthorityRole::PrimaryDetach:
            tag = PRIMARY_DETACH_TAG;
            break;
        }

        if (!tag || !frik_visual_authority::clearExternalHandWorldTransform(tag, handFromBool(isLeft))) {
            return false;
        }

        if (role == scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip) {
            clearPreFrikRetainedHandAuthority(
                RetainedHandAuthorityKind::PrimaryGrip,
                isLeft);
        } else if (role == scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip) {
            clearPreFrikRetainedHandAuthority(
                RetainedHandAuthorityKind::SupportGrip,
                isLeft);
        }

        const auto roleBit = scope_safe_hand_frame_math::roleMask(role);
        _scopeDeferredHandAuthorityClears[isLeft ? 0u : 1u] &=
            static_cast<scope_safe_hand_frame_math::HandAuthorityRoleMask>(~roleBit);
        return true;
    }


    void TwoHandedGrip::reconcileDeferredScopeHandAuthority(RE::NiNode* weaponNode)
    {
        if (_scopeMenuOpenThisFrame || !frik_visual_authority::isAvailable()) {
            return;
        }

        const auto pendingBefore = _scopeDeferredHandAuthorityClears;
        if (pendingBefore[0] == 0 && pendingBefore[1] == 0) {
            return;
        }

        const scope_safe_hand_frame_math::DesiredHandAuthorityInput ownership{
            .gripping = _state == TwoHandedState::Gripping,
            .primaryHandAuthorityEnabled =
                weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(_authorityMode),
            .firingHandIsLeft = _firingHandIsLeft,
            .leftPartGripActive = partGrip(true).active,
            .rightPartGripActive = partGrip(false).active,
        };
        const ScopeHandAuthorityCleanupVisualSnapshot visualSnapshot =
            captureScopeHandAuthorityCleanupVisuals(weaponNode);

        std::array<scope_safe_hand_frame_math::HandAuthorityRoleMask, 2> cleared{};
        std::array<scope_safe_hand_frame_math::HandAuthorityRoleMask, 2> retained{};
        bool clearAttempted = false;
        constexpr std::array roles{
            scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip,
            scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip,
            scope_safe_hand_frame_math::HandAuthorityRole::PrimaryDetach,
        };

        for (const bool isLeft : { true, false }) {
            const std::size_t index = isLeft ? 0u : 1u;
            const auto desiredRoles = scope_safe_hand_frame_math::desiredRolesForHand(ownership, isLeft);
            for (const auto role : roles) {
                if (!scope_safe_hand_frame_math::hasRole(_scopeDeferredHandAuthorityClears[index], role)) {
                    continue;
                }

                switch (scope_safe_hand_frame_math::resolveDeferredClearAction(
                    role,
                    desiredRoles,
                    _scopeHandAuthorityPublishedThisFrame[index])) {
                case scope_safe_hand_frame_math::DeferredClearAction::RetainLiveRole:
                    _scopeDeferredHandAuthorityClears[index] &=
                        static_cast<scope_safe_hand_frame_math::HandAuthorityRoleMask>(
                            ~scope_safe_hand_frame_math::roleMask(role));
                    retained[index] |= scope_safe_hand_frame_math::roleMask(role);
                    break;
                case scope_safe_hand_frame_math::DeferredClearAction::ClearStaleRole:
                    clearAttempted = true;
                    if (clearHandAuthorityRoleNow(role, isLeft)) {
                        cleared[index] |= scope_safe_hand_frame_math::roleMask(role);
                    }
                    break;
                case scope_safe_hand_frame_math::DeferredClearAction::WaitForReplacementPublication:
                    break;
                }
            }
        }

        // A stale-tag clear can ask hFRIK to restore an arm. Preserve the
        // already-solved weapon/scope frame; live replacement hand authority
        // was published before this reconciliation and remains selected.
        if (clearAttempted) {
            restoreScopeHandAuthorityCleanupVisuals(visualSnapshot);
        }
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: reconciled deferred native-scope hand authority "
            "left(clear=0x{:02X},retain=0x{:02X},pending=0x{:02X}) "
            "right(clear=0x{:02X},retain=0x{:02X},pending=0x{:02X})",
            static_cast<unsigned>(cleared[0]),
            static_cast<unsigned>(retained[0]),
            static_cast<unsigned>(_scopeDeferredHandAuthorityClears[0]),
            static_cast<unsigned>(cleared[1]),
            static_cast<unsigned>(retained[1]),
            static_cast<unsigned>(_scopeDeferredHandAuthorityClears[1]));
    }

}
