#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// Native scope presentation: rigid frame capture/synchronization, overlay calibration and target application, camera follow, and sight anchor refresh.

namespace rock
{
    bool TwoHandedGrip::tryCaptureRootFlattenedPalmWorld(bool isLeft, RE::NiPoint3& outPalmWorld, RE::NiTransform& outHandWorld)
    {
        outPalmWorld = {};
        if (!tryGetRootFlattenedHandBoneTransform(isLeft, outHandWorld)) {
            return false;
        }
        outPalmWorld = computeGrabLegacyPalmPivotAWorldFromHandBasis(outHandWorld, isLeft);
        return true;
    }

    void TwoHandedGrip::clearNativeScopeRigidFrame() { _nativeScopeRigidFrame = {}; }

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
                _nativeScopeAnchorWeaponNode = nullptr;
                _nativeScopeAnchorGenerationKey = 0;
                _nativeScopeAnchorOwnershipKey = 0;
                _nativeScopeAnchorWeaponFormID = 0;
                _nativeScopeAnchorWeaponLocal = {};
                _nativeScopeAnchorSource =
                    native_scope_sight_anchor_policy::AnchorSource::None;
                _nativeScopeAnchorValid = false;
                _nativeScopeFallbackRotationDegrees = {};
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
}
