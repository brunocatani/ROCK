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
        outPalmWorld = computeGrabPalmPointWorldFromHandBasis(outHandWorld, isLeft);
        return true;
    }

    void TwoHandedGrip::clearNativeScopeRigidFrame(bool restoreNativeLocal)
    {
        const auto& frame = _scope.rigidFrame;
        if (restoreNativeLocal && frame.hasAppliedLocal && runtime_state::isLocalSkeletonReady() &&
            RE::PlayerCharacter::GetSingleton()) {
            const auto* nodes = f4vr::getPlayerNodes();
            auto* camera = nodes ? nodes->primaryWeaponScopeCamera : nullptr;
            if (camera && camera == frame.scopeCameraIdentity && camera->parent == frame.cameraParentIdentity &&
                areTransformsNearlyEqual(camera->local, frame.lastAppliedCameraLocal)) {
                camera->local = frame.nativeCameraLocal;
                if (camera->parent) f4vr::updateTransforms(camera);
                else camera->world = camera->local;
            }
        }
        _scope.rigidFrame = {};
    }

    void TwoHandedGrip::clearImmersiveScopePresentation()
    {
        clearNativeScopeOverlayAuthority(true);
        clearNativeScopeRigidFrame(true);
        _scope.anchorWeaponNode = nullptr;
        _scope.anchorGenerationKey = 0;
        _scope.anchorOwnershipKey = 0;
        _scope.anchorWeaponFormID = 0;
        _scope.anchorWeaponLocal = {};
        _scope.anchorSource = native_scope_sight_anchor_policy::AnchorSource::None;
        _scope.anchorValid = false;
        _scope.fallbackRotationDegrees = {};
        _scope.rejectedRigidFrameGeneration = 0;
        _scope.cameraDebugSnapshot = {};
        _scope.activationDebugSnapshot = {};
    }

    bool TwoHandedGrip::rebuildNativeScopeRigidFrameTarget()
    {
        if (!_scope.rigidFrame.valid ||
            !_scope.anchorValid ||
            _scope.rigidFrame.weaponNodeIdentity !=
                _scope.anchorWeaponNode ||
            _scope.rigidFrame.weaponGenerationKey !=
                _scope.anchorGenerationKey) {
            return false;
        }

        RE::NiTransform targetCameraWeaponLocal =
            _scope.rigidFrame.nativeCameraWeaponLocal;
        targetCameraWeaponLocal.translate = _scope.anchorWeaponLocal;
        if (_scope.anchorSource ==
            native_scope_sight_anchor_policy::AnchorSource::
                FiringGripFallback) {
            targetCameraWeaponLocal =
                native_scope_camera_follow_math::
                    applyWeaponLocalRotationOffset(
                        targetCameraWeaponLocal,
                        _scope.fallbackRotationDegrees.x,
                        _scope.fallbackRotationDegrees.y,
                        _scope.fallbackRotationDegrees.z);
        }
        if (!isFiniteTransform(targetCameraWeaponLocal) ||
            std::abs(targetCameraWeaponLocal.scale) <= 0.0001f) {
            return false;
        }

        _scope.rigidFrame.cameraWeaponLocal =
            targetCameraWeaponLocal;
        return true;
    }

    NativeScopeCameraTargetPreviewSnapshot
        TwoHandedGrip::getNativeScopeCameraTargetPreviewSnapshot() const
    {
        const bool valid =
            _scope.rigidFrame.valid &&
            _scope.anchorValid &&
            _scope.rigidFrame.weaponNodeIdentity ==
                _scope.anchorWeaponNode &&
            _scope.rigidFrame.weaponGenerationKey ==
                _scope.anchorGenerationKey &&
            isFiniteTransform(
                _scope.rigidFrame.cameraWeaponLocal) &&
            std::abs(_scope.rigidFrame.cameraWeaponLocal.scale) >
                0.0001f;
        return NativeScopeCameraTargetPreviewSnapshot{
            .weaponGenerationKey =
                _scope.rigidFrame.weaponGenerationKey,
            .equippedWeaponOwnershipKey =
                _scope.anchorOwnershipKey,
            .weaponFormID = _scope.anchorWeaponFormID,
            .anchorSource = _scope.anchorSource,
            .cameraWeaponLocal =
                _scope.rigidFrame.cameraWeaponLocal,
            .valid = valid,
        };
    }

    bool TwoHandedGrip::capturePostFrikNativeScopeRigidFrame(RE::NiNode* weaponNode, const std::uint64_t currentWeaponGenerationKey, RE::NiNode* scopeCamera,
        const RE::NiTransform& nativeCameraWorld)
    {
        if (_scope.rigidFrame.valid && _scope.rigidFrame.weaponGenerationKey == currentWeaponGenerationKey && _scope.rigidFrame.weaponNodeIdentity == weaponNode &&
            _scope.rigidFrame.scopeCameraIdentity == scopeCamera) {
            return true;
        }

        clearNativeScopeRigidFrame();
        if (!weaponNode || !f4vr::isNodeVisible(weaponNode) || currentWeaponGenerationKey == 0 || !_scope.anchorValid || _scope.anchorWeaponNode != weaponNode ||
            _scope.anchorGenerationKey != currentWeaponGenerationKey || !scopeCamera || !isFiniteTransform(weaponNode->world) || !isFiniteTransform(nativeCameraWorld)) {
            return false;
        }

        RE::NiTransform nativeCameraWeaponLocal{};
        if (!native_scope_camera_follow_math::tryCaptureRigidAnchorFrameWeaponLocal(
                weaponNode->world, nativeCameraWorld, _scope.anchorWeaponLocal, nativeCameraWeaponLocal)) {
            if (_scope.rejectedRigidFrameGeneration != currentWeaponGenerationKey) {
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: rejected native scope roll calibration generation={:016X} form={:08X}: invalid camera/weapon basis or scale",
                    currentWeaponGenerationKey, _scope.anchorWeaponFormID);
                _scope.rejectedRigidFrameGeneration = currentWeaponGenerationKey;
            }
            return false;
        }
        _scope.rejectedRigidFrameGeneration = 0;

        _scope.rigidFrame = NativeScopeRigidFrameState{
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
        const auto sampledLocalRotation = transform_math::multiplyStoredRotations(
            nativeCameraWorld.rotate, transform_math::transposeRotation(weaponNode->world.rotate));
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: native scope rigid frame captured generation={:016X} form={:08X} firingHand={} cameraLocal=({:.2f},{:.2f},{:.2f}) scale={:.3f} nativeRollDeg={:.3f} retainedRollDeg={:.3f}",
            currentWeaponGenerationKey, _scope.anchorWeaponFormID, firingHandName(),
            _scope.rigidFrame.cameraWeaponLocal.translate.x, _scope.rigidFrame.cameraWeaponLocal.translate.y,
            _scope.rigidFrame.cameraWeaponLocal.translate.z, _scope.rigidFrame.cameraWeaponLocal.scale,
            native_scope_camera_follow_math::weaponLocalCameraRollDegrees(sampledLocalRotation),
            native_scope_camera_follow_math::weaponLocalCameraRollDegrees(_scope.rigidFrame.cameraWeaponLocal.rotate));
        return true;
    }

    void TwoHandedGrip::observeNativeScopeRigRest()
    {
        auto& rest = _scope.nativeRigRest;
        const auto* playerNodes = f4vr::getPlayerNodes();
        const RE::NiNode* scopeParent = playerNodes ? playerNodes->ScopeParentNode : nullptr;
        const RE::NiNode* scopeCamera = playerNodes ? playerNodes->primaryWeaponScopeCamera : nullptr;
        if (!scopeParent || !scopeCamera) return;
        if (rest.valid && rest.scopeParentIdentity == scopeParent && rest.scopeCameraIdentity == scopeCamera) return;

        // A rebuilt rig has a new rest. ROCK's own overlay local is never one,
        // and a carried rig shows no rest until it returns to the wand chain.
        rest = {};
        const bool onWandChain = scopeParent->parent && scopeParent->parent == playerNodes->primaryUIAttachNode &&
            scopeCamera->parent && scopeCamera->parent == playerNodes->primaryWeaponOffsetNOde;
        const bool rockOwnsLocal = _scope.overlayCalibration.valid && _scope.overlayCalibration.hasAppliedLocal &&
            _scope.overlayCalibration.scopeParentIdentity == scopeParent;
        if (!onWandChain || rockOwnsLocal || !isFiniteTransform(scopeParent->local) || std::abs(scopeParent->local.scale) <= 0.0001f ||
            !std::isfinite(scopeCamera->local.scale) || std::abs(scopeCamera->local.scale) <= 0.0001f) {
            return;
        }
        rest = { .scopeParentIdentity = scopeParent, .scopeCameraIdentity = scopeCamera, .scopeParentLocal = scopeParent->local,
            .cameraScale = scopeCamera->local.scale, .valid = true };
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: native scope rig rest observed scopeParentLocal=({:.2f},{:.2f},{:.2f}) scale={:.3f} cameraScale={:.3f}",
            rest.scopeParentLocal.translate.x, rest.scopeParentLocal.translate.y, rest.scopeParentLocal.translate.z, rest.scopeParentLocal.scale,
            rest.cameraScale);
    }

    void TwoHandedGrip::synchronizeFrikScopeWidgetProvider()
    {
        using Provider = ScopePresentationState::FrikWidgetProvider;
        constexpr const char* kTag = "ROCK_ImmersiveScopes";
        auto& state = _scope.frikWidgetProvider;
        const bool wanted = g_rockConfig.rockEnableImmersiveScopes;
        if (wanted == (state != Provider::Unregistered)) return;
        if (!wanted) {
            if (state == Provider::Registered && !frik_visual_authority::clearScopeProvider(kTag))
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: FRIK refused to clear scope provider '{}'", kTag);
            state = Provider::Unregistered;
            return;
        }
        if (!frik_visual_authority::supportsApiVersion(frik_visual_authority::kPlacesScopeWidgetApiVersion)) {
            ROCK_LOG_WARN(Weapon,
                "TwoHandedGrip: FRIK API v2 is older than v{}; the scope view of a weapon held in the left hand stays black (FRIK hangs the scope widget under the first-person skeleton)",
                frik_visual_authority::kPlacesScopeWidgetApiVersion);
            state = Provider::Unavailable;
            return;
        }
        if (!frik_visual_authority::setScopeProvider(kTag, static_cast<std::uint32_t>(frik_visual_authority::ScopeCapability::PlacesScopeWidget))) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: FRIK refused scope provider '{}' with PlacesScopeWidget", kTag);
            state = Provider::Unavailable;
            return;
        }
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: registered FRIK scope provider '{}' (PlacesScopeWidget)", kTag);
        state = Provider::Registered;
    }

    void TwoHandedGrip::synchronizeNativeScopePresentationAfterFrikUpdate(RE::NiNode* weaponNode, const std::uint64_t currentWeaponGenerationKey)
    {
        observeNativeScopeRigRest();
        synchronizeFrikScopeWidgetProvider();
        if (!g_rockConfig.rockEnableImmersiveScopes) {
            clearImmersiveScopePresentation();
            return;
        }
        if (_scope.secondaryOwnsPresentation) return;
        if (!weaponNode || currentWeaponGenerationKey == 0 || !_scope.anchorValid || _scope.anchorWeaponNode != weaponNode ||
            _scope.anchorGenerationKey != currentWeaponGenerationKey) {
            clearNativeScopeOverlayAuthority(true);
            clearNativeScopeRigidFrame();
            return;
        }

        if ((_scope.overlayCalibration.valid && _scope.overlayCalibration.weaponGenerationKey != currentWeaponGenerationKey) ||
            (_scope.rigidFrame.valid && _scope.rigidFrame.weaponGenerationKey != currentWeaponGenerationKey)) {
            clearNativeScopeOverlayAuthority(true);
            clearNativeScopeRigidFrame();
        }

        /*
         * PlayerCharacter's native scope gate ran earlier in the frame. hFRIK
         * has now aligned optical forward; capture that direction and scale
         * with weapon-relative roll, then publish the complete rigid weapon-local
         * scope frame before FO4VR's later mono render. Earlier weapon solves
         * may only reuse this calibration, never create it. FRIK skips camera
         * alignment for a hidden weapon, so it cannot seed a new calibration.
         */
        const NativeScopeCameraFollowCapture capture = captureNativeScopeCameraFollow(weaponNode);
        if (!capture.valid || !capturePostFrikNativeScopeRigidFrame(weaponNode, currentWeaponGenerationKey, capture.camera, capture.cameraWorldBefore)) {
            return;
        }

        const bool overlayCalibrationReady = captureNativeScopeOverlayCalibration(capture.cameraWorldBefore, currentWeaponGenerationKey);
        const RE::NiTransform targetCameraWorld = native_scope_camera_follow_math::resolveRigidAnchorFrameWorld(weaponNode->world, _scope.rigidFrame.cameraWeaponLocal);
        const NativeScopeCameraFollowResult result = applyNativeScopeCameraWorldTarget(capture, targetCameraWorld, _scope.rigidFrame);
        if (overlayCalibrationReady && result.targetValid && result.writeApplied) {
            (void)applyNativeScopeOverlayTarget(result.targetCameraWorld, currentWeaponGenerationKey);
        }
        if (g_rockConfig.rockDebugDrawNativeScopeActivation) {
            _scope.cameraDebugSnapshot = makeNativeScopeCameraDebugSnapshot(_scope.cameraDebugSnapshot, currentWeaponGenerationKey,
                NativeScopeCameraWriteSource::PostFrikPresentationSync, capture, result, _scope.anchorSource);
        }
    }

    void TwoHandedGrip::synchronizeSecondaryScopePresentation(RE::NiNode* model, const std::uint64_t generation,
        const RE::NiPoint3& anchorModelLocal)
    {
        auto& authority = _scope.secondaryCamera;
        const bool held = g_rockConfig.rockEnableImmersiveScopes && model && generation != 0 &&
            isFiniteTransform(model->world) && std::abs(model->world.scale) > 0.0001f;
        _scope.secondaryOwnsPresentation = held;
        if (!held) {
            if (!authority.valid) return;
            // Hand the camera and overlay back to index 0's presentation.
            const auto* nodes = f4vr::getPlayerNodes();
            auto* camera = nodes ? nodes->primaryWeaponScopeCamera : nullptr;
            if (authority.hasAppliedLocal && camera && camera == authority.scopeCameraIdentity &&
                camera->parent == authority.cameraParentIdentity &&
                areTransformsNearlyEqual(camera->local, authority.lastAppliedCameraLocal)) {
                camera->local = authority.nativeCameraLocal;
                if (camera->parent) f4vr::updateTransforms(camera);
                else camera->world = camera->local;
            }
            if (_scope.overlayCalibration.valid && _scope.overlayCalibration.weaponGenerationKey == authority.weaponGenerationKey)
                clearNativeScopeOverlayAuthority(true);
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: second weapon scope released generation={:016X}", authority.weaponGenerationKey);
            authority = {};
            return;
        }
        const auto capture = captureNativeScopeCameraFollow(model);
        if (!capture.valid) return;
        if (!authority.valid || authority.weaponGenerationKey != generation || authority.weaponNodeIdentity != model ||
            authority.scopeCameraIdentity != capture.camera) {
            // No FRIK alignment exists for this model. Its neutral basis maps
            // camera X to weapon +Y; roll comes from weapon +Z, as captured
            // for index 0 (tryCaptureRigidAnchorFrameWeaponLocal).
            RE::NiMatrix3 forward{};
            forward.entry[0][1] = 1.0f;
            RE::NiTransform cameraLocal{};
            if (!native_scope_camera_follow_math::tryBuildWeaponAlignedCameraRotation(forward, cameraLocal.rotate)) return;
            cameraLocal.scale = capture.cameraWorldBefore.scale / model->world.scale;
            if (!std::isfinite(cameraLocal.scale) || cameraLocal.scale <= 0.0001f) return;
            authority = {};
            authority.weaponGenerationKey = generation;
            authority.weaponNodeIdentity = model;
            authority.scopeCameraIdentity = capture.camera;
            authority.nativeCameraWeaponLocal = cameraLocal;
            authority.cameraWeaponLocal = cameraLocal;
            authority.valid = true;
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: second weapon scope held generation={:016X} anchor=({:.2f},{:.2f},{:.2f})",
                generation, anchorModelLocal.x, anchorModelLocal.y, anchorModelLocal.z);
        }
        authority.cameraWeaponLocal.translate = anchorModelLocal;
        const auto target = native_scope_camera_follow_math::resolveRigidAnchorFrameWorld(model->world, authority.cameraWeaponLocal);
        const auto result = applyNativeScopeCameraWorldTarget(capture, target, authority);
        if (result.targetValid && result.writeApplied && captureNativeScopeOverlayCalibration(capture.cameraWorldBefore, generation))
            (void)applyNativeScopeOverlayTarget(result.targetCameraWorld, generation);
    }

    void TwoHandedGrip::clearNativeScopeOverlayAuthority(const bool restoreNativeLocal)
    {
        if (restoreNativeLocal && _scope.overlayCalibration.valid && _scope.overlayCalibration.hasAppliedLocal && runtime_state::isLocalSkeletonReady() &&
            RE::PlayerCharacter::GetSingleton()) {
            const auto* playerNodes = f4vr::getPlayerNodes();
            auto* scopeParent = playerNodes ? playerNodes->ScopeParentNode : nullptr;
            if (scopeParent == _scope.overlayCalibration.scopeParentIdentity &&
                scopeParent->parent == _scope.overlayCalibration.scopeParentParentIdentity &&
                areTransformsNearlyEqual(scopeParent->local, _scope.overlayCalibration.lastAppliedScopeParentLocal)) {
                scopeParent->local = _scope.overlayCalibration.nativeScopeParentLocal;
                if (scopeParent->parent) {
                    f4vr::updateTransformsDown(scopeParent, true);
                } else {
                    scopeParent->world = scopeParent->local;
                    f4vr::updateTransformsDown(scopeParent, false);
                }
            }
        }

        _scope.overlayCalibration = {};
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
        auto* scopeCamera = playerNodes ? playerNodes->primaryWeaponScopeCamera : nullptr;
        if (!scopeCamera || !scopeCamera->parent || !isFiniteTransform(scopeCamera->local) ||
            !scopeParent || !scopeParent->parent || !isFiniteTransform(scopeParent->local) ||
            std::abs(scopeParent->parent->world.scale) <= 0.0001f) {
            return false;
        }

        auto* scopeModelRoot = f4vr::find1StChildNode(scopeParent, "world_scope.nif");
        if (!scopeModelRoot || scopeModelRoot->parent != scopeParent ||
            !isFiniteTransform(scopeModelRoot->local) || std::abs(scopeModelRoot->local.scale) <= 0.0001f) {
            return false;
        }

        if (_scope.overlayCalibration.valid) {
            const bool sameOwner =
                _scope.overlayCalibration.weaponGenerationKey == currentWeaponGenerationKey &&
                _scope.overlayCalibration.scopeParentIdentity == scopeParent &&
                _scope.overlayCalibration.scopeParentParentIdentity == scopeParent->parent &&
                _scope.overlayCalibration.scopeModelRootIdentity == scopeModelRoot &&
                areTransformsNearlyEqual(scopeModelRoot->local, _scope.overlayCalibration.scopeModelRootLocal);
            const bool engineStillHasRockLocal =
                !_scope.overlayCalibration.hasAppliedLocal ||
                areTransformsNearlyEqual(scopeParent->local, _scope.overlayCalibration.lastAppliedScopeParentLocal);
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

        /*
         * The calibration is the rig's native camera-to-overlay rotation on
         * the wand chain: the unsteered camera under primaryWeaponOffsetNOde
         * against world_scope.nif under ScopeParent at its rest local under
         * primaryUIAttachNode. Both wand-chain nodes stay live while FRIK
         * carries the rig for a left-held weapon, so the calibration is
         * composed from them and the observed rest, never from the carried
         * nodes: those keep the world they had at the carry start, which
         * would bake the right hand's pose into the overlay (runtime 22 Oct
         * 16:56:18, excludedAim 171deg).
         */
        const auto& rest = _scope.nativeRigRest;
        if (!rest.valid || rest.scopeParentIdentity != scopeParent || rest.scopeCameraIdentity != scopeCamera) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000,
                "TwoHandedGrip: native scope overlay left native generation={:016X}: the scope rig's wand-chain rest was not observed",
                currentWeaponGenerationKey);
            return false;
        }
        RE::NiTransform cameraParentWorld{};
        RE::NiTransform scopeParentParentWorld{};
        if (!playerNodes->primaryWeaponOffsetNOde || !playerNodes->primaryUIAttachNode ||
            !tryGetComposedNodeWorld(playerNodes->primaryWeaponOffsetNOde, cameraParentWorld) ||
            !tryGetComposedNodeWorld(playerNodes->primaryUIAttachNode, scopeParentParentWorld)) {
            return false;
        }
        RE::NiTransform cameraRestLocal{};
        cameraRestLocal.scale = rest.cameraScale;
        const auto unsteeredCameraWorld = native_scope_overlay_follow_math::resolveUnsteeredCameraWorld(cameraParentWorld, cameraRestLocal);
        const auto nativeScopeModelRootWorld = transform_math::composeTransforms(
            transform_math::composeTransforms(scopeParentParentWorld, rest.scopeParentLocal), scopeModelRoot->local);
        if (!isFiniteTransform(unsteeredCameraWorld) || std::abs(unsteeredCameraWorld.scale) <= 0.0001f ||
            !isFiniteTransform(nativeScopeModelRootWorld)) {
            return false;
        }
        const RE::NiTransform modelRootCalibrationInCameraLocal =
            native_scope_overlay_follow_math::captureModelRootCalibrationInCameraLocal(unsteeredCameraWorld, nativeScopeModelRootWorld);
        if (!isFiniteTransform(modelRootCalibrationInCameraLocal) || std::abs(modelRootCalibrationInCameraLocal.scale) <= 0.0001f) {
            return false;
        }
        const bool rigOnWandChain = scopeCamera->parent == playerNodes->primaryWeaponOffsetNOde &&
            scopeParent->parent == playerNodes->primaryUIAttachNode;
        // How far FRIK's aim steering turned the native camera; a carried
        // camera aims with the other hand, so the angle means nothing there.
        const float excludedAimDegrees = rigOnWandChain ?
            weapon_support_acquisition_math::rotationDistanceRadians(unsteeredCameraWorld.rotate, nativeCameraWorld.rotate) * RADIANS_TO_DEGREES :
            -1.0f;

        _scope.overlayCalibration = NativeScopeOverlayCalibrationState{
            .weaponGenerationKey = currentWeaponGenerationKey,
            .scopeParentIdentity = scopeParent,
            .scopeModelRootIdentity = scopeModelRoot,
            .scopeParentParentIdentity = scopeParent->parent,
            .scopeModelRootLocal = scopeModelRoot->local,
            .scopeModelRootCalibrationInCameraLocal = modelRootCalibrationInCameraLocal,
            .nativeScopeParentLocal = scopeParent->local,
            .lastAppliedScopeParentLocal = {},
            .valid = true,
            .hasAppliedLocal = false,
        };
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: native scope overlay calibrated generation={:016X} modelRootLocal=({:.2f},{:.2f},{:.2f}) cameraCalibrationScale={:.3f} nativeParentLocal=({:.2f},{:.2f},{:.2f}) basis={} excludedAim={:.2f}deg",
            currentWeaponGenerationKey,
            scopeModelRoot->local.translate.x,
            scopeModelRoot->local.translate.y,
            scopeModelRoot->local.translate.z,
            modelRootCalibrationInCameraLocal.scale,
            scopeParent->local.translate.x,
            scopeParent->local.translate.y,
            scopeParent->local.translate.z,
            rigOnWandChain ? "wand-chain" : "carried-rig",
            excludedAimDegrees);
        return true;
    }

    bool TwoHandedGrip::applyNativeScopeOverlayTarget(
        const RE::NiTransform& correctedCameraWorld,
        const std::uint64_t currentWeaponGenerationKey)
    {
        if (!_scope.overlayCalibration.valid ||
            _scope.overlayCalibration.weaponGenerationKey != currentWeaponGenerationKey ||
            !isFiniteTransform(correctedCameraWorld) || !RE::PlayerCharacter::GetSingleton()) {
            return false;
        }

        const auto* playerNodes = f4vr::getPlayerNodes();
        auto* scopeParent = playerNodes ? playerNodes->ScopeParentNode : nullptr;
        if (!scopeParent || scopeParent != _scope.overlayCalibration.scopeParentIdentity ||
            !scopeParent->parent || std::abs(scopeParent->parent->world.scale) <= 0.0001f) {
            return false;
        }

        auto* scopeModelRoot = f4vr::find1StChildNode(scopeParent, "world_scope.nif");
        if (!scopeModelRoot || scopeModelRoot != _scope.overlayCalibration.scopeModelRootIdentity ||
            scopeModelRoot->parent != scopeParent ||
            !areTransformsNearlyEqual(scopeModelRoot->local, _scope.overlayCalibration.scopeModelRootLocal)) {
            return false;
        }

        if (_scope.overlayCalibration.hasAppliedLocal && !areTransformsNearlyEqual(scopeParent->local, _scope.overlayCalibration.lastAppliedScopeParentLocal)) {
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
                _scope.overlayCalibration.scopeModelRootCalibrationInCameraLocal,
                modelRootFineTuneLocal);
        if (!isFiniteTransform(targetScopeModelRootWorld)) {
            return false;
        }

        const RE::NiTransform targetScopeParentWorld =
            native_scope_overlay_follow_math::resolveScopeParentWorldForModelRoot(
                targetScopeModelRootWorld,
                _scope.overlayCalibration.scopeModelRootLocal);
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
        _scope.overlayCalibration.lastAppliedScopeParentLocal = targetScopeParentLocal;
        _scope.overlayCalibration.hasAppliedLocal = true;

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
        performance_profiler::ScopedTimer gripStageTimer(performance_profiler::Scope::EquippedNativeScopeAnchor);
        if (!g_rockConfig.rockEnableImmersiveScopes) {
            clearImmersiveScopePresentation();
            // Observe native activation for this weapon without a ROCK anchor.
            _scope.anchorGenerationKey = currentWeaponGenerationKey;
            return;
        }
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
            firingGripWeaponLocal = _firing.rightCanonicalGripWeaponLocal;
            firingGripFromCanonical = true;
        } else if (isManualOwnershipActive() &&
                   _session.weaponNode == weaponNode &&
                   _session.weaponGenerationKey == currentWeaponGenerationKey &&
                   _session.equippedWeaponOwnershipKey ==
                       currentEquippedWeaponOwnershipKey &&
                   _firing.primaryGripConfidence > 0.0f &&
                   native_scope_sight_anchor_policy::isFinitePoint(
                       _firing.primaryGripLocal)) {
            firingGripWeaponLocal = _firing.primaryGripLocal;
            firingGripValid = true;
        }

        const bool sameIdentity =
            _scope.anchorWeaponNode == weaponNode &&
            _scope.anchorGenerationKey == currentWeaponGenerationKey &&
            _scope.anchorOwnershipKey ==
                currentEquippedWeaponOwnershipKey &&
            _scope.anchorWeaponFormID == currentEquippedWeaponFormID;
        if (sameIdentity &&
            _scope.anchorSource ==
                native_scope_sight_anchor_policy::AnchorSource::GeneratedSight) {
            return;
        }

        const bool identityChanged = !sameIdentity;
        if (identityChanged) {
            clearNativeScopeOverlayAuthority(true);
            clearNativeScopeRigidFrame();
            _scope.anchorWeaponNode = weaponNode;
            _scope.anchorGenerationKey = currentWeaponGenerationKey;
            _scope.anchorOwnershipKey =
                currentEquippedWeaponOwnershipKey;
            _scope.anchorWeaponFormID = currentEquippedWeaponFormID;
            _scope.anchorWeaponLocal = {};
            _scope.anchorSource =
                native_scope_sight_anchor_policy::AnchorSource::None;
            _scope.anchorValid = false;
            _scope.fallbackRotationDegrees = fallbackRotationDegrees;
        }
        if (!weaponNode || currentWeaponGenerationKey == 0 ||
            currentEquippedWeaponOwnershipKey == 0 ||
            currentEquippedWeaponFormID == 0) {
            return;
        }

        const auto snapshot = weaponCollision.getNativeScopeSightAnchorSnapshot();
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
            _scope.anchorWeaponNode = nullptr;
            _scope.anchorGenerationKey = 0;
            _scope.anchorOwnershipKey = 0;
            _scope.anchorWeaponFormID = 0;
            _scope.anchorWeaponLocal = {};
            _scope.anchorSource =
                native_scope_sight_anchor_policy::AnchorSource::None;
            _scope.anchorValid = false;
            _scope.fallbackRotationDegrees = {};
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

        const auto resolved = native_scope_sight_anchor_policy::resolve(
            snapshot.valid,
            snapshot.anchorWeaponLocal,
            firingGripValid,
            firingGripWeaponLocal,
            fallbackOffsetWeaponLocal);
        if (!resolved.valid) {
            if (_scope.anchorValid) {
                clearNativeScopeOverlayAuthority(true);
                clearNativeScopeRigidFrame();
            }
            _scope.anchorWeaponLocal = {};
            _scope.anchorSource =
                native_scope_sight_anchor_policy::AnchorSource::None;
            _scope.anchorValid = false;
            _scope.fallbackRotationDegrees =
                fallbackRotationDegrees;
            ROCK_LOG_SAMPLE_DEBUG(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "TwoHandedGrip: native scope anchor unavailable generation={:016X} generatedSight={} firingGrip={}; leaving native camera untouched",
                currentWeaponGenerationKey,
                snapshot.valid,
                firingGripValid);
            return;
        }

        const bool sameResolvedAnchor =
            !identityChanged &&
            _scope.anchorValid &&
            _scope.anchorSource == resolved.source &&
            arePointsNearlyEqual(
                _scope.anchorWeaponLocal,
                resolved.weaponLocal);
        const bool fallbackRotationChanged =
            resolved.source ==
                native_scope_sight_anchor_policy::AnchorSource::
                    FiringGripFallback &&
            !arePointsNearlyEqual(
                _scope.fallbackRotationDegrees,
                fallbackRotationDegrees);
        if (sameResolvedAnchor && !fallbackRotationChanged) {
            return;
        }

        if (sameResolvedAnchor) {
            _scope.fallbackRotationDegrees =
                fallbackRotationDegrees;
            if (_scope.rigidFrame.valid &&
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

        _scope.anchorWeaponLocal = resolved.weaponLocal;
        _scope.anchorSource = resolved.source;
        _scope.anchorValid = true;
        _scope.fallbackRotationDegrees =
            fallbackRotationDegrees;
        if (_scope.rigidFrame.valid &&
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
                "TwoHandedGrip: native scope anchor generation={:016X} source=firing-grip-fallback origin={} gripLocal=({:.2f},{:.2f},{:.2f}) offset=({:.2f},{:.2f},{:.2f}) rotation=({:.2f},{:.2f},{:.2f}) anchorLocal=({:.2f},{:.2f},{:.2f})",
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
                _scope.anchorWeaponLocal.x,
                _scope.anchorWeaponLocal.y,
                _scope.anchorWeaponLocal.z);
        }
    }
}
