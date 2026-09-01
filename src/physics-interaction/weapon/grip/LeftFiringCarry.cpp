#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// Left-firing equipped carry: position-only carry solve, weapon node ownership/reparenting, feed-forward weapon publish, and the left recoil capture/apply route.

namespace rock
{
    bool TwoHandedGrip::hasVisualOnlySupportRecoilAssist() const noexcept
    {
        if (_state != TwoHandedState::Gripping ||
            !_activeWeaponNode ||
            _activeWeaponGenerationKey == 0 ||
            _activeEquippedWeaponOwnershipKey == 0 ||
            (_firingHandIsLeft &&
                (!_weaponNodeOwnershipBlockEngaged ||
                    !isManualOwnershipActive()))) {
            return false;
        }

        const WeaponPartGrip& supportGrip = partGrip(!_firingHandIsLeft);
        return supportGrip.weaponGenerationKey ==
                   _activeWeaponGenerationKey &&
               supportGrip.gripSequence != 0 &&
               weapon_support_authority_policy::
                   shouldApplyVisualOnlySupportRecoilAssist(
                       _authorityMode,
                       supportGrip.active,
                       supportGrip.providerPartAuthority.active,
                       supportGrip.attachOnly);
    }

    bool FRIK_CALL TwoHandedGrip::controlWeaponHandRecoil(
        const frik::api::FRIKApi::RecoilSample* const sample,
        frik::api::FRIKApi::RecoilResponse* const outResponse,
        void* const userData) noexcept
    {
        auto* const self = static_cast<TwoHandedGrip*>(userData);
        if (!self ||
            !sample ||
            sample->structSize < sizeof(frik::api::FRIKApi::RecoilSample) ||
            !outResponse) {
            return false;
        }

        RE::NiTransform controlledKickLocal = sample->nativeKickLocal;
        const bool visualOnlySupportRecoilAssist =
            self->hasVisualOnlySupportRecoilAssist() &&
            weapon_recoil_authority_math::tryBuildVisualOnlySupportKick(
                sample->nativeKickLocal,
                controlledKickLocal);

        /*
         * Physical-left carry already owns a recoil route because hFRIK's
         * native right-hand weapon glue is blocked. Close visual support adds
         * the only physical-right route: it changes the primary recoil sample,
         * never the support controller or steady weapon authority.
         */
        const bool leftFiringCarryAuthority =
            self->_weaponNodeOwnershipBlockEngaged &&
            self->_firingHandIsLeft &&
            self->isManualOwnershipActive();
        self->captureLeftFiringWeaponRecoil(controlledKickLocal);
        if (!leftFiringCarryAuthority &&
            !visualOnlySupportRecoilAssist) {
            return false;
        }

        *outResponse = {};
        outResponse->structSize = sizeof(frik::api::FRIKApi::RecoilResponse);
        outResponse->handMask = static_cast<std::uint32_t>(
            frik::api::FRIKApi::RecoilHandMask::Primary);
        outResponse->delivery = frik::api::FRIKApi::RecoilDelivery::Direct;
        outResponse->controlledKickLocal = controlledKickLocal;
        return true;
    }

    void TwoHandedGrip::captureLeftFiringWeaponRecoil(
        const RE::NiTransform& controlledKickLocal) noexcept
    {
        ++_weaponRecoilSampleSequence;
        _leftFiringWeaponRecoilSampleValid = false;
        _leftFiringWeaponRecoilWorldDelta =
            transform_math::makeIdentityTransform<RE::NiTransform>();

        if (!_weaponNodeOwnershipBlockEngaged ||
            !_firingHandIsLeft ||
            !isManualOwnershipActive() ||
            !isFiniteTransform(controlledKickLocal)) {
            return;
        }

        const auto* const playerNodes = f4vr::getPlayerNodes();
        const auto* const kickbackNode = playerNodes ?
            playerNodes->primaryWeaponKickbackRecoilNode :
            nullptr;
        const auto* const kickParent = kickbackNode ? kickbackNode->parent : nullptr;
        const auto* const leftHandedMode =
            f4vr::getIniSetting("bLeftHandedMode:VR");
        if (!playerNodes ||
            !kickParent ||
            !leftHandedMode ||
            !isInvertibleTransform(kickParent->world)) {
            return;
        }

        const bool leftIsNativeOffhand = !leftHandedMode->GetBinary();
        if (leftIsNativeOffhand &&
            (!playerNodes->primaryWandNode ||
                !playerNodes->SecondaryWandNode ||
                !isInvertibleTransform(playerNodes->primaryWandNode->world) ||
                !isInvertibleTransform(playerNodes->SecondaryWandNode->world))) {
            return;
        }

        const RE::NiTransform identity =
            transform_math::makeIdentityTransform<RE::NiTransform>();
        const RE::NiTransform& primaryWandWorld = leftIsNativeOffhand ?
            playerNodes->primaryWandNode->world :
            identity;
        const RE::NiTransform& offhandWandWorld = leftIsNativeOffhand ?
            playerNodes->SecondaryWandNode->world :
            identity;
        const RE::NiTransform recoilWorldDelta =
            weapon_recoil_authority_math::resolveWorldDelta(
                controlledKickLocal,
                kickParent->world,
                primaryWandWorld,
                offhandWandWorld,
                leftIsNativeOffhand);
        if (!isFiniteTransform(recoilWorldDelta) ||
            !isInvertibleTransform(recoilWorldDelta)) {
            return;
        }

        _leftFiringWeaponRecoilWorldDelta = recoilWorldDelta;
        _leftFiringWeaponRecoilSampleValid = true;
    }

    bool TwoHandedGrip::applyLeftFiringWeaponRecoil(RE::NiNode* weaponNode)
    {
        if (!_leftFiringWeaponRecoilReadyThisUpdate) {
            return true;
        }
        _leftFiringWeaponRecoilReadyThisUpdate = false;

        if (_leftFiringWeaponRecoilSupportConstrainedThisUpdate) {
            /*
             * Full two-hand authority already consumed this kick through the
             * primary-hand target while leaving the support-hand target fixed.
             * Reapplying the raw delta here would bypass that solve and restore
             * one-handed recoil for physical-left firing.
             */
            _leftFiringWeaponRecoilSupportConstrainedThisUpdate = false;
            return true;
        }

        if (!weaponNode ||
            !_weaponNodeOwnershipBlockEngaged ||
            !_firingHandIsLeft ||
            !isManualOwnershipActive() ||
            !isFiniteTransform(weaponNode->world) ||
            !isFiniteTransform(_leftFiringWeaponRecoilWorldDelta)) {
            return true;
        }

        const RE::NiTransform identity =
            transform_math::makeIdentityTransform<RE::NiTransform>();
        if (areTransformsNearlyEqual(
                _leftFiringWeaponRecoilWorldDelta,
                identity,
                0.000001f)) {
            return true;
        }

        const RE::NiTransform recoiledWeaponWorld =
            transform_math::composeTransforms(
                _leftFiringWeaponRecoilWorldDelta,
                weaponNode->world);
        if (!isFiniteTransform(recoiledWeaponWorld) ||
            !applyWeaponVisualAuthority(weaponNode, recoiledWeaponWorld)) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "TwoHandedGrip: left-firing weapon recoil publication failed");
            return false;
        }

        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;
        return true;
    }

    bool TwoHandedGrip::publishLeftFiringFeedForwardWeaponPose(RE::NiNode* weaponNode)
    {
        if (!weaponNode || weaponNode != _activeWeaponNode ||
            (_state != TwoHandedState::Gripping && _state != TwoHandedState::PrimaryOnly) ||
            !_firingHandIsLeft || !_hasFiringHandWeaponLocal) {
            return false;
        }

        RE::NiTransform physicalHandWorld{};
        RE::NiTransform presentedHandWorld{};
        RE::NiTransform feedForwardWeaponWorld{};
        if (!tryResolveLeftPositionOnlyCarryFrames(
                weaponNode,
                physicalHandWorld,
                presentedHandWorld,
                feedForwardWeaponWorld)) {
            return false;
        }
        return applyWeaponVisualAuthority(weaponNode, feedForwardWeaponWorld);
    }

    bool TwoHandedGrip::solveLeftFiringWeaponCarry(RE::NiNode* weaponNode)
    {
        if (!weaponNode || !_hasFiringHandWeaponLocal) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing left-firing carry because the captured firing-grip frame is unavailable");
            transitionToInactive(false);
            return false;
        }

        RE::NiTransform physicalHandWorld{};
        RE::NiTransform presentedHandWorld{};
        RE::NiTransform solvedWeaponWorld{};
        RE::NiTransform dampedAimCarrierWorld{};
        if (!tryResolveLeftPositionOnlyCarryFrames(
                weaponNode,
                physicalHandWorld,
                presentedHandWorld,
                solvedWeaponWorld,
                &dampedAimCarrierWorld)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: clearing left-firing carry because the physical hand, native aim frame, or authored seat is unavailable");
            transitionToInactive(false);
            return false;
        }

        if (scope_safe_hand_frame_math::
                shouldPublishLockedHandVisualAuthority(
                    _scopeMenuOpenThisFrame)) {
            (void)publishAuthoredPrimaryFiringGripFingerPose(true);
            if (!frik_visual_authority::
                    applyExternalHandWorldTransform(
                        PRIMARY_GRIP_TAG,
                        frik_visual_authority::Hand::Left,
                        presentedHandWorld,
                        GRIP_HAND_POSE_PRIORITY)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(
                    Weapon,
                    "TwoHandedGrip: clearing left-firing carry because authored left-hand presentation failed");
                transitionToInactive(false);
                return false;
            }
            _leftFiringHandWorldActive = true;
            recordScopeHandAuthorityPublication(
                scope_safe_hand_frame_math::HandAuthorityRole::
                    PrimaryGrip,
                true);
            clearHandVisualReturn(
                true,
                "left-position-only-hand-acquired",
                false);
            recordPublishedHandWorld(true, presentedHandWorld);
        }

        // The weapon is parented under LArm_Hand during left firing. Publish
        // it after the authored wrist so the native/mirrored weapon rotation
        // remains the final frame instead of inheriting the wrist correction.
        if (!applyWeaponVisualAuthority(weaponNode, solvedWeaponWorld)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing left-firing carry because ROCK visual weapon authority failed");
            transitionToInactive(false);
            return false;
        }

        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;

        if (_leftFiringPositionOnlyTracePending) {
            auto* playerNodes = f4vr::getPlayerNodes();
            RE::NiNode* leftWand =
                playerNodes ? playerNodes->SecondaryWandNode : nullptr;
            if (leftWand && isFiniteTransform(leftWand->world) &&
                isInvertibleTransform(dampedAimCarrierWorld)) {
                const RE::NiTransform weaponInLeftAimCarrier =
                    transform_math::composeTransforms(
                        transform_math::invertTransform(
                            dampedAimCarrierWorld),
                        weaponNode->world);
                const RE::NiTransform rightNativeOrientation =
                    left_firing_position_only_math::orientationOnly(
                        _rightNativeWeaponAimFrame.
                            weaponInWandOrientation);
                const RE::NiTransform leftAppliedOrientation =
                    left_firing_position_only_math::orientationOnly(
                        weaponInLeftAimCarrier);
                const auto axisInFrame = [](
                                             const RE::NiTransform& frame,
                                             const RE::NiPoint3& axis) {
                    return transform_math::localVectorToWorld(frame, axis);
                };
                const RE::NiPoint3 rightNativeLateral = axisInFrame(
                    rightNativeOrientation,
                    RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
                const RE::NiPoint3 rightNativeBarrel = axisInFrame(
                    rightNativeOrientation,
                    RE::NiPoint3{ 0.0f, 1.0f, 0.0f });
                const RE::NiPoint3 rightNativeUp = axisInFrame(
                    rightNativeOrientation,
                    RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
                // Weapon-local lateral handedness changes with the mirror;
                // compare right +X with left -X, while +Y/+Z correspond.
                const RE::NiPoint3 leftAppliedLateral = axisInFrame(
                    leftAppliedOrientation,
                    RE::NiPoint3{ -1.0f, 0.0f, 0.0f });
                const RE::NiPoint3 leftAppliedBarrel = axisInFrame(
                    leftAppliedOrientation,
                    RE::NiPoint3{ 0.0f, 1.0f, 0.0f });
                const RE::NiPoint3 leftAppliedUp = axisInFrame(
                    leftAppliedOrientation,
                    RE::NiPoint3{ 0.0f, 0.0f, 1.0f });

                RE::NiPoint3 gripInPhysicalLeftHand =
                    transform_math::localPointToWorld(
                        transform_math::invertTransform(
                            _primaryHandWeaponLocal),
                        _primaryGripLocal);
                gripInPhysicalLeftHand.x +=
                    _handlingSettings.
                        leftFiringAimOffsetXGameUnits;
                gripInPhysicalLeftHand.y +=
                    _handlingSettings.
                        leftFiringAimOffsetYGameUnits;
                gripInPhysicalLeftHand.z +=
                    _handlingSettings.
                        leftFiringAimOffsetZGameUnits;
                const RE::NiPoint3 physicalGripTargetWorld =
                    transform_math::localPointToWorld(
                        physicalHandWorld,
                        gripInPhysicalLeftHand);
                const RE::NiPoint3 appliedGripWorld =
                    transform_math::localPointToWorld(
                        weaponNode->world,
                        _primaryGripLocal);
                const float gripError = std::sqrt(dot(
                    sub(appliedGripWorld,
                        physicalGripTargetWorld),
                    sub(appliedGripWorld,
                        physicalGripTargetWorld)));
                const float authoredHandCorrectionDegrees =
                    weapon_support_acquisition_math::
                        rotationDistanceRadians(
                            physicalHandWorld.rotate,
                            presentedHandWorld.rotate) *
                    RADIANS_TO_DEGREES;
                const float dampedFollowDegrees =
                    weapon_support_acquisition_math::
                        rotationDistanceRadians(
                            leftWand->world.rotate,
                            dampedAimCarrierWorld.rotate) *
                    RADIANS_TO_DEGREES;
                ROCK_LOG_INFO(
                    Weapon,
                    "TwoHandedGrip: left position-only normalization generation={:016X} ownership={:016X} rightNativeAxes=(+X:{:.3f}/{:.3f}/{:.3f},+Y:{:.3f}/{:.3f}/{:.3f},+Z:{:.3f}/{:.3f}/{:.3f}) leftAppliedAxes=(-X:{:.3f}/{:.3f}/{:.3f},+Y:{:.3f}/{:.3f}/{:.3f},+Z:{:.3f}/{:.3f}/{:.3f}) gripError={:.4f}gu authoredHandCorrection={:.2f}deg dampedFollow={:.2f}deg trim=(yaw={:.2f},pitch={:.2f},offset={:.2f}/{:.2f}/{:.2f})",
                    _activeWeaponGenerationKey,
                    _activeEquippedWeaponOwnershipKey,
                    rightNativeLateral.x,
                    rightNativeLateral.y,
                    rightNativeLateral.z,
                    rightNativeBarrel.x,
                    rightNativeBarrel.y,
                    rightNativeBarrel.z,
                    rightNativeUp.x,
                    rightNativeUp.y,
                    rightNativeUp.z,
                    leftAppliedLateral.x,
                    leftAppliedLateral.y,
                    leftAppliedLateral.z,
                    leftAppliedBarrel.x,
                    leftAppliedBarrel.y,
                    leftAppliedBarrel.z,
                    leftAppliedUp.x,
                    leftAppliedUp.y,
                    leftAppliedUp.z,
                    gripError,
                    authoredHandCorrectionDegrees,
                    dampedFollowDegrees,
                    _handlingSettings.leftFiringAimYawDegrees,
                    _handlingSettings.leftFiringAimPitchDegrees,
                    _handlingSettings.
                        leftFiringAimOffsetXGameUnits,
                    _handlingSettings.
                        leftFiringAimOffsetYGameUnits,
                    _handlingSettings.
                        leftFiringAimOffsetZGameUnits);
                _leftFiringPositionOnlyTracePending = false;
            }
        }

        /*
         * Carry-time aim diagnostic (~3s cadence): the final weapon +Y axis in
         * the hFRIK-damped aim carrier. It remains the mirrored native right
         * axis plus only explicit aim trim while the carrier absorbs the hand
         * damping delta shared by the weapon and authored wrist.
         */
        if (++_leftFiringAimLogCounter >= 270) {
            _leftFiringAimLogCounter = 0;
            auto* playerNodes = f4vr::getPlayerNodes();
            RE::NiNode* leftWand =
                playerNodes ? playerNodes->SecondaryWandNode : nullptr;
            if (isInvertibleTransform(dampedAimCarrierWorld) &&
                leftWand && isFiniteTransform(leftWand->world)) {
                const RE::NiTransform weaponInDampedCarrier =
                    transform_math::composeTransforms(
                        transform_math::invertTransform(
                            dampedAimCarrierWorld),
                        weaponNode->world);
                const RE::NiPoint3 barrelNow =
                    transform_math::localVectorToWorld(
                        left_firing_position_only_math::orientationOnly(
                            weaponInDampedCarrier),
                        RE::NiPoint3{ 0.0f, 1.0f, 0.0f });
                const float dampedFollowDegrees =
                    weapon_support_acquisition_math::
                        rotationDistanceRadians(
                            leftWand->world.rotate,
                            dampedAimCarrierWorld.rotate) *
                    RADIANS_TO_DEGREES;
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: left-firing carry aim barrelInDampedCarrier=({:.3f},{:.3f},{:.3f}) dampedFollow={:.2f}deg",
                    barrelNow.x,
                    barrelNow.y,
                    barrelNow.z,
                    dampedFollowDegrees);
            }
        }
        return true;
    }

    bool TwoHandedGrip::tryResolveAuthoredPositionOnlySeatWorld(
        RE::NiTransform& outHandWorld) const
    {
        outHandWorld = {};
        // Carry states only: on drop/holster the session will not resume,
        // so the return keeps its physical endpoint.
        if (_firingHandIsLeft ||
            _rightFiringHandCanonicalSource !=
                RightFiringCanonicalSource::AuthoredAnimation ||
            (_state != TwoHandedState::PrimaryOnly &&
                _state != TwoHandedState::Gripping) ||
            !_activeWeaponNode ||
            !hasRightFiringHandCanonicalFrame(
                _activeWeaponNode,
                _activeWeaponGenerationKey,
                _activeEquippedWeaponOwnershipKey) ||
            !isFiniteTransform(_activeWeaponNode->world)) {
            return false;
        }
        outHandWorld = transform_math::composeTransforms(
            _activeWeaponNode->world,
            _rightFiringHandCanonicalWeaponLocal);
        return isUsableHandAuthorityTransform(outHandWorld);
    }

    bool TwoHandedGrip::captureLeftFiringDampedFollowFrame(
        RE::NiNode* weaponNode,
        const RE::NiTransform& leftWandWorld,
        const RE::NiTransform& physicalLeftHandWorld)
    {
        if (!weaponNode || weaponNode != _activeWeaponNode ||
            !_firingHandIsLeft ||
            _activeEquippedWeaponOwnershipKey == 0 ||
            !isInvertibleTransform(leftWandWorld) ||
            !isUsableHandAuthorityTransform(physicalLeftHandWorld)) {
            return false;
        }

        const RE::NiTransform handInWand =
            transform_math::composeTransforms(
                transform_math::invertTransform(leftWandWorld),
                physicalLeftHandWorld);
        constexpr float kMaxHandToWandDistance = 30.0f;
        if (!isFiniteTransform(handInWand) ||
            std::sqrt(dot(
                handInWand.translate,
                handInWand.translate)) > kMaxHandToWandDistance) {
            return false;
        }

        _leftFiringDampedFollowFrame = LeftFiringDampedFollowFrame{
            .handInWandOrientation =
                left_firing_position_only_math::orientationOnly(
                    handInWand),
            .weaponNodeIdentity = weaponNode,
            .weaponGenerationKey = _activeWeaponGenerationKey,
            .weaponOwnershipKey = _activeEquippedWeaponOwnershipKey,
            .valid = true,
        };
        ROCK_LOG_INFO(
            Weapon,
            "TwoHandedGrip: left firing damped-follow reference captured generation={:016X} ownership={:016X}",
            _activeWeaponGenerationKey,
            _activeEquippedWeaponOwnershipKey);
        return true;
    }

    bool TwoHandedGrip::hasLeftFiringDampedFollowFrame(
        const RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t weaponOwnershipKey) const
    {
        return weaponNode && weaponOwnershipKey != 0 &&
               _leftFiringDampedFollowFrame.valid &&
               _leftFiringDampedFollowFrame.weaponNodeIdentity == weaponNode &&
               _leftFiringDampedFollowFrame.weaponGenerationKey ==
                   weaponGenerationKey &&
               _leftFiringDampedFollowFrame.weaponOwnershipKey ==
                   weaponOwnershipKey &&
               isFiniteTransform(
                   _leftFiringDampedFollowFrame.handInWandOrientation);
    }

    bool TwoHandedGrip::tryResolveLeftPositionOnlyCarryFrames(
        RE::NiNode* weaponNode,
        RE::NiTransform& outPhysicalHandWorld,
        RE::NiTransform& outPresentedHandWorld,
        RE::NiTransform& outWeaponWorld,
        RE::NiTransform* const outDampedAimCarrierWorld)
    {
        outPhysicalHandWorld = {};
        outPresentedHandWorld = {};
        outWeaponWorld = {};
        if (outDampedAimCarrierWorld) {
            *outDampedAimCarrierWorld = {};
        }
        if (!weaponNode || weaponNode != _activeWeaponNode ||
            !_firingHandIsLeft || !_hasFiringHandWeaponLocal ||
            !hasRightNativeWeaponAimFrame(
                weaponNode,
                _activeWeaponGenerationKey,
                _activeEquippedWeaponOwnershipKey) ||
            !isFiniteTransform(_primaryHandWeaponLocal) ||
            !std::isfinite(_primaryGripLocal.x) ||
            !std::isfinite(_primaryGripLocal.y) ||
            !std::isfinite(_primaryGripLocal.z) ||
            !isFiniteTransform(weaponNode->world)) {
            return false;
        }

        RE::NiTransform physicalDriverWorld{};
        if (!tryResolvePhysicalHandFrame(
                true,
                outPhysicalHandWorld,
                physicalDriverWorld)) {
            // Before the first left-hand publication, the scope-safe frame is
            // still a valid fallback only if ROCK owns no left visual output.
            if (_leftFiringHandWorldActive ||
                hasVisualAuthorityForHand(true) ||
                !tryGetSolverHandTransform(true, outPhysicalHandWorld) ||
                !isUsableHandAuthorityTransform(outPhysicalHandWorld)) {
                return false;
            }
        }

        auto* playerNodes = f4vr::getPlayerNodes();
        RE::NiNode* leftWand =
            playerNodes ? playerNodes->SecondaryWandNode : nullptr;
        if (!leftWand || !isFiniteTransform(leftWand->world)) {
            return false;
        }

        if (!hasLeftFiringDampedFollowFrame(
                weaponNode,
                _activeWeaponGenerationKey,
                _activeEquippedWeaponOwnershipKey) &&
            !captureLeftFiringDampedFollowFrame(
                weaponNode,
                leftWand->world,
                outPhysicalHandWorld)) {
            return false;
        }
        const RE::NiTransform dampedAimCarrierWorld =
            left_firing_position_only_math::
                resolveDampedAimCarrierWorld(
                    leftWand->world,
                    _leftFiringDampedFollowFrame.
                        handInWandOrientation,
                    outPhysicalHandWorld);
        if (!isFiniteTransform(dampedAimCarrierWorld)) {
            return false;
        }
        if (outDampedAimCarrierWorld) {
            *outDampedAimCarrierWorld = dampedAimCarrierWorld;
        }

        RE::NiTransform leftWeaponInWand =
            left_firing_position_only_math::
                mirrorRightWeaponInWandOrientation(
                    _rightNativeWeaponAimFrame.
                        weaponInWandOrientation);
        const RE::NiTransform aimTrim =
            makeLeftFiringWandAimTrim(_handlingSettings);
        leftWeaponInWand = transform_math::composeTransforms(
            aimTrim,
            leftWeaponInWand);

        RE::NiPoint3 firingGripInLeftHand =
            transform_math::localPointToWorld(
                transform_math::invertTransform(
                    _primaryHandWeaponLocal),
                _primaryGripLocal);
        firingGripInLeftHand.x +=
            _handlingSettings.leftFiringAimOffsetXGameUnits;
        firingGripInLeftHand.y +=
            _handlingSettings.leftFiringAimOffsetYGameUnits;
        firingGripInLeftHand.z +=
            _handlingSettings.leftFiringAimOffsetZGameUnits;
        const RE::NiPoint3 physicalGripTargetWorld =
            transform_math::localPointToWorld(
                outPhysicalHandWorld,
                firingGripInLeftHand);

        outWeaponWorld =
            left_firing_position_only_math::
                resolveWeaponWorldPositionOnly(
                    dampedAimCarrierWorld,
                    leftWeaponInWand,
                    weaponNode->world,
                    _primaryGripLocal,
                    physicalGripTargetWorld);
        outPresentedHandWorld = transform_math::composeTransforms(
            outWeaponWorld,
            _primaryHandWeaponLocal);
        return isFiniteTransform(outWeaponWorld) &&
               isUsableHandAuthorityTransform(outPhysicalHandWorld) &&
               isUsableHandAuthorityTransform(outPresentedHandWorld);
    }

    RE::NiNode* TwoHandedGrip::resolveFirstPersonHandNode(const bool isLeft)
    {
        auto* firstPersonSkeleton = f4vr::getFirstPersonSkeleton();
        if (!firstPersonSkeleton) {
            return nullptr;
        }
        return f4vr::findNode(firstPersonSkeleton, isLeft ? "LArm_Hand" : "RArm_Hand");
    }

    void TwoHandedGrip::syncFiringHandWeaponNodeOwnership(RE::NiNode* weaponNode)
    {
        const bool wantLeftFiringCarry = _firingHandIsLeft &&
            (_state == TwoHandedState::Gripping || _state == TwoHandedState::PrimaryOnly);

        if (!wantLeftFiringCarry) {
            releaseFiringHandWeaponNodeOwnership(weaponNode);
            return;
        }

        if (!_weaponNodeOwnershipBlockEngaged) {
            if (!frik_visual_authority::blockPrimaryWeaponNodeOwnership(WEAPON_NODE_OWNERSHIP_TAG, true)) {
                // Fail closed: without the FRIK block the weapon node would
                // fight two per-frame owners.
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: left-firing carry aborted because the FRIK weapon-node ownership block is unavailable");
                transitionToInactive(false);
                return;
            }
            _weaponNodeOwnershipBlockEngaged = true;
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: FRIK weapon-node ownership blocked for left-firing carry");
        }

        if (!weaponNode) {
            return;
        }

        RE::NiNode* leftHand = resolveFirstPersonHandNode(true);
        if (!leftHand) {
            return;
        }
        if (weaponNode->parent == leftHand) {
            _weaponNodeReparentedToLeftHand = true;
            return;
        }

        /*
         * Re-parent under LArm_Hand preserving world so the scene graph keeps
         * the weapon riding the firing hand at every point in the frame
         * (native fire/aim sampling included). Same operation FRIK performs
         * for the game's own left-handed mode, minus the mirrored offsets.
         */
        const RE::NiTransform worldBefore = weaponNode->world;
        RE::NiTransform localInLeftHand{};
        if (!tryResolveWeaponRootLocal(
                leftHand,
                worldBefore,
                localInLeftHand)) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                2000,
                "TwoHandedGrip: left-firing weapon reparent rejected an invalid target frame");
            return;
        }
        RE::NiPointer<RE::NiAVObject> detached;
        if (weaponNode->parent) {
            weaponNode->parent->DetachChild(weaponNode, detached);
        }
        leftHand->AttachChild(weaponNode, true);
        weaponNode->local = localInLeftHand;
        weaponNode->world = worldBefore;
        _weaponNodeReparentedToLeftHand = true;
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: equipped weapon node re-parented under LArm_Hand for left-firing carry");
    }

    void TwoHandedGrip::releaseFiringHandWeaponNodeOwnership(RE::NiNode* weaponNode)
    {
        if (_weaponNodeReparentedToLeftHand) {
            RE::NiNode* node = weaponNode ? weaponNode : _activeWeaponNode;
            RE::NiNode* rightHand = resolveFirstPersonHandNode(false);
            if (node && rightHand && node->parent != rightHand) {
                const RE::NiTransform worldBefore = node->world;
                RE::NiTransform localInRightHand{};
                if (!tryResolveWeaponRootLocal(
                        rightHand,
                        worldBefore,
                        localInRightHand)) {
                    ROCK_LOG_SAMPLE_WARN(
                        Weapon,
                        2000,
                        "TwoHandedGrip: right-hand weapon reparent rejected an invalid target frame");
                } else {
                    RE::NiPointer<RE::NiAVObject> detached;
                    if (node->parent) {
                        node->parent->DetachChild(node, detached);
                    }
                    rightHand->AttachChild(node, true);
                    node->local = localInRightHand;
                    node->world = worldBefore;
                    ROCK_LOG_INFO(Weapon, "TwoHandedGrip: equipped weapon node re-parented back under RArm_Hand");
                }
            }
            _weaponNodeReparentedToLeftHand = false;
        }

        if (_weaponNodeOwnershipBlockEngaged) {
            // FRIK also force-reattaches native weapon-node parenting once the
            // block releases (belt and braces for teardown without nodes).
            (void)frik_visual_authority::blockPrimaryWeaponNodeOwnership(WEAPON_NODE_OWNERSHIP_TAG, false);
            _weaponNodeOwnershipBlockEngaged = false;
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: FRIK weapon-node ownership restored");
        }
    }
}
