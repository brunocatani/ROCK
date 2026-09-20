#include "physics-interaction/weapon/EquipVisualBridge.h"
#include "physics-interaction/weapon/VanillaWeaponGripFrame.h"
#include "physics-interaction/hand/HandFingerMirrorMath.h"

#include <algorithm>
#include <cmath>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/EquipVisualBridgePolicy.h"
#include "physics-interaction/weapon/LooseWeaponGripZone.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/telemetry/VanillaWeaponAlignmentTelemetry.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock
{
    namespace
    {
        constexpr std::uint64_t kAppCulledFlag = 0x1ull;
        constexpr int kSubtreeWalkBudget = 1024;
        constexpr const char* kHandPoseHandoffTag = "ROCK_EquipPoseBridge";
        constexpr const char* kRightHandPoseBlockTag = "ROCK_EquipPoseBridgeRight";
        constexpr const char* kLeftHandPoseBlockTag = "ROCK_EquipPoseBridgeLeft";
        constexpr int kHandPoseHandoffPriority = 99;

        [[nodiscard]] frik_visual_authority::Hand handFromBool(const bool isLeft)
        {
            return isLeft ? frik_visual_authority::Hand::Left : frik_visual_authority::Hand::Right;
        }

        [[nodiscard]] bool isFiniteTransform(const RE::NiTransform& transform)
        {
            if (!std::isfinite(transform.translate.x) || !std::isfinite(transform.translate.y) || !std::isfinite(transform.translate.z) ||
                !std::isfinite(transform.scale)) {
                return false;
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return true;
        }

        [[nodiscard]] float rotationDistanceDegrees(
            const RE::NiTransform& lhs,
            const RE::NiTransform& rhs)
        {
            float lhsQuaternion[4]{};
            float rhsQuaternion[4]{};
            transform_math::niRowsToHavokQuaternion(
                lhs.rotate,
                lhsQuaternion);
            transform_math::niRowsToHavokQuaternion(
                rhs.rotate,
                rhsQuaternion);
            const float dot = std::clamp(std::abs(
                lhsQuaternion[0] * rhsQuaternion[0] +
                lhsQuaternion[1] * rhsQuaternion[1] +
                lhsQuaternion[2] * rhsQuaternion[2] +
                lhsQuaternion[3] * rhsQuaternion[3]),
                0.0f,
                1.0f);
            constexpr float kRadiansToDegrees = 57.29577951308232f;
            return 2.0f * std::acos(dot) * kRadiansToDegrees;
        }

        [[nodiscard]] bool buildPhysicalHandFingerPose(
            const bool isLeftHand,
            const authored_weapon_grip_library::FiringFingerPose& rightPose,
            std::array<RE::NiTransform, 15>& outTransforms,
            std::uint16_t& outMask)
        {
            outTransforms = {};
            outMask = 0;
            if (!rightPose.complete()) {
                return false;
            }

            if (!isLeftHand) {
                outTransforms = rightPose.localTransforms;
            } else if (!hand_finger_mirror_math::mirrorFingerLocalsAcrossHands<RE::NiTransform>(
                           std::span<const RE::NiTransform>(rightPose.localTransforms),
                           std::span<RE::NiTransform>(outTransforms))) {
                // Exact skeleton mirror; bone order and mask are shared.
                outTransforms = {};
                return false;
            }
            outMask = rightPose.enabledMask;

            return outMask == authored_weapon_grip_library::kCompleteFiringFingerMask &&
                   std::ranges::all_of(outTransforms, isFiniteTransform);
        }

        /*
         * The glue reference must be the WAND (controller device node), never
         * the hand bone. Hand-pose authority changes during the equip - the
         * loose-grab wrap releasing, then the left carry's authored wrist -
         * flip the bone's basis convention by ~180 degrees while the hand
         * still looks correct, and a model glued through the old basis flips
         * with it. The wand basis is authority-independent.
         */
        [[nodiscard]] RE::NiNode* resolveHandWandNode(bool isLeftHand)
        {
            auto* playerNodes = f4vr::getPlayerNodes();
            if (!f4vr::getPlayer() || !playerNodes) {
                return nullptr;
            }
            return isLeftHand ?
                playerNodes->SecondaryWandNode :
                playerNodes->primaryWandNode;
        }

        /*
         * DetachHavok destroyed the loose ref's physics during pickup; any
         * collisionObject left on the orphaned graph is a dangling pointer the
         * moment the queued ref delete runs. The bridge is visual-only, so
         * drop them all before re-entering the scene graph.
         */
        void clearCollisionObjectsRecursive(RE::NiAVObject* node, int& budget)
        {
            if (!node || budget <= 0) {
                return;
            }
            --budget;
            node->collisionObject.reset();
            auto* asNode = node->IsNode();
            if (!asNode) {
                return;
            }
            for (const auto& child : asNode->children) {
                if (child) {
                    clearCollisionObjectsRecursive(child.get(), budget);
                }
            }
        }

        // Blend translate + rotation toward the target; the model keeps its own
        // scale because the equipped clone's scale is engine-owned.
        [[nodiscard]] RE::NiTransform blendWorldTransforms(const RE::NiTransform& from, const RE::NiTransform& to, float t)
        {
            if (t <= 0.0f) {
                return from;
            }
            if (t >= 1.0f) {
                RE::NiTransform result = to;
                result.scale = from.scale;
                return result;
            }

            RE::NiTransform result = from;
            result.translate.x = from.translate.x + (to.translate.x - from.translate.x) * t;
            result.translate.y = from.translate.y + (to.translate.y - from.translate.y) * t;
            result.translate.z = from.translate.z + (to.translate.z - from.translate.z) * t;

            float fromQuat[4] = {};
            float toQuat[4] = {};
            transform_math::niRowsToHavokQuaternion(from.rotate, fromQuat);
            transform_math::niRowsToHavokQuaternion(to.rotate, toQuat);
            const float dot = fromQuat[0] * toQuat[0] + fromQuat[1] * toQuat[1] + fromQuat[2] * toQuat[2] + fromQuat[3] * toQuat[3];
            const float sign = dot < 0.0f ? -1.0f : 1.0f;
            float blended[4] = {};
            for (int i = 0; i < 4; ++i) {
                blended[i] = fromQuat[i] + (toQuat[i] * sign - fromQuat[i]) * t;
            }
            // havokQuaternionToNiRows normalizes, making this an nlerp.
            result.rotate = transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(blended);
            return result;
        }
    }

    EquipVisualBridge::~EquipVisualBridge()
    {
        clear("destroyed", false);
    }

    bool EquipVisualBridge::begin(const BeginInput& input)
    {
        if (_active) {
            clear("superseded", true);
        }

        auto* model = input.worldModel.get();
        if (!model || input.weaponFormID == 0) {
            ROCK_LOG_INFO(Weapon, "EquipVisualBridge begin skipped: model={} formID={:08X}",
                model ? "yes" : "no", input.weaponFormID);
            return false;
        }

        auto* worldRoot = f4vr::getWorldRootNode();
        auto* handNode = resolveHandWandNode(input.isLeftHand);
        if (!worldRoot || !handNode) {
            ROCK_LOG_INFO(Weapon, "EquipVisualBridge begin skipped: worldRoot={} handNode={} formID={:08X}",
                worldRoot ? "yes" : "no", handNode ? "yes" : "no", input.weaponFormID);
            return false;
        }
        if (!isFiniteTransform(model->world) || !isFiniteTransform(handNode->world) || !isFiniteTransform(worldRoot->world)) {
            ROCK_LOG_WARN(Weapon, "EquipVisualBridge begin skipped: non-finite transform formID={:08X}", input.weaponFormID);
            return false;
        }

        _modelInHandLocal = transform_math::composeTransforms(transform_math::invertTransform(handNode->world), model->world);
        RE::NiPoint3 physicalPalmWorld{};
        RE::NiTransform physicalHandWorld{};
        _hasPhysicalHandInWandLocal =
            TwoHandedGrip::tryCaptureRootFlattenedPalmWorld(
                input.isLeftHand,
                physicalPalmWorld,
                physicalHandWorld);
        if (_hasPhysicalHandInWandLocal) {
            _physicalHandInWandLocal = transform_math::composeTransforms(
                transform_math::invertTransform(handNode->world),
                physicalHandWorld);
            _hasPhysicalHandInWandLocal =
                isFiniteTransform(_physicalHandInWandLocal);
        }
        if (!_hasPhysicalHandInWandLocal) {
            _physicalHandInWandLocal = {};
        }

        const auto authoredLookup = input.weapon ?
            authored_weapon_grip_library::find(input.weapon, model, f4vr::isInPowerArmor()) :
            authored_weapon_grip_library::LookupResult{};

        RE::NiTransform resolvedHandWorld{};
        RE::NiTransform resolvedHandWeaponLocal{};
        const char* targetReason = "canonicalHoldUnavailable";
        _hasFiringHandWeaponLocal =
            loose_weapon_grip_zone::tryResolveLooseWeaponFiringHandHoldForModel(
                input.isLeftHand,
                input.weapon,
                model,
                resolvedHandWorld,
                resolvedHandWeaponLocal,
                &targetReason) &&
            isFiniteTransform(resolvedHandWeaponLocal);
        _capturedGrips = input.pairedGrips && input.pairedGrips->valid() ? *input.pairedGrips : weapon_grip_transfer::Pair{};
        if (!_capturedGrips.valid() && input.singleGrip && input.singleGrip->valid()) {
            RE::NiPoint3 sourceTranslation{};
            if (vanilla_weapon_grip_frame::resolveModelTranslation(input.weaponFormID, model, sourceTranslation)) {
                _capturedGrips.primary = *input.singleGrip;
                _capturedGrips.weaponFormID = input.weaponFormID;
                _capturedGrips.firingHandIsLeft = input.isLeftHand;
                _capturedGrips.sourceModelTranslation = sourceTranslation;
            }
        }
        if (_capturedGrips.valid()) {
            _firingHandWeaponLocal = _capturedGrips.primary.handWeaponLocal;
            _hasFiringHandWeaponLocal = true;
            targetReason = "capturedPairedHold";
        } else if (_hasFiringHandWeaponLocal) {
            _firingHandWeaponLocal = resolvedHandWeaponLocal;
        } else if (
            input.hasFiringHandWeaponLocal &&
            isFiniteTransform(input.firingHandWeaponLocal)) {
            // Preserve the authored loose hold captured before inventory transfer.
            _firingHandWeaponLocal = input.firingHandWeaponLocal;
            _hasFiringHandWeaponLocal = true;
            targetReason = "capturedLooseHoldFallback";
        } else {
            _firingHandWeaponLocal = {};
        }

        _model = input.worldModel;
        _weaponFormID = input.weaponFormID;
        _isLeftHand = input.isLeftHand;
        _elapsedSeconds = 0.0f;
        _lifetimeSeconds = 0.0f;
        _blendSeconds = input.blendSeconds;
        _presentationLeaseSeconds =
            equip_visual_bridge_policy::effectivePresentationLeaseSeconds(
                input.timeoutSeconds);
        _presentationLeaseStartedAt = std::chrono::steady_clock::now();
        _modelPresented = true;
        _nativeCarrierTraceLogged = false;
        _nativeCarrierWasUsable = false;
        _active = true;

        if (_capturedGrips.primary.valid()) {
            _handPoseHandoffActive = true;
            if (!publishHandPoseHandoff() || !publishCapturedHandWorld(model, false))
                clearHandPoseHandoff("captured-initial-publish-failed", false, false);
        } else if (authoredLookup.found &&
            authored_weapon_grip_library::isNativeIdleAuthority(authoredLookup.source) &&
            buildPhysicalHandFingerPose(
                _isLeftHand,
                authoredLookup.rightFiringFingerPose,
                _handoffFingerLocalTransforms,
                _handoffFingerLocalTransformMask)) {
            _handPoseHandoffActive = true;
            if (!publishHandPoseHandoff()) {
                clearHandPoseHandoff("initial-publish-failed", false, false);
            }
        }

        /*
         * The pickup normally detaches the model inline before ActivateRef
         * returns, so it is already orphaned here and attaches immediately.
         * If the engine's remove message got queued cross-thread instead,
         * the model is still parented (and still on screen); update() waits
         * for the detach and attaches then. Either way ROCK never steals a
         * live scene node.
         */
        const bool attachedNow = !model->parent && tryAttachToWorldRoot();
        ROCK_LOG_INFO(Weapon, "EquipVisualBridge begin formID={:08X} hand={} attachedNow={} blend={:.2f}s requestedTimeout={:.2f}s presentationLease={:.2f}s target={} exactPoseHandoff={} capturedHands={}",
            _weaponFormID, _isLeftHand ? "left" : "right", attachedNow ? "yes" : "no", _blendSeconds,
            input.timeoutSeconds, _presentationLeaseSeconds,
            targetReason, _handPoseHandoffActive ? "yes" : "no",
            hasCapturedHandPoseHandoff() ? (_capturedGrips.valid() ? 2 : 1) : 0);
        return true;
    }

    bool EquipVisualBridge::tryAttachToWorldRoot()
    {
        auto* model = _model.get();
        auto* worldRoot = f4vr::getWorldRootNode();
        if (!model || model->parent || !worldRoot || !isFiniteTransform(worldRoot->world)) {
            return false;
        }

        int budget = kSubtreeWalkBudget;
        clearCollisionObjectsRecursive(model, budget);

        // Rename so ROCK's own scans and diagnostics can identify/exclude the
        // bridge; the graph is destined for release, the NIF name is expendable.
        model->name = RE::BSFixedString("ROCK_EquipBridge");
        model->flags.flags &= ~kAppCulledFlag;
        model->local = transform_math::composeTransforms(transform_math::invertTransform(worldRoot->world), model->world);
        worldRoot->AttachChild(model, true);
        f4vr::updateDown(model, true);
        _parent = worldRoot;
        return true;
    }

    bool EquipVisualBridge::publishCapturedHandWorld(RE::NiAVObject* model, bool equippedModel)
    {
        if (!_capturedGrips.primary.valid() || !model || !isFiniteTransform(model->world)) return false;
        RE::NiPoint3 displacement{};
        if (equippedModel) {
            if (!vanilla_weapon_grip_frame::resolveModelTranslation(_weaponFormID, model, displacement)) return false;
            displacement -= _capturedGrips.sourceModelTranslation;
        }
        for (const bool primary : { true, false }) {
            if (!primary && !_capturedGrips.valid()) break;
            const bool isLeft = primary ? _isLeftHand : !_isLeftHand;
            auto local = (primary ? _capturedGrips.primary : _capturedGrips.support).handWeaponLocal;
            local.translate += displacement;
            if (!frik_visual_authority::publishHandWorld(kHandPoseHandoffTag, handFromBool(isLeft),
                    transform_math::composeTransforms(model->world, local), kHandPoseHandoffPriority)) return false;
        }
        return true;
    }

    bool EquipVisualBridge::publishHandPoseHandoff()
    {
        if (_handPoseHandoffActive && _capturedGrips.primary.valid()) {
            for (const bool primary : { true, false }) {
                if (!primary && !_capturedGrips.valid()) break;
                const bool isLeft = primary ? _isLeftHand : !_isLeftHand;
                const auto& pose = primary ? _capturedGrips.primary : _capturedGrips.support;
                auto& blocked = primary ? _handPoseBlockEngaged : _pairedSupportBlockEngaged;
                if (!blocked) {
                    if (!frik_visual_authority::blockPrimaryHandWeaponPose(isLeft ? kLeftHandPoseBlockTag : kRightHandPoseBlockTag, true)) return false;
                    blocked = true;
                }
                if (!frik_visual_authority::setHandPoseCustom(kHandPoseHandoffTag, handFromBool(isLeft),
                        frik_visual_authority::makeHandPoseDataFromJointValues(pose.fingerValues), kHandPoseHandoffPriority)) return false;
                if (pose.fingerMask) {
                    frik_visual_authority::FingerLocalTransformOverride locals{};
                    locals.enabledMask = pose.fingerMask;
                    std::copy(pose.fingerLocals.begin(), pose.fingerLocals.end(), std::begin(locals.localTransforms));
                    if (!frik_visual_authority::setHandPoseCustomLocalTransforms(kHandPoseHandoffTag, handFromBool(isLeft),
                            &locals, kHandPoseHandoffPriority)) return false;
                }
            }
            return true;
        }
        if (!_handPoseHandoffActive ||
            _handoffFingerLocalTransformMask != authored_weapon_grip_library::kCompleteFiringFingerMask) {
            return false;
        }

        const auto hand = handFromBool(_isLeftHand);
        const char* blockTag = _isLeftHand ? kLeftHandPoseBlockTag : kRightHandPoseBlockTag;
        if (!_handPoseBlockEngaged) {
            if (!frik_visual_authority::blockPrimaryHandWeaponPose(blockTag, true)) {
                return false;
            }
            _handPoseBlockEngaged = true;
        }

        if (!frik_visual_authority::setHandPoseCustom(
                kHandPoseHandoffTag,
                hand,
                frik_visual_authority::HandPoseData{},
                kHandPoseHandoffPriority)) {
            return false;
        }

        frik_visual_authority::FingerLocalTransformOverride exactPose{};
        exactPose.enabledMask = _handoffFingerLocalTransformMask;
        for (std::size_t index = 0; index < _handoffFingerLocalTransforms.size(); ++index) {
            exactPose.localTransforms[index] = _handoffFingerLocalTransforms[index];
        }
        return frik_visual_authority::setHandPoseCustomLocalTransforms(
            kHandPoseHandoffTag,
            hand,
            &exactPose,
            kHandPoseHandoffPriority);
    }

    bool EquipVisualBridge::advancePresentationLeaseImpl(
        const float deltaSeconds,
        const bool presentedForLogging)
    {
        if (!_active) {
            return false;
        }

        _lifetimeSeconds += (std::max)(0.0f, deltaSeconds);
        const float wallLifetimeSeconds = std::chrono::duration<float>(
            std::chrono::steady_clock::now() -
            _presentationLeaseStartedAt).count();
        _lifetimeSeconds = (std::max)(_lifetimeSeconds, wallLifetimeSeconds);
        if (!equip_visual_bridge_policy::presentationLeaseExpired(
                _lifetimeSeconds,
                _presentationLeaseSeconds)) {
            return false;
        }

        if (presentedForLogging) {
            ROCK_LOG_WARN(Weapon,
                "EquipVisualBridge presentation lease expired while visible formID={:08X} hand={} lease={:.3f}s; releasing visual-only model",
                _weaponFormID,
                _isLeftHand ? "left" : "right",
                _presentationLeaseSeconds);
        } else {
            ROCK_LOG_INFO(Weapon,
                "EquipVisualBridge lease expired after visual handoff formID={:08X} hand={} lease={:.3f}s",
                _weaponFormID,
                _isLeftHand ? "left" : "right",
                _presentationLeaseSeconds);
        }
        clear("presentation-lease-expired", _parent != nullptr);
        return true;
    }

    void EquipVisualBridge::advancePresentationLease(const float deltaSeconds)
    {
        static_cast<void>(advancePresentationLeaseImpl(
            deltaSeconds,
            _modelPresented && _model != nullptr));
    }

    void EquipVisualBridge::update(const UpdateInput& input)
    {
        if (!_active) {
            return;
        }

        const float frameSeconds = (std::max)(0.0f, input.deltaSeconds);
        const bool presentThisFrame = input.presentModel && _model != nullptr;
        if (input.advanceLifetime &&
            advancePresentationLeaseImpl(frameSeconds, presentThisFrame)) {
            return;
        }

        _modelPresented = presentThisFrame;
        synchronizeNativeInstanceCull(input.nativeVisual, _modelPresented);

        if (!_modelPresented) {
            // A false presentation decision is terminal for the loose model.
            // Keeping it as a hidden standby allowed later native graph loss
            // during sheath/drop/throw to resurrect an equip-only phantom.
            clearModel("presentation-ended", _parent != nullptr);
            if (_handPoseHandoffActive) {
                if (!publishHandPoseHandoff() || (_capturedGrips.primary.valid() && input.nativeVisual &&
                        input.nativeVisual->weaponRoot && !publishCapturedHandWorld(input.nativeVisual->weaponRoot, true))) {
                    clearHandPoseHandoff("native-handoff-republish-failed", true, false);
                }
            }
            if (!_model && !_handPoseHandoffActive) {
                clear("completed", false);
            }
            return;
        }

        _elapsedSeconds += frameSeconds;

        auto* model = _model.get();
        if (model && !_parent) {
            if (model->parent) {
                tracePresentation("equip-wait-native-detach");
                vanilla_weapon_alignment_telemetry::recordTransferTrace(
                    vanilla_weapon_alignment_telemetry::TransferKind::HeldEquip, _isLeftHand,
                    "equip-wait-native-instance", input.nativeVisual ? input.nativeVisual->exactInstance : nullptr);
            }
            /*
             * Before ActivateRef's removal has completed, the retained graph
             * is still visible under its original parent. Once it becomes an
             * orphan, ROCK moves it under the world root without stealing a
             * live scene node.
             */
            if (!model->parent && !tryAttachToWorldRoot()) {
                clear("attach-failed", false);
                return;
            }
        } else if (model && model->parent != _parent) {
            clear("parent-changed", false);
            return;
        }

        model = _model.get();
        if (model && _parent) {
            auto* worldRoot = f4vr::getWorldRootNode();
            auto* handNode = resolveHandWandNode(_isLeftHand);
            if (!worldRoot || worldRoot != _parent || !handNode) {
                clear("nodes-missing", true);
                return;
            }

            /*
             * Rotation carrier selection. RIGHT bridges use ROCK's controller
             * basis and the authored grip for placement. LEFT
             * bridges must use the carry's solved pose supplied by the
             * caller: this update runs before ROCK's carry re-poses the node
             * each frame, so a live root read would return the right-glue or
             * draw-animation orientation. Until a carrier is usable the model
             * stays glued to the hand, and the short blend restarts when it
             * becomes usable so the correction converges instead of snapping.
             */
            RE::NiTransform nativeCarrierWorld{};
            bool nativePositionOnlyCarrierAvailable = false;
            if (_isLeftHand) {
                nativePositionOnlyCarrierAvailable =
                    input.leftCarrySolvedWeaponWorldValid &&
                    isFiniteTransform(input.leftCarrySolvedWeaponWorld);
                if (nativePositionOnlyCarrierAvailable) {
                    nativeCarrierWorld = input.leftCarrySolvedWeaponWorld;
                }
            } else {
                nativePositionOnlyCarrierAvailable =
                    _hasFiringHandWeaponLocal && _hasPhysicalHandInWandLocal &&
                    TwoHandedGrip::tryGetRightWeaponAimWorld(_model->world.scale, nativeCarrierWorld);
            }
            if (_isLeftHand &&
                nativePositionOnlyCarrierAvailable &&
                !_nativeCarrierWasUsable) {
                _elapsedSeconds = 0.0f;
            }
            _nativeCarrierWasUsable = nativePositionOnlyCarrierAvailable;

            RE::NiTransform desiredWorld = transform_math::composeTransforms(handNode->world, _modelInHandLocal);
            RE::NiTransform blendTarget{};
            bool haveBlendTarget = false;
            if (!_capturedGrips.valid() && _hasFiringHandWeaponLocal) {
                if (_hasPhysicalHandInWandLocal) {
                    const RE::NiTransform physicalHandWorld =
                        transform_math::composeTransforms(
                            handNode->world,
                            _physicalHandInWandLocal);
                    const RE::NiPoint3 authoredGripWeaponLocal =
                        computeGrabLegacyPalmPivotAWorldFromHandBasis(
                            _firingHandWeaponLocal,
                            _isLeftHand);
                    const RE::NiPoint3 physicalPalmWorld =
                        computeGrabLegacyPalmPivotAWorldFromHandBasis(
                            physicalHandWorld,
                            _isLeftHand);
                    const RE::NiTransform& positionOnlyCarrierWorld =
                        nativePositionOnlyCarrierAvailable ?
                            nativeCarrierWorld :
                            desiredWorld;
                    blendTarget = authored_weapon_grip_capture_policy::
                        resolveAuthoredPrimaryWeaponWorldPositionOnly(
                            positionOnlyCarrierWorld,
                            authoredGripWeaponLocal,
                            physicalPalmWorld,
                            [](const RE::NiTransform& transform,
                                const RE::NiPoint3& point) {
                                return transform_math::localPointToWorld(
                                    transform,
                                    point);
                            });
                    haveBlendTarget = isFiniteTransform(blendTarget);
                    if (nativePositionOnlyCarrierAvailable &&
                        haveBlendTarget &&
                        !_nativeCarrierTraceLogged) {
                        const RE::NiPoint3 looseGripWorld =
                            transform_math::localPointToWorld(
                                desiredWorld,
                                authoredGripWeaponLocal);
                        const float dx =
                            looseGripWorld.x - physicalPalmWorld.x;
                        const float dy =
                            looseGripWorld.y - physicalPalmWorld.y;
                        const float dz =
                            looseGripWorld.z - physicalPalmWorld.z;
                        ROCK_LOG_INFO(Weapon,
                            "EquipVisualBridge native position-only convergence formID={:08X} hand={} rotationDelta={:.2f}deg gripCorrection={:.3f}gu",
                            _weaponFormID,
                            _isLeftHand ? "left" : "right",
                            rotationDistanceDegrees(
                                desiredWorld,
                                positionOnlyCarrierWorld),
                            std::sqrt(dx * dx + dy * dy + dz * dz));
                        _nativeCarrierTraceLogged = true;
                    }
                }
            } else if (!_capturedGrips.valid() && nativePositionOnlyCarrierAvailable) {
                blendTarget = nativeCarrierWorld;
                haveBlendTarget = true;
            }
            if (haveBlendTarget && _blendSeconds > 0.0001f) {
                const float t = (std::min)(1.0f, _elapsedSeconds / _blendSeconds);
                desiredWorld = blendWorldTransforms(desiredWorld, blendTarget, t);
            }
            if (!isFiniteTransform(desiredWorld)) {
                clear("non-finite-pose", true);
                return;
            }

            model->local = transform_math::composeTransforms(transform_math::invertTransform(_parent->world), desiredWorld);
            f4vr::updateDown(model, true);
            vanilla_weapon_alignment_telemetry::recordTransferTrace(
                vanilla_weapon_alignment_telemetry::TransferKind::HeldEquip, _isLeftHand,
                "equip-visual-write", model, &desiredWorld);
        }

        if (_handPoseHandoffActive) {
            // Keep the wrist even while native removal still owns the model's
            // parent. Pose publication does not take ownership of that scene.
            if (!publishHandPoseHandoff() || (_capturedGrips.primary.valid() && !publishCapturedHandWorld(model, false))) {
                clearHandPoseHandoff("republish-failed", true, false);
            }
        }
    }

    bool EquipVisualBridge::ownsNativeInstanceCull(const RE::NiAVObject* node) const noexcept
    {
        return node && _culledNativeInstance.get() == node;
    }

    void EquipVisualBridge::synchronizeNativeInstanceCull(
        const equipped_weapon_visual_state::Snapshot* nativeVisual,
        const bool bridgePresented)
    {
        auto* exactInstance = nativeVisual ? nativeVisual->exactInstance : nullptr;
        if (!bridgePresented || !exactInstance) {
            restoreNativeInstanceCull();
            return;
        }

        if (_culledNativeInstance && _culledNativeInstance.get() != exactInstance) {
            restoreNativeInstanceCull();
        }
        if (_culledNativeInstance) {
            return;
        }
        if (!equipped_weapon_visual_state::isLocallyVisible(exactInstance)) {
            return;
        }

        _culledNativeInstance.reset(exactInstance);
        _culledNativeInstanceWasVisible = true;
        equipped_weapon_visual_state::setLocallyVisible(exactInstance, false);
        f4vr::updateDown(exactInstance, true);
    }

    void EquipVisualBridge::restoreNativeInstanceCull()
    {
        auto* exactInstance = _culledNativeInstance.get();
        if (exactInstance && _culledNativeInstanceWasVisible) {
            equipped_weapon_visual_state::setLocallyVisible(exactInstance, true);
            f4vr::updateDown(exactInstance, true);
        }
        _culledNativeInstance.reset();
        _culledNativeInstanceWasVisible = false;
    }

    void EquipVisualBridge::release(const char* reason)
    {
        if (!_active) {
            return;
        }
        clear(reason ? reason : "watchdog-finished", _parent != nullptr);
    }

    void EquipVisualBridge::shutdown()
    {
        clear("shutdown", true);
    }

    void EquipVisualBridge::abandonSceneGraph()
    {
        clear("world-loss", false, false);
    }

    void EquipVisualBridge::completeHandPoseHandoff(const char* reason)
    {
        if (!_handPoseHandoffActive && !_handPoseBlockEngaged) {
            return;
        }

        clearHandPoseHandoff(reason ? reason : "equipped-pose-acquired", true, false);
        // This is terminal for every hand count. Only begin() starts a new pose
        // handoff; the remaining visual model lease cannot reclaim the wrist.
    }

    void EquipVisualBridge::clearModel(const char* reason, const bool detachFromParent)
    {
        auto* model = _model.get();
        // Abandonment passes false and may refer to an already lost scene.
        if (model && detachFromParent) {
            vanilla_weapon_alignment_telemetry::recordTransferTrace(
                vanilla_weapon_alignment_telemetry::TransferKind::HeldEquip, _isLeftHand,
                reason, model, nullptr, true);
        }
        if (detachFromParent && model && _parent && model->parent == _parent) {
            RE::NiPointer<RE::NiAVObject> detached;
            _parent->DetachChild(model, detached);
        }

        if (model) {
            ROCK_LOG_INFO(Weapon, "EquipVisualBridge model released reason={} formID={:08X} elapsed={:.3f}s",
                reason ? reason : "unknown", _weaponFormID, _elapsedSeconds);
        }

        _model.reset();
        _parent = nullptr;
        _modelInHandLocal = {};
        _physicalHandInWandLocal = {};
        _modelPresented = false;
    }

    void EquipVisualBridge::tracePresentation(const char* phase) const
    {
        if (_model) vanilla_weapon_alignment_telemetry::recordTransferTrace(
            vanilla_weapon_alignment_telemetry::TransferKind::HeldEquip, _isLeftHand, phase, _model.get());
    }

    void EquipVisualBridge::clearHandPoseHandoff(
        const char* reason,
        const bool logCompletion,
        const bool discardPayload)
    {
        const bool wasActive = _handPoseHandoffActive || _handPoseBlockEngaged;
        const auto hand = handFromBool(_isLeftHand);
        const char* blockTag = _isLeftHand ? kLeftHandPoseBlockTag : kRightHandPoseBlockTag;
        if (_handPoseHandoffActive || _handPoseBlockEngaged) {
            (void)frik_visual_authority::clearHandPose(kHandPoseHandoffTag, hand);
            (void)frik_visual_authority::clearHandWorld(kHandPoseHandoffTag, hand);
        }
        if (_handPoseBlockEngaged) {
            (void)frik_visual_authority::blockPrimaryHandWeaponPose(blockTag, false);
        }

        if (wasActive && logCompletion) {
            ROCK_LOG_INFO(Weapon, "EquipVisualBridge hand-pose handoff released reason={} formID={:08X} hand={} elapsed={:.3f}s",
                reason ? reason : "unknown", _weaponFormID, _isLeftHand ? "left" : "right", _elapsedSeconds);
        }

        if (_capturedGrips.valid() || _pairedSupportBlockEngaged) {
            (void)frik_visual_authority::clearHandPose(kHandPoseHandoffTag, handFromBool(!_isLeftHand));
            (void)frik_visual_authority::clearHandWorld(kHandPoseHandoffTag, handFromBool(!_isLeftHand));
            if (_pairedSupportBlockEngaged)
                (void)frik_visual_authority::blockPrimaryHandWeaponPose(_isLeftHand ? kRightHandPoseBlockTag : kLeftHandPoseBlockTag, false);
        }
        _pairedSupportBlockEngaged = false;
        _handPoseHandoffActive = false;
        _handPoseBlockEngaged = false;
        if (discardPayload) {
            _capturedGrips = {};
            _handoffFingerLocalTransforms = {};
            _handoffFingerLocalTransformMask = 0;
        }
    }

    void EquipVisualBridge::clear(
        const char* reason,
        const bool detachFromParent,
        const bool restoreNativeCull)
    {
        const bool wasActive = _active;
        if (restoreNativeCull) {
            restoreNativeInstanceCull();
        } else {
            _culledNativeInstance.reset();
            _culledNativeInstanceWasVisible = false;
        }
        clearModel(reason, detachFromParent);
        clearHandPoseHandoff(reason, false, true);

        if (wasActive) {
            ROCK_LOG_INFO(Weapon, "EquipVisualBridge cleared reason={} formID={:08X} elapsed={:.3f}s",
                reason ? reason : "unknown", _weaponFormID, _elapsedSeconds);
        }

        _firingHandWeaponLocal = {};
        _hasFiringHandWeaponLocal = false;
        _hasPhysicalHandInWandLocal = false;
        _elapsedSeconds = 0.0f;
        _lifetimeSeconds = 0.0f;
        _presentationLeaseStartedAt = {};
        _weaponFormID = 0;
        _isLeftHand = false;
        _modelPresented = false;
        _nativeCarrierTraceLogged = false;
        _nativeCarrierWasUsable = false;
        _active = false;
    }
}
