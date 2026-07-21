#include "physics-interaction/weapon/EquipVisualBridge.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/FrikWeaponOffsetCache.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/LooseWeaponGripZone.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "RockConfig.h"

#include "rock_support/Fo4VrRuntime.h"

namespace rock
{
    namespace
    {
        constexpr std::uint64_t kAppCulledFlag = 0x1ull;
        constexpr int kSubtreeWalkBudget = 1024;
        constexpr int kInstanceSearchDepth = 4;
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
                outMask = rightPose.enabledMask;
            } else {
                frik_visual_authority::FingerLocalTransformOverride right{};
                right.enabledMask = rightPose.enabledMask;
                for (std::size_t index = 0; index < rightPose.localTransforms.size(); ++index) {
                    right.localTransforms[index] = rightPose.localTransforms[index];
                }

                frik_visual_authority::FingerLocalTransformOverride left{};
                if (!frik_visual_authority::mirrorPrimaryWeaponFingerLocalTransforms(right, left) ||
                    left.enabledMask != authored_weapon_grip_library::kCompleteFiringFingerMask) {
                    return false;
                }
                for (std::size_t index = 0; index < outTransforms.size(); ++index) {
                    outTransforms[index] = left.localTransforms[index];
                }
                outMask = left.enabledMask;
            }

            return outMask == authored_weapon_grip_library::kCompleteFiringFingerMask &&
                   std::ranges::all_of(outTransforms, isFiniteTransform);
        }

        [[nodiscard]] RE::NiNode* resolveHandWandNode(bool isLeftHand)
        {
            if (!f4vr::getPlayer()) {
                return nullptr;
            }
            return isLeftHand ? f4vr::getLeftHandNode() : f4vr::getRightHandNode();
        }

        [[nodiscard]] RE::NiNode* resolveWeaponBone()
        {
            auto* firstPersonSkeleton = f4vr::getFirstPersonSkeleton();
            return firstPersonSkeleton ? f4vr::findNode(firstPersonSkeleton, "Weapon") : nullptr;
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

        /*
         * The engine names the built biped-slot weapon 3D "Weapon %s (%08X)"
         * (builder 0x1401c8150); matching the "(%08X)" suffix is exact per
         * weapon form and independent of the %s instance display name.
         */
        [[nodiscard]] RE::NiAVObject* findInstanceNodeByToken(RE::NiAVObject* node, const char* token, int depth, int& budget)
        {
            if (!node || depth < 0 || budget <= 0) {
                return nullptr;
            }
            --budget;
            const char* name = node->name.c_str();
            if (name && *name && std::strstr(name, token)) {
                return node;
            }
            auto* asNode = node->IsNode();
            if (!asNode) {
                return nullptr;
            }
            for (const auto& child : asNode->children) {
                if (!child) {
                    continue;
                }
                if (auto* match = findInstanceNodeByToken(child.get(), token, depth - 1, budget)) {
                    return match;
                }
            }
            return nullptr;
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

        // Re-resolving from the still-live detached model against the
        // filewatch-published cache guarantees that a newly created custom
        // offset also overrides a previously captured authored handoff frame.
        const auto frikLookup =
            frik_weapon_offset_cache::findPrimaryWeaponOffset(input.weapon, model);
        const bool customFrikOffsetPresent =
            frikLookup.found &&
            frikLookup.source == frik_weapon_offset_cache::OffsetSource::CustomFile;
        const auto authoredLookup =
            !customFrikOffsetPresent && input.weapon ?
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
        if (_hasFiringHandWeaponLocal) {
            _firingHandWeaponLocal = resolvedHandWeaponLocal;
        } else if (
            !customFrikOffsetPresent &&
            input.hasFiringHandWeaponLocal &&
            isFiniteTransform(input.firingHandWeaponLocal)) {
            // The frame captured before inventory transfer remains a safe
            // fallback only when no explicit custom correction is present.
            _firingHandWeaponLocal = input.firingHandWeaponLocal;
            _hasFiringHandWeaponLocal = true;
            targetReason = "capturedLooseHoldFallback";
        } else {
            _firingHandWeaponLocal = {};
        }

        _model = input.worldModel;
        _weaponFormID = input.weaponFormID;
        std::snprintf(_instanceNameToken, sizeof(_instanceNameToken), "(%08X)", input.weaponFormID);
        _isLeftHand = input.isLeftHand;
        _elapsedSeconds = 0.0f;
        _blendSeconds = g_rockConfig.rockGrabbedWeaponEquipBridgeBlendSeconds;
        _timeoutSeconds = g_rockConfig.rockGrabbedWeaponEquipBridgeTimeoutSeconds;
        _active = true;

        if (authoredLookup.found &&
            authoredLookup.source == authored_weapon_grip_library::CaptureSource::NativeIdlePreharvest &&
            buildPhysicalHandFingerPose(
                _isLeftHand,
                authoredLookup.rightFiringFingerPose,
                _handoffFingerLocalTransforms,
                _handoffFingerLocalTransformMask)) {
            _handPoseHandoffActive = true;
            if (!publishHandPoseHandoff()) {
                clearHandPoseHandoff("initial-publish-failed", false);
            } else if (_hasFiringHandWeaponLocal) {
                const RE::NiTransform handWorld = transform_math::composeTransforms(
                    model->world,
                    _firingHandWeaponLocal);
                if (!isFiniteTransform(handWorld) ||
                    !frik_visual_authority::applyExternalHandWorldTransform(
                        kHandPoseHandoffTag,
                        handFromBool(_isLeftHand),
                        handWorld,
                        kHandPoseHandoffPriority)) {
                    clearHandPoseHandoff("initial-hand-transform-publish-failed", false);
                }
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
        ROCK_LOG_INFO(Weapon, "EquipVisualBridge begin formID={:08X} hand={} attachedNow={} blend={:.2f}s timeout={:.2f}s target={} exactPoseHandoff={}",
            _weaponFormID, _isLeftHand ? "left" : "right", attachedNow ? "yes" : "no", _blendSeconds, _timeoutSeconds,
            targetReason, _handPoseHandoffActive ? "yes" : "no");
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

    bool EquipVisualBridge::publishHandPoseHandoff()
    {
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

        if (!frik_visual_authority::setHandPoseCustomWithPriority(
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
        return frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(
            kHandPoseHandoffTag,
            hand,
            &exactPose,
            kHandPoseHandoffPriority);
    }

    void EquipVisualBridge::update(float deltaSeconds)
    {
        if (!_active) {
            return;
        }

        _elapsedSeconds += (std::max)(0.0f, deltaSeconds);
        if (_elapsedSeconds >= _timeoutSeconds) {
            clear("timeout", _parent != nullptr);
            return;
        }

        auto* weaponBone = resolveWeaponBone();
        RE::NiAVObject* equippedInstance = nullptr;
        if (weaponBone) {
            int budget = kSubtreeWalkBudget;
            equippedInstance = findInstanceNodeByToken(weaponBone, _instanceNameToken, kInstanceSearchDepth, budget);
        }
        const bool equippedInstanceVisible =
            equippedInstance &&
            f4vr::isNodeVisible(weaponBone) &&
            f4vr::isNodeVisible(equippedInstance);

        RE::NiTransform handoffWeaponWorld{};
        bool hasHandoffWeaponWorld = false;
        if (equippedInstanceVisible && isFiniteTransform(weaponBone->world)) {
            handoffWeaponWorld = weaponBone->world;
            hasHandoffWeaponWorld = true;
            if (_model) {
                clearModel("equipped-3d-visible", _parent != nullptr);
            }
        }

        auto* model = _model.get();
        if (model && !_parent) {
            /*
             * Waiting for the engine's (rare, cross-thread queued) pickup
             * detach. The loose model is still rendered in-world, so there is
             * no visual break to cover yet. Its live world frame remains the
             * exact hand-pose target while we wait.
             */
            if (model->parent) {
                if (isFiniteTransform(model->world)) {
                    handoffWeaponWorld = model->world;
                    hasHandoffWeaponWorld = true;
                }
            } else if (!tryAttachToWorldRoot()) {
                clear("attach-failed", false);
                return;
            }
        } else if (model && model->parent != _parent) {
            // Engine (or a load) re-owned or dropped the node: abandon, never fight it.
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

            RE::NiTransform desiredWorld = transform_math::composeTransforms(handNode->world, _modelInHandLocal);
            // Blend toward the authority-selected firing relation. No valid
            // relation means a first-observation fallback to the engine bone.
            RE::NiTransform blendTarget{};
            bool haveBlendTarget = false;
            if (_hasFiringHandWeaponLocal) {
                RE::NiPoint3 palmWorld{};
                RE::NiTransform rootFlattenedHandWorld{};
                if (TwoHandedGrip::tryCaptureRootFlattenedPalmWorld(_isLeftHand, palmWorld, rootFlattenedHandWorld) &&
                    isFiniteTransform(rootFlattenedHandWorld)) {
                    blendTarget = transform_math::composeTransforms(rootFlattenedHandWorld, transform_math::invertTransform(_firingHandWeaponLocal));
                    haveBlendTarget = isFiniteTransform(blendTarget);
                }
            } else if (weaponBone && isFiniteTransform(weaponBone->world)) {
                // First-ever uncaptured weapon: the native graph will establish
                // ROCK's exact relation after equip. Until then, target only the
                // engine-owned bone; never borrow another weapon's live local.
                blendTarget = weaponBone->world;
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
            handoffWeaponWorld = desiredWorld;
            hasHandoffWeaponWorld = true;
        }

        if (_handPoseHandoffActive) {
            if (!publishHandPoseHandoff()) {
                clearHandPoseHandoff("republish-failed", true);
            } else if (_hasFiringHandWeaponLocal && hasHandoffWeaponWorld) {
                const RE::NiTransform handWorld = transform_math::composeTransforms(
                    handoffWeaponWorld,
                    _firingHandWeaponLocal);
                if (!isFiniteTransform(handWorld) ||
                    !frik_visual_authority::applyExternalHandWorldTransform(
                        kHandPoseHandoffTag,
                        handFromBool(_isLeftHand),
                        handWorld,
                        kHandPoseHandoffPriority)) {
                    clearHandPoseHandoff("hand-transform-publish-failed", true);
                }
            }
        }

        if (!_model && !_handPoseHandoffActive) {
            clear("completed", false);
        }
    }

    void EquipVisualBridge::shutdown()
    {
        clear("shutdown", true);
    }

    void EquipVisualBridge::abandonSceneGraph()
    {
        clear("world-loss", false);
    }

    void EquipVisualBridge::completeHandPoseHandoff(const char* reason)
    {
        if (!_handPoseHandoffActive && !_handPoseBlockEngaged) {
            return;
        }

        clearHandPoseHandoff(reason ? reason : "equipped-pose-acquired", true);
        if (!_model) {
            clear("handoff-completed", false);
        }
    }

    void EquipVisualBridge::clearModel(const char* reason, const bool detachFromParent)
    {
        auto* model = _model.get();
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
    }

    void EquipVisualBridge::clearHandPoseHandoff(const char* reason, const bool logCompletion)
    {
        const bool wasActive = _handPoseHandoffActive || _handPoseBlockEngaged;
        const auto hand = handFromBool(_isLeftHand);
        const char* blockTag = _isLeftHand ? kLeftHandPoseBlockTag : kRightHandPoseBlockTag;
        if (_handPoseHandoffActive || _handPoseBlockEngaged) {
            (void)frik_visual_authority::clearHandPose(kHandPoseHandoffTag, hand);
            (void)frik_visual_authority::clearExternalHandWorldTransform(kHandPoseHandoffTag, hand);
        }
        if (_handPoseBlockEngaged) {
            (void)frik_visual_authority::blockPrimaryHandWeaponPose(blockTag, false);
        }

        if (wasActive && logCompletion) {
            ROCK_LOG_INFO(Weapon, "EquipVisualBridge hand-pose handoff released reason={} formID={:08X} hand={} elapsed={:.3f}s",
                reason ? reason : "unknown", _weaponFormID, _isLeftHand ? "left" : "right", _elapsedSeconds);
        }

        _handoffFingerLocalTransforms = {};
        _handoffFingerLocalTransformMask = 0;
        _handPoseHandoffActive = false;
        _handPoseBlockEngaged = false;
    }

    void EquipVisualBridge::clear(const char* reason, bool detachFromParent)
    {
        const bool wasActive = _active;
        clearModel(reason, detachFromParent);
        clearHandPoseHandoff(reason, false);

        if (wasActive) {
            ROCK_LOG_INFO(Weapon, "EquipVisualBridge cleared reason={} formID={:08X} elapsed={:.3f}s",
                reason ? reason : "unknown", _weaponFormID, _elapsedSeconds);
        }

        _firingHandWeaponLocal = {};
        _hasFiringHandWeaponLocal = false;
        _elapsedSeconds = 0.0f;
        _weaponFormID = 0;
        _instanceNameToken[0] = '\0';
        _isLeftHand = false;
        _active = false;
    }
}
