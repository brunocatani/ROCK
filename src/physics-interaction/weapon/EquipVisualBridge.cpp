#include "physics-interaction/weapon/EquipVisualBridge.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/FrikWeaponOffsetCache.h"
#include "physics-interaction/weapon/LooseWeaponGripZone.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "RockConfig.h"

#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

namespace rock
{
    namespace
    {
        constexpr std::uint64_t kAppCulledFlag = 0x1ull;
        constexpr int kSubtreeWalkBudget = 1024;
        constexpr int kInstanceSearchDepth = 4;

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

        /*
         * The pickup normally detaches the model inline before ActivateRef
         * returns, so it is already orphaned here and attaches immediately.
         * If the engine's remove message got queued cross-thread instead,
         * the model is still parented (and still on screen); update() waits
         * for the detach and attaches then. Either way ROCK never steals a
         * live scene node.
         */
        const bool attachedNow = !model->parent && tryAttachToWorldRoot();
        ROCK_LOG_INFO(Weapon, "EquipVisualBridge begin formID={:08X} hand={} attachedNow={} blend={:.2f}s timeout={:.2f}s target={}",
            _weaponFormID, _isLeftHand ? "left" : "right", attachedNow ? "yes" : "no", _blendSeconds, _timeoutSeconds,
            targetReason);
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

    void EquipVisualBridge::update(float deltaSeconds)
    {
        if (!_active) {
            return;
        }

        auto* model = _model.get();
        if (!model) {
            clear("model-lost", false);
            return;
        }

        _elapsedSeconds += (std::max)(0.0f, deltaSeconds);
        const bool ownsSceneAttachment = _parent != nullptr;

        auto* weaponBone = resolveWeaponBone();
        if (weaponBone) {
            int budget = kSubtreeWalkBudget;
            auto* instance = findInstanceNodeByToken(weaponBone, _instanceNameToken, kInstanceSearchDepth, budget);
            if (instance && f4vr::isNodeVisible(weaponBone) && f4vr::isNodeVisible(instance)) {
                clear("equipped-3d-visible", ownsSceneAttachment);
                return;
            }
        }

        if (_elapsedSeconds >= _timeoutSeconds) {
            clear("timeout", ownsSceneAttachment);
            return;
        }

        if (!ownsSceneAttachment) {
            /*
             * Waiting for the engine's (rare, cross-thread queued) pickup
             * detach. The loose model is still rendered in-world, so there is
             * no visual break to cover yet; the overall timeout bounds this.
             */
            if (model->parent) {
                return;
            }
            if (!tryAttachToWorldRoot()) {
                clear("attach-failed", false);
                return;
            }
        } else if (model->parent != _parent) {
            // Engine (or a load) re-owned or dropped the node: abandon, never fight it.
            clear("parent-changed", false);
            return;
        }

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
    }

    void EquipVisualBridge::shutdown()
    {
        clear("shutdown", true);
    }

    void EquipVisualBridge::abandonSceneGraph()
    {
        clear("world-loss", false);
    }

    void EquipVisualBridge::clear(const char* reason, bool detachFromParent)
    {
        auto* model = _model.get();
        if (detachFromParent && model && _parent && model->parent == _parent) {
            RE::NiPointer<RE::NiAVObject> detached;
            _parent->DetachChild(model, detached);
        }

        if (_active) {
            ROCK_LOG_INFO(Weapon, "EquipVisualBridge cleared reason={} formID={:08X} elapsed={:.3f}s",
                reason ? reason : "unknown", _weaponFormID, _elapsedSeconds);
        }

        _model.reset();
        _parent = nullptr;
        _modelInHandLocal = {};
        _firingHandWeaponLocal = {};
        _hasFiringHandWeaponLocal = false;
        _elapsedSeconds = 0.0f;
        _weaponFormID = 0;
        _instanceNameToken[0] = '\0';
        _isLeftHand = false;
        _active = false;
    }
}
