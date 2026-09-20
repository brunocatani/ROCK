#include "physics-interaction/weapon/EquippedWeaponDropVisual.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/HeldScenePresentationPolicy.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/EquippedWeaponVisualState.h"
#include "physics-interaction/weapon/telemetry/VanillaWeaponAlignmentTelemetry.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock
{
    namespace
    {
        constexpr const char* kPoseTag = "ROCK_DropVisual";
        constexpr int kPosePriority = 99;

        bool stripDetachedCollision(RE::NiAVObject* model)
        {
            // Qualify the whole detached subtree before changing it. The bound
            // keeps an unexpectedly large or malformed model from partial reuse.
            std::array<RE::NiAVObject*, 1024> nodes{};
            std::size_t count = 1;
            nodes[0] = model;
            for (std::size_t index = 0; index < count; ++index) {
                if (auto* node = nodes[index]->IsNode()) {
                    for (const auto& child : node->children) {
                        if (!child) continue;
                        if (count == nodes.size()) return false;
                        nodes[count++] = child.get();
                    }
                }
            }
            for (std::size_t index = 0; index < count; ++index)
                nodes[index]->collisionObject.reset();
            return true;
        }
    }

    void EquippedWeaponDropVisual::begin(RE::NiPointer<RE::NiAVObject> model,
        const RE::NiTransform& modelInWeapon, const AuthoredWeaponGripPose& grip,
        std::uint32_t referenceId)
    {
        release("superseded");
        if (!model || !grip.valid() || !held_scene_presentation_policy::finiteTransform(modelInWeapon)) {
            ROCK_LOG_WARN(Weapon, "Toggle drop visual unavailable: ref={:08X} model={} grip={}",
                referenceId, model != nullptr, grip.valid());
            return;
        }
        _model = std::move(model);
        _modelInWeapon = modelInWeapon;
        _grip = grip;
        _referenceId = referenceId;
        ROCK_LOG_INFO(Weapon, "Toggle drop visual armed: ref={:08X} hand={} nativeDetached={}",
            _referenceId, _grip.isLeft ? "left" : "right", _model->parent == nullptr);
    }

    void EquippedWeaponDropVisual::update(const RE::NiTransform& physicalHandWorld, RE::NiAVObject* looseRoot)
    {
        if (!_model) return;
        if (looseRoot == _model.get()) {
            release("native-reused-model");
            return;
        }
        auto* worldRoot = f4vr::getWorldRootNode();
        if (!worldRoot || (_parent && _parent.get() != worldRoot)) {
            abandonSceneGraph();
            return;
        }
        const auto weaponWorld = transform_math::composeTransforms(physicalHandWorld,
            transform_math::invertTransform(_grip.placementHandWeaponLocal));
        if (!held_scene_presentation_policy::finiteTransform(physicalHandWorld) ||
            !held_scene_presentation_policy::finiteTransform(weaponWorld)) {
            release("invalid-pose");
            return;
        }
        // Native detachment gates scene reuse, not the hand target. The equipped
        // grip has already released its IK ownership on this first drop frame.
        if (!publishHandPose(weaponWorld)) {
            ROCK_LOG_WARN(Weapon, "Toggle drop visual rejected: ref={:08X} hand pose unavailable", _referenceId);
            release("hand-pose-unavailable");
            return;
        }
        if (!_parent) {
            // Asynchronous native removal still owns this scene until detach.
            if (_model->parent) {
                tracePresentation("drop-wait-native-detach");
                vanilla_weapon_alignment_telemetry::recordTransferTrace(
                    vanilla_weapon_alignment_telemetry::TransferKind::ToggleDrop, _grip.isLeft,
                    "drop-wait-loose-model", looseRoot);
                return;
            }
            if (!stripDetachedCollision(_model.get())) {
                ROCK_LOG_WARN(Weapon, "Toggle drop visual rejected: ref={:08X} subtree exceeds presentation bound", _referenceId);
                release("model-too-large");
                return;
            }
            _model->name = RE::BSFixedString("ROCK_DropBridge");
            worldRoot->AttachChild(_model.get(), true);
            _parent.reset(worldRoot);
        }
        if (_model->parent != _parent.get()) {
            release("model-reparented");
            return;
        }
        const auto modelWorld = transform_math::composeTransforms(weaponWorld, _modelInWeapon);
        const auto modelLocal = transform_math::composeTransforms(
            transform_math::invertTransform(worldRoot->world), modelWorld);
        if (!held_scene_presentation_policy::finiteTransform(modelLocal)) {
            release("invalid-pose");
            return;
        }
        _model->local = modelLocal;
        equipped_weapon_visual_state::setLocallyVisible(_model.get(), true);
        f4vr::updateDown(_model.get(), true);
        vanilla_weapon_alignment_telemetry::recordTransferTrace(
            vanilla_weapon_alignment_telemetry::TransferKind::ToggleDrop, _grip.isLeft,
            "drop-visual-write", _model.get(), &modelWorld);

        if (_hiddenLooseRoot.get() != looseRoot) {
            restoreLooseVisibility();
            if (looseRoot && looseRoot != _model.get()) {
                _hiddenLooseRoot.reset(looseRoot);
                _looseRootWasVisible = equipped_weapon_visual_state::isLocallyVisible(looseRoot);
            }
        }
        if (_hiddenLooseRoot) equipped_weapon_visual_state::setLocallyVisible(_hiddenLooseRoot.get(), false);

        ++_presentedFrames;
    }

    bool EquippedWeaponDropVisual::publishHandPose(const RE::NiTransform& weaponWorld)
    {
        const auto hand = _grip.isLeft ? frik_visual_authority::Hand::Left : frik_visual_authority::Hand::Right;
        frik_visual_authority::FingerLocalTransformOverride fingers{};
        fingers.enabledMask = _grip.fingerMask;
        for (std::size_t index = 0; index < _grip.fingerLocals.size(); ++index)
            fingers.localTransforms[index] = _grip.fingerLocals[index];
        // This tag is removed before the loose grab publishes its own pose.
        _handPoseOwned = true;
        return frik_visual_authority::setHandPoseCustom(kPoseTag, hand, {}, kPosePriority) &&
            frik_visual_authority::setHandPoseCustomLocalTransforms(kPoseTag, hand, &fingers, kPosePriority) &&
            frik_visual_authority::publishHandWorld(kPoseTag, hand,
                transform_math::composeTransforms(weaponWorld, _grip.handWeaponLocal), kPosePriority);
    }

    void EquippedWeaponDropVisual::restoreLooseVisibility()
    {
        if (_hiddenLooseRoot && _looseRootWasVisible)
            equipped_weapon_visual_state::setLocallyVisible(_hiddenLooseRoot.get(), true);
        _hiddenLooseRoot.reset();
        _looseRootWasVisible = false;
    }

    void EquippedWeaponDropVisual::prepareGrab()
    {
        tracePresentation("drop-before-grab");
        // Exact-reference acquisition must inspect the unmodified loose scene.
        // A retry can resume presentation at the end of the same update.
        restoreLooseVisibility();
        yieldHandPose();
    }

    void EquippedWeaponDropVisual::yieldHandPose()
    {
        if (_handPoseOwned) {
            const auto hand = _grip.isLeft ? frik_visual_authority::Hand::Left : frik_visual_authority::Hand::Right;
            (void)frik_visual_authority::clearHandPose(kPoseTag, hand);
            (void)frik_visual_authority::clearHandWorld(kPoseTag, hand);
        }
        _handPoseOwned = false;
    }

    void EquippedWeaponDropVisual::release(const char* reason)
    {
        clear(reason, !_parent || _parent.get() == f4vr::getWorldRootNode());
    }
    void EquippedWeaponDropVisual::abandonSceneGraph() { clear("scene-unavailable", false); }

    void EquippedWeaponDropVisual::clear(const char* reason, bool sceneAvailable)
    {
        if (sceneAvailable && _model) {
            vanilla_weapon_alignment_telemetry::recordTransferTrace(
                vanilla_weapon_alignment_telemetry::TransferKind::ToggleDrop, _grip.isLeft,
                reason, _model.get(), nullptr, true);
        }
        yieldHandPose();
        if (sceneAvailable) {
            restoreLooseVisibility();
            if (_model && _parent && _model->parent == _parent.get()) {
                RE::NiPointer<RE::NiAVObject> detached;
                _parent->DetachChild(_model.get(), detached);
            }
        }
        if (_model) ROCK_LOG_INFO(Weapon, "Toggle drop visual ended: ref={:08X} reason={} presentedFrames={}",
            _referenceId, reason, _presentedFrames);
        _hiddenLooseRoot.reset();
        _model.reset();
        _parent.reset();
        _grip = {};
        _modelInWeapon = {};
        _referenceId = 0;
        _presentedFrames = 0;
        _looseRootWasVisible = false;
        _handPoseOwned = false;
    }

    void EquippedWeaponDropVisual::tracePresentation(const char* phase) const
    {
        if (_model) vanilla_weapon_alignment_telemetry::recordTransferTrace(
            vanilla_weapon_alignment_telemetry::TransferKind::ToggleDrop, _grip.isLeft, phase, _model.get());
    }
}
