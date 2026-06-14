#include "physics-interaction/weapon/HeldWeaponEquipVisualHandoff.h"

#include <algorithm>
#include <cmath>
#include <exception>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "f4vr/F4VRUtils.h"

#include "RE/Bethesda/TESObjectREFRs.h"

namespace rock
{
    namespace
    {
        constexpr const char* kPhantomNodeName = "ROCK_HeldWeaponEquipVisualHandoff";
        constexpr float kMaxHandoffSeconds = 0.75f;
        constexpr std::uint32_t kMinHandoffFrames = 3;
        constexpr std::uint32_t kMinEquippedVisualFrames = 4;
        constexpr std::uint32_t kMaxSceneGraphDepth = 32;

        [[nodiscard]] RE::NiTransform makeLocalTransformForParent(const RE::NiNode* parent, const RE::NiTransform& world)
        {
            if (!parent) {
                return world;
            }

            return transform_math::composeTransforms(transform_math::invertTransform(parent->world), world);
        }

        [[nodiscard]] bool isRenderableNode(RE::NiAVObject* node) noexcept
        {
            return node && (node->IsGeometry() || node->IsParticlesGeom());
        }

        void forceVisibleRecursive(RE::NiAVObject* node, std::uint32_t depth) noexcept
        {
            if (!node || depth > kMaxSceneGraphDepth) {
                return;
            }

            f4vr::setNodeVisibility(node, true);
            node->fadeAmount = 1.0f;

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }

            for (auto& child : niNode->children) {
                if (child) {
                    forceVisibleRecursive(child.get(), depth + 1);
                }
            }
        }

        void clearCollisionObjectsRecursive(RE::NiAVObject* node, std::uint32_t depth) noexcept
        {
            if (!node || depth > kMaxSceneGraphDepth) {
                return;
            }

            if (node->collisionObject) {
                node->collisionObject.reset();
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }

            for (auto& child : niNode->children) {
                if (child) {
                    clearCollisionObjectsRecursive(child.get(), depth + 1);
                }
            }
        }

        void setVisible(RE::NiAVObject* node, bool visible) noexcept
        {
            if (node) {
                f4vr::setNodeVisibility(node, visible);
            }
        }
    }

    HeldWeaponEquipVisualHandoff::~HeldWeaponEquipVisualHandoff()
    {
        cancel();
    }

    bool HeldWeaponEquipVisualHandoff::begin(const BeginInput& input) noexcept
    {
        try {
            cancel();
            return beginImpl(input);
        } catch (const std::exception& e) {
            ROCK_LOG_WARN(Weapon, "Held weapon equip visual handoff failed to start ({})", e.what());
        } catch (...) {
            ROCK_LOG_WARN(Weapon, "Held weapon equip visual handoff failed to start");
        }

        resetState();
        return false;
    }

    bool HeldWeaponEquipVisualHandoff::beginImpl(const BeginInput& input)
    {
        const auto& visual = input.visual;
        auto* cloneSourceNode = visual.cloneSourceNode;
        auto* parent = visual.parent;
        if (!visual.isValid() || !cloneSourceNode || !parent) {
            return false;
        }

        f4vr::NiCloneProcess cloneProcess;
        cloneProcess.unk18 = reinterpret_cast<std::uint64_t*>(f4vr::cloneAddr1.address());
        cloneProcess.unk48 = reinterpret_cast<std::uint64_t*>(f4vr::cloneAddr2.address());
        auto* clone = f4vr::cloneNode(cloneSourceNode, &cloneProcess);
        if (!clone) {
            return false;
        }

        _phantomRoot.reset(clone);
        _phantomParent.reset(parent);
        _heldFormID = visual.formID;
        _isLeft = visual.isLeft;
        _phantomRoot->name = RE::BSFixedString(kPhantomNodeName);
        _phantomRoot->fadeAmount = 1.0f;
        clearCollisionObjectsRecursive(_phantomRoot.get(), 0);
        forceVisibleRecursive(_phantomRoot.get(), 0);
        parent->AttachChild(_phantomRoot.get(), true);
        _phantomRoot->local = makeLocalTransformForParent(parent, visual.sourceWorld);
        _phantomRoot->world = visual.sourceWorld;
        _phantomRoot->previousWorld = visual.sourceWorld;
        f4vr::updateDown(_phantomRoot.get(), true);

        _active = true;
        ROCK_LOG_DEBUG(Weapon,
            "Held weapon equip visual handoff started formID={:08X} hand={} parent='{}' source='{}'",
            _heldFormID,
            _isLeft ? "left" : "right",
            parent->name.c_str(),
            visual.source ? visual.source : "none");
        return true;
    }

    void HeldWeaponEquipVisualHandoff::prepareForWeaponCollision(const FrameInput& input) noexcept
    {
        if (!_active) {
            return;
        }

        try {
            prepareForWeaponCollisionImpl(input);
        } catch (const std::exception& e) {
            ROCK_LOG_WARN(Weapon, "Held weapon equip visual handoff cancelled before weapon collision ({})", e.what());
            cancel();
        } catch (...) {
            ROCK_LOG_WARN(Weapon, "Held weapon equip visual handoff cancelled before weapon collision");
            cancel();
        }
    }

    void HeldWeaponEquipVisualHandoff::prepareForWeaponCollisionImpl(const FrameInput& input)
    {
        (void)input;
        restoreHiddenWeaponNodes();
    }

    void HeldWeaponEquipVisualHandoff::updateAfterWeaponCollision(const FrameInput& input) noexcept
    {
        if (!_active) {
            return;
        }

        try {
            updateAfterWeaponCollisionImpl(input);
        } catch (const std::exception& e) {
            ROCK_LOG_WARN(Weapon, "Held weapon equip visual handoff cancelled after weapon collision ({})", e.what());
            cancel();
        } catch (...) {
            ROCK_LOG_WARN(Weapon, "Held weapon equip visual handoff cancelled after weapon collision");
            cancel();
        }
    }

    void HeldWeaponEquipVisualHandoff::updateAfterWeaponCollisionImpl(const FrameInput& input)
    {
        const float dt = std::isfinite(input.deltaSeconds) ? std::clamp(input.deltaSeconds, 0.0f, 0.1f) : 0.0f;
        _elapsedSeconds += dt;
        ++_frames;

        bool capturedEquippedVisual = false;
        auto* equippedRoot = input.equippedWeaponRoot;
        if (equippedRoot) {
            if (_observedEquippedWeaponRoot.get() != equippedRoot) {
                _observedEquippedWeaponRoot.reset(equippedRoot);
                _equippedVisualFrames = 0;
            }
            if (_hiddenWeaponRoot.get() != equippedRoot) {
                restoreHiddenWeaponNodes();
                if (!captureAndHideEquippedWeapon(equippedRoot)) {
                    ROCK_LOG_WARN(Weapon,
                        "Held weapon equip visual handoff cancelled: equipped weapon render node capture overflow formID={:08X}",
                        _heldFormID);
                    cancel();
                    return;
                }
            } else {
                hideCapturedWeaponNodes();
            }
            capturedEquippedVisual = _hiddenVisibleWeaponNodeCount > 0;
        } else {
            _observedEquippedWeaponRoot.reset();
        }

        if (capturedEquippedVisual) {
            ++_equippedVisualFrames;
        } else {
            _equippedVisualFrames = 0;
        }

        if (_frames >= kMinHandoffFrames && _equippedVisualFrames >= kMinEquippedVisualFrames) {
            ROCK_LOG_DEBUG(Weapon,
                "Held weapon equip visual handoff finished formID={:08X} frames={} equippedVisualFrames={} elapsed={:.3f}s hiddenNodes={}",
                _heldFormID,
                _frames,
                _equippedVisualFrames,
                _elapsedSeconds,
                _hiddenWeaponNodeCount);
            cancel();
        } else if (_elapsedSeconds >= kMaxHandoffSeconds) {
            ROCK_LOG_WARN(Weapon,
                "Held weapon equip visual handoff timed out formID={:08X} frames={} equippedVisualFrames={} elapsed={:.3f}s hiddenNodes={}",
                _heldFormID,
                _frames,
                _equippedVisualFrames,
                _elapsedSeconds,
                _hiddenWeaponNodeCount);
            cancel();
        }
    }

    void HeldWeaponEquipVisualHandoff::cancel() noexcept
    {
        restoreHiddenWeaponNodes();
        detachPhantom();
        resetState();
    }

    void HeldWeaponEquipVisualHandoff::resetState() noexcept
    {
        _phantomRoot.reset();
        _phantomParent.reset();
        _hiddenWeaponRoot.reset();
        _observedEquippedWeaponRoot.reset();
        _hiddenWeaponNodeCount = 0;
        _hiddenVisibleWeaponNodeCount = 0;
        _heldFormID = 0;
        _frames = 0;
        _equippedVisualFrames = 0;
        _elapsedSeconds = 0.0f;
        _active = false;
        _isLeft = false;
    }

    void HeldWeaponEquipVisualHandoff::detachPhantom() noexcept
    {
        auto* phantom = _phantomRoot.get();
        if (!phantom) {
            return;
        }

        auto* parent = _phantomParent.get();
        if (!parent) {
            parent = phantom->parent;
        }
        if (parent) {
            RE::NiPointer<RE::NiAVObject> detached;
            parent->DetachChild(phantom, detached);
            f4vr::updateDown(parent, true);
        }
    }

    void HeldWeaponEquipVisualHandoff::restoreHiddenWeaponNodes() noexcept
    {
        for (std::uint32_t i = 0; i < _hiddenWeaponNodeCount; ++i) {
            auto& entry = _hiddenWeaponNodes[i];
            setVisible(entry.node.get(), !entry.wasAppCulled);
            entry.node.reset();
            entry.wasAppCulled = false;
        }

        if (_hiddenWeaponRoot) {
            f4vr::updateDown(_hiddenWeaponRoot.get(), true);
        }
        _hiddenWeaponRoot.reset();
        _hiddenWeaponNodeCount = 0;
        _hiddenVisibleWeaponNodeCount = 0;
    }

    bool HeldWeaponEquipVisualHandoff::captureAndHideEquippedWeapon(RE::NiAVObject* root) noexcept
    {
        if (!root) {
            return true;
        }

        _hiddenWeaponRoot.reset(root);
        _hiddenWeaponNodeCount = 0;
        _hiddenVisibleWeaponNodeCount = 0;
        const bool captured = captureAndHideRenderableDescendants(root, 0);
        if (!captured) {
            restoreHiddenWeaponNodes();
            return false;
        }

        hideCapturedWeaponNodes();
        return true;
    }

    bool HeldWeaponEquipVisualHandoff::captureAndHideRenderableDescendants(RE::NiAVObject* node, std::uint32_t depth) noexcept
    {
        if (!node || depth > kMaxSceneGraphDepth) {
            return true;
        }

        if (depth > 0 && isRenderableNode(node)) {
            if (_hiddenWeaponNodeCount >= _hiddenWeaponNodes.size()) {
                return false;
            }

            auto& entry = _hiddenWeaponNodes[_hiddenWeaponNodeCount++];
            entry.node.reset(node);
            entry.wasAppCulled = node->GetAppCulled();
            if (!entry.wasAppCulled) {
                ++_hiddenVisibleWeaponNodeCount;
            }
            setVisible(node, false);
        }

        auto* niNode = node->IsNode();
        if (!niNode) {
            return true;
        }

        for (auto& child : niNode->children) {
            if (child && !captureAndHideRenderableDescendants(child.get(), depth + 1)) {
                return false;
            }
        }

        return true;
    }

    void HeldWeaponEquipVisualHandoff::hideCapturedWeaponNodes() noexcept
    {
        for (std::uint32_t i = 0; i < _hiddenWeaponNodeCount; ++i) {
            setVisible(_hiddenWeaponNodes[i].node.get(), false);
        }
        if (_hiddenWeaponRoot) {
            f4vr::updateDown(_hiddenWeaponRoot.get(), true);
        }
    }

}
