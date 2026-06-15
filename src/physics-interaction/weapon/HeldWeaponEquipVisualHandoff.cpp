#include "physics-interaction/weapon/HeldWeaponEquipVisualHandoff.h"

#include <algorithm>
#include <cmath>
#include <exception>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

#include "RE/Bethesda/TESObjectREFRs.h"

namespace rock
{
    namespace
    {
        constexpr const char* kPhantomNodeName = "ROCK_HeldWeaponEquipVisualHandoff";
        constexpr float kMaxHandoffSeconds = 0.75f;
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

        [[nodiscard]] bool hasUsableWorldTransform(const RE::NiAVObject* node) noexcept
        {
            return node &&
                   std::isfinite(node->world.translate.x) &&
                   std::isfinite(node->world.translate.y) &&
                   std::isfinite(node->world.translate.z) &&
                   std::isfinite(node->world.scale) &&
                   std::abs(node->world.scale) > 0.0001f;
        }

        [[nodiscard]] bool isVisibleRenderableNode(const RE::NiAVObject* node) noexcept
        {
            return isRenderableNode(const_cast<RE::NiAVObject*>(node)) &&
                   f4vr::isNodeVisible(node) &&
                   !node->GetAppCulled() &&
                   node->local.scale != 0.0f;
        }

        [[nodiscard]] bool hasVisibleRenderableDescendant(RE::NiAVObject* node, std::uint32_t depth) noexcept
        {
            if (!node || depth > kMaxSceneGraphDepth) {
                return false;
            }

            if (depth > 0 && isVisibleRenderableNode(node)) {
                return true;
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return false;
            }

            for (auto& child : niNode->children) {
                if (child && hasVisibleRenderableDescendant(child.get(), depth + 1)) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] RE::NiNode* selectStableHandoffParent() noexcept
        {
            auto usable = [](RE::NiNode* node) noexcept -> RE::NiNode* {
                return hasUsableWorldTransform(node) ? node : nullptr;
            };

            if (f4vr::getPlayer()) {
                auto* playerNodes = f4vr::getPlayerNodes();
                if (auto* node = usable(playerNodes->primaryWeaponOffsetNOde)) {
                    return node;
                }
                if (auto* node = usable(playerNodes->primaryWeapontoWeaponNode)) {
                    return node;
                }
                if (auto* node = usable(playerNodes->roomnode)) {
                    return node;
                }
                if (auto* node = usable(playerNodes->playerworldnode)) {
                    return node;
                }
            }

            if (auto* node = usable(f4vr::getFirstPersonSkeleton())) {
                return node;
            }
            if (auto* node = usable(f4vr::getRootNode())) {
                return node;
            }
            return nullptr;
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
        auto* parent = selectStableHandoffParent();
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
            "Held weapon equip visual handoff started formID={:08X} hand={} stableParent='{}' source='{}'",
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

        auto* equippedRoot = input.equippedWeaponRoot;
        if (equippedRoot) {
            if (_observedEquippedWeaponRoot.get() != equippedRoot) {
                _observedEquippedWeaponRoot.reset(equippedRoot);
                _equippedVisualFrames = 0;
            }
            if (hasVisibleRenderableDescendant(equippedRoot, 0)) {
                ++_equippedVisualFrames;
                ROCK_LOG_DEBUG(Weapon,
                    "Held weapon equip visual handoff finished on native visual formID={:08X} frames={} nativeVisibleFrames={} elapsed={:.3f}s",
                    _heldFormID,
                    _frames,
                    _equippedVisualFrames,
                    _elapsedSeconds);
                cancel();
                return;
            }
        } else {
            _observedEquippedWeaponRoot.reset();
            _equippedVisualFrames = 0;
        }

        if (_elapsedSeconds >= kMaxHandoffSeconds) {
            ROCK_LOG_WARN(Weapon,
                "Held weapon equip visual handoff timed out formID={:08X} frames={} nativeVisibleFrames={} elapsed={:.3f}s",
                _heldFormID,
                _frames,
                _equippedVisualFrames,
                _elapsedSeconds);
            cancel();
        }
    }

    void HeldWeaponEquipVisualHandoff::cancel() noexcept
    {
        detachPhantom();
        resetState();
    }

    void HeldWeaponEquipVisualHandoff::resetState() noexcept
    {
        _phantomRoot.reset();
        _phantomParent.reset();
        _observedEquippedWeaponRoot.reset();
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

}
