// Binds the second weapon model's animation targets privately to its hand,
// while the native model stays the rendered model.
#pragma once

#include <Windows.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>
#include "RE/Bethesda/BSGeometry.h"
#include "RE/Bethesda/BSSkin.h"
#include "RE/NetImmerse/NiCloningProcess.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/SkinnedBoneOwner.h"
#include "physics-interaction/weapon/grip/WeaponAuthority.h"
#include "physics-interaction/weapon/native/NativeEquippedActionGraph.h"
#include "physics-interaction/weapon/scene/WeaponScenePresentation.h"
#include "physics-interaction/weapon/secondary/NativePresentedPoseHistory.h"
#include "physics-interaction/weapon/secondary/NativeSecondaryPoseTransport.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock::native_secondary_actions
{
    // Scene-owner lifetime. The native equipped model remains the rendered
    // model; only its animation targets are private to its equipped hand.
    class BoneBindings
    {
        struct OriginalBone
        {
            RE::NiPointer<RE::NiAVObject> node;
            RE::NiPointer<RE::NiAVObject> tree;
            RE::BSFixedString name;
            std::int32_t flatIndex{ -1 };
        };

        struct Slot
        {
            OriginalBone original;
            RE::NiNode* target{};
            std::uint32_t index{};
        };

        struct Skin
        {
            RE::NiPointer<RE::BSGeometry> geometry;
            RE::NiPointer<RE::BSSkin::Instance> original, instance;
            std::vector<Slot> slots;
        };

        struct Node
        {
            RE::NiPointer<RE::NiAVObject> actual;
            RE::NiNode* target{};
            std::size_t parentIndex{}, anchorIndex{};
            RE::NiTransform originalLocal, beforeLocal, beforeWorld, presentedWorld, presentedLocal;
            native_secondary_pose_transport::EvaluatedPose transport;
            native_secondary_pose_transport::PresentedHistory history;
            RE::NiTransform lastNativeLocal{};
            RE::NiTransform evaluatedLocal{}; // What the graph wrote, while driven.
            RE::NiTransform writtenLocal{}; // What the drive wrote over it.
            bool sampledNative{};
            bool duplicate{}; // Same name as an earlier bone owner; diagnostic only.
            bool driven{};
        };

        // A provider's pose for one animated bone, in place of the graph's
        // output, until its frame lease ends.
        struct Drive
        {
            std::size_t node{};
            RE::NiTransform local{};
            std::uint64_t expiresAfterFrame{};
        };

        // Part keys name a node of one binding: a later bind retires them.
        static constexpr std::uint64_t kKeyIndexBits = 16;
        static constexpr std::uint64_t kKeyIndexMask = (std::uint64_t{ 1 } << kKeyIndexBits) - 1;
        static constexpr std::size_t kNoNode = SIZE_MAX;

        std::vector<Skin> _skins;
        std::vector<Node> _nodes;
        std::vector<Drive> _drives;
        std::uint64_t _bindSequence{}, _lastBindSequence{};
        std::uint64_t _sampleFrame{ UINT64_MAX }, _nextReport{}, _nextBoneReport{};
        std::uint32_t _sampleFrames{}, _movingSamples{};
        float _nativeTravel{}, _nativeRotation{}, _nativeScale{}, _overwrite{};
        RE::BSFixedString _movingName;

        void sampleNative(std::uint64_t frame) noexcept
        {
            if (_sampleFrame == frame)
                return;
            _sampleFrame = frame;
            ++_sampleFrames;
            for (auto& node : _nodes)
                if (node.target) {
                    const auto& current = node.target->local;
                    if (node.sampledNative) {
                        const float travel = (current.translate - node.lastNativeLocal.translate).Length();
                        float matrix = 0;
                        for (int row = 0; row < 3; ++row)
                            for (int column = 0; column < 3; ++column)
                                matrix = (std::max)(matrix,
                                    std::abs(current.rotate.entry[row][column] -
                                        node.lastNativeLocal.rotate.entry[row][column]));
                        const float scale = std::abs(current.scale - node.lastNativeLocal.scale);
                        if (travel > 0.0001f || matrix > 0.0001f || scale > 0.0001f)
                            ++_movingSamples;
                        if (travel > _nativeTravel || matrix > _nativeRotation || scale > _nativeScale)
                            _movingName = node.actual->name;
                        _nativeTravel = (std::max)(_nativeTravel, travel);
                        _nativeRotation = (std::max)(_nativeRotation, matrix);
                        _nativeScale = (std::max)(_nativeScale, scale);
                    }
                    node.lastNativeLocal = current;
                    node.sampledNative = true;
                }
        }

        static bool flatBone(RE::NiAVObject* candidate, const RE::NiTransform* world, OriginalBone& out)
        {
            std::uintptr_t vtable{};
            if (!candidate || !native_memory::tryReadField(candidate, 0, vtable) ||
                vtable != REL::Offset(0x2E580A8).address())
                return false;
            auto* tree = static_cast<f4vr::BSFlattenedBoneTree*>(candidate);
            if (tree->numTransforms <= 0 || tree->numTransforms > 768 || !tree->transforms)
                return false;
            for (int i = 0; i < tree->numTransforms; ++i) {
                const auto& bone = tree->transforms[i];
                if (world == &bone.world || (bone.refNode && world == &bone.refNode->world)) {
                    out.tree.reset(candidate);
                    out.node.reset(bone.refNode);
                    out.name = bone.name;
                    out.flatIndex = i;
                    return true;
                }
            }
            return false;
        }

        static bool originalBone(RE::NiAVObject* node, RE::NiAVObject* root, const RE::NiTransform* world,
            OriginalBone& out)
        {
            if (node && world == &node->world) {
                out.node.reset(node);
                out.name = node->name;
                return true;
            }
            if (flatBone(root, world, out))
                return true;
            if (root)
                if (auto* parent = root->IsNode()) {
                    if (parent->GetRuntimeData().children.size() > 256)
                        return false;
                    for (const auto& child : parent->GetRuntimeData().children)
                        if (flatBone(child.get(), world, out))
                            return true;
                }
            return false;
        }

        static bool arrays(RE::BSSkin::Instance* skin, RE::NiAVObject**& nodes, const RE::NiTransform**& worlds,
            std::uint32_t& count)
        {
            return native_memory::tryReadField(skin, 0x10, nodes) && nodes &&
                native_memory::tryReadField(skin, 0x28, worlds) && worlds &&
                native_memory::tryReadField(skin, 0x38, count) && count <= 768;
        }

        [[nodiscard]] std::uint64_t keyOf(std::size_t index) const noexcept
        {
            return (_bindSequence << kKeyIndexBits) | static_cast<std::uint64_t>(index + 1);
        }

        [[nodiscard]] std::size_t indexOf(std::uint64_t key) const noexcept
        {
            const auto index = key & kKeyIndexMask;
            return _bindSequence != 0 && (key >> kKeyIndexBits) == _bindSequence && index >= 1 && index <= _nodes.size()
                ? static_cast<std::size_t>(index - 1)
                : kNoNode;
        }

        [[nodiscard]] static bool sameLocal(const RE::NiTransform& left, const RE::NiTransform& right) noexcept
        {
            return std::memcmp(&left.rotate, &right.rotate, sizeof(left.rotate)) == 0 &&
                left.translate == right.translate && left.scale == right.scale;
        }

    public:
        ~BoneBindings() { restore(); }

        BoneBindings() = default;
        BoneBindings(const BoneBindings&) = delete;
        BoneBindings& operator=(const BoneBindings&) = delete;

        struct PartSource
        {
            std::uint64_t key{}, parentKey{};
            RE::NiTransform local{};
            const char* name{};
        };

        [[nodiscard]] std::uint32_t partSourceCount() const noexcept
        {
            return static_cast<std::uint32_t>(_nodes.size());
        }

        [[nodiscard]] PartSource partSource(std::uint32_t index) const noexcept
        {
            const auto& node = _nodes[index];
            PartSource source{};
            source.key = keyOf(index);
            source.parentKey = index ? keyOf(node.parentIndex) : 0;
            source.local = node.actual->local;
            source.name = node.actual->name.c_str();
            return source;
        }

        [[nodiscard]] std::uint64_t partSourceKey(const RE::NiAVObject* object) const noexcept
        {
            for (std::size_t i = 0; i < _nodes.size(); ++i)
                if (_nodes[i].actual.get() == object)
                    return keyOf(i);
            return 0;
        }

        // A driven bone reports what its graph evaluated, before the drive.
        [[nodiscard]] bool partPose(std::uint64_t key, RE::NiTransform& parentLocal, RE::NiTransform& rootLocal,
            RE::NiTransform& world) const noexcept
        {
            const auto index = indexOf(key);
            if (index == kNoNode)
                return false;
            const auto& node = _nodes[index];
            parentLocal = node.driven ? node.evaluatedLocal : node.target ? node.target->local : node.actual->local;
            world = node.actual->world;
            rootLocal = transform_math::composeTransforms(
                weapon_visual_authority_math::invertSceneTransform(_nodes[0].actual->world), world);
            return true;
        }

        [[nodiscard]] bool partIsAnimated(std::uint64_t key) const noexcept
        {
            const auto index = indexOf(key);
            return index != kNoNode && _nodes[index].target;
        }

        // Ends every drive at the next apply, which restores what they wrote.
        void endDrives() noexcept
        {
            for (auto& drive : _drives)
                drive.expiresAfterFrame = 0;
        }

        // The caller checked that the key is animated. Replaces that bone's
        // drive, keeping what the graph last evaluated under it.
        void addDrive(std::uint64_t key, const RE::NiTransform& local, std::uint64_t expiresAfterFrame)
        {
            const auto node = indexOf(key);
            for (auto& drive : _drives) {
                if (drive.node == node) {
                    drive.local = local;
                    drive.expiresAfterFrame = expiresAfterFrame;
                    return;
                }
            }
            _drives.push_back({ node, local, expiresAfterFrame });
        }

        /*
         * Runs after the graph's update and before its placement, so the bone's
         * world and every skin bound to it follow the drive. A local still equal
         * to the last drive means the graph left this bone alone: its evaluated
         * pose is still the older one, and that is what an ending drive restores.
         */
        void applyDrives(std::uint64_t frame) noexcept
        {
            for (auto drive = _drives.begin(); drive != _drives.end();) {
                auto& node = _nodes[drive->node];
                const bool graphWrote = !node.driven || !sameLocal(node.target->local, node.writtenLocal);
                if (graphWrote)
                    node.evaluatedLocal = node.target->local;
                if (frame > drive->expiresAfterFrame) {
                    if (!graphWrote)
                        node.target->local = node.evaluatedLocal;
                    node.driven = false;
                    drive = _drives.erase(drive);
                    continue;
                }
                node.driven = true;
                node.writtenLocal = drive->local;
                node.target->local = drive->local;
                ++drive;
            }
        }

        void restore() noexcept
        {
            for (auto& skin : _skins) {
                auto& current = skin.geometry->GetRuntimeData().skinInstance;
                if (current.get() == skin.instance.get())
                    current = skin.original;
            }
            for (auto& node : _nodes)
                if (node.target)
                    node.actual->local = node.originalLocal;
            _skins.clear();
            _nodes.clear();
            _drives.clear();
            _bindSequence = 0;
            _sampleFrame = UINT64_MAX;
            _nextReport = 0;
            _nextBoneReport = 0;
            _sampleFrames = _movingSamples = 0;
            _nativeTravel = _nativeRotation = _nativeScale = _overwrite = 0;
            _movingName = {};
        }

        /*
         * Hands the model over to a carrier that moves only its root (the
         * promotion bridge). present() writes each model node's world from
         * the private skeleton's locals under the model's own root, and rigid
         * shapes render from those worlds; the skeleton's bone worlds sit
         * under a root placed at the attachment and differ from them in the
         * left hand. Each model node takes its presented world as a local
         * under its parent. Each skin keeps its private clone, with every
         * bound slot re-pointed from the skeleton bone to the model node that
         * bone drove: the original instance can read a flattened bone tree
         * that nothing poses any more. A skin with a slot no model node owns
         * returns to its original instance. The model keeps its look and
         * follows its root from this frame on; it leaves the actor, so its
         * authored rest locals are not restored.
         */
        bool releaseKeepingPose() noexcept
        {
            if (_nodes.empty()) {
                restore();
                return false;
            }
            std::vector<RE::NiTransform> worlds(_nodes.size());
            worlds[0] = _nodes[0].actual->world;
            bool usable = weapon_scene_presentation::isFiniteTransform(worlds[0]);
            for (std::size_t i = 1; usable && i < _nodes.size(); ++i) {
                auto& node = _nodes[i];
                if (node.actual->parent != _nodes[node.parentIndex].actual.get())
                    usable = false;
                worlds[i] = node.actual->world;
                usable = usable && weapon_scene_presentation::isFiniteTransform(worlds[i]);
            }
            if (!usable) {
                restore();
                return false;
            }
            for (std::size_t i = 1; i < _nodes.size(); ++i) {
                auto& node = _nodes[i];
                node.actual->local = transform_math::composeTransforms(
                    weapon_visual_authority_math::invertSceneTransform(worlds[node.parentIndex]), worlds[i]);
                node.actual->world = worlds[i];
            }
            std::size_t kept = 0, returned = 0;
            for (auto& skin : _skins) {
                auto& current = skin.geometry->GetRuntimeData().skinInstance;
                if (current.get() != skin.instance.get())
                    continue;
                RE::NiAVObject** nodes{};
                const RE::NiTransform** boneWorlds{};
                std::uint32_t count{};
                bool owned = arrays(skin.instance.get(), nodes, boneWorlds, count);
                std::vector<RE::NiAVObject*> owners(skin.slots.size(), nullptr);
                for (std::size_t j = 0; owned && j < skin.slots.size(); ++j) {
                    const auto owner = std::find_if(_nodes.begin(), _nodes.end(),
                        [&](const Node& node) { return node.target == skin.slots[j].target; });
                    owners[j] = owner != _nodes.end() ? owner->actual.get() : nullptr;
                    owned = owners[j] && skin.slots[j].index < count;
                }
                if (!owned) {
                    current = skin.original;
                    ++returned;
                    continue;
                }
                for (std::size_t j = 0; j < skin.slots.size(); ++j) {
                    nodes[skin.slots[j].index] = owners[j];
                    boneWorlds[skin.slots[j].index] = &owners[j]->world;
                }
                ++kept;
            }
            ROCK_LOG_INFO(Animation,
                "AKIMBO bone binding handed to model nodes skins={} rebound={} original={} nodes={}", _skins.size(),
                kept, returned, _nodes.size());
            _skins.clear();
            _nodes.clear();
            _drives.clear();
            _bindSequence = 0;
            _sampleFrame = UINT64_MAX;
            _nextReport = 0;
            _nextBoneReport = 0;
            _sampleFrames = _movingSamples = 0;
            _nativeTravel = _nativeRotation = _nativeScale = _overwrite = 0;
            _movingName = {};
            return true;
        }

        bool bind(RE::NiAVObject* root, native_equipped_action_graph::Graph& graph)
        {
            restore();
            if (!root || !nativeSkinLayoutVerified())
                return false;
            constexpr auto prefix = std::to_array<std::uint8_t>(
                { 0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57, 0x48, 0x83, 0xEC, 0x20 });
            std::array<std::uint8_t, prefix.size()> live{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(0x1C13FF0).address()),
                    live.data(), live.size()) ||
                live != prefix)
                return false;
            // Native binding resolves each graph bone with 1C20840, a
            // pre-order depth-first search (1C220E0) that stops at the first
            // same-named node. Models can repeat a bone name (the 1911 has a
            // WeaponMagazine with its own rounds under both P-Receiver and
            // P-Mag); only the first one is animated, the others keep their
            // authored pose. Driving every duplicate pushed the second set of
            // rounds up out of the slide.
            std::vector<std::pair<RE::BSFixedString, RE::NiAVObject*>> owners;
            std::vector<RE::NiAVObject*> duplicates;
            {
                std::vector<RE::NiAVObject*> stack{ root };
                std::size_t visited = 0;
                while (!stack.empty()) {
                    if (++visited > 8192)
                        return false;
                    auto* object = stack.back();
                    stack.pop_back();
                    if (object != root && object->IsNode() && graph.bone(object->name)) {
                        const auto owner = std::find_if(owners.begin(), owners.end(),
                            [&](const auto& entry) { return entry.first == object->name; });
                        if (owner == owners.end())
                            owners.emplace_back(object->name, object);
                        else {
                            duplicates.push_back(object);
                            ROCK_LOG_INFO(Animation,
                                "AKIMBO bone owner duplicate name='{}' parent='{}' kept='{}' keptParent='{}' "
                                "native=unanimated",
                                object->name.c_str(), object->parent ? object->parent->name.c_str() : "",
                                owner->second->name.c_str(),
                                owner->second->parent ? owner->second->parent->name.c_str() : "");
                        }
                    }
                    if (auto* node = object->IsNode()) {
                        const auto& children = node->GetRuntimeData().children;
                        // Reverse push so children pop in native (forward) order.
                        for (auto i = children.size(); i > 0; --i)
                            if (children[i - 1])
                                stack.push_back(children[i - 1].get());
                    }
                }
            }
            const auto nativeOwner = [&](RE::NiAVObject* object) {
                return std::any_of(owners.begin(), owners.end(),
                    [&](const auto& owner) { return owner.second == object; });
            };
            std::vector<RE::NiAVObject*> objects;
            objects.reserve(256);
            objects.push_back(root);
            _nodes.push_back({ RE::NiPointer<RE::NiAVObject>{ root }, nullptr, 0, 0, root->local, {}, {}, {} });
            std::size_t boundNodes = 0;
            for (std::size_t i = 0; i < objects.size(); ++i) {
                auto* object = objects[i];
                if (auto* node = object->IsNode()) {
                    for (const auto& child : node->GetRuntimeData().children)
                        if (child) {
                            if (objects.size() >= 8192)
                                return false;
                            objects.push_back(child.get());
                            auto* target =
                                child->IsNode() && nativeOwner(child.get()) ? graph.bone(child->name) : nullptr;
                            if (target)
                                ++boundNodes;
                            const auto anchor = _nodes[i].target ? i : _nodes[i].anchorIndex;
                            _nodes.push_back({ RE::NiPointer<RE::NiAVObject>{ child.get() }, target, i, anchor,
                                child->local, {}, {}, {} });
                            _nodes.back().duplicate =
                                std::find(duplicates.begin(), duplicates.end(), child.get()) != duplicates.end();
                        }
                }
                auto* geometry = object->IsGeometry();
                if (!geometry)
                    geometry = object->IsTriShape();
                auto* instance = geometry ? geometry->GetRuntimeData().skinInstance.get() : nullptr;
                if (!instance)
                    continue;
                RE::NiAVObject** nodes{};
                const RE::NiTransform** worlds{};
                std::uint32_t count{};
                RE::NiAVObject* skinRoot{};
                if (!arrays(instance, nodes, worlds, count) || !native_memory::tryReadField(instance, 0x48, skinRoot))
                    return false;
                Skin skin;
                skin.geometry.reset(geometry);
                skin.original.reset(instance);
                skin.slots.reserve(count);
                std::size_t skippedDuplicates = 0;
                for (std::uint32_t j = 0; j < count; ++j) {
                    OriginalBone original;
                    if (!originalBone(nodes[j], skinRoot, worlds[j], original))
                        continue;
                    // A skin bone that is a non-owner duplicate keeps its own
                    // node, as natively: the graph never animates it.
                    if (std::find(duplicates.begin(), duplicates.end(), original.node.get()) != duplicates.end()) {
                        ++skippedDuplicates;
                        continue;
                    }
                    if (auto* target = graph.bone(original.name))
                        skin.slots.push_back({ std::move(original), target, j });
                }
                if (skippedDuplicates)
                    ROCK_LOG_INFO(Animation, "AKIMBO skin duplicate bones kept native shape='{}' skipped={}",
                        geometry->name.c_str(), skippedDuplicates);
                // Native skin cloning gives this geometry independent palette
                // arrays while retaining its original bind matrices/materials.
                // The original instance stays intact for exact restoration.
                RE::NiCloningProcess cloning{};
                cloning.copyType = RE::NiCloningProcess::CopyType::kCopyExact;
                cloning.scale = { 1.0f, 1.0f, 1.0f };
                using Clone = RE::NiObject* (*)(RE::NiObject*, RE::NiCloningProcess*);
                auto* copy = reinterpret_cast<Clone>(REL::Offset(0x1C13FF0).address())(instance, &cloning);
                skin.instance.reset(static_cast<RE::BSSkin::Instance*>(copy));
                RE::NiAVObject** copyNodes{};
                const RE::NiTransform** copyWorlds{};
                std::uint32_t copyCount{};
                if (!copy || copy == instance || !arrays(skin.instance.get(), copyNodes, copyWorlds, copyCount) ||
                    copyCount != count || copyNodes == nodes || copyWorlds == worlds)
                    return false;
                _skins.push_back(std::move(skin));
            }
            // Preparation is read-only. Commit after every shape was inspected.
            for (auto& skin : _skins) {
                RE::NiAVObject** nodes{};
                const RE::NiTransform** worlds{};
                std::uint32_t count{};
                if (!arrays(skin.instance.get(), nodes, worlds, count)) {
                    restore();
                    return false;
                }
                for (const auto& slot : skin.slots) {
                    if (slot.index >= count) {
                        restore();
                        return false;
                    }
                    nodes[slot.index] = slot.target;
                    worlds[slot.index] = &slot.target->world;
                }
                skin.geometry->GetRuntimeData().skinInstance = skin.instance;
            }
            ROCK_LOG_INFO(Animation,
                "AKIMBO independent bone binding objects={} skins={} nodes={} descendantTransport=complete",
                objects.size(), _skins.size(), boundNodes);
            // First pose sample after the graph has evaluated, not the bind pose.
            _nextBoneReport = GetTickCount64() + 1000;
            _bindSequence = ++_lastBindSequence;
            return true;
        }

        void carryPublished(const RE::NiTransform& delta) noexcept
        {
            for (auto& node : _nodes)
                node.transport.carryPublished(delta);
        }

        bool present(std::uint64_t frame) noexcept
        {
            if (_nodes.empty())
                return false;
            sampleNative(frame);
            // Snapshot evaluated worlds before moving any parent. Animated
            // descendants can differ from their local transforms; rigidly carry
            // that output with the corrected bone instead of discarding it.
            for (std::size_t i = 0; i < _nodes.size(); ++i) {
                auto& node = _nodes[i];
                if (i && node.actual->parent != _nodes[node.parentIndex].actual.get())
                    return false;
                node.beforeWorld = node.actual->world;
                node.beforeLocal = node.actual->local;
                if (!weapon_scene_presentation::isFiniteTransform(node.beforeWorld))
                    return false;
            }
            _nodes[0].presentedWorld = _nodes[0].beforeWorld;
            for (std::size_t i = 1; i < _nodes.size(); ++i) {
                auto& node = _nodes[i];
                const auto& parent = _nodes[node.parentIndex];
                if (node.target) {
                    // Native graph binding (1785010) drives a same-named model
                    // node with the bone's LOCAL transform under the model's
                    // own parent. Model hierarchies differ from the skeleton
                    // (BAS rounds hang under WeaponMagazineHelper, the skeleton
                    // has them under WeaponMagazine); copying the bone's WORLD
                    // dropped that intermediate offset and moved the rounds
                    // out of the magazine.
                    node.presentedLocal = node.target->local;
                    node.presentedWorld = transform_math::composeTransforms(parent.presentedWorld, node.presentedLocal);
                    _overwrite =
                        (std::max)(_overwrite, (node.beforeWorld.translate - node.presentedWorld.translate).Length());
                } else {
                    const auto& anchor = _nodes[node.anchorIndex];
                    if (!node.transport.reframe(anchor.beforeWorld, anchor.presentedWorld, node.beforeWorld,
                            node.presentedWorld)) {
                        ROCK_LOG_SAMPLE_WARN(Animation, 1000,
                            "AKIMBO part pose rejected stage=descendant-frame name='{}' anchor='{}'",
                            node.actual->name.c_str(), anchor.actual->name.c_str());
                        return false;
                    }
                }
                if (!weapon_scene_presentation::isFiniteTransform(node.presentedWorld))
                    return false;
            }
            for (std::size_t i = 1; i < _nodes.size(); ++i) {
                auto& node = _nodes[i];
                node.actual->world = node.presentedWorld;
                if (node.target)
                    node.actual->local = node.presentedLocal;
            }
            // 1C23777..3849 copies current world (+70) to previous (+C0)
            // before native placement. The shader accumulator (281BE35..BE4D)
            // and shader setup (28796C0..97AE) consume that prior transform.
            // Give them the prior presented pose, not the transient melee arm
            // pose or this frame's earlier private graph placement. True hand
            // movement and native part animation remain in the history.
            float historyError = 0;
            for (auto& node : _nodes) {
                const auto& previous = node.history.publish(frame, node.actual->world);
                historyError =
                    (std::max)(historyError, (node.actual->previousWorld.translate - previous.translate).Length());
                node.actual->previousWorld = previous;
            }
            ROCK_LOG_SAMPLE_INFO(Animation, 1000,
                "AKIMBO secondary motion history frame={} nodes={} correctedPreviousMaxGame={:.3f} "
                "source=previous-presentation",
                frame, _nodes.size(), historyError);
            const auto now = GetTickCount64();
            // Diagnostic: per bound model bone, its authored rest local, the
            // local the model's own graph/native code left this frame, and
            // the private skeleton's local that replaced it. Every 5 seconds.
            if (now >= _nextBoneReport) {
                _nextBoneReport = now + 5000;
                std::size_t reported = 0;
                for (const auto& node : _nodes) {
                    if (node.duplicate && reported < 32) {
                        ++reported;
                        const auto& rest = node.originalLocal.translate;
                        const auto& shown = node.actual->local.translate;
                        ROCK_LOG_INFO(Animation,
                            "AKIMBO unbound duplicate pose name='{}' parent='{}' rest=({:.3f},{:.3f},{:.3f}) "
                            "local=({:.3f},{:.3f},{:.3f})",
                            node.actual->name.c_str(), node.actual->parent ? node.actual->parent->name.c_str() : "",
                            rest.x, rest.y, rest.z, shown.x, shown.y, shown.z);
                        continue;
                    }
                    if (!node.target || ++reported > 32)
                        continue;
                    const auto& rest = node.originalLocal.translate;
                    const auto& model = node.beforeLocal.translate;
                    const auto& skeleton = node.target->local.translate;
                    ROCK_LOG_INFO(Animation,
                        "AKIMBO bound bone pose name='{}' parent='{}' rest=({:.3f},{:.3f},{:.3f}) "
                        "modelFrame=({:.3f},{:.3f},{:.3f}) skeleton=({:.3f},{:.3f},{:.3f}) skeletonParent='{}'",
                        node.actual->name.c_str(), node.actual->parent ? node.actual->parent->name.c_str() : "", rest.x,
                        rest.y, rest.z, model.x, model.y, model.z, skeleton.x, skeleton.y, skeleton.z,
                        node.target->parent ? node.target->parent->name.c_str() : "");
                }
            }
            if (now >= _nextReport) {
                ROCK_LOG_INFO(Animation,
                    "AKIMBO native part evaluation frame={} frames={} changedSamples={} nativeLocalTravelGame={:.5f} "
                    "nativeLocalMatrixDelta={:.5f} nativeLocalScaleDelta={:.5f} part='{}' "
                    "presentationRepairGame={:.3f} reference=bone-parent-local",
                    frame, _sampleFrames, _movingSamples, _nativeTravel, _nativeRotation, _nativeScale,
                    _movingName.c_str(), _overwrite);
                _nextReport = now + 1000;
                _sampleFrames = _movingSamples = 0;
                _nativeTravel = _nativeRotation = _nativeScale = _overwrite = 0;
                _movingName = {};
            }
            return true;
        }
    };
}
