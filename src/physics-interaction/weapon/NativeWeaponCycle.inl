// Included once by NativeIdleGripPreharvest.cpp. Both users share the verified
// graph-loading and resource-release code; their native managers never overlap.
#include "physics-interaction/weapon/NativeWeaponCycle.h"
#include "physics-interaction/weapon/WeaponCyclePolicy.h"

namespace rock::native_weapon_cycle
{
    using namespace native_idle_grip_preharvest;

    struct Session::State
    {
        Runtime backend{};
        RE::NiPointer<RE::NiAVObject> root{};
        RE::ObjectRefHandle reference{};
        struct Node
        {
            RE::NiPointer<RE::NiAVObject> object{};
            RE::NiTransform restLocal{}, restInRoot{}, inverseClipRest{};
            std::size_t parent{}, childCount{};
            int bone{-1};
        };
        std::array<Node, 512> nodes{};
        std::array<RE::NiTransform, 512> worlds{}, locals{};
        std::size_t nodeCount{}, partCount{};
        std::array<std::int16_t, kMaxBonesAndTracks> parents{}, mapping{};
        std::array<HkQsTransform, kMaxBonesAndTracks> samples{}, referencePose{};
        int boneCount{}, trackCount{}, mappingCount{}, weaponBone{-1};
        void* animation{}; // Pinned by backend graph/resource, never a live actor graph.
        SampleAnimationTracksFn sampler{};
        float duration{}, time{}, playbackRate{1.0f};
        bool ready{}, failed{}, playing{}, restorePending{}, retiring{};
        unsigned shotsLogged{};

        ~State() { restore(); releaseJob(backend); }

        static std::array<std::unique_ptr<State>, 2>& retired()
        {
            // Two frame-thread retirement slots survive PhysicsInteraction
            // replacement. Like the existing preharvest runtime, its storage
            // is process-lived: DLL/static teardown cannot wait for native IO.
            // New sessions are withheld until this slot has drained.
            static auto* slot = new std::array<std::unique_ptr<State>, 2>();
            return *slot;
        }

        bool canRelease()
        {
            if (!backend.job.graphHolderConstructed) return true;
            auto* holder = graphHolder(backend.job);
            // 811F50 -> 812450 waits for an unfinished loader. Its completion
            // predicate 8122C0 is a nonblocking read; retire on a later frame.
            if (!backend.native.isAnimationLoadingComplete(holder)) return false;
            if (backend.job.subgraphHandles.empty() || !holder->animationGraphManager) return true;
            std::int32_t priority = kIoTaskPriority;
            return backend.native.isAnimationSubGraphLoaded(&holder->animationGraphManager, &backend.job.subgraphHandles, &priority);
        }

        bool topologyCurrent() const
        {
            if (!root || !nodeCount || nodes[0].object.get() != root.get()) return false;
            for (std::size_t i = 0; i < nodeCount; ++i) {
                const auto& node = nodes[i];
                if (!node.object || (i && node.object->parent != nodes[node.parent].object.get())) return false;
                auto* branch = node.object->IsNode();
                if ((branch ? branch->children.size() : 0) != node.childCount) return false;
            }
            return true;
        }

        void restore()
        {
            if (!restorePending || !topologyCurrent()) return;
            for (std::size_t i = 1; i < nodeCount; ++i) {
                auto& node = nodes[i];
                node.object->local = node.restLocal;
                node.object->world = transform_math::composeTransforms(root->world, node.restInRoot);
                if (auto* geometry = node.object->IsGeometry()) geometry->UpdateWorldData(nullptr);
            }
            restorePending = false;
        }

        void fail(const char* stage)
        {
            restore();
            ready = false; failed = true; playing = false;
            ROCK_LOG_WARN(Animation, "Akimbo mechanical cycle unavailable ref={:08X} weapon={:08X} stage={}; held model retained",
                backend.job.referenceFormId, backend.job.weaponFormId, stage);
        }

        bool partInWeapon(int bone, RE::NiTransform& result) const
        {
            result = transform_math::makeIdentityTransform<RE::NiTransform>();
            for (unsigned depth = 0; depth < 64; ++depth) {
                if (bone == weaponBone) return isFiniteTransform(result);
                if (bone < 0 || bone >= boneCount) return false;
                const auto track = native_idle_grip_preharvest_policy::findTransformTrackForBone(bone, trackCount,
                    std::span<const std::int16_t>(mapping.data(), mappingCount));
                RE::NiTransform local{};
                if (!convertHavokLocalTransform(track >= 0 ? samples[track] : referencePose[bone], local)) return false;
                result = transform_math::composeTransforms(local, result);
                bone = parents[bone];
            }
            return false;
        }

        bool bind(RE::BShkbAnimationGraph* graph, void* binding)
        {
            std::uint8_t blend{};
            void* skeletonOwner{}; void* skeleton{}; void** table{};
            int parentCount{}, poseCount{};
            const std::int16_t *parentData{}, *mapData{};
            const HkQsTransform* pose{};
            if (!native_memory::tryReadField(binding, kBindingBlendHintOffset, blend) || blend != 0 ||
                !native_memory::tryReadField(binding, kAnimationFromBindingOffset, animation) || !animation ||
                !native_memory::tryReadField(animation, kAnimationDurationOffset, duration) || !std::isfinite(duration) || duration <= 0 || duration > 30 ||
                !native_memory::tryReadField(animation, kAnimationTransformTrackCountOffset, trackCount) || trackCount <= 0 || trackCount > kMaxBonesAndTracks ||
                !native_memory::tryReadField(animation, 0, table) || !table || !native_memory::tryReadValue(reinterpret_cast<SampleAnimationTracksFn*>(table + 5), sampler) || !addressIsExecutable(reinterpret_cast<void*>(sampler)) ||
                !native_memory::tryReadField(graph, kGraphSkeletonOwnerOffset, skeletonOwner) || !skeletonOwner ||
                !native_memory::tryReadField(skeletonOwner, kSkeletonFromOwnerOffset, skeleton) || !skeleton ||
                !native_memory::tryReadField(skeleton, kSkeletonBoneCountOffset, boneCount) || boneCount <= 0 || boneCount > kMaxBonesAndTracks ||
                !native_memory::tryReadField(skeleton, kSkeletonParentCountOffset, parentCount) || parentCount < boneCount || parentCount > kMaxBonesAndTracks ||
                !native_memory::tryReadField(skeleton, kSkeletonParentIndicesOffset, parentData) || !parentData ||
                !native_memory::guardedCopyFromMemory(parentData, parents.data(), boneCount * sizeof(std::int16_t)) ||
                !native_memory::tryReadField(skeleton, kSkeletonReferencePoseOffset, pose) || !pose ||
                !native_memory::tryReadField(skeleton, kSkeletonReferencePoseCountOffset, poseCount) || poseCount < boneCount || poseCount > kMaxBonesAndTracks ||
                !native_memory::guardedCopyFromMemory(pose, referencePose.data(), boneCount * sizeof(HkQsTransform)) ||
                !native_memory::tryReadField(binding, kTrackToBoneMappingCountOffset, mappingCount) || mappingCount < 0 || mappingCount > kMaxBonesAndTracks ||
                !native_memory::tryReadField(binding, kTrackToBoneMappingOffset, mapData)) return false;
            if (mappingCount && (!mapData || !native_memory::guardedCopyFromMemory(mapData, mapping.data(), mappingCount * sizeof(std::int16_t)))) return false;
            const auto weapon = backend.native.findBoneWithName(skeleton, "Weapon", nullptr);
            if (weapon >= static_cast<std::uint64_t>(boneCount)) return false;
            weaponBone = static_cast<int>(weapon);
            if (!guardedSampleTracks(sampler, animation, 0, trackCount, samples.data())) return false;
            nodeCount = 1;
            nodes[0].object = root;
            const auto rootInverse = transform_math::invertTransform(root->world);
            for (std::size_t i = 0; i < nodeCount; ++i) {
                auto& node = nodes[i];
                // Physics may already have evaluated worlds without rewriting
                // every local. Preserve that pose; rebuilding stale locals is
                // precisely what separates an assembled gun's components.
                node.restLocal = i ? transform_math::composeTransforms(
                    transform_math::invertTransform(nodes[node.parent].object->world), node.object->world) : node.object->local;
                node.restInRoot = transform_math::composeTransforms(rootInverse, node.object->world);
                if (!isFiniteTransform(node.restLocal) || !isFiniteTransform(node.restInRoot)) return false;
                const auto* name = node.object->name.c_str();
                const auto bone = name ? backend.native.findBoneWithName(skeleton, name, nullptr) : UINT64_MAX;
                if (i && bone < static_cast<std::uint64_t>(boneCount) && weapon_cycle_policy::isWeaponPart(static_cast<int>(bone), weaponBone,
                        std::span<const std::int16_t>(parents.data(), boneCount))) {
                    node.bone = static_cast<int>(bone);
                    // Ambiguous duplicated rig names cannot receive a shared transform.
                    for (std::size_t j = 1; j < i; ++j) if (nodes[j].bone == node.bone) return false;
                    RE::NiTransform clipRest{};
                    if (!partInWeapon(node.bone, clipRest)) return false;
                    node.inverseClipRest = transform_math::invertTransform(clipRest);
                    ++partCount;
                }
                auto* branch = node.object->IsNode();
                if (!branch) continue;
                node.childCount = branch->children.size();
                if (node.childCount > nodes.size() - nodeCount) return false;
                for (const auto& child : branch->children) if (child) {
                    // Reject shared subtrees or cycles before retaining them.
                    for (std::size_t j = 0; j < nodeCount; ++j) if (nodes[j].object.get() == child.get()) return false;
                    nodes[nodeCount].object = child;
                    nodes[nodeCount++].parent = i;
                }
            }
            return partCount != 0;
        }

        void load()
        {
            const auto loaded = progressGraphLoad(backend);
            if (loaded == ExtractionResult::Failed) { fail("exact-subgraph"); return; }
            if (loaded != ExtractionResult::Succeeded) return;
            auto* manager = graphHolder(backend.job)->animationGraphManager.get();
            const auto selection = native_idle_grip_preharvest_policy::selectFirstPersonGraph(manager->graph.size(),
                backend.job.subgraphHandles.size(), backend.job.subgraphIdentifiers.size());
            const auto graphIndex = static_cast<decltype(manager->graph)::size_type>(selection.graphIndex);
            if (!selection.valid || !manager->graph[graphIndex]) { fail("first-person-graph"); return; }
            auto* graph = manager->graph[graphIndex].get();
            const auto handle = backend.job.subgraphHandles[static_cast<decltype(backend.job.subgraphHandles)::size_type>(selection.graphIndex)].handle;
            std::uint64_t identifier{};
            std::array<char, 260> path{};
            IdleGripExtractionDiagnostics diagnostics{};
            if (!tryFindLoadedGraphIdlePath(graph, handle, identifier, path, diagnostics, &weapon_cycle_policy::fireClipPriority)) {
                fail("unambiguous-fire-clip"); return;
            }
            void* binding{};
            const auto resolved = resolveClipBinding(backend, graph, identifier, path.data(), binding, diagnostics);
            if (resolved == ExtractionResult::Pending) return;
            if (resolved == ExtractionResult::Failed) { fail(extractionFailureName(diagnostics.failure)); return; }
            if (!bind(graph, binding)) { fail("weapon-part-binding"); return; }
            ready = true;
            ROCK_LOG_INFO(Animation, "Akimbo mechanical cycle ready ref={:08X} weapon={:08X} parts={} tracks={} duration={:.4f} clip={}",
                backend.job.referenceFormId, backend.job.weaponFormId, partCount, trackCount, duration, path.data());
        }

        void apply(float deltaSeconds)
        {
            if (!playing) return;
            if (!topologyCurrent()) { fail("scene-topology-changed"); return; }
            time = (std::min)(duration, time + playbackRate * (std::isfinite(deltaSeconds) ? (std::max)(0.0f, deltaSeconds) : 0.0f));
            if (!guardedSampleTracks(sampler, animation, time, trackCount, samples.data())) { fail("sample"); return; }
            worlds[0] = root->world;
            // Calculate all destinations before writing. Absolute part deltas
            // avoid applying an animated parent twice and preserve OMOD offsets.
            for (std::size_t i = 1; i < nodeCount; ++i) {
                const auto& node = nodes[i];
                locals[i] = node.restLocal;
                worlds[i] = transform_math::composeTransforms(worlds[node.parent], locals[i]);
                if (node.bone >= 0) {
                    RE::NiTransform sample{};
                    if (!partInWeapon(node.bone, sample)) { fail("part-chain"); return; }
                    const auto delta = transform_math::composeTransforms(sample, node.inverseClipRest);
                    worlds[i] = transform_math::composeTransforms(worlds[0], transform_math::composeTransforms(delta, node.restInRoot));
                    locals[i] = transform_math::composeTransforms(transform_math::invertTransform(worlds[node.parent]), worlds[i]);
                }
                if (!isFiniteTransform(worlds[i]) || !isFiniteTransform(locals[i])) { fail("part-transform"); return; }
            }
            for (std::size_t i = 1; i < nodeCount; ++i) {
                nodes[i].object->local = locals[i];
                nodes[i].object->world = worlds[i];
                // Native render data must see the same mechanical pose as the
                // scene and collider sources, without updating either arm rig.
                if (auto* geometry = nodes[i].object->IsGeometry()) geometry->UpdateWorldData(nullptr);
            }
            restorePending = true;
            if (time >= duration) { restore(); playing = false; }
        }
    };

    Session::Session() noexcept = default;
    bool Session::ready() const noexcept { return _state && _state->ready && !_state->failed && !_state->retiring; }
    bool Session::failed() const noexcept { return _state && _state->failed; }
    bool Session::playing() const noexcept { return _state && _state->playing; }
    Session::~Session()
    {
        clear();
        if (_state) for (auto& slot : State::retired()) if (!slot) { slot = std::move(_state); break; }
    }
    void Session::reap() noexcept
    {
        for (auto& retired : State::retired()) if (retired && retired->canRelease()) retired.reset();
        if (_state && _state->retiring && _state->canRelease()) _state.reset();
    }

    void Session::clear() noexcept
    {
        if (_state && !_state->retiring) {
            _state->restore();
            _state->playing = false;
            _state->retiring = true;
            // Native loading may finish later; the old physical item and its
            // scene must nevertheless be released immediately for pickup.
            _state->nodes = {};
            _state->nodeCount = 0;
            _state->root.reset();
        }
        reap();
    }

    void Session::update(RE::TESObjectREFR* reference, float deltaSeconds) noexcept try
    {
        auto* root = reference ? reference->Get3D() : nullptr;
        if (!root) { clear(); return; }
        if (_state && (_state->reference != reference->GetHandle() || _state->root.get() != root)) clear();
        reap();
        if (_state && _state->retiring) return;
        if (!_state) {
            for (const auto& retired : State::retired()) if (retired) return;
            _state = std::make_unique<State>();
            _state->reference = reference->GetHandle();
            _state->root.reset(root);
            if (!claimOrValidateThread(_state->backend) || !resolveNativeFunctions(_state->backend)) {
                _state->fail("native-contract"); return;
            }
            startJob(_state->backend, describeLooseCandidate(reference, root));
        }
        if (_state->failed) return;
        if (!_state->ready) _state->load();
        if (_state->ready) _state->apply(deltaSeconds);
    }
    catch (...) {
        if (_state) { try { _state->fail("exception"); } catch (...) {} }
    }

    void Session::updateEquipped(RE::TESObjectWEAP* weapon, RE::TBO_InstanceData* instance,
        RE::NiAVObject* root, std::uint64_t content, float deltaSeconds) noexcept try
    {
        if (!root || !weapon) { clear(); return; }
        if (_state && (_state->root.get() != root || _state->backend.job.weapon != weapon ||
                _state->backend.job.instanceData.get() != instance)) clear();
        reap();
        if (_state && _state->retiring) return;
        if (!_state) {
            for (const auto& retired : State::retired()) if (retired) return;
            _state = std::make_unique<State>();
            _state->root.reset(root);
            if (!claimOrValidateThread(_state->backend) || !resolveNativeFunctions(_state->backend)) {
                _state->fail("native-contract"); return;
            }
            startJob(_state->backend, describeEquippedCandidate(weapon, root, instance, content));
        }
        if (_state->failed) return;
        if (!_state->ready) _state->load();
        if (_state->ready) _state->apply(deltaSeconds);
    }
    catch (...) {
        if (_state) { try { _state->fail("equipped-exception"); } catch (...) {} }
    }

    void Session::fire(float shotSeconds) noexcept
    {
        if (!_state || !_state->ready || _state->failed || _state->retiring) return;
        _state->playing = true;
        _state->time = 0;
        // Complete the mechanical stroke within this weapon's firing period.
        // Restarting an unscaled long single-shot clip every automatic shot
        // otherwise samples only its opening frames indefinitely.
        _state->playbackRate = weapon_cycle_policy::playbackRate(_state->duration, shotSeconds);
        if (_state->shotsLogged++ < 4) {
            try { ROCK_LOG_INFO(Animation, "Akimbo mechanical stroke ref={:08X} parts={} clipSeconds={} shotSeconds={} playbackRate={}",
                _state->backend.job.referenceFormId, _state->partCount, _state->duration, shotSeconds, _state->playbackRate); } catch (...) {}
        }
    }
}
