// Included once by NativeIdleGripPreharvest.cpp. Both users share the verified
// graph-loading and resource-release code; their native managers never overlap.
#include "physics-interaction/weapon/NativeWeaponCycle.h"
#include "physics-interaction/weapon/WeaponCyclePolicy.h"
#include "RE/Bethesda/BSAnimationGraph.h"
#include "physics-interaction/weapon/EquippedWeaponVisualState.h"

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
            RE::NiTransform originalLocal{}, restLocal{}, restInRoot{}, inverseClipRest{};
            std::size_t parent{}, childCount{};
            int bone{-1};
            bool originalVisible{}, visibilityOwned{}, visible{};
        };
        std::array<Node, 512> nodes{};
        std::array<RE::NiTransform, 512> worlds{}, locals{};
        std::size_t nodeCount{}, partCount{};
        std::array<std::int16_t, kMaxBonesAndTracks> parents{}, mapping{};
        std::array<HkQsTransform, kMaxBonesAndTracks> samples{}, referencePose{};
        int boneCount{}, trackCount{}, mappingCount{}, weaponBone{-1};
        void* animation{}; // Pinned by backend graph/resource, never a live actor graph.
        SampleAnimationTracksFn sampler{};
        struct Marker
        {
            float time{};
            RE::BSFixedString tag, payload;
        };
        struct Clip
        {
            void* animation{};
            SampleAnimationTracksFn sampler{};
            float duration{};
            int tracks{}, mappings{};
            std::array<std::int16_t, kMaxBonesAndTracks> mapping{};
            std::array<char, 260> path{};
            std::uint64_t identifier{};
            AnimationResourceHandle resource{};
            std::array<Marker, 128> sounds{};
            std::size_t soundCount{};
            bool ready{};
        };
        enum ClipId : std::size_t { Fire, Reload, ReloadEmpty, ReloadReserve, ClipCount };
        std::array<Clip, ClipCount> clips{};
        std::size_t activeClip{Fire}, nextClip{Reload}, nextSound{};
        RE::BShkbAnimationGraph* loadedGraph{}; // Owned by backend's retained graph holder.
        std::uint64_t subgraphHandle{};
        float duration{}, time{}, playbackRate{1.0f}, sampledTime{-1.0f};
        bool reloading{}, paused{};
        bool ready{}, failed{}, playing{}, restorePending{}, retiring{};
        unsigned shotsLogged{};
        unsigned poseLogs{};
        std::array<int, 2> handBones{-1, -1};
        std::array<std::array<int, 15>, 2> fingerBones{};
        std::array<loose_reload_experiment::HandPose, 2> reloadHands{};
        bool reloadPoseReady{};

        bool boneLocal(int bone, RE::NiTransform& out) const
        {
            if (bone < 0 || bone >= boneCount) return false;
            const auto track = native_idle_grip_preharvest_policy::findTransformTrackForBone(
                bone, trackCount, std::span<const std::int16_t>(mapping.data(), mappingCount));
            return convertHavokLocalTransform(track >= 0 ? samples[track] : referencePose[bone], out) && isFiniteTransform(out);
        }

        bool boneModel(int bone, RE::NiTransform& out, int& rootBone) const
        {
            std::array<int, native_idle_grip_preharvest_policy::kMaxBoneChainLength> chain{};
            const auto length = native_idle_grip_preharvest_policy::collectBoneChainToRoot(
                bone, std::span<const std::int16_t>(parents.data(), boneCount), chain);
            if (!length) return false;
            out = transform_math::makeIdentityTransform<RE::NiTransform>();
            for (std::size_t i = length; i > 0; --i) {
                RE::NiTransform local;
                if (!boneLocal(chain[i-1], local)) return false;
                out = transform_math::composeTransforms(out, local);
            }
            rootBone = chain[length-1];
            return isFiniteTransform(out);
        }

        bool captureReloadHands(bool baseline)
        {
            RE::NiTransform weaponModel;
            int weaponRoot;
            if (!boneModel(weaponBone, weaponModel, weaponRoot)) return false;
            const auto inverseWeapon = transform_math::invertTransform(weaponModel);
            for (unsigned hand = 0; hand < 2; ++hand) {
                auto& pose = reloadHands[hand];
                RE::NiTransform wristModel;
                int wristRoot;
                if (!boneModel(handBones[hand], wristModel, wristRoot) || wristRoot != weaponRoot) return false;
                const auto relative = transform_math::composeTransforms(inverseWeapon, wristModel);
                if (!isFiniteTransform(relative)) return false;
                pose.current = loose_reload_experiment::pack(relative);
                if (baseline) pose.baseline = pose.current;
                pose.valid = 1;
                pose.fingerMask = 0;
                for (unsigned finger = 0; finger < 15; ++finger) {
                    RE::NiTransform local;
                    if (!boneLocal(fingerBones[hand][finger], local)) continue;
                    pose.fingers[finger] = loose_reload_experiment::pack(local);
                    pose.fingerMask |= 1u << finger;
                }
            }
            return true;
        }

        ClipId reloadClip(bool empty) const noexcept
        {
            return empty && clips[ReloadEmpty].ready ? ReloadEmpty :
                !empty && clips[ReloadReserve].ready ? ReloadReserve : Reload;
        }

        ~State()
        {
            restore();
            for (auto& clip : clips) if (clip.resource.entry) {
                AnimationResourceHandle empty{};
                backend.native.moveAnimationResourceHandle(&clip.resource, &empty);
            }
            loadedGraph = nullptr;
            releaseJob(backend);
        }

        bool readSounds(Clip& clip)
        {
            // hkaAnimation destruction (1A31580), annotation-track destruction
            // (1A31650) and independent array teardown (1A31280/1A313E0)
            // establish these bounded arrays. hkStringPtr uses its low bit.
            void* tracks{}; int count{};
            if (!native_memory::tryReadField(clip.animation, 0x28, tracks) ||
                !native_memory::tryReadField(clip.animation, 0x30, count) || count < 0 || count > kMaxBonesAndTracks || (count && !tracks)) return false;
            for (int i = 0; i < count; ++i) {
                const auto* track = static_cast<const std::byte*>(tracks) + i * 0x18;
                void* markers{}; int markerCount{};
                if (!native_memory::tryReadField(track, 8, markers) || !native_memory::tryReadField(track, 0x10, markerCount) ||
                    markerCount < 0 || markerCount > 1024 || (markerCount && !markers)) return false;
                for (int j = 0; j < markerCount; ++j) {
                    const auto* marker = static_cast<const std::byte*>(markers) + j * 0x10;
                    float seconds{}; std::uintptr_t text{};
                    if (!native_memory::tryReadField(marker, 0, seconds) || !native_memory::tryReadField(marker, 8, text) ||
                        !std::isfinite(seconds) || seconds < 0 || seconds > clip.duration) return false;
                    text &= ~std::uintptr_t{1};
                    if (!text) continue;
                    std::array<char, 260> value{};
                    bool terminated{};
                    for (std::size_t k = 0; k < value.size(); ++k) {
                        if (!native_memory::tryReadValue(reinterpret_cast<const char*>(text + k), value[k])) return false;
                        if (!value[k]) { terminated = true; break; }
                    }
                    if (!terminated) return false;
                    const std::string_view name(value.data());
                    const auto split = name.find('.');
                    if (split == name.npos) continue;
                    const auto tag = name.substr(0, split);
                    if (weapon_cycle_policy::presentationEvent(tag) == weapon_cycle_policy::PresentationEvent::Ignore) continue;
                    if (clip.soundCount == clip.sounds.size() || split + 1 == name.size()) return false;
                    auto& sound = clip.sounds[clip.soundCount++];
                    sound.time = seconds;
                    value[split] = '\0';
                    sound.tag = value.data(); sound.payload = value.data() + split + 1;
                }
            }
            std::sort(clip.sounds.begin(), clip.sounds.begin() + clip.soundCount,
                [](const Marker& a, const Marker& b) {
                    if (a.time != b.time) return a.time < b.time;
                    const auto tagOrder = std::strcmp(a.tag.c_str(), b.tag.c_str());
                    return tagOrder ? tagOrder < 0 : std::strcmp(a.payload.c_str(), b.payload.c_str()) < 0;
                });
            // Identical annotations on several tracks represent one event.
            clip.soundCount = std::unique(clip.sounds.begin(), clip.sounds.begin() + clip.soundCount,
                [](const Marker& a, const Marker& b) { return a.time == b.time && a.tag == b.tag && a.payload == b.payload; }) - clip.sounds.begin();
            return true;
        }

        bool readClip(void* binding, Clip& clip)
        {
            std::uint8_t blend{}; void** table{}; const std::int16_t* map{};
            if (!native_memory::tryReadField(binding, kBindingBlendHintOffset, blend) || blend != 0 ||
                !native_memory::tryReadField(binding, kAnimationFromBindingOffset, clip.animation) || !clip.animation ||
                !native_memory::tryReadField(clip.animation, kAnimationDurationOffset, clip.duration) || !std::isfinite(clip.duration) || clip.duration <= 0 || clip.duration > 30 ||
                !native_memory::tryReadField(clip.animation, kAnimationTransformTrackCountOffset, clip.tracks) || clip.tracks <= 0 || clip.tracks > kMaxBonesAndTracks ||
                !native_memory::tryReadField(clip.animation, 0, table) || !table || !native_memory::tryReadValue(reinterpret_cast<SampleAnimationTracksFn*>(table + 5), clip.sampler) || !addressIsExecutable(reinterpret_cast<void*>(clip.sampler)) ||
                !native_memory::tryReadField(binding, kTrackToBoneMappingCountOffset, clip.mappings) || clip.mappings < 0 || clip.mappings > kMaxBonesAndTracks ||
                !native_memory::tryReadField(binding, kTrackToBoneMappingOffset, map)) return false;
            if (clip.mappings && (!map || !native_memory::guardedCopyFromMemory(map, clip.mapping.data(), clip.mappings * sizeof(std::int16_t)))) return false;
            clip.ready = readSounds(clip);
            return clip.ready;
        }

        void selectClip(std::size_t index)
        {
            stopSounds();
            restoreVisibility();
            const auto& clip = clips[index];
            activeClip = index; animation = clip.animation; sampler = clip.sampler;
            duration = clip.duration; trackCount = clip.tracks; mappingCount = clip.mappings; mapping = clip.mapping;
            time = 0; sampledTime = -1; nextSound = 0;
        }

        void restoreVisibility()
        {
            for (std::size_t i = 1; i < nodeCount; ++i) if (nodes[i].visibilityOwned && nodes[i].object) {
                equipped_weapon_visual_state::setLocallyVisible(nodes[i].object.get(), nodes[i].originalVisible);
                nodes[i].visibilityOwned = false;
            }
        }

        bool applyVisibility(const Marker& marker)
        {
            const std::string_view tag(marker.tag.c_str());
            const auto event = weapon_cycle_policy::presentationEvent(tag);
            if (event != weapon_cycle_policy::PresentationEvent::HidePart && event != weapon_cycle_policy::PresentationEvent::ShowPart) return false;
            for (std::size_t i = 1; i < nodeCount; ++i) {
                auto& node = nodes[i];
                if (node.bone >= 0 && node.object->name == marker.payload) {
                    node.visibilityOwned = true;
                    node.visible = event == weapon_cycle_policy::PresentationEvent::ShowPart;
                    equipped_weapon_visual_state::setLocallyVisible(node.object.get(), node.visible);
                }
            }
            return true;
        }

        void stopSounds() noexcept try
        {
            if (playing) if (const auto held = reference.get()) {
                const auto& clip = clips[activeClip];
                for (std::size_t i = 0; i < nextSound; ++i) {
                    const auto& marker = clip.sounds[i];
                    const std::string_view tag(marker.tag.c_str());
                    if (tag != "SoundPlay" && tag != "SoundPlayAt") continue;
                    const RE::BSAnimationGraphEvent event{held->GetHandle().native_handle(), RE::BSFixedString("SoundStop"), marker.payload};
                    static_cast<RE::BSTEventSink<RE::BSAnimationGraphEvent>*>(held.get())->ProcessEvent(event, nullptr);
                }
            }
        }
        catch (...) {
            try { ROCK_LOG_WARN(Animation, "Loose weapon animation sound cleanup failed ref={:08X}", backend.job.referenceFormId); } catch (...) {}
        }

        static std::unique_ptr<State>& retired()
        {
            // One frame-thread retirement slot survives PhysicsInteraction
            // replacement. Like the existing preharvest runtime, its storage
            // is process-lived: DLL/static teardown cannot wait for native IO.
            // New sessions are withheld until this slot has drained.
            static auto* slot = new std::unique_ptr<State>();
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
            const auto held = reference.get();
            if (!held || held->Get3D() != root.get()) return false;
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
            restoreVisibility();
            if (!restorePending || !topologyCurrent()) return;
            for (std::size_t i = 1; i < nodeCount; ++i) {
                auto& node = nodes[i];
                node.object->local = node.originalLocal;
                node.object->world = transform_math::composeTransforms(root->world, node.restInRoot);
            }
            restorePending = false;
        }

        void fail(const char* stage)
        {
            stopSounds();
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
            if (!readClip(binding, clips[Fire])) return false;
            selectClip(Fire);
            void* skeletonOwner{}; void* skeleton{};
            int parentCount{}, poseCount{};
            const std::int16_t* parentData{};
            const HkQsTransform* pose{};
            if (!native_memory::tryReadField(graph, kGraphSkeletonOwnerOffset, skeletonOwner) || !skeletonOwner ||
                !native_memory::tryReadField(skeletonOwner, kSkeletonFromOwnerOffset, skeleton) || !skeleton ||
                !native_memory::tryReadField(skeleton, kSkeletonBoneCountOffset, boneCount) || boneCount <= 0 || boneCount > kMaxBonesAndTracks ||
                !native_memory::tryReadField(skeleton, kSkeletonParentCountOffset, parentCount) || parentCount < boneCount || parentCount > kMaxBonesAndTracks ||
                !native_memory::tryReadField(skeleton, kSkeletonParentIndicesOffset, parentData) || !parentData ||
                !native_memory::guardedCopyFromMemory(parentData, parents.data(), boneCount * sizeof(std::int16_t)) ||
                !native_memory::tryReadField(skeleton, kSkeletonReferencePoseOffset, pose) || !pose ||
                !native_memory::tryReadField(skeleton, kSkeletonReferencePoseCountOffset, poseCount) || poseCount < boneCount || poseCount > kMaxBonesAndTracks ||
                !native_memory::guardedCopyFromMemory(pose, referencePose.data(), boneCount * sizeof(HkQsTransform))) return false;
            const auto weapon = backend.native.findBoneWithName(skeleton, "Weapon", nullptr);
            if (weapon >= static_cast<std::uint64_t>(boneCount)) return false;
            weaponBone = static_cast<int>(weapon);
            const auto boneIndex = [&](const char* name) {
                const auto found = backend.native.findBoneWithName(skeleton, name, nullptr);
                return found < static_cast<std::uint64_t>(boneCount) ? static_cast<int>(found) : -1;
            };
            handBones = {boneIndex("RArm_Hand"), boneIndex("LArm_Hand")};
            for (unsigned i = 0; i < 15; ++i) {
                fingerBones[0][i] = boneIndex(kRightFiringFingerBoneNames[i]);
                fingerBones[1][i] = boneIndex(kLeftSupportFingerBoneNames[i]);
            }
            if (!guardedSampleTracks(sampler, animation, 0, trackCount, samples.data())) return false;
            nodeCount = 1;
            nodes[0].object = root;
            const auto rootInverse = transform_math::invertTransform(root->world);
            for (std::size_t i = 0; i < nodeCount; ++i) {
                auto& node = nodes[i];
                node.originalLocal = node.object->local;
                node.originalVisible = equipped_weapon_visual_state::isLocallyVisible(node.object.get());
                // Physics may already have evaluated worlds without rewriting
                // every local. Preserve that pose; rebuilding stale locals is
                // precisely what separates an assembled gun's components.
                node.restLocal = i ? transform_math::composeTransforms(
                    transform_math::invertTransform(nodes[node.parent].object->world), node.object->world) : node.originalLocal;
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
            auto& fireClip = clips[Fire];
            auto& path = fireClip.path;
            auto& identifier = fireClip.identifier;
            IdleGripExtractionDiagnostics diagnostics{};
            if (!path[0] && !tryFindLoadedGraphIdlePath(graph, handle, identifier, path, diagnostics, &weapon_cycle_policy::fireClipPriority)) {
                fail("unambiguous-fire-clip"); return;
            }
            void* binding{};
            const auto resolved = resolveClipBinding(backend, graph, identifier, path.data(), binding, diagnostics);
            if (resolved == ExtractionResult::Pending) return;
            if (resolved == ExtractionResult::Failed) { fail(extractionFailureName(diagnostics.failure)); return; }
            if (!bind(graph, binding)) { fail("weapon-part-binding"); return; }
            loadedGraph = graph;
            subgraphHandle = handle;
            backend.native.moveAnimationResourceHandle(&clips[Fire].resource, &backend.job.idleClipResource);
            ready = true;
            ROCK_LOG_INFO(Animation, "Akimbo mechanical cycle ready ref={:08X} weapon={:08X} parts={} tracks={} duration={:.4f} clip={}",
                backend.job.referenceFormId, backend.job.weaponFormId, partCount, trackCount, duration, path.data());
        }

        void loadReloadClips()
        {
            if (nextClip >= ClipCount) return;
            auto& clip = clips[nextClip];
            const std::array<unsigned (*)(std::string_view), ClipCount> priorities{
                &weapon_cycle_policy::fireClipPriority, &weapon_cycle_policy::reloadClipPriority,
                &weapon_cycle_policy::emptyReloadClipPriority, &weapon_cycle_policy::reserveReloadClipPriority};
            IdleGripExtractionDiagnostics diagnostics{};
            if (!clip.path[0] && !tryFindLoadedGraphIdlePath(loadedGraph, subgraphHandle, clip.identifier, clip.path, diagnostics, priorities[nextClip])) {
                ++nextClip; return;
            }
            backend.native.moveAnimationResourceHandle(&backend.job.idleClipResource, &clip.resource);
            backend.job.idleClipPath = clip.path;
            void* binding{};
            const auto resolved = resolveClipBinding(backend, loadedGraph, clip.identifier, clip.path.data(), binding, diagnostics);
            backend.native.moveAnimationResourceHandle(&clip.resource, &backend.job.idleClipResource);
            if (resolved == ExtractionResult::Pending) return;
            if (resolved == ExtractionResult::Failed || !readClip(binding, clip)) {
                ROCK_LOG_WARN(Animation, "Loose weapon reload clip unavailable ref={:08X} clip={} stage=binding-or-events",
                    backend.job.referenceFormId, clip.path.data());
            } else {
                ROCK_LOG_INFO(Animation, "Loose weapon reload clip ready ref={:08X} clip={} duration={:.3f} events={}",
                    backend.job.referenceFormId, clip.path.data(), clip.duration, clip.soundCount);
            }
            ++nextClip;
        }

        void apply(float deltaSeconds, bool finalPhase = false)
        {
            if (!playing) return;
            if (!topologyCurrent()) { fail("scene-topology-changed"); return; }
            time = (std::min)(duration, time + playbackRate * (std::isfinite(deltaSeconds) ? (std::max)(0.0f, deltaSeconds) : 0.0f));
            if (sampledTime != time) {
                if (!guardedSampleTracks(sampler, animation, time, trackCount, samples.data())) { fail("sample"); return; }
                sampledTime = time;
            }
            worlds[0] = root->world;
            std::size_t movingParts{}, firstMoving{};
            const bool tracePose = finalPhase && poseLogs < 4 && (shotsLogged <= 4 || reloading);
            // Calculate all destinations before writing. Absolute part deltas
            // avoid applying an animated parent twice and preserve OMOD offsets.
            for (std::size_t i = 1; i < nodeCount; ++i) {
                const auto& node = nodes[i];
                locals[i] = node.restLocal;
                worlds[i] = transform_math::composeTransforms(worlds[node.parent], locals[i]);
                if (node.bone >= 0) {
                    RE::NiTransform sample{};
                    if (!partInWeapon(node.bone, sample)) { fail("part-chain"); return; }
                    // Carry the sampled delta through this part's bind basis.
                    // Applying a clip-space world delta directly to model space
                    // makes a rotated receiver cycle along the wrong axis.
                    worlds[i] = transform_math::composeTransforms(worlds[0],
                        weapon_cycle_policy::retargetPart(node.restInRoot, node.inverseClipRest, sample));
                    locals[i] = transform_math::composeTransforms(transform_math::invertTransform(worlds[node.parent]), worlds[i]);
                }
                if (!isFiniteTransform(worlds[i]) || !isFiniteTransform(locals[i])) { fail("part-transform"); return; }
                if (tracePose && node.bone >= 0) {
                    float change = std::abs(locals[i].translate.x - node.restLocal.translate.x) +
                        std::abs(locals[i].translate.y - node.restLocal.translate.y) + std::abs(locals[i].translate.z - node.restLocal.translate.z);
                    for (unsigned row = 0; row < 3; ++row) for (unsigned column = 0; column < 3; ++column)
                        change += std::abs(locals[i].rotate.entry[row][column] - node.restLocal.rotate.entry[row][column]);
                    if (change > 0.0001f) { if (!movingParts) firstMoving = i; ++movingParts; }
                }
            }
            if (tracePose && (movingParts || poseLogs == 0)) {
                ++poseLogs;
                const auto& node = nodes[firstMoving];
                const auto& before = node.object->world.translate;
                const auto& after = worlds[firstMoving].translate;
                ROCK_LOG_INFO(Animation, "Loose weapon pose ref={:08X} phase=after-world-final clip={} time={:.4f}/{:.4f} rate={:.3f} movingParts={} node={} before=({:.3f},{:.3f},{:.3f}) after=({:.3f},{:.3f},{:.3f})",
                    backend.job.referenceFormId, clips[activeClip].path.data(), time, duration, playbackRate, movingParts,
                    node.object->name.c_str() ? node.object->name.c_str() : "<unnamed>", before.x, before.y, before.z, after.x, after.y, after.z);
            }
            for (std::size_t i = 1; i < nodeCount; ++i) {
                nodes[i].object->local = locals[i];
                nodes[i].object->world = worlds[i];
                if (nodes[i].visibilityOwned && equipped_weapon_visual_state::isLocallyVisible(nodes[i].object.get()) != nodes[i].visible)
                    equipped_weapon_visual_state::setLocallyVisible(nodes[i].object.get(), nodes[i].visible);
            }
            restorePending = true;
            if (!paused) if (const auto held = reference.get()) {
                const auto& clip = clips[activeClip];
                while (nextSound < clip.soundCount && clip.sounds[nextSound].time <= time) {
                    const auto& marker = clip.sounds[nextSound++];
                    if (applyVisibility(marker)) continue;
                    // Only sound events reach the loose reference's native
                    // sink. No actor/body reload or equip event is dispatched.
                    const RE::BSAnimationGraphEvent event{held->GetHandle().native_handle(), marker.tag, marker.payload};
                    static_cast<RE::BSTEventSink<RE::BSAnimationGraphEvent>*>(held.get())->ProcessEvent(event, nullptr);
                }
            }
            // Keep the authored final weapon pose until the next operation.
            // The render phase reapplies it after native loose-body updates.
        }
    };

    Session::Session() noexcept = default;
    bool Session::ready() const noexcept { return _state && _state->ready && !_state->failed && !_state->retiring; }
    bool Session::failed() const noexcept { return _state && _state->failed; }
    bool Session::reloading() const noexcept { return ready() && _state->reloading; }
    void Session::present() noexcept try
    {
        if (ready()) _state->apply(0.0f, true);
    }
    catch (...) { if (_state) { try { _state->fail("late-presentation"); } catch (...) {} } }
    Session::~Session()
    {
        clear();
        if (_state) State::retired() = std::move(_state);
    }
    void Session::reap() noexcept
    {
        auto& retired = State::retired();
        if (retired && retired->canRelease()) retired.reset();
        if (_state && _state->retiring && _state->canRelease()) _state.reset();
    }

    void Session::clear() noexcept
    {
        if (_state && !_state->retiring) {
            _state->stopSounds();
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

    void Session::update(RE::TESObjectREFR* reference, float deltaSeconds, bool inputAllowed) noexcept try
    {
        auto* root = reference ? reference->Get3D() : nullptr;
        if (!root) { clear(); return; }
        if (_state && (_state->reference != reference->GetHandle() || _state->root.get() != root)) clear();
        reap();
        if (_state && _state->retiring) return;
        if (State::retired()) return;
        if (!_state) {
            _state = std::make_unique<State>();
            _state->reference = reference->GetHandle();
            _state->root.reset(root);
            if (!claimOrValidateThread(_state->backend) || !resolveNativeFunctions(_state->backend)) {
                _state->fail("native-contract"); return;
            }
            startJob(_state->backend, describeLooseCandidate(reference, root));
        }
        if (_state->failed) return;
        if (!inputAllowed && !_state->paused) _state->stopSounds();
        _state->paused = !inputAllowed;
        if (!_state->ready) _state->load();
        if (_state->ready) {
            _state->loadReloadClips();
            _state->apply(inputAllowed ? deltaSeconds : 0.0f);
        }
    }
    catch (...) {
        if (_state) { try { _state->fail("exception"); } catch (...) {} }
    }

    void Session::fire(float secondsPerShot) noexcept
    {
        if (!_state || !_state->ready || _state->failed || _state->retiring) return;
        _state->playing = true;
        _state->reloading = false;
        _state->selectClip(State::Fire);
        _state->poseLogs = 0;
        // A long single-shot clip must complete between automatic shots.
        _state->playbackRate = std::isfinite(secondsPerShot) && secondsPerShot > 0 ?
            (std::max)(1.0f, _state->duration / secondsPerShot) : 1.0f;
        if (_state->shotsLogged++ < 4) {
            try { ROCK_LOG_INFO(Animation, "Akimbo mechanical stroke ref={:08X} parts={}", _state->backend.job.referenceFormId, _state->partCount); } catch (...) {}
        }
    }

    bool Session::reload(float seconds, bool empty, float elapsed) noexcept try
    {
        if (!ready() || !std::isfinite(seconds) || seconds <= 0 || _state->reloading) return false;
        const auto choice = _state->reloadClip(empty);
        if (!_state->clips[choice].ready) {
            ROCK_LOG_SAMPLE_WARN(Animation, 1000, "Loose weapon reload waiting/unavailable ref={:08X}: exact weapon-only clip not ready",
                _state->backend.job.referenceFormId);
            return false;
        }
        _state->selectClip(choice);
        _state->reloadHands = {};
        _state->reloadPoseReady = guardedSampleTracks(_state->sampler, _state->animation, 0.0f,
            _state->trackCount, _state->samples.data()) && _state->captureReloadHands(true);
        _state->sampledTime = _state->reloadPoseReady ? 0.0f : -1.0f;
        if (!_state->reloadPoseReady) {
            ROCK_LOG_WARN(Animation, "Loose reload IK unavailable ref={:08X} clip={}: wrist tracks or common root missing",
                _state->backend.job.referenceFormId, _state->clips[choice].path.data());
        }
        _state->poseLogs = 0;
        _state->playbackRate = _state->duration / seconds;
        _state->time = std::isfinite(elapsed) ? std::clamp(elapsed / seconds, 0.0f, 1.0f) * _state->duration : 0.0f;
        while (_state->nextSound < _state->clips[choice].soundCount && _state->clips[choice].sounds[_state->nextSound].time < _state->time)
            _state->applyVisibility(_state->clips[choice].sounds[_state->nextSound++]);
        _state->playing = true; _state->reloading = true;
        return true;
    }
    catch (...) {
        if (_state) { try { _state->fail("reload-presentation"); } catch (...) {} }
        return false;
    }

    float Session::reloadSeconds(bool empty, float speed) const noexcept
    {
        if (!ready() || _state->nextClip < State::ClipCount || !std::isfinite(speed) || speed <= 0) return 0;
        const auto& clip = _state->clips[_state->reloadClip(empty)];
        return clip.ready ? clip.duration / speed : 0;
    }

    void Session::finishReload() noexcept
    {
        if (_state) _state->reloading = false;
    }

    bool Session::copyReloadPose(loose_reload_experiment::Snapshot& out) noexcept try
    {
        if (!reloading() || _state->paused || !_state->reloadPoseReady) return false;
        // Same sample and clock as the parts; this never advances the reload.
        _state->apply(0.0f);
        if (!ready() || !_state->captureReloadHands(false)) return false;
        out.time = _state->time;
        out.duration = _state->duration;
        out.weaponWorld = loose_reload_experiment::pack(_state->root->world);
        for (unsigned hand = 0; hand < 2; ++hand) out.hands[hand] = _state->reloadHands[hand];
        return true;
    }
    catch (...) {
        try { ROCK_LOG_SAMPLE_WARN(Animation, 1000, "Loose reload IK snapshot failed; weapon reload retained"); } catch (...) {}
        return false;
    }
}
