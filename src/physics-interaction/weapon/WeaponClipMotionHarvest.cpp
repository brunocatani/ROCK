#include "physics-interaction/weapon/WeaponClipMotionHarvest.h"

#include "physics-interaction/PhysicsLog.h"

#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstring>
#include <mutex>

namespace rock::weapon_clip_motion_harvest
{
    namespace
    {
        /*
         * All offsets below were verified against the FO4VR binary on
         * 2026-07-03 (Ghidra decompiles + VR address library cross-check);
         * see docs/research/2026-07-03-baked-animation-motion-extraction.md
         * Addendum 2. Chain: WeaponAnimationGraphManagerHolder →
         * BSAnimationGraphManager → BShkbAnimationGraph → inline hkbCharacter
         * → hkbCharacterSetup → { hkaSkeleton, hkbAnimationBindingSet }.
         */

        // WeaponAnimationGraphManagerHolder (ctor disassembly, 0x140812c20):
        // +0x00 IAnimationGraphManagerHolder vtable, +0x08 second base
        // vtable, +0x10 refcount, +0x18 =
        // BSTSmartPointer<BSAnimationGraphManager>. (+0x8 is a vtable
        // pointer — plausible-looking, which is why a wrong read here fails
        // one hop later at the graphs array.)
        constexpr std::uintptr_t kHolderManagerOffset = 0x18;

        // BSAnimationGraphManager (ctor decompile, 0x14168f4f0): graphs are a
        // BSTSmallArray of BSTSmartPointer<BShkbAnimationGraph> — capacity
        // dword at +0x40 (bit31 set = inline storage), storage at +0x48
        // (inline entry or heap pointer), active graph index at +0xD8.
        constexpr std::uintptr_t kManagerGraphsCapacityOffset = 0x40;
        constexpr std::uintptr_t kManagerGraphsStorageOffset = 0x48;
        constexpr std::uintptr_t kManagerActiveGraphOffset = 0xD8;
        constexpr std::uint32_t kGraphsInlineStorageFlag = 0x8000'0000u;
        constexpr std::uint32_t kMaxPlausibleActiveGraphIndex = 15;

        // BShkbAnimationGraph: hkbCharacter is INLINE at +0x1C8 — the graph
        // ctor (0x1416a3150) constructs it at this[1].field_0x50 with a
        // 0x178-byte Ghidra struct (0x178 + 0x50), matching the binding-set
        // builder receiving graph+0x1C8 as its owner (hkBaseObject_data base
        // is +0x8, so its field_0x1c0 is absolute 0x1C8).
        constexpr std::uintptr_t kGraphCharacterOffset = 0x1C8;

        // hkbCharacter: +0x78 hkbCharacterSetup*, +0x90 binding-set override
        // (engine getter 0x141902dc0 prefers the override).
        constexpr std::uintptr_t kCharacterSetupOffset = 0x78;
        constexpr std::uintptr_t kCharacterBindingSetOverrideOffset = 0x90;

        // hkbCharacterSetup: +0x20 m_animationSkeleton (the skeleton the clip
        // tracks map to — for the weapon graph, the weapon rig), +0x38
        // m_animationBindingSet.
        constexpr std::uintptr_t kSetupAnimationSkeletonOffset = 0x20;
        constexpr std::uintptr_t kSetupBindingSetOffset = 0x38;

        // hkbAnimationBindingSet: +0x10 bindings data
        // (hkbAnimationBindingWithTriggers*[]), +0x18 int count.
        constexpr std::uintptr_t kBindingSetDataOffset = 0x10;
        constexpr std::uintptr_t kBindingSetCountOffset = 0x18;
        constexpr std::int32_t kMaxPlausibleBindingCount = 4096;

        // hkbAnimationBindingWithTriggers (0x30-byte hkReferencedObject):
        // +0x8 is the memSizeAndFlags/refCount header — the binding pointer
        // is at +0x10 (crash-log verified; reading +0x8 dereferences the
        // 0xFFFF0001 refcount pattern).
        constexpr std::uintptr_t kBindingWithTriggersBindingOffset = 0x10;

        // hkaAnimationBinding members.
        constexpr std::uintptr_t kBindingAnimationOffset = 0x18;
        constexpr std::uintptr_t kBindingTrackToBoneDataOffset = 0x40;
        constexpr std::uintptr_t kBindingTrackToBoneCountOffset = 0x48;

        // hkaSkeleton members (bones array of 0x10-byte hkaBone entries with
        // the name char* at +0; low pointer bit is an engine flag).
        constexpr std::uintptr_t kSkeletonBonesDataOffset = 0x28;
        constexpr std::uintptr_t kSkeletonBonesCountOffset = 0x30;
        constexpr std::uintptr_t kSkeletonBoneStride = 0x10;

        // hkaAnimation members.
        constexpr std::uintptr_t kAnimationDurationOffset = 0x14;
        constexpr std::uintptr_t kAnimationTrackCountOffset = 0x18;
        // hkaAnimation vtable slot 6: sampleIndividualTransformTracks(
        //   float time, const int16* trackIndices, uint32 count, out*)
        constexpr std::size_t kSampleIndividualTransformTracksSlot = 6;

        constexpr float kMinClipDurationSeconds = 0.01f;
        constexpr float kMaxClipDurationSeconds = 300.0f;
        constexpr std::int32_t kMaxPlausibleTrackCount = 512;
        constexpr std::int32_t kMaxPlausibleBoneCount = 4096;
        // Sized for a burst of consecutive weapon clips on an actor graph
        // (several groups per clip between per-frame drains).
        constexpr std::size_t kQueueCapacity = 64;
        // Bindings sampled per stepHarvest call; bounds the per-frame cost of
        // the at-equip walk (each binding = up to kMaxTracksPerClip tracks x
        // kClipSampleCount engine sampler calls).
        constexpr std::int32_t kBindingsPerStep = 6;

        // hkQsTransform: vec4 translate, quaternion (x,y,z,w), vec4 scale.
        struct HkQsTransform
        {
            float translate[4];
            float rotate[4];
            float scale[4];
        };
        static_assert(sizeof(HkQsTransform) == 48);

        using SampleIndividualTransformTracks_t = void (*)(void*, float, const std::int16_t*, std::uint32_t, HkQsTransform*);

        std::mutex s_queueMutex;
        std::array<weapon_clip_stroke::AuthoredStrokeGroup, kQueueCapacity> s_queue{};
        std::uint32_t s_queueCount = 0;

        std::atomic<std::uint64_t> s_bindingsSeen{ 0 };
        std::atomic<std::uint64_t> s_bindingsHarvested{ 0 };
        std::atomic<std::uint64_t> s_bindingsNoTargets{ 0 };
        std::atomic<std::uint64_t> s_groupsQueued{ 0 };
        std::atomic<std::uint64_t> s_groupsDropped{ 0 };
        std::atomic<std::uint64_t> s_skippedNonSpline{ 0 };
        std::atomic<std::uint64_t> s_walksCompleted{ 0 };
        // Bail-reason counters: which harvestBinding gate rejected a binding
        // (a binding that passes all gates lands in harvested/noTargets).
        std::atomic<std::uint64_t> s_bailAnimationPtr{ 0 };
        std::atomic<std::uint64_t> s_bailClipParams{ 0 };
        std::atomic<std::uint64_t> s_bailTrackMap{ 0 };
        std::atomic<std::uint64_t> s_bailBoneCount{ 0 };
        std::atomic<std::uint64_t> s_bailSampler{ 0 };

        // Detailed bail dumps per walk (main thread; reset with the cursor)
        // so a failing binding is identifiable without flooding the log.
        constexpr std::uint32_t kMaxBindingDetailLogsPerWalk = 3;
        std::uint32_t s_bindingDetailLogs = 0;

        // Walk cursor (main thread only). No engine pointers are stored —
        // the chain is re-resolved from the holder on every step.
        std::uint32_t s_walkFormId = 0;
        std::uint64_t s_walkGenerationKey = 0;
        std::int32_t s_walkBindingIndex = 0;
        // Data pointer of the binding set the cursor indexes into; a change
        // (graph swap / candidate switch) restarts the walk.
        std::uintptr_t s_walkBindingsData = 0;
        bool s_walkDone = false;
        // Deepest chain hop reached by the most recent resolve attempt;
        // reported by the caller when a walk gives up so the failing stage
        // is visible in the log instead of a generic timeout.
        const char* s_lastResolveStage = "none";

        // Coarse pointer plausibility gate for values read out of engine
        // objects; rejects null, refcount headers, and other small integers
        // before they are dereferenced.
        [[nodiscard]] bool plausiblePointer(std::uintptr_t value)
        {
            return value > 0x10000 && value < 0x0000'8000'0000'0000ull;
        }

        [[nodiscard]] std::uintptr_t moduleRelative(std::uintptr_t address)
        {
            const auto base = REL::Module::get().base();
            return address >= base ? address - base : address;
        }

        // Module-relative vtable of a heap object (0 when the pointer is
        // implausible); a rebased value can be looked up directly in the
        // binary to identify the object's real runtime type.
        [[nodiscard]] std::uintptr_t objectVtableRel(std::uintptr_t object)
        {
            return plausiblePointer(object) ? moduleRelative(*reinterpret_cast<const std::uintptr_t*>(object)) : 0;
        }

        void logBindingBail(
            const char* reason,
            std::uintptr_t binding,
            std::uintptr_t animation,
            float duration,
            std::int32_t trackCount,
            std::int32_t trackToBoneCount,
            std::int32_t boneCount)
        {
            if (s_bindingDetailLogs >= kMaxBindingDetailLogsPerWalk) {
                return;
            }
            ++s_bindingDetailLogs;
            ROCK_LOG_WARN(Weapon,
                "WeaponClipMotionHarvest: binding bail [{}] binding={:#x}(vt+{:#x}) anim={:#x}(vt+{:#x}) duration={} trackCount={} trackToBone={} bones={}",
                reason,
                binding,
                objectVtableRel(binding),
                animation,
                objectVtableRel(animation),
                duration,
                trackCount,
                trackToBoneCount,
                boneCount);
        }

        const char* skeletonBoneName(std::uintptr_t skeleton, std::int32_t boneIndex)
        {
            const auto bonesData = *reinterpret_cast<std::uintptr_t*>(skeleton + kSkeletonBonesDataOffset);
            if (!plausiblePointer(bonesData)) {
                return nullptr;
            }
            const auto entry = bonesData + static_cast<std::uintptr_t>(boneIndex) * kSkeletonBoneStride;
            const auto namePtr = *reinterpret_cast<std::uintptr_t*>(entry) & ~static_cast<std::uintptr_t>(1);
            return plausiblePointer(namePtr) ? reinterpret_cast<const char*>(namePtr) : nullptr;
        }

        // ASCII case-insensitive equality over `length` characters.
        bool namesEqualNoCase(const char* a, const char* b, std::size_t length)
        {
            for (std::size_t i = 0; i < length; ++i) {
                const auto ca = static_cast<unsigned char>(a[i]);
                const auto cb = static_cast<unsigned char>(b[i]);
                if (std::tolower(ca) != std::tolower(cb)) {
                    return false;
                }
            }
            return true;
        }

        /*
         * A rig bone is a harvest target when it matches one of the weapon's
         * scene-node names: exact (case-insensitive, engine names are
         * case-insensitive) or with a ':N' instancing suffix on the node
         * side ('Bolt_Carrier' bone vs 'Bolt_Carrier:0' node). The filter is
         * what makes walking the ACTOR's graph safe — body-clip tracks never
         * match a weapon node name.
         */
        bool isHarvestTargetBone(const char* boneName, const char* const* allowedNodeNames, std::uint32_t allowedNodeNameCount)
        {
            if (!boneName || boneName[0] == '\0' || !allowedNodeNames) {
                return false;
            }
            const auto boneLength = std::strlen(boneName);
            for (std::uint32_t i = 0; i < allowedNodeNameCount; ++i) {
                const char* nodeName = allowedNodeNames[i];
                if (!nodeName) {
                    continue;
                }
                const auto nodeLength = std::strlen(nodeName);
                if (nodeLength < boneLength || !namesEqualNoCase(boneName, nodeName, boneLength)) {
                    continue;
                }
                if (nodeLength == boneLength || nodeName[boneLength] == ':') {
                    return true;
                }
            }
            return false;
        }

        void harvestBinding(
            std::uintptr_t binding,
            std::uintptr_t skeleton,
            const char* const* allowedNodeNames,
            std::uint32_t allowedNodeNameCount)
        {
            const auto animation = *reinterpret_cast<std::uintptr_t*>(binding + kBindingAnimationOffset);
            if (!plausiblePointer(animation)) {
                s_bailAnimationPtr.fetch_add(1, std::memory_order_relaxed);
                logBindingBail("animation-ptr", binding, animation, 0.0f, 0, 0, 0);
                return;
            }

            // Only spline-compressed clips are supported; the sampler slot is
            // dispatched virtually but the slot semantics were verified on
            // this class specifically. Other compressions are counted so the
            // gap is visible instead of silent.
            const auto vtable = *reinterpret_cast<std::uintptr_t*>(animation);
            if (vtable != RE::VTABLE::hkaSplineCompressedAnimation[0].address()) {
                s_skippedNonSpline.fetch_add(1, std::memory_order_relaxed);
                logBindingBail("non-spline", binding, animation, 0.0f, 0, 0, 0);
                return;
            }

            const float duration = *reinterpret_cast<float*>(animation + kAnimationDurationOffset);
            const auto animationTrackCount = *reinterpret_cast<std::int32_t*>(animation + kAnimationTrackCountOffset);
            if (!std::isfinite(duration) || duration < kMinClipDurationSeconds || duration > kMaxClipDurationSeconds ||
                animationTrackCount <= 0 || animationTrackCount > kMaxPlausibleTrackCount) {
                s_bailClipParams.fetch_add(1, std::memory_order_relaxed);
                logBindingBail("clip-params", binding, animation, duration, animationTrackCount, 0, 0);
                return;
            }

            const auto trackToBoneData = *reinterpret_cast<std::uintptr_t*>(binding + kBindingTrackToBoneDataOffset);
            const auto trackToBoneCount = *reinterpret_cast<std::int32_t*>(binding + kBindingTrackToBoneCountOffset);
            if (!plausiblePointer(trackToBoneData) || trackToBoneCount <= 0 || trackToBoneCount > kMaxPlausibleTrackCount) {
                s_bailTrackMap.fetch_add(1, std::memory_order_relaxed);
                logBindingBail("track-map", binding, animation, duration, animationTrackCount, trackToBoneCount, 0);
                return;
            }
            const auto boneCount = *reinterpret_cast<std::int32_t*>(skeleton + kSkeletonBonesCountOffset);
            if (boneCount <= 0 || boneCount > kMaxPlausibleBoneCount) {
                s_bailBoneCount.fetch_add(1, std::memory_order_relaxed);
                logBindingBail("bone-count", binding, animation, duration, animationTrackCount, trackToBoneCount, boneCount);
                return;
            }
            const auto* trackToBone = reinterpret_cast<const std::int16_t*>(trackToBoneData);

            // Collect the weapon-part tracks for this clip.
            std::array<std::int16_t, weapon_clip_stroke::kMaxTracksPerClip> trackIndices{};
            std::array<weapon_clip_stroke::TrackSamples, weapon_clip_stroke::kMaxTracksPerClip> tracks{};
            std::uint32_t targetCount = 0;
            const auto usableTrackCount = (std::min)(trackToBoneCount, animationTrackCount);
            for (std::int32_t track = 0; track < usableTrackCount && targetCount < tracks.size(); ++track) {
                const auto boneIndex = trackToBone[track];
                if (boneIndex < 0 || boneIndex >= boneCount) {
                    continue;
                }
                const char* name = skeletonBoneName(skeleton, boneIndex);
                if (!isHarvestTargetBone(name, allowedNodeNames, allowedNodeNameCount)) {
                    continue;
                }
                trackIndices[targetCount] = static_cast<std::int16_t>(track);
                auto& samples = tracks[targetCount];
                samples = {};
                std::size_t nameLength = 0;
                while (nameLength < samples.boneName.size() - 1 && name[nameLength] != '\0') {
                    ++nameLength;
                }
                std::memcpy(samples.boneName.data(), name, nameLength);
                ++targetCount;
            }
            if (targetCount == 0) {
                s_bindingsNoTargets.fetch_add(1, std::memory_order_relaxed);
                return;
            }

            const auto sampler = reinterpret_cast<SampleIndividualTransformTracks_t>(
                reinterpret_cast<std::uintptr_t*>(vtable)[kSampleIndividualTransformTracksSlot]);
            if (!sampler) {
                s_bailSampler.fetch_add(1, std::memory_order_relaxed);
                logBindingBail("sampler", binding, animation, duration, animationTrackCount, trackToBoneCount, boneCount);
                return;
            }

            std::array<HkQsTransform, weapon_clip_stroke::kMaxTracksPerClip> sampled{};
            for (std::uint32_t step = 0; step < weapon_clip_stroke::kClipSampleCount; ++step) {
                const float time = duration * static_cast<float>(step) /
                                   static_cast<float>(weapon_clip_stroke::kClipSampleCount - 1);
                sampler(reinterpret_cast<void*>(animation), time, trackIndices.data(), targetCount, sampled.data());
                for (std::uint32_t i = 0; i < targetCount; ++i) {
                    auto& pose = tracks[i].samples[step];
                    pose.translate = weapon_part_motion_path::Vec3{
                        sampled[i].translate[0],
                        sampled[i].translate[1],
                        sampled[i].translate[2],
                    };
                    // Havok quaternion order is (x, y, z, w).
                    pose.rotate = weapon_part_motion_path::quatNormalizeOrIdentity(weapon_part_motion_path::Quat{
                        sampled[i].rotate[3],
                        sampled[i].rotate[0],
                        sampled[i].rotate[1],
                        sampled[i].rotate[2],
                    });
                }
            }
            for (std::uint32_t i = 0; i < targetCount; ++i) {
                tracks[i].sampleCount = weapon_clip_stroke::kClipSampleCount;
            }

            std::array<weapon_clip_stroke::AuthoredStrokeGroup, weapon_clip_stroke::kMaxGroupsPerClip> groups{};
            const auto groupCount = weapon_clip_stroke::buildAuthoredGroups(
                tracks.data(),
                targetCount,
                groups.data(),
                static_cast<std::uint32_t>(groups.size()));
            if (groupCount == 0) {
                return;
            }
            s_bindingsHarvested.fetch_add(1, std::memory_order_relaxed);

            std::scoped_lock lock(s_queueMutex);
            for (std::uint32_t i = 0; i < groupCount; ++i) {
                if (s_queueCount >= s_queue.size()) {
                    s_groupsDropped.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                s_queue[s_queueCount++] = groups[i];
                s_groupsQueued.fetch_add(1, std::memory_order_relaxed);
            }
        }

        /*
         * Resolve holder → binding set + skeleton. Returns false while the
         * weapon graph or its bindings are not available yet (still loading)
         * or any pointer fails the plausibility gate.
         */
        struct ResolvedBindings
        {
            std::uintptr_t skeleton{ 0 };
            std::uintptr_t bindingsData{ 0 };
            std::int32_t bindingCount{ 0 };
        };

        bool resolveManagerBindings(const void* graphManager, ResolvedBindings& out)
        {
            const auto manager = reinterpret_cast<std::uintptr_t>(graphManager);
            s_lastResolveStage = "manager";
            if (!plausiblePointer(manager)) {
                return false;
            }
            s_lastResolveStage = "graphs-array";
            const auto capacityAndFlags = *reinterpret_cast<std::uint32_t*>(manager + kManagerGraphsCapacityOffset);
            const auto storageAddress = manager + kManagerGraphsStorageOffset;
            const auto graphsBase = (capacityAndFlags & kGraphsInlineStorageFlag) != 0
                ? storageAddress
                : *reinterpret_cast<std::uintptr_t*>(storageAddress);
            if (!plausiblePointer(graphsBase)) {
                return false;
            }
            s_lastResolveStage = "active-index";
            const auto activeGraphIndex = *reinterpret_cast<std::uint32_t*>(manager + kManagerActiveGraphOffset);
            if (activeGraphIndex > kMaxPlausibleActiveGraphIndex) {
                return false;
            }
            s_lastResolveStage = "graph";
            const auto graph = reinterpret_cast<const std::uintptr_t*>(graphsBase)[activeGraphIndex];
            if (!plausiblePointer(graph)) {
                return false;
            }
            s_lastResolveStage = "character-setup";
            const auto character = graph + kGraphCharacterOffset;
            const auto setup = *reinterpret_cast<std::uintptr_t*>(character + kCharacterSetupOffset);
            if (!plausiblePointer(setup)) {
                return false;
            }
            s_lastResolveStage = "skeleton";
            const auto skeleton = *reinterpret_cast<std::uintptr_t*>(setup + kSetupAnimationSkeletonOffset);
            if (!plausiblePointer(skeleton)) {
                return false;
            }
            s_lastResolveStage = "binding-set";
            auto bindingSet = *reinterpret_cast<std::uintptr_t*>(character + kCharacterBindingSetOverrideOffset);
            if (!plausiblePointer(bindingSet)) {
                bindingSet = *reinterpret_cast<std::uintptr_t*>(setup + kSetupBindingSetOffset);
            }
            if (!plausiblePointer(bindingSet)) {
                return false;
            }
            s_lastResolveStage = "bindings";
            const auto bindingsData = *reinterpret_cast<std::uintptr_t*>(bindingSet + kBindingSetDataOffset);
            const auto bindingCount = *reinterpret_cast<std::int32_t*>(bindingSet + kBindingSetCountOffset);
            if (!plausiblePointer(bindingsData) || bindingCount <= 0 || bindingCount > kMaxPlausibleBindingCount) {
                return false;
            }
            s_lastResolveStage = "ok";
            out.skeleton = skeleton;
            out.bindingsData = bindingsData;
            out.bindingCount = bindingCount;
            return true;
        }
    }

    const char* lastResolveStage()
    {
        return s_lastResolveStage;
    }

    const void* managerFromWeaponHolder(const void* weaponGraphHolder)
    {
        const auto holder = reinterpret_cast<std::uintptr_t>(weaponGraphHolder);
        if (!plausiblePointer(holder)) {
            return nullptr;
        }
        const auto manager = *reinterpret_cast<std::uintptr_t*>(holder + kHolderManagerOffset);
        return plausiblePointer(manager) ? reinterpret_cast<const void*>(manager) : nullptr;
    }

    bool probeBindings(const void* graphManager)
    {
        ResolvedBindings resolved{};
        return resolveManagerBindings(graphManager, resolved);
    }

    void logResolveDiagnostics(const void* graphManager, const char* label)
    {
        const auto vtableRel = [](std::uintptr_t object) { return objectVtableRel(object); };

        const auto manager = reinterpret_cast<std::uintptr_t>(graphManager);
        std::uintptr_t graphsBase = 0;
        std::uintptr_t graph = 0;
        std::uintptr_t character = 0;
        std::uintptr_t setup = 0;
        std::uintptr_t skeleton = 0;
        std::uintptr_t bindingSet = 0;
        std::uintptr_t bindingsData = 0;
        std::uint32_t capacityAndFlags = 0;
        std::uint32_t activeGraphIndex = 0;
        std::int32_t bindingCount = -1;
        std::int32_t boneCount = -1;
        const char* bindingSetSource = "none";
        std::array<const char*, 3> firstBoneNames{ "", "", "" };
        const char* lastBoneName = "";

        if (plausiblePointer(manager)) {
            capacityAndFlags = *reinterpret_cast<std::uint32_t*>(manager + kManagerGraphsCapacityOffset);
            const auto storageAddress = manager + kManagerGraphsStorageOffset;
            graphsBase = (capacityAndFlags & kGraphsInlineStorageFlag) != 0
                ? storageAddress
                : *reinterpret_cast<std::uintptr_t*>(storageAddress);
            activeGraphIndex = *reinterpret_cast<std::uint32_t*>(manager + kManagerActiveGraphOffset);
        }
        if (plausiblePointer(graphsBase) && activeGraphIndex <= kMaxPlausibleActiveGraphIndex) {
            graph = reinterpret_cast<const std::uintptr_t*>(graphsBase)[activeGraphIndex];
        }
        if (plausiblePointer(graph)) {
            character = graph + kGraphCharacterOffset;
            setup = *reinterpret_cast<std::uintptr_t*>(character + kCharacterSetupOffset);
            const auto overrideSet = *reinterpret_cast<std::uintptr_t*>(character + kCharacterBindingSetOverrideOffset);
            if (plausiblePointer(overrideSet)) {
                bindingSet = overrideSet;
                bindingSetSource = "override";
            } else if (plausiblePointer(setup)) {
                bindingSet = *reinterpret_cast<std::uintptr_t*>(setup + kSetupBindingSetOffset);
                if (plausiblePointer(bindingSet)) {
                    bindingSetSource = "setup";
                }
            }
        }
        if (plausiblePointer(setup)) {
            skeleton = *reinterpret_cast<std::uintptr_t*>(setup + kSetupAnimationSkeletonOffset);
        }
        if (plausiblePointer(skeleton)) {
            boneCount = *reinterpret_cast<std::int32_t*>(skeleton + kSkeletonBonesCountOffset);
            if (boneCount > 0 && boneCount <= kMaxPlausibleBoneCount) {
                for (std::int32_t i = 0; i < boneCount && i < 3; ++i) {
                    if (const char* name = skeletonBoneName(skeleton, i)) {
                        firstBoneNames[static_cast<std::size_t>(i)] = name;
                    }
                }
                if (const char* name = skeletonBoneName(skeleton, boneCount - 1)) {
                    lastBoneName = name;
                }
            }
        }
        if (plausiblePointer(bindingSet)) {
            bindingsData = *reinterpret_cast<std::uintptr_t*>(bindingSet + kBindingSetDataOffset);
            bindingCount = *reinterpret_cast<std::int32_t*>(bindingSet + kBindingSetCountOffset);
        }

        ROCK_LOG_WARN(Weapon,
            "WeaponClipMotionHarvest diagnostics [{}]: mgr={:#x}(vt+{:#x}) "
            "graphsFlags={:#010x} activeIdx={} graph={:#x}(vt+{:#x}) charVt=+{:#x} "
            "setup={:#x}(vt+{:#x}) skel={:#x}(vt+{:#x}) bones={} first=[{}|{}|{}] last=[{}] "
            "setSrc={} set={:#x}(vt+{:#x}) data={:#x} count={}",
            label ? label : "?",
            manager, vtableRel(manager),
            capacityAndFlags, activeGraphIndex, graph, vtableRel(graph),
            plausiblePointer(graph) ? vtableRel(character) : 0,
            setup, vtableRel(setup),
            skeleton, vtableRel(skeleton),
            boneCount, firstBoneNames[0], firstBoneNames[1], firstBoneNames[2], lastBoneName,
            bindingSetSource, bindingSet, vtableRel(bindingSet),
            bindingsData, bindingCount);
    }

    StepResult stepHarvest(
        const void* graphManager,
        std::uint32_t weaponFormId,
        std::uint64_t weaponGenerationKey,
        const char* const* allowedNodeNames,
        std::uint32_t allowedNodeNameCount)
    {
        if (s_walkFormId != weaponFormId || s_walkGenerationKey != weaponGenerationKey) {
            s_walkFormId = weaponFormId;
            s_walkGenerationKey = weaponGenerationKey;
            s_walkBindingIndex = 0;
            s_walkBindingsData = 0;
            s_walkDone = false;
            s_bindingDetailLogs = 0;
        }
        if (s_walkDone) {
            return StepResult::Completed;
        }

        ResolvedBindings resolved{};
        if (!resolveManagerBindings(graphManager, resolved)) {
            // Graph or bindings not built yet; the caller retries next frame.
            return StepResult::Pending;
        }
        if (resolved.bindingsData != s_walkBindingsData) {
            // Different binding set than the cursor was walking (graph swap
            // at equip, or the caller switched candidate managers); indices
            // are not comparable across sets, so restart.
            s_walkBindingsData = resolved.bindingsData;
            s_walkBindingIndex = 0;
        }

        const auto* bindings = reinterpret_cast<const std::uintptr_t*>(resolved.bindingsData);
        std::int32_t processed = 0;
        while (s_walkBindingIndex < resolved.bindingCount && processed < kBindingsPerStep) {
            const auto bindingWithTriggers = bindings[s_walkBindingIndex++];
            ++processed;
            if (!plausiblePointer(bindingWithTriggers)) {
                continue;
            }
            const auto binding = *reinterpret_cast<std::uintptr_t*>(bindingWithTriggers + kBindingWithTriggersBindingOffset);
            if (!plausiblePointer(binding)) {
                continue;
            }
            s_bindingsSeen.fetch_add(1, std::memory_order_relaxed);
            harvestBinding(binding, resolved.skeleton, allowedNodeNames, allowedNodeNameCount);
        }

        if (s_walkBindingIndex >= resolved.bindingCount) {
            s_walkDone = true;
            s_walksCompleted.fetch_add(1, std::memory_order_relaxed);
            ROCK_LOG_INFO(Weapon,
                "WeaponClipMotionHarvest: walked weapon {:08X} graph — {} binding(s), {} harvested, {} without part tracks (cumulative)",
                weaponFormId,
                resolved.bindingCount,
                s_bindingsHarvested.load(std::memory_order_relaxed),
                s_bindingsNoTargets.load(std::memory_order_relaxed));
            return StepResult::Completed;
        }
        return StepResult::Pending;
    }

    void resetWalk()
    {
        s_walkFormId = 0;
        s_walkGenerationKey = 0;
        s_walkBindingIndex = 0;
        s_walkBindingsData = 0;
        s_walkDone = false;
        s_bindingDetailLogs = 0;
    }

    std::uint32_t drainGroups(weapon_clip_stroke::AuthoredStrokeGroup* outGroups, std::uint32_t maxGroups)
    {
        if (!outGroups || maxGroups == 0) {
            return 0;
        }
        std::scoped_lock lock(s_queueMutex);
        const auto count = (std::min)(maxGroups, s_queueCount);
        for (std::uint32_t i = 0; i < count; ++i) {
            outGroups[i] = s_queue[i];
        }
        if (count < s_queueCount) {
            for (std::uint32_t i = count; i < s_queueCount; ++i) {
                s_queue[i - count] = s_queue[i];
            }
        }
        s_queueCount -= count;
        return count;
    }

    void clearPending()
    {
        std::scoped_lock lock(s_queueMutex);
        s_queueCount = 0;
    }

    Stats snapshotStats()
    {
        return Stats{
            .bindingsSeen = s_bindingsSeen.load(std::memory_order_relaxed),
            .bindingsHarvested = s_bindingsHarvested.load(std::memory_order_relaxed),
            .bindingsNoTargets = s_bindingsNoTargets.load(std::memory_order_relaxed),
            .groupsQueued = s_groupsQueued.load(std::memory_order_relaxed),
            .groupsDropped = s_groupsDropped.load(std::memory_order_relaxed),
            .skippedNonSpline = s_skippedNonSpline.load(std::memory_order_relaxed),
            .walksCompleted = s_walksCompleted.load(std::memory_order_relaxed),
            .bailAnimationPtr = s_bailAnimationPtr.load(std::memory_order_relaxed),
            .bailClipParams = s_bailClipParams.load(std::memory_order_relaxed),
            .bailTrackMap = s_bailTrackMap.load(std::memory_order_relaxed),
            .bailBoneCount = s_bailBoneCount.load(std::memory_order_relaxed),
            .bailSampler = s_bailSampler.load(std::memory_order_relaxed),
        };
    }
}
