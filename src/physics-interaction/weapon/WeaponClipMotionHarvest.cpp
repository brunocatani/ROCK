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

        // hkaAnimationBinding members — corrected 2026-07-04 against the
        // engine's own clip validator (hkbClipGenerator vtable slot 14,
        // 0x14192d7a0): +0x18 animation (duration read at anim+0x14 confirms),
        // +0x20/+0x28 transform track-to-bone shorts (the validator requires
        // this array to be identity when partitions are used), +0x40/+0x48 is
        // the PARTITION indices array — the earlier hook-era claim that the
        // track map lived there was a misidentification (weapon clips showing
        // "trackToBone=1" were reporting one partition).
        constexpr std::uintptr_t kBindingAnimationOffset = 0x18;
        constexpr std::uintptr_t kBindingTrackToBoneDataOffset = 0x20;
        constexpr std::uintptr_t kBindingTrackToBoneCountOffset = 0x28;
        // Partition-mapped clips (the weapon clips: empty track map, one
        // partition): tracks cover the bones of the listed skeleton
        // partitions in order. Engine validator 0x141a0c4e0: binding+0x40
        // partition indices (ushort) / +0x48 count, checked against the
        // skeleton's partition count at skeleton+0x80 (partitions hkArray
        // data at +0x78). Partition entry = { name*, int16 startBone,
        // int16 numBones }, 0x10 stride.
        constexpr std::uintptr_t kBindingPartitionIndicesOffset = 0x40;
        constexpr std::uintptr_t kBindingPartitionCountOffset = 0x48;
        constexpr std::uintptr_t kSkeletonPartitionsDataOffset = 0x78;
        constexpr std::uintptr_t kSkeletonPartitionsCountOffset = 0x80;
        constexpr std::uintptr_t kSkeletonPartitionStride = 0x10;
        constexpr std::uintptr_t kPartitionStartBoneOffset = 0x8;
        constexpr std::uintptr_t kPartitionNumBonesOffset = 0xA;

        // hkaSkeleton members (bones array of 0x10-byte hkaBone entries with
        // the name char* at +0; low pointer bit is an engine flag).
        // parentIndices (int16 per bone) sits at +0x18/+0x20, consistent
        // with the verified bones array at +0x28.
        constexpr std::uintptr_t kSkeletonParentIndicesOffset = 0x18;
        constexpr std::uintptr_t kSkeletonParentIndicesCountOffset = 0x20;
        constexpr std::uintptr_t kSkeletonBonesDataOffset = 0x28;
        constexpr std::uintptr_t kSkeletonBonesCountOffset = 0x30;
        constexpr std::uintptr_t kSkeletonBoneStride = 0x10;

        // hkaAnimation members.
        constexpr std::uintptr_t kAnimationDurationOffset = 0x14;
        constexpr std::uintptr_t kAnimationTrackCountOffset = 0x18;

        // hkaSplineCompressedAnimation internals (playback sampler
        // disassembly 0x141F71B70, block resolver 0x142051280): +0x3C
        // numBlocks, +0x40 maxFramesPerBlock (the sampler divides by
        // maxFramesPerBlock-1 unguarded), hkArrays m_blockOffsets (+0x58
        // data / +0x60 count), m_floatBlockOffsets (+0x68 / +0x70 — read
        // unconditionally by the sampler even with zero float tracks) and
        // m_data (+0x98 / +0xA0). The sampler guards none of these — it
        // assumes the clip is loaded because it only samples playing clips.
        // Binding-set STUBS carry token allocations here (m_data ~0x20
        // bytes), so residency is gated on the array SIZES: a real clip's
        // mask stream needs at least one byte per track.
        constexpr std::uintptr_t kSplineNumBlocksOffset = 0x3C;
        constexpr std::uintptr_t kSplineMaxFramesPerBlockOffset = 0x40;
        constexpr std::uintptr_t kSplineBlockOffsetsOffset = 0x58;
        constexpr std::uintptr_t kSplineBlockOffsetsCountOffset = 0x60;
        constexpr std::uintptr_t kSplineFloatBlockOffsetsOffset = 0x68;
        constexpr std::uintptr_t kSplineFloatBlockOffsetsCountOffset = 0x70;
        constexpr std::uintptr_t kSplineDataBaseOffset = 0x98;
        constexpr std::uintptr_t kSplineDataSizeOffset = 0xA0;
        // Full-pose decode buffer: covers the 95-bone player rigs with
        // slack; targets on higher track indices are skipped rather than
        // silently truncated.
        constexpr std::uint32_t kSampleBufferTracks = 160;

        /*
         * hkbClipGenerator (0x160 bytes; clone at 0x14192d950 allocates it):
         * clips can also stream in late, in which case the loaded binding
         * appears on the clip generator —
         * +0xD0 is Bethesda's loaded-binding wrapper (the engine's clip
         * validator, vtable slot 14 / 0x14192d7a0, reports "The animation
         * has not been loaded." when it is null) and wrapper+0x38 is the
         * loaded hkaAnimationBinding. Vtable slot 10 (0x14192d510, vtable
         * 0x2E0FB38 + 0x50) is the install override that swaps a prepared
         * binding in and notifies Bethesda's loading manager — the one
         * moment the payload is guaranteed resident, so the harvest hooks
         * that slot with an atomic pointer swap.
         */
        constexpr std::uintptr_t kClipGeneratorVtableModuleOffset = 0x2E0FB38;
        constexpr std::uintptr_t kClipGeneratorInstallSlotOffset = 0x50;
        constexpr std::uintptr_t kClipGeneratorLoadedBindingOffset = 0xD0;
        constexpr std::uintptr_t kLoadedBindingWrapperBindingOffset = 0x38;
        // hkaAnimation vtable slot 5 — the engine's own playback sampler
        // ("TtSampleSpline" profile marker, 0x141F71B70): sampleTracks(
        //   float time, uint32 transformTracks, hkQsTransform* out,
        //   uint32 floatTracks, float* floatsOut). Decodes tracks
        // 0..transformTracks-1 sequentially from the per-block mask streams
        // inside m_data. Slot 6 (per-track sampling through
        // m_transformOffsets) is DEAD code in FO4 — that array is never
        // populated, loaded or stub, which is what crashed the 2026-07-04
        // session and what made every clip look non-resident.
        constexpr std::size_t kSampleTracksSlot = 5;

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

        // sampleTracks(this, time, transformTrackCount, transformsOut,
        // floatTrackCount, floatsOut) — MSVC x64: time lands in XMM1.
        using SampleTracks_t = void (*)(void*, float, std::uint32_t, HkQsTransform*, std::uint32_t, float*);

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
        std::atomic<std::uint64_t> s_bailSplineData{ 0 };
        // Hook telemetry: every shim entry, entries passing the character
        // filter, and entries that reached sampling. fires==0 means the
        // engine never dispatched the hooked slot; fires>0 with matched==0
        // means the filter rejects the characters the engine passes.
        std::atomic<std::uint64_t> s_hookFires{ 0 };
        std::atomic<std::uint64_t> s_hookActivations{ 0 };
        std::atomic<std::uint32_t> s_hookUnmatchedLogs{ 0 };
        constexpr std::uint32_t kMaxHookUnmatchedLogs = 3;

        /*
         * Clip-activation hook targets. The hook fires on the engine's graph
         * update thread for EVERY actor's clip generators, so it harvests
         * only when the character it receives is one of the registered
         * candidate-graph characters, and node names are COPIED here so the
         * hook never touches scene-graph memory. Guarded by s_hookMutex
         * (writers: main thread, rare; reader: hook, rare — clip activations
         * are sparse).
         */
        constexpr std::size_t kMaxHookCharacters = 8;
        constexpr std::size_t kMaxHookNodeNames = 64;
        constexpr std::size_t kMaxHookNodeNameLength = 64;
        std::mutex s_hookMutex;
        std::array<std::uintptr_t, kMaxHookCharacters> s_hookCharacters{};
        std::uint32_t s_hookCharacterCount = 0;
        std::array<std::array<char, kMaxHookNodeNameLength>, kMaxHookNodeNames> s_hookNodeNames{};
        std::array<const char*, kMaxHookNodeNames> s_hookNodeNamePointers{};
        std::uint32_t s_hookNodeNameCount = 0;

        /*
         * Bindings terminally processed this weapon generation (harvested or
         * proven target-less): re-walk passes and repeat clip activations
         * skip them, so a part cannot be overwritten by a different clip on
         * every 2s pass. Bails (e.g. payload not loaded yet) are NOT marked
         * and retry naturally. Guarded by s_hookMutex; the *Locked helpers
         * assume the caller holds it (the hook already does).
         */
        std::array<std::uintptr_t, 128> s_processedBindings{};
        std::uint32_t s_processedBindingCount = 0;

        [[nodiscard]] bool bindingProcessedLocked(std::uintptr_t binding)
        {
            for (std::uint32_t i = 0; i < s_processedBindingCount; ++i) {
                if (s_processedBindings[i] == binding) {
                    return true;
                }
            }
            return false;
        }

        void markBindingProcessedLocked(std::uintptr_t binding)
        {
            if (s_processedBindingCount < s_processedBindings.size()) {
                s_processedBindings[s_processedBindingCount++] = binding;
            }
        }

        // Detailed bail dumps per walk (main thread; reset with the cursor)
        // so a failing binding is identifiable without flooding the log.
        constexpr std::uint32_t kMaxBindingDetailLogsPerWalk = 8;
        std::uint32_t s_bindingDetailLogs = 0;

        // Walk cursor (main thread only). No engine pointers are stored —
        // the chain is re-resolved from the holder on every step.
        std::uint32_t s_walkFormId = 0;
        std::uint64_t s_walkGenerationKey = 0;
        // Cursor spans every graph in the manager's array (weapon clips do
        // not live on the active graph).
        std::uint32_t s_walkGraphIndex = 0;
        std::int32_t s_walkBindingIndex = 0;
        // Data pointer of the binding set the cursor indexes into; a change
        // (graph swap / candidate switch) restarts the current graph's walk.
        std::uintptr_t s_walkBindingsData = 0;
        // Bindings visited across the whole walk; a pass that saw none keeps
        // the walk pending (sets may still be filling at equip).
        std::uint32_t s_walkBindingsVisited = 0;
        bool s_walkDone = false;
        // Pass bookkeeping for periodic re-walks (clip payloads stream in
        // only while playing): completion is logged for the first pass and
        // for any pass that harvested something new.
        std::uint32_t s_walkPassIndex = 0;
        std::uint64_t s_walkPassStartHarvested = 0;
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
         * Vanilla TEMPLATE rig bones ('WeaponBolt', 'WeaponMagazineChild3',
         * 'WeaponExtra1'…) carry generic motions that never match a mod
         * weapon's geometry — in-game calibration (2026-07-04) showed the
         * generic mag arc cannot map onto a straight-drop magazine under any
         * rotation. Authored data must come from the weapon's OWN clip
         * tracks, the same custom animation the reload plays (Bruno
         * directive), so template bones are excluded even when the scene
         * tree carries same-named template nodes.
         */
        bool isGenericTemplateBone(const char* name)
        {
            constexpr const char kPrefix[] = "Weapon";
            constexpr std::size_t kPrefixLength = sizeof(kPrefix) - 1;
            if (!name || std::strlen(name) < kPrefixLength) {
                return false;
            }
            return namesEqualNoCase(name, kPrefix, kPrefixLength);
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

        // Returns true when the binding reached a terminal outcome (harvested
        // or target-less) and should not be revisited this generation; false
        // on bails that may succeed later.
        bool harvestBinding(
            std::uintptr_t binding,
            std::uintptr_t skeleton,
            const char* const* allowedNodeNames,
            std::uint32_t allowedNodeNameCount)
        {
            const auto animation = *reinterpret_cast<std::uintptr_t*>(binding + kBindingAnimationOffset);
            if (!plausiblePointer(animation)) {
                s_bailAnimationPtr.fetch_add(1, std::memory_order_relaxed);
                logBindingBail("animation-ptr", binding, animation, 0.0f, 0, 0, 0);
                return false;
            }

            // Only spline-compressed clips are supported; the sampler slot is
            // dispatched virtually but the slot semantics were verified on
            // this class specifically. Other compressions are counted so the
            // gap is visible instead of silent.
            const auto vtable = *reinterpret_cast<std::uintptr_t*>(animation);
            if (vtable != RE::VTABLE::hkaSplineCompressedAnimation[0].address()) {
                s_skippedNonSpline.fetch_add(1, std::memory_order_relaxed);
                logBindingBail("non-spline", binding, animation, 0.0f, 0, 0, 0);
                return false;
            }

            const float duration = *reinterpret_cast<float*>(animation + kAnimationDurationOffset);
            const auto animationTrackCount = *reinterpret_cast<std::int32_t*>(animation + kAnimationTrackCountOffset);
            if (!std::isfinite(duration) || duration < kMinClipDurationSeconds || duration > kMaxClipDurationSeconds ||
                animationTrackCount <= 0 || animationTrackCount > kMaxPlausibleTrackCount) {
                s_bailClipParams.fetch_add(1, std::memory_order_relaxed);
                logBindingBail("clip-params", binding, animation, duration, animationTrackCount, 0, 0);
                return false;
            }

            // The engine sampler dereferences the spline payload unguarded;
            // binding-set stubs (token allocations, tiny m_data) and
            // anything else unloadable is skipped here. A real clip's mask
            // stream needs at least one byte per track per block.
            const auto splineNumBlocks = *reinterpret_cast<std::int32_t*>(animation + kSplineNumBlocksOffset);
            const auto splineMaxFramesPerBlock = *reinterpret_cast<std::int32_t*>(animation + kSplineMaxFramesPerBlockOffset);
            const auto splineBlockOffsets = *reinterpret_cast<std::uintptr_t*>(animation + kSplineBlockOffsetsOffset);
            const auto splineBlockOffsetsCount = *reinterpret_cast<std::int32_t*>(animation + kSplineBlockOffsetsCountOffset);
            const auto splineFloatBlockOffsets = *reinterpret_cast<std::uintptr_t*>(animation + kSplineFloatBlockOffsetsOffset);
            const auto splineFloatBlockOffsetsCount = *reinterpret_cast<std::int32_t*>(animation + kSplineFloatBlockOffsetsCountOffset);
            const auto splineDataBase = *reinterpret_cast<std::uintptr_t*>(animation + kSplineDataBaseOffset);
            const auto splineDataSize = *reinterpret_cast<std::int32_t*>(animation + kSplineDataSizeOffset);
            if (splineNumBlocks <= 0 || splineMaxFramesPerBlock < 2 ||
                !plausiblePointer(splineBlockOffsets) || splineBlockOffsetsCount < splineNumBlocks ||
                !plausiblePointer(splineFloatBlockOffsets) || splineFloatBlockOffsetsCount < splineNumBlocks ||
                !plausiblePointer(splineDataBase) || splineDataSize < animationTrackCount) {
                s_bailSplineData.fetch_add(1, std::memory_order_relaxed);
                if (s_bindingDetailLogs < kMaxBindingDetailLogsPerWalk) {
                    ++s_bindingDetailLogs;
                    ROCK_LOG_WARN(Weapon,
                        "WeaponClipMotionHarvest: binding bail [spline-data] anim={:#x} duration={} tracks={} blocks={} maxFrames={} blockOffsets={:#x}({}) floatBlockOffsets={:#x}({}) data={:#x}({})",
                        animation,
                        duration,
                        animationTrackCount,
                        splineNumBlocks,
                        splineMaxFramesPerBlock,
                        splineBlockOffsets,
                        splineBlockOffsetsCount,
                        splineFloatBlockOffsets,
                        splineFloatBlockOffsetsCount,
                        splineDataBase,
                        splineDataSize);
                }
                return false;
            }

            const auto trackToBoneData = *reinterpret_cast<std::uintptr_t*>(binding + kBindingTrackToBoneDataOffset);
            const auto trackToBoneCount = *reinterpret_cast<std::int32_t*>(binding + kBindingTrackToBoneCountOffset);
            const auto boneCount = *reinterpret_cast<std::int32_t*>(skeleton + kSkeletonBonesCountOffset);
            if (boneCount <= 0 || boneCount > kMaxPlausibleBoneCount) {
                s_bailBoneCount.fetch_add(1, std::memory_order_relaxed);
                logBindingBail("bone-count", binding, animation, duration, animationTrackCount, trackToBoneCount, boneCount);
                return false;
            }

            /*
             * Resolve which bone each transform track drives, in priority:
             * an explicit track-to-bone map; the flattened bone ranges of
             * the clip's skeleton partitions (weapon clips ship this way:
             * empty map, one partition — identity over the FULL rig would
             * scatter the tracks onto wrong bones); identity, valid only
             * when the clip covers the whole rig.
             */
            std::array<std::int16_t, kSampleBufferTracks> trackBones{};
            std::int32_t mappedTrackCount = 0;
            const auto partitionIndicesData = *reinterpret_cast<std::uintptr_t*>(binding + kBindingPartitionIndicesOffset);
            const auto partitionCount = *reinterpret_cast<std::int32_t*>(binding + kBindingPartitionCountOffset);
            if (trackToBoneCount > 0) {
                if (!plausiblePointer(trackToBoneData) || trackToBoneCount > kMaxPlausibleTrackCount) {
                    s_bailTrackMap.fetch_add(1, std::memory_order_relaxed);
                    logBindingBail("track-map", binding, animation, duration, animationTrackCount, trackToBoneCount, boneCount);
                    return false;
                }
                const auto* map = reinterpret_cast<const std::int16_t*>(trackToBoneData);
                const auto count = (std::min)(trackToBoneCount, static_cast<std::int32_t>(trackBones.size()));
                for (std::int32_t i = 0; i < count; ++i) {
                    trackBones[static_cast<std::size_t>(i)] = map[i];
                }
                mappedTrackCount = count;
            } else if (partitionCount > 0) {
                const auto skeletonPartitionsData = *reinterpret_cast<std::uintptr_t*>(skeleton + kSkeletonPartitionsDataOffset);
                const auto skeletonPartitionCount = *reinterpret_cast<std::int32_t*>(skeleton + kSkeletonPartitionsCountOffset);
                if (!plausiblePointer(partitionIndicesData) || partitionCount > kMaxPlausibleTrackCount ||
                    !plausiblePointer(skeletonPartitionsData) || skeletonPartitionCount <= 0 ||
                    skeletonPartitionCount > kMaxPlausibleBoneCount) {
                    s_bailTrackMap.fetch_add(1, std::memory_order_relaxed);
                    logBindingBail("partition-map", binding, animation, duration, animationTrackCount, partitionCount, boneCount);
                    return false;
                }
                const auto* partitionIndices = reinterpret_cast<const std::uint16_t*>(partitionIndicesData);
                for (std::int32_t p = 0; p < partitionCount && mappedTrackCount < static_cast<std::int32_t>(trackBones.size()); ++p) {
                    const auto partitionIndex = static_cast<std::int32_t>(partitionIndices[p]);
                    if (partitionIndex >= skeletonPartitionCount) {
                        continue;
                    }
                    const auto entry = skeletonPartitionsData +
                                       static_cast<std::uintptr_t>(partitionIndex) * kSkeletonPartitionStride;
                    const auto startBone = *reinterpret_cast<const std::int16_t*>(entry + kPartitionStartBoneOffset);
                    const auto numBones = *reinterpret_cast<const std::int16_t*>(entry + kPartitionNumBonesOffset);
                    if (startBone < 0 || numBones <= 0 || startBone + numBones > boneCount) {
                        continue;
                    }
                    for (std::int16_t b = 0; b < numBones && mappedTrackCount < static_cast<std::int32_t>(trackBones.size()); ++b) {
                        trackBones[static_cast<std::size_t>(mappedTrackCount++)] = static_cast<std::int16_t>(startBone + b);
                    }
                }
                if (mappedTrackCount == 0) {
                    s_bailTrackMap.fetch_add(1, std::memory_order_relaxed);
                    logBindingBail("partition-map", binding, animation, duration, animationTrackCount, partitionCount, boneCount);
                    return false;
                }
            } else if (animationTrackCount == boneCount) {
                const auto count = (std::min)(animationTrackCount, static_cast<std::int32_t>(trackBones.size()));
                for (std::int32_t i = 0; i < count; ++i) {
                    trackBones[static_cast<std::size_t>(i)] = static_cast<std::int16_t>(i);
                }
                mappedTrackCount = count;
            } else {
                // No map, no partitions, and the track count does not cover
                // the rig — identity would misattribute every track.
                s_bailTrackMap.fetch_add(1, std::memory_order_relaxed);
                logBindingBail("track-map", binding, animation, duration, animationTrackCount, trackToBoneCount, boneCount);
                return false;
            }

            // Collect the weapon-part tracks for this clip. Tracks beyond
            // the decode buffer cannot be sampled (slot 5 decodes
            // sequentially from track 0) and are skipped.
            std::array<std::int16_t, weapon_clip_stroke::kMaxTracksPerClip> trackIndices{};
            std::array<weapon_clip_stroke::TrackSamples, weapon_clip_stroke::kMaxTracksPerClip> tracks{};
            std::uint32_t targetCount = 0;
            std::int32_t maxTargetTrack = 0;
            const auto usableTrackCount = (std::min)(mappedTrackCount, animationTrackCount);
            for (std::int32_t track = 0; track < usableTrackCount && targetCount < tracks.size(); ++track) {
                const auto boneIndex = trackBones[static_cast<std::size_t>(track)];
                if (boneIndex < 0 || boneIndex >= boneCount) {
                    continue;
                }
                const char* name = skeletonBoneName(skeleton, boneIndex);
                if (isGenericTemplateBone(name) ||
                    !isHarvestTargetBone(name, allowedNodeNames, allowedNodeNameCount)) {
                    continue;
                }
                trackIndices[targetCount] = static_cast<std::int16_t>(track);
                maxTargetTrack = (std::max)(maxTargetTrack, track);
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
                return true;
            }

            const auto sampler = reinterpret_cast<SampleTracks_t>(
                reinterpret_cast<std::uintptr_t*>(vtable)[kSampleTracksSlot]);
            if (!sampler) {
                s_bailSampler.fetch_add(1, std::memory_order_relaxed);
                logBindingBail("sampler", binding, animation, duration, animationTrackCount, trackToBoneCount, boneCount);
                return false;
            }

            // Full-pose decode: slot 5 decodes tracks 0..decodeCount-1, then
            // the target tracks are picked out of the buffer.
            const auto decodeCount = static_cast<std::uint32_t>(maxTargetTrack) + 1;
            std::array<HkQsTransform, kSampleBufferTracks> sampled{};
            for (std::uint32_t step = 0; step < weapon_clip_stroke::kClipSampleCount; ++step) {
                const float time = duration * static_cast<float>(step) /
                                   static_cast<float>(weapon_clip_stroke::kClipSampleCount - 1);
                sampler(reinterpret_cast<void*>(animation), time, decodeCount, sampled.data(), 0, nullptr);
                for (std::uint32_t i = 0; i < targetCount; ++i) {
                    const auto& decoded = sampled[static_cast<std::size_t>(trackIndices[i])];
                    auto& pose = tracks[i].samples[step];
                    pose.translate = weapon_part_motion_path::Vec3{
                        decoded.translate[0],
                        decoded.translate[1],
                        decoded.translate[2],
                    };
                    // Havok quaternion order is (x, y, z, w).
                    pose.rotate = weapon_part_motion_path::quatNormalizeOrIdentity(weapon_part_motion_path::Quat{
                        decoded.rotate[3],
                        decoded.rotate[0],
                        decoded.rotate[1],
                        decoded.rotate[2],
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
                return false;
            }
            s_bindingsHarvested.fetch_add(1, std::memory_order_relaxed);

            // Frame diagnostics (capped): the targets' rig parents plus the
            // lead target's raw sample endpoints. Compared against the
            // learned-path endpoints for the same part, this pins down any
            // frame mismatch between rig-derived and scene-observed motion.
            if (s_bindingDetailLogs < kMaxBindingDetailLogsPerWalk) {
                ++s_bindingDetailLogs;
                const auto parentIndicesData = *reinterpret_cast<std::uintptr_t*>(skeleton + kSkeletonParentIndicesOffset);
                const auto parentIndicesCount = *reinterpret_cast<std::int32_t*>(skeleton + kSkeletonParentIndicesCountOffset);
                const auto parentNameOf = [&](std::uint32_t target) -> const char* {
                    const auto bone = trackBones[static_cast<std::size_t>(trackIndices[target])];
                    if (!plausiblePointer(parentIndicesData) || parentIndicesCount < boneCount ||
                        bone < 0 || bone >= boneCount) {
                        return "?";
                    }
                    const auto parent = reinterpret_cast<const std::int16_t*>(parentIndicesData)[bone];
                    if (parent < 0 || parent >= boneCount) {
                        return "<root>";
                    }
                    const char* name = skeletonBoneName(skeleton, parent);
                    return name ? name : "?";
                };
                const auto& rawStart = tracks[0].samples[0].translate;
                const auto& rawEnd = tracks[0].samples[weapon_clip_stroke::kClipSampleCount - 1].translate;
                ROCK_LOG_INFO(Weapon,
                    "WeaponClipMotionHarvest: harvested clip duration={:.2f} targets={} lead {}(parent={}) next [{}(parent={}) {}(parent={})] lead raw start=({:.2f},{:.2f},{:.2f}) end=({:.2f},{:.2f},{:.2f})",
                    duration,
                    targetCount,
                    tracks[0].boneName.data(),
                    parentNameOf(0),
                    targetCount > 1 ? tracks[1].boneName.data() : "",
                    targetCount > 1 ? parentNameOf(1) : "",
                    targetCount > 2 ? tracks[2].boneName.data() : "",
                    targetCount > 2 ? parentNameOf(2) : "",
                    rawStart.x,
                    rawStart.y,
                    rawStart.z,
                    rawEnd.x,
                    rawEnd.y,
                    rawEnd.z);
            }

            std::scoped_lock lock(s_queueMutex);
            for (std::uint32_t i = 0; i < groupCount; ++i) {
                if (s_queueCount >= s_queue.size()) {
                    s_groupsDropped.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                s_queue[s_queueCount++] = groups[i];
                s_groupsQueued.fetch_add(1, std::memory_order_relaxed);
            }
            return true;
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

        struct GraphArrayView
        {
            std::uintptr_t base{ 0 };
            std::uint32_t capacity{ 0 };
        };

        // Graph slots the walk may visit; bounded far above any real manager
        // (the player carries two graphs: first- and third-person). Weapon
        // clips do NOT live on the active graph — in-game diagnostics
        // (2026-07-04) showed the player's active graph is the 94-bone body
        // rig — so every entry is visited, gated per slot by the
        // BShkbAnimationGraph vtable (in-game confirmed module offset) so a
        // stale capacity slot degrades into a skip.
        constexpr std::uint32_t kMaxWalkGraphs = 8;
        constexpr std::uintptr_t kGraphVtableModuleOffset = 0x2E00A48;

        bool resolveGraphArray(std::uintptr_t manager, GraphArrayView& out)
        {
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
            out.base = graphsBase;
            out.capacity = (std::min)(capacityAndFlags & ~kGraphsInlineStorageFlag, kMaxWalkGraphs);
            return true;
        }

        // Vtable-gated graphs-array entry; 0 when the slot is empty, stale,
        // or not a BShkbAnimationGraph.
        [[nodiscard]] std::uintptr_t graphAtIndex(const GraphArrayView& graphs, std::uint32_t index)
        {
            if (index >= graphs.capacity) {
                return 0;
            }
            const auto graph = reinterpret_cast<const std::uintptr_t*>(graphs.base)[index];
            if (!plausiblePointer(graph) || objectVtableRel(graph) != kGraphVtableModuleOffset) {
                return 0;
            }
            return graph;
        }

        bool resolveGraphBindings(std::uintptr_t graph, ResolvedBindings& out)
        {
            s_lastResolveStage = "graph";
            if (!graph) {
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

        using ClipGeneratorInstallFn = void (*)(void*, void*);
        ClipGeneratorInstallFn s_originalClipInstall = nullptr;

        /*
         * Runs on the engine's graph-update thread right after the original
         * install override completed, i.e. while the loaded binding at
         * clipGenerator+0xD0 is guaranteed resident. Every read is
         * plausibility-gated and the sampler's spline gates still apply, so
         * an unexpected state degrades into a counted skip. The character
         * filter keeps NPC clip activations out.
         */
        void harvestFromClipGenerator(void* clipGeneratorRaw, void* contextRaw)
        {
            s_hookFires.fetch_add(1, std::memory_order_relaxed);
            const auto clipGenerator = reinterpret_cast<std::uintptr_t>(clipGeneratorRaw);
            const auto context = reinterpret_cast<std::uintptr_t>(contextRaw);
            if (!plausiblePointer(clipGenerator) || !plausiblePointer(context)) {
                return;
            }

            std::scoped_lock lock(s_hookMutex);
            /*
             * The second argument is a stack-built hkbContext, not the
             * character (in-game hook dumps 2026-07-04: stack-range
             * addresses, first qword not a module vtable). The owning
             * hkbCharacter is one of its leading pointer members, so the
             * first few qwords are identity-matched against the registered
             * candidate-graph characters; only the match is dereferenced.
             */
            std::uintptr_t character = 0;
            std::array<std::uintptr_t, 4> contextSlots{};
            for (std::uint32_t slot = 0; slot < contextSlots.size(); ++slot) {
                contextSlots[slot] = *reinterpret_cast<const std::uintptr_t*>(context + slot * sizeof(std::uintptr_t));
            }
            for (std::uint32_t slot = 0; slot < contextSlots.size() && character == 0; ++slot) {
                for (std::uint32_t i = 0; i < s_hookCharacterCount; ++i) {
                    if (s_hookCharacters[i] == contextSlots[slot]) {
                        character = contextSlots[slot];
                        break;
                    }
                }
            }
            if (character == 0 || s_hookNodeNameCount == 0) {
                if (s_hookCharacterCount > 0 &&
                    s_hookUnmatchedLogs.fetch_add(1, std::memory_order_relaxed) < kMaxHookUnmatchedLogs) {
                    ROCK_LOG_WARN(Weapon,
                        "WeaponClipMotionHarvest: hook fired, no registered character in context {:#x} slots=[{:#x}|{:#x}|{:#x}|{:#x}] clipGen={:#x}; registered[0]={:#x}",
                        context,
                        contextSlots[0],
                        contextSlots[1],
                        contextSlots[2],
                        contextSlots[3],
                        clipGenerator,
                        s_hookCharacters[0]);
                }
                return;
            }
            s_hookActivations.fetch_add(1, std::memory_order_relaxed);

            const auto wrapper = *reinterpret_cast<std::uintptr_t*>(clipGenerator + kClipGeneratorLoadedBindingOffset);
            if (!plausiblePointer(wrapper)) {
                return;
            }
            const auto binding = *reinterpret_cast<std::uintptr_t*>(wrapper + kLoadedBindingWrapperBindingOffset);
            if (!plausiblePointer(binding)) {
                return;
            }
            const auto setup = *reinterpret_cast<std::uintptr_t*>(character + kCharacterSetupOffset);
            if (!plausiblePointer(setup)) {
                return;
            }
            const auto skeleton = *reinterpret_cast<std::uintptr_t*>(setup + kSetupAnimationSkeletonOffset);
            if (!plausiblePointer(skeleton)) {
                return;
            }
            if (bindingProcessedLocked(binding)) {
                return;
            }
            s_bindingsSeen.fetch_add(1, std::memory_order_relaxed);
            if (harvestBinding(binding, skeleton, s_hookNodeNamePointers.data(), s_hookNodeNameCount)) {
                markBindingProcessedLocked(binding);
            }
        }

        void clipGeneratorInstallShim(void* clipGenerator, void* context)
        {
            if (s_originalClipInstall) {
                s_originalClipInstall(clipGenerator, context);
            }
            harvestFromClipGenerator(clipGenerator, context);
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

    void ensureClipActivationHookInstalled()
    {
        static bool s_installed = false;
        if (s_installed) {
            return;
        }
        s_installed = true;
        const auto slotAddress =
            REL::Module::get().base() + kClipGeneratorVtableModuleOffset + kClipGeneratorInstallSlotOffset;
        s_originalClipInstall = reinterpret_cast<ClipGeneratorInstallFn>(*reinterpret_cast<std::uintptr_t*>(slotAddress));
        // 8-byte aligned pointer store is atomic on x64, so concurrent graph
        // updates dispatching through the slot stay safe during the swap.
        REL::safe_write(slotAddress, reinterpret_cast<std::uintptr_t>(&clipGeneratorInstallShim));
        ROCK_LOG_INFO(Weapon,
            "WeaponClipMotionHarvest: clip-activation hook installed (vtable slot +{:#x}, original +{:#x})",
            kClipGeneratorVtableModuleOffset + kClipGeneratorInstallSlotOffset,
            moduleRelative(reinterpret_cast<std::uintptr_t>(s_originalClipInstall)));
    }

    void setClipActivationTargets(
        const void* const* graphManagers,
        std::uint32_t managerCount,
        const char* const* allowedNodeNames,
        std::uint32_t allowedNodeNameCount)
    {
        std::scoped_lock lock(s_hookMutex);
        s_hookCharacterCount = 0;
        for (std::uint32_t m = 0; m < managerCount; ++m) {
            GraphArrayView graphs{};
            if (!resolveGraphArray(reinterpret_cast<std::uintptr_t>(graphManagers[m]), graphs)) {
                continue;
            }
            for (std::uint32_t i = 0; i < graphs.capacity && s_hookCharacterCount < s_hookCharacters.size(); ++i) {
                const auto graph = graphAtIndex(graphs, i);
                if (graph != 0) {
                    s_hookCharacters[s_hookCharacterCount++] = graph + kGraphCharacterOffset;
                }
            }
        }
        s_hookNodeNameCount = 0;
        for (std::uint32_t i = 0; i < allowedNodeNameCount && s_hookNodeNameCount < s_hookNodeNames.size(); ++i) {
            const char* name = allowedNodeNames[i];
            if (!name || name[0] == '\0') {
                continue;
            }
            auto& storage = s_hookNodeNames[s_hookNodeNameCount];
            std::size_t length = 0;
            while (length < storage.size() - 1 && name[length] != '\0') {
                storage[length] = name[length];
                ++length;
            }
            storage[length] = '\0';
            s_hookNodeNamePointers[s_hookNodeNameCount] = storage.data();
            ++s_hookNodeNameCount;
        }
    }

    void clearClipActivationTargets()
    {
        std::scoped_lock lock(s_hookMutex);
        s_hookCharacterCount = 0;
        s_hookNodeNameCount = 0;
    }

    bool probeBindings(const void* graphManager)
    {
        GraphArrayView graphs{};
        if (!resolveGraphArray(reinterpret_cast<std::uintptr_t>(graphManager), graphs)) {
            return false;
        }
        for (std::uint32_t i = 0; i < graphs.capacity; ++i) {
            ResolvedBindings resolved{};
            if (resolveGraphBindings(graphAtIndex(graphs, i), resolved)) {
                return true;
            }
        }
        return false;
    }

    void logResolveDiagnostics(const void* graphManager, const char* label)
    {
        const auto manager = reinterpret_cast<std::uintptr_t>(graphManager);
        std::uint32_t capacityAndFlags = 0;
        std::uint32_t activeGraphIndex = 0;
        if (plausiblePointer(manager)) {
            capacityAndFlags = *reinterpret_cast<std::uint32_t*>(manager + kManagerGraphsCapacityOffset);
            activeGraphIndex = *reinterpret_cast<std::uint32_t*>(manager + kManagerActiveGraphOffset);
        }
        GraphArrayView graphs{};
        const bool haveArray = resolveGraphArray(manager, graphs);
        ROCK_LOG_WARN(Weapon,
            "WeaponClipMotionHarvest diagnostics [{}]: mgr={:#x}(vt+{:#x}) graphsFlags={:#010x} activeIdx={} slots={}",
            label ? label : "?",
            manager,
            objectVtableRel(manager),
            capacityAndFlags,
            activeGraphIndex,
            graphs.capacity);
        if (!haveArray) {
            return;
        }

        for (std::uint32_t index = 0; index < graphs.capacity; ++index) {
            const auto rawGraph = reinterpret_cast<const std::uintptr_t*>(graphs.base)[index];
            // Deep reads only through the vtable gate; the raw slot value and
            // its vtable are still printed so a rejected slot is explainable.
            const auto graph = graphAtIndex(graphs, index);
            std::uintptr_t character = 0;
            std::uintptr_t setup = 0;
            std::uintptr_t skeleton = 0;
            std::uintptr_t bindingSet = 0;
            std::uintptr_t bindingsData = 0;
            std::int32_t bindingCount = -1;
            std::int32_t boneCount = -1;
            const char* bindingSetSource = "none";
            std::array<const char*, 3> firstBoneNames{ "", "", "" };
            const char* lastBoneName = "";
            if (graph != 0) {
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
            }
            ROCK_LOG_WARN(Weapon,
                "WeaponClipMotionHarvest diagnostics [{}] graph[{}]{}: graph={:#x}(vt+{:#x}) charVt=+{:#x} "
                "setup={:#x}(vt+{:#x}) skel={:#x}(vt+{:#x}) bones={} first=[{}|{}|{}] last=[{}] "
                "setSrc={} set={:#x}(vt+{:#x}) data={:#x} count={}",
                label ? label : "?",
                index,
                index == activeGraphIndex ? "*" : "",
                rawGraph,
                objectVtableRel(rawGraph),
                graph != 0 ? objectVtableRel(character) : 0,
                setup,
                objectVtableRel(setup),
                skeleton,
                objectVtableRel(skeleton),
                boneCount,
                firstBoneNames[0],
                firstBoneNames[1],
                firstBoneNames[2],
                lastBoneName,
                bindingSetSource,
                bindingSet,
                objectVtableRel(bindingSet),
                bindingsData,
                bindingCount);
        }
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
            s_walkGraphIndex = 0;
            s_walkBindingIndex = 0;
            s_walkBindingsData = 0;
            s_walkBindingsVisited = 0;
            s_walkDone = false;
            s_bindingDetailLogs = 0;
            s_walkPassIndex = 0;
            s_walkPassStartHarvested = s_bindingsHarvested.load(std::memory_order_relaxed);
            std::scoped_lock lock(s_hookMutex);
            s_processedBindingCount = 0;
        }
        if (s_walkDone) {
            return StepResult::Completed;
        }

        GraphArrayView graphs{};
        if (!resolveGraphArray(reinterpret_cast<std::uintptr_t>(graphManager), graphs)) {
            // Manager not readable yet; the caller retries next frame.
            return StepResult::Pending;
        }

        std::int32_t processed = 0;
        while (processed < kBindingsPerStep) {
            if (s_walkGraphIndex >= graphs.capacity) {
                if (s_walkBindingsVisited == 0) {
                    // Every graph was empty/unresolvable this pass; the sets
                    // may still be filling at equip — retry from the top
                    // until the caller's attempt budget runs out.
                    s_walkGraphIndex = 0;
                    s_walkBindingIndex = 0;
                    s_walkBindingsData = 0;
                    return StepResult::Pending;
                }
                s_walkDone = true;
                s_walksCompleted.fetch_add(1, std::memory_order_relaxed);
                const auto harvested = s_bindingsHarvested.load(std::memory_order_relaxed);
                // Periodic re-walk passes only log when something new landed.
                if (s_walkPassIndex == 0 || harvested != s_walkPassStartHarvested) {
                    ROCK_LOG_INFO(Weapon,
                        "WeaponClipMotionHarvest: walked weapon {:08X} pass {} — {} graph slot(s), {} binding(s) visited, {} harvested, {} without part tracks (cumulative)",
                        weaponFormId,
                        s_walkPassIndex,
                        graphs.capacity,
                        s_walkBindingsVisited,
                        harvested,
                        s_bindingsNoTargets.load(std::memory_order_relaxed));
                }
                ++s_walkPassIndex;
                return StepResult::Completed;
            }

            ResolvedBindings resolved{};
            const auto graph = graphAtIndex(graphs, s_walkGraphIndex);
            if (!graph || !resolveGraphBindings(graph, resolved)) {
                // Empty slot, dummy graph, or empty binding set: next graph.
                ++s_walkGraphIndex;
                s_walkBindingIndex = 0;
                s_walkBindingsData = 0;
                continue;
            }
            if (resolved.bindingsData != s_walkBindingsData) {
                // Different binding set than the cursor was walking (rebuilt
                // at equip); indices are not comparable across sets, restart
                // this graph.
                s_walkBindingsData = resolved.bindingsData;
                s_walkBindingIndex = 0;
            }
            if (s_walkBindingIndex >= resolved.bindingCount) {
                ++s_walkGraphIndex;
                s_walkBindingIndex = 0;
                s_walkBindingsData = 0;
                continue;
            }

            const auto* bindings = reinterpret_cast<const std::uintptr_t*>(resolved.bindingsData);
            const auto bindingWithTriggers = bindings[s_walkBindingIndex++];
            ++processed;
            ++s_walkBindingsVisited;
            if (!plausiblePointer(bindingWithTriggers)) {
                continue;
            }
            const auto binding = *reinterpret_cast<std::uintptr_t*>(bindingWithTriggers + kBindingWithTriggersBindingOffset);
            if (!plausiblePointer(binding)) {
                continue;
            }
            {
                std::scoped_lock lock(s_hookMutex);
                if (bindingProcessedLocked(binding)) {
                    continue;
                }
            }
            s_bindingsSeen.fetch_add(1, std::memory_order_relaxed);
            if (harvestBinding(binding, resolved.skeleton, allowedNodeNames, allowedNodeNameCount)) {
                std::scoped_lock lock(s_hookMutex);
                markBindingProcessedLocked(binding);
            }
        }
        return StepResult::Pending;
    }

    void resetWalk()
    {
        s_walkFormId = 0;
        s_walkGenerationKey = 0;
        s_walkGraphIndex = 0;
        s_walkBindingIndex = 0;
        s_walkBindingsData = 0;
        s_walkBindingsVisited = 0;
        s_walkDone = false;
        s_bindingDetailLogs = 0;
        s_walkPassIndex = 0;
        s_walkPassStartHarvested = s_bindingsHarvested.load(std::memory_order_relaxed);
        std::scoped_lock lock(s_hookMutex);
        s_processedBindingCount = 0;
    }

    void restartWalkPass()
    {
        s_walkGraphIndex = 0;
        s_walkBindingIndex = 0;
        s_walkBindingsData = 0;
        s_walkBindingsVisited = 0;
        s_walkDone = false;
        s_walkPassStartHarvested = s_bindingsHarvested.load(std::memory_order_relaxed);
        // The detail-log budget is intentionally NOT reset: bail dumps stay
        // capped per weapon generation so periodic re-walks cannot spam.
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
            .bailSplineData = s_bailSplineData.load(std::memory_order_relaxed),
            .hookFires = s_hookFires.load(std::memory_order_relaxed),
            .hookActivations = s_hookActivations.load(std::memory_order_relaxed),
        };
    }
}
