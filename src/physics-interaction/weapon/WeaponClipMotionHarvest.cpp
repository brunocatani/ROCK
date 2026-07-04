#include "physics-interaction/weapon/WeaponClipMotionHarvest.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/EntryTrampolineHook.h"

#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>

namespace rock::weapon_clip_motion_harvest
{
    namespace
    {
        /*
         * All offsets below were verified against the FO4VR binary on
         * 2026-07-03 (Ghidra + VR address library cross-check); see
         * docs/research/2026-07-03-baked-animation-motion-extraction.md.
         */

        /*
         * Bethesda's runtime animation-binding-set builder — VR 0x141783f90
         * (flat FO4 0x141703590, exact instruction match in the VR address
         * library diff). Runs on every behavior graph / weapon subgraph
         * instantiation (BehaviorGraphSwapInstanceInitializationFunctor
         * path): it iterates the graph's clip generators, loads each
         * animation via BSResourceAssetLoader, and appends an
         * hkbAnimationBindingWithTriggers per clip to the owner's
         * hkbAnimationBindingSet. The Havok SDK utility
         * hkbBehaviorLoadingUtils::assignAnimationBinding (0x14193a9c0) is
         * NOT on this path — a season of bindingsSeen=0 proved it dead at
         * runtime. Entry prologue is 15 position-independent bytes.
         */
        constexpr std::uintptr_t kFuncBuildAnimationBindingSet = 0x1783f90;
        constexpr std::array<std::uint8_t, 15> kBuildAnimationBindingSetExpectedPrefix{
            0x48, 0x89, 0x5C, 0x24, 0x08,
            0x48, 0x89, 0x6C, 0x24, 0x10,
            0x48, 0x89, 0x74, 0x24, 0x18,
        };

        /*
         * Binding-set owner (the builder's second argument) — decompile-
         * verified layout: +0x78 hkbCharacterSetup*, +0x90 binding-set
         * override (used by the engine's own getter before falling back to
         * the setup's set). hkbCharacterSetup: +0x20 m_animationSkeleton
         * (hkaSkeleton*, the skeleton the clip tracks map to), +0x38
         * m_animationBindingSet. hkbAnimationBindingSet: +0x10 bindings
         * data (hkbAnimationBindingWithTriggers*[]), +0x18 int count.
         * hkbAnimationBindingWithTriggers: +0x8 hkaAnimationBinding*.
         */
        constexpr std::uintptr_t kOwnerCharacterSetupOffset = 0x78;
        constexpr std::uintptr_t kOwnerBindingSetOverrideOffset = 0x90;
        constexpr std::uintptr_t kSetupAnimationSkeletonOffset = 0x20;
        constexpr std::uintptr_t kSetupBindingSetOffset = 0x38;
        constexpr std::uintptr_t kBindingSetDataOffset = 0x10;
        constexpr std::uintptr_t kBindingSetCountOffset = 0x18;
        constexpr std::uintptr_t kBindingWithTriggersBindingOffset = 0x8;
        constexpr std::int32_t kMaxPlausibleBindingCount = 4096;

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
        constexpr std::size_t kQueueCapacity = 32;

        // hkQsTransform: vec4 translate, quaternion (x,y,z,w), vec4 scale.
        struct HkQsTransform
        {
            float translate[4];
            float rotate[4];
            float scale[4];
        };
        static_assert(sizeof(HkQsTransform) == 48);

        // int32 return (a count read from the loaded graph data); five
        // arguments, the fifth on the stack.
        using BuildAnimationBindingSet_t = std::int32_t (*)(void*, void*, void*, void*, void*);
        using SampleIndividualTransformTracks_t = void (*)(void*, float, const std::int16_t*, std::uint32_t, HkQsTransform*);

        BuildAnimationBindingSet_t s_originalBuildAnimationBindingSet = nullptr;
        std::atomic<bool> s_hookInstalled{ false };

        std::mutex s_queueMutex;
        std::array<weapon_clip_stroke::AuthoredStrokeGroup, kQueueCapacity> s_queue{};
        std::uint32_t s_queueCount = 0;

        std::atomic<std::uint64_t> s_bindingsSeen{ 0 };
        std::atomic<std::uint64_t> s_bindingsHarvested{ 0 };
        std::atomic<std::uint64_t> s_bindingsNoTargets{ 0 };
        std::atomic<std::uint64_t> s_groupsQueued{ 0 };
        std::atomic<std::uint64_t> s_groupsDropped{ 0 };
        std::atomic<std::uint64_t> s_skippedNonSpline{ 0 };

        // Bone-name dump budget for zero-target bindings (see header). Small
        // bindings only: partial/weapon clips have few tracks, so a low track
        // cap keeps full-body character clips out of the dump.
        constexpr std::int32_t kMaxNoTargetDumpTracks = 24;
        std::atomic<std::uint32_t> s_noTargetDumpBudget{ 16 };

        const char* skeletonBoneName(std::uintptr_t skeleton, std::int32_t boneIndex)
        {
            const auto bonesData = *reinterpret_cast<std::uintptr_t*>(skeleton + kSkeletonBonesDataOffset);
            if (bonesData == 0) {
                return nullptr;
            }
            const auto entry = bonesData + static_cast<std::uintptr_t>(boneIndex) * kSkeletonBoneStride;
            const auto namePtr = *reinterpret_cast<std::uintptr_t*>(entry) & ~static_cast<std::uintptr_t>(1);
            return reinterpret_cast<const char*>(namePtr);
        }

        bool isHarvestTargetBoneName(const char* name)
        {
            if (!name || std::strncmp(name, "Weapon", 6) != 0) {
                return false;
            }
            // The bare weapon attach bones move with recoil/aim, not as parts;
            // driving them would steer the whole weapon.
            if (std::strcmp(name, "Weapon") == 0 || std::strcmp(name, "WeaponLeft") == 0) {
                return false;
            }
            return true;
        }

        void dumpNoTargetBindingNames(
            std::uintptr_t skeleton,
            const std::int16_t* trackToBone,
            std::int32_t usableTrackCount,
            std::int32_t boneCount,
            float duration)
        {
            if (usableTrackCount > kMaxNoTargetDumpTracks) {
                return;
            }
            auto budget = s_noTargetDumpBudget.load(std::memory_order_relaxed);
            if (budget == 0 || !s_noTargetDumpBudget.compare_exchange_strong(budget, budget - 1, std::memory_order_relaxed)) {
                return;
            }
            std::array<char, 512> names{};
            std::size_t length = 0;
            for (std::int32_t track = 0; track < usableTrackCount; ++track) {
                const auto boneIndex = trackToBone[track];
                if (boneIndex < 0 || boneIndex >= boneCount) {
                    continue;
                }
                const char* name = skeletonBoneName(skeleton, boneIndex);
                if (!name) {
                    continue;
                }
                if (length > 0 && length < names.size() - 1) {
                    names[length++] = ',';
                }
                while (*name != '\0' && length < names.size() - 1) {
                    names[length++] = *name++;
                }
                if (length >= names.size() - 1) {
                    break;
                }
            }
            ROCK_LOG_INFO(
                Weapon,
                "ClipHarvest: binding dur={:.2f}s tracks={} has no Weapon* tracks; bones=[{}]",
                duration,
                usableTrackCount,
                names.data());
        }

        void harvestBinding(std::uintptr_t binding, std::uintptr_t skeleton)
        {
            const auto animation = *reinterpret_cast<std::uintptr_t*>(binding + kBindingAnimationOffset);
            if (animation == 0) {
                return;
            }

            // Only spline-compressed clips are supported; the sampler slot is
            // dispatched virtually but the slot semantics were verified on
            // this class specifically. Other compressions are counted so the
            // gap is visible instead of silent.
            const auto vtable = *reinterpret_cast<std::uintptr_t*>(animation);
            if (vtable != RE::VTABLE::hkaSplineCompressedAnimation[0].address()) {
                s_skippedNonSpline.fetch_add(1, std::memory_order_relaxed);
                return;
            }

            const float duration = *reinterpret_cast<float*>(animation + kAnimationDurationOffset);
            const auto animationTrackCount = *reinterpret_cast<std::int32_t*>(animation + kAnimationTrackCountOffset);
            if (!std::isfinite(duration) || duration < kMinClipDurationSeconds || duration > kMaxClipDurationSeconds ||
                animationTrackCount <= 0 || animationTrackCount > kMaxPlausibleTrackCount) {
                return;
            }

            const auto trackToBoneData = *reinterpret_cast<std::uintptr_t*>(binding + kBindingTrackToBoneDataOffset);
            const auto trackToBoneCount = *reinterpret_cast<std::int32_t*>(binding + kBindingTrackToBoneCountOffset);
            if (trackToBoneData == 0 || trackToBoneCount <= 0 || trackToBoneCount > kMaxPlausibleTrackCount) {
                return;
            }
            const auto boneCount = *reinterpret_cast<std::int32_t*>(skeleton + kSkeletonBonesCountOffset);
            if (boneCount <= 0 || boneCount > kMaxPlausibleBoneCount) {
                return;
            }
            const auto* trackToBone = reinterpret_cast<const std::int16_t*>(trackToBoneData);

            // Collect Weapon* part tracks for this clip.
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
                if (!isHarvestTargetBoneName(name)) {
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
                dumpNoTargetBindingNames(skeleton, trackToBone, usableTrackCount, boneCount, duration);
                return;
            }

            const auto sampler = reinterpret_cast<SampleIndividualTransformTracks_t>(
                reinterpret_cast<std::uintptr_t*>(vtable)[kSampleIndividualTransformTracksSlot]);
            if (!sampler) {
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
         * Runs after the engine finishes building a graph's binding set:
         * every clip's binding is fully populated, and the set plus the
         * animation skeleton stay alive on the owner for the graph's
         * lifetime, so a synchronous in-hook walk touches only live data on
         * the build thread. Fails closed on any null/implausible field.
         */
        std::int32_t hookedBuildAnimationBindingSet(void* a, void* owner, void* c, void* d, void* e)
        {
            const std::int32_t result = s_originalBuildAnimationBindingSet
                ? s_originalBuildAnimationBindingSet(a, owner, c, d, e)
                : 0;

            if (!owner || !g_rockConfig.rockBoltDriveSandboxEnabled) {
                return result;
            }
            const auto ownerAddress = reinterpret_cast<std::uintptr_t>(owner);
            const auto setup = *reinterpret_cast<std::uintptr_t*>(ownerAddress + kOwnerCharacterSetupOffset);
            if (setup == 0) {
                return result;
            }
            const auto skeleton = *reinterpret_cast<std::uintptr_t*>(setup + kSetupAnimationSkeletonOffset);
            if (skeleton == 0) {
                return result;
            }
            auto bindingSet = *reinterpret_cast<std::uintptr_t*>(ownerAddress + kOwnerBindingSetOverrideOffset);
            if (bindingSet == 0) {
                bindingSet = *reinterpret_cast<std::uintptr_t*>(setup + kSetupBindingSetOffset);
            }
            if (bindingSet == 0) {
                return result;
            }
            const auto bindingsData = *reinterpret_cast<std::uintptr_t*>(bindingSet + kBindingSetDataOffset);
            const auto bindingCount = *reinterpret_cast<std::int32_t*>(bindingSet + kBindingSetCountOffset);
            if (bindingsData == 0 || bindingCount <= 0 || bindingCount > kMaxPlausibleBindingCount) {
                return result;
            }
            for (std::int32_t i = 0; i < bindingCount; ++i) {
                const auto bindingWithTriggers = reinterpret_cast<const std::uintptr_t*>(bindingsData)[i];
                if (bindingWithTriggers == 0) {
                    continue;
                }
                const auto binding = *reinterpret_cast<std::uintptr_t*>(bindingWithTriggers + kBindingWithTriggersBindingOffset);
                if (binding == 0) {
                    continue;
                }
                s_bindingsSeen.fetch_add(1, std::memory_order_relaxed);
                harvestBinding(binding, skeleton);
            }
            return result;
        }
    }

    bool installHook()
    {
        if (s_hookInstalled.load(std::memory_order_acquire)) {
            return true;
        }

        void* original = reinterpret_cast<void*>(s_originalBuildAnimationBindingSet);
        const bool installed = entry_trampoline_hook::install(
            "animation binding-set build clip-motion harvest",
            kFuncBuildAnimationBindingSet,
            kBuildAnimationBindingSetExpectedPrefix.data(),
            kBuildAnimationBindingSetExpectedPrefix.size(),
            reinterpret_cast<void*>(&hookedBuildAnimationBindingSet),
            original);
        s_originalBuildAnimationBindingSet = reinterpret_cast<BuildAnimationBindingSet_t>(original);
        s_hookInstalled.store(installed && s_originalBuildAnimationBindingSet != nullptr, std::memory_order_release);
        return s_hookInstalled.load(std::memory_order_acquire);
    }

    bool hookInstalled()
    {
        return s_hookInstalled.load(std::memory_order_acquire);
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

    void armNoTargetNameDumps(std::uint32_t budget)
    {
        s_noTargetDumpBudget.store(budget, std::memory_order_relaxed);
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
        };
    }
}
