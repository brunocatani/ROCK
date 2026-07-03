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

        // hkbBehaviorLoadingUtils::assignAnimationBinding — REL::ID 128842,
        // VR 0x14193a9c0. Entry prologue is 15 position-independent bytes.
        constexpr std::uintptr_t kFuncAssignAnimationBinding = 0x193a9c0;
        constexpr std::array<std::uint8_t, 15> kAssignAnimationBindingExpectedPrefix{
            0x48, 0x89, 0x5C, 0x24, 0x08,
            0x48, 0x89, 0x6C, 0x24, 0x10,
            0x48, 0x89, 0x74, 0x24, 0x18,
        };

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

        using AssignAnimationBinding_t = bool (*)(void*, void*, void*, void*);
        using SampleIndividualTransformTracks_t = void (*)(void*, float, const std::int16_t*, std::uint32_t, HkQsTransform*);

        AssignAnimationBinding_t s_originalAssignAnimationBinding = nullptr;
        std::atomic<bool> s_hookInstalled{ false };

        std::mutex s_queueMutex;
        std::array<weapon_clip_stroke::AuthoredStrokeGroup, kQueueCapacity> s_queue{};
        std::uint32_t s_queueCount = 0;

        std::atomic<std::uint64_t> s_bindingsSeen{ 0 };
        std::atomic<std::uint64_t> s_bindingsHarvested{ 0 };
        std::atomic<std::uint64_t> s_groupsQueued{ 0 };
        std::atomic<std::uint64_t> s_groupsDropped{ 0 };
        std::atomic<std::uint64_t> s_skippedNonSpline{ 0 };

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

        bool hookedAssignAnimationBinding(void* bindingWithTriggers, void* binding, void* stringMap, void* skeleton)
        {
            const bool result = s_originalAssignAnimationBinding
                ? s_originalAssignAnimationBinding(bindingWithTriggers, binding, stringMap, skeleton)
                : false;

            if (result && binding && skeleton && g_rockConfig.rockBoltDriveSandboxEnabled) {
                s_bindingsSeen.fetch_add(1, std::memory_order_relaxed);
                harvestBinding(reinterpret_cast<std::uintptr_t>(binding), reinterpret_cast<std::uintptr_t>(skeleton));
            }
            return result;
        }
    }

    bool installHook()
    {
        if (s_hookInstalled.load(std::memory_order_acquire)) {
            return true;
        }

        void* original = reinterpret_cast<void*>(s_originalAssignAnimationBinding);
        const bool installed = entry_trampoline_hook::install(
            "hkbBehaviorLoadingUtils::assignAnimationBinding clip-motion harvest",
            kFuncAssignAnimationBinding,
            kAssignAnimationBindingExpectedPrefix.data(),
            kAssignAnimationBindingExpectedPrefix.size(),
            reinterpret_cast<void*>(&hookedAssignAnimationBinding),
            original);
        s_originalAssignAnimationBinding = reinterpret_cast<AssignAnimationBinding_t>(original);
        s_hookInstalled.store(installed && s_originalAssignAnimationBinding != nullptr, std::memory_order_release);
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

    Stats snapshotStats()
    {
        return Stats{
            .bindingsSeen = s_bindingsSeen.load(std::memory_order_relaxed),
            .bindingsHarvested = s_bindingsHarvested.load(std::memory_order_relaxed),
            .groupsQueued = s_groupsQueued.load(std::memory_order_relaxed),
            .groupsDropped = s_groupsDropped.load(std::memory_order_relaxed),
            .skippedNonSpline = s_skippedNonSpline.load(std::memory_order_relaxed),
        };
    }
}
