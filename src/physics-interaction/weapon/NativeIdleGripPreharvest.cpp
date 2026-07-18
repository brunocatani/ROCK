#include "physics-interaction/weapon/NativeIdleGripPreharvest.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/NativeIdleGripPreharvestPolicy.h"

#include "f4vr/F4VRUtils.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/BSAnimationGraph.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/BSFixedString.h"
#include "RE/Bethesda/BSStringT.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"

#include <Windows.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string_view>
#include <utility>

namespace rock::native_idle_grip_preharvest
{
    namespace
    {
        constexpr std::uintptr_t kSimpleAnimationGraphManagerHolderCtor = 0x0811F10;
        constexpr std::uintptr_t kSimpleAnimationGraphManagerHolderDtor = 0x0811F50;
        constexpr std::uintptr_t kCreateBackgroundSimpleManager = 0x0811FE0;
        constexpr std::uintptr_t kIsAnimationLoadingComplete = 0x08122C0;
        constexpr std::uintptr_t kRequestAnimationSubGraph = 0x10162B0;
        constexpr std::uintptr_t kIsAnimationSubGraphLoaded = 0x07F4320;
        constexpr std::uintptr_t kReleaseAnimationSubGraph = 0x07F43C0;
        constexpr std::uintptr_t kAddItemToTargetKeywords = 0x0EDA2F0;
        constexpr std::uintptr_t kGetClipGeneratorBinding = 0x1774800;
        constexpr std::uintptr_t kGetAnimationFilesForSubgraph = 0x1769140;
        constexpr std::uintptr_t kFindBoneWithName = 0x190A580;
        constexpr std::uintptr_t kBehaviorGraphSwapSingleton = 0x5AB9200;
        constexpr std::uintptr_t kAnimationFileLookupSingleton = 0x5B64318;

        constexpr std::size_t kSimpleAnimationGraphManagerHolderSize = 0x18;
        static_assert(kSimpleAnimationGraphManagerHolderSize == sizeof(RE::SimpleAnimationGraphManagerHolder));
        constexpr std::ptrdiff_t kGraphSkeletonOwnerOffset = 0x240;
        constexpr std::ptrdiff_t kSkeletonFromOwnerOffset = 0x20;
        constexpr std::ptrdiff_t kSkeletonParentIndicesOffset = 0x18;
        constexpr std::ptrdiff_t kSkeletonParentCountOffset = 0x20;
        constexpr std::ptrdiff_t kSkeletonBoneCountOffset = 0x30;
        constexpr std::ptrdiff_t kBindingFromBindingWithTriggersOffset = 0x10;
        constexpr std::ptrdiff_t kAnimationFromBindingOffset = 0x18;
        constexpr std::ptrdiff_t kTrackToBoneMappingOffset = 0x20;
        constexpr std::ptrdiff_t kTrackToBoneMappingCountOffset = 0x28;
        constexpr std::ptrdiff_t kAnimationTransformTrackCountOffset = 0x14;

        constexpr std::size_t kMaxBonesAndTracks = 768;
        constexpr std::size_t kFailureCapacity = 64;
        constexpr ULONGLONG kFailureRetryDelayMilliseconds = 30000;
        constexpr ULONGLONG kLongLoadLogDelayMilliseconds = 5000;
        constexpr std::uint64_t kPreharvestCaptureSequenceDomain = 1ull << 63;
        constexpr std::int32_t kWeaponAnimationRole = 1;
        constexpr std::int32_t kIoTaskPriority = 3;

        struct alignas(16) HkQsTransform
        {
            float translation[4]{};
            float rotation[4]{};
            float scale[4]{};
        };
        static_assert(sizeof(HkQsTransform) == 0x30);

        using GraphHolderCtorFn = void* (*)(void*);
        using GraphHolderDtorFn = void (*)(void*);
        using CreateBackgroundSimpleManagerFn = bool (*)(void*, RE::BSScrapArray<RE::BSStaticStringT<260>>*, std::int32_t);
        using IsAnimationLoadingCompleteFn = bool (*)(void*);
        using AddItemToTargetKeywordsFn = void (*)(RE::BGSObjectInstance*, RE::BSScrapArray<RE::IKeywordFormBase*>*);
        using RequestAnimationSubGraphFn = void (*)(RE::Actor*, RE::BSAnimationGraphManager*, std::int32_t*, RE::BSScrapArray<RE::IKeywordFormBase*>*, std::int32_t*,
            RE::BSTSmallArray<RE::SubgraphHandle, 2>*, RE::BSTSmallArray<RE::SubgraphIdentifier, 2>*);
        using IsAnimationSubGraphLoadedFn = bool (*)(RE::BSTSmartPointer<RE::BSAnimationGraphManager>*, RE::BSTSmallArray<RE::SubgraphHandle, 2>*, std::int32_t*);
        using ReleaseAnimationSubGraphFn = void (*)(RE::BSTSmartPointer<RE::BSAnimationGraphManager>*, RE::BSTSmallArray<RE::SubgraphHandle, 2>*);
        using GetClipGeneratorBindingFn = void* (*)(void*, RE::BShkbAnimationGraph*, std::uint64_t, char*);
        using GetAnimationFilesForSubgraphFn = const RE::BSTArray<RE::BSFixedString>* (*)(const std::uint64_t*);
        using FindBoneWithNameFn = std::uint64_t (*)(void*, const char*, void*);
        using SampleAnimationTracksFn = void (*)(void*, float, int, HkQsTransform*, int, float*);

        struct NativeFunctions
        {
            GraphHolderCtorFn graphHolderCtor{ nullptr };
            GraphHolderDtorFn graphHolderDtor{ nullptr };
            CreateBackgroundSimpleManagerFn createBackgroundSimpleManager{ nullptr };
            IsAnimationLoadingCompleteFn isAnimationLoadingComplete{ nullptr };
            AddItemToTargetKeywordsFn addItemToTargetKeywords{ nullptr };
            RequestAnimationSubGraphFn requestAnimationSubGraph{ nullptr };
            IsAnimationSubGraphLoadedFn isAnimationSubGraphLoaded{ nullptr };
            ReleaseAnimationSubGraphFn releaseAnimationSubGraph{ nullptr };
            GetClipGeneratorBindingFn getClipGeneratorBinding{ nullptr };
            GetAnimationFilesForSubgraphFn getAnimationFilesForSubgraph{ nullptr };
            FindBoneWithNameFn findBoneWithName{ nullptr };
        };

        enum class Phase : std::uint8_t
        {
            Idle,
            BaseGraphsLoading,
            WeaponSubgraphLoading,
        };

        struct Job
        {
            alignas(16) std::array<std::byte, kSimpleAnimationGraphManagerHolderSize> graphHolderStorage{};
            RE::ObjectRefHandle reference{};
            RE::BSTSmartPointer<RE::TBO_InstanceData> instanceData{};
            RE::BSTSmallArray<RE::SubgraphHandle, 2> subgraphHandles{};
            RE::BSTSmallArray<RE::SubgraphIdentifier, 2> subgraphIdentifiers{};
            RE::TESObjectWEAP* weapon{ nullptr }; // Stable loaded-form identity; never owns the form.
            RE::TESRace* race{ nullptr }; // Stable loaded-form identity; never owns the form.
            std::uintptr_t instanceIdentity{ 0 };
            std::uint32_t referenceFormId{ 0 };
            std::uint32_t weaponFormId{ 0 };
            ULONGLONG startedAtMilliseconds{ 0 };
            Phase phase{ Phase::Idle };
            bool inPowerArmor{ false };
            bool graphHolderConstructed{ false };
            bool longLoadLogged{ false };
        };

        struct FailureEntry
        {
            std::uint32_t referenceFormId{ 0 };
            std::uint32_t weaponFormId{ 0 };
            ULONGLONG retryAfterMilliseconds{ 0 };
            bool inPowerArmor{ false };
            bool occupied{ false };
        };

        struct Runtime
        {
            NativeFunctions native{};
            Job job{};
            std::array<FailureEntry, kFailureCapacity> failures{};
            std::uint64_t nextCaptureSequence{ 0 };
            DWORD ownerThreadId{ 0 };
            bool nativeValidationAttempted{ false };
            bool nativeValidated{ false };
            bool threadMismatchLogged{ false };
        };

        [[nodiscard]] Runtime& runtime()
        {
            // Deliberate process-lifetime ownership. An in-flight Bethesda IO
            // task must not be synchronously cancelled from DLL/static teardown;
            // the OS reclaims it with the game process.
            static Runtime* instance = new Runtime();
            return *instance;
        }

        [[nodiscard]] bool claimOrValidateThread(Runtime& state)
        {
            const DWORD currentThreadId = GetCurrentThreadId();
            if (state.ownerThreadId == 0) {
                state.ownerThreadId = currentThreadId;
                return true;
            }
            if (state.ownerThreadId == currentThreadId) {
                return true;
            }
            if (!state.threadMismatchLogged) {
                ROCK_LOG_ERROR(Animation, "Native idle-grip preharvest rejected a non-owner thread call owner={} caller={}", state.ownerThreadId, currentThreadId);
                state.threadMismatchLogged = true;
            }
            return false;
        }

        [[nodiscard]] bool addressIsInGameText(const std::uintptr_t address)
        {
            const auto text = REL::Module::get().segment(REL::Segment::text);
            return address >= text.address() && address < text.address() + text.size();
        }

        template <std::size_t N>
        [[nodiscard]] bool validateNativeEntry(const char* label, const std::uintptr_t offset, const std::array<std::uint8_t, N>& expected)
        {
            const auto address = REL::Offset(offset).address();
            std::array<std::uint8_t, N> actual{};
            if (!addressIsInGameText(address) || !native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), actual.data(), actual.size()) ||
                actual != expected) {
                ROCK_LOG_ERROR(Init, "Native idle-grip preharvest validation failed for {} at 0x{:X}", label, address);
                return false;
            }
            return true;
        }

        [[nodiscard]] bool resolveNativeFunctions(Runtime& state)
        {
            if (state.nativeValidationAttempted) {
                return state.nativeValidated;
            }
            state.nativeValidationAttempted = true;

            if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
                ROCK_LOG_ERROR(Init, "Native idle-grip preharvest requires the verified Fallout4VR.exe 1.2.72 layout");
                return false;
            }

            const bool entriesMatch = validateNativeEntry(
                                          "SimpleAnimationGraphManagerHolder::ctor", kSimpleAnimationGraphManagerHolderCtor,
                                          std::array<std::uint8_t, 6>{ 0x40, 0x53, 0x48, 0x83, 0xEC, 0x20 }) &&
                validateNativeEntry("SimpleAnimationGraphManagerHolder::dtor", kSimpleAnimationGraphManagerHolderDtor,
                    std::array<std::uint8_t, 5>{ 0x48, 0x89, 0x5C, 0x24, 0x10 }) &&
                validateNativeEntry("SimpleAnimationGraphManagerHolder::CreateBackgroundSimpleManager", kCreateBackgroundSimpleManager,
                    std::array<std::uint8_t, 5>{ 0x48, 0x89, 0x5C, 0x24, 0x18 }) &&
                validateNativeEntry("SimpleAnimationGraphManagerHolder::IsAnimationLoadingComplete", kIsAnimationLoadingComplete,
                    std::array<std::uint8_t, 9>{ 0x48, 0x8B, 0x41, 0x10, 0x48, 0x85, 0xC0, 0x74, 0x0C }) &&
                validateNativeEntry("RequestAnimationSubGraph", kRequestAnimationSubGraph, std::array<std::uint8_t, 5>{ 0x48, 0x89, 0x5C, 0x24, 0x08 }) &&
                validateNativeEntry("IsAnimationSubGraphLoaded", kIsAnimationSubGraphLoaded, std::array<std::uint8_t, 5>{ 0x48, 0x89, 0x5C, 0x24, 0x08 }) &&
                validateNativeEntry("ReleaseAnimationSubGraph", kReleaseAnimationSubGraph, std::array<std::uint8_t, 7>{ 0x48, 0x83, 0xEC, 0x28, 0x83, 0x7A, 0x18 }) &&
                validateNativeEntry("AddItemToTargetKeywords", kAddItemToTargetKeywords, std::array<std::uint8_t, 5>{ 0x48, 0x89, 0x5C, 0x24, 0x08 }) &&
                validateNativeEntry("GetClipGeneratorBinding", kGetClipGeneratorBinding,
                    std::array<std::uint8_t, 10>{ 0x48, 0x8B, 0xC4, 0x53, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56 }) &&
                validateNativeEntry("AnimationFileData numeric lookup", kGetAnimationFilesForSubgraph, std::array<std::uint8_t, 6>{ 0x48, 0x8B, 0xD1, 0x48, 0x8B, 0x0D }) &&
                validateNativeEntry("hkaSkeletonUtils::findBoneWithName", kFindBoneWithName, std::array<std::uint8_t, 5>{ 0x48, 0x89, 0x5C, 0x24, 0x08 });
            if (!entriesMatch) {
                return false;
            }

            state.native.graphHolderCtor = reinterpret_cast<GraphHolderCtorFn>(REL::Offset(kSimpleAnimationGraphManagerHolderCtor).address());
            state.native.graphHolderDtor = reinterpret_cast<GraphHolderDtorFn>(REL::Offset(kSimpleAnimationGraphManagerHolderDtor).address());
            state.native.createBackgroundSimpleManager = reinterpret_cast<CreateBackgroundSimpleManagerFn>(REL::Offset(kCreateBackgroundSimpleManager).address());
            state.native.isAnimationLoadingComplete = reinterpret_cast<IsAnimationLoadingCompleteFn>(REL::Offset(kIsAnimationLoadingComplete).address());
            state.native.requestAnimationSubGraph = reinterpret_cast<RequestAnimationSubGraphFn>(REL::Offset(kRequestAnimationSubGraph).address());
            state.native.isAnimationSubGraphLoaded = reinterpret_cast<IsAnimationSubGraphLoadedFn>(REL::Offset(kIsAnimationSubGraphLoaded).address());
            state.native.releaseAnimationSubGraph = reinterpret_cast<ReleaseAnimationSubGraphFn>(REL::Offset(kReleaseAnimationSubGraph).address());
            state.native.addItemToTargetKeywords = reinterpret_cast<AddItemToTargetKeywordsFn>(REL::Offset(kAddItemToTargetKeywords).address());
            state.native.getClipGeneratorBinding = reinterpret_cast<GetClipGeneratorBindingFn>(REL::Offset(kGetClipGeneratorBinding).address());
            state.native.getAnimationFilesForSubgraph = reinterpret_cast<GetAnimationFilesForSubgraphFn>(REL::Offset(kGetAnimationFilesForSubgraph).address());
            state.native.findBoneWithName = reinterpret_cast<FindBoneWithNameFn>(REL::Offset(kFindBoneWithName).address());
            state.nativeValidated = true;
            ROCK_LOG_INFO(Init, "Native idle-grip preharvest validated: plain off-screen graph holder, exact first-person subgraph, and idle clip sampler ready");
            return true;
        }

        [[nodiscard]] RE::SimpleAnimationGraphManagerHolder* graphHolder(Job& job)
        {
            if (!job.graphHolderConstructed) {
                return nullptr;
            }
            return reinterpret_cast<RE::SimpleAnimationGraphManagerHolder*>(job.graphHolderStorage.data());
        }

        [[nodiscard]] bool sameFailureIdentity(const FailureEntry& entry, const Job& job)
        {
            return entry.occupied && entry.referenceFormId == job.referenceFormId && entry.weaponFormId == job.weaponFormId && entry.inPowerArmor == job.inPowerArmor;
        }

        [[nodiscard]] bool candidateIsCoolingDown(const Runtime& state, const Job& candidate, const ULONGLONG now)
        {
            for (const auto& entry : state.failures) {
                if (sameFailureIdentity(entry, candidate) && now < entry.retryAfterMilliseconds) {
                    return true;
                }
            }
            return false;
        }

        void recordFailure(Runtime& state, const Job& job, const ULONGLONG now)
        {
            FailureEntry* destination = nullptr;
            FailureEntry* earliestRetry = nullptr;
            for (auto& entry : state.failures) {
                if (sameFailureIdentity(entry, job)) {
                    destination = &entry;
                    break;
                }
                if (!entry.occupied && !destination) {
                    destination = &entry;
                }
                if (entry.occupied && (!earliestRetry || entry.retryAfterMilliseconds < earliestRetry->retryAfterMilliseconds)) {
                    earliestRetry = &entry;
                }
            }
            if (!destination) {
                destination = earliestRetry;
            }
            if (!destination) {
                return;
            }
            *destination = FailureEntry{
                .referenceFormId = job.referenceFormId,
                .weaponFormId = job.weaponFormId,
                .retryAfterMilliseconds = now + kFailureRetryDelayMilliseconds,
                .inPowerArmor = job.inPowerArmor,
                .occupied = true,
            };
        }

        void releaseJob(Runtime& state)
        {
            auto& job = state.job;
            if (job.graphHolderConstructed) {
                auto* holder = graphHolder(job);
                if (holder && holder->animationGraphManager && !job.subgraphHandles.empty()) {
                    state.native.releaseAnimationSubGraph(&holder->animationGraphManager, &job.subgraphHandles);
                }
                job.subgraphHandles.clear();
                job.subgraphIdentifiers.clear();
                state.native.graphHolderDtor(job.graphHolderStorage.data());
                job.graphHolderConstructed = false;
            }
            job = {};
        }

        void failJob(Runtime& state, const char* reason)
        {
            const Job failed = {
                .instanceIdentity = state.job.instanceIdentity,
                .referenceFormId = state.job.referenceFormId,
                .weaponFormId = state.job.weaponFormId,
                .inPowerArmor = state.job.inPowerArmor,
            };
            ROCK_LOG_WARN(Animation, "Native idle-grip preharvest failed formID={:08X} refID={:08X} powerArmor={} phase={} reason={}", state.job.weaponFormId,
                state.job.referenceFormId, state.job.inPowerArmor ? "yes" : "no", static_cast<unsigned>(state.job.phase), reason ? reason : "unknown");
            releaseJob(state);
            recordFailure(state, failed, GetTickCount64());
        }

        void finishWithoutPublishing(Runtime& state, const char* reason)
        {
            ROCK_LOG_DEBUG(Animation, "Native idle-grip preharvest released without publication formID={:08X} phase={} reason={}", state.job.weaponFormId,
                static_cast<unsigned>(state.job.phase), reason ? reason : "unknown");
            releaseJob(state);
        }

        [[nodiscard]] RE::BSTSmartPointer<RE::TBO_InstanceData> resolveInstanceData(RE::TESObjectREFR* reference, const RE::TESObjectWEAP* weapon)
        {
            RE::BSTSmartPointer<RE::TBO_InstanceData> instanceData{};
            if (!reference || !weapon || !reference->extraList) {
                return instanceData;
            }
            if (const auto* instanceExtra = reference->extraList->GetByType<RE::ExtraInstanceData>()) {
                if (instanceExtra->data) {
                    return instanceExtra->data;
                }
            }
            if (const auto* objectInstanceExtra = reference->extraList->GetByType<RE::BGSObjectInstanceExtra>()) {
                weapon->ApplyMods(instanceData, objectInstanceExtra);
            }
            return instanceData;
        }

        [[nodiscard]] bool isFiniteTransform(const RE::NiTransform& transform)
        {
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return std::isfinite(transform.translate.x) && std::isfinite(transform.translate.y) && std::isfinite(transform.translate.z) && std::isfinite(transform.scale) &&
                std::abs(transform.scale) > 0.000001f;
        }

        [[nodiscard]] bool addressIsExecutable(const void* address)
        {
            if (!address) {
                return false;
            }
            MEMORY_BASIC_INFORMATION memoryInfo{};
            if (VirtualQuery(address, &memoryInfo, sizeof(memoryInfo)) == 0 || memoryInfo.State != MEM_COMMIT || (memoryInfo.Protect & (PAGE_GUARD | PAGE_NOACCESS)) != 0) {
                return false;
            }
            const DWORD protection = memoryInfo.Protect & 0xFF;
            return protection == PAGE_EXECUTE || protection == PAGE_EXECUTE_READ || protection == PAGE_EXECUTE_READWRITE || protection == PAGE_EXECUTE_WRITECOPY;
        }

        [[nodiscard]] bool guardedSampleTracks(const SampleAnimationTracksFn sample, void* animation, const int transformTrackCount, HkQsTransform* output) noexcept
        {
#if defined(_MSC_VER)
            __try {
                sample(animation, 0.0f, transformTrackCount, output, 0, nullptr);
                return true;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
#else
            sample(animation, 0.0f, transformTrackCount, output, 0, nullptr);
            return true;
#endif
        }

        [[nodiscard]] bool convertWeaponTrackToHandInWeapon(const HkQsTransform& sampledWeaponLocal, RE::NiTransform& outHandInWeapon)
        {
            for (const float value : sampledWeaponLocal.translation) {
                if (!std::isfinite(value)) {
                    return false;
                }
            }
            float quaternionNormSquared = 0.0f;
            for (const float value : sampledWeaponLocal.rotation) {
                if (!std::isfinite(value)) {
                    return false;
                }
                quaternionNormSquared += value * value;
            }
            if (quaternionNormSquared < 0.000001f) {
                return false;
            }
            for (const float value : sampledWeaponLocal.scale) {
                if (!std::isfinite(value)) {
                    return false;
                }
            }
            if (std::abs(sampledWeaponLocal.scale[0]) < 0.000001f || std::abs(sampledWeaponLocal.scale[0] - sampledWeaponLocal.scale[1]) > 0.001f ||
                std::abs(sampledWeaponLocal.scale[0] - sampledWeaponLocal.scale[2]) > 0.001f) {
                return false;
            }

            RE::NiTransform weaponLocal{};
            weaponLocal.translate = RE::NiPoint3{
                sampledWeaponLocal.translation[0],
                sampledWeaponLocal.translation[1],
                sampledWeaponLocal.translation[2],
            };
            weaponLocal.rotate = transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(sampledWeaponLocal.rotation);
            weaponLocal.scale = sampledWeaponLocal.scale[0];
            if (!isFiniteTransform(weaponLocal)) {
                return false;
            }

            outHandInWeapon = transform_math::invertTransform(weaponLocal);
            return isFiniteTransform(outHandInWeapon);
        }

        [[nodiscard]] bool trySampleClip(Runtime& state, RE::BShkbAnimationGraph* graph, const std::uint64_t subgraphIdentifier, char* clipName, RE::NiTransform& outHandInWeapon)
        {
            void* swapSingleton = nullptr;
            const auto singletonAddress = REL::Offset(kBehaviorGraphSwapSingleton).address();
            if (!native_memory::tryReadValue(reinterpret_cast<void* const*>(singletonAddress), swapSingleton) || !swapSingleton) {
                return false;
            }

            void* bindingWithTriggers = state.native.getClipGeneratorBinding(swapSingleton, graph, subgraphIdentifier, clipName);
            if (!bindingWithTriggers) {
                return false;
            }

            void* binding = nullptr;
            void* animation = nullptr;
            if (!native_memory::tryReadField(bindingWithTriggers, kBindingFromBindingWithTriggersOffset, binding) || !binding ||
                !native_memory::tryReadField(binding, kAnimationFromBindingOffset, animation) || !animation) {
                return false;
            }

            int transformTrackCount = 0;
            if (!native_memory::tryReadField(animation, kAnimationTransformTrackCountOffset, transformTrackCount) || transformTrackCount <= 0 ||
                transformTrackCount > static_cast<int>(kMaxBonesAndTracks)) {
                return false;
            }

            void* skeletonOwner = nullptr;
            void* skeleton = nullptr;
            if (!native_memory::tryReadField(graph, kGraphSkeletonOwnerOffset, skeletonOwner) || !skeletonOwner ||
                !native_memory::tryReadField(skeletonOwner, kSkeletonFromOwnerOffset, skeleton) || !skeleton) {
                return false;
            }

            int boneCount = 0;
            int parentCount = 0;
            const std::int16_t* parentIndices = nullptr;
            if (!native_memory::tryReadField(skeleton, kSkeletonBoneCountOffset, boneCount) || !native_memory::tryReadField(skeleton, kSkeletonParentCountOffset, parentCount) ||
                !native_memory::tryReadField(skeleton, kSkeletonParentIndicesOffset, parentIndices) || boneCount <= 0 || boneCount > static_cast<int>(kMaxBonesAndTracks) ||
                parentCount < boneCount || !parentIndices) {
                return false;
            }

            const auto weaponBoneRaw = state.native.findBoneWithName(skeleton, "Weapon", nullptr);
            const auto handBoneRaw = state.native.findBoneWithName(skeleton, "RArm_Hand", nullptr);
            if (weaponBoneRaw == 0xFFFFFFFFull || handBoneRaw == 0xFFFFFFFFull || weaponBoneRaw >= static_cast<std::uint64_t>(boneCount) ||
                handBoneRaw >= static_cast<std::uint64_t>(boneCount)) {
                return false;
            }
            const int weaponBoneIndex = static_cast<int>(weaponBoneRaw);
            const int handBoneIndex = static_cast<int>(handBoneRaw);

            std::array<std::int16_t, kMaxBonesAndTracks> parentBuffer{};
            if (!native_memory::guardedCopyFromMemory(parentIndices, parentBuffer.data(), static_cast<std::size_t>(boneCount) * sizeof(std::int16_t)) ||
                !native_idle_grip_preharvest_policy::weaponIsDirectChildOfHand(weaponBoneIndex, handBoneIndex,
                    std::span<const std::int16_t>{ parentBuffer.data(), static_cast<std::size_t>(boneCount) })) {
                return false;
            }

            const std::int16_t* trackToBoneIndices = nullptr;
            int mappingCount = 0;
            if (!native_memory::tryReadField(binding, kTrackToBoneMappingOffset, trackToBoneIndices) ||
                !native_memory::tryReadField(binding, kTrackToBoneMappingCountOffset, mappingCount) || mappingCount < 0 || mappingCount > static_cast<int>(kMaxBonesAndTracks)) {
                return false;
            }

            std::array<std::int16_t, kMaxBonesAndTracks> mappingBuffer{};
            std::span<const std::int16_t> mapping{};
            if (mappingCount > 0) {
                if (!trackToBoneIndices ||
                    !native_memory::guardedCopyFromMemory(trackToBoneIndices, mappingBuffer.data(), static_cast<std::size_t>(mappingCount) * sizeof(std::int16_t))) {
                    return false;
                }
                mapping = std::span<const std::int16_t>{
                    mappingBuffer.data(),
                    static_cast<std::size_t>(mappingCount),
                };
            }
            const int weaponTrackIndex = native_idle_grip_preharvest_policy::findTransformTrackForBone(weaponBoneIndex, transformTrackCount, mapping);
            if (weaponTrackIndex < 0) {
                return false;
            }

            void** animationVtable = nullptr;
            SampleAnimationTracksFn sampleTracks = nullptr;
            if (!native_memory::tryReadValue(reinterpret_cast<void***>(animation), animationVtable) || !animationVtable ||
                !native_memory::tryReadValue(reinterpret_cast<SampleAnimationTracksFn*>(animationVtable + 5), sampleTracks) ||
                !addressIsExecutable(reinterpret_cast<const void*>(sampleTracks))) {
                return false;
            }

            alignas(16) std::array<HkQsTransform, kMaxBonesAndTracks> sampledTracks{};
            if (!guardedSampleTracks(sampleTracks, animation, transformTrackCount, sampledTracks.data())) {
                return false;
            }
            return convertWeaponTrackToHandInWeapon(sampledTracks[static_cast<std::size_t>(weaponTrackIndex)], outHandInWeapon);
        }

        [[nodiscard]] bool tryExtractIdleGrip(Runtime& state, RE::BSAnimationGraphManager& manager, RE::NiTransform& outHandInWeapon, std::array<char, 260>& outClipPath,
            std::uint64_t& outSubgraphIdentifier)
        {
            const auto selection = native_idle_grip_preharvest_policy::selectFirstPersonGraph(manager.graph.size(), state.job.subgraphIdentifiers.size());
            if (!selection.valid) {
                return false;
            }
            const auto firstPersonIndex = static_cast<decltype(manager.graph)::size_type>(selection.graphIndex);
            auto* graph = manager.graph[firstPersonIndex].get();
            if (!graph) {
                return false;
            }
            outSubgraphIdentifier = state.job.subgraphIdentifiers[static_cast<decltype(state.job.subgraphIdentifiers)::size_type>(selection.graphIndex)].identifier;
            if (outSubgraphIdentifier == 0) {
                return false;
            }

            const auto* animationFiles = [&]() -> const RE::BSTArray<RE::BSFixedString>* {
                void* lookupSingleton = nullptr;
                const auto lookupSingletonAddress = REL::Offset(kAnimationFileLookupSingleton).address();
                if (!native_memory::tryReadValue(reinterpret_cast<void* const*>(lookupSingletonAddress), lookupSingleton) || !lookupSingleton) {
                    return nullptr;
                }
                return state.native.getAnimationFilesForSubgraph(&outSubgraphIdentifier);
            }();
            if (!animationFiles) {
                return false;
            }

            constexpr std::array<std::string_view, 2> desiredClipStems{
                "WPNIdleReady",
                "WPNIdle",
            };

            for (const auto desiredStem : desiredClipStems) {
                for (const auto& animationFile : *animationFiles) {
                    const char* pathChars = animationFile.c_str();
                    if (!pathChars) {
                        continue;
                    }
                    const std::string_view path{ pathChars };
                    if (!native_idle_grip_preharvest_policy::clipPathHasStem(path, desiredStem) || path.size() >= outClipPath.size()) {
                        continue;
                    }

                    outClipPath.fill('\0');
                    std::memcpy(outClipPath.data(), path.data(), path.size());
                    if (trySampleClip(state, graph, outSubgraphIdentifier, outClipPath.data(), outHandInWeapon)) {
                        return true;
                    }
                }
            }
            return false;
        }

        [[nodiscard]] bool progressJob(Runtime& state)
        {
            auto& job = state.job;
            if (job.phase == Phase::Idle) {
                return true;
            }

            const ULONGLONG now = GetTickCount64();
            if (!job.longLoadLogged && now - job.startedAtMilliseconds >= kLongLoadLogDelayMilliseconds) {
                ROCK_LOG_INFO(Animation, "Native idle-grip preharvest still loading formID={:08X} phase={} elapsedMs={}", job.weaponFormId, static_cast<unsigned>(job.phase),
                    now - job.startedAtMilliseconds);
                job.longLoadLogged = true;
            }

            auto* player = RE::PlayerCharacter::GetSingleton();
            if (!player) {
                return false;
            }

            if (job.phase == Phase::BaseGraphsLoading) {
                auto* holder = graphHolder(job);
                if (!holder) {
                    failJob(state, "backgroundGraphHolderUnavailable");
                    return true;
                }
                if (!state.native.isAnimationLoadingComplete(holder)) {
                    return false;
                }

                if (!g_rockConfig.rockAuthoredPrimaryFiringGripTestEnabled || f4vr::isLeftHandedMode()) {
                    finishWithoutPublishing(state, "experimentNoLongerEligible");
                    return true;
                }

                if (player->race != job.race || f4vr::isInPowerArmor() != job.inPowerArmor) {
                    failJob(state, "playerRaceOrPowerArmorChanged");
                    return true;
                }
                auto* manager = holder ? holder->animationGraphManager.get() : nullptr;
                if (!holder || !manager) {
                    failJob(state, "backgroundManagerUnavailable");
                    return true;
                }

                RE::BGSObjectInstance objectInstance(job.weapon, job.instanceData.get());
                RE::BSScrapArray<RE::IKeywordFormBase*> targetKeywords{};
                state.native.addItemToTargetKeywords(&objectInstance, &targetKeywords);
                if (targetKeywords.empty()) {
                    failJob(state, "weaponInstanceProducedNoTargetKeywords");
                    return true;
                }

                std::int32_t role = kWeaponAnimationRole;
                std::int32_t priority = kIoTaskPriority;
                state.native.requestAnimationSubGraph(player, manager, &role, &targetKeywords, &priority, &job.subgraphHandles, &job.subgraphIdentifiers);
                if (job.subgraphHandles.empty() || job.subgraphIdentifiers.empty()) {
                    failJob(state, "exactWeaponSubgraphRequestReturnedEmpty");
                    return true;
                }

                job.phase = Phase::WeaponSubgraphLoading;
                job.longLoadLogged = false;
                job.startedAtMilliseconds = now;
                return false;
            }

            auto* holder = graphHolder(job);
            auto* manager = holder ? holder->animationGraphManager.get() : nullptr;
            if (!holder || !manager) {
                failJob(state, "backgroundManagerLost");
                return true;
            }
            std::int32_t priority = kIoTaskPriority;
            if (!state.native.isAnimationSubGraphLoaded(&holder->animationGraphManager, &job.subgraphHandles, &priority)) {
                return false;
            }
            if (!g_rockConfig.rockAuthoredPrimaryFiringGripTestEnabled || f4vr::isLeftHandedMode()) {
                finishWithoutPublishing(state, "experimentNoLongerEligible");
                return true;
            }

            RE::NiTransform handInWeapon{};
            std::array<char, 260> clipPath{};
            std::uint64_t subgraphIdentifier = 0;
            if (!tryExtractIdleGrip(state, *manager, handInWeapon, clipPath, subgraphIdentifier)) {
                failJob(state, "firstPersonIdleWeaponTrackUnavailable");
                return true;
            }

            const auto reference = job.reference.get();
            auto* referenceRaw = reference.get();
            auto* root = referenceRaw ? referenceRaw->Get3D() : nullptr;
            if (!referenceRaw || !root || referenceRaw->GetObjectReference() != job.weapon) {
                failJob(state, "looseReferenceUnavailableAtPublish");
                return true;
            }

            const std::uint64_t captureSequence = kPreharvestCaptureSequenceDomain | (++state.nextCaptureSequence);
            if (!authored_weapon_grip_library::publish(job.weapon, root, job.inPowerArmor, handInWeapon, captureSequence)) {
                failJob(state, "authoredGripLibraryRejectedSample");
                return true;
            }

            ROCK_LOG_INFO(Animation,
                "Native idle-grip preharvest succeeded formID={:08X} refID={:08X} subgraph={} clip={} powerArmor={} handInWeaponT=({:.3f},{:.3f},{:.3f}) scale={:.5f}",
                job.weaponFormId, job.referenceFormId, subgraphIdentifier, clipPath.data(), job.inPowerArmor ? "yes" : "no", handInWeapon.translate.x, handInWeapon.translate.y,
                handInWeapon.translate.z, handInWeapon.scale);
            releaseJob(state);
            return true;
        }

        [[nodiscard]] Job describeCandidate(RE::TESObjectREFR* reference)
        {
            Job candidate{};
            if (!reference) {
                return candidate;
            }
            auto* baseForm = reference->GetObjectReference();
            auto* weapon = baseForm ? baseForm->As<RE::TESObjectWEAP>() : nullptr;
            if (!weapon || weapon->weaponData.type == RE::WEAPON_TYPE::kGrenade || weapon->weaponData.type == RE::WEAPON_TYPE::kMine || !reference->Get3D()) {
                return candidate;
            }

            candidate.reference = reference->GetHandle();
            candidate.weapon = weapon;
            candidate.referenceFormId = reference->GetFormID();
            candidate.weaponFormId = weapon->GetFormID();
            candidate.inPowerArmor = f4vr::isInPowerArmor();
            return candidate;
        }

        void startJob(Runtime& state, Job&& candidate)
        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (!player || !player->race || !candidate.weapon || !candidate.reference || candidate.weaponFormId == 0) {
                return;
            }

            const auto reference = candidate.reference.get();
            auto* referenceRaw = reference.get();
            if (!referenceRaw || referenceRaw->GetObjectReference() != candidate.weapon) {
                return;
            }
            candidate.instanceData = resolveInstanceData(referenceRaw, candidate.weapon);
            candidate.instanceIdentity = reinterpret_cast<std::uintptr_t>(candidate.instanceData.get());

            candidate.race = player->race;
            candidate.startedAtMilliseconds = GetTickCount64();
            candidate.phase = Phase::BaseGraphsLoading;
            state.job = std::move(candidate);

            void* constructed = state.native.graphHolderCtor(state.job.graphHolderStorage.data());
            if (constructed != state.job.graphHolderStorage.data()) {
                state.job.graphHolderConstructed = false;
                failJob(state, "simpleGraphHolderConstructionFailed");
                return;
            }
            state.job.graphHolderConstructed = true;

            auto* playerRoot = player->Get3D();
            RE::BSScrapArray<RE::BSStaticStringT<260>> graphProjects{};
            if (!playerRoot || !player->PopulateGraphProjectsToLoad(playerRoot, graphProjects) || graphProjects.size() < 2) {
                failJob(state, "playerGraphProjectsUnavailable");
                return;
            }
            if (!state.native.createBackgroundSimpleManager(state.job.graphHolderStorage.data(), &graphProjects, kIoTaskPriority)) {
                failJob(state, "backgroundGraphLoadRequestRejected");
                return;
            }

            const char* baseGraph = graphProjects[0].c_str();
            const char* firstPersonGraph = graphProjects[1].c_str();
            ROCK_LOG_INFO(Animation,
                "Native idle-grip preharvest started formID={:08X} refID={:08X} powerArmor={} instance=0x{:X} graphProjects={} base={} firstPerson={}",
                state.job.weaponFormId, state.job.referenceFormId, state.job.inPowerArmor ? "yes" : "no", state.job.instanceIdentity, graphProjects.size(),
                baseGraph ? baseGraph : "<null>", firstPersonGraph ? firstPersonGraph : "<null>");
        }
    }

    void observeCandidate(RE::TESObjectREFR* candidate) noexcept
    {
        auto& state = runtime();
        if (!claimOrValidateThread(state)) {
            return;
        }

        if (state.job.phase != Phase::Idle) {
            (void)progressJob(state);
        }
        if (state.job.phase != Phase::Idle || !candidate || !g_rockConfig.rockAuthoredPrimaryFiringGripTestEnabled || f4vr::isLeftHandedMode() || !resolveNativeFunctions(state)) {
            return;
        }

        Job candidateDescription = describeCandidate(candidate);
        if (!candidateDescription.weapon || !candidateDescription.reference || candidateDescription.weaponFormId == 0) {
            return;
        }
        if (authored_weapon_grip_library::find(candidateDescription.weapon, candidate->Get3D(), candidateDescription.inPowerArmor).found) {
            return;
        }
        if (candidateIsCoolingDown(state, candidateDescription, GetTickCount64())) {
            return;
        }

        startJob(state, std::move(candidateDescription));
    }
}
