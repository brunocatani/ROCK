#include "physics-interaction/weapon/NativeIdleGripPreharvest.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/NativeIdleGripPreharvestPolicy.h"

#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/BSAnimationGraph.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/BSFixedString.h"
#include "RE/Bethesda/BSLock.h"
#include "RE/Bethesda/BSStringPool.h"
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
        constexpr std::uintptr_t kLoadIdleAnimationResource = 0x1728BA0;
        constexpr std::uintptr_t kMoveAnimationResourceHandle = 0x172AB40;
        constexpr std::uintptr_t kIsHkxDerivativeDbData = 0x152C0D0;
        constexpr std::uintptr_t kRetrieveBindingFromContainer = 0x17865C0;
        constexpr std::uintptr_t kFindBoneWithName = 0x190A580;
        constexpr std::uintptr_t kBehaviorGraphSwapSingleton = 0x5AB9200;
        constexpr std::uintptr_t kAnimationFileLookupSingleton = 0x5B64318;

        constexpr std::size_t kSimpleAnimationGraphManagerHolderSize = 0x18;
        static_assert(kSimpleAnimationGraphManagerHolderSize == sizeof(RE::SimpleAnimationGraphManagerHolder));
        constexpr std::uint32_t kSubgraphOutputInlineCapacity = 2;
        constexpr std::size_t kSmallArrayInlineStorageOffset = 0x8;
        static_assert(sizeof(RE::BSTSmallArray<RE::SubgraphHandle, kSubgraphOutputInlineCapacity>) == 0x20);
        static_assert(sizeof(RE::BSTSmallArray<RE::SubgraphIdentifier, kSubgraphOutputInlineCapacity>) == 0x20);
        constexpr std::ptrdiff_t kGraphSkeletonOwnerOffset = 0x240;
        constexpr std::ptrdiff_t kSkeletonFromOwnerOffset = 0x20;
        constexpr std::ptrdiff_t kSkeletonParentIndicesOffset = 0x18;
        constexpr std::ptrdiff_t kSkeletonParentCountOffset = 0x20;
        constexpr std::ptrdiff_t kSkeletonBoneCountOffset = 0x30;
        // hkaSkeleton's reference pose is a bone-indexed hkQsTransform array.
        // FO4VR's skeleton copy path independently establishes pointer/count
        // at +0x38/+0x40 with a 0x30-byte element stride.
        constexpr std::ptrdiff_t kSkeletonReferencePoseOffset = 0x38;
        constexpr std::ptrdiff_t kSkeletonReferencePoseCountOffset = 0x40;
        constexpr std::ptrdiff_t kBindingFromBindingWithTriggersOffset = 0x10;
        constexpr std::ptrdiff_t kAnimationFromBindingOffset = 0x18;
        constexpr std::ptrdiff_t kTrackToBoneMappingOffset = 0x20;
        constexpr std::ptrdiff_t kTrackToBoneMappingCountOffset = 0x28;
        // FO4VR hkaAnimation constructors and independent sample paths establish
        // type/duration/transform-count/float-count at +0x10/+0x14/+0x18/+0x1C.
        constexpr std::ptrdiff_t kAnimationTypeOffset = 0x10;
        constexpr std::ptrdiff_t kAnimationDurationOffset = 0x14;
        constexpr std::ptrdiff_t kAnimationTransformTrackCountOffset = 0x18;
        constexpr std::ptrdiff_t kAnimationFloatTrackCountOffset = 0x1C;
        // hkaAnimationBinding's FO4VR constructor places the byte reflected as
        // blendHint immediately after its three track/partition arrays;
        // hkbClipGenerator::vfunction24 at 0x14192D9A0 independently passes
        // binding+0x50 to generateInternal's BlendHint parameter.
        constexpr std::ptrdiff_t kBindingBlendHintOffset = 0x50;
        constexpr std::ptrdiff_t kAnimationResourceFlagsOffset = 0x0C;
        constexpr std::ptrdiff_t kAnimationResourceDataOffset = 0x20;
        constexpr std::ptrdiff_t kRootContainerFromAnimationDataOffset = 0x08;

        // GetClipGeneratorBinding at 0x141774800 and the loaded-subgraph map
        // walker at 0x141777570 independently establish this layout. Bethesda's
        // load-completion path at 0x1417743B0 selects the same entry by the
        // SubgraphHandle stored at +0x00. That handle remains authoritative
        // when malformed mod metadata gives the binding table a different
        // identifier from its AnimationStanceData/AnimationFileData key.
        constexpr std::ptrdiff_t kGraphLoadedSubgraphsOffset = 0x3A0;
        constexpr std::ptrdiff_t kLoadedSubgraphsEntriesOffset = 0x00;
        constexpr std::ptrdiff_t kLoadedSubgraphsCountOffset = 0x10;
        constexpr std::ptrdiff_t kLoadedSubgraphsLockOffset = 0x28;
        constexpr std::size_t kLoadedSubgraphEntryStride = 0x48;
        constexpr std::ptrdiff_t kLoadedSubgraphHandleOffset = 0x00;
        constexpr std::ptrdiff_t kLoadedSubgraphBindingTableOffset = 0x08;
        constexpr std::ptrdiff_t kBindingTableBucketCountOffset = 0x0C;
        constexpr std::ptrdiff_t kBindingTableBucketsOffset = 0x28;
        constexpr std::ptrdiff_t kBindingTableSubgraphIdentifierOffset = 0xC0;
        constexpr std::size_t kBindingTableNodeStride = 0x18;
        constexpr std::ptrdiff_t kBindingTableNodeKeyOffset = 0x00;
        constexpr std::ptrdiff_t kBindingTableNodeOccupancyOffset = 0x10;

        constexpr std::size_t kMaxBonesAndTracks = 768;
        constexpr std::size_t kMaxLoadedSubgraphs = 128;
        constexpr std::size_t kMaxClipBindingBuckets = 4096;
        constexpr std::size_t kMaxFixedStringShallowDepth = 8;
        constexpr std::size_t kFailureCapacity = 64;
        constexpr ULONGLONG kFailureRetryDelayMilliseconds = 30000;
        constexpr ULONGLONG kLongLoadLogDelayMilliseconds = 5000;
        constexpr std::uint64_t kPreharvestCaptureSequenceDomain = 1ull << 63;
        constexpr std::int32_t kWeaponAnimationRole = 1;
        constexpr std::int32_t kIoTaskPriority = 3;

        constexpr std::array<const char*, authored_weapon_grip_library::kFiringFingerBoneCount> kRightFiringFingerBoneNames{
            "RArm_Finger11",
            "RArm_Finger12",
            "RArm_Finger13",
            "RArm_Finger21",
            "RArm_Finger22",
            "RArm_Finger23",
            "RArm_Finger31",
            "RArm_Finger32",
            "RArm_Finger33",
            "RArm_Finger41",
            "RArm_Finger42",
            "RArm_Finger43",
            "RArm_Finger51",
            "RArm_Finger52",
            "RArm_Finger53",
        };

        struct alignas(16) HkQsTransform
        {
            float translation[4]{};
            float rotation[4]{};
            float scale[4]{};
        };
        static_assert(sizeof(HkQsTransform) == 0x30);

        // Opaque one-pointer mirror of Bethesda's BShkbHkxDB resource handle.
        // The native move assignment below is the sole owner-transfer/release path.
        struct AnimationResourceHandle
        {
            void* entry{ nullptr };
        };
        static_assert(sizeof(AnimationResourceHandle) == sizeof(void*));

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
        using LoadIdleAnimationResourceFn = bool (*)(RE::BSFixedString*, AnimationResourceHandle*);
        using MoveAnimationResourceHandleFn = AnimationResourceHandle* (*)(AnimationResourceHandle*, AnimationResourceHandle*);
        using IsHkxDerivativeDbDataFn = bool (*)(void*);
        using RetrieveBindingFromContainerFn = void (*)(void*, void**, char*);
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
            LoadIdleAnimationResourceFn loadIdleAnimationResource{ nullptr };
            MoveAnimationResourceHandleFn moveAnimationResourceHandle{ nullptr };
            IsHkxDerivativeDbDataFn isHkxDerivativeDbData{ nullptr };
            RetrieveBindingFromContainerFn retrieveBindingFromContainer{ nullptr };
            FindBoneWithNameFn findBoneWithName{ nullptr };
        };

        enum class Phase : std::uint8_t
        {
            Idle,
            BaseGraphsLoading,
            WeaponSubgraphLoading,
            IdleClipLoading,
        };

        enum class CandidateOrigin : std::uint8_t
        {
            Unknown,
            LooseReference,
            EquippedWeapon,
        };

        enum class ExtractionResult : std::uint8_t
        {
            Failed,
            Pending,
            Succeeded,
        };

        enum class IdleGripExtractionFailure : std::uint8_t
        {
            None,
            FirstPersonGraphPairUnavailable,
            FirstPersonGraphUnavailable,
            FirstPersonSubgraphHandleUnavailable,
            FirstPersonSubgraphIdentifierUnavailable,
            AnimationFileLookupUnavailable,
            AnimationFileListUnavailable,
            AnimationFileListEmpty,
            IdleClipPathUnavailable,
            IdleClipPathTooLong,
            BehaviorGraphSwapSingletonUnavailable,
            ClipBindingUnavailable,
            DirectClipLoadRequestFailed,
            DirectClipPathMismatch,
            DirectClipResourceLayoutUnavailable,
            DirectClipDataInvalid,
            DirectClipContainerUnavailable,
            DirectClipBindingUnavailable,
            AnimationBindingUnavailable,
            BoundAnimationUnavailable,
            TransformTrackCountInvalid,
            GraphSkeletonUnavailable,
            SkeletonLayoutInvalid,
            RequiredBoneUnavailable,
            SkeletonParentCopyUnavailable,
            WeaponNotDirectChildOfHand,
            TrackMappingLayoutInvalid,
            TrackMappingCopyUnavailable,
            WeaponTrackUnavailable,
            AnimationSamplerUnavailable,
            AnimationSamplingFault,
            SampledWeaponTransformInvalid,
            IncompleteFiringFingerPose,
        };

        struct IdleGripExtractionDiagnostics
        {
            IdleGripExtractionFailure failure{ IdleGripExtractionFailure::None };
            std::size_t graphCount{ 0 };
            std::size_t handleCount{ 0 };
            std::size_t identifierCount{ 0 };
            std::size_t animationFileCount{ 0 };
            std::size_t idlePathMatchCount{ 0 };
            std::size_t sampleAttemptCount{ 0 };
            std::size_t loadedSubgraphCount{ 0 };
            std::size_t graphHandleMatchCount{ 0 };
            std::size_t graphClipBucketCount{ 0 };
            std::size_t graphClipPathCount{ 0 };
            std::size_t graphIdlePathCandidateCount{ 0 };
            std::uint64_t subgraphHandle{ 0 };
            std::uint64_t subgraphIdentifier{ 0 };
            std::uint64_t bindingSubgraphIdentifier{ 0 };
            std::uint64_t weaponBone{ 0xFFFFFFFFull };
            std::uint64_t handBone{ 0xFFFFFFFFull };
            std::uint32_t directResourceState{ 0xFFFFFFFFu };
            std::uint32_t bindingBlendHint{ 0xFFFFFFFFu };
            int weaponParentIndex{ -1 };
            int animationType{ -1 };
            int transformTrackCount{ 0 };
            int floatTrackCount{ -1 };
            int mappingCount{ 0 };
            float animationDurationSeconds{ -1.0f };
            std::uint16_t sampledFingerMask{ 0 };
            std::uint16_t referenceFingerMask{ 0 };
            std::uint16_t missingFingerMask{ authored_weapon_grip_library::kCompleteFiringFingerMask };
            bool usedGraphClipPathFallback{ false };
            bool graphIdlePathAmbiguous{ false };
        };

        struct Job
        {
            alignas(16) std::array<std::byte, kSimpleAnimationGraphManagerHolderSize> graphHolderStorage{};
            RE::ObjectRefHandle reference{};
            RE::BSTSmartPointer<RE::TBO_InstanceData> instanceData{};
            RE::BSTSmallArray<RE::SubgraphHandle, 2> subgraphHandles{};
            RE::BSTSmallArray<RE::SubgraphIdentifier, 2> subgraphIdentifiers{};
            AnimationResourceHandle idleClipResource{};
            std::array<char, 260> idleClipPath{};
            RE::TESObjectWEAP* weapon{ nullptr }; // Stable loaded-form identity; never owns the form.
            RE::TESRace* race{ nullptr }; // Stable loaded-form identity; never owns the form.
            authored_weapon_grip_library::WeaponVariantIdentity variant{};
            std::uintptr_t instanceIdentity{ 0 };
            std::uint32_t referenceFormId{ 0 };
            std::uint32_t weaponFormId{ 0 };
            std::uint32_t idleClipResourceState{ 0xFFFFFFFFu };
            ULONGLONG startedAtMilliseconds{ 0 };
            Phase phase{ Phase::Idle };
            bool inPowerArmor{ false };
            bool graphHolderConstructed{ false };
            bool longLoadLogged{ false };
            CandidateOrigin origin{ CandidateOrigin::Unknown };
        };

        struct FailureEntry
        {
            std::uint32_t weaponFormId{ 0 };
            std::uint64_t variantKey{ 0 };
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
                validateNativeEntry("LoadIdle", kLoadIdleAnimationResource,
                    std::array<std::uint8_t, 9>{ 0x48, 0x89, 0x5C, 0x24, 0x18, 0x56, 0x57, 0x41, 0x56 }) &&
                validateNativeEntry("BShkbHkxDB resource-handle move assignment", kMoveAnimationResourceHandle,
                    std::array<std::uint8_t, 10>{ 0x48, 0x89, 0x5C, 0x24, 0x10, 0x57, 0x48, 0x83, 0xEC, 0x20 }) &&
                validateNativeEntry("BShkbHkxDBUtils::IsHkxDerivativeDBData", kIsHkxDerivativeDbData,
                    std::array<std::uint8_t, 10>{ 0x48, 0x83, 0xEC, 0x28, 0x48, 0x8B, 0x01, 0xFF, 0x50, 0x08 }) &&
                validateNativeEntry("BShkbUtils::RetrieveBindingFromContainer", kRetrieveBindingFromContainer,
                    std::array<std::uint8_t, 10>{ 0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x20 }) &&
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
            state.native.loadIdleAnimationResource = reinterpret_cast<LoadIdleAnimationResourceFn>(REL::Offset(kLoadIdleAnimationResource).address());
            state.native.moveAnimationResourceHandle = reinterpret_cast<MoveAnimationResourceHandleFn>(REL::Offset(kMoveAnimationResourceHandle).address());
            state.native.isHkxDerivativeDbData = reinterpret_cast<IsHkxDerivativeDbDataFn>(REL::Offset(kIsHkxDerivativeDbData).address());
            state.native.retrieveBindingFromContainer = reinterpret_cast<RetrieveBindingFromContainerFn>(REL::Offset(kRetrieveBindingFromContainer).address());
            state.native.findBoneWithName = reinterpret_cast<FindBoneWithNameFn>(REL::Offset(kFindBoneWithName).address());
            state.nativeValidated = true;
            ROCK_LOG_INFO(Init,
                "Native idle-grip preharvest validated: plain off-screen graph holder, exact first-person subgraph, direct idle resource fallback, and clip sampler ready");
            return true;
        }

        [[nodiscard]] RE::SimpleAnimationGraphManagerHolder* graphHolder(Job& job)
        {
            if (!job.graphHolderConstructed) {
                return nullptr;
            }
            return reinterpret_cast<RE::SimpleAnimationGraphManagerHolder*>(job.graphHolderStorage.data());
        }

        template <class T>
        [[nodiscard]] bool prepareNativeSubgraphOutput(RE::BSTSmallArray<T, kSubgraphOutputInlineCapacity>& output)
        {
            static_assert(sizeof(T) == sizeof(std::uint64_t));
            if (!output.empty()) {
                return false;
            }

            // Bethesda initializes an empty BSTSmallArray with its inline-storage bit set.
            // CommonLibF4VR leaves a default array in null heap mode, so reserve and verify
            // the embedded two-element buffer before exposing the object to native code.
            output.reserve(kSubgraphOutputInlineCapacity);
            const auto* expectedInlineData = reinterpret_cast<const std::byte*>(&output) + kSmallArrayInlineStorageOffset;
            return output.capacity() == kSubgraphOutputInlineCapacity &&
                   static_cast<const void*>(output.data()) == static_cast<const void*>(expectedInlineData);
        }

        [[nodiscard]] const char* extractionFailureName(const IdleGripExtractionFailure failure)
        {
            switch (failure) {
            case IdleGripExtractionFailure::None:
                return "none";
            case IdleGripExtractionFailure::FirstPersonGraphPairUnavailable:
                return "firstPersonGraphPairUnavailable";
            case IdleGripExtractionFailure::FirstPersonGraphUnavailable:
                return "firstPersonGraphUnavailable";
            case IdleGripExtractionFailure::FirstPersonSubgraphHandleUnavailable:
                return "firstPersonSubgraphHandleUnavailable";
            case IdleGripExtractionFailure::FirstPersonSubgraphIdentifierUnavailable:
                return "firstPersonSubgraphIdentifierUnavailable";
            case IdleGripExtractionFailure::AnimationFileLookupUnavailable:
                return "animationFileLookupUnavailable";
            case IdleGripExtractionFailure::AnimationFileListUnavailable:
                return "animationFileListUnavailable";
            case IdleGripExtractionFailure::AnimationFileListEmpty:
                return "animationFileListEmpty";
            case IdleGripExtractionFailure::IdleClipPathUnavailable:
                return "idleClipPathUnavailable";
            case IdleGripExtractionFailure::IdleClipPathTooLong:
                return "idleClipPathTooLong";
            case IdleGripExtractionFailure::BehaviorGraphSwapSingletonUnavailable:
                return "behaviorGraphSwapSingletonUnavailable";
            case IdleGripExtractionFailure::ClipBindingUnavailable:
                return "clipBindingUnavailable";
            case IdleGripExtractionFailure::DirectClipLoadRequestFailed:
                return "directClipLoadRequestFailed";
            case IdleGripExtractionFailure::DirectClipPathMismatch:
                return "directClipPathMismatch";
            case IdleGripExtractionFailure::DirectClipResourceLayoutUnavailable:
                return "directClipResourceLayoutUnavailable";
            case IdleGripExtractionFailure::DirectClipDataInvalid:
                return "directClipDataInvalid";
            case IdleGripExtractionFailure::DirectClipContainerUnavailable:
                return "directClipContainerUnavailable";
            case IdleGripExtractionFailure::DirectClipBindingUnavailable:
                return "directClipBindingUnavailable";
            case IdleGripExtractionFailure::AnimationBindingUnavailable:
                return "animationBindingUnavailable";
            case IdleGripExtractionFailure::BoundAnimationUnavailable:
                return "boundAnimationUnavailable";
            case IdleGripExtractionFailure::TransformTrackCountInvalid:
                return "transformTrackCountInvalid";
            case IdleGripExtractionFailure::GraphSkeletonUnavailable:
                return "graphSkeletonUnavailable";
            case IdleGripExtractionFailure::SkeletonLayoutInvalid:
                return "skeletonLayoutInvalid";
            case IdleGripExtractionFailure::RequiredBoneUnavailable:
                return "requiredBoneUnavailable";
            case IdleGripExtractionFailure::SkeletonParentCopyUnavailable:
                return "skeletonParentCopyUnavailable";
            case IdleGripExtractionFailure::WeaponNotDirectChildOfHand:
                return "weaponNotDirectChildOfHand";
            case IdleGripExtractionFailure::TrackMappingLayoutInvalid:
                return "trackMappingLayoutInvalid";
            case IdleGripExtractionFailure::TrackMappingCopyUnavailable:
                return "trackMappingCopyUnavailable";
            case IdleGripExtractionFailure::WeaponTrackUnavailable:
                return "weaponTrackUnavailable";
            case IdleGripExtractionFailure::AnimationSamplerUnavailable:
                return "animationSamplerUnavailable";
            case IdleGripExtractionFailure::AnimationSamplingFault:
                return "animationSamplingFault";
            case IdleGripExtractionFailure::SampledWeaponTransformInvalid:
                return "sampledWeaponTransformInvalid";
            case IdleGripExtractionFailure::IncompleteFiringFingerPose:
                return "incompleteFiringFingerPose";
            }
            return "unknownExtractionFailure";
        }

        [[nodiscard]] bool failExtraction(IdleGripExtractionDiagnostics& diagnostics, const IdleGripExtractionFailure failure)
        {
            diagnostics.failure = failure;
            return false;
        }

        [[nodiscard]] ExtractionResult failExtractionResult(IdleGripExtractionDiagnostics& diagnostics, const IdleGripExtractionFailure failure)
        {
            diagnostics.failure = failure;
            return ExtractionResult::Failed;
        }

        [[nodiscard]] bool sameFailureIdentity(const FailureEntry& entry, const Job& job)
        {
            return entry.occupied &&
                   entry.weaponFormId == job.weaponFormId &&
                   entry.variantKey == job.variant.key &&
                   entry.inPowerArmor == job.inPowerArmor;
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
                .weaponFormId = job.weaponFormId,
                .variantKey = job.variant.key,
                .retryAfterMilliseconds = now + kFailureRetryDelayMilliseconds,
                .inPowerArmor = job.inPowerArmor,
                .occupied = true,
            };
        }

        void releaseJob(Runtime& state)
        {
            auto& job = state.job;
            if (job.idleClipResource.entry) {
                // Bethesda's own RHandleType move assignment releases the old
                // BShkbHkxDB entry through the correct deferred-release queue.
                AnimationResourceHandle empty{};
                state.native.moveAnimationResourceHandle(&job.idleClipResource, &empty);
            }
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
            ROCK_LOG_WARN(Animation, "Native idle-grip preharvest failed formID={:08X} refID={:08X} variant={:016X} origin={} powerArmor={} phase={} reason={}",
                state.job.weaponFormId,
                state.job.referenceFormId,
                state.job.variant.key,
                state.job.origin == CandidateOrigin::EquippedWeapon ? "equipped" : "loose",
                state.job.inPowerArmor ? "yes" : "no",
                static_cast<unsigned>(state.job.phase),
                reason ? reason : "unknown");
            recordFailure(state, state.job, GetTickCount64());
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

        [[nodiscard]] bool copyBorrowedFixedString(const void* stringEntry, std::array<char, 260>& outPath)
        {
            outPath.fill('\0');
            const void* current = stringEntry;
            for (std::size_t depth = 0; depth < kMaxFixedStringShallowDepth; ++depth) {
                std::uint16_t flags = 0;
                if (!native_memory::tryReadField(current, 0x08, flags)) {
                    return false;
                }
                if ((flags & RE::BSStringPool::Entry::kShallow) != 0) {
                    void* right = nullptr;
                    if (!native_memory::tryReadField(current, 0x10, right) || !right) {
                        return false;
                    }
                    current = right;
                    continue;
                }
                if ((flags & RE::BSStringPool::Entry::kWide) != 0) {
                    return false;
                }

                std::uint32_t length = 0;
                if (!native_memory::tryReadField(current, 0x10, length) || length == 0 || length >= outPath.size()) {
                    return false;
                }
                const auto* characters = reinterpret_cast<const char*>(current) + sizeof(RE::BSStringPool::Entry);
                if (!native_memory::guardedCopyFromMemory(characters, outPath.data(), static_cast<std::size_t>(length) + 1)) {
                    return false;
                }
                return outPath[length] == '\0';
            }
            return false;
        }

        [[nodiscard]] bool tryFindLoadedGraphIdlePath(RE::BShkbAnimationGraph* graph, const std::uint64_t subgraphHandle,
            std::uint64_t& outBindingSubgraphIdentifier, std::array<char, 260>& outPath, IdleGripExtractionDiagnostics& diagnostics)
        {
            outPath.fill('\0');
            outBindingSubgraphIdentifier = 0;
            if (subgraphHandle == 0) {
                return false;
            }

            void* loadedSubgraphs = nullptr;
            if (!native_memory::tryReadField(graph, kGraphLoadedSubgraphsOffset, loadedSubgraphs) || !loadedSubgraphs) {
                return false;
            }

            RE::BSSpinLock* graphLock = nullptr;
            if (!native_memory::tryReadField(loadedSubgraphs, kLoadedSubgraphsLockOffset, graphLock) || !graphLock ||
                !native_memory::pointerRangeLooksWritable(graphLock, sizeof(*graphLock))) {
                return false;
            }

            RE::BSAutoLock<RE::BSSpinLock> lock{ graphLock };
            void* entries = nullptr;
            std::uint32_t entryCount = 0;
            if (!native_memory::tryReadField(loadedSubgraphs, kLoadedSubgraphsEntriesOffset, entries) || !entries ||
                !native_memory::tryReadField(loadedSubgraphs, kLoadedSubgraphsCountOffset, entryCount) || entryCount == 0 || entryCount > kMaxLoadedSubgraphs) {
                return false;
            }
            diagnostics.loadedSubgraphCount = entryCount;

            void* bindingTable = nullptr;
            for (std::uint32_t index = 0; index < entryCount; ++index) {
                const auto* entry = reinterpret_cast<const std::byte*>(entries) + static_cast<std::size_t>(index) * kLoadedSubgraphEntryStride;
                std::uint64_t candidateHandle = 0;
                if (!native_memory::tryReadField(entry, kLoadedSubgraphHandleOffset, candidateHandle) || candidateHandle != subgraphHandle) {
                    continue;
                }
                ++diagnostics.graphHandleMatchCount;

                void* candidateTable = nullptr;
                std::uint64_t candidateIdentifier = 0;
                if (!native_memory::tryReadField(entry, kLoadedSubgraphBindingTableOffset, candidateTable) || !candidateTable ||
                    !native_memory::tryReadField(candidateTable, kBindingTableSubgraphIdentifierOffset, candidateIdentifier)) {
                    continue;
                }
                bindingTable = candidateTable;
                outBindingSubgraphIdentifier = candidateIdentifier;
            }
            if (diagnostics.graphHandleMatchCount != 1 || !bindingTable || outBindingSubgraphIdentifier == 0) {
                return false;
            }
            diagnostics.bindingSubgraphIdentifier = outBindingSubgraphIdentifier;

            void* buckets = nullptr;
            std::uint32_t bucketCount = 0;
            if (!native_memory::tryReadField(bindingTable, kBindingTableBucketsOffset, buckets) || !buckets ||
                !native_memory::tryReadField(bindingTable, kBindingTableBucketCountOffset, bucketCount) || bucketCount == 0 || bucketCount > kMaxClipBindingBuckets) {
                return false;
            }
            diagnostics.graphClipBucketCount = bucketCount;

            auto selectedPriority = native_idle_grip_preharvest_policy::IdleClipPriority::None;
            bool ambiguous = false;
            for (std::uint32_t index = 0; index < bucketCount; ++index) {
                const auto* node = reinterpret_cast<const std::byte*>(buckets) + static_cast<std::size_t>(index) * kBindingTableNodeStride;
                void* occupancy = nullptr;
                if (!native_memory::tryReadField(node, kBindingTableNodeOccupancyOffset, occupancy) || !occupancy) {
                    continue;
                }

                void* stringEntry = nullptr;
                std::array<char, 260> candidatePath{};
                if (!native_memory::tryReadField(node, kBindingTableNodeKeyOffset, stringEntry) || !stringEntry || !copyBorrowedFixedString(stringEntry, candidatePath)) {
                    continue;
                }
                ++diagnostics.graphClipPathCount;
                const std::string_view candidate{ candidatePath.data() };
                const auto candidatePriority = native_idle_grip_preharvest_policy::idleClipPriority(candidate);
                if (candidatePriority == native_idle_grip_preharvest_policy::IdleClipPriority::None) {
                    continue;
                }
                ++diagnostics.graphIdlePathCandidateCount;

                if (static_cast<std::uint8_t>(candidatePriority) > static_cast<std::uint8_t>(selectedPriority)) {
                    outPath = candidatePath;
                    selectedPriority = candidatePriority;
                    ambiguous = false;
                    continue;
                }
                if (candidatePriority == selectedPriority &&
                    !native_idle_grip_preharvest_policy::sameClipPath(std::string_view{ outPath.data() }, candidate)) {
                    ambiguous = true;
                }
            }

            diagnostics.graphIdlePathAmbiguous = ambiguous;
            if (selectedPriority == native_idle_grip_preharvest_policy::IdleClipPriority::None || ambiguous) {
                outPath.fill('\0');
                outBindingSubgraphIdentifier = 0;
                return false;
            }
            diagnostics.usedGraphClipPathFallback = true;
            return true;
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

        [[nodiscard]] bool convertHavokLocalTransform(const HkQsTransform& sampledLocal, RE::NiTransform& outLocal)
        {
            for (const float value : sampledLocal.translation) {
                if (!std::isfinite(value)) {
                    return false;
                }
            }
            float quaternionNormSquared = 0.0f;
            for (const float value : sampledLocal.rotation) {
                if (!std::isfinite(value)) {
                    return false;
                }
                quaternionNormSquared += value * value;
            }
            if (quaternionNormSquared < 0.000001f) {
                return false;
            }
            for (const float value : sampledLocal.scale) {
                if (!std::isfinite(value)) {
                    return false;
                }
            }
            if (std::abs(sampledLocal.scale[0]) < 0.000001f || std::abs(sampledLocal.scale[0] - sampledLocal.scale[1]) > 0.001f ||
                std::abs(sampledLocal.scale[0] - sampledLocal.scale[2]) > 0.001f) {
                return false;
            }

            outLocal.translate = RE::NiPoint3{
                sampledLocal.translation[0],
                sampledLocal.translation[1],
                sampledLocal.translation[2],
            };
            // hkaAnimation's quaternion matrix is the opposite stored-axis
            // convention from ROCK's NiTransform relationships. Convert it at
            // this boundary for both the Weapon and finger-bone locals.
            outLocal.rotate = transform_math::transposeRotation(transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(sampledLocal.rotation));
            outLocal.scale = sampledLocal.scale[0];
            return isFiniteTransform(outLocal);
        }

        [[nodiscard]] bool convertWeaponTrackToHandInWeapon(const HkQsTransform& sampledWeaponLocal, RE::NiTransform& outHandInWeapon)
        {
            RE::NiTransform weaponLocal{};
            if (!convertHavokLocalTransform(sampledWeaponLocal, weaponLocal)) {
                return false;
            }

            outHandInWeapon = transform_math::invertTransform(weaponLocal);
            return isFiniteTransform(outHandInWeapon);
        }

        void extractRightFiringFingerPose(Runtime& state, void* skeleton, const int boneCount, const int transformTrackCount, const std::span<const std::int16_t> mapping,
            const std::array<HkQsTransform, kMaxBonesAndTracks>& sampledTracks, authored_weapon_grip_library::FiringFingerPose& outPose, IdleGripExtractionDiagnostics& diagnostics)
        {
            outPose = {};
            diagnostics.sampledFingerMask = 0;
            diagnostics.referenceFingerMask = 0;
            diagnostics.missingFingerMask = authored_weapon_grip_library::kCompleteFiringFingerMask;

            const HkQsTransform* referencePose = nullptr;
            int referencePoseCount = 0;
            const bool referencePoseAvailable = native_memory::tryReadField(skeleton, kSkeletonReferencePoseOffset, referencePose) &&
                native_memory::tryReadField(skeleton, kSkeletonReferencePoseCountOffset, referencePoseCount) && referencePose && referencePoseCount >= boneCount &&
                referencePoseCount <= static_cast<int>(kMaxBonesAndTracks);

            for (std::size_t fingerIndex = 0; fingerIndex < kRightFiringFingerBoneNames.size(); ++fingerIndex) {
                const std::uint16_t bit = static_cast<std::uint16_t>(1U << fingerIndex);
                const std::uint64_t boneRaw = state.native.findBoneWithName(skeleton, kRightFiringFingerBoneNames[fingerIndex], nullptr);
                if (boneRaw == 0xFFFFFFFFull || boneRaw >= static_cast<std::uint64_t>(boneCount)) {
                    continue;
                }

                const int boneIndex = static_cast<int>(boneRaw);
                const int trackIndex = native_idle_grip_preharvest_policy::findTransformTrackForBone(boneIndex, transformTrackCount, mapping);
                HkQsTransform local{};
                bool localAvailable = false;
                if (trackIndex >= 0) {
                    local = sampledTracks[static_cast<std::size_t>(trackIndex)];
                    diagnostics.sampledFingerMask = static_cast<std::uint16_t>(diagnostics.sampledFingerMask | bit);
                    localAvailable = true;
                } else if (referencePoseAvailable && boneIndex < referencePoseCount && native_memory::guardedCopyFromMemory(referencePose + boneIndex, &local, sizeof(local))) {
                    diagnostics.referenceFingerMask = static_cast<std::uint16_t>(diagnostics.referenceFingerMask | bit);
                    localAvailable = true;
                }

                RE::NiTransform converted{};
                if (!localAvailable || !convertHavokLocalTransform(local, converted)) {
                    diagnostics.sampledFingerMask = static_cast<std::uint16_t>(diagnostics.sampledFingerMask & ~bit);
                    diagnostics.referenceFingerMask = static_cast<std::uint16_t>(diagnostics.referenceFingerMask & ~bit);
                    continue;
                }

                outPose.localTransforms[fingerIndex] = converted;
                outPose.enabledMask = static_cast<std::uint16_t>(outPose.enabledMask | bit);
            }

            diagnostics.missingFingerMask = static_cast<std::uint16_t>(authored_weapon_grip_library::kCompleteFiringFingerMask & ~outPose.enabledMask);
        }

        [[nodiscard]] bool trySampleAnimationBinding(Runtime& state, RE::BShkbAnimationGraph* graph, void* binding, RE::NiTransform& outHandInWeapon,
            authored_weapon_grip_library::FiringFingerPose& outRightFiringFingerPose, IdleGripExtractionDiagnostics& diagnostics)
        {
            outRightFiringFingerPose = {};
            diagnostics.weaponBone = 0xFFFFFFFFull;
            diagnostics.handBone = 0xFFFFFFFFull;
            diagnostics.weaponParentIndex = -1;
            diagnostics.animationType = -1;
            diagnostics.transformTrackCount = 0;
            diagnostics.floatTrackCount = -1;
            diagnostics.mappingCount = 0;
            diagnostics.animationDurationSeconds = -1.0f;
            diagnostics.bindingBlendHint = 0xFFFFFFFFu;

            void* animation = nullptr;
            if (!native_memory::tryReadField(binding, kAnimationFromBindingOffset, animation) || !animation) {
                return failExtraction(diagnostics, IdleGripExtractionFailure::BoundAnimationUnavailable);
            }

            std::uint8_t bindingBlendHint = 0;
            if (native_memory::tryReadField(binding, kBindingBlendHintOffset, bindingBlendHint)) {
                diagnostics.bindingBlendHint = bindingBlendHint;
            }
            (void)native_memory::tryReadField(animation, kAnimationTypeOffset, diagnostics.animationType);
            (void)native_memory::tryReadField(animation, kAnimationDurationOffset, diagnostics.animationDurationSeconds);
            (void)native_memory::tryReadField(animation, kAnimationFloatTrackCountOffset, diagnostics.floatTrackCount);

            int transformTrackCount = 0;
            if (!native_memory::tryReadField(animation, kAnimationTransformTrackCountOffset, transformTrackCount)) {
                return failExtraction(diagnostics, IdleGripExtractionFailure::TransformTrackCountInvalid);
            }
            diagnostics.transformTrackCount = transformTrackCount;
            if (transformTrackCount <= 0 || transformTrackCount > static_cast<int>(kMaxBonesAndTracks)) {
                return failExtraction(diagnostics, IdleGripExtractionFailure::TransformTrackCountInvalid);
            }

            void* skeletonOwner = nullptr;
            void* skeleton = nullptr;
            if (!native_memory::tryReadField(graph, kGraphSkeletonOwnerOffset, skeletonOwner) || !skeletonOwner ||
                !native_memory::tryReadField(skeletonOwner, kSkeletonFromOwnerOffset, skeleton) || !skeleton) {
                return failExtraction(diagnostics, IdleGripExtractionFailure::GraphSkeletonUnavailable);
            }

            int boneCount = 0;
            int parentCount = 0;
            const std::int16_t* parentIndices = nullptr;
            if (!native_memory::tryReadField(skeleton, kSkeletonBoneCountOffset, boneCount) || !native_memory::tryReadField(skeleton, kSkeletonParentCountOffset, parentCount) ||
                !native_memory::tryReadField(skeleton, kSkeletonParentIndicesOffset, parentIndices) || boneCount <= 0 || boneCount > static_cast<int>(kMaxBonesAndTracks) ||
                parentCount < boneCount || !parentIndices) {
                return failExtraction(diagnostics, IdleGripExtractionFailure::SkeletonLayoutInvalid);
            }

            const auto weaponBoneRaw = state.native.findBoneWithName(skeleton, "Weapon", nullptr);
            const auto handBoneRaw = state.native.findBoneWithName(skeleton, "RArm_Hand", nullptr);
            diagnostics.weaponBone = weaponBoneRaw;
            diagnostics.handBone = handBoneRaw;
            if (weaponBoneRaw == 0xFFFFFFFFull || handBoneRaw == 0xFFFFFFFFull || weaponBoneRaw >= static_cast<std::uint64_t>(boneCount) ||
                handBoneRaw >= static_cast<std::uint64_t>(boneCount)) {
                return failExtraction(diagnostics, IdleGripExtractionFailure::RequiredBoneUnavailable);
            }
            const int weaponBoneIndex = static_cast<int>(weaponBoneRaw);
            const int handBoneIndex = static_cast<int>(handBoneRaw);

            std::array<std::int16_t, kMaxBonesAndTracks> parentBuffer{};
            if (!native_memory::guardedCopyFromMemory(parentIndices, parentBuffer.data(), static_cast<std::size_t>(boneCount) * sizeof(std::int16_t))) {
                return failExtraction(diagnostics, IdleGripExtractionFailure::SkeletonParentCopyUnavailable);
            }
            diagnostics.weaponParentIndex = parentBuffer[static_cast<std::size_t>(weaponBoneIndex)];
            if (!native_idle_grip_preharvest_policy::weaponIsDirectChildOfHand(weaponBoneIndex, handBoneIndex,
                    std::span<const std::int16_t>{ parentBuffer.data(), static_cast<std::size_t>(boneCount) })) {
                return failExtraction(diagnostics, IdleGripExtractionFailure::WeaponNotDirectChildOfHand);
            }

            const std::int16_t* trackToBoneIndices = nullptr;
            int mappingCount = 0;
            if (!native_memory::tryReadField(binding, kTrackToBoneMappingOffset, trackToBoneIndices) ||
                !native_memory::tryReadField(binding, kTrackToBoneMappingCountOffset, mappingCount) || mappingCount < 0 || mappingCount > static_cast<int>(kMaxBonesAndTracks)) {
                return failExtraction(diagnostics, IdleGripExtractionFailure::TrackMappingLayoutInvalid);
            }
            diagnostics.mappingCount = mappingCount;

            std::array<std::int16_t, kMaxBonesAndTracks> mappingBuffer{};
            std::span<const std::int16_t> mapping{};
            if (mappingCount > 0) {
                if (!trackToBoneIndices ||
                    !native_memory::guardedCopyFromMemory(trackToBoneIndices, mappingBuffer.data(), static_cast<std::size_t>(mappingCount) * sizeof(std::int16_t))) {
                    return failExtraction(diagnostics, IdleGripExtractionFailure::TrackMappingCopyUnavailable);
                }
                mapping = std::span<const std::int16_t>{
                    mappingBuffer.data(),
                    static_cast<std::size_t>(mappingCount),
                };
            }
            const int weaponTrackIndex = native_idle_grip_preharvest_policy::findTransformTrackForBone(weaponBoneIndex, transformTrackCount, mapping);
            if (weaponTrackIndex < 0) {
                return failExtraction(diagnostics, IdleGripExtractionFailure::WeaponTrackUnavailable);
            }

            void** animationVtable = nullptr;
            SampleAnimationTracksFn sampleTracks = nullptr;
            if (!native_memory::tryReadValue(reinterpret_cast<void***>(animation), animationVtable) || !animationVtable ||
                !native_memory::tryReadValue(reinterpret_cast<SampleAnimationTracksFn*>(animationVtable + 5), sampleTracks) ||
                !addressIsExecutable(reinterpret_cast<const void*>(sampleTracks))) {
                return failExtraction(diagnostics, IdleGripExtractionFailure::AnimationSamplerUnavailable);
            }

            alignas(16) std::array<HkQsTransform, kMaxBonesAndTracks> sampledTracks{};
            if (!guardedSampleTracks(sampleTracks, animation, transformTrackCount, sampledTracks.data())) {
                return failExtraction(diagnostics, IdleGripExtractionFailure::AnimationSamplingFault);
            }
            if (!convertWeaponTrackToHandInWeapon(sampledTracks[static_cast<std::size_t>(weaponTrackIndex)], outHandInWeapon)) {
                return failExtraction(diagnostics, IdleGripExtractionFailure::SampledWeaponTransformInvalid);
            }
            extractRightFiringFingerPose(state, skeleton, boneCount, transformTrackCount, mapping, sampledTracks, outRightFiringFingerPose, diagnostics);
            if (!outRightFiringFingerPose.complete()) {
                return failExtraction(diagnostics, IdleGripExtractionFailure::IncompleteFiringFingerPose);
            }
            diagnostics.failure = IdleGripExtractionFailure::None;
            return true;
        }

        [[nodiscard]] bool guardedIsHkxDerivativeDbData(const IsHkxDerivativeDbDataFn isHkxDerivativeDbData, void* animationData, bool& outIsDerivative) noexcept
        {
#if defined(_MSC_VER)
            __try {
                outIsDerivative = isHkxDerivativeDbData(animationData);
                return true;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
#else
            outIsDerivative = isHkxDerivativeDbData(animationData);
            return true;
#endif
        }

        [[nodiscard]] bool guardedRetrieveBindingFromContainer(const RetrieveBindingFromContainerFn retrieveBindingFromContainer, void* rootContainer, void*& outBinding,
            char* clipName) noexcept
        {
#if defined(_MSC_VER)
            __try {
                retrieveBindingFromContainer(rootContainer, &outBinding, clipName);
                return true;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
#else
            retrieveBindingFromContainer(rootContainer, &outBinding, clipName);
            return true;
#endif
        }

        [[nodiscard]] ExtractionResult trySampleClip(Runtime& state, RE::BShkbAnimationGraph* graph, const std::uint64_t subgraphIdentifier, char* clipName,
            RE::NiTransform& outHandInWeapon, authored_weapon_grip_library::FiringFingerPose& outRightFiringFingerPose, IdleGripExtractionDiagnostics& diagnostics)
        {
            diagnostics.bindingSubgraphIdentifier = subgraphIdentifier;
            void* swapSingleton = nullptr;
            const auto singletonAddress = REL::Offset(kBehaviorGraphSwapSingleton).address();
            if (!native_memory::tryReadValue(reinterpret_cast<void* const*>(singletonAddress), swapSingleton) || !swapSingleton) {
                return failExtractionResult(diagnostics, IdleGripExtractionFailure::BehaviorGraphSwapSingletonUnavailable);
            }

            if (void* bindingWithTriggers = state.native.getClipGeneratorBinding(swapSingleton, graph, subgraphIdentifier, clipName)) {
                void* binding = nullptr;
                if (!native_memory::tryReadField(bindingWithTriggers, kBindingFromBindingWithTriggersOffset, binding) || !binding) {
                    return failExtractionResult(diagnostics, IdleGripExtractionFailure::AnimationBindingUnavailable);
                }
                return trySampleAnimationBinding(state, graph, binding, outHandInWeapon, outRightFiringFingerPose, diagnostics) ? ExtractionResult::Succeeded : ExtractionResult::Failed;
            }

            auto& job = state.job;
            if (!job.idleClipResource.entry) {
                const std::string_view path{ clipName ? clipName : "" };
                if (path.empty() || path.size() >= job.idleClipPath.size()) {
                    return failExtractionResult(diagnostics, IdleGripExtractionFailure::IdleClipPathTooLong);
                }
                job.idleClipPath.fill('\0');
                std::memcpy(job.idleClipPath.data(), path.data(), path.size());

                RE::BSFixedString idlePath{ clipName };
                if (!state.native.loadIdleAnimationResource(&idlePath, &job.idleClipResource) || !job.idleClipResource.entry) {
                    return failExtractionResult(diagnostics, IdleGripExtractionFailure::DirectClipLoadRequestFailed);
                }

                job.phase = Phase::IdleClipLoading;
                job.startedAtMilliseconds = GetTickCount64();
                job.longLoadLogged = false;
                job.idleClipResourceState = 0xFFFFFFFFu;
                diagnostics.failure = IdleGripExtractionFailure::ClipBindingUnavailable;
                ROCK_LOG_INFO(Animation, "Native idle-grip preharvest requested direct idle resource formID={:08X} clip={}", job.weaponFormId, job.idleClipPath.data());
                return ExtractionResult::Pending;
            }

            if (!clipName || std::string_view{ clipName } != std::string_view{ job.idleClipPath.data() }) {
                return failExtractionResult(diagnostics, IdleGripExtractionFailure::DirectClipPathMismatch);
            }

            std::uint32_t resourceFlags = 0;
            if (!native_memory::tryReadField(job.idleClipResource.entry, kAnimationResourceFlagsOffset, resourceFlags)) {
                return failExtractionResult(diagnostics, IdleGripExtractionFailure::DirectClipResourceLayoutUnavailable);
            }
            job.idleClipResourceState = native_idle_grip_preharvest_policy::animationResourceState(resourceFlags);
            diagnostics.directResourceState = job.idleClipResourceState;
            if (!native_idle_grip_preharvest_policy::animationResourceCanExposeData(resourceFlags)) {
                return ExtractionResult::Pending;
            }

            void* animationData = nullptr;
            if (!native_memory::tryReadField(job.idleClipResource.entry, kAnimationResourceDataOffset, animationData) || !animationData) {
                return failExtractionResult(diagnostics, IdleGripExtractionFailure::DirectClipResourceLayoutUnavailable);
            }

            bool isHkxDerivative = false;
            if (!guardedIsHkxDerivativeDbData(state.native.isHkxDerivativeDbData, animationData, isHkxDerivative) || !isHkxDerivative) {
                return failExtractionResult(diagnostics, IdleGripExtractionFailure::DirectClipDataInvalid);
            }

            void* rootContainer = nullptr;
            if (!native_memory::tryReadField(animationData, kRootContainerFromAnimationDataOffset, rootContainer) || !rootContainer) {
                return failExtractionResult(diagnostics, IdleGripExtractionFailure::DirectClipContainerUnavailable);
            }

            void* binding = nullptr;
            if (!guardedRetrieveBindingFromContainer(state.native.retrieveBindingFromContainer, rootContainer, binding, clipName) || !binding) {
                return failExtractionResult(diagnostics, IdleGripExtractionFailure::DirectClipBindingUnavailable);
            }

            return trySampleAnimationBinding(state, graph, binding, outHandInWeapon, outRightFiringFingerPose, diagnostics) ? ExtractionResult::Succeeded : ExtractionResult::Failed;
        }

        [[nodiscard]] ExtractionResult tryExtractIdleGrip(Runtime& state, RE::BSAnimationGraphManager& manager, RE::NiTransform& outHandInWeapon,
            authored_weapon_grip_library::FiringFingerPose& outRightFiringFingerPose,
            std::array<char, 260>& outClipPath,
            std::uint64_t& outSubgraphIdentifier, IdleGripExtractionDiagnostics& diagnostics)
        {
            diagnostics = {};
            diagnostics.graphCount = manager.graph.size();
            diagnostics.handleCount = state.job.subgraphHandles.size();
            diagnostics.identifierCount = state.job.subgraphIdentifiers.size();
            outClipPath.fill('\0');

            const auto selection = native_idle_grip_preharvest_policy::selectFirstPersonGraph(
                manager.graph.size(), state.job.subgraphHandles.size(), state.job.subgraphIdentifiers.size());
            if (!selection.valid) {
                return failExtractionResult(diagnostics, IdleGripExtractionFailure::FirstPersonGraphPairUnavailable);
            }
            const auto firstPersonIndex = static_cast<decltype(manager.graph)::size_type>(selection.graphIndex);
            auto* graph = manager.graph[firstPersonIndex].get();
            if (!graph) {
                return failExtractionResult(diagnostics, IdleGripExtractionFailure::FirstPersonGraphUnavailable);
            }
            const auto firstPersonOutputIndex = static_cast<decltype(state.job.subgraphHandles)::size_type>(selection.graphIndex);
            const std::uint64_t subgraphHandle = state.job.subgraphHandles[firstPersonOutputIndex].handle;
            diagnostics.subgraphHandle = subgraphHandle;
            if (subgraphHandle == 0) {
                return failExtractionResult(diagnostics, IdleGripExtractionFailure::FirstPersonSubgraphHandleUnavailable);
            }
            outSubgraphIdentifier = state.job.subgraphIdentifiers[static_cast<decltype(state.job.subgraphIdentifiers)::size_type>(selection.graphIndex)].identifier;
            diagnostics.subgraphIdentifier = outSubgraphIdentifier;
            if (outSubgraphIdentifier == 0) {
                return failExtractionResult(diagnostics, IdleGripExtractionFailure::FirstPersonSubgraphIdentifierUnavailable);
            }

            void* lookupSingleton = nullptr;
            const auto lookupSingletonAddress = REL::Offset(kAnimationFileLookupSingleton).address();
            if (!native_memory::tryReadValue(reinterpret_cast<void* const*>(lookupSingletonAddress), lookupSingleton) || !lookupSingleton) {
                return failExtractionResult(diagnostics, IdleGripExtractionFailure::AnimationFileLookupUnavailable);
            }
            const auto* animationFiles = state.native.getAnimationFilesForSubgraph(&outSubgraphIdentifier);
            if (!animationFiles) {
                return failExtractionResult(diagnostics, IdleGripExtractionFailure::AnimationFileListUnavailable);
            }
            diagnostics.animationFileCount = animationFiles->size();

            const auto tryGraphPathFallback = [&](const IdleGripExtractionFailure unavailableFailure) {
                std::uint64_t bindingSubgraphIdentifier = 0;
                if (!tryFindLoadedGraphIdlePath(graph, subgraphHandle, bindingSubgraphIdentifier, outClipPath, diagnostics)) {
                    return failExtractionResult(diagnostics, unavailableFailure);
                }
                ++diagnostics.idlePathMatchCount;
                ++diagnostics.sampleAttemptCount;
                if (!state.job.idleClipResource.entry) {
                    ROCK_LOG_INFO(Animation,
                        "Native idle-grip preharvest recovered handle-owned idle path from loaded graph formID={:08X} handle={:016X} requestedSubgraph={:016X} "
                        "bindingSubgraph={:016X} loaded={} handleMatches={} buckets={} paths={} idleCandidates={} clip={}",
                        state.job.weaponFormId, subgraphHandle, outSubgraphIdentifier, bindingSubgraphIdentifier, diagnostics.loadedSubgraphCount,
                        diagnostics.graphHandleMatchCount, diagnostics.graphClipBucketCount, diagnostics.graphClipPathCount,
                        diagnostics.graphIdlePathCandidateCount, outClipPath.data());
                }
                return trySampleClip(state, graph, bindingSubgraphIdentifier, outClipPath.data(), outHandInWeapon, outRightFiringFingerPose, diagnostics);
            };

            if (animationFiles->empty()) {
                return tryGraphPathFallback(IdleGripExtractionFailure::AnimationFileListEmpty);
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
                    if (!native_idle_grip_preharvest_policy::clipPathHasStem(path, desiredStem)) {
                        continue;
                    }
                    ++diagnostics.idlePathMatchCount;
                    if (path.size() >= outClipPath.size()) {
                        diagnostics.failure = IdleGripExtractionFailure::IdleClipPathTooLong;
                        continue;
                    }

                    outClipPath.fill('\0');
                    std::memcpy(outClipPath.data(), path.data(), path.size());
                    ++diagnostics.sampleAttemptCount;
                    const auto sampleResult = trySampleClip(state, graph, outSubgraphIdentifier, outClipPath.data(), outHandInWeapon, outRightFiringFingerPose, diagnostics);
                    if (sampleResult != ExtractionResult::Failed) {
                        return sampleResult;
                    }
                    if (state.job.idleClipResource.entry) {
                        return ExtractionResult::Failed;
                    }
                }
            }
            if (diagnostics.idlePathMatchCount == 0) {
                return tryGraphPathFallback(IdleGripExtractionFailure::IdleClipPathUnavailable);
            }
            if (diagnostics.sampleAttemptCount == 0) {
                return failExtractionResult(diagnostics, IdleGripExtractionFailure::IdleClipPathTooLong);
            }
            return ExtractionResult::Failed;
        }

        [[nodiscard]] bool progressJob(Runtime& state)
        {
            auto& job = state.job;
            if (job.phase == Phase::Idle) {
                return true;
            }

            const ULONGLONG now = GetTickCount64();
            if (!job.longLoadLogged && now - job.startedAtMilliseconds >= kLongLoadLogDelayMilliseconds) {
                ROCK_LOG_INFO(Animation, "Native idle-grip preharvest still loading formID={:08X} phase={} elapsedMs={} resourceState={:X}", job.weaponFormId,
                    static_cast<unsigned>(job.phase), now - job.startedAtMilliseconds, job.idleClipResourceState);
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
            RE::NiTransform handInWeapon{};
            authored_weapon_grip_library::FiringFingerPose rightFiringFingerPose{};
            std::array<char, 260> clipPath{};
            std::uint64_t subgraphIdentifier = 0;
            IdleGripExtractionDiagnostics extractionDiagnostics{};
            const auto extractionResult = tryExtractIdleGrip(state, *manager, handInWeapon, rightFiringFingerPose, clipPath, subgraphIdentifier, extractionDiagnostics);
            if (extractionResult == ExtractionResult::Pending) {
                return false;
            }
            if (extractionResult == ExtractionResult::Failed) {
                const char* failure = extractionFailureName(extractionDiagnostics.failure);
                ROCK_LOG_INFO(Animation,
                    "Native idle-grip preharvest extraction detail formID={:08X} reason={} graphs={} handles={} identifiers={} handle={:016X} "
                    "requestedSubgraph={:016X} bindingSubgraph={:016X} files={} idleMatches={} sampleAttempts={} loadedSubgraphs={} handleMatches={} "
                    "graphBuckets={} graphPaths={} idleCandidates={} graphAmbiguous={} graphFallback={} resourceState={:X} animationType={} duration={:.6f} tracks={} floatTracks={} "
                    "bindingBlendHint={:X} mapping={} weaponBone={:X} handBone={:X} weaponParent={} sampledFingerMask=0x{:04X} "
                    "referenceFingerMask=0x{:04X} missingFingerMask=0x{:04X} clip={}",
                    job.weaponFormId, failure, extractionDiagnostics.graphCount, extractionDiagnostics.handleCount, extractionDiagnostics.identifierCount,
                    extractionDiagnostics.subgraphHandle, extractionDiagnostics.subgraphIdentifier, extractionDiagnostics.bindingSubgraphIdentifier,
                    extractionDiagnostics.animationFileCount, extractionDiagnostics.idlePathMatchCount, extractionDiagnostics.sampleAttemptCount,
                    extractionDiagnostics.loadedSubgraphCount, extractionDiagnostics.graphHandleMatchCount, extractionDiagnostics.graphClipBucketCount,
                    extractionDiagnostics.graphClipPathCount, extractionDiagnostics.graphIdlePathCandidateCount, extractionDiagnostics.graphIdlePathAmbiguous,
                    extractionDiagnostics.usedGraphClipPathFallback,
                    extractionDiagnostics.directResourceState, extractionDiagnostics.animationType, extractionDiagnostics.animationDurationSeconds,
                    extractionDiagnostics.transformTrackCount, extractionDiagnostics.floatTrackCount, extractionDiagnostics.bindingBlendHint, extractionDiagnostics.mappingCount,
                    extractionDiagnostics.weaponBone, extractionDiagnostics.handBone, extractionDiagnostics.weaponParentIndex, extractionDiagnostics.sampledFingerMask,
                    extractionDiagnostics.referenceFingerMask, extractionDiagnostics.missingFingerMask, clipPath[0] != '\0' ? clipPath.data() : "<none>");
                failJob(state, failure);
                return true;
            }

            if (!job.weapon || job.weapon->GetFormID() != job.weaponFormId) {
                failJob(state, "weaponFormUnavailableAtPublish");
                return true;
            }

            const std::uint64_t captureSequence = kPreharvestCaptureSequenceDomain | (++state.nextCaptureSequence);
            if (!authored_weapon_grip_library::publishResolvedVariant(job.weapon, job.variant, job.inPowerArmor, handInWeapon, captureSequence,
                    authored_weapon_grip_library::CaptureSource::NativeIdlePreharvest, &rightFiringFingerPose)) {
                failJob(state, "authoredGripLibraryRejectedSample");
                return true;
            }

            ROCK_LOG_INFO(Animation,
                "Native idle-grip preharvest succeeded formID={:08X} refID={:08X} variant={:016X} origin={} handle={:016X} requestedSubgraph={:016X} "
                "bindingSubgraph={:016X} clip={} powerArmor={} animationType={} duration={:.6f} "
                "tracks={} floatTracks={} bindingBlendHint={:X} sampledFingerMask=0x{:04X} referenceFingerMask=0x{:04X} missingFingerMask=0x{:04X} "
                "handInWeaponT=({:.6f},{:.6f},{:.6f}) scale={:.7f}",
                job.weaponFormId, job.referenceFormId, job.variant.key,
                job.origin == CandidateOrigin::EquippedWeapon ? "equipped" : "loose",
                extractionDiagnostics.subgraphHandle, subgraphIdentifier, extractionDiagnostics.bindingSubgraphIdentifier,
                clipPath.data(), job.inPowerArmor ? "yes" : "no", extractionDiagnostics.animationType,
                extractionDiagnostics.animationDurationSeconds, extractionDiagnostics.transformTrackCount, extractionDiagnostics.floatTrackCount,
                extractionDiagnostics.bindingBlendHint, extractionDiagnostics.sampledFingerMask, extractionDiagnostics.referenceFingerMask, extractionDiagnostics.missingFingerMask, handInWeapon.translate.x, handInWeapon.translate.y, handInWeapon.translate.z, handInWeapon.scale);
            releaseJob(state);
            return true;
        }

        [[nodiscard]] bool eligibleWeapon(const RE::TESObjectWEAP* weapon)
        {
            return weapon &&
                   weapon->weaponData.type != RE::WEAPON_TYPE::kGrenade &&
                   weapon->weaponData.type != RE::WEAPON_TYPE::kMine;
        }

        [[nodiscard]] Job describeLooseCandidate(
            RE::TESObjectREFR* reference,
            RE::NiAVObject* weaponRoot)
        {
            Job candidate{};
            if (!reference || !weaponRoot) {
                return candidate;
            }
            auto* baseForm = reference->GetObjectReference();
            auto* weapon = baseForm ? baseForm->As<RE::TESObjectWEAP>() : nullptr;
            if (!eligibleWeapon(weapon)) {
                return candidate;
            }

            candidate.reference = reference->GetHandle();
            candidate.weapon = weapon;
            candidate.variant = authored_weapon_grip_library::identifyWeaponVariant(weaponRoot);
            candidate.referenceFormId = reference->GetFormID();
            candidate.weaponFormId = weapon->GetFormID();
            candidate.inPowerArmor = f4vr::isInPowerArmor();
            candidate.origin = CandidateOrigin::LooseReference;
            return candidate;
        }

        [[nodiscard]] Job describeEquippedCandidate(
            RE::TESObjectWEAP* weapon,
            RE::NiAVObject* weaponRoot,
            RE::TBO_InstanceData* instanceData)
        {
            Job candidate{};
            if (!eligibleWeapon(weapon) || !weaponRoot) {
                return candidate;
            }

            candidate.instanceData = RE::BSTSmartPointer<RE::TBO_InstanceData>(instanceData);
            candidate.weapon = weapon;
            candidate.variant = authored_weapon_grip_library::identifyWeaponVariant(weaponRoot);
            candidate.instanceIdentity = reinterpret_cast<std::uintptr_t>(candidate.instanceData.get());
            candidate.weaponFormId = weapon->GetFormID();
            candidate.inPowerArmor = f4vr::isInPowerArmor();
            candidate.origin = CandidateOrigin::EquippedWeapon;
            return candidate;
        }

        [[nodiscard]] bool advanceAndCanStart(Runtime& state)
        {
            if (state.job.phase != Phase::Idle) {
                (void)progressJob(state);
            }
            return state.job.phase == Phase::Idle &&
                   resolveNativeFunctions(state);
        }

        [[nodiscard]] bool shouldStartCandidate(
            const Runtime& state,
            const Job& candidate,
            RE::NiAVObject* weaponRoot)
        {
            if (!candidate.weapon || candidate.weaponFormId == 0 || candidate.origin == CandidateOrigin::Unknown || !weaponRoot) {
                return false;
            }
            const auto existing = authored_weapon_grip_library::find(
                candidate.weapon,
                weaponRoot,
                candidate.inPowerArmor);
            if (!native_idle_grip_preharvest_policy::shouldStartNativeIdleHarvest(
                    existing.found,
                    existing.source == authored_weapon_grip_library::CaptureSource::NativeIdlePreharvest,
                    existing.usedVariantFallback,
                    candidate.variant.key)) {
                return false;
            }
            return !candidateIsCoolingDown(state, candidate, GetTickCount64());
        }

        void startJob(Runtime& state, Job&& candidate)
        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (!player || !player->race || !candidate.weapon || candidate.weaponFormId == 0 || candidate.origin == CandidateOrigin::Unknown) {
                return;
            }

            if (candidate.origin == CandidateOrigin::LooseReference) {
                if (!candidate.reference) {
                    return;
                }
                const auto reference = candidate.reference.get();
                auto* referenceRaw = reference.get();
                if (!referenceRaw || referenceRaw->GetObjectReference() != candidate.weapon) {
                    return;
                }
                candidate.instanceData = resolveInstanceData(referenceRaw, candidate.weapon);
                candidate.instanceIdentity = reinterpret_cast<std::uintptr_t>(candidate.instanceData.get());
            }

            candidate.race = player->race;
            candidate.startedAtMilliseconds = GetTickCount64();
            candidate.phase = Phase::BaseGraphsLoading;
            state.job = std::move(candidate);

            if (!prepareNativeSubgraphOutput(state.job.subgraphHandles) ||
                !prepareNativeSubgraphOutput(state.job.subgraphIdentifiers)) {
                failJob(state, "nativeSubgraphOutputStorageUnavailable");
                return;
            }

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
                "Native idle-grip preharvest started formID={:08X} refID={:08X} variant={:016X} origin={} powerArmor={} instance=0x{:X} graphProjects={} base={} firstPerson={}",
                state.job.weaponFormId,
                state.job.referenceFormId,
                state.job.variant.key,
                state.job.origin == CandidateOrigin::EquippedWeapon ? "equipped" : "loose",
                state.job.inPowerArmor ? "yes" : "no",
                state.job.instanceIdentity,
                graphProjects.size(),
                baseGraph ? baseGraph : "<null>", firstPersonGraph ? firstPersonGraph : "<null>");
        }
    }

    void observeCandidate(RE::NiPointer<RE::TESObjectREFR> candidate) noexcept
    {
        auto& state = runtime();
        if (!claimOrValidateThread(state)) {
            return;
        }

        if (!advanceAndCanStart(state) || !candidate) {
            return;
        }

        auto* candidateRaw = candidate.get();
        RE::NiPointer<RE::NiAVObject> weaponRoot(candidateRaw->Get3D());
        Job candidateDescription = describeLooseCandidate(candidateRaw, weaponRoot.get());
        if (!shouldStartCandidate(state, candidateDescription, weaponRoot.get())) {
            return;
        }

        startJob(state, std::move(candidateDescription));
    }

    void observeEquippedWeapon(
        RE::TESObjectWEAP* weapon,
        RE::NiAVObject* weaponRoot,
        RE::TBO_InstanceData* instanceData) noexcept
    {
        auto& state = runtime();
        if (!claimOrValidateThread(state)) {
            return;
        }

        if (!advanceAndCanStart(state) || !weapon || !weaponRoot) {
            return;
        }

        Job candidateDescription = describeEquippedCandidate(weapon, weaponRoot, instanceData);
        if (!shouldStartCandidate(state, candidateDescription, weaponRoot)) {
            return;
        }

        startJob(state, std::move(candidateDescription));
    }
}
