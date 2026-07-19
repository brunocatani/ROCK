#include "physics-interaction/animation/NativeAnimationAuthority.h"

#include "api/ROCKProviderApi.h"
#include "physics-interaction/animation/NativeAnimationAuthorityPolicy.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"

#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"

#include <Windows.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <intrin.h>
#include <string_view>

namespace rock::native_animation_authority
{
    namespace
    {
        using BoneTree = f4vr::BSFlattenedBoneTree;
        using BoneTransform = BoneTree::BoneTransforms;
        using PostUpdateAnimationGraphManagerFn = void (*)(void* holder);
        using UpdateFirstPersonArmFn = void* (*)(const RE::PlayerCharacter* player, RE::NiNode** weapon, RE::NiNode** offsetNode);
        using WeaponFireHandlerFn = bool (*)(void* handler, RE::Actor* actor, RE::BSFixedString* eventData);
        using ReloadStateChangeHandlerFn = bool (*)(void* handler, RE::Actor* actor, RE::BSFixedString* stateToken);
        using ReloadStateTokenFn = RE::BSFixedString* (*)();

        constexpr std::size_t kMaxBindings = 192;
        constexpr int kMaxFlattenedTransforms = 768;
        constexpr std::uint32_t kLocalReloadTestLeaseFrames = 600;
        constexpr float kManualCycleFallbackWatchdogSeconds = 6.0f;
        constexpr float kManualCycleWatchdogPaddingSeconds = 0.75f;
        constexpr float kManualCycleMinimumWatchdogSeconds = 1.0f;
        constexpr float kManualCycleMaximumWatchdogSeconds = 12.0f;
        constexpr int kManualCycleVisualAuthorityPriority = 110;
        constexpr const char* kManualCycleVisualAuthorityTag =
            "ROCK_NativeManualCycle";
        constexpr std::uint32_t kImplementedFlags = native_animation_authority_policy::kReloadPose;
        constexpr std::uint16_t kAuthoredSupportFingerTransformMask = 0x7FFFu;
        constexpr std::array<std::string_view, 15> kManualCyclePrimaryFingerBoneNames{
            "RArm_Finger11", "RArm_Finger12", "RArm_Finger13",
            "RArm_Finger21", "RArm_Finger22", "RArm_Finger23",
            "RArm_Finger31", "RArm_Finger32", "RArm_Finger33",
            "RArm_Finger41", "RArm_Finger42", "RArm_Finger43",
            "RArm_Finger51", "RArm_Finger52", "RArm_Finger53",
        };
        constexpr std::array<std::string_view, 15> kAuthoredSupportFingerBoneNames{
            "LArm_Finger11", "LArm_Finger12", "LArm_Finger13",
            "LArm_Finger21", "LArm_Finger22", "LArm_Finger23",
            "LArm_Finger31", "LArm_Finger32", "LArm_Finger33",
            "LArm_Finger41", "LArm_Finger42", "LArm_Finger43",
            "LArm_Finger51", "LArm_Finger52", "LArm_Finger53",
        };

        struct Binding
        {
            int sourceIndex{ -1 };
            int destinationIndex{ -1 };
            std::uint32_t flags{ 0 };
            RE::NiTransform capturedLocal{};
            bool captured{ false };
        };

        struct BindingCache
        {
            BoneTree* sourceTree{ nullptr };
            BoneTree* destinationTree{ nullptr };
            BoneTransform* sourceTransforms{ nullptr };
            BoneTransform* destinationTransforms{ nullptr };
            int sourceCount{ 0 };
            int destinationCount{ 0 };
            int sourceWeaponIndex{ -1 };
            int destinationWeaponIndex{ -1 };
            std::array<Binding, kMaxBindings> bindings{};
            std::size_t bindingCount{ 0 };
        };

        struct PrimaryFiringGripBoneCache
        {
            BoneTree* tree{ nullptr };
            BoneTransform* transforms{ nullptr };
            int transformCount{ 0 };
            int primaryHandIndex{ -1 };
            int supportHandIndex{ -1 };
            int weaponIndex{ -1 };
            std::array<int, kManualCyclePrimaryFingerBoneNames.size()> primaryFingerIndices{};
            std::array<int, kAuthoredSupportFingerBoneNames.size()> supportFingerIndices{};
        };

        struct ManualCyclePoseCapture
        {
            RE::NiTransform primaryHandInWeapon{};
            RE::NiTransform supportHandInWeapon{};
            frik_visual_authority::FingerLocalTransformOverride primaryFingerLocals{};
            frik_visual_authority::FingerLocalTransformOverride supportFingerLocals{};
            bool primaryHandValid{ false };
            bool supportHandValid{ false };
        };

        struct ManualCycleVisualPublication
        {
            bool worldPublished{ false };
            bool fingerPosePublished{ false };
        };

        struct ControllerAimFrame
        {
            // Non-owning scene references. They are valid only while the
            // matching flattened tree/cache and authority session remain
            // active; resetHybridPoseState clears them at every lease edge.
            RE::NiNode* weaponNode{ nullptr };
            RE::NiNode* controlParent{ nullptr };
            RE::NiTransform weaponInControlParent{};
            RE::NiTransform controlWeaponWorld{};
            RE::NiTransform nativeBaselineWeaponWorld{};
            RE::NiTransform desiredWeaponWorld{};
            bool controlBindingCaptured{ false };
            bool controlCaptured{ false };
            bool nativeBaselineCaptured{ false };
            bool desiredCaptured{ false };
            bool destinationAlignmentLogged{ false };
            bool manualCycleIkLogged{ false };
        };

        BindingCache s_cache{};
        PrimaryFiringGripBoneCache s_primaryFiringGripBoneCache{};
        ManualCyclePoseCapture s_manualCyclePoseCapture{};
        std::array<ManualCycleVisualPublication, 2> s_manualCycleVisualPublications{};
        ControllerAimFrame s_sourceAimFrame{};
        PostUpdateAnimationGraphManagerFn s_originalPostUpdate{ nullptr };
        UpdateFirstPersonArmFn s_originalUpdateFirstPersonArm{ nullptr };
        WeaponFireHandlerFn s_originalWeaponFire{ nullptr };
        ReloadStateChangeHandlerFn s_originalReloadStateChange{ nullptr };
        std::atomic<bool> s_hookInstalled{ false };
        std::atomic<bool> s_primaryFiringGripHookInstalled{ false };
        std::atomic<bool> s_primaryFiringGripHookInstallFailed{ false };
        std::atomic<bool> s_reloadStateHookInstalled{ false };
        std::atomic<bool> s_weaponFireHookInstalled{ false };
        std::atomic<bool> s_weaponFireHookInstallFailed{ false };
        std::atomic<bool> s_hookInstallFailed{ false };
        std::atomic<bool> s_runtimeEnabled{ false };
        std::atomic<bool> s_localManualCycleTestEnabled{ false };
        std::atomic<bool> s_localManualCycleTestLeaseActive{ false };
        std::atomic<bool> s_primaryFiringGripCaptureEnabled{ false };
        std::atomic<bool> s_primaryFiringGripCaptureValid{ false };
        std::atomic<bool> s_authoredSupportGraphPoseValid{ false };
        std::atomic<bool> s_authoredSupportGripCaptureValid{ false };
        std::atomic<bool> s_captureValid{ false };
        std::atomic<bool> s_threadMismatch{ false };
        std::atomic<bool> s_captureFault{ false };
        std::atomic<DWORD> s_ownerThreadId{ 0 };
        std::atomic<std::uint32_t> s_localReloadTestLeaseFrames{ 0 };
        std::atomic<std::uint64_t> s_localReloadTestRequestSequence{ 0 };
        std::atomic<std::uint64_t> s_localManualCycleTestRequestSequence{ 0 };
        std::atomic<std::uint32_t> s_localManualCycleRequestedWatchdogMilliseconds{ 0 };
        std::atomic<std::uint64_t> s_localManualCycleReloadStartSequenceAtArm{ 0 };
        std::atomic<std::uint64_t> s_localManualCycleReloadEndSequenceAtArm{ 0 };
        std::atomic<std::uint64_t> s_playerReloadStartSequence{ 0 };
        std::atomic<std::uint64_t> s_playerReloadEndSequence{ 0 };
        std::atomic<bool> s_playerReloadEventActive{ false };
        std::atomic<std::uint32_t> s_capturedFlags{ 0 };
        std::atomic<std::uint32_t> s_capturedTransformCount{ 0 };
        std::atomic<std::uint64_t> s_captureSequence{ 0 };
        std::atomic<std::uint64_t> s_primaryFiringGripCaptureSequence{ 0 };
        std::atomic<std::uint64_t> s_authoredSupportGraphPoseSequence{ 0 };
        std::atomic<std::uint64_t> s_authoredSupportGripCaptureSequence{ 0 };
        std::atomic<std::uint64_t> s_authoredSupportGripSecondaryPassSequence{ 0 };
        std::atomic<AuthoredSupportGripCaptureFailureReason> s_authoredSupportGripCaptureFailureReason{
            AuthoredSupportGripCaptureFailureReason::SecondaryPassNotObserved
        };
        std::atomic<std::uint16_t> s_authoredSupportGripInvalidOrMissingFingerMask{ 0 };

        // Published only after Bethesda's validated paired arm passes and
        // consumed on the same claimed ROCK game thread. Each atomic
        // valid/sequence pair is its publication boundary; scene identity
        // witnesses are never retained after invalidation or skeleton reset.
        RE::NiTransform s_authoredPrimaryHandInWeapon{};
        std::atomic<RE::NiNode*> s_primaryFiringGripHandNode{ nullptr };
        std::atomic<RE::NiNode*> s_primaryFiringGripWeaponNode{ nullptr };
        RE::NiTransform s_authoredSupportHandInPrimaryHand{};
        std::array<RE::NiTransform, kAuthoredSupportFingerBoneNames.size()>
            s_authoredSupportGraphFingerLocals{};
        std::atomic<BoneTree*> s_authoredSupportGraphTree{ nullptr };
        std::atomic<BoneTransform*> s_authoredSupportGraphTransforms{ nullptr };
        std::atomic<int> s_authoredSupportGraphTransformCount{ 0 };
        std::atomic<RE::NiNode*> s_authoredSupportGraphPrimaryHandNode{ nullptr };
        RE::NiTransform s_authoredSupportHandInWeapon{};
        std::array<RE::NiTransform, kAuthoredSupportFingerBoneNames.size()> s_authoredSupportFingerLocals{};
        std::atomic<RE::NiNode*> s_authoredSupportGripPrimaryHandNode{ nullptr };
        std::atomic<RE::NiNode*> s_authoredSupportGripWeaponNode{ nullptr };
        std::uintptr_t s_nativePrimaryArmReturnAddress{ 0 };
        std::uintptr_t s_nativeSupportArmReturnAddress{ 0 };
        std::uint64_t s_primaryFiringGripGraphPoseSequence{ 0 };
        std::uint64_t s_lastConsumedAuthoredSupportGraphPoseSequence{ 0 };

        std::uint64_t s_frameCaptureSequence{ 0 };
        std::uint64_t s_lastCompletedCaptureSequence{ 0 };
        std::uint64_t s_seenLocalReloadTestRequestSequence{ 0 };
        std::uint64_t s_seenLocalManualCycleTestRequestSequence{ 0 };
        std::uint32_t s_frameCaptureFlags{ 0 };
        std::uint32_t s_lastLoggedEffectiveFlags{ 0 };
        native_animation_authority_policy::LocalReloadLeaseState s_localReloadLeaseState{};
        native_animation_authority_policy::LocalManualCycleLeaseState s_localManualCycleLeaseState{};
        bool s_frameCaptureReady{ false };
        bool s_frameManualCycleExpected{ false };
        bool s_frameManualCycleApplied{ false };

        [[nodiscard]] bool validTree(const BoneTree* tree)
        {
            return tree && tree->transforms && tree->numTransforms > 0 && tree->numTransforms <= kMaxFlattenedTransforms;
        }

        [[nodiscard]] std::string_view transformName(const BoneTransform& transform)
        {
            const char* name = transform.name.c_str();
            return name ? std::string_view(name) : std::string_view{};
        }

        [[nodiscard]] bool finiteTransform(const RE::NiTransform& transform)
        {
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return std::isfinite(transform.translate.x) &&
                   std::isfinite(transform.translate.y) &&
                   std::isfinite(transform.translate.z) &&
                   std::isfinite(transform.scale) &&
                   std::abs(transform.scale) > 0.000001f;
        }

        [[nodiscard]] constexpr std::size_t manualCycleHandIndex(
            const frik_visual_authority::Hand hand)
        {
            return hand == frik_visual_authority::Hand::Left ? 1u : 0u;
        }

        [[nodiscard]] bool clearManualCycleVisualForHand(
            const frik_visual_authority::Hand hand)
        {
            auto& publication =
                s_manualCycleVisualPublications[manualCycleHandIndex(hand)];
            if (!publication.worldPublished && !publication.fingerPosePublished) {
                return true;
            }

            const bool skeletonReady =
                frik_visual_authority::isSkeletonReadyHint();
            const bool fingerPoseCleared = !publication.fingerPosePublished ||
                frik_visual_authority::clearHandPose(
                    kManualCycleVisualAuthorityTag,
                    hand);
            const bool worldCleared = !publication.worldPublished ||
                frik_visual_authority::clearExternalHandWorldTransform(
                    kManualCycleVisualAuthorityTag,
                    hand);

            if (fingerPoseCleared || !skeletonReady) {
                publication.fingerPosePublished = false;
            }
            if (worldCleared || !skeletonReady) {
                publication.worldPublished = false;
            }
            return !publication.worldPublished &&
                   !publication.fingerPosePublished;
        }

        void clearManualCycleVisualAuthority()
        {
            (void)clearManualCycleVisualForHand(
                frik_visual_authority::Hand::Right);
            (void)clearManualCycleVisualForHand(
                frik_visual_authority::Hand::Left);
        }

        [[nodiscard]] bool manualCycleVisualAuthorityPublished()
        {
            for (const auto& publication : s_manualCycleVisualPublications) {
                if (publication.worldPublished ||
                    publication.fingerPosePublished) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] bool claimOrValidateThread()
        {
            const DWORD currentThread = GetCurrentThreadId();
            DWORD expected = 0;
            if (s_ownerThreadId.compare_exchange_strong(expected, currentThread, std::memory_order_acq_rel)) {
                return true;
            }
            if (expected == currentThread) {
                return true;
            }
            s_threadMismatch.store(true, std::memory_order_release);
            s_captureValid.store(false, std::memory_order_release);
            return false;
        }

        [[nodiscard]] std::uint32_t effectiveRequestedFlags()
        {
            if (!s_runtimeEnabled.load(std::memory_order_acquire)) {
                return 0;
            }
            const std::uint32_t providerFlags = provider::currentNativeAnimationAuthorityFlagsV1();
            std::uint32_t localFlags =
                s_localReloadTestLeaseFrames.load(std::memory_order_acquire) > 0 ?
                kImplementedFlags :
                0;
            if (s_localManualCycleTestLeaseActive.load(std::memory_order_acquire)) {
                localFlags |= native_animation_authority_policy::kManualCyclePose;
            }
            return (providerFlags | localFlags) & kImplementedFlags;
        }

        [[nodiscard]] int findDestinationIndex(const BoneTree& destination, std::string_view sourceName)
        {
            for (int index = 0; index < destination.numTransforms; ++index) {
                if (native_animation_authority_policy::equalsIgnoreCase(
                        transformName(destination.transforms[index]), sourceName)) {
                    return index;
                }
            }
            return -1;
        }

        void resetHybridPoseState()
        {
            s_sourceAimFrame = {};
        }

        [[nodiscard]] bool cacheMatches(BoneTree* source, BoneTree* destination)
        {
            return s_cache.sourceTree == source &&
                   s_cache.destinationTree == destination &&
                   s_cache.sourceTransforms == source->transforms &&
                   s_cache.destinationTransforms == destination->transforms &&
                   s_cache.sourceCount == source->numTransforms &&
                   s_cache.destinationCount == destination->numTransforms &&
                   s_cache.sourceWeaponIndex >= 0 &&
                   s_cache.bindingCount > 0;
        }

        [[nodiscard]] bool rebuildCache(BoneTree* source, BoneTree* destination)
        {
            resetHybridPoseState();
            s_cache = {};
            if (!validTree(source) || !validTree(destination)) {
                return false;
            }

            s_cache.sourceTree = source;
            s_cache.destinationTree = destination;
            s_cache.sourceTransforms = source->transforms;
            s_cache.destinationTransforms = destination->transforms;
            s_cache.sourceCount = source->numTransforms;
            s_cache.destinationCount = destination->numTransforms;

            for (int sourceIndex = 0; sourceIndex < source->numTransforms; ++sourceIndex) {
                const auto name = transformName(source->transforms[sourceIndex]);
                const auto flags = native_animation_authority_policy::classifyBone(name);
                if (flags == 0) {
                    continue;
                }
                if (s_cache.bindingCount >= s_cache.bindings.size()) {
                    s_cache = {};
                    return false;
                }

                auto& binding = s_cache.bindings[s_cache.bindingCount++];
                binding.sourceIndex = sourceIndex;
                binding.destinationIndex = findDestinationIndex(*destination, name);
                binding.flags = flags;
                if (native_animation_authority_policy::equalsIgnoreCase(name, "Weapon")) {
                    s_cache.sourceWeaponIndex = binding.sourceIndex;
                    s_cache.destinationWeaponIndex = binding.destinationIndex;
                }
            }
            return s_cache.bindingCount > 0 &&
                   s_cache.sourceWeaponIndex >= 0 &&
                   s_cache.destinationWeaponIndex >= 0;
        }

        [[nodiscard]] const RE::NiTransform& authoritativeLocal(const BoneTransform& transform)
        {
            return transform.refNode ? transform.refNode->local : transform.local;
        }

        struct AuthoredLogicalModelTransform
        {
            RE::NiTransform model{};
            int rootIndex{ -1 };
        };

        [[nodiscard]] bool composeAuthoredLogicalModelTransform(
            const BoneTree& tree,
            const int leafIndex,
            AuthoredLogicalModelTransform& out)
        {
            out = {};
            if (leafIndex < 0 || leafIndex >= tree.numTransforms) {
                return false;
            }

            // BSFlattenedBoneTree_UpdateBoneArray at FO4VR 0x141C214B0
            // proves that `world` is presentation cache: when refNode exists,
            // the engine copies refNode->world into it. It is therefore the
            // wrong source for the controller-independent authored pose.
            // Walk parPos and compose only graph-local transforms instead.
            std::array<int, kMaxFlattenedTransforms> chain{};
            std::size_t chainLength = 0;
            int currentIndex = leafIndex;
            while (currentIndex >= 0) {
                if (currentIndex >= tree.numTransforms ||
                    chainLength >= static_cast<std::size_t>(tree.numTransforms) ||
                    chainLength >= chain.size()) {
                    return false;
                }
                chain[chainLength++] = currentIndex;
                const int parentIndex = tree.transforms[currentIndex].parPos;
                if (parentIndex < 0) {
                    break;
                }
                currentIndex = parentIndex;
            }
            if (chainLength == 0 ||
                tree.transforms[chain[chainLength - 1]].parPos >= 0) {
                return false;
            }

            RE::NiTransform model =
                transform_math::makeIdentityTransform<RE::NiTransform>();
            for (std::size_t chainPosition = chainLength;
                 chainPosition > 0;
                 --chainPosition) {
                const int transformIndex = chain[chainPosition - 1];
                const RE::NiTransform& local =
                    authoritativeLocal(tree.transforms[transformIndex]);
                if (!finiteTransform(local)) {
                    return false;
                }
                model = transform_math::composeTransforms(model, local);
                if (!finiteTransform(model)) {
                    return false;
                }
            }

            out.model = model;
            out.rootIndex = chain[chainLength - 1];
            return true;
        }

        void invalidateCapture()
        {
            s_captureValid.store(false, std::memory_order_release);
            s_capturedFlags.store(0, std::memory_order_release);
            s_capturedTransformCount.store(0, std::memory_order_release);
            s_manualCyclePoseCapture = {};
        }

        void invalidatePrimaryFiringGripCapture()
        {
            s_primaryFiringGripCaptureValid.store(false, std::memory_order_release);
            s_primaryFiringGripHandNode.store(nullptr, std::memory_order_release);
            s_primaryFiringGripWeaponNode.store(nullptr, std::memory_order_release);
            s_primaryFiringGripGraphPoseSequence = 0;
        }

        void invalidateAuthoredSupportGraphPose()
        {
            s_authoredSupportGraphPoseValid.store(false, std::memory_order_release);
            s_authoredSupportGraphTree.store(nullptr, std::memory_order_release);
            s_authoredSupportGraphTransforms.store(nullptr, std::memory_order_release);
            s_authoredSupportGraphTransformCount.store(0, std::memory_order_release);
            s_authoredSupportGraphPrimaryHandNode.store(nullptr, std::memory_order_release);
        }

        void invalidateAuthoredSupportGripCapture()
        {
            s_authoredSupportGripCaptureValid.store(false, std::memory_order_release);
            s_authoredSupportGripPrimaryHandNode.store(nullptr, std::memory_order_release);
            s_authoredSupportGripWeaponNode.store(nullptr, std::memory_order_release);
        }

        void recordAuthoredSupportGripCaptureFailure(
            const AuthoredSupportGripCaptureFailureReason reason,
            const std::uint16_t invalidOrMissingFingerMask = 0)
        {
            s_authoredSupportGripInvalidOrMissingFingerMask.store(
                invalidOrMissingFingerMask,
                std::memory_order_release);
            s_authoredSupportGripCaptureFailureReason.store(
                reason,
                std::memory_order_release);
        }

        [[nodiscard]] bool primaryFiringGripCacheMatches(const BoneTree& source)
        {
            return s_primaryFiringGripBoneCache.tree == &source &&
                   s_primaryFiringGripBoneCache.transforms == source.transforms &&
                   s_primaryFiringGripBoneCache.transformCount == source.numTransforms &&
                   s_primaryFiringGripBoneCache.primaryHandIndex >= 0 &&
                   s_primaryFiringGripBoneCache.weaponIndex >= 0;
        }

        [[nodiscard]] bool rebuildPrimaryFiringGripBoneCache(BoneTree& source)
        {
            s_primaryFiringGripBoneCache = {};
            s_primaryFiringGripBoneCache.primaryFingerIndices.fill(-1);
            s_primaryFiringGripBoneCache.supportFingerIndices.fill(-1);
            if (!validTree(&source)) {
                return false;
            }

            s_primaryFiringGripBoneCache.tree = &source;
            s_primaryFiringGripBoneCache.transforms = source.transforms;
            s_primaryFiringGripBoneCache.transformCount = source.numTransforms;
            for (int index = 0; index < source.numTransforms; ++index) {
                const auto name = transformName(source.transforms[index]);
                if (native_animation_authority_policy::equalsIgnoreCase(name, "RArm_Hand")) {
                    s_primaryFiringGripBoneCache.primaryHandIndex = index;
                } else if (native_animation_authority_policy::equalsIgnoreCase(name, "LArm_Hand")) {
                    s_primaryFiringGripBoneCache.supportHandIndex = index;
                } else if (native_animation_authority_policy::equalsIgnoreCase(name, "Weapon")) {
                    s_primaryFiringGripBoneCache.weaponIndex = index;
                }
                for (std::size_t fingerIndex = 0;
                     fingerIndex < kManualCyclePrimaryFingerBoneNames.size();
                     ++fingerIndex) {
                    if (native_animation_authority_policy::equalsIgnoreCase(
                            name,
                            kManualCyclePrimaryFingerBoneNames[fingerIndex])) {
                        s_primaryFiringGripBoneCache.primaryFingerIndices[fingerIndex] = index;
                        break;
                    }
                }
                for (std::size_t fingerIndex = 0;
                     fingerIndex < kAuthoredSupportFingerBoneNames.size();
                     ++fingerIndex) {
                    if (native_animation_authority_policy::equalsIgnoreCase(
                            name,
                            kAuthoredSupportFingerBoneNames[fingerIndex])) {
                        s_primaryFiringGripBoneCache.supportFingerIndices[fingerIndex] = index;
                        break;
                    }
                }
            }
            return s_primaryFiringGripBoneCache.primaryHandIndex >= 0 &&
                   s_primaryFiringGripBoneCache.weaponIndex >= 0;
        }

        [[nodiscard]] bool authoredSupportGripCacheReady(
            const BoneTree& source,
            std::uint16_t& outMissingFingerMask)
        {
            outMissingFingerMask = 0;
            if (s_primaryFiringGripBoneCache.supportHandIndex < 0 ||
                s_primaryFiringGripBoneCache.supportHandIndex >= source.numTransforms) {
                return false;
            }
            for (std::size_t fingerIndex = 0;
                 fingerIndex < s_primaryFiringGripBoneCache.supportFingerIndices.size();
                 ++fingerIndex) {
                const int index =
                    s_primaryFiringGripBoneCache.supportFingerIndices[fingerIndex];
                if (index < 0 || index >= source.numTransforms) {
                    outMissingFingerMask |= static_cast<std::uint16_t>(1u << fingerIndex);
                }
            }
            return outMissingFingerMask == 0;
        }

        void captureManualCycleFingerLocals(
            const BoneTree& source,
            const std::array<int, kManualCyclePrimaryFingerBoneNames.size()>& indices,
            frik_visual_authority::FingerLocalTransformOverride& out)
        {
            out = {};
            for (std::size_t fingerIndex = 0;
                 fingerIndex < indices.size();
                 ++fingerIndex) {
                const int transformIndex = indices[fingerIndex];
                if (transformIndex < 0 || transformIndex >= source.numTransforms) {
                    continue;
                }
                const RE::NiTransform& local =
                    authoritativeLocal(source.transforms[transformIndex]);
                if (!finiteTransform(local)) {
                    continue;
                }
                out.localTransforms[fingerIndex] = local;
                out.enabledMask |= static_cast<std::uint16_t>(1u << fingerIndex);
            }
        }

        [[nodiscard]] bool captureManualCyclePose(BoneTree& source)
        {
            s_manualCyclePoseCapture = {};
            if (!primaryFiringGripCacheMatches(source) &&
                !rebuildPrimaryFiringGripBoneCache(source)) {
                return false;
            }

            const int primaryHandIndex =
                s_primaryFiringGripBoneCache.primaryHandIndex;
            const int supportHandIndex =
                s_primaryFiringGripBoneCache.supportHandIndex;
            const int weaponIndex = s_primaryFiringGripBoneCache.weaponIndex;
            if (primaryHandIndex < 0 || primaryHandIndex >= source.numTransforms ||
                weaponIndex < 0 || weaponIndex >= source.numTransforms) {
                return false;
            }

            const auto& weaponTransform = source.transforms[weaponIndex];
            if (weaponTransform.parPos != primaryHandIndex ||
                !finiteTransform(weaponTransform.local)) {
                return false;
            }

            AuthoredLogicalModelTransform primaryHandModel{};
            if (!composeAuthoredLogicalModelTransform(
                    source,
                    primaryHandIndex,
                    primaryHandModel)) {
                return false;
            }
            const RE::NiTransform nativeWeaponModel =
                transform_math::composeTransforms(
                    primaryHandModel.model,
                    weaponTransform.local);
            if (!finiteTransform(nativeWeaponModel)) {
                return false;
            }

            const auto resolveHandInWeapon = [&](
                                                   const RE::NiTransform& handModel) {
                return native_animation_authority_policy::resolveNativeHandInWeapon(
                    nativeWeaponModel,
                    handModel,
                    [](const RE::NiTransform& parent,
                       const RE::NiTransform& child) {
                        return transform_math::composeTransforms(parent, child);
                    },
                    [](const RE::NiTransform& transform) {
                        return transform_math::invertTransform(transform);
                    });
            };

            s_manualCyclePoseCapture.primaryHandInWeapon =
                resolveHandInWeapon(primaryHandModel.model);
            if (!finiteTransform(
                    s_manualCyclePoseCapture.primaryHandInWeapon)) {
                s_manualCyclePoseCapture = {};
                return false;
            }
            s_manualCyclePoseCapture.primaryHandValid = true;
            captureManualCycleFingerLocals(
                source,
                s_primaryFiringGripBoneCache.primaryFingerIndices,
                s_manualCyclePoseCapture.primaryFingerLocals);

            if (supportHandIndex >= 0 &&
                supportHandIndex < source.numTransforms) {
                AuthoredLogicalModelTransform supportHandModel{};
                if (composeAuthoredLogicalModelTransform(
                        source,
                        supportHandIndex,
                        supportHandModel) &&
                    supportHandModel.rootIndex == primaryHandModel.rootIndex) {
                    const RE::NiTransform supportHandInWeapon =
                        resolveHandInWeapon(supportHandModel.model);
                    if (finiteTransform(supportHandInWeapon)) {
                        s_manualCyclePoseCapture.supportHandInWeapon =
                            supportHandInWeapon;
                        s_manualCyclePoseCapture.supportHandValid = true;
                        captureManualCycleFingerLocals(
                            source,
                            s_primaryFiringGripBoneCache.supportFingerIndices,
                            s_manualCyclePoseCapture.supportFingerLocals);
                    }
                }
            }
            return true;
        }

        void captureAuthoredSupportGraphPose()
        {
            if (!s_primaryFiringGripCaptureEnabled.load(std::memory_order_acquire) ||
                s_playerReloadEventActive.load(std::memory_order_acquire)) {
                invalidateAuthoredSupportGraphPose();
                invalidateAuthoredSupportGripCapture();
                return;
            }

            // The published support candidate must be frame-fresh. This early
            // graph snapshot is paired later in the same player update with
            // Bethesda's exact primary hand-in-weapon capture.
            invalidateAuthoredSupportGraphPose();
            invalidateAuthoredSupportGripCapture();

            auto* source = f4vr::getFirstPersonBoneTree();
            if (!validTree(source)) {
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::SourceTreeUnavailable);
                return;
            }
            if (!primaryFiringGripCacheMatches(*source) &&
                !rebuildPrimaryFiringGripBoneCache(*source)) {
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::BoneCacheIncomplete);
                return;
            }

            std::uint16_t missingFingerMask = 0;
            if (!authoredSupportGripCacheReady(*source, missingFingerMask)) {
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::BoneCacheIncomplete,
                    missingFingerMask);
                return;
            }

            const int primaryHandIndex =
                s_primaryFiringGripBoneCache.primaryHandIndex;
            const int supportHandIndex =
                s_primaryFiringGripBoneCache.supportHandIndex;
            if (primaryHandIndex < 0 || primaryHandIndex >= source->numTransforms ||
                supportHandIndex < 0 || supportHandIndex >= source->numTransforms) {
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::BoneCacheIncomplete);
                return;
            }

            const auto& primaryHandTransform = source->transforms[primaryHandIndex];
            if (!primaryHandTransform.refNode) {
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::TopologyInvalid);
                return;
            }

            AuthoredLogicalModelTransform primaryHandModel{};
            AuthoredLogicalModelTransform supportHandModel{};
            if (!composeAuthoredLogicalModelTransform(
                    *source,
                    primaryHandIndex,
                    primaryHandModel) ||
                !composeAuthoredLogicalModelTransform(
                    *source,
                    supportHandIndex,
                    supportHandModel) ||
                primaryHandModel.rootIndex != supportHandModel.rootIndex) {
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::AuthoredHandHierarchyInvalid);
                return;
            }

            const RE::NiTransform supportHandInPrimaryHand =
                native_animation_authority_policy::resolveAuthoredSupportHandInPrimaryHand(
                    primaryHandModel.model,
                    supportHandModel.model,
                    [](const RE::NiTransform& parent, const RE::NiTransform& child) {
                        return transform_math::composeTransforms(parent, child);
                    },
                    [](const RE::NiTransform& transform) {
                        return transform_math::invertTransform(transform);
                    });
            if (!finiteTransform(supportHandInPrimaryHand)) {
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::SupportHandTransformInvalid);
                return;
            }

            std::array<RE::NiTransform, kAuthoredSupportFingerBoneNames.size()>
                fingerLocals{};
            std::uint16_t invalidFingerMask = 0;
            for (std::size_t fingerIndex = 0;
                 fingerIndex < s_primaryFiringGripBoneCache.supportFingerIndices.size();
                 ++fingerIndex) {
                const int transformIndex =
                    s_primaryFiringGripBoneCache.supportFingerIndices[fingerIndex];
                const RE::NiTransform& fingerLocal =
                    authoritativeLocal(source->transforms[transformIndex]);
                if (!finiteTransform(fingerLocal)) {
                    invalidFingerMask |= static_cast<std::uint16_t>(1u << fingerIndex);
                    continue;
                }
                fingerLocals[fingerIndex] = fingerLocal;
            }
            if (invalidFingerMask != 0) {
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::FingerTransformInvalid,
                    invalidFingerMask);
                return;
            }

            s_authoredSupportHandInPrimaryHand = supportHandInPrimaryHand;
            s_authoredSupportGraphFingerLocals = fingerLocals;
            s_authoredSupportGraphTree.store(source, std::memory_order_release);
            s_authoredSupportGraphTransforms.store(
                source->transforms,
                std::memory_order_release);
            s_authoredSupportGraphTransformCount.store(
                source->numTransforms,
                std::memory_order_release);
            s_authoredSupportGraphPrimaryHandNode.store(
                primaryHandTransform.refNode,
                std::memory_order_release);
            s_authoredSupportGraphPoseSequence.fetch_add(
                1,
                std::memory_order_acq_rel);
            s_authoredSupportGraphPoseValid.store(true, std::memory_order_release);
            recordAuthoredSupportGripCaptureFailure(
                AuthoredSupportGripCaptureFailureReason::None);
        }

        [[nodiscard]] bool captureNativeAuthoredSupportGrip()
        {
            auto* source = f4vr::getFirstPersonBoneTree();
            if (!validTree(source)) {
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::SourceTreeUnavailable);
                return false;
            }
            if (!primaryFiringGripCacheMatches(*source) &&
                !rebuildPrimaryFiringGripBoneCache(*source)) {
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::BoneCacheIncomplete);
                return false;
            }
            std::uint16_t missingFingerMask = 0;
            if (!authoredSupportGripCacheReady(*source, missingFingerMask)) {
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::BoneCacheIncomplete,
                    missingFingerMask);
                return false;
            }

            const int primaryHandIndex = s_primaryFiringGripBoneCache.primaryHandIndex;
            const int weaponIndex = s_primaryFiringGripBoneCache.weaponIndex;
            if (primaryHandIndex < 0 || primaryHandIndex >= source->numTransforms ||
                weaponIndex < 0 || weaponIndex >= source->numTransforms) {
                return false;
            }

            const auto& primaryHandTransform = source->transforms[primaryHandIndex];
            const auto& weaponTransform = source->transforms[weaponIndex];
            if (weaponTransform.parPos != primaryHandIndex ||
                !primaryHandTransform.refNode ||
                !weaponTransform.refNode) {
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::TopologyInvalid);
                return false;
            }

            // The primary call immediately precedes this paired support call.
            // Its per-weapon native hand relation is the calibrated Weapon
            // anchor; do not fall back to the generic flattened Weapon local.
            if (!s_primaryFiringGripCaptureValid.load(std::memory_order_acquire) ||
                s_primaryFiringGripHandNode.load(std::memory_order_acquire) !=
                    primaryHandTransform.refNode ||
                s_primaryFiringGripWeaponNode.load(std::memory_order_acquire) !=
                    weaponTransform.refNode ||
                !finiteTransform(s_authoredPrimaryHandInWeapon)) {
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::PrimaryPoseUnavailable);
                return false;
            }

            const std::uint64_t graphPoseSequence =
                s_authoredSupportGraphPoseSequence.load(std::memory_order_acquire);
            if (!s_authoredSupportGraphPoseValid.load(std::memory_order_acquire) ||
                graphPoseSequence == 0 ||
                graphPoseSequence != s_primaryFiringGripGraphPoseSequence ||
                graphPoseSequence ==
                    s_lastConsumedAuthoredSupportGraphPoseSequence ||
                s_authoredSupportGraphTree.load(std::memory_order_acquire) != source ||
                s_authoredSupportGraphTransforms.load(std::memory_order_acquire) !=
                    source->transforms ||
                s_authoredSupportGraphTransformCount.load(std::memory_order_acquire) !=
                    source->numTransforms ||
                s_authoredSupportGraphPrimaryHandNode.load(std::memory_order_acquire) !=
                    primaryHandTransform.refNode) {
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::AuthoredGraphPoseUnavailable);
                return false;
            }

            const RE::NiTransform supportHandInWeapon =
                native_animation_authority_policy::resolveAuthoredSupportHandInWeapon(
                    s_authoredPrimaryHandInWeapon,
                    s_authoredSupportHandInPrimaryHand,
                    [](const RE::NiTransform& parent, const RE::NiTransform& child) {
                        return transform_math::composeTransforms(parent, child);
                    });
            if (!finiteTransform(supportHandInWeapon)) {
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::SupportHandTransformInvalid);
                return false;
            }

            s_authoredSupportHandInWeapon = supportHandInWeapon;
            s_authoredSupportFingerLocals = s_authoredSupportGraphFingerLocals;
            s_authoredSupportGripPrimaryHandNode.store(primaryHandTransform.refNode, std::memory_order_release);
            s_authoredSupportGripWeaponNode.store(weaponTransform.refNode, std::memory_order_release);
            s_lastConsumedAuthoredSupportGraphPoseSequence = graphPoseSequence;
            s_authoredSupportGripCaptureSequence.fetch_add(1, std::memory_order_acq_rel);
            recordAuthoredSupportGripCaptureFailure(
                AuthoredSupportGripCaptureFailureReason::None);
            s_authoredSupportGripCaptureValid.store(true, std::memory_order_release);
            return true;
        }

        [[nodiscard]] bool captureNativePrimaryFiringGrip()
        {
            auto* source = f4vr::getFirstPersonBoneTree();
            if (!validTree(source) ||
                (!primaryFiringGripCacheMatches(*source) &&
                    !rebuildPrimaryFiringGripBoneCache(*source))) {
                return false;
            }

            const int handIndex = s_primaryFiringGripBoneCache.primaryHandIndex;
            const int weaponIndex = s_primaryFiringGripBoneCache.weaponIndex;
            if (handIndex < 0 || handIndex >= source->numTransforms ||
                weaponIndex < 0 || weaponIndex >= source->numTransforms) {
                return false;
            }

            const auto& handTransform = source->transforms[handIndex];
            const auto& weaponTransform = source->transforms[weaponIndex];
            if (weaponTransform.parPos != handIndex ||
                !handTransform.refNode ||
                !weaponTransform.refNode ||
                !finiteTransform(handTransform.refNode->world) ||
                !finiteTransform(weaponTransform.refNode->world)) {
                return false;
            }

            const RE::NiTransform handInWeapon = transform_math::composeTransforms(
                transform_math::invertTransform(weaponTransform.refNode->world),
                handTransform.refNode->world);
            if (!finiteTransform(handInWeapon)) {
                return false;
            }

            s_authoredPrimaryHandInWeapon = handInWeapon;
            const std::uint64_t graphPoseSequence =
                s_authoredSupportGraphPoseSequence.load(std::memory_order_acquire);
            const bool graphPoseMatches =
                s_authoredSupportGraphPoseValid.load(std::memory_order_acquire) &&
                graphPoseSequence != 0 &&
                graphPoseSequence !=
                    s_lastConsumedAuthoredSupportGraphPoseSequence &&
                s_authoredSupportGraphTree.load(std::memory_order_acquire) == source &&
                s_authoredSupportGraphTransforms.load(std::memory_order_acquire) ==
                    source->transforms &&
                s_authoredSupportGraphTransformCount.load(std::memory_order_acquire) ==
                    source->numTransforms &&
                s_authoredSupportGraphPrimaryHandNode.load(std::memory_order_acquire) ==
                    handTransform.refNode;
            s_primaryFiringGripGraphPoseSequence =
                graphPoseMatches ? graphPoseSequence : 0;
            s_primaryFiringGripHandNode.store(handTransform.refNode, std::memory_order_release);
            s_primaryFiringGripWeaponNode.store(weaponTransform.refNode, std::memory_order_release);
            s_primaryFiringGripCaptureSequence.fetch_add(1, std::memory_order_acq_rel);
            s_primaryFiringGripCaptureValid.store(true, std::memory_order_release);
            return true;
        }

        __declspec(noinline) void* onUpdateFirstPersonArm(
            const RE::PlayerCharacter* player,
            RE::NiNode** weapon,
            RE::NiNode** offsetNode)
        {
            const auto returnAddress = reinterpret_cast<std::uintptr_t>(_ReturnAddress());
            void* result = s_originalUpdateFirstPersonArm ?
                               s_originalUpdateFirstPersonArm(player, weapon, offsetNode) :
                               nullptr;

            const bool primaryPass = returnAddress == s_nativePrimaryArmReturnAddress;
            const bool supportPass = returnAddress == s_nativeSupportArmReturnAddress;
            if ((!primaryPass && !supportPass) ||
                !s_primaryFiringGripCaptureEnabled.load(std::memory_order_acquire) ||
                s_playerReloadEventActive.load(std::memory_order_acquire)) {
                return result;
            }
            if (supportPass) {
                s_authoredSupportGripSecondaryPassSequence.fetch_add(
                    1,
                    std::memory_order_acq_rel);
            }

#if defined(_MSC_VER)
            __try {
                const bool threadValid = claimOrValidateThread();
                if (!threadValid && supportPass) {
                    recordAuthoredSupportGripCaptureFailure(
                        AuthoredSupportGripCaptureFailureReason::ThreadMismatch);
                }
                const bool captured = threadValid &&
                    (primaryPass ?
                            captureNativePrimaryFiringGrip() :
                            captureNativeAuthoredSupportGrip());
                if (!captured) {
                    if (primaryPass) {
                        invalidatePrimaryFiringGripCapture();
                    } else {
                        invalidateAuthoredSupportGripCapture();
                    }
                }
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                s_captureFault.store(true, std::memory_order_release);
                if (primaryPass) {
                    invalidatePrimaryFiringGripCapture();
                } else {
                    recordAuthoredSupportGripCaptureFailure(
                        AuthoredSupportGripCaptureFailureReason::CaptureFault);
                    invalidateAuthoredSupportGripCapture();
                }
            }
#else
            const bool threadValid = claimOrValidateThread();
            if (!threadValid && supportPass) {
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::ThreadMismatch);
            }
            const bool captured = threadValid &&
                (primaryPass ?
                        captureNativePrimaryFiringGrip() :
                        captureNativeAuthoredSupportGrip());
            if (!captured) {
                if (primaryPass) {
                    invalidatePrimaryFiringGripCapture();
                } else {
                    invalidateAuthoredSupportGripCapture();
                }
            }
#endif
            return result;
        }

        void captureNativePose()
        {
            const std::uint32_t requestedFlags = effectiveRequestedFlags();
            if (requestedFlags == 0 || !claimOrValidateThread()) {
                invalidateCapture();
                return;
            }

            auto* source = f4vr::getFirstPersonBoneTree();
            auto* destination = f4vr::getFlattenedBoneTree();
            if (!validTree(source) || !validTree(destination)) {
                invalidateCapture();
                return;
            }
            if (!cacheMatches(source, destination) && !rebuildCache(source, destination)) {
                invalidateCapture();
                return;
            }

            std::uint32_t capturedCount = 0;
            std::uint32_t destinationMappedFlags = 0;
            for (std::size_t i = 0; i < s_cache.bindingCount; ++i) {
                auto& binding = s_cache.bindings[i];
                binding.captured = false;
                const bool requestedBinding =
                    native_animation_authority_policy::isRequested(binding.flags, requestedFlags);
                const bool controllerAimAnchor = binding.sourceIndex == s_cache.sourceWeaponIndex;
                if ((!requestedBinding && !controllerAimAnchor) ||
                    binding.sourceIndex < 0 || binding.sourceIndex >= source->numTransforms) {
                    continue;
                }

                // Arms/hands-only authority counter-transforms the visible
                // Weapon child after each apply. During that mode the scene
                // node local is presentation state, not the graph's native
                // weapon local; retain the flattened logical local as the
                // alignment anchor for the next capture.
                const auto& local = controllerAimAnchor &&
                        (requestedFlags & native_animation_authority_policy::kWeapon) == 0 ?
                    source->transforms[binding.sourceIndex].local :
                    authoritativeLocal(source->transforms[binding.sourceIndex]);
                if (!finiteTransform(local)) {
                    continue;
                }
                binding.capturedLocal = local;
                binding.captured = true;
                ++capturedCount;
                if (requestedBinding && binding.destinationIndex >= 0) {
                    destinationMappedFlags |= binding.flags;
                }
            }

            if (capturedCount == 0 || (destinationMappedFlags & requestedFlags) != requestedFlags) {
                s_manualCyclePoseCapture = {};
                invalidateCapture();
                return;
            }

            const bool handOnlyAuthority =
                (requestedFlags & native_animation_authority_policy::kWeapon) == 0;
            if (handOnlyAuthority) {
                if ((requestedFlags & native_animation_authority_policy::kArms) == 0 ||
                    !captureManualCyclePose(*source)) {
                    s_manualCyclePoseCapture = {};
                    invalidateCapture();
                    return;
                }
            } else {
                s_manualCyclePoseCapture = {};
            }

            s_capturedFlags.store(requestedFlags, std::memory_order_release);
            s_capturedTransformCount.store(capturedCount, std::memory_order_release);
            s_captureSequence.fetch_add(1, std::memory_order_acq_rel);
            s_captureValid.store(true, std::memory_order_release);
        }

        void onPostUpdateAnimationGraphManager(void* holder)
        {
#if defined(_MSC_VER)
            __try {
                captureAuthoredSupportGraphPose();
                captureNativePose();
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                s_captureFault.store(true, std::memory_order_release);
                invalidateCapture();
                invalidatePrimaryFiringGripCapture();
                invalidateAuthoredSupportGraphPose();
                invalidateAuthoredSupportGripCapture();
            }
#else
            captureAuthoredSupportGraphPose();
            captureNativePose();
#endif
            if (s_originalPostUpdate) {
                s_originalPostUpdate(holder);
            }
        }

        [[nodiscard]] RE::TESObjectWEAP::InstanceData* currentPlayerWeaponInstanceData(
            RE::TESObjectWEAP*& outWeapon)
        {
            outWeapon = nullptr;
            auto* player = f4vr::getPlayer();
            auto* processData = player && player->middleProcess ? player->middleProcess->unk08 : nullptr;
            auto* equipData = processData ? processData->equipData : nullptr;
            auto* weaponForm = equipData ? equipData->item : nullptr;
            if (!weaponForm ||
                weaponForm->formType != static_cast<std::uint8_t>(RE::ENUM_FORM_ID::kWEAP)) {
                return nullptr;
            }

            auto* reForm = reinterpret_cast<RE::TESForm*>(weaponForm);
            outWeapon = reForm ? reForm->As<RE::TESObjectWEAP>() : nullptr;
            if (!outWeapon) {
                return nullptr;
            }
            return equipData->instanceData ?
                static_cast<RE::TESObjectWEAP::InstanceData*>(equipData->instanceData) :
                &outWeapon->weaponData;
        }

        [[nodiscard]] float manualCycleWatchdogSeconds(
            const RE::TESObjectWEAP::InstanceData& weaponData)
        {
            const auto* rangedData = weaponData.rangedData;
            if (!rangedData ||
                !std::isfinite(rangedData->boltChargeSeconds) ||
                rangedData->boltChargeSeconds <= 0.0f) {
                return kManualCycleFallbackWatchdogSeconds;
            }

            const float fireSeconds =
                std::isfinite(rangedData->fireSeconds) && rangedData->fireSeconds > 0.0f ?
                rangedData->fireSeconds :
                0.0f;
            float watchdogSeconds =
                fireSeconds + rangedData->boltChargeSeconds +
                kManualCycleWatchdogPaddingSeconds;
            if (watchdogSeconds < kManualCycleMinimumWatchdogSeconds) {
                watchdogSeconds = kManualCycleMinimumWatchdogSeconds;
            } else if (watchdogSeconds > kManualCycleMaximumWatchdogSeconds) {
                watchdogSeconds = kManualCycleMaximumWatchdogSeconds;
            }
            return watchdogSeconds;
        }

        void cancelLocalManualCycleTestLease()
        {
            s_localManualCycleRequestedWatchdogMilliseconds.store(0, std::memory_order_release);
            s_localManualCycleTestLeaseActive.store(false, std::memory_order_release);
        }

        bool onWeaponFire(void* handler, RE::Actor* actor, RE::BSFixedString* eventData)
        {
            // Sample before Bethesda handles the fire event. Replacement clips
            // may publish their frame-zero ReloadEnd marker synchronously from
            // the original handler; arming from the post-call value would lose
            // the first half of the native cycle bracket.
            const auto reloadStartSequenceBeforeFire =
                s_playerReloadStartSequence.load(std::memory_order_acquire);
            const auto reloadEndSequenceBeforeFire =
                s_playerReloadEndSequence.load(std::memory_order_acquire);
            const bool handled = s_originalWeaponFire ?
                s_originalWeaponFire(handler, actor, eventData) :
                false;

            const auto* player = RE::PlayerCharacter::GetSingleton();
            if (!handled || !actor || actor != player ||
                !s_runtimeEnabled.load(std::memory_order_acquire) ||
                !s_localManualCycleTestEnabled.load(std::memory_order_acquire) ||
                s_localReloadTestLeaseFrames.load(std::memory_order_acquire) > 0 ||
                s_playerReloadEventActive.load(std::memory_order_acquire)) {
                return handled;
            }

            RE::TESObjectWEAP* weapon = nullptr;
            const auto* weaponData = currentPlayerWeaponInstanceData(weapon);
            if (!weapon || !weaponData ||
                !weaponData->flags.any(RE::WEAPON_FLAGS::kBoltAction)) {
                return handled;
            }

            const float watchdogSeconds = manualCycleWatchdogSeconds(*weaponData);
            const auto watchdogMilliseconds = static_cast<std::uint32_t>(
                watchdogSeconds * 1000.0f + 0.5f);
            s_localManualCycleReloadStartSequenceAtArm.store(
                reloadStartSequenceBeforeFire,
                std::memory_order_release);
            s_localManualCycleReloadEndSequenceAtArm.store(
                reloadEndSequenceBeforeFire,
                std::memory_order_release);
            s_localManualCycleRequestedWatchdogMilliseconds.store(
                watchdogMilliseconds,
                std::memory_order_release);
            s_localManualCycleTestRequestSequence.fetch_add(1, std::memory_order_acq_rel);
            s_localManualCycleTestLeaseActive.store(true, std::memory_order_release);
            return handled;
        }

        [[nodiscard]] const RE::BSFixedString* nativeReloadStartStateToken()
        {
            static REL::Relocation<ReloadStateTokenFn> getToken{
                REL::Offset(offsets::kFunc_GetReloadStartStateToken)
            };
            return getToken();
        }

        [[nodiscard]] const RE::BSFixedString* nativeReloadEndStateToken()
        {
            static REL::Relocation<ReloadStateTokenFn> getToken{
                REL::Offset(offsets::kFunc_GetReloadEndStateToken)
            };
            return getToken();
        }

        bool onReloadStateChange(void* handler, RE::Actor* actor, RE::BSFixedString* stateToken)
        {
            const bool handled = s_originalReloadStateChange ?
                s_originalReloadStateChange(handler, actor, stateToken) :
                false;

            const auto* player = RE::PlayerCharacter::GetSingleton();
            if (!actor || actor != player || !stateToken) {
                return handled;
            }

            const auto* startToken = nativeReloadStartStateToken();
            if (startToken && *stateToken == *startToken) {
                cancelLocalManualCycleTestLease();
                s_playerReloadEventActive.store(true, std::memory_order_release);
                s_playerReloadStartSequence.fetch_add(1, std::memory_order_acq_rel);
                return handled;
            }

            const auto* endToken = nativeReloadEndStateToken();
            if (endToken && *stateToken == *endToken) {
                s_playerReloadEndSequence.fetch_add(1, std::memory_order_acq_rel);
                s_playerReloadEventActive.store(false, std::memory_order_release);
            }
            return handled;
        }

        [[nodiscard]] bool bindingSelectedForTree(
            const Binding& binding,
            bool destination,
            std::uint32_t requestedFlags)
        {
            const int index = destination ? binding.destinationIndex : binding.sourceIndex;
            return binding.captured && index >= 0 &&
                   native_animation_authority_policy::isRequested(binding.flags, requestedFlags);
        }

        [[nodiscard]] bool parentIsSelected(
            int parentIndex,
            bool destination,
            std::uint32_t requestedFlags)
        {
            if (parentIndex < 0) {
                return false;
            }
            for (std::size_t i = 0; i < s_cache.bindingCount; ++i) {
                const auto& candidate = s_cache.bindings[i];
                const int candidateIndex = destination ? candidate.destinationIndex : candidate.sourceIndex;
                if (candidateIndex == parentIndex && bindingSelectedForTree(candidate, destination, requestedFlags)) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] bool usesDifferentSceneParent(const BoneTree& tree, int index)
        {
            if (index < 0 || index >= tree.numTransforms) {
                return false;
            }
            const auto& transform = tree.transforms[index];
            if (!transform.refNode || transform.parPos < 0 || transform.parPos >= tree.numTransforms) {
                return false;
            }
            return transform.refNode->parent != tree.transforms[transform.parPos].refNode;
        }

        [[nodiscard]] bool readPresentationWorld(
            const BoneTree& tree,
            int index,
            RE::NiTransform& outWorld)
        {
            if (index < 0 || index >= tree.numTransforms) {
                return false;
            }
            const auto& transform = tree.transforms[index];
            outWorld = transform.refNode ? transform.refNode->world : transform.world;
            return finiteTransform(outWorld);
        }

        [[nodiscard]] bool readLogicalWorld(
            const BoneTree& tree,
            int index,
            RE::NiTransform& outWorld)
        {
            if (index < 0 || index >= tree.numTransforms) {
                return false;
            }
            const auto& transform = tree.transforms[index];
            if (transform.parPos >= 0 && transform.parPos < tree.numTransforms) {
                const auto& logicalParent = tree.transforms[transform.parPos];
                const RE::NiTransform& parentWorld = logicalParent.refNode ? logicalParent.refNode->world : logicalParent.world;
                if (!finiteTransform(parentWorld) || !finiteTransform(transform.local)) {
                    return false;
                }
                outWorld = transform_math::composeTransforms(parentWorld, transform.local);
                return finiteTransform(outWorld);
            }
            return readPresentationWorld(tree, index, outWorld);
        }

        [[nodiscard]] bool prepareControllerAimFrame(
            const BoneTree& tree,
            int weaponIndex,
            ControllerAimFrame& aimFrame)
        {
            if (weaponIndex < 0 || weaponIndex >= tree.numTransforms) {
                return false;
            }

            const auto& weaponTransform = tree.transforms[weaponIndex];
            auto* weaponNode = weaponTransform.refNode;
            auto* controlParent = weaponNode ? weaponNode->parent : nullptr;
            if (!weaponNode || !controlParent ||
                !finiteTransform(controlParent->world) ||
                !finiteTransform(weaponNode->local)) {
                return false;
            }

            if (!aimFrame.controlBindingCaptured) {
                aimFrame.weaponNode = weaponNode;
                aimFrame.controlParent = controlParent;
                aimFrame.weaponInControlParent = weaponNode->local;
                aimFrame.controlBindingCaptured = true;
            } else if (aimFrame.weaponNode != weaponNode || aimFrame.controlParent != controlParent) {
                // A weapon/skeleton replacement or a live reparent invalidates
                // both the controller and authored baselines. Fail closed; the
                // next authority edge will establish a coherent new session.
                return false;
            }

            aimFrame.controlWeaponWorld = transform_math::composeTransforms(
                aimFrame.controlParent->world,
                aimFrame.weaponInControlParent);
            aimFrame.controlCaptured = finiteTransform(aimFrame.controlWeaponWorld);
            return aimFrame.controlCaptured;
        }

        [[nodiscard]] bool prepareControllerAimFrames()
        {
            if (!validTree(s_cache.sourceTree) || !validTree(s_cache.destinationTree)) {
                return false;
            }
            // The first-person Weapon node is the rendered gun and therefore
            // the only valid live control frame. hFRIK deliberately culls the
            // full-body Weapon node; its world follows the solved body arm and
            // must never become an independent target for the visible arms.
            return prepareControllerAimFrame(
                *s_cache.sourceTree,
                s_cache.sourceWeaponIndex,
                s_sourceAimFrame);
        }

        [[nodiscard]] bool prepareManualCycleWeaponNode()
        {
            if (!validTree(s_cache.sourceTree) ||
                s_cache.sourceWeaponIndex < 0 ||
                s_cache.sourceWeaponIndex >= s_cache.sourceTree->numTransforms) {
                return false;
            }
            auto* weaponNode = s_cache.sourceTree
                                   ->transforms[s_cache.sourceWeaponIndex]
                                   .refNode;
            if (!weaponNode) {
                return false;
            }
            if (s_sourceAimFrame.weaponNode &&
                s_sourceAimFrame.weaponNode != weaponNode) {
                return false;
            }
            s_sourceAimFrame.weaponNode = weaponNode;
            return true;
        }

        [[nodiscard]] bool refreshFixedVisibleWeaponTarget(
            RE::NiTransform& outWeaponWorld)
        {
            auto& aimFrame = s_sourceAimFrame;
            auto* weaponNode = aimFrame.weaponNode;
            if (!weaponNode || !weaponNode->parent ||
                !finiteTransform(weaponNode->world)) {
                return false;
            }

            outWeaponWorld = weaponNode->world;
            aimFrame.controlWeaponWorld = outWeaponWorld;
            aimFrame.controlCaptured = true;
            aimFrame.desiredWeaponWorld = outWeaponWorld;
            aimFrame.desiredCaptured = true;
            return true;
        }

        [[nodiscard]] bool restoreFixedVisibleWeaponTarget(
            const RE::NiTransform& weaponWorld)
        {
            auto& aimFrame = s_sourceAimFrame;
            auto* weaponNode = aimFrame.weaponNode;
            auto* weaponParent = weaponNode ? weaponNode->parent : nullptr;
            if (!weaponNode || !weaponParent ||
                !finiteTransform(weaponWorld) ||
                !finiteTransform(weaponParent->world)) {
                return false;
            }

            const RE::NiTransform weaponLocal = transform_math::composeTransforms(
                transform_math::invertTransform(weaponParent->world),
                weaponWorld);
            if (!finiteTransform(weaponLocal)) {
                return false;
            }
            weaponNode->local = weaponLocal;
            f4vr::updateTransformsDown(weaponNode, true);
            if (!finiteTransform(weaponNode->world)) {
                return false;
            }

            if (validTree(s_cache.sourceTree) &&
                s_cache.sourceWeaponIndex >= 0 &&
                s_cache.sourceWeaponIndex < s_cache.sourceTree->numTransforms) {
                // Keep the flattened local as the native graph anchor. Only
                // its presentation world follows the counter-transformed node.
                s_cache.sourceTree->transforms[s_cache.sourceWeaponIndex].world =
                    weaponNode->world;
            }
            aimFrame.controlWeaponWorld = weaponWorld;
            aimFrame.desiredWeaponWorld = weaponWorld;
            aimFrame.controlCaptured = true;
            aimFrame.desiredCaptured = true;
            return true;
        }

        [[nodiscard]] bool tryRestoreFixedVisibleWeaponTarget(
            const RE::NiTransform& weaponWorld)
        {
#if defined(_MSC_VER)
            __try {
                return restoreFixedVisibleWeaponTarget(weaponWorld);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
#else
            return restoreFixedVisibleWeaponTarget(weaponWorld);
#endif
        }

        void clearManualCycleVisualAuthorityPreservingWeapon()
        {
            RE::NiTransform fixedWeaponWorld{};
            const bool restoreWeapon =
                manualCycleVisualAuthorityPublished() &&
                refreshFixedVisibleWeaponTarget(fixedWeaponWorld);
            clearManualCycleVisualAuthority();
            if (restoreWeapon) {
                (void)tryRestoreFixedVisibleWeaponTarget(fixedWeaponWorld);
            }
        }

        [[nodiscard]] bool publishManualCycleHandVisual(
            const frik_visual_authority::Hand hand,
            const RE::NiTransform& handInWeapon,
            const frik_visual_authority::FingerLocalTransformOverride& fingerLocals,
            const RE::NiTransform& fixedWeaponWorld)
        {
            auto& publication =
                s_manualCycleVisualPublications[manualCycleHandIndex(hand)];
            const RE::NiTransform handWorld =
                native_animation_authority_policy::resolveAuthoredPrimaryHandWorld(
                    fixedWeaponWorld,
                    handInWeapon,
                    [](const RE::NiTransform& parent,
                       const RE::NiTransform& child) {
                        return transform_math::composeTransforms(parent, child);
                    });
            if (!finiteTransform(handWorld)) {
                (void)clearManualCycleVisualForHand(hand);
                return false;
            }

            if (fingerLocals.enabledMask != 0) {
                if (!frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(
                        kManualCycleVisualAuthorityTag,
                        hand,
                        &fingerLocals,
                        kManualCycleVisualAuthorityPriority)) {
                    (void)clearManualCycleVisualForHand(hand);
                    return false;
                }
                publication.fingerPosePublished = true;
            } else if (publication.fingerPosePublished) {
                if (!frik_visual_authority::clearHandPose(
                        kManualCycleVisualAuthorityTag,
                        hand)) {
                    return false;
                }
                publication.fingerPosePublished = false;
            }

            if (!frik_visual_authority::applyExternalHandWorldTransform(
                    kManualCycleVisualAuthorityTag,
                    hand,
                    handWorld,
                    kManualCycleVisualAuthorityPriority)) {
                (void)clearManualCycleVisualForHand(hand);
                return false;
            }
            publication.worldPublished = true;
            return true;
        }

        [[nodiscard]] bool applyManualCyclePoseAfterRock()
        {
            if (!s_manualCyclePoseCapture.primaryHandValid) {
                clearManualCycleVisualAuthority();
                return false;
            }

            RE::NiTransform fixedWeaponWorld{};
            if (!refreshFixedVisibleWeaponTarget(fixedWeaponWorld)) {
                clearManualCycleVisualAuthority();
                return false;
            }

            const bool primaryApplied = publishManualCycleHandVisual(
                frik_visual_authority::Hand::Right,
                s_manualCyclePoseCapture.primaryHandInWeapon,
                s_manualCyclePoseCapture.primaryFingerLocals,
                fixedWeaponWorld);
            bool supportApplied = false;
            if (s_manualCyclePoseCapture.supportHandValid) {
                supportApplied = publishManualCycleHandVisual(
                    frik_visual_authority::Hand::Left,
                    s_manualCyclePoseCapture.supportHandInWeapon,
                    s_manualCyclePoseCapture.supportFingerLocals,
                    fixedWeaponWorld);
            } else {
                (void)clearManualCycleVisualForHand(
                    frik_visual_authority::Hand::Left);
            }

            // hFRIK's external-hand solver deliberately excludes the Weapon
            // child while moving an arm. Recompute the Weapon local anyway so
            // its scene hierarchy remains coherent with the controller-fixed
            // world before the next engine transform propagation.
            if (!restoreFixedVisibleWeaponTarget(fixedWeaponWorld)) {
                clearManualCycleVisualAuthority();
                (void)tryRestoreFixedVisibleWeaponTarget(fixedWeaponWorld);
                return false;
            }
            if (!primaryApplied) {
                clearManualCycleVisualAuthority();
                (void)tryRestoreFixedVisibleWeaponTarget(fixedWeaponWorld);
                return false;
            }

            if (!s_sourceAimFrame.manualCycleIkLogged) {
                ROCK_LOG_INFO(Animation,
                    "Native manual-cycle IK ready priority={} hands={} weaponT=({:.3f},{:.3f},{:.3f})",
                    kManualCycleVisualAuthorityPriority,
                    supportApplied ? 2 : 1,
                    fixedWeaponWorld.translate.x,
                    fixedWeaponWorld.translate.y,
                    fixedWeaponWorld.translate.z);
                s_sourceAimFrame.manualCycleIkLogged = true;
            }
            return true;
        }

        [[nodiscard]] RE::NiTransform resolveControllerAimCorrection(
            const RE::NiTransform& liveControl,
            const RE::NiTransform& authoredBaseline,
            const RE::NiTransform& authoredCurrent)
        {
            return native_animation_authority_policy::resolveControllerAnchoredPoseCorrection(
                liveControl,
                authoredBaseline,
                authoredCurrent,
                [](const RE::NiTransform& parent, const RE::NiTransform& child) {
                    return transform_math::composeTransforms(parent, child);
                },
                [](const RE::NiTransform& transform) {
                    return transform_math::invertTransform(transform);
                });
        }

        [[nodiscard]] RE::NiTransform resolveWorldTargetCorrection(
            const RE::NiTransform& worldTarget,
            const RE::NiTransform& authoredCurrent)
        {
            return native_animation_authority_policy::resolvePoseCorrectionToWorldTarget(
                worldTarget,
                authoredCurrent,
                [](const RE::NiTransform& parent, const RE::NiTransform& child) {
                    return transform_math::composeTransforms(parent, child);
                },
                [](const RE::NiTransform& transform) {
                    return transform_math::invertTransform(transform);
                });
        }

        void writeCapturedLocals(BoneTree& tree, bool destination, std::uint32_t requestedFlags)
        {
            for (std::size_t i = 0; i < s_cache.bindingCount; ++i) {
                const auto& binding = s_cache.bindings[i];
                if (!bindingSelectedForTree(binding, destination, requestedFlags)) {
                    continue;
                }
                const int index = destination ? binding.destinationIndex : binding.sourceIndex;
                if (index < 0 || index >= tree.numTransforms) {
                    continue;
                }
                auto& transform = tree.transforms[index];
                transform.local = binding.capturedLocal;
                if (transform.refNode && !usesDifferentSceneParent(tree, index)) {
                    transform.refNode->local = binding.capturedLocal;
                }
            }
        }

        void propagateSelectedRoots(BoneTree& tree, bool destination, std::uint32_t requestedFlags)
        {
            for (std::size_t i = 0; i < s_cache.bindingCount; ++i) {
                const auto& binding = s_cache.bindings[i];
                if (!bindingSelectedForTree(binding, destination, requestedFlags)) {
                    continue;
                }
                const int index = destination ? binding.destinationIndex : binding.sourceIndex;
                if (index < 0 || index >= tree.numTransforms) {
                    continue;
                }
                auto& transform = tree.transforms[index];
                if (transform.refNode &&
                    !usesDifferentSceneParent(tree, index) &&
                    !parentIsSelected(transform.parPos, destination, requestedFlags)) {
                    f4vr::updateTransformsDown(transform.refNode, true);
                }
            }
        }

        [[nodiscard]] bool applyControllerAimFrame(
            BoneTree& tree,
            bool destination,
            std::uint32_t requestedFlags)
        {
            auto& aimFrame = s_sourceAimFrame;
            const int weaponIndex = destination ? s_cache.destinationWeaponIndex : s_cache.sourceWeaponIndex;
            if (!aimFrame.controlCaptured || (destination && !aimFrame.desiredCaptured)) {
                return false;
            }

            RE::NiTransform nativeWeaponWorld{};
            if (!readLogicalWorld(tree, weaponIndex, nativeWeaponWorld)) {
                return false;
            }
            const bool weaponTransformRequested =
                (requestedFlags & native_animation_authority_policy::kWeapon) != 0;
            if (!weaponTransformRequested) {
                return false;
            }
            const bool initializedBaseline =
                !destination &&
                !aimFrame.nativeBaselineCaptured;
            RE::NiTransform correction{};
            if (!destination) {
                if (initializedBaseline) {
                    aimFrame.nativeBaselineWeaponWorld = nativeWeaponWorld;
                    aimFrame.nativeBaselineCaptured = true;
                }
                correction = resolveControllerAimCorrection(
                    aimFrame.controlWeaponWorld,
                    aimFrame.nativeBaselineWeaponWorld,
                    nativeWeaponWorld);
                aimFrame.desiredWeaponWorld = transform_math::composeTransforms(
                    correction,
                    nativeWeaponWorld);
                aimFrame.desiredCaptured = finiteTransform(aimFrame.desiredWeaponWorld);
                if (!aimFrame.desiredCaptured) {
                    return false;
                }
            } else {
                // The destination tree can be in a completely different world
                // basis. Resolve its own native Weapon to the source tree's
                // visible-gun target instead of reusing a source-space matrix.
                correction = resolveWorldTargetCorrection(
                    aimFrame.desiredWeaponWorld,
                    nativeWeaponWorld);
            }
            if (!finiteTransform(correction)) {
                return false;
            }

            struct RootTarget
            {
                int index{ -1 };
                RE::NiTransform local{};
                RE::NiTransform world{};
            };
            std::array<RootTarget, kMaxBindings> roots{};
            std::size_t rootCount = 0;

            for (std::size_t i = 0; i < s_cache.bindingCount; ++i) {
                const auto& binding = s_cache.bindings[i];
                if (!bindingSelectedForTree(binding, destination, requestedFlags)) {
                    continue;
                }
                const int index = destination ? binding.destinationIndex : binding.sourceIndex;
                if (index < 0 || index >= tree.numTransforms) {
                    return false;
                }
                const auto& transform = tree.transforms[index];
                if (parentIsSelected(transform.parPos, destination, requestedFlags)) {
                    continue;
                }
                if (rootCount >= roots.size()) {
                    return false;
                }

                RE::NiTransform nativeRootWorld{};
                if (!readLogicalWorld(tree, index, nativeRootWorld)) {
                    return false;
                }
                auto& target = roots[rootCount++];
                target.index = index;
                target.world = transform_math::composeTransforms(correction, nativeRootWorld);
                if (!finiteTransform(target.world)) {
                    return false;
                }

                if (transform.parPos >= 0 && transform.parPos < tree.numTransforms) {
                    const auto& logicalParent = tree.transforms[transform.parPos];
                    const RE::NiTransform& parentWorld = logicalParent.refNode ? logicalParent.refNode->world : logicalParent.world;
                    if (!finiteTransform(parentWorld)) {
                        return false;
                    }
                    target.local = transform_math::composeTransforms(
                        transform_math::invertTransform(parentWorld),
                        target.world);
                } else {
                    target.local = target.world;
                }
                if (!finiteTransform(target.local)) {
                    return false;
                }
            }

            if (rootCount == 0) {
                return false;
            }
            for (std::size_t i = 0; i < rootCount; ++i) {
                const auto& target = roots[i];
                auto& transform = tree.transforms[target.index];
                transform.local = target.local;
                if (transform.refNode && !usesDifferentSceneParent(tree, target.index)) {
                    transform.refNode->local = target.local;
                    f4vr::updateTransformsDown(transform.refNode, true);
                    transform.world = transform.refNode->world;
                } else {
                    transform.world = target.world;
                }
            }

            if (initializedBaseline) {
                ROCK_LOG_INFO(Animation,
                    "Native animation shared weapon frame ready tree=first-person roots={} controlWeaponT=({:.3f},{:.3f},{:.3f}) nativeWeaponT=({:.3f},{:.3f},{:.3f}) sharedWeaponT=({:.3f},{:.3f},{:.3f})",
                    rootCount,
                    aimFrame.controlWeaponWorld.translate.x,
                    aimFrame.controlWeaponWorld.translate.y,
                    aimFrame.controlWeaponWorld.translate.z,
                    nativeWeaponWorld.translate.x,
                    nativeWeaponWorld.translate.y,
                    nativeWeaponWorld.translate.z,
                    aimFrame.desiredWeaponWorld.translate.x,
                    aimFrame.desiredWeaponWorld.translate.y,
                    aimFrame.desiredWeaponWorld.translate.z);
            } else if (destination && !aimFrame.destinationAlignmentLogged) {
                const RE::NiTransform resolvedWeaponWorld = transform_math::composeTransforms(
                    correction,
                    nativeWeaponWorld);
                ROCK_LOG_INFO(Animation,
                    "Native animation shared weapon frame applied tree=full-body roots={} nativeWeaponT=({:.3f},{:.3f},{:.3f}) sharedWeaponT=({:.3f},{:.3f},{:.3f}) resolvedWeaponT=({:.3f},{:.3f},{:.3f})",
                    rootCount,
                    nativeWeaponWorld.translate.x,
                    nativeWeaponWorld.translate.y,
                    nativeWeaponWorld.translate.z,
                    aimFrame.desiredWeaponWorld.translate.x,
                    aimFrame.desiredWeaponWorld.translate.y,
                    aimFrame.desiredWeaponWorld.translate.z,
                    resolvedWeaponWorld.translate.x,
                    resolvedWeaponWorld.translate.y,
                    resolvedWeaponWorld.translate.z);
                aimFrame.destinationAlignmentLogged = true;
            }
            return true;
        }

        void synchronizeFlattenedWorlds(BoneTree& tree, bool destination, std::uint32_t requestedFlags)
        {
            for (std::size_t i = 0; i < s_cache.bindingCount; ++i) {
                const auto& binding = s_cache.bindings[i];
                if (!bindingSelectedForTree(binding, destination, requestedFlags)) {
                    continue;
                }
                const int index = destination ? binding.destinationIndex : binding.sourceIndex;
                if (index < 0 || index >= tree.numTransforms) {
                    continue;
                }

                auto& transform = tree.transforms[index];
                if (transform.refNode && !usesDifferentSceneParent(tree, index)) {
                    transform.world = transform.refNode->world;
                } else if (!transform.refNode && transform.parPos >= 0 && transform.parPos < tree.numTransforms) {
                    transform.world = transform_math::composeTransforms(
                        tree.transforms[transform.parPos].world,
                        transform.local);
                } else if (!transform.refNode) {
                    transform.world = transform.local;
                }
            }
        }

        void rebaseDifferentSceneParents(BoneTree& tree, bool destination, std::uint32_t requestedFlags)
        {
            for (std::size_t i = 0; i < s_cache.bindingCount; ++i) {
                const auto& binding = s_cache.bindings[i];
                if (!bindingSelectedForTree(binding, destination, requestedFlags)) {
                    continue;
                }
                const int index = destination ? binding.destinationIndex : binding.sourceIndex;
                if (index < 0 || index >= tree.numTransforms || !usesDifferentSceneParent(tree, index)) {
                    continue;
                }

                auto& transform = tree.transforms[index];
                auto* refNode = transform.refNode;
                const auto& logicalParent = tree.transforms[transform.parPos];
                const RE::NiTransform& logicalParentWorld = logicalParent.refNode ? logicalParent.refNode->world : logicalParent.world;
                const RE::NiTransform logicalTargetWorld = transform_math::composeTransforms(
                    logicalParentWorld,
                    transform.local);

                if (refNode->parent) {
                    refNode->local = transform_math::composeTransforms(
                        transform_math::invertTransform(refNode->parent->world),
                        logicalTargetWorld);
                } else {
                    refNode->local = logicalTargetWorld;
                }
                f4vr::updateTransformsDown(refNode, true);
                transform.world = refNode->world;
            }
        }

        [[nodiscard]] bool applyToTree(BoneTree* tree, bool destination, std::uint32_t requestedFlags)
        {
            if (!validTree(tree)) {
                return false;
            }
            const bool identityMatches = destination ?
                tree == s_cache.destinationTree && tree->transforms == s_cache.destinationTransforms && tree->numTransforms == s_cache.destinationCount :
                tree == s_cache.sourceTree && tree->transforms == s_cache.sourceTransforms && tree->numTransforms == s_cache.sourceCount;
            if (!identityMatches) {
                return false;
            }

            writeCapturedLocals(*tree, destination, requestedFlags);
            propagateSelectedRoots(*tree, destination, requestedFlags);
            synchronizeFlattenedWorlds(*tree, destination, requestedFlags);
            if (!applyControllerAimFrame(*tree, destination, requestedFlags)) {
                return false;
            }
            synchronizeFlattenedWorlds(*tree, destination, requestedFlags);
            // hFRIK may retain Weapon/WeaponLeft under a wand node. Preserve
            // the corrected logical pose, but convert its target world into
            // that live scene parent's local before presentation.
            rebaseDifferentSceneParents(*tree, destination, requestedFlags);
            return true;
        }

        [[nodiscard]] const char* localReloadLeaseEndReasonName(
            native_animation_authority_policy::LocalReloadLeaseEndReason reason)
        {
            using Reason = native_animation_authority_policy::LocalReloadLeaseEndReason;
            switch (reason) {
            case Reason::ReloadEnded:
                return "native reload-end event observed";
            case Reason::WatchdogExpired:
                return "watchdog expired";
            case Reason::None:
            default:
                return "active";
            }
        }

        [[nodiscard]] bool refreshLocalReloadTestLease()
        {
            const auto requestSequence = s_localReloadTestRequestSequence.load(std::memory_order_acquire);
            const bool requestChanged = requestSequence != s_seenLocalReloadTestRequestSequence;
            if (requestChanged) {
                s_seenLocalReloadTestRequestSequence = requestSequence;
                s_localReloadLeaseState = native_animation_authority_policy::LocalReloadLeaseState{
                    .watchdogFramesRemaining = s_localReloadTestLeaseFrames.load(std::memory_order_acquire),
                    .startSequenceAtArm = s_playerReloadStartSequence.load(std::memory_order_acquire),
                    .endSequenceAtArm = s_playerReloadEndSequence.load(std::memory_order_acquire),
                    .observedReloadStart = s_playerReloadEventActive.load(std::memory_order_acquire),
                };
            }

            if (s_localReloadTestLeaseFrames.load(std::memory_order_acquire) == 0) {
                return requestChanged;
            }

            const bool wasReloadStartObserved = s_localReloadLeaseState.observedReloadStart;
            const auto step = native_animation_authority_policy::advanceLocalReloadLease(
                s_localReloadLeaseState,
                native_animation_authority_policy::LocalReloadLifecycleSignal{
                    .startSequence = s_playerReloadStartSequence.load(std::memory_order_acquire),
                    .endSequence = s_playerReloadEndSequence.load(std::memory_order_acquire),
                    .reloadActive = s_playerReloadEventActive.load(std::memory_order_acquire),
                });
            s_localReloadLeaseState = step.state;
            if (!wasReloadStartObserved && step.state.observedReloadStart) {
                ROCK_LOG_INFO(Animation,
                    "Native reload animation authority observed Bethesda's player reload-start event; exact-end return armed");
            }

            if (step.active()) {
                s_localReloadTestLeaseFrames.store(step.state.watchdogFramesRemaining, std::memory_order_release);
            } else {
                s_localReloadTestLeaseFrames.store(0, std::memory_order_release);
                ROCK_LOG_INFO(Animation,
                    "Native reload animation authority local test lease released: {}",
                    localReloadLeaseEndReasonName(step.endReason));
            }
            return requestChanged;
        }

        [[nodiscard]] const char* localManualCycleLeaseEndReasonName(
            native_animation_authority_policy::LocalManualCycleLeaseEndReason reason)
        {
            using Reason = native_animation_authority_policy::LocalManualCycleLeaseEndReason;
            switch (reason) {
            case Reason::CycleBracketEnded:
                return "native bolt/lever clip end observed";
            case Reason::ReloadStarted:
                return "native reload-start event preempted cycle";
            case Reason::WatchdogExpired:
                return "animation-duration watchdog expired";
            case Reason::None:
            default:
                return "active";
            }
        }

        [[nodiscard]] bool refreshLocalManualCycleTestLease(const float deltaSeconds)
        {
            const auto requestSequence =
                s_localManualCycleTestRequestSequence.load(std::memory_order_acquire);
            const bool requestChanged =
                requestSequence != s_seenLocalManualCycleTestRequestSequence;
            if (requestChanged) {
                s_seenLocalManualCycleTestRequestSequence = requestSequence;
                const auto watchdogMilliseconds =
                    s_localManualCycleRequestedWatchdogMilliseconds.load(
                        std::memory_order_acquire);
                s_localManualCycleLeaseState =
                    native_animation_authority_policy::LocalManualCycleLeaseState{
                        .watchdogSecondsRemaining =
                            static_cast<float>(watchdogMilliseconds) / 1000.0f,
                        .reloadStartSequenceAtArm =
                            s_localManualCycleReloadStartSequenceAtArm.load(
                                std::memory_order_acquire),
                        .lastReloadEndSequence =
                            s_localManualCycleReloadEndSequenceAtArm.load(
                                std::memory_order_acquire),
                    };
            }

            if (!s_localManualCycleTestLeaseActive.load(std::memory_order_acquire)) {
                return requestChanged;
            }

            const auto step =
                native_animation_authority_policy::advanceLocalManualCycleLease(
                    s_localManualCycleLeaseState,
                    native_animation_authority_policy::LocalManualCycleLifecycleSignal{
                        .reloadStartSequence =
                            s_playerReloadStartSequence.load(std::memory_order_acquire),
                        .reloadEndSequence =
                            s_playerReloadEndSequence.load(std::memory_order_acquire),
                        .deltaSeconds = deltaSeconds,
                    });
            s_localManualCycleLeaseState = step.state;
            if (!step.active()) {
                s_localManualCycleTestLeaseActive.store(false, std::memory_order_release);
                ROCK_LOG_DEBUG(Animation,
                    "Native bolt/lever hand-only authority released: {}",
                    localManualCycleLeaseEndReasonName(step.endReason));
            }
            return requestChanged;
        }

        [[nodiscard]] bool installWeaponFireHook()
        {
            if (s_weaponFireHookInstalled.load(std::memory_order_acquire)) {
                return s_originalWeaponFire != nullptr;
            }
            if (s_weaponFireHookInstallFailed.load(std::memory_order_acquire)) {
                return false;
            }

            REL::Relocation<std::uintptr_t> entry{
                REL::Offset(offsets::kVtableEntry_WeaponFireHandler_Handle)
            };
            REL::Relocation<std::uintptr_t> expectedTarget{
                REL::Offset(offsets::kFunc_WeaponFireHandler_Handle)
            };
            auto* slot = reinterpret_cast<std::uintptr_t*>(entry.address());
            if (!slot || *slot != expectedTarget.address()) {
                ROCK_LOG_ERROR(Init,
                    "WeaponFireHandler hook validation failed at 0x{:X}; expected target 0x{:X}, found 0x{:X}",
                    entry.address(),
                    expectedTarget.address(),
                    slot ? *slot : 0);
                s_weaponFireHookInstallFailed.store(true, std::memory_order_release);
                return false;
            }

            s_originalWeaponFire = reinterpret_cast<WeaponFireHandlerFn>(*slot);
            DWORD oldProtect = 0;
            if (!VirtualProtect(slot, sizeof(*slot), PAGE_EXECUTE_READWRITE, &oldProtect)) {
                ROCK_LOG_ERROR(Init,
                    "WeaponFireHandler hook install failed at 0x{:X}: VirtualProtect failed",
                    entry.address());
                s_originalWeaponFire = nullptr;
                s_weaponFireHookInstallFailed.store(true, std::memory_order_release);
                return false;
            }

            *slot = reinterpret_cast<std::uintptr_t>(&onWeaponFire);
            FlushInstructionCache(GetCurrentProcess(), slot, sizeof(*slot));
            DWORD unusedProtect = 0;
            if (!VirtualProtect(slot, sizeof(*slot), oldProtect, &unusedProtect)) {
                ROCK_LOG_WARN(Init,
                    "WeaponFireHandler hook installed at 0x{:X}, but restoring page protection failed",
                    entry.address());
            }

            s_weaponFireHookInstalled.store(true, std::memory_order_release);
            ROCK_LOG_INFO(Init,
                "Installed validated WeaponFireHandler manual-cycle hook at 0x{:X}, original=0x{:X}",
                entry.address(),
                reinterpret_cast<std::uintptr_t>(s_originalWeaponFire));
            return true;
        }

        [[nodiscard]] bool installReloadStateChangeHook()
        {
            if (s_reloadStateHookInstalled.load(std::memory_order_acquire)) {
                return s_originalReloadStateChange != nullptr;
            }

            REL::Relocation<std::uintptr_t> entry{
                REL::Offset(offsets::kVtableEntry_ReloadStateChangeHandler_Handle)
            };
            REL::Relocation<std::uintptr_t> expectedTarget{
                REL::Offset(offsets::kFunc_ReloadStateChangeHandler_Handle)
            };
            auto* slot = reinterpret_cast<std::uintptr_t*>(entry.address());
            if (!slot || *slot != expectedTarget.address()) {
                ROCK_LOG_ERROR(Init,
                    "ReloadStateChangeHandler hook validation failed at 0x{:X}; expected target 0x{:X}, found 0x{:X}",
                    entry.address(),
                    expectedTarget.address(),
                    slot ? *slot : 0);
                return false;
            }

            s_originalReloadStateChange = reinterpret_cast<ReloadStateChangeHandlerFn>(*slot);
            DWORD oldProtect = 0;
            if (!VirtualProtect(slot, sizeof(*slot), PAGE_EXECUTE_READWRITE, &oldProtect)) {
                ROCK_LOG_ERROR(Init,
                    "ReloadStateChangeHandler hook install failed at 0x{:X}: VirtualProtect failed",
                    entry.address());
                s_originalReloadStateChange = nullptr;
                return false;
            }

            *slot = reinterpret_cast<std::uintptr_t>(&onReloadStateChange);
            FlushInstructionCache(GetCurrentProcess(), slot, sizeof(*slot));
            DWORD unusedProtect = 0;
            if (!VirtualProtect(slot, sizeof(*slot), oldProtect, &unusedProtect)) {
                ROCK_LOG_WARN(Init,
                    "ReloadStateChangeHandler hook installed at 0x{:X}, but restoring page protection failed",
                    entry.address());
            }

            s_reloadStateHookInstalled.store(true, std::memory_order_release);
            ROCK_LOG_INFO(Init,
                "Installed validated ReloadStateChangeHandler lifecycle hook at 0x{:X}, original=0x{:X}",
                entry.address(),
                reinterpret_cast<std::uintptr_t>(s_originalReloadStateChange));
            return true;
        }

        [[nodiscard]] bool installPrimaryFiringGripCaptureHook()
        {
            if (s_primaryFiringGripHookInstalled.load(std::memory_order_acquire)) {
                return true;
            }
            if (s_primaryFiringGripHookInstallFailed.load(std::memory_order_acquire)) {
                return false;
            }

            // MOV RAX,RSP; PUSH RBP; PUSH RBX; PUSH R14;
            // LEA RBP,[RAX-0x108]. All stolen instructions are position
            // independent and were verified against FO4VR 1.2.72.
            constexpr std::array<std::uint8_t, 14> kExpectedUpdateFirstPersonArmPrefix{
                0x48, 0x8B, 0xC4, 0x55, 0x53, 0x41, 0x56,
                0x48, 0x8D, 0xA8, 0xF8, 0xFE, 0xFF, 0xFF,
            };

            REL::Relocation<std::uintptr_t> primaryReturn{
                REL::Offset(offsets::kCallsite_UpdateFirstPersonArmPrimaryReturn)
            };
            REL::Relocation<std::uintptr_t> supportReturn{
                REL::Offset(offsets::kCallsite_UpdateFirstPersonArmSecondaryReturn)
            };
            if (primaryReturn.address() == 0 || supportReturn.address() == 0) {
                s_primaryFiringGripHookInstallFailed.store(true, std::memory_order_release);
                return false;
            }

            void* original = nullptr;
            const bool installed = entry_trampoline_hook::install(
                "native authored firing-grip arm passes",
                offsets::kFunc_UpdateFirstPersonArm,
                kExpectedUpdateFirstPersonArmPrefix.data(),
                kExpectedUpdateFirstPersonArmPrefix.size(),
                reinterpret_cast<void*>(&onUpdateFirstPersonArm),
                original);
            if (!installed || !original) {
                s_primaryFiringGripHookInstallFailed.store(true, std::memory_order_release);
                return false;
            }

            s_originalUpdateFirstPersonArm = reinterpret_cast<UpdateFirstPersonArmFn>(original);
            s_nativePrimaryArmReturnAddress = primaryReturn.address();
            s_nativeSupportArmReturnAddress = supportReturn.address();
            s_primaryFiringGripHookInstalled.store(true, std::memory_order_release);
            ROCK_LOG_INFO(Init,
                "Native authored firing-grip capture ready; helper=0x{:X} BethesdaPrimaryReturn=0x{:X} BethesdaSupportReturn=0x{:X} source=pre-hFRIK-hand-in-weapon",
                REL::Relocation<std::uintptr_t>{ REL::Offset(offsets::kFunc_UpdateFirstPersonArm) }.address(),
                s_nativePrimaryArmReturnAddress,
                s_nativeSupportArmReturnAddress);
            return true;
        }
    }

    bool installPostUpdateHook()
    {
        if (s_hookInstalled.load(std::memory_order_acquire)) {
            if (!s_weaponFireHookInstalled.load(std::memory_order_acquire) &&
                !s_weaponFireHookInstallFailed.load(std::memory_order_acquire)) {
                if (!installWeaponFireHook()) {
                    ROCK_LOG_WARN(Init,
                        "Native reload authority remains available, but bolt/lever hand-only animation is disabled because WeaponFireHandler was not installed");
                }
            }
            if (!s_primaryFiringGripHookInstalled.load(std::memory_order_acquire) &&
                !s_primaryFiringGripHookInstallFailed.load(std::memory_order_acquire)) {
                if (!installPrimaryFiringGripCaptureHook()) {
                    ROCK_LOG_WARN(Init,
                        "Native reload authority remains available, but the authored firing-grip experiment is disabled because its paired native arm hook was not installed");
                }
            }
            return true;
        }
        if (s_hookInstallFailed.load(std::memory_order_acquire)) {
            return false;
        }
        if (!installReloadStateChangeHook()) {
            s_hookInstallFailed.store(true, std::memory_order_release);
            return false;
        }

        /*
         * Fallout4VR.exe 1.2.72 PlayerCharacter::PostUpdateAnimationGraphManager
         * starts with this eight-byte prologue at module+0xF2F0A0. hFRIK then
         * NOPs the native body-reset/first-to-third bridge beginning at +0x8.
         * Installing after kSkeletonReady proves both identities at once.
         */
        constexpr std::array<std::uint8_t, 14> kExpectedPostFrikPrefix{
            0x48, 0x8B, 0xC4, 0x55, 0x48, 0x83, 0xEC, 0x60,
            0x90, 0x90, 0x90, 0x90, 0x90, 0x90,
        };

        void* original = nullptr;
        const bool installed = entry_trampoline_hook::install(
            "native animation authority PostUpdateAnimationGraphManager",
            offsets::kFunc_PlayerPostUpdateAnimationGraphManager,
            kExpectedPostFrikPrefix.data(),
            kExpectedPostFrikPrefix.size(),
            reinterpret_cast<void*>(&onPostUpdateAnimationGraphManager),
            original);
        if (!installed || !original) {
            s_hookInstallFailed.store(true, std::memory_order_release);
            return false;
        }

        s_originalPostUpdate = reinterpret_cast<PostUpdateAnimationGraphManagerFn>(original);
        s_hookInstalled.store(true, std::memory_order_release);
        if (!installWeaponFireHook()) {
            ROCK_LOG_WARN(Init,
                "Native reload authority remains available, but bolt/lever hand-only animation is disabled because WeaponFireHandler was not installed");
        }
        if (!installPrimaryFiringGripCaptureHook()) {
            ROCK_LOG_WARN(Init,
                "Native reload authority remains available, but the authored firing-grip experiment is disabled because its paired native arm hook was not installed");
        }
        ROCK_LOG_INFO(Init,
            "Native animation authority hooks ready; scope=arms,hands,Weapon/WeaponLeft lifecycle=WeaponFireHandler+ReloadStateChangeHandler API=v1");
        return true;
    }

    void setRuntimeEnabled(const bool enabled)
    {
        s_runtimeEnabled.store(enabled && s_hookInstalled.load(std::memory_order_acquire), std::memory_order_release);
        if (!enabled) {
            cancelLocalManualCycleTestLease();
            const DWORD ownerThread =
                s_ownerThreadId.load(std::memory_order_acquire);
            if (ownerThread == 0 || ownerThread == GetCurrentThreadId()) {
                clearManualCycleVisualAuthorityPreservingWeapon();
            }
            invalidateCapture();
            s_frameCaptureReady = false;
            resetHybridPoseState();
        }
    }

    void setLocalManualCycleTestEnabled(const bool enabled)
    {
        const bool effectiveEnabled = enabled &&
            s_hookInstalled.load(std::memory_order_acquire) &&
            s_weaponFireHookInstalled.load(std::memory_order_acquire);
        s_localManualCycleTestEnabled.store(effectiveEnabled, std::memory_order_release);
        if (!effectiveEnabled) {
            cancelLocalManualCycleTestLease();
        }
    }

    void setPrimaryFiringGripCaptureEnabled(const bool enabled)
    {
        const bool effectiveEnabled = enabled &&
            s_primaryFiringGripHookInstalled.load(std::memory_order_acquire);
        const bool wasEnabled = s_primaryFiringGripCaptureEnabled.exchange(
            effectiveEnabled,
            std::memory_order_acq_rel);
        if (!effectiveEnabled) {
            invalidatePrimaryFiringGripCapture();
            invalidateAuthoredSupportGraphPose();
            invalidateAuthoredSupportGripCapture();
        } else if (!wasEnabled) {
            invalidateAuthoredSupportGraphPose();
            invalidateAuthoredSupportGripCapture();
            recordAuthoredSupportGripCaptureFailure(
                AuthoredSupportGripCaptureFailureReason::SecondaryPassNotObserved);
        }
    }

    PrimaryFiringGripCaptureStatus queryPrimaryFiringGripCaptureStatus()
    {
        return PrimaryFiringGripCaptureStatus{
            .captureSequence = s_primaryFiringGripCaptureSequence.load(std::memory_order_acquire),
            .valid = s_primaryFiringGripCaptureValid.load(std::memory_order_acquire),
        };
    }

    AuthoredSupportGripCaptureStatus queryAuthoredSupportGripCaptureStatus()
    {
        return AuthoredSupportGripCaptureStatus{
            .captureSequence = s_authoredSupportGripCaptureSequence.load(std::memory_order_acquire),
            .secondaryPassSequence =
                s_authoredSupportGripSecondaryPassSequence.load(std::memory_order_acquire),
            .failureReason =
                s_authoredSupportGripCaptureFailureReason.load(std::memory_order_acquire),
            .invalidOrMissingFingerMask =
                s_authoredSupportGripInvalidOrMissingFingerMask.load(std::memory_order_acquire),
            .valid = s_authoredSupportGripCaptureValid.load(std::memory_order_acquire),
        };
    }

    const char* authoredSupportGripCaptureFailureReasonName(
        const AuthoredSupportGripCaptureFailureReason reason)
    {
        switch (reason) {
        case AuthoredSupportGripCaptureFailureReason::None:
            return "none";
        case AuthoredSupportGripCaptureFailureReason::SecondaryPassNotObserved:
            return "secondary-pass-not-observed";
        case AuthoredSupportGripCaptureFailureReason::SourceTreeUnavailable:
            return "source-tree-unavailable";
        case AuthoredSupportGripCaptureFailureReason::BoneCacheIncomplete:
            return "bone-cache-incomplete";
        case AuthoredSupportGripCaptureFailureReason::TopologyInvalid:
            return "topology-invalid";
        case AuthoredSupportGripCaptureFailureReason::PrimaryPoseUnavailable:
            return "primary-pose-unavailable";
        case AuthoredSupportGripCaptureFailureReason::AuthoredGraphPoseUnavailable:
            return "authored-graph-pose-unavailable";
        case AuthoredSupportGripCaptureFailureReason::AuthoredHandHierarchyInvalid:
            return "authored-hand-hierarchy-invalid";
        case AuthoredSupportGripCaptureFailureReason::SupportHandTransformInvalid:
            return "support-hand-transform-invalid";
        case AuthoredSupportGripCaptureFailureReason::FingerTransformInvalid:
            return "finger-transform-invalid";
        case AuthoredSupportGripCaptureFailureReason::ThreadMismatch:
            return "thread-mismatch";
        case AuthoredSupportGripCaptureFailureReason::CaptureFault:
            return "capture-fault";
        }
        return "unknown";
    }

    bool tryResolvePrimaryFiringGripAlignment(
        const RE::NiNode* expectedWeaponNode,
        const RE::NiTransform& liveWeaponWorld,
        const RE::NiTransform& trackedPrimaryHandWorld,
        RE::NiTransform& outWeaponWorld,
        RE::NiTransform& outCurrentAuthoredHandWorld,
        RE::NiTransform& outAuthoredPrimaryHandInWeapon,
        std::uint64_t& outCaptureSequence)
    {
        outAuthoredPrimaryHandInWeapon = {};
        outCaptureSequence = 0;
        auto* const capturedHandNode =
            s_primaryFiringGripHandNode.load(std::memory_order_acquire);
        if (!expectedWeaponNode ||
            !s_primaryFiringGripCaptureEnabled.load(std::memory_order_acquire) ||
            !s_primaryFiringGripCaptureValid.load(std::memory_order_acquire) ||
            !claimOrValidateThread() ||
            !capturedHandNode ||
            expectedWeaponNode != s_primaryFiringGripWeaponNode.load(std::memory_order_acquire) ||
            expectedWeaponNode->parent != capturedHandNode ||
            !finiteTransform(liveWeaponWorld) ||
            !finiteTransform(trackedPrimaryHandWorld) ||
            !finiteTransform(s_authoredPrimaryHandInWeapon)) {
            return false;
        }

        const auto sequence = s_primaryFiringGripCaptureSequence.load(std::memory_order_acquire);
        if (sequence == 0) {
            return false;
        }

        // Publish the exact captured RArm_Hand-in-Weapon relation alongside
        // the world solve. ROCK's ambidextrous firing path generation-binds
        // this same datum and mirrors it only when the physical left hand
        // takes the firing role; it must never reconstruct the canonical from
        // presentation-world transforms after hFRIK has moved the weapon.
        outAuthoredPrimaryHandInWeapon = s_authoredPrimaryHandInWeapon;
        outCurrentAuthoredHandWorld = native_animation_authority_policy::resolveAuthoredPrimaryHandWorld(
            liveWeaponWorld,
            outAuthoredPrimaryHandInWeapon,
            [](const RE::NiTransform& parent, const RE::NiTransform& child) {
                return transform_math::composeTransforms(parent, child);
            });
        outWeaponWorld = native_animation_authority_policy::resolveAuthoredPrimaryWeaponWorld(
            trackedPrimaryHandWorld,
            outAuthoredPrimaryHandInWeapon,
            [](const RE::NiTransform& parent, const RE::NiTransform& child) {
                return transform_math::composeTransforms(parent, child);
            },
            [](const RE::NiTransform& transform) {
                return transform_math::invertTransform(transform);
            });
        if (!finiteTransform(outCurrentAuthoredHandWorld) ||
            !finiteTransform(outWeaponWorld)) {
            return false;
        }

        outCaptureSequence = sequence;
        return true;
    }

    bool tryResolveAuthoredSupportGrip(
        const RE::NiNode* expectedWeaponNode,
        RE::NiTransform& outSupportHandInWeapon,
        std::array<RE::NiTransform, 15>& outFingerLocalTransforms,
        std::uint16_t& outFingerLocalTransformMask,
        std::uint64_t& outCaptureSequence)
    {
        outFingerLocalTransformMask = 0;
        outCaptureSequence = 0;
        auto* const capturedPrimaryHandNode =
            s_authoredSupportGripPrimaryHandNode.load(std::memory_order_acquire);
        if (!expectedWeaponNode ||
            !s_primaryFiringGripCaptureEnabled.load(std::memory_order_acquire) ||
            !s_authoredSupportGripCaptureValid.load(std::memory_order_acquire) ||
            !claimOrValidateThread() ||
            !capturedPrimaryHandNode ||
            expectedWeaponNode != s_authoredSupportGripWeaponNode.load(std::memory_order_acquire) ||
            expectedWeaponNode->parent != capturedPrimaryHandNode ||
            !finiteTransform(s_authoredSupportHandInWeapon)) {
            return false;
        }

        /*
         * The live resolver is valid only while the scene still has the
         * right-primary topology whose world transforms produced this value.
         * AuthoredPrimaryFiringGripRuntime copies the value before physical-
         * left takeover and republishes that generation-bound snapshot after
         * ROCK reparents Weapon. Accepting a new capture after that reparent
         * converts ROCK's presentation world into an enormous false local.
         */

        for (const auto& fingerLocal : s_authoredSupportFingerLocals) {
            if (!finiteTransform(fingerLocal)) {
                return false;
            }
        }

        const auto sequence =
            s_authoredSupportGripCaptureSequence.load(std::memory_order_acquire);
        if (sequence == 0) {
            return false;
        }

        outSupportHandInWeapon = s_authoredSupportHandInWeapon;
        outFingerLocalTransforms = s_authoredSupportFingerLocals;
        outFingerLocalTransformMask = kAuthoredSupportFingerTransformMask;
        outCaptureSequence = sequence;
        return true;
    }

    void requestLocalReloadTestLease()
    {
        if (!s_runtimeEnabled.load(std::memory_order_acquire) || !s_hookInstalled.load(std::memory_order_acquire)) {
            ROCK_LOG_WARN(Animation,
                "Native reload animation authority test lease ignored because the capture hook/skeleton is not ready");
            return;
        }
        cancelLocalManualCycleTestLease();
        s_localReloadTestLeaseFrames.store(kLocalReloadTestLeaseFrames, std::memory_order_release);
        s_localReloadTestRequestSequence.fetch_add(1, std::memory_order_acq_rel);
        ROCK_LOG_INFO(Animation,
            "Native reload animation authority local test lease armed; exact end follows Bethesda reload events, watchdog={} ROCK frames",
            kLocalReloadTestLeaseFrames);
    }

    void beginRockFrame(const float deltaSeconds)
    {
        provider::refreshNativeAnimationAuthorityLeasesV1();
        const bool localReloadRequestChanged = refreshLocalReloadTestLease();
        const bool localManualCycleRequestChanged =
            refreshLocalManualCycleTestLease(deltaSeconds);
        s_frameCaptureReady = false;
        s_frameCaptureFlags = 0;
        s_frameCaptureSequence = 0;
        s_frameManualCycleExpected = false;
        s_frameManualCycleApplied = false;

        const std::uint32_t currentFlags = effectiveRequestedFlags();
        const std::uint32_t previousFlags = s_lastLoggedEffectiveFlags;
        // Removing last frame's IK target moves the arm back under its lower
        // controller/grip authority. Preserve and rebase the Weapon across
        // that transition before any authority-edge reset drops its node.
        clearManualCycleVisualAuthorityPreservingWeapon();
        if (localReloadRequestChanged || localManualCycleRequestChanged ||
            currentFlags != previousFlags) {
            resetHybridPoseState();
        }
        s_frameManualCycleExpected = currentFlags != 0 &&
            (currentFlags & native_animation_authority_policy::kWeapon) == 0 &&
            (currentFlags & native_animation_authority_policy::kArms) != 0;
        if (currentFlags != s_lastLoggedEffectiveFlags) {
            const char* composition = s_frameManualCycleExpected ?
                "post-rock-weapon-anchored-hand-ik" :
                "visible-weapon-shared-rigid-pose";
            ROCK_LOG_INFO(Animation,
                "Native animation authority {} flags=0x{:X} composition={}",
                currentFlags != 0 ? "enabled" : "disabled",
                currentFlags,
                composition);
            s_lastLoggedEffectiveFlags = currentFlags;
        }

        if (currentFlags == 0 || !s_captureValid.load(std::memory_order_acquire) || !claimOrValidateThread()) {
            if (s_frameManualCycleExpected) {
                clearManualCycleVisualAuthority();
            }
            return;
        }

        const auto sequence = s_captureSequence.load(std::memory_order_acquire);
        const auto capturedFlags = s_capturedFlags.load(std::memory_order_acquire) & currentFlags;
        if (sequence == 0 || sequence == s_lastCompletedCaptureSequence || capturedFlags == 0) {
            if (s_frameManualCycleExpected) {
                clearManualCycleVisualAuthority();
            }
            return;
        }
        const bool framePrepared = s_frameManualCycleExpected ?
            prepareManualCycleWeaponNode() :
            prepareControllerAimFrames();
        if (!framePrepared) {
            clearManualCycleVisualAuthority();
            invalidateCapture();
            return;
        }

        s_frameCaptureSequence = sequence;
        s_frameCaptureFlags = capturedFlags;
        s_frameCaptureReady = true;
    }

    bool applyCapturedPose(const ApplyPhase phase)
    {
        if (!s_frameCaptureReady || s_frameCaptureFlags == 0 || !claimOrValidateThread()) {
            return false;
        }

        if (s_frameManualCycleExpected) {
            // ROCK's collision/grab/two-hand pass must see only controller-
            // tracked hands. Publishing the animated IK target before that
            // pass feeds the clip back into the weapon solve and rotates the
            // gun away from the controllers.
            if (phase == ApplyPhase::BeforeRock) {
                return true;
            }

            bool applied = false;
#if defined(_MSC_VER)
            __try {
                applied = applyManualCyclePoseAfterRock();
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                clearManualCycleVisualAuthority();
                s_captureFault.store(true, std::memory_order_release);
            }
#else
            applied = applyManualCyclePoseAfterRock();
#endif
            if (!applied) {
                s_frameCaptureReady = false;
                invalidateCapture();
                return false;
            }
            s_frameManualCycleApplied = true;
            return true;
        }

        bool sourceApplied = false;
        bool destinationApplied = false;
#if defined(_MSC_VER)
        __try {
            sourceApplied = applyToTree(
                s_cache.sourceTree,
                false,
                s_frameCaptureFlags);
            destinationApplied =
                s_cache.destinationTree == s_cache.sourceTree ?
                sourceApplied :
                sourceApplied && applyToTree(
                    s_cache.destinationTree,
                    true,
                    s_frameCaptureFlags);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            s_captureFault.store(true, std::memory_order_release);
            invalidateCapture();
            s_frameCaptureReady = false;
            return false;
        }
#else
        sourceApplied = applyToTree(
            s_cache.sourceTree,
            false,
            s_frameCaptureFlags);
        destinationApplied =
            s_cache.destinationTree == s_cache.sourceTree ?
            sourceApplied :
            sourceApplied && applyToTree(
                s_cache.destinationTree,
                true,
                s_frameCaptureFlags);
#endif
        if (!sourceApplied || !destinationApplied) {
            s_frameCaptureReady = false;
            invalidateCapture();
            return false;
        }
        return true;
    }

    void completeRockFrame()
    {
        if (s_frameManualCycleExpected && !s_frameManualCycleApplied) {
            clearManualCycleVisualAuthority();
        }
        if (s_frameCaptureReady) {
            s_lastCompletedCaptureSequence = s_frameCaptureSequence;
        }
        s_frameCaptureReady = false;
        s_frameCaptureFlags = 0;
        s_frameCaptureSequence = 0;
        s_frameManualCycleExpected = false;
        s_frameManualCycleApplied = false;
    }

    void resetTransientState()
    {
        s_runtimeEnabled.store(false, std::memory_order_release);
        s_localManualCycleTestEnabled.store(false, std::memory_order_release);
        s_primaryFiringGripCaptureEnabled.store(false, std::memory_order_release);
        s_localReloadTestLeaseFrames.store(0, std::memory_order_release);
        cancelLocalManualCycleTestLease();
        s_localReloadLeaseState = {};
        s_localManualCycleLeaseState = {};
        s_seenLocalReloadTestRequestSequence = s_localReloadTestRequestSequence.load(std::memory_order_acquire);
        s_seenLocalManualCycleTestRequestSequence =
            s_localManualCycleTestRequestSequence.load(std::memory_order_acquire);
        s_playerReloadEventActive.store(false, std::memory_order_release);
        const DWORD ownerThread = s_ownerThreadId.load(std::memory_order_acquire);
        if (ownerThread == 0 || ownerThread == GetCurrentThreadId()) {
            clearManualCycleVisualAuthorityPreservingWeapon();
        }
        invalidateCapture();
        invalidatePrimaryFiringGripCapture();
        invalidateAuthoredSupportGraphPose();
        invalidateAuthoredSupportGripCapture();
        recordAuthoredSupportGripCaptureFailure(
            AuthoredSupportGripCaptureFailureReason::SecondaryPassNotObserved);
        resetHybridPoseState();
        s_frameCaptureReady = false;
        s_frameCaptureFlags = 0;
        s_frameCaptureSequence = 0;
        s_frameManualCycleExpected = false;
        s_frameManualCycleApplied = false;
        s_lastCompletedCaptureSequence = s_captureSequence.load(std::memory_order_acquire);
        s_lastConsumedAuthoredSupportGraphPoseSequence =
            s_authoredSupportGraphPoseSequence.load(std::memory_order_acquire);
        if (ownerThread == 0 || ownerThread == GetCurrentThreadId()) {
            s_cache = {};
            s_primaryFiringGripBoneCache = {};
            s_manualCycleVisualPublications = {};
        }
    }

    bool isHookInstalled()
    {
        return s_hookInstalled.load(std::memory_order_acquire);
    }

    RuntimeStatus queryRuntimeStatus()
    {
        RuntimeStatus result{};
        result.effectiveFlags = effectiveRequestedFlags();
        result.capturedTransformCount = s_capturedTransformCount.load(std::memory_order_acquire);
        result.captureSequence = s_captureSequence.load(std::memory_order_acquire);
        if (s_hookInstalled.load(std::memory_order_acquire)) {
            result.statusFlags |= static_cast<std::uint32_t>(RuntimeStatusFlag::HookInstalled);
        }
        if (s_runtimeEnabled.load(std::memory_order_acquire)) {
            result.statusFlags |= static_cast<std::uint32_t>(RuntimeStatusFlag::RuntimeEnabled);
        }
        if (s_captureValid.load(std::memory_order_acquire)) {
            result.statusFlags |= static_cast<std::uint32_t>(RuntimeStatusFlag::CaptureValid);
        }
        if (s_localReloadTestLeaseFrames.load(std::memory_order_acquire) > 0) {
            result.statusFlags |= static_cast<std::uint32_t>(RuntimeStatusFlag::LocalReloadTestLeaseActive);
        }
        if (s_hookInstallFailed.load(std::memory_order_acquire)) {
            result.statusFlags |= static_cast<std::uint32_t>(RuntimeStatusFlag::HookInstallFailed);
        }
        if (s_threadMismatch.load(std::memory_order_acquire)) {
            result.statusFlags |= static_cast<std::uint32_t>(RuntimeStatusFlag::ThreadMismatch);
        }
        if (s_captureFault.load(std::memory_order_acquire)) {
            result.statusFlags |= static_cast<std::uint32_t>(RuntimeStatusFlag::CaptureFault);
        }
        return result;
    }
}
