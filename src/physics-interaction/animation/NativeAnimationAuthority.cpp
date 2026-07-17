#include "physics-interaction/animation/NativeAnimationAuthority.h"

#include "api/ROCKProviderApi.h"
#include "physics-interaction/animation/NativeAnimationAuthorityPolicy.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"

#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

#include <Windows.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace rock::native_animation_authority
{
    namespace
    {
        using BoneTree = f4vr::BSFlattenedBoneTree;
        using BoneTransform = BoneTree::BoneTransforms;
        using PostUpdateAnimationGraphManagerFn = void (*)(void* holder);

        constexpr std::size_t kMaxBindings = 192;
        constexpr int kMaxFlattenedTransforms = 768;
        constexpr std::uint32_t kLocalReloadTestLeaseFrames = 600;
        constexpr std::uint32_t kImplementedFlags = native_animation_authority_policy::kReloadPose;

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
            std::array<Binding, kMaxBindings> bindings{};
            std::size_t bindingCount{ 0 };
        };

        BindingCache s_cache{};
        PostUpdateAnimationGraphManagerFn s_originalPostUpdate{ nullptr };
        std::atomic<bool> s_hookInstalled{ false };
        std::atomic<bool> s_hookInstallFailed{ false };
        std::atomic<bool> s_runtimeEnabled{ false };
        std::atomic<bool> s_captureValid{ false };
        std::atomic<bool> s_threadMismatch{ false };
        std::atomic<bool> s_captureFault{ false };
        std::atomic<DWORD> s_ownerThreadId{ 0 };
        std::atomic<std::uint32_t> s_localReloadTestLeaseFrames{ 0 };
        std::atomic<std::uint32_t> s_capturedFlags{ 0 };
        std::atomic<std::uint32_t> s_capturedTransformCount{ 0 };
        std::atomic<std::uint64_t> s_captureSequence{ 0 };

        std::uint64_t s_frameCaptureSequence{ 0 };
        std::uint64_t s_lastCompletedCaptureSequence{ 0 };
        std::uint32_t s_frameCaptureFlags{ 0 };
        std::uint32_t s_lastLoggedEffectiveFlags{ 0 };
        bool s_frameCaptureReady{ false };

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
            const std::uint32_t localFlags = s_localReloadTestLeaseFrames.load(std::memory_order_acquire) > 0 ? kImplementedFlags : 0;
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

        [[nodiscard]] bool cacheMatches(BoneTree* source, BoneTree* destination)
        {
            return s_cache.sourceTree == source &&
                   s_cache.destinationTree == destination &&
                   s_cache.sourceTransforms == source->transforms &&
                   s_cache.destinationTransforms == destination->transforms &&
                   s_cache.sourceCount == source->numTransforms &&
                   s_cache.destinationCount == destination->numTransforms &&
                   s_cache.bindingCount > 0;
        }

        [[nodiscard]] bool rebuildCache(BoneTree* source, BoneTree* destination)
        {
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
            }
            return s_cache.bindingCount > 0;
        }

        [[nodiscard]] const RE::NiTransform& authoritativeLocal(const BoneTransform& transform)
        {
            return transform.refNode ? transform.refNode->local : transform.local;
        }

        void invalidateCapture()
        {
            s_captureValid.store(false, std::memory_order_release);
            s_capturedFlags.store(0, std::memory_order_release);
            s_capturedTransformCount.store(0, std::memory_order_release);
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
                if (!native_animation_authority_policy::isRequested(binding.flags, requestedFlags) ||
                    binding.sourceIndex < 0 || binding.sourceIndex >= source->numTransforms) {
                    continue;
                }

                const auto& local = authoritativeLocal(source->transforms[binding.sourceIndex]);
                if (!finiteTransform(local)) {
                    continue;
                }
                binding.capturedLocal = local;
                binding.captured = true;
                ++capturedCount;
                if (binding.destinationIndex >= 0) {
                    destinationMappedFlags |= binding.flags;
                }
            }

            if (capturedCount == 0 || (destinationMappedFlags & requestedFlags) != requestedFlags) {
                invalidateCapture();
                return;
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
                captureNativePose();
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                s_captureFault.store(true, std::memory_order_release);
                invalidateCapture();
            }
#else
            captureNativePose();
#endif
            if (s_originalPostUpdate) {
                s_originalPostUpdate(holder);
            }
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
                    binding.capturedLocal);

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
            // hFRIK may retain Weapon/WeaponLeft under a wand node. Preserve
            // Bethesda's logical flattened-tree local, but convert the target
            // world into that live scene parent's local before presentation.
            rebaseDifferentSceneParents(*tree, destination, requestedFlags);
            return true;
        }

        void decrementLocalReloadTestLease()
        {
            auto remaining = s_localReloadTestLeaseFrames.load(std::memory_order_acquire);
            while (remaining > 0 &&
                   !s_localReloadTestLeaseFrames.compare_exchange_weak(
                       remaining,
                       remaining - 1,
                       std::memory_order_acq_rel)) {
            }
        }
    }

    bool installPostUpdateHook()
    {
        if (s_hookInstalled.load(std::memory_order_acquire)) {
            return true;
        }
        if (s_hookInstallFailed.load(std::memory_order_acquire)) {
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
        ROCK_LOG_INFO(Init,
            "Native animation authority capture hook ready; scope=arms,hands,Weapon/WeaponLeft API=v1");
        return true;
    }

    void setRuntimeEnabled(const bool enabled)
    {
        s_runtimeEnabled.store(enabled && s_hookInstalled.load(std::memory_order_acquire), std::memory_order_release);
        if (!enabled) {
            invalidateCapture();
            s_frameCaptureReady = false;
        }
    }

    void requestLocalReloadTestLease()
    {
        if (!s_runtimeEnabled.load(std::memory_order_acquire) || !s_hookInstalled.load(std::memory_order_acquire)) {
            ROCK_LOG_WARN(Animation,
                "Native reload animation authority test lease ignored because the capture hook/skeleton is not ready");
            return;
        }
        s_localReloadTestLeaseFrames.store(kLocalReloadTestLeaseFrames, std::memory_order_release);
        ROCK_LOG_INFO(Animation,
            "Native reload animation authority local test lease armed for up to {} ROCK frames",
            kLocalReloadTestLeaseFrames);
    }

    void beginRockFrame()
    {
        provider::refreshNativeAnimationAuthorityLeasesV1();
        decrementLocalReloadTestLease();
        s_frameCaptureReady = false;
        s_frameCaptureFlags = 0;
        s_frameCaptureSequence = 0;

        const std::uint32_t currentFlags = effectiveRequestedFlags();
        if (currentFlags != s_lastLoggedEffectiveFlags) {
            ROCK_LOG_INFO(Animation,
                "Native animation authority {} flags=0x{:X}",
                currentFlags != 0 ? "enabled" : "disabled",
                currentFlags);
            s_lastLoggedEffectiveFlags = currentFlags;
        }

        if (currentFlags == 0 || !s_captureValid.load(std::memory_order_acquire) || !claimOrValidateThread()) {
            return;
        }

        const auto sequence = s_captureSequence.load(std::memory_order_acquire);
        const auto capturedFlags = s_capturedFlags.load(std::memory_order_acquire) & currentFlags;
        if (sequence == 0 || sequence == s_lastCompletedCaptureSequence || capturedFlags == 0) {
            return;
        }

        s_frameCaptureSequence = sequence;
        s_frameCaptureFlags = capturedFlags;
        s_frameCaptureReady = true;
    }

    bool applyCapturedPose()
    {
        if (!s_frameCaptureReady || s_frameCaptureFlags == 0 || !claimOrValidateThread()) {
            return false;
        }

        bool sourceApplied = false;
        bool destinationApplied = false;
#if defined(_MSC_VER)
        __try {
            sourceApplied = applyToTree(s_cache.sourceTree, false, s_frameCaptureFlags);
            destinationApplied = s_cache.destinationTree == s_cache.sourceTree ? sourceApplied :
                applyToTree(s_cache.destinationTree, true, s_frameCaptureFlags);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            s_captureFault.store(true, std::memory_order_release);
            invalidateCapture();
            s_frameCaptureReady = false;
            return false;
        }
#else
        sourceApplied = applyToTree(s_cache.sourceTree, false, s_frameCaptureFlags);
        destinationApplied = s_cache.destinationTree == s_cache.sourceTree ? sourceApplied :
            applyToTree(s_cache.destinationTree, true, s_frameCaptureFlags);
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
        if (s_frameCaptureReady) {
            s_lastCompletedCaptureSequence = s_frameCaptureSequence;
        }
        s_frameCaptureReady = false;
        s_frameCaptureFlags = 0;
        s_frameCaptureSequence = 0;
    }

    void resetTransientState()
    {
        s_runtimeEnabled.store(false, std::memory_order_release);
        s_localReloadTestLeaseFrames.store(0, std::memory_order_release);
        invalidateCapture();
        s_frameCaptureReady = false;
        s_frameCaptureFlags = 0;
        s_frameCaptureSequence = 0;
        s_lastCompletedCaptureSequence = s_captureSequence.load(std::memory_order_acquire);
        const DWORD ownerThread = s_ownerThreadId.load(std::memory_order_acquire);
        if (ownerThread == 0 || ownerThread == GetCurrentThreadId()) {
            s_cache = {};
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
