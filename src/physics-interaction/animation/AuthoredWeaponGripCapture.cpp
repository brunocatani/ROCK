#include "physics-interaction/animation/AuthoredWeaponGripCapture.h"

#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/NetImmerse/NiNode.h"

#include <Windows.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <intrin.h>
#include <string_view>

namespace rock::authored_weapon_grip_capture
{
    namespace
    {
        using BoneTree = f4vr::BSFlattenedBoneTree;
        using BoneTransform = BoneTree::BoneTransforms;
        using UpdateFirstPersonArmFn =
            void* (*)(const RE::PlayerCharacter*, RE::NiNode**, RE::NiNode**);
        using PostUpdateAnimationGraphManagerFn = void (*)(void* holder);

        constexpr int kMaxFlattenedTransforms = 768;
        constexpr std::uint16_t kAuthoredSupportFingerTransformMask = 0x7FFFu;
        constexpr std::array<std::string_view, 15>
            kAuthoredSupportFingerBoneNames{
                "LArm_Finger11", "LArm_Finger12", "LArm_Finger13",
                "LArm_Finger21", "LArm_Finger22", "LArm_Finger23",
                "LArm_Finger31", "LArm_Finger32", "LArm_Finger33",
                "LArm_Finger41", "LArm_Finger42", "LArm_Finger43",
                "LArm_Finger51", "LArm_Finger52", "LArm_Finger53",
            };

        struct PrimaryFiringGripBoneCache
        {
            BoneTree* tree{ nullptr };
            BoneTransform* transforms{ nullptr };
            int transformCount{ 0 };
            int primaryHandIndex{ -1 };
            int supportHandIndex{ -1 };
            int weaponIndex{ -1 };
            std::array<int, kAuthoredSupportFingerBoneNames.size()>
                supportFingerIndices{};
        };

        PrimaryFiringGripBoneCache s_primaryFiringGripBoneCache{};
        UpdateFirstPersonArmFn s_originalUpdateFirstPersonArm{ nullptr };
        PostUpdateAnimationGraphManagerFn s_originalPostUpdate{ nullptr };
        std::atomic<bool> s_hookInstalled{ false };
        std::atomic<bool> s_hookInstallFailed{ false };
        std::atomic<bool> s_primaryFiringGripCaptureEnabled{ false };
        std::atomic<bool> s_primaryFiringGripCaptureValid{ false };
        std::atomic<bool> s_authoredSupportGraphPoseValid{ false };
        std::atomic<bool> s_authoredSupportGripCaptureValid{ false };
        std::atomic<bool> s_threadMismatch{ false };
        std::atomic<bool> s_captureFault{ false };
        std::atomic<DWORD> s_ownerThreadId{ 0 };
        std::atomic<std::uint64_t> s_primaryFiringGripCaptureSequence{ 0 };
        std::atomic<std::uint64_t> s_authoredSupportGraphPoseSequence{ 0 };
        std::atomic<std::uint64_t> s_authoredSupportGripCaptureSequence{ 0 };
        std::atomic<std::uint64_t>
            s_authoredSupportGripSecondaryPassSequence{ 0 };
        std::atomic<AuthoredSupportGripCaptureFailureReason>
            s_authoredSupportGripCaptureFailureReason{
                AuthoredSupportGripCaptureFailureReason::SecondaryPassNotObserved
            };
        std::atomic<std::uint16_t>
            s_authoredSupportGripInvalidOrMissingFingerMask{ 0 };

        RE::NiTransform s_authoredPrimaryHandInWeapon{};
        std::atomic<RE::NiNode*> s_primaryFiringGripHandNode{ nullptr };
        std::atomic<RE::NiNode*> s_primaryFiringGripWeaponNode{ nullptr };
        RE::NiTransform s_authoredSupportHandInPrimaryHand{};
        std::array<RE::NiTransform, kAuthoredSupportFingerBoneNames.size()>
            s_authoredSupportGraphFingerLocals{};
        std::atomic<BoneTree*> s_authoredSupportGraphTree{ nullptr };
        std::atomic<BoneTransform*> s_authoredSupportGraphTransforms{ nullptr };
        std::atomic<int> s_authoredSupportGraphTransformCount{ 0 };
        std::atomic<RE::NiNode*>
            s_authoredSupportGraphPrimaryHandNode{ nullptr };
        RE::NiTransform s_authoredSupportHandInWeapon{};
        std::array<RE::NiTransform, kAuthoredSupportFingerBoneNames.size()>
            s_authoredSupportFingerLocals{};
        std::atomic<RE::NiNode*>
            s_authoredSupportGripPrimaryHandNode{ nullptr };
        std::atomic<RE::NiNode*> s_authoredSupportGripWeaponNode{ nullptr };
        std::uintptr_t s_nativePrimaryArmReturnAddress{ 0 };
        std::uintptr_t s_nativeSupportArmReturnAddress{ 0 };
        std::uint64_t s_primaryFiringGripGraphPoseSequence{ 0 };
        std::uint64_t s_lastConsumedAuthoredSupportGraphPoseSequence{ 0 };

        [[nodiscard]] bool nativeAnimationAuthorityActive()
        {
            return provider::currentNativeAnimationAuthorityFlagsV1() != 0;
        }

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
            s_primaryFiringGripBoneCache.supportFingerIndices.fill(-1);
            if (!validTree(&source)) {
                return false;
            }

            s_primaryFiringGripBoneCache.tree = &source;
            s_primaryFiringGripBoneCache.transforms = source.transforms;
            s_primaryFiringGripBoneCache.transformCount = source.numTransforms;
            for (int index = 0; index < source.numTransforms; ++index) {
                const auto name = transformName(source.transforms[index]);
                if (authored_weapon_grip_capture_policy::equalsIgnoreCase(name, "RArm_Hand")) {
                    s_primaryFiringGripBoneCache.primaryHandIndex = index;
                } else if (authored_weapon_grip_capture_policy::equalsIgnoreCase(name, "LArm_Hand")) {
                    s_primaryFiringGripBoneCache.supportHandIndex = index;
                } else if (authored_weapon_grip_capture_policy::equalsIgnoreCase(name, "Weapon")) {
                    s_primaryFiringGripBoneCache.weaponIndex = index;
                }
                for (std::size_t fingerIndex = 0;
                     fingerIndex < kAuthoredSupportFingerBoneNames.size();
                     ++fingerIndex) {
                    if (authored_weapon_grip_capture_policy::equalsIgnoreCase(
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


        [[nodiscard]] bool claimOrValidateThread()
        {
            const DWORD currentThread = GetCurrentThreadId();
            DWORD expected = 0;
            if (s_ownerThreadId.compare_exchange_strong(
                    expected,
                    currentThread,
                    std::memory_order_acq_rel)) {
                return true;
            }
            if (expected == currentThread) {
                return true;
            }
            s_threadMismatch.store(true, std::memory_order_release);
            return false;
        }

        void captureAuthoredSupportGraphPose()
        {
            if (!s_primaryFiringGripCaptureEnabled.load(std::memory_order_acquire) ||
                nativeAnimationAuthorityActive()) {
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
                authored_weapon_grip_capture_policy::resolveAuthoredSupportHandInPrimaryHand(
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
                authored_weapon_grip_capture_policy::resolveAuthoredSupportHandInWeapon(
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

        void onPostUpdateAnimationGraphManager(void* holder)
        {
#if defined(_MSC_VER)
            __try {
                // Animation addons capture first and may publish their ROCK V1
                // authority before the local authored-grip capture decides
                // whether it must yield for this exact graph sample.
                provider::dispatchAnimationPhaseCallbacksV1(
                    provider::RockProviderAnimationPhaseV1::NativeGraphOutput,
                    0.0f);
                captureAuthoredSupportGraphPose();
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                s_captureFault.store(true, std::memory_order_release);
                invalidatePrimaryFiringGripCapture();
                invalidateAuthoredSupportGraphPose();
                invalidateAuthoredSupportGripCapture();
                recordAuthoredSupportGripCaptureFailure(
                    AuthoredSupportGripCaptureFailureReason::CaptureFault);
            }
#else
            provider::dispatchAnimationPhaseCallbacksV1(
                provider::RockProviderAnimationPhaseV1::NativeGraphOutput,
                0.0f);
            captureAuthoredSupportGraphPose();
#endif
            if (s_originalPostUpdate) {
                s_originalPostUpdate(holder);
            }
        }

        __declspec(noinline) void* onUpdateFirstPersonArm(
            const RE::PlayerCharacter* player,
            RE::NiNode** weapon,
            RE::NiNode** offsetNode)
        {
            const auto returnAddress =
                reinterpret_cast<std::uintptr_t>(_ReturnAddress());
            const bool primaryPass =
                returnAddress == s_nativePrimaryArmReturnAddress;
            const bool supportPass =
                returnAddress == s_nativeSupportArmReturnAddress;
            const bool captureEnabled =
                (primaryPass || supportPass) &&
                s_primaryFiringGripCaptureEnabled.load(
                    std::memory_order_acquire);
            const bool authorityActive =
                captureEnabled && nativeAnimationAuthorityActive();

            if (authorityActive) {
                invalidatePrimaryFiringGripCapture();
                invalidateAuthoredSupportGraphPose();
                invalidateAuthoredSupportGripCapture();
            }

            void* result = s_originalUpdateFirstPersonArm ?
                s_originalUpdateFirstPersonArm(player, weapon, offsetNode) :
                nullptr;
            if (!captureEnabled || authorityActive) {
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

        [[nodiscard]] bool installCaptureHook()
        {
            if (s_hookInstalled.load(std::memory_order_acquire)) {
                return true;
            }
            if (s_hookInstallFailed.load(std::memory_order_acquire)) {
                return false;
            }

            // hFRIK preserves the native eight-byte prologue and NOPs the
            // downstream first-to-third-person bridge. Owning this single
            // validated entry detour lets ROCK retain its proven early grip
            // sample while addons consume the same graph-output API phase.
            constexpr std::array<std::uint8_t, 14> kExpectedPostFrikPrefix{
                0x48, 0x8B, 0xC4, 0x55, 0x48, 0x83, 0xEC, 0x60,
                0x90, 0x90, 0x90, 0x90, 0x90, 0x90,
            };
            void* originalPostUpdate = nullptr;
            const bool postUpdateInstalled = entry_trampoline_hook::install(
                "shared native graph-output coordinator",
                offsets::kFunc_PlayerPostUpdateAnimationGraphManager,
                kExpectedPostFrikPrefix.data(),
                kExpectedPostFrikPrefix.size(),
                reinterpret_cast<void*>(&onPostUpdateAnimationGraphManager),
                originalPostUpdate);
            if (!postUpdateInstalled || !originalPostUpdate) {
                s_hookInstallFailed.store(true, std::memory_order_release);
                return false;
            }
            s_originalPostUpdate =
                reinterpret_cast<PostUpdateAnimationGraphManagerFn>(
                    originalPostUpdate);

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
                s_hookInstallFailed.store(true, std::memory_order_release);
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
                s_hookInstallFailed.store(true, std::memory_order_release);
                return false;
            }

            s_originalUpdateFirstPersonArm = reinterpret_cast<UpdateFirstPersonArmFn>(original);
            s_nativePrimaryArmReturnAddress = primaryReturn.address();
            s_nativeSupportArmReturnAddress = supportReturn.address();
            s_hookInstalled.store(true, std::memory_order_release);
            ROCK_LOG_INFO(Init,
                "Native authored firing-grip capture ready; graphOutput=0x{:X} helper=0x{:X} BethesdaPrimaryReturn=0x{:X} BethesdaSupportReturn=0x{:X} source=pre-presentation-graph-locals",
                REL::Relocation<std::uintptr_t>{ REL::Offset(offsets::kFunc_PlayerPostUpdateAnimationGraphManager) }.address(),
                REL::Relocation<std::uintptr_t>{ REL::Offset(offsets::kFunc_UpdateFirstPersonArm) }.address(),
                s_nativePrimaryArmReturnAddress,
                s_nativeSupportArmReturnAddress);
            return true;
        }
    }

    bool installHook()
    {
        return installCaptureHook();
    }

    void setEnabled(const bool enabled)
    {
        const bool effectiveEnabled = enabled &&
            s_hookInstalled.load(std::memory_order_acquire);
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
        outCurrentAuthoredHandWorld = authored_weapon_grip_capture_policy::resolveAuthoredPrimaryHandWorld(
            liveWeaponWorld,
            outAuthoredPrimaryHandInWeapon,
            [](const RE::NiTransform& parent, const RE::NiTransform& child) {
                return transform_math::composeTransforms(parent, child);
            });
        outWeaponWorld = authored_weapon_grip_capture_policy::resolveAuthoredPrimaryWeaponWorld(
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

    void resetTransientState()
    {
        s_primaryFiringGripCaptureEnabled.store(
            false,
            std::memory_order_release);
        invalidatePrimaryFiringGripCapture();
        invalidateAuthoredSupportGraphPose();
        invalidateAuthoredSupportGripCapture();
        recordAuthoredSupportGripCaptureFailure(
            AuthoredSupportGripCaptureFailureReason::SecondaryPassNotObserved);
        s_lastConsumedAuthoredSupportGraphPoseSequence =
            s_authoredSupportGraphPoseSequence.load(std::memory_order_acquire);
        const DWORD ownerThread =
            s_ownerThreadId.load(std::memory_order_acquire);
        if (ownerThread == 0 || ownerThread == GetCurrentThreadId()) {
            s_primaryFiringGripBoneCache = {};
        }
    }

    bool isHookInstalled()
    {
        return s_hookInstalled.load(std::memory_order_acquire);
    }
}
