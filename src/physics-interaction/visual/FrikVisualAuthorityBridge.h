#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string_view>

#include "api/FRIKApi.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock::frik_visual_authority
{
    using Hand = frik::api::FRIKApi::Hand;
    using HandPoseKind = frik::api::FRIKApi::HandPoseKind;
    using HandPoseTagState = frik::api::FRIKApi::HandPoseTagState;
    using HandPoseData = frik::api::FRIKApi::HandPoseData;
    using FingerLocalTransformOverride = frik::api::FRIKApi::FingerLocalTransformOverride;
    using RecoilDelivery = frik::api::FRIKApi::RecoilDelivery;
    using RecoilHandMask = frik::api::FRIKApi::RecoilHandMask;
    using RecoilSample = frik::api::FRIKApi::RecoilSample;
    using RecoilResponse = frik::api::FRIKApi::RecoilResponse;
    using WeaponHandRecoilController = frik::api::FRIKApi::WeaponHandRecoilController;

    namespace detail
    {
        constexpr std::size_t kCachedHandPosePublicationCount = 8;
        constexpr std::size_t kCachedHandPoseTagCapacity = 64;

        struct CachedHandPosePublication
        {
            std::array<char, kCachedHandPoseTagCapacity> tag{};
            std::size_t tagLength = 0;
            Hand hand = Hand::Left;
            int priority = 0;
            HandPoseData pose{};
            bool valid = false;
        };

        struct CachedFingerLocalTransformPublication
        {
            std::array<char, kCachedHandPoseTagCapacity> tag{};
            std::size_t tagLength = 0;
            Hand hand = Hand::Left;
            int priority = 0;
            FingerLocalTransformOverride transforms{};
            bool valid = false;
        };

        inline std::array<CachedHandPosePublication, kCachedHandPosePublicationCount> g_cachedHandPosePublications{};
        inline std::array<CachedFingerLocalTransformPublication, kCachedHandPosePublicationCount> g_cachedFingerLocalTransformPublications{};
        inline std::size_t g_nextCachedHandPosePublication = 0;
        inline std::size_t g_nextCachedFingerLocalTransformPublication = 0;

        struct PresentedHandNodeCache
        {
            // Non-owning game scene pointers. FRIK lifecycle messages reset
            // this cache before the first-person skeleton can be destroyed.
            RE::NiNode* skeleton = nullptr;
            RE::NiNode* rightHand = nullptr;
            RE::NiNode* leftHand = nullptr;
        };

        inline PresentedHandNodeCache g_presentedHandNodeCache{};

        using MirrorFingerLocalTransformsFn = bool(FRIK_CALL*)(Hand, const FingerLocalTransformOverride*, FingerLocalTransformOverride*);

        [[nodiscard]] inline MirrorFingerLocalTransformsFn mirrorFingerLocalTransformsExport()
        {
            static MirrorFingerLocalTransformsFn fn = nullptr;
            static bool attemptedWithLoadedFrik = false;
            if (!fn) {
                const auto frikDll = GetModuleHandleA("FRIK.dll");
                if (!frikDll) {
                    return nullptr;
                }
                if (attemptedWithLoadedFrik) {
                    return nullptr;
                }
                attemptedWithLoadedFrik = true;
                fn = reinterpret_cast<MirrorFingerLocalTransformsFn>(GetProcAddress(frikDll, "FRIKAPI_MirrorFingerLocalTransforms"));
            }
            return fn;
        }

        [[nodiscard]] inline bool makeCacheableTagView(const char* tag, std::string_view& outTag)
        {
            if (!tag) {
                return false;
            }

            outTag = std::string_view(tag);
            return !outTag.empty() && outTag.size() < kCachedHandPoseTagCapacity;
        }

        [[nodiscard]] inline std::string_view cachedTagView(const CachedHandPosePublication& entry)
        {
            return std::string_view(entry.tag.data(), entry.tagLength);
        }

        [[nodiscard]] inline std::string_view cachedTagView(const CachedFingerLocalTransformPublication& entry)
        {
            return std::string_view(entry.tag.data(), entry.tagLength);
        }

        [[nodiscard]] inline bool sameFingerPoseData(
            const frik::api::FRIKApi::FingerPoseData& lhs,
            const frik::api::FRIKApi::FingerPoseData& rhs)
        {
            return lhs.prox == rhs.prox &&
                   lhs.mid == rhs.mid &&
                   lhs.dist == rhs.dist &&
                   lhs.splay == rhs.splay;
        }

        [[nodiscard]] inline bool sameHandPoseData(const HandPoseData& lhs, const HandPoseData& rhs)
        {
            return sameFingerPoseData(lhs.thumb, rhs.thumb) &&
                   sameFingerPoseData(lhs.index, rhs.index) &&
                   sameFingerPoseData(lhs.middle, rhs.middle) &&
                   sameFingerPoseData(lhs.ring, rhs.ring) &&
                   sameFingerPoseData(lhs.pinky, rhs.pinky) &&
                   lhs.palmPitch == rhs.palmPitch &&
                   lhs.palmYaw == rhs.palmYaw;
        }

        [[nodiscard]] inline bool sameNiTransform(const RE::NiTransform& lhs, const RE::NiTransform& rhs)
        {
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (lhs.rotate.entry[row][column] != rhs.rotate.entry[row][column]) {
                        return false;
                    }
                }
            }
            return lhs.translate.x == rhs.translate.x && lhs.translate.y == rhs.translate.y && lhs.translate.z == rhs.translate.z && lhs.scale == rhs.scale;
        }

        [[nodiscard]] inline bool sameFingerLocalTransforms(const FingerLocalTransformOverride& lhs, const FingerLocalTransformOverride& rhs)
        {
            if (lhs.enabledMask != rhs.enabledMask) {
                return false;
            }
            for (std::size_t index = 0; index < std::size(lhs.localTransforms); ++index) {
                const std::uint16_t bit = static_cast<std::uint16_t>(1U << index);
                if ((lhs.enabledMask & bit) != 0 && !sameNiTransform(lhs.localTransforms[index], rhs.localTransforms[index])) {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] inline CachedHandPosePublication* findCachedHandPosePublication(std::string_view tag, Hand hand)
        {
            for (auto& entry : g_cachedHandPosePublications) {
                if (entry.valid && entry.hand == hand && cachedTagView(entry) == tag) {
                    return &entry;
                }
            }
            return nullptr;
        }

        [[nodiscard]] inline CachedFingerLocalTransformPublication* findCachedFingerLocalTransformPublication(std::string_view tag, Hand hand)
        {
            for (auto& entry : g_cachedFingerLocalTransformPublications) {
                if (entry.valid && entry.hand == hand && cachedTagView(entry) == tag) {
                    return &entry;
                }
            }
            return nullptr;
        }

        inline void invalidateCachedHandPosePublication(const char* tag, Hand hand)
        {
            std::string_view tagView;
            if (!makeCacheableTagView(tag, tagView)) {
                return;
            }

            if (auto* entry = findCachedHandPosePublication(tagView, hand)) {
                entry->valid = false;
            }
        }

        inline void invalidateCachedFingerLocalTransformPublication(const char* tag, Hand hand)
        {
            std::string_view tagView;
            if (!makeCacheableTagView(tag, tagView)) {
                return;
            }

            if (auto* entry = findCachedFingerLocalTransformPublication(tagView, hand)) {
                entry->valid = false;
            }
        }

        inline void rememberCachedHandPosePublication(std::string_view tag, Hand hand, const HandPoseData& handPose, int priority)
        {
            auto* entry = findCachedHandPosePublication(tag, hand);
            if (!entry) {
                for (auto& candidate : g_cachedHandPosePublications) {
                    if (!candidate.valid) {
                        entry = &candidate;
                        break;
                    }
                }
            }
            if (!entry) {
                entry = &g_cachedHandPosePublications[g_nextCachedHandPosePublication % g_cachedHandPosePublications.size()];
                ++g_nextCachedHandPosePublication;
            }

            entry->tag.fill('\0');
            std::copy(tag.begin(), tag.end(), entry->tag.begin());
            entry->tagLength = tag.size();
            entry->hand = hand;
            entry->priority = priority;
            entry->pose = handPose;
            entry->valid = true;
        }

        inline void rememberCachedFingerLocalTransformPublication(std::string_view tag,
            Hand hand, const FingerLocalTransformOverride& transforms,
            int priority)
        {
            auto* entry = findCachedFingerLocalTransformPublication(tag, hand);
            if (!entry) {
                for (auto& candidate : g_cachedFingerLocalTransformPublications) {
                    if (!candidate.valid) {
                        entry = &candidate;
                        break;
                    }
                }
            }
            if (!entry) {
                entry = &g_cachedFingerLocalTransformPublications[g_nextCachedFingerLocalTransformPublication % g_cachedFingerLocalTransformPublications.size()];
                ++g_nextCachedFingerLocalTransformPublication;
            }

            entry->tag.fill('\0');
            std::copy(tag.begin(), tag.end(), entry->tag.begin());
            entry->tagLength = tag.size();
            entry->hand = hand;
            entry->priority = priority;
            entry->transforms = transforms;
            entry->valid = true;
        }

        [[nodiscard]] inline bool cachedHandPosePublicationStillActive(
            const frik::api::FRIKApi* frikApi,
            const char* tag,
            Hand hand)
        {
            return frikApi &&
                   frikApi->getHandPoseSetTagState &&
                   frikApi->getHandPoseSetTagState(tag, hand) == HandPoseTagState::Active;
        }

        [[nodiscard]] inline bool shouldSkipCachedHandPosePublication(
            const frik::api::FRIKApi* frikApi,
            const char* tag,
            Hand hand,
            const HandPoseData& handPose,
            int priority,
            std::string_view& outTagView)
        {
            if (!makeCacheableTagView(tag, outTagView)) {
                return false;
            }

            const auto* entry = findCachedHandPosePublication(outTagView, hand);
            return entry &&
                   entry->priority == priority &&
                   sameHandPoseData(entry->pose, handPose) && cachedHandPosePublicationStillActive(frikApi, tag, hand);
        }

        [[nodiscard]] inline bool shouldSkipCachedFingerLocalTransformPublication(const frik::api::FRIKApi* frikApi, const char* tag, Hand hand,
            const FingerLocalTransformOverride& transforms, int priority, std::string_view& outTagView)
        {
            if (!makeCacheableTagView(tag, outTagView)) {
                return false;
            }

            const auto* entry = findCachedFingerLocalTransformPublication(outTagView, hand);
            return entry && entry->priority == priority && sameFingerLocalTransforms(entry->transforms, transforms) &&
                   cachedHandPosePublicationStillActive(frikApi, tag, hand);
        }
    }

    [[nodiscard]] inline float finiteOrZero(float value)
    {
        return std::isfinite(value) ? value : 0.0f;
    }

    [[nodiscard]] inline HandPoseData makeHandPoseDataFromJointValues(
        const float values[15],
        const std::array<float, 5>& splayRadians,
        float palmPitchDegrees = 0.0f,
        float palmYawDegrees = 0.0f)
    {
        return HandPoseData{
            .thumb = { values[0], values[1], values[2], finiteOrZero(splayRadians[0]) },
            .index = { values[3], values[4], values[5], finiteOrZero(splayRadians[1]) },
            .middle = { values[6], values[7], values[8], finiteOrZero(splayRadians[2]) },
            .ring = { values[9], values[10], values[11], finiteOrZero(splayRadians[3]) },
            .pinky = { values[12], values[13], values[14], finiteOrZero(splayRadians[4]) },
            .palmPitch = finiteOrZero(palmPitchDegrees),
            .palmYaw = finiteOrZero(palmYawDegrees),
        };
    }

    [[nodiscard]] inline HandPoseData makeHandPoseDataFromJointValues(const float values[15])
    {
        return makeHandPoseDataFromJointValues(values, std::array<float, 5>{});
    }

    [[nodiscard]] inline HandPoseData makeHandPoseDataFromJointValues(
        const std::array<float, 15>& values,
        const std::array<float, 5>& splayRadians,
        float palmPitchDegrees = 0.0f,
        float palmYawDegrees = 0.0f)
    {
        return makeHandPoseDataFromJointValues(values.data(), splayRadians, palmPitchDegrees, palmYawDegrees);
    }

    [[nodiscard]] inline HandPoseData makeHandPoseDataFromJointValues(const std::array<float, 15>& values)
    {
        return makeHandPoseDataFromJointValues(values.data());
    }

    [[nodiscard]] inline HandPoseData makeUniformHandPoseData(
        float thumb,
        float index,
        float middle,
        float ring,
        float pinky,
        const std::array<float, 5>& splayRadians = {},
        float palmPitchDegrees = 0.0f,
        float palmYawDegrees = 0.0f)
    {
        return HandPoseData{
            .thumb = { thumb, thumb, thumb, finiteOrZero(splayRadians[0]) },
            .index = { index, index, index, finiteOrZero(splayRadians[1]) },
            .middle = { middle, middle, middle, finiteOrZero(splayRadians[2]) },
            .ring = { ring, ring, ring, finiteOrZero(splayRadians[3]) },
            .pinky = { pinky, pinky, pinky, finiteOrZero(splayRadians[4]) },
            .palmPitch = finiteOrZero(palmPitchDegrees),
            .palmYaw = finiteOrZero(palmYawDegrees),
        };
    }

    [[nodiscard]] inline const frik::api::FRIKApi* api()
    {
        return frik::api::FRIKApi::inst;
    }

    [[nodiscard]] inline Hand handFromBool(bool isLeft)
    {
        return isLeft ? Hand::Left : Hand::Right;
    }

    [[nodiscard]] inline bool isAvailable()
    {
        return api() != nullptr;
    }

    [[nodiscard]] inline bool isSkeletonReadyHint()
    {
        auto* frikApi = api();
        return frikApi && frikApi->isSkeletonReady();
    }

    [[nodiscard]] inline bool isCompatibilityConfigBlocking()
    {
        auto* frikApi = api();
        return frikApi && (frikApi->isConfigOpen() || frikApi->isWristPipboyOpen());
    }

    [[nodiscard]] inline bool clearHandPose(const char* tag, Hand hand)
    {
        detail::invalidateCachedHandPosePublication(tag, hand);
        detail::invalidateCachedFingerLocalTransformPublication(tag, hand);
        auto* frikApi = api();
        return frikApi && frikApi->clearHandPose && frikApi->clearHandPose(tag, hand);
    }

    [[nodiscard]] inline bool setHandPoseCustomWithPriority(const char* tag, Hand hand, const HandPoseData& handPose, int priority)
    {
        auto* frikApi = api();
        if (!frikApi || !frikApi->setHandPoseCustomWithPriority) {
            detail::invalidateCachedHandPosePublication(tag, hand);
            return false;
        }

        std::string_view tagView;
        if (detail::shouldSkipCachedHandPosePublication(frikApi, tag, hand, handPose, priority, tagView)) {
            return true;
        }

        const bool published = frikApi->setHandPoseCustomWithPriority(tag, hand, handPose, priority);
        if (published) {
            // Updating the scalar pose replaces hFRIK's tagged entry and
            // clears any local-transform payload previously attached to it.
            detail::invalidateCachedFingerLocalTransformPublication(tag, hand);
            if (!tagView.empty()) {
                detail::rememberCachedHandPosePublication(tagView, hand, handPose, priority);
            }
        } else if (!published) {
            detail::invalidateCachedHandPosePublication(tag, hand);
            detail::invalidateCachedFingerLocalTransformPublication(tag, hand);
        }
        return published;
    }

    [[nodiscard]] inline bool setHandPoseWithPriority(const char* tag, Hand hand, HandPoseKind handPose, int priority)
    {
        detail::invalidateCachedHandPosePublication(tag, hand);
        detail::invalidateCachedFingerLocalTransformPublication(tag, hand);
        auto* frikApi = api();
        return frikApi && frikApi->setHandPoseWithPriority && frikApi->setHandPoseWithPriority(tag, hand, handPose, priority);
    }

    [[nodiscard]] inline bool applyExternalHandWorldTransform(const char* tag, Hand hand, const RE::NiTransform& worldTarget, int priority)
    {
        auto* frikApi = api();
        return frikApi && frikApi->applyExternalHandWorldTransform && frikApi->applyExternalHandWorldTransform(tag, hand, worldTarget, priority);
    }

    [[nodiscard]] inline bool clearExternalHandWorldTransform(const char* tag, Hand hand)
    {
        auto* frikApi = api();
        return frikApi && frikApi->clearExternalHandWorldTransform && frikApi->clearExternalHandWorldTransform(tag, hand);
    }

    [[nodiscard]] inline bool setHandPoseCustomLocalTransformsWithPriority(
        const char* tag,
        Hand hand,
        const FingerLocalTransformOverride* overrideData,
        int priority)
    {
        auto* frikApi = api();
        if (!frikApi || !frikApi->setHandPoseCustomLocalTransformsWithPriority || !overrideData) {
            detail::invalidateCachedFingerLocalTransformPublication(tag, hand);
            return false;
        }

        std::string_view tagView;
        if (detail::shouldSkipCachedFingerLocalTransformPublication(frikApi, tag, hand, *overrideData, priority, tagView)) {
            return true;
        }

        const bool published = frikApi->setHandPoseCustomLocalTransformsWithPriority(tag, hand, overrideData, priority);
        if (published && !tagView.empty()) {
            detail::rememberCachedFingerLocalTransformPublication(tagView, hand, *overrideData, priority);
        } else if (!published) {
            detail::invalidateCachedFingerLocalTransformPublication(tag, hand);
        }
        return published;
    }

    [[nodiscard]] inline bool getHandPoseLocalTransformsForPose(
        Hand hand,
        const HandPoseData& handPose,
        FingerLocalTransformOverride* outTransforms)
    {
        auto* frikApi = api();
        return frikApi &&
            frikApi->getHandPoseLocalTransformsForPose &&
            frikApi->getHandPoseLocalTransformsForPose(hand, handPose, outTransforms);
    }

    [[nodiscard]] inline bool blockOffHandWeaponGripping(const char* tag, bool block)
    {
        auto* frikApi = api();
        return frikApi && frikApi->blockOffHandWeaponGripping && frikApi->blockOffHandWeaponGripping(tag, block);
    }

    [[nodiscard]] inline bool blockPrimaryHandWeaponPose(const char* tag, bool block)
    {
        auto* frikApi = api();
        return frikApi &&
            frikApi->blockPrimaryHandWeaponPose &&
            frikApi->blockPrimaryHandWeaponPose(tag, block);
    }

    [[nodiscard]] inline bool canBlockPrimaryHandWeaponPose()
    {
        auto* frikApi = api();
        return frikApi && frikApi->blockPrimaryHandWeaponPose != nullptr;
    }

    [[nodiscard]] inline bool mirrorFingerLocalTransforms(Hand sourceHand, const FingerLocalTransformOverride& sourceTransforms, FingerLocalTransformOverride& outTargetTransforms)
    {
        if (const auto fn = detail::mirrorFingerLocalTransformsExport()) {
            return fn(sourceHand, &sourceTransforms, &outTargetTransforms);
        }
        return false;
    }

    [[nodiscard]] inline bool mirrorPrimaryWeaponFingerLocalTransforms(const FingerLocalTransformOverride& rightTransforms, FingerLocalTransformOverride& outLeftTransforms)
    {
        return mirrorFingerLocalTransforms(Hand::Right, rightTransforms, outLeftTransforms);
    }

    [[nodiscard]] inline bool canMirrorPrimaryWeaponFingerLocalTransforms()
    {
        return detail::mirrorFingerLocalTransformsExport() != nullptr;
    }

    [[nodiscard]] inline bool canMirrorFingerLocalTransforms() { return detail::mirrorFingerLocalTransformsExport() != nullptr; }

    [[nodiscard]] inline bool blockPrimaryWeaponNodeOwnership(const char* tag, bool block)
    {
        auto* frikApi = api();
        return frikApi && frikApi->blockPrimaryWeaponNodeOwnership && frikApi->blockPrimaryWeaponNodeOwnership(tag, block);
    }

    [[nodiscard]] inline bool canBlockPrimaryWeaponNodeOwnership()
    {
        auto* frikApi = api();
        return frikApi && frikApi->blockPrimaryWeaponNodeOwnership != nullptr;
    }

    [[nodiscard]] inline bool registerWeaponHandRecoilController(
        const char* tag,
        WeaponHandRecoilController controller,
        void* userData,
        int priority)
    {
        auto* frikApi = api();
        return frikApi &&
            frikApi->registerWeaponHandRecoilController &&
            frikApi->registerWeaponHandRecoilController(tag, controller, userData, priority);
    }

    [[nodiscard]] inline bool unregisterWeaponHandRecoilController(const char* tag)
    {
        auto* frikApi = api();
        return frikApi &&
            frikApi->unregisterWeaponHandRecoilController &&
            frikApi->unregisterWeaponHandRecoilController(tag);
    }

    inline void resetPresentedHandNodeCache()
    {
        detail::g_presentedHandNodeCache = {};
    }

    [[nodiscard]] inline RE::NiTransform getHandWorldTransform(Hand hand)
    {
        RE::NiTransform identity{};
        identity.MakeIdentity();
        if (!isSkeletonReadyHint()) {
            resetPresentedHandNodeCache();
            return identity;
        }

        bool isLeft = false;
        switch (hand) {
        case Hand::Left:
            isLeft = true;
            break;
        case Hand::Right:
            break;
        case Hand::Primary:
        case Hand::Offhand: {
            const auto* leftHandedMode = f4vr::getIniSetting("bLeftHandedMode:VR");
            if (!leftHandedMode) {
                return identity;
            }
            const bool primaryIsLeft = leftHandedMode->GetBinary();
            isLeft = hand == Hand::Primary ? primaryIsLeft : !primaryIsLeft;
            break;
        }
        default:
            return identity;
        }

        auto* const skeleton = f4vr::getFirstPersonSkeleton();
        auto& cache = detail::g_presentedHandNodeCache;
        if (!skeleton) {
            resetPresentedHandNodeCache();
            return identity;
        }
        if (cache.skeleton != skeleton) {
            cache = {};
            cache.skeleton = skeleton;
        }
        if (!cache.rightHand) {
            cache.rightHand = f4vr::findNode(skeleton, "RArm_Hand");
        }
        if (!cache.leftHand) {
            cache.leftHand = f4vr::findNode(skeleton, "LArm_Hand");
        }

        const auto* const handNode = isLeft ? cache.leftHand : cache.rightHand;
        return handNode ? handNode->world : identity;
    }
}
