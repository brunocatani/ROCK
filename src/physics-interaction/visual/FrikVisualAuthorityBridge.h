#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>

#include "api/FRIKApiV2.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock::frik_visual_authority
{
    using Hand = frik::api::FRIKApiV2::Hand;
    using HandPoseKind = frik::api::FRIKApiV2::HandPoseKind;
    using HandPoseTagState = frik::api::FRIKApiV2::HandPoseTagState;
    using HandPoseData = frik::api::FRIKApiV2::HandPoseData;
    using FingerLocalTransformOverride = frik::api::FRIKApiV2::FingerLocalTransformOverride;
    using RecoilDelivery = frik::api::FRIKApiV2::RecoilDelivery;
    using RecoilHandMask = frik::api::FRIKApiV2::RecoilHandMask;
    using RecoilSample = frik::api::FRIKApiV2::RecoilSample;
    using RecoilResponse = frik::api::FRIKApiV2::RecoilResponse;
    using WeaponHandRecoilController = frik::api::FRIKApiV2::WeaponHandRecoilController;
    using FramePhase = frik::api::FRIKApiV2::FramePhase;
    using FrameCallback = frik::api::FRIKApiV2::FrameCallback;
    using TrackedHandKind = frik::api::FRIKApiV2::TrackedHandKind;
    using ArmChainTransforms = frik::api::FRIKApiV2::ArmChainTransforms;
    using HandSolveState = frik::api::FRIKApiV2::HandSolveState;
    using ScopeCapability = frik::api::FRIKApiV2::ScopeCapability;
    using SkeletonLifecycleData = frik::api::FRIKApiV2::SkeletonLifecycleData;

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
            const frik::api::FRIKApiV2::FingerPoseData& lhs,
            const frik::api::FRIKApiV2::FingerPoseData& rhs)
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

        [[nodiscard]] inline bool isFiniteNiTransform(
            const RE::NiTransform& transform)
        {
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(
                            transform.rotate.entry[row][column])) {
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
            const frik::api::FRIKApiV2* frikApi,
            const char* tag,
            Hand hand)
        {
            return frikApi &&
                   frikApi->getHandPoseSetTagState &&
                   frikApi->getHandPoseSetTagState(tag, hand) == HandPoseTagState::Active;
        }

        [[nodiscard]] inline bool shouldSkipCachedHandPosePublication(
            const frik::api::FRIKApiV2* frikApi,
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

        [[nodiscard]] inline bool shouldSkipCachedFingerLocalTransformPublication(const frik::api::FRIKApiV2* frikApi, const char* tag, Hand hand,
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

    [[nodiscard]] inline const frik::api::FRIKApiV2* api()
    {
        return frik::api::FRIKApiV2::inst;
    }

    [[nodiscard]] inline Hand handFromBool(bool isLeft)
    {
        return isLeft ? Hand::Left : Hand::Right;
    }

    /*
     * Physical side of an API hand. Primary/Offhand follow the game's
     * left-handed mode setting; false when that setting is unavailable.
     */
    [[nodiscard]] inline bool tryResolveHandIsLeft(Hand hand, bool& outIsLeft)
    {
        switch (hand) {
        case Hand::Left:
            outIsLeft = true;
            return true;
        case Hand::Right:
            outIsLeft = false;
            return true;
        case Hand::Primary:
        case Hand::Offhand: {
            const auto* leftHandedMode = f4vr::getIniSetting("bLeftHandedMode:VR");
            if (!leftHandedMode) {
                return false;
            }
            const bool primaryIsLeft = leftHandedMode->GetBinary();
            outIsLeft = hand == Hand::Primary ? primaryIsLeft : !primaryIsLeft;
            return true;
        }
        default:
            return false;
        }
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

    [[nodiscard]] inline bool isHandPoseTagActive(const char* tag, Hand hand)
    {
        auto* frikApi = api();
        return frikApi && tag && frikApi->getHandPoseSetTagState &&
               frikApi->getHandPoseSetTagState(tag, hand) ==
                   HandPoseTagState::Active;
    }

    [[nodiscard]] inline bool setHandPoseCustom(const char* tag, Hand hand, const HandPoseData& handPose, int priority)
    {
        auto* frikApi = api();
        if (!frikApi || !frikApi->setHandPoseCustom) {
            detail::invalidateCachedHandPosePublication(tag, hand);
            return false;
        }

        std::string_view tagView;
        if (detail::shouldSkipCachedHandPosePublication(frikApi, tag, hand, handPose, priority, tagView)) {
            return true;
        }

        const bool published = frikApi->setHandPoseCustom(tag, hand, handPose, priority);
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

    [[nodiscard]] inline bool setHandPose(const char* tag, Hand hand, HandPoseKind handPose, int priority)
    {
        detail::invalidateCachedHandPosePublication(tag, hand);
        detail::invalidateCachedFingerLocalTransformPublication(tag, hand);
        auto* frikApi = api();
        return frikApi && frikApi->setHandPose && frikApi->setHandPose(tag, hand, handPose, priority);
    }

    /*
     * Hand world claims go through the hand world authority service: they are
     * published inside FRIK's AfterArmSolve phase and FRIK re-solves the
     * claimed hand in the same frame. False means the claim is not held
     * anywhere (gate closed, FRIK rejected it, or FRIK fell back to the
     * tracked hand for it); the caller runs its failure reaction.
     */
    [[nodiscard]] inline bool publishHandWorld(const char* tag, Hand hand, const RE::NiTransform& worldTarget, int priority)
    {
        bool isLeft = false;
        if (!tryResolveHandIsLeft(hand, isLeft)) {
            return false;
        }
        return frik_hand_world_authority::publish(tag, isLeft, worldTarget, priority);
    }

    [[nodiscard]] inline bool clearHandWorld(const char* tag, Hand hand)
    {
        bool isLeft = false;
        if (!tryResolveHandIsLeft(hand, isLeft)) {
            return false;
        }
        return frik_hand_world_authority::clear(tag, isLeft);
    }

    /*
     * The target FRIK currently solves this hand to, from ROCK's own claim
     * registry (no scene read), optionally filtering the tag and priority.
     */
    [[nodiscard]] inline bool tryGetPublishedHandWorld(Hand hand, RE::NiTransform& outWorld,
        const char* excludedTag = nullptr, int maximumPriority = (std::numeric_limits<int>::max)())
    {
        bool isLeft = false;
        if (!tryResolveHandIsLeft(hand, isLeft)) {
            outWorld = {};
            return false;
        }
        return frik_hand_world_authority::tryGetPublishedHandWorld(isLeft, outWorld, excludedTag, maximumPriority);
    }

    [[nodiscard]] inline bool setHandPoseCustomLocalTransforms(
        const char* tag,
        Hand hand,
        const FingerLocalTransformOverride* overrideData,
        int priority)
    {
        auto* frikApi = api();
        if (!frikApi || !frikApi->setHandPoseCustomLocalTransforms || !overrideData) {
            detail::invalidateCachedFingerLocalTransformPublication(tag, hand);
            return false;
        }

        std::string_view tagView;
        if (detail::shouldSkipCachedFingerLocalTransformPublication(frikApi, tag, hand, *overrideData, priority, tagView)) {
            return true;
        }

        const bool published = frikApi->setHandPoseCustomLocalTransforms(tag, hand, overrideData, priority);
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

    /*
     * Since FRIK API v2.3 this is a pure write blocker: while blocked FRIK
     * writes nothing to the primary weapon node (no offsets, no per-frame
     * re-glue) and no longer changes which hand the node is parented under.
     */
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

    [[nodiscard]] inline bool canBlockSecondaryWeaponNodeOwnership()
    {
        auto* frikApi = api();
        return frikApi && frik::api::FRIKApiV2::supportsVersion(5) && frikApi->blockSecondaryWeaponNodeOwnership;
    }

    [[nodiscard]] inline bool blockSecondaryWeaponNodeOwnership(const char* tag, bool block)
    {
        return canBlockSecondaryWeaponNodeOwnership() && api()->blockSecondaryWeaponNodeOwnership(tag, block);
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

    // ---- Weapon node ownership (v2.3) ----

    /*
     * Parent the primary weapon node under a hand (left carry). FRIK does the
     * reparent plus its bookkeeping (first-person arm source, off-side hand
     * pose copy, recoil hand) and restores the game's setting when the tag
     * clears or the skeleton rebuilds.
     */
    [[nodiscard]] inline bool setWeaponNodeParentHand(const char* tag, Hand hand)
    {
        auto* frikApi = api();
        return frikApi && frikApi->setWeaponNodeParentHand && frikApi->setWeaponNodeParentHand(tag, hand);
    }

    [[nodiscard]] inline bool clearWeaponNodeParentHand(const char* tag)
    {
        auto* frikApi = api();
        return frikApi && frikApi->clearWeaponNodeParentHand && frikApi->clearWeaponNodeParentHand(tag);
    }

    [[nodiscard]] inline bool canSetWeaponNodeParentHand()
    {
        auto* frikApi = api();
        return frikApi && frikApi->setWeaponNodeParentHand != nullptr && frikApi->clearWeaponNodeParentHand != nullptr;
    }

    /*
     * Report or drop a two-handed grip on the current weapon so FRIK's
     * Pip-Boy guards and isOffHandGrippingWeapon see it. FRIK drops the grip
     * on a drawn weapon change and on skeleton release.
     */
    [[nodiscard]] inline bool setOffHandGripping(const char* tag, bool active, Hand supportHand, const RE::NiTransform* supportWorld)
    {
        auto* frikApi = api();
        return frikApi && frikApi->setOffHandGripping && frikApi->setOffHandGripping(tag, active, supportHand, supportWorld);
    }

    // ---- Frame phases (v2.3) ----

    [[nodiscard]] inline bool registerFrameCallback(const char* tag, FramePhase phase, FrameCallback callback, void* userData, int priority)
    {
        auto* frikApi = api();
        return frikApi && frikApi->registerFrameCallback &&
            frikApi->registerFrameCallback(tag, static_cast<std::uint32_t>(phase), callback, userData, priority);
    }

    [[nodiscard]] inline bool unregisterFrameCallback(const char* tag)
    {
        auto* frikApi = api();
        return frikApi && frikApi->unregisterFrameCallback && frikApi->unregisterFrameCallback(tag);
    }

    // ---- Body reads (v2.3) ----

    /*
     * A tracked input of a hand as FRIK uses it this frame. Current from
     * BeforeArmSolve on; before that phase it holds the previous frame.
     */
    [[nodiscard]] inline bool tryGetTrackedHandTransform(Hand hand, TrackedHandKind kind, RE::NiTransform& outWorld,
        const char** failure = nullptr)
    {
        outWorld = {};
        if (failure) *failure = "none";
        auto* frikApi = api();
        if (!frikApi || !frikApi->getTrackedHandTransform) {
            if (failure) *failure = frikApi ? "tracked-input-api-unavailable" : "frik-api-unavailable";
            return false;
        }
        if (!frikApi->getTrackedHandTransform(hand, kind, &outWorld)) {
            if (failure) *failure = "tracked-input-rejected";
            outWorld = {};
            return false;
        }
        const bool valid = detail::isFiniteNiTransform(outWorld);
        if (!valid && failure) *failure = "tracked-input-invalid-transform";
        return valid;
    }

    /*
     * The first-person hand FRIK solves the body arm to when no claim is
     * published: the controller hand, dampening and native kick included.
     */
    [[nodiscard]] inline bool tryGetHandWorldTransform(Hand hand, RE::NiTransform& outWorld)
    {
        return tryGetTrackedHandTransform(hand, TrackedHandKind::FirstPersonHand, outWorld);
    }

    // A bone of the flattened first-person tree: final after AfterWorldFinal, the previous frame before that.
    [[nodiscard]] inline bool tryGetBoneWorldTransform(const char* boneName, RE::NiTransform& outWorld)
    {
        outWorld = {};
        auto* frikApi = api();
        if (!frikApi || !frikApi->getBoneWorldTransform || !boneName || !frikApi->getBoneWorldTransform(boneName, &outWorld)) {
            outWorld = {};
            return false;
        }
        return detail::isFiniteNiTransform(outWorld);
    }

    /*
     * The last completed visual hand, claims and palm blend included. Read
     * before FRIK's world final it is the previous frame's presentation;
     * FirstPersonHand can instead hold the native re-glue at that point.
     */
    [[nodiscard]] inline bool tryGetPresentedHandWorldTransform(const bool isLeft, RE::NiTransform& outWorld)
    {
        return tryGetBoneWorldTransform(isLeft ? "LArm_Hand" : "RArm_Hand", outWorld);
    }

    // Live arm chain nodes for a hand: valid after AfterArmSolve, final after AfterWorldFinal.
    [[nodiscard]] inline bool tryGetArmChain(Hand hand, ArmChainTransforms& outChain)
    {
        outChain = {};
        outChain.structSize = sizeof(ArmChainTransforms);
        auto* frikApi = api();
        return frikApi && frikApi->getArmChain && frikApi->getArmChain(hand, &outChain);
    }

    // How a hand was solved this frame; latched once the frame's world transforms are final.
    [[nodiscard]] inline HandSolveState getHandSolveResult(Hand hand, RE::NiTransform& outWrist)
    {
        outWrist = {};
        auto* frikApi = api();
        if (!frikApi || !frikApi->getHandSolveResult) {
            return HandSolveState::SkeletonNotReady;
        }
        return frikApi->getHandSolveResult(hand, &outWrist);
    }

    // ---- Lifecycle and scope (v2.2) ----

    [[nodiscard]] inline std::uint32_t getSkeletonGeneration()
    {
        auto* frikApi = api();
        return frikApi && frikApi->getSkeletonGeneration ? frikApi->getSkeletonGeneration() : 0u;
    }

    // FRIK's debounced power armor state; flips only together with the skeleton generation.
    [[nodiscard]] inline bool isInPowerArmor()
    {
        auto* frikApi = api();
        return frikApi && frikApi->isInPowerArmor && frikApi->isInPowerArmor();
    }

    [[nodiscard]] inline bool canReportPowerArmor()
    {
        auto* frikApi = api();
        return frikApi && frikApi->isInPowerArmor != nullptr;
    }

    [[nodiscard]] inline bool setScopeProvider(const char* tag, std::uint32_t capabilities)
    {
        auto* frikApi = api();
        return frikApi && frikApi->setScopeProvider && frikApi->setScopeProvider(tag, capabilities);
    }

    [[nodiscard]] inline bool clearScopeProvider(const char* tag)
    {
        auto* frikApi = api();
        return frikApi && frikApi->clearScopeProvider && frikApi->clearScopeProvider(tag);
    }

    [[nodiscard]] inline bool isLookingThroughScope()
    {
        auto* frikApi = api();
        return frikApi && frikApi->isLookingThroughScope && frikApi->isLookingThroughScope();
    }
}
