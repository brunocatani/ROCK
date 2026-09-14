#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "api/FRIKApiV2.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"
#include "rock_support/Fo4VrRuntime.h"

/*
 * Null-checked wrapper over FRIK API v2 (v2.3). Every ROCK call into FRIK
 * goes through here so the table lookups, hand mapping and lifecycle rules
 * live in one place.
 *
 * Since v2.3 FRIK logs a hand pose or hand transform only when the claim
 * starts and ends, and re-setting a tag you hold updates it in place, so
 * there is no publish dedupe on this side any more.
 */
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

    using RebaseDriver = frik_hand_world_authority::RebaseDriver;

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

    /*
     * Rebase drivers name the controller chain a claim is expressed against.
     * FRIK v2.3 solves a claim in the frame it is published, so the driver is
     * only recorded with the claim; the helpers stay so owners keep declaring
     * their intent.
     */
    [[nodiscard]] inline RebaseDriver ownHandDriver(Hand hand)
    {
        bool isLeft = false;
        return tryResolveHandIsLeft(hand, isLeft) ? hand_world_claim_registry_policy::driverForHand(isLeft) : RebaseDriver::Static;
    }

    [[nodiscard]] inline RebaseDriver physicalHandDriver(bool isLeft)
    {
        return isLeft ? RebaseDriver::LeftHand : RebaseDriver::RightHand;
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

    // ---- Hand poses ----

    [[nodiscard]] inline bool clearHandPose(const char* tag, Hand hand)
    {
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
        return frikApi && frikApi->setHandPoseCustom && frikApi->setHandPoseCustom(tag, hand, handPose, priority);
    }

    [[nodiscard]] inline bool setHandPose(const char* tag, Hand hand, HandPoseKind handPose, int priority)
    {
        auto* frikApi = api();
        return frikApi && frikApi->setHandPose && frikApi->setHandPose(tag, hand, handPose, priority);
    }

    /*
     * Since v2.3 the per-bone locals survive a later setHandPose* update of
     * the same tag, so an owner may publish them in either order.
     */
    [[nodiscard]] inline bool setHandPoseCustomLocalTransforms(
        const char* tag,
        Hand hand,
        const FingerLocalTransformOverride* overrideData,
        int priority)
    {
        auto* frikApi = api();
        return frikApi && frikApi->setHandPoseCustomLocalTransforms && overrideData &&
            frikApi->setHandPoseCustomLocalTransforms(tag, hand, overrideData, priority);
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

    // ---- Hand world claims ----

    /*
     * Hand world claims go through the hand world authority service, which
     * mirrors them for ROCK's own readers. FRIK solves a claim published from
     * BeforeArmSolve in the same frame and re-solves one published from
     * AfterArmSolve before the frame continues. False means the claim is not
     * held anywhere (FRIK rejected it, or FRIK reported the target unreachable
     * last frame); the caller runs its failure reaction.
     */
    [[nodiscard]] inline bool publishHandWorld(const char* tag, Hand hand, const RE::NiTransform& worldTarget, int priority, RebaseDriver driver)
    {
        bool isLeft = false;
        if (!tryResolveHandIsLeft(hand, isLeft)) {
            return false;
        }
        return frik_hand_world_authority::publish(tag, isLeft, worldTarget, priority, driver);
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
     * registry (no scene read), optionally ignoring one tag.
     */
    [[nodiscard]] inline bool tryGetPublishedHandWorld(Hand hand, RE::NiTransform& outWorld, const char* excludedTag = nullptr)
    {
        bool isLeft = false;
        if (!tryResolveHandIsLeft(hand, isLeft)) {
            outWorld = {};
            return false;
        }
        return frik_hand_world_authority::tryGetPublishedHandWorld(isLeft, outWorld, excludedTag);
    }

    // ---- Blockers ----

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
     * Since v2.3 this is a pure write blocker: while blocked FRIK writes
     * nothing to the primary weapon node (no offsets, no per-frame re-glue)
     * and no longer changes which hand the node is parented under.
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

    // ---- Recoil ----

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
    [[nodiscard]] inline bool tryGetTrackedHandTransform(Hand hand, TrackedHandKind kind, RE::NiTransform& outWorld)
    {
        outWorld = {};
        auto* frikApi = api();
        if (!frikApi || !frikApi->getTrackedHandTransform || !frikApi->getTrackedHandTransform(hand, kind, &outWorld)) {
            outWorld = {};
            return false;
        }
        return detail::isFiniteNiTransform(outWorld);
    }

    /*
     * The first-person hand FRIK solves the body arm to when no claim is
     * published: the controller hand, dampening and native kick included.
     */
    [[nodiscard]] inline bool tryGetHandWorldTransform(Hand hand, RE::NiTransform& outWorld)
    {
        return tryGetTrackedHandTransform(hand, TrackedHandKind::FirstPersonHand, outWorld);
    }

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

    // Kept for callers that reset per-skeleton caches; the bridge holds none since v2.3.
    inline void resetPresentedHandNodeCache()
    {
    }
}
