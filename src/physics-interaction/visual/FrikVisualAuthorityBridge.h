#pragma once

#include "api/FRIKApi.h"

namespace rock::frik_visual_authority
{
    using Hand = frik::api::FRIKApi::Hand;
    using HandPoses = frik::api::FRIKApi::HandPoses;
    using FingerLocalTransformOverride = frik::api::FRIKApi::FingerLocalTransformOverride;

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
        auto* frikApi = api();
        return frikApi && frikApi->clearHandPose && frikApi->clearHandPose(tag, hand);
    }

    [[nodiscard]] inline bool setHandPoseWithPriority(const char* tag, Hand hand, HandPoses pose, int priority)
    {
        auto* frikApi = api();
        return frikApi && frikApi->setHandPoseWithPriority && frikApi->setHandPoseWithPriority(tag, hand, pose, priority);
    }

    [[nodiscard]] inline bool setHandPoseCustomFingerPositionsWithPriority(
        const char* tag,
        Hand hand,
        float thumb,
        float index,
        float middle,
        float ring,
        float pinky,
        int priority)
    {
        auto* frikApi = api();
        return frikApi &&
            frikApi->setHandPoseCustomFingerPositionsWithPriority &&
            frikApi->setHandPoseCustomFingerPositionsWithPriority(tag, hand, thumb, index, middle, ring, pinky, priority);
    }

    [[nodiscard]] inline bool setHandPoseCustomJointPositionsWithPriority(const char* tag, Hand hand, const float values[15], int priority)
    {
        auto* frikApi = api();
        return frikApi && frikApi->setHandPoseCustomJointPositionsWithPriority && frikApi->setHandPoseCustomJointPositionsWithPriority(tag, hand, values, priority);
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
        return frikApi &&
            frikApi->setHandPoseCustomLocalTransformsWithPriority &&
            frikApi->setHandPoseCustomLocalTransformsWithPriority(tag, hand, overrideData, priority);
    }

    [[nodiscard]] inline bool getHandPoseLocalTransformsForJointPositions(
        Hand hand,
        const float values[15],
        FingerLocalTransformOverride* outTransforms)
    {
        auto* frikApi = api();
        return frikApi &&
            frikApi->getHandPoseLocalTransformsForJointPositions &&
            frikApi->getHandPoseLocalTransformsForJointPositions(hand, values, outTransforms);
    }

    [[nodiscard]] inline bool blockOffHandWeaponGripping(const char* tag, bool block)
    {
        auto* frikApi = api();
        return frikApi && frikApi->blockOffHandWeaponGripping && frikApi->blockOffHandWeaponGripping(tag, block);
    }

    [[nodiscard]] inline RE::NiTransform getHandWorldTransform(Hand hand)
    {
        auto* frikApi = api();
        return frikApi ? frikApi->getHandWorldTransform(hand) : RE::NiTransform();
    }
}
