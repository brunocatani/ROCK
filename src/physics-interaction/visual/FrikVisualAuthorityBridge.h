#pragma once

#include <array>

#include "api/FRIKApi.h"

namespace rock::frik_visual_authority
{
    using Hand = frik::api::FRIKApi::Hand;
    using HandPoseData = frik::api::FRIKApi::HandPoseData;
    using FingerLocalTransformOverride = frik::api::FRIKApi::FingerLocalTransformOverride;

    [[nodiscard]] inline HandPoseData makeHandPoseDataFromJointValues(const float values[15])
    {
        return HandPoseData{
            .thumb = { values[0], values[1], values[2], 0.0f },
            .index = { values[3], values[4], values[5], 0.0f },
            .middle = { values[6], values[7], values[8], 0.0f },
            .ring = { values[9], values[10], values[11], 0.0f },
            .pinky = { values[12], values[13], values[14], 0.0f },
            .palmPitch = 0.0f,
            .palmYaw = 0.0f,
        };
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
        float pinky)
    {
        return HandPoseData{
            .thumb = { thumb, thumb, thumb, 0.0f },
            .index = { index, index, index, 0.0f },
            .middle = { middle, middle, middle, 0.0f },
            .ring = { ring, ring, ring, 0.0f },
            .pinky = { pinky, pinky, pinky, 0.0f },
            .palmPitch = 0.0f,
            .palmYaw = 0.0f,
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
        auto* frikApi = api();
        return frikApi && frikApi->clearHandPose && frikApi->clearHandPose(tag, hand);
    }

    [[nodiscard]] inline bool setHandPoseCustomWithPriority(const char* tag, Hand hand, const HandPoseData& handPose, int priority)
    {
        auto* frikApi = api();
        return frikApi && frikApi->setHandPoseCustomWithPriority && frikApi->setHandPoseCustomWithPriority(tag, hand, handPose, priority);
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

    [[nodiscard]] inline RE::NiTransform getHandWorldTransform(Hand hand)
    {
        auto* frikApi = api();
        return frikApi ? frikApi->getHandWorldTransform(hand) : RE::NiTransform();
    }
}
