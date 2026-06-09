#pragma once

#include <array>
#include <cmath>

#include "api/FRIKApi.h"

namespace rock::frik_visual_authority
{
    using Hand = frik::api::FRIKApi::Hand;
    using HandPoseData = frik::api::FRIKApi::HandPoseData;
    using FingerLocalTransformOverride = frik::api::FRIKApi::FingerLocalTransformOverride;

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
