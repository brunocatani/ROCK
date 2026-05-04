#pragma once

#include <Windows.h>
#include <cstdint>

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

namespace RE
{
    class NiNode;
}

namespace frik::api
{
#if defined(FRIK_API_EXPORTS)
#define FRIK_API extern "C" __declspec(dllexport)
#else
#define FRIK_API extern "C" __declspec(dllimport)
#endif

#define FRIK_CALL __cdecl

    inline constexpr std::uint32_t FRIK_API_VERSION = 9;

    struct FRIKApi
    {
        static constexpr auto FRIK_F4SE_MOD_NAME = "F4VRBody";

        enum class Hand : std::uint8_t
        {
            Primary,
            Offhand,
            Right,
            Left,
        };

        enum class HandPoses : std::uint8_t
        {

            Unset,

            Custom,
            Open,
            Fist,
            Pointing,
            HoldingGun,
            HoldingMelee,
        };

        enum class HandPoseTagState : std::uint8_t
        {

            None,

            Active,

            Overriden,
        };

        struct OpenExternalModConfigData
        {
            const char* buttonIconNifPath;
            const char* callbackReceiverName;
            std::uint32_t callbackMessageType;
        };

        struct FingerLocalTransformOverride
        {
            std::uint16_t enabledMask = 0;
            std::uint16_t reserved[3] = {};
            RE::NiTransform localTransforms[15] = {};
        };

        enum class LifecycleEvent : std::uint32_t
        {

            kSkeletonReady = 100,

            kSkeletonDestroying = 101,

            kPowerArmorChanged = 102,
        };

        std::uint32_t(FRIK_CALL* getVersion)();

        const char*(FRIK_CALL* getModVersion)();

        bool(FRIK_CALL* isSkeletonReady)();

        bool(FRIK_CALL* isConfigOpen)();

        bool(FRIK_CALL* isSelfieModeOn)();

        void(FRIK_CALL* setSelfieModeOn)(bool setOn);

        bool(FRIK_CALL* isOffHandGrippingWeapon)();

        bool(FRIK_CALL* isWristPipboyOpen)();

        RE::NiPoint3(FRIK_CALL* getIndexFingerTipPosition)(Hand hand);

        HandPoseTagState(FRIK_CALL* getHandPoseSetTagState)(const char* tag, Hand hand);

        HandPoses(FRIK_CALL* getCurrentHandPose)(Hand hand);

        bool(FRIK_CALL* setHandPose)(const char* tag, Hand hand, HandPoses handPose);

        bool(FRIK_CALL* setHandPoseCustomFingerPositions)(const char* tag, Hand hand, float thumb, float index, float middle, float ring, float pinky);

        bool(FRIK_CALL* clearHandPose)(const char* tag, Hand hand);

        void(FRIK_CALL* setHandPoseFingerPositions)(Hand hand, float thumb, float index, float middle, float ring, float pinky);

        void(FRIK_CALL* clearHandPoseFingerPositions)(Hand hand);

        bool(FRIK_CALL* registerOpenModSettingButtonToMainConfig)(const OpenExternalModConfigData& data);

        bool(FRIK_CALL* blockOffHandWeaponGripping)(const char* tag, bool block);

        bool(FRIK_CALL* setHandPoseCustomJointPositions)(const char* tag, Hand hand, const float values[15]);

        bool(FRIK_CALL* setHandPoseWithPriority)(const char* tag, Hand hand, HandPoses handPose, int priority);

        RE::NiTransform(FRIK_CALL* getHandWorldTransform)(Hand hand);

        RE::NiNode*(FRIK_CALL* getHandNode)(Hand hand);

        bool(FRIK_CALL* isAnyMenuOpen)();

        RE::NiPoint3(FRIK_CALL* getSmoothedPlayerPosition)();

        bool(FRIK_CALL* isPlayerMoving)();

        bool(FRIK_CALL* isOffhandNearWeaponBarrel)();

        bool(FRIK_CALL* isInPowerArmor)();

        bool(FRIK_CALL* isWeaponDrawn)();

        bool(FRIK_CALL* isMeleeWeaponDrawn)();

        float(FRIK_CALL* getFrameTime)();

        bool(FRIK_CALL* setHandPoseCustomFingerPositionsWithPriority)(const char* tag, Hand hand, float thumb, float index, float middle, float ring, float pinky, int priority);

        bool(FRIK_CALL* setHandPoseCustomJointPositionsWithPriority)(const char* tag, Hand hand, const float values[15], int priority);

        bool(FRIK_CALL* applyExternalHandWorldTransform)(const char* tag, Hand hand, const RE::NiTransform& worldTarget, int priority);

        bool(FRIK_CALL* clearExternalHandWorldTransform)(const char* tag, Hand hand);

        // Local transforms augment an existing scalar/per-joint pose under the
        // same tag; FRIK rejects transform-only tags so vanilla weapon hand poses
        // are not suppressed by incomplete overrides.
        bool(FRIK_CALL* setHandPoseCustomLocalTransformsWithPriority)(const char* tag, Hand hand, const FingerLocalTransformOverride* overrideData, int priority);

        bool(FRIK_CALL* getHandPoseLocalTransformsForJointPositions)(Hand hand, const float values[15], FingerLocalTransformOverride* outTransforms);

        [[nodiscard]] static int initialize(const uint32_t minVersion = FRIK_API_VERSION)
        {
            if (inst) {
                return 0;
            }

            const auto frikDll = GetModuleHandleA("FRIK.dll");
            if (!frikDll) {
                return 1;
            }

            const auto getApi = reinterpret_cast<const FRIKApi*(FRIK_CALL*)()>(GetProcAddress(frikDll, "FRIKAPI_GetApi"));
            if (!getApi) {
                return 2;
            }

            const auto frikApi = getApi();
            if (!frikApi) {
                return 3;
            }

            if (frikApi->getVersion() < minVersion) {
                return 4;
            }

            inst = frikApi;
            return 0;
        }

        inline static const FRIKApi* inst = nullptr;
    };

    static_assert(FRIK_API_VERSION == 9, "ROCK requires FRIK API v9 baseline local finger transform support");
}
