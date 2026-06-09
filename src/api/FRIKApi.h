#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef NOMMNOSOUND
#define NOMMNOSOUND
#endif

#pragma push_macro("near")
#pragma push_macro("far")
#pragma push_macro("MEM_RELEASE")
#pragma push_macro("MEM_COMMIT")
#pragma push_macro("MEM_RESERVE")
#pragma push_macro("PAGE_EXECUTE_READ")
#pragma push_macro("PAGE_EXECUTE_READWRITE")
#pragma push_macro("MAX_PATH")
#ifdef near
#undef near
#endif
#ifdef far
#undef far
#endif
#ifdef MEM_RELEASE
#undef MEM_RELEASE
#endif
#ifdef MEM_COMMIT
#undef MEM_COMMIT
#endif
#ifdef MEM_RESERVE
#undef MEM_RESERVE
#endif
#ifdef PAGE_EXECUTE_READ
#undef PAGE_EXECUTE_READ
#endif
#ifdef PAGE_EXECUTE_READWRITE
#undef PAGE_EXECUTE_READWRITE
#endif
#ifdef MAX_PATH
#undef MAX_PATH
#endif

#include <cstdint>

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#pragma pop_macro("MAX_PATH")
#pragma pop_macro("PAGE_EXECUTE_READWRITE")
#pragma pop_macro("PAGE_EXECUTE_READ")
#pragma pop_macro("MEM_RESERVE")
#pragma pop_macro("MEM_COMMIT")
#pragma pop_macro("MEM_RELEASE")
#pragma pop_macro("far")
#pragma pop_macro("near")

#if !defined(_WINDOWS_) && !defined(_INC_WINDOWS)
struct HINSTANCE__;
using HMODULE = HINSTANCE__*;
using LPCSTR = const char*;
using FARPROC = std::intptr_t(__stdcall*)();
extern "C" __declspec(dllimport) HMODULE __stdcall GetModuleHandleA(LPCSTR lpModuleName);
extern "C" __declspec(dllimport) FARPROC __stdcall GetProcAddress(HMODULE hModule, LPCSTR lpProcName);
#endif

namespace frik::api
{
#if defined(FRIK_API_EXPORTS)
#define FRIK_API extern "C" __declspec(dllexport)
#else
#define FRIK_API extern "C" __declspec(dllimport)
#endif

#define FRIK_CALL __cdecl

    inline constexpr std::uint32_t FRIK_API_VERSION = 5;

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

        enum class HandPoseKind : std::uint8_t
        {

            Unset = 0,

            Custom = 1,
            Open = 2,
            Pointing = 3,
            HoldingWeapon = 4,
            OffhandGrip = 5,
            Attaboy = 6,
            ThumbsUp = 7,

            Fist = 8,
            HoldingGun = 9,
            HoldingMelee = 10,
        };

        using HandPoses = HandPoseKind;

        struct FingerPoseData
        {
            float prox = 0.0f;
            float mid = 0.0f;
            float dist = 0.0f;
            float splay = 0.0f;
        };

        struct HandPoseData
        {
            FingerPoseData thumb;
            FingerPoseData index;
            FingerPoseData middle;
            FingerPoseData ring;
            FingerPoseData pinky;
            float palmPitch = 0.0f;
            float palmYaw = 0.0f;
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

        HandPoseKind(FRIK_CALL* getCurrentHandPose)(Hand hand);

        bool(FRIK_CALL* setHandPose)(const char* tag, Hand hand, HandPoseKind handPose);

        bool(FRIK_CALL* setHandPoseCustomFingerPositions)(const char* tag, Hand hand, float thumb, float index, float middle, float ring, float pinky);

        bool(FRIK_CALL* clearHandPose)(const char* tag, Hand hand);

        void(FRIK_CALL* setHandPoseFingerPositions)(Hand hand, float thumb, float index, float middle, float ring, float pinky);

        void(FRIK_CALL* clearHandPoseFingerPositions)(Hand hand);

        bool(FRIK_CALL* registerOpenModSettingButtonToMainConfig)(const OpenExternalModConfigData& data);

        bool(FRIK_CALL* blockOffHandWeaponGripping)(const char* tag, bool block);

        bool(FRIK_CALL* setHandPoseCustom)(const char* tag, Hand hand, const HandPoseData& handPose, bool forceTop);

        bool(FRIK_CALL* setHandPoseWithPriority)(const char* tag, Hand hand, HandPoseKind handPose, int priority);

        RE::NiTransform(FRIK_CALL* getHandWorldTransform)(Hand hand);

        bool(FRIK_CALL* setHandPoseCustomFingerPositionsWithPriority)(const char* tag, Hand hand, float thumb, float index, float middle, float ring, float pinky, int priority);

        bool(FRIK_CALL* setHandPoseCustomWithPriority)(const char* tag, Hand hand, const HandPoseData& handPose, int priority);

        bool(FRIK_CALL* applyExternalHandWorldTransform)(const char* tag, Hand hand, const RE::NiTransform& worldTarget, int priority);

        bool(FRIK_CALL* clearExternalHandWorldTransform)(const char* tag, Hand hand);

        // Local transforms augment an existing scalar/per-joint pose under the
        // same tag; FRIK rejects transform-only tags so vanilla weapon hand poses
        // are not suppressed by incomplete overrides.
        bool(FRIK_CALL* setHandPoseCustomLocalTransformsWithPriority)(const char* tag, Hand hand, const FingerLocalTransformOverride* overrideData, int priority);

        bool(FRIK_CALL* getHandPoseLocalTransformsForPose)(Hand hand, const HandPoseData& handPose, FingerLocalTransformOverride* outTransforms);

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

    static_assert(FRIK_API_VERSION == 5, "ROCK requires the local FRIK API v5 canonical 22-float hand-pose and visual-authority contract");
}
