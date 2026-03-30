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

namespace rock::api
{
#if defined(ROCK_API_EXPORTS)
#define ROCK_API extern "C" __declspec(dllexport)
#else
#define ROCK_API extern "C" __declspec(dllimport)
#endif

#define ROCK_CALL __cdecl

    inline constexpr std::uint32_t ROCK_API_VERSION = 4;

    struct ROCKApi
    {
        static constexpr auto ROCK_F4SE_MOD_NAME = "ROCK";

        enum class Hand : std::uint8_t
        {
            Primary,
            Offhand,
            Right,
            Left,
        };

        enum PhysicsMessage : std::uint32_t
        {
            kOnTouch = 100,
            kOnTouchEnd = 101,
            kOnGrab = 102,
            kOnRelease = 103,
            kOnPhysicsInit = 104,
            kOnPhysicsShutdown = 105,
            kOnGrabEvent = 200,
        };

        enum class GrabEventType : std::uint32_t
        {
            Unknown = 0,
            SelectionLocked = 1,
            PullStarted = 2,
            PullArrived = 3,
            PullCatchAttempt = 4,
            PullCatchSucceeded = 5,
            GrabCommitted = 6,
            HeldImpact = 7,
            Released = 8,
            // Reserved scaffold events below are ABI-visible but are only
            // dispatched once their backing gameplay/runtime state exists.
            TwoHandStarted = 9,
            TwoHandStopped = 10,
            StashCandidate = 11,
            ConsumeCandidate = 12,
            Stashed = 13,
            Consumed = 14,
            LootStarted = 15,
            LootCompleted = 16,
            SelectionUnlocked = 17,
        };

        enum class GrabEventSourceKind : std::uint32_t
        {
            Unknown = 0,
            Hand = 1,
            HeldObject = 2,
            PulledObject = 3,
            Weapon = 4,
            External = 5,
        };

        static constexpr std::uint32_t kGrabEventFlagHeldImpactDamped = 1u << 0;
        static constexpr std::uint32_t kGrabEventFlagSuppressHaptic = 1u << 1;
        static constexpr std::uint32_t kGrabEventFlagPositionValid = 1u << 2;
        static constexpr std::uint32_t kGrabEventFlagVelocityValid = 1u << 3;
        static constexpr std::uint32_t kGrabEventFlagMassValid = 1u << 4;
        static constexpr std::uint32_t kGrabEventFlagSpeedValid = 1u << 5;
        static constexpr std::uint32_t kGrabEventFlagIntensityValid = 1u << 6;

        enum class WeaponPartKind : std::uint8_t
        {
            Receiver,
            Barrel,
            Handguard,
            Foregrip,
            Pump,
            Stock,
            Grip,
            Magazine,
            Magwell,
            Bolt,
            Slide,
            ChargingHandle,
            BreakAction,
            Cylinder,
            Chamber,
            Shell,
            Round,
            LaserCell,
            Lever,
            Sight,
            Accessory,
            CosmeticAmmo,
            Other,
        };

        struct PhysicsEventData
        {
            bool isLeft;
            RE::TESObjectREFR* refr;
            std::uint32_t formID;
            std::uint32_t collisionLayer;
        };

        struct GrabEventData
        {
            std::uint32_t size{ sizeof(GrabEventData) };
            std::uint32_t version{ 1 };
            GrabEventType type{ GrabEventType::Unknown };
            GrabEventSourceKind sourceKind{ GrabEventSourceKind::Unknown };
            bool isLeft{ false };
            std::uint8_t reservedBool[3]{};
            RE::TESObjectREFR* refr{ nullptr };
            std::uint32_t formID{ 0 };
            std::uint32_t primaryBodyId{ 0x7FFF'FFFF };
            std::uint32_t secondaryBodyId{ 0x7FFF'FFFF };
            std::uint32_t collisionLayer{ 0 };
            std::uint32_t flags{ 0 };
            std::uint64_t frameIndex{ 0 };
            float positionGame[3]{};
            float velocityGame[3]{};
            float mass{ 0.0f };
            float speedGameUnitsPerSecond{ 0.0f };
            float intensityHint{ 0.0f };
            float reservedFloat[3]{};
        };

        std::uint32_t(ROCK_CALL* getVersion)();
        const char*(ROCK_CALL* getModVersion)();
        bool(ROCK_CALL* isPhysicsInteractionReady)();
        RE::NiPoint3(ROCK_CALL* getPalmPosition)(Hand hand);
        RE::NiPoint3(ROCK_CALL* getPalmForward)(Hand hand);
        bool(ROCK_CALL* isHandTouching)(Hand hand);
        RE::TESObjectREFR*(ROCK_CALL* getLastTouchedObject)(Hand hand);
        std::uint32_t(ROCK_CALL* getLastTouchedLayer)(Hand hand);
        bool(ROCK_CALL* isHandHolding)(Hand hand);
        RE::TESObjectREFR*(ROCK_CALL* getHeldObject)(Hand hand);
        RE::TESObjectREFR*(ROCK_CALL* getSelectedObject)(Hand hand);
        void(ROCK_CALL* disablePhysicsHand)(Hand hand);
        void(ROCK_CALL* enablePhysicsHand)(Hand hand);
        bool(ROCK_CALL* isPhysicsHandDisabled)(Hand hand);
        bool(ROCK_CALL* claimPhysicsObject)(RE::TESObjectREFR* refr);
        bool(ROCK_CALL* releasePhysicsObject)(RE::TESObjectREFR* refr);
        bool(ROCK_CALL* isPhysicsObjectClaimed)(RE::TESObjectREFR* refr);
        void(ROCK_CALL* forceDropObject)(Hand hand);
        std::uint32_t(ROCK_CALL* getLastTouchedWeaponPartKind)();

        [[nodiscard]] static int initialize(const uint32_t minVersion = ROCK_API_VERSION)
        {
            if (inst) {
                return 0;
            }

            const auto rockDll = GetModuleHandleA("ROCK.dll");
            if (!rockDll) {
                return 1;
            }

            const auto getApi = reinterpret_cast<const ROCKApi*(ROCK_CALL*)()>(GetProcAddress(rockDll, "ROCKAPI_GetApi"));
            if (!getApi) {
                return 2;
            }

            const auto rockApi = getApi();
            if (!rockApi) {
                return 3;
            }

            if (rockApi->getVersion() < minVersion) {
                return 4;
            }

            inst = rockApi;
            return 0;
        }

        inline static const ROCKApi* inst = nullptr;
    };
}

#if defined(ROCK_API_EXPORTS)
namespace rock
{
    class PhysicsInteraction;
}

namespace rock::api
{
    void setPhysicsInteractionInstance(rock::PhysicsInteraction* pi);
}
#endif
