#pragma once

#include <Windows.h>
#include <cstdint>

#include "RE/NetImmerse/NiPoint.h"

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
        };

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
namespace frik::rock
{
    class PhysicsInteraction;
}

namespace rock::api
{
    void setPhysicsInteractionInstance(frik::rock::PhysicsInteraction* pi);
}
#endif
