#pragma once

// ROCKApi.h — Consumer header for the ROCK physics interaction API.
//
// Copy this file into your mod project to query ROCK physics state from an external DLL.
// Pattern mirrors FRIKApi.h: struct of function pointers, loaded via GetModuleHandle/GetProcAddress.
//
// EXAMPLE USAGE:
// Copy this whole file into your project AS IS.
// Call initialize in the GameLoaded event.
//
// {
//     const int err = rock::api::ROCKApi::initialize();
//     if (err != 0) {
//         logger::error("ROCK API init failed with error: {}!", err);
//     }
//     logger::info("ROCK (v{}) API (v{}) init successful!",
//         rock::api::ROCKApi::inst->getModVersion(), rock::api::ROCKApi::inst->getVersion());
//
//     // later...
//     if (!rock::api::ROCKApi::inst->isPhysicsInteractionReady())
//         return;
//
//     bool touching = rock::api::ROCKApi::inst->isHandTouching(rock::api::ROCKApi::Hand::Left);
//     auto* held = rock::api::ROCKApi::inst->getHeldObject(rock::api::ROCKApi::Hand::Primary);
//
//     // Disable ROCK for a hand while your mod controls it:
//     rock::api::ROCKApi::inst->disablePhysicsHand(rock::api::ROCKApi::Hand::Right);
//     // ... later:
//     rock::api::ROCKApi::inst->enablePhysicsHand(rock::api::ROCKApi::Hand::Right);
// }

#include <cstdint>
#include <Windows.h>

#include "RE/NetImmerse/NiPoint.h"

namespace rock::api
{
#if defined(ROCK_API_EXPORTS)
#   define ROCK_API extern "C" __declspec(dllexport)
#else
#   define ROCK_API extern "C" __declspec(dllimport)
#endif

#define ROCK_CALL __cdecl

    /// API version — bump when function pointers are added or reordered.
    inline constexpr std::uint32_t ROCK_API_VERSION = 1;

    struct ROCKApi
    {
        /**
         * The name of ROCK mod as registered in F4SE, used to send/receive messages.
         * Example:
         * _messaging->RegisterListener(onROCKMessage, rock::api::ROCKApi::ROCK_F4SE_MOD_NAME);
         */
        static constexpr auto ROCK_F4SE_MOD_NAME = "ROCK";

        /**
         * The player hand to act on, with support for left-handed mode.
         * Matches FRIKApi::Hand for consistency.
         */
        enum class Hand : std::uint8_t
        {
            Primary,   ///< Dominant hand (right unless left-handed mode)
            Offhand,   ///< Non-dominant hand
            Right,     ///< Always right
            Left,      ///< Always left
        };

        /// F4SE message types for physics events.
        /// Listen via: _messaging->RegisterListener(callback, ROCKApi::ROCK_F4SE_MOD_NAME);
        enum PhysicsMessage : std::uint32_t
        {
            kOnTouch            = 100,
            kOnTouchEnd         = 101,
            kOnGrab             = 102,
            kOnRelease          = 103,
            kOnPhysicsInit      = 104,
            kOnPhysicsShutdown  = 105,
        };

        /// Data payload for physics F4SE messages.
        struct PhysicsEventData
        {
            bool              isLeft;
            RE::TESObjectREFR* refr;
            std::uint32_t     formID;
            std::uint32_t     collisionLayer;
        };

        // =====================================================================
        // Function pointers — populated by ROCK.dll at load time
        // =====================================================================

        /**
         * Get the ROCK API version number.
         * Use this to check compatibility before calling other functions.
         */
        std::uint32_t (ROCK_CALL*getVersion)();

        /**
         * Get the ROCK mod version string. e.g. "0.1.0"
         */
        const char* (ROCK_CALL*getModVersion)();

        // --- Physics interaction state ---

        /**
         * Check if the ROCK physics interaction module is initialized and ready.
         */
        bool (ROCK_CALL*isPhysicsInteractionReady)();

        /**
         * Get the world-space palm position for the given hand.
         */
        RE::NiPoint3 (ROCK_CALL*getPalmPosition)(Hand hand);

        /**
         * Get the palm forward direction (fingertip pointing direction).
         */
        RE::NiPoint3 (ROCK_CALL*getPalmForward)(Hand hand);

        /**
         * Check if the hand is currently touching a physics object.
         */
        bool (ROCK_CALL*isHandTouching)(Hand hand);

        /**
         * Get the last object touched by the hand (nullptr if none).
         */
        RE::TESObjectREFR* (ROCK_CALL*getLastTouchedObject)(Hand hand);

        /**
         * Get the collision layer of the last touched object.
         */
        std::uint32_t (ROCK_CALL*getLastTouchedLayer)(Hand hand);

        /**
         * Check if the hand is currently holding a physics object via ROCK grab.
         */
        bool (ROCK_CALL*isHandHolding)(Hand hand);

        /**
         * Get the object currently held by the hand (nullptr if not holding).
         */
        RE::TESObjectREFR* (ROCK_CALL*getHeldObject)(Hand hand);

        /**
         * Get the currently selected (highlighted) object for the hand.
         */
        RE::TESObjectREFR* (ROCK_CALL*getSelectedObject)(Hand hand);

        // --- Hand enable/disable ---

        /**
         * Disable ROCK physics updates for the given hand (external mod takes over).
         */
        void (ROCK_CALL*disablePhysicsHand)(Hand hand);

        /**
         * Re-enable ROCK physics updates for the given hand.
         */
        void (ROCK_CALL*enablePhysicsHand)(Hand hand);

        /**
         * Check if ROCK physics is disabled for the given hand.
         */
        bool (ROCK_CALL*isPhysicsHandDisabled)(Hand hand);

        // --- Object ownership ---

        /**
         * Claim ownership of an object (prevents ROCK from interacting with it).
         */
        bool (ROCK_CALL*claimPhysicsObject)(RE::TESObjectREFR* refr);

        /**
         * Release ownership of an object (allows ROCK to interact with it again).
         */
        bool (ROCK_CALL*releasePhysicsObject)(RE::TESObjectREFR* refr);

        /**
         * Check if an object is currently claimed by any external mod.
         */
        bool (ROCK_CALL*isPhysicsObjectClaimed)(RE::TESObjectREFR* refr);

        // --- Force actions ---

        /**
         * Force the hand to drop its held object. No-op if not holding.
         * Must be called from the main thread.
         */
        void (ROCK_CALL*forceDropObject)(Hand hand);

        // =====================================================================
        // Initialization
        // =====================================================================

        /**
         * Initialize the ROCK API object.
         * NOTE: call after all mods have been loaded in the game (GameLoaded event).
         *
         * @param minVersion the minimal version required (default: the compiled-against version)
         * @return error codes:
         * 0 - Successful
         * 1 - Failed to find ROCK.dll (trying to init too early?)
         * 2 - No ROCKAPI_GetApi export found
         * 3 - Failed ROCKAPI_GetApi call
         * 4 - ROCK API version is older than the minimal required version
         */
        [[nodiscard]] static int initialize(const uint32_t minVersion = ROCK_API_VERSION)
        {
            if (inst) {
                return 0;
            }

            const auto rockDll = GetModuleHandleA("ROCK.dll");
            if (!rockDll) {
                return 1;
            }

            const auto getApi = reinterpret_cast<const ROCKApi* (ROCK_CALL*)()>(GetProcAddress(rockDll, "ROCKAPI_GetApi"));
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

        /**
         * The initialized instance of the ROCK API interface.
         * Use after a successful call to initialize().
         */
        inline static const ROCKApi* inst = nullptr;
    };
}

// =====================================================================
// Internal-only declarations (ROCK DLL build only, not part of the consumer API)
// =====================================================================

#if defined(ROCK_API_EXPORTS)
namespace frik::rock { class PhysicsInteraction; }

namespace rock::api
{
    /// Set the PhysicsInteraction instance that API functions will query.
    /// Called by ROCKMain.cpp after creating PhysicsInteraction.
    /// Pass nullptr on shutdown to prevent stale pointer access.
    void setPhysicsInteractionInstance(frik::rock::PhysicsInteraction* pi);
}
#endif
