// ROCKApi.cpp — ROCK physics interaction API implementation.
//
// WHY: ROCK is being split from FRIK into its own DLL. External mods that want to
// query physics interaction state (touch, grab, selection, ownership) use ROCKApi
// instead of FRIKApi's physics section. This file provides 18 API function
// implementations (2 version + 16 physics) plus the exported ROCKAPI_GetApi entry point.
//
// DESIGN DECISIONS:
// - Module-level PhysicsInteraction* pointer (`s_physicsInteraction`) set by ROCKMain
//   after creating the PhysicsInteraction instance. This avoids g_frik dependency.
// - Palm position/forward queries call through FRIKApi to get the hand world transform,
//   then compute palm offset locally using PalmTransform.h. This keeps the skeleton
//   ownership cleanly in FRIK while ROCK only computes the physics-relevant palm offset.
// - The getIsLeftForHandEnum() helper is duplicated here (same logic as FRIK's) because
//   ROCK is a separate DLL and cannot share the monolith's anonymous-namespace helper.
// - PhysicsEventData layout is verified via static_assert against the internal struct
//   to guarantee binary compatibility over F4SE messaging.

#define ROCK_API_EXPORTS
#include "ROCKApi.h"

#include <atomic>

#include "FRIKApi.h"
#include "physics-interaction/PalmTransform.h"
#include "physics-interaction/PhysicsInteraction.h"

// Verify binary layout compatibility between the internal and API PhysicsEventData structs.
// Both are transmitted as raw bytes via F4SE messaging — layout MUST match exactly.
static_assert(sizeof(frik::rock::PhysicsEventData) == sizeof(rock::api::ROCKApi::PhysicsEventData),
    "Internal and API PhysicsEventData must have identical size");
static_assert(offsetof(frik::rock::PhysicsEventData, isLeft) == offsetof(rock::api::ROCKApi::PhysicsEventData, isLeft),
    "PhysicsEventData::isLeft offset mismatch");
static_assert(offsetof(frik::rock::PhysicsEventData, refr) == offsetof(rock::api::ROCKApi::PhysicsEventData, refr),
    "PhysicsEventData::refr offset mismatch");
static_assert(offsetof(frik::rock::PhysicsEventData, formID) == offsetof(rock::api::ROCKApi::PhysicsEventData, formID),
    "PhysicsEventData::formID offset mismatch");
static_assert(offsetof(frik::rock::PhysicsEventData, collisionLayer) == offsetof(rock::api::ROCKApi::PhysicsEventData, collisionLayer),
    "PhysicsEventData::collisionLayer offset mismatch");

// Verify expected struct size (bool + 7 padding + ptr + uint32 + uint32 = 24 bytes on x64)
static_assert(sizeof(frik::rock::PhysicsEventData) == 24, "PhysicsEventData unexpected size — check struct packing");
static_assert(sizeof(rock::api::ROCKApi::PhysicsEventData) == 24, "ROCKApi::PhysicsEventData unexpected size — check struct packing");

// Verify Hand enum values match between ROCKApi and FRIKApi
static_assert(static_cast<int>(rock::api::ROCKApi::Hand::Primary) == static_cast<int>(frik::api::FRIKApi::Hand::Primary), "Hand::Primary mismatch");
static_assert(static_cast<int>(rock::api::ROCKApi::Hand::Offhand) == static_cast<int>(frik::api::FRIKApi::Hand::Offhand), "Hand::Offhand mismatch");
static_assert(static_cast<int>(rock::api::ROCKApi::Hand::Right) == static_cast<int>(frik::api::FRIKApi::Hand::Right), "Hand::Right mismatch");
static_assert(static_cast<int>(rock::api::ROCKApi::Hand::Left) == static_cast<int>(frik::api::FRIKApi::Hand::Left), "Hand::Left mismatch");

namespace
{
    using namespace rock::api;
    using namespace frik::rock;

    // =====================================================================
    // Module-level state
    // =====================================================================

    /// PhysicsInteraction instance pointer, set by setPhysicsInteractionInstance().
    /// ROCKMain.cpp calls this after creating PhysicsInteraction. All API functions
    /// check this pointer before accessing physics state.
    /// Atomic because the setter runs on the main thread while API reads may occur
    /// from any thread (e.g. external mod queries via ROCKApi).
    std::atomic<PhysicsInteraction*> s_physicsInteraction{ nullptr };

    // =====================================================================
    // Hand enum → isLeft resolution
    // =====================================================================

    /// Resolve Hand enum to bool isLeft, respecting left-handed mode.
    /// Duplicated from FRIK's FRIKApi.cpp because ROCK is a separate DLL.
    bool getIsLeftForHandEnum(const ROCKApi::Hand hand)
    {
        switch (hand) {
        case ROCKApi::Hand::Primary:
            return f4vr::isLeftHandedMode();
        case ROCKApi::Hand::Offhand:
            return !f4vr::isLeftHandedMode();
        case ROCKApi::Hand::Right:
            return false;
        case ROCKApi::Hand::Left:
            return true;
        }
        return false;
    }

    // =====================================================================
    // API version / info
    // =====================================================================

    std::uint32_t ROCK_CALL apiGetVersion()
    {
        return ROCK_API_VERSION;
    }

    const char* ROCK_CALL apiGetModVersion()
    {
        // ROCK version — updated with each release.
        // Using a static string literal so the pointer remains valid for the DLL lifetime.
        static constexpr const char* version = "0.1.0";
        return version;
    }

    // =====================================================================
    // Physics interaction state queries
    // =====================================================================

    bool ROCK_CALL apiIsPhysicsInteractionReady()
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        return pi && pi->isInitialized();
    }

    RE::NiPoint3 ROCK_CALL apiGetPalmPosition(const ROCKApi::Hand hand)
    {
        // Get the hand world transform from FRIK (skeleton owner), then compute palm offset locally.
        const auto* frikApi = frik::api::FRIKApi::inst;
        if (!frikApi) return RE::NiPoint3{};

        // Map ROCKApi::Hand to FRIKApi::Hand — identical enum layout, safe to cast.
        auto frikHand = static_cast<frik::api::FRIKApi::Hand>(static_cast<std::uint8_t>(hand));
        auto transform = frikApi->getHandWorldTransform(frikHand);

        bool isLeft = getIsLeftForHandEnum(hand);
        return computePalmPosition(transform, isLeft);
    }

    RE::NiPoint3 ROCK_CALL apiGetPalmForward(const ROCKApi::Hand hand)
    {
        const auto* frikApi = frik::api::FRIKApi::inst;
        if (!frikApi) return RE::NiPoint3{};

        auto frikHand = static_cast<frik::api::FRIKApi::Hand>(static_cast<std::uint8_t>(hand));
        auto transform = frikApi->getHandWorldTransform(frikHand);

        bool isLeft = getIsLeftForHandEnum(hand);
        return computePalmForward(transform, isLeft);
    }

    // =====================================================================
    // Touch state
    // =====================================================================

    bool ROCK_CALL apiIsHandTouching(const ROCKApi::Hand hand)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) return false;
        bool isLeft = getIsLeftForHandEnum(hand);
        const auto& h = isLeft ? pi->getLeftHand() : pi->getRightHand();
        return h.isTouching();
    }

    RE::TESObjectREFR* ROCK_CALL apiGetLastTouchedObject(const ROCKApi::Hand hand)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) return nullptr;
        bool isLeft = getIsLeftForHandEnum(hand);
        const auto& h = isLeft ? pi->getLeftHand() : pi->getRightHand();
        return h.getLastTouchedRef();
    }

    std::uint32_t ROCK_CALL apiGetLastTouchedLayer(const ROCKApi::Hand hand)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) return 0;
        bool isLeft = getIsLeftForHandEnum(hand);
        const auto& h = isLeft ? pi->getLeftHand() : pi->getRightHand();
        return h.getLastTouchedLayer();
    }

    // =====================================================================
    // Grab state
    // =====================================================================

    bool ROCK_CALL apiIsHandHolding(const ROCKApi::Hand hand)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) return false;
        bool isLeft = getIsLeftForHandEnum(hand);
        const auto& h = isLeft ? pi->getLeftHand() : pi->getRightHand();
        return h.isHolding();
    }

    RE::TESObjectREFR* ROCK_CALL apiGetHeldObject(const ROCKApi::Hand hand)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) return nullptr;
        bool isLeft = getIsLeftForHandEnum(hand);
        const auto& h = isLeft ? pi->getLeftHand() : pi->getRightHand();
        if (!h.isHolding()) return nullptr;
        return h.getHeldRef();
    }

    RE::TESObjectREFR* ROCK_CALL apiGetSelectedObject(const ROCKApi::Hand hand)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) return nullptr;
        bool isLeft = getIsLeftForHandEnum(hand);
        const auto& h = isLeft ? pi->getLeftHand() : pi->getRightHand();
        if (!h.hasSelection()) return nullptr;
        return h.getSelection().refr;
    }

    // =====================================================================
    // Hand enable/disable
    // =====================================================================

    void ROCK_CALL apiDisablePhysicsHand(const ROCKApi::Hand hand)
    {
        bool isLeft = getIsLeftForHandEnum(hand);
        if (isLeft) {
            PhysicsInteraction::s_leftHandDisabled.store(true, std::memory_order_release);
        } else {
            PhysicsInteraction::s_rightHandDisabled.store(true, std::memory_order_release);
        }
    }

    void ROCK_CALL apiEnablePhysicsHand(const ROCKApi::Hand hand)
    {
        bool isLeft = getIsLeftForHandEnum(hand);
        if (isLeft) {
            PhysicsInteraction::s_leftHandDisabled.store(false, std::memory_order_release);
        } else {
            PhysicsInteraction::s_rightHandDisabled.store(false, std::memory_order_release);
        }
    }

    bool ROCK_CALL apiIsPhysicsHandDisabled(const ROCKApi::Hand hand)
    {
        bool isLeft = getIsLeftForHandEnum(hand);
        if (isLeft) {
            return PhysicsInteraction::s_leftHandDisabled.load(std::memory_order_acquire);
        } else {
            return PhysicsInteraction::s_rightHandDisabled.load(std::memory_order_acquire);
        }
    }

    // =====================================================================
    // Object ownership
    // =====================================================================

    bool ROCK_CALL apiClaimPhysicsObject(RE::TESObjectREFR* refr)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized() || !refr) return false;
        if (pi->physicsModOwnsObject(refr)) return false;  // already claimed
        pi->claimObject(refr);
        return true;
    }

    bool ROCK_CALL apiReleasePhysicsObject(RE::TESObjectREFR* refr)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized() || !refr) return false;
        if (!pi->physicsModOwnsObject(refr)) return false;  // not claimed
        pi->releaseObject(refr);
        return true;
    }

    bool ROCK_CALL apiIsPhysicsObjectClaimed(RE::TESObjectREFR* refr)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized() || !refr) return false;
        return pi->physicsModOwnsObject(refr);
    }

    // =====================================================================
    // Force actions
    // =====================================================================

    void ROCK_CALL apiForceDropObject(const ROCKApi::Hand hand)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) return;
        bool isLeft = getIsLeftForHandEnum(hand);
        pi->forceDropHeldObject(isLeft);
    }

    // =====================================================================
    // API function table — constexpr struct with all function pointers
    // =====================================================================

    constexpr ROCKApi ROCK_API_FUNCTIONS_TABLE{
        .getVersion = &apiGetVersion,
        .getModVersion = &apiGetModVersion,
        .isPhysicsInteractionReady = &apiIsPhysicsInteractionReady,
        .getPalmPosition = &apiGetPalmPosition,
        .getPalmForward = &apiGetPalmForward,
        .isHandTouching = &apiIsHandTouching,
        .getLastTouchedObject = &apiGetLastTouchedObject,
        .getLastTouchedLayer = &apiGetLastTouchedLayer,
        .isHandHolding = &apiIsHandHolding,
        .getHeldObject = &apiGetHeldObject,
        .getSelectedObject = &apiGetSelectedObject,
        .disablePhysicsHand = &apiDisablePhysicsHand,
        .enablePhysicsHand = &apiEnablePhysicsHand,
        .isPhysicsHandDisabled = &apiIsPhysicsHandDisabled,
        .claimPhysicsObject = &apiClaimPhysicsObject,
        .releasePhysicsObject = &apiReleasePhysicsObject,
        .isPhysicsObjectClaimed = &apiIsPhysicsObjectClaimed,
        .forceDropObject = &apiForceDropObject,
    };
}

// =====================================================================
// Exported entry points
// =====================================================================

namespace rock::api
{
    ROCK_API const ROCKApi* ROCK_CALL ROCKAPI_GetApi()
    {
        return &ROCK_API_FUNCTIONS_TABLE;
    }

    /// Called by ROCKMain.cpp after creating the PhysicsInteraction instance.
    /// Must be called before any API function that queries physics state.
    void setPhysicsInteractionInstance(frik::rock::PhysicsInteraction* pi)
    {
        s_physicsInteraction.store(pi, std::memory_order_release);
    }
}
