

#define ROCK_API_EXPORTS
#include "ROCKApi.h"

#include <atomic>

#include "FRIKApi.h"
#include "physics-interaction/PalmTransform.h"
#include "physics-interaction/PhysicsInteraction.h"

static_assert(sizeof(frik::rock::PhysicsEventData) == sizeof(rock::api::ROCKApi::PhysicsEventData), "Internal and API PhysicsEventData must have identical size");
static_assert(offsetof(frik::rock::PhysicsEventData, isLeft) == offsetof(rock::api::ROCKApi::PhysicsEventData, isLeft), "PhysicsEventData::isLeft offset mismatch");
static_assert(offsetof(frik::rock::PhysicsEventData, refr) == offsetof(rock::api::ROCKApi::PhysicsEventData, refr), "PhysicsEventData::refr offset mismatch");
static_assert(offsetof(frik::rock::PhysicsEventData, formID) == offsetof(rock::api::ROCKApi::PhysicsEventData, formID), "PhysicsEventData::formID offset mismatch");
static_assert(offsetof(frik::rock::PhysicsEventData, collisionLayer) == offsetof(rock::api::ROCKApi::PhysicsEventData, collisionLayer),
    "PhysicsEventData::collisionLayer offset mismatch");

static_assert(sizeof(frik::rock::PhysicsEventData) == 24, "PhysicsEventData unexpected size — check struct packing");
static_assert(sizeof(rock::api::ROCKApi::PhysicsEventData) == 24, "ROCKApi::PhysicsEventData unexpected size — check struct packing");

static_assert(static_cast<int>(rock::api::ROCKApi::Hand::Primary) == static_cast<int>(frik::api::FRIKApi::Hand::Primary), "Hand::Primary mismatch");
static_assert(static_cast<int>(rock::api::ROCKApi::Hand::Offhand) == static_cast<int>(frik::api::FRIKApi::Hand::Offhand), "Hand::Offhand mismatch");
static_assert(static_cast<int>(rock::api::ROCKApi::Hand::Right) == static_cast<int>(frik::api::FRIKApi::Hand::Right), "Hand::Right mismatch");
static_assert(static_cast<int>(rock::api::ROCKApi::Hand::Left) == static_cast<int>(frik::api::FRIKApi::Hand::Left), "Hand::Left mismatch");

namespace
{
    using namespace rock::api;
    using namespace frik::rock;

    std::atomic<PhysicsInteraction*> s_physicsInteraction{ nullptr };

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

    std::uint32_t ROCK_CALL apiGetVersion() { return ROCK_API_VERSION; }

    const char* ROCK_CALL apiGetModVersion()
    {
        static constexpr const char* version = "0.1.0";
        return version;
    }

    bool ROCK_CALL apiIsPhysicsInteractionReady()
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        return pi && pi->isInitialized();
    }

    RE::NiPoint3 ROCK_CALL apiGetPalmPosition(const ROCKApi::Hand hand)
    {
        const auto* frikApi = frik::api::FRIKApi::inst;
        if (!frikApi)
            return RE::NiPoint3{};

        auto frikHand = static_cast<frik::api::FRIKApi::Hand>(static_cast<std::uint8_t>(hand));
        auto transform = frikApi->getHandWorldTransform(frikHand);

        bool isLeft = getIsLeftForHandEnum(hand);
        return computePalmPosition(transform, isLeft);
    }

    RE::NiPoint3 ROCK_CALL apiGetPalmForward(const ROCKApi::Hand hand)
    {
        const auto* frikApi = frik::api::FRIKApi::inst;
        if (!frikApi)
            return RE::NiPoint3{};

        auto frikHand = static_cast<frik::api::FRIKApi::Hand>(static_cast<std::uint8_t>(hand));
        auto transform = frikApi->getHandWorldTransform(frikHand);

        bool isLeft = getIsLeftForHandEnum(hand);
        return computePalmForward(transform, isLeft);
    }

    bool ROCK_CALL apiIsHandTouching(const ROCKApi::Hand hand)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized())
            return false;
        bool isLeft = getIsLeftForHandEnum(hand);
        const auto& h = isLeft ? pi->getLeftHand() : pi->getRightHand();
        return h.isTouching();
    }

    RE::TESObjectREFR* ROCK_CALL apiGetLastTouchedObject(const ROCKApi::Hand hand)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized())
            return nullptr;
        bool isLeft = getIsLeftForHandEnum(hand);
        const auto& h = isLeft ? pi->getLeftHand() : pi->getRightHand();
        return h.getLastTouchedRef();
    }

    std::uint32_t ROCK_CALL apiGetLastTouchedLayer(const ROCKApi::Hand hand)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized())
            return 0;
        bool isLeft = getIsLeftForHandEnum(hand);
        const auto& h = isLeft ? pi->getLeftHand() : pi->getRightHand();
        return h.getLastTouchedLayer();
    }

    bool ROCK_CALL apiIsHandHolding(const ROCKApi::Hand hand)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized())
            return false;
        bool isLeft = getIsLeftForHandEnum(hand);
        const auto& h = isLeft ? pi->getLeftHand() : pi->getRightHand();
        return h.isHolding();
    }

    RE::TESObjectREFR* ROCK_CALL apiGetHeldObject(const ROCKApi::Hand hand)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized())
            return nullptr;
        bool isLeft = getIsLeftForHandEnum(hand);
        const auto& h = isLeft ? pi->getLeftHand() : pi->getRightHand();
        if (!h.isHolding())
            return nullptr;
        return h.getHeldRef();
    }

    RE::TESObjectREFR* ROCK_CALL apiGetSelectedObject(const ROCKApi::Hand hand)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized())
            return nullptr;
        bool isLeft = getIsLeftForHandEnum(hand);
        const auto& h = isLeft ? pi->getLeftHand() : pi->getRightHand();
        if (!h.hasSelection())
            return nullptr;
        return h.getSelection().refr;
    }

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

    bool ROCK_CALL apiClaimPhysicsObject(RE::TESObjectREFR* refr)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized() || !refr)
            return false;
        if (pi->physicsModOwnsObject(refr))
            return false;
        pi->claimObject(refr);
        return true;
    }

    bool ROCK_CALL apiReleasePhysicsObject(RE::TESObjectREFR* refr)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized() || !refr)
            return false;
        if (!pi->physicsModOwnsObject(refr))
            return false;
        pi->releaseObject(refr);
        return true;
    }

    bool ROCK_CALL apiIsPhysicsObjectClaimed(RE::TESObjectREFR* refr)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized() || !refr)
            return false;
        return pi->physicsModOwnsObject(refr);
    }

    void ROCK_CALL apiForceDropObject(const ROCKApi::Hand hand)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized())
            return;
        bool isLeft = getIsLeftForHandEnum(hand);
        pi->forceDropHeldObject(isLeft);
    }

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

namespace rock::api
{
    ROCK_API const ROCKApi* ROCK_CALL ROCKAPI_GetApi() { return &ROCK_API_FUNCTIONS_TABLE; }

    void setPhysicsInteractionInstance(frik::rock::PhysicsInteraction* pi) { s_physicsInteraction.store(pi, std::memory_order_release); }
}
