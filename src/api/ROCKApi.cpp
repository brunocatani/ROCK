

#define ROCK_API_EXPORTS
#include "ROCKApi.h"

#include <atomic>
#include <cstddef>

#include "FRIKApi.h"
#include "physics-interaction/grab/GrabEvent.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/core/PhysicsInteraction.h"

static_assert(sizeof(rock::PhysicsEventData) == sizeof(rock::api::ROCKApi::PhysicsEventData), "Internal and API PhysicsEventData must have identical size");
static_assert(offsetof(rock::PhysicsEventData, isLeft) == offsetof(rock::api::ROCKApi::PhysicsEventData, isLeft), "PhysicsEventData::isLeft offset mismatch");
static_assert(offsetof(rock::PhysicsEventData, refr) == offsetof(rock::api::ROCKApi::PhysicsEventData, refr), "PhysicsEventData::refr offset mismatch");
static_assert(offsetof(rock::PhysicsEventData, formID) == offsetof(rock::api::ROCKApi::PhysicsEventData, formID), "PhysicsEventData::formID offset mismatch");
static_assert(offsetof(rock::PhysicsEventData, collisionLayer) == offsetof(rock::api::ROCKApi::PhysicsEventData, collisionLayer),
    "PhysicsEventData::collisionLayer offset mismatch");

static_assert(sizeof(rock::PhysicsEventData) == 24, "PhysicsEventData unexpected size — check struct packing");
static_assert(sizeof(rock::api::ROCKApi::PhysicsEventData) == 24, "ROCKApi::PhysicsEventData unexpected size — check struct packing");
static_assert(sizeof(rock::GrabEventData) == sizeof(rock::api::ROCKApi::GrabEventData), "Internal and API GrabEventData must have identical size");
static_assert(offsetof(rock::GrabEventData, type) == offsetof(rock::api::ROCKApi::GrabEventData, type), "GrabEventData::type offset mismatch");
static_assert(offsetof(rock::GrabEventData, refr) == offsetof(rock::api::ROCKApi::GrabEventData, refr), "GrabEventData::refr offset mismatch");
static_assert(offsetof(rock::GrabEventData, speedGameUnitsPerSecond) == offsetof(rock::api::ROCKApi::GrabEventData, speedGameUnitsPerSecond),
    "GrabEventData::speed offset mismatch");

#define ROCK_ASSERT_GRAB_EVENT_OFFSET(FIELD)                                                                                                                       \
    static_assert(offsetof(rock::GrabEventData, FIELD) == offsetof(rock::api::ROCKApi::GrabEventData, FIELD), "GrabEventData::" #FIELD " offset mismatch")

ROCK_ASSERT_GRAB_EVENT_OFFSET(size);
ROCK_ASSERT_GRAB_EVENT_OFFSET(version);
ROCK_ASSERT_GRAB_EVENT_OFFSET(type);
ROCK_ASSERT_GRAB_EVENT_OFFSET(sourceKind);
ROCK_ASSERT_GRAB_EVENT_OFFSET(isLeft);
ROCK_ASSERT_GRAB_EVENT_OFFSET(reservedBool);
ROCK_ASSERT_GRAB_EVENT_OFFSET(refr);
ROCK_ASSERT_GRAB_EVENT_OFFSET(formID);
ROCK_ASSERT_GRAB_EVENT_OFFSET(primaryBodyId);
ROCK_ASSERT_GRAB_EVENT_OFFSET(secondaryBodyId);
ROCK_ASSERT_GRAB_EVENT_OFFSET(collisionLayer);
ROCK_ASSERT_GRAB_EVENT_OFFSET(flags);
ROCK_ASSERT_GRAB_EVENT_OFFSET(frameIndex);
ROCK_ASSERT_GRAB_EVENT_OFFSET(positionGame);
ROCK_ASSERT_GRAB_EVENT_OFFSET(velocityGame);
ROCK_ASSERT_GRAB_EVENT_OFFSET(mass);
ROCK_ASSERT_GRAB_EVENT_OFFSET(speedGameUnitsPerSecond);
ROCK_ASSERT_GRAB_EVENT_OFFSET(intensityHint);
ROCK_ASSERT_GRAB_EVENT_OFFSET(reservedFloat);

#undef ROCK_ASSERT_GRAB_EVENT_OFFSET

static_assert(static_cast<int>(rock::api::ROCKApi::Hand::Primary) == static_cast<int>(frik::api::FRIKApi::Hand::Primary), "Hand::Primary mismatch");
static_assert(static_cast<int>(rock::api::ROCKApi::Hand::Offhand) == static_cast<int>(frik::api::FRIKApi::Hand::Offhand), "Hand::Offhand mismatch");
static_assert(static_cast<int>(rock::api::ROCKApi::Hand::Right) == static_cast<int>(frik::api::FRIKApi::Hand::Right), "Hand::Right mismatch");
static_assert(static_cast<int>(rock::api::ROCKApi::Hand::Left) == static_cast<int>(frik::api::FRIKApi::Hand::Left), "Hand::Left mismatch");

#define ROCK_ASSERT_API_ENUM(API_ENUM, INTERNAL_ENUM, VALUE)                                                                                                      \
    static_assert(static_cast<int>(rock::api::ROCKApi::API_ENUM::VALUE) == static_cast<int>(rock::INTERNAL_ENUM::VALUE), #API_ENUM "::" #VALUE " mismatch")

ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, Unknown);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, SelectionLocked);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, PullStarted);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, PullArrived);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, PullCatchAttempt);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, PullCatchSucceeded);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, GrabCommitted);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, HeldImpact);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, Released);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, TwoHandStarted);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, TwoHandStopped);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, StashCandidate);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, ConsumeCandidate);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, Stashed);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, Consumed);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, LootStarted);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, LootCompleted);
ROCK_ASSERT_API_ENUM(GrabEventType, GrabEventType, SelectionUnlocked);

ROCK_ASSERT_API_ENUM(GrabEventSourceKind, GrabEventSourceKind, Unknown);
ROCK_ASSERT_API_ENUM(GrabEventSourceKind, GrabEventSourceKind, Hand);
ROCK_ASSERT_API_ENUM(GrabEventSourceKind, GrabEventSourceKind, HeldObject);
ROCK_ASSERT_API_ENUM(GrabEventSourceKind, GrabEventSourceKind, PulledObject);
ROCK_ASSERT_API_ENUM(GrabEventSourceKind, GrabEventSourceKind, Weapon);
ROCK_ASSERT_API_ENUM(GrabEventSourceKind, GrabEventSourceKind, External);

ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Receiver);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Barrel);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Handguard);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Foregrip);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Pump);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Stock);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Grip);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Magazine);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Magwell);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Bolt);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Slide);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, ChargingHandle);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, BreakAction);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Cylinder);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Chamber);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Shell);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Round);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, LaserCell);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Lever);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Sight);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Accessory);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, CosmeticAmmo);
ROCK_ASSERT_API_ENUM(WeaponPartKind, WeaponPartKind, Other);

#undef ROCK_ASSERT_API_ENUM

static_assert(rock::api::ROCKApi::kGrabEventFlagHeldImpactDamped == rock::ROCK_GRAB_EVENT_FLAG_HELD_IMPACT_DAMPED,
    "Grab event held-impact flag mismatch");
static_assert(rock::api::ROCKApi::kGrabEventFlagSuppressHaptic == rock::ROCK_GRAB_EVENT_FLAG_SUPPRESS_HAPTIC,
    "Grab event suppress-haptic flag mismatch");
static_assert(rock::api::ROCKApi::kGrabEventFlagPositionValid == rock::ROCK_GRAB_EVENT_FLAG_POSITION_VALID,
    "Grab event position-valid flag mismatch");
static_assert(rock::api::ROCKApi::kGrabEventFlagVelocityValid == rock::ROCK_GRAB_EVENT_FLAG_VELOCITY_VALID,
    "Grab event velocity-valid flag mismatch");
static_assert(rock::api::ROCKApi::kGrabEventFlagMassValid == rock::ROCK_GRAB_EVENT_FLAG_MASS_VALID,
    "Grab event mass-valid flag mismatch");
static_assert(rock::api::ROCKApi::kGrabEventFlagSpeedValid == rock::ROCK_GRAB_EVENT_FLAG_SPEED_VALID,
    "Grab event speed-valid flag mismatch");
static_assert(rock::api::ROCKApi::kGrabEventFlagIntensityValid == rock::ROCK_GRAB_EVENT_FLAG_INTENSITY_VALID,
    "Grab event intensity-valid flag mismatch");

namespace
{
    using namespace rock::api;
    using namespace rock;

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
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized())
            return RE::NiPoint3{};

        bool isLeft = getIsLeftForHandEnum(hand);
        RE::NiTransform transform{};
        if (!pi->tryGetRootFlattenedHandTransform(isLeft, transform))
            return RE::NiPoint3{};

        return computeGrabLegacyPalmPivotAWorldFromHandBasis(transform, isLeft);
    }

    RE::NiPoint3 ROCK_CALL apiGetPalmForward(const ROCKApi::Hand hand)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized())
            return RE::NiPoint3{};

        bool isLeft = getIsLeftForHandEnum(hand);
        RE::NiTransform transform{};
        if (!pi->tryGetRootFlattenedHandTransform(isLeft, transform))
            return RE::NiPoint3{};

        return computePalmNormalFromHandBasis(transform, isLeft);
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
        if (!pi->physicsModOwnsObject(refr, PhysicsObjectClaimOwner::External))
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

    std::uint32_t ROCK_CALL apiGetLastTouchedWeaponPartKind()
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized())
            return static_cast<std::uint32_t>(ROCKApi::WeaponPartKind::Other);
        return pi->getLastTouchedWeaponPartKind();
    }

    std::uint32_t ROCK_CALL apiLegacyUintSlot0()
    {
        return 0;
    }

    std::uint32_t ROCK_CALL apiLegacyUintSlot1()
    {
        return 0;
    }

    std::uint32_t ROCK_CALL apiLegacyUintSlot2()
    {
        return 0;
    }

    bool ROCK_CALL apiLegacyBoolSlotWithUint(std::uint32_t)
    {
        return false;
    }

    bool ROCK_CALL apiLegacyBoolSlot0()
    {
        return false;
    }

    bool ROCK_CALL apiLegacyBoolSlot1()
    {
        return false;
    }

    /*
    ROCK keeps the removed PAPER-owned API entries as inert trailing table slots. This preserves
    the binary function-table shape already consumed by sibling plugins without reintroducing
    feature ownership or public header surface in ROCK.
    */
    struct ROCKApiPrefixWithLegacyTail
    {
        std::uint32_t(ROCK_CALL* getVersion)();
        const char*(ROCK_CALL* getModVersion)();
        bool(ROCK_CALL* isPhysicsInteractionReady)();
        RE::NiPoint3(ROCK_CALL* getPalmPosition)(ROCKApi::Hand hand);
        RE::NiPoint3(ROCK_CALL* getPalmForward)(ROCKApi::Hand hand);
        bool(ROCK_CALL* isHandTouching)(ROCKApi::Hand hand);
        RE::TESObjectREFR*(ROCK_CALL* getLastTouchedObject)(ROCKApi::Hand hand);
        std::uint32_t(ROCK_CALL* getLastTouchedLayer)(ROCKApi::Hand hand);
        bool(ROCK_CALL* isHandHolding)(ROCKApi::Hand hand);
        RE::TESObjectREFR*(ROCK_CALL* getHeldObject)(ROCKApi::Hand hand);
        RE::TESObjectREFR*(ROCK_CALL* getSelectedObject)(ROCKApi::Hand hand);
        void(ROCK_CALL* disablePhysicsHand)(ROCKApi::Hand hand);
        void(ROCK_CALL* enablePhysicsHand)(ROCKApi::Hand hand);
        bool(ROCK_CALL* isPhysicsHandDisabled)(ROCKApi::Hand hand);
        bool(ROCK_CALL* claimPhysicsObject)(RE::TESObjectREFR* refr);
        bool(ROCK_CALL* releasePhysicsObject)(RE::TESObjectREFR* refr);
        bool(ROCK_CALL* isPhysicsObjectClaimed)(RE::TESObjectREFR* refr);
        void(ROCK_CALL* forceDropObject)(ROCKApi::Hand hand);
        std::uint32_t(ROCK_CALL* getLastTouchedWeaponPartKind)();
        std::uint32_t(ROCK_CALL* legacyUintSlot0)();
        std::uint32_t(ROCK_CALL* legacyUintSlot1)();
        std::uint32_t(ROCK_CALL* legacyUintSlot2)();
        bool(ROCK_CALL* legacyBoolSlotWithUint)(std::uint32_t value);
        bool(ROCK_CALL* legacyBoolSlot0)();
        bool(ROCK_CALL* legacyBoolSlot1)();
    };

    static_assert(offsetof(ROCKApiPrefixWithLegacyTail, getLastTouchedWeaponPartKind) == offsetof(ROCKApi, getLastTouchedWeaponPartKind),
        "ROCKApi public prefix must stay binary-compatible with the exported legacy-tail table");
    static_assert(sizeof(ROCKApiPrefixWithLegacyTail) > sizeof(ROCKApi), "ROCKApi export must retain inert legacy tail slots");

    constexpr ROCKApiPrefixWithLegacyTail ROCK_API_FUNCTIONS_TABLE{
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
        .getLastTouchedWeaponPartKind = &apiGetLastTouchedWeaponPartKind,
        .legacyUintSlot0 = &apiLegacyUintSlot0,
        .legacyUintSlot1 = &apiLegacyUintSlot1,
        .legacyUintSlot2 = &apiLegacyUintSlot2,
        .legacyBoolSlotWithUint = &apiLegacyBoolSlotWithUint,
        .legacyBoolSlot0 = &apiLegacyBoolSlot0,
        .legacyBoolSlot1 = &apiLegacyBoolSlot1,
    };
}

namespace rock::api
{
    ROCK_API const ROCKApi* ROCK_CALL ROCKAPI_GetApi() { return reinterpret_cast<const ROCKApi*>(&ROCK_API_FUNCTIONS_TABLE); }

    void setPhysicsInteractionInstance(rock::PhysicsInteraction* pi) { s_physicsInteraction.store(pi, std::memory_order_release); }
}
