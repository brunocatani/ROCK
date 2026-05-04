#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/collision/ContactActivityTracker.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/native/PhysicsStepDriveCoordinator.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/weapon/WeaponDebug.h"
#include "api/ROCKProviderApi.h"

namespace RE
{
    class bhkWorld;
    class hknpWorld;
    class TESAmmo;
    class TESObjectREFR;
}

namespace rock
{

    enum PhysicsMessageType : std::uint32_t
    {
        kPhysMsg_OnTouch = 100,
        kPhysMsg_OnTouchEnd = 101,
        kPhysMsg_OnGrab = 102,
        kPhysMsg_OnRelease = 103,
        kPhysMsg_OnPhysicsInit = 104,
        kPhysMsg_OnPhysicsShutdown = 105,
    };

    struct PhysicsEventData
    {
        bool isLeft;
        RE::TESObjectREFR* refr;
        std::uint32_t formID;
        std::uint32_t collisionLayer;
    };

    class PhysicsInteraction
    {
    public:
        static inline std::atomic<PhysicsInteraction*> s_instance{ nullptr };

        static inline std::atomic<bool> s_hooksEnabled{ false };

        static inline std::atomic<bool> s_rightHandDisabled{ false };
        static inline std::atomic<bool> s_leftHandDisabled{ false };

        PhysicsInteraction();
        ~PhysicsInteraction();

        void init();

        void update();

        void shutdown();

        bool isInitialized() const { return _initialized; }

        bool physicsModOwnsObject(RE::TESObjectREFR* ref) const;
        void claimObject(RE::TESObjectREFR* ref);
        void releaseObject(RE::TESObjectREFR* ref);
        void releaseAllObjects();

        void forceDropHeldObject(bool isLeft);

        Hand& getRightHand() { return _rightHand; }
        Hand& getLeftHand() { return _leftHand; }
        const Hand& getRightHand() const { return _rightHand; }
        const Hand& getLeftHand() const { return _leftHand; }

        std::uint32_t getLastTouchedWeaponPartKind() const
        {
            if (_leftWeaponContactMissedFrames.load(std::memory_order_acquire) > WEAPON_CONTACT_TIMEOUT_FRAMES) {
                return static_cast<std::uint32_t>(WeaponPartKind::Other);
            }
            return _leftWeaponContactPartKind.load(std::memory_order_acquire);
        }
        bool tryGetRootFlattenedHandTransform(bool isLeft, RE::NiTransform& outTransform) const;
        void fillProviderFrameSnapshot(::rock::provider::RockProviderFrameSnapshot& outSnapshot) const;
        bool queryProviderWeaponContactAtPoint(
            const ::rock::provider::RockProviderWeaponContactQuery& query,
            ::rock::provider::RockProviderWeaponContactResult& outResult) const;
        std::uint32_t copyProviderWeaponEvidenceDescriptors(
            ::rock::provider::RockProviderWeaponEvidenceDescriptor* outDescriptors,
            std::uint32_t maxDescriptors) const;
        std::uint32_t getProviderWeaponEvidenceDetailCountV3() const;
        std::uint32_t copyProviderWeaponEvidenceDetailsV3(
            ::rock::provider::RockProviderWeaponEvidenceDetailV3* outDetails,
            std::uint32_t maxDetails) const;
        std::uint32_t getProviderWeaponEvidenceDetailPointCountV3(std::uint32_t bodyId) const;
        std::uint32_t copyProviderWeaponEvidenceDetailPointsV3(
            std::uint32_t bodyId,
            ::rock::provider::RockProviderPoint3* outPoints,
            std::uint32_t maxPoints) const;

    private:
        bool validateCriticalOffsets() const;

        bool refreshHandBoneCache();

        void sampleHandTransformParity();

        RE::NiTransform getInteractionHandTransform(bool isLeft) const;

        RE::NiNode* getInteractionHandNode(bool isLeft) const;

        RE::bhkWorld* getPlayerBhkWorld() const;

        static RE::hknpWorld* getHknpWorld(RE::bhkWorld* bhk);

        PhysicsFrameContext buildFrameContext(RE::bhkWorld* bhk, RE::hknpWorld* hknp, float deltaSeconds);

        void registerCollisionLayer(RE::hknpWorld* world);

        bool createHandCollisions(RE::hknpWorld* world, void* bhkWorld);

        void destroyHandCollisions(void* bhkWorld);

        void updateHandCollisions(const PhysicsFrameContext& frame);

        void driveGeneratedBodiesFromPhysicsStep(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);

        static void onGeneratedBodyPhysicsStep(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);

        void updateSelection(const PhysicsFrameContext& frame);

        void updateGrabInput(const PhysicsFrameContext& frame);

        HeldObjectPlayerSpaceFrame sampleHeldObjectPlayerSpaceFrame(float deltaSeconds);

        void applyHeldPlayerSpaceVelocity(RE::hknpWorld* hknp);

        void resolveContacts(const PhysicsFrameContext& frame);

        void resolveAndLogContact(const char* handName, RE::bhkWorld* bhk, RE::hknpWorld* hknp, RE::hknpBodyId bodyId);

        void applyDynamicPushAssist(const char* sourceName,
            RE::bhkWorld* bhk,
            RE::hknpWorld* hknp,
            std::uint32_t sourceBodyId,
            std::uint32_t targetBodyId,
            bool sourceIsWeapon,
            const Hand* sourceHand = nullptr);

        void publishDebugBodyOverlay(const PhysicsFrameContext& frame);

        void clearLeftWeaponContact();

        void suppressLeftHandCollisionForWeaponSupport(RE::hknpWorld* world);

        void restoreLeftHandCollisionAfterWeaponSupport(RE::hknpWorld* world);

        void subscribeContactEvents(RE::hknpWorld* world);
        void unsubscribeContactEvents(RE::hknpWorld* liveWorld);

        void dispatchPhysicsMessage(std::uint32_t msgType, bool isLeft, RE::TESObjectREFR* refr = nullptr, std::uint32_t formID = 0, std::uint32_t layer = 0);

        static void onContactCallback(void* userData, void** worldPtrHolder, void* contactEventData);

        void handleContactEvent(RE::hknpWorld* world, void* contactEventData);

        std::atomic<bool> _initialized{ false };
        bool _collisionLayerRegistered = false;
        std::uint64_t _expectedHandLayerMask = 0;
        std::uint64_t _expectedWeaponLayerMask = 0;
        std::uint64_t _expectedReloadLayerMask = 0;
        HandBoneCache _handBoneCache;
        HandFrameResolver _handFrameResolver;

        Hand _rightHand{ false };
        Hand _leftHand{ true };

        WeaponCollision _weaponCollision;

        PhysicsStepDriveCoordinator _generatedBodyStepDrive;

        TwoHandedGrip _twoHandedGrip;

        mutable std::mutex _ownedObjectsMutex;
        std::unordered_set<std::uint32_t> _ownedObjects;

        RE::bhkWorld* _cachedBhkWorld = nullptr;

        float _deltaTime = 1.0f / 90.0f;

        std::atomic<int> _contactLogCounter{ 0 };
        std::atomic<RE::hknpWorld*> _contactEventWorld{ nullptr };
        std::atomic<void*> _contactEventSignal{ nullptr };
        contact_activity_tracker::ContactActivityTracker _handContactActivity;

        std::atomic<std::uint32_t> _lastContactSourceRight{ 0xFFFFFFFF };
        std::atomic<std::uint32_t> _lastContactSourceLeft{ 0xFFFFFFFF };
        std::atomic<std::uint32_t> _lastContactBodyRight{ 0xFFFFFFFF };
        std::atomic<std::uint32_t> _lastContactBodyLeft{ 0xFFFFFFFF };
        std::atomic<std::uint32_t> _lastContactBodyWeapon{ 0xFFFFFFFF };
        std::atomic<std::uint32_t> _lastContactSourceWeapon{ 0xFFFFFFFF };
        float _dynamicPushElapsedSeconds = 0.0f;
        std::unordered_map<std::uint64_t, float> _dynamicPushCooldownUntil;

        static constexpr std::uint32_t INVALID_CONTACT_BODY_ID = 0x7FFF'FFFF;
        static constexpr std::uint32_t WEAPON_CONTACT_TIMEOUT_FRAMES = 5;
        std::atomic<std::uint32_t> _leftWeaponContactBodyId{ INVALID_CONTACT_BODY_ID };
        std::atomic<std::uint32_t> _leftWeaponContactPartKind{ static_cast<std::uint32_t>(WeaponPartKind::Other) };
        std::atomic<std::uint32_t> _leftWeaponContactReloadRole{ static_cast<std::uint32_t>(WeaponReloadRole::None) };
        std::atomic<std::uint32_t> _leftWeaponContactSupportRole{ static_cast<std::uint32_t>(WeaponSupportGripRole::None) };
        std::atomic<std::uint32_t> _leftWeaponContactSocketRole{ static_cast<std::uint32_t>(WeaponSocketRole::None) };
        std::atomic<std::uint32_t> _leftWeaponContactActionRole{ static_cast<std::uint32_t>(WeaponActionRole::None) };
        std::atomic<std::uint32_t> _leftWeaponContactGripPose{ static_cast<std::uint32_t>(WeaponGripPoseId::None) };
        std::atomic<std::uint32_t> _leftWeaponContactSequence{ 0 };
        std::atomic<std::uint32_t> _leftWeaponContactMissedFrames{ WEAPON_CONTACT_TIMEOUT_FRAMES + 1 };
        int _weaponInteractionProbeLogCounter = 0;
        hand_collision_suppression_math::SuppressionSet<hand_collider_semantics::kHandColliderBodyCountPerHand> _leftWeaponSupportCollisionSuppression{};
        weapon_debug_notification_policy::WeaponNotificationState _weaponDebugNotificationState{};

        int _handCacheResolveLogCounter = 0;

        struct RawHandParityState
        {
            RE::NiTransform previousApiTransform{};
            float lastPositionDelta = 0.0f;
            float lastRotationDeltaDegrees = 0.0f;
            int warnFrames = 0;
            int failFrames = 0;
            int lagFrames = 0;
            bool hasPreviousApiTransform = false;
        };

        std::array<RawHandParityState, 2> _rawHandParityStates{};
        int _paritySummaryCounter = 0;
        bool _parityEnabledLogged = false;
        bool _runtimeScaleLogged = false;
        struct GrabTransformTelemetryState
        {
            bool active = false;
            std::uint32_t session = 0;
            std::uint64_t frame = 0;
            std::uint64_t logFrameCounter = 0;
        };

        std::array<GrabTransformTelemetryState, 2> _grabTransformTelemetryStates{};
        std::uint32_t _grabTransformTelemetryNextSession = 1;

        RE::NiPoint3 _prevSmoothedPos;
        int _deltaLogCounter = 0;
        bool _hasPrevPositions = false;
        RE::NiPoint3 _prevHeldPlayerSpacePosition{};
        RE::NiTransform _prevHeldPlayerSpaceTransform{};
        HeldObjectPlayerSpaceFrame _heldObjectPlayerSpaceFrame{};
        bool _hasHeldPlayerSpacePosition = false;
        bool _hasHeldPlayerSpaceTransform = false;
        RE::NiPoint3 _lastCentralHeldPlayerSpaceVelocityHavok{};
        int _heldPlayerSpaceLogCounter = 0;

        int _wpnNodeLogCounter = 0;
    };
}
