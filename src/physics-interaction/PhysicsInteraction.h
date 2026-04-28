#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

#include "Hand.h"
#include "HandBoneCache.h"
#include "HandCollisionSuppressionMath.h"
#include "HandFrameResolver.h"
#include "PhysicsLog.h"
#include "TwoHandedGrip.h"
#include "WeaponCollision.h"
#include "WeaponDebugNotificationPolicy.h"
#include "WeaponPrimaryGripAuthority.h"
#include "WeaponReloadEventBridge.h"
#include "WeaponReloadStageObserver.h"

namespace RE
{
    class bhkWorld;
    class hknpWorld;
    class TESObjectREFR;
}

namespace frik::rock
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
        std::uint32_t getActiveWeaponReloadState() const { return _activeWeaponReloadState.load(std::memory_order_acquire); }
        std::uint32_t getObservedWeaponReloadStage() const { return _observedWeaponReloadStage.load(std::memory_order_acquire); }
        std::uint32_t getWeaponReloadStageSource() const { return _weaponReloadStageSource.load(std::memory_order_acquire); }

    private:
        bool validateCriticalOffsets() const;

        bool refreshHandBoneCache();

        void sampleHandTransformParity();

        RE::NiTransform getInteractionHandTransform(bool isLeft) const;

        RE::NiNode* getInteractionHandNode(bool isLeft) const;

        RE::bhkWorld* getPlayerBhkWorld() const;

        static RE::hknpWorld* getHknpWorld(RE::bhkWorld* bhk);

        void registerCollisionLayer(RE::hknpWorld* world);

        bool createHandCollisions(RE::hknpWorld* world, void* bhkWorld);

        void destroyHandCollisions(void* bhkWorld);

        void updateHandCollisions(RE::hknpWorld* world);

        void updateSelection(RE::bhkWorld* bhk, RE::hknpWorld* hknp);

        void updateGrabInput(RE::hknpWorld* hknp);

        HeldObjectPlayerSpaceFrame sampleHeldObjectPlayerSpaceFrame();

        void resolveContacts(RE::bhkWorld* bhk, RE::hknpWorld* hknp);

        void resolveAndLogContact(const char* handName, RE::bhkWorld* bhk, RE::hknpWorld* hknp, RE::hknpBodyId bodyId);

        void applyDynamicPushAssist(const char* sourceName,
            RE::bhkWorld* bhk,
            RE::hknpWorld* hknp,
            std::uint32_t sourceBodyId,
            std::uint32_t targetBodyId,
            bool sourceIsWeapon);

        void publishDebugBodyOverlay(RE::hknpWorld* hknp);

        void updateWeaponReloadState(bool weaponEquipped);

        void resetWeaponReloadStateForInterruption(const char* reason);

        void clearLeftWeaponContact();

        void suppressLeftHandCollisionForWeaponSupport(RE::hknpWorld* world);

        void restoreLeftHandCollisionAfterWeaponSupport(RE::hknpWorld* world);

        void subscribeContactEvents(RE::hknpWorld* world);

        void dispatchPhysicsMessage(std::uint32_t msgType, bool isLeft, RE::TESObjectREFR* refr = nullptr, std::uint32_t formID = 0, std::uint32_t layer = 0);

        static void onContactCallback(void* userData, void** worldPtrHolder, void* contactEventData);

        void handleContactEvent(void* contactEventData);

        std::atomic<bool> _initialized{ false };
        bool _collisionLayerRegistered = false;
        std::uint64_t _expectedHandLayerMask = 0;
        std::uint64_t _expectedWeaponLayerMask = 0;
        HandBoneCache _handBoneCache;
        HandFrameResolver _handFrameResolver;

        Hand _rightHand{ false };
        Hand _leftHand{ true };

        WeaponCollision _weaponCollision;

        WeaponPrimaryGripAuthority _primaryGripAuthority;

        TwoHandedGrip _twoHandedGrip;

        WeaponReloadEventBridge _weaponReloadEventBridge;
        WeaponReloadObserverState _weaponReloadObserver;
        WeaponReloadCoordinatorState _weaponReloadCoordinator;
        WeaponReloadObserverOutput _lastWeaponReloadObservation;
        int _reloadStateLogCounter = 0;
        std::uint32_t _reloadObserverStaleFrames = 0;

        mutable std::mutex _ownedObjectsMutex;
        std::unordered_set<std::uint32_t> _ownedObjects;

        RE::bhkWorld* _cachedBhkWorld = nullptr;

        float _deltaTime = 1.0f / 90.0f;

        std::atomic<int> _contactLogCounter{ 0 };

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
        hand_collision_suppression_math::SuppressionState _leftWeaponSupportCollisionSuppression{};
        bool _leftWeaponSupportBroadPhaseSuppressed = false;
        std::atomic<std::uint32_t> _activeWeaponReloadState{ static_cast<std::uint32_t>(WeaponReloadState::Idle) };
        std::atomic<std::uint32_t> _observedWeaponReloadStage{ static_cast<std::uint32_t>(WeaponVanillaReloadStage::Idle) };
        std::atomic<std::uint32_t> _weaponReloadStageSource{ static_cast<std::uint32_t>(WeaponReloadStageSource::None) };
        weapon_debug_notification_policy::WeaponNotificationState _weaponDebugNotificationState{};

        float _cachedHalfExtentX = 0.0f;
        float _cachedHalfExtentY = 0.0f;
        float _cachedHalfExtentZ = 0.0f;
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

        RE::NiPoint3 _prevSmoothedPos;
        int _deltaLogCounter = 0;
        bool _hasPrevPositions = false;
        RE::NiPoint3 _prevHeldPlayerSpacePosition{};
        HeldObjectPlayerSpaceFrame _heldObjectPlayerSpaceFrame{};
        bool _hasHeldPlayerSpacePosition = false;
        int _heldPlayerSpaceLogCounter = 0;

        int _wpnNodeLogCounter = 0;
    };
}
