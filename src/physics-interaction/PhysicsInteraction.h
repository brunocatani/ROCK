#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <unordered_set>

#include "Hand.h"
#include "HandBoneCache.h"
#include "HandFrameResolver.h"
#include "PhysicsLog.h"
#include "TwoHandedGrip.h"
#include "WeaponCollision.h"

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

        void resolveContacts(RE::bhkWorld* bhk, RE::hknpWorld* hknp);

        void resolveAndLogContact(const char* handName, RE::bhkWorld* bhk, RE::hknpWorld* hknp, RE::hknpBodyId bodyId);

        void publishDebugBodyOverlay(RE::hknpWorld* hknp);

        void subscribeContactEvents(RE::hknpWorld* world);

        void dispatchPhysicsMessage(std::uint32_t msgType, bool isLeft, RE::TESObjectREFR* refr = nullptr, std::uint32_t formID = 0, std::uint32_t layer = 0);

        static void onContactCallback(void* userData, void** worldPtrHolder, void* contactEventData);

        void handleContactEvent(void* contactEventData);

        std::atomic<bool> _initialized{ false };
        bool _collisionLayerRegistered = false;
        std::uint64_t _expectedHandLayerMask = 0;
        HandBoneCache _handBoneCache;
        HandFrameResolver _handFrameResolver;

        Hand _rightHand{ false };
        Hand _leftHand{ true };

        WeaponCollision _weaponCollision;

        TwoHandedGrip _twoHandedGrip;

        mutable std::mutex _ownedObjectsMutex;
        std::unordered_set<std::uint32_t> _ownedObjects;

        RE::bhkWorld* _cachedBhkWorld = nullptr;

        float _deltaTime = 1.0f / 90.0f;

        std::atomic<int> _contactLogCounter{ 0 };

        std::atomic<std::uint32_t> _lastContactBodyRight{ 0xFFFFFFFF };
        std::atomic<std::uint32_t> _lastContactBodyLeft{ 0xFFFFFFFF };

        std::atomic<bool> _offhandTouchingWeapon{ false };

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

        int _wpnNodeLogCounter = 0;
    };
}
