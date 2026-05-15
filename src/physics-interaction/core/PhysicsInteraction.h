#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/body/BodyContactRuntime.h"
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/grab/GrabEvent.h"
#include "physics-interaction/contact/SoftContactRuntime.h"
#include "physics-interaction/contact/NativeContactEvidence.h"
#include "physics-interaction/collision/ContactActivityTracker.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/core/PhysicsLifecycleState.h"
#include "physics-interaction/input/GrabInputIntentPolicy.h"
#include "physics-interaction/native/PhysicsStepDriveCoordinator.h"
#include "physics-interaction/native/GrabAuthorityPhase0Probe.h"
#include "physics-interaction/stash/ShoulderStashDetector.h"
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
        kPhysMsg_OnGrabEvent = 200,
    };

    enum class PhysicsObjectClaimOwner : std::uint8_t
    {
        External = 0,
        RightHand = 1,
        LeftHand = 2,
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

        PhysicsInteraction(std::uint32_t skeletonGeneration = 1, std::uint32_t providerGeneration = 1);
        ~PhysicsInteraction();

        void init();

        void update();

        void shutdown(::rock::provider::RockProviderLifecycleReason reason = ::rock::provider::RockProviderLifecycleReason::Shutdown);

        bool isInitialized() const { return _initialized; }
        void noteSkeletonLifecycle(std::uint32_t skeletonGeneration, ::rock::provider::RockProviderLifecycleReason reason);
        void noteProviderLifecycle(std::uint32_t providerGeneration, ::rock::provider::RockProviderLifecycleReason reason);

        bool physicsModOwnsObject(RE::TESObjectREFR* ref) const;
        bool physicsModOwnsObject(RE::TESObjectREFR* ref, PhysicsObjectClaimOwner owner) const;
        void claimObject(RE::TESObjectREFR* ref, PhysicsObjectClaimOwner owner = PhysicsObjectClaimOwner::External);
        void releaseObject(RE::TESObjectREFR* ref, PhysicsObjectClaimOwner owner = PhysicsObjectClaimOwner::External);
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
        std::uint32_t copyProviderBodyContactsV6(
            ::rock::provider::RockProviderBodyContactV6* outContacts,
            std::uint32_t maxContacts) const;

    private:
        bool validateCriticalOffsets() const;

        bool refreshHandBoneCache();

        void sampleHandTransformParity();

        RE::NiTransform getInteractionHandTransform(bool isLeft) const;

        RE::NiNode* getInteractionHandNode(bool isLeft) const;

        RE::bhkWorld* getPlayerBhkWorld() const;

        static RE::hknpWorld* getHknpWorld(RE::bhkWorld* bhk);

        PhysicsFrameContext buildFrameContext(RE::bhkWorld* bhk, RE::hknpWorld* hknp, float deltaSeconds);

        bool generatedBodiesExistForConfig() const;
        bool generatedBodiesMatchLifecycle(RE::bhkWorld* bhk, RE::hknpWorld* hknp) const;
        void markGeneratedBodiesRebuilt(RE::bhkWorld* bhk, RE::hknpWorld* hknp);
        void markGeneratedBodiesInvalidated();
        bool rebuildGeneratedBodiesForLifecycle(RE::bhkWorld* bhk, RE::hknpWorld* hknp, const char* reason);
        void observeLifecycleFrame(RE::bhkWorld* bhk, RE::hknpWorld* hknp, ::rock::provider::RockProviderLifecycleReason reasonHint);
        bool physicsWritesAllowedForWorld(RE::hknpWorld* world) const;

        void registerCollisionLayer(RE::hknpWorld* world);

        bool createHandCollisions(RE::hknpWorld* world, void* bhkWorld);

        void destroyHandCollisions(void* bhkWorld);

        void updateHandCollisions(const PhysicsFrameContext& frame);

        bool createBodyBoneCollisions(RE::hknpWorld* world, void* bhkWorld);

        void destroyBodyBoneCollisions(void* bhkWorld);

        void updateBodyBoneCollisions(const PhysicsFrameContext& frame);

        void driveGeneratedCollidersFromPhysicsSubstep(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        void driveGrabAuthorityPhase0ProbeFromBetweenStep(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        void observeGrabAuthorityPhase0ProbeAfterSolve(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);

        static void onGeneratedColliderPhysicsSubstep(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        static void onGrabAuthorityPhase0BetweenStep(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        static void onGrabAuthorityPhase0AfterSolve(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);

        void updateSelection(const PhysicsFrameContext& frame);
        GrabReleaseContext makeGrabReleaseContext(const Hand& hand, bool isLeft) const;
        GrabSharedObjectContext makeGrabSharedObjectContext(const Hand& hand, bool isLeft) const;

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

        void suppressRightHandCollisionForDominantWeapon(RE::hknpWorld* world);

        void restoreRightHandCollisionAfterDominantWeapon(RE::hknpWorld* world);

        void suppressLeftHandCollisionForWeaponSupport(RE::hknpWorld* world);

        void restoreLeftHandCollisionAfterWeaponSupport(RE::hknpWorld* world);

        void subscribeContactEvents(RE::hknpWorld* world);
        void unsubscribeContactEvents(RE::hknpWorld* liveWorld);

        void dispatchPhysicsMessage(std::uint32_t msgType, bool isLeft, RE::TESObjectREFR* refr = nullptr, std::uint32_t formID = 0, std::uint32_t layer = 0);
        void dispatchGrabEvent(GrabEventData eventData);
        void dispatchSimpleGrabEvent(
            GrabEventType type,
            bool isLeft,
            RE::TESObjectREFR* refr,
            std::uint32_t primaryBodyId = ROCK_GRAB_EVENT_INVALID_BODY_ID,
            std::uint32_t flags = 0);
        void dispatchGrabCommittedEvent(bool isLeft, RE::TESObjectREFR* refr, std::uint32_t primaryBodyId, RE::hknpWorld* world);
        void dispatchHeldImpactGrabEvent(bool isLeft, RE::TESObjectREFR* refr, std::uint32_t heldBodyId, std::uint32_t otherBodyId, float mass, float speedGameUnitsPerSecond);
        void handleGrabEventHaptics(const GrabEventData& eventData);
        void pruneHeldImpactHapticCooldowns();

        static void onContactCallback(void* userData, void** worldPtrHolder, void* contactEventData);
        static void onContactCallbackSeh(void* userData, void** worldPtrHolder, void* contactEventData);
        static void onContactCallbackUnsafe(void* userData, void** worldPtrHolder, void* contactEventData);
        static void onContactCallbackException();

        void handleContactEvent(RE::hknpWorld* world, void* contactEventData);
        bool isHandContactEvidenceSuppressed(bool isLeft) const;
        void clearContactEvidenceForHand(bool isLeft, const char* reason);
        void synchronizeContactEvidenceOwnership(bool rightHandWeaponEquipped, bool leftSupportGripActive);

        std::atomic<bool> _initialized{ false };
        bool _collisionLayerRegistered = false;
        std::uint64_t _expectedHandLayerMask = 0;
        std::uint64_t _expectedWeaponLayerMask = 0;
        std::uint64_t _expectedReloadLayerMask = 0;
        std::uint64_t _expectedBodyLayerMask = 0;
        HandBoneCache _handBoneCache;
        HandFrameResolver _handFrameResolver;

        Hand _rightHand{ false };
        Hand _leftHand{ true };

        BodyBoneColliderSet _bodyBoneColliders;

        WeaponCollision _weaponCollision;

        PhysicsStepDriveCoordinator _generatedBodyStepDrive;
        grab_authority_phase0::Probe _grabAuthorityPhase0Probe;

        TwoHandedGrip _twoHandedGrip;
        SoftContactRuntime _softContactRuntime;
        contact_evidence::NativeContactEvidenceCache _nativeContactEvidence;

        mutable std::mutex _ownedObjectsMutex;
        std::unordered_map<std::uint32_t, std::uint32_t> _ownedObjects;

        RE::bhkWorld* _cachedBhkWorld = nullptr;
        RE::hknpWorld* _cachedHknpWorld = nullptr;
        RE::bhkWorld* _generatedBodiesBhkWorld = nullptr;
        RE::hknpWorld* _generatedBodiesHknpWorld = nullptr;
        std::uint32_t _generatedBodiesWorldGeneration = 0;
        std::uint32_t _generatedBodiesSkeletonGeneration = 0;
        std::uint32_t _generatedBodiesProviderGeneration = 0;
        physics_lifecycle::RuntimeState _lifecycleState{};
        std::atomic<std::uint32_t> _lifecycleFlagsAtomic{ 0 };
        std::atomic<std::uint32_t> _lastLifecycleReasonAtomic{ static_cast<std::uint32_t>(::rock::provider::RockProviderLifecycleReason::None) };
        std::atomic<std::uint32_t> _worldGenerationAtomic{ 1 };
        std::atomic<std::uint32_t> _skeletonGenerationAtomic{ 1 };
        std::atomic<std::uint32_t> _providerGenerationAtomic{ 1 };
        std::atomic<std::uint32_t> _stableFrameCountAtomic{ 0 };
        std::atomic<RE::hknpWorld*> _lifecycleHknpWorldAtomic{ nullptr };
        int _handColliderCreateRetryFrames = 0;
        int _bodyBoneColliderCreateRetryFrames = 0;

        float _deltaTime = 1.0f / 90.0f;

        std::atomic<int> _contactLogCounter{ 0 };
        std::atomic<RE::hknpWorld*> _contactEventWorld{ nullptr };
        std::atomic<void*> _contactEventSignal{ nullptr };
        contact_activity_tracker::ContactActivityTracker _handContactActivity;
        body_contact_runtime::BodyContactRuntime _bodyContactRuntime;

        std::atomic<std::uint32_t> _lastContactSourceRight{ 0xFFFFFFFF };
        std::atomic<std::uint32_t> _lastContactSourceLeft{ 0xFFFFFFFF };
        std::atomic<std::uint32_t> _lastContactBodyRight{ 0xFFFFFFFF };
        std::atomic<std::uint32_t> _lastContactBodyLeft{ 0xFFFFFFFF };
        std::atomic<std::uint32_t> _lastContactBodyWeapon{ 0xFFFFFFFF };
        std::atomic<std::uint32_t> _lastContactSourceWeapon{ 0xFFFFFFFF };
        static constexpr std::uint64_t INVALID_HELD_IMPACT_PAIR = 0xFFFF'FFFF'FFFF'FFFFull;
        std::atomic<std::uint64_t> _lastHeldImpactPairRight{ INVALID_HELD_IMPACT_PAIR };
        std::atomic<std::uint64_t> _lastHeldImpactPairLeft{ INVALID_HELD_IMPACT_PAIR };
        float _dynamicPushElapsedSeconds = 0.0f;
        std::unordered_map<std::uint64_t, float> _dynamicPushCooldownUntil;
        std::unordered_map<std::uint64_t, float> _heldImpactHapticCooldownUntil;
        std::uint64_t _grabEventFrameCounter = 0;
        std::array<shoulder_stash::RuntimeState, 2> _shoulderStashStates{};

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
        std::atomic<bool> _rightDominantWeaponCollisionSuppressed{ false };
        std::atomic<bool> _leftWeaponSupportCollisionSuppressed{ false };
        hand_collision_suppression_math::SuppressionSet<hand_collider_semantics::kHandColliderBodyCountPerHand> _rightDominantWeaponCollisionSuppression{};
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
            bool hasPreviousAngularDeltaSample = false;
            RE::NiTransform previousRawHandWorld{};
            RE::NiTransform previousPalmAnchorGrabAuthorityWorld{};
            RE::NiTransform previousProxyReadbackWorld{};
            RE::NiTransform previousRawDesiredObjectWorld{};
            RE::NiTransform previousConstraintDesiredObjectWorld{};
            RE::NiTransform previousHeldNodeWorld{};
            RE::NiTransform previousHeldBodyWorld{};
            RE::NiTransform previousNativeBodyWorld{};
            bool previousHasPalmAnchorGrabAuthority = false;
            bool previousHasProxyReadback = false;
            bool previousHasHeldNodeWorld = false;
            bool previousHasHeldBodyWorld = false;
            bool previousHasHeldNativeBodyWorld = false;
        };

        std::array<GrabTransformTelemetryState, 2> _grabTransformTelemetryStates{};
        std::uint32_t _grabTransformTelemetryNextSession = 1;
        std::array<grab_input_intent_policy::RuntimeState, 2> _grabInputIntentStates{};

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
        std::uint64_t _heldPlayerSpaceDebugFrameCounter = 0;

        int _wpnNodeLogCounter = 0;
    };
}
