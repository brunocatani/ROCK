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
#include "physics-interaction/grab/GrabLocomotionAuthorityBridge.h"
#include "physics-interaction/grenade/LooseGrenadeRuntime.h"
#include "physics-interaction/contact/SoftContactRuntime.h"
#include "physics-interaction/contact/GeneratedBodyContactRegistry.h"
#include "physics-interaction/contact/NativeContactEvidence.h"
#include "physics-interaction/collision/ContactActivityTracker.h"
#include "physics-interaction/consume/MouthConsumeDetector.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/core/PhysicsLifecycleState.h"
#include "physics-interaction/feedback/FeedbackHaptics.h"
#include "physics-interaction/input/GrabInputIntentPolicy.h"
#include "physics-interaction/native/PhysicsStepDriveCoordinator.h"
#include "physics-interaction/stash/ShoulderStashDetector.h"
#include "physics-interaction/weapon/EquippedWeaponDropMomentum.h"
#include "physics-interaction/weapon/EquippedWeaponDropPolicy.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/weapon/WeaponDebug.h"
#include "physics-interaction/weapon/WeaponPartDriveSandbox.h"
#include "physics-interaction/weapon/WeaponPartMotionLearner.h"
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
        void requestWeaponCollisionRebuildAfterWorkbenchExit(const char* sourceMenuName);
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
        std::uint32_t getProviderWeaponEvidenceDetailCountV1() const;
        std::uint32_t copyProviderWeaponEvidenceDetailsV1(
            ::rock::provider::RockProviderWeaponEvidenceDetailV1* outDetails,
            std::uint32_t maxDetails) const;
        std::uint32_t getProviderWeaponEvidenceDetailPointCountV1(std::uint32_t bodyId) const;
        std::uint32_t copyProviderWeaponEvidenceDetailPointsV1(
            std::uint32_t bodyId,
            ::rock::provider::RockProviderPoint3* outPoints,
            std::uint32_t maxPoints) const;
        std::uint32_t copyProviderBodyContacts(
            ::rock::provider::RockProviderBodyContactV1* outContacts,
            std::uint32_t maxContacts) const;
        bool queryProviderEquippedWeaponClassificationV1(::rock::provider::RockProviderWeaponClassificationV1& outResult) const;
        void fillProviderWeaponPartGripStates(
            std::array<::rock::provider::RockProviderWeaponPartGripStateV1, 2>& outStates) const;

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
        void clearGeneratedBodyContactRegistry();
        void refreshGeneratedBodyContactRegistry();
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

        void updateNativePlayerCollisionSuppression(RE::bhkWorld* bhk, RE::hknpWorld* hknp);

        void restoreNativePlayerCollisionSuppression(RE::hknpWorld* hknp, const char* reason);

        void refreshNativePlayerCollisionSuppression(RE::hknpWorld* hknp, const char* context);

        bool shouldSuppressNativePlayerCollisionBody(RE::bhkWorld* bhk, RE::hknpWorld* hknp, std::uint32_t bodyId) const;

        void driveGeneratedCollidersFromPhysicsSubstep(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        void driveCustomGrabAuthorityFromBetweenStep(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        void observeCustomGrabAuthorityAfterSolve(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);

        static void onGeneratedColliderPhysicsSubstep(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        static void onCustomGrabAuthorityBetweenStep(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        static void onCustomGrabAuthorityAfterSolve(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);

        void updateSelection(const PhysicsFrameContext& frame);
        GrabReleaseContext makeGrabReleaseContext(const Hand& hand, bool isLeft) const;
        GrabSharedObjectContext makeGrabSharedObjectContext(const Hand& hand, bool isLeft) const;

        void updateGrabInput(const PhysicsFrameContext& frame);
        void processProviderInteractionCommands(const PhysicsFrameContext& frame);
        void servicePendingLooseGrenadeEquip(const PhysicsFrameContext& frame);
        void updateEquippedWeaponReleaseCapture(const PhysicsFrameContext& frame, RE::NiNode* weaponNode);
        void armEquippedWeaponDropMomentumHandoff(
            const RE::ObjectRefHandle& handle,
            std::uint32_t droppedFormId,
            equipped_weapon_drop_policy::SourceHand sourceHand);
        void serviceEquippedWeaponDropMomentumHandoff(const PhysicsFrameContext& frame);
        bool armHeldLooseGrenade(Hand& hand, const PhysicsFrameContext& frame);
        void updateLooseGrenadeFuses(const PhysicsFrameContext& frame);
        void clearLooseGrenadeRuntimeState();

        std::size_t applyProviderWeaponPartDrives(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            const PhysicsFrameContext& frame,
            std::array<const RE::NiAVObject*, ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1>& outDrivenSourceNodes);

        void restoreExpiredProviderWeaponPartDriveNodes(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey);

        void refreshDrivePartCache(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey);
        void observeWeaponPartMotion(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey);
        void updateWeaponClipHarvestWalk(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey);
        void drainWeaponClipHarvest(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey);
        void updateWeaponPartDriveSandbox(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey, const PhysicsFrameContext& frame);

        grab_locomotion_authority_bridge::Output updateGrabLocomotionAuthorityBridge(float deltaSeconds, bool worldReady);

        HeldObjectPlayerSpaceFrame sampleHeldObjectPlayerSpaceFrame(float deltaSeconds);

        void applyHeldPlayerSpaceVelocity(RE::hknpWorld* hknp);
        void updateHeldMassMovementSlowdown(RE::hknpWorld* hknp, float deltaSeconds);
        void restoreHeldMassMovementSlowdown(const char* reason);

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
        void clearRightWeaponContact();

        void suppressRightHandCollisionForDominantWeapon(RE::hknpWorld* world);

        void restoreRightHandCollisionAfterDominantWeapon(RE::hknpWorld* world);

        void suppressHandCollisionForWeaponSupport(RE::hknpWorld* world, bool isLeft);

        void restoreHandCollisionAfterWeaponSupport(RE::hknpWorld* world, bool isLeft);

        void suppressHandCollisionAfterEquippedWeaponDrop(
            RE::hknpWorld* world,
            equipped_weapon_drop_policy::SourceHand sourceHand);

        void restoreHandCollisionAfterEquippedWeaponDrop(RE::hknpWorld* world, bool isLeft);

        void updateEquippedWeaponPostDropCollisionSuppression(RE::hknpWorld* world, float deltaSeconds);

        void clearEquippedWeaponPostDropCollisionSuppressionState();

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
        void updateFeedbackHaptics(float deltaSeconds);
        void pruneHeldImpactHapticCooldowns();

        static void onContactCallback(void* userData, void** worldPtrHolder, void* contactEventData);
        static void onContactCallbackSeh(void* userData, void** worldPtrHolder, void* contactEventData);
        static void onContactCallbackUnsafe(void* userData, void** worldPtrHolder, void* contactEventData);
        static void onContactCallbackException();

        void handleContactEvent(RE::hknpWorld* world, void* contactEventData);
        bool isHandContactEvidenceSuppressed(bool isLeft) const;
        void clearContactEvidenceForHand(bool isLeft, const char* reason);
        void synchronizeContactEvidenceOwnership(bool rightHandWeaponAuthorityActive, bool leftSupportGripActive, bool rightPartGripActive);

        std::atomic<bool> _initialized{ false };
        bool _collisionLayerRegistered = false;
        std::uint64_t _expectedHandLayerMask = 0;
        std::uint64_t _expectedWeaponLayerMask = 0;
        std::uint64_t _expectedReloadLayerMask = 0;
        std::uint64_t _expectedBodyLayerMask = 0;
        std::uint64_t _originalNativeCharacterControllerLayerMask = 0;
        std::uint64_t _expectedNativeCharacterControllerLayerMask = 0;
        bool _nativeCharacterControllerLayerPolicyCaptured = false;
        bool _nativeCharacterControllerLayerPolicyEnabled = false;
        HandBoneCache _handBoneCache;
        HandFrameResolver _handFrameResolver;

        Hand _rightHand{ false };
        Hand _leftHand{ true };

        BodyBoneColliderSet _bodyBoneColliders;

        WeaponCollision _weaponCollision;

        PhysicsStepDriveCoordinator _generatedBodyStepDrive;

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

        static constexpr std::size_t kGeneratedBodyContactRegistryCapacity =
            (hand_collider_semantics::kHandColliderBodyCountPerHand * 2u) +
            MAX_WEAPON_COLLISION_BODIES +
            kBodyBoneColliderBodyCount;
        generated_body_contact_registry::Registry<kGeneratedBodyContactRegistryCapacity> _generatedBodyContactRegistry;

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
        // Dedicated stash detector states for the equipped-weapon carry gesture so
        // dwell/hysteresis never mixes with a loose object held by the same hand.
        std::array<shoulder_stash::RuntimeState, 2> _equippedWeaponStashStates{};
        std::array<mouth_consume::RuntimeState, 2> _mouthConsumeStates{};
        feedback_haptics::FeedbackHaptics _feedbackHaptics;

        static constexpr std::uint32_t INVALID_CONTACT_BODY_ID = 0x7FFF'FFFF;
        static constexpr std::uint32_t WEAPON_CONTACT_TIMEOUT_FRAMES = 5;
        struct HeldWeaponAutoEquipState
        {
            std::uint32_t formID{ 0 };
            std::uint32_t bodyId{ INVALID_CONTACT_BODY_ID };
            float settledSeconds{ 0.0f };
        };
        struct PendingLooseGrenadeGrabState
        {
            bool active{ false };
            std::uint64_t requestId{ 0 };
            RE::ObjectRefHandle handle{};
            loose_grenade_runtime::GrenadeRuntimeData runtime{};
            float elapsedSeconds{ 0.0f };
        };
        struct ArmedLooseGrenadeFuseState
        {
            bool active{ false };
            RE::ObjectRefHandle handle{};
            std::uint32_t refFormID{ 0 };
            loose_grenade_runtime::GrenadeRuntimeData runtime{};
            float remainingSeconds{ 0.0f };
        };
        static constexpr std::size_t kArmedLooseGrenadeFuseCapacity = 4;
        PendingLooseGrenadeGrabState _pendingLooseGrenadeGrab{};
        std::array<ArmedLooseGrenadeFuseState, kArmedLooseGrenadeFuseCapacity> _armedLooseGrenadeFuses{};
        /*
         * Release capture for manually carried equipped weapons: the last
         * ROCK-visible weapon pose (captured one frame ahead of the release,
         * because the release transition restores the node to the FRIK hand
         * baseline before the drop request is consumed) plus per-hand motion
         * histories for drop momentum. Index 0 = right hand, 1 = left hand.
         */
        struct EquippedWeaponReleaseCapture
        {
            bool hasWeaponWorld{ false };
            RE::NiTransform weaponWorld{};
            std::array<equipped_weapon_drop_momentum::HandMotionHistory<RE::NiPoint3>, 2> handHistories{};
            std::array<bool, 2> hasPreviousHandWorld{};
            std::array<RE::NiTransform, 2> previousHandWorld{};
        };
        /*
         * Deferred momentum application for a dropped equipped weapon: the
         * spawned ref's 3D and physics bodies load asynchronously, so the
         * captured release velocity is applied on the first frame the body
         * set resolves and abandoned fail-closed on timeout.
         */
        struct EquippedWeaponDropMomentumHandoff
        {
            bool active{ false };
            RE::ObjectRefHandle handle{};
            std::uint32_t droppedFormId{ 0 };
            float elapsedSeconds{ 0.0f };
            RE::NiPoint3 linearVelocityHavok{};
            RE::NiPoint3 angularVelocityRadiansPerSecond{};
        };
        EquippedWeaponReleaseCapture _equippedWeaponReleaseCapture{};
        EquippedWeaponDropMomentumHandoff _equippedWeaponDropMomentumHandoff{};
        std::atomic<std::uint32_t> _leftWeaponContactBodyId{ INVALID_CONTACT_BODY_ID };
        std::atomic<std::uint32_t> _leftWeaponContactPartKind{ static_cast<std::uint32_t>(WeaponPartKind::Other) };
        std::atomic<std::uint32_t> _leftWeaponContactReloadRole{ static_cast<std::uint32_t>(WeaponReloadRole::None) };
        std::atomic<std::uint32_t> _leftWeaponContactSupportRole{ static_cast<std::uint32_t>(WeaponSupportGripRole::None) };
        std::atomic<std::uint32_t> _leftWeaponContactSocketRole{ static_cast<std::uint32_t>(WeaponSocketRole::None) };
        std::atomic<std::uint32_t> _leftWeaponContactActionRole{ static_cast<std::uint32_t>(WeaponActionRole::None) };
        std::atomic<std::uint32_t> _leftWeaponContactGripPose{ static_cast<std::uint32_t>(WeaponGripPoseId::None) };
        std::atomic<std::uint32_t> _leftWeaponContactSequence{ 0 };
        std::atomic<std::uint32_t> _leftWeaponContactMissedFrames{ WEAPON_CONTACT_TIMEOUT_FRAMES + 1 };
        std::atomic<std::uint32_t> _rightWeaponContactBodyId{ INVALID_CONTACT_BODY_ID };
        std::atomic<std::uint32_t> _rightWeaponContactPartKind{ static_cast<std::uint32_t>(WeaponPartKind::Other) };
        std::atomic<std::uint32_t> _rightWeaponContactReloadRole{ static_cast<std::uint32_t>(WeaponReloadRole::None) };
        std::atomic<std::uint32_t> _rightWeaponContactSupportRole{ static_cast<std::uint32_t>(WeaponSupportGripRole::None) };
        std::atomic<std::uint32_t> _rightWeaponContactSocketRole{ static_cast<std::uint32_t>(WeaponSocketRole::None) };
        std::atomic<std::uint32_t> _rightWeaponContactActionRole{ static_cast<std::uint32_t>(WeaponActionRole::None) };
        std::atomic<std::uint32_t> _rightWeaponContactGripPose{ static_cast<std::uint32_t>(WeaponGripPoseId::None) };
        std::atomic<std::uint32_t> _rightWeaponContactSequence{ 0 };
        std::atomic<std::uint32_t> _rightWeaponContactMissedFrames{ WEAPON_CONTACT_TIMEOUT_FRAMES + 1 };
        int _weaponInteractionProbeLogCounter = 0;
        std::atomic<bool> _rightDominantWeaponCollisionSuppressed{ false };
        std::atomic<bool> _leftWeaponSupportCollisionSuppressed{ false };
        std::atomic<bool> _rightWeaponSupportCollisionSuppressed{ false };
        std::atomic<bool> _rightEquippedWeaponDropCollisionSuppressed{ false };
        std::atomic<bool> _leftEquippedWeaponDropCollisionSuppressed{ false };
        hand_collision_suppression_math::SuppressionSet<hand_collider_semantics::kHandColliderBodyCountPerHand> _rightDominantWeaponCollisionSuppression{};
        hand_collision_suppression_math::SuppressionSet<hand_collider_semantics::kHandColliderBodyCountPerHand> _leftWeaponSupportCollisionSuppression{};
        hand_collision_suppression_math::SuppressionSet<hand_collider_semantics::kHandColliderBodyCountPerHand> _rightWeaponSupportCollisionSuppression{};
        hand_collision_suppression_math::SuppressionSet<kGrabCollisionSuppressionBodyCountPerHand> _rightEquippedWeaponDropCollisionSuppression{};
        hand_collision_suppression_math::SuppressionSet<kGrabCollisionSuppressionBodyCountPerHand> _leftEquippedWeaponDropCollisionSuppression{};
        hand_collision_suppression_math::DelayedRestoreState _rightEquippedWeaponDropDelayedRestore{};
        hand_collision_suppression_math::DelayedRestoreState _leftEquippedWeaponDropDelayedRestore{};
        weapon_debug_notification_policy::WeaponNotificationState _weaponDebugNotificationState{};
        bool _pendingEquippedWeaponPrimaryOnlyGripStart = false;
        /*
         * Single-consumption snapshot of the firing hand's grab button. The
         * equipped-weapon manual ownership path consumes the raw edges once per
         * frame; the normal grab pipeline must reuse this snapshot instead of
         * re-reading, or it sees cleared press/release edges.
         */
        struct SharedGrabButtonFrameState
        {
            bool valid{ false };
            bool held{ false };
            bool pressed{ false };
            bool released{ false };
        };
        SharedGrabButtonFrameState _rightGrabButtonFrameState{};
        struct ProviderWeaponPartDriveNodeState
        {
            RE::NiAVObject* node{ nullptr };
            RE::NiTransform baselineLocal{};
            bool activeThisFrame{ false };
        };
        std::array<ProviderWeaponPartDriveNodeState, ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1> _providerWeaponPartDriveNodeStates{};
        std::uint64_t _providerWeaponPartDriveGenerationKey{ 0 };

        /*
         * Bolt-drive sandbox (rockBoltDriveSandboxEnabled): per-generation
         * cache of drive-eligible parts (weaponPartDriveSandboxEligible) so
         * the per-frame learner/sandbox path never touches the heap-allocating
         * evidence descriptor copies. Nodes are non-owning engine pointers
         * valid only while the cached generation key matches the current
         * weapon generation.
         */
        struct DrivePartCacheEntry
        {
            std::uint32_t bodyId{ 0x7FFF'FFFFu };
            RE::NiAVObject* node{ nullptr };
            std::array<char, 64> sourceName{};
        };
        struct DrivePartCache
        {
            std::uint64_t generationKey{ 0 };
            std::uint32_t count{ 0 };
            // Sized to the learner's recorder capacity: eligibility spans the
            // whole feed chain (mag + bullets + casings + sockets), so an
            // AK-class weapon exposes well over a dozen eligible parts at once.
            std::array<DrivePartCacheEntry, WeaponPartMotionLearner::kMaxActiveRecorders> entries{};
        };
        DrivePartCache _drivePartCache{};
        WeaponPartMotionLearner _weaponPartMotionLearner;
        WeaponPartDriveSandbox _weaponPartDriveSandbox;
        bool _weaponPartDriveSandboxWasEnabled{ false };
        // Weapon the clip-harvest queue is currently attributed to; a form
        // change drops pending strokes so a previous weapon's clips cannot
        // attach to the new weapon through shared rig-bone names.
        std::uint32_t _lastClipHarvestWeaponFormId{ 0 };
        // At-equip weapon-graph walk bookkeeping: one walk per weapon
        // generation, started only after the collider evidence snapshot
        // commits, with a bounded retry window while the weapon graph's
        // bindings finish loading.
        std::uint64_t _clipHarvestWalkGenerationKey{ 0 };
        std::uint32_t _clipHarvestWalkAttempts{ 0 };
        bool _clipHarvestWalkCompleted{ false };
        // Whether the equipped weapon's graph holder was located at least
        // once this generation; separates "holder missing" from "bindings
        // never resolved" in the give-up diagnostics.
        bool _clipHarvestWalkHolderSeen{ false };
        // One-shot guard for the walking-candidate chain dump per generation.
        bool _clipHarvestWalkCandidateLogged{ false };
        // Give-up is terminal for the generation; completion is not — clip
        // payloads stream in only while playing, so completed walks re-run
        // periodically (a re-walk pass in flight keeps stepping each frame).
        bool _clipHarvestWalkGaveUp{ false };
        bool _clipHarvestRewalkActive{ false };
        std::uint32_t _clipHarvestRewalkCooldownFrames{ 0 };
        static constexpr std::size_t kNativePlayerCollisionSuppressionBodyCapacity = 64;
        std::array<std::uint32_t, kNativePlayerCollisionSuppressionBodyCapacity> _nativePlayerCollisionSuppressedBodyIds{};
        std::uint32_t _nativePlayerCollisionSuppressedBodyCount = 0;
        std::uint32_t _nativePlayerCollisionSuppressionRefreshFrames = 0;
        bool _nativePlayerCollisionSuppressionOverflowLogged = false;

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
        std::atomic<std::uint64_t> _palmClockGameFrameIndex{ 0 };
        std::atomic<float> _palmClockGameDeltaSeconds{ 1.0f / 90.0f };
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
        struct ProviderHandInputSuppressionRuntimeState
        {
            bool deferredGrabRelease = false;
        };

        std::array<ProviderHandInputSuppressionRuntimeState, 2> _providerHandInputSuppressionStates{};
        std::array<grab_input_intent_policy::RuntimeState, 2> _grabInputIntentStates{};
        std::array<peer_held_join_retry_policy::RuntimeState, 2> _peerHeldJoinRetryStates{};
        std::array<HeldWeaponAutoEquipState, 2> _heldWeaponAutoEquipStates{};

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
        grab_locomotion_authority_bridge::State _grabLocomotionAuthorityBridge{};
        int _grabLocomotionAuthorityLogCounter = 0;
        float _heldMassMovementSpeedReduction = 0.0f;
        float _heldMassMovementFadeStartReduction = 0.0f;
        float _heldMassMovementFadeElapsedSeconds = 0.0f;
        int _heldMassMovementLogCounter = 0;

        int _wpnNodeLogCounter = 0;
    };
}
