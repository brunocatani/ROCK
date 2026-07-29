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
#include "physics-interaction/grab/SavedGrabOffsetStore.h"
#include "physics-interaction/grab/TouchGrabRuntime.h"
#include "physics-interaction/grenade/LooseGrenadeRuntime.h"
#include "physics-interaction/hand/DynamicHandCollision.h"
#include "physics-interaction/contact/GeneratedBodyContactRegistry.h"
#include "physics-interaction/collision/ContactActivityTracker.h"
#include "physics-interaction/consume/MouthConsumeDetector.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/core/PendingForceGrabCommit.h"
#include "physics-interaction/core/ForceGrabPolicy.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/core/PhysicsLifecycleState.h"
#include "physics-interaction/feedback/FeedbackHaptics.h"
#include "physics-interaction/input/GrabInputIntentPolicy.h"
#include "physics-interaction/native/PhysicsStepDriveCoordinator.h"
#include "physics-interaction/stash/ShoulderStashDetector.h"
#include "physics-interaction/weapon/AuthoredPrimaryFiringGrip.h"
#include "physics-interaction/weapon/EquippedWeaponDropMomentum.h"
#include "physics-interaction/weapon/EquippedWeaponDropPolicy.h"
#include "physics-interaction/weapon/EquippedWeaponTransitionCoordinator.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/weapon/WeaponDebug.h"
#include "physics-interaction/weapon/BareFistGuardPolicy.h"
#include "api/ROCKProviderApi.h"

namespace RE
{
    class bhkWorld;
    class hknpWorld;
    class NiAVObject;
    class NiCollisionObject;
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

        void synchronizeNativeScopePresentationAfterFrikUpdate();

        bool tryResolveNativeScopeGeometryDecision(bool nativeGeometryDecision, bool& outRockGeometryDecision);

        [[nodiscard]] bool tryGetManualScopeDirectTransitionTarget(
            std::uint64_t& outWeaponGenerationKey,
            std::uint32_t& outNativeOverlayIndex) const;

        void update();

        // Observes and repairs native equipped-weapon presentation before any
        // weapon-relative ROCK authority reads the first-person graph.
        void updateEquippedWeaponTransition();

        // Runs before the normal ROCK interaction frame so weapon-relative
        // consumers see one authored primary-grip frame. Runtime eligibility
        // failures clear the tagged hand authority deterministically.
        void updateAuthoredPrimaryFiringGrip();

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
        std::uint32_t getProviderWeaponEmitterCountV1() const;
        std::uint32_t copyProviderWeaponEmittersV1(
            ::rock::provider::RockProviderWeaponEmitterV1* outEmitters,
            std::uint32_t maxEmitters) const;
        bool queryProviderWorldRaycastV1(
            const ::rock::provider::RockProviderWorldRaycastRequestV1& request,
            ::rock::provider::RockProviderWorldRaycastResultV1& outResult) const;
        std::uint32_t copyProviderBodyContacts(
            ::rock::provider::RockProviderBodyContactV1* outContacts,
            std::uint32_t maxContacts) const;
        bool queryProviderEquippedWeaponClassificationV1(::rock::provider::RockProviderWeaponClassificationV1& outResult) const;
        bool queryProviderEquippedWeaponGripStateV1(
            ::rock::provider::RockProviderEquippedWeaponGripStateV1& outState) const;
        bool queryProviderEquippedWeaponHandlingStateV1(
            ::rock::provider::RockProviderEquippedWeaponHandlingStateV1& outState) const;
        ::rock::provider::RockProviderResultV1 requestProviderEquippedWeaponHandV1(
            std::uint64_t ownerToken,
            const ::rock::provider::RockProviderEquippedWeaponHandRequestV1& request);
        void fillProviderWeaponPartGripStates(
            std::array<::rock::provider::RockProviderWeaponPartGripStateV1, 2>& outStates) const;
        void fillProviderHandInteractionStates(
            std::array<::rock::provider::RockProviderHandInteractionStateV1, 2>& outStates) const;
        bool queryProviderEquippedWeaponStateV1(
            ::rock::provider::RockProviderEquippedWeaponStateV1& outState) const;
        std::uint32_t copyProviderWeaponPartPosesV1(
            ::rock::provider::RockProviderWeaponPartPoseV1* outParts,
            std::uint32_t maxParts) const;
        std::uint32_t copyProviderWeaponPartDriveResultsV1(
            std::uint64_t ownerToken,
            ::rock::provider::RockProviderWeaponPartDriveApplicationResultV1* outResults,
            std::uint32_t maxResults) const;
        bool queryProviderScopeSightStateV1(
            ::rock::provider::RockProviderScopeSightStateV1& outState) const;
        bool queryProviderWeaponCompositionStateV1(
            ::rock::provider::RockProviderWeaponCompositionStateV1& outState) const;
        std::uint32_t copyProviderWeaponCompositionEntriesV1(
            ::rock::provider::RockProviderWeaponCompositionEntryV1* outEntries,
            std::uint32_t maxEntries) const;
        bool queryProviderSelectedAuthoredGripPoseV1(
            ::rock::provider::RockProviderAuthoredGripPoseV1& outPose) const;
        bool queryProviderPresentedHandPoseV1(
            ::rock::provider::RockProviderHand hand,
            ::rock::provider::RockProviderPresentedHandPoseV1& outPose) const;
        std::uint32_t copyProviderSemanticHandContactsV1(
            ::rock::provider::RockProviderHand hand,
            std::uint32_t maxFramesSinceContact,
            ::rock::provider::RockProviderSemanticHandContactV1* outContacts,
            std::uint32_t maxContacts) const;
        std::uint32_t copyProviderPlayerColliderDescriptorsV1(
            ::rock::provider::RockProviderPlayerColliderDescriptorV1* outDescriptors,
            std::uint32_t maxDescriptors) const;
        bool queryProviderHandCollisionAvailabilityV1(
            ::rock::provider::RockProviderHand hand,
            ::rock::provider::RockProviderHandCollisionAvailabilityV1& outState) const;

    private:
        struct EquippedWeaponDropMomentumHandoff;

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

        void refreshNativePlayerCollisionSuppressionFromPhysicsSubstep(RE::hknpWorld* hknp, const char* context);

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
        std::uint32_t forceGrabHandBlockerMask(const Hand& hand, bool isLeft, bool handDisabled, bool includePendingCommit) const;
        bool canHandAcceptForceGrab(const Hand& hand, bool isLeft, bool handDisabled) const;
        bool handHoldsLooseGrenade(const Hand& hand) const;
        bool hasActiveLooseGrenadeCommit() const;
        bool isPendingForceGrabTarget(RE::TESObjectREFR* ref) const;
        void pruneInactiveProviderForceGrabCommits();
        void servicePendingLooseGrenadeEquip(const PhysicsFrameContext& frame);
        void servicePendingForceGrabCommits(const PhysicsFrameContext& frame);
        void clearPendingForceGrabCommitsForOrigin(PendingForceGrabCommitOrigin origin);
        void updateSavedGrabOffsetGesture(const PhysicsFrameContext& frame);
        void saveGrabOffsetForHand(Hand& hand, bool isLeft, RE::hknpWorld* hknpWorld);
        void updateEquippedWeaponReleaseCapture(const PhysicsFrameContext& frame, RE::NiNode* weaponNode);
        void armEquippedWeaponDropMomentumHandoff(
            const RE::ObjectRefHandle& handle,
            std::uint32_t droppedFormId,
            equipped_weapon_drop_policy::SourceHand sourceHand,
            const WeaponCollision::ReleaseGeometrySnapshot& releaseGeometry);
        bool hasAvailableEquippedWeaponDropHandoff() const;
        void serviceEquippedWeaponDropMomentumHandoff(const PhysicsFrameContext& frame);
        void serviceEquippedWeaponDropMomentumTransaction(
            EquippedWeaponDropMomentumHandoff& handoff,
            const PhysicsFrameContext& frame);
        bool armHeldLooseGrenade(Hand& hand, const PhysicsFrameContext& frame);
        void updateLooseGrenadeFuses(const PhysicsFrameContext& frame);
        void clearLooseGrenadeImpactWatches();
        void clearLooseGrenadeRuntimeState(bool clearPendingEquipRequest);
        void enforceNoBareFistState(bool forceRecheck);

        std::size_t applyProviderWeaponPartDrives(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            const PhysicsFrameContext& frame,
            std::array<const RE::NiAVObject*, ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1>& outDrivenSourceNodes);

        void restoreExpiredProviderWeaponPartDriveNodes(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey);

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
        void logGrabOverlayPointProbe(const PhysicsFrameContext& frame);

        void clearLeftWeaponContact();
        void clearRightWeaponContact();

        void refreshEquippedWeaponHandlingSettings();
        void reconcileEquippedWeaponHandlingMode();
        void serviceFixedWeaponHand(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            bool menuInputActive);

        void serviceEquippedWeaponHandAssignment(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            bool menuInputActive,
            const EquippedWeaponHandlingSettings& handlingSettings);
        void reconcileEquippedWeaponHandAssignmentAfterGrip();
        void clearEquippedWeaponHandAssignment(const char* reason, bool clearUiAssignment);

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
        void clearContactEvidenceForHand(bool isLeft);
        void synchronizeContactEvidenceOwnership(bool rightHandWeaponAuthorityActive, bool leftSupportGripActive, bool rightPartGripActive);

        std::atomic<bool> _initialized{ false };
        bool _collisionLayerRegistered = false;
        std::uint64_t _expectedHandLayerMask = 0;
        std::uint64_t _expectedWeaponLayerMask = 0;
        std::uint64_t _expectedReloadLayerMask = 0;
        std::uint64_t _expectedBodyLayerMask = 0;
        std::uint64_t _expectedHeldObjectLayerMask = 0;
        std::uint64_t _originalNativeCharacterControllerLayerMask = 0;
        std::uint64_t _expectedNativeCharacterControllerLayerMask = 0;
        bool _nativeCharacterControllerLayerPolicyCaptured = false;
        bool _nativeCharacterControllerLayerPolicyEnabled = false;
        HandBoneCache _handBoneCache;
        HandFrameResolver _handFrameResolver;

        Hand _rightHand{ false };
        Hand _leftHand{ true };
        TouchGrabRuntime _touchGrabRuntime;

        BodyBoneColliderSet _bodyBoneColliders;

        WeaponCollision _weaponCollision;

        EquippedWeaponTransitionCoordinator _equippedWeaponTransition;

        PhysicsStepDriveCoordinator _generatedBodyStepDrive;
        // Written only by the post-solve callback and sampled by the main-frame
        // equipped-drop service. This explicit atomic is the cross-thread
        // settle barrier; PhysicsStepDriveCoordinator's internal counter is not
        // read across threads.
        std::atomic<std::uint64_t> _completedPhysicsSolveSequence{ 0 };

        TwoHandedGrip _twoHandedGrip;
        EquippedWeaponHandlingSettings _equippedWeaponHandlingSettings{};
        bool _fixedFiringHandIsLeft{ false };
        bool _equippedWeaponHandlingModeInitialized{ false };
        bool _equippedWeaponHandlingModeReconcilePending{ false };
        AuthoredPrimaryFiringGripRuntime _authoredPrimaryFiringGrip;
        DynamicHandCollisionRuntime _dynamicHandCollision;

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
        std::atomic<std::uint32_t> _collisionGenerationAtomic{ 1 };
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
        struct EquippedWeaponStashCommitLease
        {
            bool active = false;
            std::uint64_t ownershipKey = 0;
            std::uint8_t remainingOpenFrames = 0;
            body_zone::BodyZoneKind zone = body_zone::BodyZoneKind::Unknown;
            shoulder_stash::EvidenceSource source = shoulder_stash::EvidenceSource::None;
            shoulder_stash::RuntimeState spatialState{};
        };
        // The lease bridges only the release debounce after a confirmed dwell.
        // It is bound to the live equipped instance and revalidates the same
        // spatial candidate on every open-grip frame.
        std::array<EquippedWeaponStashCommitLease, 2> _equippedWeaponStashCommitLeases{};
        std::array<mouth_consume::RuntimeState, 2> _mouthConsumeStates{};
        feedback_haptics::FeedbackHaptics _feedbackHaptics;

        static constexpr std::uint32_t INVALID_CONTACT_BODY_ID = 0x7FFF'FFFF;
        static constexpr std::uint32_t WEAPON_CONTACT_TIMEOUT_FRAMES = 5;
        struct HeldWeaponTriggerEquipIntent
        {
            bool pending{ false };
            std::uint32_t formID{ 0 };
            float remainingSeconds{ 0.0f };
        };

        /*
         * Loose-to-equipped handoff state. The loose root disappears during
         * inventory transfer, so the physical hand and its weapon-local frame
         * are retained by value until the equipped node becomes observable.
         */
        struct PendingEquippedWeaponPrimaryOnlyGripStart
        {
            bool pending{ false };
            bool isLeft{ false };
            // Zero means "the current weapon" (menu reconciliation). Held
            // equip requests bind these fields to the accepted target and its
            // pre-request baseline so a cloned instance may be recognized
            // without ever starting manual ownership on an old same-base gun.
            std::uint32_t targetWeaponFormID{ 0 };
            std::uintptr_t targetWeaponInstanceData{ 0 };
            std::uint32_t previousWeaponFormID{ 0 };
            std::uintptr_t previousWeaponInstanceData{ 0 };
            float remainingSeconds{ 0.0f };
            bool hasFiringHandWeaponLocal{ false };
            RE::NiTransform firingHandWeaponLocal{};
            bool hasFiringGripWeaponLocal{ false };
            RE::NiPoint3 firingGripWeaponLocal{};
        };
        struct ArmedLooseGrenadeFuseState
        {
            bool active{ false };
            RE::ObjectRefHandle handle{};
            std::uint32_t refFormID{ 0 };
            loose_grenade_runtime::GrenadeRuntimeData runtime{};
            float remainingSeconds{ 0.0f };
            std::uint32_t impactBodyId{ INVALID_CONTACT_BODY_ID };
        };
        static constexpr std::size_t kArmedLooseGrenadeFuseCapacity = 4;
        std::array<PendingForceGrabCommit, 2> _pendingForceGrabCommits{};
        std::array<HeldWeaponTriggerEquipIntent, 2> _heldWeaponTriggerEquipIntents{};
        std::array<bool, 2> _forceGrabCommittedThisFrame{};
        bare_fist_guard_policy::RecheckState _bareFistGuardState{};
        std::array<ArmedLooseGrenadeFuseState, kArmedLooseGrenadeFuseCapacity> _armedLooseGrenadeFuses{};
        std::array<std::atomic<std::uint32_t>, kArmedLooseGrenadeFuseCapacity> _armedLooseGrenadeImpactBodyIds{};
        std::atomic<std::uint64_t> _pendingLooseGrenadeImpactPair{ INVALID_HELD_IMPACT_PAIR };
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
        enum class EquippedWeaponDropHandoffStage : std::uint8_t
        {
            ResolvingBodies,
            WaitingForSettleStep,
        };

        static constexpr std::size_t kEquippedWeaponDropBodySnapshotCapacity = 32;
        struct EquippedWeaponDropBodySnapshot
        {
            bool valid{ false };
            equipped_weapon_drop_momentum::BodyIdentityKey identity{};
        };

        /*
         * Deferred native-drop transaction. RemoveItem can publish the ref and
         * body tree asynchronously, so ROCK first resolves exact-ref bodies,
         * enables collision, places every native motion at the frozen visual
         * release pose with zero velocity, and waits for one completed native
         * solve before applying captured release momentum exactly once. After
         * that atomic handoff, Bethesda owns the weapon's normal flight.
         */
        struct EquippedWeaponDropMomentumHandoff
        {
            bool active{ false };
            bool hasReleaseVelocity{ false };
            bool referenceResolvedOnce{ false };
            bool threeDResolvedOnce{ false };
            RE::ObjectRefHandle handle{};
            std::uint32_t droppedFormId{ 0 };
            float elapsedSeconds{ 0.0f };
            std::uint32_t identityRestartCount{ 0 };
            RE::NiPoint3 linearVelocityHavok{};
            RE::NiPoint3 angularVelocityRadiansPerSecond{};
            bool hasReleaseWeaponWorld{ false };
            RE::NiTransform releaseWeaponWorld{};
            EquippedWeaponDropHandoffStage stage{ EquippedWeaponDropHandoffStage::ResolvingBodies };
            std::uint64_t progressSolveSequence{ 0 };
            std::uint64_t bodyDiscoverySolveSequence{ 0 };
            std::array<EquippedWeaponDropBodySnapshot, kEquippedWeaponDropBodySnapshotCapacity> bodySnapshots{};
            std::size_t bodySnapshotCount{ 0 };
        };
        EquippedWeaponReleaseCapture _equippedWeaponReleaseCapture{};
        static constexpr std::size_t kEquippedWeaponDropHandoffCapacity = 4;
        std::array<EquippedWeaponDropMomentumHandoff, kEquippedWeaponDropHandoffCapacity> _equippedWeaponDropMomentumHandoffs{};
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
        std::array<weapon_interaction_acquisition_policy::State, 2> _weaponInteractionAcquisitionStates{};
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
        PendingEquippedWeaponPrimaryOnlyGripStart _pendingEquippedWeaponPrimaryOnlyGripStart{};
        enum class EquippedWeaponHandAssignmentSource : std::uint8_t
        {
            None = 0,
            Pipboy = 1,
            Provider = 2,
        };
        struct EquippedWeaponHandAssignmentState
        {
            EquippedWeaponHandAssignmentSource source{
                EquippedWeaponHandAssignmentSource::None
            };
            bool pending{ false };
            bool active{ false };
            bool assignedLeft{ false };
            bool effectiveLeft{ false };
            std::uint16_t remainingResolveFrames{ 0 };
            std::uint32_t handleId{ 0 };
            std::uint32_t stackId{ 0 };
            std::uint32_t formId{ 0 };
            std::uint64_t ownerToken{ 0 };
            std::uint64_t requestedWeaponGenerationKey{ 0 };
            std::uint64_t ownershipKey{ 0 };
            std::uint64_t nativeOffsetGenerationKey{ 0 };
            bool nativeOffsetSampleValid{ false };
            bool nativeOffsetReadinessLogged{ false };
            std::uint8_t matchingNativeOffsetFrames{ 0 };
            RE::NiTransform nativeOffsetSample{};
        };
        EquippedWeaponHandAssignmentState _equippedWeaponHandAssignment{};
        std::uint64_t _lastPipboyWeaponSelectionSequence{ 0 };
        struct FixedLeftCarryState
        {
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint64_t weaponOwnershipKey{ 0 };
            RE::NiTransform nativeOffsetSample{};
            std::uint16_t remainingResolveFrames{ 0 };
            std::uint8_t matchingNativeOffsetFrames{ 0 };
            bool nativeOffsetSampleValid{ false };
            bool infrastructureWarningLogged{ false };
        };
        FixedLeftCarryState _fixedLeftCarry{};
        bool _equippedWeaponMenuReconcilePending = false;
        /*
         * Single-consumption snapshot of the firing hand's grab button. The
         * equipped-weapon manual ownership path consumes the raw edges once per
         * frame; the normal grab pipeline must reuse this snapshot instead of
         * re-reading, or it sees cleared press/release edges.
         */
        struct SharedGrabButtonFrameState
        {
            bool valid{ false };
            // Physical hand the snapshot was consumed from (the CURRENT
            // firing hand); the normal grab pipeline matches on it.
            bool isLeft{ false };
            bool held{ false };
            bool pressed{ false };
            bool released{ false };
        };
        SharedGrabButtonFrameState _firingHandGrabButtonFrameState{};
        struct ProviderWeaponPartDriveNodeState
        {
            RE::NiAVObject* node{ nullptr };
            RE::NiTransform baselineLocal{};
            std::uint64_t ownerToken{ 0 };
            std::uint32_t bodyId{ 0x7FFF'FFFFu };
            std::uint32_t groupId{ 0 };
            std::uint32_t priority{ 0 };
            std::array<char, ::rock::provider::ROCK_PROVIDER_MAX_EVIDENCE_NAME>
                sourceName{};
            bool activeThisFrame{ false };
        };
        std::array<ProviderWeaponPartDriveNodeState, ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1> _providerWeaponPartDriveNodeStates{};
        std::uint64_t _providerWeaponPartDriveGenerationKey{ 0 };
        std::array<::rock::provider::RockProviderWeaponPartDriveApplicationResultV1,
            ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVE_RESULTS_V1>
            _providerWeaponPartDriveResults{};
        std::uint32_t _providerWeaponPartDriveResultCount{ 0 };
        mutable DirectSkeletonBoneReader _providerPresentedPoseReader{};

        static constexpr std::size_t kNativePlayerCollisionSuppressionBodyCapacity = 64;
        struct NativePlayerCollisionSuppressedBody
        {
            std::uint32_t bodyId = 0x7FFF'FFFFu;
            std::uint32_t motionIndex = 0;
            RE::NiCollisionObject* collisionObject = nullptr;
            RE::NiAVObject* ownerNode = nullptr;
        };
        std::array<NativePlayerCollisionSuppressedBody, kNativePlayerCollisionSuppressionBodyCapacity> _nativePlayerCollisionSuppressedBodies{};
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

        RE::NiPoint3 _prevSmoothedPos;
        int _deltaLogCounter = 0;
        bool _hasPrevPositions = false;
        float _heldMassMovementSpeedReduction = 0.0f;
        float _heldMassMovementFadeStartReduction = 0.0f;
        float _heldMassMovementFadeElapsedSeconds = 0.0f;
        int _heldMassMovementLogCounter = 0;

        int _wpnNodeLogCounter = 0;
    };
}
