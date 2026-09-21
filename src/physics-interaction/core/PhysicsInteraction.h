#pragma once

#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/collision/PushContact.h"

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
#include "physics-interaction/hand/SkeletonBoneNameIndex.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/grab/GrabEvent.h"
#include "physics-interaction/grab/SavedGrabOffsetStore.h"
#include "physics-interaction/grab/TouchGrabRuntime.h"
#include "physics-interaction/grenade/LooseGrenadeRuntime.h"
#include "physics-interaction/hand/DynamicHandCollision.h"
#include "physics-interaction/object/DynamicWorldCarCollision.h"
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
#include "physics-interaction/input/TransferredWeaponGrabPolicy.h"
#include "physics-interaction/native/PhysicsStepDriveCoordinator.h"
#include "physics-interaction/stash/ShoulderStashDetector.h"
#include "physics-interaction/weapon/AuthoredPrimaryFiringGrip.h"
#include "physics-interaction/weapon/EquippedWeaponDropMomentum.h"
#include "physics-interaction/weapon/EquippedWeaponDropPolicy.h"
#include "physics-interaction/weapon/EquippedWeaponDropVisual.h"
#include "physics-interaction/weapon/EquippedWeaponShoulderCoordinator.h"
#include "physics-interaction/weapon/EquippedWeaponTransitionCoordinator.h"
#include "physics-interaction/weapon/EquippedWeaponToggleGrabPolicy.h"
#include "physics-interaction/weapon/VirtualHolstersCompatibility.h"
#include "physics-interaction/weapon/grip/LeftCarryReadiness.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/DynamicWeaponCollision.h"
#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/weapon/WeaponDebug.h"
#include "physics-interaction/weapon/BareFistGuardPolicy.h"
#include "physics-interaction/input/BareFistGesturePolicy.h"
#include "api/ProviderRuntimeTypes.h"
#include "api/WeaponSourceCatalogRuntime.h"

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

        /*
         * FRIK writes its stored weapon offset after ROCK's AfterArmSolve
         * work. The latch (AfterWeaponPosition) keeps the local FRIK wrote;
         * present applies it to the Weapon node for ROCK's frame, and restore
         * hands FRIK back the re-glue local when that frame ends.
         */
        // End of ROCK's tick: settle FRIK's weapon-node write block before FRIK's weapon pass.
        void finalizeFrikWeaponOwnershipForFrame();
        // AfterWeaponPosition: report the two-handed grip to FRIK after its own grip invalidation.
        void syncFrikOffHandGripReport();
        /*
         * AfterWeaponPosition, before the offset latch: FRIK's weapon pass
         * writes the whole Weapon local, so the presentation scale baseline
         * (animation graph scale or unit default) is re-applied after it,
         * where the render and the next frame's latch both see it.
         */
        void normalizeWeaponPresentationScaleAfterFrikWeaponPass();
        void synchronizeEquipVisualBridgeAfterFrikWeaponPass();
        void traceScopeColliderState() const;

        [[nodiscard]] bool tryGetManualScopePresentationTarget(
            std::uint64_t& outWeaponGenerationKey,
            std::uint32_t& outNativeOverlayIndex,
            bool& outDirectTransitionRequired,
            const void* expectedWeapon = nullptr,
            const void* expectedInstance = nullptr) const;

        void update();

        // Publishes grip indicators after every animation phase has
        // committed, immediately before the frame's later VR render submit.
        void publishGripZoneIndicatorRenderFrame(
            std::uint64_t gameFrameIndex);

        // Observes and repairs native equipped-weapon presentation before any
        // weapon-relative ROCK authority reads the first-person graph.
        void updateEquippedWeaponTransition();
        // Value-only continuity belongs to this game session, independently
        // of the scene nodes and Havok bodies retired by shutdown().
        [[nodiscard]] EquippedWeaponTransitionCoordinator::PendingGrip equippedWeaponContinuity() const;
        void restoreEquippedWeaponContinuity(const EquippedWeaponTransitionCoordinator::PendingGrip& grip);
        void captureEquippedWeaponContinuity();

        // Runs before the normal ROCK interaction frame so weapon-relative
        // consumers see one authored primary-grip frame. Runtime eligibility
        // failures clear the tagged hand authority deterministically.
        void updateAuthoredPrimaryFiringGrip();

        void shutdown(::rock::provider::RockProviderLifecycleReason reason = ::rock::provider::RockProviderLifecycleReason::Shutdown);

        bool isInitialized() const { return _lifecycle.initialized; }
        bool isProviderReady() const
        {
            return _lifecycle.initialized.load(std::memory_order_acquire) &&
                ::rock::provider::hasLifecycleFlag(
                    _lifecycle.flagsAtomic.load(std::memory_order_acquire),
                    ::rock::provider::RockProviderLifecycleFlag::ProviderReady);
        }
        void requestWeaponCollisionRebuildAfterWorkbenchExit(const char* sourceMenuName);
        void noteSkeletonLifecycle(std::uint32_t skeletonGeneration, ::rock::provider::RockProviderLifecycleReason reason);
        void noteProviderLifecycle(std::uint32_t providerGeneration, ::rock::provider::RockProviderLifecycleReason reason);

        bool physicsModOwnsObject(RE::TESObjectREFR* ref) const;
        bool physicsModOwnsObject(RE::TESObjectREFR* ref, PhysicsObjectClaimOwner owner) const;
        void claimObject(RE::TESObjectREFR* ref, PhysicsObjectClaimOwner owner = PhysicsObjectClaimOwner::External);
        void releaseObject(RE::TESObjectREFR* ref, PhysicsObjectClaimOwner owner = PhysicsObjectClaimOwner::External);
        void releaseAllObjects();

        void forceDropHeldObject(bool isLeft);

        // Native input dispatch runs on the game thread. Complete collision
        // protection before the native handler can create its grenade/preview.
        bool protectNativeGrenadeThrow();

        Hand& getRightHand() { return _rightHand; }
        Hand& getLeftHand() { return _leftHand; }
        const Hand& getRightHand() const { return _rightHand; }
        const Hand& getLeftHand() const { return _leftHand; }

        std::uint32_t getLastTouchedWeaponPartKind() const
        {
            if (_weaponContact.left.missedFrames.load(std::memory_order_acquire) > WEAPON_CONTACT_TIMEOUT_FRAMES) {
                return static_cast<std::uint32_t>(WeaponPartKind::Other);
            }
            return _weaponContact.left.partKind.load(std::memory_order_acquire);
        }
        bool tryGetRootFlattenedHandTransform(bool isLeft, RE::NiTransform& outTransform) const;
        void refreshProviderWeaponSources();
        api::Status queryProviderWeaponSourcePath(std::uint64_t generation,std::uint64_t key,std::uint64_t& parentKey,std::uint32_t& childIndex) const;
        api::Status queryProviderWeaponSourcePose(std::uint64_t generation,std::uint64_t key,provider::WeaponSourcePose&) const;
        std::uintptr_t resolveProviderWeaponSource(std::uint64_t generation, std::uint64_t key) const;
        std::uint64_t providerWeaponSourceKey(std::uint64_t generation, std::uintptr_t node) const;
        std::uint64_t providerWeaponSourceKeyForBody(std::uint64_t generation, std::uint32_t body) const;
        api::Status copyProviderWeaponSources(std::uint64_t generation,std::uint32_t offset, provider::WeaponSourceRecord*, std::uint32_t capacity, std::uint32_t& copied, std::uint32_t& total) const;
        std::uintptr_t resolveProviderWeaponSourceName(std::uint64_t generation, const char* name) const;
        void fillProviderFrameSnapshot(::rock::provider::RockProviderFrameSnapshot& outSnapshot) const;
        bool isProviderWeaponBodyCurrentV1(
            std::uint64_t weaponGenerationKey,
            std::uint32_t bodyId) const;
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
        void releaseProviderPowerArmorGrabs(std::uint64_t ownerToken);
        bool getProviderHandTargetDetailsV1(
            const ::rock::provider::RockProviderHandInteractionStateV1& handState,
            ::rock::provider::RockProviderHandTargetDetailsV1& outDetails) const;
        std::uint32_t copyProviderBodyContacts(
            ::rock::provider::RockProviderBodyContactV1* outContacts,
            std::uint32_t maxContacts) const;
        bool queryProviderEquippedWeaponClassificationV1(::rock::provider::RockProviderWeaponClassificationV1& outResult) const;
        bool queryProviderEquippedWeaponGripStateV1(
            ::rock::provider::RockProviderEquippedWeaponGripStateV1& outState) const;
        bool queryProviderEquippedWeaponHandlingStateV1(
            ::rock::provider::RockProviderEquippedWeaponHandlingStateV1& outState) const;
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
            ::rock::provider::RockProviderPresentedHandPoseV1& outPose,
            ::rock::provider::RockProviderFrameSnapshot* outMetadata = nullptr) const;
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

        /*
         * Resolve this frame's hand bone cache and isolated controller hands.
         * FRIK's AfterArmSolve phase calls it right after the arm solve so the
         * provider callbacks that run before update() read this frame's
         * hands; update() refreshes again (same inputs).
         */
        void resolveFrameHands() { (void)refreshHandBoneCache(); }

        /*
         * FRIK's AfterWorldFinal phase: the rendered hand bones are final.
         * Latch them, and FRIK's verdict on each claimed hand, for the next
         * frame's controller-hand isolation and chain transport.
         */
        void captureRenderedHands();
        void finalizeFramePose();
        void discardUnfinishedFramePose();
        void captureProviderPresentedHandPoses();
        void traceHeldPresentationPhase(const char* phase);
        void publishDebugRenderFrame();

    private:
        provider::WeaponSourceCatalog _providerSources{};
        struct EquippedWeaponNativeHandoff;

        bool validateCriticalOffsets() const;

        bool refreshHandBoneCache();

        void sampleHandTransformParity();

        RE::NiTransform getInteractionHandTransform(bool isLeft) const;

        RE::NiNode* getInteractionHandNode(bool isLeft) const;

        RE::bhkWorld* getPlayerBhkWorld() const;

        static RE::hknpWorld* getHknpWorld(RE::bhkWorld* bhk);

        PhysicsFrameContext buildFrameContext(RE::bhkWorld* bhk, RE::hknpWorld* hknp);

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
        void captureHandColliderBones();

        bool createBodyBoneCollisions(RE::hknpWorld* world, void* bhkWorld);

        void destroyBodyBoneCollisions(void* bhkWorld);

        void updateBodyBoneCollisions(const PhysicsFrameContext& frame);

        void updateNativePlayerCollisionFilter(RE::bhkWorld* bhk, RE::hknpWorld* hknp);

        void clearNativePlayerCollisionFilter(RE::hknpWorld* hknp);

        bool isNativePlayerCollisionBody(RE::bhkWorld* bhk, RE::hknpWorld* hknp, std::uint32_t bodyId) const;

        void driveGeneratedCollidersFromPhysicsSubstep(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        void driveCustomGrabAuthorityFromBetweenStep(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        void observeCustomGrabAuthorityAfterSolve(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);

        static void onGeneratedColliderPhysicsSubstep(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        static void onCustomGrabAuthorityBetweenStep(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        static void onCustomGrabAuthorityAfterSolve(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);

        void updateSelection(const PhysicsFrameContext& frame);
        struct EquippedWeaponFrameResult;
        EquippedWeaponFrameResult updateEquippedWeaponFrame(const PhysicsFrameContext& frame, RE::bhkWorld* bhk, RE::hknpWorld* hknp);
        void finalizeInteractionFrame(const PhysicsFrameContext& frame,
            RE::hknpWorld* hknp,
            const EquippedWeaponFrameResult& equippedWeaponFrame);
        void prepareDynamicWorldCarCollisionForGrab(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, RE::TESObjectREFR* ref);
        GrabReleaseContext makeGrabReleaseContext(const Hand& hand, bool isLeft) const;
        GrabSharedObjectContext makeGrabSharedObjectContext(const Hand& hand, bool isLeft) const;

        struct GrabInputHandContext;
        struct GrabInputHandPrelude;
        void clearShoulderStashForHand(Hand& hand, bool isLeft);
        void clearMouthConsumeForHand(Hand& hand, bool isLeft);
        void clearGameplayCandidatesForHand(Hand& hand, bool isLeft);
        void publishHandInputOwnership(const Hand& hand, bool isLeft);
        void releaseSuppressedHeldObject(RE::hknpWorld* world, Hand& hand, bool isLeft, const char* reason);
        void cancelPeerHeldJoinRetry(Hand& hand,
            peer_held_join_retry_policy::RuntimeState& retryState,
            const char* reason,
            bool logCancellation);
        bool prepareGrabInputHand(const PhysicsFrameContext& frame,
            Hand& hand,
            bool isLeft,
            const GrabInputHandContext& context,
            GrabInputHandPrelude& outPrelude);
        bool processTouchGrabInput(const PhysicsFrameContext& frame,
            Hand& hand,
            bool isLeft,
            const GrabInputHandContext& context,
            const GrabInputHandPrelude& prelude);
        void processGrabIntentAndCommit(const PhysicsFrameContext& frame,
            Hand& hand,
            bool isLeft,
            const GrabInputHandContext& context,
            const GrabInputHandPrelude& prelude);
        void processGrabInputHand(const PhysicsFrameContext& frame, Hand& hand, bool isLeft, const GrabInputHandContext& context);
        void updateGrabInput(const PhysicsFrameContext& frame);
        void processProviderInteractionCommands(const PhysicsFrameContext& frame);
        std::uint32_t forceGrabHandBlockerMask(const Hand& hand, bool isLeft, bool handDisabled, bool includePendingCommit) const;
        bool equippedWeaponFiringHandForGrabIsLeft() const;
        bool canHandAcceptForceGrab(const Hand& hand, bool isLeft, bool handDisabled) const;
        bool handHoldsLooseGrenade(const Hand& hand) const;
        bool hasActiveLooseGrenadeCommit() const;
        bool isPendingForceGrabTarget(RE::TESObjectREFR* ref) const;
        void pruneInactiveProviderForceGrabCommits();
        void serviceLooseGrenadeQuickDraw(const PhysicsFrameContext& frame);
        void servicePendingForceGrabCommits(const PhysicsFrameContext& frame);
        void clearPendingForceGrabCommits();
        void cancelEquippedWeaponTransfersForMenu();
        void updateSavedGrabOffsetGesture(const PhysicsFrameContext& frame);
        void saveGrabOffsetForHand(Hand& hand, bool isLeft, RE::hknpWorld* hknpWorld);
        void armEquippedWeaponNativeHandoff(
            const RE::ObjectRefHandle& handle,
            std::uint32_t droppedFormId,
            equipped_weapon_drop_policy::SourceHand sourceHand,
            const WeaponCollision::ReleaseGeometrySnapshot& releaseGeometry);
        bool hasAvailableEquippedWeaponDropHandoff() const;
        void serviceEquippedWeaponNativeHandoff(const PhysicsFrameContext& frame);
        bool dropEquippedWeaponToWorld(const PhysicsFrameContext& frame,
            const EquippedWeaponManualDropRequest& request, equipped_weapon_drop_policy::Mode mode);
        void updateEquippedWeaponDropVisuals(const PhysicsFrameContext& frame);
        void finishEquippedWeaponHandPoseHandoff();
        void serviceEquippedWeaponNativeTransaction(
            EquippedWeaponNativeHandoff& handoff,
            const PhysicsFrameContext& frame);
        bool armHeldLooseGrenade(Hand& hand, const PhysicsFrameContext& frame);
        void updateLooseGrenadeFuses(const PhysicsFrameContext& frame);
        void clearLooseGrenadeImpactWatches();
        void clearLooseGrenadeRuntimeState();
        void enforceNoBareFistState(bool forceRecheck);
        void updateBareFistMode(const PhysicsFrameContext& frame);
        void cancelBareFistMode(const char* reason);
        [[nodiscard]] bool bareFistHandsAvailable(const PhysicsFrameContext& frame) const;

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
            const Hand* sourceHand,
            const push_assist::Contact& contact);

        void publishDebugBodyOverlay(const PhysicsFrameContext& frame);
        void logGrabOverlayPointProbe(const PhysicsFrameContext& frame);

        void clearLeftWeaponContact();
        void clearRightWeaponContact();

        void refreshEquippedWeaponHandlingSettings();
        void reconcileEquippedWeaponHandlingMode();
        struct EquippedWeaponShoulderFrameResult
        {
            equipped_weapon_shoulder::Decision decision{};
            equipped_weapon_drop_policy::SourceHand sourceHand{
                equipped_weapon_drop_policy::SourceHand::None
            };
            shoulder_stash::Decision detectorDecision{};
        };
        bool submitEquippedWeaponShoulderSheath(
            std::uint32_t observedWeaponFormID,
            std::uintptr_t observedWeaponInstanceData,
            equipped_weapon_drop_policy::SourceHand sourceHand,
            const shoulder_stash::Decision& stashDecision,
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey);
        void clearEquippedWeaponShoulderSheath(
            const char* reason,
            bool resetCoordinator = true);
        EquippedWeaponShoulderFrameResult
            advanceEquippedWeaponShoulderCoordinator(
            const PhysicsFrameContext& frame,
            bool handlingEnabled,
            bool menuInputActive,
            std::uint32_t observedWeaponFormID,
            std::uintptr_t observedWeaponInstanceData,
            RE::NiNode* weaponNode,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            bool firingHandIsLeft);
        void suppressRightHandCollisionForDominantWeapon(RE::hknpWorld* world);

        void restoreRightHandCollisionAfterDominantWeapon(RE::hknpWorld* world);

        void suppressHandCollisionForWeaponSupport(RE::hknpWorld* world, bool isLeft);

        void beginDelayedHandCollisionRestoreAfterWeaponSupport(
            RE::hknpWorld* world,
            bool isLeft);

        void restoreHandCollisionAfterWeaponSupport(
            RE::hknpWorld* world,
            bool isLeft,
            bool forceImmediate = false);

        void updateWeaponSupportCollisionSuppression(
            RE::hknpWorld* world,
            float deltaSeconds);

        void suppressHandCollisionAfterEquippedWeaponDrop(
            RE::hknpWorld* world,
            equipped_weapon_drop_policy::SourceHand sourceHand);

        void restoreHandCollisionAfterEquippedWeaponDrop(RE::hknpWorld* world, bool isLeft);

        void updateEquippedWeaponPostDropCollisionSuppression(RE::hknpWorld* world, float deltaSeconds);

        void clearEquippedWeaponPostDropCollisionSuppressionState();
        bool refreshNativeGrenadeCollisionSuppression(RE::hknpWorld* world);
        void updateNativeGrenadeCollisionSuppression(RE::hknpWorld* world, float deltaSeconds);
        void restoreNativeGrenadeCollisionSuppression(RE::hknpWorld* world);

        void subscribeContactEvents(RE::hknpWorld* world);
        void unsubscribeContactEvents(RE::hknpWorld* liveWorld);

        void dispatchPhysicsMessage(std::uint32_t msgType, bool isLeft, RE::TESObjectREFR* refr = nullptr, std::uint32_t formID = 0, std::uint32_t layer = 0);
        api::SampleV1 providerEventSample() const;
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
        void handleManifoldProcessedEvent(RE::hknpWorld* world, void* eventData);
        bool isHandContactEvidenceSuppressed(bool isLeft) const;
        void clearContactEvidenceForHand(bool isLeft);
        void synchronizeContactEvidenceOwnership(bool rightHandWeaponAuthorityActive, bool leftSupportGripActive, bool rightPartGripActive);

        /*
         * ---- Partitioned member state ----
         * Each interaction/ module owns one state struct below. Long-lived
         * subsystem objects (hands, grips, collision runtimes, caches,
         * haptics) remain direct members at the end; nested types and
         * constants stay at class scope so implementation references remain
         * unqualified.
         */

        static constexpr std::size_t kGeneratedBodyContactRegistryCapacity =
            (hand_collider_semantics::kHandColliderBodyCountPerHand * 2u) +
            MAX_WEAPON_COLLISION_BODIES +
            kBodyBoneColliderBodyCount;
        static constexpr std::uint64_t INVALID_HELD_IMPACT_PAIR = 0xFFFF'FFFF'FFFF'FFFFull;
        static constexpr std::uint32_t INVALID_CONTACT_BODY_ID = 0x7FFF'FFFF;
        static constexpr std::uint32_t WEAPON_CONTACT_TIMEOUT_FRAMES = 5;
        // Placed mines remain active after release, so retain a bounded pool
        // larger than the original simultaneous hand-grenade fuse budget.
        static constexpr std::size_t kArmedLooseGrenadeFuseCapacity = 8;
        static constexpr std::size_t kEquippedWeaponDropBodySnapshotCapacity = 32;
        static constexpr std::size_t kEquippedWeaponDropHandoffCapacity = 4;

        struct EquippedWeaponShoulderSheathState
        {
            bool active{ false };
            bool stashedByLeftHand{ false };
            std::uint32_t weaponFormID{ 0 };
            std::uintptr_t weaponInstanceData{ 0 };
            std::uint32_t equipIndex{ 0 };
            std::uint64_t weaponOwnershipKey{ 0 };
            body_zone::BodyZoneKind zone{ body_zone::BodyZoneKind::Unknown };
            bool hasLeftFiringGripTransfer{ false };
            RE::NiTransform leftFiringHandWeaponLocal{};
            RE::NiPoint3 leftFiringGripWeaponLocal{};
        };

        struct HeldWeaponTriggerEquipIntent
        {
            bool pending{ false };
            std::uint32_t formID{ 0 };
            // Zero only while a same-frame selection is awaiting its first grab.
            std::uint64_t grabIdentity{ 0 };
            float remainingSeconds{ 0.0f };
        };

        /*
         * Loose-to-equipped handoff state. The loose root disappears during
         * inventory transfer, so the physical hand and its weapon-local frame
         * are retained by value until the equipped node becomes observable.
         */
        using PendingEquippedWeaponPrimaryOnlyGripStart = EquippedWeaponTransitionCoordinator::PendingGrip;

        struct ArmedLooseGrenadeFuseState
        {
            bool active{ false };
            RE::ObjectRefHandle handle{};
            std::uint32_t refFormID{ 0 };
            loose_grenade_runtime::GrenadeRuntimeData runtime{};
            float remainingSeconds{ 0.0f };
            std::uint32_t impactBodyId{ INVALID_CONTACT_BODY_ID };
            bool releasedSinceArming{ false };
        };

        enum class EquippedWeaponDropHandoffStage : std::uint8_t
        {
            ResolvingBodies,
            WaitingForSettleStep,
        };

        struct EquippedWeaponDropBodySnapshot
        {
            bool valid{ false };
            equipped_weapon_drop_momentum::BodyIdentityKey identity{};
        };

        /*
         * Deferred native-drop transaction. RemoveItem can publish the ref and
         * body tree asynchronously, so ROCK first resolves exact-ref bodies,
         * enables collision, places every native motion at the mode's placement
         * pose with zero velocity, and waits for one completed native
         * solve before the exact-reference force grab takes ownership. The
         * held-object release path owns any later throw momentum.
         */
        struct EquippedWeaponNativeHandoff
        {
            bool active{ false };
            bool referenceResolvedOnce{ false };
            bool threeDResolvedOnce{ false };
            RE::ObjectRefHandle handle{};
            std::uint32_t droppedFormId{ 0 };
            float elapsedSeconds{ 0.0f };
            std::uint32_t identityRestartCount{ 0 };
            bool hasReleaseWeaponWorld{ false };
            RE::NiTransform releaseWeaponWorld{};
            EquippedWeaponDropHandoffStage stage{ EquippedWeaponDropHandoffStage::ResolvingBodies };
            std::uint64_t progressSolveSequence{ 0 };
            std::uint64_t bodyDiscoverySolveSequence{ 0 };
            std::array<EquippedWeaponDropBodySnapshot, kEquippedWeaponDropBodySnapshotCapacity> bodySnapshots{};
            std::size_t bodySnapshotCount{ 0 };
            // Value-only terminal evidence. waitReason points only to literals;
            // no scene/body pointers survive the service callback.
            const char* waitReason{ "not-serviced" };
            std::uint32_t visitedNodes{ 0 };
            std::uint32_t collisionObjects{ 0 };
            std::uint32_t scannedBodies{ 0 };
            std::uint32_t acceptedBodies{ 0 };
            std::uint32_t uniqueMotions{ 0 };
            std::uint32_t scanFailures{ 0 };
            std::uint32_t invalidSystems{ 0 };
            std::uint32_t depthSkips{ 0 };
            std::uint32_t foreignRefSkips{ 0 };
            std::uint64_t rejectionMask{ 0 };
            std::uint32_t inspectedBodyId{ 0x7FFFFFFF };
            std::uint32_t identityProofMask{ 0 };
            std::uint64_t observedSolveSequence{ 0 };
        };

        void reportEquippedWeaponPlacementFailure(const EquippedWeaponNativeHandoff& handoff, const char* reason) const;

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

        struct ProviderHandInputSuppressionRuntimeState
        {
            bool deferredGrabRelease = false;
        };

        /*
         * Cross-thread contact witness of which generated weapon part one
         * physical hand currently touches. Written by the contact callback,
         * aged by the frame update, consumed by suppression and the equipped
         * weapon frame.
         */
        struct WeaponContactWitness
        {
            std::atomic<std::uint32_t> bodyId{ INVALID_CONTACT_BODY_ID };
            std::atomic<std::uint32_t> partKind{ static_cast<std::uint32_t>(WeaponPartKind::Other) };
            std::atomic<std::uint32_t> reloadRole{ static_cast<std::uint32_t>(WeaponReloadRole::None) };
            std::atomic<std::uint32_t> supportRole{ static_cast<std::uint32_t>(WeaponSupportGripRole::None) };
            std::atomic<std::uint32_t> socketRole{ static_cast<std::uint32_t>(WeaponSocketRole::None) };
            std::atomic<std::uint32_t> actionRole{ static_cast<std::uint32_t>(WeaponActionRole::None) };
            std::atomic<std::uint32_t> gripPose{ static_cast<std::uint32_t>(WeaponGripPoseId::None) };
            std::atomic<std::uint32_t> sequence{ 0 };
            std::atomic<std::uint32_t> missedFrames{ WEAPON_CONTACT_TIMEOUT_FRAMES + 1 };
        };

        // State owned by the PhysicsInteractionLifecycle module: init state,
        // the lifecycle state machine and its atomic mirrors, cross-thread
        // generation counters, the bound Havok worlds, the worlds/generations
        // the generated bodies were built against, and collider-creation
        // retry backoff.
        struct LifecycleState
        {
            std::atomic<bool> initialized{ false };
            physics_lifecycle::RuntimeState state{};
            std::atomic<std::uint32_t> flagsAtomic{ 0 };
            std::atomic<std::uint32_t> lastReasonAtomic{ static_cast<std::uint32_t>(::rock::provider::RockProviderLifecycleReason::None) };
            std::atomic<std::uint32_t> worldGenerationAtomic{ 1 };
            std::atomic<std::uint32_t> skeletonGenerationAtomic{ 1 };
            std::atomic<std::uint32_t> providerGenerationAtomic{ 1 };
            std::atomic<std::uint32_t> collisionGenerationAtomic{ 1 };
            std::atomic<std::uint32_t> stableFrameCountAtomic{ 0 };
            std::atomic<RE::hknpWorld*> hknpWorldAtomic{ nullptr };
            RE::bhkWorld* cachedBhkWorld = nullptr;
            RE::hknpWorld* cachedHknpWorld = nullptr;
            RE::bhkWorld* generatedBodiesBhkWorld = nullptr;
            RE::hknpWorld* generatedBodiesHknpWorld = nullptr;
            std::uint32_t generatedBodiesWorldGeneration = 0;
            std::uint32_t generatedBodiesSkeletonGeneration = 0;
            std::uint32_t generatedBodiesProviderGeneration = 0;
            int handColliderCreateRetryFrames = 0;
            int bodyBoneColliderCreateRetryFrames = 0;
        };

        // Collision-layer registration and audit expectations, captured at
        // registration by the lifecycle module and audited by the update
        // module.
        struct CollisionLayerAuditState
        {
            bool registered = false;
            std::uint64_t expectedHandMask = 0;
            std::uint64_t expectedWeaponMask = 0;
            std::uint64_t expectedReloadMask = 0;
            std::uint64_t expectedBodyMask = 0;
            std::uint64_t expectedDynamicHandProxyMask = 0;
            std::uint64_t expectedDynamicLeftHandProxyMask = 0;
            std::uint64_t expectedDynamicWeaponProxyMask = 0;
            std::uint64_t expectedDynamicWorldCarClutterMask = 0;
            std::uint64_t expectedDynamicWorldCarLargeClutterMask = 0;
        };

        // State owned by the contact callback (PhysicsInteractionContacts.inl):
        // event registration witnesses, contact trackers, last-contact
        // witnesses, and the dynamic-push clock.
        struct ContactEvidenceState
        {
            std::atomic<RE::hknpWorld*> eventWorld{ nullptr };
            std::atomic<void*> eventSignal{ nullptr };
            std::atomic<RE::hknpWorld*> manifoldEventWorld{ nullptr };
            std::atomic<void*> manifoldEventSignal{ nullptr };
            contact_activity_tracker::ContactActivityTracker handActivity;
            body_contact_runtime::BodyContactRuntime bodyRuntime;
            generated_body_contact_registry::Registry<kGeneratedBodyContactRegistryCapacity> generatedBodyRegistry;
            push_assist::ContactChannel rightPush, leftPush, weaponPush;
            std::atomic<std::uint64_t> lastHeldImpactPairRight{ INVALID_HELD_IMPACT_PAIR };
            std::atomic<std::uint64_t> lastHeldImpactPairLeft{ INVALID_HELD_IMPACT_PAIR };
            float dynamicPushElapsedSeconds = 0.0f;
            std::unordered_map<std::uint64_t, float> dynamicPushCooldownUntil;
        };

        struct WeaponContactWitnessPair
        {
            WeaponContactWitness left;
            WeaponContactWitness right;
        };

        // State owned by the EquippedWeaponFrame module: handling settings,
        // the transition coordinator, the authored primary grip runtime,
        // shoulder sheath/stash/retrieval, toggle grab, and the pending
        // loose-to-equipped handoff.
        struct EquippedWeaponFrameState
        {
            EquippedWeaponTransitionCoordinator transition;
            AuthoredPrimaryFiringGripRuntime authoredPrimaryFiringGrip;
            EquippedWeaponHandlingSettings handlingSettings{};
            bool handlingModeInitialized{ false };
            bool handlingModeReconcilePending{ false };
            bool menuReconcilePending = false;
            EquippedWeaponTransitionCoordinator::PendingGrip continuityGrip{};
            bool gripResumePending{ false };
            std::array<bool, 2> resumeAwaitingHold{};
            // Dedicated stash detector states for the equipped-weapon carry
            // gesture so dwell/hysteresis never mixes with a loose object
            // held by the same hand.
            std::array<shoulder_stash::RuntimeState, 2> stashStates{};
            // The equipped instance remains equipped while native
            // presentation is sheathed. Retrieval owns independent per-hand
            // dwell so either empty physical hand can claim the same stored
            // shoulder in ambidextrous mode.
            EquippedWeaponShoulderSheathState shoulderSheath{};
            std::array<shoulder_stash::RuntimeState, 2> sheathRetrievalStates{};
            equipped_weapon_shoulder::RuntimeState shoulderCoordinator{};
            std::array<bool, 2> shoulderGestureConsumedThisFrame{};
            equipped_weapon_toggle_grab_policy::RuntimeState toggleGrabState{};
            std::array<bool, 2> toggleGrabReleasePressConsumedThisFrame{};
            std::array<virtual_holsters::HandState, 2> holsterInputStates{};
            std::array<bool, 2> holsterInputConsumedThisFrame{};
            std::array<weapon_interaction_acquisition_policy::State, 2> weaponInteractionAcquisitionStates{};
            // Left/right candidates from the actual grab probes, valid only for
            // this frame and weapon generation. No scene pointers cross phases.
            std::array<std::uint32_t, 2> partIndicatorBodyIds{
                weapon_part_runtime::kInvalidBodyId, weapon_part_runtime::kInvalidBodyId };
            std::uint64_t partIndicatorFrame{ 0 };
            std::uint64_t partIndicatorGeneration{ 0 };
        };

        // State owned by the EquippedWeaponDrop module.
        struct EquippedWeaponDropState
        {
            std::array<EquippedWeaponDropVisual, 2> visuals{};
            std::array<EquippedWeaponNativeHandoff, kEquippedWeaponDropHandoffCapacity> nativeHandoffs{};
        };

        // State owned by the GrabInput module: grab intents, the shared
        // firing-hand button snapshot, bare-fist guard, provider input
        // suppression, peer-join retries, and the loose-object mouth/shoulder
        // gestures.
        struct GrabInputState
        {
            std::array<grab_input_intent_policy::RuntimeState, 2> intentStates{};
            std::array<HeldWeaponTriggerEquipIntent, 2> heldWeaponTriggerEquipIntents{};
            SharedGrabButtonFrameState firingHandButtonFrame{};
            bare_fist_guard_policy::RecheckState bareFistGuardState{};
            bare_fist_gesture::State bareFistGesture{};
            bool bareFistDrawOwned{ false };
            bool bareFistHolsterRequested{ false };
            std::uint32_t bareFistWorldGeneration{ 0 };
            std::array<ProviderHandInputSuppressionRuntimeState, 2> providerHandInputSuppressionStates{};
            std::array<peer_held_join_retry_policy::RuntimeState, 2> peerHeldJoinRetryStates{};
            std::array<mouth_consume::RuntimeState, 2> mouthConsumeStates{};
            std::array<shoulder_stash::RuntimeState, 2> shoulderStashStates{};
        };

        // State owned by the ForceGrabAndGrenades module.
        struct ForceGrabState
        {
            struct RetainedWeaponGrab
            {
                // Native handles belong to pending reference resolution. Once
                // committed, the Hand's acquisition identity owns retention.
                std::uint64_t grabIdentity{ 0 };
                transferred_weapon_grab_policy::State inputState{};
            };
            std::array<PendingForceGrabCommit, 2> pendingCommits{};
            std::array<RetainedWeaponGrab, 2> retainedWeaponGrabs{};
            std::array<bool, 2> committedThisFrame{};
            std::array<ArmedLooseGrenadeFuseState, kArmedLooseGrenadeFuseCapacity> grenadeFuses{};
            std::array<std::atomic<std::uint32_t>, kArmedLooseGrenadeFuseCapacity> grenadeImpactBodyIds{};
            std::atomic<std::uint64_t> pendingGrenadeImpactPair{ INVALID_HELD_IMPACT_PAIR };
        };

        // State owned by the GrabEventsAndHaptics module.
        struct GrabEventState
        {
            std::uint64_t frameCounter = 0;
            std::unordered_map<std::uint64_t, float> heldImpactHapticCooldownUntil;
        };

        // State owned by the ProviderCommands module and the provider API
        // surface (PhysicsInteractionProvider.inl).
        struct ProviderDriveState
        {
            std::array<ProviderWeaponPartDriveNodeState, ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1> nodeStates{};
            std::uint64_t generationKey{ 0 };
            std::array<::rock::provider::RockProviderWeaponPartDriveApplicationResultV1,
                ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVE_RESULTS_V1>
                results{};
            std::uint32_t resultCount{ 0 };
            SkeletonBoneNameIndex presentedPoseNames{};
            std::array<::rock::provider::RockProviderPresentedHandPoseV1, 2> presentedPoses{};
            ::rock::provider::RockProviderFrameSnapshot presentedMetadata{};
        };

        // State owned by the HandCollisionSuppression module: per-hand
        // suppression flags and lease sets, and native player body
        // suppression.
        struct HandSuppressionState
        {
            collision_suppression_registry::SuppressionLeaseSet<kGeneratedBodyContactRegistryCapacity + 3>
                nativeGrenadeLeases{ collision_suppression_registry::CollisionSuppressionOwner::NativeGrenadeThrow };
            std::atomic<bool> rightDominantSuppressed{ false };
            std::atomic<bool> leftWeaponSupportSuppressed{ false };
            std::atomic<bool> rightWeaponSupportSuppressed{ false };
            std::atomic<bool> rightDropSuppressed{ false };
            std::atomic<bool> leftDropSuppressed{ false };
            collision_suppression_registry::SuppressionLeaseSet<hand_collider_semantics::kHandColliderBodyCountPerHand>
                rightDominantLeases{
                    collision_suppression_registry::CollisionSuppressionOwner::WeaponDominantHand };
            collision_suppression_registry::SuppressionLeaseSet<hand_collider_semantics::kHandColliderBodyCountPerHand>
                leftWeaponSupportLeases{
                    collision_suppression_registry::CollisionSuppressionOwner::WeaponSupportHand };
            collision_suppression_registry::SuppressionLeaseSet<hand_collider_semantics::kHandColliderBodyCountPerHand>
                rightWeaponSupportLeases{
                    collision_suppression_registry::CollisionSuppressionOwner::WeaponSupportHand };
            collision_suppression_registry::SuppressionLeaseSet<kGrabCollisionSuppressionBodyCountPerHand>
                rightDropLeases{
                    collision_suppression_registry::CollisionSuppressionOwner::EquippedWeaponDropHand };
            collision_suppression_registry::SuppressionLeaseSet<kGrabCollisionSuppressionBodyCountPerHand>
                leftDropLeases{
                    collision_suppression_registry::CollisionSuppressionOwner::EquippedWeaponDropHand };
            std::uint32_t nativePlayerRefreshFrames = 0;
            bool nativePlayerOverflowLogged = false;
        };

        // Frame clock and per-frame products owned by the update module.
        struct FrameClockState
        {
            // Main-thread eligibility only; never retain frame-local engine pointers.
            std::uint64_t debugOverlayFrameIndex = 0;
            std::uint64_t gripZoneIndicatorFrameIndex = 0;
            std::uint64_t poseFrameIndex = 0;
            // Central sanitized game delta captured each update; zero until
            // the first frame is measured.
            float deltaTime = 0.0f;
            std::atomic<std::uint64_t> palmClockGameFrameIndex{ 0 };
            std::atomic<float> palmClockGameDeltaSeconds{ 0.0f };
            // Written only by the post-solve callback and sampled by the
            // main-frame equipped-drop service. This explicit atomic is the
            // cross-thread settle barrier; PhysicsStepDriveCoordinator's
            // internal counter is not read across threads.
            std::atomic<std::uint64_t> completedPhysicsSolveSequence{ 0 };
            RE::NiPoint3 prevSmoothedPos;
            bool hasPrevPositions = false;
            float heldMassSpeedReduction = 0.0f;
            float heldMassFadeStartReduction = 0.0f;
            float heldMassFadeElapsedSeconds = 0.0f;
        };

        // Physics object claims (ObjectClaims module).
        struct ObjectClaimsState
        {
            mutable std::mutex mutex;
            std::unordered_map<std::uint32_t, std::uint32_t> owned;
        };

        // Diagnostic counters, one-shot log latches, parity auditing, and
        // grab-transform telemetry.
        struct InteractionDiagnosticsState
        {
            std::atomic<int> contactLogCounter{ 0 };
            int deltaLogCounter = 0;
            int handCacheResolveLogCounter = 0;
            int heldMassLogCounter = 0;
            int weaponInteractionProbeLogCounter = 0;
            int wpnNodeLogCounter = 0;
            int paritySummaryCounter = 0;
            bool parityEnabledLogged = false;
            bool runtimeScaleLogged = false;
            std::array<RawHandParityState, 2> rawHandParityStates{};
            std::array<GrabTransformTelemetryState, 2> grabTransformTelemetryStates{};
            std::uint32_t grabTransformTelemetryNextSession = 1;
            weapon_debug_notification_policy::WeaponNotificationState weaponDebugNotification{};
        };

        LifecycleState _lifecycle;
        CollisionLayerAuditState _layers;
        ContactEvidenceState _contacts;
        WeaponContactWitnessPair _weaponContact;
        EquippedWeaponFrameState _equipped;
        EquippedWeaponDropState _drop;
        GrabInputState _grabInput;
        ForceGrabState _forceGrab;
        GrabEventState _grabEvents;
        ProviderDriveState _providerDrives;
        HandSuppressionState _suppression;
        FrameClockState _frame;
        ObjectClaimsState _claims;
        InteractionDiagnosticsState _diagnostics;

        // ---- Long-lived subsystem objects ----
        HandBoneCache _handBoneCache;
        // Game-thread scratch, recaptured immediately before the two collider
        // updates. Never shared with rendered-space body or presentation reads.
        DirectSkeletonBoneReader _handColliderBoneReader;
        DirectSkeletonBoneSnapshot _handColliderBoneSnapshot;
        // A separate cache keeps early hand-only and final full-body topology
        // stable. API readback and both collider owners share this final copy.
        DirectSkeletonBoneReader _finalPoseBoneReader;
        DirectSkeletonBoneSnapshot _finalPoseBoneSnapshot;
        HandFrameResolver _handFrameResolver;
        // Last native recoil kick the FRIK recoil controller saw; a change
        // marks a frame whose rendered hand carries a composed kick.
        std::uint64_t _observedNativeRecoilKickSequence = 0;
        Hand _rightHand{ false };
        Hand _leftHand{ true };
        TouchGrabRuntime _touchGrabRuntime;
        std::array<TouchGrabRuntime::PowerArmorCandidate, 2> _powerArmorCandidates{};
        std::array<TouchGrabRuntime::PowerArmorProbeDiagnostics, 2> _powerArmorProbeDiagnostics{};
        std::uint64_t _powerArmorCandidateFrame = 0;
        BodyBoneColliderSet _bodyBoneColliders;
        WeaponCollision _weaponCollision;
        DynamicWeaponCollisionRuntime _dynamicWeaponCollision;
        PhysicsStepDriveCoordinator _generatedBodyStepDrive;
        TwoHandedGrip _twoHandedGrip;
        DynamicHandCollisionRuntime _dynamicHandCollision;
        DynamicWorldCarCollisionRuntime _dynamicWorldCarCollision;
        feedback_haptics::FeedbackHaptics _feedbackHaptics;
    };
}
