#pragma once

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "api/FRIKApiV2.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/weapon/AuthoredSupportGrabPolicy.h"
#include "physics-interaction/weapon/EquippedWeaponDropPolicy.h"
#include "physics-interaction/weapon/EquippedWeaponHandlingSettings.h"
#include "physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h"
#include "physics-interaction/weapon/FiringGripReattachZonePolicy.h"
#include "physics-interaction/weapon/NativeScopeSightAnchorPolicy.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
#include "physics-interaction/weapon/WeaponPartGripReportPolicy.h"
#include "physics-interaction/weapon/WeaponSupport.h"

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTransform.h"

namespace rock
{
    namespace authored_weapon_grip_library
    {
        struct FiringFingerPose;
    }

    namespace grab_finger_pose_runtime
    {
        struct SolvedGrabFingerPose;
    }

    /*
     * Equipped-weapon grip is modeled as two stations plus the free state:
     * the firing grip (game/FRIK weapon ownership follows this hand) and
     * per-hand part grips (ROCK-owned mesh grips on generated weapon parts).
     * The weapon itself owns collision and firing authority; whichever hand
     * occupies the firing grip carries firing input authority. PartCarry is
     * the state where no hand occupies the firing grip and the weapon is
     * carried entirely by one or two part grips.
     */
    enum class TwoHandedState
    {
        Inactive,
        Touching,
        Gripping,
        PartCarry,
        PrimaryOnly,
    };

    struct EquippedWeaponPrimaryGripInput
    {
        bool held{ false };
        bool pressed{ false };
        bool released{ false };
    };

    struct EquippedWeaponScopeHandDriverFrame
    {
        bool valid{ false };
        RE::NiTransform world{};
        // Diagnostic witnesses preserve why capture failed without changing
        // the existing validity contract consumed by the solver.
        bool nodeAvailable{ false };
        bool worldFinite{ false };
    };

    struct EquippedWeaponGripFrameInput
    {
        bool leftGripHeld{ false };
        bool rightGripHeld{ false };
        bool leftHandHoldingObject{ false };
        bool rightHandHoldingObject{ false };
        bool leftReattachEligible{ false };
        bool rightReattachEligible{ false };
        // Menu-open state and renderer-request state are deliberately separate.
        // FO4VR may keep WSScope presentation alive outside an active sight;
        // the renderer request is the authority for actual scope entry/exit.
        bool scopeMenuOpen{ false };
        bool manualScopeActivationRequested{ false };
        bool nativeScopeRequestStateValid{ false };
        bool nativeScopeRequestActive{ false };
        EquippedWeaponScopeHandDriverFrame leftHandDriverFrame{};
        EquippedWeaponScopeHandDriverFrame rightHandDriverFrame{};
        // Grab state of the CURRENT firing hand (debounced release), read by
        // the caller from whichever physical hand isFiringHandLeft() reports.
        EquippedWeaponPrimaryGripInput primaryGripInput{};
        // Failure-only telemetry inputs. They never participate in grip
        // decisions; they let an involuntary teardown distinguish physical
        // input, toggle translation, animation boundaries, and tracking.
        EquippedWeaponPrimaryGripInput leftPhysicalGripInput{};
        EquippedWeaponPrimaryGripInput rightPhysicalGripInput{};
        RE::NiPoint3 hmdPositionWorld{};
        bool toggleGrabEnabled{ false };
        bool animationBoundaryActive{ false };
        bool hasHmdFrame{ false };
    };

    struct EquippedWeaponHandGripOccupancy
    {
        bool firingGripActive{ false };
        bool partGripActive{ false };

        [[nodiscard]] bool weaponEngaged() const noexcept
        {
            return firingGripActive || partGripActive;
        }
    };

    struct EquippedWeaponGripOccupancy
    {
        EquippedWeaponHandGripOccupancy left{};
        EquippedWeaponHandGripOccupancy right{};
    };

    // Hands whose open-hand release was refused during this update because
    // they are the weapon's last carrier and the last-grip drop is disabled.
    struct EquippedWeaponGripReleaseRetention
    {
        bool left{ false };
        bool right{ false };
    };

    struct TwoHandedGripUpdateResult
    {
        EquippedWeaponGripOccupancy before{};
        EquippedWeaponGripOccupancy after{};
        EquippedWeaponGripReleaseRetention releaseRetained{};
    };

    struct AuthoredSupportGripIndicatorFrame
    {
        RE::NiPoint3 positionWorld{};
        // Generation-bound anchor for mode 2. The final presentation pass
        // resolves it through the weapon transform that will actually render.
        RE::NiPoint3 positionWeaponLocal{};
        std::uint64_t weaponGenerationKey{ 0 };
        bool supportHandIsLeft{ true };
        bool weaponLocalValid{ false };
        bool visible{ false };
    };

    struct FiringGripReattachIndicatorFrame
    {
        RE::NiPoint3 positionWorld{};
        // Same render-current contract as the support-seat marker above.
        RE::NiPoint3 positionWeaponLocal{};
        std::uint64_t weaponGenerationKey{ 0 };
        bool handIsLeft{ false };
        bool weaponLocalValid{ false };
        bool visible{ false };
    };

    /*
     * Main-thread diagnostic record of the firing-grip reattach zone for the
     * debug overlay: where the cylinders start, their axis and size, plus
     * each evaluated free palm's verdict. Values only, never engine pointers.
     */
    struct FiringGripReattachZoneDebugSnapshot
    {
        struct HandSample
        {
            RE::NiPoint3 palmWorld{};
            float alongAxisGameUnits{ 0.0f };
            float perpendicularDistanceGameUnits{ 0.0f };
            float radialDistanceGameUnits{ 0.0f };
            firing_grip_reattach_zone_policy::Side side{
                firing_grip_reattach_zone_policy::Side::None
            };
            bool evaluated{ false };
            bool gripHeld{ false };
            bool reachPass{ false };
            bool radiusPass{ false };
            bool inside{ false };
        };

        RE::NiPoint3 gripWorld{};
        RE::NiPoint3 weaponLeftAxisWorld{};
        float reachGameUnits{ 0.0f };
        float cylinderRadiusGameUnits{ 0.0f };
        // Index 0 left hand, 1 right hand.
        std::array<HandSample, 2> hands{};
        bool valid{ false };
    };

    struct TwoHandedGripDebugSnapshot
    {
        RE::NiTransform weaponWorld{};
        RE::NiTransform rightRequestedHandWorld{};
        RE::NiTransform leftRequestedHandWorld{};
        RE::NiPoint3 rightGripWorld{};
        RE::NiPoint3 leftGripWorld{};
    };

    // Frame-local authored support-seat diagnostics. Both palm points are
    // expressed in the exact current Weapon frame; world values exist only
    // for renderer visualization/readback.
    struct AuthoredSupportGripDebugSnapshot
    {
        static constexpr std::size_t kPoseLandmarkCount = 6;

        RE::NiTransform weaponWorld{};
        RE::NiPoint3 authoredPalmSeatWeaponLocal{};
        RE::NiPoint3 authoredPalmSeatWorld{};
        RE::NiPoint3 liveTouchProbeWeaponLocal{};
        RE::NiPoint3 liveTouchProbeWorld{};
        RE::NiPoint3 supportSideAxisWorld{};
        RE::NiPoint3 downAxisWorld{};
        RE::NiPoint3 referenceAxisWorld{};
        RE::NiPoint3 approachDirectionWorld{};
        std::array<RE::NiPoint3, kPoseLandmarkCount> poseLandmarksWorld{};
        std::array<RE::NiPoint3, kPoseLandmarkCount> poseSurfaceWitnessWorld{};
        std::array<float, kPoseLandmarkCount> poseSurfaceDistanceGameUnits{};
        float weaponRelativeDistanceGameUnits{ 0.0f };
        float worldReadbackDistanceGameUnits{ 0.0f };
        float frameAgreementErrorGameUnits{ 0.0f };
        float touchRadiusGameUnits{ 0.0f };
        float radialCapGameUnits{ 0.0f };
        float supportSideDot{ -1.0f };
        float downDot{ -1.0f };
        float sweptArcDot{ -1.0f };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t captureSequence{ 0 };
        std::uint32_t weaponFormID{ 0 };
        std::uint32_t effectiveEquipSlotFormID{ 0 };
        std::uint32_t baseEquipSlotFormID{ 0 };
        authored_weapon_grip_activation_policy::WeaponFamily weaponFamily{
            authored_weapon_grip_activation_policy::WeaponFamily::Unknown
        };
        authored_weapon_grip_activation_policy::HandTopology handTopology{
            authored_weapon_grip_activation_policy::HandTopology::Invalid
        };
        authored_weapon_grip_activation_policy::ActivationRegion selectedRegion{
            authored_weapon_grip_activation_policy::ActivationRegion::None
        };
        std::uint8_t poseSurfaceWitnessMask{ 0 };
        std::uint8_t poseSurfaceWitnessCount{ 0 };
        bool supportHandIsLeft{ true };
        bool insideTouchRadius{ false };
        bool effectiveEquipSlotUsesInstanceData{ false };
        bool classifierSupported{ false };
        bool canonicalAxesValid{ false };
        bool directionUsedLastStableSample{ false };
        bool radialPass{ false };
        bool directionPass{ false };
        bool topologyPass{ false };
        bool activationSpatialPass{ false };
        bool poseEvidenceEvaluated{ false };
        bool poseEvidencePass{ false };
        authored_support_grab_policy::Capability authoredCapability{
            authored_support_grab_policy::Capability::Pending
        };
        authored_support_grab_policy::CapabilityReason
            authoredCapabilityReason{
                authored_support_grab_policy::CapabilityReason::AwaitingIdentity
        };
        authored_support_grab_policy::Selection lastSelection{
            authored_support_grab_policy::Selection::None
        };
        authored_support_grab_policy::SelectionReason lastSelectionReason{
            authored_support_grab_policy::SelectionReason::None
        };
        float authoredCapabilityReadySeconds{ 0.0f };
        bool currentSupportGripActive{ false };
        bool currentAuthoredSupportGripActive{ false };
        bool valid{ false };
    };

    enum class NativeScopeCameraWriteSource : std::uint8_t
    {
        None,
        PostFrikPresentationSync,
        WeaponVisualAuthority,
    };

    /*
     * Main-thread diagnostic record for the native scope camera handoff. It
     * deliberately stores transform values instead of engine pointers so the
     * renderer-side overlay cannot retain a transient scene-graph reference.
     */
    struct NativeScopeCameraDebugSnapshot
    {
        std::uint64_t applySequence{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t framesSinceApply{ 0xFFFF'FFFFu };
        NativeScopeCameraWriteSource writeSource{ NativeScopeCameraWriteSource::None };
        bool captureValid{ false };
        bool targetValid{ false };
        bool writeApplied{ false };
        bool immediateReadbackValid{ false };
        native_scope_sight_anchor_policy::AnchorSource anchorSource{
            native_scope_sight_anchor_policy::AnchorSource::None
        };
        RE::NiTransform cameraWorldBefore{};
        RE::NiTransform targetCameraWorld{};
        RE::NiTransform immediateCameraWorldAfter{};
    };

    /*
     * Read-only view of the exact generation-bound camera target ROCK would
     * publish from the current weapon transform. The debug overlay resolves
     * this retained frame without touching the live camera, so ScopeMenu does
     * not need to be open while tuning fallback position or rotation.
     */
    struct NativeScopeCameraTargetPreviewSnapshot
    {
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t equippedWeaponOwnershipKey{ 0 };
        std::uint32_t weaponFormID{ 0 };
        native_scope_sight_anchor_policy::AnchorSource anchorSource{
            native_scope_sight_anchor_policy::AnchorSource::None
        };
        RE::NiTransform cameraWeaponLocal{};
        bool valid{ false };
    };

    struct NativeScopeResolvedAnchorSnapshot
    {
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t equippedWeaponOwnershipKey{ 0 };
        std::uint32_t weaponFormID{ 0 };
        RE::NiPoint3 anchorWeaponLocal{};
        native_scope_sight_anchor_policy::AnchorSource source{
            native_scope_sight_anchor_policy::AnchorSource::None
        };
        bool valid{ false };
    };

    struct NativeScopeActivationDebugSnapshot
    {
        std::uint64_t publicationSequence{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        native_scope_sight_anchor_policy::AnchorSource anchorSource{
            native_scope_sight_anchor_policy::AnchorSource::None
        };
        bool manualInputRequested{ false };
        bool rendererStateValid{ false };
        bool rendererActive{ false };
    };

    struct SelectedAuthoredGripPoseSnapshot
    {
        enum class Source : std::uint8_t
        {
            Unknown = 0,
            LiveEquippedGraph = 1,
            NativeIdlePreharvest = 2,
            RuntimeCanonical = 3,
        };

        bool valid{ false };
        bool rightHandValid{ false };
        bool leftHandValid{ false };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t variantKey{ 0 };
        std::uint64_t captureSequence{ 0 };
        Source source{ Source::Unknown };
        RE::NiTransform rightHandWeaponLocal{};
        RE::NiTransform leftHandWeaponLocal{};
        std::array<RE::NiTransform, 15> rightFingerLocalTransforms{};
        std::array<RE::NiTransform, 15> leftFingerLocalTransforms{};
        std::uint16_t rightFingerLocalTransformMask{ 0 };
        std::uint16_t leftFingerLocalTransformMask{ 0 };
    };

    struct NativeScopeRigidFrameState
    {
        std::uint64_t weaponGenerationKey{ 0 };
        RE::NiNode* weaponNodeIdentity{ nullptr };
        RE::NiNode* scopeCameraIdentity{ nullptr };
        // Immutable native axis/scale calibration captured before ROCK writes.
        RE::NiTransform nativeCameraWeaponLocal{};
        // Resolved anchor plus any fallback-only weapon-axis correction.
        RE::NiTransform cameraWeaponLocal{};
        bool valid{ false };
    };

    struct NativeScopeOverlayCalibrationState
    {
        std::uint64_t weaponGenerationKey{ 0 };

        /*
         * Non-owning identity witnesses only. They are never dereferenced
         * unless the current PlayerNodes hierarchy resolves the same nodes.
         */
        RE::NiNode* scopeParentIdentity{ nullptr };
        RE::NiNode* scopeModelRootIdentity{ nullptr };

        RE::NiTransform scopeModelRootLocal{};
        RE::NiTransform scopeModelRootCalibrationInCameraLocal{};
        RE::NiTransform nativeScopeParentLocal{};
        RE::NiTransform lastAppliedScopeParentLocal{};
        bool valid{ false };
        bool hasAppliedLocal{ false };
    };

    struct EquippedWeaponManualDropRequest
    {
        bool requested{ false };
        equipped_weapon_drop_policy::SourceHand sourceHand{ equipped_weapon_drop_policy::SourceHand::None };
    };

    /*
     * One-shot grip transition events for controller haptics. TwoHandedGrip
     * never talks to the VR controllers directly; PhysicsInteraction consumes
     * these once per frame right after update() and queues the pulses.
     */
    struct TwoHandedGripHapticEvents
    {
        bool firingGripAttached{ false };
        bool firingGripAttachedHandIsLeft{ false };
        bool firingGripDetached{ false };
        bool firingGripDetachedHandIsLeft{ false };
        bool leftPartGripCaptured{ false };
        bool rightPartGripCaptured{ false };
    };

    /*
     * Value snapshot of what one physical hand currently holds on the equipped
     * weapon, published to the provider API each frame. sourceRoot is a
     * non-owning engine pointer valid only within the reported
     * weaponGenerationKey; handPartLocal is the hand frame captured at grip
     * start, in part-source-local space when handPartLocalIsSourceLocal is set
     * and weapon-root-local space otherwise.
     */
    struct HandGripReport
    {
        weapon_part_grip_report_policy::HandGripKind kind{ weapon_part_grip_report_policy::HandGripKind::None };
        bool active{ false };
        bool attachOnly{ false };
        bool authoredSupportGrip{ false };
        std::uint64_t gripSequence{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFFu };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        std::uint64_t providerOwnerToken{ 0 };
        std::uint32_t providerGroupId{ 0 };
        std::uint32_t providerGrabMode{ 0 };
        bool hasHandPartLocal{ false };
        bool handPartLocalIsSourceLocal{ false };
        RE::NiTransform handPartLocal{};
        std::array<char, kWeaponProviderSourceNameCapacity> sourceName{};
        // Record-authored identity of the gripped part (0 when unpaired).
        std::uint32_t omodFormId{ 0 };
        std::uint32_t attachPointFormId{ 0 };
        std::uint32_t classificationSource{ 0 };
    };

    class TwoHandedGrip
    {
    public:
        using WeaponVisualIntentObserver = void (*)(
            void* context,
            RE::NiNode* weaponNode,
            const RE::NiTransform& requestedWeaponWorld,
            std::uint64_t weaponGenerationKey);

        TwoHandedGrip();
        ~TwoHandedGrip();

        TwoHandedGrip(const TwoHandedGrip&) = delete;
        TwoHandedGrip& operator=(const TwoHandedGrip&) = delete;
        TwoHandedGrip(TwoHandedGrip&&) = delete;
        TwoHandedGrip& operator=(TwoHandedGrip&&) = delete;

        TwoHandedGripUpdateResult update(
            RE::NiNode* weaponNode,
            const WeaponInteractionContact& leftWeaponContact,
            const WeaponInteractionContact& rightWeaponContact,
            const EquippedWeaponGripFrameInput& frameInput,
            float dt,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            const WeaponCollision& weaponCollision,
            const WeaponInteractionRuntimeState& leftRuntimeState,
            const WeaponInteractionRuntimeState& rightRuntimeState,
            weapon_support_authority_policy::WeaponSupportAuthorityMode supportAuthorityMode,
            bool firingGripProximityAuthorityEnabled,
            const EquippedWeaponHandlingSettings& handlingSettings);

        [[nodiscard]] EquippedWeaponGripOccupancy
            getGripOccupancy() const noexcept;

        [[nodiscard]] AuthoredSupportGripIndicatorFrame
            getAuthoredSupportGripIndicatorFrame() const noexcept
        {
            return _support.authoredIndicatorFrame;
        }

        // Marker for the firing-grip reattach zone: visible while an open
        // free palm hovers inside a lateral cylinder during part carry.
        [[nodiscard]] FiringGripReattachIndicatorFrame
            getFiringGripReattachIndicatorFrame() const noexcept
        {
            return _firing.reattachIndicatorFrame;
        }

        /*
         * Called after hFRIK's weapon pass. FO4VR has already completed its
         * activation update earlier in PlayerCharacter::Update; this method
         * synchronizes the mono camera and native presentation to the current
         * weapon before the later render pass.
         */
        void synchronizeNativeScopePresentationAfterFrikUpdate(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey);

        /*
         * Baseline one-hand calibration derived from Bethesda's native
         * primary grip. This intentionally reuses the complete equipped-
         * weapon visual path so the scope camera and later manual-grip state
         * observe the same corrected frame. It fails closed while a ROCK
         * conflicting weapon-transform authority or its return transition
         * owns the node.
         */
        bool applyAuthoredPrimaryGripWeaponAlignment(
            RE::NiNode* weaponNode,
            const RE::NiTransform& solvedWeaponWorld,
            const RE::NiTransform& solvedFiringHandWorld,
            std::uint64_t currentWeaponGenerationKey);

        /*
         * Position-only frame contract: the authored runtime must republish
         * the right firing-hand world every frame it stays active. A frame
         * that ends without a refresh releases the hand back to hFRIK.
         */
        void beginAuthoredPrimaryFiringGripFrame();
        void finishAuthoredPrimaryFiringGripFrame();

        /*
         * Controller intent for the position-only authored solve. Resolves
         * the physical damped-driver wrist frame; once ROCK owns the right
         * firing-hand presentation it fails closed instead of reading the
         * presented hand back as solver input.
         */
        bool tryGetAuthoredPrimaryTrackedFiringHandWorld(
            RE::NiTransform& outHandWorld) const;

        /*
         * Physical hand frame for consumers outside this session that must
         * not read ROCK's presented hand back: hFRIK's damped driver composed
         * with the natural bone relation cached while the hand was native.
         * Unavailable until that relation exists; never falls back to the
         * rendered bone, which is ROCK's own output while a support lock,
         * part carry, or authored seat presents the hand.
         */
        bool tryGetPhysicalHandWorld(
            bool isLeft,
            RE::NiTransform& outHandWorld) const;

        /*
         * Authored seat on the live weapon while a position-only carry will
         * resume; the right-hand visual return targets it so the hand lands
         * where the session re-seats it instead of flashing the physical
         * wrist first.
         */
        bool tryResolveAuthoredPositionOnlySeatWorld(
            RE::NiTransform& outHandWorld) const;

        /*
         * Binds Bethesda's exact RArm_Hand-in-Weapon relation to ROCK's
         * existing firing canonical by node identity, collision generation,
         * and equipped ownership. When a complete firing-finger pose is
         * available, the right animation locals and hFRIK's anatomy-correct
         * left mirror are retained only for the matching weapon identity.
         * Generation zero is a provisional equip-only publication: it can
         * preserve the exact firing pose before collision is ready, but the
         * manual grip paths reject it until a nonzero generation is bound.
         * The node pointer is comparison-only and is never dereferenced after
         * publication.
         */
        bool setAuthoredPrimaryFiringGripCanonical(
            RE::NiNode* weaponNode,
            const RE::NiTransform& rightHandWeaponLocal,
            std::uint64_t weaponGenerationKey,
            std::uint64_t weaponOwnershipKey,
            std::uint64_t captureSequence,
            const authored_weapon_grip_library::FiringFingerPose* rightFingerPose = nullptr,
            const authored_weapon_grip_library::FiringFingerPose* leftFingerPose = nullptr);
        void clearAuthoredPrimaryFiringGripCanonical(const char* reason);
        bool publishAuthoredPrimaryFiringGripFingerPose(bool isLeft);
        bool retainAuthoredPrimaryFiringGripFingerPoseForHandoff(
            RE::NiNode* weaponNode,
            std::uint64_t weaponGenerationKey,
            std::uint64_t weaponOwnershipKey);
        [[nodiscard]] bool hasPublishedAuthoredPrimaryFiringGripFingerPose(const bool isLeft) const noexcept
        {
            return _firing.authoredFingerPosePublished && _firing.publishedFingerPoseIsLeft == isLeft;
        }
        void clearAuthoredPrimaryFiringGripFingerPose();
        void setAuthoredPrimaryFiringGripFingerPoseSuppressed(bool suppressed);
        void setGrabbedObjectHandPoseOwnership(
            bool leftHandHoldingObject,
            bool rightHandHoldingObject);

        /*
         * Ephemeral pre-update candidate derived from Bethesda's paired
         * support-arm pass. AuthoredPrimaryFiringGripRuntime clears it at the
         * start of every frame and republishes one identity/generation-bound
         * relation. During physical-left firing, that relation comes from the
         * last topology-valid value snapshot captured before ROCK reparented
         * Weapon. capturePartGrip latches it only during acquisition; it never
         * changes an already-active grip.
         */
        void clearAuthoredSupportGripCandidate();
        [[nodiscard]] authored_support_grab_policy::
            LeftFiringTakeoverReadiness
            getLeftFiringTakeoverReadiness(
                RE::NiNode* weaponNode,
                std::uint64_t authoredGenerationKey,
                std::uint64_t weaponOwnershipKey,
                bool authoredOnlyModeEnabled) const noexcept;
        bool setAuthoredSupportGripCandidate(
            RE::NiNode* weaponNode,
            const RE::NiTransform& handWeaponLocal,
            const std::array<RE::NiTransform, 15>& fingerLocalTransforms,
            std::uint16_t fingerLocalTransformMask,
            std::uint64_t weaponGenerationKey,
            std::uint64_t captureSequence);

        void reset();

        void setWeaponVisualIntentObserver(
            void* context,
            WeaponVisualIntentObserver observer)
        {
            _visuals.weaponIntentObserverContext = context;
            _visuals.weaponIntentObserver = observer;
        }

        // Captures which hands retained collision presentation through the
        // previous render interval, then releases those tags. FRIK has already
        // updated the current skeleton, so input selection uses that witness to
        // avoid reading the collision-corrected roots back as physical intent.
        void beginWeaponCollisionPresentationFrame();

        bool previousWeaponCollisionPresentationWasLive() const
        {
            return _visuals.weaponCollisionHandPresentationFromPreviousFrame[0] ||
                   _visuals.weaponCollisionHandPresentationFromPreviousFrame[1];
        }

        // Republishes a physics-resolved visual pose without feeding that
        // correction back into the next dynamic-weapon drive target.
        bool applyWeaponCollisionResolvedAuthority(
            RE::NiNode* weaponNode,
            const RE::NiTransform& resolvedWeaponWorld,
            std::uint64_t authorityGenerationKey);

        bool isGripping() const { return _session.state == TwoHandedState::Gripping || _session.state == TwoHandedState::PartCarry; }

        bool isManualOwnershipActive() const
        {
            return _session.state == TwoHandedState::Gripping ||
                   _session.state == TwoHandedState::PartCarry ||
                   _session.state == TwoHandedState::PrimaryOnly;
        }

        bool isWeaponVisualReturnActive() const;

        bool isPartCarryActive() const { return _session.state == TwoHandedState::PartCarry; }

        bool isPrimaryOnlyActive() const { return _session.state == TwoHandedState::PrimaryOnly; }

        bool isHandPartGripping(bool isLeft) const { return partGrip(isLeft).active; }

        /*
         * True only for part grips that hold weapon transform authority.
         * AttachOnly glue grips report isHandPartGripping (the hand is
         * occupied) but never count as a carry anchor.
         */
        bool isHandPartCarryGripping(bool isLeft) const
        {
            const WeaponPartGrip& grip = partGrip(isLeft);
            return weapon_part_grip_report_policy::partGripCountsAsCarry(grip.active, grip.attachOnly);
        }

        bool isFiringGripOccupied() const { return _session.state == TwoHandedState::Gripping || _session.state == TwoHandedState::PrimaryOnly; }

        /*
         * True while an OPEN free palm hovers inside the firing-grip reattach
         * zone during part carry: squeezing the grab right now would re-take
         * the firing grip. Recomputed every update(); PhysicsInteraction
         * consumes it each frame to drive continuous hover haptics on that
         * hand.
         */
        bool isFiringGripReattachHoverInsideZone() const { return _firing.reattachHoverInsideZone; }

        // Which physical hand the hover above refers to (either free hand can
        // hover the firing grip when ambidextrous takeover is available).
        bool isFiringGripReattachHoverHandLeft() const { return _firing.reattachHoverHandIsLeft; }

        bool canUseFiringGripInput() const
        {
            return _session.state == TwoHandedState::Gripping ||
                   _session.state == TwoHandedState::PartCarry ||
                   _session.state == TwoHandedState::PrimaryOnly;
        }

        bool isTouching() const { return _session.state == TwoHandedState::Touching; }

        bool isFiringHandLeft() const { return _session.firingHandIsLeft; }
        [[nodiscard]] std::uint64_t nativeRecoilKickSequence() const noexcept { return _leftCarry.nativeRecoilKickSequence; }

        /*
         * True while a manual left-firing carry has published a solved weapon
         * pose in the latest grip update. The equip visual bridge keys its
         * rotation carrier on this: before the first solve, the native weapon
         * root still holds the right-hand glue or draw-animation orientation.
         */
        [[nodiscard]] bool hasSolvedLeftFiringCarryPose() const noexcept
        {
            return _session.firingHandIsLeft &&
                   isManualOwnershipActive() &&
                   _hasSolvedWeaponTransform;
        }

        /*
         * Latest solved LEFT-carry weapon world - the pose the weapon node is
         * actually rendered at. The equip visual bridge must use this as its
         * rotation carrier instead of sampling the node early in the frame:
         * FRIK re-glues the node to the RIGHT hand before ROCK runs, so an
         * early-frame read returns the right-glue orientation (~180 degrees
         * from the mirrored left carry) even while the carry owns the node.
         */
        [[nodiscard]] bool tryGetSolvedLeftFiringWeaponWorld(
            RE::NiTransform& outWeaponWorld) const noexcept
        {
            if (!hasSolvedLeftFiringCarryPose()) {
                return false;
            }
            outWeaponWorld = _lastSolvedWeaponTransform;
            return true;
        }

        TwoHandedState getState() const { return _session.state; }

        weapon_support_authority_policy::WeaponSupportAuthorityMode getAuthorityMode() const { return _session.authorityMode; }

        bool ownsWeaponTransform() const;

        /*
         * The manual-ownership state machine also tracks right-hand
         * PrimaryOnly input ownership, but that state deliberately leaves the
         * weapon transform with hFRIK. Authored calibration must continue to
         * correct that native carry. Only a ROCK weapon solve or left-firing
         * topology conflicts with the authored right-hand alignment.
         */
        bool blocksAuthoredPrimaryGripWeaponAlignment() const;

        bool getSolvedWeaponTransform(RE::NiTransform& outTransform) const;

        /*
         * Returns the exact ROCK-authored right firing-hand and left support-
         * hand targets in the final solved Weapon frame. Native cycle motion
         * uses these as its neutral pose instead of reading hFRIK's residual
         * IK result back from the skeleton.
         */
        bool getManualCycleRockGripBaselines(
            RE::NiTransform& outRightHandInWeapon,
            RE::NiTransform& outLeftHandInWeapon) const;

        bool getDebugAuthoritySnapshot(TwoHandedGripDebugSnapshot& outSnapshot) const;

        bool getAuthoredSupportGripDebugSnapshot(
            AuthoredSupportGripDebugSnapshot& outSnapshot) const;

        bool getFiringGripReattachZoneDebugSnapshot(
            FiringGripReattachZoneDebugSnapshot& outSnapshot) const;

        NativeScopeCameraDebugSnapshot getNativeScopeCameraDebugSnapshot() const { return _scope.cameraDebugSnapshot; }
        NativeScopeCameraTargetPreviewSnapshot
            getNativeScopeCameraTargetPreviewSnapshot() const;
        NativeScopeActivationDebugSnapshot getNativeScopeActivationDebugSnapshot() const { return _scope.activationDebugSnapshot; }
        NativeScopeResolvedAnchorSnapshot getNativeScopeResolvedAnchorSnapshot() const
        {
            return NativeScopeResolvedAnchorSnapshot{
                .weaponGenerationKey = _scope.anchorGenerationKey,
                .equippedWeaponOwnershipKey = _scope.anchorOwnershipKey,
                .weaponFormID = _scope.anchorWeaponFormID,
                .anchorWeaponLocal = _scope.anchorWeaponLocal,
                .source = _scope.anchorSource,
                .valid = _scope.anchorValid,
            };
        }
        bool getSelectedAuthoredGripPoseSnapshot(
            SelectedAuthoredGripPoseSnapshot& outSnapshot) const;

        bool isScopeMenuOpenThisFrame() const { return _scope.menuOpenThisFrame; }

        bool hasVisualAuthorityForHand(bool isLeft) const;
        bool isHandVisualReturnActive(bool isLeft) const;
        void cancelHandVisualReturn(bool isLeft, const char* reason);

        bool beginPrimaryOnlyGrip(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            bool firingHandIsLeft,
            const RE::NiTransform* capturedFiringHandWeaponLocal,
            const RE::NiPoint3* capturedFiringGripWeaponLocal,
            bool retainUntilPhysicalGrip = false,
            bool emitAttachHaptic = true);

        /*
         * A persistent carry is a PrimaryOnly session that survives without
         * a physical grab hold (started by beginPrimaryOnlyGrip with
         * retainUntilPhysicalGrip, e.g. after a committed equip transfer or
         * shoulder retrieval) until the selected stack is unequipped or the
         * player deliberately acquires and releases the firing-hand grab.
         */
        bool commitPersistentEquippedCarryInputAcquisition(
            bool handIsLeft) noexcept;

        void clearPersistentEquippedCarry(const char* reason);
        void restoreNativeRightEquippedCarry(const char* reason);
        bool isPersistentEquippedCarryInputAcquisitionPending() const
        {
            return _firing.persistentCarryInputAcquisitionPending;
        }

        // Left-hand primary ownership requires ROCK's hFRIK weapon-pose and
        // node-ownership blockers; right-hand native ownership is always
        // eligible. Native Fallout/FRIK handedness is deliberately irrelevant.
        static bool canBeginPrimaryOnlyGripForHand(bool isLeft);

        bool tryBuildCurrentLeftFiringGripCapture(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            RE::NiTransform& outFiringHandWeaponLocal,
            RE::NiPoint3& outFiringGripWeaponLocal);

        /*
         * Captures a left-hand transfer frame before a native transition can
         * clear manual ownership or retire the current collision generation.
         * An active left carry is copied directly; native-right carry falls
         * back to the canonical mirrored frame for the same equipped owner.
         */
        bool tryCaptureLeftFiringGripTransfer(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            RE::NiTransform& outFiringHandWeaponLocal,
            RE::NiPoint3& outFiringGripWeaponLocal);

        /*
         * Legacy loose-model hold resolver. It mirrors the canonical right
         * hand relation and applies the effective left-aim trim because loose
         * weapons have no native equipped weapon-in-wand baseline. Equipped
         * carry uses the untrimmed authored-seat variant privately and applies
         * aim trim to the separately mirrored native weapon orientation.
         */
        static bool tryBuildMirroredLeftFiringHandWeaponLocal(
            const RE::NiTransform& canonicalRightHandWeaponLocal,
            const RE::NiPoint3& firingGripWeaponLocal,
            const RE::NiTransform& rightHandWorld,
            const RE::NiTransform& leftHandWorld,
            RE::NiTransform& outHandWeaponLocal,
            bool logDiagnostic = false);

        /*
         * Publishes the left-firing canonical carry pose (firing hand o
         * inverse(captured hold)) onto the weapon node. While ROCK owns the
         * node, FRIK's earlier pass leaves it at its OFFHAND GLUE pose, so
         * any world<->node-local math run before this publish operates in
         * glue space. update() calls it internally before its grip math;
         * PhysicsInteraction MUST also call it before the frame's weapon
         * interaction probes (ranked part selection converts the real palm
         * point into node-local space - glue space made a forend grab pick
         * the scope's sight body ~10gu away). Safe pre-update: it reads the
         * previous frame's scope-safe hand frame, a millimeter-scale error
         * against the ~10gu glue displacement it removes. No-op unless
         * left-firing with a valid captured hold on the current weapon.
         */
        bool publishLeftFiringFeedForwardWeaponPose(RE::NiNode* weaponNode);

        /*
         * FRIK re-attaches the weapon node to the firing hand every frame
         * before ROCK runs, even while the firing hand is detached. Callers
         * must republish ROCK's solved part-carry transform before reading the
         * weapon node (probes, grip-zone checks, capture frames), or every
         * weapon-relative computation sees the weapon glued to the firing hand.
         */
        bool republishPartCarryWeaponTransform(RE::NiNode* weaponNode);

        EquippedWeaponManualDropRequest consumeEquippedWeaponDropRequest();

        /*
         * Per-hand grip report for the provider API. Always succeeds; an idle
         * hand reports kind None. Main-thread only (reads live grip state).
         */
        void getHandGripReport(bool isLeft, HandGripReport& outReport) const;

        TwoHandedGripHapticEvents consumeHapticEvents();

        /*
         * Shared raw palm capture for external grip-point consumers. Outside
         * ScopeMenu this uses the same root-flattened frame and palm-pivot
         * formula as the internal grip capture. Internal two-hand authority
         * switches to its hFRIK-arm-driver-relative cached frame while hFRIK
         * hides that root in ScopeMenu.
         */
        static bool tryCaptureRootFlattenedPalmWorld(bool isLeft, RE::NiPoint3& outPalmWorld, RE::NiTransform& outHandWorld);

    private:
        struct FingerPoseSolveScratch;

        static bool FRIK_CALL controlWeaponHandRecoil(
            const frik::api::FRIKApiV2::RecoilSample* sample,
            frik::api::FRIKApiV2::RecoilResponse* outResponse,
            void* userData) noexcept;

        [[nodiscard]] bool hasVisualOnlySupportRecoilAssist() const noexcept;

        void captureLeftFiringWeaponRecoil(
            const RE::NiTransform& controlledKickLocal) noexcept;
        bool applyLeftFiringWeaponRecoil(RE::NiNode* weaponNode);

        struct LockedHandVisualLerpState
        {
            // The start relation remains initialized for the grip lifetime
            // after interpolation reaches alpha 1. This prevents the tracked
            // hand from restarting acquisition every frame.
            bool initialized = false;
            RE::NiTransform startWorld{};
            float elapsedSeconds = 0.0f;
            float durationSeconds = 0.0f;
            float lastAlpha = 1.0f;
        };

        /*
         * Value-only acquisition transaction for a normal, non-authored
         * support grip. It owns no scene/provider pointers: generation and
         * grip-sequence witnesses make every later use fail closed if the
         * weapon or captured part changes.
         */
        struct DynamicSupportAcquisitionState
        {
            bool active{ false };
            bool durationInitialized{ false };
            bool firstPublicationRecorded{ false };
            bool supportHandIsLeft{ false };
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint64_t gripSequence{ 0 };
            RE::NiTransform primaryStartWorld{};
            RE::NiTransform supportStartWorld{};
            float elapsedSeconds{ 0.0f };
            float durationSeconds{ 0.0f };
            float rawAlpha{ 0.0f };
            float easedAlpha{ 0.0f };
            float fullCorrectionRadians{ 0.0f };
            float axisCorrectionRadians{ 0.0f };
            float twistContributionRadians{ 0.0f };
            float initialSeatDistanceGameUnits{ 0.0f };
            float lastAppliedRotationRadians{ 0.0f };
            float lastPrimaryPivotError{ 0.0f };
            float lastSupportTargetError{ 0.0f };
        };

        struct ReturningHandVisualState
        {
            hand_visual_lerp_math::VisualReturnTransition<RE::NiTransform> transition{};
        };

        struct ReturningWeaponVisualState
        {
            hand_visual_lerp_math::VisualReturnTransition<RE::NiTransform> localTransition{};
            RE::NiNode* weaponNode{ nullptr };
            RE::NiNode* nativeParent{ nullptr };
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint64_t equippedWeaponOwnershipKey{ 0 };
            RE::NiTransform nativeBaselineLocal{};
            RE::NiTransform lastTargetLocal{};
            bool retainPrimaryPoseBlocker{ false };
            bool followsAuthoredPrimaryGrip{ false };
        };

        /*
         * Residual blend from the last rendered weapon pose into a replacement
         * solve (firing-grip detach into part carry, reattach into the
         * two-hand solve). Armed by the transition, captured by the first
         * solved publication, value-only: the generation witness fails it
         * closed when the weapon changes underneath.
         */
        struct WeaponPoseHandoffBlendState
        {
            hand_visual_lerp_math::VisualReturnTransition<RE::NiTransform> residual{};
            std::uint64_t weaponGenerationKey{ 0 };
            bool armed{ false };
        };

        struct ScopeSafeHandFrameDiagnostic
        {
            RE::NiTransform rootHandWorld{};
            RE::NiTransform driverWorld{};
            RE::NiTransform reconstructedHandWorld{};
            RE::NiTransform currentHandWorld{};
            scope_safe_hand_frame_math::ResolutionMode resolutionMode{
                scope_safe_hand_frame_math::ResolutionMode::Unavailable
            };
            std::uint32_t consecutiveDriverMissFramesBefore{ 0 };
            std::uint32_t consecutiveDriverMissFramesAfter{ 0 };
            bool rootSampleAllowed{ false };
            bool rootHandValid{ false };
            bool driverNodeAvailable{ false };
            bool driverWorldFinite{ false };
            bool driverFrameValid{ false };
            bool driverWorldUsable{ false };
            bool driverToHandLocalAvailable{ false };
            bool reconstructedHandValid{ false };
            bool lastHandWorldAvailable{ false };
            bool collisionPresentationWasLive{ false };
            bool scopeDriverFrameAuthorityActive{ false };
            bool physicalFrameOverrideApplied{ false };
            bool currentHandWorldValid{ false };
        };

        struct ScopeSafeHandFrameState
        {
            RE::NiTransform driverToHandLocal{};
            RE::NiTransform currentHandWorld{};
            RE::NiTransform lastHandWorld{};
            RE::NiTransform rootRebaseLocalStart{};
            float rootRebaseElapsedSeconds{ 0.0f };
            std::uint32_t consecutiveDriverMissFrames{ 0 };
            bool hasDriverToHandLocal{ false };
            bool currentHandWorldValid{ false };
            bool hasLastHandWorld{ false };
            bool rootRebaseActive{ false };
            ScopeSafeHandFrameDiagnostic diagnostic{};
        };

        enum class LockedHandAuthorityRole : std::uint8_t
        {
            None,
            PrimaryGrip,
            SupportGrip,
        };

        struct LockedHandAuthorityAttemptDiagnostic
        {
            RE::NiTransform targetWorld{};
            RE::NiTransform liveWorld{};
            LockedHandAuthorityRole role{ LockedHandAuthorityRole::None };
            bool requested{ false };
            bool bridgeAvailable{ false };
            bool targetUsable{ false };
            bool liveWorldAvailable{ false };
            bool liveWorldUsable{ false };
            bool applied{ false };
        };

        struct GripFailureFrameSnapshot
        {
            std::array<ScopeSafeHandFrameDiagnostic, 2> handFrames{};
            std::array<LockedHandAuthorityAttemptDiagnostic, 2>
                authorityAttempts{};
            RE::NiTransform weaponWorld{};
            RE::NiPoint3 hmdPositionWorld{};
            RE::NiPoint3 playerSpaceDeltaGameUnits{};
            EquippedWeaponPrimaryGripInput leftPhysicalGripInput{};
            EquippedWeaponPrimaryGripInput rightPhysicalGripInput{};
            EquippedWeaponPrimaryGripInput primaryLogicalInput{};
            EquippedWeaponPrimaryGripInput primaryDebouncedInput{};
            std::uint64_t frameIndex{ 0 };
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint64_t equippedWeaponOwnershipKey{ 0 };
            std::uint64_t supportGripSequence{ 0 };
            std::uint32_t weaponFormID{ 0 };
            std::uint8_t primaryReleaseOpenFrames{ 0 };
            float deltaSeconds{ 0.0f };
            TwoHandedState state{ TwoHandedState::Inactive };
            weapon_support_authority_policy::WeaponSupportAuthorityMode
                authorityMode{
                    weapon_support_authority_policy::
                        WeaponSupportAuthorityMode::FullTwoHandedSolver
                };
            bool firingHandIsLeft{ false };
            bool leftGripHeld{ false };
            bool rightGripHeld{ false };
            bool leftHandHoldingObject{ false };
            bool rightHandHoldingObject{ false };
            bool toggleGrabEnabled{ false };
            bool animationBoundaryActive{ false };
            bool scopeMenuOpen{ false };
            bool manualScopeActivationRequested{ false };
            bool nativeScopeRequestStateValid{ false };
            bool nativeScopeRequestActive{ false };
            bool hasHmdFrame{ false };
            bool visualAuthorityAvailable{ false };
            bool weaponWorldValid{ false };
        };

        static constexpr std::size_t kGripFailureHistoryCapacity = 30;

        enum class SupportInputBaselineKind : std::uint8_t
        {
            None = 0,
            Dynamic = 1,
            PartCarry = 2,
        };

        struct SupportInputBaselineState
        {
            RE::NiTransform inputToGripTargetLocal{};
            // PartCarry only: direct raw-driver-to-weapon relation avoids
            // inverting the authored wrist frame during the one-anchor solve.
            RE::NiTransform inputToWeaponLocal{};
            RE::NiTransform primaryInputToGripTargetLocal{};
            RE::NiTransform weaponWorldAtCapture{};
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint64_t equippedWeaponOwnershipKey{ 0 };
            std::uint64_t gripSequence{ 0 };
            bool supportHandIsLeft{ false };
            SupportInputBaselineKind kind{ SupportInputBaselineKind::None };
            bool active{ false };
            bool pairedDynamicDrivers{ false };
            bool firstPublicationPending{ false };
            /*
             * 0 -> 1 ramp that retargets the calibrated support solver input
             * from the captured tandem-delta target onto the true physical
             * support hand, closing the controller-to-seat gap frozen in at
             * capture. Stays 0 on the attach frame so the first publication
             * cannot move the weapon. PartCarry intentionally leaves this at
             * zero for the entire detached carry.
             */
            float alignmentBlend{ 0.0f };
        };

        /*
         * One hand's captured part grip on the equipped weapon. Both
         * weapon-root-local and dynamic part-source-local frames are stored so the
         * grip survives moving mod parts. attachmentRoot is a non-owning engine
         * pointer and must be validated against the current weapon tree
         * (resolveCurrentSupportAttachmentRoot) before every dereference.
         */
        struct WeaponPartGrip
        {
            bool active{ false };
            RE::NiPoint3 gripLocal{};
            RE::NiPoint3 grabNormalWorld{};
            RE::NiPoint3 normalLocal{};
            RE::NiPoint3 gripSourceLocal{};
            RE::NiPoint3 normalSourceLocal{};
            RE::NiTransform handWeaponLocal{};
            RE::NiTransform handSourceLocal{};
            RE::NiTransform attachmentWeaponLocal{};
            bool hasHandWeaponLocal{ false };
            bool hasSourceFrames{ false };
            bool hasAttachmentWeaponLocal{ false };
            RE::NiAVObject* attachmentRoot{ nullptr };
            WeaponGripPoseId gripPose{ WeaponGripPoseId::BarrelWrap };
            WeaponPartKind partKind{ WeaponPartKind::Other };
            WeaponProviderPartAuthority providerPartAuthority{};
            bool authoredSupportGrip{ false };
            // Both authored support topologies keep axis aiming and the
            // primary-anchored solve but never apply palm-normal twist.
            bool disableAuthoredSupportNormalTwist{ false };
            std::uint64_t authoredSupportCaptureSequence{ 0 };
            /*
             * AttachOnly glue: the hand stays visually attached to the part
             * (source frames survive part-carry so it follows provider-driven
             * part motion) but the grip never holds weapon pivot authority.
             * Resolved once at capture from the provider whitelist grab mode.
             */
            bool attachOnly{ false };
            /*
             * Contact identity captured at grip start for the provider API
             * grip report. Distinct from providerPartAuthority, which only
             * exists when a consumer whitelist matched the part.
             */
            std::uint32_t contactBodyId{ 0x7FFF'FFFFu };
            WeaponReloadRole reloadRole{ WeaponReloadRole::None };
            WeaponSupportGripRole supportRole{ WeaponSupportGripRole::None };
            WeaponSocketRole socketRole{ WeaponSocketRole::None };
            WeaponActionRole actionRole{ WeaponActionRole::None };
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint64_t gripSequence{ 0 };
            std::array<char, kWeaponProviderSourceNameCapacity> sourceName{};
            // Record-authored identity from the evidence descriptor at capture.
            std::uint32_t omodFormId{ 0 };
            std::uint32_t attachPointFormId{ 0 };
            WeaponPartClassificationSource classificationSource{ WeaponPartClassificationSource::NameToken };
            std::array<float, 15> fingerPose{};
            std::array<float, 5> fingerSplayRadians{};
            std::array<RE::NiTransform, 15> fingerLocalTransforms{};
            std::uint16_t fingerLocalTransformMask{ 0 };
            bool hasFingerPose{ false };
            bool hasFingerSplay{ false };
            bool hasFingerLocalTransforms{ false };
            LockedHandVisualLerpState visualLerp{};
            SupportInputBaselineState supportInputBaseline{};
            float surfaceSeatRotationRadians{ 0.0f };
        };

        struct AuthoredSupportGripCandidate
        {
            // Non-owning identity witness, valid only for the current ROCK
            // pre-update/update pair. It is never dereferenced after the
            // candidate is cleared or across a weapon-generation boundary.
            RE::NiNode* weaponNode{ nullptr };
            RE::NiTransform leftHandWeaponLocal{};
            RE::NiTransform rightHandWeaponLocal{};
            std::array<RE::NiTransform, 15> leftFingerLocalTransforms{};
            std::array<RE::NiTransform, 15> rightFingerLocalTransforms{};
            std::uint16_t leftFingerLocalTransformMask{ 0 };
            std::uint16_t rightFingerLocalTransformMask{ 0 };
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint64_t captureSequence{ 0 };
            bool rightMirrorValid{ false };
            bool valid{ false };
        };

        /*
         * Authored-only support acquisition cannot use the frame-scoped
         * candidate as capability authority: the native graph intentionally
         * invalidates and republishes that candidate around its arm passes.
         * This identity-bound state latches positive capture and qualifies
         * persistent absence in measured ready-state time. weaponNodeIdentity
         * is a non-owning witness and is never dereferenced from the cache.
         */
        struct AuthoredSupportCapabilityState
        {
            RE::NiNode* weaponNodeIdentity{ nullptr };
            std::uint64_t weaponOwnershipKey{ 0 };
            std::uint64_t weaponGenerationKey{ 0 };
            authored_weapon_grip_activation_policy::HandTopology handTopology{
                authored_weapon_grip_activation_policy::HandTopology::Invalid
            };
            authored_support_grab_policy::Capability capability{
                authored_support_grab_policy::Capability::Pending
            };
            authored_support_grab_policy::CapabilityReason reason{
                authored_support_grab_policy::CapabilityReason::AwaitingIdentity
            };
            float readySeconds{ 0.0f };
            float usableCandidateMissingSeconds{ 0.0f };
            bool initialized{ false };
        };

        WeaponPartGrip& partGrip(bool isLeft) { return _support.partGrips[isLeft ? 0u : 1u]; }
        const WeaponPartGrip& partGrip(bool isLeft) const { return _support.partGrips[isLeft ? 0u : 1u]; }

        /*
         * Role predicates. Grip math is weapon-relative and hands own roles;
         * these name the two carry programs so call sites read as intent
         * instead of flag algebra. usesNativeRightCarry: FRIK native carry
         * stays authoritative and ROCK applies position-only authored
         * alignment. usesLeftFiringCarry: ROCK owns the weapon node
         * end-to-end (reparent, pose blockers, feed-forward publish, left
         * recoil route). _session.firingHandIsLeft itself is written only by
         * setFiringHand() and reset().
         */
        [[nodiscard]] bool isFiringHand(bool isLeft) const noexcept { return isLeft == _session.firingHandIsLeft; }
        [[nodiscard]] bool isSupportHandLeft() const noexcept { return !_session.firingHandIsLeft; }
        [[nodiscard]] bool usesLeftFiringCarry() const noexcept { return _session.firingHandIsLeft; }
        [[nodiscard]] bool usesNativeRightCarry() const noexcept { return !_session.firingHandIsLeft; }
        [[nodiscard]] const char* firingHandName() const noexcept { return _session.firingHandIsLeft ? "left" : "right"; }
        [[nodiscard]] WeaponPartGrip& supportPartGrip() noexcept { return partGrip(!_session.firingHandIsLeft); }
        [[nodiscard]] const WeaponPartGrip& supportPartGrip() const noexcept { return partGrip(!_session.firingHandIsLeft); }

        bool clearWeaponCollisionHandAuthority(bool isLeft);

        void transitionToTouching(RE::NiNode* weaponNode, const WeaponInteractionDecision& decision);
        void transitionToGripping(
            RE::NiNode* weaponNode,
            const WeaponInteractionDecision& decision,
            const WeaponCollision& weaponCollision,
            weapon_support_authority_policy::WeaponSupportAuthorityMode supportAuthorityMode,
            bool firingGripProximityAuthorityEnabled,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            const WeaponProviderPartAuthority& providerPartAuthority);
        void transitionToInactive(bool publishRestoredWeaponTransform);

        void updateGripping(RE::NiNode* weaponNode, float dt);

        void recordGripFailureFrame(
            RE::NiNode* weaponNode,
            const EquippedWeaponGripFrameInput& frameInput,
            const EquippedWeaponGripFrameInput& stableFrameInput,
            float dt,
            std::uint32_t currentWeaponFormID,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey);
        void recordLockedHandAuthorityAttempt(
            bool isLeft,
            LockedHandAuthorityRole role,
            const RE::NiTransform& targetWorld,
            const RE::NiTransform* liveWorld,
            bool bridgeAvailable,
            bool applied);
        void logGripFailureIncident(const char* reason);
        void resetGripFailureDiagnostics();

        bool providerPartAuthorityStillCurrent(WeaponPartGrip& grip, std::uint64_t currentWeaponGenerationKey);

        // Upgrade twin of the check above: a grip captured without provider
        // authority whose part now resolves to a matched provider target
        // (consumer armed its whitelist mid-hold) releases to recapture.
        bool providerPartTargetNewlyMatchesGrip(const WeaponPartGrip& grip, std::uint64_t currentWeaponGenerationKey) const;

        void updateFullWeaponAuthorityGrip(RE::NiNode* weaponNode, float dt);

        void updatePartCarryGrip(
            RE::NiNode* weaponNode,
            float dt,
            const EquippedWeaponGripFrameInput& frameInput,
            const WeaponInteractionContact& leftWeaponContact,
            const WeaponInteractionContact& rightWeaponContact,
            const WeaponCollision& weaponCollision,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            const WeaponInteractionRuntimeState& leftRuntimeState,
            const WeaponInteractionRuntimeState& rightRuntimeState);

        bool solvePartCarryWeaponAuthority(RE::NiNode* weaponNode, float dt);

        float partCarryGripSeparation(RE::NiNode* weaponNode) const;

        void updateVisualOnlySupportGrip(RE::NiNode* weaponNode, float dt);

        bool transitionToPartCarry();

        bool tryBuildIntegratedDetachPartCarryBaseline(
            bool carryHandIsLeft,
            SupportInputBaselineState& outBaseline,
            const char*& outFailureReason) const;

        bool transitionToPrimaryOnly(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            const char* reason);

        bool reconcileCollisionGeneration(
            RE::NiNode* currentWeaponNode,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            const WeaponCollision& weaponCollision);

        bool tryRebindPartGripToCurrentGeneration(
            WeaponPartGrip& grip,
            std::uint64_t currentWeaponGenerationKey,
            const WeaponCollision& weaponCollision);

        void recordFiringGripDetachedHaptic() noexcept;
        // Marks a refused last-carrier release for this update and logs the
        // first refusal of each open-hand episode.
        void recordGripReleaseRetained(bool isLeft, const char* reason);
        void requestEquippedWeaponDrop(const char* reason, equipped_weapon_drop_policy::SourceHand sourceHand);

        void updatePrimaryOnlyGrip(
            RE::NiNode* weaponNode,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            const EquippedWeaponPrimaryGripInput& primaryGripInput,
            bool primaryDetachEnabled,
            float dt);

        // Position-only left-firing carry. The actual native right
        // weapon-in-wand orientation is mirrored to the left wand, translated
        // to the physical left firing point, and then the authored left wrist
        // is published separately. Used by PrimaryOnly and VisualOnlySupport.
        bool solveLeftFiringWeaponCarry(RE::NiNode* weaponNode, float dt);

        /*
         * Support-release weapon return for the left carry: eases the last
         * rendered two-hand pose into the wand-aimed pose in the physical
         * firing-hand frame (see left_firing_position_only_math). Begun on
         * "support released, primary held" under full weapon authority; a
         * new support grip, part carry, hand switch, or inactive session
         * cancels it. advanceSeconds=0 evaluates the current blend without
         * advancing it (basis pre-writes).
         */
        void beginLeftFiringSupportReleaseReturn(const char* reason);
        void clearLeftFiringSupportReleaseReturn(const char* reason);
        RE::NiTransform resolveLeftFiringSupportReleaseReturn(
            const RE::NiTransform& physicalHandWorld,
            const RE::NiTransform& positionOnlyWeaponWorld,
            float advanceSeconds);

        /*
         * Left-carry owner for weapon identity changes: validates the
         * mirrored native aim frame for the current equipped owner (fail
         * closed when missing) and rebinds it - plus the damped follow
         * frame - to the new node and generation. A right-carry session has
         * no left frames to rebind and succeeds unconditionally.
         */
        bool rebindLeftCarryFramesToWeapon(
            RE::NiNode* currentWeaponNode,
            std::uint64_t targetWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            bool logMissingAimFrame);

        bool tryPromoteSupportGripToFiringGrip(RE::NiNode* weaponNode, float dt);

        void releaseFiringHandWeaponNodeOwnership(RE::NiNode* weaponNode);

        /*
         * Canonical right-hand firing seat. It retains authored wrist/position
         * data and is mirrored only for left-hand presentation. Left weapon
         * aim comes from the separate native weapon-in-wand frame below; the
         * canonical wrist must never become weapon rotation authority again.
         */
        void rememberRightFiringHandCanonicalFrame();
        void refreshRightNativeCanonicalFrame(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey);
        bool canCaptureRightNativeWeaponAimFrame() const;
        bool captureRightNativeWeaponAimFrame(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey);
        bool hasRightNativeWeaponAimFrame(
            const RE::NiNode* weaponNode,
            std::uint64_t weaponGenerationKey,
            std::uint64_t weaponOwnershipKey) const;
        bool tryResolveLeftPositionOnlyCarryFrames(
            RE::NiNode* weaponNode,
            RE::NiTransform& outPhysicalHandWorld,
            RE::NiTransform& outPresentedHandWorld,
            RE::NiTransform& outWeaponWorld,
            float supportReleaseReturnAdvanceSeconds,
            RE::NiTransform* outDampedAimCarrierWorld = nullptr);
        bool captureLeftFiringDampedFollowFrame(
            RE::NiNode* weaponNode,
            const RE::NiTransform& leftWandWorld,
            const RE::NiTransform& physicalLeftHandWorld);
        bool hasLeftFiringDampedFollowFrame(
            const RE::NiNode* weaponNode,
            std::uint64_t weaponGenerationKey,
            std::uint64_t weaponOwnershipKey) const;
        void refreshNaturalHandInWandFrames();
        void clearRightFiringHandCanonicalFrame();
        bool hasRightFiringHandCanonicalFrame(
            const RE::NiNode* weaponNode,
            std::uint64_t weaponGenerationKey,
            std::uint64_t weaponOwnershipKey) const;

        bool tryComputeMirroredLeftFiringHandWeaponLocal(
            RE::NiTransform& outHandWeaponLocal,
            bool* outUsedAuthoredCanonical = nullptr,
            bool logDiagnostic = true) const;

        static bool tryBuildMirroredLeftFiringHandWeaponLocalImpl(
            const RE::NiTransform& canonicalRightHandWeaponLocal,
            const RE::NiPoint3& firingGripWeaponLocal,
            const RE::NiTransform& rightHandWorld,
            const RE::NiTransform& leftHandWorld,
            RE::NiTransform& outHandWeaponLocal,
            bool applyHandlingTrim,
            bool logDiagnostic);

        // Hand frame composed from the wand and the cached natural
        // bone-in-wand map (orientation only). Transient-free input for the
        // equip-takeover seat conjugation; the damped weapon-offset drivers
        // ride the draw/equip animation and must not feed it.
        bool tryResolveNaturalWandHandOrientationFrame(
            bool isLeft,
            RE::NiTransform& outHandWorld) const;

        bool tryBuildMirroredRightSupportHandWeaponLocal(
            const RE::NiTransform& leftHandWeaponLocal,
            RE::NiTransform& outRightHandWeaponLocal) const;

        void refreshAuthoredSupportRightMirror();

        bool tryResolveAuthoredSupportGripCandidateForHand(
            bool isLeft,
            RE::NiNode* weaponNode,
            std::uint64_t weaponGenerationKey,
            RE::NiTransform& outHandWeaponLocal,
            std::array<RE::NiTransform, 15>& outFingerLocalTransforms,
            std::uint16_t& outFingerLocalTransformMask) const;
        bool tryResolveAuthoredSupportActivationAxes(
            RE::NiNode* weaponNode,
            const RE::NiTransform& weaponWorld,
            std::uint64_t currentWeaponGenerationKey,
            authored_weapon_grip_activation_policy::HandTopology handTopology,
            RE::NiPoint3& outSupportSideAxisWorld,
            RE::NiPoint3& outDownAxisWorld,
            RE::NiPoint3& outReferenceAxisWorld) const;
        void refreshAuthoredSupportGripActivationState(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            const WeaponCollision& weaponCollision);
        void resetAuthoredSupportCapability(const char* reason);
        void synchronizeAuthoredSupportCapabilityIdentity(
            RE::NiNode* weaponNode,
            std::uint64_t weaponOwnershipKey,
            std::uint64_t weaponGenerationKey,
            authored_weapon_grip_activation_policy::HandTopology handTopology);
        void advanceAuthoredSupportCapabilityQualification(
            bool ready,
            float deltaSeconds);
        void observeAuthoredSupportCapability(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey);
        void setAuthoredSupportCapability(
            authored_support_grab_policy::Capability capability,
            authored_support_grab_policy::CapabilityReason reason);
        void recordSupportGrabSelection(
            const authored_support_grab_policy::SelectionDecision& decision,
            bool isLeft,
            std::uint64_t weaponGenerationKey);

        /*
         * Reattach validates the hand first and only then commits; a takeover
         * by the non-firing hand flips the firing-hand role inside the commit
         * (setFiringHand), reusing the SAME captured weapon-relative grip
         * frames - the hands only choose who fires, the grip stays
         * weapon-relative.
         */
        bool tryReattachFiringGrip(
            bool handIsLeft,
            RE::NiNode* weaponNode,
            const WeaponInteractionContact& handWeaponContact,
            bool authoredProviderAuthorityActive,
            bool authoredAttachOnlyAuthorityActive);

        bool tryResolveAuthoredFiringHandCanonicalForProbe(
            bool handIsLeft,
            RE::NiTransform& outHandWeaponLocal,
            const char*& outSource) const;

        bool firingGripContactMatchesCapturedGrip(
            RE::NiNode* weaponNode,
            const WeaponInteractionContact& handWeaponContact,
            const RE::NiTransform& handTransform,
            bool handIsLeft) const;

        /*
         * Evaluates the firing-grip reattach zone for one free hand: its palm
         * pivot against the lateral cylinders that start at the captured grip
         * point. The lateral axis is the seated canonical right palm normal on
         * the current weapon; without that canonical hold the zone fails
         * closed. Records the overlay sample for that hand.
         */
        bool tryEvaluateFiringGripReattachZoneForHand(
            RE::NiNode* weaponNode,
            bool handIsLeft,
            firing_grip_reattach_zone_policy::ZoneResult& outZone);

        /*
         * Firing-hand role transition. Clears role-tagged FRIK publications of
         * the old hand and resets firing-hand transient state; callers own the
         * grip-frame capture for the new hand.
         */
        void setFiringHand(bool isLeft, const char* reason);

        /*
         * While the LEFT hand occupies the firing grip, ROCK owns the equipped
         * weapon node end to end: FRIK's per-frame weapon glue is blocked
         * (blockPrimaryWeaponNodeOwnership) and the node is re-parented under
         * LArm_Hand so the scene graph keeps the weapon riding the firing hand
         * at every point in the frame (native fire/aim sampling included).
         * Right-firing states keep today's FRIK-native ownership exactly.
         * Idempotent; call after every state/role transition.
         */
        void syncFiringHandWeaponNodeOwnership(RE::NiNode* weaponNode);

        static RE::NiNode* resolveFirstPersonHandNode(bool isLeft);

        authored_support_grab_policy::Selection capturePartGrip(
            bool isLeft,
            RE::NiNode* weaponNode,
            const WeaponInteractionDecision& decision,
            const WeaponCollision& weaponCollision,
            const WeaponProviderPartAuthority& providerPartAuthority,
            bool firingGripProximityAuthorityEnabled,
            bool ambidextrousHandoffCaptureContext,
            RE::NiTransform* outCapturedHandWorld = nullptr);

        // Per-finger surface solve for a freshly seated support grip: selects
        // evidence triangles, solves the frozen-mesh finger pose, and
        // publishes the resulting pose and full-hand local transforms into
        // the grip.
        void solveSupportGripFingerPose(
            bool isLeft,
            RE::NiNode* weaponNode,
            const WeaponInteractionDecision& decision,
            const WeaponCollision& weaponCollision,
            const RE::NiTransform& handTransform,
            const RE::NiTransform& adjustedHandTransform,
            const RE::NiPoint3& gripWorldPoint,
            bool cachedTrianglesFound,
            const WeaponCollision::SupportGripEvidenceView& evidenceView,
            WeaponPartGrip& grip);

        void lockPartGripToWeaponRoot(bool isLeft);

        void releasePartGrip(bool isLeft, const char* reason, bool smoothHandReturn = false);

        enum class SupportGripPoseFallback : std::uint8_t
        {
            SelectedClose = 0,
            FullyClosed = 1,
        };

        void setSupportGripPose(
            bool isLeft,
            const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPose,
            const std::array<float, 5>* capturedSplayRadians,
            SupportGripPoseFallback fallback =
                SupportGripPoseFallback::SelectedClose);

        void clearSupportGripPose(bool isLeft);

        void clearPrimaryDetachVisualAuthority(bool isLeft);

        void deferScopeHandAuthorityClear(scope_safe_hand_frame_math::HandAuthorityRole role, bool isLeft);

        void recordScopeHandAuthorityPublication(scope_safe_hand_frame_math::HandAuthorityRole role, bool isLeft);

        bool clearHandAuthorityRoleNow(scope_safe_hand_frame_math::HandAuthorityRole role, bool isLeft);

        void reconcileDeferredScopeHandAuthority(RE::NiNode* weaponNode);

        /*
         * recordRenderedWeaponWorld=false marks a basis pre-write: the
         * left-firing feed-forward pose published so in-frame math has a
         * real-space weapon frame. The state handler republishes the final
         * pose afterwards, so a pre-write never counts as the rendered
         * weapon that visual returns, the part-carry handoff, and the seat
         * overlay start from.
         */
        bool applyWeaponVisualAuthority(
            RE::NiNode* weaponNode,
            const RE::NiTransform& solvedWeaponWorld,
            std::uint64_t authorityGenerationKey = 0,
            bool notifyVisualIntentObserver = true,
            bool recordRenderedWeaponWorld = true);

        [[nodiscard]] bool isDynamicSupportBaselineActive(
            bool supportHandIsLeft,
            const WeaponPartGrip& supportGrip) const;
        [[nodiscard]] bool isPartCarryInputBaselineActive(
            bool pivotHandIsLeft,
            const WeaponPartGrip& pivotGrip) const;
        [[nodiscard]] bool isSupportInputBaselineActive(
            bool supportHandIsLeft,
            const WeaponPartGrip& supportGrip,
            SupportInputBaselineKind kind) const;
        bool initializeDynamicSupportBaseline(
            RE::NiNode* weaponNode,
            bool supportHandIsLeft,
            const char* reason);
        void clearSupportInputBaselines();
        bool tryResolvePhysicalHandFrame(
            bool isLeft,
            RE::NiTransform& outHandWorld,
            RE::NiTransform& outDriverWorld) const;

        bool applyFiringHandLockedVisual(RE::NiNode* weaponNode, float dt, const RE::NiTransform* liveHandWorld);

        bool applyPartGripLockedVisual(bool isLeft, RE::NiNode* weaponNode, float dt, const RE::NiTransform* liveHandWorld);

        bool applyLockedHandVisualAuthority(
            RE::NiNode* weaponNode,
            bool applyPrimaryHand,
            bool applySupportHand,
            float dt,
            const RE::NiTransform* livePrimaryHandWorld = nullptr,
            const RE::NiTransform* liveSupportHandWorld = nullptr);

        void publishGripHandPoses(bool isLeft);

        void clearPrimaryGripFingerPose(
            bool isLeft,
            bool preserveAuthoredFingerPose = false);
        void clearPrimaryGripWorldAuthority(bool isLeft);
        void clearAuthoredPrimaryFiringHandWorldAuthority();

        static bool blockFrikPrimaryWeaponPose();

        static void restoreFrikPrimaryWeaponPose();

        static RE::NiPoint3 worldToWeaponLocal(const RE::NiPoint3& worldPos, const RE::NiAVObject* weaponNode);

        static RE::NiPoint3 weaponLocalToWorld(const RE::NiPoint3& localPos, const RE::NiAVObject* weaponNode);

        RE::NiPoint3 resolvePartGripWorld(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const;

        RE::NiPoint3 resolvePartGripWeaponLocal(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const;

        RE::NiPoint3 resolvePartGripNormalWeaponLocal(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const;

        RE::NiTransform resolvePartGripHandWorld(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const;

        RE::NiAVObject* resolveCurrentSupportAttachmentRoot(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const;

        void resetLockedHandVisualLerp();
        void beginDynamicSupportAcquisition(
            bool supportHandIsLeft,
            const WeaponPartGrip& supportGrip,
            const RE::NiTransform& primaryStartWorld,
            const RE::NiTransform& supportStartWorld);
        void clearDynamicSupportAcquisition(
            const char* reason,
            bool logCancellation);
        [[nodiscard]] bool dynamicSupportAcquisitionMatches(
            bool supportHandIsLeft,
            const WeaponPartGrip& supportGrip) const;
        RE::NiTransform resolveDynamicSupportAcquisitionHandTarget(
            const RE::NiTransform& targetWorld,
            bool primaryHand,
            LockedHandVisualLerpState& visualState);
        void recordPublishedHandWorld(bool isLeft, const RE::NiTransform& appliedWorld);
        void beginHandVisualReturn(bool isLeft, const char* reason);
        void updateHandVisualReturns(float dt);
        void clearHandVisualReturn(bool isLeft, const char* reason, bool logCancellation);
        void beginWeaponVisualReturn(const char* reason);
        void updateWeaponVisualReturn(
            RE::NiNode* currentWeaponNode,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            float dt);
        void clearWeaponVisualReturn(
            const char* reason,
            bool logCancellation,
            bool restoreBlockers,
            bool preserveAuthoredPrimaryPose = false);
        void clearAllVisualReturns(const char* reason, bool logCancellation, bool restoreBlockers);
        /*
         * Weapon pose handoff blend. Arm at the grip transition; the solve
         * that publishes next resolves its pose through the blend, which
         * captures the residual against the last rendered pose on that first
         * call and slerps it out over the shared equipped-weapon return
         * timing. A residual inside the exact-handoff tolerance never blends.
         */
        void armWeaponPoseHandoffBlend(const char* reason);
        [[nodiscard]] RE::NiTransform resolveWeaponPoseHandoffBlend(
            const RE::NiTransform& solvedWeaponWorld,
            float dt);
        void clearWeaponPoseHandoffBlend(const char* reason, bool logCancellation);
        bool tryResolveAuthoredPrimaryWeaponReturnTargetLocal(
            RE::NiNode* weaponNode,
            RE::NiNode* nativeParent,
            const RE::NiTransform& nativeBaselineLocal,
            std::uint64_t weaponGenerationKey,
            std::uint64_t equippedWeaponOwnershipKey,
            RE::NiTransform& outTargetLocal) const;
        void clearNativeScopeOverlayAuthority(bool restoreNativeLocal);
        void clearNativeScopeRigidFrame();
        bool rebuildNativeScopeRigidFrameTarget();
        bool captureNativeScopeRigidFrame(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey, RE::NiNode* scopeCamera, const RE::NiTransform& nativeCameraWorld);
        bool captureNativeScopeOverlayCalibration(const RE::NiTransform& nativeCameraWorld, std::uint64_t currentWeaponGenerationKey);
        bool applyNativeScopeOverlayTarget(const RE::NiTransform& correctedCameraWorld, std::uint64_t currentWeaponGenerationKey);
        void refreshNativeScopeAnchor(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            std::uint32_t currentEquippedWeaponFormID,
            const WeaponCollision& weaponCollision);
        void refreshScopeSafeHandFrames(RE::NiNode* weaponNode, const EquippedWeaponGripFrameInput& frameInput, float dt);
        void publishCollisionIsolatedRightNativeWeaponIntent(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey);
        void traceNativeScopeTransitionFinalState(RE::NiNode* weaponNode);
        bool tryGetSolverHandTransform(bool isLeft, RE::NiTransform& outTransform) const;
        RE::NiTransform resolveLockedHandVisualTarget(
            const RE::NiTransform& targetWorld,
            const RE::NiTransform* liveHandWorld,
            float dt,
            LockedHandVisualLerpState& state);

        /*
         * ---- Partitioned member state ----
         * Each implementation module (grip/*.cpp, scope/*.cpp,
         * telemetry/*.cpp) owns one state struct below; GripSession is the
         * only state every module shares. Genuinely cross-module values (the
         * solved-weapon product, the native weapon-node baseline, outbound
         * one-shot queues, the handling-settings snapshot) remain direct
         * members at the end.
         */

        static constexpr float TOUCH_TIMEOUT_SECONDS = 5.0f / 90.0f;
        static constexpr float ROTATION_BLEND_SPEED = 8.0f;

        /*
         * Core grip session: the manual-ownership state machine, the
         * firing-hand role, the identity of the weapon the session is bound
         * to, the support authority mode, and the monotonic capture
         * sequences.
         *
         * Legal combinations:
         * - Inactive: no grip authority anywhere; weaponNode and the identity
         *   keys are stale bookkeeping and must be revalidated before use.
         * - Touching: support hand hovers the weapon; no transform authority.
         * - Gripping: firing grip and a support part grip are both occupied;
         *   authorityMode selects the active two-hand solver flavor.
         * - PartCarry: the firing grip is vacant and the weapon is carried by
         *   one or two part grips; firingHandIsLeft still names the hand that
         *   would re-take the firing grip.
         * - PrimaryOnly: the firing grip alone is occupied. With the right
         *   role the weapon transform stays native (FRIK); with the left role
         *   ROCK owns the weapon node end to end (LeftFiringCarryState).
         */
        struct GripSession
        {
            TwoHandedState state{ TwoHandedState::Inactive };

            /*
             * Physical hand that owns the firing grip when occupied. Seeded
             * right (false) and reset to right whenever authority clears
             * (weapon change, holster, drop, teardown). Flipped only through
             * setFiringHand() from a hand-specific loose-weapon equip or the
             * two takeover paths (free-hand firing-grip squeeze in PartCarry,
             * support-grip promotion on firing-hand release); PhysicsInteraction
             * and InputRemap read isFiringHandLeft() each frame to route input
             * and ownership per role. Read only through the role predicates;
             * written only by setFiringHand() and reset().
             */
            bool firingHandIsLeft{ false };

            weapon_support_authority_policy::WeaponSupportAuthorityMode authorityMode{
                weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver
            };

            // Weapon identity the session is bound to. The node is a
            // non-owning identity witness for the currently published grip
            // math; generation and ownership keys make later use fail closed
            // when the weapon or equip changes.
            RE::NiNode* weaponNode{ nullptr };
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint64_t equippedWeaponOwnershipKey{ 0 };

            /*
             * Monotonic capture sequences so API consumers can detect a
             * re-grab of the same part without frame-edge callbacks. One
             * counter for part grips (stamped per capture) and one for fresh
             * firing-grip captures.
             */
            std::uint64_t gripCaptureSequence{ 0 };
            std::uint64_t firingGripSequence{ 0 };
        };

        enum class RightFiringCanonicalSource : std::uint8_t
        {
            None,
            NativeCarry,
            AuthoredAnimation,
        };

        struct RightNativeWeaponAimFrame
        {
            RE::NiTransform weaponInWandOrientation{};
            // Non-owning identity witness only; compared, never dereferenced.
            RE::NiNode* weaponNodeIdentity{ nullptr };
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint64_t weaponOwnershipKey{ 0 };
            bool valid{ false };
        };

        struct LeftFiringDampedFollowFrame
        {
            RE::NiTransform handInWandOrientation{};
            // Non-owning identity witness only; compared, never dereferenced.
            RE::NiNode* weaponNodeIdentity{ nullptr };
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint64_t weaponOwnershipKey{ 0 };
            bool valid{ false };
        };

        // State owned by the FiringGrip module: the canonical right firing
        // seat and its mirrors, published finger poses, authored primary
        // presentation flags, natural hand-in-wand calibration, the captured
        // firing-grip frames, persistent-carry bookkeeping, and the
        // reattach-hover witness.
        struct FiringGripState
        {
            // Canonical right-hand firing hold, keyed by node identity,
            // collision generation, and equipped ownership; see
            // rememberRightFiringHandCanonicalFrame().
            RE::NiTransform rightCanonicalHandWeaponLocal{};
            RE::NiPoint3 rightCanonicalGripWeaponLocal{};
            // Non-owning identity witness only; compared, never dereferenced.
            RE::NiNode* rightCanonicalWeaponNode{ nullptr };
            std::uint64_t rightCanonicalGenerationKey{ 0 };
            std::uint64_t rightCanonicalOwnershipKey{ 0 };
            std::uint64_t rightCanonicalCaptureSequence{ 0 };
            RightFiringCanonicalSource rightCanonicalSource{
                RightFiringCanonicalSource::None
            };
            bool hasRightCanonicalHandWeaponLocal{ false };

            RightNativeWeaponAimFrame rightNativeWeaponAimFrame{};
            LeftFiringDampedFollowFrame leftDampedFollowFrame{};

            std::array<RE::NiTransform, 15> rightFingerLocalTransforms{};
            std::array<RE::NiTransform, 15> leftFingerLocalTransforms{};
            std::uint16_t rightFingerLocalTransformMask{ 0 };
            std::uint16_t leftFingerLocalTransformMask{ 0 };
            bool publishedFingerPoseIsLeft{ false };
            bool authoredFingerPosePublished{ false };
            bool authoredFingerPoseBlockEngaged{ false };
            bool authoredFingerPoseSuppressed{ false };
            // Position-only authored presentation: ROCK owns the right
            // firing-hand world (weapon-relative authored wrist) while the
            // weapon keeps its native rotation. Cleared whenever the authored
            // runtime skips a frame.
            bool authoredHandWorldActive{ false };
            bool authoredHandWorldRefreshed{ false };
            // Left position-only carry owns the authored firing wrist
            // separately from the mirrored native weapon orientation.
            bool leftHandWorldActive{ false };
            bool leftHandHoldingObjectForPose{ false };
            bool rightHandHoldingObjectForPose{ false };

            /*
             * Natural physical hand-bone relations in the raw wand and
             * hFRIK's damped driver. They are refreshed only while ROCK has
             * no visual authority for that hand and are deliberately not
             * weapon-generation keyed so final alignment never consumes
             * ROCK's previous output.
             */
            RE::NiTransform rightNaturalBoneInWand{};
            RE::NiTransform leftNaturalBoneInWand{};
            RE::NiTransform rightNaturalBoneInDampedDriver{};
            RE::NiTransform leftNaturalBoneInDampedDriver{};
            bool hasRightNaturalBoneInWand{ false };
            bool hasLeftNaturalBoneInWand{ false };
            bool hasRightNaturalBoneInDampedDriver{ false };
            bool hasLeftNaturalBoneInDampedDriver{ false };

            // Captured firing-grip frames on the current weapon.
            RE::NiPoint3 primaryGripLocal{};
            RE::NiTransform primaryHandWeaponLocal{};
            bool hasPrimaryHandWeaponLocal{ false };
            float primaryGripConfidence{ 0.0f };

            equipped_weapon_manual_ownership_policy::GripReleaseDebounceState primaryReleaseDebounce{};
            bool persistentCarryActive{ false };
            bool persistentCarryDetachArmed{ false };
            bool persistentCarryInputAcquisitionPending{ false };

            // Per-frame hover state; only ever true in PartCarry (see getter).
            bool reattachHoverInsideZone{ false };
            bool reattachHoverHandIsLeft{ false };
            // Per-frame indicator and overlay records; rebuilt every update().
            FiringGripReattachIndicatorFrame reattachIndicatorFrame{};
            FiringGripReattachZoneDebugSnapshot reattachDebugSnapshot{};
        };

        // State owned by the SupportGrip module: the per-hand part grips,
        // the authored support candidate/capability/selection pipeline, and
        // the dynamic support acquisition transaction.
        struct SupportGripState
        {
            std::array<WeaponPartGrip, 2> partGrips{};
            AuthoredSupportGripCandidate authoredCandidate{};
            AuthoredSupportCapabilityState authoredCapability{};
            authored_support_grab_policy::SelectionDecision lastSelection{};
            std::uint64_t lastSelectionGenerationKey{ 0 };
            bool lastSelectionHandIsLeft{ true };
            bool lastSelectionValid{ false };
            AuthoredSupportGripIndicatorFrame authoredIndicatorFrame{};
            AuthoredSupportGripDebugSnapshot authoredDebugSnapshot{};
            RE::NiPoint3 lastStableApproachDirectionWorld{};
            std::uint64_t lastStableDirectionGenerationKey{ 0 };
            std::uint64_t lastStableDirectionCaptureSequence{ 0 };
            authored_weapon_grip_activation_policy::HandTopology
                lastStableDirectionHandTopology{
                    authored_weapon_grip_activation_policy::HandTopology::Invalid
                };
            bool lastStableApproachDirectionValid{ false };

            DynamicSupportAcquisitionState dynamicAcquisition{};

            // Separation between primary and support grips captured when the
            // two-hand solve locks.
            float lockedGripSeparationWorld{ 0.0f };

            float rotationBlend{ 0.0f };

            /*
             * Whole frames spent in Gripping since the support grab was
             * captured. Gates promotion/detach on a confirmed firing-grip
             * release: a release that confirms while this is still fresh is
             * the same gesture (or a grab-synchronized grip flicker) and is
             * deferred.
             */
            float gripAgeSeconds{ 0.0f };
            bool freshGripDeferLogged{ false };
        };

        // State owned by the PartCarry module: which grip anchors the
        // detached carry and the captured carry contracts.
        struct PartCarryState
        {
            /*
             * Which active part grip anchors the part-carry solve. The older
             * grip carries the weapon (translation pivot); the newer grip
             * aims it.
             */
            bool pivotIsLeft{ true };

            // Captured when PartCarry begins so a later provider/config
            // snapshot cannot silently change which detach contract owns the
            // live carry.
            immersive_weapon_policy::DetachAuthority detachAuthority{
                immersive_weapon_policy::DetachAuthority::None
            };

            float gripSeparationWorld{ 0.0f };
        };

        // State owned by the LeftFiringCarry module: FRIK weapon-node
        // ownership blocking, the LArm_Hand reparent witness, and the
        // left-firing recoil route.
        struct LeftFiringCarryState
        {
            // FRIK weapon-node ownership block + reparent bookkeeping for
            // left-firing carry; see syncFiringHandWeaponNodeOwnership().
            bool weaponNodeOwnershipBlockEngaged{ false };
            bool weaponNodeReparented{ false };

            // Support-release weapon return: the last rendered two-hand pose
            // in the physical firing-hand frame, eased into the wand-aimed
            // pose over the shared equipped-weapon return timing.
            hand_visual_lerp_math::VisualReturnTransition<RE::NiTransform>
                supportReleaseReturn{};

            /*
             * hFRIK calls the recoil controller before ROCK's update on the
             * same game thread. The sequence makes a sample valid for one
             * ROCK update only. It prevents a skipped callback from reusing
             * an old gun kick. Full two-hand left carry consumes that sample
             * in the primary-hand solver target; one-hand carry consumes the
             * native sample in the final direct weapon publication; close
             * visual support consumes the attenuated sample there. These
             * routes must remain mutually exclusive or supported left firing
             * regresses to one-hand recoil.
             */
            RE::NiTransform recoilWorldDelta{};
            std::uint64_t recoilSampleSequence{ 0 };
            std::uint64_t observedRecoilSampleSequence{ 0 };
            bool recoilSampleValid{ false };
            bool recoilReadyThisUpdate{ false };
            bool recoilSupportConstrainedThisUpdate{ false };
            bool recoilControllerRegistered{ false };
            // Advanced by the recoil controller callback whenever FRIK hands
            // it a non-identity native kick, accepted or not: FRIK composes
            // that kick onto ROCK's hand claims in the same skeleton frame.
            std::uint64_t nativeRecoilKickSequence{ 0 };
        };

        // State owned by the HandVisualTransitions module: hand and weapon
        // visual return transitions, published-world witnesses, and the
        // collision-presentation frame tags.
        struct HandVisualTransitionState
        {
            std::array<ReturningHandVisualState, 2> returningHands{};
            std::array<RE::NiTransform, 2> lastPublishedHandWorld{};
            std::array<bool, 2> hasLastPublishedHandWorld{};
            // Per physical hand (left index 0, right index 1). A collision
            // target survives only through the render interval that follows
            // post-solve.
            std::array<bool, 2> weaponCollisionHandAuthorityLive{};
            // Captured before those tags are cleared. Because ROCK runs after
            // FRIK, this identifies current root/weapon poses that already
            // include the previous render interval's collision presentation.
            std::array<bool, 2> weaponCollisionHandPresentationFromPreviousFrame{};
            ReturningWeaponVisualState returningWeapon{};
            WeaponPoseHandoffBlendState weaponHandoff{};
            RE::NiTransform lastRenderedWeaponWorld{};
            bool hasLastRenderedWeaponWorld{ false };
            LockedHandVisualLerpState primaryHandLerp{};
            void* weaponIntentObserverContext{ nullptr };
            WeaponVisualIntentObserver weaponIntentObserver{ nullptr };
        };

        // State owned by the scope modules (NativeScopePresentation and
        // ScopeSafeHandFrames): scope-safe hand frames, ScopeMenu edges, the
        // native scope request witness, transition traces, deferred hand
        // authority clears, the resolved sight anchor, and the camera/overlay
        // calibration.
        struct ScopePresentationState
        {
            std::array<ScopeSafeHandFrameState, 2> safeHandFrames{};
            // Frame-scoped hFRIK/controller drivers captured by
            // PhysicsInteraction before ROCK publishes any hand visuals.
            std::array<EquippedWeaponScopeHandDriverFrame, 2>
                currentHandDriverFrames{};
            bool menuOpenThisFrame{ false };
            // True only for the first visible frame after ScopeMenu. Role
            // clears on this edge use the same deferred transaction as
            // hidden-frame clears so a same-frame handoff can publish its
            // replacement first.
            bool menuClosedThisFrame{ false };
            // Bounded edge trace comparing the UI signal, verified renderer
            // request, restored root hands, hFRIK drivers, and ROCK solver
            // frames. This remains low-volume and makes future
            // scope-transition reports diagnosable from one session without
            // enabling per-frame telemetry.
            bool nativeRequestStateValid{ false };
            bool nativeRequestActive{ false };
            bool manualActivationRequested{ false };
            std::uint64_t transitionTraceSequence{ 0 };
            std::uint32_t transitionTraceFramesRemaining{ 0 };
            std::uint64_t transitionFinalTraceSequence{ 0 };
            std::uint32_t transitionFinalTraceSample{ 0 };
            bool transitionFinalTracePending{ false };
            // Latched across a manual grip session after its first scoped
            // frame so ScopeMenu presentation edges cannot reselect the
            // weapon-solver basis.
            bool driverFrameAuthorityActive{ false };
            // Per physical hand (left index 0, right index 1). Clears
            // requested while hFRIK's root is collapsed remain role-specific
            // so scope exit never tears down authority that the current
            // weapon state still owns.
            std::array<scope_safe_hand_frame_math::HandAuthorityRoleMask, 2> deferredHandAuthorityClears{};
            std::array<scope_safe_hand_frame_math::HandAuthorityRoleMask, 2> handAuthorityPublishedThisFrame{};

            // Generation-bound resolved anchor. Generated sight evidence
            // remains preferred; malformed/missing optics use the current
            // firing-grip origin plus the configured Weapon-local offset. The
            // pointer is an identity witness only and is never dereferenced
            // from this cache.
            RE::NiNode* anchorWeaponNode{ nullptr };
            std::uint64_t anchorGenerationKey{ 0 };
            std::uint64_t anchorOwnershipKey{ 0 };
            std::uint32_t anchorWeaponFormID{ 0 };
            RE::NiPoint3 anchorWeaponLocal{};
            native_scope_sight_anchor_policy::AnchorSource anchorSource{
                native_scope_sight_anchor_policy::AnchorSource::None
            };
            bool anchorValid{ false };
            RE::NiPoint3 fallbackRotationDegrees{};
            NativeScopeCameraDebugSnapshot cameraDebugSnapshot{};
            NativeScopeActivationDebugSnapshot activationDebugSnapshot{};
            NativeScopeRigidFrameState rigidFrame{};
            NativeScopeOverlayCalibrationState overlayCalibration{};
        };

        // State owned by the GripFailureTelemetry module. Fixed-capacity,
        // value-only prehistory: it allocates and formats nothing in the
        // frame path; an involuntary teardown emits it once at a controlled
        // rate so transient frame loss is reconstructible.
        struct GripFailureTelemetryState
        {
            std::array<GripFailureFrameSnapshot, kGripFailureHistoryCapacity>
                history{};
            std::size_t historyNext{ 0 };
            std::size_t historyCount{ 0 };
            std::size_t currentHistoryIndex{ kGripFailureHistoryCapacity };
            std::uint64_t incidentSequence{ 0 };
            float detailedLogCooldownSeconds{ 0.0f };
        };

        GripSession _session{};
        FiringGripState _firing{};
        SupportGripState _support{};
        PartCarryState _partCarry{};
        LeftFiringCarryState _leftCarry{};
        HandVisualTransitionState _visuals{};
        ScopePresentationState _scope{};
        GripFailureTelemetryState _telemetry{};

        // ---- Shared cross-module members ----

        // Reused one-shot solve storage. It owns no engine pointers and keeps
        // bounded vector/BVH capacity across re-grabs without polluting the
        // per-frame WeaponPartGrip state.
        std::unique_ptr<FingerPoseSolveScratch> _fingerPoseSolveScratch;

        /*
         * Elapsed time the support hand has been off the weapon while in the
         * Touching state. An elapsed contract (the historical 5-frame window
         * at the 90 Hz tuning baseline), rate-independent in seconds.
         */
        float _touchAbsentSeconds{ 0.0f };

        // Rate limiter shared by the support/part-carry diagnostics.
        int _gripLogCounter{ 0 };

        // Solved-weapon product of the active carry program, consumed by
        // every module and the debug snapshots.
        RE::NiTransform _lastSolvedWeaponTransform{};
        bool _hasSolvedWeaponTransform{ false };

        // Native local transform of the weapon node captured before ROCK's
        // first write, restored when manual ownership ends.
        RE::NiTransform _weaponNodeLocalBaseline{};
        bool _hasWeaponNodeLocalBaseline{ false };

        // Outbound one-shot queues consumed by PhysicsInteraction each frame.
        EquippedWeaponManualDropRequest _equippedWeaponDropRequest{};
        TwoHandedGripHapticEvents _hapticEvents{};

        // Refused last-carrier releases: rebuilt every update() and returned
        // in the update result. The logged flags (index 0 right, 1 left)
        // mark one open-hand episode and clear when that hand's logical grip
        // closes again.
        EquippedWeaponGripReleaseRetention _gripReleaseRetained{};
        std::array<bool, 2> _gripReleaseRetainedLogged{};

        // Per-update snapshot of the configured handling settings.
        EquippedWeaponHandlingSettings _handlingSettings{};
    };

}
