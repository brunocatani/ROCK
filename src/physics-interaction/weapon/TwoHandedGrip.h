#pragma once

#include <atomic>
#include <array>
#include <cstdint>
#include <memory>

#include "api/FRIKApi.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/weapon/EquippedWeaponDropPolicy.h"
#include "physics-interaction/weapon/EquippedWeaponHandlingSettings.h"
#include "physics-interaction/weapon/NativeScopeSightAnchorPolicy.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
#include "physics-interaction/weapon/WeaponPartGripReportPolicy.h"
#include "physics-interaction/weapon/WeaponSupport.h"

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTransform.h"

namespace rock
{
    class WeaponCollision;

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
    };

    struct EquippedWeaponGripFrameInput
    {
        bool leftGripHeld{ false };
        bool rightGripHeld{ false };
        bool leftHandHoldingObject{ false };
        bool rightHandHoldingObject{ false };
        bool leftReattachEligible{ false };
        bool rightReattachEligible{ false };
        bool scopeMenuOpen{ false };
        EquippedWeaponScopeHandDriverFrame leftHandDriverFrame{};
        EquippedWeaponScopeHandDriverFrame rightHandDriverFrame{};
        // Grab state of the CURRENT firing hand (debounced release), read by
        // the caller from whichever physical hand isFiringHandLeft() reports.
        EquippedWeaponPrimaryGripInput primaryGripInput{};
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
        RE::NiTransform weaponWorld{};
        RE::NiPoint3 authoredPalmSeatWeaponLocal{};
        RE::NiPoint3 authoredPalmSeatWorld{};
        RE::NiPoint3 liveTouchProbeWeaponLocal{};
        RE::NiPoint3 liveTouchProbeWorld{};
        float weaponRelativeDistanceGameUnits{ 0.0f };
        float worldReadbackDistanceGameUnits{ 0.0f };
        float frameAgreementErrorGameUnits{ 0.0f };
        float touchRadiusGameUnits{ 0.0f };
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t captureSequence{ 0 };
        bool supportHandIsLeft{ true };
        bool mirroredForRightSupport{ false };
        bool insideTouchRadius{ false };
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
        std::uint64_t evaluationSequence{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
        native_scope_sight_anchor_policy::AnchorSource anchorSource{
            native_scope_sight_anchor_policy::AnchorSource::None
        };
        bool nativeGeometryDecision{ false };
        bool rockGeometryDecision{ false };
        bool nativeScopeAlreadyActive{ false };
        native_scope_activation_geometry::ConeSample sample{};
        native_scope_activation_geometry::ConeThresholds thresholds{};
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
        TwoHandedGrip();
        ~TwoHandedGrip();

        TwoHandedGrip(const TwoHandedGrip&) = delete;
        TwoHandedGrip& operator=(const TwoHandedGrip&) = delete;
        TwoHandedGrip(TwoHandedGrip&&) = delete;
        TwoHandedGrip& operator=(TwoHandedGrip&&) = delete;

        void update(
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
            std::uint64_t currentWeaponGenerationKey);

        /*
         * Binds Bethesda's exact RArm_Hand-in-Weapon relation to ROCK's
         * existing firing canonical by node identity, collision generation,
         * and equipped ownership. When a complete firing-finger pose is
         * available, the right animation locals and hFRIK's anatomy-correct
         * left mirror are retained only for the matching weapon identity.
         * The node pointer is comparison-only and is never dereferenced after
         * publication.
         */
        bool setAuthoredPrimaryFiringGripCanonical(
            RE::NiNode* weaponNode,
            const RE::NiTransform& rightHandWeaponLocal,
            std::uint64_t weaponGenerationKey,
            std::uint64_t weaponOwnershipKey,
            std::uint64_t captureSequence, const authored_weapon_grip_library::FiringFingerPose* rightFingerPose = nullptr,
            const authored_weapon_grip_library::FiringFingerPose* leftFingerPose = nullptr);
        void clearAuthoredPrimaryFiringGripCanonical(const char* reason);
        bool publishAuthoredPrimaryFiringGripFingerPose(bool isLeft);
        [[nodiscard]] bool hasPublishedAuthoredPrimaryFiringGripFingerPose(const bool isLeft) const noexcept
        {
            return _authoredPrimaryFingerPosePublished && _publishedFiringFingerPoseIsLeft == isLeft;
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
        bool setAuthoredSupportGripCandidate(
            RE::NiNode* weaponNode,
            const RE::NiTransform& handWeaponLocal,
            const std::array<RE::NiTransform, 15>& fingerLocalTransforms,
            std::uint16_t fingerLocalTransformMask,
            std::uint64_t weaponGenerationKey,
            std::uint64_t captureSequence);

        bool tryResolveNativeScopeGeometryDecision(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey, RE::NiNode* hmdNode, const RE::NiPoint3& hmdSampleOffsetLocal,
            const native_scope_activation_geometry::ConeThresholds& thresholds, bool nativeScopeAlreadyActive, bool nativeGeometryDecision, bool& outRockGeometryDecision);

        void reset();

        bool isGripping() const { return _state == TwoHandedState::Gripping || _state == TwoHandedState::PartCarry; }

        bool isManualOwnershipActive() const
        {
            return _state == TwoHandedState::Gripping ||
                   _state == TwoHandedState::PartCarry ||
                   _state == TwoHandedState::PrimaryOnly;
        }

        bool isWeaponVisualReturnActive() const;

        bool isPartCarryActive() const { return _state == TwoHandedState::PartCarry; }

        bool isPrimaryOnlyActive() const { return _state == TwoHandedState::PrimaryOnly; }

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

        bool isFiringGripOccupied() const { return _state == TwoHandedState::Gripping || _state == TwoHandedState::PrimaryOnly; }

        /*
         * True while the OPEN firing palm hovers inside the reattach radius
         * during part carry: squeezing the grab right now would re-take the
         * firing grip. Recomputed every update(); PhysicsInteraction consumes
         * it each frame to drive continuous hover haptics on the firing hand.
         */
        bool isFiringGripReattachHoverInsideRadius() const { return _firingGripReattachHoverInsideRadius; }

        // Which physical hand the hover above refers to (either free hand can
        // hover the firing grip when ambidextrous takeover is available).
        bool isFiringGripReattachHoverHandLeft() const { return _firingGripReattachHoverHandIsLeft; }

        bool canUseFiringGripInput() const
        {
            return _state == TwoHandedState::Gripping ||
                   _state == TwoHandedState::PartCarry ||
                   _state == TwoHandedState::PrimaryOnly;
        }

        bool isTouching() const { return _state == TwoHandedState::Touching; }

        bool isFiringHandLeft() const { return _firingHandIsLeft; }

        TwoHandedState getState() const { return _state; }

        weapon_support_authority_policy::WeaponSupportAuthorityMode getAuthorityMode() const { return _authorityMode; }

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

        NativeScopeCameraDebugSnapshot getNativeScopeCameraDebugSnapshot() const { return _nativeScopeCameraDebugSnapshot; }
        NativeScopeCameraTargetPreviewSnapshot getNativeScopeCameraTargetPreviewSnapshot() const
        {
            const bool valid =
                _nativeScopeRigidFrame.valid &&
                _nativeScopeAnchorValid &&
                _nativeScopeRigidFrame.weaponNodeIdentity ==
                    _nativeScopeAnchorWeaponNode &&
                _nativeScopeRigidFrame.weaponGenerationKey ==
                    _nativeScopeAnchorGenerationKey;
            return NativeScopeCameraTargetPreviewSnapshot{
                .weaponGenerationKey =
                    _nativeScopeRigidFrame.weaponGenerationKey,
                .equippedWeaponOwnershipKey =
                    _nativeScopeAnchorOwnershipKey,
                .weaponFormID = _nativeScopeAnchorWeaponFormID,
                .anchorSource = _nativeScopeAnchorSource,
                .cameraWeaponLocal =
                    _nativeScopeRigidFrame.cameraWeaponLocal,
                .valid = valid,
            };
        }
        NativeScopeActivationDebugSnapshot getNativeScopeActivationDebugSnapshot() const { return _nativeScopeActivationDebugSnapshot; }
        NativeScopeResolvedAnchorSnapshot getNativeScopeResolvedAnchorSnapshot() const
        {
            return NativeScopeResolvedAnchorSnapshot{
                .weaponGenerationKey = _nativeScopeAnchorGenerationKey,
                .equippedWeaponOwnershipKey = _nativeScopeAnchorOwnershipKey,
                .weaponFormID = _nativeScopeAnchorWeaponFormID,
                .anchorWeaponLocal = _nativeScopeAnchorWeaponLocal,
                .source = _nativeScopeAnchorSource,
                .valid = _nativeScopeAnchorValid,
            };
        }
        bool getSelectedAuthoredGripPoseSnapshot(
            SelectedAuthoredGripPoseSnapshot& outSnapshot) const;

        bool isScopeMenuOpenThisFrame() const { return _scopeMenuOpenThisFrame; }

        bool hasVisualAuthorityForHand(bool isLeft) const;
        bool isHandVisualReturnActive(bool isLeft) const;
        void cancelHandVisualReturn(bool isLeft, const char* reason);

        bool beginPrimaryOnlyGrip(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            bool firingHandIsLeft,
            const RE::NiTransform* capturedFiringHandWeaponLocal,
            const RE::NiPoint3* capturedFiringGripWeaponLocal);

        /*
         * Pip-Boy left-hand assignment starts without a physical grab hold.
         * This entry point reuses the generation-bound native right-hand
         * canonical pose, mirrors it through the live wand frames, and marks
         * the resulting PrimaryOnly session as persistent until the selected
         * inventory stack is unequipped or the player deliberately arms and
         * releases the firing-hand grab.
         */
        bool beginPersistentEquippedCarry(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey);

        void clearPersistentEquippedCarry(const char* reason);
        void restoreNativeRightEquippedCarry(const char* reason);
        bool isPersistentEquippedCarryActive() const { return _persistentEquippedCarryActive; }

        // Left-hand primary ownership requires ROCK's hFRIK weapon-pose and
        // node-ownership blockers; right-hand native ownership is always
        // eligible. Native Fallout/FRIK handedness is deliberately irrelevant.
        static bool canBeginPrimaryOnlyGripForHand(bool isLeft);

        /*
         * Mirrors FRIK's canonical right-hand per-weapon hold into the left
         * controller/hand basis. Both inputs are weapon-root-local, so the
         * result is independent of whichever hand is currently probing or
         * loosely holding the world model.
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
            const frik::api::FRIKApi::RecoilSample* sample,
            frik::api::FRIKApi::RecoilResponse* outResponse,
            void* userData) noexcept;

        struct LockedHandVisualLerpState
        {
            bool active = false;
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
            bool retainPrimaryPoseBlocker{ false };
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
        };

        /*
         * One hand's captured part grip on the equipped weapon. Both
         * weapon-root-local and authored-source-local frames are stored so the
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

        WeaponPartGrip& partGrip(bool isLeft) { return _partGrips[isLeft ? 0u : 1u]; }
        const WeaponPartGrip& partGrip(bool isLeft) const { return _partGrips[isLeft ? 0u : 1u]; }

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

        void requestEquippedWeaponDrop(const char* reason, equipped_weapon_drop_policy::SourceHand sourceHand);

        void updatePrimaryOnlyGrip(
            RE::NiNode* weaponNode,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            const EquippedWeaponPrimaryGripInput& primaryGripInput,
            bool primaryDetachEnabled);

        // Rigid left-firing weapon carry: weapon = firing hand ∘ inverse of the
        // captured weapon-relative grip frame. Used by PrimaryOnly and
        // VisualOnlySupport while the LEFT hand occupies the firing grip
        // (right-firing keeps FRIK-native carry in those states).
        bool solveLeftFiringWeaponCarry(RE::NiNode* weaponNode);

        bool tryPromoteSupportGripToFiringGrip(RE::NiNode* weaponNode);

        void releaseFiringHandWeaponNodeOwnership(RE::NiNode* weaponNode);

        /*
         * Canonical right-hand firing hold. Snapshotted whenever the RIGHT
         * hand captures the firing grip (it then carries FRIK's authored
         * per-weapon offsets); a LEFT takeover applies this frame MIRRORED so
         * the left hand holds the weapon with the same offsets adapted to the
         * left bone basis, instead of freezing the live squeeze orientation.
         */
        void rememberRightFiringHandCanonicalFrame();
        void refreshRightNativeCanonicalFrame(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey);
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

        bool tryComputePalmToGripDistanceForHand(RE::NiNode* weaponNode, bool handIsLeft, float& outDistance) const;

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

        bool capturePartGrip(
            bool isLeft,
            RE::NiNode* weaponNode,
            const WeaponInteractionDecision& decision,
            const WeaponCollision& weaponCollision,
            const WeaponProviderPartAuthority& providerPartAuthority,
            bool firingGripProximityAuthorityEnabled);

        void lockPartGripToWeaponRoot(bool isLeft);

        void releasePartGrip(bool isLeft, const char* reason, bool smoothHandReturn = false);

        void setSupportGripPose(
            bool isLeft,
            const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPose,
            const std::array<float, 5>* capturedSplayRadians);

        void clearSupportGripPose(bool isLeft);

        void clearPrimaryDetachVisualAuthority(bool isLeft);

        void deferScopeHandAuthorityClear(scope_safe_hand_frame_math::HandAuthorityRole role, bool isLeft);

        void recordScopeHandAuthorityPublication(scope_safe_hand_frame_math::HandAuthorityRole role, bool isLeft);

        bool clearHandAuthorityRoleNow(scope_safe_hand_frame_math::HandAuthorityRole role, bool isLeft);

        void reconcileDeferredScopeHandAuthority(RE::NiNode* weaponNode);

        bool applyWeaponVisualAuthority(
            RE::NiNode* weaponNode,
            const RE::NiTransform& solvedWeaponWorld,
            std::uint64_t authorityGenerationKey = 0);

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

        void clearPrimaryGripPose(bool isLeft);

        static void killFrikOffhandGrip();

        static void restoreFrikOffhandGrip();

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
        void clearWeaponVisualReturn(const char* reason, bool logCancellation, bool restoreBlockers);
        void clearAllVisualReturns(const char* reason, bool logCancellation, bool restoreBlockers);
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
        void refreshScopeSafeHandFrames(const EquippedWeaponGripFrameInput& frameInput, float dt);
        bool tryGetSolverHandTransform(bool isLeft, RE::NiTransform& outTransform) const;
        RE::NiTransform resolveLockedHandVisualTarget(
            const RE::NiTransform& targetWorld,
            const RE::NiTransform* liveHandWorld,
            float dt,
            LockedHandVisualLerpState& state);

        TwoHandedState _state{ TwoHandedState::Inactive };

        // Reused one-shot solve storage. It owns no engine pointers and keeps
        // bounded vector/BVH capacity across re-grabs without polluting the
        // per-frame WeaponPartGrip state.
        std::unique_ptr<FingerPoseSolveScratch> _fingerPoseSolveScratch;

        weapon_support_authority_policy::WeaponSupportAuthorityMode _authorityMode{
            weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver
        };

        /*
         * Physical hand that owns the firing grip when occupied. Seeded right
         * (false) and reset to right whenever authority clears (weapon change,
         * holster, drop, teardown). Flipped only through setFiringHand() from
         * a hand-specific loose-weapon equip or the two takeover paths
         * (free-hand firing-grip squeeze in PartCarry, support-grip promotion
         * on firing-hand release); PhysicsInteraction and InputRemap read
         * isFiringHandLeft() each frame to route input and ownership per role.
         */
        bool _firingHandIsLeft{ false };

        // FRIK weapon-node ownership block + reparent bookkeeping for
        // left-firing carry; see syncFiringHandWeaponNodeOwnership().
        bool _weaponNodeOwnershipBlockEngaged{ false };
        bool _weaponNodeReparentedToLeftHand{ false };
        bool _recoilControllerRegistered{ false };

        enum class RightFiringCanonicalSource : std::uint8_t
        {
            None,
            NativeCarry,
            AuthoredAnimation,
        };

        // Canonical right-hand firing hold, keyed by node identity, collision
        // generation, and equipped ownership; see
        // rememberRightFiringHandCanonicalFrame().
        RE::NiTransform _rightFiringHandCanonicalWeaponLocal{};
        RE::NiPoint3 _rightFiringGripCanonicalWeaponLocal{};
        // Non-owning identity witness only; compared, never dereferenced.
        RE::NiNode* _rightFiringHandCanonicalWeaponNode{ nullptr };
        std::uint64_t _rightFiringHandCanonicalGenerationKey{ 0 };
        std::uint64_t _rightFiringHandCanonicalOwnershipKey{ 0 };
        std::uint64_t _rightFiringHandCanonicalCaptureSequence{ 0 };
        RightFiringCanonicalSource _rightFiringHandCanonicalSource{
            RightFiringCanonicalSource::None
        };
        bool _hasRightFiringHandCanonicalWeaponLocal{ false };
        std::array<RE::NiTransform, 15> _rightFiringFingerLocalTransforms{};
        std::array<RE::NiTransform, 15> _leftFiringFingerLocalTransforms{};
        std::uint16_t _rightFiringFingerLocalTransformMask{ 0 };
        std::uint16_t _leftFiringFingerLocalTransformMask{ 0 };
        bool _publishedFiringFingerPoseIsLeft{ false };
        bool _authoredPrimaryFingerPosePublished{ false };
        bool _authoredPrimaryFingerPoseBlockEngaged{ false };
        bool _authoredPrimaryFingerPoseSuppressed{ false };
        bool _leftHandHoldingObjectForPose{ false };
        bool _rightHandHoldingObjectForPose{ false };

        /*
         * Natural physical hand-bone-in-wand relations. They are refreshed
         * only while ROCK has no visual authority for that hand and are
         * deliberately not weapon-generation keyed. The firing and support
         * mirrors consume these anatomy frames instead of a locked hand pose.
         */
        RE::NiTransform _rightNaturalBoneInWand{};
        RE::NiTransform _leftNaturalBoneInWand{};
        bool _hasRightNaturalBoneInWand{ false };
        bool _hasLeftNaturalBoneInWand{ false };

        std::array<ScopeSafeHandFrameState, 2> _scopeSafeHandFrames{};
        bool _scopeMenuOpenThisFrame{ false };
        // True only for the first visible frame after ScopeMenu. Role clears
        // on this edge use the same deferred transaction as hidden-frame
        // clears so a same-frame handoff can publish its replacement first.
        bool _scopeMenuClosedThisFrame{ false };
        // Latched across a manual grip session after its first scoped frame so
        // ScopeMenu presentation edges cannot reselect the weapon-solver basis.
        bool _scopeDriverFrameAuthorityActive{ false };
        // Per physical hand (left index 0, right index 1). Clears requested
        // while hFRIK's root is collapsed remain role-specific so scope exit
        // never tears down authority that the current weapon state still owns.
        std::array<scope_safe_hand_frame_math::HandAuthorityRoleMask, 2> _scopeDeferredHandAuthorityClears{};
        std::array<scope_safe_hand_frame_math::HandAuthorityRoleMask, 2> _scopeHandAuthorityPublishedThisFrame{};

        // Generation-bound resolved anchor. Generated sight evidence remains
        // preferred; malformed/missing optics use the current firing-grip
        // origin plus the configured Weapon-local offset. The pointer is an
        // identity witness only and is never dereferenced from this cache.
        RE::NiNode* _nativeScopeAnchorWeaponNode{ nullptr };
        std::uint64_t _nativeScopeAnchorGenerationKey{ 0 };
        std::uint64_t _nativeScopeAnchorOwnershipKey{ 0 };
        std::uint32_t _nativeScopeAnchorWeaponFormID{ 0 };
        RE::NiPoint3 _nativeScopeAnchorWeaponLocal{};
        native_scope_sight_anchor_policy::AnchorSource _nativeScopeAnchorSource{
            native_scope_sight_anchor_policy::AnchorSource::None
        };
        bool _nativeScopeAnchorValid{ false };
        RE::NiPoint3 _nativeScopeFallbackRotationDegrees{};
        // Exit-only stabilization is generation-bound; entry remains immediate.
        std::uint64_t _nativeScopeExitDebounceGenerationKey{ 0 };
        std::uint32_t _nativeScopeExitOutsideFrames{ 0 };
        NativeScopeCameraDebugSnapshot _nativeScopeCameraDebugSnapshot{};
        NativeScopeActivationDebugSnapshot _nativeScopeActivationDebugSnapshot{};
        NativeScopeRigidFrameState _nativeScopeRigidFrame{};
        NativeScopeOverlayCalibrationState _nativeScopeOverlayCalibration{};

        std::array<WeaponPartGrip, 2> _partGrips{};
        AuthoredSupportGripCandidate _authoredSupportGripCandidate{};

        /*
         * Monotonic capture sequences so API consumers can detect a re-grab
         * of the same part without frame-edge callbacks. One counter for part
         * grips (stamped per capture) and one for fresh firing-grip captures.
         */
        std::uint64_t _gripCaptureSequence{ 0 };
        std::uint64_t _firingGripSequence{ 0 };

        /*
         * Which active part grip anchors the part-carry solve. The older grip
         * carries the weapon (translation pivot); the newer grip aims it.
         */
        bool _partCarryPivotIsLeft{ true };

        RE::NiPoint3 _primaryGripLocal{};

        float _lockedGripSeparationWorld{ 0.0f };

        float _partCarryGripSeparationWorld{ 0.0f };

        int _touchFrames{ 0 };
        static constexpr int TOUCH_TIMEOUT_FRAMES = 5;

        float _rotationBlend{ 0.0f };
        static constexpr float ROTATION_BLEND_SPEED = 8.0f;

        int _gripLogCounter{ 0 };

        RE::NiTransform _lastSolvedWeaponTransform{};

        bool _hasSolvedWeaponTransform{ false };

        RE::NiTransform _primaryHandWeaponLocal{};

        bool _hasFiringHandWeaponLocal{ false };

        LockedHandVisualLerpState _primaryHandVisualLerp{};
        DynamicSupportAcquisitionState _dynamicSupportAcquisition{};

        std::array<ReturningHandVisualState, 2> _returningHandVisuals{};
        std::array<RE::NiTransform, 2> _lastPublishedHandWorld{};
        std::array<bool, 2> _hasLastPublishedHandWorld{};
        ReturningWeaponVisualState _returningWeaponVisual{};
        RE::NiTransform _lastRenderedWeaponWorld{};
        bool _hasLastRenderedWeaponWorld{ false };

        float _primaryGripConfidence{ 0.0f };

        RE::NiNode* _activeWeaponNode{ nullptr };
        std::uint64_t _activeWeaponGenerationKey{ 0 };
        std::uint64_t _activeEquippedWeaponOwnershipKey{ 0 };
        equipped_weapon_manual_ownership_policy::GripReleaseDebounceState _primaryReleaseDebounce{};
        bool _persistentEquippedCarryActive{ false };
        bool _persistentEquippedCarryDetachArmed{ false };

        /*
         * Whole frames spent in Gripping since the support grab was captured.
         * Gates promotion/detach on a confirmed firing-grip release: a
         * release that confirms while this is still fresh is the same
         * gesture (or a grab-synchronized grip flicker) and is deferred.
         */
        std::uint32_t _supportGripAgeFrames{ 0 };
        bool _freshSupportGripDeferLogged{ false };

        RE::NiTransform _weaponNodeLocalBaseline{};
        bool _hasWeaponNodeLocalBaseline{ false };

        EquippedWeaponManualDropRequest _equippedWeaponDropRequest{};
        TwoHandedGripHapticEvents _hapticEvents{};
        EquippedWeaponHandlingSettings _handlingSettings{};

        // Per-frame hover state; only ever true in PartCarry (see getter).
        bool _firingGripReattachHoverInsideRadius{ false };
        bool _firingGripReattachHoverHandIsLeft{ false };

        // Rate limiter for the left-firing carry aim diagnostic.
        int _leftFiringAimLogCounter{ 0 };
    };

}
