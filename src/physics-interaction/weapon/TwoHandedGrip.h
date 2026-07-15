#pragma once

#include <atomic>
#include <array>

#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/weapon/EquippedWeaponDropPolicy.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
#include "physics-interaction/weapon/WeaponPartGripReportPolicy.h"
#include "physics-interaction/weapon/WeaponSupport.h"

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTransform.h"

namespace rock
{
    class WeaponCollision;

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

    struct EquippedWeaponGripMode
    {
        // Keeps ROCK's firing-hand role/left-hand carry state alive.
        bool firingGripOwnershipEnabled{ false };
        // Allows the wrapped support hand to inherit the firing grip.
        bool ambidextrousHandoffEnabled{ false };
        // Enables realistic firing-hand detach, part carry, and drop.
        bool primaryDetachEnabled{ false };
    };

    struct TwoHandedGripDebugSnapshot
    {
        RE::NiTransform weaponWorld{};
        RE::NiTransform rightRequestedHandWorld{};
        RE::NiTransform leftRequestedHandWorld{};
        RE::NiPoint3 rightGripWorld{};
        RE::NiPoint3 leftGripWorld{};
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
            const EquippedWeaponGripMode& gripMode);

        void reset();

        bool isGripping() const { return _state == TwoHandedState::Gripping || _state == TwoHandedState::PartCarry; }

        bool isManualOwnershipActive() const
        {
            return _state == TwoHandedState::Gripping ||
                   _state == TwoHandedState::PartCarry ||
                   _state == TwoHandedState::PrimaryOnly;
        }

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

        bool getSolvedWeaponTransform(RE::NiTransform& outTransform) const;

        bool getCollisionRequestedWeaponTransform(
            const RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            const RE::NiTransform* rawRightHandWorld,
            RE::NiTransform& outTransform) const;

        bool applyCollisionResolvedWeaponAuthority(
            RE::NiNode* weaponNode,
            const RE::NiTransform& requestedWeaponWorld,
            const RE::NiTransform& resolvedWeaponWorld,
            const RE::NiTransform* rawRightHandWorld,
            const RE::NiTransform* rawLeftHandWorld,
            float dt);

        void clearCollisionResolvedWeaponAuthority();

        bool getDebugAuthoritySnapshot(TwoHandedGripDebugSnapshot& outSnapshot) const;

        bool beginPrimaryOnlyGrip(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            std::uint64_t currentEquippedWeaponOwnershipKey,
            bool firingHandIsLeft,
            const RE::NiTransform* capturedFiringHandWeaponLocal,
            const RE::NiPoint3* capturedFiringGripWeaponLocal);

        // Left-hand primary ownership requires the hFRIK ambidextrous weapon-
        // node blockers; right-hand native ownership is always eligible.
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
        struct LockedHandVisualLerpState
        {
            bool active = false;
            RE::NiTransform startWorld{};
            float elapsedSeconds = 0.0f;
            float durationSeconds = 0.0f;
            float lastAlpha = 1.0f;
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
        void refreshRightNativeCanonicalFrame(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey);

        bool tryComputeMirroredLeftFiringHandWeaponLocal(RE::NiTransform& outHandWeaponLocal) const;

        /*
         * Reattach validates the hand first and only then commits; a takeover
         * by the non-firing hand flips the firing-hand role inside the commit
         * (setFiringHand), reusing the SAME captured weapon-relative grip
         * frames - the hands only choose who fires, the grip stays
         * weapon-relative.
         */
        bool tryReattachFiringGrip(bool handIsLeft, RE::NiNode* weaponNode, const WeaponInteractionContact& handWeaponContact);

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
            const WeaponProviderPartAuthority& providerPartAuthority);

        void lockPartGripToWeaponRoot(bool isLeft);

        void releasePartGrip(bool isLeft, const char* reason);

        void setSupportGripPose(bool isLeft, WeaponGripPoseId poseId, const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPose);

        void clearSupportGripPose(bool isLeft);

        void clearPrimaryDetachVisualAuthority(bool isLeft);

        bool applyWeaponVisualAuthority(RE::NiNode* weaponNode, const RE::NiTransform& solvedWeaponWorld);

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
        void refreshNativeScopeSightAnchor(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey, const WeaponCollision& weaponCollision);
        void refreshScopeSafeHandFrames(RE::NiNode* weaponNode, const EquippedWeaponGripFrameInput& frameInput, float dt);
        bool tryGetSolverHandTransform(bool isLeft, RE::NiTransform& outTransform) const;
        RE::NiTransform resolveLockedHandVisualTarget(
            const RE::NiTransform& targetWorld,
            const RE::NiTransform* liveHandWorld,
            float dt,
            LockedHandVisualLerpState& state);

        TwoHandedState _state{ TwoHandedState::Inactive };

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

        // Canonical right-hand firing hold (weapon-generation-keyed); see
        // rememberRightFiringHandCanonicalFrame().
        RE::NiTransform _rightFiringHandCanonicalWeaponLocal{};
        std::uint64_t _rightFiringHandCanonicalGenerationKey{ 0 };
        bool _hasRightFiringHandCanonicalWeaponLocal{ false };

        /*
         * Natural right hand-bone-in-wand relation, snapshotted only while
         * the weapon rides the native right carry (hand guaranteed unlocked,
         * same gates as the canonical refresh). Weapon-independent anatomy,
         * so it is deliberately NOT generation-keyed. Consumed by the left
         * mirror when the live right bone is part-grip-locked and therefore
         * not expressing the wand-riding relation the conjugation needs.
         */
        RE::NiTransform _rightNaturalBoneInWand{};
        bool _hasRightNaturalBoneInWand{ false };

        std::array<ScopeSafeHandFrameState, 2> _scopeSafeHandFrames{};
        bool _scopeMenuOpenThisFrame{ false };
        bool _scopeHandAuthorityCleanupPending{ false };

        // Cached once per generated weapon generation. The pointer is only an
        // identity witness; the anchor itself is a value in weapon-root local
        // space and is never derived from a retained transient engine object.
        RE::NiNode* _nativeScopeSightAnchorWeaponNode{ nullptr };
        std::uint64_t _nativeScopeSightAnchorGenerationKey{ 0 };
        RE::NiPoint3 _nativeScopeSightAnchorWeaponLocal{};
        bool _nativeScopeSightAnchorValid{ false };

        std::array<WeaponPartGrip, 2> _partGrips{};

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
        bool _collisionResolvedWeaponAuthorityActive{ false };

        RE::NiTransform _primaryHandWeaponLocal{};

        bool _hasFiringHandWeaponLocal{ false };

        LockedHandVisualLerpState _primaryHandVisualLerp{};

        float _primaryGripConfidence{ 0.0f };

        RE::NiNode* _activeWeaponNode{ nullptr };
        std::uint64_t _activeWeaponGenerationKey{ 0 };
        std::uint64_t _activeEquippedWeaponOwnershipKey{ 0 };
        equipped_weapon_manual_ownership_policy::GripReleaseDebounceState _primaryReleaseDebounce{};

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

        // Per-frame hover state; only ever true in PartCarry (see getter).
        bool _firingGripReattachHoverInsideRadius{ false };
        bool _firingGripReattachHoverHandIsLeft{ false };

        // Rate limiter for the left-firing carry aim diagnostic.
        int _leftFiringAimLogCounter{ 0 };
    };

}
