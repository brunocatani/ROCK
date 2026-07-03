#pragma once

#include <atomic>
#include <array>

#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/weapon/EquippedWeaponDropPolicy.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
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

    struct EquippedWeaponGripFrameInput
    {
        bool leftGripHeld{ false };
        bool leftHandHoldingObject{ false };
        bool rightHandHoldingObject{ false };
        bool reattachChordPressed{ false };
        bool reattachAutoEligible{ false };
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

    struct EquippedWeaponManualDropRequest
    {
        bool requested{ false };
        equipped_weapon_drop_policy::SourceHand sourceHand{ equipped_weapon_drop_policy::SourceHand::None };
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
            const WeaponCollision& weaponCollision,
            const WeaponInteractionRuntimeState& leftRuntimeState,
            const WeaponInteractionRuntimeState& rightRuntimeState,
            weapon_support_authority_policy::WeaponSupportAuthorityMode supportAuthorityMode,
            bool primaryDetachEnabled);

        void reset();

        bool isGripping() const { return _state == TwoHandedState::Gripping || _state == TwoHandedState::PartCarry; }

        bool isManualOwnershipActive() const
        {
            return _state == TwoHandedState::Gripping ||
                   _state == TwoHandedState::PartCarry ||
                   _state == TwoHandedState::PrimaryOnly;
        }

        bool isPartCarryActive() const { return _state == TwoHandedState::PartCarry; }

        bool isHandPartGripping(bool isLeft) const { return partGrip(isLeft).active; }

        bool isFiringGripOccupied() const { return _state == TwoHandedState::Gripping || _state == TwoHandedState::PrimaryOnly; }

        bool canUsePrimaryDetachInput() const
        {
            return _state == TwoHandedState::Gripping ||
                   _state == TwoHandedState::PartCarry ||
                   _state == TwoHandedState::PrimaryOnly;
        }

        bool isTouching() const { return _state == TwoHandedState::Touching; }

        TwoHandedState getState() const { return _state; }

        weapon_support_authority_policy::WeaponSupportAuthorityMode getAuthorityMode() const { return _authorityMode; }

        bool ownsWeaponTransform() const;

        bool getSolvedWeaponTransform(RE::NiTransform& outTransform) const;

        bool getDebugAuthoritySnapshot(TwoHandedGripDebugSnapshot& outSnapshot) const;

        bool beginPrimaryOnlyGrip(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey);

        /*
         * FRIK re-attaches the weapon node to the firing hand every frame
         * before ROCK runs, even while the firing hand is detached. Callers
         * must republish ROCK's solved part-carry transform before reading the
         * weapon node (probes, grip-zone checks, capture frames), or every
         * weapon-relative computation sees the weapon glued to the firing hand.
         */
        bool republishPartCarryWeaponTransform(RE::NiNode* weaponNode);

        EquippedWeaponManualDropRequest consumeEquippedWeaponDropRequest();

    private:
        struct LockedHandVisualLerpState
        {
            bool active = false;
            RE::NiTransform startWorld{};
            float elapsedSeconds = 0.0f;
            float durationSeconds = 0.0f;
            float lastAlpha = 1.0f;
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
            const WeaponProviderPartAuthority& providerPartAuthority);
        void transitionToInactive(bool publishRestoredWeaponTransform);

        void updateGripping(RE::NiNode* weaponNode, float dt);

        bool providerPartAuthorityStillCurrent(const WeaponPartGrip& grip, std::uint64_t currentWeaponGenerationKey) const;

        void updateFullWeaponAuthorityGrip(RE::NiNode* weaponNode, float dt);

        void updatePartCarryGrip(
            RE::NiNode* weaponNode,
            float dt,
            const EquippedWeaponGripFrameInput& frameInput,
            const WeaponInteractionContact& leftWeaponContact,
            const WeaponInteractionContact& rightWeaponContact,
            const WeaponCollision& weaponCollision,
            std::uint64_t currentWeaponGenerationKey,
            const WeaponInteractionRuntimeState& leftRuntimeState,
            const WeaponInteractionRuntimeState& rightRuntimeState);

        bool solvePartCarryWeaponAuthority(RE::NiNode* weaponNode, float dt);

        float partCarryGripSeparation(RE::NiNode* weaponNode) const;

        void updateVisualOnlySupportGrip(RE::NiNode* weaponNode, float dt);

        bool transitionToPartCarry();

        bool transitionToPrimaryOnly(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey, const char* reason);

        void requestEquippedWeaponDrop(const char* reason, equipped_weapon_drop_policy::SourceHand sourceHand);

        void updatePrimaryOnlyGrip(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            const EquippedWeaponPrimaryGripInput& primaryGripInput);

        bool tryReattachFiringGrip(RE::NiNode* weaponNode, const WeaponInteractionContact& firingHandWeaponContact, bool fromChord);

        bool firingGripContactMatchesCapturedGrip(
            RE::NiNode* weaponNode,
            const WeaponInteractionContact& firingHandWeaponContact,
            const RE::NiTransform& firingHandTransform,
            bool logRejections) const;

        bool tryComputeFiringPalmToGripDistance(RE::NiNode* weaponNode, float& outDistance) const;

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
         * (false); flipping this is the designed entry point for left-handed
         * mode once PhysicsInteraction's per-hand suppression and input reads
         * are role-driven as well. Do not flip it in isolation.
         */
        bool _firingHandIsLeft{ false };

        std::array<WeaponPartGrip, 2> _partGrips{};

        /*
         * Which active part grip anchors the part-carry solve. The older grip
         * carries the weapon (translation pivot); the newer grip aims it.
         */
        bool _partCarryPivotIsLeft{ true };

        RE::NiPoint3 _primaryGripLocal{};

        float _lockedGripSeparationWorld{ 0.0f };

        float _partCarryGripSeparationWorld{ 0.0f };

        /*
         * Proximity auto-reattach hysteresis: false until the free firing palm
         * has left the reattach radius (with margin) after entering PartCarry,
         * so detaching never gets instantly re-captured. Reset on every
         * PartCarry entry.
         */
        bool _autoReattachArmed{ false };

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

        float _primaryGripConfidence{ 0.0f };

        RE::NiNode* _activeWeaponNode{ nullptr };
        std::uint64_t _activeWeaponGenerationKey{ 0 };
        RE::NiTransform _weaponNodeLocalBaseline{};
        bool _hasWeaponNodeLocalBaseline{ false };

        EquippedWeaponManualDropRequest _equippedWeaponDropRequest{};
    };

}
