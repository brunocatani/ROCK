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

    enum class TwoHandedState
    {
        Inactive,
        Touching,
        Gripping,
        PrimaryDetached,
        PrimaryDetachedManipulation,
        PrimaryOnly,
    };

    struct EquippedWeaponPrimaryGripInput
    {
        bool held{ false };
        bool pressed{ false };
        bool released{ false };
        RE::NiTransform handWorld{};
        bool hasHandWorld{ false };
        RE::NiPoint3 firingGripProbeWorld{};
        bool hasFiringGripProbeWorld{ false };
        bool handHoldingObject{ false };
    };

    struct EquippedWeaponFiringGripReference
    {
        bool valid{ false };
        RE::NiPoint3 gripLocal{};
        const RE::NiAVObject* weaponRoot{ nullptr };
        const char* reason{ "notEvaluated" };
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
            bool leftGripPressed,
            bool supportHandHoldingObject,
            float dt,
            std::uint64_t currentWeaponGenerationKey,
            const WeaponCollision& weaponCollision,
            const WeaponInteractionRuntimeState& supportRuntimeState,
            const WeaponInteractionRuntimeState& detachedPrimaryRuntimeState,
            weapon_support_authority_policy::WeaponSupportAuthorityMode supportAuthorityMode,
            bool primaryDetachEnabled,
            const EquippedWeaponPrimaryGripInput& primaryGripInput,
            const EquippedWeaponFiringGripReference& firingGripReference);

        void reset();

        bool isGripping() const
        {
            return _state == TwoHandedState::Gripping ||
                   _state == TwoHandedState::PrimaryDetached ||
                   _state == TwoHandedState::PrimaryDetachedManipulation;
        }

        bool isManualOwnershipActive() const
        {
            return _state == TwoHandedState::Gripping ||
                   _state == TwoHandedState::PrimaryDetached ||
                   _state == TwoHandedState::PrimaryDetachedManipulation ||
                   _state == TwoHandedState::PrimaryOnly;
        }

        bool isPrimaryDetached() const
        {
            return _state == TwoHandedState::PrimaryDetached ||
                   _state == TwoHandedState::PrimaryDetachedManipulation;
        }

        bool isDetachedPrimarySupportGripActive() const { return _state == TwoHandedState::PrimaryDetachedManipulation; }

        bool canUsePrimaryDetachInput() const
        {
            return _state == TwoHandedState::Gripping ||
                   _state == TwoHandedState::PrimaryDetached ||
                   _state == TwoHandedState::PrimaryDetachedManipulation ||
                   _state == TwoHandedState::PrimaryOnly;
        }

        bool isTouching() const { return _state == TwoHandedState::Touching; }

        TwoHandedState getState() const { return _state; }

        weapon_support_authority_policy::WeaponSupportAuthorityMode getAuthorityMode() const { return _authorityMode; }

        bool ownsWeaponTransform() const;

        bool getSolvedWeaponTransform(RE::NiTransform& outTransform) const;

        bool getDebugAuthoritySnapshot(TwoHandedGripDebugSnapshot& outSnapshot) const;

        bool beginPrimaryOnlyGrip(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey);

        EquippedWeaponManualDropRequest consumeEquippedWeaponDropRequest();

    private:
        void transitionToTouching(RE::NiNode* weaponNode, const WeaponInteractionDecision& decision);
        void transitionToGripping(
            RE::NiNode* weaponNode,
            const WeaponInteractionDecision& decision,
            const WeaponCollision& weaponCollision,
            weapon_support_authority_policy::WeaponSupportAuthorityMode supportAuthorityMode,
            const WeaponProviderPartAuthority& providerPartAuthority);
        void transitionToInactive(bool publishRestoredWeaponTransform);

        void updateGripping(RE::NiNode* weaponNode, float dt);

        bool providerPartAuthorityStillCurrent(std::uint64_t currentWeaponGenerationKey) const;

        bool providerPartAuthorityStillCurrent(const WeaponProviderPartAuthority& authority, std::uint64_t currentWeaponGenerationKey) const;

        void clearProviderPartAuthority();

        void updateFullWeaponAuthorityGrip(RE::NiNode* weaponNode, float dt);

        void updatePrimaryDetachedGrip(
            RE::NiNode* weaponNode,
            float dt,
            const EquippedWeaponPrimaryGripInput& primaryGripInput,
            const WeaponInteractionContact& rightWeaponContact,
            const WeaponCollision& weaponCollision,
            const WeaponInteractionRuntimeState& detachedPrimaryRuntimeState,
            const EquippedWeaponFiringGripReference& firingGripReference);

        void updatePrimaryDetachedManipulationGrip(
            RE::NiNode* weaponNode,
            float dt,
            const EquippedWeaponPrimaryGripInput& primaryGripInput,
            const WeaponInteractionRuntimeState& detachedPrimaryRuntimeState);

        void updateVisualOnlySupportGrip(RE::NiNode* weaponNode, float dt);

        bool transitionToPrimaryDetached();

        bool transitionToPrimaryOnly(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey, const char* reason);

        void requestEquippedWeaponDrop(const char* reason, equipped_weapon_drop_policy::SourceHand sourceHand);

        void updatePrimaryOnlyGrip(
            RE::NiNode* weaponNode,
            std::uint64_t currentWeaponGenerationKey,
            const EquippedWeaponPrimaryGripInput& primaryGripInput);

        bool tryReattachPrimaryGrip(
            RE::NiNode* weaponNode,
            const EquippedWeaponPrimaryGripInput& primaryGripInput,
            const EquippedWeaponFiringGripReference& firingGripReference);

        bool primaryGripContactMatchesFiringGrip(
            RE::NiNode* weaponNode,
            const RE::NiPoint3& primaryProbeWorld,
            const EquippedWeaponFiringGripReference& firingGripReference) const;

        bool tryStartDetachedPrimarySupportGrip(
            RE::NiNode* weaponNode,
            const WeaponInteractionContact& rightWeaponContact,
            const WeaponCollision& weaponCollision,
            const EquippedWeaponPrimaryGripInput& primaryGripInput,
            const WeaponInteractionRuntimeState& detachedPrimaryRuntimeState);

        void clearDetachedPrimarySupportGrip();

        RE::NiPoint3 resolveDetachedPrimarySupportGripWorld(RE::NiNode* weaponNode) const;
        RE::NiPoint3 resolveDetachedPrimarySupportGripWeaponLocal(RE::NiNode* weaponNode) const;
        RE::NiAVObject* resolveCurrentDetachedPrimarySupportAttachmentRoot(RE::NiNode* weaponNode) const;
        RE::NiTransform resolveDetachedPrimarySupportHandWorld(RE::NiNode* weaponNode) const;

        void setSupportGripPose(bool isLeft, WeaponGripPoseId poseId, const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPose);

        void clearSupportGripPose(bool isLeft);

        void setDetachedPrimarySupportGripPose(WeaponGripPoseId poseId, const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPose);

        void clearDetachedPrimarySupportGripPose();

        void clearPrimaryDetachVisualAuthority(bool isLeft);

        bool applyWeaponVisualAuthority(RE::NiNode* weaponNode, const RE::NiTransform& solvedWeaponWorld);

        bool applyLockedHandVisualAuthority(
            RE::NiNode* weaponNode,
            bool applyPrimaryHand,
            bool applySupportHand,
            float dt,
            const RE::NiTransform* livePrimaryHandWorld = nullptr,
            const RE::NiTransform* liveSupportHandWorld = nullptr);

        void publishGripHandPoses(bool supportHandIsLeft);

        void publishDetachedPrimarySupportGripPose();

        void clearPrimaryGripPose(bool isLeft);

        static void killFrikOffhandGrip();

        static void restoreFrikOffhandGrip();

        static bool blockFrikPrimaryWeaponPose();

        static void restoreFrikPrimaryWeaponPose();

        static RE::NiPoint3 worldToWeaponLocal(const RE::NiPoint3& worldPos, const RE::NiAVObject* weaponNode);

        static RE::NiPoint3 weaponLocalToWorld(const RE::NiPoint3& localPos, const RE::NiAVObject* weaponNode);

        RE::NiPoint3 resolveSupportGripWorld(RE::NiNode* weaponNode) const;

        RE::NiPoint3 resolveSupportGripWeaponLocal(RE::NiNode* weaponNode) const;

        RE::NiPoint3 resolveSupportNormalWeaponLocal(RE::NiNode* weaponNode) const;

        RE::NiTransform resolveSupportHandWorld(RE::NiNode* weaponNode) const;

        RE::NiAVObject* resolveCurrentSupportAttachmentRoot(RE::NiNode* weaponNode) const;

        struct LockedHandVisualLerpState
        {
            bool active = false;
            RE::NiTransform startWorld{};
            float elapsedSeconds = 0.0f;
            float durationSeconds = 0.0f;
            float lastAlpha = 1.0f;
        };

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

        RE::NiPoint3 _offhandGripLocal{};

        RE::NiPoint3 _primaryGripLocal{};

        RE::NiPoint3 _grabNormal{};

        RE::NiPoint3 _supportNormalLocal{};

        RE::NiPoint3 _offhandGripSourceLocal{};

        RE::NiPoint3 _supportNormalSourceLocal{};

        float _lockedGripSeparationWorld{ 0.0f };

        WeaponGripPoseId _supportGripPose{ WeaponGripPoseId::BarrelWrap };

        WeaponPartKind _supportPartKind{ WeaponPartKind::Other };

        int _touchFrames{ 0 };
        static constexpr int TOUCH_TIMEOUT_FRAMES = 5;

        float _rotationBlend{ 0.0f };
        static constexpr float ROTATION_BLEND_SPEED = 8.0f;

        int _gripLogCounter{ 0 };

        RE::NiTransform _lastSolvedWeaponTransform{};

        bool _hasSolvedWeaponTransform{ false };

        RE::NiTransform _primaryHandWeaponLocal{};

        RE::NiTransform _supportHandWeaponLocal{};

        RE::NiTransform _supportHandSourceLocal{};

        RE::NiTransform _supportAttachmentWeaponLocal{};

        bool _hasHandWeaponLocalFrames{ false };

        bool _hasSupportSourceLocalFrame{ false };

        bool _hasSupportAttachmentWeaponLocal{ false };

        LockedHandVisualLerpState _primaryHandVisualLerp{};
        LockedHandVisualLerpState _supportHandVisualLerp{};

        std::array<float, 15> _supportFingerPose{};
        std::array<float, 5> _supportFingerSplayRadians{};
        std::array<RE::NiTransform, 15> _supportFingerLocalTransforms{};
        std::uint16_t _supportFingerLocalTransformMask{ 0 };
        bool _hasSupportFingerPose{ false };
        bool _hasSupportFingerSplay{ false };
        bool _hasSupportFingerLocalTransforms{ false };

        RE::NiPoint3 _detachedPrimarySupportGripLocal{};
        RE::NiPoint3 _detachedPrimarySupportGripSourceLocal{};
        RE::NiPoint3 _detachedPrimarySupportNormalLocal{};
        RE::NiPoint3 _detachedPrimarySupportNormalSourceLocal{};
        RE::NiTransform _detachedPrimarySupportHandWeaponLocal{};
        RE::NiTransform _detachedPrimarySupportHandSourceLocal{};
        RE::NiTransform _detachedPrimarySupportAttachmentWeaponLocal{};
        RE::NiAVObject* _detachedPrimarySupportAttachmentRoot{ nullptr };
        bool _hasDetachedPrimarySupportSourceLocalFrame{ false };
        bool _hasDetachedPrimarySupportAttachmentWeaponLocal{ false };
        std::array<float, 15> _detachedPrimarySupportFingerPose{};
        std::array<float, 5> _detachedPrimarySupportFingerSplayRadians{};
        std::array<RE::NiTransform, 15> _detachedPrimarySupportFingerLocalTransforms{};
        std::uint16_t _detachedPrimarySupportFingerLocalTransformMask{ 0 };
        bool _hasDetachedPrimarySupportFingerPose{ false };
        bool _hasDetachedPrimarySupportFingerSplay{ false };
        bool _hasDetachedPrimarySupportFingerLocalTransforms{ false };
        LockedHandVisualLerpState _detachedPrimarySupportHandVisualLerp{};

        float _primaryGripConfidence{ 0.0f };

        RE::NiNode* _activeWeaponNode{ nullptr };
        RE::NiAVObject* _activeSourceRoot{ nullptr };
        RE::NiAVObject* _supportAttachmentRoot{ nullptr };
        std::uint64_t _activeWeaponGenerationKey{ 0 };
        WeaponProviderPartAuthority _providerPartAuthority{};
        WeaponProviderPartAuthority _detachedPrimarySupportProviderPartAuthority{};
        RE::NiTransform _weaponNodeLocalBaseline{};
        bool _hasWeaponNodeLocalBaseline{ false };

        EquippedWeaponManualDropRequest _equippedWeaponDropRequest{};
    };

}
