#pragma once

#include "physics-interaction/contact/SoftContactMath.h"
#include "physics-interaction/contact/NativeContactEvidence.h"
#include "physics-interaction/contact/ContactTargetIdentity.h"
#include "physics-interaction/core/PhysicsFrameContext.h"

#include <array>
#include <cstdint>

namespace rock
{
    class Hand;

    struct SoftContactDebugContact
    {
        bool valid = false;
        bool isLeft = false;
        bool suppressed = false;
        soft_contact_math::ContactKind kind = soft_contact_math::ContactKind::None;
        soft_contact_math::ContactState state = soft_contact_math::ContactState::Inactive;
        RE::NiPoint3 point{};
        RE::NiPoint3 normalEnd{};
        RE::NiPoint3 correctionEnd{};
        float penetration = 0.0f;
        float responseScale = 0.0f;
        float maxCorrection = 0.0f;
        float correctionLength = 0.0f;
        std::uint32_t movableId = 0;
        std::uint32_t targetId = 0;
        std::uint32_t targetLayer = contact_target_identity::kUnknownLayer;
        std::uint32_t targetFilterInfo = contact_target_identity::kUnknownFilterInfo;
        std::uint32_t targetRefFormId = contact_target_identity::kInvalidFormId;
        std::uint32_t targetBaseFormId = contact_target_identity::kInvalidFormId;
        contact_target_identity::SurfaceHint surfaceHint = contact_target_identity::SurfaceHint::Unknown;
    };

    struct SoftContactDebugSnapshot
    {
        static constexpr std::size_t kMaxContacts = 16;
        std::array<SoftContactDebugContact, kMaxContacts> contacts{};
        std::uint32_t contactCount = 0;
        soft_contact_math::ContactState rightState = soft_contact_math::ContactState::Inactive;
        soft_contact_math::ContactState leftState = soft_contact_math::ContactState::Inactive;
    };

    class SoftContactRuntime
    {
    public:
        static constexpr std::size_t kMaxWorldContactProbesPerHand = 7;

        void reset();

        void clearHandForStrongerOwner(bool isLeft, const char* reason);

        void update(const PhysicsFrameContext& frame,
            const Hand& rightHand,
            const Hand& leftHand,
            bool rightHandWeaponEquipped,
            bool leftSupportGripActive,
            const contact_evidence::NativeContactEvidenceSnapshot& nativeContactEvidence);

        bool getDebugSnapshot(SoftContactDebugSnapshot& outSnapshot) const;

    private:
        struct HandRuntime
        {
            struct WorldProbeState
            {
                bool valid = false;
                RE::NiPoint3 previous{};
                float restQueryCooldownSeconds = 0.0f;
            };

            struct CachedWorldPlane
            {
                bool active = false;
                std::uint32_t bodyId = 0x7FFF'FFFFu;
                std::uint32_t probeId = 0;
                RE::NiPoint3 surfacePoint{};
                RE::NiPoint3 normal{};
                float approachSpeedGameUnits = 0.0f;
                contact_target_identity::ContactTargetIdentity targetIdentity{};
            };

            soft_contact_math::ContactState state = soft_contact_math::ContactState::Inactive;
            soft_contact_math::ContactKind lastContactKind = soft_contact_math::ContactKind::None;
            RE::NiPoint3 correction{};
            bool externalTransformActive = false;
            std::array<WorldProbeState, kMaxWorldContactProbesPerHand> worldProbes{};
            CachedWorldPlane cachedWorldPlane{};
            soft_contact_math::HapticEdgeState worldHaptic{};
        };

        void clearHand(bool isLeft);
        void clearAllHands();

        std::array<HandRuntime, 2> _hands{};
        SoftContactDebugSnapshot _debugSnapshot{};
        bool _wasEnabled = false;
        std::uint32_t _logCounter = 0;
    };
}
