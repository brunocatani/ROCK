#pragma once

#include "api/ROCKProviderApi.h"

#include <cstdint>

namespace RE
{
    class Actor;
    class BGSBodyPart;
    class bhkWorld;
}

namespace rock::physical_melee
{
    enum class TargetResolutionStage : std::uint32_t
    {
        NotStarted = 0,
        CollisionObjectResolved,
        SceneObjectResolved,
        ReferenceResolved,
        LiveActorResolved,
        BodyPartDataResolved,
        AnatomyResolved,
        RegisteredBodyResolved,
    };

    struct TargetResolution
    {
        RE::Actor* actor{ nullptr };
        std::uint32_t actorFormId{ 0 };
        std::uint32_t referenceFormId{ 0 };
        std::uint32_t bodyId{ 0x7FFF'FFFFu };
        std::uint32_t collisionLayer{ 0xFFFF'FFFFu };
        RE::BGSBodyPart* bodyPart{ nullptr };
        std::uint32_t nativeDamageLimb{ 0xFFFF'FFFFu };
        TargetResolutionStage stage{ TargetResolutionStage::NotStarted };
        bool accessViolation{ false };
        bool referenceIsDead{ false };
        bool referenceIsPlayer{ false };
        bool measuredContactPointValid{ false };
        bool pointFallbackUsed{ false };
        bool registeredBodyEvidence{ false };
        bool registeredIdentityMismatch{ false };
        float measuredContactPointGame[3]{};
        std::uint32_t directMatchCount{ 0 };
        std::uint32_t fallbackCandidateCount{ 0 };
        float selectedNodeDistanceGame{ -1.0f };
        float runnerUpNodeDistanceGame{ -1.0f };
        std::uint32_t anatomyFlags{ 0 };
        std::uint32_t bodyPartIndex{ 0xFFFF'FFFFu };
        provider::RockProviderBodyZoneKind zone{ provider::RockProviderBodyZoneKind::Unknown };
        provider::RockProviderBodyZoneSide side{ provider::RockProviderBodyZoneSide::Center };
        float bodyPartDamageMultiplier{ 1.0f };
        std::uint32_t limbActorValueFormId{ 0 };
        std::uint64_t nodeNameHash{ 0 };
        char nodeName[provider::ROCK_PROVIDER_MAX_EVIDENCE_NAME]{};

        [[nodiscard]] bool actorValid() const noexcept { return actor != nullptr && actorFormId != 0; }
        [[nodiscard]] bool anatomyValid() const noexcept;
    };

    [[nodiscard]] const char* targetResolutionStageName(TargetResolutionStage stage) noexcept;

    [[nodiscard]] TargetResolution resolveTarget(
        RE::bhkWorld* bhkWorld,
        std::uint32_t targetBodyId,
        const float* measuredContactPointHavok,
        float havokToGameScale) noexcept;

    [[nodiscard]] TargetResolution resolveRegisteredTarget(
        const provider::RockProviderExternalBodyRegistration& registration) noexcept;
}
