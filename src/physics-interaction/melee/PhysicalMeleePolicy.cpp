#include "physics-interaction/melee/PhysicalMeleePolicy.h"

#include <algorithm>
#include <cmath>

namespace rock::physical_melee
{
    namespace
    {
        constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;
        constexpr float kReferenceWeaponMass = 12.0f;

        constexpr std::uint32_t contactFlag(provider::RockProviderExternalContactFlagV1 flag)
        {
            return static_cast<std::uint32_t>(flag);
        }

        constexpr std::uint32_t anatomyFlag(provider::RockProviderTargetAnatomyFlagV1 flag)
        {
            return static_cast<std::uint32_t>(flag);
        }

    }

    Settings sanitizeSettings(Settings settings)
    {
        const Settings defaults{};
        if (!std::isfinite(settings.minSourceSpeedGame) || settings.minSourceSpeedGame <= 0.0f) {
            settings.minSourceSpeedGame = defaults.minSourceSpeedGame;
        }
        if (!std::isfinite(settings.virtualWeaponMass) || settings.virtualWeaponMass <= 0.0f) {
            settings.virtualWeaponMass = defaults.virtualWeaponMass;
        }
        if (!std::isfinite(settings.damageMultiplier) || settings.damageMultiplier < 0.0f) {
            settings.damageMultiplier = defaults.damageMultiplier;
        }
        if (!std::isfinite(settings.maxNativeDamageMultiplier) || settings.maxNativeDamageMultiplier <= 0.0f) {
            settings.maxNativeDamageMultiplier = defaults.maxNativeDamageMultiplier;
        }
        if (!std::isfinite(settings.sourceTargetCooldownSeconds) || settings.sourceTargetCooldownSeconds < 0.0f) {
            settings.sourceTargetCooldownSeconds = defaults.sourceTargetCooldownSeconds;
        }
        settings.maxDamageEventsPerFrame = std::clamp(
            settings.maxDamageEventsPerFrame,
            1u,
            provider::ROCK_PROVIDER_MAX_EXTERNAL_CONTACTS_V1);
        return settings;
    }

    bool weaponWitnessMatches(
        const WeaponIdentityWitness& expected,
        const WeaponIdentityWitness& current) noexcept
    {
        return expected.bodyGenerationKey != 0 &&
               expected.bodyGenerationKey == current.bodyGenerationKey &&
               expected.identityKey != 0 && expected.identityKey == current.identityKey &&
               expected.ownershipKey != 0 && expected.ownershipKey == current.ownershipKey &&
               expected.instanceDataAddress == current.instanceDataAddress &&
               expected.weaponFormId != 0 && expected.weaponFormId == current.weaponFormId &&
               expected.collisionGeneration != 0 &&
               expected.collisionGeneration == current.collisionGeneration &&
               expected.equipIndex == current.equipIndex;
    }

    bool isBetterImpactCandidate(
        const ImpactCandidateScore& candidate,
        const ImpactCandidateScore& incumbent) noexcept
    {
        if (candidate.nativeDamageMultiplier != incumbent.nativeDamageMultiplier) {
            return candidate.nativeDamageMultiplier > incumbent.nativeDamageMultiplier;
        }
        if (candidate.positiveImpulseSum != incumbent.positiveImpulseSum) {
            return candidate.positiveImpulseSum > incumbent.positiveImpulseSum;
        }
        if (candidate.surfaceConfidencePermille != incumbent.surfaceConfidencePermille) {
            return candidate.surfaceConfidencePermille > incumbent.surfaceConfidencePermille;
        }
        if (candidate.sourceBodyId != incumbent.sourceBodyId) {
            return candidate.sourceBodyId < incumbent.sourceBodyId;
        }
        return candidate.descriptorIndex < incumbent.descriptorIndex;
    }

    Decision evaluate(
        const provider::RockProviderExternalContactRecordV1& contact,
        const Settings& rawSettings,
        float gameToHavokScale)
    {
        const auto settings = sanitizeSettings(rawSettings);
        Decision decision{};
        decision.targetBodyId = contact.targetExternalBodyId;

        if (!settings.enabled) {
            decision.reason = DecisionReason::Disabled;
            return decision;
        }
        if ((contact.flags & contactFlag(provider::RockProviderExternalContactFlagV1::TransitionSuppressed)) != 0) {
            decision.reason = DecisionReason::TransitionSuppressed;
            return decision;
        }
        if ((contact.flags & contactFlag(provider::RockProviderExternalContactFlagV1::CollisionAvailable)) == 0) {
            decision.reason = DecisionReason::CollisionUnavailable;
            return decision;
        }
        if (contact.sourceBodyId == kInvalidBodyId || contact.targetExternalBodyId == kInvalidBodyId ||
            contact.sourceBodyId == contact.targetExternalBodyId || contact.impactId == 0) {
            decision.reason = DecisionReason::InvalidBodyId;
            return decision;
        }
        if (contact.sourceKind != provider::RockProviderExternalSourceKind::Weapon) {
            decision.reason = DecisionReason::UnsupportedSource;
            return decision;
        }
        if (contact.quality != provider::RockProviderExternalContactQuality::RawPoint) {
            decision.reason = DecisionReason::ContactPointNotVerified;
            return decision;
        }
        if ((contact.flags & contactFlag(provider::RockProviderExternalContactFlagV1::RawManifoldValid)) == 0 ||
            contact.manifoldPointCount == 0 || contact.selectedPointIndex >= contact.manifoldPointCount) {
            decision.reason = DecisionReason::MissingRawManifold;
            return decision;
        }
        if ((contact.flags & contactFlag(provider::RockProviderExternalContactFlagV1::RelativeVelocityValid)) == 0) {
            decision.reason = DecisionReason::MissingRelativeVelocity;
            return decision;
        }
        if (contact.sourceWeaponFormId == 0) {
            decision.reason = DecisionReason::MissingWeaponIdentity;
            return decision;
        }
        if ((contact.flags & contactFlag(provider::RockProviderExternalContactFlagV1::SourceSurfaceClassified)) == 0 ||
            contact.sourceSurfaceRegion == provider::RockProviderImpactSurfaceRegionV1::Unknown ||
            !std::isfinite(contact.sourceSurfaceDamageCoefficient) || contact.sourceSurfaceDamageCoefficient < 0.0f) {
            decision.reason = DecisionReason::MissingSurfaceClassification;
            return decision;
        }
        if (contact.sourceSurfaceDamageCoefficient == 0.0f) {
            decision.reason = DecisionReason::NonDamagingSource;
            return decision;
        }
        if ((contact.flags & contactFlag(provider::RockProviderExternalContactFlagV1::TargetAnatomyValid)) == 0 ||
            (contact.targetAnatomyFlags & anatomyFlag(provider::RockProviderTargetAnatomyFlagV1::BodyPartValid)) == 0 ||
            contact.targetBodyPartIndex >= 26) {
            decision.reason = DecisionReason::MissingTargetAnatomy;
            return decision;
        }
        if ((contact.targetAnatomyFlags & anatomyFlag(provider::RockProviderTargetAnatomyFlagV1::BodyPartAmbiguous)) != 0) {
            decision.reason = DecisionReason::AmbiguousTargetAnatomy;
            return decision;
        }
        if (!std::isfinite(gameToHavokScale) || gameToHavokScale <= 0.0f) {
            decision.reason = DecisionReason::InvalidScale;
            return decision;
        }
        if (!std::isfinite(contact.closingSpeedHavok) || contact.closingSpeedHavok < 0.0f ||
            !std::isfinite(contact.tangentSpeedHavok) || contact.tangentSpeedHavok < 0.0f) {
            decision.reason = DecisionReason::NonFiniteInput;
            return decision;
        }

        decision.closingSpeedHavok = contact.closingSpeedHavok;
        decision.closingSpeedGame = contact.closingSpeedHavok / gameToHavokScale;
        decision.tangentSpeedHavok = contact.tangentSpeedHavok;
        decision.relativePointSpeedHavok = std::hypot(
            contact.closingSpeedHavok,
            contact.tangentSpeedHavok);
        decision.relativePointSpeedGame = decision.relativePointSpeedHavok / gameToHavokScale;
        decision.virtualMass = settings.virtualWeaponMass;
        decision.sourceSurfaceDamageCoefficient = contact.sourceSurfaceDamageCoefficient;
        if (settings.damageMultiplier == 0.0f) {
            decision.reason = DecisionReason::DamageMultiplierZero;
            return decision;
        }
        if (!std::isfinite(decision.relativePointSpeedGame) ||
            decision.relativePointSpeedGame < settings.minSourceSpeedGame) {
            decision.reason = DecisionReason::SourceTooSlow;
            return decision;
        }

        const float normalizedSpeed = decision.relativePointSpeedGame / settings.minSourceSpeedGame;
        const float normalizedKineticEnergy =
            (settings.virtualWeaponMass / kReferenceWeaponMass) * normalizedSpeed * normalizedSpeed;
        float multiplier = normalizedKineticEnergy * settings.damageMultiplier * contact.sourceSurfaceDamageCoefficient;
        if (settings.maxNativeDamageMultiplier > 0.0f) {
            multiplier = (std::min)(multiplier, settings.maxNativeDamageMultiplier);
        }
        if (!std::isfinite(multiplier) || multiplier <= 0.0f) {
            decision.reason = DecisionReason::NonFiniteInput;
            return decision;
        }

        decision.nativeDamageMultiplier = multiplier;
        decision.accepted = true;
        decision.reason = DecisionReason::Accepted;
        return decision;
    }

    const char* decisionReasonName(DecisionReason reason)
    {
        switch (reason) {
        case DecisionReason::Accepted: return "Accepted";
        case DecisionReason::Disabled: return "Disabled";
        case DecisionReason::InvalidBodyId: return "InvalidBodyId";
        case DecisionReason::UnsupportedSource: return "UnsupportedSource";
        case DecisionReason::ContactPointNotVerified: return "ContactPointNotVerified";
        case DecisionReason::MissingRawManifold: return "MissingRawManifold";
        case DecisionReason::MissingRelativeVelocity: return "MissingRelativeVelocity";
        case DecisionReason::MissingWeaponIdentity: return "MissingWeaponIdentity";
        case DecisionReason::MissingSurfaceClassification: return "MissingSurfaceClassification";
        case DecisionReason::MissingTargetAnatomy: return "MissingTargetAnatomy";
        case DecisionReason::AmbiguousTargetAnatomy: return "AmbiguousTargetAnatomy";
        case DecisionReason::ContinuedEpisode: return "ContinuedEpisode";
        case DecisionReason::SourceTooSlow: return "SourceTooSlow";
        case DecisionReason::NonFiniteInput: return "NonFiniteInput";
        case DecisionReason::InvalidScale: return "InvalidScale";
        case DecisionReason::CooldownActive: return "CooldownActive";
        case DecisionReason::FrameBudgetExceeded: return "FrameBudgetExceeded";
        case DecisionReason::MissingTargetActor: return "MissingTargetActor";
        case DecisionReason::NativeSubmissionFailed: return "NativeSubmissionFailed";
        case DecisionReason::CollisionUnavailable: return "CollisionUnavailable";
        case DecisionReason::TransitionSuppressed: return "TransitionSuppressed";
        case DecisionReason::NonDamagingSource: return "NonDamagingSource";
        case DecisionReason::DamageMultiplierZero: return "DamageMultiplierZero";
        case DecisionReason::RuntimeUnavailable: return "RuntimeUnavailable";
        case DecisionReason::WeaponWitnessMismatch: return "WeaponWitnessMismatch";
        case DecisionReason::SupersededCandidate: return "SupersededCandidate";
        case DecisionReason::RegisteredTargetMismatch: return "RegisteredTargetMismatch";
        default: return "Unknown";
        }
    }
}
